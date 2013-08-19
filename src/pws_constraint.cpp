/*
 * Copyright (C) 2013 Magnus Lång
 *
 * This file is part of Memorax.
 *
 * Memorax is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Memorax is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public
 * License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "pws_constraint.h"
#include "intersection_iterator.h"
#include <iostream>
#include <iomanip>
#include <iterator>
#include "pputils.h"

/*****************/
/* Configuration */
/*****************/

const bool PwsConstraint::use_channel_suffix_equality = true;
const bool PwsConstraint::use_limit_other_updates = true;
const bool PwsConstraint::use_pending_sets = true;
const bool PwsConstraint::use_serialisations_only_after_writes = true;
const bool PwsConstraint::use_last_write_sets = true;

/*****************/

void PwsConstraint::Common::init_pending(std::function<bool(const Lang::Stmt<int>&, Lang::MemLoc<int>)> pred,
                                         const std::vector<Automaton::State> &states,
                                         std::vector<std::vector<std::map<Lang::NML, value_t> > > &sets) {
  sets.push_back(std::vector<std::map<Lang::NML, ZStar<int> > >(states.size()));
  int pid = sets.size() - 1;
  for (unsigned i = 0; i < states.size(); ++i) {
    std::map<Lang::NML, value_t> &set = sets[pid][i];
    for (const Automaton::Transition *trans : states[i].bwd_transitions) {
      for (const VecSet<Lang::MemLoc<int>> &ws : trans->instruction.get_write_sets()) {
        Store store = store_of_write(Machine::PTransition(*trans, pid));
        for (Lang::MemLoc<int> ml : ws) {
          if (!pred(trans->instruction, ml)) {
            Lang::NML nml(ml, pid);
            value_t write = store[index(nml)];
            if (set.count(nml)) {
              if (set[nml] != write) set[nml] = value_t::STAR;
            } else {
              set[nml] = write;
            }
          }
        }
      }
    }
  }
}

void PwsConstraint::Common::iterate_pending(std::function<bool(const Lang::Stmt<int>&, Lang::MemLoc<int>)> pred,
                                            const std::vector<Automaton::State> &states,
                                            int state, int pid,
                                            std::vector<std::map<Lang::NML, ZStar<int> > > &set,
                                            bool &changed) {
  for (std::pair<Lang::NML, ZStar<int>> elem : set[state]) {
    for (const Automaton::Transition *trans : states[state].fwd_transitions) {
      if (!pred(trans->instruction, elem.first.localize(pid))) {
        std::map<Lang::NML, ZStar<int>> &target_set = set[trans->target];
        if (target_set.count(elem.first)) {
          if (target_set[elem.first] != elem.second && target_set[elem.first] != ZStar<int>::STAR) {
            target_set[elem.first] = ZStar<int>::STAR;
            changed = true;
          }
        } else {
          target_set[elem.first] = elem.second;
          changed = true;
        }
      }
    }
  }
}

PwsConstraint::Common::Common(const Machine &m) : ChannelConstraint::Common(m) {
  /* Check that all variables and registers have finite domains */
  for (Lang::VarDecl gvar : m.gvars) {
    if (gvar.domain.is_int()) {
      throw new std::logic_error("PwsConstraint::Common: PwsConstraint requires finite domains. Infinite domain for global variable " +
                                 gvar.name);
    }
  }
  for (unsigned p = 0; p < m.lvars.size(); ++p) {
    for (Lang::VarDecl lvar : m.lvars[p]) {
      if (lvar.domain.is_int()) {
        std::stringstream ss;
        ss << "PwsConstraint::Common: PwsConstraint requires finite domains. Infinite domain for local variable "
           << lvar.name << "[P" << p << "]";
        throw new std::logic_error(ss.str());
      }
    }
    for (Lang::VarDecl reg : m.regs[p]) {
      if (reg.domain.is_int()) {
        std::stringstream ss;
        ss << "PwsConstraint::Common: PwsConstraint requires finite domains. Infinite domain for register "
           << "P" << p << ":" << reg.name;
        throw new std::logic_error(ss.str());
      }
    }
  }

  /* Setup all_transitions */
  std::vector<std::vector<bool> > has_writes;
  for (unsigned p = 0; p < machine.automata.size(); ++p) {
    const std::vector<Automaton::State> &states = machine.automata[p].get_states();
    has_writes.push_back(std::vector<bool>(states.size(), false));
    for (unsigned i = 0; i < states.size(); ++i) {
      for (const Automaton::Transition *t : states[i].bwd_transitions) {
        all_transitions.push_back(Machine::PTransition(*t, p));
        if (t->instruction.get_writes().size() > 0) has_writes[p][i] = true;

        /* Check that the machine has been properly been converted from lock to fence semantics. */
        if (t->instruction.get_type() == Lang::SLOCKED ||
            (t->instruction.get_type() == Lang::LOCKED &&
             t->instruction.get_statement(0)->get_type() == Lang::WRITE))
          throw new std::logic_error("PwsConstraint::Common: PwsConstraint requires the machine "
                                     "uses fence semantics rather than lock semantics. Have you "
                                     "forgotten to use Machine::convert_locks_to_fences?");
      }
      for (const MsgHdr &m : messages)
        if (m.nmls.size() > 0) {
          VecSet<Lang::MemLoc<int>> mls;
          for (Lang::NML nml : m.nmls) mls.insert(nml.localize(p));
          all_transitions.push_back(Machine::PTransition(i, Lang::Stmt<int>::update(m.wpid, mls), i, p));
        }
    }
  }

  /* Messages contains every memory location that is ever written
   * We insert serialise transitions for that memory location and the writing process
   * at each state. */
  for (const Common::MsgHdr &header : messages) {
    int p = header.wpid;
    if (header.nmls.size() > 0) { //We are not interested in the dummy message
      VecSet<Lang::MemLoc<int>> mls;
      for (const Lang::NML &nml : header.nmls)
        mls.insert(nml.localize(p));
      const std::vector<Automaton::State> &states = machine.automata[p].get_states();
      for (unsigned i = 0; i < states.size(); i++) {
        if (!use_serialisations_only_after_writes || has_writes[p][i])
          all_transitions.push_back(Machine::PTransition(i, Lang::Stmt<int>::serialise(mls), i, p));
      }
    }
  }

  /* Setup transitions_by_pc. */
  for(unsigned p = 0; p < machine.automata.size(); ++p){
    transitions_by_pc.push_back(std::vector<std::vector<const Machine::PTransition*> >
                                (machine.automata[p].get_states().size()));
  }
  for(unsigned i = 0; i < all_transitions.size(); ++i){
    transitions_by_pc[all_transitions[i].pid][all_transitions[i].target]
      .push_back(&all_transitions[i]);
  }

  auto is_mfence = [](const Lang::Stmt<int> &stmt, Lang::MemLoc<int> ml) {
    return (stmt.get_type() == Lang::MFENCE ||
            (stmt.get_type() == Lang::LOCKED &&
             std::find(stmt.get_writes().begin(), stmt.get_writes().end(), ml)
             != stmt.get_writes().end()));
  };

  if (use_pending_sets) {
    /* Setup pending_set and pending_buffers */
    for (unsigned p = 0; p < machine.automata.size(); ++p) {
      const std::vector<Automaton::State> &states = machine.automata[p].get_states();
      init_pending(is_mfence, states, pending_set);

      /* Use a fix-point iteration to find the complete sets. */
      bool changed = true;
      while (changed) {
        changed = false;
        const std::vector<Automaton::State> &states = machine.automata[p].get_states();
        for (unsigned i = 0; i < states.size(); ++i) {
          /* Complete pending_set and pending_buffers. */
          iterate_pending(is_mfence, states, i, p, pending_set[p],     changed);
        }
      }
    }
    auto prpair = [this](Log::redirection_stream &rs, std::pair<Lang::NML, value_t> elem) {
      rs << machine.pretty_string_nml.at(elem.first) << ":" << elem.second;
    };
    Log::extreme << "Pending sets\n";
    for (unsigned p = 0; p < pending_set.size(); ++p) {
      Log::extreme << "For process P" << p << "\n";
      for (unsigned i = 0; i < pending_set[p].size(); ++i) {
        Log::extreme << "  Q" << i << " {";
        PPUtils::print_sequence_with_separator(Log::extreme, pending_set[p][i], ", ", prpair);
        Log::extreme << "}" << std::endl;
      }
    }
  }

  if (use_last_write_sets) {
    /* A sequence is normalised by removing all empty maps except for the last one. */
    auto normalise_last_write_sequence = [](std::list<std::map<Lang::NML, value_t> > &seq) {
      auto iter = seq.begin();
      auto end = --seq.end(); // We skip the last element
      while (iter != end) {
        if (iter->empty()) iter = seq.erase(iter);
        else iter++;
      }
    };

    auto print_last_write_sequence = [this](Log::redirection_stream &rs,
                                            const std::list<std::map<Lang::NML, value_t> > &seq,
                                            int pid) {
      auto mlts = this->machine.ml_pretty_vts(pid);
      auto nmlts = [mlts, pid](Lang::NML nml) { return mlts(nml.localize(pid)); };
      auto prpair = [nmlts](Log::redirection_stream &rs, const std::pair<Lang::NML, value_t> &pair) {
        rs << nmlts(pair.first) << "=" << pair.second.to_string();
      };
      auto prmap = [prpair](Log::redirection_stream &rs, const std::map<Lang::NML, value_t> &map) {
          rs << "{";
          PPUtils::print_sequence_with_separator(rs, map, ", ", prpair);
          rs << "}";
        };
      PPUtils::print_sequence_with_separator(rs, seq, ":", prmap);
    };

    Log::extreme << "Last write sets:\n";
    /* Setup last_write_sets. */
    for (unsigned pid = 0; pid < machine.automata.size(); ++pid) {
      last_write_sets.push_back(std::vector<std::set<map_sequence> >(machine.automata[pid].get_states().size()));
      map_sequence empty({{}}, {{}});
      last_write_sets[pid][0].insert(empty); // In the initial state there are no writes

      /* Use a fix-point iteration to find the complete sets. */
      bool changed = true;
      while (changed) {
        changed = false;
        const std::vector<Automaton::State> &states = machine.automata[pid].get_states();
        for (unsigned i = 0; i < states.size(); ++i) {
          for (const Automaton::Transition *trans : states[i].fwd_transitions) {
            /* We purposefully make a copy of the source sequence. We modify it
             * according to trans and insert it in target. */
            for (map_sequence source_sequence : last_write_sets[pid][i]) {
              switch (trans->instruction.get_type()) {
              case Lang::WRITE: {
                assert(trans->instruction.get_writes().size() == 1);
                value_t value = value_of_write(Machine::PTransition(*trans, pid));
                Lang::NML loc(trans->instruction.get_writes()[0], pid);
                /* We add the value to the last map. We make sure loc is in no
                 * other map so the invariant holds */
                for (std::map<Lang::NML, value_t> &map : source_sequence.first)  map.erase(loc);
                for (std::map<Lang::NML, value_t> &map : source_sequence.second) map.erase(loc);
                source_sequence.second.back()[loc] = value;
              } break;
              case Lang::LOCKED: {
                assert(trans->instruction.get_writes().size() == 1);
                value_t value = value_of_write(Machine::PTransition(*trans, pid));
                Lang::NML loc(trans->instruction.get_writes()[0], pid);
                for (std::map<Lang::NML, value_t> &map : source_sequence.first)  map.erase(loc);
                for (std::map<Lang::NML, value_t> &map : source_sequence.second) map.erase(loc);
                /* Since cas requires the buffer to loc to be empty, and the
                 * cpointer to be to the right like an MFENCE, we move all but
                 * the last set in second to first. */
                source_sequence.first.splice(source_sequence.first.end(), source_sequence.second,
                                             source_sequence.second.begin(),
                                             --source_sequence.second.end());

                for (std::pair<std::map<Lang::NML, value_t>, std::map<Lang::NML, value_t> >
                       &powermap : powermaps(std::move(source_sequence.second.back()))) {
                  map_sequence copy(source_sequence.first, {});
                  copy.first.push_back(std::move(powermap.first));
                  /* The write is added as it's own set because we know it is
                   * strictly later than any of the already serialised writes
                   * (those that we spliced above). */
                  copy.first.push_back({{loc, value}});
                  copy.second = {std::move(powermap.second)};
                  normalise_last_write_sequence(copy.first);
                  changed |= last_write_sets[pid][trans->target]
                    .insert(std::move(copy)).second;

                }
                /* We have normalised and inserted the modified sequence
                 * ourselves, no need to do the rest of the loop */
                continue;
              }
              case Lang::MFENCE:
                // Move all elements to the right of the cpointer
                source_sequence.first.splice(source_sequence.first.end(), source_sequence.second);
                source_sequence.second.push_back({}); // The sequence must not be empty
                break;
              case Lang::SFENCE:
                source_sequence.second.push_back({});
                break;
              default:; //No effect
              }
              /* We normalise the sequence before adding it to the target set. */
              normalise_last_write_sequence(source_sequence.first);
              normalise_last_write_sequence(source_sequence.second);
              changed |= last_write_sets[pid][trans->target]
                .insert(std::move(source_sequence)).second;
            }
          }
        }
      }

      Log::extreme << "  For process P" << pid << std::endl;
      for (unsigned i = 0; i < last_write_sets[pid].size(); ++i) {
        Log::extreme << "    Q" << std::setiosflags(std::ios::left) << std::setw(3) << i
                     << std::setw(0) << "{";
        PPUtils::print_sequence_with_separator(Log::extreme, last_write_sets[pid][i], ",\n         ",
          [print_last_write_sequence, pid](Log::redirection_stream &rs, const map_sequence &seq) {
            print_last_write_sequence(rs, seq.first, pid);
            rs << "|";
            print_last_write_sequence(rs, seq.second, pid);
          });
        Log::extreme << "}" << std::endl;
      }
    }
  }
}
std::list<std::pair<std::map<Lang::NML, ZStar<int> >,
                    std::map<Lang::NML, ZStar<int> > > >
PwsConstraint::Common::powermaps(std::map<Lang::NML, value_t> map) const {
  if (map.empty()) return {{map, map}};
  std::pair<Lang::NML, value_t> kvp = *map.begin();
  map.erase(map.begin());
  std::list<std::pair<std::map<Lang::NML, ZStar<int> >, std::map<Lang::NML, ZStar<int> > > >
    list = powermaps(std::move(map));

  auto iter = list.begin();
  while (iter != list.end()) {
    /* Duplicate the element, and then insert our kvp into the first or second
     * element of the first and second copy respectively. */
    iter = list.insert(iter, *iter);
    (iter++)->first.insert(kvp);
    (iter++)->second.insert(kvp);
  }

  return list;
}


std::list<Constraint*> PwsConstraint::Common::get_bad_states() {
  std::list<Constraint*> l;
  for (const std::vector<int> &pcs : machine.forbidden) {
    for (const MsgHdr &msg : messages) {
      PwsConstraint *pwsc = new PwsConstraint(pcs, msg, *this);
      if (pwsc->unreachable()) delete pwsc;
      else l.push_back(pwsc);
    }
  }
  return l;
};

PwsConstraint::PwsConstraint(std::vector<int> pcs, const Common::MsgHdr &msg, Common &c)
  : ChannelConstraint(pcs, msg, c), common(c) {
  for (unsigned p = 0; p < common.machine.automata.size(); p++)
    write_buffers.push_back(std::vector<Store>(common.mem_size, Store(0)));
}

std::list<const Machine::PTransition*> PwsConstraint::partred() const{
  std::list<const Machine::PTransition*> l;
  if (use_limit_other_updates) {
    for (unsigned p = 0; p < pcs.size(); ++p) {
      bool has_read = false;
      for (const Machine::PTransition *trans : common.transitions_by_pc[p][pcs[p]]) {
        if (trans->instruction.get_reads().size()) {
          has_read = true;
        }
      }
      for (const Machine::PTransition *trans : common.transitions_by_pc[p][pcs[p]]) {
        if (trans->instruction.get_type() != Lang::UPDATE || has_read ||
            cpointers[p] == int(channel.size())-1) {
          l.push_back(trans);
        }
      }
    }
  } else {
    for (unsigned p = 0; p < pcs.size(); ++p) {
      l.insert(l.end(), common.transitions_by_pc[p][pcs[p]].begin(),
                        common.transitions_by_pc[p][pcs[p]].end());
    }
  }
  return l;
}

std::list<Constraint*> PwsConstraint::pre(const Machine::PTransition &t) const {
  std::list<Constraint*> res;
  std::list<pre_constr_t> r = pre(t, false);
  for (const pre_constr_t &c : r) {
    bool move_p_to_last = false;
    std::vector<PwsConstraint*> pwscs;
    if (c.channel_pop_back) {
      assert(c.pwsc->channel.back().nmls == c.written_nmls);
      assert(c.pwsc->channel.size() > 1);

      assert((c.pwsc->cpointers[t.pid] == int(c.pwsc->channel.size()) - 1)
             ==
             (t.instruction.get_type() == Lang::LOCKED));
      if (c.pwsc->cpointers[t.pid] == int(c.pwsc->channel.size()) - 1) {
        --c.pwsc->cpointers[t.pid];
        /* Make sure after hidden messages have been added that
         * process t.pid points to the rightmost message */
        move_p_to_last = true;
      }
      for (ChannelConstraint *chc : c.pwsc->channel_pop_back()) {
        assert (dynamic_cast<PwsConstraint*>(chc));
        pwscs.push_back(static_cast<PwsConstraint*>(chc));
      }
      delete c.pwsc;
    }
    else if (c.buffer_pop_back) {
      assert(c.written_nmls.size() == 1);
      Lang::NML nml = c.written_nmls[0];
      pwscs = c.pwsc->buffer_pop_back(t.pid, nml);
      delete c.pwsc;
    }
    else {
      pwscs.push_back(c.pwsc);
    }

    for (PwsConstraint *pwsc : pwscs) {
      if (move_p_to_last) pwsc->cpointers[t.pid] = int(pwsc->channel.size()) - 1;
      if (pwsc->unreachable()) delete pwsc;
      else                     res.push_back(pwsc);
    }
  }
  return res;
}

std::list<PwsConstraint::pre_constr_t> PwsConstraint::pre(const Machine::PTransition &t, bool locked) const {
  const Lang::Stmt<int> &s = t.instruction;
  /* We only allow locked blocks representing a cas. */
  assert(!locked || s.get_type() == Lang::READASSERT ||
                    s.get_type() == Lang::WRITE);
  std::list<pre_constr_t> res;
  if (pcs[t.pid] != t.target)
    return res;

  switch (s.get_type()) {
  case Lang::READASSERT: {
    Lang::NML nml(s.get_memloc(), t.pid);
    int nmli = common.index(nml);
    int msgi = index_of_read(nml, t.pid);
    if (locked) {
      /* We are in a locked block and the value of the message we just wrote
       * isn't supposed to be visible to us; channel_pop_back have yet to be
       * run. We find the next message that contains nml or is under the
       * cpointer. */
      assert(msgi == int(channel.size()) - 1);
      while(--msgi > cpointers[t.pid]) {
        if (channel[msgi].wpid == t.pid && channel[msgi].nmls.count(nml))
          break;
      }
    }
    VecSet<int> all_vals;
    int buffer_size = write_buffers[t.pid][nmli].size();
    if (buffer_size > 0) {
      value_t bufferValue = write_buffers[t.pid][nmli][buffer_size - 1];
      all_vals = possible_values(bufferValue, nml);
    } else {
      all_vals = ChannelConstraint::possible_values(channel[msgi].store, nml);
    }
    for (int val : all_vals) {
      /* For each possible value for nml, try to pair it with an
       * assignment to the reg_store such that s.expr() evaluates to
       * the same value. */
      VecSet<Store> stores = possible_reg_stores(reg_stores[t.pid], t.pid, s.get_expr(), val);
      for (int j = 0; j < stores.size(); ++j) {
        PwsConstraint *pwsc = this->clone();
        pwsc->pcs[t.pid] = t.source;
        if (buffer_size > 0) pwsc->write_buffers[t.pid][nmli] =
                             pwsc->write_buffers[t.pid][nmli].assign(buffer_size - 1, val);
        else pwsc->channel[msgi].store = pwsc->channel[msgi].store.assign(nmli, val);
        pwsc->reg_stores[t.pid] = stores[j];
        res.push_back(pwsc);
      }
    }
  } break;

  case Lang::WRITE: {
    int pid = t.pid;
    Lang::NML nml = Lang::NML(s.get_memloc(), pid);
    int nmli = common.index(nml);

    /* When performing an atomic read-write (locked = true), the buffers must be
     * empty and when we're finished, pid must be the only process with a
     * cpointer in the newly created message. */
    bool ok_buffers = locked ? (write_buffers[pid][nmli].size() == 0):
                               (write_buffers[pid][nmli].size()  > 0);
    bool ok_cpointers = true;
    bool ok_channel   = true;
    if (locked) {
      ok_cpointers = cpointers[t.pid] == int(channel.size()) - 1;
      for (int p = 0; p < int(cpointers.size()); ++p) {
        if (p != pid && cpointers[p] == int(channel.size()) - 1)
          ok_cpointers = false;
      }
      ok_channel &= channel.back().wpid == pid;
      ok_channel &= channel.back().nmls.count(nml);
    }

    if (ok_buffers && ok_cpointers && ok_channel) {
      VecSet<int> vals = locked ? ChannelConstraint::possible_values(channel.back().store, nml):
                                  possible_values(write_buffers[pid][nmli].back(),         nml);
      for (int val : vals) {
        VecSet<Store> rstores = possible_reg_stores(reg_stores[pid], pid, s.get_expr(), val);
        for (const Store &rstore : rstores) {
          PwsConstraint *pwsc = this->clone();
          pwsc->pcs[pid] = t.source;
          if (locked) pwsc->channel.back().store = pwsc->channel.back().store.assign(nmli, val);
          else         pwsc->write_buffers[pid][nmli].assign(pwsc->write_buffers[pid][nmli].size() - 1, val);
          pwsc->reg_stores[pid] = rstore;
          res.push_back(pre_constr_t(pwsc, locked, !locked, VecSet<Lang::NML>::singleton(nml)));
        }
      }
    }
  } break;

  case Lang::LOCKED: {
    if (cpointers[t.pid] == int(channel.size()) - 1 &&
        is_fully_serialised(t.pid, s.get_writes())) {
      assert(s.get_statement_count() == 1);
      Lang::Stmt<int> si = *s.get_statement(0);
      assert(si.get_type() == Lang::SEQUENCE);

      /* From here we do SEQUENCE */
      std::vector<pre_constr_t> V { this->clone() };
      for (int i = si.get_statement_count() - 1; i >= 0; --i) {
        std::vector<pre_constr_t> W;
        Machine::PTransition t2(t.target, *si.get_statement(i), t.target, t.pid);
        for (pre_constr_t v : V) {
          for (pre_constr_t w : v.pwsc->pre(t2, true)) {
            w.channel_pop_back = w.channel_pop_back || v.channel_pop_back;
            w.buffer_pop_back  = w.buffer_pop_back  || v.buffer_pop_back;
            w.written_nmls.insert(v.written_nmls);
            W.push_back(w);
          }
          delete v.pwsc;
        }
        V = W;
      }
      for (pre_constr_t v : V) {
        v.pwsc->pcs[t.pid] = t.source;
        res.push_back(v);
      }
    }
  } break;

  case Lang::UPDATE: {
    VecSet<Lang::NML> snmls;
    for (Lang::MemLoc<int> ml : s.get_writes())
      snmls.insert(Lang::NML(ml, t.pid));
    /* Check that the message matches the update */
    if (channel[cpointers[t.pid]].wpid == s.get_writer() &&
        channel[cpointers[t.pid]].nmls == snmls) {
      if (cpointers[t.pid] == 0) {
        /* Need to insert a fresh message. */
        for (const Common::MsgHdr &hdr : common.messages) {
          PwsConstraint *pwsc = this->clone();
          Msg msg(Store(common.mem_size), hdr.wpid, hdr.nmls);
          pwsc->channel.insert(pwsc->channel.begin(), msg);
          for (int p = 0; p < int(pwsc->cpointers.size()); p++) {
            if (p != t.pid) pwsc->cpointers[p]++;
          }
          res.push_back(pwsc);
        }
      } else {
        /* Update with a message in the channel */
        PwsConstraint *pwsc = this->clone();
        pwsc->cpointers[t.pid]--;
        res.push_back(pwsc);
      }
    }
  } break;

  case Lang::SERIALISE: {
    assert(!locked);
    int pid = t.pid;
    uint maxcptr = *std::max_element(cpointers.begin(), cpointers.end());
    /* We can only do serialise if the last message has the right process and
     * memory locations and isn't pointed to by any cpointer. */
    if (maxcptr < channel.size() - 1 && channel.back().wpid == pid) {
      /* TODO: Consider if there should be a serialise message for each unique
       * memory location instead */
      /* We assume that the writes in s.get_writes are sorted and unique, and
       * then map them into normalised form, which we assume have the same ordering. */
      std::vector<Lang::NML> vector;
      for (const Lang::MemLoc<int> &ml : s.get_writes()) {
        vector.push_back(Lang::NML(ml, pid));
      }
      VecSet<Lang::NML> nmls(vector);
      // Each memory location that is both in the last message in the channel
      // and the serialise action (and thus in the intersection of the sets) is
      // expanded to it's own new constraint since we cannot represent a write
      // to a set of memory locations in the buffers.
      Intersection<VecSet<Lang::NML>, Lang::NML, VecSet<Lang::NML>::const_iterator> inter(channel.back().nmls, nmls);
      for (const Lang::NML &nml : inter) {
        int nmli = common.index(nml);
        PwsConstraint *pwsc = this->clone();
        pwsc->write_buffers[pid][nmli] = pwsc->write_buffers[pid][nmli].push_front(channel.back().store[nmli]);
        res.push_back(pre_constr_t(pwsc, true, false, VecSet<Lang::NML>::singleton(nml)));
      }
    }
  } break;

  case Lang::MFENCE:
    if (cpointers[t.pid] != int(channel.size()) - 1) break;
  case Lang::SFENCE:
    if (!is_fully_serialised(t.pid)) break;
    /* All conditions are fullfilled, fall through to NOP */
  case Lang::NOP: {
    PwsConstraint *pwsc = this->clone();
    pwsc->pcs[t.pid] = t.source;
    res.push_back(pwsc);
  } break;

  case Lang::ASSIGNMENT: {
    VecSet<int> vals_r = ChannelConstraint::possible_values(reg_stores[t.pid], t.pid,
                                                           Lang::Expr<int>::reg(s.get_reg()));
    for (int val_r : vals_r) {
      VecSet<Store> rss = possible_reg_stores(reg_stores[t.pid].assign(s.get_reg(), value_t::STAR),
                                              t.pid, s.get_expr(), val_r);
      for (const Store &new_reg_store : rss) {
        PwsConstraint *pwsc = this->clone();
        pwsc->reg_stores[t.pid] = new_reg_store;
        pwsc->pcs[t.pid] = t.source;
        res.push_back(pwsc);
      }
    }
  } break;

  case Lang::READASSIGN: {
    if (reg_stores[t.pid][s.get_reg()] == value_t::STAR) {
      PwsConstraint *pwsc = this->clone();
      pwsc->pcs[t.pid] = t.source;
      res.push_back(pwsc);
    } else {
      Lang::NML nml(s.get_memloc(), t.pid);
      int nmli = common.index(nml);
      int msgi = index_of_read(nml, t.pid);
      VecSet<int> all_vals;
      int buffer_size = write_buffers[t.pid][nmli].size();
      if (buffer_size > 0) {
        value_t bufferValue = write_buffers[t.pid][nmli][buffer_size - 1];
        all_vals = possible_values(bufferValue, nml);
      } else {
        all_vals = ChannelConstraint::possible_values(channel[msgi].store, nml);
      }

      int reg_val = reg_stores[t.pid][s.get_reg()].get_int();

      if (all_vals.count(reg_val) > 0) {
        PwsConstraint *pwsc = this->clone();
        pwsc->pcs[t.pid] = t.source;
        if (buffer_size > 0) pwsc->write_buffers[t.pid][nmli] =
                               pwsc->write_buffers[t.pid][nmli].assign(buffer_size - 1, reg_val);
        else pwsc->channel[msgi].store = pwsc->channel[msgi].store.assign(nmli, reg_val);
        res.push_back(pwsc);
      }
    }
  } break;

  case Lang::ASSUME: {
    VecSet<Store> rstores = possible_reg_stores(reg_stores[t.pid], t.pid, s.get_condition());
    for (const Store &store : rstores) {
      PwsConstraint *pwsc = this->clone();
      pwsc->pcs[t.pid] = t.source;
      pwsc->reg_stores[t.pid] = store;
      res.push_back(pwsc);
    }
  } break;

  default:
    throw new std::logic_error("PwsConstraint::pre: Unsupported transition: "
                               + t.to_string(common.machine));
  }
  return res;
}

bool PwsConstraint::is_fully_serialised(int pid) const {
  for (unsigned nmli = 0; nmli < write_buffers[pid].size(); nmli++) {
    if (write_buffers[pid][nmli].size() != 0) return false;
  }
  return true;
}

bool PwsConstraint::is_fully_serialised(int pid, const std::vector<Lang::MemLoc<int>> nmls) const {
  for (Lang::MemLoc<int> ml : nmls) {
    Lang::NML nml(ml, pid);
    if (write_buffers[pid][common.index(nml)].size() != 0) return false;
  }
  return true;
}

std::vector<PwsConstraint*> PwsConstraint::buffer_pop_back(int pid, Lang::NML nml) const {
  // TODO: ponder the completeness of this
  int nmli = common.index(nml);
  assert(write_buffers[pid][nmli].size() > 0); // Precondition
  std::vector<PwsConstraint*> res;
  // Case one: no write was hidden by the popped one
  res.push_back(this->clone());
  res.back()->write_buffers[pid][nmli] =
    res.back()->write_buffers[pid][nmli].pop_back();
  // Case two: some write was hidden by the popped one
  res.push_back(this->clone());
  res.back()->write_buffers[pid][nmli] =
    res.back()->write_buffers[pid][nmli].assign(write_buffers[pid][nmli].size() - 1, value_t::STAR);
  return res;
}


bool PwsConstraint::is_init_state() const {
  if (!ChannelConstraint::is_init_state()) return false;
  for (auto pwb : write_buffers)
    for (const Store &buffer : pwb)
      if (buffer.size() > 0) return false;
  return true;
}

void PwsConstraint::process_to_string(int p, std::stringstream &ss) const noexcept {
  ChannelConstraint::process_to_string(p, ss);
  ss << " buffer={";
  for (int g = 0; g < common.gvar_count; g++) {
    pretty_print_buffer(ss, write_buffers[p], Lang::NML::global(g));
  }
  for (uint tp = 0; tp < common.machine.automata.size(); tp++)
    for (int l = 0; l < common.max_lvar_count; l++)
      pretty_print_buffer(ss, write_buffers[p], Lang::NML::local(l, tp));
  ss << "}";
}

void PwsConstraint::pretty_print_buffer(std::stringstream &ss, const std::vector<Store> &buffer,
                                        Lang::NML nml) const {
  int nmli = common.index(nml);
  if (buffer[nmli].size() == 0) return;
  ss << common.machine.pretty_string_nml.at(nml) << ": ";
  for (int i = 0; i < buffer[nmli].size(); i++)
    if (buffer[nmli][i].is_star()) ss << "*,";
    else ss << buffer[nmli][i].get_int() << ",";
  ss << "; ";
}

Constraint::Comparison PwsConstraint::entailment_compare(const Constraint &c) const {
  const PwsConstraint &pwsc = dynamic_cast<const PwsConstraint&>(c);
  return entailment_compare_impl(pwsc);
}

Constraint::Comparison PwsConstraint::entailment_compare_impl(const PwsConstraint &pwsc) const {
  Constraint::Comparison cmp = entailment_compare_buffers(pwsc);
  if (cmp == Constraint::INCOMPARABLE) return cmp;
  return comb_comp(ChannelConstraint::entailment_compare(pwsc), cmp);
}

Constraint::Comparison PwsConstraint::entailment_compare_buffers(const PwsConstraint &pwsc) const {
  Comparison cmp = EQUAL;
  for (int p = 0; p < common.machine.proc_count(); p++) {
    for (int ml = 0; ml < common.mem_size; ml++) {
      cmp = comb_comp(entailment_compare_buffer(write_buffers[p][ml], pwsc.write_buffers[p][ml]), cmp);
      if (cmp == Constraint::INCOMPARABLE) return cmp;
    }
  }
  return cmp;
}

Constraint::Comparison PwsConstraint::entailment_compare_buffer(const Store& a, const Store& b) const {
  if (a.size() == b.size()) {
    return a.entailment_compare(b);
  } else if (a.size() > b.size()) {
    return invert_comp(entailment_compare_buffer(b, a));
  }
  // a is either LESS than b, or INCOMPARABLE.
  if (a.size() == 0) return INCOMPARABLE; // a must be empty iff b is
  // a and b must end with the same value
  if (a[a.size() - 1] != b[b.size() - 1] && a[a.size() - 1] != value_t::STAR) return INCOMPARABLE;
  // We check if a is a subword of b
  for (int ai = 0, bi = 0; ai < a.size(); bi++) {
    if (bi == b.size()) return INCOMPARABLE; // a isn't a subword of b
    if (a[ai] == b[bi] || a[ai] == value_t::STAR) //TODO: Ponder the correctness of this
      ai++; // They match
  }
  return LESS; // a is a subword of b
}

std::vector<std::pair<int, Lang::NML> > PwsConstraint::get_filled_buffers() const {
  std::vector<std::pair<int, Lang::NML> > res;
  for (unsigned p = 0; p < write_buffers.size(); ++p) {
    for (Lang::NML nml : common.nmls) {
      if (write_buffers[p][common.index(nml)].size() > 0)
        res.push_back(std::pair<int, Lang::NML>(p, nml));
    }
  }
  return res;
}

bool PwsConstraint::unreachable() {
  if (PwsConstraint::use_pending_sets) {
    // Check each message that is ahead of the writing process' cpointer
    for (int ci = channel.size()-1; ci >= 0; --ci) {
      int pid = channel[ci].wpid;
      if (ci > cpointers[pid]) {
        for (Lang::NML nml : channel[ci].nmls) {
          if (!common.pending_set[pid][pcs[pid]].count(nml)) return true;
          value_t possible_pending = common.pending_set[pid][pcs[pid]][nml];
          if (possible_pending != value_t::STAR) {
            int nmli = common.index(nml);
            if (channel[ci].store[nmli] == value_t::STAR)
              channel[ci].store = channel[ci].store.assign(nmli, possible_pending);
            else if (channel[ci].store[nmli] != possible_pending)
              return true;
          }
        }
      }
    }
  }

  struct map_sequence_ref {
    map_sequence_ref(const Common::map_sequence &ms)
    : first(ms.first), second(ms.second), ptr(&ms) {};
    std::list<std::map<Lang::NML, value_t> > first, second;
    const Common::map_sequence *ptr;
    /* Get the value of nml from ptr.
     * Pre: nml is in this sequence. */
    value_t get_value(Lang::NML nml) const {
      auto pred = [nml](const std::map<Lang::NML, value_t> &map) { return map.count(nml); };
      auto secondresult = std::find_if(ptr->second.rbegin(), ptr->second.rend(), pred);
      if (secondresult != ptr->second.rend()) return secondresult->at(nml);
      return std::find_if(ptr->first.rbegin(), ptr->first.rend(), pred)->at(nml);
    }
  };

  if (use_last_write_sets) {
    for (int pid = 0; pid < int(common.machine.automata.size()); ++pid) {
      /* We will be removing elements that do not apply, and also modifying
       * elements as we go, so we make a copy. */
      std::list<map_sequence_ref> cases;
      std::copy(common.last_write_sets[pid][pcs[pid]].begin(),
                common.last_write_sets[pid][pcs[pid]].end(),
                std::back_inserter(cases));
      /* These memory locations have already been considered. We map them to the
       * channel index where they were found, -1 if they were found in the write
       * buffers or -2 if they were found in the channel but belonged to another
       * process. */
      std::map<Lang::NML, int> considered;

      /* We begin with the buffers. */
      for (Lang::NML nml : common.nmls) {
        int nmli = common.index(nml);
        const Store &buffer = write_buffers[pid][nmli];
        if (buffer.size() > 0) {
          value_t buffer_value = buffer.back();
          /* Remove cases that do not unify with message_value. */
          for (auto case_iter = cases.begin(); case_iter != cases.end();)
            if (case_iter->second.back().count(nml) &&
                case_iter->second.back().at(nml).unifiable(buffer_value)) {
              case_iter->second.back().erase(nml);
              ++case_iter;
            } else {
              /* This case does not apply. */
              case_iter = cases.erase(case_iter);
            }
          considered.insert({nml, -1});
          if (cases.empty()) return true;
        }
      }

      /* If the last set was emptied above, we remove it if there is more sets
       * in the sequence. */
      for (map_sequence_ref &seq : cases)
        if (seq.second.back().empty() && seq.second.size() > 1)
          seq.second.pop_back();

      /* Now we consider the channel. */
      for (int i = int(channel.size())-1; i >= 0; --i) {
        if (i == cpointers[pid]) {
          /* We heave all elements from first into second of the tuple, except
           * the empty set that might exist at the end of first. */
          for (map_sequence_ref &seq : cases) {
            if (seq.first.back().empty())
              seq.first.pop_back();
            seq.second.splice(seq.second.begin(), seq.first);
            /* Incase second only contained a single empty map, we remove it,
             * unless it is the only map in the new sequence */
            if (seq.second.back().empty() && seq.second.size() > 1)
              seq.second.pop_back();
          }
        }
        assert(channel[i].nmls.size() <= 1);
        if (channel[i].nmls.empty()) {
          /* This is the dummy message. It is only allowed as the very first
           * message. */
          if (i > 0) return true;
          /* A case must be empty for the dummy message to be allowed since no
           * messages older than it can exist and thus be lost. */
          cases.remove_if([](const map_sequence_ref &seq) {
              return !(seq.first.size() == 0 ||
                       (seq.first.size() == 1 && seq.first.begin()->empty())) ||
                     !(seq.second.size() == 1 && seq.second.begin()->empty());
            });
        } else {
          Lang::NML nml = channel[i].nmls.get_vector()[0];
          if (!considered.count(nml)) {
            if (channel[i].wpid == pid) {
              value_t message_value = channel[i].store[common.index(nml)];
              /* Remove cases that do not unify with message_value. */
              for (auto case_iter = cases.begin(); case_iter != cases.end();)
                if (case_iter->second.back().count(nml) &&
                    case_iter->second.back().at(nml).unifiable(message_value)) {
                  case_iter->second.back().erase(nml);
                  /* In case we have emptied the set, we move on to the next set
                   * in the sequence. */
                  if (case_iter->second.back().empty() && case_iter->second.size() > 1)
                    case_iter->second.pop_back();
                  ++case_iter;
                } else {
                  /* This case does not apply. */
                  case_iter = cases.erase(case_iter);
                }
              considered.insert({nml, i});
              if (cases.empty()) return true;
            } else {
              /* We have found the most recent write to nml that process pid can
               * see. Beyond this point writes to nml might have been lost (as
               * per the entailment relation). We can no longer do anything
               * about writes to nml, so we mark it considered and remove it
               * from all sequences. */
              if (i <= cpointers[pid]) {
                considered.insert({nml, -2});
                for (map_sequence_ref &seq : cases) {
                  for (auto &map : seq.second) map.erase(nml);
                  if (seq.second.back().empty() && seq.second.size() > 1)
                    seq.second.pop_back();
                }
              }
            }
          }
        }
      }

      /* No case applied to this constraint - it is unreachable. */
      if (cases.empty()) return true;
      /* If all cases agree on a value, use that value for the constraint */
      for (const std::pair<Lang::NML, int> &pair : considered) {
        if (pair.second == -2) continue; // We cannot do anything about other processes' writes
        ZStar<int> value;
        bool agrees = true;
        for (const map_sequence_ref &seq : cases)
          value = seq.get_value(pair.first).unify(value, &agrees);
        if (agrees && !value.is_star()) {
          int nmli = common.index(pair.first);
          if (pair.second == -1) {
            write_buffers[pid][nmli] =
              write_buffers[pid][nmli].assign(write_buffers[pid][nmli].size() - 1, value);
          } else {
            channel[pair.second].store = channel[pair.second].store.assign(nmli, value);
          }
        }
      }
    }
  }

  if (use_channel_suffix_equality) {
    /* For each memory location, check that in the suffix of channel
     * to the right of the rightmost message updating that memory
     * location, all messages can agree on a single value for that
     * memory location.
     */
    for (auto nml : common.nmls) {
      if (!propagate_value_in_channel(nml)) return true;
    }
  }

  return false;
}

bool PwsConstraint::propagate_value_in_channel(const Lang::NML &nml, int nmli) {
  if (nmli < 0) nmli = common.index(nml);
  for (const std::vector<Store> &processwb : write_buffers)
    // This write is the last write and disallows any write propagation.
    if (processwb[nmli].size() > 0) return true;
  return ChannelConstraint::propagate_value_in_channel(nml, nmli);
}

int PwsConstraint::get_weight() const {
    int buffered_values = 0;
    for (const std::vector<Store> &pwrite_buffer : write_buffers)
      for (const Store &store : pwrite_buffer)
        buffered_values += store.size();
    return channel.size() + buffered_values;
}
