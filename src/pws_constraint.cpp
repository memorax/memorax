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
#include <iterator>

/*****************/
/* Configuration */
/*****************/

const bool PwsConstraint::use_channel_suffix_equality = true;
const bool PwsConstraint::use_limit_other_updates = true;
const bool PwsConstraint::use_pending_sets = true;
const bool PwsConstraint::use_serialisations_only_after_writes = true;

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
        if (t->instruction.get_writes().size() > 0 && !t->instruction.is_fence()) has_writes[p][i] = true;

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
    transitions_by_pc.push_back(std::vector<std::vector<const Machine::PTransition*> >(machine.automata[p].get_states().size()));
  }
  for(unsigned i = 0; i < all_transitions.size(); ++i){
    transitions_by_pc[all_transitions[i].pid][all_transitions[i].target].push_back(&all_transitions[i]);
  }

  auto is_sfence = [](const Lang::Stmt<int> &stmt, Lang::MemLoc<int> ml) {
    return (stmt.get_type() == Lang::SFENCE ||
            stmt.get_type() == Lang::MFENCE ||
            (stmt.get_type() == Lang::LOCKED &&
             std::find(stmt.get_writes().begin(), stmt.get_writes().end(), ml)
             != stmt.get_writes().end()));

  };

  auto is_mfence = [](const Lang::Stmt<int> &stmt, Lang::MemLoc<int> ml) {
    return (stmt.get_type() == Lang::MFENCE ||
            stmt.get_type() == Lang::LOCKED ||
            (stmt.get_type() == Lang::LOCKED &&
             std::find(stmt.get_writes().begin(), stmt.get_writes().end(), ml)
             != stmt.get_writes().end()));
  };

  /* Setup pending_set and pending_buffers */
  for (unsigned p = 0; p < machine.automata.size(); ++p) {
    const std::vector<Automaton::State> &states = machine.automata[p].get_states();
    init_pending(is_mfence, states, pending_set);
    init_pending(is_sfence, states, pending_buffers);

    /* Use a fix-point iteration to find the complete sets. */
    bool changed = true;
    while (changed) {
      changed = false;
      const std::vector<Automaton::State> &states = machine.automata[p].get_states();
      for (unsigned i = 0; i < states.size(); ++i) {
        /* Complete pending_set and pending_buffers. */
        iterate_pending(is_mfence, states, i, p, pending_set[p],     changed);
        iterate_pending(is_sfence, states, i, p, pending_buffers[p], changed);
      }
    }
  }
  Log::extreme << "Pending sets\n";
  for (unsigned p = 0; p < pending_set.size(); ++p) {
    Log::extreme << "For process P" << p << "\n";
    for (unsigned i = 0; i < pending_set[p].size(); ++i) {
      Log::extreme << "  Q" << i << " {";
      for (std::pair<Lang::NML, value_t> elem : pending_set[p][i])
        Log::extreme << machine.pretty_string_nml.find(elem.first)->second << ":" << elem.second << ", ";
      Log::extreme << "} buffer: {";
      for (std::pair<Lang::NML, value_t> elem : pending_buffers[p][i])
        Log::extreme << machine.pretty_string_nml.find(elem.first)->second << ":" << elem.second << ", ";
      Log::extreme << "}\n";
    }
  }
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
  if(use_limit_other_updates){
    for(unsigned p = 0; p < pcs.size(); ++p){
      bool has_read = false;
      for(auto it = common.transitions_by_pc[p][pcs[p]].begin(); it != common.transitions_by_pc[p][pcs[p]].end(); ++it){
        if((*it)->instruction.get_reads().size()){
          has_read = true;
        }
      }
      for(auto it = common.transitions_by_pc[p][pcs[p]].begin(); it != common.transitions_by_pc[p][pcs[p]].end(); ++it){
        if((*it)->instruction.get_type() != Lang::UPDATE || has_read || cpointers[p] == int(channel.size())-1){
          l.push_back(*it);
        }
      }
    }
  }else{
    for(unsigned p = 0; p < pcs.size(); ++p){
      l.insert(l.end(),common.transitions_by_pc[p][pcs[p]].begin(),common.transitions_by_pc[p][pcs[p]].end());
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
    VecSet<int> val_nml;
    int buffer_size = write_buffers[t.pid][nmli].size();
    if (buffer_size > 0) {
      value_t bufferValue = write_buffers[t.pid][nmli][buffer_size - 1];
      val_nml = possible_values(bufferValue, nml);
    } else {
      val_nml = ChannelConstraint::possible_values(channel[msgi].store, nml);
    }
    for (int i = 0; i < val_nml.size(); i++) {
      /* For each possible value for nml, try to pair it with an
       * assignment to the reg_store such that s.expr() evaluates to
       * the same value. */
      VecSet<Store> stores = possible_reg_stores(reg_stores[t.pid], t.pid, s.get_expr(), val_nml[i]);
      for (int j = 0; j < stores.size(); ++j) {
        PwsConstraint *pwsc = this->clone();
        pwsc->pcs[t.pid] = t.source;
        if (buffer_size > 0) pwsc->write_buffers[t.pid][nmli] =
                             pwsc->write_buffers[t.pid][nmli].assign(buffer_size - 1, val_nml[i]);
        else pwsc->channel[msgi].store = pwsc->channel[msgi].store.assign(nmli, val_nml[i]);
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
    if (cpointers[t.pid] != int(channel.size()) - 1)  break;
  case Lang::SFENCE:
    if (!is_fully_serialised(t.pid)) break;
    /* All conditions are fullfilled, fall through to NOP */
  case Lang::NOP: {
    PwsConstraint *pwsc = this->clone();
    pwsc->pcs[t.pid] = t.source;
    res.push_back(pwsc);
  } break;

  default:
    Log::debug << "PwsConstraint::pre: Received unknown type of statement: " << s.get_type() << "\n";
    std::stringstream ss;
    ss << "PwsConstraint::pre: Statement \""
       << s.to_string(common.machine.reg_pretty_vts(t.pid), common.machine.ml_pretty_vts(t.pid))
       << "\" of not implemented type\n P"
       << t.pid << " Q" << t.source << "->Q" << t.target;
    throw new std::logic_error(ss.str());
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
  Constraint::Comparison cmp = ChannelConstraint::entailment_compare(pwsc);
  if (cmp == Constraint::INCOMPARABLE) return cmp;
  return comb_comp(entailment_compare_buffers(pwsc), cmp);
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
    for (uint pid = 0; pid < common.machine.automata.size(); ++pid) {
      for (Lang::NML nml : common.nmls) {
        int nmli = common.index(nml);
        if (write_buffers[pid][nmli].size() > 0) {
          if (!common.pending_buffers[pid][pcs[pid]].count(nml)) return true;
          value_t possible_pending = common.pending_buffers[pid][pcs[pid]][nml];
          if (possible_pending != value_t::STAR) {
            for (int i = 0; i < write_buffers[pid][nmli].size(); ++i) {
              if (write_buffers[pid][nmli][i] == value_t::STAR)
                write_buffers[pid][nmli] = write_buffers[pid][nmli].assign(i, possible_pending);
              else if (write_buffers[pid][nmli][i] != possible_pending)
                return true;
            }
          }
        }
      }
    }

    // Check each message that is ahead of the writing process's cpointer
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
