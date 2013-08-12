/*
 * Copyright (C) 2013 Carl Leonardsson, Magnus Lång
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

#include "channel_constraint.h"

/**************************/
/* ChannelConstraint::Msg */
/**************************/

std::string ChannelConstraint::Msg::to_short_string(const Common &common) const{
  std::stringstream ss;
  ss << "<P" << wpid << ", ";
  if(nmls.size() == 1){
    ss << common.machine.pretty_string_nml.at(nmls[0]);
  }else{
    ss << "[";
    for(int i = 0; i < nmls.size(); ++i){
      if(i != 0) ss << ", ";
      ss << common.machine.pretty_string_nml.at(nmls[i]);
    }
    ss << "]";
  }
  ss << ", {";
  bool first_var = true;
  for(int i = 0; i < common.gvar_count; ++i){
    if(!first_var) ss << ", ";
    first_var = false;
    ss << common.machine.gvars[i].name << "=";
    if(store[common.index(Lang::NML::global(i))] == value_t::STAR){
      ss << "*";
    }else{
      ss << store[common.index(Lang::NML::global(i))];
    }
  }
  for(unsigned p = 0; p < common.reg_count.size(); ++p){
    for(unsigned i = 0; i < common.machine.lvars[p].size(); ++i){
      if(!first_var) ss << ", ";
      first_var = false;
      ss << common.machine.lvars[p][i].name << "[P" << p << "]=";
      if(store[common.index(Lang::NML::local(i,p))] == value_t::STAR){
        ss << "*";
      }else{
        ss << store[common.index(Lang::NML::local(i,p))];
      }
    }
  }
  ss << "}>";
  return ss.str();
};

int ChannelConstraint::Msg::compare(const Msg &msg) const{
  if(wpid < msg.wpid){
    return -1;
  }else if(wpid > msg.wpid){
    return 1;
  }

  if(nmls < msg.nmls){
    return -1;
  }else if(nmls > msg.nmls){
    return 1;
  }

  return store.compare(msg.store);
};

/*****************************/
/* ChannelConstraint::Common */
/*****************************/

ChannelConstraint::Common::Common(const Machine &m)
  : machine(m) {
  gvar_count = machine.gvars.size();
  max_lvar_count = 0;
  for(unsigned p = 0; p < machine.lvars.size(); ++p){
    max_lvar_count = std::max<int>(max_lvar_count, machine.lvars[p].size());
    reg_count.push_back(machine.regs[p].size());
  }
  mem_size = gvar_count + machine.automata.size()*max_lvar_count;

  /* Setup messages */
  {
    /* Insert a dummy message */
    messages.insert(MsgHdr(0,VecSet<Lang::NML>()));
    /* Add messages for all writes */
    for(unsigned p = 0; p < machine.automata.size(); ++p){
      const std::vector<Automaton::State> &states = machine.automata[p].get_states();
      for(unsigned i = 0; i < states.size(); ++i){
        for(auto it = states[i].fwd_transitions.begin(); it != states[i].fwd_transitions.end(); ++it){
          VecSet<VecSet<Lang::MemLoc<int> > > wss = (*it)->instruction.get_write_sets();
          for(auto wsit = wss.begin(); wsit != wss.end(); ++wsit){
            if(wsit->size() > 0){
              VecSet<Lang::NML> nmls;
              for(auto wit = wsit->begin(); wit != wsit->end(); ++wit){
                nmls.insert(Lang::NML(*wit,p));
                this->nmls.insert(Lang::NML(*wit,p));
              }
              messages.insert(MsgHdr(p,nmls));
            }
          }
        }
      }
    }
  }
}

ChannelConstraint::Store ChannelConstraint::Common::store_of_write(const Machine::PTransition &t) const{
  Store store(mem_size);
  if(t.instruction.get_type() == Lang::WRITE && t.instruction.get_expr().is_integer()){
    store = store.assign(index(Lang::NML(t.instruction.get_memloc(),t.pid)),
                         t.instruction.get_expr().get_integer());
  }else if((t.instruction.get_type() == Lang::LOCKED ||
            t.instruction.get_type() == Lang::SLOCKED) &&
           t.instruction.get_statement_count() == 1 &&
           t.instruction.get_statement(0)->get_type() == Lang::WRITE &&
           t.instruction.get_statement(0)->get_expr().is_integer()){
    store = store.assign(index(Lang::NML(t.instruction.get_statement(0)->get_memloc(),t.pid)),
                         t.instruction.get_statement(0)->get_expr().get_integer());
  }else if(t.instruction.get_type() == Lang::LOCKED &&
           t.instruction.get_statement_count() == 1 &&
           t.instruction.get_statement(0)->get_type() == Lang::SEQUENCE) {
    const Lang::Stmt<int> &seq = *t.instruction.get_statement(0);
    if (seq.get_statement_count() == 2 &&
        seq.get_statement(0)->get_type() == Lang::READASSERT &&
        seq.get_statement(1)->get_type() == Lang::WRITE &&
        seq.get_statement(1)->get_expr().is_integer()){
      store = store.assign(index(Lang::NML(seq.get_statement(1)->get_memloc(), t.pid)),
                           seq.get_statement(1)->get_expr().get_integer());
    }
  }
  /* Generalize */
  return store;
};

/*********************/
/* ChannelConstraint */
/*********************/

ChannelConstraint::ChannelConstraint(std::vector<int> pcs, const Common::MsgHdr &msg, Common &c)
  : common(c), pcs(pcs) {
  /* Initialize the channel with a single STAR filled message. */
  channel.push_back(Msg(Store(common.mem_size),msg.wpid,msg.nmls));
  cpointers = std::vector<int>(pcs.size(),0);
  /* Initialize registers with STARs */
  for(unsigned p = 0; p < pcs.size(); p++){
    reg_stores.push_back(Store(common.reg_count[p]));
  }
};


bool ChannelConstraint::is_init_state() const{
  for(unsigned p = 0; p < pcs.size(); ++p){
    if(pcs[p] != 0){
      return false;
    }
  }
  if(channel.size() != 1){
    /* Note that channel.size() == 1 also implies that cpointers[p] ==
     * 0 for all processes p. */
    return false;
  }
  /* Check all memory locations against their intended initial values */
  const Store &str = channel[0].store;
  for(unsigned i = 0; i < common.machine.gvars.size(); ++i){
    assert(common.index(Lang::NML::global(i)) == int(i));
    if(str[i] != value_t::STAR && !common.machine.gvars[i].value.is_wild() &&
       str[i].get_int() != common.machine.gvars[i].value.get_value()){
      return false;
    }
  }
  for(unsigned p = 0; p < pcs.size(); ++p){
    for(unsigned i = 0; i < common.machine.lvars[p].size(); ++i){
      int ix = common.index(Lang::NML::local(i,p));
      if(str[ix] != value_t::STAR && !common.machine.lvars[p][i].value.is_wild() &&
         str[ix].get_int() != common.machine.lvars[p][i].value.get_value()){
        return false;
      }
    }
  }
  /* Check all registers against their intended initial values */
  for(unsigned p = 0; p < reg_stores.size(); ++p){
    for(int i = 0; i < common.reg_count[p]; ++i){
      if(reg_stores[p][i] != value_t::STAR && !common.machine.regs[p][i].value.is_wild() &&
         reg_stores[p][i].get_int() != common.machine.regs[p][i].value.get_value()){
        return false;
      }
    }
  }
  return true;
};

std::string ChannelConstraint::to_string() const noexcept {
  std::stringstream ss;
  for(unsigned p = 0; p < pcs.size(); ++p){
    process_to_string(p, ss);
    ss << "\n";
  }
  ss << "Channel: [";
  for(unsigned i = 0; i < channel.size(); ++i){
    if(i != 0){
      ss << ", ";
    }
    ss << channel[i].to_short_string(common);
  }
  ss << "]\n";
  return ss.str();
};

void ChannelConstraint::process_to_string(int p, std::stringstream &ss) const noexcept {
  ss << "P" << p << " @Q" << pcs[p] << " {cpointer=" << cpointers[p];
  for(int r = 0; r < common.reg_count[p]; ++r){
    ss << ", " << common.machine.regs[p][r].name << "=";
    if(reg_stores[p][r] == value_t::STAR){
      ss << "*";
    }else{
      ss << reg_stores[p][r];
    }
  }
  ss << "}";
};

int ChannelConstraint::index_of_read(Lang::NML nml, int pid) const{
  int i = channel.size()-1;
  while(i > cpointers[pid]){
    if(channel[i].wpid == pid && channel[i].nmls.count(nml)){
      return i;
    }
    i--;
  }
  return i;
};

std::vector<ChannelConstraint*> ChannelConstraint::channel_pop_back() const{
#ifndef NDEBUG
  for(unsigned p = 0; p < cpointers.size(); ++p){
    assert(cpointers[p] < int(channel.size())-1);
  }
#endif
  /* Treat the special case when channel.back().nmls.size() == 1 in a more efficient manner. */
  if(channel.back().nmls.size() == 1){
    Lang::NML nml = channel.back().nmls[0];
    int nmli = common.index(nml);
    int wpid = channel.back().wpid;

    Msg new_msg(Store(channel.back().store.size()),wpid,VecSet<Lang::NML>::singleton(nml));
    std::vector<ChannelConstraint*> res;
    ChannelConstraint *chc;
    bool last_unifiable;

    std::vector<Msg> ch0(channel);
    ch0.pop_back();
    ch0[ch0.size()-1].store = ch0[ch0.size()-1].store.unify(channel.back().store.assign(nmli,value_t::STAR),&last_unifiable);

    if(last_unifiable){
      /* The case when there is no hidden message */
      chc = this->clone();
      chc->channel = ch0;
      res.push_back(chc);

      /* Guess a position to reinsert a hidden message. */
      for(int i = int(ch0.size())-1; i > 0; i--){
        std::vector<Msg> ch1(ch0);
        ch1.push_back(ch0.back());
        for(int j = int(ch1.size()) - 2; j > i; j--){
          ch1[j] = ch1[j-1];
        }
        ch1[i] = new_msg;
        chc = this->clone();
        chc->channel = ch1;
        /* Update cpointers */
        for(unsigned p = 0; p < chc->cpointers.size(); ++p){
          if(chc->cpointers[p] >= i){
            ++chc->cpointers[p];
          }
        }
        res.push_back(chc);
        if(i > 0 && ch0[i-1].wpid == wpid && ch0[i-1].nmls.count(nml)){
          break;
        }
      }
    }

    /* The case when the hidden message is the last of the new channel */
    {
      chc = this->clone();
      chc->channel.back().store = chc->channel.back().store.assign(nmli,value_t::STAR);
      res.push_back(chc);
    }

    return res;
  }else{
    throw new std::logic_error("ChannelConstraint::channel_pop_back: Support for multiple memory locations not implemented.");
  }
};

Constraint::Comparison ChannelConstraint::entailment_compare(const Constraint &c) const{
  assert(dynamic_cast<const ChannelConstraint*>(&c));
  return entailment_compare_impl(static_cast<const ChannelConstraint&>(c));
};

Constraint::Comparison ChannelConstraint::entailment_compare_impl(const ChannelConstraint &chc) const{
  if(pcs != chc.pcs){
    return Constraint::INCOMPARABLE;
  }

  Constraint::Comparison cmp = Constraint::EQUAL;

  for(unsigned p = 0; p < reg_stores.size(); ++p){
    cmp = Constraint::comb_comp(cmp,reg_stores[p].entailment_compare(chc.reg_stores[p]));
    if(cmp == Constraint::INCOMPARABLE) return cmp;
  }

  return entailment_compare_channels(chc,cmp);
};

Constraint::Comparison ChannelConstraint::entailment_compare_channels(const ChannelConstraint &sbc, Constraint::Comparison cmp) const{
  if(channel.size() == sbc.channel.size()){
    /* Each message in the channel must match the corresponding message in the other channel */
    if(cpointers != sbc.cpointers){
      return Constraint::INCOMPARABLE;
    }
    for(unsigned i = 0; i < channel.size(); ++i){
      cmp = Constraint::comb_comp(cmp,channel[i].entailment_compare(sbc.channel[i]));
      if(cmp == Constraint::INCOMPARABLE) return cmp;
    }
    return cmp;
  }else{
    if(channel.size() > sbc.channel.size()){
      return Constraint::invert_comp(sbc.entailment_compare_channels(*this,Constraint::invert_comp(cmp)));
    }else{
      /* this->channel should be a strict subword of sbc.channel */
      if(Constraint::comb_comp(cmp,Constraint::LESS) == Constraint::INCOMPARABLE){
        return Constraint::INCOMPARABLE;
      };
      std::vector<VecSet<VecSet<Lang::NML> > > has_written(pcs.size());
      for(unsigned p = 0; p < pcs.size(); ++p){
        has_written[p].reserve(channel.size());
      }
      int j = int(sbc.channel.size())-1;
      int i = int(channel.size())-1;
      while(i >= 0){
        if(j < i){
          /* There are not enough messages left in sbc.channel to
           * match the ones in this->channel */
          return Constraint::INCOMPARABLE;
        }
        bool necessary_match = ((i == j) || // otherwise there is not enough messages left in sbc.channel
                                // sbc.channel[j] is a strong message
                                (has_written[sbc.channel[j].wpid].count(sbc.channel[j].nmls) == 0)); 
        bool necessary_pass = false;
        /* Check cpointers */
        for(unsigned p = 0; p < cpointers.size(); ++p){
          if(cpointers[p] == i && sbc.cpointers[p] == j){
            necessary_match = true;
          }else if(cpointers[p] == i){
            necessary_pass = true;
          }else if(sbc.cpointers[p] == j){
            return Constraint::INCOMPARABLE;
          }
        }
        if(necessary_match && necessary_pass){
          return Constraint::INCOMPARABLE;
        }
        if(necessary_match){
          if(Constraint::comb_comp(Constraint::LESS,channel[i].entailment_compare(sbc.channel[j]))){
            return Constraint::INCOMPARABLE;
          }
          has_written[channel[i].wpid].insert(channel[i].nmls);
          --i;
          --j;
        }else if(necessary_pass){
          --j;
        }else{
          if(Constraint::comb_comp(Constraint::LESS,channel[i].entailment_compare(sbc.channel[j]))){
            /* No match */
            --j;
          }else{
            /* Match */
            has_written[channel[i].wpid].insert(channel[i].nmls);
            --i;
            --j;
          }
        }
      }
      assert(i == -1 && j >= i);
      return Constraint::LESS;
    }
  }
};

std::vector<ChannelConstraint::MsgCharacterization> ChannelConstraint::characterize_channel() const{
  std::vector<MsgCharacterization> v; // Build the vector backwards, turn it around before returning
  v.reserve(channel.size());

  std::vector<VecSet<VecSet<Lang::NML> > > has_written(pcs.size());
  for(unsigned p = 0; p < pcs.size(); ++p){
    has_written[p].reserve(channel.size());
  }
  for(int i = int(channel.size())-1; i >= 0; --i){
    VecSet<int> cps;
    for(unsigned p = 0; p < cpointers.size(); ++p){
      if(cpointers[p] == i){
        cps.insert(p);
      }
    }
    if(cps.size() > 0 || has_written[channel[i].wpid].count(channel[i].nmls) == 0){
      v.push_back(MsgCharacterization(channel[i].wpid,channel[i].nmls,cps));
      has_written[channel[i].wpid].insert(channel[i].nmls);
    }
  }

  /* Turn v backwards */
  std::vector<MsgCharacterization> w;
  w.reserve(v.size());
  for(unsigned i = 0; i < v.size(); i++){
    w.push_back(v[v.size() - i - 1]);
  }
  return w;
};

bool ChannelConstraint::propagate_value_in_channel(const Lang::NML &nml, int nmli){
  if(nmli < 0){
    nmli = common.index(nml);
  }
  return propagate_value_in_channel(&channel,nml,nmli);
}

bool ChannelConstraint::propagate_value_in_channel(std::vector<Msg> *ch, const Lang::NML &nml, int nmli){
  value_t val = value_t::STAR;
  int i;
  for(i = int(ch->size())-1; i >= 0; --i){
    if(val == value_t::STAR){
      val = (*ch)[i].store[nmli];
    }else{
      value_t val2 = (*ch)[i].store[nmli];
      if(val2 != value_t::STAR && val2 != val){
        return false;
      }
    }
    if((*ch)[i].nmls.count(nml) > 0){
      /* Found the last write to nml */
      /* Stop searching */
      break;
    }
  }
  /* Update the channel */
  if(val != value_t::STAR){
    if(i == -1) i = 0;
    for( ; i < int(ch->size()); ++i){
      if((*ch)[i].store[nmli] == value_t::STAR){
        (*ch)[i].store = (*ch)[i].store.assign(nmli,val);
      }
    }
  }
  return true;
}
