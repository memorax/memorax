/*
 * Copyright (C) 2012 Carl Leonardsson
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

#include "sb_constraint.h"

/*****************/
/* Configuration */
/*****************/

const bool SbConstraint::use_last_msg = false;
const bool SbConstraint::use_last_msgs_vec = false;
const bool SbConstraint::use_updates_only_after_reads = true;
const bool SbConstraint::use_channel_suffix_equality = true;
const bool SbConstraint::use_can_have_pending = true;
const bool SbConstraint::use_limit_other_updates = true;

/*****************/

SbConstraint::Common::Common(const Machine &m)
  : machine(m)
{
  /* Check that all variables and registers have finite domains */
  for(unsigned i = 0; i < m.gvars.size(); ++i){
    if(m.gvars[i].domain.is_int()){
      throw new std::logic_error("SbConstraint::Common: SbConstraint requires finite domains. Infinite domain for global variable "+
                                 m.gvars[i].name);
    }
  }
  for(unsigned p = 0; p < m.lvars.size(); ++p){
    for(unsigned i = 0; i < m.lvars[p].size(); ++i){
      if(m.lvars[p][i].domain.is_int()){
        std::stringstream ss;
        ss << "SbConstraint::Common: SbConstraint requires finite domains. Infinite domain for local variable "
           << m.lvars[p][i].name << "[P" << p << "]";
        throw new std::logic_error(ss.str());
      }
    }
    for(unsigned i = 0; i < m.regs[p].size(); ++i){
      if(m.regs[p][i].domain.is_int()){
        std::stringstream ss;
        ss << "SbConstraint::Common: SbConstraint requires finite domains. Infinite domain for register "
           << "P" << p << ":" << m.regs[p][i].name;
        throw new std::logic_error(ss.str());
      }
    }
  }

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

  /* Setup all_transitions */
  for(unsigned p = 0; p < machine.automata.size(); ++p){
    const std::vector<Automaton::State> &states = machine.automata[p].get_states();
    for(unsigned i = 0; i < states.size(); ++i){
      bool has_reads = false;
      /* Actual machine transitions */
      for(auto it = states[i].bwd_transitions.begin(); it != states[i].bwd_transitions.end(); ++it){
        all_transitions.push_back(Machine::PTransition(**it,p));
        if(use_updates_only_after_reads && (*it)->instruction.get_writes().size() > 0 && !(*it)->instruction.is_fence()){
          /* For non-locked writes, also add the option of locked execution */
          Lang::Stmt<int> aw = Lang::Stmt<int>::locked_block(std::vector<Lang::Stmt<int> >(1,(*it)->instruction),
                                                             (*it)->instruction.get_pos());
          all_transitions.push_back(Machine::PTransition((*it)->source, aw, (*it)->target,p));
        }
        if((*it)->instruction.get_reads().size() > 0 && !(*it)->instruction.is_fence()){
          has_reads=true;
        }
      }
      /* Add updates */
      for(auto it = messages.begin(); it != messages.end(); ++it){
        if(it->nmls.size() > 0){ /* Catch all messages except the dummy message */
          VecSet<Lang::MemLoc<int> > mls;
          for(auto nmlit = it->nmls.begin(); nmlit != it->nmls.end(); ++nmlit){
            mls.insert(nmlit->localize(p));
          }
          if(it->wpid != int(p) || (!use_updates_only_after_reads || has_reads)){
            all_transitions.push_back(Machine::PTransition(i,Lang::Stmt<int>::update(it->wpid,mls),i,p));
          }
        }
      }
    }
  }

  /* Setup transitions_by_pc */
  for(unsigned p = 0; p < machine.automata.size(); ++p){
    transitions_by_pc.push_back(std::vector<std::vector<const Machine::PTransition*> >(machine.automata[p].get_states().size()));
  }
  for(unsigned i = 0; i < all_transitions.size(); ++i){
    transitions_by_pc[all_transitions[i].pid][all_transitions[i].target].push_back(&all_transitions[i]);
  }

  /* Setup last_msgs */
  Log::debug << "Last messages:\n";
  for(unsigned p = 0; p < machine.automata.size(); ++p){
    last_msgs.push_back(std::vector<VecSet<Msg> >(machine.automata[p].get_states().size(),
                                                  VecSet<Msg>()));
    can_have_pending.push_back(std::vector<bool>(machine.automata[p].get_states().size(),false));
  }
  /* Make sure the dummy message is available at the start control state */
  last_msgs[0][0].insert(Msg(Store(mem_size),0,VecSet<Lang::NML>()));
  for(unsigned p = 0; p < machine.automata.size(); ++p){
    /* Populate */
    const std::vector<Automaton::State> &states = machine.automata[p].get_states();
    for(unsigned i = 0; i < states.size(); ++i){
      for(auto it = states[i].bwd_transitions.begin(); it != states[i].bwd_transitions.end(); ++it){
        /* Note that it does not matter whether this instruction is locked or not. */
        VecSet<VecSet<Lang::MemLoc<int> > > ws = (*it)->instruction.get_write_sets();
        for(auto wit = ws.begin(); wit != ws.end(); ++wit){
          if(wit->size() > 0){
            VecSet<Lang::NML> S;
            for(int j = 0; j < wit->size(); ++j){
              S.insert(Lang::NML((*wit)[j],p));
            }
            last_msgs[p][i].insert(Msg(store_of_write(Machine::PTransition(**it,p)),p,S));
            if(!(*it)->instruction.is_fence()){
              can_have_pending[p][i] = true;
            }
          }
        }
      }
    }
    
    /* Use a fix-point iteration to find the complete sets. */
    bool changed = true;
    while(changed){
      changed = false;
      const std::vector<Automaton::State> &states = machine.automata[p].get_states();
      for(unsigned i = 0; i < states.size(); ++i){
        for(auto it = states[i].bwd_transitions.begin(); it != states[i].bwd_transitions.end(); ++it){
          if((*it)->instruction.get_writes().empty()){
            int nw = last_msgs[p][i].insert(last_msgs[p][(*it)->source]);
            bool chm_change = !can_have_pending[p][i] && can_have_pending[p][(*it)->source];
            can_have_pending[p][i] = can_have_pending[p][i] || can_have_pending[p][(*it)->source];
            changed = changed || (nw > 0) || chm_change;
          }
        }
      }
    }
    /* Printing */
    Log::debug << "For process P" << p << "\n";
    for(unsigned i = 0; i < states.size(); ++i){
      std::function<std::string(const Msg&)> f = [this](const Msg &msg){
        return msg.to_short_string(*this);
      };
      Log::debug << "  Q" << i << last_msgs[p][i].to_string_one_line(f);
      if(!can_have_pending[p][i]){
        Log::debug << " (channel must be empty)";
      }
      Log::debug << "\n";
    }
    Log::debug << std::flush;
  }

  /* Setup last_msgs_vec */
  for(unsigned p = 0; p < machine.automata.size(); ++p){
    const std::vector<Automaton::State> &states = machine.automata[p].get_states();
    last_msgs_vec.push_back(std::vector<VecSet<std::vector<Msg> > >(states.size(),
                                                                    VecSet<std::vector<Msg> >::singleton(std::vector<Msg>())));
    if(p == 0){
      /* Add dummy message */
      std::vector<Msg> v;
      v.push_back(Msg(Store(mem_size),0,VecSet<Lang::NML>()));
      last_msgs_vec[0][0].insert(v);
    }
    bool changed = true;
    while(changed){
      changed = false;
      for(unsigned i = 0; i < states.size(); ++i){
        for(auto trit = states[i].bwd_transitions.begin(); trit != states[i].bwd_transitions.end(); ++trit){
          int src = (*trit)->source;
          VecSet<std::vector<Msg> > old_set = last_msgs_vec[p][i];
          VecSet<std::vector<Msg> > new_set = last_msgs_vec[p][i];
          if((*trit)->instruction.get_writes().size()){
            VecSet<VecSet<Lang::MemLoc<int> > > wsets = (*trit)->instruction.get_write_sets();
            for(auto wsit = wsets.begin(); wsit != wsets.end(); ++wsit){
              VecSet<Lang::NML> nmls;
              for(auto wit = wsit->begin(); wit != wsit->end(); ++wit){
                nmls.insert(Lang::NML(*wit,p));
              }
              /* Modify all old channels */
              for(auto vit = last_msgs_vec[p][src].begin(); vit != last_msgs_vec[p][src].end(); ++vit){
                /* Add new message */
                std::vector<Msg> v = *vit;
                v.push_back(Msg(store_of_write(Machine::PTransition(**trit,p)),p,nmls));
                /* Remove messages that are no longer rightmost */
                VecSet<Lang::NML> covered = nmls;
                for(int j = int(v.size())-2; j >= 0; --j){
                  if(v[j].nmls.size() > 0 && v[j].nmls.subset_of(covered)){
                    /* remove v[j] */
                    for(int k = j; k < int(v.size())-1; ++k){
                      v[k] = v[k+1];
                    }
                    v.resize(int(v.size())-1,v[0]); // Note: v[0] is just to make gcc happy, it will not be copied
                  }else{
                    covered.insert(v[j].nmls);
                  }
                }
                new_set.insert(v);
              }
            }
          }else{
            new_set.insert(last_msgs_vec[p][src]);
          }
          /* Eliminate channels that are suffixes of other channels */
          last_msgs_vec[p][i].clear();
          for(auto it = new_set.begin(); it != new_set.end(); ++it){
            bool is_suffix = false;
            for(auto it2 = new_set.begin(); it2 != new_set.end(); ++it2){
              if(it != it2 && vector_is_suffix(*it,*it2)){
                is_suffix = true;
                break;
              }
            }
            if(!is_suffix){
              last_msgs_vec[p][i].insert(*it);
            }
          }
          if(last_msgs_vec[p][i] != old_set){
            changed = true;
          }
        }
      }
    }
  }
  /* Printing */
  Log::debug << "last_msgs_vec:\n";
  for(unsigned p = 0; p < last_msgs_vec.size(); ++p){
    Log::debug << "P" << p << ":\n";
    for(unsigned i = 0; i < last_msgs_vec[p].size(); ++i){
      Log::debug << "  Q" << i << ":\n";
      for(auto it = last_msgs_vec[p][i].begin(); it != last_msgs_vec[p][i].end(); ++it){
        Log::debug << "    [";
        for(unsigned j = 0; j < it->size(); ++j){
          if(j != 0) Log::debug << ", ";
          Log::debug << (*it)[j].to_short_string(*this);
        }
        Log::debug << "]\n";
      }
    }
  }
};

SbConstraint::Common::~Common(){
};

SbConstraint::Store SbConstraint::Common::store_of_write(const Machine::PTransition &t) const{
  Store store(mem_size);
  if(t.instruction.get_type() == Lang::WRITE && t.instruction.get_expr().is_integer()){
    store = store.assign(index(Lang::NML(t.instruction.get_memloc(),t.pid)),
                         t.instruction.get_expr().get_integer());
  }else if(t.instruction.get_type() == Lang::LOCKED &&
           t.instruction.get_statement_count() == 1 &&
           t.instruction.get_statement(0)->get_type() == Lang::WRITE &&
           t.instruction.get_statement(0)->get_expr().is_integer()){
    store = store.assign(index(Lang::NML(t.instruction.get_statement(0)->get_memloc(),t.pid)),
                         t.instruction.get_statement(0)->get_expr().get_integer());
  }
  /* Generalize */
  return store;
};

template<class T>
bool SbConstraint::Common::vector_is_suffix(const std::vector<T> &a, const std::vector<T> &b) const{
  if(a.size() > b.size()){
    return false;
  }
  for(unsigned i = 0; i < a.size(); ++i){
    if(a[int(a.size())-int(i)-1] != b[int(b.size())-int(i)-1]){
      return false;
    }
  }
  return true;
};

std::list<Constraint*> SbConstraint::Common::get_bad_states(){
  std::list<Constraint*> l;
  for(unsigned i = 0; i < machine.forbidden.size(); ++i){
    for(int j = 0; j < messages.size(); ++j){
      SbConstraint *sbc = new SbConstraint(machine.forbidden[i],messages[j],*this);
      if(sbc->ok_channel()){
        l.push_back(sbc);
      }else{
        delete sbc;
      }
    }
  }
  return l;
};

SbConstraint::SbConstraint(std::vector<int> pcs, const Common::MsgHdr &msg, Common &c)
  : common(c), pcs(pcs) {
  /* Initialize the channel with a single STAR filled message. */
  channel.push_back(Msg(Store(common.mem_size),msg.wpid,msg.nmls));
  cpointers = std::vector<int>(pcs.size(),0);
  /* Initialize registers with STARs */
  for(unsigned p = 0; p < pcs.size(); p++){
    reg_stores.push_back(Store(common.reg_count[p]));
  }
};

SbConstraint::~SbConstraint() throw(){
};

bool SbConstraint::is_init_state() const{
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

std::list<const Machine::PTransition*> SbConstraint::partred() const{
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
};

std::string SbConstraint::to_string() const throw(){
  std::stringstream ss;
  for(unsigned p = 0; p < pcs.size(); ++p){
    ss << "P" << p << " @Q" << pcs[p] << " {cpointer=" << cpointers[p];
    for(int r = 0; r < common.reg_count[p]; ++r){
      ss << ", " << common.machine.regs[p][r].name << "=";
      if(reg_stores[p][r] == value_t::STAR){
        ss << "*";
      }else{
        ss << reg_stores[p][r];
      }
    }
    ss << "}\n";
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

int SbConstraint::Msg::compare(const Msg &msg) const{
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

std::string SbConstraint::Msg::to_short_string(const SbConstraint::Common &common) const{
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

VecSet<int> SbConstraint::possible_values(const Store &reg_store, int pid, const Lang::Expr<int> &e) const{
  return reg_store.possible_values(e,common.machine.regs[pid]);
};

bool SbConstraint::possibly_holds(const Store &reg_store, int pid, const Lang::BExpr<int> &b) const{
  throw new std::logic_error("SbConstraint::possibly_holds: Not implemented");
};

int SbConstraint::index_of_read(Lang::NML nml, int pid) const{
  int i = channel.size()-1;
  while(i > cpointers[pid]){
    if(channel[i].wpid == pid && channel[i].nmls.count(nml)){
      return i;
    }
    i--;
  }
  return i;
};

std::vector<SbConstraint*> SbConstraint::channel_pop_back() const{
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
    std::vector<SbConstraint*> res;
    SbConstraint *sbc;
    bool last_unifiable;

    std::vector<Msg> ch0(channel);
    ch0.pop_back();
    ch0[ch0.size()-1].store = ch0[ch0.size()-1].store.unify(channel.back().store.assign(nmli,value_t::STAR),&last_unifiable);

    if(last_unifiable){
      /* The case when there is no hidden message */
      sbc = new SbConstraint(*this);
      sbc->channel = ch0;
      res.push_back(sbc);

      /* Guess a position to reinsert a hidden message. */
      for(int i = int(ch0.size())-1; i > 0; i--){
        std::vector<Msg> ch1(ch0);
        ch1.push_back(ch0.back());
        for(int j = int(ch1.size()) - 2; j > i; j--){
          ch1[j] = ch1[j-1];
        }
        ch1[i] = new_msg;
        sbc = new SbConstraint(*this);
        sbc->channel = ch1;
        /* Update cpointers */
        for(unsigned p = 0; p < sbc->cpointers.size(); ++p){
          if(sbc->cpointers[p] >= i){
            ++sbc->cpointers[p];
          }
        }
        res.push_back(sbc);
        if(i > 0 && ch0[i-1].wpid == wpid && ch0[i-1].nmls.count(nml)){
          break;
        }
      }
    }

    /* The case when the hidden message is the last of the new channel */
    {
      sbc = new SbConstraint(*this);
      sbc->channel.back().store = sbc->channel.back().store.assign(nmli,value_t::STAR);
      res.push_back(sbc);
    }

    return res;
  }else{
    throw new std::logic_error("SbConstraint::channel_pop_back: Support for multiple memory locations not implemented.");
  }
};

void SbConstraint::test(){
  std::cout << " * test_possible_values\n";
  test_possible_values();
  std::cout << " * test_pre\n";
  test_pre();
  std::cout << " * Common::test\n";
  Common::test();
  std::cout << " * test_comparison\n";
  test_comparison();
};

void SbConstraint::Common::test(){
  /* Test 1: Check messages */
  {
    std::stringstream rmm;
    rmm << "forbidden L0 L0\n"
        << "data\n"
        << "  x = 0 : [0:2]\n"
        << "  y = * : [0:1]\n"
        << "process\n"
        << "data\n"
        << "  z = 42 : [0:100]\n"
        << "text\n"
        << "L0:\n"
        << "  nop;\n"
        << "  write: x := 0;\n"
        << "  locked write: y := 0;\n"
        << "  locked{\n"
        << "    write: x := 1;\n"
        << "    write: z[my] := 13\n"
        << "  or\n"
        << "    write: z[my] := 10;\n"
        << "    write: y := 0\n"
        << "  }\n"
        << "process\n"
        << "text\n"
        << "  L0:\n"
        << "  locked{write: x := 0; read: y = 0; write: y := 1; write: x := 2}";
    Lexer lex(rmm);
    Machine machine(Parser::p_test(lex));
    Common common(machine);
    
    VecSet<MsgHdr> expected;
    VecSet<Lang::NML> x = VecSet<Lang::NML>::singleton(Lang::NML::global(0));
    VecSet<Lang::NML> y = VecSet<Lang::NML>::singleton(Lang::NML::global(1));
    VecSet<Lang::NML> xy = x; xy.insert(y);
    VecSet<Lang::NML> xz = x; xz.insert(Lang::NML::local(0,0));
    VecSet<Lang::NML> yz = y; yz.insert(Lang::NML::local(0,0));
    expected.insert(MsgHdr(0,x));
    expected.insert(MsgHdr(0,y));
    expected.insert(MsgHdr(0,xz));
    expected.insert(MsgHdr(0,yz));
    expected.insert(MsgHdr(1,xy));

    if(common.messages == expected){
      std::cout << "Test1: Success!\n";
    }else{
      std::cout << "Test1: Failure\n";
    }
  }
}

void SbConstraint::test_possible_values(){
  try{
    /* Build a dummy machine */
    std::stringstream dummy_rmm;
    dummy_rmm << "forbidden L0\n"
              << "data\n"
              << "  x = 0 : [0:2]\n"
              << "  y = * : [0:1]\n"
              << "process\n"
              << "data\n"
              << "  z = 42 : [0:100]\n"
              << "registers\n"
              << "  $r0 = * : [0:2]\n"
              << "  $r1 = * : [10:12]\n"
              << "  $r2 = * : [1:1]\n"
              << "text\n"
              << "L0:\n"
              << "  nop";
    Lexer lex(dummy_rmm);
    Machine dummy_machine(Parser::p_test(lex));

    /* Construct dummy SbConstraint */
    Common common(dummy_machine);
    SbConstraint sbc(std::vector<int>(1,0),common.messages[0],common);

    /* Start testing */

    /* Test 1: r0 + r1 + r1 + 42 : {62, 63, 64, 65, 66, 67, 68} */
    Lang::Expr<int> test1_e = Lang::Expr<int>::reg(0) + Lang::Expr<int>::reg(1) + Lang::Expr<int>::reg(1) + 
      Lang::Expr<int>::integer(42);
    std::vector<int> test1_v(7);
    test1_v[0] = 62; test1_v[1] = 63; test1_v[2] = 64;
    test1_v[3] = 65; test1_v[4] = 66; test1_v[5] = 67;
    test1_v[6] = 68;
    if(sbc.possible_values(sbc.reg_stores[0],0,test1_e) == test1_v){
      std::cout << "Test1: Success!\n";
    }else{
      std::cout << "Test1: Failure\n";
      VecSet<int> v = sbc.possible_values(sbc.reg_stores[0],0,test1_e);
      std::cout << "  Got: [";
      for(int i = 0; i < v.size(); i++){
        if(i != 0) std::cout << ", ";
        std::cout << v[i];
      }
      std::cout << "]\n";
    }

    /* Test 2: r0 + r2 : {1, 2, 3} */
    Lang::Expr<int> test2_e = Lang::Expr<int>::reg(0) + Lang::Expr<int>::reg(2);
    std::vector<int> test2_v(3);
    test2_v[0] = 1; test2_v[1] = 2; test2_v[2] = 3;
    if(sbc.possible_values(sbc.reg_stores[0],0,test2_e) == test2_v){
      std::cout << "Test2: Success!\n";
    }else{
      std::cout << "Test2: Failure\n";
    }

    /* Test 3: 42 : {42} */
    Lang::Expr<int> test3_e = Lang::Expr<int>::integer(42);
    std::vector<int> test3_v(1,42);
    if(sbc.possible_values(sbc.reg_stores[0],0,test3_e) == test3_v){
      std::cout << "Test3: Success!\n";
    }else{
      std::cout << "Test3: Failure\n";
    }

    /* Test 4: possible_reg_stores: r0 + r1 == 12 */
    Lang::Expr<int> test4_e = Lang::Expr<int>::reg(0) + Lang::Expr<int>::reg(1);
    std::vector<Store> test4_v;
    {
      std::vector<value_t> v;
      v.push_back(0);
      v.push_back(12);
      v.push_back(value_t::STAR);
      test4_v.push_back(Store(v));
      v[0] = 1;
      v[1] = 11;
      test4_v.push_back(Store(v));
      v[0] = 2;
      v[1] = 10;
      test4_v.push_back(Store(v));
    }
    if(sbc.possible_reg_stores(sbc.reg_stores[0],0,test4_e,12) == test4_v){
      std::cout << "Test4: Success!\n";
    }else{
      std::cout << "Test4:: Failure\n";
    }

    /* Test 5: possible_reg_stores: r0 + r1 == 15 */
    Lang::Expr<int> test5_e = Lang::Expr<int>::reg(0) + Lang::Expr<int>::reg(1);
    std::vector<Store> test5_v;
    if(sbc.possible_reg_stores(sbc.reg_stores[0],0,test5_e,15) == test5_v){
      std::cout << "Test5: Success!\n";
    }else{
      std::cout << "Test5:: Failure\n";
    }
  }catch(std::exception *exc){
    std::cout << "Error: " << exc->what() << "\n";
    throw;
  }
};

void SbConstraint::test_pre(){
  /* Build a dummy machine */
  std::stringstream dummy_rmm;
  dummy_rmm << "forbidden L0 L0\n"
            << "data\n"
            << "  x = * : [8:12]\n"
            << "  y = * : [0:1]\n"
            << "process\n"
            << "data\n"
            << "  z = 42 : [0:100]\n"
            << "registers\n"
            << "  $r0 = * : [0:2]\n"
            << "  $r1 = * : [10:12]\n"
            << "  $r2 = * : [1:1]\n"
            << "text\n"
            << "L0:\n"
            << "  nop\n"
            << "process\n"
            << "text\n"
            << "L0:\n"
            << "  nop\n";
  Lexer lex(dummy_rmm);
  Machine dummy_machine(Parser::p_test(lex));

  /* Construct dummy SbConstraint */
  Common common(dummy_machine);

  /* Test NOP */
  {
    std::cout << " ** NOP **\n";
    std::vector<int> pcs;
    pcs.push_back(1); pcs.push_back(3);
    SbConstraint sbc(pcs,common.messages[0],common);
    Machine::PTransition t(0,Lang::Stmt<int>::nop(),1,0);
    std::list<Constraint*> res = sbc.pre(t);
    std::cout << res.front()->to_string() << "\n\n";
  }

  /* Test READASSERT */
  {
    std::cout << " ** READASSERT x = $r0 + $r1 **\n";
    std::vector<int> pcs;
    pcs.push_back(1); pcs.push_back(3);
    SbConstraint sbc(pcs,common.messages[0],common);
    sbc.reg_stores[0] = sbc.reg_stores[0].assign(0,1);
    std::cout << "Initial:\n" << sbc.to_string() << "\n\n";
    Machine::PTransition t(0,Lang::Stmt<int>::read_assert(Lang::MemLoc<int>::global(0),
                                                          Lang::Expr<int>::reg(0) + Lang::Expr<int>::reg(1)),1,0);
    std::list<Constraint*> res = sbc.pre(t);
    std::cout << "Pre:\n";
    for(auto it = res.begin(); it != res.end(); ++it){
      std::cout << (*it)->to_string() << "\n";
    }
  }

  /* Test WRITE (Test3) */
  {
    std::cout << " ** WRITE with too short buffer **\n";
    std::vector<int> pcs;
    pcs.push_back(1); pcs.push_back(3);
    SbConstraint sbc(pcs,common.messages[0],common);
    Machine::PTransition t(0,Lang::Stmt<int>::write(Lang::MemLoc<int>::global(0),
                                                    Lang::Expr<int>::reg(0) + Lang::Expr<int>::reg(1)),1,0);
    if(sbc.pre(t).empty()){
      std::cout << "Test3: Success!\n";
    }else{
      std::cout << "Test3: Failure\n";
    }
  }

  /* Test WRITE (Test4) */
  {
    std::cout << " ** WRITE x := r0 + r1 [r0=0,r1=10,x=*] **\n";
    std::vector<int> pcs;
    pcs.push_back(1); pcs.push_back(3);
    SbConstraint sbc(pcs,common.messages[0],common);
    Machine::PTransition t(0,Lang::Stmt<int>::write(Lang::MemLoc<int>::global(0),
                                                    Lang::Expr<int>::reg(0) + Lang::Expr<int>::reg(1)),1,0);
    Msg msg(Store(3),0,VecSet<Lang::NML>::singleton(Lang::NML::global(0)));
    sbc.channel.push_back(msg);
    sbc.reg_stores[0] = sbc.reg_stores[0].assign(0,0).assign(1,10);

    std::cout << "Initial:\n" << sbc.to_string() << "\n";
    std::list<Constraint*> res = sbc.pre(t);
    std::cout << "Pre:\n";
    for(auto it = res.begin(); it != res.end(); ++it){
      std::cout << (*it)->to_string() << "\n";
    }
  }

  /* Test LOCKED WRITE */
  {
    std::cout << " ** LOCKED WRITE x := r0 + r1 [r0=0,r1=10,x=*] **\n";
    std::vector<int> pcs;
    pcs.push_back(1); pcs.push_back(3);
    SbConstraint sbc(pcs,common.messages[0],common);
    Machine::PTransition t(0,Lang::Stmt<int>::locked_write(Lang::MemLoc<int>::global(0),
                                                           Lang::Expr<int>::reg(0) + Lang::Expr<int>::reg(1)),1,0);
    Msg msg(Store(3),0,VecSet<Lang::NML>::singleton(Lang::NML::global(0)));
    sbc.channel.push_back(msg);
    sbc.reg_stores[0] = sbc.reg_stores[0].assign(0,0).assign(1,10);
    sbc.cpointers[0] = 1;

    std::cout << "Initial:\n" << sbc.to_string() << "\n";
    std::list<Constraint*> res = sbc.pre(t);
    std::cout << "Pre:\n";
    for(auto it = res.begin(); it != res.end(); ++it){
      std::cout << (*it)->to_string() << "\n";
    }
  }

  /* Test fresh UPDATE */
  {
    std::cout << " ** P0: update(P1,x) **\n";
    std::vector<int> pcs;
    pcs.push_back(1); pcs.push_back(3);
    SbConstraint sbc(pcs,common.messages[0],common);
    Machine::PTransition t(0,Lang::Stmt<int>::update(1,VecSet<Lang::MemLoc<int> >::singleton(Lang::MemLoc<int>::global(0))),1,0);
    sbc.channel[0].wpid = 1;
    sbc.channel[0].nmls = VecSet<Lang::NML>::singleton(Lang::NML::global(0));
    std::cout << "Initial:\n" << sbc.to_string() << "\n";
    std::list<Constraint*> res = sbc.pre(t);
    std::cout << "Pre:\n";
    for(auto it = res.begin(); it != res.end(); ++it){
      std::cout << (*it)->to_string() << "\n";
    }
  }

  /* Test non-fresh UPDATE */
  {
    std::cout << " ** P0: update(P1,x) **\n";
    std::vector<int> pcs;
    pcs.push_back(1); pcs.push_back(3);
    SbConstraint sbc(pcs,common.messages[0],common);
    Machine::PTransition t(0,Lang::Stmt<int>::update(1,VecSet<Lang::MemLoc<int> >::singleton(Lang::MemLoc<int>::global(0))),1,0);
    Msg msg(Store(3),1,VecSet<Lang::NML>::singleton(Lang::NML::global(0)));
    sbc.channel.push_back(msg);
    sbc.cpointers[0] = 1;
    std::cout << "Initial:\n" << sbc.to_string() << "\n";
    std::list<Constraint*> res = sbc.pre(t);
    std::cout << "Pre:\n";
    for(auto it = res.begin(); it != res.end(); ++it){
      std::cout << (*it)->to_string() << "\n";
    }
  }

  /* Test blocked non-fresh UPDATE */
  {
    std::cout << " ** P0: update(P1,y) **\n";
    std::vector<int> pcs;
    pcs.push_back(1); pcs.push_back(3);
    SbConstraint sbc(pcs,common.messages[0],common);
    Machine::PTransition t(0,Lang::Stmt<int>::update(1,VecSet<Lang::MemLoc<int> >::singleton(Lang::MemLoc<int>::global(1))),1,0);
    Msg msg(Store(3),1,VecSet<Lang::NML>::singleton(Lang::NML::global(0)));
    sbc.channel.push_back(msg);
    sbc.cpointers[0] = 1;
    std::cout << "Initial:\n" << sbc.to_string() << "\n";
    std::list<Constraint*> res = sbc.pre(t);
    if(res.empty()){
      std::cout << "Test (non-fresh update): Success!\n";
    }else{
      std::cout << "Test (non-fresh update): Failure\n";
    }
  }
  
};

std::list<SbConstraint::pre_constr_t> SbConstraint::pre(const Machine::PTransition &t, bool locked) const{
  std::list<pre_constr_t> res;
  const Lang::Stmt<int> &s = t.instruction;
  if(pcs[t.pid] != t.target){
    return res;
  }

  switch(s.get_type()){
  case Lang::NOP:
    {
      SbConstraint *sbc = new SbConstraint(*this);
      sbc->pcs[t.pid] = t.source;
      res.push_back(sbc);
      break;
    }
  case Lang::ASSIGNMENT:
    {
      VecSet<int> val_r = possible_values(reg_stores[t.pid],t.pid,Lang::Expr<int>::reg(s.get_reg()));
      for(int i = 0; i < val_r.size(); ++i){
        VecSet<Store> rss = possible_reg_stores(reg_stores[t.pid].assign(s.get_reg(),value_t::STAR),
                                                t.pid,
                                                s.get_expr(),
                                                val_r[i]);
        for(int j = 0; j < rss.size(); ++j){
          SbConstraint *sbc = new SbConstraint(*this);
          sbc->reg_stores[t.pid] = rss[j];
          res.push_back(sbc);
        }
      }
      break;
    }
  case Lang::READASSERT:
    {
      Lang::NML nml(s.get_memloc(),t.pid);
      int nmli = common.index(nml);
      int msgi = index_of_read(nml,t.pid);
      VecSet<int> val_nml = possible_values(channel[msgi].store,nml);
      for(int i = 0; i < val_nml.size(); i++){
        /* For each possible value for nml, try to pair it with an
         * assignment to the reg_store such that s.expr() evaluates to
         * the same value. */
        VecSet<Store> stores = possible_reg_stores(reg_stores[t.pid],t.pid,s.get_expr(),val_nml[i]);
        for(int j = 0; j < stores.size(); ++j){
          SbConstraint *sbc = new SbConstraint(*this);
          sbc->pcs[t.pid] = t.source;
          sbc->channel[msgi].store = channel[msgi].store.assign(nmli,val_nml[i]);
          sbc->reg_stores[t.pid] = stores[j];
          res.push_back(sbc);
        }
      }
      break;
    }
  case Lang::READASSIGN:
    {
      if(reg_stores[t.pid][s.get_reg()] == value_t::STAR){
        SbConstraint *sbc = new SbConstraint(*this);
        sbc->pcs[t.pid] = t.source;
        res.push_back(sbc);
      }else{
        Lang::NML nml(s.get_memloc(),t.pid);
        int nmli = common.index(nml);
        int msgi = index_of_read(nml,t.pid);
        VecSet<int> val_nml = possible_values(channel[msgi].store,nml);
        if(val_nml.count(reg_stores[t.pid][s.get_reg()].get_int()) > 0){
          SbConstraint *sbc = new SbConstraint(*this);
          sbc->pcs[t.pid] = t.source;
          if(sbc->channel[msgi].store[nmli] == value_t::STAR){
            sbc->channel[msgi].store = sbc->channel[msgi].store.assign(nmli,reg_stores[t.pid][s.get_reg()]);
          }
          res.push_back(sbc);
        }
      }
      break;
    }
  case Lang::WRITE:
    {
      /* Check that cpointers point to the right messages in the channel */
      bool ok_cpointers = true;
      if(locked){
        ok_cpointers = cpointers[t.pid] == int(channel.size())-1;
        for(int p = 0; p < int(cpointers.size()); ++p){
          if(p != t.pid && cpointers[p] == int(channel.size())-1)
            ok_cpointers = false;
        }
      }else{
        for(int p = 0; p < int(cpointers.size()); ++p){
          if(cpointers[p] == int(channel.size())-1)
            ok_cpointers = false;
        }
      }

      if(ok_cpointers){
        Lang::NML nml = Lang::NML(s.get_memloc(),t.pid);
        int nmli = common.index(nml);
        /* Check that the rightmost message in the channel matches the
         * write, and produce matching reg_stores */
        bool ok_nmls = true;
        if(locked){
          ok_nmls = channel.back().nmls.count(nml);
        }else{
          ok_nmls = (channel.back().nmls.size() == 1 && channel.back().nmls.count(nml));
        }
        if(channel.back().wpid == t.pid && ok_nmls){
          VecSet<int> val_nml = possible_values(channel.back().store,nml);
          for(int vali = 0; vali < val_nml.size(); ++vali){
            VecSet<Store> rstores = possible_reg_stores(reg_stores[t.pid],t.pid,s.get_expr(),val_nml[vali]);
            for(int regi = 0; regi < rstores.size(); ++regi){
              SbConstraint *sbc = new SbConstraint(*this);
              sbc->pcs[t.pid] = t.source;
              sbc->channel.back().store = sbc->channel.back().store.assign(nmli,val_nml[vali]);
              sbc->reg_stores[t.pid] = rstores[regi];
              res.push_back(pre_constr_t(sbc,true,VecSet<Lang::NML>::singleton(nml)));
            }
          }
        }
      }
      break;
    }
  case Lang::SEQUENCE:
    {
      assert(locked);
      std::vector<pre_constr_t> v;
      v.push_back(pre_constr_t(new SbConstraint(*this)));
      for(int i = s.get_statement_count()-1; i >= 0; --i){
        std::vector<pre_constr_t> w;
        Machine::PTransition t2(t.target,*s.get_statement(i),t.target,t.pid);
        for(unsigned j = 0; j < v.size(); ++j){
          std::list<pre_constr_t> l = v[j].sbc->pre(t2,locked);
          for(auto it = l.begin(); it != l.end(); ++it){
            it->pop_back = it->pop_back || v[j].pop_back;
            it->written_nmls.insert(v[j].written_nmls);
            w.push_back(*it);
          }
          delete v[j].sbc;
        }
        v = w;
      }
      for(unsigned i = 0; i < v.size(); ++i){
        v[i].sbc->pcs[t.pid] = t.source;
        res.push_back(v[i]);
      }
      break;
    }
  case Lang::LOCKED:
    {
      /* Check if the locked block contains writes.
       * If so, it is fencing. */
      if(s.get_writes().size() == 0 || cpointers[t.pid] == int(channel.size())-1){
        for(int i = 0; i < s.get_statement_count(); ++i){
          Machine::PTransition ti(t.source,*s.get_statement(i),t.target,t.pid);
          std::list<pre_constr_t> v = pre(ti,true);
          res.insert(res.end(),v.begin(),v.end());
        }
      }
      break;
    }
  case Lang::UPDATE:
    {
      assert(!locked);
      VecSet<Lang::NML> snmls;
      for(unsigned i = 0; i < s.get_writes().size(); ++i){
        snmls.insert(Lang::NML(s.get_writes()[i],t.pid));
      }
      /* Check that the message matches the update */
      if(channel[cpointers[t.pid]].wpid == s.get_writer() &&
         channel[cpointers[t.pid]].nmls == snmls){
        if(cpointers[t.pid] == 0){
          /* Need to insert a fresh message */
          for(auto msgit = common.messages.begin(); msgit != common.messages.end(); ++msgit){
            SbConstraint *sbc = new SbConstraint(*this);
            Msg msg(Store(common.mem_size),msgit->wpid,msgit->nmls);
            sbc->channel.insert(sbc->channel.begin(),msg);
            for(unsigned p = 0; p < sbc->cpointers.size(); ++p){
              if(int(p) != t.pid){
                ++sbc->cpointers[p];
              }
            }
            res.push_back(sbc);
          }
        }else{
          /* Update with a message in the channel */
          SbConstraint *sbc = new SbConstraint(*this);
          --sbc->cpointers[t.pid];
          res.push_back(sbc);
        }
      }
      break;
    }
  case Lang::ASSUME:
    {
      VecSet<Store> rstores = possible_reg_stores(reg_stores[t.pid],t.pid,s.get_condition());
      for(int i = 0; i < rstores.size(); ++i){
        SbConstraint *sbc = new SbConstraint(*this);
        sbc->pcs[t.pid] = t.source;
        sbc->reg_stores[t.pid] = rstores[i];
        res.push_back(sbc);
      }
      break;
    }
  default:
    throw new std::logic_error("SbConstraint::pre: Unsupported transition: "+t.to_string(common.machine));
  }

  return res;
};

std::list<Constraint*> SbConstraint::pre(const Machine::PTransition &t) const{
  std::list<Constraint*> res;
  std::list<SbConstraint::pre_constr_t> r = pre(t,false);
  for(auto it = r.begin(); it != r.end(); ++it){
    if(it->pop_back){
      /* Check that all NMLs that are associated with the message really were written */
      if(it->sbc->channel.back().nmls == it->written_nmls){
        if(it->sbc->channel.size() == 1){
          /* Special case: There is only one process, and it is performing an locked write. */
          assert(it->sbc->cpointers.size() == 1);
          assert(t.instruction.get_type() == Lang::LOCKED);
          throw new std::logic_error("SbConstraint::pre: Single process locked write: Not implemented.");
        }else{
          assert((it->sbc->cpointers[t.pid] == int(it->sbc->channel.size())-1)
                 ==
                 (t.instruction.get_type() == Lang::LOCKED));
          bool move_p_to_last = false;
          if(it->sbc->cpointers[t.pid] == int(it->sbc->channel.size())-1){
            --it->sbc->cpointers[t.pid];
            /* Make sure after hidden messages have been added that
             * process t.pid points to the rightmost message */
            move_p_to_last = true;
          }
          std::vector<SbConstraint*> sbcs = it->sbc->channel_pop_back();
          for(unsigned i = 0; i < sbcs.size(); ++i){
            if(move_p_to_last){
              sbcs[i]->cpointers[t.pid] = int(sbcs[i]->channel.size())-1;
            }
            if(sbcs[i]->ok_channel()){
              res.push_back(sbcs[i]);
            }else{
              delete sbcs[i];
            }
          }
          delete it->sbc;
        }
      }else{
        /* The writes of this execution do not match the NMLs specified by the message */
        delete it->sbc;
      }
    }else{
      if(it->sbc->ok_channel()){
        res.push_back(it->sbc);
      }else{
        delete it->sbc;
      }
    }
  }
  return res;
};

void SbConstraint::test_comparison(){
  /* Build a dummy machine */
  std::stringstream dummy_rmm;
  dummy_rmm << "forbidden L0 L0\n"
            << "data\n"
            << "  u = * : [0:1]\n"
            << "  v = * : [0:1]\n"
            << "  w = * : [0:1]\n"
            << "  x = * : [0:1]\n"
            << "  y = * : [0:1]\n"
            << "  z = * : [0:1]\n"
            << "process\n"
            << "text\n"
            << "L0:\n"
            << "  nop\n"
            << "process\n"
            << "text\n"
            << "L0:\n"
            << "  nop\n";
  Lang::NML u = Lang::NML::global(0);
  Lang::NML v = Lang::NML::global(1);
  Lang::NML w = Lang::NML::global(2);
  Lang::NML x = Lang::NML::global(3);
  Lang::NML y = Lang::NML::global(4);
  Lang::NML z = Lang::NML::global(5);
  VecSet<Lang::NML> us = VecSet<Lang::NML>::singleton(u);
  VecSet<Lang::NML> vs = VecSet<Lang::NML>::singleton(v);
  VecSet<Lang::NML> ws = VecSet<Lang::NML>::singleton(w);
  VecSet<Lang::NML> xs = VecSet<Lang::NML>::singleton(x);
  VecSet<Lang::NML> ys = VecSet<Lang::NML>::singleton(y);
  VecSet<Lang::NML> zs = VecSet<Lang::NML>::singleton(z);
  Lexer lex(dummy_rmm);

  try{
    Machine dummy_machine(Parser::p_test(lex));

    /* Construct dummy SbConstraint */
    Common common(dummy_machine);

    std::function<bool(std::string,bool)> test = 
      [](std::string name, bool result)->bool{
      if(result){
        std::cout << name << ": Success!\n";
      }else{
        std::cout << name << ": Failure\n";
      }
      return result;
    };

    std::function<std::string(const SbConstraint &)> char_to_string =
      [](const SbConstraint &sbc)->std::string{
      std::vector<MsgCharacterization> chr = sbc.characterize_channel();
      std::stringstream ss;
      ss << "[";
      for(unsigned i = 0; i < chr.size(); ++i){
        if(i != 0) ss << ", ";
        ss << "<P" << chr[i].wpid << ", ";
        if(chr[i].nmls.size() == 1){
          ss << chr[i].nmls[0].to_string();
        }else{
          ss << "{";
          for(int j = 0; j < chr[i].nmls.size(); ++j){
            if(j != 0) ss << ", ";
            ss << chr[i].nmls[j].to_string();
          }
          ss << "}";
        }
        for(int j = 0; j < chr[i].cpointers.size(); ++j){
          ss << ", ptr" << chr[i].cpointers[j];
        }
        ss << ">";
      }
      ss << "]";
      return ss.str();
    };

    /* Test1: */
    {
      std::vector<int> pcs(2,0);
      SbConstraint sbc0(pcs,common.messages[0],common);
      SbConstraint sbc1(pcs,common.messages[0],common);
      Msg msg(Store(6),0,us);
      sbc0.channel.clear(); sbc1.channel.clear();
      sbc0.channel.push_back(msg);
      sbc0.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc0.cpointers[0] = 0;
      sbc0.cpointers[1] = 1;
      sbc1.cpointers[0] = 0;
      sbc1.cpointers[1] = 2;
      test("Test1a",sbc0.entailment_compare(sbc1) == Constraint::LESS);
      test("Test1b",sbc1.entailment_compare(sbc0) == Constraint::GREATER);
      test("Test1c",sbc0.characterize_channel() == sbc1.characterize_channel());
    }
    /* Test2:  */
    {
      std::vector<int> pcs(2,0);
      SbConstraint sbc0(pcs,common.messages[0],common);
      SbConstraint sbc1(pcs,common.messages[0],common);
      Msg msg(Store(6),0,us);
      sbc0.channel.clear(); sbc1.channel.clear();
      sbc0.channel.push_back(msg);
      sbc0.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc1.channel[1].wpid = 1;
      sbc0.cpointers[0] = 0;
      sbc0.cpointers[1] = 1;
      sbc1.cpointers[0] = 0;
      sbc1.cpointers[1] = 2;
      test("Test2a",sbc0.entailment_compare(sbc1) == Constraint::INCOMPARABLE);
      test("Test2b",sbc1.entailment_compare(sbc0) == Constraint::INCOMPARABLE);
      test("Test2c",sbc0.characterize_channel() != sbc1.characterize_channel());
    }
    /* Test3: */
    {
      std::vector<int> pcs(2,0);
      SbConstraint sbc0(pcs,common.messages[0],common);
      SbConstraint sbc1(pcs,common.messages[0],common);
      Msg msg(Store(6),0,us);
      sbc0.channel.clear(); sbc1.channel.clear();
      sbc0.channel.push_back(msg);
      sbc0.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc0.cpointers[0] = 0;
      sbc0.cpointers[1] = 1;
      sbc1.cpointers[0] = 0;
      sbc1.cpointers[1] = 1;
      test("Test3a",sbc0.entailment_compare(sbc1) == Constraint::INCOMPARABLE);
      test("Test3b",sbc1.entailment_compare(sbc0) == Constraint::INCOMPARABLE);
      test("Test3c",sbc0.characterize_channel() != sbc1.characterize_channel());
    }
    /* Test4: */
    {
      std::vector<int> pcs(2,0);
      SbConstraint sbc0(pcs,common.messages[0],common);
      SbConstraint sbc1(pcs,common.messages[0],common);
      Msg msg(Store(6),0,us);
      sbc0.channel.clear(); sbc1.channel.clear();
      sbc0.channel.push_back(msg);
      sbc0.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc0.channel[0].store = sbc0.channel[0].store.assign(0,0);
      sbc1.channel[0].store = sbc1.channel[0].store.assign(0,0).assign(1,1);
      sbc0.cpointers[0] = 0;
      sbc0.cpointers[1] = 1;
      sbc1.cpointers[0] = 0;
      sbc1.cpointers[1] = 2;
      test("Test4a",sbc0.entailment_compare(sbc1) == Constraint::LESS);
      test("Test4b",sbc1.entailment_compare(sbc0) == Constraint::GREATER);
      test("Test4c",sbc0.characterize_channel() == sbc1.characterize_channel());
    }
    /* Test5: */
    {
      std::vector<int> pcs(2,0);
      SbConstraint sbc0(pcs,common.messages[0],common);
      SbConstraint sbc1(pcs,common.messages[0],common);
      Msg msg(Store(6),0,us);
      sbc0.channel.clear(); sbc1.channel.clear();
      sbc0.channel.push_back(msg);
      sbc0.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc1.channel.push_back(msg);
      sbc0.channel[0].store = sbc0.channel[0].store.assign(0,0).assign(1,1);
      sbc1.channel[0].store = sbc1.channel[0].store.assign(0,0);
      sbc0.cpointers[0] = 0;
      sbc0.cpointers[1] = 1;
      sbc1.cpointers[0] = 0;
      sbc1.cpointers[1] = 2;
      test("Test5a",sbc0.entailment_compare(sbc1) == Constraint::INCOMPARABLE);
      test("Test5b",sbc1.entailment_compare(sbc0) == Constraint::INCOMPARABLE);
      test("Test5c",sbc0.characterize_channel() == sbc1.characterize_channel());
      test("Test5d",sbc0.entailment_compare(sbc0) == Constraint::EQUAL);
      test("Test5e",sbc1.entailment_compare(sbc1) == Constraint::EQUAL);
    }
  }catch(std::exception *exc){
    std::cout << "Error: " << exc->what() << "\n";
    throw;
  }
};

Constraint::Comparison SbConstraint::entailment_compare(const Constraint &c) const{
  assert(dynamic_cast<const SbConstraint*>(&c));
  return entailment_compare(static_cast<const SbConstraint&>(c));
};

Constraint::Comparison SbConstraint::entailment_compare(const SbConstraint &sbc) const{
  if(pcs != sbc.pcs){
    return Constraint::INCOMPARABLE;
  }

  Constraint::Comparison cmp = Constraint::EQUAL;

  for(unsigned p = 0; p < reg_stores.size(); ++p){
    cmp = Constraint::comb_comp(cmp,reg_stores[p].entailment_compare(sbc.reg_stores[p]));
    if(cmp == Constraint::INCOMPARABLE) return cmp;
  }

  return entailment_compare_channels(sbc,cmp);
};

Constraint::Comparison SbConstraint::entailment_compare_channels(const SbConstraint &sbc, Constraint::Comparison cmp) const{
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

std::vector<SbConstraint::MsgCharacterization> SbConstraint::characterize_channel() const{
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

bool SbConstraint::propagate_value_in_channel(const Lang::NML &nml, int nmli){
  if(nmli < 0){
    nmli = common.index(nml);
  }
  return propagate_value_in_channel(&channel,nml,nmli);
}

bool SbConstraint::propagate_value_in_channel(std::vector<Msg> *ch, const Lang::NML &nml, int nmli){
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

std::vector<SbConstraint::Msg> 
SbConstraint::unify_last_msgs_vec(const std::vector<Msg> &channel, 
                                  const std::vector<Msg> &lmv, int pid,
                                  const Common &common,
                                  bool *unifiable){
  std::vector<Msg> ch = channel;
  VecSet<Lang::NML> covered;
  int j = int(lmv.size())-1;
  for(int i = int(ch.size())-1; i >= 0; --i){
    if(ch[i].wpid == pid && (ch[i].nmls.empty() || !ch[i].nmls.subset_of(covered))){
      // ch[i] is a last message for pid
      // or the dummy message
      if(j < 0){
        // There is no matching message in lmv
        *unifiable = false;
        return std::vector<Msg>();
      }
      if(lmv[j].wpid != pid || ch[i].nmls != lmv[j].nmls){
        *unifiable = false;
        return std::vector<Msg>();
      }
      ch[i].store = ch[i].store.unify(lmv[j].store,unifiable);
      if(!*unifiable){
        return std::vector<Msg>();
      }
      --j;
      covered.insert(ch[i].nmls);
    }
  }
  
  /* Propagate values */
  if(use_channel_suffix_equality){
    for(auto it = covered.begin(); it != covered.end(); ++it){
      bool ok = propagate_value_in_channel(&ch,*it,common.index(*it));
      if(!ok){
        *unifiable = false;
        return std::vector<Msg>();
      }
    }
  }

  *unifiable = true;
  return ch;
};

bool SbConstraint::ok_channel(){
  if(use_channel_suffix_equality){
    /* For each memory location, check that in the suffix of channel
     * to the right of the rightmost message updating that memory
     * location, all messages can agree on a single value for that
     * memory location.
     */
    for(auto nmlit = common.nmls.begin(); nmlit != common.nmls.end(); ++nmlit){
      bool b = propagate_value_in_channel(*nmlit);
      if(!b){
        return false;
      }
    }
  }
  if(use_last_msgs_vec){
    for(unsigned p = 0; p < pcs.size(); ++p){
      std::vector<Msg> new_channel;
      int matching = 0;
      for(auto lmvit = common.last_msgs_vec[p][pcs[p]].begin();
          matching < 2 && lmvit != common.last_msgs_vec[p][pcs[p]].end(); ++lmvit){
        bool unifiable;
        std::vector<Msg> ch = unify_last_msgs_vec(channel,*lmvit,p,common,&unifiable);
        if(unifiable){
          ++matching;
          new_channel = ch;
        }
      }
      if(matching == 0){
        return false;
      }else if(matching == 1){
        channel = new_channel;
      }else{
        // Do nothing
      }
    }
  }else if(use_last_msg){
    std::vector<bool> ps(pcs.size(),false);
    for(int i = channel.size()-1; i >= 0; --i){
      int p = channel[i].wpid;
      if(!ps[p]){ // Rightmost message for process p
        /* Wrong memory locations in message? */
        bool match = false;
        Store orig_store = channel[i].store;
        for(auto msgit = common.last_msgs[p][pcs[p]].begin();
            msgit != common.last_msgs[p][pcs[p]].end(); ++msgit){
          if(msgit->nmls == channel[i].nmls){
            bool unifiable;
            Store new_store = orig_store.unify(msgit->store,&unifiable);
            if(unifiable){
              if(match){
                /* Ambiguous match: there are more than one message in
                 * last_msgs that match this message. */
                /* We cannot pick any one of them */
                channel[i].store = orig_store;
              }else{
                channel[i].store = new_store;
                match = true;
              }
            }
          }
        }
        if(!match){
          return false;
        }
        /* Propagate newly constrained values */
        if(use_channel_suffix_equality){
          for(auto nmlit = channel[i].nmls.begin(); nmlit != channel[i].nmls.end(); ++nmlit){
            int nmli = common.index(*nmlit);
            if(channel[i].store[nmli] != orig_store[nmli]){
              propagate_value_in_channel(*nmlit,nmli);
            }
          }
        }
      }
      ps[channel[i].wpid] = true;
    }
  }
  if(use_can_have_pending){
    std::vector<bool> ps(pcs.size(),false);
    for(int i = channel.size()-1; i >= 0; --i){
      int p = channel[i].wpid;
      if(!ps[p]){ // Rightmost message for process p
        /* Illegally pending? */
        if(!common.can_have_pending[p][pcs[p]] && cpointers[p] < i){
          return false;
        }
      }
      ps[channel[i].wpid] = true;
    }
  }
  return true;
};
