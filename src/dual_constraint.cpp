/*
 * Copyright (C) 2018 Tuan Phong Ngo
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

#include "dual_constraint.h"



/*****************/
/* Configuration */
/*****************/
const bool DualConstraint::use_limit_other_delete_propagate = false;
const bool DualConstraint::use_propagate_only_after_write = false;
const bool DualConstraint::use_allow_all_delete = false;
const bool DualConstraint::use_allow_all_propagate = false;

/*****************/

DualConstraint::Common::Common(const Machine &m)
  : DualChannelConstraint::Common(m)
{
  /* Check that all variables and registers have finite domains */
  for(unsigned i = 0; i < m.gvars.size(); ++i){
    if(m.gvars[i].domain.is_int()){
      throw new std::logic_error("DualConstraint::Common: DualConstraint requires finite domains. Infinite domain for global variable "+
                                 m.gvars[i].name);
    }
  }
  for(unsigned p = 0; p < m.lvars.size(); ++p){
    for(unsigned i = 0; i < m.lvars[p].size(); ++i){
      if(m.lvars[p][i].domain.is_int()){
        std::stringstream ss;
        ss << "DualConstraint::Common: DualConstraint requires finite domains. Infinite domain for local variable "
           << m.lvars[p][i].name << "[P" << p << "]";
        throw new std::logic_error(ss.str());
      }
    }
    for(unsigned i = 0; i < m.regs[p].size(); ++i){
      if(m.regs[p][i].domain.is_int()){
        std::stringstream ss;
        ss << "DualConstraint::Common: DualConstraint requires finite domains. Infinite domain for register "
           << "P" << p << ":" << m.regs[p][i].name;
        throw new std::logic_error(ss.str());
      }
    }
  }

  /* Setup all_transitions */
  for(unsigned p = 0; p < machine.automata.size(); ++p){
    const std::vector<Automaton::State> &states = machine.automata[p].get_states();
    for(unsigned i = 0; i < states.size(); ++i){
      bool has_reads = false;
      bool has_writes = false;
      
      /* Actual machine transitions */
      for(auto it = states[i].bwd_transitions.begin(); it != states[i].bwd_transitions.end(); ++it){
        all_transitions.push_back(Machine::PTransition(**it,p));
        if((*it)->instruction.get_writes().size() > 0 && !(*it)->instruction.is_fence()){
          /* For non-locked writes, also add the option of locked execution */
          Lang::Stmt<int> aw = Lang::Stmt<int>::locked_block(std::vector<Lang::Stmt<int> >(1,(*it)->instruction),
                                                             (*it)->instruction.get_pos());
          all_transitions.push_back(Machine::PTransition((*it)->source, aw, (*it)->target,p));
        }
        if((*it)->instruction.get_reads().size() > 0 && !(*it)->instruction.is_fence()){
          has_reads=true;
        }
      }
      
      for(auto it = states[i].fwd_transitions.begin(); it != states[i].fwd_transitions.end(); ++it){
        if((*it)->instruction.get_type() == Lang::WRITE){
          has_writes = true;
        }
      }
      
      /* Add deletee */
      for(auto it = messages.begin(); it != messages.end(); ++it){
        if(it->nmls.size() > 0){ /* Catch all messages except the dummy message */
          if (removed_lock_blocks_messages.count(*it)) { // do not add messages inside a lock block
          if (use_allow_all_delete || 
              /* The second case: process p deletes its own messages.
               * Other deletes will be performed when we calculate the pre sets 
               * If no read in the code of p, all writes should be locked writes
               */
              (it->wpid == int(p) && has_reads)) { 
              VecSet<Lang::MemLoc<int> > mls;
              for(auto nmlit = it->nmls.begin(); nmlit != it->nmls.end(); ++nmlit){
                mls.insert(nmlit->localize(p));
              }
              all_transitions.push_back(Machine::PTransition(i,Lang::Stmt<int>::deletee(it->wpid,mls),i,p));
            }
          }
        }
      }
      
      // add propagate
      for(auto it = messages.begin(); it != messages.end(); ++it){
        if (it->nmls.size() > 0 && !use_propagate_only_after_write) {
          if (use_allow_all_propagate || (it->wpid != int(p) && has_writes)) {
            VecSet<Lang::MemLoc<int> > mls;
            for(auto nmlit = it->nmls.begin(); nmlit != it->nmls.end(); ++nmlit){
              mls.insert(nmlit->localize(p));
            }
            all_transitions.push_back(Machine::PTransition(i,Lang::Stmt<int>::propagate(it->wpid,mls),i,p));
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
};

template<class T>
bool DualConstraint::Common::vector_is_suffix(const std::vector<T> &a, const std::vector<T> &b) const{
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

std::list<Constraint*> DualConstraint::Common::get_bad_states(){
  std::list<Constraint*> l;
  for(unsigned fbi = 0; fbi < machine.forbidden.size(); ++fbi){
    DualConstraint *sbc = new DualConstraint(machine.forbidden[fbi], *this);
    l.push_back(sbc);
  }
  return l;
  
};

DualConstraint::DualConstraint(std::vector<int> pcs, const Common::MsgHdr &msg, Common &c)
  : DualChannelConstraint(pcs, msg, c), common(c) {
};

DualConstraint::DualConstraint(std::vector<int> pcs, Common &c)
: DualChannelConstraint(pcs, c), common(c) {
};

std::list<const Machine::PTransition*> DualConstraint::partred() const{
  std::list<const Machine::PTransition*> l;
  if(use_limit_other_delete_propagate){
    for(unsigned p = 0; p < pcs.size(); ++p){
      bool has_read = false;
      
      for(auto it = common.transitions_by_pc[p][pcs[p]].begin(); it != common.transitions_by_pc[p][pcs[p]].end(); ++it){
        if((*it)->instruction.get_reads().size()){
          has_read = true;
        }
      }
      
      for(auto it = common.transitions_by_pc[p][pcs[p]].begin(); it != common.transitions_by_pc[p][pcs[p]].end(); ++it){
        if(((*it)->instruction.get_type() != Lang::DELETEE)|| has_read){
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




std::list<DualConstraint::pre_constr_t> DualConstraint::pre(const Machine::PTransition &t, bool locked) const{
  std::list<pre_constr_t> res;
  const Lang::Stmt<int> &s = t.instruction;
  if(pcs[t.pid] != t.target){
    return res;
  }

  switch(s.get_type()){
      
  case Lang::NOP: // nop
  {    
    DualConstraint *sbc = new DualConstraint(*this);
    sbc->pcs[t.pid] = t.source;
    res.push_back(sbc);
    break;
  }
      
  case Lang::ASSIGNMENT: // r = e
  {    
    if (reg_stores[t.pid][s.get_reg()].is_wild()) {
      DualConstraint *sbc = new DualConstraint(*this);
      sbc->pcs[t.pid] = t.source;
      res.push_back(sbc);
    }else{
      int reg_val = reg_stores[t.pid][s.get_reg()].get_int();
      
      VecSet<Store> rss = possible_reg_stores(reg_stores[t.pid],
                                              t.pid,
                                              s.get_expr(),
                                              reg_val);
      std::set<int> regs = s.get_expr().get_registers();
      // get STAR registers that not in expr e and register r to STAR
      VecSet<Store> correct_rss;
      for (int rssi=0; rssi<rss.size(); rssi++) {
        Store st = rss[rssi];
        for (int rsi=0; rsi<reg_stores[t.pid].size(); rsi++) {
          if ((regs.count(rsi)==0 && reg_stores[t.pid][rsi].is_wild()) ||
              (rsi==s.get_reg() && regs.count(rsi)==0)) {
            st = st.assign(rsi, value_t::STAR);
          }
        }
        correct_rss.insert(st);
      }

      for(int j = 0; j < correct_rss.size(); ++j){
        DualConstraint *sbc = new DualConstraint(*this);
        sbc->reg_stores[t.pid] = correct_rss[j];
        sbc->pcs[t.pid] = t.source;
        res.push_back(sbc);
      }
    }
    break;
  }
      
  case Lang::ASSUME: // assume: e;
  {    
    VecSet<Store> rstores = possible_reg_stores(reg_stores[t.pid],t.pid,s.get_condition());
    std::set<int> regs = s.get_condition().get_registers();
    // get STAR registers that not in expr from 0 to STAR
    VecSet<Store> correct_rstores;
    for (int rsi=0; rsi<rstores.size(); rsi++) {
      Store st = rstores[rsi];
      for (int rsi=0; rsi<reg_stores[t.pid].size(); rsi++) {
        if (regs.count(rsi)==0 && reg_stores[t.pid][rsi].is_wild()) {
          st = st.assign(rsi, value_t::STAR);
        }
      }
      correct_rstores.insert(st);
    }
    
    for(int i = 0; i < correct_rstores.size(); ++i){
      DualConstraint *sbc = new DualConstraint(*this);
      sbc->pcs[t.pid] = t.source;
      sbc->reg_stores[t.pid] = correct_rstores[i];
      res.push_back(sbc);
    }
    break;
  }
      
  case Lang::PROPAGATE: // propagate from mem to process t.pid
  {    
    bool ok_nmls = false;
    Lang::NML nml(s.get_memloc(),t.pid);
    int nmli = common.index(nml);
    int relation = -1;
    
    if (channels[t.pid].size()>0) {
      if (!channels[t.pid].back().store[0].is_wild()) {
        if (!mems[0][nmli].is_wild()) relation = 0;
        else relation = 1;
      } else {
        if (!mems[0][nmli].is_wild()) relation = 2;
        else relation = 3;
      }
    
      bool conflict = false;
      if ((relation == 0)
          && (mems[0][nmli].get_int() != channels[t.pid].back().store[0].get_int())) {
        conflict = true;
      }
      
      ok_nmls = (channels[t.pid].back().wpid == -1) &&
                (!conflict) &&
                (channels[t.pid].back().nmls.count(nml)!=0) &&
                (!use_propagate_only_after_write);
    }
    
    if(ok_nmls){
      DualConstraint *sbc = new DualConstraint(*this);
      sbc->pcs[t.pid] = t.source;
      std::vector<Msg> ch0(sbc->channels[t.pid]);
      ch0.pop_back();
      sbc->channels[t.pid] = ch0;
      if (relation == 1) { // set mems[0][nmli]
        sbc->mems[0] = sbc->mems[0].assign(nmli, channels[t.pid].back().store[0].get_int());
      }
      res.push_back(pre_constr_t(sbc,false,VecSet<Lang::NML>()));
    }
    break;
  }
    
  case Lang::LOCKED:
  {    
    /* Check if the locked block contains writes.
     * If so, it is fencing. */
    if(s.get_writes().size() == 0 || channels[t.pid].size() == 0){
      for(int i = 0; i < s.get_statement_count(); ++i){ // s can only contain single or sequence
        Machine::PTransition ti(t.source,*s.get_statement(i),t.target,t.pid);
        std::list<pre_constr_t> v = pre(ti,true);
        res.insert(res.end(),v.begin(),v.end());
      }
    }
    break;
  }
      
  case Lang::DELETEE:
  {    
    Lang::NML nml(s.get_memloc(),t.pid);
    bool own_exist = false;
    for (int mi = 0; mi < channels[t.pid].size(); mi++) {
      if (channels[t.pid][mi].nmls.count(nml)!=0 && channels[t.pid][mi].wpid == t.pid) {
        own_exist = true;
        break;
      }
    }
    if (!own_exist) {
      std::vector<value_t> v;
      v.push_back(value_t::STAR);
      Store st = Store(v);
      
      if (t.pid==s.get_writer()) { //insert an own message
        DualConstraint *sbc = new DualConstraint(*this);
        Msg msg(st,t.pid,VecSet<Lang::NML>::singleton(nml));
        sbc->channels[t.pid].insert(sbc->channels[t.pid].begin(), msg);
        res.push_back(sbc);
      }
    }    
    break;
  }
      
  case Lang::READASSERT:  // read: x = e. Allow read from mem if empty buffer
  {
    Lang::NML nml(s.get_memloc(),t.pid);
    int nmli = common.index(nml);
    int msgi = index_of_read(nml,t.pid);
    bool hidden = true;
    VecSet<Store> rss, correct_rss;
    int val_nml = 0;
    bool is_star = false;
    
    if(msgi==-1) {
      if (channels[t.pid][0].store[0].is_wild()) is_star = true;
      else val_nml = channels[t.pid][0].store[0].get_int();
    } else if (msgi>=0){
      if  (channels[t.pid][msgi].store[0].is_wild()) is_star = true;
      else val_nml = channels[t.pid][msgi].store[0].get_int();
    } else {
      if (mems[0][nmli].is_wild()) is_star = true;
      else val_nml = mems[0][nmli].get_int();
    }
    
    if (!is_star) {
      rss = possible_reg_stores(reg_stores[t.pid],
                                t.pid,
                                s.get_expr(),
                                val_nml);
      std::set<int> regs = s.get_expr().get_registers();
      // get STAR registers that not in expr from 0 to STAR
      VecSet<Store> correct_rss;
      for (int rssi=0; rssi<rss.size(); rssi++) {
        Store st = rss[rssi];
        for (int rsi=0; rsi<reg_stores[t.pid].size(); rsi++) {
          if (regs.count(rsi)==0 && reg_stores[t.pid][rsi].is_wild()) {
            st = st.assign(rsi, value_t::STAR);
          }
        }
        correct_rss.insert(st);
      }
    }
  
    VecSet<int> val_es = possible_values(reg_stores[t.pid],t.pid,s.get_expr());
      
    if (msgi>=0 || correct_rss.size()) hidden = false;

    if (is_star || correct_rss.size()) { // read from channels      
      if (correct_rss.size()) { // this case !is_star
        for (int rssi=0; rssi<correct_rss.size(); rssi++) {
          DualConstraint *sbc = new DualConstraint(*this);
          sbc->pcs[t.pid] = t.source;
          sbc->reg_stores[t.pid] = correct_rss[rssi]; // restrict registers
          res.push_back(sbc);
        }
      } else {
        for (int vei=0; vei<val_es.size(); vei++) {
          VecSet<Store> val_regss = possible_reg_stores(reg_stores[t.pid],
                                                        t.pid,
                                                        s.get_expr(),
                                                        val_es[vei]);
          std::set<int> regs = s.get_expr().get_registers();
          // get STAR registers that not in expr from 0 to STAR
          VecSet<Store> correct_val_regss;
          for (int vri=0; vri<val_regss.size(); vri++) {
            Store st = val_regss[vri];
            for (int rsi=0; rsi<reg_stores[t.pid].size(); rsi++) {
              if (regs.count(rsi)==0 && reg_stores[t.pid][rsi].is_wild()) {
                st = st.assign(rsi, value_t::STAR);
              }
            }
            correct_val_regss.insert(st);
          }
          
          for (int vri=0; vri<correct_val_regss.size(); vri++) {
            DualConstraint *sbc = new DualConstraint(*this);
            sbc->pcs[t.pid] = t.source;
            sbc->reg_stores[t.pid] = correct_val_regss[vri];
            
            if(msgi==-1) { // restrict mem
              sbc->channels[t.pid][0].store = sbc->channels[t.pid][0].store.assign(0,val_es[vei]);
              res.push_back(sbc);
            } else if (msgi>=0) {
              sbc->channels[t.pid][msgi].store = sbc->channels[t.pid][msgi].store.assign(0,val_es[vei]);
              res.push_back(sbc);
            } else if (sbc->channels[t.pid].size()==0) { //only for an empty channel
              sbc->mems[0] = sbc->mems[0].assign(nmli,val_es[vei]);
              res.push_back(sbc);  
            }
          }
        }
      }
    }
  
    if (hidden) { // read a hidden message in channel
      for (int vei=0; vei<val_es.size(); vei++) {
        std::vector<value_t> v;
        v.push_back(val_es[vei]);
        Store st = Store(v);
        
        VecSet<Store> val_regss = possible_reg_stores(reg_stores[t.pid],
                                                      t.pid,
                                                      s.get_expr(),
                                                      val_es[vei]);
        
        std::set<int> regs = s.get_expr().get_registers();
        // get STAR registers that not in expr from 0 to STAR
        VecSet<Store> correct_val_regss;
        for (int vri=0; vri<val_regss.size(); vri++) {
          Store st = val_regss[vri];
          for (int rsi=0; rsi<reg_stores[t.pid].size(); rsi++) {
            if (regs.count(rsi)==0 && reg_stores[t.pid][rsi].is_wild()) {
              st = st.assign(rsi, value_t::STAR);
            }
          }
          correct_val_regss.insert(st);
        }

        for (int vri=0; vri<correct_val_regss.size(); vri++) {
          DualConstraint *sbc = new DualConstraint(*this);
          sbc->pcs[t.pid] = t.source;
          sbc->reg_stores[t.pid] = correct_val_regss[vri];

          Msg msg(st,-1,VecSet<Lang::NML>::singleton(nml));
          if (sbc->channels[t.pid].size()>0) {
            sbc->channels[t.pid].insert(sbc->channels[t.pid].begin(),msg);
          } else {
            sbc->channels[t.pid].push_back(msg);
          }
          res.push_back(sbc);
        }
      }
    }
    break;
  }
      
  case Lang::READASSIGN: // read: r = x. Allow read from mem if empty buffer
  {    
    Lang::NML nml(s.get_memloc(),t.pid);
    int nmli = common.index(nml);
    int msgi = index_of_read(nml,t.pid);
    
    if (reg_stores[t.pid][s.get_reg()].is_wild()) {
      DualConstraint *sbc = new DualConstraint(*this);
      sbc->pcs[t.pid] = t.source;
      res.push_back(sbc);
    }else{
      int reg_val = reg_stores[t.pid][s.get_reg()].get_int();
      bool hidden = true;
      int val_nml;
      bool is_star = false;
      if(msgi==-1) {
        if (channels[t.pid][0].store[0].is_wild()) is_star = true;
        else val_nml = channels[t.pid][0].store[0].get_int();
      } else if (msgi>=0){
        if  (channels[t.pid][msgi].store[0].is_wild()) is_star = true;
        else val_nml = channels[t.pid][msgi].store[0].get_int();
      } else {
        if (mems[0][nmli].is_wild()) is_star = true;
        else val_nml = mems[0][nmli].get_int();
      }

      if (msgi>=0 || (!is_star && val_nml == reg_val)) hidden = false;

      if (is_star || val_nml == reg_val) {
        DualConstraint *sbc = new DualConstraint(*this);
        sbc->pcs[t.pid] = t.source;
        sbc->reg_stores[t.pid] = sbc->reg_stores[t.pid].assign(s.get_reg(), value_t::STAR);

        if (is_star) { // restrict mem
          if(msgi==-1) {
            sbc->channels[t.pid][0].store = sbc->channels[t.pid][0].store.assign(0,reg_val);
            res.push_back(sbc);
          } else if (msgi>=0) {
            sbc->channels[t.pid][msgi].store = sbc->channels[t.pid][msgi].store.assign(0,reg_val);
            res.push_back(sbc);
          } else if (sbc->channels[t.pid].size()==0) { //only for an empty channel
            sbc->mems[0] = sbc->mems[0].assign(nmli,reg_val);
            res.push_back(sbc);          
          }
        }
      }
            
      if (hidden) {
        std::vector<value_t> v;
        if (reg_stores[t.pid][s.get_reg()].is_wild()) {
          v.push_back(value_t::STAR);
        } else {
          v.push_back(reg_stores[t.pid][s.get_reg()].get_int());
        }
        Store st = Store(v);
        
        DualConstraint *sbc = new DualConstraint(*this);
        sbc->pcs[t.pid] = t.source;
        sbc->reg_stores[t.pid] = sbc->reg_stores[t.pid].assign(s.get_reg(), value_t::STAR);

        Msg msg(st,-1,VecSet<Lang::NML>::singleton(nml));
        if (sbc->channels[t.pid].size()>0) {
          sbc->channels[t.pid].insert(sbc->channels[t.pid].begin(),msg);
        } else {
          sbc->channels[t.pid].push_back(msg);
        }
        res.push_back(sbc);
      }
    }
    break;
  }
      
  case Lang::SEQUENCE:
  {    
    std::vector<pre_constr_t> v;
    v.push_back(pre_constr_t(new DualConstraint(*this)));
    for(int i = s.get_statement_count()-1; i >= 0; --i){
      std::vector<pre_constr_t> w;
      Machine::PTransition t2(t.target,*s.get_statement(i),t.target,t.pid);
      for(unsigned j = 0; j < v.size(); ++j){
        std::list<pre_constr_t> l = v[j].sbc->pre(t2,locked);
        for(auto it = l.begin(); it != l.end(); ++it){
          it->pop_back = it->pop_back || v[j].pop_back;
          it->written_nmls.insert(v[j].written_nmls);
          if (it->written_nmls.size() > 2) {
            throw new std::logic_error("DualConstraint::pre: Support for multiple memory locations not implemented.");
          }
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
  
  case Lang::WRITE: // x = e
  {    
    Lang::NML nml = Lang::NML(s.get_memloc(),t.pid);
    int nmli = common.index(nml);
    
    int relation = -1;
    bool conflict = false;
    bool is_star = false;
    int ok_nmls = 0;

    if (channels[t.pid].size()>0) {
      if (!channels[t.pid].back().store[0].is_wild()) {
        if (!mems[0][nmli].is_wild()) relation = 0;
        else relation = 1;
      } else {
        if (!mems[0][nmli].is_wild()) relation = 2;
        else relation = 3;
      }
      
      if ((relation == 0) && (mems[0][nmli].get_int() != channels[t.pid].back().store[0].get_int())) {
        conflict = true;
      }
    }

    if(locked && channels[t.pid].size() == 0){
      ok_nmls = 1;
    }else{
      if(channels[t.pid].size()>0) {
        if (channels[t.pid].back().nmls.size() == 1 && channels[t.pid].back().nmls.count(nml)
             && channels[t.pid].back().wpid == t.pid && !conflict)
          ok_nmls = 2;
      }
    }
    
    if(ok_nmls){
      int nml_val = -10000;
      if (ok_nmls ==2) {
        if (relation==3) is_star = true;
        else if (relation==1) nml_val = channels[t.pid].back().store[0].get_int();
        else nml_val = mems[0][nmli].get_int();
      }
      else { // locked case
        if (mems[0][nmli].is_wild()) is_star = true;
        else nml_val = mems[0][nmli].get_int();
      }
      
      if (is_star) {
          Store new_mem = mems[0].assign(nmli,value_t::STAR);
          
          // shorten channels
          DualConstraint *sbc = new DualConstraint(*this);
          sbc->pcs[t.pid] = t.source;
          sbc->mems[0] = new_mem;

          if(!locked) {
            std::vector<Msg> ch0(sbc->channels[t.pid]);
            assert(ch0.size());
            ch0.pop_back();
            sbc->channels[t.pid] = ch0;
          }
          
          res.push_back(pre_constr_t(sbc,false,VecSet<Lang::NML>::singleton(nml)));
          
          if (!locked) {
            assert(channels[t.pid].size());

            std::vector<value_t> v;
            v.push_back(value_t::STAR);
            Store st = Store(v);
            
            Msg msg(st,t.pid,VecSet<Lang::NML>::singleton(nml));
            
            //insert to the end of the channels
            DualConstraint *sbc = new DualConstraint(*this);
            sbc->pcs[t.pid] = t.source;
            sbc->mems[0] = new_mem;
          
            std::vector<Msg> ch0(channels[t.pid]);
            ch0.pop_back();
            ch0.push_back(msg);
            sbc->channels[t.pid] = ch0;
          
            res.push_back(pre_constr_t(sbc,false,VecSet<Lang::NML>::singleton(nml)));
                        
            // insert to other possible positions of channels
            for (int it=channels[t.pid].size()-2; it>=0;  it--) {
              bool varSame = (channels[t.pid][it].nmls.size() == 1 && 
                              channels[t.pid][it].nmls.count(nml));
              if (channels[t.pid][it].wpid != t.pid || !varSame) {
                DualConstraint *sbc = new DualConstraint(*this);
                sbc->pcs[t.pid] = t.source;
            
                std::vector<Msg> ch0(channels[t.pid]);
                ch0.pop_back();
                ch0.insert(ch0.begin()+it, msg);
                sbc->mems[0] = new_mem;
                sbc->channels[t.pid] = ch0;
                
                res.push_back(pre_constr_t(sbc,false,VecSet<Lang::NML>::singleton(nml)));
              } else { // do not insert more after getting an own message to same variable
                break;
              }
            }
          }
      } else { // restrict registers
        VecSet<Store> rstores = possible_reg_stores(reg_stores[t.pid],t.pid,s.get_expr(),nml_val);
        std::set<int> regs = s.get_expr().get_registers();
        // get STAR registers that not in expr e and register r to STAR
        VecSet<Store> correct_rstores;
        for (int rssi=0; rssi<rstores.size(); rssi++) {
          Store st = rstores[rssi];
          for (int rsi=0; rsi<reg_stores[t.pid].size(); rsi++) {
            if (regs.count(rsi)==0 && reg_stores[t.pid][rsi].is_wild()) {
              st = st.assign(rsi, value_t::STAR);
            }
          }
          correct_rstores.insert(st);
        }

        for (int rssi=0; rssi<correct_rstores.size(); rssi++) {
          Store new_mem = mems[0].assign(nmli,value_t::STAR);
          
          // shorten channels
          DualConstraint *sbc = new DualConstraint(*this);
          sbc->pcs[t.pid] = t.source;
          sbc->mems[0] = new_mem;
          sbc->reg_stores[t.pid] = correct_rstores[rssi];
        
          if(!locked) {
            std::vector<Msg> ch0(sbc->channels[t.pid]);
            assert(ch0.size()>0);
            ch0.pop_back();
            sbc->channels[t.pid] = ch0;
          }
        
          res.push_back(pre_constr_t(sbc,false,VecSet<Lang::NML>::singleton(nml)));
          
          if (!locked) {
            assert(channels[t.pid].size());

            std::vector<value_t> v;
            v.push_back(value_t::STAR);
            Store st = Store(v);
            
            Msg msg(st,t.pid,VecSet<Lang::NML>::singleton(nml));
            
            //insert to the end of the channels
            DualConstraint *sbc = new DualConstraint(*this);
            sbc->pcs[t.pid] = t.source;
            sbc->mems[0] = new_mem;
            sbc->reg_stores[t.pid] = correct_rstores[rssi];
          
            std::vector<Msg> ch0(channels[t.pid]);
            ch0.pop_back();
            ch0.push_back(msg);
            sbc->channels[t.pid] = ch0;
            
            res.push_back(pre_constr_t(sbc,false,VecSet<Lang::NML>::singleton(nml)));
                        
            // insert to other possible positions of channels
            for (int it=channels[t.pid].size()-2; it>=0;  it--) {
              bool varSame = (channels[t.pid][it].nmls.size() == 1 && 
                              channels[t.pid][it].nmls.count(nml));
              if (channels[t.pid][it].wpid != t.pid || !varSame) {
                DualConstraint *sbc = new DualConstraint(*this);
                sbc->pcs[t.pid] = t.source;
                
                std::vector<Msg> ch0(channels[t.pid]);
                ch0.pop_back();
                ch0.insert(ch0.begin()+it, msg);
                sbc->reg_stores[t.pid] = correct_rstores[rssi];
                sbc->mems[0] = new_mem;
                sbc->channels[t.pid] = ch0;
                
                res.push_back(pre_constr_t(sbc,false,VecSet<Lang::NML>::singleton(nml)));
              } else { // do not insert more after getting an own message to same variable
                break;
              }
            }
          }
        }
      }
    }
    break;
  }
      
  default:
    throw new std::logic_error("DualConstraint::pre: Unsupported transition: "+t.to_string(common.machine));
  }
    
  return res;
};

std::list<Constraint*> DualConstraint::pre(const Machine::PTransition &t) const{
  std::list<Constraint*> res;
  std::list<DualConstraint::pre_constr_t> r = pre(t,false);
  for(auto it = r.begin(); it != r.end(); ++it){
    if(1){
      res.push_back(it->sbc);
    }else{
      delete it->sbc;
    }
  }

  return res;
};



void DualConstraint::Common::test(){
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
    expected.insert(MsgHdr(0,VecSet<Lang::NML>()));
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

void DualConstraint::test_possible_values(){
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

    /* Construct dummy DualConstraint */
    Common common(dummy_machine);
    DualConstraint sbc(std::vector<int>(1,0),common.messages[0],common);

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

void DualConstraint::test_pre(){
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

  /* Construct dummy DualConstraint */
  Common common(dummy_machine);

   /* Test NOP */
  {
    std::cout << " ** NOP **\n";
    std::vector<int> pcs;
    pcs.push_back(1); pcs.push_back(3);
    DualConstraint sbc(pcs,common.messages[0],common);
    Machine::PTransition t(0,Lang::Stmt<int>::nop(),1,0);
    std::list<Constraint*> res = sbc.pre(t);
    std::cout << res.front()->to_string() << "\n\n";
  }

  /* Test READASSERT */
  {
    std::cout << " ** READASSERT x = $r0 + $r1 **\n";
    std::vector<int> pcs;
    pcs.push_back(1); pcs.push_back(3);
    DualConstraint sbc(pcs,common.messages[0],common);
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
    DualConstraint sbc(pcs,common.messages[0],common);
    Machine::PTransition t(0,Lang::Stmt<int>::write(Lang::MemLoc<int>::global(0),
                                                    Lang::Expr<int>::reg(0) + Lang::Expr<int>::reg(1)),1,0);
    if(sbc.pre(t).empty()){
      std::cout << "Test3: Success!\n\n";
    }else{
      std::cout << "Test3: Failure\n\n";
    }
  }

  /* Test WRITE (Test4) */
  {
    std::cout << " ** WRITE x := r0 + r1 [r0=0,r1=10,y=*] **\n";
    std::vector<int> pcs;
    pcs.push_back(1); pcs.push_back(3);
    DualConstraint sbc(pcs,common.messages[0],common);
    Machine::PTransition t(0,Lang::Stmt<int>::write(Lang::MemLoc<int>::global(0),
                                                    Lang::Expr<int>::reg(0) + Lang::Expr<int>::reg(1)),1,0);
    Msg msg(Store(1),0,VecSet<Lang::NML>::singleton(Lang::NML::global(0)));
    sbc.channels[0].push_back(msg);
    sbc.reg_stores[0] = sbc.reg_stores[0].assign(0,0).assign(1,10);
    std::cout << "Initial:\n" << sbc.to_string() << "\n";
    std::list<Constraint*> res = sbc.pre(t);
    std::cout << "Pre:\n";
    for(auto it = res.begin	(); it != res.end(); ++it){
      std::cout << (*it)->to_string() << "\n";
    }
  }

  /* Test LOCKED WRITE */
  {
    std::cout << " ** LOCKED WRITE x := r0 + r1 [r0=0,r1=10,x=*] **\n";
    std::vector<int> pcs;
    pcs.push_back(1); pcs.push_back(3);
    DualConstraint sbc(pcs,common.messages[0],common);
    Machine::PTransition t(0,Lang::Stmt<int>::locked_write(Lang::MemLoc<int>::global(0),
                                                            Lang::Expr<int>::reg(0) + Lang::Expr<int>::reg(1)),1,0);
    Msg msg(Store(1),0,VecSet<Lang::NML>::singleton(Lang::NML::global(0)));
    sbc.channels[0].push_back(msg);
    sbc.reg_stores[0] = sbc.reg_stores[0].assign(0,0).assign(1,10);
    std::cout << "Initial:\n" << sbc.to_string() << "\n";
    std::list<Constraint*> res = sbc.pre(t);
    std::cout << "Pre:\n";
    for(auto it = res.begin(); it != res.end(); ++it){
      std::cout << (*it)->to_string() << "\n";
    }
  }

};



void DualConstraint::test_comparison(){
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

    /* Construct dummy DualConstraint */
    Common common(dummy_machine);

    std::function<bool(std::string,bool)> test = 
      [](std::string name, bool result)->bool{
      if(result){
        std::cout << name << "***: Success !\n";
      }else{
        std::cout << name << "***: Failure \n";
      }
      return result;
    };

    /* Test1: */
    {
      std::vector<int> pcs(2,0);
      DualConstraint sbc0(pcs,common);
      DualConstraint sbc1(pcs,common);
      Msg msg0(Store(6),0,us);
      //Msg msg1(Store(6),1,us);
      sbc0.channels[0].clear(); sbc1.channels[0].clear();
      sbc0.channels[0].push_back(msg0);
      sbc0.channels[0].push_back(msg0);
      sbc1.channels[0].push_back(msg0);
      sbc1.channels[0].push_back(msg0);
      sbc1.channels[0].push_back(msg0);
      test("Test1a",sbc0.entailment_compare(sbc1) == Constraint::LESS);
      test("Test1b",sbc1.entailment_compare(sbc0) == Constraint::GREATER);
      test("Test1c",(sbc0.characterize_channels())[0].size() == (sbc1.characterize_channels())[0].size());
    }
    /* Test2:  */
    {
      std::vector<int> pcs(2,0);
      DualConstraint sbc0(pcs,common);
      DualConstraint sbc1(pcs,common);
      Msg msg(Store(6),0,us);
      sbc0.channels[0].clear(); sbc1.channels[0].clear();
      sbc0.channels[0].push_back(msg);
      sbc0.channels[0].push_back(msg);
      sbc1.channels[0].push_back(msg);
      sbc1.channels[0].push_back(msg);
      sbc1.channels[0].push_back(msg);
      sbc1.channels[0][1].wpid = 1;
      test("Test2a",sbc0.entailment_compare(sbc1) != Constraint::EQUAL);
      test("Test2b",sbc1.entailment_compare(sbc0) != Constraint::EQUAL);
      test("Test2c",sbc0.characterize_channels() == sbc1.characterize_channels());
      test("Test2d",sbc0.entailment_compare(sbc1) == Constraint::LESS);
      test("Test2e",sbc1.entailment_compare(sbc0) == Constraint::GREATER);
    }
    /* Test3: */
    {
      std::vector<int> pcs(2,0);
      DualConstraint sbc0(pcs,common);
      DualConstraint sbc1(pcs,common);
      Msg msg(Store(6),0,us);
      sbc0.channels[0].clear(); sbc1.channels[0].clear();
      sbc0.channels[0].push_back(msg);
      sbc0.channels[0].push_back(msg);
      sbc1.channels[0].push_back(msg);
      sbc1.channels[0].push_back(msg);
      sbc1.channels[0].push_back(msg);
      test("Test3a",sbc0.entailment_compare(sbc1) != Constraint::EQUAL);
      test("Test3b",sbc1.entailment_compare(sbc0) != Constraint::EQUAL);
      test("Test3c",sbc0.characterize_channels() == sbc1.characterize_channels());
      test("Test3d",sbc0.entailment_compare(sbc1) == Constraint::LESS);
      test("Test3e",sbc1.entailment_compare(sbc0) == Constraint::GREATER);
    }
    /* Test4: */
    {
      std::vector<int> pcs(2,0);
      DualConstraint sbc0(pcs,common);
      DualConstraint sbc1(pcs,common);
      Msg msg(Store(6),0,us);
      sbc0.channels[0].clear(); sbc1.channels[0].clear();
      sbc0.channels[0].push_back(msg);
      sbc0.channels[0].push_back(msg);
      sbc1.channels[0].push_back(msg);
      sbc1.channels[0].push_back(msg);
      sbc1.channels[0].push_back(msg);
      sbc0.channels[0][0].store = sbc0.channels[0][0].store.assign(0,0);
      sbc1.channels[0][0].store = sbc1.channels[0][0].store.assign(0,0).assign(1,1);
      test("Test4a",sbc0.entailment_compare(sbc1) == Constraint::LESS);
      test("Test4b",sbc1.entailment_compare(sbc0) == Constraint::GREATER);
      test("Test4c",sbc0.characterize_channels() == sbc1.characterize_channels());
      test("Test4d",sbc0.channels[0][0].entailment_compare(sbc1.channels[0][0]) == Constraint::LESS);
      test("Test4e",sbc1.channels[0][0].entailment_compare(sbc0.channels[0][0]) == Constraint::GREATER);
    }
    /* Test5: */
    {
      std::vector<int> pcs(2,0);
      DualConstraint sbc0(pcs,common);
      DualConstraint sbc1(pcs,common);
      Msg msg(Store(6),0,us);
      sbc0.channels[0].clear(); sbc1.channels[0].clear();
      sbc0.channels[0].push_back(msg);
      sbc0.channels[0].push_back(msg);
      sbc1.channels[0].push_back(msg);
      sbc1.channels[0].push_back(msg);
      sbc1.channels[0].push_back(msg);
      sbc0.channels[0][0].store = sbc0.channels[0][0].store.assign(0,0).assign(1,1);
      sbc1.channels[0][0].store = sbc1.channels[0][0].store.assign(0,0);
      test("Test5a",sbc0.entailment_compare(sbc1) == Constraint::INCOMPARABLE);
      test("Test5b",sbc1.entailment_compare(sbc0) == Constraint::INCOMPARABLE);
      test("Test5c",sbc0.characterize_channels() == sbc1.characterize_channels());
      test("Test5d",sbc0.entailment_compare(sbc0) == Constraint::EQUAL);
      test("Test5e",sbc1.entailment_compare(sbc1) == Constraint::EQUAL);
    }
  }catch(std::exception *exc){
    std::cout << "Error: " << exc->what() << "\n";
    throw;
  }
};



void DualConstraint::test(){
  std::cout << " * test_possible_values\n";
  test_possible_values();
  std::cout << " * test_pre\n";
  test_pre();
  std::cout << " * Common::test\n";
  Common::test();
  std::cout << " * test_comparison\n";
  test_comparison();
};