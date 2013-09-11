/*
 * Copyright (C) 2013 Carl Leonardsson
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

#include "fence_sync.h"
#include "log.h"
#include "parser.h"
#include "preprocessor.h"
#include "test.h"
#include "tso_lock_sync.h"
#include "vips_syncwr_sync.h"

#include <algorithm>
#include <list>
#include <sstream>
#include <stdexcept>

FenceSync::FenceSync(Lang::Stmt<int> f, int pid, int q, 
                     TSet IN, TSet OUT)
  : f(f), pid(pid), q(q), IN(IN), OUT(OUT) {
};

std::string FenceSync::to_raw_string() const{
  return to_string_aux(Lang::int_reg_to_string(),
                       Lang::int_memloc_to_string());
};

std::string FenceSync::to_string(const Machine &m) const{
  return to_string_aux(m.reg_pretty_vts(pid),m.ml_pretty_vts(pid));
};

Machine *FenceSync::insert(const Machine &m,
                           m_infos_t m_infos,
                           Sync::InsInfo **info) const{
  if(!applies_to(m,m_infos)){
    throw new std::logic_error("FenceSync::insert: "
                               "This FenceSync does not apply to the target machine.");
  }

  struct FS{
    FS(m_infos_t m_infos,
       const FenceSync *fs, int cstate, const InsInfo *info)
      : fs(fs), cstate(cstate), info(info) {
      for(auto it = fs->IN.begin(); it != fs->IN.end(); ++it){
        IN_m.insert(InsInfo::all_tchanges(m_infos,Machine::PTransition(*it,fs->pid)));
      }
      for(auto it = fs->OUT.begin(); it != fs->OUT.end(); ++it){
        OUT_m.insert(InsInfo::all_tchanges(m_infos,Machine::PTransition(*it,fs->pid)));
      }
    };
    const FenceSync *fs;
    int cstate;
    // fs->IN resp. fs->OUT, updated as the transitions occur in m
    TSet IN_m, OUT_m;
    const InsInfo *info;
    /* Sort FenceSyncs such that
     * - If fs0.IN is a strict subset of fs1.IN, then fs0 < fs1
     * - If fs0.OUT is a strict superset of fs1.OUT, then fs0 < fs1
     * - Arbitrary tie breaking.
     */
    bool operator<(const FS &b) const{
      int c = ss_cmp(b);
      if(c == 0){
        // Break tie
        return fs->compare(*b.fs) < 0;
      }else{
        return c < 0;
      }
    };
    int ss_cmp(const FS &b) const{
      if(fs->IN != b.fs->IN && subset(fs->IN,b.fs->IN)){
        return -1;
      }else if(fs->IN != b.fs->IN && subset(b.fs->IN,fs->IN)){
        return 1;
      }else if(fs->OUT != b.fs->OUT && subset(fs->OUT,b.fs->OUT)){
        return 1;
      }else if(fs->OUT != b.fs->OUT && subset(b.fs->OUT,fs->OUT)){
        return -1;
      }
      return 0;
    };
  };

  /* Find all FenceSyncs that apply to the same control location as this one */
  std::vector<FS> q_fss;
  {
    q_fss.push_back(FS(m_infos,this,-1,0));
    for(unsigned i = 0; i < m_infos.size(); ++i){
      const FenceSync *fs = dynamic_cast<const FenceSync*>(m_infos[i]->sync);
      if(fs && fs->q == q && fs->pid == pid){
        assert(dynamic_cast<const InsInfo*>(m_infos[i]));
        q_fss.push_back(FS(m_infos,fs,-1,static_cast<const InsInfo*>(m_infos[i])));
      }
    }
  }

  std::sort(q_fss.begin(),q_fss.end());

  /* Find all control locations currently corresponding to q */
  std::set<int> old_qs;
  old_qs.insert(q);
  for(int i = (int)m_infos.size()-1; i >= 0; --i){
    const FenceSync *fs = dynamic_cast<const FenceSync*>(m_infos[i]->sync);
    if(fs && fs->pid == pid && fs->q == q){
      assert(dynamic_cast<const InsInfo*>(m_infos[i]));
      old_qs = static_cast<const InsInfo*>(m_infos[i])->new_qs;
      break;
    }
  }

  Machine *m2 = new Machine(m);
  InsInfo *my_info = new InsInfo(static_cast<const FenceSync*>(clone()));
  *info = my_info;
  my_info->setup_id_tchanges(m);

  /* Find all original transitions to and from q as they appear in m2 */
  TSet q_IN, q_OUT;
  {
    auto pr = get_m_q_IN_OUT(m,m_infos);
    q_IN = pr.first;
    q_OUT = pr.second;
  }

  /* Remove all transitions to and from q in m2 */
  {
    TSet ts;
    for(auto q_it = old_qs.begin(); q_it != old_qs.end(); ++q_it){
      const Automaton::State &st = m2->automata[pid].get_states()[*q_it];
      for(auto t_it = st.bwd_transitions.begin(); t_it != st.bwd_transitions.end(); ++t_it){
        ts.insert(**t_it);
      }
      for(auto t_it = st.fwd_transitions.begin(); t_it != st.fwd_transitions.end(); ++t_it){
        ts.insert(**t_it);
      }
    }
    for(auto t_it = ts.begin(); t_it != ts.end(); ++t_it){
      m2->automata[pid].del_transition(*t_it);
    }
  }

  /* Return an unused control state that is not q.
   * Uses states from old_qs as long as there are such left.
   */
  auto old_qs_it = old_qs.begin();
  int next_new_state = m.automata[this->pid].get_states().size();
  std::set<int> new_qs = old_qs;
  std::function<int()> new_state = 
    [this,&old_qs,&m,&old_qs_it,&next_new_state,&new_qs](){
    if(old_qs_it != old_qs.end() && *old_qs_it == this->q){
      ++old_qs_it;
    }
    if(old_qs_it == old_qs.end()){
      int ns = next_new_state;
      ++next_new_state;
      new_qs.insert(ns);
      return ns;
    }else{
      int ns = *old_qs_it;
      ++old_qs_it;
      return ns;
    }
  };

  /* Collect transitions that should be assigned new source/target
   * control states here. */
  std::map<Automaton::Transition,int,TransCmp> q_IN_tgts, q_OUT_srcs;

  /* Find the border in q_fss between incoming and outgoing fences */
  unsigned q_i; // The first fence in q_fss that is outgoing
  for(q_i = 0; q_i < q_fss.size(); ++q_i){
    if(q_fss[q_i].fs->OUT.size() < q_OUT.size()){
      break;
    }
  }

  std::function<void(int,const FS&,int)> add_fence = 
    [this,m2,my_info,&m_infos](int src,const FS &fs,int tgt){
    Automaton::Transition t(src,fs.fs->f,tgt);
    m2->automata[this->pid].add_transition(t);
    if(fs.info){
      /* This is an old fence. Remember it in tchanges. */
      int i = 0;
      while(m_infos[i] != fs.info) ++i;
      ++i;
      Machine::PTransition fnc = InsInfo::all_tchanges(m_infos,fs.info->fence,i);
      assert(my_info->tchanges.count(fnc));
      my_info->bind(fnc,Machine::PTransition(t,this->pid));
    }else{
      /* This is the fence of this FenceSync */
      my_info->fence = Machine::PTransition(t,this->pid);
    }
  };

  /* Setup structure of incoming fences */
  {
    std::list<FS> cur_lvl;
    TSet q_IN_rem = q_IN;
    for(unsigned i = 0; i < q_i; ++i){
      q_fss[i].cstate = new_state();
      TSet IN_rem = q_fss[i].IN_m;
      for(auto it = cur_lvl.begin(); it != cur_lvl.end();){
        if(subset(it->IN_m,q_fss[i].IN_m)){
          add_fence(it->cstate,*it,q_fss[i].cstate);
          IN_rem = set_minus(IN_rem,it->IN_m);
          it = cur_lvl.erase(it);
        }else{
          ++it;
        }
      }
      cur_lvl.push_back(q_fss[i]);
      for(auto it = q_fss[i].IN_m.begin(); it != q_fss[i].IN_m.end(); ++it){
        if(IN_rem.count(*it)){
          /* Remember that incoming non-fence transitions should be
           * redirected through this fence. */
          assert(q_IN_tgts.count(*it) == 0);
          assert(q_IN_rem.count(*it));
          q_IN_tgts[*it] = q_fss[i].cstate;
        }
      }
    }
    for(auto it = cur_lvl.begin(); it != cur_lvl.end(); ++it){
      add_fence(it->cstate,*it,q);
    }
  }

  /* Setup structure of outgoing fences */
  {
    std::list<FS> cur_lvl;
    TSet q_OUT_rem = q_OUT;
    for(int i = (int)q_fss.size()-1; (int)q_i <= i; --i){
      q_fss[i].cstate = new_state();
      TSet OUT_rem = q_fss[i].OUT_m;
      for(auto it = cur_lvl.begin(); it != cur_lvl.end();){
        if(subset(it->OUT_m,q_fss[i].OUT_m)){
          add_fence(q_fss[i].cstate,*it,it->cstate);
          OUT_rem = set_minus(OUT_rem,it->OUT_m);
          it = cur_lvl.erase(it);
        }else{
          ++it;
        }
      }
      cur_lvl.push_back(q_fss[i]);
      for(auto it = q_fss[i].OUT_m.begin(); it != q_fss[i].OUT_m.end(); ++it){
        if(OUT_rem.count(*it)){
          /* Remember that outgoing non-fence transitions should be
           * redirected through this fence. */
          assert(q_OUT_srcs.count(*it) == 0);
          assert(q_OUT_rem.count(*it));
          q_OUT_srcs[*it] = q_fss[i].cstate;
        }
      }
    }
    for(auto it = cur_lvl.begin(); it != cur_lvl.end(); ++it){
      add_fence(q,*it,it->cstate);
    }
  }

  /* Add non-fence transitions */
  {
    TSet q_INOUT = q_IN;
    q_INOUT.insert(q_OUT.begin(),q_OUT.end());
    for(auto it = q_INOUT.begin(); it != q_INOUT.end(); ++it){
      int src = it->source;
      int tgt = it->target;
      if(q_IN_tgts.count(*it)){
        tgt = q_IN_tgts[*it];
      }
      if(q_OUT_srcs.count(*it)){
        src = q_OUT_srcs[*it];
      }
      Automaton::Transition t(src,it->instruction,tgt);
      m2->automata[pid].add_transition(t);
      assert(my_info->tchanges.count(Machine::PTransition(*it,pid)));
      my_info->bind(Machine::PTransition(*it,pid),
                    Machine::PTransition(t,pid));
    }
  }

  my_info->new_qs = new_qs;
  return m2;
};

bool FenceSync::applies_to(const Machine &m, m_infos_t m_infos) const{
  if(pid < 0){
    std::stringstream ss;
    ss << "FenceSync::insert: Invalid process: " << pid;
    throw new std::logic_error(ss.str());
  }
  if((int)m.automata.size() <= pid){
    return false;
  }
  if(q < 0){
    std::stringstream ss;
    ss << "FenceSync::insert: Invalid control state: " << q;
    throw new std::logic_error(ss.str());
  }
  if((int)m.automata[pid].get_states().size() <= q){
    return false;
  }

  for(unsigned i = 0; i < m_infos.size(); ++i){
    if(dynamic_cast<const FenceSync::InsInfo*>(m_infos[i])){
      assert(dynamic_cast<const FenceSync*>(m_infos[i]->sync));
      const FenceSync *fs = static_cast<const FenceSync*>(m_infos[i]->sync);
      if(fs->q == q && fs->pid == pid && !compatible(*fs)){
        throw new Incompatible(m_infos[i],"FenceSync: Previous FenceSync to same "
                               "control location not subset related.");
      }
    }else if(dynamic_cast<const TsoLockSync::InsInfo*>(m_infos[i]) ||
             dynamic_cast<const VipsSyncwrSync::InsInfo*>(m_infos[i])){
      // Ok
    }else{
      throw new std::logic_error("FenceSync: Unsupported kind of Sync has been inserted before.");
    }
  }

  auto pr = get_orig_q_IN_OUT(m,m_infos);

  return
    (subset(IN,pr.first) && subset(OUT,pr.second) &&
     (IN.size() == pr.first.size() || OUT.size() == pr.second.size()));
};

std::pair<FenceSync::TSet,FenceSync::TSet>
FenceSync::get_orig_q_IN_OUT(const Machine &m,
                             m_infos_t m_infos) const{
  TSet I,O;
  if(m_infos.size()){
    /* Search the first tchanges for transitions */
    std::map<Machine::PTransition,Machine::PTransition> tch;
    {
      const InsInfo *ii0 = dynamic_cast<const InsInfo*>(m_infos[0]);
      const TsoLockSync::InsInfo *ii1 = dynamic_cast<const TsoLockSync::InsInfo*>(m_infos[0]);
      const VipsSyncwrSync::InsInfo *ii2 =
        dynamic_cast<const VipsSyncwrSync::InsInfo*>(m_infos[0]);
      if(ii0){
        tch = ii0->tchanges;
      }else if(ii1){
        tch = ii1->tchanges;
      }else{
        assert(ii2);
        tch = ii2->tchanges;
      }
    }
    for(auto it = tch.begin(); it != tch.end(); ++it){
      if(it->first.source == q && it->first.pid == pid){
        O.insert(it->first);
      }
      if(it->first.target == q && it->first.pid == pid){
        I.insert(it->first);
      }
    }
  }else{
    /* m is the original machine, take the transitions as they are */
    const Automaton::State &st = m.automata[pid].get_states()[q];
    for(auto t_it = st.bwd_transitions.begin(); t_it != st.bwd_transitions.end(); ++t_it){
      I.insert(**t_it);
    }
    for(auto t_it = st.fwd_transitions.begin(); t_it != st.fwd_transitions.end(); ++t_it){
      O.insert(**t_it);
    }
  }
  return std::pair<TSet,TSet>(I,O);
};

std::pair<FenceSync::TSet,FenceSync::TSet>
FenceSync::get_m_q_IN_OUT(const Machine &m,
                          m_infos_t m_infos) const{
  auto pr = get_orig_q_IN_OUT(m,m_infos);
  TSet I, O;
  for(auto it = pr.first.begin(); it != pr.first.end(); ++it){
    I.insert(InsInfo::all_tchanges(m_infos,Machine::PTransition(*it,pid)));
  }
  for(auto it = pr.second.begin(); it != pr.second.end(); ++it){
    O.insert(InsInfo::all_tchanges(m_infos,Machine::PTransition(*it,pid)));
  }
  return std::pair<TSet,TSet>(I,O);
};

void FenceSync::print_raw(Log::redirection_stream &os, Log::redirection_stream &json_os) const{
  print_aux(Lang::int_reg_to_string(),
            Lang::int_memloc_to_string(),
            os,json_os);
};

void FenceSync::print(const Machine &m, Log::redirection_stream &os, Log::redirection_stream &json_os) const{
  print_aux(m.reg_pretty_vts(pid),m.ml_pretty_vts(pid),
            os,json_os);
};

void FenceSync::print_aux(const std::function<std::string(const int&)> &regts, 
                          const std::function<std::string(const Lang::MemLoc<int> &)> &mlts,
                          Log::redirection_stream &os, Log::redirection_stream &json_os) const{
  os << "FenceSync(P" << pid << ",Q" << q << ",f:" << f.to_string(regts,mlts) << ")\n";
  for(auto it = IN.begin(); it != IN.end(); ++it){
    os << "  IN: " << it->to_string(regts,mlts) << "\n";
    if(*os.os && it->instruction.get_pos().get_line_no() >= 0){
      json_os << "json: {\"action\":\"Link Fence\", \"pos\":" << it->instruction.get_pos().to_json() << "}\n";
    }
  }
  for(auto it = OUT.begin(); it != OUT.end(); ++it){
    os << "  OUT: " << it->to_string(regts,mlts) << "\n";
    if(*os.os && it->instruction.get_pos().get_line_no() >= 0){
      json_os << "json: {\"action\":\"Link Fence\", \"pos\":" << it->instruction.get_pos().to_json() << "}\n";
    }
  }
};

std::string FenceSync::to_string_aux(const std::function<std::string(const int&)> &regts, 
                                     const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const{
  std::stringstream ss;
  ss << "FenceSync(P" << pid << ",Q" << q << ",f:" << f.to_string(regts,mlts) << ")\n";
  for(auto it = IN.begin(); it != IN.end(); ++it){
    ss << "  IN: " << it->to_string(regts,mlts) << "\n";
  }
  for(auto it = OUT.begin(); it != OUT.end(); ++it){
    ss << "  OUT: " << it->to_string(regts,mlts) << "\n";
  }
  return ss.str();
};

void FenceSync::InsInfo::setup_id_tchanges(const Machine &m){
  tchanges.clear();
  for(unsigned p = 0; p < m.automata.size(); ++p){
    auto states = m.automata[p].get_states();
    for(unsigned i = 0; i < states.size(); ++i){
      for(auto it = states[i].fwd_transitions.begin();
          it != states[i].fwd_transitions.end(); ++it){
        bind(Machine::PTransition(**it,p),
             Machine::PTransition(**it,p));
      }
    }
  }
};

FenceSync::TSet FenceSync::set_minus(const TSet &S, const TSet &T){
  TSet U;
  auto S_it = S.begin();
  auto T_it = T.begin();
  while(S_it != S.end()){
    if(T_it == T.end() || *S_it < *T_it){
      U.insert(*S_it);
      ++S_it;
    }else if(*S_it == *T_it){
      ++S_it;
      ++T_it;
    }else{
      ++T_it;
    }
  }
  return U;
};

bool FenceSync::disjunct(const TSet &S, const TSet &T){
  auto S_it = S.begin();
  auto T_it = T.begin();
  TransCmp cmp;
  while(S_it != S.end() && T_it != T.end()){
    if(*S_it == *T_it){
      return false;
    }else if(cmp(*S_it,*T_it)){
      ++S_it;
    }else{
      ++T_it;
    }
  }
  return true;
};

bool FenceSync::subset_related(const TSet &S, const TSet &T){
  return subset(S,T) || subset(T,S);
};

bool FenceSync::subset(const TSet &S, const TSet &T){
  return std::includes<TSet::const_iterator,TSet::const_iterator,TransCmp>
    (T.begin(),T.end(),S.begin(),S.end(),TransCmp());
};

bool FenceSync::subset_related(const FenceSync &fs) const{
  return subset_related(IN,fs.IN) && subset_related(OUT,fs.OUT);
};

bool FenceSync::compatible(const FenceSync &fs) const{
  return (subset_related(IN,fs.IN) || disjunct(IN,fs.IN)) &&
    (subset_related(OUT,fs.OUT) || disjunct(OUT,fs.OUT));
};

std::set<Sync*> FenceSync::get_all_possible(const Machine &m,
                                            const std::set<Lang::Stmt<int> > &fs,
                                            const fs_init_t &fsinit){
  std::set<Sync*> ss;
  for(unsigned p = 0; p < m.automata.size(); ++p){
    const std::vector<Automaton::State> &states = m.automata[p].get_states();
    for(unsigned i = 0; i < states.size(); ++i){
      TSet IN, OUT;
      for(auto it = states[i].bwd_transitions.begin(); it != states[i].bwd_transitions.end(); ++it){
        IN.insert(**it);
      }
      for(auto it = states[i].fwd_transitions.begin(); it != states[i].fwd_transitions.end(); ++it){
        OUT.insert(**it);
      }
      if(IN.size() && OUT.size()){
        std::set<TSet> INS = powerset(IN);
        std::set<TSet> OUTS = powerset(OUT);
        for(auto init = INS.begin(); init != INS.end(); ++init){
          /* Skip empty set and the complete set */
          if(init->empty() || init->size() == IN.size()) continue;
          for(auto fit = fs.begin(); fit != fs.end(); ++fit){
            ss.insert(fsinit(*fit,p,i,*init,OUT));
          }
        }
        for(auto outit = OUTS.begin(); outit != OUTS.end(); ++outit){
          if(outit->empty()) continue; // Skip empty set
          for(auto fit = fs.begin(); fit != fs.end(); ++fit){
            ss.insert(fsinit(*fit,p,i,IN,*outit));
          }
        }
      }
    }
  }
  return ss;
};

template<class T,class L>
std::set<std::set<T,L> > FenceSync::powerset(const std::set<T,L> &s){
  std::vector<std::set<T,L> > v;
  v.push_back(std::set<T,L>());
  int cur = 1;

  for(auto it = s.begin(); it != s.end(); ++it){
    for(int i = 0; i < cur; ++i){
      std::set<T,L> subs = v[i];
      subs.insert(*it);
      v.push_back(subs);
    }
    cur *= 2;
  }

  return std::set<std::set<T,L> >(v.begin(),v.end());
};

void FenceSync::InsInfo::bind(const Machine::PTransition &a,const Machine::PTransition &b){
  auto res = tchanges.insert(std::pair<Machine::PTransition,Machine::PTransition>(a,b));
  if(!res.second){
    tchanges.at(a) = b;
  }
};

const Machine::PTransition &FenceSync::InsInfo::operator[](const Machine::PTransition &t) const{
  return tchanges.at(t);
};

Machine::PTransition FenceSync::InsInfo::all_tchanges(m_infos_t ivec,
                                                      const Machine::PTransition &t,
                                                      int first){
  assert(0 <= first);
  Machine::PTransition t2 = t;
  for(unsigned i = first; i < ivec.size(); ++i){
    if(dynamic_cast<const InsInfo*>(ivec[i])){
      const InsInfo *ii = static_cast<const InsInfo*>(ivec[i]);
      t2 = ii->tchanges.at(t2);
    }else if(dynamic_cast<const TsoLockSync::InsInfo*>(ivec[i])){
      const TsoLockSync::InsInfo *ii = static_cast<const TsoLockSync::InsInfo*>(ivec[i]);
      t2 = ii->tchanges.at(t2);
    }else{
      assert(dynamic_cast<const VipsSyncwrSync::InsInfo*>(ivec[i]));
      const VipsSyncwrSync::InsInfo *ii = static_cast<const VipsSyncwrSync::InsInfo*>(ivec[i]);
      t2 = ii->tchanges.at(t2);
    }
  }
  return t2;
};

int FenceSync::InsInfo::original_q(m_infos_t ivec, int q){
  for(int i = (int)ivec.size() - 1; i >= 0; --i){
    if(dynamic_cast<const InsInfo*>(ivec[i])){
      const InsInfo *info = static_cast<const InsInfo*>(ivec[i]);
      assert(dynamic_cast<const FenceSync*>(info->sync));
      const FenceSync *fs = static_cast<const FenceSync*>(info->sync);
      if(info->new_qs.count(q)){
        q = fs->get_q();
      }
    }else{
      assert(dynamic_cast<const TsoLockSync::InsInfo*>(ivec[i]) || 
             dynamic_cast<const VipsSyncwrSync::InsInfo*>(ivec[i]));
    }
  }
  return q;
};

int FenceSync::compare(const Sync &s) const{
  assert(dynamic_cast<const FenceSync*>(&s));
  const FenceSync *fs = static_cast<const FenceSync*>(&s);

  int f_cmp = f.compare(fs->f,false);
  if(f_cmp != 0) return f_cmp;

  if(pid < fs->pid) return -1;
  if(pid > fs->pid) return 1;

  if(q < fs->q) return -1;
  if(q > fs->q) return 1;

  if(IN < fs->IN){
    return -1;
  }else if(IN > fs->IN){
    return 1;
  }else if(OUT < fs->OUT){
    return -1;
  }else if(OUT > fs->OUT){
    return 1;
  }else{
    return 0;
  }
};

void FenceSync::test(){
  /* Test powerset */
  {
    /* Test 1: empty set */
    {
      std::set<std::set<int> > ps = powerset(std::set<int>());
      Test::inner_test("Powerset #1",
                       ps.size() == 1 &&
                       ps.begin()->size() == 0);
    }

    /* Test 2: singleton set */
    {
      std::set<int> s;
      s.insert(1);
      std::set<std::set<int> > tgt;
      {
        std::set<int> ss;
        tgt.insert(ss);
        ss.insert(1);
        tgt.insert(ss);
      }
      std::set<std::set<int> > ps = powerset(s);
      Test::inner_test("Powerset #2",ps == tgt);
    }

    /* Test 3: larger set */
    {
      std::set<int> s;
      s.insert(1);
      s.insert(2);
      s.insert(3);
      std::set<std::set<int> > tgt;
      {
        std::set<int> ss1, ss2, ss3;
        tgt.insert(ss1); // {}
        ss1.insert(1);
        tgt.insert(ss1); // {1}
        ss2.insert(2);
        tgt.insert(ss2); // {2}
        ss3.insert(3);
        tgt.insert(ss3); // {3}
        ss1.insert(2);
        tgt.insert(ss1); // {1,2}
        ss2.insert(3);
        tgt.insert(ss2); // {2,3}
        ss3.insert(1);
        tgt.insert(ss3); // {1,3}
        ss1.insert(3);
        tgt.insert(ss1); // {1,2,3}
      }
      std::set<std::set<int> > ps = powerset(s);
      Test::inner_test("Powerset #3",ps == tgt);
    }
  }

  std::function<Machine*(std::string)> get_machine = 
    [](std::string rmm){
    std::stringstream ss(rmm);
    PPLexer pp(ss);
    return new Machine(Parser::p_test(pp));
  };

  class StmtCmp{
  public:
    bool operator()(const Lang::Stmt<int> &a, const Lang::Stmt<int> &b) const{
      return a.compare(b,false) < 0;
    }
  };
  /* Dummy concrete implementation of FenceSync. */
  class Dummy : public FenceSync{
  public:
    Dummy(Lang::Stmt<int> f, int pid, int q, 
          TSet IN, TSet OUT)
      : FenceSync(f,pid,q,IN,OUT) {};
    ~Dummy() {};
    virtual FenceSync *clone() const{
      return new Dummy(f,pid,q,IN,OUT);
    };
    bool operator==(const Dummy &d) const{
      return f == d.f && pid == d.pid && q == d.q &&
        IN == d.IN && OUT == d.OUT;
    };
    bool operator<(const Dummy &d) const{
      return 
        (f < d.f) ||
        (f == d.f && pid < d.pid) ||
        (f == d.f && pid == d.pid && q < d.q) ||
        (f == d.f && pid == d.pid && q == d.q && IN < d.IN) ||
        (f == d.f && pid == d.pid && q == d.q && IN == d.IN && OUT < d.OUT);
    };
    static Dummy parse_dummy(const Machine *m, std::string s){
      int pid;
      {
        std::stringstream ss(s);
        ss >> pid;
        s = s.substr(s.find('{')); // Skip until after pid
      }
      std::set<Lang::Stmt<int>,StmtCmp> IN_stmts, OUT_stmts;
      {
        std::string pre = 
          "forbidden *\n"
          "data\n"
          "  fnc = *\n"
          "  x = *\n"
          "  y = *\n"
          "process\n"
          "registers\n"
          "  $r0 = *\n"
          "  $r1 = *\n"
          "  $r2 = *\n"
          "text\n";
        std::string IN_s, OUT_s;
        int i = s.find("to");
        IN_s = s.substr(0,i);
        OUT_s = s.substr(i+2);
        {
          std::stringstream ss(pre+IN_s);
          PPLexer lex(ss);
          Machine m(Parser::p_test(lex));
          for(unsigned j = 0; j < m.automata[0].get_states().size(); ++j){
            const std::set<Automaton::Transition*> &s = 
              m.automata[0].get_states()[j].fwd_transitions;
            for(auto it = s.begin(); it != s.end(); ++it){
              IN_stmts.insert((*it)->instruction);
            }
          }
        }
        {
          std::stringstream ss(pre+OUT_s);
          PPLexer lex(ss);
          Machine m(Parser::p_test(lex));
          for(unsigned j = 0; j < m.automata[0].get_states().size(); ++j){
            const std::set<Automaton::Transition*> &s = 
              m.automata[0].get_states()[j].fwd_transitions;
            for(auto it = s.begin(); it != s.end(); ++it){
              OUT_stmts.insert((*it)->instruction);
            }
          }
        }
      }
      int q = -1;
      {
        /* Find the right control location */
        const std::vector<Automaton::State> &states = m->automata[pid].get_states();
        for(unsigned i = 0; i < states.size(); ++i){
          if(std::any_of(states[i].fwd_transitions.begin(),states[i].fwd_transitions.end(),
                         [&OUT_stmts](const Automaton::Transition *pt){
                           return OUT_stmts.count(pt->instruction) > 0;
                         })){
            q = i;
            break;
          }
        }
      }

      assert(q >= 0);

      TSet IN, OUT;
      {
        for(auto it = m->automata[pid].get_states()[q].bwd_transitions.begin();
            it != m->automata[pid].get_states()[q].bwd_transitions.end(); ++it){
          if(IN_stmts.count((*it)->instruction)){
            IN.insert(**it);
          }
        }
        for(auto it = m->automata[pid].get_states()[q].fwd_transitions.begin();
            it != m->automata[pid].get_states()[q].fwd_transitions.end(); ++it){
          if(OUT_stmts.count((*it)->instruction)){
            OUT.insert(**it);
          }
        }
      }
      Lang::Stmt<int> fence = Lang::Stmt<int>::locked_write(Lang::MemLoc<int>::global(0),Lang::Expr<int>::integer(0));
      return Dummy(fence,pid,q,IN,OUT);
    };
  };

  /* Test get_all_possible */
  {

    std::set<Lang::Stmt<int> > fs0;
    /* Under TSO, a locked write to the dummy memory location
     * (global(0)) acts as a total fence. */
    fs0.insert(Lang::Stmt<int>::locked_write(Lang::MemLoc<int>::global(0),
                                             Lang::Expr<int>::integer(0)));

    fs_init_t fsinit = 
      [](Lang::Stmt<int> f, int pid, int q, 
         TSet IN, TSet OUT){ return new Dummy(f,pid,q,IN,OUT); };

    /* tgt_s specifies one Dummy per line. The format is
     * P{a;b;c;...}to{A;B;C;...}
     * where P is a natural number; the pid of the Dummy
     * a, b, c, ... are the statements of IN
     * A, B, C, ... are the statements of OUT
     *
     * All statements in m should be unique and may reference global
     * memory locations fnc, x, y, and registers $r0, $r1, $r2.
     */
    std::function<bool(const Machine*,const std::set<Sync*>&,std::string)> tst = 
      [](const Machine *m,const std::set<Sync*> &syncset,std::string tgt_s){
      /* Parse tgt_s, create target Dummys */
      std::set<Dummy> dummys;
      {
        std::size_t lnbegin = 0;
        std::size_t lnend;
        while(lnbegin < tgt_s.size()){
          lnend = tgt_s.find('\n',lnbegin);

          if(lnend == std::string::npos){
            dummys.insert(Dummy::parse_dummy(m,tgt_s.substr(lnbegin)));
          }else{
            dummys.insert(Dummy::parse_dummy(m,tgt_s.substr(lnbegin,lnend - lnbegin)));
          }

          lnbegin = (lnend == std::string::npos) ? tgt_s.size() : lnend+1;
        }
      }

      /* Perform the actual test */
      return syncset.size() == dummys.size() &&
      std::all_of(syncset.begin(),syncset.end(),
                  [&dummys](const Sync *sync){
                    const Dummy *d = static_cast<const Dummy*>(sync);
                    return 
                    std::any_of(dummys.begin(),dummys.end(),
                                [d](const Dummy &d2){
                                  return
                                    d->pid == d2.pid &&
                                    d->q == d2.q &&
                                    d->IN == d2.IN &&
                                    d->OUT == d2.OUT;
                                });
                  });
    };

    /* Test 1: Minimal */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "text\n"
         "  nop"
         );
      std::set<Sync*> s = get_all_possible(*m,fs0,fsinit);
      Test::inner_test("get_all #1",s.empty());

      delete m;
    }

    /* Test 2: Small */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = * : [0:1]"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1"
         );

      std::set<Sync*> s = get_all_possible(*m,fs0,fsinit);
      Test::inner_test("get_all #2",tst(m,s,"0{$r0 := 0}to{$r0 := 1}"));

      for(auto it = s.begin(); it != s.end(); ++it){
        delete *it;
      }

      delete m;
    }

    /* Test 3: Small 2 proc */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = * : [0:1]"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1\n"
         "process\n"
         "registers\n"
         "  $r0 = * : [0:1]"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1"
         );

      std::set<Sync*> s = get_all_possible(*m,fs0,fsinit);
      Test::inner_test("get_all #3",tst(m,s,
                                        "0{$r0 := 0}to{$r0 := 1}\n"
                                        "1{$r0 := 0}to{$r0 := 1}"));

      for(auto it = s.begin(); it != s.end(); ++it){
        delete *it;
      }

      delete m;
    }

    /* Test 4: Multiple in/out transitions */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = *"
         "text\n"
         "  L0:\n"
         "  $r0 := 0;\n"
         "  either{\n"
         "    $r0 := 1\n"
         "  or\n"
         "    $r0 := 2\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 3\n"
         "  or\n"
         "    $r0 := 4\n"
         "  };\n"
         "  $r0 := 5;"
         "  goto L0"
         );

      /*     .<-.
       *     0   \
       *     .    .
       *    1 2   |
       *     .    |
       *    3 4   |
       *     .    ,
       *     5   /
       *     .--'
       */

      std::set<Sync*> s = get_all_possible(*m,fs0,fsinit);
      Test::inner_test("get_all #4",tst(m,s,
                                        "0{$r0:=0}to{$r0:=1}\n"
                                        "0{$r0:=0}to{$r0:=2}\n"
                                        "0{$r0:=0}to{$r0:=1;$r0:=2}\n"
                                        
                                        /* Disallowed since neither IN nor OUT is complete:
                                         * "0{$r0:=1}to{$r0:=3}\n"
                                         * "0{$r0:=1}to{$r0:=4}\n"
                                         * "0{$r0:=2}to{$r0:=4}\n"
                                         * "0{$r0:=2}to{$r0:=3}\n"
                                         */
                                        "0{$r0:=1}to{$r0:=3;$r0:=4}\n"
                                        "0{$r0:=2}to{$r0:=3;$r0:=4}\n"
                                        "0{$r0:=1;$r0:=2}to{$r0:=3}\n"
                                        "0{$r0:=1;$r0:=2}to{$r0:=4}\n"
                                        "0{$r0:=1;$r0:=2}to{$r0:=3;$r0:=4}\n"

                                        "0{$r0:=3}to{$r0:=5}\n"
                                        "0{$r0:=4}to{$r0:=5}\n"
                                        "0{$r0:=3;$r0:=4}to{$r0:=5}\n"

                                        "0{$r0:=5}to{$r0:=0}"
                                        ));

      for(auto it = s.begin(); it != s.end(); ++it){
        delete *it;
      }

      delete m;
    }
  }

  /* Test insert */
  {
    /* Test 1: Simple */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 1"
         );

      Dummy d = Dummy::parse_dummy(m,"0{$r0 := 0}to{$r0 := 1}");
      Sync::InsInfo *info;
      Machine *m2 = d.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #1",m2->automata[1].same_automaton(m2->automata[0],false));

      delete info;
      delete m2;
      delete m;
    }

    /* Test 2: Simple */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  $r0 := 2"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 1;\n"
         "  $r0 := 2"
         );

      Dummy d = Dummy::parse_dummy(m,"0{$r0 := 0}to{$r0 := 1}");
      Sync::InsInfo *info;
      Machine *m2 = d.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #2",m2->automata[1].same_automaton(m2->automata[0],false));

      delete info;
      delete m2;
      delete m;
    }

    /* Test 3: Multiple transitions */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0\n"
         "  or\n"
         "    $r0 := 1\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 2\n"
         "  or\n"
         "    $r0 := 3\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0\n"
         "  or\n"
         "    $r0 := 1\n"
         "  };\n"
         "  locked write: fnc := 0;\n"
         "  either{\n"
         "    $r0 := 2\n"
         "  or\n"
         "    $r0 := 3\n"
         "  }\n"
         );

      Dummy d = Dummy::parse_dummy(m,"0{$r0 := 0; $r0 := 1}to{$r0 := 2; $r0 := 3}");
      Sync::InsInfo *info;
      Machine *m2 = d.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #3",m2->automata[1].same_automaton(m2->automata[0],false));

      delete info;
      delete m2;
      delete m;
    }

    /* Test 4: Multiple transitions */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0\n"
         "  or\n"
         "    $r0 := 1\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 2\n"
         "  or\n"
         "    $r0 := 3\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    locked write: fnc := 0\n"
         "  or\n"
         "    $r0 := 1\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 2\n"
         "  or\n"
         "    $r0 := 3\n"
         "  }\n"
         );

      Dummy d = Dummy::parse_dummy(m,"0{$r0 := 0}to{$r0 := 2; $r0 := 3}");
      Sync::InsInfo *info;
      Machine *m2 = d.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #4",m2->automata[1].same_automaton(m2->automata[0],false));

      delete info;
      delete m2;
      delete m;
    }

    /* Test 5,6: Multiple transitions */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0\n"
         "  or\n"
         "    $r0 := 1\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 2\n"
         "  or\n"
         "    $r0 := 3\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0\n"
         "  or\n"
         "    $r0 := 1\n"
         "  };\n"
         "  either{\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 2\n"
         "  or\n"
         "    $r0 := 3\n"
         "  }\n"
         );

      Dummy d = Dummy::parse_dummy(m,"0{$r0 := 0; $r0 := 1}to{$r0 := 2}");
      Sync::InsInfo *info;
      Machine *m2 = d.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #5",m2->automata[1].same_automaton(m2->automata[0],false));

      InsInfo *finfo = static_cast<InsInfo*>(info);
      std::function<bool(const std::pair<Machine::PTransition,Machine::PTransition>&)> iiall =
        [m,m2](const std::pair<Machine::PTransition,Machine::PTransition> &pr){
            Lang::Stmt<int> instr = pr.first.instruction;
            if(pr.first.instruction.compare(pr.second.instruction,false) != 0){
              return false;
            }
            if(pr.first.pid != pr.second.pid){
              return false;
            }
            int pid = pr.first.pid;
            int src0 = pr.first.source;
            int src1 = pr.second.source;
            Automaton::Transition t0 = pr.first;
            Automaton::Transition t1 = pr.second;
            const std::set<Automaton::Transition*> fwd0 = 
              m->automata[pid].get_states()[src0].fwd_transitions;
            const std::set<Automaton::Transition*> fwd1 = 
              m2->automata[pid].get_states()[src1].fwd_transitions;
            if(!std::any_of(fwd0.begin(),fwd0.end(),
                            [&t0](const Automaton::Transition *t){
                              return t->compare(t0,false) == 0;
                            })){
              return false;
            }
            if(!std::any_of(fwd1.begin(),fwd1.end(),
                            [&t1](const Automaton::Transition *t){
                              return t->compare(t1,false) == 0;
                            })){
              return false;
            }
            return true;
          };

      std::function<bool(const std::pair<Machine::PTransition,Machine::PTransition>&)> iiany =
        [m,m2](const std::pair<Machine::PTransition,Machine::PTransition> &pr){
            return pr.first.source != pr.second.source || pr.first.target != pr.second.target;
          };

      Test::inner_test("insert #6 (InsInfo)",
                       std::all_of(finfo->tchanges.begin(),finfo->tchanges.end(),iiall) &&
                       std::any_of(finfo->tchanges.begin(),finfo->tchanges.end(),iiany));

      delete info;
      delete m2;
      delete m;
    }

    /* Test 7: Multiple inserts */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  $r0 := 2;\n"
         "  $r0 := 3;\n"
         "  $r0 := 4\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 2;\n"
         "  $r0 := 3;\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 4"
         );

      Dummy d0 = Dummy::parse_dummy(m,"0{$r0:=1}to{$r0:=2}");
      Dummy d1 = Dummy::parse_dummy(m,"0{$r0:=3}to{$r0:=4}");

      InsInfo *d0info, *d1info;
      Machine *md0 = d0.insert(*m,std::vector<const Sync::InsInfo*>(),(Sync::InsInfo**)&d0info);
      Machine *md1 = d1.insert(*m,std::vector<const Sync::InsInfo*>(),(Sync::InsInfo**)&d1info);
      std::vector<const Sync::InsInfo*> d0ivec(1,d0info), d1ivec(1,d1info);

      Machine *md01 = d1.insert(*md0,d0ivec,(Sync::InsInfo**)&d1info);
      Machine *md10 = d0.insert(*md1,d1ivec,(Sync::InsInfo**)&d0info);

      Test::inner_test("insert #7 (multiple inserts)",
                       /* Both orders of insertion yields identical automata */
                       md01->automata[0].same_automaton(md10->automata[0],false) &&
                       /* It's the correct automaton */
                       m->automata[1].same_automaton(md01->automata[0],false));

      delete d0info;
      delete d1info;
      for(unsigned i = 0; i < d0ivec.size(); ++i){
        delete d0ivec[i];
      }
      for(unsigned i = 0; i < d1ivec.size(); ++i){
        delete d1ivec[i];
      }
      delete md01;
      delete md10;
      delete md0;
      delete md1;
      delete m;
    }

    /* Test 8: Multiple inserts */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  $r0 := 2\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 1;\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 2\n"
         );

      Dummy d0 = Dummy::parse_dummy(m,"0{$r0:=0}to{$r0:=1}");
      Dummy d1 = Dummy::parse_dummy(m,"0{$r0:=1}to{$r0:=2}");

      InsInfo *d0info, *d1info;
      Machine *md0 = d0.insert(*m,std::vector<const Sync::InsInfo*>(),(Sync::InsInfo**)&d0info);
      Machine *md1 = d1.insert(*m,std::vector<const Sync::InsInfo*>(),(Sync::InsInfo**)&d1info);
      std::vector<const Sync::InsInfo*> d0ivec(1,d0info), d1ivec(1,d1info);

      Machine *md01 = d1.insert(*md0,d0ivec,(Sync::InsInfo**)&d1info);
      Machine *md10 = d0.insert(*md1,d1ivec,(Sync::InsInfo**)&d0info);

      Test::inner_test("insert #8 (multiple inserts)",
                       /* Both orders of insertion yields identical automata */
                       md01->automata[0].same_automaton(md10->automata[0],false) &&
                       /* It's the correct automaton */
                       m->automata[1].same_automaton(md01->automata[0],false));

      delete d0info;
      delete d1info;
      for(unsigned i = 0; i < d0ivec.size(); ++i){
        delete d0ivec[i];
      }
      for(unsigned i = 0; i < d1ivec.size(); ++i){
        delete d1ivec[i];
      }
      delete md01;
      delete md10;
      delete md0;
      delete md1;
      delete m;
    }

    /* Test 9: Multiple inserts */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  L2: $r0 := 2;\n"
         "  goto L2\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 1;\n"
         "  locked write: fnc := 0;\n"
         "  L2: $r0 := 2;\n"
         "  goto L2\n"
         );

      Dummy d0 = Dummy::parse_dummy(m,"0{$r0:=0}to{$r0:=1}");
      Dummy d1 = Dummy::parse_dummy(m,"0{$r0:=1}to{$r0:=2}");

      InsInfo *d0info, *d1info;
      Machine *md0 = d0.insert(*m,std::vector<const Sync::InsInfo*>(),(Sync::InsInfo**)&d0info);
      Machine *md1 = d1.insert(*m,std::vector<const Sync::InsInfo*>(),(Sync::InsInfo**)&d1info);
      std::vector<const Sync::InsInfo*> d0ivec(1,d0info), d1ivec(1,d1info);

      Machine *md01 = d1.insert(*md0,d0ivec,(Sync::InsInfo**)&d1info);
      Machine *md10 = d0.insert(*md1,d1ivec,(Sync::InsInfo**)&d0info);

      Test::inner_test("insert #9 (multiple inserts)",
                       /* Both orders of insertion yields identical automata */
                       md01->automata[0].same_automaton(md10->automata[0],false) &&
                       /* It's the correct automaton */
                       m->automata[1].same_automaton(md01->automata[0],false));

      delete d0info;
      delete d1info;
      for(unsigned i = 0; i < d0ivec.size(); ++i){
        delete d0ivec[i];
      }
      for(unsigned i = 0; i < d1ivec.size(); ++i){
        delete d1ivec[i];
      }
      delete md01;
      delete md10;
      delete md0;
      delete md1;
      delete m;
    }

    /* Test 10: Multiple inserts */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 1;\n"
         "    $r0 := 3\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 4\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 5;\n"
         "    $r0 := 7\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    either{\n"
         "      $r0 := 8\n"
         "    or\n"
         "      $r0 := 9\n"
         "    }\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 1;\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 3\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 4\n"
         "  };\n"
         "  locked write: fnc := 0;\n"
         "  either{\n"
         "    $r0 := 5;\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 7\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    either{\n"
         "      $r0 := 8\n"
         "    or\n"
         "      locked write: fnc := 0;\n"
         "      $r0 := 9\n"
         "    }\n"
         "  }\n"
         );

      /*      .
       *     / \
       *    1   2
       * -> .   . <-
       *    3   4
       *     \ /
       *      .   <-
       *     / \
       *    5   6
       * -> .    \
       *    |     .
       *    |    / \ <-
       *    7   8   9
       *    |   |  /
       *     \ /  /
       *      Y  /
       *      | /
       *      |/
       *      .
       */

      Dummy d13 = Dummy::parse_dummy(m,"0{$r0 := 1}to{$r0 := 3}");
      Dummy d24 = Dummy::parse_dummy(m,"0{$r0 := 2}to{$r0 := 4}");
      Dummy d3456 = Dummy::parse_dummy(m,"0{$r0 := 3; $r0 := 4}to{$r0 := 5; $r0 := 6}");
      Dummy d57 = Dummy::parse_dummy(m,"0{$r0 := 5}to{$r0 := 7}");
      Dummy d69 = Dummy::parse_dummy(m,"0{$r0 := 6}to{$r0 := 9}");

      /* ma# order: d13, d24, d57, d69, d3456 */
      /* mb# order: d3456, d13, d24, d57, d69 */
      std::vector<const Sync::InsInfo*> a_infos, b_infos;
      Sync::InsInfo *info;
      Machine *ma1 = d13.insert(*m,a_infos,&info); a_infos.push_back(info);
      Machine *ma2 = d24.insert(*ma1,a_infos,&info); a_infos.push_back(info);
      Machine *ma3 = d57.insert(*ma2,a_infos,&info); a_infos.push_back(info);
      Machine *ma4 = d69.insert(*ma3,a_infos,&info); a_infos.push_back(info);
      Machine *ma5 = d3456.insert(*ma4,a_infos,&info); a_infos.push_back(info);

      Machine *mb1 = d3456.insert(*m,b_infos,&info); b_infos.push_back(info);
      Machine *mb2 = d13.insert(*mb1,b_infos,&info); b_infos.push_back(info);
      Machine *mb3 = d24.insert(*mb2,b_infos,&info); b_infos.push_back(info);
      Machine *mb4 = d57.insert(*mb3,b_infos,&info); b_infos.push_back(info);
      Machine *mb5 = d69.insert(*mb4,b_infos,&info); b_infos.push_back(info);

      Test::inner_test("insert #10 (multiple inserts)",
                       m->automata[1].same_automaton(ma5->automata[0],false) && 
                       m->automata[1].same_automaton(mb5->automata[0],false));

      for(unsigned i = 0; i < a_infos.size(); ++i) delete a_infos[i];
      for(unsigned i = 0; i < b_infos.size(); ++i) delete b_infos[i];
      delete ma1;
      delete ma2;
      delete ma3;
      delete ma4;
      delete ma5;
      delete mb1;
      delete mb2;
      delete mb3;
      delete mb4;
      delete mb5;
      delete m;
    }

    /* Test 11: Multiple inserts */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 1;\n"
         "    $r0 := 3\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 4\n"
         "  or\n"
         "    $r0 := 10;\n"
         "    goto L6\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 5;\n"
         "    $r0 := 7\n"
         "  or\n"
         "    $r0 := 6;\n"
         "  L6:\n"
         "    either{\n"
         "      $r0 := 8\n"
         "    or\n"
         "      $r0 := 9\n"
         "    }\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 1;\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 3\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 4\n"
         "  or\n"
         "    $r0 := 10;\n"
         "    goto L6\n"
         "  };\n"
         "  locked write: fnc := 0;\n"
         "  either{\n"
         "    $r0 := 5;\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 7\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    locked write: fnc := 0;\n"
         "  L6:\n"
         "    either{\n"
         "      $r0 := 8\n"
         "    or\n"
         "      $r0 := 9\n"
         "    }\n"
         "  }\n"
         );

      /*      .----.
       *     / \    \
       *    1   2    \
       * -> .   . <-  .
       *    3   4     |
       *     \ /      10
       *      .   <-  |
       *     / \     /
       *    5   6   /
       * -> . -> \ /
       *    |     .
       *    |    / \
       *    7   8   9
       *    |   |  /
       *     \ /  /
       *      Y  /
       *      | /
       *      |/
       *      .
       */

      Dummy d13 = Dummy::parse_dummy(m,"0{$r0 := 1}to{$r0 := 3}");
      Dummy d24 = Dummy::parse_dummy(m,"0{$r0 := 2}to{$r0 := 4}");
      Dummy d3456 = Dummy::parse_dummy(m,"0{$r0 := 3; $r0 := 4}to{$r0 := 5; $r0 := 6}");
      Dummy d57 = Dummy::parse_dummy(m,"0{$r0 := 5}to{$r0 := 7}");
      Dummy d689 = Dummy::parse_dummy(m,"0{$r0 := 6}to{$r0 := 8; $r0 := 9}");
      // Dummy d6109 = Dummy::parse_dummy(m,"0{$r0 := 6; $r0 := 10}to{$r0 := 9}");

      /* ma# order: d13, d24, d57, d689, d6109, d3456 */
      /* mb# order: d3456, d13, d24, d57, d6109, d689 */
      std::vector<const Sync::InsInfo*> a_infos, b_infos;
      Sync::InsInfo *info;
      Machine *ma1 = d13.insert(*m,a_infos,&info); a_infos.push_back(info);
      Machine *ma2 = d24.insert(*ma1,a_infos,&info); a_infos.push_back(info);
      Machine *ma3 = d57.insert(*ma2,a_infos,&info); a_infos.push_back(info);
      Machine *ma4 = d689.insert(*ma3,a_infos,&info); a_infos.push_back(info);
      Machine *ma5 = d3456.insert(*ma4,a_infos,&info); a_infos.push_back(info);

      Machine *mb1 = d3456.insert(*m,b_infos,&info); b_infos.push_back(info);
      Machine *mb2 = d13.insert(*mb1,b_infos,&info); b_infos.push_back(info);
      Machine *mb3 = d24.insert(*mb2,b_infos,&info); b_infos.push_back(info);
      Machine *mb4 = d57.insert(*mb3,b_infos,&info); b_infos.push_back(info);
      Machine *mb5 = d689.insert(*mb4,b_infos,&info); b_infos.push_back(info);

      Test::inner_test("insert #11 (multiple inserts)",
                       m->automata[1].same_automaton(ma5->automata[0],false) && 
                       m->automata[1].same_automaton(mb5->automata[0],false));

      for(unsigned i = 0; i < a_infos.size(); ++i) delete a_infos[i];
      for(unsigned i = 0; i < b_infos.size(); ++i) delete b_infos[i];
      delete ma1;
      delete ma2;
      delete ma3;
      delete ma4;
      delete ma5;
      delete mb1;
      delete mb2;
      delete mb3;
      delete mb4;
      delete mb5;
      delete m;
    }

    /* Test 12: Insertion at initial state */
    {
      Machine *m = get_machine
        ("forbidden * * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  goto L0\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  goto L0\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  locked write: fnc := 0;\n"
         "  goto L0\n"
         );

      Dummy d = Dummy::parse_dummy(m,"0{$r0:=1}to{$r0:=0}");

      Sync::InsInfo *info;
      Machine *m2 = d.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #12",
                       m->automata[1].same_automaton(m2->automata[0],false) ||
                       m->automata[2].same_automaton(m2->automata[0],false));

      delete info;
      delete m2;
      delete m;
    }

    /* Test 13: Insertion at initial state (one-transition self loop) */
    {
      Machine *m = get_machine
        ("forbidden * * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  $r0 := 0;\n"
         "  goto L0\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 0;\n"
         "  goto L0\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  $r0 := 0;\n"
         "  locked write: fnc := 0;\n"
         "  goto L0\n"
         );

      Dummy d = Dummy::parse_dummy(m,"0{$r0:=0}to{$r0:=0}");

      Sync::InsInfo *info;
      Machine *m2 = d.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #13",
                       m->automata[1].same_automaton(m2->automata[0],false) ||
                       m->automata[2].same_automaton(m2->automata[0],false));

      delete info;
      delete m2;
      delete m;
    }

    /* Test 14,15,16: Multiple insertions to the same control state */
    {
      Machine *m = get_machine
        ("forbidden L0 L0 L0 L0 L0 L0 L0\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "L0:\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    $r0 := 1\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 3\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 4;\n"
         "    $r0 := 5\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    $r0 := 7\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "L0:\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    $r0 := 1\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 3\n"
         "  };\n"
         "  either{\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 4;\n"
         "    $r0 := 5\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    $r0 := 7\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "L0:\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    $r0 := 1\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 3\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 4;\n"
         "    $r0 := 5\n"
         "  or\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 6;\n"
         "    $r0 := 7\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "L0:\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    $r0 := 1\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 3\n"
         "  };\n"
         "  locked write: fnc := 0;\n"
         "  either{\n"
         "    $r0 := 4;\n"
         "    $r0 := 5\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    $r0 := 7\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "L0:\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    $r0 := 1\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 3\n"
         "  };\n"
         "  either{\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 4;\n"
         "    $r0 := 5\n"
         "  or\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 6;\n"
         "    $r0 := 7\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "L0:\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    $r0 := 1;\n"
         "    locked write: fnc := 0\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 3;\n"
         "    locked write: fnc := 0\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 4;\n"
         "    $r0 := 5\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    $r0 := 7\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "L0:\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    $r0 := 1;\n"
         "    locked write: fnc := 0\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 3\n"
         "  };\n"
         "  either{\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 4;\n"
         "    $r0 := 5\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    $r0 := 7\n"
         "  }\n"
         );

      Dummy d0 = Dummy::parse_dummy(m,"0{$r0:=1;$r0:=3}to{$r0:=4}");
      Dummy d1 = Dummy::parse_dummy(m,"0{$r0:=1;$r0:=3}to{$r0:=6}");
      Dummy d2 = Dummy::parse_dummy(m,"0{$r0:=1;$r0:=3}to{$r0:=4;$r0:=6}");
      Dummy d3 = Dummy::parse_dummy(m,"0{$r0:=1}to{$r0:=4;$r0:=6}");
      Dummy d4 = Dummy::parse_dummy(m,"0{$r0:=3}to{$r0:=4;$r0:=6}");
      
      Sync::InsInfo *info;
      std::vector<const Sync::InsInfo*> m_infos_01, m_infos_10;
      Machine *m0 = d0.insert(*m,m_infos_01,&info); m_infos_01.push_back(info);
      Machine *m1 = d1.insert(*m,m_infos_10,&info); m_infos_10.push_back(info);
      Machine *m2 = d2.insert(*m,std::vector<const Sync::InsInfo*>(),&info); delete info;
      Machine *m01 = d1.insert(*m0,m_infos_01,&info); delete info;
      Machine *m10 = d0.insert(*m1,m_infos_10,&info); delete info;

      Test::inner_test("insert #14 (multiple on same control state)",
                       m->automata[1].same_automaton(m0->automata[0],false) &&
                       m->automata[2].same_automaton(m1->automata[0],false) &&
                       m->automata[3].same_automaton(m2->automata[0],false) &&
                       m->automata[4].same_automaton(m01->automata[0],false) &&
                       m->automata[4].same_automaton(m10->automata[0],false));

      std::vector<const Sync::InsInfo*> m_infos_34, m_infos_43;
      Machine *m3 = d3.insert(*m,m_infos_34,&info); m_infos_34.push_back(info);
      Machine *m4 = d4.insert(*m,m_infos_43,&info); m_infos_43.push_back(info);
      Machine *m34 = d4.insert(*m3,m_infos_34,&info); delete info;
      Machine *m43 = d3.insert(*m4,m_infos_43,&info); delete info;

      Test::inner_test("insert #15 (multiple on same control state)",
                       m->automata[5].same_automaton(m34->automata[0],false) &&
                       m->automata[5].same_automaton(m43->automata[0],false));

      std::vector<const Sync::InsInfo*> m_infos_03, m_infos_30;
      Machine *m03 = d3.insert(*m0,m_infos_01,&info); delete info;
      Machine *m30 = d0.insert(*m3,m_infos_34,&info); delete info;

      Test::inner_test("insert #16 (multiple on same control state)",
                       m->automata[6].same_automaton(m03->automata[0],false) &&
                       m->automata[6].same_automaton(m30->automata[0],false));

      for(unsigned i = 0; i < m_infos_01.size(); ++i){
        delete m_infos_01[i];
      }
      for(unsigned i = 0; i < m_infos_10.size(); ++i){
        delete m_infos_10[i];
      }
      for(unsigned i = 0; i < m_infos_34.size(); ++i){
        delete m_infos_34[i];
      }
      for(unsigned i = 0; i < m_infos_43.size(); ++i){
        delete m_infos_43[i];
      }
      for(unsigned i = 0; i < m_infos_03.size(); ++i){
        delete m_infos_03[i];
      }
      for(unsigned i = 0; i < m_infos_30.size(); ++i){
        delete m_infos_30[i];
      }
      delete m0;
      delete m1;
      delete m2;
      delete m01;
      delete m10;
      delete m3;
      delete m4;
      delete m34;
      delete m43;
      delete m03;
      delete m30;
      delete m;
    }

    /* Test 17,18,19: Deeper tree structure */
    {
      Machine *m = get_machine
        ("forbidden L0 L0 L0 L0\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  either{\n"
         "    $r0:=0;\n"
         "    $r0:=1\n"
         "  or\n"
         "    $r0:=2;\n"
         "    $r0:=3\n"
         "  or\n"
         "    $r0:=4;\n"
         "    $r0:=5\n"
         "  or\n"
         "    $r0:=6;\n"
         "    $r0:=7\n"
         "  };\n"
         "  either{\n"
         "    $r0:=8;\n"
         "    $r0:=9\n"
         "  or\n"
         "    $r0:=10;\n"
         "    $r0:=11\n"
         "  or\n"
         "    $r0:=12;\n"
         "    $r0:=13\n"
         "  or\n"
         "    $r0:=14;\n"
         "    $r0:=15\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  either{\n"
         "    either{\n"
         "      either{\n"
         "        $r0:=0;\n"
         "        $r0:=1;\n"
         "        locked write: fnc := 0\n"
         "      or\n"
         "        $r0:=2;\n"
         "        $r0:=3;\n"
         "        locked write: fnc := 0\n"
         "      };\n"
         "      locked write: fnc := 0\n"
         "    or\n"
         "      $r0:=4;\n"
         "      $r0:=5\n"
         "    };\n"
         "    locked write: fnc := 0\n"
         "  or\n"
         "    $r0:=6;\n"
         "    $r0:=7\n"
         "  };"
         "  either{\n"
         "    $r0:=8;\n"
         "    $r0:=9\n"
         "  or\n"
         "    $r0:=10;\n"
         "    $r0:=11\n"
         "  or\n"
         "    $r0:=12;\n"
         "    $r0:=13\n"
         "  or\n"
         "    $r0:=14;\n"
         "    $r0:=15\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  either{\n"
         "    $r0:=0;\n"
         "    $r0:=1\n"
         "  or\n"
         "    $r0:=2;\n"
         "    $r0:=3\n"
         "  or\n"
         "    $r0:=4;\n"
         "    $r0:=5\n"
         "  or\n"
         "    $r0:=6;\n"
         "    $r0:=7\n"
         "  };\n"
         "  either{\n"
         "    locked write: fnc := 0;\n"
         "    either{\n"
         "      locked write: fnc := 0;\n"
         "      either{\n"
         "        locked write: fnc := 0;\n"
         "        $r0:=8;\n"
         "        $r0:=9\n"
         "      or\n"
         "        locked write: fnc := 0;\n"
         "        $r0:=10;\n"
         "        $r0:=11\n"
         "      }\n"
         "    or\n"
         "      $r0:=12;\n"
         "      $r0:=13\n"
         "    }\n"
         "  or\n"
         "    $r0:=14;\n"
         "    $r0:=15\n"
         "  }"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  either{\n"
         "    either{\n"
         "      either{\n"
         "        $r0:=0;\n"
         "        $r0:=1;\n"
         "        locked write: fnc := 0\n"
         "      or\n"
         "        $r0:=2;\n"
         "        $r0:=3;\n"
         "        locked write: fnc := 0\n"
         "      };\n"
         "      locked write: fnc := 0\n"
         "    or\n"
         "      $r0:=4;\n"
         "      $r0:=5\n"
         "    };\n"
         "    locked write: fnc := 0\n"
         "  or\n"
         "    $r0:=6;\n"
         "    $r0:=7\n"
         "  };"
         "  either{\n"
         "    locked write: fnc := 0;\n"
         "    either{\n"
         "      locked write: fnc := 0;\n"
         "      either{\n"
         "        locked write: fnc := 0;\n"
         "        $r0:=8;\n"
         "        $r0:=9\n"
         "      or\n"
         "        locked write: fnc := 0;\n"
         "        $r0:=10;\n"
         "        $r0:=11\n"
         "      }\n"
         "    or\n"
         "      $r0:=12;\n"
         "      $r0:=13\n"
         "    }\n"
         "  or\n"
         "    $r0:=14;\n"
         "    $r0:=15\n"
         "  }"
         );
      Dummy d0 = Dummy::parse_dummy(m,"0{$r0:=1;$r0:=3;$r0:=5}to{$r0:=8;$r0:=10;$r0:=12;$r0:=14}");
      Dummy d1 = Dummy::parse_dummy(m,"0{$r0:=1;$r0:=3}to{$r0:=8;$r0:=10;$r0:=12;$r0:=14}");
      Dummy d2 = Dummy::parse_dummy(m,"0{$r0:=1}to{$r0:=8;$r0:=10;$r0:=12;$r0:=14}");
      Dummy d3 = Dummy::parse_dummy(m,"0{$r0:=3}to{$r0:=8;$r0:=10;$r0:=12;$r0:=14}");

      std::vector<const Sync::InsInfo*> m_infos;
      Sync::InsInfo *info;
      Machine *m2 = d0.insert(*m,m_infos,&info); m_infos.push_back(info);
      Machine *m3 = d1.insert(*m2,m_infos,&info); m_infos.push_back(info);
      Machine *m4 = d2.insert(*m3,m_infos,&info); m_infos.push_back(info);
      Machine *m5 = d3.insert(*m4,m_infos,&info); m_infos.push_back(info);

      Test::inner_test("insert #17 (deep tree structure in)",
                       m->automata[1].same_automaton(m5->automata[0],false));

      for(unsigned i = 0; i < m_infos.size(); ++i){
        delete m_infos[i];
      }
      m_infos.clear();
      delete m2;
      delete m3;
      delete m4;
      delete m5;

      Dummy d4 = Dummy::parse_dummy(m,"0{$r0:=1;$r0:=3;$r0:=5;$r0:=7}to{$r0:=8;$r0:=10;$r0:=12}");
      Dummy d5 = Dummy::parse_dummy(m,"0{$r0:=1;$r0:=3;$r0:=5;$r0:=7}to{$r0:=8;$r0:=10}");
      Dummy d6 = Dummy::parse_dummy(m,"0{$r0:=1;$r0:=3;$r0:=5;$r0:=7}to{$r0:=8}");
      Dummy d7 = Dummy::parse_dummy(m,"0{$r0:=1;$r0:=3;$r0:=5;$r0:=7}to{$r0:=10}");

      m2 = d4.insert(*m,m_infos,&info); m_infos.push_back(info);
      m3 = d5.insert(*m2,m_infos,&info); m_infos.push_back(info);
      m4 = d6.insert(*m3,m_infos,&info); m_infos.push_back(info);
      m5 = d7.insert(*m4,m_infos,&info); m_infos.push_back(info);

      Test::inner_test("insert #18 (deep tree structure out)",
                       m->automata[2].same_automaton(m5->automata[0],false));

      Machine *m6 = d0.insert(*m5,m_infos,&info); m_infos.push_back(info);
      Machine *m7 = d1.insert(*m6,m_infos,&info); m_infos.push_back(info);
      Machine *m8 = d2.insert(*m7,m_infos,&info); m_infos.push_back(info);
      Machine *m9 = d3.insert(*m8,m_infos,&info); m_infos.push_back(info);

      Test::inner_test("insert #19 (deep tree structure in & out)",
                       m->automata[3].same_automaton(m9->automata[0],false));

      for(unsigned i = 0; i < m_infos.size(); ++i){
        delete m_infos[i];
      }
      m_infos.clear();
      delete m;
      delete m2;
      delete m3;
      delete m4;
      delete m5;
      delete m6;
      delete m7;
      delete m8;
      delete m9;
      
    }

    /* Test 20: Two fences at the same location */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    $r0 := 1\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 3\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 4;\n"
         "    $r0 := 5\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    $r0 := 7\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    $r0 := 1\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 3\n"
         "  };\n"
         "  locked write: fnc := 0;\n"
         "  locked write: fnc := 0;\n"
         "  either{\n"
         "    $r0 := 4;\n"
         "    $r0 := 5\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    $r0 := 7\n"
         "  }\n"
         );

      Dummy d0 = Dummy::parse_dummy(m,"0{$r0:=1;$r0:=3}to{$r0:=4;$r0:=6}");
      Dummy d1 = Dummy::parse_dummy(m,"0{$r0:=1;$r0:=3}to{$r0:=4;$r0:=6}");

      Sync::InsInfo *info;
      std::vector<const Sync::InsInfo*> m_infos;
      Machine *m1 = d0.insert(*m,m_infos,&info); m_infos.push_back(info);
      Machine *m2 = d1.insert(*m1,m_infos,&info); m_infos.push_back(info);

      Test::inner_test("insert #20",
                       m->automata[1].same_automaton(m2->automata[0],false));

      for(unsigned i = 0; i < m_infos.size(); ++i){
        delete m_infos[i];
      }
      delete m;
      delete m1;
      delete m2;
    }

  }
};
