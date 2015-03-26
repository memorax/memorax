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

#include "parser.h"
#include "preprocessor.h"
#include "vips_fence_sync.h"
#include "test.h"

#include <functional>

VipsFenceSync::VipsFenceSync(Lang::Stmt<int> f, int pid, int q, TSet IN, TSet OUT)
  : FenceSync(f,pid,q,IN,OUT) {
};

VipsFenceSync::~VipsFenceSync(){};

std::set<Sync*> VipsFenceSync::get_all_possible(const Machine &m,bool full_branch_only){
  std::set<Lang::Stmt<int> > fs;
  fs.insert(Lang::Stmt<int>::nop()); // Does not matter since fsinit will provide the right fence instruction
  FenceSync::fs_init_t fsinit_full =
    [](Lang::Stmt<int> f, int pid, int q,TSet IN,TSet OUT){
    return new VipsFullFenceSync(pid,q,IN,OUT);
  };
  FenceSync::fs_init_t fsinit_ss =
    [](Lang::Stmt<int> f, int pid, int q,TSet IN,TSet OUT){
    return new VipsSSFenceSync(pid,q,IN,OUT);
  };
  FenceSync::fs_init_t fsinit_ll =
    [](Lang::Stmt<int> f, int pid, int q,TSet IN,TSet OUT){
    return new VipsLLFenceSync(pid,q,IN,OUT);
  };
  std::set<Sync*> S, S_ss, S_ll;
  S = FenceSync::get_all_possible(m,fs,fsinit_full,full_branch_only);
  S_ss = FenceSync::get_all_possible(m,fs,fsinit_ss,full_branch_only);
  S_ll = FenceSync::get_all_possible(m,fs,fsinit_ll,full_branch_only);
  S.insert(S_ss.begin(),S_ss.end());
  S.insert(S_ll.begin(),S_ll.end());
  return S;
};

Machine *VipsFenceSync::insert(const Machine &m, m_infos_t m_infos, Sync::InsInfo **info) const{
  /* Check that no other fences are inserted to the same location. */
  for(unsigned i = 0; i < m_infos.size(); ++i){
    if(dynamic_cast<const VipsFenceSync*>(m_infos[i]->sync)){
      const VipsFenceSync *vfs = static_cast<const VipsFenceSync*>(m_infos[i]->sync);
      if(vfs->pid == pid && vfs->q == q){
        throw new Incompatible(m_infos[i],
                               "At most one VipsFenceSync may be inserted at each program location.");
      }
    }
  }

  /* OK. Insert as usual. */
  return FenceSync::insert(m,m_infos,info);
};

VipsFullFenceSync::VipsFullFenceSync(int pid, int q, TSet IN, TSet OUT)
  : VipsFenceSync(Lang::Stmt<int>::full_fence(),pid,q,IN,OUT) {
};

VipsFullFenceSync::~VipsFullFenceSync(){};

Sync *VipsFullFenceSync::clone() const{
  return new VipsFullFenceSync(*this);
};

VipsSSFenceSync::VipsSSFenceSync(int pid, int q, TSet IN, TSet OUT)
  : VipsFenceSync(Lang::Stmt<int>::ss_fence(),pid,q,IN,OUT) {
};

VipsSSFenceSync::~VipsSSFenceSync(){};

Sync *VipsSSFenceSync::clone() const{
  return new VipsSSFenceSync(*this);
};

VipsLLFenceSync::VipsLLFenceSync(int pid, int q, TSet IN, TSet OUT)
  : VipsFenceSync(Lang::Stmt<int>::ll_fence(),pid,q,IN,OUT) {
};

VipsLLFenceSync::~VipsLLFenceSync(){};

Sync *VipsLLFenceSync::clone() const{
  return new VipsLLFenceSync(*this);
};

bool VipsFenceSync::strict_subset(const FenceSync::TSet &A, const FenceSync::TSet &B){
  return A.size() < B.size() &&
    std::includes(B.begin(),B.end(),A.begin(),A.end(),FenceSync::TransCmp());
};

bool VipsFenceSync::equal(const FenceSync::TSet &A, const FenceSync::TSet &B){
  if(A.size() != B.size()){
    return false;
  }
  auto it_A = A.begin(), it_B = B.begin();
  while(it_A != A.end()){
    if(it_A->compare(*it_B,false) != 0){
      return false;
    }
    ++it_A;
    ++it_B;
  }
  return true;
};

bool VipsSSFenceSync::stackable(const VipsLLFenceSync *vfs_ll) const{
  if(vfs_ll->get_pid() != get_pid() || vfs_ll->get_q() != get_q()){
    return false;
  }

  return !strict_subset(IN,vfs_ll->get_IN()) && !strict_subset(vfs_ll->get_OUT(),OUT);
};

bool VipsLLFenceSync::stackable(const VipsSSFenceSync *vfs_ss) const{
  return vfs_ss->stackable(this);
};


Machine *VipsFenceSync::insert_double(const Machine &m, m_infos_t m_infos, Sync::InsInfo **info,
                                      const FenceSync::InsInfo *prev_info,
                                      bool insert_before) const{
  assert(prev_info->new_qs.size() == 2);
  assert(prev_info->new_qs.count(q));

  int q3 = m.automata[pid].get_states().size(); // New control state

  /* Setup new Machine and InsInfo. */
  Machine *m2 = new Machine(m);
  FenceSync::InsInfo *my_info = new FenceSync::InsInfo(static_cast<FenceSync*>(clone()));
  *info = my_info;
  my_info->setup_id_tchanges(m);
  my_info->new_qs = prev_info->new_qs;
  my_info->new_qs.insert(q3);

  int q_other = -1; // The control state in new_qs that is not q
  for(int qtmp : prev_info->new_qs){
    if(qtmp != q){
      q_other = qtmp;
      break;
    }
  }
  /* Determine the order of q and q_other in the automaton. */
  Automaton::Transition *prev_trans = 0; // The fence transition of prev_info
  int q1 = -1, q2 = -1; // q1==prev_trans->source, q2==prev_trans->target
  {
    const std::set<Automaton::Transition*> &A =
      m2->automata[pid].get_states()[q].fwd_transitions;
    const std::set<Automaton::Transition*> &B =
      m2->automata[pid].get_states()[q_other].fwd_transitions;
    for(auto it = A.begin(); it != B.end(); (++it == A.end() ? it = B.begin() : it)){
      if((*it)->source == q && (*it)->target == q_other){
        q1 = q;
        q2 = q_other;
        prev_trans = *it;
        break;
      }
      if((*it)->source == q_other && (*it)->target == q){
        q1 = q_other;
        q2 = q;
        prev_trans = *it;
        break;
      }
    }
    assert(prev_trans);
    assert(q1 == q || q1 == q_other);
    assert(q2 == q || q2 == q_other);
    assert(q1 != q2);
  }

  /* Reroute transitions */
  {
    int pq1, pq2, nq1, nq2;
    if(insert_before){
      nq1 = q1;
      nq2 = pq1 = q3;
      pq2 = q2;
    }else{
      pq1 = q1;
      pq2 = nq1 = q3;
      nq2 = q2;
    }
    Automaton::Transition pfence(pq1,prev_trans->instruction,pq2);
    Automaton::Transition nfence(nq1,f,nq2);
    my_info->bind(Machine::PTransition(*prev_trans,pid),
                  Machine::PTransition(pfence,pid));
#ifndef NDEBUG
    bool del_success =
#endif
      m2->automata[pid].del_transition(*prev_trans);
    assert(del_success);
    m2->automata[pid].add_transition(pfence);
    m2->automata[pid].add_transition(nfence);
  }

  return m2;
};

Machine *VipsSSFenceSync::insert(const Machine &m, m_infos_t m_infos, Sync::InsInfo **info) const{
  const VipsLLFenceSync *vfs_ll_collision = 0;
  const FenceSync::InsInfo *info_collision = 0;

  /* Check that no other fences are inserted to the same location. */
  for(unsigned i = 0; i < m_infos.size(); ++i){
    if(dynamic_cast<const VipsFenceSync*>(m_infos[i]->sync)){
      const VipsFenceSync *vfs = static_cast<const VipsFenceSync*>(m_infos[i]->sync);
      const VipsLLFenceSync *vfs_ll = dynamic_cast<const VipsLLFenceSync*>(m_infos[i]->sync);
      if(vfs->get_pid() == pid && vfs->get_q() == q && vfs_ll){
        assert(vfs_ll_collision == 0);
        vfs_ll_collision = vfs_ll;
        assert(dynamic_cast<const FenceSync::InsInfo*>(m_infos[i]));
        info_collision = static_cast<const FenceSync::InsInfo*>(m_infos[i]);
      }
      if(vfs->get_pid() == pid && vfs->get_q() == q &&
         (!vfs_ll || !stackable(vfs_ll))){
        throw new Incompatible(m_infos[i],
                               "At most one VipsFenceSync may be inserted at each program location.");
      }
    }
  }

  if(vfs_ll_collision){
    /* Insert this fence *after* vfs_ll_collision. */
    /* Note that this is weaker than inserting ssfence before
     * llfence. */
    if(equal(IN,vfs_ll_collision->get_IN()) &&
       equal(OUT,vfs_ll_collision->get_OUT())){
      return insert_double(m,m_infos,info,info_collision,false);
    }else{
      /* In this case, FenceSync::insert will insert this fence in the
       * right order with vfs_ll_collision since
       * stackable(vfs_ll_collision). */
      return FenceSync::insert(m,m_infos,info);
    }
  }else{
    /* OK. Insert as usual. */
    return FenceSync::insert(m,m_infos,info);
  }
};

Machine *VipsLLFenceSync::insert(const Machine &m, m_infos_t m_infos, Sync::InsInfo **info) const{
  const VipsSSFenceSync * vfs_ss_collision = 0;
  const FenceSync::InsInfo *info_collision = 0;

  /* Check that no other fences are inserted to the same location. */
  for(unsigned i = 0; i < m_infos.size(); ++i){
    if(dynamic_cast<const VipsFenceSync*>(m_infos[i]->sync)){
      const VipsFenceSync *vfs = static_cast<const VipsFenceSync*>(m_infos[i]->sync);
      const VipsSSFenceSync *vfs_ss = dynamic_cast<const VipsSSFenceSync*>(m_infos[i]->sync);
      if(vfs->get_pid() == pid && vfs->get_q() == q && vfs_ss){
        assert(vfs_ss_collision == 0);
        vfs_ss_collision = vfs_ss;
        assert(dynamic_cast<const FenceSync::InsInfo*>(m_infos[i]));
        info_collision = static_cast<const FenceSync::InsInfo*>(m_infos[i]);
      }
      if(vfs->get_pid() == pid && vfs->get_q() == q &&
         (!vfs_ss || !stackable(vfs_ss))){
        throw new Incompatible(m_infos[i],
                               "At most one VipsFenceSync may be inserted at each program location.");
      }
    }
  }

  if(vfs_ss_collision){
    /* Insert this fence *before* vfs_ss_collision. */
    /* Note that this is weaker than inserting ssfence before
     * llfence. */
    if(equal(IN,vfs_ss_collision->get_IN()) &&
       equal(OUT,vfs_ss_collision->get_OUT())){
      return insert_double(m,m_infos,info,info_collision,true);
    }else{
      /* In this case, FenceSync::insert will insert this fence in the
       * right order with vfs_ss_collision since
       * stackable(vfs_ss_collision). */
      return FenceSync::insert(m,m_infos,info);
    }
  }else{
    /* OK. Insert as usual. */
    return FenceSync::insert(m,m_infos,info);
  }
};

void VipsFenceSync::test(){
  std::function<Machine*(std::string)> get_machine =
    [](std::string rmm){
    std::stringstream ss(rmm);
    PPLexer pp(ss);
    return new Machine(Parser::p_test(pp));
  };

  std::function<Automaton::Transition(const Machine*,int,std::string)> get_trans =
    [&get_machine](const Machine *m,int pid, std::string strans){
    Machine *m_ref = get_machine(R"(
forbidden *
process
registers
  $r0 = *
text
  )"+strans);
    Lang::Stmt<int> stmt = (*m_ref->automata[0].get_states()[0].fwd_transitions.begin())->instruction;
    delete m_ref;
    const std::vector<Automaton::State> &states = m->automata[pid].get_states();
    for(unsigned i = 0; i < states.size(); ++i){
      for(auto t : states[i].fwd_transitions){
        if(t->instruction.compare(stmt,false) == 0){
          return *t;
        }
      }
    }
    assert(0);
    return Automaton::Transition(-1,stmt,-1);
  };

  std::function<FenceSync::TSet(const Machine *,int,std::set<std::string>)> get_tset =
    [&get_trans](const Machine *m, int pid, std::set<std::string> S){
    FenceSync::TSet T;
    for(std::string s : S){
      T.insert(get_trans(m,pid,s));
    }
    return T;
  };

  std::function<VipsSSFenceSync(const Machine*, int,std::set<std::string>,std::set<std::string>)> get_ssfence =
    [&get_tset](const Machine *m,int pid, std::set<std::string> IN, std::set<std::string> OUT){
    FenceSync::TSet TIN = get_tset(m,pid,IN);
    FenceSync::TSet TOUT = get_tset(m,pid,OUT);
    int q = TIN.begin()->target;
    return VipsSSFenceSync(pid,q,TIN,TOUT);
  };

  std::function<VipsLLFenceSync(const Machine*, int,std::set<std::string>,std::set<std::string>)> get_llfence =
    [&get_tset](const Machine *m,int pid, std::set<std::string> IN, std::set<std::string> OUT){
    FenceSync::TSet TIN = get_tset(m,pid,IN);
    FenceSync::TSet TOUT = get_tset(m,pid,OUT);
    int q = TIN.begin()->target;
    return VipsLLFenceSync(pid,q,TIN,TOUT);
  };

  std::function<VipsFullFenceSync(const Machine*,int,std::set<std::string>,std::set<std::string>)> get_fence =
    [&get_tset](const Machine *m,int pid, std::set<std::string> IN, std::set<std::string> OUT){
    FenceSync::TSet TIN = get_tset(m,pid,IN);
    FenceSync::TSet TOUT = get_tset(m,pid,OUT);
    int q = TIN.begin()->target;
    return VipsFullFenceSync(pid,q,TIN,TOUT);
  };

  /* Test insert for ssfence and llfence */
  {
    /* Test 1-4 */
    {
      Machine *m = get_machine(R"(
forbidden L0 L0 L0 L0
process
registers
  $r0 = *
text
  L0: $r0 := 0;
  $r0 := 1
process
registers
  $r0 = *
text
  L0: $r0 := 0;
  ssfence;
  $r0 := 1
process
registers
  $r0 = *
text
  L0: $r0 := 0;
  llfence;
  $r0 := 1
process
registers
  $r0 = *
text
  L0: $r0 := 0;
  llfence;
  ssfence;
  $r0 := 1
)");
      VipsSSFenceSync ssfence = get_ssfence(m,0,{"$r0:=0"},{"$r0:=1"});
      VipsLLFenceSync llfence = get_llfence(m,0,{"$r0:=0"},{"$r0:=1"});

      Sync::InsInfo *info;

      std::vector<const Sync::InsInfo*> m_infos_ssll, m_infos_llss;

      Machine *mss = ssfence.insert(*m,m_infos_ssll,&info); m_infos_ssll.push_back(info);

      Test::inner_test("insert #1 (ssfence)",
                       m->automata[1].same_automaton(mss->automata[0],false));

      Machine *mll = llfence.insert(*m,m_infos_llss,&info); m_infos_llss.push_back(info);

      Test::inner_test("insert #2 (llfence)",
                       m->automata[2].same_automaton(mll->automata[0],false));

      Machine *mssll = llfence.insert(*mss,m_infos_ssll,&info); m_infos_ssll.push_back(info);

      Test::inner_test("insert #3 (ssfence,llfence)",
                       m->automata[3].same_automaton(mssll->automata[0],false));

      Machine *mllss = ssfence.insert(*mll,m_infos_llss,&info); m_infos_llss.push_back(info);

      Test::inner_test("insert #4 (llfence,ssfence)",
                       m->automata[3].same_automaton(mllss->automata[0],false));

      for(auto i : m_infos_ssll){
        delete i;
      }
      for(auto i : m_infos_llss){
        delete i;
      }
      delete mllss;
      delete mssll;
      delete mss;
      delete mll;
      delete m;

    }

    /* Test 5 */
    {
      Machine *m = get_machine(R"(
forbidden L0 L0 L0 L0 L0 L0
process
registers
  $r0 = *
text
  L0: $r0 := 0;
  either{
    $r0 := 1
  or
    $r0 := 2
  };
  either{
    $r0 := 3
  or
    $r0 := 4
  }
process
registers
  $r0 = *
text
  L0: $r0 := 0;
  either{
    $r0 := 1
  or
    $r0 := 2
  };
  llfence;
  ssfence;
  either{
    $r0 := 3
  or
    $r0 := 4
  }
process
registers
  $r0 = *
text
  L0: $r0 := 0;
  either{
    $r0 := 1
  or
    $r0 := 2
  };
  llfence;
  either{
    ssfence;
    $r0 := 3
  or
    $r0 := 4
  }
process
registers
  $r0 = *
text
  L0: $r0 := 0;
  either{
    $r0 := 1;
    llfence
  or
    $r0 := 2
  };
  ssfence;
  either{
    $r0 := 3
  or
    $r0 := 4
  }
process
registers
  $r0 = *
text
  L0: $r0 := 0;
  either{
    $r0 := 1;
    llfence
  or
    $r0 := 2;
    ssfence
  };
  either{
    $r0 := 3
  or
    $r0 := 4
  }
process
registers
  $r0 = *
text
  L0: $r0 := 0;
  either{
    $r0 := 1
  or
    $r0 := 2
  };
  either{
    ssfence;
    $r0 := 3
  or
    llfence;
    $r0 := 4
  }
)");

      VipsSSFenceSync ssfence0 = get_ssfence(m,0,{"$r0:=1","$r0:=2"},{"$r0:=3"});
      VipsSSFenceSync ssfence1 = get_ssfence(m,0,{"$r0:=1","$r0:=2"},{"$r0:=3","$r0:=4"});
      VipsSSFenceSync ssfence2 = get_ssfence(m,0,{"$r0:=2"},{"$r0:=3","$r0:=4"});
      VipsSSFenceSync ssfence3 = get_ssfence(m,0,{"$r0:=1","$r0:=2"},{"$r0:=3"});
      VipsLLFenceSync llfence0 = get_llfence(m,0,{"$r0:=1","$r0:=2"},{"$r0:=3"});
      VipsLLFenceSync llfence1 = get_llfence(m,0,{"$r0:=1","$r0:=2"},{"$r0:=3","$r0:=4"});
      VipsLLFenceSync llfence2 = get_llfence(m,0,{"$r0:=1"},{"$r0:=3","$r0:=4"});
      VipsLLFenceSync llfence3 = get_llfence(m,0,{"$r0:=1","$r0:=2"},{"$r0:=4"});

      Sync::InsInfo *info;

      std::set<Sync::InsInfo*> infos;

      std::vector<const Sync::InsInfo*>
        mi_ss0, mi_ll0, mi_ll1, mi_ss1, mi_ss2, mi_ll2,
        mi_ss3, mi_ll3,
        mi_ll1ss1, mi_ss1ll1,
        mi_ll1ss0, mi_ss0ll1,
        mi_ll0ss1, mi_ss1ll0,
        mi_ss1ll2, mi_ll2ss1,
        mi_ss2ll2, mi_ll2ss2,
        mi_ss3ll3, mi_ll3ss3;

      Machine *mss1 = ssfence1.insert(*m,mi_ss1,&info); mi_ss1.push_back(info); infos.insert(info);
      Machine *mss0 = ssfence0.insert(*m,mi_ss0,&info); mi_ss0.push_back(info); infos.insert(info);
      Machine *mll1 = llfence1.insert(*m,mi_ll1,&info); mi_ll1.push_back(info); infos.insert(info);
      Machine *mll0 = llfence0.insert(*m,mi_ll0,&info); mi_ll0.push_back(info); infos.insert(info);
      Machine *mss2 = ssfence2.insert(*m,mi_ss2,&info); mi_ss2.push_back(info); infos.insert(info);
      Machine *mll2 = llfence2.insert(*m,mi_ll2,&info); mi_ll2.push_back(info); infos.insert(info);
      Machine *mss3 = ssfence3.insert(*m,mi_ss3,&info); mi_ss3.push_back(info); infos.insert(info);
      Machine *mll3 = llfence3.insert(*m,mi_ll3,&info); mi_ll3.push_back(info); infos.insert(info);

      mi_ll1ss1 = mi_ll1;
      Machine *mll1ss1 = ssfence1.insert(*mll1,mi_ll1ss1,&info); mi_ll1ss1.push_back(info); infos.insert(info);
      Test::inner_test("insert #5 (ll,ss,both full)",
                       m->automata[1].same_automaton(mll1ss1->automata[0],false));

      mi_ss1ll1 = mi_ss1;
      Machine *mss1ll1 = llfence1.insert(*mss1,mi_ss1ll1,&info); mi_ss1ll1.push_back(info); infos.insert(info);
      Test::inner_test("insert #6 (ss,ll,both full)",
                       m->automata[1].same_automaton(mss1ll1->automata[0],false));

      mi_ll1ss0 = mi_ll1;
      Machine *mll1ss0 = ssfence0.insert(*mll1,mi_ll1,&info); mi_ll1ss0.push_back(info); infos.insert(info);
      Test::inner_test("insert #7 (ll,ss, ss.OUT < ll.OUT)",
                       m->automata[2].same_automaton(mll1ss0->automata[0],false));

      mi_ss0ll1 = mi_ss0;
      Machine *mss0ll1 = llfence1.insert(*mss0,mi_ss0ll1,&info); mi_ss0ll1.push_back(info); infos.insert(info);
      Test::inner_test("insert #8 (ss,ll, ss.OUT < ll.OUT)",
                       m->automata[2].same_automaton(mss0ll1->automata[0],false));

      try{
        mi_ss1ll0 = mi_ss1;
        Machine *mss1ll0 = llfence0.insert(*mss1,mi_ss1ll0,&info); mi_ss1ll0.push_back(info);
        Test::inner_test("insert #9 (ss,ll, ll.OUT < ss.OUT, Incompatible)",false);
        delete info;
        delete mss1ll0;
      }catch(Sync::Incompatible *exc){
        Test::inner_test("insert #9 (ss,ll, ll.OUT < ss.OUT, Incompatible)",true);
        delete exc;
      }

      try{
        mi_ll0ss1 = mi_ll0;
        Machine *mll0ss1 = ssfence1.insert(*mll0,mi_ll0ss1,&info); mi_ll0ss1.push_back(info);
        Test::inner_test("insert #10 (ll,ss, ll.OUT < ss.OUT, Incompatible)",false);
        delete info;
        delete mll0ss1;
      }catch(Sync::Incompatible *exc){
        Test::inner_test("insert #10 (ll,ss, ll.OUT < ss.OUT, Incompatible)",true);
        delete exc;
      }

      mi_ss1ll2 = mi_ss1;
      Machine *mss1ll2 = llfence2.insert(*mss1,mi_ss1ll2,&info); mi_ss1ll2.push_back(info); infos.insert(info);
      Test::inner_test("insert #11 (ss,ll, ll.IN < ss.IN)",
                       m->automata[3].same_automaton(mss1ll2->automata[0],false));

      mi_ll2ss1 = mi_ll2;
      Machine *mll2ss1 = ssfence1.insert(*mll2,mi_ll2ss1,&info); mi_ll2ss1.push_back(info); infos.insert(info);
      Test::inner_test("insert #12 (ll,ss, ll.IN < ss.IN)",
                       m->automata[3].same_automaton(mll2ss1->automata[0],false));

      mi_ss2ll2 = mi_ss2;
      Machine *mss2ll2 = llfence2.insert(*mss2,mi_ss2ll2,&info); mi_ss2ll2.push_back(info); infos.insert(info);
      Test::inner_test("insert #13 (ss,ll, ll.IN disjunct from ss.IN)",
                       m->automata[4].same_automaton(mss2ll2->automata[0],false));

      mi_ll2ss2 = mi_ll2;
      Machine *mll2ss2 = ssfence2.insert(*mll2,mi_ll2ss2,&info); mi_ll2ss2.push_back(info); infos.insert(info);
      Test::inner_test("insert #14 (ll,ss, ll.IN disjunct from ss.IN)",
                       m->automata[4].same_automaton(mll2ss2->automata[0],false));

      mi_ss3ll3 = mi_ss3;
      Machine *mss3ll3 = llfence3.insert(*mss3,mi_ss3ll3,&info); mi_ss3ll3.push_back(info); infos.insert(info);
      Test::inner_test("insert #15 (ss,ll, ll.OUT disjunct from ss.OUT)",
                       m->automata[5].same_automaton(mss3ll3->automata[0],false));

      mi_ll3ss3 = mi_ll3;
      Machine *mll3ss3 = ssfence3.insert(*mll3,mi_ll3ss3,&info); mi_ll3ss3.push_back(info); infos.insert(info);
      Test::inner_test("insert #16 (ll,ss, ll.OUT disjunct from ss.OUT)",
                       m->automata[5].same_automaton(mll3ss3->automata[0],false));

      for(auto i : infos) delete i;

      delete mss0;
      delete mll0;
      delete mss1;
      delete mll1;
      delete mss2;
      delete mll2;
      delete mss3;
      delete mll3;
      delete mss1ll1;
      delete mll1ss1;
      delete mss0ll1;
      delete mll1ss0;
      delete mss1ll2;
      delete mll2ss1;
      delete mss2ll2;
      delete mll2ss2;
      delete mss3ll3;
      delete mll3ss3;
      delete m;
    }
  }

};
