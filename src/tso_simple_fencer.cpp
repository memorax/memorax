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

#include "preprocessor.h"
#include "test.h"
#include "tso_fence_sync.h"
#include "tso_lock_sync.h"
#include "tso_simple_fencer.h"
#include "vecset.h"

#include <list>
#include <stdexcept>
#include <vector>

TsoSimpleFencer::TsoSimpleFencer(const Machine &m,fence_rule_t rule)
  : TraceFencer(m), fence_rule(rule){
  for(unsigned p = 0; p < machine.automata.size(); ++p){
    fences_by_pq.push_back(std::vector<std::set<TsoFenceSync*> >(machine.automata[p].get_states().size()));
    locks_by_pq.push_back(std::vector<std::set<TsoLockSync*> >(machine.automata[p].get_states().size()));
  }
  if(fence_rule == FENCE || fence_rule == LOCKED_AND_FENCE){
    /* Get all fences for m */
    std::set<Sync*> tfs = TsoFenceSync::get_all_possible(machine);
    for(auto it = tfs.begin(); it != tfs.end(); ++it){
      assert(dynamic_cast<TsoFenceSync*>(*it));
      TsoFenceSync *f = static_cast<TsoFenceSync*>(*it);
      all_fences.insert(f);
      fences_by_pq[f->get_pid()][f->get_q()].insert(f);
    }
  }
  if(fence_rule == LOCKED || fence_rule == LOCKED_AND_FENCE){
    /* Get all locks for m */
    std::set<Sync*> tls = TsoLockSync::get_all_possible(machine);
    for(auto it = tls.begin(); it != tls.end(); ++it){
      assert(dynamic_cast<TsoLockSync*>(*it));
      TsoLockSync *l = static_cast<TsoLockSync*>(*it);
      all_locks.insert(l);
      locks_by_pq[l->get_pid()][l->get_write().source].insert(l);
      locks_by_pq[l->get_pid()][l->get_write().target].insert(l);
    }
  }
};

TsoSimpleFencer::~TsoSimpleFencer(){
  for(auto it = all_fences.begin(); it != all_fences.end(); ++it){
    delete *it;
  }
  for(auto it = all_locks.begin(); it != all_locks.end(); ++it){
    delete *it;
  }
};

std::set<std::set<Sync*> > TsoSimpleFencer::fence(const Trace &t, const std::vector<const Sync::InsInfo*> &m_infos) const{
  /* buffers[p] is the current buffer content for process p. Each
   * element b in buffers[p] is a set of the memory locations modified
   * by one message in the buffer. Elements closer to the front are
   * older.
   */
  std::vector<std::list<VecSet<Lang::MemLoc<int> > > > buffers(machine.automata.size());
  /* pcs[p] is the current control state of process p. */
  std::vector<int> pcs(machine.automata.size(),0);
  /* The synchronizations that prevent some reordering in t. */
  std::set<Sync*> preventing_syncs;
  /* When the buffer of process p is non-empty, collect
   * synchronizations in pending_syncs[p] that would force the buffer
   * to flush. If a read of p is encountered before the buffer of p
   * empties, the synchronizations in pending_syncs[p] have been shown
   * to prevent reordering, and so are removed from pending_syncs[p]
   * and inserted into preventing_syncs.
   */
  std::vector<std::set<Sync*> > pending_syncs(machine.automata.size());

  bool check_rowe = true;

  for(int i = 1; i <= t.size(); ++i){
    int pid = t[i]->pid;
    switch(t[i]->instruction.get_type()){
    case Lang::WRITE:
      buffers[pid].push_back(VecSet<Lang::MemLoc<int> >(t[i]->instruction.get_writes()));
      break;
    case Lang::UPDATE:
      buffers[pid].pop_front();
      if(buffers[pid].empty()){
        /* The delayed writes did not delay past any read */
        pending_syncs[pid].clear();
      }
      break;
    case Lang::LOCKED:
      if(t[i]->instruction.get_statement_count() > 1){
        /* Non-deterministic locked statement. ROWE checking does not
         * work reliably. Disable it. This may cause unnecessary
         * fences to be returned, but it will not affect correctness
         * of this method according to specification.
         */
        check_rowe = false;
      }
      break;
    default:
      // Do nothing
      break;
    }
    pcs[pid] = t[i]->target;
    if(t[i]->instruction.get_reads().size() > 0 && (!check_rowe || !is_ROWE(t[i],buffers[pid]))){
      for(auto it = pending_syncs[pid].begin(); it != pending_syncs[pid].end(); ++it){
        preventing_syncs.insert((*it)->clone());
      }
      pending_syncs[pid].clear();
    }
    /* Should we look for synchronization to insert? */
    if(buffers[pid].size() > 0
       && t[i]->instruction.get_type() != Lang::UPDATE){
      /* Find the next instruction of pid */
      const Automaton::Transition *next_instr = 0;
      for(int j = i+1; next_instr == 0 && j <= t.size(); ++j){
        if(t[j]->pid == pid && t[j]->instruction.get_type() != Lang::UPDATE){
          next_instr = t[j];
        }
      }
      if(next_instr){
        std::set<Sync*> fs = fences_between(pid,*t[i],*next_instr,m_infos);
        pending_syncs[pid].insert(fs.begin(),fs.end());
        Sync *l = lock_trans(pid,*t[i],m_infos);
        if(l){
          pending_syncs[pid].insert(l);
        }
      }
    }
  }

  if(preventing_syncs.empty()){
    /* No W->R relaxation, hence there can be no cycles in the trace
     * graph of t.
     */
    return std::set<std::set<Sync*> >();
  }else{
    std::set<std::set<Sync*> > s;
    s.insert(preventing_syncs);
    return s;
  }
};

bool TsoSimpleFencer::is_ROWE(const Machine::PTransition *t,
                              const std::list<VecSet<Lang::MemLoc<int> > > &buf) const{
  std::vector<Lang::MemLoc<int> > reads = t->instruction.get_reads();
  std::set<Lang::MemLoc<int> > writes;
  for(auto it = buf.begin(); it != buf.end(); ++it){
    writes.insert(it->begin(),it->end());
  }
  return std::all_of(reads.begin(),reads.end(),
                     [&writes](const Lang::MemLoc<int> &ml){
                       return writes.count(ml);
                     });
};

Sync *TsoSimpleFencer::lock_trans(int pid,
                                  const Automaton::Transition &in,
                                  const std::vector<const Sync::InsInfo*> &m_infos) const{
  int q = in.source;
  if(q >= (int)fences_by_pq[pid].size()){
    q = FenceSync::InsInfo::original_q(m_infos,q);
  }

  const std::set<TsoLockSync*> &pqs = locks_by_pq[pid][q];

  Machine::PTransition pt_in(in,pid);

  for(auto it = pqs.begin(); it != pqs.end(); ++it){
    Machine::PTransition pt =
      TsoLockSync::InsInfo::all_tchanges(m_infos,
                                         Machine::PTransition((*it)->get_write(),
                                                              (*it)->get_pid()));
    if(pt.compare(pt_in,false) == 0){
      return *it;
    }
  }

  return 0;
};

std::set<Sync*> TsoSimpleFencer::fences_between(int pid,
                                                const Automaton::Transition &in,
                                                const Automaton::Transition &out,
                                                const std::vector<const Sync::InsInfo*> &m_infos) const{
  assert(in.target == out.source);
  int q = in.target;

  if(q >= (int)fences_by_pq[pid].size()){
    q = FenceSync::InsInfo::original_q(m_infos,q);
  }

  const std::set<TsoFenceSync*> &pqs = fences_by_pq[pid][q];

  /* Return true iff there is some t' in S such that t' changed
   * according to m_infos equals t.
   */
  std::function<bool(const Automaton::Transition&,const FenceSync::TSet&)> member = 
    [&m_infos,pid](const Automaton::Transition &t,const FenceSync::TSet &S){
    for(auto it = S.begin(); it != S.end(); ++it){
      if(t.compare(FenceSync::InsInfo::all_tchanges(m_infos,Machine::PTransition(*it,pid)),false) == 0){
        return true;
      }
    }
    return false;
  };

  std::set<Sync*> syncs;

  for(auto it = pqs.begin(); it != pqs.end(); ++it){
    /* Check that in and out are in IN and OUT of *it. */
    if(member(in,(*it)->get_IN()) && member(out,(*it)->get_OUT())){
      syncs.insert(*it);
    }
  }

  return syncs;
};

void TsoSimpleFencer::test(){
  std::function<Machine*(std::string)> get_machine = 
    [](std::string rmm){
    std::stringstream ss(rmm);
    PPLexer pp(ss);
    return new Machine(Parser::p_test(pp));
  };

  /* Returns the transition from m of process pid going from (and to)
   * the control state labeled src_lbl (and tgt_lbl) with an
   * instruction like instr.
   *
   * The global memory locations in m should be u,v,w,x,y,z in that
   * order. instr may only access global memory locations.
   */
  std::function<Machine::PTransition(const Machine*,int,std::string,std::string,std::string)> trans = 
    [&get_machine](const Machine *m,int pid,std::string src_lbl,std::string instr, std::string tgt_lbl){
    Machine *m2 = get_machine
    ("forbidden *\n"
     "data\n"
     "  u = *\n"
     "  v = *\n"
     "  w = *\n"
     "  x = *\n"
     "  y = *\n"
     "  z = *\n"
     "process\n"
     "text\n"+instr);
    Lang::Stmt<int> stmt = (*m2->automata[0].get_states()[0].fwd_transitions.begin())->instruction;
    delete m2;
    int src = m->automata[pid].state_index_of_label(src_lbl);
    int tgt = m->automata[pid].state_index_of_label(tgt_lbl);
    Automaton::Transition t_like(src,stmt,tgt);
    const Automaton::State &s = m->automata[pid].get_states()[src];
    for(auto it = s.fwd_transitions.begin(); it != s.fwd_transitions.end(); ++it){
      if((*it)->compare(t_like,false) == 0){
        return Machine::PTransition(**it,pid);
      }
    }
    throw new std::logic_error("TsoSimpleFencer::test::trans: No such transition.");
  };

  std::function<int(const Machine*,int,std::string)> cs =
    [](const Machine *m,int pid,std::string lbl){
    return m->automata[pid].state_index_of_label(lbl);
  };

  std::function<Machine::PTransition(const Machine*,int,Lang::NML,std::string)> update =
    [&cs](const Machine *m,int pid,Lang::NML nml,std::string lbl){
    VecSet<Lang::MemLoc<int> > mls;
    mls.insert(nml.localize(pid));
    Lang::Stmt<int> stmt = Lang::Stmt<int>::update(pid,mls);
    int q = cs(m,pid,lbl);
    return Machine::PTransition(q,stmt,q,pid);
  };

  Lang::NML u = Lang::NML::global(0);
  Lang::NML v = Lang::NML::global(1);
  Lang::NML w = Lang::NML::global(2);
  Lang::NML x = Lang::NML::global(3);
  Lang::NML y = Lang::NML::global(4);
  Lang::NML z = Lang::NML::global(5);

  /* Test fence */
  {
    std::function<bool(const std::set<std::set<Sync*> > &,const Machine*,int,int,std::string)> check_S = 
      [&cs](const std::set<std::set<Sync*> > &S,const Machine *m,int sz,int pid,std::string lbl){
      if(S.size() != 1){
        return false;
      }
      std::set<Sync*> Z = *S.begin();
      if((int)Z.size() != sz){
        return false;
      }
      return std::any_of(Z.begin(),Z.end(),
                         [m,&cs,pid,lbl](const Sync *s){
                           const FenceSync *fs = dynamic_cast<const FenceSync*>(s);
                           return fs && fs->get_pid() == pid && fs->get_q() == cs(m,pid,lbl);
                         });
    };

    std::function<bool(const std::set<std::set<Sync*> > &,const Machine*,int,int,std::string,Lang::NML)> check_S_lock = 
      [&cs](const std::set<std::set<Sync*> > &S,const Machine *m,int sz,int pid,std::string lbl,Lang::NML v){
      if(S.size() != 1){
        return false;
      }
      std::set<Sync*> Z = *S.begin();
      if((int)Z.size() != sz){
        return false;
      }
      return std::any_of(Z.begin(),Z.end(),
                         [m,&cs,pid,lbl,v](const Sync *s){
                           const TsoLockSync *ls = dynamic_cast<const TsoLockSync*>(s);
                           if(!ls) return false;
                           Lang::NML w = Lang::NML(ls->get_write().instruction.get_memloc(),pid);
                           return ls->get_pid() == pid && 
                           ls->get_write().source == cs(m,pid,lbl) &&
                           w == v;
                         });
    };

    std::function<void(const std::set<std::set<Sync*> > &)> delete_S = 
      [](const std::set<std::set<Sync*> > &S){
      for(auto it = S.begin(); it != S.end(); ++it){
        for(auto it2 = it->begin(); it2 != it->end(); ++it2){
          delete *it2;
        }
      }
    };

    /* Test 1,2 */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  u = *\n"
         "  v = *\n"
         "  w = *\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "  L0: write: x := 1;\n"
         "  L1: read: y = 0;\n"
         "  L2: nop\n"
         );

      TsoSimpleFencer tsf(*m,FENCE);
      Trace t(0);
      t.push_back(trans(m,0,"L0","write: x := 1","L1"),0);
      t.push_back(trans(m,0,"L1","read: y = 0","L2"),0);
      t.push_back(update(m,0,x,"L2"),0);

      std::set<std::set<Sync*> > fs = tsf.fence(t,std::vector<const Sync::InsInfo*>());
      std::set<Sync*> f = *fs.begin();
      Test::inner_test("fence #1",
                       fs.size() == 1 &&
                       f.size() == 1 &&
                       static_cast<const FenceSync*>(*f.begin())->get_q() == cs(m,0,"L1"));

      Trace t2(0);
      t2.push_back(trans(m,0,"L0","write: x := 1","L1"),0);
      t2.push_back(update(m,0,x,"L1"),0);
      t2.push_back(trans(m,0,"L1","read: y = 0","L2"),0);

      std::set<std::set<Sync*> > fs2 = tsf.fence(t2,std::vector<const Sync::InsInfo*>());
      Test::inner_test("fence #2",fs2.size() == 0);

      delete_S(fs);
      delete_S(fs2);

      delete m;
    }

    /* Test 3,4: ROWE */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  u = *\n"
         "  v = *\n"
         "  w = *\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "  L0: write: x := 1;\n"
         "  L1: read: x = 1;\n"
         "  L2: read: y = 0;\n"
         "  L3: nop\n"
         );

      TsoSimpleFencer tsf(*m,FENCE);
      Trace t(0);
      t.push_back(trans(m,0,"L0","write: x := 1","L1"),0);
      t.push_back(trans(m,0,"L1","read: x = 1","L2"),0);
      t.push_back(trans(m,0,"L2","read: y = 0","L3"),0);
      t.push_back(update(m,0,x,"L3"),0);

      std::set<std::set<Sync*> > fs = tsf.fence(t,std::vector<const Sync::InsInfo*>());
      Test::inner_test("fence #3",check_S(fs,m,2,0,"L1") && check_S(fs,m,2,0,"L2"));

      delete_S(fs);

      Trace t2(0);
      t2.push_back(trans(m,0,"L0","write: x := 1","L1"),0);
      t2.push_back(trans(m,0,"L1","read: x = 1","L2"),0);
      t2.push_back(update(m,0,x,"L2"),0);
      t2.push_back(trans(m,0,"L2","read: y = 0","L3"),0);

      std::set<std::set<Sync*> > fs2 = tsf.fence(t2,std::vector<const Sync::InsInfo*>());
      Test::inner_test("fence #4",fs2.size() == 0);

      delete_S(fs2);

      delete m;
    }

    /* Test 5: Multiple insert before fence */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  u = *\n"
         "  v = *\n"
         "  w = *\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "  L0: nop;\n"
         /* insert fence here (f0) */
         "  L1: either{\n"
         "    write: x := 1\n"
         "  or\n"
         "    nop\n"
         "  };\n"
         "  L2: read: y = 0;\n"
         /* insert fence here (f1) */
         "  L3: either{\n"
         "    write: u := 1\n"
         "  or\n"
         "    nop\n"
         "  };\n"
         "  L4: read: v = 0;\n"
         "  L5: nop\n"
         );

      FenceSync::TSet tmp_IN, tmp_OUT;
      tmp_IN.insert(trans(m,0,"L0","nop","L1"));
      tmp_OUT.insert(trans(m,0,"L1","write: x := 1","L2"));
      TsoFenceSync f0(0,cs(m,0,"L1"),tmp_IN,tmp_OUT);
      tmp_IN.clear(); tmp_OUT.clear();
      tmp_IN.insert(trans(m,0,"L2","read: y = 0","L3"));
      tmp_OUT.insert(trans(m,0,"L3","write: u := 1","L4"));
      TsoFenceSync f1(0,cs(m,0,"L3"),tmp_IN,tmp_OUT);

      std::vector<const Sync::InsInfo*> m_infos;
      Sync::InsInfo *info;
      Machine *m2 = f0.insert(*m,m_infos,&info); m_infos.push_back(info);
      Machine *m3 = f1.insert(*m2,m_infos,&info); m_infos.push_back(info);

      Trace t(0);
      {
        int q = 0;
        const std::vector<Automaton::State> &ss = m3->automata[0].get_states();

        /* nop */
        assert(ss[q].fwd_transitions.size() == 1);
        t.push_back(Machine::PTransition(**ss[q].fwd_transitions.begin(),0),0);
        q = (*ss[q].fwd_transitions.begin())->target;

        /* fence */
        assert(ss[q].fwd_transitions.size() == 2);
        for(auto it = ss[q].fwd_transitions.begin(); it != ss[q].fwd_transitions.end(); ++it){
          if((*it)->instruction.get_type() == Lang::LOCKED){
            t.push_back(Machine::PTransition(**it,0),0);
            q = (*it)->target;
            break;
          }
        }
        assert(t.size() == 2);

        /* write: x := 1 */
        assert(ss[q].fwd_transitions.size() == 1);
        t.push_back(Machine::PTransition(**ss[q].fwd_transitions.begin(),0),0);
        q = (*ss[q].fwd_transitions.begin())->target;

        /* read: y = 0 */
        assert(ss[q].fwd_transitions.size() == 1);
        t.push_back(Machine::PTransition(**ss[q].fwd_transitions.begin(),0),0);
        q = (*ss[q].fwd_transitions.begin())->target;

        /* update x */
        {
          VecSet<Lang::MemLoc<int> > mls;
          mls.insert(x.localize(0));
          t.push_back(Machine::PTransition(q,Lang::Stmt<int>::update(0,mls),q,0),0);
        }

        /* fence */
        assert(ss[q].fwd_transitions.size() == 2);
        for(auto it = ss[q].fwd_transitions.begin(); it != ss[q].fwd_transitions.end(); ++it){
          if((*it)->instruction.get_type() == Lang::LOCKED){
            t.push_back(Machine::PTransition(**it,0),0);
            q = (*it)->target;
            break;
          }
        }
        assert(t.size() == 6);

        /* write: u := 1 */
        assert(ss[q].fwd_transitions.size() == 1);
        t.push_back(Machine::PTransition(**ss[q].fwd_transitions.begin(),0),0);
        q = (*ss[q].fwd_transitions.begin())->target;

        /* read: v = 0 */
        assert(ss[q].fwd_transitions.size() == 1);
        t.push_back(Machine::PTransition(**ss[q].fwd_transitions.begin(),0),0);
        q = (*ss[q].fwd_transitions.begin())->target;

        /* update u */
        {
          VecSet<Lang::MemLoc<int> > mls;
          mls.insert(u.localize(0));
          t.push_back(Machine::PTransition(q,Lang::Stmt<int>::update(0,mls),q,0),0);
        }

      }

      TsoSimpleFencer tsf(*m,FENCE);

      std::set<std::set<Sync*> > fs = tsf.fence(t,m_infos);

      Test::inner_test("fence #5", check_S(fs,m,4,0,"L2") && check_S(fs,m,4,0,"L4"));

      delete_S(fs);
      for(unsigned i = 0; i < m_infos.size(); ++i){
        delete m_infos[i];
      }
      delete m3;
      delete m2;
      delete m;
    }

    /* Test 6: Stacking writes */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  u = *\n"
         "  v = *\n"
         "  w = *\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "  L0: write: u := 1;\n"
         "  L1: write: v := 1;\n"
         "  L2: write: w := 1;\n"
         "  L3: read: v = 1;\n"
         "  L4: read: z = 0;\n"
         "  L5: read: z = 1;\n"
         "  L6: write: x := 1;\n"
         "  L7: read: y = 0;\n"
         "  L8: nop;\n"
         "  L9: nop;\n"
         "  L10: nop"
         );

      TsoSimpleFencer tsf(*m,FENCE);

      Trace t(0);
      t.push_back(trans(m,0,"L0","write: u := 1","L1"),0);
      t.push_back(trans(m,0,"L1","write: v := 1","L2"),0);
      t.push_back(trans(m,0,"L2","write: w := 1","L3"),0);
      t.push_back(trans(m,0,"L3","read: v = 1","L4"),0);
      t.push_back(trans(m,0,"L4","read: z = 0","L5"),0);
      t.push_back(update(m,0,u,"L5"),0);
      t.push_back(update(m,0,v,"L5"),0);
      t.push_back(update(m,0,w,"L5"),0);
      t.push_back(trans(m,0,"L5","read: z = 1","L6"),0);
      t.push_back(trans(m,0,"L6","write: x := 1","L7"),0);
      t.push_back(trans(m,0,"L7","read: y = 0","L8"),0);
      t.push_back(trans(m,0,"L8","nop","L9"),0);
      t.push_back(trans(m,0,"L9","nop","L10"),0);
      t.push_back(update(m,0,x,"L10"),0);

      std::set<std::set<Sync*> > fs = tsf.fence(t,std::vector<const Sync::InsInfo*>());

      Test::inner_test("fence #6",
                       check_S(fs,m,5,0,"L1") &&
                       check_S(fs,m,5,0,"L2") &&
                       check_S(fs,m,5,0,"L3") &&
                       check_S(fs,m,5,0,"L4") &&
                       check_S(fs,m,5,0,"L7"));

      delete_S(fs);
      delete m;
    }

    /* Test 7: Spontaneously locked writes */
    /* A non-locked write executes as a locked write in this trace.
     * (Artifact of TSO reachability implementations.)
     * Should not interfere with fence insertion.
     */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  u = *\n"
         "  v = *\n"
         "  w = *\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "  L0: write: u := 1;\n"
         "  L1: write: x := 1;\n"
         "  L2: read: y = 0;\n"
         "  L3: nop\n"
         );

      Trace t(0);
      {
        Machine::PTransition lwu(cs(m,0,"L0"),
                                 Lang::Stmt<int>::locked_write(u.localize(0),Lang::Expr<int>::integer(1)),
                                 cs(m,0,"L1"),0);
        t.push_back(lwu,0);
      }
      t.push_back(trans(m,0,"L1","write: x := 1","L2"),0);
      t.push_back(trans(m,0,"L2","read: y = 0","L3"),0);
      t.push_back(update(m,0,x,"L3"),0);

      TsoSimpleFencer tsf(*m,FENCE);

      std::set<std::set<Sync*> > fs = tsf.fence(t,std::vector<const Sync::InsInfo*>());

      Test::inner_test("fence #7", check_S(fs,m,1,0,"L2"));

      delete_S(fs);
      delete m;
    }

    /* Test 8: Multiple interleaved overtakings */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  u = *\n"
         "  v = *\n"
         "  w = *\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "  L0: write: u := 1;\n"
         "  L1: read: v = 0;\n"
         "  L2: write: x := 1;\n"
         "  L3: write: z := 1;\n"
         "  L4: read: y = 0;\n"
         "  L5: read: w = 0;\n"
         "  L6: nop\n"
         "process\n"
         "text\n"
         "  K0: write: u := 1;\n"
         "  K1: read: v = 0;\n"
         "  L0: write: y := 1;\n"
         "  L1: write: z := 2;\n"
         "  L2: read: x = 0;\n"
         "  L3: nop\n"
         );

      Trace t(0);
      t.push_back(trans(m,1,"K0","write: u := 1","K1"),0);
      t.push_back(trans(m,0,"L0","write: u := 1","L1"),0);
      t.push_back(trans(m,1,"K1","read: v = 0","L0"),0);
      t.push_back(trans(m,0,"L1","read: v = 0","L2"),0);
      t.push_back(update(m,0,u,"L2"),0);
      t.push_back(update(m,1,u,"L0"),0);
      t.push_back(trans(m,0,"L2","write: x := 1","L3"),0);
      t.push_back(trans(m,0,"L3","write: z := 1","L4"),0);
      t.push_back(trans(m,1,"L0","write: y := 1","L1"),0);
      t.push_back(trans(m,1,"L1","write: z := 2","L2"),0);
      t.push_back(trans(m,0,"L4","read: y = 0","L5"),0);
      t.push_back(update(m,1,y,"L2"),0);
      t.push_back(update(m,1,z,"L2"),0);
      t.push_back(trans(m,1,"L2","read: x = 0","L3"),0);
      t.push_back(update(m,0,x,"L5"),0);
      t.push_back(update(m,0,z,"L5"),0);
      t.push_back(trans(m,0,"L5","read: w = 0","L6"),0);

      TsoSimpleFencer tsf(*m,FENCE);

      std::set<std::set<Sync*> > fs = tsf.fence(t,std::vector<const Sync::InsInfo*>());

      Test::inner_test("fence #8",
                       check_S(fs,m,4,0,"L1") &&
                       check_S(fs,m,4,0,"L3") &&
                       check_S(fs,m,4,0,"L4") &&
                       check_S(fs,m,4,1,"K1"));

      delete_S(fs);
      delete m;
    }

    /* Test 9,10,11,12: Test 1,2 with LOCKED and LOCKED_AND_FENCE */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  u = *\n"
         "  v = *\n"
         "  w = *\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "  L0: write: x := 1;\n"
         "  L1: read: y = 0;\n"
         "  L2: nop\n"
         );

      TsoSimpleFencer tsf_l(*m,LOCKED);
      TsoSimpleFencer tsf_lf(*m,LOCKED_AND_FENCE);
      Trace t(0);
      t.push_back(trans(m,0,"L0","write: x := 1","L1"),0);
      t.push_back(trans(m,0,"L1","read: y = 0","L2"),0);
      t.push_back(update(m,0,x,"L2"),0);

      std::set<std::set<Sync*> > fs_l = tsf_l.fence(t,std::vector<const Sync::InsInfo*>());
      std::set<std::set<Sync*> > fs_lf = tsf_lf.fence(t,std::vector<const Sync::InsInfo*>());
      Test::inner_test("fence #9",
                       check_S_lock(fs_l,m,1,0,"L0",x));
      Test::inner_test("fence #10",
                       check_S(fs_lf,m,2,0,"L1") &&
                       check_S_lock(fs_lf,m,2,0,"L0",x));

      Trace t2(0);
      t2.push_back(trans(m,0,"L0","write: x := 1","L1"),0);
      t2.push_back(update(m,0,x,"L1"),0);
      t2.push_back(trans(m,0,"L1","read: y = 0","L2"),0);

      std::set<std::set<Sync*> > fs_l_2 = tsf_l.fence(t2,std::vector<const Sync::InsInfo*>());
      std::set<std::set<Sync*> > fs_lf_2 = tsf_lf.fence(t2,std::vector<const Sync::InsInfo*>());
      Test::inner_test("fence #11",fs_l_2.size() == 0);
      Test::inner_test("fence #12",fs_lf_2.size() == 0);

      delete_S(fs_l);
      delete_S(fs_lf);
      delete_S(fs_l_2);
      delete_S(fs_lf_2);

      delete m;
    }

    /* Test 13,14,15,16: ROWE with LOCKED and LOCKED_AND_FENCE */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  u = *\n"
         "  v = *\n"
         "  w = *\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "  L0: write: x := 1;\n"
         "  L1: read: x = 1;\n"
         "  L2: read: y = 0;\n"
         "  L3: nop\n"
         );

      TsoSimpleFencer tsf_l(*m,LOCKED);
      TsoSimpleFencer tsf_lf(*m,LOCKED_AND_FENCE);
      Trace t(0);
      t.push_back(trans(m,0,"L0","write: x := 1","L1"),0);
      t.push_back(trans(m,0,"L1","read: x = 1","L2"),0);
      t.push_back(trans(m,0,"L2","read: y = 0","L3"),0);
      t.push_back(update(m,0,x,"L3"),0);

      std::set<std::set<Sync*> > fs_l = tsf_l.fence(t,std::vector<const Sync::InsInfo*>());
      std::set<std::set<Sync*> > fs_lf = tsf_lf.fence(t,std::vector<const Sync::InsInfo*>());
      Test::inner_test("fence #13",check_S_lock(fs_l,m,1,0,"L0",x));
      Test::inner_test("fence #14",
                       check_S(fs_lf,m,3,0,"L1") && 
                       check_S(fs_lf,m,3,0,"L2") &&
                       check_S_lock(fs_lf,m,3,0,"L0",x));

      delete_S(fs_l);
      delete_S(fs_lf);

      Trace t2(0);
      t2.push_back(trans(m,0,"L0","write: x := 1","L1"),0);
      t2.push_back(trans(m,0,"L1","read: x = 1","L2"),0);
      t2.push_back(update(m,0,x,"L2"),0);
      t2.push_back(trans(m,0,"L2","read: y = 0","L3"),0);

      std::set<std::set<Sync*> > fs_l_2 = tsf_l.fence(t2,std::vector<const Sync::InsInfo*>());
      std::set<std::set<Sync*> > fs_lf_2 = tsf_lf.fence(t2,std::vector<const Sync::InsInfo*>());
      Test::inner_test("fence #15",fs_l_2.size() == 0);
      Test::inner_test("fence #16",fs_lf_2.size() == 0);

      delete_S(fs_l_2);
      delete_S(fs_lf_2);

      delete m;
    }

    /* Test 17: Multiple insert before fence with LOCKED */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  u = *\n"
         "  v = *\n"
         "  w = *\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "  L0: nop;\n"
         /* insert fence here (f0) */
         "  L1: either{\n"
         "    write: x := 1\n"
         "  or\n"
         "    nop\n"
         "  };\n"
         "  L2: read: y = 0;\n"
         /* insert fence here (f1) */
         "  L3: either{\n"
         "    write: u := 1\n"
         "  or\n"
         "    nop\n"
         "  };\n"
         "  L4: read: v = 0;\n"
         "  L5: nop\n"
         );

      FenceSync::TSet tmp_IN, tmp_OUT;
      tmp_IN.insert(trans(m,0,"L0","nop","L1"));
      tmp_OUT.insert(trans(m,0,"L1","write: x := 1","L2"));
      TsoFenceSync f0(0,cs(m,0,"L1"),tmp_IN,tmp_OUT);
      tmp_IN.clear(); tmp_OUT.clear();
      tmp_IN.insert(trans(m,0,"L2","read: y = 0","L3"));
      tmp_OUT.insert(trans(m,0,"L3","write: u := 1","L4"));
      TsoFenceSync f1(0,cs(m,0,"L3"),tmp_IN,tmp_OUT);

      std::vector<const Sync::InsInfo*> m_infos;
      Sync::InsInfo *info;
      Machine *m2 = f0.insert(*m,m_infos,&info); m_infos.push_back(info);
      Machine *m3 = f1.insert(*m2,m_infos,&info); m_infos.push_back(info);

      Trace t(0);
      {
        int q = 0;
        const std::vector<Automaton::State> &ss = m3->automata[0].get_states();

        /* nop */
        assert(ss[q].fwd_transitions.size() == 1);
        t.push_back(Machine::PTransition(**ss[q].fwd_transitions.begin(),0),0);
        q = (*ss[q].fwd_transitions.begin())->target;

        /* fence */
        assert(ss[q].fwd_transitions.size() == 2);
        for(auto it = ss[q].fwd_transitions.begin(); it != ss[q].fwd_transitions.end(); ++it){
          if((*it)->instruction.get_type() == Lang::LOCKED){
            t.push_back(Machine::PTransition(**it,0),0);
            q = (*it)->target;
            break;
          }
        }
        assert(t.size() == 2);

        /* write: x := 1 */
        assert(ss[q].fwd_transitions.size() == 1);
        t.push_back(Machine::PTransition(**ss[q].fwd_transitions.begin(),0),0);
        q = (*ss[q].fwd_transitions.begin())->target;

        /* read: y = 0 */
        assert(ss[q].fwd_transitions.size() == 1);
        t.push_back(Machine::PTransition(**ss[q].fwd_transitions.begin(),0),0);
        q = (*ss[q].fwd_transitions.begin())->target;

        /* update x */
        {
          VecSet<Lang::MemLoc<int> > mls;
          mls.insert(x.localize(0));
          t.push_back(Machine::PTransition(q,Lang::Stmt<int>::update(0,mls),q,0),0);
        }

        /* fence */
        assert(ss[q].fwd_transitions.size() == 2);
        for(auto it = ss[q].fwd_transitions.begin(); it != ss[q].fwd_transitions.end(); ++it){
          if((*it)->instruction.get_type() == Lang::LOCKED){
            t.push_back(Machine::PTransition(**it,0),0);
            q = (*it)->target;
            break;
          }
        }
        assert(t.size() == 6);

        /* write: u := 1 */
        assert(ss[q].fwd_transitions.size() == 1);
        t.push_back(Machine::PTransition(**ss[q].fwd_transitions.begin(),0),0);
        q = (*ss[q].fwd_transitions.begin())->target;

        /* read: v = 0 */
        assert(ss[q].fwd_transitions.size() == 1);
        t.push_back(Machine::PTransition(**ss[q].fwd_transitions.begin(),0),0);
        q = (*ss[q].fwd_transitions.begin())->target;

        /* update u */
        {
          VecSet<Lang::MemLoc<int> > mls;
          mls.insert(u.localize(0));
          t.push_back(Machine::PTransition(q,Lang::Stmt<int>::update(0,mls),q,0),0);
        }

      }

      TsoSimpleFencer tsf(*m,LOCKED);

      std::set<std::set<Sync*> > fs = tsf.fence(t,m_infos);

      Test::inner_test("fence #17", 
                       check_S_lock(fs,m,2,0,"L1",x) &&
                       check_S_lock(fs,m,2,0,"L3",u));

      delete_S(fs);
      for(unsigned i = 0; i < m_infos.size(); ++i){
        delete m_infos[i];
      }
      delete m3;
      delete m2;
      delete m;
    }

    /* Test 18: Stacking writes with LOCKED */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  u = *\n"
         "  v = *\n"
         "  w = *\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "  L0: write: u := 1;\n"
         "  L1: write: v := 1;\n"
         "  L2: write: w := 1;\n"
         "  L3: read: v = 1;\n"
         "  L4: read: z = 0;\n"
         "  L5: read: z = 1;\n"
         "  L6: write: x := 1;\n"
         "  L7: read: y = 0;\n"
         "  L8: nop;\n"
         "  L9: nop;\n"
         "  L10: nop"
         );

      TsoSimpleFencer tsf(*m,LOCKED);

      Trace t(0);
      t.push_back(trans(m,0,"L0","write: u := 1","L1"),0);
      t.push_back(trans(m,0,"L1","write: v := 1","L2"),0);
      t.push_back(trans(m,0,"L2","write: w := 1","L3"),0);
      t.push_back(trans(m,0,"L3","read: v = 1","L4"),0);
      t.push_back(trans(m,0,"L4","read: z = 0","L5"),0);
      t.push_back(update(m,0,u,"L5"),0);
      t.push_back(update(m,0,v,"L5"),0);
      t.push_back(update(m,0,w,"L5"),0);
      t.push_back(trans(m,0,"L5","read: z = 1","L6"),0);
      t.push_back(trans(m,0,"L6","write: x := 1","L7"),0);
      t.push_back(trans(m,0,"L7","read: y = 0","L8"),0);
      t.push_back(trans(m,0,"L8","nop","L9"),0);
      t.push_back(trans(m,0,"L9","nop","L10"),0);
      t.push_back(update(m,0,x,"L10"),0);

      std::set<std::set<Sync*> > fs = tsf.fence(t,std::vector<const Sync::InsInfo*>());

      Test::inner_test("fence #18",
                       check_S_lock(fs,m,4,0,"L0",u) &&
                       check_S_lock(fs,m,4,0,"L1",v) &&
                       check_S_lock(fs,m,4,0,"L2",w) &&
                       check_S_lock(fs,m,4,0,"L6",x));

      delete_S(fs);
      delete m;
    }

    /* Test 19: Spontaneously locked writes with LOCKED */
    /* A non-locked write executes as a locked write in this trace.
     * (Artifact of TSO reachability implementations.)
     * Should not interfere with fence insertion.
     */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  u = *\n"
         "  v = *\n"
         "  w = *\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "  L0: write: u := 1;\n"
         "  L1: write: x := 1;\n"
         "  L2: read: y = 0;\n"
         "  L3: nop\n"
         );

      Trace t(0);
      {
        Machine::PTransition lwu(cs(m,0,"L0"),
                                 Lang::Stmt<int>::locked_write(u.localize(0),Lang::Expr<int>::integer(1)),
                                 cs(m,0,"L1"),0);
        t.push_back(lwu,0);
      }
      t.push_back(trans(m,0,"L1","write: x := 1","L2"),0);
      t.push_back(trans(m,0,"L2","read: y = 0","L3"),0);
      t.push_back(update(m,0,x,"L3"),0);

      TsoSimpleFencer tsf(*m,LOCKED);

      std::set<std::set<Sync*> > fs = tsf.fence(t,std::vector<const Sync::InsInfo*>());

      Test::inner_test("fence #19", 
                       check_S_lock(fs,m,1,0,"L1",x));

      delete_S(fs);
      delete m;
    }

    /* Test 20: Multiple interleaved overtakings with LOCKED */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  u = *\n"
         "  v = *\n"
         "  w = *\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "  L0: write: u := 1;\n"
         "  L1: read: v = 0;\n"
         "  L2: write: x := 1;\n"
         "  L3: write: z := 1;\n"
         "  L4: read: y = 0;\n"
         "  L5: read: w = 0;\n"
         "  L6: nop\n"
         "process\n"
         "text\n"
         "  K0: write: u := 1;\n"
         "  K1: read: v = 0;\n"
         "  L0: write: y := 1;\n"
         "  L1: write: z := 2;\n"
         "  L2: read: x = 0;\n"
         "  L3: nop\n"
         );

      Trace t(0);
      t.push_back(trans(m,1,"K0","write: u := 1","K1"),0);
      t.push_back(trans(m,0,"L0","write: u := 1","L1"),0);
      t.push_back(trans(m,1,"K1","read: v = 0","L0"),0);
      t.push_back(trans(m,0,"L1","read: v = 0","L2"),0);
      t.push_back(update(m,0,u,"L2"),0);
      t.push_back(update(m,1,u,"L0"),0);
      t.push_back(trans(m,0,"L2","write: x := 1","L3"),0);
      t.push_back(trans(m,0,"L3","write: z := 1","L4"),0);
      t.push_back(trans(m,1,"L0","write: y := 1","L1"),0);
      t.push_back(trans(m,1,"L1","write: z := 2","L2"),0);
      t.push_back(trans(m,0,"L4","read: y = 0","L5"),0);
      t.push_back(update(m,1,y,"L2"),0);
      t.push_back(update(m,1,z,"L2"),0);
      t.push_back(trans(m,1,"L2","read: x = 0","L3"),0);
      t.push_back(update(m,0,x,"L5"),0);
      t.push_back(update(m,0,z,"L5"),0);
      t.push_back(trans(m,0,"L5","read: w = 0","L6"),0);

      TsoSimpleFencer tsf(*m,LOCKED);

      std::set<std::set<Sync*> > fs = tsf.fence(t,std::vector<const Sync::InsInfo*>());

      Test::inner_test("fence #20",
                       check_S_lock(fs,m,4,1,"K0",u) &&
                       check_S_lock(fs,m,4,0,"L0",u) &&
                       check_S_lock(fs,m,4,0,"L2",x) &&
                       check_S_lock(fs,m,4,0,"L3",z));

      delete_S(fs);
      delete m;
    }

  }
};
