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
#include "test.h"
#include "vips_simple_fencer.h"

#include <cassert>

VipsSimpleFencer::VipsSimpleFencer(const Machine &m) : TraceFencer(m) {
  {
    for(unsigned p = 0; p < m.automata.size(); ++p){
      fences_by_pq.push_back(std::vector<std::set<VipsFenceSync*> >(m.automata[p].get_states().size()));
    }
    auto S = VipsFenceSync::get_all_possible(m);
    for(auto s : S){
      assert(dynamic_cast<VipsFenceSync*>(s));
      VipsFenceSync *vfs = static_cast<VipsFenceSync*>(s);
      all_fences.insert(vfs);
      fences_by_pq[vfs->get_pid()][vfs->get_q()].insert(vfs);
    }
  }
};

VipsSimpleFencer::~VipsSimpleFencer(){
  for(auto s : all_fences){
    delete s;
  }
};

std::set<std::set<Sync*> > VipsSimpleFencer::fence(const Trace &t, const std::vector<const Sync::InsInfo*> &m_infos) const{
  Log::debug << "VipsSimpleFencer trace:\n" << t.to_string() << "\n";
  Trace *t2 = decrease_reorderings(t);
  Log::extreme << "VipsSimpleFencer trace after rewriting:\n" << t2->to_string() << "\n";

  std::vector<std::set<int> > rps = get_reordered_procs(*t2);

  std::set<Sync*> syncs;

  for(int i = 1; i <= t2->size(); ++i){
    int pid = (*t2)[i]->pid;
    if(rps[i].count(pid)){
      /* Find the next instruction transition of pid */
      int i_next = i+1;
      while(i_next <= t2->size() && 
            ((*t2)[i_next]->pid != pid || is_sys_event((*t2)[i_next]->instruction))){
        ++i_next;
      }
      assert(i_next <= t2->size());
      std::set<Sync*> ss = fences_between(pid,*(*t2)[i],*(*t2)[i_next],m_infos);
      syncs.insert(ss.begin(),ss.end());
    }
  }

  delete t2;

  if(syncs.empty()){
    return {};
  }else{
    /* Clone syncs and put them into an enclosing singleton set */
    std::set<Sync*> syncs_clone;
    for(auto s : syncs){
      syncs_clone.insert(s->clone());
    }
    return {syncs_clone};
  }
};

std::vector<std::set<int> > VipsSimpleFencer::get_reordered_procs(const Trace &t){
  std::map<int,int> sps = get_sync_points(t);
  std::vector<std::set<int> > rps(t.size()+1);

  for(int i = 1; i < t.size(); ++i){
    if(sps.count(i) == 0) continue; // Ignore this transition
    for(int j = i+1; j <= t.size(); ++j){
      if(sps.count(j) == 0 || t[j]->pid != t[i]->pid) continue; // Ignore this transition
      if(sps[j] < sps[i]){
        // Reordering, add pid to all intermediate elements in rps
        for(int k = i; k < j; ++k){
          rps[k].insert(t[i]->pid);
        }
      }
    }
  }

  return rps;
};

std::map<int,int> VipsSimpleFencer::get_sync_points(const Trace &t){
  std::map<int,int> sps;

  for(int i = 1; i <= t.size(); ++i){
    switch(t[i]->instruction.get_type()){
    case Lang::NOP: case Lang::ASSIGNMENT: case Lang::ASSUME:
    case Lang::FENCE: case Lang::FETCH: case Lang::EVICT: case Lang::WRLLC:
      // Do nothing
      break;
    case Lang::WRITE:
      {
        /* Search for later, matching wrllc */
        for(int j = i+1; j <= t.size(); ++j){
          if(t[j]->pid == t[i]->pid && 
             t[j]->instruction.get_type() == Lang::WRLLC &&
             t[j]->instruction.get_writes() == t[i]->instruction.get_writes()){
            sps[i] = j;
            break;
          }
        }
        /* If there is no later wrllc, then search for a coalesced, earlier write. */
        if(sps.count(i) == 0){
          for(int j = i-1; j >= 1; --j){
            if(t[j]->pid == t[i]->pid &&
               t[j]->instruction.get_type() == Lang::WRITE &&
               t[j]->instruction.get_writes() == t[i]->instruction.get_writes() &&
               sps[j] > t.size()){
              assert(sps.count(j));
              sps[i] = sps[j];
              break;
            }
          }
        }
        /* If there is no coalesced write, then assign a
         * synchronization point after the end of t. */
        if(sps.count(i) == 0){
          sps[i] = t.size() + i;
        }
        break;
      }
    case Lang::READASSERT: case Lang::READASSIGN:
      {
        /* Find the greatest point that is either a preceeding fetch,
         * or the synchronization point of a preceeding write. */
        int j = -1;
        for(int k = 1; k < i; ++k){
          if(t[k]->pid == t[i]->pid &&
             t[k]->instruction.get_writes() == t[i]->instruction.get_reads()){
            if(t[k]->instruction.get_type() == Lang::FETCH){
              assert(k > j);
              j = k;
            }else if(t[k]->instruction.get_type() == Lang::WRITE){
              assert(sps[k] >= j);
              j = sps[k];
            }
          }
        }
        assert(j > 0);
        sps[i] = j;
        break;
      }
    case Lang::SYNCWR: case Lang::LOCKED:
      assert(t[i]->instruction.get_type() != Lang::LOCKED ||
             is_cas(t[i]->instruction));
      sps[i] = i;
      break;
    default:
      throw new std::logic_error("VipsSimpleFencer::get_sync_points: Unsupported instruction in trace: "+
                                 t[i]->to_string(Lang::int_reg_to_string(),Lang::int_memloc_to_string()));
    }
  }

  return sps;
};

std::set<Sync*> VipsSimpleFencer::fences_between(int pid,
                                                 const Automaton::Transition &in,
                                                 const Automaton::Transition &out,
                                                 const std::vector<const Sync::InsInfo*> &m_infos) const{
  assert(in.target == out.source);
  int q = in.target;

  if(q >= (int)fences_by_pq[pid].size()){
    q = FenceSync::InsInfo::original_q(m_infos,q);
  }

  const std::set<VipsFenceSync*> &pqs = fences_by_pq[pid][q];

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

bool VipsSimpleFencer::is_sys_event(const Lang::Stmt<int> &s){
  return s.get_type() == Lang::FETCH ||
    s.get_type() == Lang::WRLLC ||
    s.get_type() == Lang::EVICT;
};

bool VipsSimpleFencer::is_cas(const Lang::Stmt<int> &s){
  if(s.get_statement_count() != 1) return false;

  const Lang::Stmt<int> *seq = s.get_statement(0);
  if(seq->get_type() != Lang::SEQUENCE) return false;
  if(seq->get_statement_count() != 2) return false;

  const Lang::Stmt<int> *r = seq->get_statement(0);
  const Lang::Stmt<int> *w = seq->get_statement(1);
  if(r->get_type() != Lang::READASSERT) return false;
  if(w->get_type() != Lang::WRITE) return false;
  if(r->get_memloc() != w->get_memloc()) return false;

  return true;
}

Trace *VipsSimpleFencer::decrease_reorderings(const Trace &t){
  std::vector<Machine::PTransition> ptv;
  for(int i = 1; i <= t.size(); ++i){
    ptv.push_back(*t[i]);
  }

  std::function<void(const std::set<int>&)> remove_pts =
    [&ptv](const std::set<int> &rm){
    unsigned next = 0;
    for(unsigned i = 0; i < ptv.size(); ++i){
      if(rm.count(i)){
        // Do nothing
      }else{
        ptv[next] = ptv[i];
        ++next;
      }
    }
    ptv.resize(next,{0,Lang::Stmt<int>::nop(),0,0});
  };

  std::function<void(int,int)> move_before =
    [&ptv](int i, int tgt){
    if(i == tgt || i == tgt-1){
      // Do nothing
    }else if(i < tgt){
      Machine::PTransition pt_i = ptv[i];
      int j = i;
      while(j < tgt-1){
        ptv[j] = ptv[j+1];
        ++j;
      }
      ptv[tgt-1] = pt_i;
    }else{ // tgt < i
      Machine::PTransition pt_i = ptv[i];
      int j = i;
      while(j > tgt){
        ptv[j] = ptv[j-1];
        --j;
      }
      ptv[tgt] = pt_i;
    }
  };

  std::function<bool(const Machine::PTransition&,const Lang::NML&)> pt_accesses =
    [](const Machine::PTransition &pt,const Lang::NML &nml){
    std::set<Lang::NML> pt_nmls;
    auto ws = pt.instruction.get_writes();
    auto rs = pt.instruction.get_reads();
    for(unsigned i = 0; i < ws.size(); ++i){
      pt_nmls.insert(Lang::NML(ws[i],pt.pid));
    }
    for(unsigned i = 0; i < rs.size(); ++i){
      pt_nmls.insert(Lang::NML(rs[i],pt.pid));
    }
    return pt_nmls.count(nml);
  };

  std::function<int(int,int)> src_pc =
    [&ptv](int pid, int i){
    int q = 0;
    for(int j = 0; j < i; ++j){
      if(ptv[j].pid == pid){
        q = ptv[j].target;
      }
    }
    return q;
  };

  /* Eliminate unnecessary fetch/evict transitions */
  {
    std::set<int> remove;
    // last_fetch[{pid,ml}] is the last seen fetch of ml by pid
    // only memory locations that are currently in L1 are in last_fetch
    // only fetches that have not been followed by a variable use are in last_fetch
    std::map<std::pair<int,Lang::MemLoc<int> >,int> last_fetch;
    for(unsigned i = 0; i < ptv.size(); ++i){
      int pid = ptv[i].pid;
      switch(ptv[i].instruction.get_type()){
      case Lang::FETCH:
        assert(last_fetch.count({pid,ptv[i].instruction.get_writes()[0]}) == 0);
        last_fetch[{pid,ptv[i].instruction.get_writes()[0]}] = i;
        break;
      case Lang::EVICT:
        if(last_fetch.count({pid,ptv[i].instruction.get_writes()[0]})){
          remove.insert(last_fetch[{pid,ptv[i].instruction.get_writes()[0]}]);
          last_fetch.erase({pid,ptv[i].instruction.get_writes()[0]});
          remove.insert(i);
        }
        break;
      case Lang::WRITE:
        last_fetch.erase({pid,ptv[i].instruction.get_writes()[0]});
        break;
      case Lang::READASSERT: case Lang::READASSIGN:
        last_fetch.erase({pid,ptv[i].instruction.get_reads()[0]});
        break;
      case Lang::LOCKED: case Lang::SYNCWR:
        assert(last_fetch.count({pid,ptv[i].instruction.get_writes()[0]}) == 0);
        break;
      default:
        break;
      }
    }
    for(auto it = last_fetch.begin(); it != last_fetch.end(); ++it){
      remove.insert(it->second);
    }
    // Find unnecessary final evicts
    {
      std::set<int> p_has_fenced;
      std::set<std::pair<int,Lang::MemLoc<int> > > need_evicted;
      for(int i = int(ptv.size()) - 1; i >= 0; --i){
        int pid = ptv[i].pid;
        switch(ptv[i].instruction.get_type()){
        case Lang::FETCH: case Lang::LOCKED: case Lang::SYNCWR:
          need_evicted.insert({pid,ptv[i].instruction.get_writes()[0]});
          break;
        case Lang::FENCE:
          p_has_fenced.insert(pid);
          break;
        case Lang::EVICT:
          if(p_has_fenced.count(pid) == 0 &&
             need_evicted.count({pid,ptv[i].instruction.get_writes()[0]}) == 0){
            remove.insert(i);
          }
          break;
        default:
          break;
        }
      }
    }

    // Remove
    remove_pts(remove);
  }

  /* Delay fetches as much as possible */
  {
    unsigned i = 0;
    while(i < ptv.size()){
      int pid = ptv[i].pid;
      if(ptv[i].instruction.get_type() == Lang::FETCH){
        Lang::NML nml(ptv[i].instruction.get_writes()[0],pid);
        unsigned j;
        for(j = i+1; j < ptv.size(); ++j){
          int j_pid = ptv[j].pid;
          if(j_pid == pid && pt_accesses(ptv[j],nml)){
            ptv[i].source = ptv[i].target = src_pc(pid,j);
            move_before(i,j);
            break;
          }else if(j_pid != pid &&
                   pt_accesses(ptv[j],nml) &&
                   (ptv[j].instruction.get_type() == Lang::WRLLC ||
                    ptv[j].instruction.get_type() == Lang::LOCKED ||
                    ptv[j].instruction.get_type() == Lang::SYNCWR)){
            ptv[i].source = ptv[i].target = src_pc(pid,j);
            move_before(i,j);
            break;
          }else{
            // Do nothing
          }
        }
        assert(j < ptv.size());
        if(j == i+1){
          ++i;
        }
      }else{
        ++i;
      }
    }
  }

  /* Advance wrllcs as much as possible */
  {
    /* Add wrllcs where missing */
    {
      std::set<std::pair<int,Lang::MemLoc<int> > > pending_writes;
      for(unsigned i = 0; i < ptv.size(); ++i){
        int pid = ptv[i].pid;
        if(ptv[i].instruction.get_type() == Lang::WRITE){
          pending_writes.insert({pid,ptv[i].instruction.get_writes()[0]});
        }else if(ptv[i].instruction.get_type() == Lang::WRLLC){
          assert(pending_writes.count({pid,ptv[i].instruction.get_writes()[0]}));
          pending_writes.erase({pid,ptv[i].instruction.get_writes()[0]});
        }
      }
      for(auto it = pending_writes.begin(); it != pending_writes.end(); ++it){
        int pid = it->first;
        Lang::MemLoc<int> ml = it->second;
        int q = src_pc(pid,ptv.size());
        ptv.push_back({q,Lang::Stmt<int>::wrllc(ml),q,pid});
      }
    }
    /* Push back wrllcs */
    for(unsigned i = 0; i < ptv.size(); ++i){
      if(ptv[i].instruction.get_type() == Lang::WRLLC){
        int pid = ptv[i].pid;
        Lang::NML nml(ptv[i].instruction.get_writes()[0],pid);
        int j;
        for(j = i-1; j >= 0; --j){
          int j_pid = ptv[j].pid;
          if(j_pid == pid &&
             ptv[j].instruction.get_type() == Lang::WRITE &&
             pt_accesses(ptv[j],nml)){
            ptv[i].source = ptv[i].target = src_pc(pid,j+1);
            move_before(i,j+1);
            break;
          }else if(j_pid != pid &&
                   pt_accesses(ptv[j],nml) &&
                   (ptv[j].instruction.get_type() == Lang::WRLLC ||
                    ptv[j].instruction.get_type() == Lang::FETCH ||
                    ptv[j].instruction.get_type() == Lang::LOCKED ||
                    ptv[j].instruction.get_type() == Lang::SYNCWR)){
            ptv[i].source = ptv[i].target = src_pc(pid,j+1);
            move_before(i,j+1);
            break;
          }else{
            // Do nothing
          }
        }
        assert(j >= 0);
      }
    }
  }

  Trace *t2 = new Trace(0);
  for(unsigned i = 0; i < ptv.size(); ++i){
    t2->push_back(ptv[i],0);
  }
  return t2;
};

void VipsSimpleFencer::test(){
  std::function<Machine*(std::string)> get_machine =
    [](std::string rmm){
    std::stringstream ss(rmm);
    PPLexer lex(ss);
    return new Machine(Parser::p_test(lex));
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
    throw new std::logic_error("TsoSimpleFencer::test::trans: No such transition: ("+
                               src_lbl+","+instr+","+tgt_lbl+")");
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

  std::function<Trace*(const Machine*,std::string)> get_vips_trace = 
    [&trans,&cs,&update](const Machine *m,std::string tstr){
    while(std::isspace(tstr[0])) tstr = tstr.substr(1);
    while(std::isspace(tstr[tstr.size()-1])) tstr = tstr.substr(0,tstr.size()-1);
    Trace *trace = new Trace(0);
    std::vector<int> pcs(m->automata.size(),0);
    std::stringstream ss(tstr);
    while(!ss.eof()){
      std::string ln;
      getline(ss,ln);
      int pid;
      std::string src_lbl, tgt_lbl, sinstr;
      bool is_sys_event = false;
      {
        while(std::isspace(ln[0])) ln = ln.substr(1);
        while(std::isspace(ln[ln.size()-1])) ln = ln.substr(0,ln.size()-1);
        std::stringstream lnss(ln);
        std::string s;
        getline(lnss,s,' ');
        assert(s[0] == 'P');
        std::stringstream pss(s.substr(1));
        pss >> pid;

        getline(lnss,s,' ');
        if(s == "fetch" || s == "wrllc" || s == "evict"){
          is_sys_event = true;
          std::string v;
          getline(lnss,v);
          Lang::MemLoc<int> ml = Lang::MemLoc<int>::global(v[0] - 'u');
          Lang::Stmt<int> instr = Lang::Stmt<int>::nop();
          if(s == "fetch"){
            instr = Lang::Stmt<int>::fetch(ml);
          }else if(s == "wrllc"){
            instr = Lang::Stmt<int>::wrllc(ml);
          }else{
            instr = Lang::Stmt<int>::evict(ml);
          }
          Machine::PTransition pt(pcs[pid],instr,pcs[pid],pid);
          trace->push_back(pt,0);
        }else{
          is_sys_event = false;
          src_lbl = s;
          getline(lnss,tgt_lbl,' ');
          getline(lnss,sinstr);
        }
      }
      if(!is_sys_event){
        Machine::PTransition pt = trans(m,pid,src_lbl,sinstr,tgt_lbl);
        pcs[pid] = pt.target;
        trace->push_back(pt,0);
      }
    }
    return trace;
  };

  /* Test get_sync_points */
  {
    std::function<bool(const Trace*,std::vector<int>)> tst_sps =
      [](const Trace *t, std::vector<int> tgt){
      std::map<int,int> sps = get_sync_points(*t);
      for(auto pr : sps){
        if(pr.first < 1 || pr.first >= tgt.size()){
          return false;
        }
      }
      for(unsigned i = 1; i < tgt.size(); ++i){
        if(tgt[i] > 0){
          if(sps.count(i) == 0 || sps[i] != tgt[i]){
            return false;
          }
        }else{
          if(sps.count(i)){
            return false;
          }
        }
      }
      return true;
    };

    /* Test 1,2 */
    {
      Machine *m = get_machine(R"(
forbidden CS CS
data
  u = *
  v = *
  w = *
  x = *
  y = *
  z = *
process
text
  L0: write: x := 1;
  L1: read: y = 0;
  CS: write: x := 0;
  goto L0
process
text
  L0: write: y := 1;
  L1: read: x = 0;
  CS: write: y := 0;
  goto L0
)");

      Trace *t = get_vips_trace(m,R"(
P0 fetch x
P0 L0 L1 write: x := 1
P0 fetch y
P1 fetch x
P0 L1 CS read: y = 0
P1 fetch y
P1 L0 L1 write: y := 1
P0 wrllc x
P0 evict x
P1 L1 CS read: x = 0
P1 wrllc y
)");

      Test::inner_test("get_sync_points #1",
                       tst_sps(t,{-1,-1,8,-1,-1,3,-1,11,-1,-1,4}));

      delete t;

      t = get_vips_trace(m,R"(
P0 fetch y
P0 fetch x
P1 fetch x
P1 fetch y
P0 L0 L1 write: x := 1
P0 L1 CS read: y = 0
P1 L0 L1 write: y := 1
P1 L1 CS read: x = 0
)");

      Test::inner_test("get_sync_points #2",
                       tst_sps(t,{-1,-1,-1,-1,-1,13,1,15,3}));


      delete t;

      delete m;
    }

    /* Test 3,4 */
    {
      Machine *m = get_machine(R"(
forbidden * *
data
  u = *
  v = *
  w = *
  x = *
  y = *
  z = *
process
text
  L0: write: x := 1;
  L1: write: x := 2;
  L2: read: x = 2;
  L3: read: x = 3;
  L4: nop
process
text
  L0: nop;
  L1: write: x := 3;
  L2: read: x = 3;
  L3: nop
)");

      Trace *t = get_vips_trace(m,R"(
P0 fetch x
P1 fetch x
P0 L0 L1 write: x := 1
P0 L1 L2 write: x := 2
P1 L0 L1 nop
P1 L1 L2 write: x := 3
P0 L2 L3 read: x = 2
P0 wrllc x
P0 evict x
P1 L2 L3 read: x = 3
P1 wrllc x
P0 fetch x
P0 L3 L4 read: x = 3
)");

      Test::inner_test("get_sync_points #3",
                       tst_sps(t,{-1,-1,-1,8,8,-1,11,8,-1,-1,11,-1,-1,12}));

      delete t;

      t = get_vips_trace(m,R"(
P0 fetch x
P0 L0 L1 write: x := 1
P0 L1 L2 write: x := 2
P0 L2 L3 read: x = 2
)");

      Test::inner_test("get_sync_points #4",
                       tst_sps(t,{-1,-1,6,6,6}));

      delete t;
      delete m;

    }

    /* Test 5 */
    {
      Machine *m = get_machine(R"(
forbidden *
data
  u = *
  v = *
  w = *
  x = *
  y = *
  z = *
process
text
  L0: cas(x,0,1);
  L1: syncwr: x := 2;
  L2: nop
)");

      Trace *t = get_vips_trace(m,R"(
P0 L0 L1 cas(x,0,1)
P0 L1 L2 syncwr: x := 2
)");

      Test::inner_test("get_sync_points #5",
                       tst_sps(t,{-1,1,2}));

      delete t;
      delete m;

    }

    /* Test 6 */
    {
      Machine *m = get_machine(R"(
forbidden *
data
  u = *
  v = *
  w = *
  x = *
  y = *
  z = *
process
text
  L0: write: x := 1;
  L1: write: x := 2;
  L2: nop
)");

      Trace *t = get_vips_trace(m,R"(
P0 fetch x
P0 L0 L1 write: x := 1
P0 wrllc x
P0 L1 L2 write: x := 2
)");

      Test::inner_test("get_sync_points #6",
                       tst_sps(t,{-1,-1,3,-1,8}));

      delete t;
      delete m;
    }

  }

  /* Test get_reordered_procs */
  {
    std::function<bool(const Trace*,std::vector<std::set<int> >)> tst_rps =
      [](const Trace *t,std::vector<std::set<int> > tgt){
      return get_reordered_procs(*t) == tgt;
    };

    /* Test 1,2 */
    {
      Machine *m = get_machine(R"(
forbidden * *
data
  u = *
  v = *
  w = *
  x = *
  y = *
  z = *
process
text
  L0: write: x := 1;
  L1: read: y = 0;
  L2: write: x := 0;
  goto L0
process
text
  L0: write: y := 1;
  L1: read: x = 0;
  L2: write: y := 0;
  goto L0
)");

      Trace *t = get_vips_trace(m,R"(
P0 fetch x
P0 fetch y
P0 L0 L1 write: x := 1
P1 fetch y
P0 L1 L2 read: y = 0
P1 L0 L1 write: y := 1
P1 wrllc y
P1 fetch x
P1 L1 L2 read: x = 0
)");

      Test::inner_test("get_reordered_procs #1",
                       tst_rps(t,{{},{},{},{0},{0},{},{},{},{},{}}));

      delete t;

      t = get_vips_trace(m,R"(
P0 fetch x
P0 fetch y
P0 L0 L1 write: x := 1
P1 fetch y
P1 L0 L1 write: y := 1
P0 L1 L2 read: y = 0
P1 fetch x
P1 wrllc y
P1 L1 L2 read: x = 0
)");

      Test::inner_test("get_reordered_procs #2",
                       tst_rps(t,{{},{},{},{0},{0},{0,1},{1},{1},{1},{}}));

      delete t;
      delete m;
    }

    /* Test 3,4 */
    {
      Machine *m = get_machine(R"(
forbidden *
data
  u = *
  v = *
  w = *
  x = *
  y = *
  z = *
process
text
  L0: write: x := 1;
  L1: read: x = 1;
  L2: read: y = 0;
  L3: nop
)");

      Trace *t = get_vips_trace(m,R"(
P0 fetch x
P0 L0 L1 write: x := 1
P0 L1 L2 read: x = 1
P0 wrllc x
P0 fetch y
P0 L2 L3 read: y = 0
)");

      Test::inner_test("get_reordered_procs #3",
                       tst_rps(t,{{},{},{},{},{},{},{}}));
      delete t;

      t = get_vips_trace(m,R"(
P0 fetch y
P0 fetch x
P0 L0 L1 write: x := 1
P0 L1 L2 read: x = 1
P0 wrllc x
P0 L2 L3 read: y = 0
)");

      Test::inner_test("get_reordered_procs #4",
                       tst_rps(t,{{},{},{},{0},{0},{0},{}}));
      delete t;
      delete m;
    }
  }

  /* Test fence */
  {
    std::function<bool(const Machine*,const std::set<std::set<Sync*> >, std::vector<std::set<std::string> >)> tst_fence =
      [&cs](const Machine *m,const std::set<std::set<Sync*> > syncs, std::vector<std::set<std::string> > tgt){
      std::set<std::pair<int,int> > syncs_pcs, tgt_pcs;
      for(unsigned p = 0; p < tgt.size(); ++p){
        for(auto lbl : tgt[p]){
          tgt_pcs.insert({p,cs(m,p,lbl)});
        }
      }
      if(syncs.size() != 1){
        return false;
      }
      for(auto s : *syncs.begin()){
        VipsFenceSync *vfs = static_cast<VipsFenceSync*>(s);
        syncs_pcs.insert({vfs->get_pid(),vfs->get_q()});
      }
      return syncs_pcs == tgt_pcs;
    };

    /* Test 1 */
    {
      Machine *m = get_machine(R"(
forbidden CS CS
data
  u = *
  v = *
  w = *
  x = *
  y = *
  z = *
process
text
  L0: write: x := 1;
  L1: read: y = 0;
  CS: write: x := 0;
  goto L0
process
text
  L0: write: y := 1;
  L1: read: x = 0;
  CS: write: y := 0;
  goto L0
)");

      Trace *t = get_vips_trace(m,R"(
P0 fetch x
P0 L0 L1 write: x := 1
P0 fetch y
P0 L1 CS read: y = 0
  P1 fetch y
  P1 L0 L1 write: y := 1
  P1 wrllc y
  P1 fetch x
  P1 L1 CS read: x = 0
P0 wrllc x
P0 evict x
)");

      VipsSimpleFencer vsf(*m);
      std::set<std::set<Sync*> > syncs = vsf.fence(*t,{});

      Test::inner_test("fence #1",tst_fence(m,syncs,{{"L1"},{}}));

      for(auto ss : syncs){
        for(auto s : ss){
          delete s;
        }
      }

      delete t;
      delete m;
    }
  }

  /* Test decrease_reorderings */
  {
    std::function<bool(const Trace&,const Trace &)> eq_trace =
      [](const Trace &t1, const Trace &t2){
      if(t1.size() != t2.size()){
        return false;
      }
      for(int i = 1; i <= t1.size(); ++i){
        if(t1[i]->compare(*t2[i],false) != 0){
          return false;
        }
      }
      return true;
    };
    std::function<bool(const Machine*,std::string,std::string)> tst_dec_reord =
      [&eq_trace,&get_vips_trace](const Machine *m,std::string t,std::string tgt){
      Trace *t_tgt = get_vips_trace(m,tgt);
      Trace *t_t = get_vips_trace(m,t);
      Trace *t_dec = decrease_reorderings(*t_t);
      bool eq = eq_trace(*t_tgt,*t_dec);
      if(!eq){
        Log::debug << "Expected trace:\n" << t_tgt->to_string(*m)
                   << "Got trace:\n" << t_dec->to_string(*m);
      }
      delete t_dec;
      delete t_tgt;
      delete t_t;
      return eq;
    };

    /* Test 1 */
    {
      Machine *m = get_machine(R"(
forbidden * *
data
  u = *
  v = *
  w = *
  x = *
  y = *
  z = *
process
text
  L0: write: x := 1;
  L1: read: y = 0;
  CS: write: x := 0;
  goto L0
process
text
  L0: write: y := 1;
  L1: read: x = 0;
  CS: write: y := 0;
  goto L0
)");

      Machine *m_fenced = get_machine(R"(
forbidden * *
data
  u = *
  v = *
  w = *
  x = *
  y = *
  z = *
process
text
  L0: write: x := 1;
  L1: fence;
  L2: read: y = 0;
  CS: write: x := 0;
  goto L0
process
text
  L0: write: y := 1;
  L1: fence;
  L2: read: x = 0;
  CS: write: y := 0;
  goto L0
)");

      /* Unnecessary fetch */
      Test::inner_test("decrease_reorderings #1",
                       tst_dec_reord(m,
                                     R"(
P0 fetch y
P0 fetch x
P0 L0 L1 write: x := 1
)",
                                     R"(
P0 fetch x
P0 L0 L1 write: x := 1
P0 wrllc x
)"));

      /* Unnecessary fetch/evict */
      Test::inner_test("decrease_reorderings #2",
                       tst_dec_reord(m,
                                     R"(
P0 fetch y
P0 fetch x
P0 evict y
P0 L0 L1 write: x := 1
)",
                                     R"(
P0 fetch x
P0 L0 L1 write: x := 1
P0 wrllc x
)"));

      /* Unnecessary evict */
      Test::inner_test("decrease_reorderings #3",
                       tst_dec_reord(m,
                                     R"(
P0 fetch x
P0 L0 L1 write: x := 1
P0 wrllc x
P0 evict x
)",
                                     R"(
P0 fetch x
P0 L0 L1 write: x := 1
P0 wrllc x
)"));

      /* Necessary evict */
      Test::inner_test("decrease_reorderings #4",
                       tst_dec_reord(m_fenced,
                                     R"(
P0 fetch x
P0 L0 L1 write: x := 1
P0 wrllc x
P0 evict x
P0 L1 L2 fence
)",
                                     R"(
P0 fetch x
P0 L0 L1 write: x := 1
P0 wrllc x
P0 evict x
P0 L1 L2 fence
)"));

      /* Unnecessary fetch/evict before fence */
      Test::inner_test("decrease_reorderings #5",
                       tst_dec_reord(m_fenced,
                                     R"(
P0 fetch x
P0 fetch y
P0 L0 L1 write: x := 1
P0 wrllc x
P0 evict x
P0 evict y
P0 L1 L2 fence
)",
                                     R"(
P0 fetch x
P0 L0 L1 write: x := 1
P0 wrllc x
P0 evict x
P0 L1 L2 fence
)"));

      Test::inner_test("decrease_reorderings #6",
                       tst_dec_reord(m,
                                     R"(
P0 fetch x
P0 fetch y
  P1 fetch x
  P1 fetch y
P0 L0 L1 write: x := 1
P0 L1 CS read: y = 0
  P1 L0 L1 write: y := 1
  P1 L1 CS read: x = 0
)",
                                     R"(
P0 fetch x
P0 L0 L1 write: x := 1
P0 fetch y
P0 L1 CS read: y = 0
  P1 fetch y
  P1 L0 L1 write: y := 1
  P1 wrllc y
  P1 fetch x
P0 wrllc x
  P1 L1 CS read: x = 0
)"));

      delete m;
      delete m_fenced;

    }
  }

};
