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
    auto S = VipsFenceSync::get_all_possible(m);
    for(auto s : S){
      assert(dynamic_cast<VipsFenceSync*>(s));
      all_fences.insert(static_cast<VipsFenceSync*>(s));
    }
  }
};

VipsSimpleFencer::~VipsSimpleFencer(){
  for(auto s : all_fences){
    delete s;
  }
};

std::set<std::set<Sync*> > VipsSimpleFencer::fence(const Trace &t, const std::vector<const Sync::InsInfo*> &m_infos) const{
  throw new std::logic_error("VipsSimpleFencer::fence: Not implemented.");
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
               t[j]->instruction.get_writes() == t[i]->instruction.get_writes()){
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

    }
  }

};
