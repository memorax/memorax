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

#include <cassert>
#include <cctype>
#include <set>

TsoLockSync::TsoLockSync(const Machine::PTransition &w) : w(w) {};

TsoLockSync::InsInfo::InsInfo(const TsoLockSync *creator_copy) : Sync::InsInfo(creator_copy) {};

void TsoLockSync::InsInfo::bind(const Machine::PTransition &a,const Machine::PTransition &b){
  auto res = tchanges.insert(std::pair<Machine::PTransition,Machine::PTransition>(a,b));
  if(!res.second){
    tchanges.at(a) = b;
  }
};

const Machine::PTransition &TsoLockSync::InsInfo::operator[](const Machine::PTransition &t) const{
  return tchanges.at(t);
};

Machine::PTransition TsoLockSync::InsInfo::all_tchanges(const std::vector<const Sync::InsInfo*> &ivec,
                                                        const Machine::PTransition &t){
  Machine::PTransition t2 = t;
  for(unsigned i = 0; i < ivec.size(); ++i){
    if(dynamic_cast<const InsInfo*>(ivec[i])){
      const InsInfo *ii = static_cast<const InsInfo*>(ivec[i]);
      t2 = ii->tchanges.at(t2);
    }else{
      assert(dynamic_cast<const TsoFenceSync::InsInfo*>(ivec[i]));
      const TsoFenceSync::InsInfo *ii = static_cast<const TsoFenceSync::InsInfo*>(ivec[i]);
      t2 = ii->tchanges.at(t2);
    }
  }
  return t2;
};

Machine *TsoLockSync::insert(const Machine &m, const std::vector<const Sync::InsInfo*> &m_infos, Sync::InsInfo **info) const{
  Machine::PTransition w2 = InsInfo::all_tchanges(m_infos,w);
  assert(w2.pid == w.pid);
  assert(w2.instruction.get_type() == Lang::WRITE);

  Machine *m2 = new Machine(m);

  Automaton::Transition tw(0,Lang::Stmt<int>::nop(),0);

  const Automaton::State &state = m2->automata[w2.pid].get_states()[w2.source];
  for(auto it = state.fwd_transitions.begin(); it != state.fwd_transitions.end(); ++it){
    if((*it)->compare(w2,false) == 0){
      tw = **it;
      break;
    }
  }

  Lang::Stmt<int> lw = Lang::Stmt<int>::locked_write(tw.instruction.get_memloc(),
                                                     tw.instruction.get_expr(),
                                                     tw.instruction.get_pos());

  m2->automata[w2.pid].del_transition(tw);
  m2->automata[w2.pid].add_transition(Automaton::Transition(tw.source,lw,tw.target));

  InsInfo *my_info = new InsInfo(static_cast<TsoLockSync*>(clone()));
  *info = my_info;
  for(unsigned p = 0; p < m.automata.size(); ++p){
    const std::vector<Automaton::State> &states = m.automata[p].get_states();
    for(unsigned i = 0; i < states.size(); ++i){
      for(auto it = states[i].fwd_transitions.begin(); it != states[i].fwd_transitions.end(); ++it){
        my_info->bind(Machine::PTransition(**it,p),Machine::PTransition(**it,p));
      }
    }
  }
  my_info->bind(w2,Machine::PTransition(tw.source,lw,tw.target,w2.pid));

  return m2;
};

Sync *TsoLockSync::clone() const{
  return new TsoLockSync(w);
};

std::string TsoLockSync::to_raw_string() const{
  return to_string_aux(Lang::int_reg_to_string(),Lang::int_memloc_to_string());
};

std::string TsoLockSync::to_string(const Machine &m) const{
  return to_string_aux(m.reg_pretty_vts(w.pid),m.ml_pretty_vts(w.pid));
};

std::string TsoLockSync::to_string_aux(const std::function<std::string(const int&)> &regts, 
                                       const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const{
  return "Lock write: "+w.to_string(regts,mlts);
};

int TsoLockSync::compare(const Sync &s) const{
  assert(dynamic_cast<const TsoLockSync*>(&s));
  const TsoLockSync *ls = static_cast<const TsoLockSync*>(&s);

  return w.compare(ls->w,false);
};

std::set<Sync*> TsoLockSync::get_all_possible(const Machine &m){
  std::set<Sync*> S;

  for(unsigned p = 0; p < m.automata.size(); ++p){
    const std::vector<Automaton::State> &states = m.automata[p].get_states();
    for(unsigned i = 0; i < states.size(); ++i){
      for(auto it = states[i].fwd_transitions.begin(); it != states[i].fwd_transitions.end(); ++it){
        if((*it)->instruction.get_type() == Lang::WRITE){
          S.insert(new TsoLockSync(Machine::PTransition(**it,p)));
        }
      }
    }
  }

  return S;
};

void TsoLockSync::test(){
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

  /* Test insert */
  {

    /* Test 1 */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "  L0: write: x := 1;"
         "  L1: nop\n"
         "process\n"
         "text\n"
         "  L0: locked write: x := 1;"
         "  L1: nop\n"
         );

      TsoLockSync tls(trans(m,0,"L0","write: x := 1","L1"));

      Sync::InsInfo *info;
      Machine *m2 = tls.insert(*m,std::vector<const Sync::InsInfo*>(),&info); delete info;

      Test::inner_test("insert #1",
                       m->automata[1].same_automaton(m2->automata[0],false));

      delete m;
      delete m2;
    }

    /* Test 2 */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "either{\n"
         "  nop;\n"
         "  write: x := 1\n"
         "or\n"
         "  nop;\n"
         "  write: y := 1\n"
         "};\n"
         "L0: either{\n"
         "  write: x := 1;\n"
         "  L1: nop\n"
         "or\n"
         "  write: x := 1;\n"
         "  nop"
         "}"
         "process\n"
         "text\n"
         "either{\n"
         "  nop;\n"
         "  write: x := 1\n"
         "or\n"
         "  nop;\n"
         "  write: y := 1\n"
         "};\n"
         "L0: either{\n"
         "  locked write: x := 1;\n"
         "  L1: nop\n"
         "or\n"
         "  write: x := 1;\n"
         "  nop"
         "}"
         );

      TsoLockSync tls(trans(m,0,"L0","write: x := 1","L1"));

      Sync::InsInfo *info;
      Machine *m2 = tls.insert(*m,std::vector<const Sync::InsInfo*>(),&info); delete info;

      Test::inner_test("insert #2",
                       m->automata[1].same_automaton(m2->automata[0],false));

      delete m;
      delete m2;
    }

    /* Test 3,4,5: compatibility with TsoFenceSync */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "process\n"
         "text\n"
         "either{\n"
         "  nop;\n"
         "  L0: write: x := 1\n"
         "or\n"
         "  nop;\n"
         "  L1: write: y := 1\n"
         "};\n"
         "L2: either{\n"
         "  write: x := 1;\n"
         "  L3: nop\n"
         "or\n"
         "  write: x := 1;\n"
         "  L4: nop\n"
         "};\n"
         "L5: nop"
         );

      Machine *m2 = get_machine
        ("forbidden * *\n"
         "data\n"
         "  x = *\n"
         "  y = *\n"
         "  z = *\n"
         "  tsofence = 0 : [0:0]\n"
         "process\n"
         "text\n"
         "either{\n"
         "  nop;\n"
         "  L0: write: x := 1\n"
         "or\n"
         "  nop;\n"
         "  L1: write: y := 1\n"
         "};\n"
         "locked write: tsofence := 0;\n"
         "L2: either{\n"
         "  locked write: x := 1;\n"
         "  L3: nop\n"
         "or\n"
         "  write: x := 1;\n"
         "  L4: nop\n"
         "};\n"
         "L5: nop\n"
         "process\n"
         "text\n"
         "either{\n"
         "  nop;\n"
         "  L0: write: x := 1\n"
         "or\n"
         "  nop;\n"
         "  L1: write: y := 1\n"
         "};\n"
         "locked write: tsofence := 0;\n"
         "L2: either{\n"
         "  locked write: x := 1;\n"
         "  locked write: tsofence := 0;\n"
         "  L3: nop\n"
         "or\n"
         "  write: x := 1;\n"
         "  L4: nop\n"
         "};\n"
         "L5: nop\n"
         );

      std::function<std::set<Automaton::Transition>(const Machine *,int,std::string)> get_trans_set = 
        [&trans](const Machine *m,int pid,std::string T_s){
        std::set<Automaton::Transition> tset;

        std::size_t i = 0;
        while(i != std::string::npos){
          std::size_t j = T_s.find("\n",i);
          std::string s = T_s.substr(i,(j == std::string::npos) ? std::string::npos : (j-i));
          /* Format of s: (src_lbl,instr,tgt_lbl) */

          /* Extract src_lbl */
          std::string src_lbl;
          int k = 0;
          while(std::isspace(s[k])) ++k;
          assert(s[k] == '('); ++k;
          while(std::isspace(s[k])) ++k;
          while(!std::isspace(s[k]) && s[k] != ','){
            src_lbl += s[k];
            ++k;
          };
          while(std::isspace(s[k])) ++k;
          assert(s[k] == ','); ++k;
          while(std::isspace(s[k])) ++k;

          /* Extract tgt_lbl */
          std::string tgt_lbl;
          int l = s.size() - 1;
          while(std::isspace(s[l])) --l;
          assert(s[l] == ')'); --l;
          while(std::isspace(s[l])) --l;
          while(!std::isspace(s[l]) && s[l] != ','){
            tgt_lbl = s[l]+tgt_lbl;
            --l;
          };
          while(std::isspace(s[l])) --l;
          assert(s[l] == ','); --l;
          while(std::isspace(s[l])) --l;

          /* Extract instr */
          assert(k < l);
          std::string instr = s.substr(k,l-k+1);
          
          tset.insert(trans(m,pid,src_lbl,instr,tgt_lbl));

          if(j == std::string::npos){
            i = std::string::npos;
          }else{
            i = j+1;
          }
        }

        return tset;
      };

      std::function<TsoFenceSync*(const Machine*,int,std::string,std::string,std::string)> get_tfs = 
        [&get_trans_set,&cs](const Machine *m,int pid, std::string q_lbl,std::string IN_s, std::string OUT_s){
        return new TsoFenceSync(pid,cs(m,pid,q_lbl),
                                get_trans_set(m,pid,IN_s),
                                get_trans_set(m,pid,OUT_s));
      };

      TsoFenceSync *tfs = get_tfs(m,0,"L2",
                                  "(L0,write: x := 1,L2)\n(L1,write: y := 1,L2)",
                                  "(L2,write: x := 1,L3)\n(L2,write: x := 1,L4)");

      TsoFenceSync *tfs2 = get_tfs(m,0,"L3",
                                   "(L2,write: x := 1,L3)",
                                   "(L3,nop,L5)");

      TsoLockSync tls(trans(m,0,"L2","write: x := 1","L3"));

      Sync::InsInfo *info;
      Machine *m3 = tfs->insert(*m,std::vector<const Sync::InsInfo*>(),&info);
      std::vector<const Sync::InsInfo*> m_infos(1,info);

      Machine *m4 = tls.insert(*m3,m_infos,&info); delete info;

      Test::inner_test("insert #3",
                       m2->automata[0].same_automaton(m4->automata[0],false));
      for(unsigned i = 0; i < m_infos.size(); ++i){
        delete m_infos[i];
      }
      m_infos.clear();
      delete m3;
      delete m4;

      /* Try the same test, but with syncs inserted in opposite order */
      m3 = tls.insert(*m,std::vector<const Sync::InsInfo*>(),&info);
      m_infos.push_back(info);
      m4 = tfs->insert(*m3,m_infos,&info); delete info;
      Test::inner_test("insert #4",
                       m2->automata[0].same_automaton(m4->automata[0],false));
      for(unsigned i = 0; i < m_infos.size(); ++i){
        delete m_infos[i];
      }
      m_infos.clear();
      delete m3;
      delete m4;

      /* Try with fences both before and after the lock */
      m3 = tfs->insert(*m,std::vector<const Sync::InsInfo*>(),&info);
      m_infos.push_back(info);
      m4 = tfs2->insert(*m3,m_infos,&info);
      m_infos.push_back(info);
      Machine *m5 = tls.insert(*m4,m_infos,&info); delete info;
      Test::inner_test("insert #5",
                       m2->automata[1].same_automaton(m5->automata[0],false));
      for(unsigned i = 0; i < m_infos.size(); ++i){
        delete m_infos[i];
      }
      m_infos.clear();
      delete m3;
      delete m4;
      delete m5;

      delete m;
      delete m2;
      delete tfs;
      delete tfs2;
    }

  }
};
