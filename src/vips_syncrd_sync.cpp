/*
 * Copyright (C) 2014 Carl Leonardsson
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
#include "vips_fence_sync.h"
#include "vips_syncrd_sync.h"
#include "vips_syncwr_sync.h"

#include <cassert>
#include <cctype>
#include <set>

VipsSyncrdSync::VipsSyncrdSync(const Machine::PTransition &r) : r(r) {};

VipsSyncrdSync::InsInfo::InsInfo(const VipsSyncrdSync *creator_copy) : Sync::InsInfo(creator_copy) {};

void VipsSyncrdSync::InsInfo::bind(const Machine::PTransition &a,const Machine::PTransition &b){
  auto res = tchanges.insert(std::pair<Machine::PTransition,Machine::PTransition>(a,b));
  if(!res.second){
    tchanges.at(a) = b;
  }
};

const Machine::PTransition &VipsSyncrdSync::InsInfo::operator[](const Machine::PTransition &t) const{
  return tchanges.at(t);
};

Machine::PTransition VipsSyncrdSync::InsInfo::all_tchanges(const std::vector<const Sync::InsInfo*> &ivec,
                                                           const Machine::PTransition &t){
  Machine::PTransition t2 = t;
  for(unsigned i = 0; i < ivec.size(); ++i){
    if(dynamic_cast<const InsInfo*>(ivec[i])){
      const InsInfo *ii = static_cast<const InsInfo*>(ivec[i]);
      t2 = ii->tchanges.at(t2);
    }else if(dynamic_cast<const VipsSyncwrSync::InsInfo*>(ivec[i])){
      const VipsSyncwrSync::InsInfo *ii = static_cast<const VipsSyncwrSync::InsInfo*>(ivec[i]);
      t2 = ii->tchanges.at(t2);
    }else{
      assert(dynamic_cast<const FenceSync::InsInfo*>(ivec[i]));
      const FenceSync::InsInfo *ii = static_cast<const FenceSync::InsInfo*>(ivec[i]);
      t2 = ii->tchanges.at(t2);
    }
  }
  return t2;
};

Machine *VipsSyncrdSync::insert(const Machine &m, const std::vector<const Sync::InsInfo*> &m_infos, Sync::InsInfo **info) const{
  Machine::PTransition r2 = InsInfo::all_tchanges(m_infos,r);
  assert(r2.pid == r.pid);
  assert(r2.instruction.get_type() == r.instruction.get_type());

  Machine *m2 = new Machine(m);

  Automaton::Transition tr(0,Lang::Stmt<int>::nop(),0);

  const Automaton::State &state = m2->automata[r2.pid].get_states()[r2.source];
  for(auto it = state.fwd_transitions.begin(); it != state.fwd_transitions.end(); ++it){
    if((*it)->compare(r2,false) == 0){
      tr = **it;
      break;
    }
  }

  Lang::Stmt<int> sr = Lang::Stmt<int>::nop();
  switch(r.instruction.get_type()){
  case Lang::READASSERT:
    sr = Lang::Stmt<int>::syncrd_assert(tr.instruction.get_memloc(),
                                        tr.instruction.get_expr(),
                                        tr.instruction.get_pos());
    break;
  case Lang::READASSIGN:
    sr = Lang::Stmt<int>::syncrd_assign(tr.instruction.get_reg(),
                                        tr.instruction.get_memloc(),
                                        tr.instruction.get_pos());
    break;
  default:
    throw new std::logic_error("VipsSyncrdSync::insert: Instruction is not a read.");
  }

  m2->automata[r2.pid].del_transition(tr);
  m2->automata[r2.pid].add_transition(Automaton::Transition(tr.source,sr,tr.target));

  InsInfo *my_info = new InsInfo(static_cast<VipsSyncrdSync*>(clone()));
  *info = my_info;
  for(unsigned p = 0; p < m.automata.size(); ++p){
    const std::vector<Automaton::State> &states = m.automata[p].get_states();
    for(unsigned i = 0; i < states.size(); ++i){
      for(auto it = states[i].fwd_transitions.begin(); it != states[i].fwd_transitions.end(); ++it){
        my_info->bind(Machine::PTransition(**it,p),Machine::PTransition(**it,p));
      }
    }
  }
  my_info->bind(r2,Machine::PTransition(tr.source,sr,tr.target,r2.pid));

  return m2;
};

Sync *VipsSyncrdSync::clone() const{
  return new VipsSyncrdSync(r);
};

std::string VipsSyncrdSync::to_raw_string() const{
  return to_string_aux(Lang::int_reg_to_string(),Lang::int_memloc_to_string());
};

std::string VipsSyncrdSync::to_string(const Machine &m) const{
  return to_string_aux(m.reg_pretty_vts(r.pid),m.ml_pretty_vts(r.pid));
};

std::string VipsSyncrdSync::to_string_aux(const std::function<std::string(const int&)> &regts,
                                          const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const{
  return "Make read into syncrd: "+r.to_string(regts,mlts)+"\n";
};

void VipsSyncrdSync::print_raw(Log::redirection_stream &os, Log::redirection_stream &json_os) const{
  os << to_raw_string() << "\n";
  if(*os.os && r.instruction.get_pos().get_line_no() >= 0){
    json_os << "json: {\"action\":\"Link Fence\", \"pos\":" << r.instruction.get_pos().to_json() << "}\n";
  }
};

void VipsSyncrdSync::print(const Machine &m, Log::redirection_stream &os, Log::redirection_stream &json_os) const{
  os << to_string(m);
  if(*os.os && r.instruction.get_pos().get_line_no() >= 0){
    json_os << "json: {\"action\":\"Link Fence\", \"pos\":" << r.instruction.get_pos().to_json() << "}\n";
  }
};

int VipsSyncrdSync::compare(const Sync &s) const{
  assert(dynamic_cast<const VipsSyncrdSync*>(&s));
  const VipsSyncrdSync *ls = static_cast<const VipsSyncrdSync*>(&s);

  return r.compare(ls->r,false);
};

std::set<Sync*> VipsSyncrdSync::get_all_possible(const Machine &m){
  std::set<Sync*> S;

  for(unsigned p = 0; p < m.automata.size(); ++p){
    const std::vector<Automaton::State> &states = m.automata[p].get_states();
    for(unsigned i = 0; i < states.size(); ++i){
      for(auto it = states[i].fwd_transitions.begin(); it != states[i].fwd_transitions.end(); ++it){
        if((*it)->instruction.get_type() == Lang::READASSERT ||
           (*it)->instruction.get_type() == Lang::READASSIGN){
          S.insert(new VipsSyncrdSync(Machine::PTransition(**it,p)));
        }
      }
    }
  }

  return S;
};

void VipsSyncrdSync::test(){
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
     "registers\n"
     "  $r0 = *\n"
     "  $r1 = *\n"
     "  $r2 = *\n"
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
    throw new std::logic_error("trans: No such transition.");
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
         "  L0: read: x = 0;\n"
         "  L1: nop\n"
         "process\n"
         "text\n"
         "  L0: syncrd: x = 0;"
         "  L1: nop\n"
         );

      VipsSyncrdSync vss(trans(m,0,"L0","read: x = 0","L1"));

      Sync::InsInfo *info;
      Machine *m2 = vss.insert(*m,std::vector<const Sync::InsInfo*>(),&info); delete info;

      Test::inner_test("insert #1",
                       m->automata[1].same_automaton(m2->automata[0],false));

      delete m;
      delete m2;
    }

    /* Test 2 */
    {
      Machine *m = get_machine
        (R"(
forbidden * *
data
  x = *
  y = *
  z = *
process
registers
  $r0 = *
text
  L0: read: $r0 := x;
  L1: nop
process
registers
  $r0 = *
text
  L0: syncrd: $r0 := x;
  L1: nop)");

      VipsSyncrdSync vss(trans(m,0,"L0","read: $r0 := x","L1"));

      Sync::InsInfo *info;
      Machine *m2 = vss.insert(*m,std::vector<const Sync::InsInfo*>(),&info); delete info;

      Test::inner_test("insert #2",
                       m->automata[1].same_automaton(m2->automata[0],false));

      delete m;
      delete m2;
    }

    /* Test 3 */
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
         "  read: x = 1\n"
         "or\n"
         "  nop;\n"
         "  read: y = 1\n"
         "};\n"
         "L0: either{\n"
         "  read: x = 1;\n"
         "  L1: nop\n"
         "or\n"
         "  read: x = 1;\n"
         "  nop"
         "}"
         "process\n"
         "text\n"
         "either{\n"
         "  nop;\n"
         "  read: x = 1\n"
         "or\n"
         "  nop;\n"
         "  read: y = 1\n"
         "};\n"
         "L0: either{\n"
         "  syncrd: x = 1;\n"
         "  L1: nop\n"
         "or\n"
         "  read: x = 1;\n"
         "  nop"
         "}"
         );

      VipsSyncrdSync vss(trans(m,0,"L0","read: x = 1","L1"));

      Sync::InsInfo *info;
      Machine *m2 = vss.insert(*m,std::vector<const Sync::InsInfo*>(),&info); delete info;

      Test::inner_test("insert #3",
                       m->automata[1].same_automaton(m2->automata[0],false));

      delete m;
      delete m2;
    }

    /* Test 4,5,6: compatibility with VipsFenceSync */
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
         "  L0: read: x = 1\n"
         "or\n"
         "  nop;\n"
         "  L1: read: y = 1\n"
         "};\n"
         "L2: either{\n"
         "  read: x = 1;\n"
         "  L3: nop\n"
         "or\n"
         "  read: x = 1;\n"
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
         "process\n"
         "text\n"
         "either{\n"
         "  nop;\n"
         "  L0: read: x = 1\n"
         "or\n"
         "  nop;\n"
         "  L1: read: y = 1\n"
         "};\n"
         "fence;\n"
         "L2: either{\n"
         "  syncrd: x = 1;\n"
         "  L3: nop\n"
         "or\n"
         "  read: x = 1;\n"
         "  L4: nop\n"
         "};\n"
         "L5: nop\n"
         "process\n"
         "text\n"
         "either{\n"
         "  nop;\n"
         "  L0: read: x = 1\n"
         "or\n"
         "  nop;\n"
         "  L1: read: y = 1\n"
         "};\n"
         "fence;\n"
         "L2: either{\n"
         "  syncrd: x = 1;\n"
         "  fence;\n"
         "  L3: nop\n"
         "or\n"
         "  read: x = 1;\n"
         "  L4: nop\n"
         "};\n"
         "L5: nop\n"
         );

      std::function<FenceSync::TSet(const Machine *,int,std::string)> get_trans_set =
        [&trans](const Machine *m,int pid,std::string T_s){
        FenceSync::TSet tset;

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

      std::function<VipsFenceSync*(const Machine*,int,std::string,std::string,std::string)> get_vfs =
        [&get_trans_set,&cs](const Machine *m,int pid, std::string q_lbl,std::string IN_s, std::string OUT_s){
        return new VipsFullFenceSync(pid,cs(m,pid,q_lbl),
                                     get_trans_set(m,pid,IN_s),
                                     get_trans_set(m,pid,OUT_s));
      };

      VipsFenceSync *vfs = get_vfs(m,0,"L2",
                                   "(L0,read: x = 1,L2)\n(L1,read: y = 1,L2)",
                                   "(L2,read: x = 1,L3)\n(L2,read: x = 1,L4)");

      VipsFenceSync *vfs2 = get_vfs(m,0,"L3",
                                    "(L2,read: x = 1,L3)",
                                    "(L3,nop,L5)");

      VipsSyncrdSync vss(trans(m,0,"L2","read: x = 1","L3"));

      Sync::InsInfo *info;
      Machine *m3 = vfs->insert(*m,std::vector<const Sync::InsInfo*>(),&info);
      std::vector<const Sync::InsInfo*> m_infos(1,info);

      Machine *m4 = vss.insert(*m3,m_infos,&info); delete info;

      Test::inner_test("insert #4",
                       m2->automata[0].same_automaton(m4->automata[0],false));
      for(unsigned i = 0; i < m_infos.size(); ++i){
        delete m_infos[i];
      }
      m_infos.clear();
      delete m3;
      delete m4;

      /* Try the same test, but with syncs inserted in opposite order */
      m3 = vss.insert(*m,std::vector<const Sync::InsInfo*>(),&info);
      m_infos.push_back(info);
      m4 = vfs->insert(*m3,m_infos,&info); delete info;
      Test::inner_test("insert #5",
                       m2->automata[0].same_automaton(m4->automata[0],false));
      for(unsigned i = 0; i < m_infos.size(); ++i){
        delete m_infos[i];
      }
      m_infos.clear();
      delete m3;
      delete m4;

      /* Try with fences both before and after the lock */
      m3 = vfs->insert(*m,std::vector<const Sync::InsInfo*>(),&info);
      m_infos.push_back(info);
      m4 = vfs2->insert(*m3,m_infos,&info);
      m_infos.push_back(info);
      Machine *m5 = vss.insert(*m4,m_infos,&info); delete info;
      Test::inner_test("insert #6",
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
      delete vfs;
      delete vfs2;
    }

  }
};
