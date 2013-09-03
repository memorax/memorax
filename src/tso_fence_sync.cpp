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

#include <sstream>
#include <stdexcept>

TsoFenceSync::InsInfo::InsInfo(const FenceSync::InsInfo &fs_info, const Lang::NML &nml)
  : FenceSync::InsInfo(fs_info), fence_nml(nml){
};

TsoFenceSync::InsInfo::~InsInfo() {};

TsoFenceSync::TsoFenceSync(int pid, int q, TSet IN, TSet OUT)
  : FenceSync(Lang::Stmt<int>::nop(),pid,q,IN,OUT)
{ 
};

TsoFenceSync::~TsoFenceSync(){
};

Machine *TsoFenceSync::insert(const Machine &m, const std::vector<const Sync::InsInfo*> &m_infos, Sync::InsInfo **info) const{
  Machine *m2;
  Lang::NML nml = Lang::NML::global(0);
  {
    auto pr = get_dummy_nml(m,m_infos);
    m2 = pr.first;
    nml = pr.second;
  }

  /* Create a temporary object where the fence can be set to the
   * correct instruction, then perform the insertuion using the
   * temporary TsoFenceSync. */
  TsoFenceSync tfs(pid,q,IN,OUT);
  assert(m2->get_var_decl(nml).domain.is_finite());
  assert(m2->get_var_decl(nml).domain.get_lower_bound() == 
         m2->get_var_decl(nml).domain.get_upper_bound());
  int val = m2->get_var_decl(nml).domain.get_lower_bound();
  tfs.f = Lang::Stmt<int>::locked_write(nml.localize(pid),
                                        Lang::Expr<int>::integer(val));
  Sync::InsInfo *fs_info;
  Machine *m3 = tfs.FenceSync::insert(*m2,m_infos,&fs_info);
  delete m2;

  *info = new InsInfo(*(FenceSync::InsInfo*)fs_info,nml);
  delete fs_info;
  return m3;
};

Sync *TsoFenceSync::clone() const{
  return new TsoFenceSync(pid,q,IN,OUT);
};

std::string TsoFenceSync::to_raw_string() const{
  return to_string_aux(Lang::int_reg_to_string(),
                       Lang::int_memloc_to_string());
};

std::string TsoFenceSync::to_string(const Machine &m) const{
  return to_string_aux(m.reg_pretty_vts(pid),m.ml_pretty_vts(pid));
};

std::string TsoFenceSync::to_string_aux(const std::function<std::string(const int&)> &regts, 
                                        const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const{
  std::stringstream ss;
  ss << "TsoFenceSync(P" << pid << ",Q" << q << ")\n";
  for(auto it = IN.begin(); it != IN.end(); ++it){
    ss << "IN: " << it->to_string(regts,mlts) << "\n";
  }
  for(auto it = OUT.begin(); it != OUT.end(); ++it){
    ss << "OUT: " << it->to_string(regts,mlts) << "\n";
  }
  return ss.str();
};

void TsoFenceSync::print_raw(Log::redirection_stream &os, Log::redirection_stream &json_os) const{
  print_aux(Lang::int_reg_to_string(),
            Lang::int_memloc_to_string(),
            os,json_os);
};

void TsoFenceSync::print(const Machine &m, Log::redirection_stream &os, Log::redirection_stream &json_os) const{
  print_aux(m.reg_pretty_vts(pid),m.ml_pretty_vts(pid),
            os,json_os);
};

void TsoFenceSync::print_aux(const std::function<std::string(const int&)> &regts, 
                             const std::function<std::string(const Lang::MemLoc<int> &)> &mlts,
                             Log::redirection_stream &os, Log::redirection_stream &json_os) const{
  os << "TsoFenceSync(P" << pid << ",Q" << q << ")\n";
  for(auto it = IN.begin(); it != IN.end(); ++it){
    os << "IN: " << it->to_string(regts,mlts) << "\n";
    if(*os.os && it->instruction.get_pos().get_line_no() >= 0){
      json_os << "json: {\"action\":\"Link Fence\", \"pos\":" << it->instruction.get_pos().to_json() << "}\n";
    }
  }
  for(auto it = OUT.begin(); it != OUT.end(); ++it){
    os << "OUT: " << it->to_string(regts,mlts) << "\n";
    if(*os.os && it->instruction.get_pos().get_line_no() >= 0){
      json_os << "json: {\"action\":\"Link Fence\", \"pos\":" << it->instruction.get_pos().to_json() << "}\n";
    }
  }
};

std::set<Sync*> TsoFenceSync::get_all_possible(const Machine &m){
  std::set<Lang::Stmt<int> > fs;
  fs.insert(Lang::Stmt<int>::nop()); // Does not matter
  FenceSync::fs_init_t fsinit = 
    [](Lang::Stmt<int> f, int pid, int q, TSet IN, TSet OUT){
    return new TsoFenceSync(pid,q,IN,OUT);
  };
  return FenceSync::get_all_possible(m,fs,fsinit);
};

std::pair<Machine*,Lang::NML> TsoFenceSync::get_dummy_nml(const Machine &m, const std::vector<const Sync::InsInfo*> &m_infos){
  Machine *m2 = new Machine(m);

  /* Check m_infos */
  for(unsigned i = 0; i < m_infos.size(); ++i){
    const TsoFenceSync::InsInfo *info = 
      dynamic_cast<const TsoFenceSync::InsInfo*>(m_infos[i]);
    if(info){
      return std::pair<Machine*,Lang::NML>(m2,info->fence_nml);
    }
  }

  /* Must create a new NML */
  std::string name;
  {
    std::set<std::string> ml_names; // all memory location names in m
    for(unsigned i = 0; i < m.gvars.size(); ++i){
      ml_names.insert(m.gvars[i].name);
    }
    for(unsigned p = 0; p < m.lvars.size(); ++p){
      for(unsigned i = 0; i < m.lvars[p].size(); ++i){
        ml_names.insert(m.lvars[p][i].name);
      }
    }
    unsigned i = 0;
    name = "tsofence";
    while(ml_names.count(name)){
      ++i;
      std::stringstream ss;
      ss << "tsofence" << i;
      name = ss.str();
    }
  }
  m2->gvars.push_back(Lang::VarDecl(name,Lang::Value(0),Lang::VarDecl::Domain(0,0))); // name = 0 : [0:0]
  m2->pretty_string_nml[Lang::NML::global(m2->gvars.size() - 1)] = name;

  return std::pair<Machine*,Lang::NML>(m2,Lang::NML::global(m2->gvars.size() - 1));
};

void TsoFenceSync::test(){

  std::function<Machine*(std::string)> get_machine = 
    [](std::string rmm){
    std::stringstream ss(rmm);
    PPLexer pp(ss);
    return new Machine(Parser::p_test(pp));
  };

  /* Test get_dummy_nml */
  {
    /* Test 1 */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  x = 0 : [0:1]\n"
         "  y = *\n"
         "process\n"
         "data\n"
         "  flag = *\n"
         "text nop\n"
         "process\n"
         "data\n"
         "  flag = *\n"
         "text nop\n"
         );

      auto pr = get_dummy_nml(*m,std::vector<const Sync::InsInfo*>());

      Test::inner_test("get_dummy_nml #1",
                       pr.second == Lang::NML::global(2) &&
                       pr.first->gvars.size() == 3 &&
                       pr.first->gvars[2].name == "tsofence" &&
                       m->automata[0].same_automaton(pr.first->automata[0],true) &&
                       m->automata[1].same_automaton(pr.first->automata[1],true)
                       );

      delete pr.first;
      delete m;
    }

    /* Test 2 */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  x = 0 : [0:1]\n"
         "  y = *\n"
         "  tsofence = *\n"
         "  tsofence1 = *\n"
         "process\n"
         "data\n"
         "  flag = *\n"
         "text nop\n"
         "process\n"
         "data\n"
         "  flag = *\n"
         "text nop\n"
         );

      auto pr = get_dummy_nml(*m,std::vector<const Sync::InsInfo*>());

      Test::inner_test("get_dummy_nml #2",
                       pr.second == Lang::NML::global(4) &&
                       pr.first->gvars.size() == 5 &&
                       pr.first->gvars[4].name == "tsofence2" &&
                       m->automata[0].same_automaton(pr.first->automata[0],true) &&
                       m->automata[1].same_automaton(pr.first->automata[1],true)
                       );

      delete pr.first;
      delete m;
    }

    /* Test 3 */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  x = 0 : [0:1]\n"
         "  y = * : [0:0]\n"
         "  tsofence = *\n"
         "  tsofence1 = *\n"
         "process\n"
         "data\n"
         "  flag = *\n"
         "text nop; nop\n"
         "process\n"
         "data\n"
         "  flag = *\n"
         "text nop\n"
         );

      InsInfo info(FenceSync::InsInfo(0),Lang::NML::global(1));
      auto pr = get_dummy_nml(*m,std::vector<const Sync::InsInfo*>(1,(const Sync::InsInfo*)&info));

      Test::inner_test("get_dummy_nml #3",
                       pr.second == Lang::NML::global(1) &&
                       pr.first->gvars.size() == 4 &&
                       m->automata[0].same_automaton(pr.first->automata[0],true) &&
                       m->automata[1].same_automaton(pr.first->automata[1],true)
                       );

      delete pr.first;
      delete m;
    }
  }

  /* Get a TsoFenceSync for m at the control state labeled lbl of
   * process pid. Use maximal sets for IN and OUT.
   */
  std::function<TsoFenceSync(const Machine*,int,std::string)> get_tfs =
    [](const Machine *m, int pid, std::string lbl){
    int q = m->automata[pid].state_index_of_label(lbl);
    TSet IN, OUT;
    const Automaton::State &qstate = m->automata[pid].get_states()[q];
    for(auto it = qstate.fwd_transitions.begin(); it != qstate.fwd_transitions.end(); ++it){
      OUT.insert(**it);
    }
    for(auto it = qstate.bwd_transitions.begin(); it != qstate.bwd_transitions.end(); ++it){
      IN.insert(**it);
    }
    return TsoFenceSync(pid,q,IN,OUT);
  };

  /* Test insert */
  {
    /* Test 1 */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  x = *\n"
         "process\n"
         "data\n"
         "  flag = *\n"
         "text\n"
         "  write: x := 1;\n"
         "  write: x := 2;\n"
         "  F:\n"
         "  write: x := 3;\n"
         "  write: x := 4\n"
         );

      Machine *m_tgt = get_machine
        ("forbidden *\n"
         "data\n"
         "  x = *\n"
         "  tsofence = 0 : [0:0]\n"
         "process\n"
         "data\n"
         "  flag = *\n"
         "text\n"
         "  write: x := 1;\n"
         "  write: x := 2;\n"
         "  locked write: tsofence := 0;\n"
         "  F:\n"
         "  write: x := 3;\n"
         "  write: x := 4\n"
         );

      TsoFenceSync tfs = get_tfs(m,0,"F");

      Sync::InsInfo *info;
      Machine *m2 = tfs.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #1",
                       m_tgt->automata[0].same_automaton(m2->automata[0],false) &&
                       m2->gvars.size() == 2);

      delete info;
      delete m_tgt;
      delete m2;
      delete m;
    }
  }
};
