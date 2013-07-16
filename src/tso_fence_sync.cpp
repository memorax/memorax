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

#include "test.h"
#include "tso_fence_sync.h"

#include <sstream>
#include <stdexcept>

TsoFenceSync::InsInfo::InsInfo(const FenceSync::InsInfo &fs_info, const Lang::NML &nml)
  : FenceSync::InsInfo(fs_info), fence_nml(nml){
};

TsoFenceSync::InsInfo::~InsInfo() {};

TsoFenceSync::TsoFenceSync(int pid, int q, 
                           std::set<Automaton::Transition> IN, 
                           std::set<Automaton::Transition> OUT)
  : FenceSync(Lang::Stmt<int>::nop(),pid,q,IN,OUT)
{ 
};

TsoFenceSync::~TsoFenceSync(){
};

Machine *TsoFenceSync::insert(const Machine &m, const std::vector<const Sync::InsInfo*> &m_infos, Sync::InsInfo **info) const{
  throw new std::logic_error("TsoFenceSync::insert: Not implemented.");
};

bool TsoFenceSync::prevents(const Trace &t, const std::vector<const Sync::InsInfo*> &m_infos) const{
  throw new std::logic_error("TsoFenceSync::prevents: Not implemented.");
};

FenceSync *TsoFenceSync::clone() const{
  throw new std::logic_error("TsoFenceSync::clone: Not implemented.");
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

std::set<Sync*> TsoFenceSync::get_all_possible(const Machine &m){
  std::set<Lang::Stmt<int> > fs;
  fs.insert(Lang::Stmt<int>::nop()); // Does not matter
  FenceSync::fs_init_t fsinit = 
    [](Lang::Stmt<int> f, int pid, int q, 
       std::set<Automaton::Transition> IN, 
       std::set<Automaton::Transition> OUT){
    return new TsoFenceSync(pid,q,IN,OUT);
  };
  return FenceSync::get_all_possible(m,fs,fsinit);
};

void TsoFenceSync::test(){
  Test::inner_test("Testing?",true);
};
