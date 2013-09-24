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

#include "vips_fence_sync.h"

#include <functional>

VipsFenceSync::VipsFenceSync(int pid, int q, TSet IN, TSet OUT)
  : FenceSync(Lang::Stmt<int>::full_fence(),pid,q,IN,OUT) {
};

VipsFenceSync::~VipsFenceSync(){};

Sync *VipsFenceSync::clone() const{
  return new VipsFenceSync(*this);
};

std::set<Sync*> VipsFenceSync::get_all_possible(const Machine &m){
  std::set<Lang::Stmt<int> > fs;
  fs.insert(Lang::Stmt<int>::nop()); // Does not matter since fsinit will provide the right fence instruction
  FenceSync::fs_init_t fsinit =
    [](Lang::Stmt<int> f, int pid, int q,
       TSet IN,
       TSet OUT){
    return new VipsFenceSync(pid,q,IN,OUT);
  };
  return FenceSync::get_all_possible(m,fs,fsinit);
};

std::string VipsFenceSync::to_string(const Machine &m) const{
  return print_to_string(m,0,0);
};

void VipsFenceSync::print(const Machine &m, Log::redirection_stream &os, Log::redirection_stream &json_os) const{
  print_to_string(m,&os,&json_os);
};

std::string VipsFenceSync::print_to_string(const Machine &m, Log::redirection_stream *os, Log::redirection_stream *json_os) const{
  std::function<std::string(const Automaton::Transition&)> tts =
    [&m,this](const Automaton::Transition &t){
    return Machine::PTransition(t,this->pid).to_string(m);
  };
  std::function<std::string(const Lang::Stmt<int>&)> sts =
    [&m,this](const Lang::Stmt<int> &s){
    return s.to_string(m.reg_pretty_vts(this->pid),
                       m.ml_pretty_vts(this->pid));
  };
  std::stringstream ss;
  ss << "Insert " << sts(f) << " for P" << pid << " at Q" << q << "\n";
  if(os){
    (*os) << "Insert " << sts(f) << " for P" << pid << " at Q" << q << "\n";
  }
  for(auto it = IN.begin(); it != IN.end(); ++it){
    ss << "  IN: " << tts(*it) << "\n";
    if(os){
      (*os) << "  IN: " << tts(*it) << "\n";
    }
    if(os && json_os && *os->os && it->instruction.get_pos().get_line_no() >= 0){
      (*json_os) << "json: {\"action\":\"Link Fence\", \"pos\":" << it->instruction.get_pos().to_json() << "}\n";
    }
  }
  for(auto it = OUT.begin(); it != OUT.end(); ++it){
    ss << "  OUT: " << tts(*it) << "\n";
    if(os){
      (*os) << "  OUT: " << tts(*it) << "\n";
    }
    if(os && json_os && *os->os && it->instruction.get_pos().get_line_no() >= 0){
      (*json_os) << "json: {\"action\":\"Link Fence\", \"pos\":" << it->instruction.get_pos().to_json() << "}\n";
    }
  }
  return ss.str();
};
