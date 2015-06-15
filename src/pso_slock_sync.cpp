/*
 * Copyright (C) 2015 Carl Leonardsson, Magnus Lång
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
#include "pso_slock_sync.h"

#include <cassert>
#include <cctype>
#include <set>

PsoSlockSync::PsoSlockSync(const Machine::PTransition &w) : w(w) {};

PsoSlockSync::InsInfo::InsInfo(const PsoSlockSync *creator_copy) : Sync::InsInfo(creator_copy) {};

void PsoSlockSync::InsInfo::bind(const Machine::PTransition &a,const Machine::PTransition &b){
  auto res = tchanges.insert(std::pair<Machine::PTransition,Machine::PTransition>(a,b));
  if(!res.second){
    tchanges.at(a) = b;
  }
};

const Machine::PTransition &PsoSlockSync::InsInfo::operator[](const Machine::PTransition &t) const{
  return tchanges.at(t);
};

Machine::PTransition PsoSlockSync::InsInfo::all_tchanges(const std::vector<const Sync::InsInfo*> &ivec,
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

Machine *PsoSlockSync::insert(const Machine &m, const std::vector<const Sync::InsInfo*> &m_infos, Sync::InsInfo **info) const{
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

  InsInfo *my_info = new InsInfo(static_cast<PsoSlockSync*>(clone()));
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

Sync *PsoSlockSync::clone() const{
  return new PsoSlockSync(w);
};

std::string PsoSlockSync::to_raw_string() const{
  return to_string_aux(Lang::int_reg_to_string(),Lang::int_memloc_to_string());
};

std::string PsoSlockSync::to_string(const Machine &m) const{
  return to_string_aux(m.reg_pretty_vts(w.pid),m.ml_pretty_vts(w.pid));
};

std::string PsoSlockSync::to_string_aux(const std::function<std::string(const int&)> &regts,
                                       const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const{
  return "Store-Store Lock write: "+w.to_string(regts,mlts);
};

void PsoSlockSync::print_raw(Log::redirection_stream &os, Log::redirection_stream &json_os) const{
  os << to_raw_string() << "\n";
  if(*os.os && w.instruction.get_pos().get_line_no() >= 0){
    json_os << "json: {\"action\":\"Link Fence\", \"pos\":" << w.instruction.get_pos().to_json() << "}\n";
  }
};

void PsoSlockSync::print(const Machine &m, Log::redirection_stream &os, Log::redirection_stream &json_os) const{
  os << to_string(m) << "\n";
  if(*os.os && w.instruction.get_pos().get_line_no() >= 0){
    json_os << "json: {\"action\":\"Link Fence\", \"pos\":" << w.instruction.get_pos().to_json() << "}\n";
  }
};

int PsoSlockSync::compare(const Sync &s) const{
  assert(dynamic_cast<const PsoSlockSync*>(&s));
  const PsoSlockSync *ls = static_cast<const PsoSlockSync*>(&s);

  return w.compare(ls->w,false);
};
