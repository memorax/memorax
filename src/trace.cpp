/*
 * Copyright (C) 2012 Carl Leonardsson
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

#include "trace.h"

Trace::Trace(Constraint *c0){
  trace_vec.push_back(trace_elem_t(0,c0));
}

Trace::~Trace(){
  for(unsigned i = 0; i < trace_vec.size(); i++){
    if(trace_vec[i].trans) delete trace_vec[i].trans;
    if(trace_vec[i].constr) delete trace_vec[i].constr;
  }
}

void Trace::push_back(const Machine::PTransition &t, Constraint *c){
  trace_vec.push_back(trace_elem_t(new Machine::PTransition(t),c));
}

void Trace::push_front(Constraint *c, const Machine::PTransition &t){
  /* Shift all trace elements one step to the right */
  trace_vec.push_back(trace_elem_t(0,0)); // Will be overwritten
  for(unsigned i = trace_vec.size()-1; i > 0; i--){
    trace_vec[i] = trace_vec[i-1];
  }
  trace_vec[0] = trace_elem_t(0,c);
  assert(trace_vec[1].trans == 0);
  trace_vec[1].trans = new Machine::PTransition(t);
}

int Trace::get_proc_count() const throw(){
  int pc = 0;
  for(unsigned t = 1; t < trace_vec.size(); t++){
    pc = std::max(pc,trace_vec[t].trans->pid+1);
  }
  return pc;
}

std::string Trace::to_string(const Machine &m, bool include_constraints, bool proc_indent) const{
  std::function<std::string(Machine::PTransition*)> tts = 
    [&m](Machine::PTransition *t){ return t->to_string(m); };
  return to_string(tts, 0,0,0, include_constraints, proc_indent);
};

std::string Trace::to_string(bool include_constraints, bool proc_indent) const{
  std::function<std::string(Machine::PTransition*)> tts = 
    [](Machine::PTransition *t){
    return t->to_string(Lang::int_reg_to_string(),
                        Lang::int_memloc_to_string());
  };
  return to_string(tts, 0,0,0, include_constraints, proc_indent);
};

void Trace::print(Log::redirection_stream &trans_os, Log::redirection_stream &constr_os, Log::redirection_stream &json_os,
                  const Machine &m, bool include_constraints, bool proc_indent) const{
  std::function<std::string(Machine::PTransition*)> tts = 
    [&m](Machine::PTransition *t){ return t->to_string(m); };
  to_string(tts, &trans_os, &constr_os, &json_os, include_constraints, proc_indent);
};

void Trace::print(Log::redirection_stream &trans_os, Log::redirection_stream &constr_os, Log::redirection_stream &json_os,
                  bool include_constraints, bool proc_indent) const{
  std::function<std::string(Machine::PTransition*)> tts = 
    [](Machine::PTransition *t){
    return t->to_string(Lang::int_reg_to_string(),
                        Lang::int_memloc_to_string());
  };
  to_string(tts, &trans_os, &constr_os, &json_os, include_constraints, proc_indent);
};

std::string Trace::to_string(std::function<std::string(Machine::PTransition*)> &tts,
                             Log::redirection_stream *trans_os, Log::redirection_stream *constr_os, Log::redirection_stream *json_os,
                             bool include_constraints, bool proc_indent) const{
  std::string s;

  if(include_constraints && trace_vec[0].constr){
    s = trace_vec[0].constr->to_string() + "\n";
    if(constr_os) (*constr_os) << trace_vec[0].constr->to_string() << "\n";
  }
  for(unsigned i = 1; i < trace_vec.size(); i++){
    std::string ind;
    if(proc_indent){
      for(int j = 0; j < trace_vec[i].trans->pid; ++j){
        ind += "    ";
      }
    }
    s += ind+tts(trace_vec[i].trans)+"\n";
    if(trans_os) (*trans_os) << ind << tts(trace_vec[i].trans) << "\n";
    if(trans_os && *trans_os->os && json_os && trace_vec[i].trans->instruction.get_pos().lineno >= 0){
      (*json_os) << "json: {\"action\":\"Link Fence\", \"pos\":{"
                 << "\"lineno\":" << trace_vec[i].trans->instruction.get_pos().lineno
                 << ", \"charno\":" << trace_vec[i].trans->instruction.get_pos().charno
                 << "}}\n";
    }
    if(include_constraints && trace_vec[i].constr){
      std::string t = trace_vec[i].constr->to_string();
      std::size_t pos = -1;
      while((pos = t.find("\n",pos+1)) != std::string::npos){
        t.replace(pos,1,"\n"+ind);
      }
      s += ind + t + "\n";
      if(constr_os) (*constr_os) << ind << t << "\n";
    }
  }

  return s;
};
