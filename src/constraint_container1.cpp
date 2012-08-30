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

#include "constraint_container1.h"

bool ConstraintContainer1::check_uniqueness(const std::set<Constraint*,valcmp> &s){
  for(std::set<Constraint*,valcmp>::const_iterator it = s.begin(); it != s.end(); it++){
    std::set<Constraint*,valcmp>::const_iterator it2 = it;
    it2++;
    std::string s_it = (*it)->to_string();
    for(; it2 != s.end(); it2++){
      if(s_it == (*it2)->to_string()){
        std::cout << "Duplicate constraints: \n\n";
        std::cout << s_it << "\n";
        std::cout << (*it2)->to_string() << "\n";
      }
    }
  }
  return true;
}

inline void ConstraintContainer1::push_on_q(Constraint *c){
  if(q_size == int(q.size())){
    q.resize(q_size*2);
  }
  q[q_size] = c;
  q_size++;
};

inline void ConstraintContainer1::push_on_edges(Constraint *p, const Machine::PTransition *t, Constraint *c){
  if(edge_count == int(edges.size())){
    edges.resize(edge_count*2);
  }
  edges[edge_count] = edge_t(p,t,c);
  edge_count++;
};

inline bool ConstraintContainer1::insert_constraint_in_f(Constraint *c){
  /*
  std::cout << "pcs_to_f.size: " << pcs_to_f.size() << "\n"
            << "proc_count: " << proc_count << "\n"
            << "max_state_count: " << max_state_count << "\n"
            << "pc_index(c): " << pc_index(c) << "\n";
  */
  assert(int(pcs_to_f.size()) > pc_index(c));
  if(pcs_to_f[pc_index(c)].insert(c).second){
    f_size++;
    return true;
  }else{
    return false;
  }
};

ConstraintContainer1::ConstraintContainer1(const Machine &m)
: machine(m), edges(EDGES_START_SIZE), edge_count(0), q(Q_START_SIZE), q_size(0), f_size(0){
  proc_count = m.proc_count();
  max_state_count = 0;
  for(int i = 0; i < proc_count; i++){
    max_state_count = std::max(max_state_count,int(m.automata[i].get_states().size()));
  }
  int sz = 1;
  for(int i = 0; i < proc_count; i++){
    sz *= max_state_count;
  }
  pcs_to_f.resize(sz);
};

inline int ConstraintContainer1::pc_index(const Constraint *c) const{
  int index = 0;
  int multiplier = 1;
  const std::vector<int> &pcs = c->get_control_states();
  // std::cout << "pcs: [";
  for(int i = 0; i < proc_count; i++){
    // std::cout << pcs[i] << ", ";
    index += pcs[i]*multiplier;
    multiplier *= max_state_count;
  }
  // std::cout << "]\n";
  return index;
};

Constraint *ConstraintContainer1::pop(){
  if(q_size == 0){
    return 0;
  }else{
    q_size--;
    return q[q_size];
  }
};

void ConstraintContainer1::insert_root(Constraint *r){
  if(insert_constraint_in_f(r)){
    push_on_q(r);
    f_size++;
  }else{
    delete r;
  }
};

void ConstraintContainer1::insert(Constraint *p, const Machine::PTransition *t, Constraint *c){
  if(insert_constraint_in_f(c)){
    push_on_q(c);
    f_size++;
    push_on_edges(p,t,c);
  }else{
    delete c;
  }
  /*
#ifndef NDEBUG
  for(unsigned i = 0; i < pcs_to_f.size(); i++){
    assert(check_uniqueness(pcs_to_f[i]));
  }
#endif
  */
};

Trace *ConstraintContainer1::clear_and_get_trace(Constraint *c){
  /* Create the trace */
  Trace *trace = new Trace(c);
  pcs_to_f[pc_index(c)].erase(c); // Remove from F to avoid later deallocation
  int i = edge_count-1; // Index into edges
  while(i >= 0){
    if(edges[i].child == c){
      c = edges[i].parent;
      pcs_to_f[pc_index(c)].erase(c); // Remove from F to avoid later deallocation
      trace->push_front(c,*edges[i].transition);
    }
    i--;
  }

  /* Clear */
  clear();

  return trace;
};

void ConstraintContainer1::clear(){
  for(unsigned i = 0; i < pcs_to_f.size(); i++){
    for(auto it = pcs_to_f[i].begin(); it != pcs_to_f[i].end(); it++){
      delete *it;
    }
    pcs_to_f[i].clear();
  }
  f_size = 0;
  edge_count = 0;
  q_size = 0;
};
