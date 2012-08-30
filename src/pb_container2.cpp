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

#include "pb_container2.h"

PbContainer2::PbContainer2(const Machine &m)
: machine(m),
  f_size(0), 
  clean_q(Q_SIZE), dirty_q(Q_SIZE), q_size(0), dummy_wrapper(0)
{
  prev_popped = &dummy_wrapper;
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

inline int PbContainer2::pc_index(PbConstraint *c) const{
  int index = 0;
  int multiplier = 1;
  const std::vector<int> &pcs = c->pcs;
  for(int i = 0; i < proc_count; i++){
    index += multiplier*pcs[i];
    multiplier *= max_state_count;
  }
  return index;
}

inline PbContainer2::wrapper_t *PbContainer2::insert_in_f(PbConstraint *c){
  int pci = pc_index(c);
  std::vector<wrapper_t*> &v = pcs_to_f[pci][c];
  bool subsumed = false;
  for(unsigned i = 0; !subsumed && i < v.size(); i++){
    switch(c->entailment_compare(*v[i]->constraint)){
    case Constraint::LESS:
      /* v[i] is subsumed */
      remove_from_q(v[i]);
      break;
    case Constraint::GREATER: case Constraint::EQUAL:
      subsumed = true;
      break;
    case Constraint::INCOMPARABLE:
      // Do nothing
      break;
    }
  }
  if(subsumed){
    delete c;
    return 0;
  }else{
    wrapper_t *w = new wrapper_t(c);
    v.push_back(w);
    return w;
  }
};

void PbContainer2::insert_root(Constraint *r){
  assert(dynamic_cast<PbConstraint*>(r));
  wrapper_t *w = insert_in_f(static_cast<PbConstraint*>(r));
  if(w){
    f_size++;
    insert_in_q(w);
  }
};

void PbContainer2::insert(Constraint *p, const Machine::PTransition *t, Constraint *c){
  assert(dynamic_cast<PbConstraint*>(c));
  assert(dynamic_cast<PbConstraint*>(p));
  wrapper_t *w = insert_in_f(static_cast<PbConstraint*>(c));
  if(w){
    f_size++;
    insert_in_q(w);
    wrapper_t *pw = get_wrapper(static_cast<PbConstraint*>(p));
    w->parent.transition = t;
    w->parent.target = pw;
    pw->children.push_back(wrapper_t::trans_t(t,w));
  }
};

void PbContainer2::insert_in_q(wrapper_t *w){
  if(w->constraint->dirty_bit){
    w->q_index = dirty_q.push(w);
  }else{
    w->q_index = clean_q.push(w);
  }
  q_size++;
};

bool PbContainer2::pointer_in_f(PbConstraint *c){
  int pci = pc_index(c);
  std::vector<wrapper_t*> &v = pcs_to_f[pci][c];
  for(unsigned i = 0; i < v.size(); i++){
    if(v[i]->constraint == c){
      return true;
    }
  }
  return false;
};

Constraint *PbContainer2::pop(){
  assert(q_size <= clean_q.size() + dirty_q.size());
  assert(q_size >= 0);
  if(q_size == 0){
    return 0;
  }else{
    q_size--;
    wrapper_t *w = 0;
    while(w == 0 && clean_q.size() > 0){
      w = clean_q.pop();
    }
    if(!w){
      while(w == 0){
        assert(dirty_q.size() > 0);
        w = dirty_q.pop();
      }
    }
    prev_popped = w;
    return w->constraint;
  }
};

Trace *PbContainer2::clear_and_get_trace(Constraint *cc){
  assert(dynamic_cast<PbConstraint*>(cc));
  PbConstraint *c = static_cast<PbConstraint*>(cc);
  assert(pointer_in_f(c));
  /* Create the trace */
  wrapper_t *w = get_wrapper(c);
  Trace *trace = new Trace(c);
  w->constraint = 0; // Remove reference to constraint to avoid later deallocation
  while(w->parent.target){
    trace->push_back(*w->parent.transition,w->parent.target->constraint);
    w = w->parent.target;
    /* Remove reference to constraint to avoid later deallocation */
    w->constraint = 0;
  }

  /* Clear */
  clear();

  Log::msg << "Trace length: " << trace->size() << "\n";

  return trace;
};

void PbContainer2::clear(){
  for(unsigned i = 0; i < pcs_to_f.size(); i++){
    for(auto it = pcs_to_f[i].begin(); it != pcs_to_f[i].end(); it++){
      std::vector<wrapper_t*> &v = it->second;
      for(unsigned j = 0; j < v.size(); j++){
        if(v[j]->constraint) // May have been removed by clear_and_get_trace
          delete v[j]->constraint;
        delete v[j];
      }
    }
    pcs_to_f[i].clear();
  }
  f_size = 0;
  q_size = 0;
  clean_q.clear();
  dirty_q.clear();
  prev_popped = &dummy_wrapper;
};

bool PbContainer2::pbcmp::operator()(PbConstraint * const &a, PbConstraint * const &b){
  return (a->channels < b->channels) || ((a->channels == b->channels) && a->cycle_locks < b->cycle_locks);
}

PbContainer2::wrapper_t *PbContainer2::get_wrapper(PbConstraint *c){
  if(c == prev_popped->constraint){
    return prev_popped;
  }
  int pci = pc_index(c);
  std::vector<wrapper_t*> &v = pcs_to_f[pci][c];
  for(unsigned i = 0; i < v.size(); i++){
    if(v[i]->constraint == c){
      return v[i];
    }
  }
  throw new std::logic_error("PbContainer2::get_wrapper: Constraint not in F.");
};

void PbContainer2::remove_from_q(wrapper_t *w){
  bool removed = false;
  if(w->constraint->dirty_bit){
    if(dirty_q.in_queue(w->q_index) && dirty_q.at(w->q_index) == w){
      dirty_q.at(w->q_index) = 0;
      q_size--;
      removed = true;
    }
  }else{
    if(clean_q.in_queue(w->q_index) && clean_q.at(w->q_index) == w){
      clean_q.at(w->q_index) = 0;
      q_size--;
      removed = true;
    }
  }

  if(removed){
    for(unsigned i = 0; i < w->children.size(); i++){
      remove_from_q(w->children[i].target);
    }
  }
}
