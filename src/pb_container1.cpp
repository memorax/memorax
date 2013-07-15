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

#include "pb_container1.h"

PbContainer1::PbContainer1(const Machine &m)
: edges(EDGES_SIZE), edge_count(0), 
  f_size(0), 
  clean_q(Q_SIZE), dirty_q(Q_SIZE), q_size(0)
{
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

inline int PbContainer1::pc_index(PbConstraint *c) const{
  int index = 0;
  int multiplier = 1;
  const std::vector<int> &pcs = c->pcs;
  for(int i = 0; i < proc_count; i++){
    index += multiplier*pcs[i];
    multiplier *= max_state_count;
  }
  return index;
}

void PbContainer1::remove_from_f(int pci, std::set<wrapper_t*,wrappercmp>::iterator it){
  /* Remove from F */
  wrapper_t w = **it;
  PbConstraint *c = w.constraint;
  bool dirty = c->dirty_bit;
  delete c;
  delete *it;
  pcs_to_f[pci].erase(it);
  f_size--;

  /* Remove from Q */
  if(dirty){
    if(dirty_q.in_queue(w.q_index) && dirty_q.at(w.q_index) == c){
      dirty_q.at(w.q_index) = 0;
      q_size--;
    }
  }else{
    if(dirty_q.in_queue(w.q_index) && clean_q.at(w.q_index) == c){
      clean_q.at(w.q_index) = 0;
      q_size--;
    }
  }

  /* Remove Edges, and recursively remove descendants */
  edge_t *cur_edge;
  edge_t *end_edge = &edges[edge_count];
  if(w.edge_index >= 0){
    cur_edge = &edges[w.edge_index];
    assert(cur_edge->child == c);
    cur_edge->parent = 0;
    cur_edge->transition = 0;
    cur_edge->child = 0;
  }else{
    cur_edge = &edges[0];
  }
  while(cur_edge != end_edge){
    assert(cur_edge < end_edge);
    if(cur_edge->parent == c){
      remove_from_f(cur_edge->child);
    }
    cur_edge++;
  }
  
}

void PbContainer1::remove_from_f(PbConstraint *c){
  wrapper_t w(c);
  int pci = pc_index(c);
  std::set<wrapper_t*,wrappercmp>::iterator it = pcs_to_f[pci].find(&w);
  assert(it != pcs_to_f[pci].end());
  remove_from_f(pci,it);
}

inline PbContainer1::wrapper_t *PbContainer1::insert_in_f(PbConstraint *c){
  wrapper_t *w = new wrapper_t(c);
  int pci = pc_index(c);
  auto pr = pcs_to_f[pci].insert(w);
  if(pr.second){
    // Ok c was not present in F
    return w;
  }else{
    wrapper_t *w2 = *pr.first;
    if(w2->constraint->dirty_bit && !c->dirty_bit){
      // The old constraint is subsumed by c
      remove_from_f(pci,pr.first);
#ifdef NDEBUG
      pcs_to_f[pci].insert(w);
#else
      pr = pcs_to_f[pci].insert(w);
      assert(pr.second);
#endif
      return w;
    }else{
      // c is identical to or subsumed by the old constraint
      delete c;
      delete w;
      return 0;
    }
  }
};

void PbContainer1::insert_root(Constraint *r){
  assert(dynamic_cast<PbConstraint*>(r));
  wrapper_t *w = insert_in_f(static_cast<PbConstraint*>(r));
  if(w){
    f_size++;
    insert_in_q(w);
    w->edge_index = -1;
  }
};

void PbContainer1::insert(Constraint *p, const Machine::PTransition *t, Constraint *c){
  assert(dynamic_cast<PbConstraint*>(c));
  assert(dynamic_cast<PbConstraint*>(p));
  wrapper_t *w = insert_in_f(static_cast<PbConstraint*>(c));
  if(w){
    f_size++;
    insert_in_q(w);
    add_edge(static_cast<PbConstraint*>(p),t,w);
  }
};

void PbContainer1::insert_in_q(wrapper_t *w){
  if(w->constraint->dirty_bit){
    w->q_index = dirty_q.push(w->constraint);
  }else{
    w->q_index = clean_q.push(w->constraint);
  }
  q_size++;
};

void PbContainer1::add_edge(PbConstraint *p, const Machine::PTransition *t, wrapper_t *w){
  assert(pointer_in_f(p));
  if(edge_count >= int(edges.size())){
    assert(edge_count == int(edges.size()));
    edges.resize(edge_count*2);
  }
  edges[edge_count].parent = p;
  edges[edge_count].transition = t;
  edges[edge_count].child = w->constraint;
  w->edge_index = edge_count;
  edge_count++;
}

bool PbContainer1::pointer_in_f(PbConstraint *c){
  wrapper_t w(c);
  int pci = pc_index(c);
  auto it = pcs_to_f[pci].find(&w);
  return it != pcs_to_f[pci].end() && (*it)->constraint == c;
};

Constraint *PbContainer1::pop(){
  assert(q_size <= clean_q.size() + dirty_q.size());
  assert(q_size >= 0);
  if(q_size == 0){
    return 0;
  }else{
    q_size--;
    PbConstraint *c = 0;
    while(c == 0 && clean_q.size() > 0){
      c = clean_q.pop();
    }
    if(!c){
      while(c == 0){
        c = dirty_q.pop();
      }
    }
    return c;
  }
};

Trace *PbContainer1::clear_and_get_trace(Constraint *cc){
  assert(dynamic_cast<PbConstraint*>(cc));
  PbConstraint *c = static_cast<PbConstraint*>(cc);
  assert(pointer_in_f(c));
  /* Create the trace */
  Trace *trace = new Trace(c);
  wrapper_t w(c);
  wrapper_t *wp = *pcs_to_f[pc_index(c)].find(&w);
  pcs_to_f[pc_index(c)].erase(&w); // Remove from F to avoid later deallocation
  delete wp;
  int i = edge_count-1; // Index into edges
  while(i >= 0){
    if(edges[i].child == c){
      c = edges[i].parent;
      assert(pointer_in_f(c));
      w.constraint = c;
      wp = *pcs_to_f[pc_index(c)].find(&w);
      pcs_to_f[pc_index(c)].erase(&w); // Remove from F to avoid later deallocation
      delete wp;
      trace->push_back(*edges[i].transition,c);
    }
    i--;
  }

  /* Clear */
  clear();

  std::cout << "Trace length: " << trace->size() << "\n";

  return trace;
};

void PbContainer1::clear(){
  for(unsigned i = 0; i < pcs_to_f.size(); i++){
    for(auto it = pcs_to_f[i].begin(); it != pcs_to_f[i].end(); it++){
      delete (*it)->constraint;
      delete *it;
    }
    pcs_to_f[i].clear();
  }
  f_size = 0;
  edge_count = 0;
  q_size = 0;
  clean_q.clear();
  dirty_q.clear();
};

bool PbContainer1::wrappercmp::operator()(wrapper_t * const &a, wrapper_t * const &b){
  const PbConstraint &pbc_a = *a->constraint;
  const PbConstraint &pbc_b = *b->constraint;
  assert(pbc_a.pcs == pbc_b.pcs);
  assert(&pbc_a.common == &pbc_b.common);
  if(pbc_a.channels < pbc_b.channels){
    return true;
  }else if(pbc_a.channels == pbc_b.channels){
    if(pbc_a.ap_list < pbc_b.ap_list){
      return true;
    }
  }

  return false;
}
