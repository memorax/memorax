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

#include "sb_container.h"

const bool SbContainer::print_every_state_on_clear = false;
const bool SbContainer::use_genealogy = false;

SbContainer::SbContainer(){
  last_popped.first = 0;
  last_popped.second = 0;
  q_size = f_size = 0;
};

SbContainer::~SbContainer(){
#ifndef NDEBUG
  stats.print();
#endif
  clear();
};

void SbContainer::insert_root(Constraint *r){
  insert(new CWrapper(static_cast<SbConstraint*>(r)));
};

void SbContainer::insert(Constraint *p, const Machine::PTransition *t, Constraint *c){
  CWrapper *pcw = get_cwrapper(static_cast<SbConstraint*>(p));
  CWrapper *cw = new CWrapper(static_cast<SbConstraint*>(c), pcw, t);
  if(insert(cw)){
    if(use_genealogy){
      pcw->children.push_back(cw);
    }
  }
};

bool SbContainer::insert(CWrapper *cw){
  std::vector<CWrapper*> &v = F[cw->sbc->get_control_states()][cw->sbc->characterize_channel()];
  /* Go through v to see if cw is subsumed or if cw subsumes any of
   * the existing constraints */
  for(unsigned i = 0; i < v.size(); ++i){
    assert(v[i]->valid);
    switch(cw->sbc->entailment_compare(*v[i]->sbc)){
    case Constraint::LESS:
      /* The new constraint subsumes an old one. */
      invalidate(v[i],&v);
      --i;
      break;
    case Constraint::GREATER: case Constraint::EQUAL:
      /* The new constraint is subsumed by an old one. */
      delete cw;
      return false;
    case Constraint::INCOMPARABLE:
      break;
    }
  }
  v.push_back(cw);
  update_longest_comparable_array(v);
  update_longest_channel(cw->sbc->channel.size());
  ptr_to_F[cw->sbc] = cw;
  cw->Q_ticket = Q.push(cw);
  ++q_size;
  ++f_size;
  return true;
};

void SbContainer::invalidate(CWrapper *cw, std::vector<CWrapper*> *Fv){
  if(Fv == 0){
    Fv = &F[cw->sbc->get_control_states()][cw->sbc->characterize_channel()];
  }
#ifndef NDEBUG
  bool erased = false;
#endif
  for(unsigned i = 0; i < Fv->size(); ++i){
    if(Fv->at(i) == cw){
#ifndef NDEBUG
      erased = true;
#endif
      for(unsigned j = i; j < Fv->size()-1; ++j){
        Fv->at(j) = Fv->at(j+1);
      }
      Fv->resize(Fv->size()-1);
      break;
    }
  }
  assert(erased);
  invalid_from_F.push_back(cw);
  cw->valid = false;
  if(Q.in_queue(cw->Q_ticket,cw->sbc->get_channel_length())){
    --q_size;
  }
  --f_size;
  inc_invalidate_count();
  if(use_genealogy){
    for(unsigned i = 0; i < cw->children.size(); ++i){
      if(cw->children[i]->valid){
        invalidate(cw->children[i]);
      }
    }
  }
};

Constraint *SbContainer::pop(){
  if(q_size == 0){
    return 0;
  }else{
    CWrapper *cw = Q.pop();
    while(!cw->valid){
      cw = Q.pop();
    }
    --q_size;
    last_popped.first = cw->sbc;
    last_popped.second = cw;
    return cw->sbc;
  }
};

Trace *SbContainer::clear_and_get_trace(Constraint *c){
  Trace *t = new Trace(c);
  CWrapper *cw = get_cwrapper(static_cast<SbConstraint*>(c));
  cw->sbc = 0;
  while(cw->parent){
    t->push_back(*cw->p_transition,cw->parent->sbc);
    cw->parent->sbc = 0;
    cw = cw->parent;
  }
  clear();
  return t;
};

void SbContainer::clear(){
  if(print_every_state_on_clear){
    Log::extreme << "  **************************************\n";
    Log::extreme << "  *** All constraints in visited set ***\n";
    Log::extreme << "  **************************************\n\n";
  }
  for(auto it = F.begin(); it != F.end(); ++it){
    for(auto it2 = it->second.begin(); it2 != it->second.end(); ++it2){
      for(unsigned i = 0; i < it2->second.size(); ++i){
        if(print_every_state_on_clear){
          if(it2->second[i]->sbc){
            Log::extreme << it2->second[i]->sbc->to_string() << "\n";
          }
        }
        delete it2->second[i];
      }
    }
  }
  for(auto it = invalid_from_F.begin(); it != invalid_from_F.end(); ++it){
    delete *it;
  }
  invalid_from_F.clear();
  F.clear();
  Q.clear();
  ptr_to_F.clear();
  f_size = 0;
  q_size = 0;
  last_popped.first = 0;
  last_popped.second = 0;
};
