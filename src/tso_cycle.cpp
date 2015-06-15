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

#include "tso_cycle.h"
#include "lang.h"
#include <cassert>

TsoCycle::TsoCycle(int proc_count) 
  : proc_count(proc_count), frag_len(0), complete(false){
  frag.resize(proc_count*2,0);
}

void TsoCycle::push_back(const Machine::PTransition *pt){
  assert(can_push_back(pt));
  assert(pt != 0);
  assert(frag_len < proc_count*2);
  frag[frag_len] = pt;
  frag_len++;
#ifndef NDEBUG
  for(int i = 0; i < frag_len; i++){
    assert(frag[i] != 0);
  }
#endif
  complete = compute_is_complete();
}

void TsoCycle::push_front(const Machine::PTransition *pt){
  assert(can_push_front(pt));
  assert(pt != 0);
  for(unsigned i = frag_len; i > 0; i--) frag[i] = frag[i-1];
  frag[0] = pt;
  frag_len++;
#ifndef NDEBUG
  for(int i = 0; i < frag_len; i++){
    assert(frag[i] != 0);
  }
#endif
  complete = compute_is_complete();
}

bool TsoCycle::overlaps(const std::vector<Lang::MemLoc<int> > &a, int a_pid,
                        const std::vector<Lang::MemLoc<int> > &b, int b_pid){
  std::vector<Lang::NML> na(a.size(),Lang::NML::global(0));
  std::vector<Lang::NML> nb(b.size(),Lang::NML::global(0));
  for(unsigned i = 0; i < a.size(); i++) na[i] = Lang::NML(a[i],a_pid);
  for(unsigned i = 0; i < b.size(); i++) nb[i] = Lang::NML(b[i],b_pid);
  unsigned i = 0; // Pointer into na
  unsigned j = 0; // Pointer into nb
  while(i < na.size() && j < nb.size()){
    if(na[i] == nb[j]){
      return true;
    }else if(na[i] < nb[j]){
      i++;
    }else{
      assert(nb[j] < na[i]);
      j++;
    }
  }
  return false;
}

bool TsoCycle::connects(const Machine::PTransition *a, const Machine::PTransition *b){
  if(a->pid == b->pid){
    return true;
  }else{
    const std::vector<Lang::MemLoc<int> > &aw = a->instruction.get_writes();
    const std::vector<Lang::MemLoc<int> > &ar = a->instruction.get_reads();
    const std::vector<Lang::MemLoc<int> > &bw = b->instruction.get_writes();
    const std::vector<Lang::MemLoc<int> > &br = b->instruction.get_reads();
    return overlaps(aw,a->pid,bw,b->pid) || overlaps(aw,a->pid,br,b->pid) || overlaps(ar,a->pid,bw,b->pid);
  }
}

bool TsoCycle::can_push_back(const Machine::PTransition *pt) const{
  if(!mem_access(pt)){
    return false;
  }

  if(frag_len == 0){
    return true;
  }

  if(complete){
    return false;
  }

  if(!connects(frag[frag_len-1],pt)){
    return false;
  }

  for(int i = 1; i < frag_len-1; i++){
    if(connects(frag[i],pt)){
      /* There is a chord making this cycle non-minimal */
      return false;
    }
  }

  if(frag_len == 2 && connects(frag[0],pt)){
    /* Need to check that frag[0], frag[1], pt is really a cycle, and
     * not contained in one process. */
    if(frag[0]->pid == frag[1]->pid && frag[1]->pid == pt->pid){
      return false;
    }
  }

  return true;
}

bool TsoCycle::can_push_front(const Machine::PTransition *pt) const{
  if(!mem_access(pt)){
    return false;
  }

  if(frag_len == 0){
    return true;
  }

  if(complete){
    return false;
  }

  if(!connects(pt,frag[0])){
    return false;
  }

  for(int i = 1; i < frag_len-1; i++){
    if(connects(frag[i],pt)){
      /* There is a chord making this cycle non-minimal */
      return false;
    }
  }

  if(frag_len == 2 && connects(frag[1],pt)){
    /* Need to check that pt, frag[0], frag[1] is really a cycle, and
     * not contained in one process. */
    if(frag[0]->pid == frag[1]->pid && frag[1]->pid == pt->pid){
      return false;
    }
  }

  return true;
}

std::list<std::pair<const Machine::PTransition*,const Machine::PTransition*> > TsoCycle::get_critical_pairs() const{
  std::list<std::pair<const Machine::PTransition*,const Machine::PTransition*> > l;
  if(frag_len > 1){
    for(int i = 0; i < frag_len; i++){
      if(frag[i]->pid == frag[(i+1) % frag_len]->pid &&
         (frag_len > 2 || i < ((i+1) % frag_len))){
        l.push_back(std::pair<const Machine::PTransition*,const Machine::PTransition*>(frag[i],frag[(i+1) % frag_len]));
      }
    }
  }
  return l;
};

std::string TsoCycle::to_string(const Machine &m) const{
  std::string s;
  if(complete){
    s = "TsoCycle (complete):\n";
  }else{
    s = "TsoCycle (fragment):\n";
  }
  for(int i = 0; i < frag_len; i++){
    s += "  "+frag[i]->to_string(m)+"\n";
  }
  return s;
};

std::string TsoCycle::to_string(const std::function<std::string(const int&)> &regts, 
                                const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const{
  std::string s;
  if(complete){
    s = "TsoCycle (complete):\n";
  }else{
    s = "TsoCycle (fragment):\n";
  }
  for(int i = 0; i < frag_len; i++){
    s += "  "+frag[i]->to_string(regts,mlts)+"\n";
  }
  return s;
};

int TsoCycle::compare(const TsoCycle &tc) const throw(){
  if(proc_count < tc.proc_count){
    return -1;
  }else if(proc_count > tc.proc_count){
    return 1;
  }else if(frag < tc.frag){
    return -1;
  }else if(frag > tc.frag){
    return 1;
  }else{
    /* frag_len and complete are both determined by frag, so there is
     * no need to test them separately. */
    return 0;
  }
};
