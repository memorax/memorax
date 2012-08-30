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

#include "tso_cycle_lock.h"

TsoCycleLock::TsoCycleLock(const Machine::PTransition *r, const Machine::PTransition *u, int proc_count)
  : unlocked(false), proc_count(proc_count), r(r), u(u) {
  assert(r->instruction.get_reads().size() > 0);
  assert(!r->instruction.is_fence());
  assert(u->instruction.get_type() == Lang::UPDATE);
  if(r->instruction.get_reads().size() != 1 || r->instruction.get_reads()[0] != u->instruction.get_memloc()){
    ccycles.push_back(TsoCycle(proc_count));
    ccycles[0].push_back(r);
    ccycles[0].push_back(u);
  }
};

bool TsoCycleLock::execute(const Machine::PTransition *pt){
  if(ccycles.empty()){
    /* Either unlocked or permanently locked */
    return unlocked;
  }

  if(unlocked){
    /* No need to do anything with pt */
    return true;
  }

  int sz = ccycles.size(); // Count only the cycles which are in the ccycles from the start
  for(int i = 0; !unlocked && i < sz; i++){
    if(ccycles[i].can_push_back(pt)){
      ccycles.push_back(ccycles[i]); // Copy ccycles[i]
      ccycles[ccycles.size()-1].push_back(pt);
      if(ccycles[ccycles.size()-1].is_complete()){
        unlocked = true;
      }
    }
  }

  if(unlocked){
    ccycles.clear();
  }else{
    /* Insert the new locks at their place in the order */
    // ccycles is sorted and distinct in the interval [0,sz)
    int i; // ccycles has unsorted, new cycles in the interval [i,ccycles.size())
    for(i = sz; i < int(ccycles.size()); i++){
      /* Does ccycles[i] exist in ccycles[0] - ccycles[sz2-1]? */
      int j = 0;
      while(j < sz && ccycles[j] < ccycles[i]) j++;
      if(j == sz){
        ccycles[sz] = ccycles[i];
        sz++;
      }else if(ccycles[j] > ccycles[i]){
        /* Move cycles to make place for ccycles[i] */
        TsoCycle cci(ccycles[i]);
        for(int k = sz; k > j; k--){
          ccycles[k] = ccycles[k-1];
        }
        ccycles[j] = cci;
        sz++;
      }else{
        assert(ccycles[j] == ccycles[i]);
        /* Ignore duplicate cycle */
      }
    }
    assert(sz <= int(ccycles.size()));
    ccycles.resize(sz,ccycles[0]);
  }

  return unlocked;
}

std::string TsoCycleLock::to_short_string(const std::function<std::string(const Lang::MemLoc<int>&)> &mlts) const{
  std::stringstream ss;

  ss << "TsoCycleLock: P" << r->pid << " (read ";
  if(r->instruction.get_reads().size() == 1){
    ss << mlts(r->instruction.get_reads()[0]);
  }else{
    for(unsigned i = 0; i < r->instruction.get_reads().size(); ++i){
      if(i != 0) ss << ", ";
      ss << mlts(r->instruction.get_reads()[i]);
    }
  }
  ss << ", update " << mlts(u->instruction.get_memloc()) << "): ";
  if(unlocked){
    ss << "Unlocked";
  }else{
    if(ccycles.empty()){
      ss << "Permanently locked";
    }else{
      ss << "Locked";
    }
  }
  return ss.str();
}

std::string TsoCycleLock::to_long_string(const std::function<std::string(const int&)> &regts,
                                         const std::function<std::string(const Lang::MemLoc<int>&)> &mlts) const{
  std::string s = to_short_string(mlts);
  if(!unlocked){
    s += "\n";
    for(unsigned i = 0; i < ccycles.size(); i++){
      s += " * "+ccycles[i].to_string(regts,mlts);
    }
  }
  return s;
}

int TsoCycleLock::compare(const TsoCycleLock &cl) const throw(){
  if(proc_count < cl.proc_count){
    return -1;
  }else if(proc_count > cl.proc_count){
    return 1;
  }else if(r < cl.r){
    return -1;
  }else if(r > cl.r){
    return 1;
  }else if(u < cl.u){
    return -1;
  }else if(u > cl.u){
    return 1;
  }else if(ccycles < cl.ccycles){
    return -1;
  }else if(ccycles == cl.ccycles){
    return 0;
  }else{
    return 1;
  }
};
