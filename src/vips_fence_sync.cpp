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
