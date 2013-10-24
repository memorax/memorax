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

VipsFenceSync::VipsFenceSync(Lang::Stmt<int> f, int pid, int q, TSet IN, TSet OUT)
  : FenceSync(f,pid,q,IN,OUT) {
};

VipsFenceSync::~VipsFenceSync(){};

std::set<Sync*> VipsFenceSync::get_all_possible(const Machine &m){
  std::set<Lang::Stmt<int> > fs;
  fs.insert(Lang::Stmt<int>::nop()); // Does not matter since fsinit will provide the right fence instruction
  FenceSync::fs_init_t fsinit_full =
    [](Lang::Stmt<int> f, int pid, int q,TSet IN,TSet OUT){
    return new VipsFullFenceSync(pid,q,IN,OUT);
  };
  FenceSync::fs_init_t fsinit_ss =
    [](Lang::Stmt<int> f, int pid, int q,TSet IN,TSet OUT){
    return new VipsSSFenceSync(pid,q,IN,OUT);
  };
  FenceSync::fs_init_t fsinit_ll =
    [](Lang::Stmt<int> f, int pid, int q,TSet IN,TSet OUT){
    return new VipsLLFenceSync(pid,q,IN,OUT);
  };
  std::set<Sync*> S, S_ss, S_ll;
  S = FenceSync::get_all_possible(m,fs,fsinit_full);
  S_ss = FenceSync::get_all_possible(m,fs,fsinit_ss);
  S_ll = FenceSync::get_all_possible(m,fs,fsinit_ll);
  S.insert(S_ss.begin(),S_ss.end());
  S.insert(S_ll.begin(),S_ll.end());
  return S;
};

Machine *VipsFenceSync::insert(const Machine &m, m_infos_t m_infos, Sync::InsInfo **info) const{
  /* Check that no other fences are inserted to the same location. */
  for(unsigned i = 0; i < m_infos.size(); ++i){
    if(dynamic_cast<const VipsFenceSync*>(m_infos[i]->sync)){
      const VipsFenceSync *vfs = static_cast<const VipsFenceSync*>(m_infos[i]->sync);
      if(vfs->pid == pid && vfs->q == q){
        throw new Incompatible(m_infos[i],
                               "At most one VipsFenceSync may be inserted at each program location.");
      }
    }
  }

  /* OK. Insert as usual. */
  return FenceSync::insert(m,m_infos,info);
};

VipsFullFenceSync::VipsFullFenceSync(int pid, int q, TSet IN, TSet OUT)
  : VipsFenceSync(Lang::Stmt<int>::full_fence(),pid,q,IN,OUT) {
};

VipsFullFenceSync::~VipsFullFenceSync(){};

Sync *VipsFullFenceSync::clone() const{
  return new VipsFullFenceSync(*this);
};

VipsSSFenceSync::VipsSSFenceSync(int pid, int q, TSet IN, TSet OUT)
  : VipsFenceSync(Lang::Stmt<int>::ss_fence(),pid,q,IN,OUT) {
};

VipsSSFenceSync::~VipsSSFenceSync(){};

Sync *VipsSSFenceSync::clone() const{
  return new VipsSSFenceSync(*this);
};

VipsLLFenceSync::VipsLLFenceSync(int pid, int q, TSet IN, TSet OUT)
  : VipsFenceSync(Lang::Stmt<int>::ll_fence(),pid,q,IN,OUT) {
};

VipsLLFenceSync::~VipsLLFenceSync(){};

Sync *VipsLLFenceSync::clone() const{
  return new VipsLLFenceSync(*this);
};
