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

#ifndef __VIPS_FENCE_SYNC_H__
#define __VIPS_FENCE_SYNC_H__

#include "fence_sync.h"

/* VipsFenceSync is an abstract class which is the super class of all
 * FenceSyncs for VIPS.
 */
class VipsFenceSync : public FenceSync{
public:
  /* Constructs the FenceSync (f,pid,q,IN,OUT).
   */
  VipsFenceSync(Lang::Stmt<int> f, int pid, int q, TSet IN, TSet OUT);
  virtual ~VipsFenceSync();
  virtual Sync *clone() const = 0;

  virtual Machine *insert(const Machine &m, m_infos_t m_infos, Sync::InsInfo **info) const;

  /* Returns all VipsFenceSyncs that can be inserted into m.
   *
   * If full_branch_only is set then only VipsFenceSyncs
   * (f,pid,q,IN,OUT) where IN and OUT are maximal are returned.
   */
  static std::set<Sync*> get_all_possible(const Machine &m,bool full_branch_only);

  static void test();
protected:
  virtual int compare(const Sync &s) const { return FenceSync::compare(s); };
  /* Helper for VipsSSFence::insert and VipsLLFence::insert for the
   * case when there is already a fence at the same control location
   * for the same process. Inserts this fence before prev_info->sync
   * if insert_before is set, and after prev_info->sync if
   * insert_before is not set.
   *
   * Pre: prev_info->sync->IN == this->IN && prev_info->sync->OUT == this->OUT.
   *      prev_info->sync is an llfence if this is an ssfence.
   *      prev_info->sync is an ssfence if this is an llfence.
   *      stackable(prev_info->sync).
   *      There is no other fence at q for pid except prev_info->sync.
   */
  virtual Machine *insert_double(const Machine &m, m_infos_t m_infos, Sync::InsInfo **info,
                                 const FenceSync::InsInfo *prev_info,
                                 bool insert_before) const;
  /* Returns true iff A is a strict subset of B. */
  static bool strict_subset(const FenceSync::TSet &A, const FenceSync::TSet &B);
  /* Returns true iff A and B have the same elements */
  static bool equal(const FenceSync::TSet &A, const FenceSync::TSet &B);
};

/* VipsFullFenceSync is an instance of VipsFenceSync corresponding to
 * the full fence.
 */
class VipsFullFenceSync : public VipsFenceSync {
public:
  /* Constructs the FenceSync (f,pid,q,IN,OUT) where f is a full
   * fence.
   */
  VipsFullFenceSync(int pid, int q, TSet IN, TSet OUT);
  virtual ~VipsFullFenceSync();
  virtual Sync *clone() const;
};

class VipsLLFenceSync;

/* VipsSSFenceSync is an instance of VipsFenceSync corresponding to
 * the ssfence.
 */
class VipsSSFenceSync : public VipsFenceSync {
public:
  /* Constructs the FenceSync (f,pid,q,IN,OUT) where f is an ssfence.
   */
  VipsSSFenceSync(int pid, int q, TSet IN, TSet OUT);
  virtual ~VipsSSFenceSync();
  virtual Sync *clone() const;
  virtual Machine *insert(const Machine &m, m_infos_t m_infos, Sync::InsInfo **info) const;
  /* Returns true iff this fence may be inserted together with vfs_ll
   * to the same control location.
   *
   * I.e. returns true iff vfs_ll is for the same process and control
   * location as this fence and neither this->IN is a strict subset of
   * vfs_ll->IN, nor vfs_ll->OUT is a strict subset of this->OUT.
   *
   * Note that before vfs_ll can be stacked with this fence, they also
   * have to be compatible according to FenceSync.
   */
  bool stackable(const VipsLLFenceSync *vfs_ll) const;
};

/* VipsLLFenceSync is an instance of VipsFenceSync corresponding to
 * the llfence.
 */
class VipsLLFenceSync : public VipsFenceSync {
public:
  /* Constructs the FenceSync (f,pid,q,IN,OUT) where f is an llfence.
   */
  VipsLLFenceSync(int pid, int q, TSet IN, TSet OUT);
  virtual ~VipsLLFenceSync();
  virtual Sync *clone() const;
  virtual Machine *insert(const Machine &m, m_infos_t m_infos, Sync::InsInfo **info) const;
  /* Returns true iff this fence may be inserted together with vfs_ss
   * to the same control location.
   *
   * I.e. returns true iff vfs_ss->stackable(this) .
   */
  bool stackable(const VipsSSFenceSync *vfs_ss) const;
};

#endif
