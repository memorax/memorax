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

  /* Returns all VipsFenceSyncs that can be inserted into m. */
  static std::set<Sync*> get_all_possible(const Machine &m);
protected:
  virtual int compare(const Sync &s) const { return FenceSync::compare(s); };
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
};

#endif
