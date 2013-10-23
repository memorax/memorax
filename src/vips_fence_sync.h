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

/* VipsFenceSync instantiates FenceSync for the VIPS fence
 * instruction.
 */
class VipsFenceSync : public FenceSync{
public:
  /* Constructs the FenceSync (f,pid,q,IN,OUT) where f is the VIPS
   * fence instruction.
   */
  VipsFenceSync(int pid, int q, TSet IN, TSet OUT);
  virtual ~VipsFenceSync();
  virtual Sync *clone() const;

  /* Returns all VipsFenceSyncs that can be inserted into m. */
  static std::set<Sync*> get_all_possible(const Machine &m);

  static void test();
protected:
  virtual int compare(const Sync &s) const { return FenceSync::compare(s); };
};

#endif
