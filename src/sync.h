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

#ifndef __SYNC_H__
#define __SYNC_H__

#include "machine.h"
#include "trace.h"

/* Sync is an abstract class representing a piece of synchronization
 * that can be inserted into a machine. Derived classes could
 * implement particular pieces of synchronization, e.g. a fence at a
 * given position.
 */
class Sync{
public:
  /* An InsInfo object is created by a Sync derivative s when s is
   * inserted into a Machine m. The InsInfo object describes the
   * changes to m by s, such that Sync derivatives that are inserted
   * later can relate the changed m with the original.
   *
   * What exactly is kept in an InsInfo object depends on the Sync
   * derivative that created it. Different classes extending Sync
   * should make sure to understand each others' InsInfo objects if it
   * should be possible to use the different Sync derivatives in the
   * same Machine.
   *
   * InsInfo should be overloaded.
   */
  class InsInfo{
  public:
    /* Construct an InsInfo object with a copy creator_copy of the
     * Sync derivative that created it.
     *
     * InsInfo takes ownership over creator_copy.
     */
    InsInfo(const Sync *creator_copy) : sync(creator_copy) {};
    InsInfo(const InsInfo &ii) : sync(ii.sync ? ii.sync->clone() : 0) {};
    virtual InsInfo &operator=(const InsInfo &ii){
      if(&ii != this){
        if(sync) delete sync;
        sync = ii.sync;
      }
      return *this;
    };
    virtual ~InsInfo() { if(sync) delete sync; };
    /* A copy of the Sync that created this InsInfo. */
    const Sync *sync;
  };

  virtual ~Sync() {};
  /* Inserts this synchronization into m. Returns the result. Sets
   * *info to point to a new InsInfo object that describes the changes
   * in the returned Machine, compared to m.
   *
   * If this Sync is created with respect to a Machine m_org, then
   * m_infos should contain all InsInfo objects corresponding to Sync
   * insertions performed to m_org, turning m_org into m. The InsInfo
   * objects in m_infos should be in the order the corresponding Sync
   * insertions were performed.
   *
   * This Sync surrenders ownership of *info to the caller.
   */
  virtual Machine *insert(const Machine &m, const std::vector<const InsInfo*> &m_infos, InsInfo **info) const = 0;
  virtual bool prevents(const Trace &t, const std::vector<const InsInfo*> &m_infos) const = 0;
  /* Return a deep copy of this object. */
  virtual Sync *clone() const = 0;
  virtual std::string to_raw_string() const = 0;
  virtual std::string to_string() const { return to_raw_string(); };
  virtual std::string to_string(const Machine &m) const { return to_string(); };
};

#endif
