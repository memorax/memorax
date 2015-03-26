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

#ifndef __TSO_LOCK_SYNC_H__
#define __TSO_LOCK_SYNC_H__

#include "machine.h"
#include "sync.h"

#include <map>

/* A TsoLockSync synchronization is synchronization by changing a
 * specific write (write: x := e) into a locked write (locked write: x
 * := e).
 */
class TsoLockSync : public Sync{
public:
  /* Construct synchronization that replaces w with w_pid:
   * (w_src,locked write: w_x := w_e ,w_tgt).
   *
   * Pre: w is w_pid: (w_src,write: w_x := w_e ,w_tgt) for some w_pid,
   * w_src, w_x, w_e, w_tgt.
   */
  TsoLockSync(const Machine::PTransition &w);
  virtual ~TsoLockSync() {};

  class InsInfo : public Sync::InsInfo{
  public:
    InsInfo(const TsoLockSync *creator_copy);
    InsInfo(const InsInfo &) = default;
    InsInfo &operator=(const InsInfo &) = delete;
    virtual ~InsInfo() {};

    /* When a TsoLockSync is inserted into a Machine m, creating the
     * Machine m', for each transition t in m, tchanges[t] is that
     * transition as it occurs in tchanges.
     *
     * This means that all transitions will map to themselves, except
     * for one write which maps to a locked, but otherwise unchanged,
     * copy of itself.
     *
     * tchanges[t] is defined for all transitions t in m.
     */
    std::map<Machine::PTransition,Machine::PTransition> tchanges;

    /* Insert a->b into tchanges. */
    void bind(const Machine::PTransition &a,const Machine::PTransition &b);
    /* Shorthand for tchanges[t]. */
    const Machine::PTransition &operator[](const Machine::PTransition &t) const;

    /* If ivec = [&a,&b,...,&z] then the returned transition is
     *
     * z[...b[a[t]]...]
     *
     * Pre: All elements in ivec are pointers to FenceSync::InsInfo
     * objects (or derivatives).
     */
    static Machine::PTransition all_tchanges(const std::vector<const Sync::InsInfo*> &ivec,
                                             const Machine::PTransition &t);
  };

  virtual Machine *insert(const Machine &m, const std::vector<const Sync::InsInfo*> &m_infos, Sync::InsInfo **info) const;

  /* Return a deep copy of this object. */
  virtual Sync *clone() const;
  virtual std::string to_raw_string() const;
  virtual std::string to_string(const Machine &m) const;
  virtual void print_raw(Log::redirection_stream &os, Log::redirection_stream &json_os) const;
  virtual void print(const Machine &m, Log::redirection_stream &os, Log::redirection_stream &json_os) const;

  /* Returns all TsoFenceSyncs that can be inserted into m. */
  static std::set<Sync*> get_all_possible(const Machine &m);

  virtual int get_pid() const { return w.pid; };
  virtual const Machine::PTransition &get_write() const { return w; };

  static void test();
protected:
  virtual int compare(const Sync &s) const;
private:
  Machine::PTransition w;

  std::string to_string_aux(const std::function<std::string(const int&)> &regts,
                            const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const;
};

#endif
