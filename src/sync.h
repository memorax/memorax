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

#include "log.h"
#include "machine.h"

#include <stdexcept>

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
    InsInfo &operator=(const InsInfo &ii) = delete;
    virtual ~InsInfo() { if(sync) delete sync; };
    /* A copy of the Sync that created this InsInfo. */
    const Sync *sync;
  };

  virtual ~Sync() {};

  /* When a Sync s is to be inserted into a machine where already a
   * Sync s' has been inserted such that s and s' cannot coexist in
   * the machine, then Incompatible(s',...) is thrown by s.insert.
   */
  class Incompatible : public std::exception {
  public:
    Incompatible(const InsInfo *ii, std::string msg) : ii(ii), msg(msg) {};
    virtual ~Incompatible() throw() {};
    virtual const char *what() const throw() { return msg.c_str(); };
    const InsInfo *ii;
    std::string msg;
  };

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
  /* Return a deep copy of this object. */
  virtual Sync *clone() const = 0;
  virtual std::string to_raw_string() const = 0;
  virtual std::string to_string() const { return to_raw_string(); };
  virtual std::string to_string(const Machine &m) const { return to_string(); };
  /* The print methods work as the to_string methods, but instead of
   * returning a string, they print it to the stream os. JSON
   * meta-data is printed to json_os.
   *
   * Hint: Use with e.g. Log::msg, Log::json
   */
  virtual void print_raw(Log::redirection_stream &os, Log::redirection_stream &json_os) const { os << to_raw_string(); };
  virtual void print(Log::redirection_stream &os, Log::redirection_stream &json_os) const { print_raw(os,json_os); };
  virtual void print(const Machine &m, Log::redirection_stream &os, Log::redirection_stream &json_os) const { print(os,json_os); };
  bool operator<(const Sync &s) const;
  bool operator==(const Sync &s) const;
protected:
  virtual int compare(const Sync &s) const = 0;
};

#endif
