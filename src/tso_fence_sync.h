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

#ifndef __TSO_FENCE_SYNC_H__
#define __TSO_FENCE_SYNC_H__

#include "fence_sync.h"

/* A TsoFenceSync is a synchronization under TSO that inserts separate
 * instructions actions as fences. The fence instructions are
 * implemented as a locked write to a dedicated dummy memory location.
 */
class TsoFenceSync : public FenceSync{
public:
  /* Constructs the FenceSync (f,pid,q,IN,OUT) for some f.
   *
   * The instruciton f here is really just a place-holder. The actual
   * fence instruction that is inserted into a Machine, is determined
   * at the time of insertion.
   */
  TsoFenceSync(int pid, int q, TSet IN, TSet OUT);
  virtual ~TsoFenceSync();

  class InsInfo : public FenceSync::InsInfo{
  public:
    InsInfo(const FenceSync::InsInfo &fs_info, const Lang::NML &fence_nml);
    InsInfo(const InsInfo &) = default;
    InsInfo &operator=(const InsInfo &) = delete;
    virtual ~InsInfo();
    /* The dummy memory location that was used for the inserted fence
     * instruction. */
    Lang::NML fence_nml;
  };
  virtual Machine *insert(const Machine &m, const std::vector<const Sync::InsInfo*> &m_infos, Sync::InsInfo **info) const;
  virtual Sync *clone() const;
  virtual std::string to_raw_string() const;
  virtual std::string to_string(const Machine &m) const;
  virtual void print_raw(Log::redirection_stream &os, Log::redirection_stream &json_os) const;
  virtual void print(const Machine &m, Log::redirection_stream &os, Log::redirection_stream &json_os) const;
  static void test();

  /* Returns all TsoFenceSyncs that can be inserted into m. */
  static std::set<Sync*> get_all_possible(const Machine &m);
protected:
  virtual int compare(const Sync &s) const { return FenceSync::compare(s); };
private:
  std::string to_string_aux(const std::function<std::string(const int&)> &regts, 
                            const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const;
  void print_aux(const std::function<std::string(const int&)> &regts, 
                 const std::function<std::string(const Lang::MemLoc<int> &)> &mlts,
                 Log::redirection_stream &os, Log::redirection_stream &json_os) const;

  /* Get a pair (m2,nml) where m2 is a clone of m, where additionally
   * nml is declared. The memory location nml is a memory location
   * with a singleton domain. If there is a TsoFenceSync::InsInfo
   * object info in m_infos, then nml == info.fence_nml.
   */
  static std::pair<Machine*,Lang::NML> get_dummy_nml(const Machine &m, const std::vector<const Sync::InsInfo*> &m_infos);
};

#endif
