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

#ifndef __VIPS_SIMPLE_FENCER_H__
#define __VIPS_SIMPLE_FENCER_H__

#include "trace_fencer.h"
#include "vips_fence_sync.h"
#include "vips_syncwr_sync.h"

#include <map>

/* VipsSimpleFencer is a Fencer for traces under the VIPS-M
 * semantics. It will identify all fences that prevent some memory
 * reordering occuring in a given trace.
 */
class VipsSimpleFencer : public TraceFencer{
public:
  VipsSimpleFencer(const Machine &m);
  VipsSimpleFencer(const VipsSimpleFencer&) = default;
  VipsSimpleFencer &operator=(const VipsSimpleFencer&) = default;
  ~VipsSimpleFencer();
  virtual std::set<std::set<Sync*> > fence(const Trace &t, const std::vector<const Sync::InsInfo*> &m_infos) const;

  /* Return a trace t' with the same trace graph (c.f. Shasha, Snir
   * 1988) as t. In t' transitions have been interleaved in a
   * different manner, and system events have been timed differently,
   * according to heuristics meant to decrease the number of
   * irrelevant instruction reorderings.
   */
  static Trace *decrease_reorderings(const Trace &t);

  static void test();
private:
  std::set<VipsFenceSync*> all_fences;
  std::set<VipsSyncwrSync*> all_syncwrs;

  /* For each process p with a control state q, fences_by_pq[p][q] is
   * the subset of all_fences corresponding to that process and
   * control state.
   *
   * The pointers point to the same objects as those in all_fences.
   */
  std::vector<std::vector<std::set<VipsFenceSync*> > > fences_by_pq;
  /* For each process p with a control state q, syncwrs_by_pq[p][q] is
   * the subset of all_syncwrs corresponding to that process and a
   * write with source control state q.
   *
   * The pointers point to the same objects as those in all_syncwrs.
   */
  std::vector<std::vector<std::set<VipsSyncwrSync*> > > syncwrs_by_pq;

  /* Returns a map m such that for each transition index i into t,
   * m[i] is the synchronization point of t[i] in t.
   */
  static std::map<int,int> get_sync_points(const Trace &t);

  /* Returns a vector m such that for each constraint index i into t,
   * the processes in m[i] are precisely those p such that there
   * exists indices j <= i < k where t[j]->pid == t[k]->pid == p and
   * the synchronization point of t[k] strictly preceeds that of t[j].
   *
   * The returned vector has the same size as the number of
   * constraints in t.
   *
   * sps should be the synchronization points of t, as given by
   * get_sync_points.
   */
  static std::vector<std::set<int> > get_reordered_procs(const Trace &t,
                                                         const std::map<int,int> &sps);

  /* Returns a vector v such that for each transition index i in t,
   * the set v[i] contains precisely the transition indices j in t
   * such that t[i] and t[j] are memory accessing transitions of the
   * same process, and the synchronization points of i and j are in
   * the opposite order of i and j
   *
   * (i.e. i != j && sps[i] != sps[j] && (i < j) == (sps[j] < sps[i])).
   *
   * sps should be the synchronization points of t, as given by
   * get_sync_points.
   */
  static std::vector<std::set<int> > get_reordered_transes(const Trace &t,
                                                           const std::map<int,int> &sps);

  /* Returns the least j such that i < j and t[j] is a transition of
   * the same process as t[i], and t[j] is an instruction transition
   * (not a system event).
   *
   * If there is no such j, then 0 is returned.
   */
  static int get_next_instr(const Trace &t, int i);
  /* Returns the greatest j such that j < i and t[j] is a transition
   * of the same process as t[i], and t[j] is an instruction
   * transition (not a system event).
   *
   * If there is no such j, then 0 is returned.
   */
  static int get_prev_instr(const Trace &t, int i);

  /* Returns true iff s is a CAS statement */
  static bool is_cas(const Lang::Stmt<int> &s);

  /* Returns true iff s is a fetch, wrllc or evict */
  static bool is_sys_event(const Lang::Stmt<int> &s);

  /* Return the subset of all_fences of fences of process pid that fit
   * immediately between in and out.
   *
   * Always returns full fences. Also returns ssfences (and llfences)
   * if have_ssfences (and have_llfence) is set.
   */
  std::set<Sync*> fences_between(int pid,
                                 const Automaton::Transition &in,
                                 const Automaton::Transition &out,
                                 const std::vector<const Sync::InsInfo*> &m_infos,
                                 bool have_ssfence, bool have_llfence) const;

  /* Helper for fence(t,m_infos). Returns the syncs of
   * fence(t,m_infos) that are VipsSyncwrSyncs.
   *
   * All returned syncs are pointers into all_syncwrs.
   */
  std::set<Sync*> fence_syncwr(const Trace &t, const std::vector<const Sync::InsInfo*> &m_infos) const;

  /* Return a trace following the same path as t, but with all
   * synchronization described in m_infos removed.
   */
  Trace *unsync_trace(const Trace &t, FenceSync::m_infos_t m_infos) const;
};

#endif
