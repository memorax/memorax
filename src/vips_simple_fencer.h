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

  static void test();
private:
  std::set<VipsFenceSync*> all_fences;

  /* For each process p with a control state q, fences_by_pq[p][q] is
   * the subset of all_fences corresponding to that process and
   * control state.
   *
   * The pointers point to the same objects as those in all_fences.
   */
  std::vector<std::vector<std::set<VipsFenceSync*> > > fences_by_pq;

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
   */
  static std::vector<std::set<int> > get_reordered_procs(const Trace &t);

  /* Returns true iff s is a CAS statement */
  static bool is_cas(const Lang::Stmt<int> &s);

  /* Returns true iff s is a fetch, wrllc or evict */
  static bool is_sys_event(const Lang::Stmt<int> &s);

  /* Return the subset of all_fences of fences of process pid that fit
   * immediately between in and out.
   */
  std::set<Sync*> fences_between(int pid,
                                 const Automaton::Transition &in,
                                 const Automaton::Transition &out,
                                 const std::vector<const Sync::InsInfo*> &m_infos) const;

  /* Return a trace t' with the same trace graph (c.f. Shasha, Snir
   * 1988) as t. In t' transitions have been interleaved in a
   * different manner, and system events have been timed differently,
   * according to heuristics meant to decrease the number of
   * irrelevant instruction reorderings.
   */
  static Trace *decrease_reorderings(const Trace &t);

};

#endif
