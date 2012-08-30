/*
 * Copyright (C) 2012 Carl Leonardsson
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

#ifndef __TSO_CYCLE_LOCK_H__
#define __TSO_CYCLE_LOCK_H__

#include "lang.h"
#include "machine.h"
#include "tso_cycle.h"

/* A TsoCycleLock keeps track of the trace of transitions occuring
 * between a read r and an update u from the same process where r
 * overtakes u.
 * 
 * If the trace r,...,u contains a critical cycle, then the lock is
 * unlocked, otherwise it is locked.
 *
 * The case when r reads only the same variable as updated by u is a
 * special case. If that is the case, then the lock is permanently
 * locked. This corresponds to the intuition that a reordering of a
 * read and a write to the same variable is in itself meaningless; it
 * can only be made meaningful if it allows another reordering by the
 * same process.
 *
 * The intended usage is the following: During backward reachability,
 * in order to motivate an update transition we need to show that it
 * participates in a reordering which unfolds a critical cycle
 * (otherwise the reordering does not introduce any non-SC
 * behavior). Therefore updates u for process pid are only allowed
 * immediately before (i.e. after in the trace) reads r for process
 * pid. When an update is executed in the backward analysis we keep a
 * lock TsoCyclelock(r,u). No transition for process pid is allowed to
 * execute until the lock is unlocked, and then only r may
 * execute. This limits the exploration of meaningless TSO
 * reorderings, and improves performance of the analysis.
 */
class TsoCycleLock{
public:
  /* Constructs a lock for the trace r,u.
   *
   * proc_count should give the maximal number of processes that may
   * participate in the trace. Participating processes should have
   * process id in the interval [0,proc_count).
   *
   * Pre: 
   *   r->instruction reads some variable
   *   r->instruction is not a fence
   *   u->instruction.get_type() == Lang::UPDATE
   */
  TsoCycleLock(const Machine::PTransition *r, const Machine::PTransition *u, int proc_count);
  /* Returns true iff this lock is unlocked. */
  bool is_unlocked() const { return unlocked; };
  /* If this lock keeps the trace r,t0,...,tn,u, 
   * then the lock changes to keep the trace r,pt,t0,...,tn,u
   * 
   * Returns true iff the new lock is unlocked */
  bool execute(const Machine::PTransition *pt);
  /* Returns the read that was given as the r argument to the
   * construction of this lock. */
  const Machine::PTransition *get_read() const{ return r; };
  /* Returns the update that was given as the u argument to the
   * construction of this lock. */
  const Machine::PTransition *get_update() const{ return u; };
  /* Returns the process id of the process owning u and r. */
  int get_pid() const { return r->pid; };
  /* Returns a short one-line description of the lock */
  std::string to_short_string(const std::function<std::string(const Lang::MemLoc<int>&)> &mlts) const;
  /* Returns a detailed, multi-line description of the lock */
  std::string to_long_string(const std::function<std::string(const int&)> &regts,
                             const std::function<std::string(const Lang::MemLoc<int>&)> &mlts) const;
  /* Defines a total ordering over TsoCycleLocks.
   *
   * Returns -1 if this is less than cl,
   * 0 if this is equal to cl,
   * 1 if this is greater than cl.
   */
  int compare(const TsoCycleLock &cl) const throw();
  bool operator==(const TsoCycleLock &cl) const throw() { return compare(cl) == 0; };
  bool operator<(const TsoCycleLock &cl) const throw() { return compare(cl) < 0; };
  bool operator>(const TsoCycleLock &cl) const throw() { return compare(cl) > 0; };
private:
  /* If !unlocked, then all cycle fragments that have been detected in the trace.
   * If unlocked, then undefined content.
   *
   * If !unlocked and ccycles is empty, then this lock is permanently locked.
   * 
   * Let the trace be r,t0,...,tn,u
   * Then each cycle in ccycles has the form r,u,ti,...,tj with j<i
   * I.e. the cycles are stored backwards, and shifted one step.
   * This is because push_back is more efficient than push_front for TsoCycle.
   *
   * ccycles is sorted and distinct.
   */
  std::vector<TsoCycle> ccycles;
  /* true iff the trace contains some complete cycle. */
  bool unlocked;
  /* The maximal number of processes that may participate in the
   * trace. Participating processes should have process id in the
   * interval [0,proc_count).
   */
  int proc_count;
  /* The original r from construction of this lock */
  const Machine::PTransition *r;
  /* The original u from construction of this lock */
  const Machine::PTransition *u;
};

#endif
