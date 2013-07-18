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

#ifndef __TSO_SIMPLE_FENCER_H__
#define __TSO_SIMPLE_FENCER_H__

#include "trace_fencer.h"
#include "tso_fence_sync.h"
#include "vecset.h"

#include <list>

/* TsoSimpleFencer is a TraceFencer for TSO traces. For a trace t, a
 * TsoSimpleFencer will compute all synchronizations that prevent some
 * W->R reordering in t.
 *
 * TsoSimpleFencer considers two kinds of synchronization: by locking
 * writes and by inserting fence instructions. The behaviour can be
 * fine-tuned per TsoSimpleFencer object (see constructor).
 */
class TsoSimpleFencer : public TraceFencer{
public:
  /* A fence_rule_t describes the rules for synchronization that this
   * fencer will consider inserting. */
  enum fence_rule_t {
    /* Consider only synchronization by locking writes. */
    LOCKED,
    /* Consider only synchronization by inserting fence
     * instructions. */
    FENCE,
    /* Consider both synchronization by locking writes and by
     * inserting fence instructions. */
    LOCKED_AND_FENCE
  };
  /* Construct a TsoSimpleFencer that will only insert synchronization
   * adhering to rule.
   */
  TsoSimpleFencer(const Machine &m, fence_rule_t rule);
  TsoSimpleFencer(const TsoSimpleFencer&) = default;
  virtual TsoSimpleFencer &operator=(const TsoSimpleFencer&) = default;
  virtual ~TsoSimpleFencer();
  virtual std::set<std::set<Sync*> > fence(const Trace &t, const std::vector<const Sync::InsInfo*> &m_infos) const;

  static void test();
private:
  /* The rule for synchronization inserted by this fencer. */
  fence_rule_t fence_rule;
  /* All fences that can be inserted into machine. */
  std::set<TsoFenceSync*> all_fences;
  /* For each process p with a control state q, fences_by_pq[p][q] is
   * the subset of all_fences corresponding to that process and
   * control state.
   *
   * The pointers point to the same objects as those in all_fences.
   */
  std::vector<std::vector<std::set<TsoFenceSync*> > > fences_by_pq;

  /* Return the subset of all_fences of fences of process pid that fit
   * immediately between in and out.
   */
  std::set<Sync*> fences_between(int pid,
                                 const Automaton::Transition &in,
                                 const Automaton::Transition &out,
                                 const std::vector<const Sync::InsInfo*> &m_infos) const;

  /* Returns true iff all memory locations read by t exist in the
   * union of all elements of buf.
   */
  bool is_ROWE(const Machine::PTransition *t,
               const std::list<VecSet<Lang::MemLoc<int> > > &buf) const;
};

#endif
