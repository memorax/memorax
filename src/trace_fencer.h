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

#ifndef __TRACE_FENCER_H__
#define __TRACE_FENCER_H__

#include "sync.h"
#include "trace.h"

#include <set>

/* TraceFencer is an abstract class that should be extended by classes
 * that analyse traces, and compute sets of synchronization that can
 * prevent traces.
 */
class TraceFencer{
public:
  /* Construct a TraceFencer for traces through m and through
   * synchronized versions of m.
   */
  TraceFencer(const Machine &m) : machine(m) {};
  TraceFencer(const TraceFencer&) = delete;
  TraceFencer &operator=(const TraceFencer&) = delete;
  virtual ~TraceFencer(){};
  /* Pre: t should run through the Machine m', where m' is the result
   * of applying synchronization to the Machine m as described by
   * m_infos. Here m is assumed to be the Machine for which this
   * TraceFencer was created. Earlier entries in m_infos correspond to
   * synchronization applied earlier.
   *
   * Returns a set S with the following properties.
   *
   * For all s in S, and all z in s, z is a synchronization for m.
   *
   * If S is empty, then the Shasha-Snir trace graph of t is acyclic.
   *
   * All s in S are non-empty.
   *
   * Let c be the final configuration of t. Consider the set F of all
   * sets f of synchronization such that 1) f is sufficient to make c
   * unreachable in m', and 2) f is minimal in the sense that any
   * strict subset of f is insufficient in the sense of 1). Note that
   * F is the empty set if no synchronization can make c unreachable
   * in m'.
   *
   * For every f in F and for every s in S, the intersection of f and
   * s is non-empty.
   *
   * Ownership of all synchronization that is returned is surrendered
   * to the caller.
   *
   * *** Intuition ***
   *
   * Typically it should be the case that each set s in S corresponds
   * to some reordering in t that *must* be prevented, and the
   * synchronizations in s are the different alternative ways to
   * prevent it. One way to implement fence is to return a singleton
   * set S = {s} where s is the set of all synchronizations that
   * prevent any reordering occuring in t. Another, naive, way is to,
   * regardless of t return the singleton set S = {s} where s is all
   * synchronization that can be inserted into m'. The sets s in S may
   * be pruned according to some normal form on synchronization in
   * order to decrease searching in the main fence insertion
   * procedure.
   */
  virtual std::set<std::set<Sync*> > fence(const Trace &t, const std::vector<const Sync::InsInfo*> &m_infos) const = 0;
protected:
  /* The original machine through which, or through synchronized
   * versions of which, analyzed traces should run. */
  const Machine &machine;
};

#endif
