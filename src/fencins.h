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

#ifndef __FENCINS_H__
#define __FENCINS_H__

#include "machine.h"
#include "reachability.h"
#include "sync.h"
#include "trace_fencer.h"

#include <functional>
#include <set>

namespace Fencins{
  /* A cost_fn_t object gives the cost of a particular piece of
   * synchronization. The fence insertion algorithm will prefer
   * cheaper synchronization over more expensive ones.
   *
   * For each Sync s, and each cost_fn_t f, it should hold that f(&s)
   * >= 0.
   */
  typedef std::function<int(const Sync*)> cost_fn_t;

  /* Describes an aspect with respect to which fencins minimizes
   * synchronization sets.
   */
  enum min_aspect_t{
    /* Minimize w.r.t. the sum of the costs of synchronizations as
     * given by a cost function. */
    COST,
    /* Minimize w.r.t. set inclusion. */
    SUBSET
  };

  /* Repeatedly applies r to reach_arg_init(m,prev_result) in order to
   * detect which synchronization must be inserted in order for the
   * forbidden states of m to become unreachable. tf will be used to
   * identify synchronization that can be used to prevent certain
   * traces through m.
   *
   * Returns a set S of sets Z such that each Z contains
   * synchronization sufficient to make the forbidden states in m
   * unreachable. Each Z is minimal with respect to Sigma_{z in
   * Z}(cost(z)). If no synchronization insertion can make the
   * forbidden states in m unreachable then S is empty. Otherwise if
   * only_one is set, then S is a singleton. Otherwise if only_one is
   * unset, then S contains all minimal, sufficient sets Z.
   *
   * For each reachability analysis an argument
   * reach_arg_init(m',prev_result) will be used. The machine m' will
   * be a version of m with some synchronization inserted. The Result
   * prev_result will be the previous result produced by the
   * reachability analysis, or 0 if there has been no previous
   * reachability analysis.
   */
  typedef std::function<Reachability::Arg*(const Machine&,const Reachability::Result*)> reach_arg_init_t;
  std::set<std::set<Sync*> > fencins(const Machine &m,
                                     Reachability &r,
                                     reach_arg_init_t reach_arg_init,
                                     TraceFencer &tf,
                                     min_aspect_t ma,
                                     bool only_one = true,
                                     cost_fn_t cost = [](const Sync*){return 1;});

  void test();
};

#endif
