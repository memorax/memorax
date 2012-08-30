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

#ifndef __CEGAR_REACHABILITY_H__
#define __CEGAR_REACHABILITY_H__

#include "reachability.h"

/* An abstract class from which classes should be derived, that
 * implement reachability through a CEGAR loop that repeatedly uses
 * another Reachability algorithm.
 */
class CegarReachability : public Reachability{
public:
  /* Arguments to a cegar reachability analysis. */
  class Arg : public Reachability::Arg{
  public:
    /* Takes ownership of abstract_reach and first */
    Arg(const Machine &m,Reachability *abstract_reach, Reachability::Arg *first,int max_loop_count)
      : Reachability::Arg(m), abstract_reach(abstract_reach), first_refinement(first), max_loop_count(max_loop_count) {};
    virtual ~Arg() {
      delete abstract_reach;
    };
    /* The underlying reachability algorithm. */
    Reachability *abstract_reach;
    /* The first arguments to use for abstract_reach. */
    Reachability::Arg *first_refinement;
    /* The maximum number of times the underlying reachability
     * analysis abstract_reach should be run during the CEGAR loop. If
     * no conclusion about the reachability is reached after
     * max_loop_count loops, the CEGAR reachability analysis fails.
     *
     * max_loop_count < 0, means that there is no limit on the number
     * of loops.
     */
    int max_loop_count;
  };

  class Result : public Reachability::Result{
  public:
    Result(const Machine &m) : Reachability::Result(m), loop_count(0), last_result(0) {};
    virtual ~Result() {
      if(last_result) delete last_result;
    };
    /* The number of loops performed during the CEGAR loop */
    int loop_count;
    /* The result from the last cegar loop, or null. */
    Reachability::Result *last_result;
    virtual std::string to_string() const;
  };

  /* Pre: arg should be of type CegarReachability::Arg
   */
  virtual Result *reachability(Reachability::Arg *arg) const;

  /* Analyzes the result result from the underlying reachability
   * algorithm executed on argument prev_arg.
   *
   * If result is deemed correct, then CORRECT is returned.
   *
   * If result is not deemed correct, then an attempt is made to
   * refine prev_arg such that the underlying reachability analysis
   * can be executed again. Upon success, next_arg is set to point to
   * a newly allocated, refined argument, and REFINED is
   * returned. Upon failure FAILURE is returned.
   */
  enum refinement_result_t{ CORRECT, REFINED, FAILURE };
  virtual refinement_result_t refine(Reachability::Result *result, 
                                     Reachability::Arg    *prev_arg,
                                     CegarReachability::Arg *cegarg,
                                     Reachability::Arg **next_arg) const = 0;

  /* Returns a human-readable multi-line string describing the next
   * argument to the underlying reachability analysis.
   */
  virtual std::string refinement_to_string(const Reachability::Arg *refinement) const{
    return ""; // Overload me!
  };
};

#endif
