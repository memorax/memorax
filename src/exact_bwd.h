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

#ifndef __EXACT_BWD_H__
#define __EXACT_BWD_H__

#include "reachability.h"
#include "constraint.h"
#include "machine.h"
#include "pb_constraint.h"
#include "constraint_container.h"

/* ExactBwd implements backward reachability. Most of the details of
 * the analysis are left to the container which is given as an
 * argument.
 */
class ExactBwd : public Reachability{
public:
  /* Arguments to the reachability analysis */
  class Arg : public Reachability::Arg{
  public:
    /* Takes ownership of all constraints in bad, of common and of cont.
     *
     * Starts from the forbidden states in bad and tries to reach an
     * initial state in the machine m using backward reachability
     * analysis. The container cont is used for storing constraints,
     * entailment checking and priority.
     */
    Arg(const Machine &m, std::list<Constraint*> bad, Constraint::Common *common, ConstraintContainer *cont)
      : Reachability::Arg(m), bad_states(bad), common(common), container(cont) {};
    /* Same as Arg(m,b,common,cont), where b are newly allocated bad states
     * based on m.forbidden and common. */
    Arg(const Machine &m, PbConstraint::Common *common, ConstraintContainer *cont);
    virtual ~Arg(){
      if(common) delete common;
      if(container) delete container;
    };
    Arg(const Arg&) = delete;
    Arg &operator =(const Arg&) = delete;
    /* Reachability analysis will start from these bad states and try
     * to reach an initial state.
     */
    std::list<Constraint*> bad_states;
    /* Carries the ownership of the common object used by constraints
     * in bad_states.
     *
     * 0 if the constraints do not use a common object.
     */
    Constraint::Common *common;
    /* The container that should be used. */
    ConstraintContainer *container;
  };

  /* pb_init_arg(a,c) returns a new Arg object with the same machine
   * and container as a, the Common object common and bad states
   * initialized based on the forbidden states of the machine.
   *
   * (This function is here to make usage of ExactBwd together with
   * PbCegar more convenient.)
   */
  static Reachability::Arg *pb_init_arg(Reachability::Arg *prev_arg,
                                        PbConstraint::Common *common){
    assert(dynamic_cast<Arg*>(prev_arg));
    ConstraintContainer *c = static_cast<Arg*>(prev_arg)->container;
    static_cast<Arg*>(prev_arg)->container = 0;
    return new Arg(prev_arg->machine,common,c);
  };

  class Result : public Reachability::Result{
  public:
    Result(const Machine &m) : Reachability::Result(m), common(0) {};
    virtual ~Result(){
      if(common) delete common;
    };
    /* The Common used by Constraints in the trace. (owned)
     * 0 if the Constraints do not use a common object. */
    Constraint::Common *common;
  };

  /* Pre: arg should be of type ExactBwd::Arg
   */
  virtual Reachability::Result *reachability(Reachability::Arg *arg) const;
};

#endif
