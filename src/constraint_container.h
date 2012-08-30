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

#ifndef __CONSTRAINT_CONTAINER_H__
#define __CONSTRAINT_CONTAINER_H__

#include "constraint.h"
#include "machine.h"
#include "trace.h"

/* A ConstraintContainer is meant to be used for state space
 * exploration.
 *
 * It keeps a forest F of Constraints where each root is a starting
 * Constraint (initial constraint in forward analysis, forbidden
 * constraint in backward). If a node n0 has a child node n1 then the
 * edge between them is labeled by a PTransition t, and n0 can
 * transition to n1 by executing t (whether forward or backward
 * depends on the exploration algorithm).
 *
 * A ConstraintContainer also keeps a queue Q of Constraints. Whenever
 * a new Constraint is inserted into F it is also inserted into Q. Q
 * is not guaranteed to be FIFO.
 *
 * F and Q are not guaranteed to be implemented as or perform like a
 * forest and a queue.
 *
 * Depending on implementation there may be a concept of constraints
 * subsuming other constraints.
 *
 * Depending on implementation, when a constraint is subsumed and
 * removed, all its descendants may also be removed.
 */
class ConstraintContainer{
public:
  ConstraintContainer(){};
  virtual ~ConstraintContainer() {};
  ConstraintContainer(const ConstraintContainer&) = delete; // Use references instead
  ConstraintContainer &operator=(const ConstraintContainer&) = delete;
  /* Conventions:
   *
   * A Constraint* p is said to be present in F if there is some
   * Constraint* p' in F such that either *p == *p', or if there is
   * subsumption, *p' subsumes *p.
   *
   * A Constraint* p is said to point into F if there is some
   * Constraint* p' in F such that p == p'.
   */

  /* If r is present in F, then *r is deallocated.
   *
   * If r is not present in F, then a tree consisting only of r is
   * added to F and r is pushed onto Q. If there is subsumption then
   * all constraints r' in F which are subsumed by r, are deallocated
   * and removed from F and Q.
   */
  virtual void insert_root(Constraint *r) = 0;
  /* If c is present in F, then *c is deallocated.
   *
   * If c is not present in F, then c is added to F as a child of p,
   * with the transition between them labeled t. Furthermore c is
   * pushed onto Q. If there is subsumption then all constraints c' in
   * F which are subsumed by c, are deallocated and removed from F and
   * Q.
   */
  virtual void insert(Constraint *p, const Machine::PTransition *t, Constraint *c) = 0;
  /* If Q is empty then null is returned.
   * 
   * If Q is not empty, then some Constraint* c in Q is popped from Q
   * and returned.
   */
  virtual Constraint *pop() = 0;
  /* Returns the number of constraints currently in Q. */
  virtual int Q_size() const = 0;
  /* Returns the number of constraints currently in F */
  virtual int F_size() const = 0;
  /* Returns a trace starting at a root constraint and leading to c
   * through F.
   *
   * Clears F and Q. Deallocates all Constraints in F except those
   * that occur in the returned trace.
   */
  virtual Trace *clear_and_get_trace(Constraint *c) = 0;
  /* Clears F and Q. Deallocates all Constraints in F. */
  virtual void clear() = 0;
};

#endif
