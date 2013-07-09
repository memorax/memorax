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

#ifndef __PB_CONTAINER_2_H__
#define __PB_CONTAINER_2_H__

#include "constraint_container.h"
#include "pb_constraint.h"
#include "machine.h"
#include "ticket_queue.h"

/* PbContainer is a ConstraintContainer that should be used only for
 * PbConstraints. It has subsumption by
 * PbConstraint::entailment_compare, but does not deallocate subsumed
 * constraints, only removes them and their descendants from Q.
 */
class PbContainer2 : public ConstraintContainer{
public:
  /* Constructs a PbContainer2 for constraints belonging to the machine
   * m. */
  PbContainer2(const Machine &m);
  virtual ~PbContainer2() { clear(); };
  PbContainer2(const PbContainer2&) = delete; // Use references instead
  PbContainer2 &operator=(const PbContainer2&) = delete;
  virtual void insert_root(Constraint *r);
  virtual void insert(Constraint *p, const Machine::PTransition *t, Constraint *c);
  virtual Constraint *pop();
  virtual int Q_size() const { return q_size; };
  virtual int F_size() const { return f_size; };
  virtual Trace *clear_and_get_trace(Constraint *c);
  virtual void clear();
private:
  /* A constraint in F is kept in a wrapper. */
  struct wrapper_t{
    wrapper_t(PbConstraint *c) : constraint(c), q_index(-1) {};
    PbConstraint *constraint; // The constraint
    /* Index to the position in Q of constraint.
     * If constraint is clean then constraint is at clean_q[q_index].
     * If constraint is dirty then constraint is at dirty_q[q_index].
     * q_index will not be updated when constraints are popped.
     */
    long q_index;
    /* A transition backward or forward from this constraint to the
     * constraint of target.
     */
    struct trans_t{
      trans_t() : transition(0), target(0) {};
      trans_t(const Machine::PTransition *t, wrapper_t *tgt) : transition(t), target(tgt) {};
      const Machine::PTransition *transition;
      wrapper_t *target;
    };
    /* The predecessor of this constraint in the analysis. */
    trans_t parent;
    /* The successors of this constraint in the analysis. */
    std::vector<trans_t> children;
  };
  /* Compares parts of a and b that need to be exactly equal for a and b to be comparable.
   * Program counters are the exception: they are not compared but assumed equal.
   */
  class pbcmp{
  public:
    bool operator()(PbConstraint * const &a, PbConstraint * const &b);
  };
  /* The set of constraints in F.
   *
   * pcs_to_f[i] contains all constraints c with i == pc_index(c).
   *
   * pcs_to_f[i][c] is a vector containing all constraints that are
   * considered equal to c by pbcmp.
   */
  std::vector<std::map<PbConstraint*,std::vector<wrapper_t*>,pbcmp> > pcs_to_f;
  int pc_index(PbConstraint *c) const;
  /* The number of constraints currently in F */
  int f_size;
  /* Q is divided into clean_q and dirty_q, for respectively the clean
   * constraints and the dirty ones.
   *
   * clean_q and dirty_q may also contain null-pointers. They are not
   * entries in Q, and can be ignored. They are places where
   * constraints have been removed from the queue. 
   */
  TicketQueue<wrapper_t*> clean_q;
  TicketQueue<wrapper_t*> dirty_q;
  int q_size;

  /* Caches the wrapper of the previous constraint that was popped from Q */
  wrapper_t *prev_popped;
  /* prev_popped points here before anything has been popped 
   * All fields are null. */
  wrapper_t dummy_wrapper;

  /* Original sizes for clean_q, dirty_q */
  static const int Q_SIZE = 10000;

  int proc_count; // The number of processes in machine
  int max_state_count; // The largest number of automaton states of any process in machine.

  /* If c is present in F, then c is deallocated and null is returned.
   *
   * If c is not present in F, then c is inserted into pcs_to_f. A
   * pointer to the wrapper around c is returned. If there are
   * constraints in F that are subsumed by c, then they are
   * removed from Q together with their descendants.
   */
  wrapper_t *insert_in_f(PbConstraint *c);
  /* Remove w and all of its descendants from Q */
  void remove_from_q(wrapper_t *w);
  /* Adds w to Q.
   * Points w->q_index to the new entry in Q. */
  void insert_in_q(wrapper_t *w);
  /* Returns true if c is present in F *and* the constraint c' in F
   * that equals *c is the same object as c (c and c' are the same
   * pointer: c == c').
   */
  bool pointer_in_f(PbConstraint *c);
  /* Returns the wrapper of c in F.
   *
   * Pre: c is present in F
   */
  wrapper_t *get_wrapper(PbConstraint *c);
};

#endif

