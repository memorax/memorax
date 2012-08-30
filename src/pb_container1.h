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

#ifndef __PB_CONTAINER_1_H__
#define __PB_CONTAINER_1_H__

#include "constraint_container.h"
#include "pb_constraint.h"
#include "machine.h"
#include "ticket_queue.h"

/* PbContainer is a ConstraintContainer that should be used only for
 * PbConstraints. It has subsumption, only in that a clean constraint
 * is considered to subsume an otherwise identical dirty
 * constraint. Uses genealogy.
 */
class PbContainer1 : public ConstraintContainer{
public:
  /* Constructs a PbContainer1 for constraints belonging to the machine
   * m. */
  PbContainer1(const Machine &m);
  virtual ~PbContainer1() { clear(); };
  PbContainer1(const PbContainer1&) = delete; // Use references instead
  PbContainer1 &operator=(const PbContainer1&) = delete;
  virtual void insert_root(Constraint *r);
  virtual void insert(Constraint *p, const Machine::PTransition *t, Constraint *c);
  virtual Constraint *pop();
  virtual int Q_size() const { return q_size; };
  virtual int F_size() const { return f_size; };
  virtual Trace *clear_and_get_trace(Constraint *c);
  virtual void clear();
private:
  /* The machine to which all constraints should belong. */
  const Machine &machine;
  struct edge_t{
    edge_t() : parent(0), transition(0), child(0) {};
    edge_t(PbConstraint *p, const Machine::PTransition *t, PbConstraint *c) 
      : parent(p), transition(t), child(c) {};
    PbConstraint *parent;
    const Machine::PTransition *transition;
    PbConstraint *child;
  };
  /* A constraint in F is kept in a wrapper. */
  struct wrapper_t{
    wrapper_t(PbConstraint *c) : constraint(c), q_index(-1), edge_index(-1) {};
    PbConstraint *constraint; // The constraint
    /* Index to the position in Q of constraint.
     * If constraint is clean then constraint is at clean_q[q_index].
     * If constraint is dirty then constraint is at dirty_q[q_index].
     * q_index will not be updated when constraints are popped.
     */
    long q_index;
    /* Index to the unique edge e in edges s.t. edges[edge_index].child == constraint.
     * -1 if constraint is a root constraint and thus there is no such edge.
     */
    int edge_index;
  };
  /* Compares *a->constraint with *b->constraint. */
  class wrappercmp{
  public:
    bool operator()(wrapper_t * const &a, wrapper_t * const &b);
  };
  /* The prefix of length edge_count of edges is all edges in F.
   *
   * Edges with parent == transition == child == 0 are not counted as
   * edges. They may occur at the places where an edge has been
   * removed.
   *
   * Invariant: 
   *   There are no duplicate edges.
   *   For each constraint c in F, there is at most one edge e s.t. e.child == c.
   *   All constraints pointed to frome edges are in F.
   */
  std::vector<edge_t> edges;
  int edge_count;
  /* The set of constraints in F.
   *
   * A constraint c is kept in a wrapper in pcs_to_f[i] where i is
   * uniquely determined by the program counters in c. (i == pc_index(c)).
   */
  std::vector<std::set<wrapper_t*,wrappercmp> > pcs_to_f;
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
  TicketQueue<PbConstraint*> clean_q;
  TicketQueue<PbConstraint*> dirty_q;
  int q_size;

  /* Original sizes for clean_q, dirty_q and edges */
  static const int Q_SIZE = 10000;
  static const int EDGES_SIZE = 10000;

  int proc_count; // The number of processes in machine
  int max_state_count; // The largest number of automaton states of any process in machine.

  /* If c is present in F, then c is deallocated and null is returned.
   *
   * If c is not present in F, then c is inserted into pcs_to_f. A
   * pointer to the wrapper around c is returned. If there are
   * constraints in F that are subsumed by c, then they are
   * deallocated and removed from pcs_to_f, edges and Q.
   */
  wrapper_t *insert_in_f(PbConstraint *c);
  /* pci should be an index into pcs_to_f. it should be a valid
   * iterator into pcs_to_f[pci].
   *
   * Deallocates the constraint and the wrapper pointed to by it, and
   * removes it from pcs_to_f[pci]. Removes all edges to and from the
   * constraint of it. Removes the constraint of it from
   * Q. Recursively calles remove_from_f for all descendants of it.
   */
  void remove_from_f(int pci, std::set<wrapper_t*,wrappercmp>::iterator it);
  /* Finds pci and it such that it is a valid iterator into
   * pcs_to_f[pci] and the constraint of it is c, then calls
   * remove_from_f(pci,it).
   */
  void remove_from_f(PbConstraint *c);
  /* Adds w->constraint to Q.
   * Points w->q_index to the new entry in Q. */
  void insert_in_q(wrapper_t *w);
  /* Adds an edge edge_t(p,t,w->constraint) to edges. 
   * Points w->edge_index to the new edge. */
  void add_edge(PbConstraint *p, const Machine::PTransition *t, wrapper_t *w);
  /* Returns true if c is present in F *and* the constraint c' in F
   * that equals *c is the same object as c (c and c' are the same
   * pointer: c == c').
   */
  bool pointer_in_f(PbConstraint *c);
};

#endif
