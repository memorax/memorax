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

#ifndef __CONSTRAINT_CONTAINER_1_H__
#define __CONSTRAINT_CONTAINER_1_H__

#include "constraint_container.h"

/* Implements a ConstraintContainer.
 *
 * F is represented by pcs_to_f, edges, edge_count.
 * - pcs_to_f represents the set of Constraints in F. 
 *   It maps from program counters to std::sets of Constraints 
 *   with those program counters.
 * - edges, edge_count represents the edges in F. The prefix of edges
 *   of length edge_count is the set of edges.
 *
 * Q is a LIFO stack represented by q, q_size. The prefix of q of
 * length q_size is all Constraints in Q ordered from older to newer.
 * 
 */
class ConstraintContainer1 : public ConstraintContainer{
public:
  /* Constructs a container with empty F and empty Q for Constraints
   * belonging to m.
   */
  ConstraintContainer1(const Machine &m);  
  virtual ~ConstraintContainer1() { clear(); };
  ConstraintContainer1(const ConstraintContainer1&) = delete;
  ConstraintContainer1 &operator=(const ConstraintContainer1&) = delete;
  virtual void insert_root(Constraint *r);
  virtual void insert(Constraint *p, const Machine::PTransition *t, Constraint *c);
  virtual Constraint *pop();
  virtual int Q_size() const { return q_size; };
  virtual int F_size() const { return f_size; };
  virtual Trace *clear_and_get_trace(Constraint *c);
  virtual void clear();
private:
  /* The machine to which all Constraints in F belong. */
  const Machine &machine;
  /* The number of processes in machine */
  int proc_count;
  /* The greatest number of program locations (automaton states) of
   * any process in machine. */
  int max_state_count;
  /* Comparison function
   *
   * Compare Constraint pointers a and b by a->operator<(*b).
   */
  class valcmp{
  public:
    bool operator()(Constraint* const &a, Constraint* const &b) const{
      return *a < *b;
    };
  };
  std::vector<std::set<Constraint*,valcmp> > pcs_to_f;
  struct edge_t{
    edge_t() : parent(0), transition(0), child(0) {};
    edge_t(Constraint *p, const Machine::PTransition *t, Constraint *c)
      : parent(p), transition(t), child(c) {};
    Constraint *parent;
    const Machine::PTransition *transition;
    Constraint *child;
  };
  /* Invariant: For all Constraints c, there is only one edge e
   * s.t. e.child == c, and all edges e' with e'.parent == c occur
   * after e in edges.
   */
  std::vector<edge_t> edges;
  int edge_count;
  std::vector<Constraint*> q;
  int q_size;
  /* The number of constraints in F */
  int f_size;

  /* The initial sizes of edges and q */
  static const int EDGES_START_SIZE = 10000;
  static const int Q_START_SIZE = 10000;

  /* An index into pcs_to_f, uniquely determined by the program counters in c. */
  int pc_index(const Constraint *c) const;
  /* Pushes c on q and expands q if necessary */
  void push_on_q(Constraint *c);
  /* Pushes edge_t(p,t,c) on edges and expands edges if necessary */
  void push_on_edges(Constraint *p, const Machine::PTransition *t, Constraint *c);
  /* Attempts to insert c into pcs_to_f.
   *
   * If c is present in F, then returns false.
   * Otherwise inserts c into pcs_to_f, increases f_size, and returns true.
   */
  bool insert_constraint_in_f(Constraint *c);
  static bool check_uniqueness(const std::set<Constraint*,valcmp> &s);
};

#endif
  

