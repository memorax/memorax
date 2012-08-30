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

#ifndef __TRACE_H__
#define __TRACE_H__

#include "constraint.h"
#include "machine.h"

/* A trace is a sequence c0,t1,c1,t2,...,tn,cn, where all ci are
 * Constraints, and all ti are transitions taking ci-1 into ci,
 * according to some semantic. The Trace class can be used by
 * different kinds of constraints and different kinds of reachability
 * analyses. Therefore the precise semantic may vary.
 *
 * Constraints ci and transitions ti are owned by the Trace object.
 */
class Trace{
public:
  /* Constructs a trace of length 0, with start (and end) constraint
   * c0.
   *
   * c0 may be 0.
   *
   * Takes ownership of c0.
   */
  Trace(Constraint *c0);
  Trace(const Trace&) = delete;
  const Trace &operator=(const Trace&) = delete;
  ~Trace();
  /* Adds t,c to the end of this trace.
   *
   * c may be 0.
   *
   * Takes ownership of c.
   */
  void push_back(const Machine::PTransition &t,Constraint *c);
  /* Adds c,t to the beginning of this trace.
   *
   * c may be 0.
   *
   * Takes ownership of c.
   */
  void push_front(Constraint *c,const Machine::PTransition &t);
  /* Returns the number of transitions in this trace. */
  int size() const throw() { return trace_vec.size()-1; };
  /* Returns the i:th transition in this trace.
   * Counts indices from 1 to size().
   */
  const Machine::PTransition *transition(int i) const{
    if(i < 1 || i >= int(trace_vec.size())) 
      throw new std::logic_error("Trace::transition: Index out of bounds.");
    return trace_vec[i].trans;
  };
  const Machine::PTransition *operator[](int i) const { return transition(i); };
  /* Returns the i:th constraint in this trace.
   * Counts indices from 0 to size().
   *
   * Note that the returned pointer may be 0.
   */
  const Constraint *constraint(int i) const{ return trace_vec[i].constr; };
  /* Returns p+1, where p is the greatest process id occurring in this
   * trace. */
  int get_proc_count() const throw();
  /* A human-readable string representation over multiple lines.
   *
   * If include_constraints == true, then constraints and transitions
   * are included in the string, otherwise only transitions are
   * included.
   *
   * If proc_indent == true, then every entry in the trace is indented
   * by an amount depending on which process it belongs to.
   */
  std::string to_string(const Machine &m, bool include_constraints = true, bool proc_indent = true) const;
  /* A (less) human-readable string representation over multiple lines. */
  std::string to_string(bool include_constraints = true, bool proc_indent = true) const;
  /* The print methods work as the to_string methods, but instead of
   * returning a string, they distribute it and print it to the three
   * streams trans_os, constr_os, json_os.
   *
   * - trans_os receives all transitions
   * - constr_os receives all constraints
   * - json_os receives JSON meta-data
   *
   * Hint: Use with e.g. Log::msg, Log::debug, Log::json
   */
  void print(Log::redirection_stream &trans_os, Log::redirection_stream &constr_os, Log::redirection_stream &json_os,
             const Machine &m, bool include_constraints = true, bool proc_indent = true) const;
  void print(Log::redirection_stream &trans_os, Log::redirection_stream &constr_os, Log::redirection_stream &json_os,
             bool include_constraints = true, bool proc_indent = true) const;
private:
  /* A human-readable string representation over multiple lines.
   *
   * Transitions t will be represented as tts(t).
   */
  std::string to_string(std::function<std::string(Machine::PTransition*)> &tts,
                        Log::redirection_stream *trans_os, Log::redirection_stream *constr_os, Log::redirection_stream *json_os,
                        bool include_constraints, bool proc_indent) const;
  struct trace_elem_t{
    trace_elem_t(Machine::PTransition *t, Constraint *c) : trans(t), constr(c) {};
    Machine::PTransition *trans; // Owned
    Constraint *constr;          // Owned. May be 0.
  };
  /* The trace.
   *
   * In the trace c0,t1,c1,t2,...,tn,cn, the constraint ci is
   * trace_vec[i-1].constr and the transition ti is trace_vec[i].trans.
   *
   * Invariant: trace_vec[0].trans == 0.
   */
  std::vector<trace_elem_t> trace_vec;
};

#endif
