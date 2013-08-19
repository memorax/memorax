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

#ifndef __TSO_FENCINS_H__
#define __TSO_FENCINS_H__

#include <list>
#include "automaton.h"
#include "machine.h"
#include "reachability.h"
#include "trace.h"
#include "tso_cycle.h"

namespace TsoFencins{

  /* Represents a cycle in a trace.
   *
   * All PTransition pointers are pointers into the trace.
   */
  class cycle_t{
  public:
    cycle_t(const TsoCycle &c, const Machine::PTransition *w, const Machine::PTransition *r)
      : cycle(c), write(w), read(r) {};
    /* The cycle */
    TsoCycle cycle;
    /* The cycle is enabled in the trace by the reordering of these
     * instructions. */
    const Machine::PTransition *write;
    const Machine::PTransition *read;
  };

  /* A set of fences.
   *
   * A FenceSet is a set W of non-locked writes occuring in a
   * machine M. It represents the set of fences corresponding to making
   * each of the writes locked.
   */
  class FenceSet{
  public:
    /* Constructs a set of fences for the machine m, corresponding to
     * making each non-atomi write in f locked.
     *
     * Pre: Every transition in f is a non-locked write in m.
     */
    FenceSet(const Machine &m,const std::set<Machine::PTransition> f);
    /* Same as FenceSet(m,std::set<Machine::PTransition>()) */
    FenceSet(const Machine &m);
    /* Get the machine M. */
    const Machine &get_machine() const { return machine; };
    /* Get the machine M with fences inserted. */
    const Machine &get_atomized_machine() const { return atomized_machine; };
    /* Get the writes W which should be atomized. */
    const std::set<Machine::PTransition> &get_writes() const { return writes; };
    /* Add t to W
     *
     * Pre: t is a non-locked write in M
     */
    void insert(const Machine::PTransition &t);
    /* Returns the number of occurrences (0 or 1) of t in W. */
    int count(const Machine::PTransition &t) const { return writes.count(t); };
    /* Iterators over W */
    typedef std::set<Machine::PTransition>::const_iterator const_iterator;
    const_iterator begin() const { return writes.begin(); };
    const_iterator end() const { return writes.end(); };
    /* Adds fences to W that are necessary to prevent the cycle cycle
     * in trace.
     *
     * Returns the extended fence set.
     *
     * Pre: cycle is a cycle in trace.
     *      trace is a trace in the machine get_atomized_machine
     */
    FenceSet atomize(const cycle_t &cycle, const Trace &trace) const;
    void print(Log::redirection_stream &text, Log::redirection_stream &json) const throw();

    /* Returns true iff this fence set is empty */
    bool empty() const throw() { return writes.empty(); }

    /* Returns true iff the W of fs is a subset of the W of this. */
    bool includes(const FenceSet &fs) const;
  private:
    /* Non-atomized machine M */
    Machine machine;
    /* Atomized machine 
     * Same as M, but with each write in writes made locked. */
    Machine atomized_machine;
    /* The set W of non-locked writes that should be made locked.
     *
     * Invariant: Every transition in writes is a non-locked write in machine.
     */
    std::set<Machine::PTransition> writes;
  };

  /* Repeatedly applies r to reach_arg_init(m,prev_result) in order to
   * detect which writes must be made locked in order for the
   * forbidden states of m to become unreachable. Returns a list l
   * containing precisely for each minimal set F of fences that make
   * the forbidden states unreachable, the machine m' which is the
   * result of making locked the writes suggested by F. If only_one ==
   * true, then the algorithm will stop early, and l will only contain
   * one (the least) such m'.
   *
   * For each reachability analysis an argument
   * reach_arg_init(m,prev_result) will be used. The machine m will be
   * a version of m with some writes atomized. The Result prev_result
   * will be the previous result produced by the reachability
   * analysis, or 0 if there has been no previous reachability
   * analysis.
   */
  typedef std::function<Reachability::Arg*(const Machine&,const Reachability::Result*)> reach_arg_init_t;
  std::list<FenceSet> fencins(const Machine &m, 
                              Reachability &r, 
                              reach_arg_init_t reach_arg_init,
                              bool only_one = true);

  /* Returns all cycles in trace that are enabled by some TSO
   * reordering in trace. Handles both TSO and PSO traces.
   */
  std::list<cycle_t> find_cycles(const Trace &trace);

  /* Checks whether the wi:th transition in trace is a write w (or an
   * update u corresponding to the write w), and the ri:th transition
   * in trace is a read r by the same process as w, such that w is
   * non-locked in m, wi < ri and it is possilbe to delay the update
   * of w until after r without making any other changes to trace.
   *
   * If this function returns true then reordering is guaranteed
   * possible, but if the function returns false, reordering is not
   * necessarily impossible.
   *
   * Note also that this function assumes that trace follows TSO
   * semantics.
   */
  bool can_overtake(const Trace &trace, const Machine &m, int wi, int ri);

  /* Returns a non-empty set of transitions w. For each such w, there
   * is a transition r such that (w,r) is a critical pair of cycle,
   * and cycle can be opened by a reordering of w and r.
   * 
   * (Note that not all such w are guaranteed to be returned.)
   */
  std::set<Machine::PTransition>
  get_critical_writes(const cycle_t &cycle, const Trace &trace, const Machine &m);

  /* Returns all writes w such that (u,r) is a critical pair of cycle,
   * u is the transition where the write w reaches memory, and u can
   * be overtaken by r according to can_overtake.
   */
  std::set<Machine::PTransition>
  get_critical_writes_old(const cycle_t &cycle, const Trace &trace, const Machine &m);

  /* Returns true iff there is exactly one critical pair (a,b) in
   * cycle where a and b have been reordered by a TSO overtaking.
   */
  bool cycle_pairs_1_reordering(const cycle_t &cycle, const Trace &trace, const Machine &m);

  /* Let w0 be the overtaken write that opens cycle. Let u0 be the
   * update that corresponds to w0.
   *
   * Returns true iff there are no two transitions t, t' that both
   * occur after (and including) w0 but before (and including) u0,
   * such that t is not part of cycle, but t and t' have a Shasha-Snir
   * conflict.
   */
  bool cycle_no_extra_conflict(const cycle_t &cycle, const Trace &trace, const Machine &m);

  /* Returns true iff there is a transition f in trace that comes
   * after (and including) the wi:th position and before (and
   * including) the ri:th position, s.t. f belongs to the same process
   * as the wi:th transition in trace and f is a fencing transition in
   * m.
   *
   * Pre: the wi:th and ri:th transition in trace belong to the same
   * process.
   */
  bool fence_between(const Trace &trace, const Machine &m, int wi, int ri);

  /* Returns the transition from m which equals w modulo atomicity. Handles both
   * TSO and PSO semantics.
   *
   * Pre: w is a write transition, a slocked write transition or an locked write
   *        instruction.
   *      There is exactly one transition in m which equals w modulo atomicity.
   */
  Machine::PTransition get_transition_from_machine(const Machine &m, const Machine::PTransition &w);

  /* Returns the index i, such that trace[i] == t.
   * 
   * Pre: There is an index i such that trace[i] == t.
   */
  int index(const Trace &trace, const Machine::PTransition *t);

  /* If t is an update corresponding to the writing transition w, then
   * index(trace,w) is returned. Otherwise index(trace,t) is returned.
   */
  int committed_index(const Trace &trace, const Machine::PTransition *t);

  /* Returns (a,b) where a (resp. b) is the least (resp. greatest)
   * index or committed_index in trace of any transition in cycle.
   */
  std::pair<int,int> get_cycle_bounds(const cycle_t &cycle, const Trace &trace);
  
  /* Returns a map which maps pointers to writes in trace to the corresponding
   * pointers to updates in trace. Handles both TSO and PSO semantics.
   */
  std::map<const Machine::PTransition*,const Machine::PTransition*>
  pair_writes_with_updates(const Trace &trace);
};

#endif
