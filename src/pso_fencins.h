/*
 * Copyright (C) 2013 Magnus Lång
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

#ifndef __PSO_FENCINS_H__
#define __PSO_FENCINS_H__

#include "tso_fencins.h"

namespace PsoFencins{

  /* Represents a cycle in a trace.
   *
   * All PTransition pointers are pointers into the trace.
   */
  class cycle_t{
  public:
    cycle_t(const TsoFencins::cycle_t &c)
      : cycle(c.cycle), write1(c.write), read(c.read), is_write_write(false) {};
    cycle_t(const TsoCycle &c, const Machine::PTransition *w, const Machine::PTransition *r)
      : cycle(c), write1(w), read(r), is_write_write(false) {};
    cycle_t(const TsoCycle &c, const Machine::PTransition *w1, const Machine::PTransition *o, bool is_write_write)
      : cycle(c), write1(w1), is_write_write(is_write_write) {
      if (is_write_write)
        write2 = o;
      else
        read = o;
    };
    /* The cycle */
    TsoCycle cycle;
    bool is_write_write;
    /* If is_write_write, the cycle is enabled by reordering write1 after
     * write2, otherwise, the cycle is enabled by reordering write1 after
     * read. */
    const Machine::PTransition *write1, *write2;
    const Machine::PTransition *read;

    TsoFencins::cycle_t to_tso() const {
      return TsoFencins::cycle_t(cycle, write1, is_write_write ? write2 : read);
    }
  };

  class FenceSet {
  public:
    FenceSet(const Machine &m) : machine(m), atomized_machine(m) {};

    /* Get the machine M. */
    const Machine &get_machine() const { return machine; };
    /* Get the machine M with fences inserted. */
    const Machine &get_atomized_machine() const { return atomized_machine; };

    /* Adds fences to W that are necessary to prevent the cycle cycle
     * in trace.
     *
     * Returns the extended fence set.
     *
     * Pre: cycle is a cycle in trace.
     *      trace is a trace in the machine get_atomized_machine
     */
    FenceSet atomize(const cycle_t &cycle, const Trace &trace) const;
    void print(Log::redirection_stream &text, Log::redirection_stream &json) const;

    /* Returns true iff this fence set is empty */
    bool empty() const throw() { return slocks.empty() && mlocks.empty(); }

    /* Returns the set of transitions in get_machine that are Store-Store or
     * Store-Load locked respectively in this fence set. No transition will be
     * in both. */
    const std::set<Machine::PTransition> &get_slocks() const { return slocks; };
    const std::set<Machine::PTransition> &get_mlocks() const { return mlocks; };

    /* Returns true iff fs.mlocks subsetof mlocks and fs.slocks subsetof (mlocks
     *  union slocks) */
    bool includes(const FenceSet &fs) const;
  private:
    /* Adds t to slocks or mlocks
     *
     * Pre: t is a write transition in machine.
     */
    void insert_slock(const Machine::PTransition &t);
    void insert_mlock(const Machine::PTransition &t);

    /* Non-atomized machine M */
    const Machine machine;
    /* Atomized machine
     *
     * Same as M, but with each write in slocks made
     * store-store locked and each write in mlocks made load-store locked. */
    Machine atomized_machine;
    /* The set W of non-locked writes that should be made store-store and load-store locked respectively.
     *
     * Invariant: Every transition in slocks is a non-locked write in machine.
     *            Every transition in mlocks is a non-load-store-locked write in machine.
     *            slocks cut mlocks = {}
     */
    std::set<Machine::PTransition> slocks, mlocks;
  };

  std::list<FenceSet> fencins(const Machine &m, Reachability &r,
                              TsoFencins::reach_arg_init_t reach_arg_init,
                              bool only_one);

  /* Returns all cycles in trace that are enabled by either some TSO or PSO
   * reordering in trace.
   */
  std::list<cycle_t> find_cycles(const Trace &trace);
};

#endif
