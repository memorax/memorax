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

#ifndef __REACHABILITY_2_H__
#define __REACHABILITY_2_H__

#include "trace.h"
#include "machine.h"
#include "timer.h"

/* Reachability is an abstract class that should be inherited by
 * classes implementing reachability analysis.
 */
class Reachability{
public:
  virtual ~Reachability() {};
  /* The possible outcomes of a reachability analysis. */
  enum result_t {
    REACHABLE,
    UNREACHABLE,
    FAILURE
  };

  /* A Result object expresses the result of a reachability
   * analysis. Overloading the Result class to include algorithm
   * specific details is encouraged.
   */
  class Result{
  public:
    Result(const Machine &m);
    virtual ~Result(){
      if(trace)
        delete trace;
    };
    /* The outcome */
    result_t result;
    /* In case result == REACHABLE, trace will contain a witnessing
     * trace.
     *
     * trace is owned.
     */
    Trace *trace;
    /* A human-readable string representation spanning multiple
     * lines. */
    virtual std::string to_string() const;
    /* Keeping track of the time consumption of the analysis */
    Timer timer;
    /* Number of constraints visited */
    int generated_constraints;
    /* Number of constraints stored */
    int stored_constraints;
  protected:
    const Machine &machine;
  };

  /* An Arg object contains the arguments to a reachability
   * analysis. Overloading is encouraged.
   */
  class Arg{
  public:
    Arg(const Machine &m) : machine(m) {};
    virtual ~Arg(){};
    /* The machine that should be analysed. */
    const Machine &machine;
  };

  /* Performs the reachability analysis on the problem defined by arg.
   * Returns a Result (or derived class) allocated on heap. This
   * object does not take ownership of arg. Ownership of the returned
   * Result is given to the caller.
   */
  virtual Result *reachability(Arg *arg) const = 0;
};

#endif
