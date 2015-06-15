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

#ifndef __CONSTRAINT_H__
#define __CONSTRAINT_H__

#include <vector>
#include <list>
#include "automaton.h"
#include "machine.h"
#include "log.h"

class Constraint{
public:
  /* Common contains various information that is to be shared between
   * many constraints. E.g. caches, preprocessed information about the
   * problem etc.
   *
   * Common classes that are used by derived Constraints should be
   * derived from Constraint::Common.
   */
  class Common{
  public:
    virtual ~Common() {};
    /* A human-readable string representation on multiple lines. */
    virtual std::string to_string() const = 0;
    /* A verbose human-readable string representation on multiple
     * lines. */
    virtual std::string to_verbose_string() const{
      return to_string();
    };
  };
  virtual ~Constraint() {};
  virtual const std::vector<int> &get_control_states() const throw() = 0;
  /* Returns the set of transitions that should be explored from this
   * constraint.
   */
  virtual std::list<const Machine::PTransition*> partred() const = 0;
  /* The returned constraints are allocated on heap. Ownership is left
   * to the caller. Returned constraints are not abstracted.
   */
  virtual std::list<Constraint*> pre(const Machine::PTransition &) const {
    throw new std::logic_error("Constraint::pre: Not implemented.");
  };
  /* The returned constraints are allocated on heap. Ownership is left
   * to the caller. Returned constraints are not abstracted.
   */
  virtual std::list<Constraint*> post(const Machine::PTransition &) const {
    throw new std::logic_error("Constraint::post: Not implemented.");
  };
  /* Returns constraint describing exactly the part of this constraint
   * which is in the domain of the transition trans. Returned
   * constraints are not abstracted.
   */
  virtual std::list<Constraint*> domain(const Machine::PTransition &trans) const{
    throw new std::logic_error("Constraint::domain: Not implemented.");
  };
  /* Returns constraint describing exactly the part of this constraint
   * which is in the range of the transition trans. Returned
   * constraints are not abstracted.
   */
  virtual std::list<Constraint*> range(const Machine::PTransition &trans) const{
    throw new std::logic_error("Constraint::range: Not implemented.");
  };
  /* Lose information by applying abstraction to the constraint. 
   */
  virtual void abstract() = 0;
  /* Returns true if this constraint is abstracted, false otherwise.
   */
  virtual bool is_abstracted() const = 0;
  /* Checks if this constraint intersects with the set of initial
   * states for the machine of this constraint.
   */
  virtual bool is_init_state() const = 0;
  /* Returns true if this and c contain common configurations. Returns
   * false otherwise.
   */
  virtual bool intersects(const Constraint &c) const{
    throw new std::logic_error("Constraint::intersects: Not implemented.");
  };
  enum Comparison { LESS , EQUAL , GREATER , INCOMPARABLE };
  /* Compares this to c
   * Returns LESS if this is smaller than c.
   * Returns GREATER if this is greater than c.
   * Returns EQUAL if this is equal to c, and INCOMPARABLE otherwise.
   */
  virtual Comparison entailment_compare(const Constraint &c) const{
    throw new std::logic_error("Constraint::entailment_compare: Not implemented.");
  };
  static Comparison comb_comp(Comparison a, Comparison b){
    if(a == INCOMPARABLE || b == INCOMPARABLE){
      return INCOMPARABLE;
    }
    if(a == EQUAL){
      return b;
    }
    if(b == EQUAL){
      return a;
    }
    if(a == b){
      return a;
    }
    return INCOMPARABLE;
  };
  static Comparison invert_comp(Comparison c){
    if(c == LESS){
      return GREATER;
    }else if(c == GREATER){
      return LESS;
    }else{
      return c;
    }
  };
  /* A total order on constraints.
   * Used for collections.
   */
  virtual bool operator<(const Constraint &c) const{
    throw new std::logic_error("Constraint::operator<: Not implemented.");
  };
  virtual std::string to_string() const throw() = 0;
};

inline std::ostream &operator<<(std::ostream &os, const Constraint &c){
  return os << c.to_string();
}

#endif // __CONSTRAINT_H__
