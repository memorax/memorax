/*
 * Copyright (C) 2013 Carl Leonardsson
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

#ifndef __VIPS_BIT_CONSTRAINT_H__
#define __VIPS_BIT_CONSTRAINT_H__

#include "automaton.h"
#include "machine.h"
#include "lang.h"
#include "vecset.h"

#include <vector>

/* VipsBitConstraint is an explicit representation of a VIPS-M
 * configuration. It is meant for explicit state forward model
 * checking under VIPS-M. 
 *
 * VipsBitConstraint uses bit-packing to achieve small configurations
 * with fast comparison. For this reason it does not inherit from
 * Constraint, and does not include a reference to a Common object in
 * each configuration. For this reason, most operations will require a
 * Common object as a parameter. In general, a pre-condition is that
 * the Common object sent as argument to VipsBitConstraint methods
 * needs to be the same object that was given as an argument in the
 * construction of the VipsBitConstraint object.
 *
 * The VIPS-M semantics used by this Constraint are those of
 * formalization.VIPS-M.simplified.3.alt.pdf. However, the L1 cache of
 * each process will always have an entry for each memory
 * location. Invisible evicts are assumed before each instruction or
 * event that depends on a memory location not being in L1.
 */
class VipsBitConstraint{
public:
  /* A Common object contains information common to all
   * VipsBitConstraints for a particular Machine.
   *
   * The Common object provides information about the Machine, which
   * is necessary to interpret VipsBitConstraints.
   */
  class Common{
  public:
    /* Construct a Common object for constraints of m. */
    Common(const Machine &m);
    /* Compute and return all initial constraints of this->machine. */
    std::set<VipsBitConstraint> get_initial_constraints() const;
    /* The Machine which this Common object corresponds to. */
    const Machine &machine;
  private:
    friend class VipsBitConstraint;
  };
  /* Constructs an initial constraint based on common. All memory
   * locations will be initialized to some initial value, but for
   * memory locations where the initial value is not uniquely
   * determined in common.machine, no guarantees are given as to
   * which initial value will be used.
   */
  VipsBitConstraint(const Common &common);
  VipsBitConstraint(const VipsBitConstraint&);
  ~VipsBitConstraint();

  /* Returns the set of transitions that should be explored from this
   * constraint.
   */
  VecSet<const Machine::PTransition*> partred(const Common &common) const;

  /* Returns the result of applying t to this constraint. */
  VipsBitConstraint post(const Common &common, 
                         const Machine::PTransition &t) const;

  /* A vector v such that for each process pid, v[pid] is the control
   * state of pid in this constraint.
   */
  std::vector<int> get_control_states(const Common &common) const throw();

  static void test();
private:
  friend class Common;
};

#endif
