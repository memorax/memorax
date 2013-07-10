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

#ifndef __VIPS_BIT_REACHABILITY__
#define __VIPS_BIT_REACHABILITY__

#include "reachability.h"
#include "vips_bit_constraint.h"

/* VipsBitReachability implements a reachability analysis specifically
 * for VipsBitConstraint. The analysis is forward and explicit state.
 */
class VipsBitReachability : public Reachability{
public:
  virtual ~VipsBitReachability(){};

  virtual Result *reachability(Arg *arg) const;

  static void test();
private:
  /* Each constraint will use a parent_t to keep track of its parent
   * and the transition moving from the parent to itself.
   *
   * Both trans and parent are null if the constraint is an initial
   * constraint.
   */
  struct parent_t{
    parent_t() : trans(0), parent(0) {};
    parent_t(const Machine::PTransition *t, const VipsBitConstraint *p)
      : trans(t), parent(p) {};
    const Machine::PTransition *trans;
    const VipsBitConstraint *parent;
  };

  /* get_comparator(common) returns a function f that uses
   * common.compare to compare VipsBitConstraint pointers.
   *
   * f(a,b) returns true iff common.compare(*a,*b) < 0.
   */
  typedef std::function<bool(const VipsBitConstraint*,const VipsBitConstraint*)> vbcmp_t;
  static vbcmp_t get_comparator(const VipsBitConstraint::Common&);
};

#endif
