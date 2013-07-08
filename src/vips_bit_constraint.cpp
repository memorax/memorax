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

#include "log.h"
#include "vips_bit_constraint.h"
#include "test.h"

#include <stdexcept>

VipsBitConstraint::Common::Common(const Machine &m) : machine(m) {
};

VipsBitConstraint::VipsBitConstraint(const Common &common){
};

VipsBitConstraint::VipsBitConstraint(const VipsBitConstraint &vbc){
};

VipsBitConstraint::~VipsBitConstraint(){
};

VipsBitConstraint VipsBitConstraint::post(const Common &common, 
                                          const Machine::PTransition &t) const{
  throw new std::logic_error("VipsBitConstraint::post: Not implemented");
};

std::vector<int> VipsBitConstraint::get_control_states(const Common &common) const throw(){
  throw new std::logic_error("VipsBitConstraint::get_control_states: Not implemented");
};

VecSet<const Machine::PTransition*> VipsBitConstraint::partred(const Common &common) const{
  throw new std::logic_error("VipsBitConstraint::partred: Not implemented");
};

void VipsBitConstraint::test(){
  Test::inner_test("Testing?",true);
};
