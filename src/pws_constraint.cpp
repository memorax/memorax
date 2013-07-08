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

#include "pws_constraint.h"


std::list<const Machine::PTransition*> PwsConstraint::partred() const {
  throw new std::logic_error("PwsConstraint::partred: Not implemented");
}

std::list<Constraint*> PwsConstraint::pre(const Machine::PTransition &t) const {
  throw new std::logic_error("PwsConstraint::pre: Not implemented");
}

std::string PwsConstraint::to_string() const throw() {
  return SbConstraint::to_string();
}

Constraint::Comparison PwsConstraint::entailment_compare(const SbConstraint &sbc) const{
  const PwsConstraint &pwsc = dynamic_cast<const PwsConstraint&>(sbc);
  return entailment_compare(pwsc);
}

Constraint::Comparison PwsConstraint::entailment_compare(const PwsConstraint &pwsc) const{
  Constraint::Comparison cmp = SbConstraint::entailment_compare(pwsc);
  if (cmp == Constraint::INCOMPARABLE) return cmp;
  return comb_comp(entailment_compare_buffers(pwsc), cmp);
}

Constraint::Comparison PwsConstraint::entailment_compare_buffers(const PwsConstraint &pwsc) const{
  Comparison cmp = EQUAL;
  for (int p = 0; p < common.machine.proc_count(); p++) {
    for (int ml = 0; ml < common.mem_size; ml++) {
      cmp = comb_comp(entailment_compare_buffer(write_buffers[p][ml], pwsc.write_buffers[p][ml]), cmp);
      if (cmp == Constraint::INCOMPARABLE) return cmp;
    }
  }
  return cmp;
}

Constraint::Comparison PwsConstraint::entailment_compare_buffer(const Store& a, const Store& b) const{
  if (a.size() == b.size()) {
    return a.entailment_compare(b);
  } else if (a.size() > b.size()) {
    return invert_comp(entailment_compare_buffer(b, a));
  }
  // a is either LESS than b, or INCOMPARABLE.
  if (a.size() == 0) return INCOMPARABLE; // a must be empty iff b is
  // a and b must end with the same value
  if (a[a.size() - 1] != b[b.size() - 1] && a[a.size() - 1] != value_t::STAR) return INCOMPARABLE;
  // We check if a is a subword of b
  for (int ai = 0, bi = 0; ai < a.size(); bi++) {
    if (bi == b.size()) return INCOMPARABLE; // a isn't a subword of b
    if (a[ai] == b[bi] || a[ai] == value_t::STAR) //TODO: Ponder the correctness of this
      ai++; // They match
  }
  return LESS; // a is a subword of b
}
