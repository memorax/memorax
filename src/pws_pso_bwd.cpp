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

#include "pws_pso_bwd.h"

Trace *PwsPsoBwd::convert_trace(Trace *trace, SbConstraint::Common *common) const {
  std::unique_ptr<Trace> temp(SbTsoBwd::convert_trace(trace, common));
  // Filter out serialise transitions
  Trace *result = new Trace(0);
  for (int i = 1; i <= temp->size(); ++i) {
    if (temp->transition(i)->instruction.get_type() != Lang::SERIALISE)
      result->push_back(*temp->transition(i), 0);
  }
  return result;
}

bool PwsPsoBwd::produces_message(const Lang::Stmt<int> &s) const{
  assert(s.get_writes().size() == 0    ||
         s.get_type() == Lang::UPDATE  ||
         s.get_type() == Lang::WRITE   ||
         s.get_type() == Lang::LOCKED  ||
         s.get_type() == Lang::SLOCKED ||
         s.get_type() == Lang::SERIALISE);
  return (s.get_writes().size() > 0 &&
          s.get_type() != Lang::UPDATE &&
          s.get_type() != Lang::WRITE);
}
