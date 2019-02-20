/*
 * Copyright (C) 2018 Tuan Phong Ngo
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

#include "parser.h"
#include "preprocessor.h"
#include "dual_tso_bwd.h"
#include "test.h"

#include <cctype>
#include <functional>
#include <sstream>

Trace *DualTsoBwd::convert_trace(Trace *trace, DualChannelConstraint::Common *common) const{
  Trace *tso_trace = DualChannelBwd::convert_trace(trace, common);

  Log::extreme << " *** DUAL trace ***\n";
  trace->print(Log::extreme,Log::extreme,Log::json,common->machine);
  Log::extreme << "\n\n";
  //Log::extreme << " *** TSO trace ***\n";
  //tso_trace->print(Log::extreme,Log::extreme,Log::json,common->machine);
  //Log::extreme << "\n";

  return tso_trace;
};

bool DualTsoBwd::produces_message(const Lang::Stmt<int> &s) const{
  return (s.get_writes().size() > 0 &&
          s.get_type() != Lang::UPDATE);
};

bool DualTsoBwd::consumes_message(const Lang::Stmt<int> &s) const{
  return (s.get_type() == Lang::UPDATE ||
          (s.get_type() == Lang::LOCKED && s.get_writes().size() > 0));
};

void DualTsoBwd::test(){
  std::cout << "DualTsoBwd:test is not implemented\n";
};
