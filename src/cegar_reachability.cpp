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

#include "cegar_reachability.h"

std::string CegarReachability::Result::to_string() const{
  std::stringstream ss;
  ss << Reachability::Result::to_string();
  ss << "  Number of CEGAR loops: " << loop_count << "\n";
  return ss.str();
};

CegarReachability::Result *CegarReachability::reachability(Reachability::Arg *arg) const{
  CegarReachability::Arg *cegarg = static_cast<CegarReachability::Arg*>(arg);
  Result *result = new Result(arg->machine);
  result->timer.start();

  Reachability::Result *sub_result;
  Reachability::Arg *next_arg = cegarg->first_refinement;

  bool done = false;
  while(!done && (cegarg->max_loop_count < 0 || result->loop_count < cegarg->max_loop_count)){
    Log::msg << "Refinement " << result->loop_count << ":\n" 
             << refinement_to_string(next_arg) << "\n" << std::flush;
    result->loop_count++;
    sub_result = cegarg->abstract_reach->reachability(next_arg);
    Log::msg << sub_result->to_string() << "\n";
    Reachability::Arg *tmp_arg;
    switch(refine(sub_result,next_arg,cegarg,&tmp_arg)){
    case CORRECT:
      done = true;
      result->result = sub_result->result;
      result->trace = sub_result->trace;
      result->last_result = sub_result;
      result->generated_constraints += sub_result->generated_constraints;
      result->stored_constraints += sub_result->stored_constraints;
      sub_result->trace = 0;
      sub_result = 0; // Prevent deallocation of sub_result
      break;
    case REFINED:
      result->generated_constraints += sub_result->generated_constraints;
      result->stored_constraints += sub_result->stored_constraints;
      break;
    case FAILURE:
      done = true;
      result->result = Reachability::FAILURE;
      break;
    }
    delete next_arg;
    if(sub_result) delete sub_result;
    next_arg = tmp_arg;
  }
  if(!done && cegarg->max_loop_count >= 0 && result->loop_count >= cegarg->max_loop_count){
    Log::result << "CEGAR loop exceeded loop count limit.\n";
    result->result = Reachability::FAILURE;
  }

  result->timer.stop();
  return result;
};
