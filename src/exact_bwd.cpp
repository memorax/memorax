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

#include "exact_bwd.h"

Reachability::Result *ExactBwd::reachability(Reachability::Arg *arg) const{
  Arg *earg = static_cast<Arg*>(arg);
  Result *result = new Result(arg->machine);
  result->common = earg->common;
  earg->common = 0;
  result->timer.start();

  ConstraintContainer &container = *earg->container;

  /* Check arg->bad_states and setup container */
  if(earg->bad_states.empty()){
    result->result = Reachability::UNREACHABLE;
    result->timer.stop();
    return result;
  }else{
    for(auto it = earg->bad_states.begin(); it != earg->bad_states.end(); it++){
      //Log::extreme << " *** Bad states ***\n" << (*it)->to_string();
      if(earg->machine.proc_count() != int((*it)->get_control_states().size())){
        throw new std::logic_error("ExactBwd::reachability: Incompatible process count in machine and bad states.");
      }
      if((*it)->is_init_state()){
        result->trace = new Trace(*it);
        it++;
        while(it != earg->bad_states.end()){
          delete *it;
          it++;
        }
        result->result = Reachability::REACHABLE;
        result->timer.stop();
        return result;
      }else{
        container.insert_root(*it);
      }
    }
  }

  /* Start analysing */
  bool is_reachable = false;
  while(!is_reachable && container.Q_size()){
    Constraint *c = container.pop();
    std::list<const Machine::PTransition*> ts = c->partred();

    for(auto trans_it = ts.begin(); !is_reachable && trans_it != ts.end(); trans_it++){
      std::list<Constraint*> new_consts = c->pre(**trans_it);
      result->generated_constraints += new_consts.size();

      // Log::extreme << "\n\n Origin conf " << c->to_string() << "\n";
      // Log::extreme << " *** Transition " << (*trans_it)->to_string(arg->machine) << "***\n";
      for(auto c_it = new_consts.begin(); c_it != new_consts.end(); c_it++){
        if(is_reachable){
          /* Found an initial state earlier in this loop: deallocate */
          delete *c_it;
        }else{
          // Log::extreme << " *** Getting state \n" << (*c_it)->to_string() << "\n";
          c->abstract();
          bool is_init = (*c_it)->is_init_state();
          container.insert(c,*trans_it,*c_it);
          if(is_init){
            is_reachable = true;
            result->stored_constraints = container.F_size();
            result->trace = container.clear_and_get_trace(*c_it);
          }
        }
      }
    }
  }

  result->stored_constraints = 
    std::max(result->stored_constraints,container.F_size());
  if(is_reachable){
    result->result = Reachability::REACHABLE;
  }else{
    result->result = Reachability::UNREACHABLE;
  }
  
  container.clear();

  result->timer.stop();
  return result;
};

ExactBwd::Arg::Arg(const Machine &m, PbConstraint::Common *common,ConstraintContainer *cont)
  : Reachability::Arg(m), common(common), container(cont)
{
  for(unsigned i = 0; i < m.forbidden.size(); i++){
    bad_states.push_back(new PbConstraint(m.forbidden[i],*common));
  }
}

