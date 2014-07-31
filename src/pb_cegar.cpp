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

#include "pb_cegar.h"
#include <queue>
#include "ap_list.h"
#include "exact_bwd.h"

CegarReachability::refinement_result_t 
PbCegar::refine(Reachability::Result *result, 
                Reachability::Arg    *prev_arg,
                CegarReachability::Arg *cegarg,
                Reachability::Arg **next_arg) const{
  assert(dynamic_cast<PbCegar::Arg*>(cegarg));
  PbCegar::Arg *pbcegarg = static_cast<PbCegar::Arg*>(cegarg);
  switch(result->result){
  case Reachability::REACHABLE:
    {
      PbConstraint::Common *common = cegar(*result->trace);
      if(common){
        *next_arg = pbcegarg->init_arg(prev_arg,common);
        return CegarReachability::REFINED;
      }else{
        return CegarReachability::CORRECT;
      }
    }
  case Reachability::UNREACHABLE:
    return CegarReachability::CORRECT;
  case Reachability::FAILURE:
    return CegarReachability::FAILURE;
  default:
    throw new std::logic_error("PbCegar::refine: Invalid result.");
  }
};

PbConstraint::Common *PbCegar::cegar(const Trace &trace){  
  const PbConstraint::Common &c = static_cast<const PbConstraint*>(trace.constraint(0))->common;

  /* Is the trace dirty? */
  bool is_dirty = false;
  for(int i = 0; i <= trace.size(); i++){
    if(static_cast<const PbConstraint*>(trace.constraint(i))->dirty_bit){
      is_dirty = true;
      break;
    }
  }

  if(is_dirty){
    Log::debug << " *** (Dirty) Trace: ***\n\n";
    Log::debug << trace.to_string(c.machine);
    PbConstraint::pred_set new_preds;
    for(unsigned i = 0; i < c.predicates.size(); i++){
      new_preds.push_back(new Predicate(*c.predicates[i]));
    }
    PbConstraint::Common *common = new PbConstraint::Common(c.k+1,c.machine,new_preds,c.auto_abstract);
    return common;
  }else{
    Log::debug << " *** Interpolating. ***\n\n";

    Log::debug << " *** Trace: ***\n\n";
    Log::debug << trace.to_string(c.machine);
    Log::debug << " *** Conflict trace: ***\n\n";
    PbConstraint::Common *ctrace_common;
    Trace *ctrace = conflict_trace(trace,&c,&ctrace_common);
    if(ctrace){
      Log::debug << ctrace->to_string(c.machine);
      Predicate orig_interpolant = interpolate(trace,*ctrace);
      Predicate interpolant = orig_interpolant.generalise().simplify();
      Log::debug << "Interpolant: " << interpolant.to_string([&c](int r,int p)
                                                             { return c.machine.pretty_string_reg.at(std::pair<int,int>(r,p)); },
                                                             [&c](Lang::NML nml)
                                                             { return c.machine.pretty_string_nml.at(nml); }) << "\n";
      PbConstraint::pred_set new_preds_2;
      PbConstraint::pred_set new_preds_copy;
      for(unsigned i = 0; i < c.predicates.size(); i++){
        new_preds_2.push_back(new Predicate(*c.predicates[i]));
        new_preds_copy.push_back(new Predicate(*c.predicates[i]));
      }
      new_preds_2.push_back(new Predicate(interpolant));
      new_preds_copy.push_back(new Predicate(interpolant));
    
      PbConstraint::Common *new_common = new PbConstraint::Common(c.k,c.machine,new_preds_copy,c.auto_abstract);
      if(can_simulate(*new_common,trace)){

        std::list<Predicate> interpolant_helpers = get_interpolant_helpers(trace,*ctrace,orig_interpolant);
        Log::debug << "It is necessary to add interpolant helpers.\n";
        Log::debug << "Interpolant helpers: [\n";
        for(std::list<Predicate>::iterator it = interpolant_helpers.begin(); it != interpolant_helpers.end(); it++){
          Log::debug << "  " << it->to_string([&c](int r, int p){ return c.machine.pretty_string_reg.at(std::pair<int,int>(r,p)); },
                                              [&c](Lang::NML nml){ return c.machine.pretty_string_nml.at(nml); }) << "\n";
        }
        Log::debug << "]\n";

        for(std::list<Predicate>::iterator it = interpolant_helpers.begin(); it != interpolant_helpers.end(); it++){
          bool exists = false;
          for(unsigned i = 0; i < new_preds_2.size(); i++){
            if(*new_preds_2[i] == *it){
              exists = true;
              break;
            }
          }
          if(!exists)
            new_preds_2.push_back(new Predicate(*it));
        }
      }
      delete new_common;
      new_common = new PbConstraint::Common(c.k,c.machine,new_preds_2,c.auto_abstract);
      delete ctrace;
      delete ctrace_common;
      return new_common;
    }else{
      assert(ctrace_common == 0);
      Log::debug << "No conflict trace found.\n";
      return 0;
    }
  }
}

std::list<PbCegar::Predicate> PbCegar::least_implying(const std::list<AppliedPredicate> &ap_list1,
                                                        const std::list<AppliedPredicate> &ap_list2,
                                                        Predicate p){
  std::list<AppliedPredicate> ap_list_1p;
  std::copy(ap_list1.begin(),ap_list1.end(),std::back_inserter(ap_list_1p));
  Predicate np = !p;
  ap_list_1p.push_back(AppliedPredicate(&np,std::vector<TsoVar>()));
  return least_inconsistent(ap_list_1p,ap_list2);
}

std::list<PbCegar::Predicate> PbCegar::least_inconsistent(const std::list<AppliedPredicate> &ap_list1,
                                                            const std::list<AppliedPredicate> &ap_list2){
  /* Perform a bredth first search for a subset of ap_list2 which is
   * inconsistent with ap_list1 */
  typedef std::pair<std::list<Predicate>, std::list<Predicate> > task_t;
  std::queue<task_t> task_queue;
  task_t t0;
  for(std::list<AppliedPredicate>::const_iterator it = ap_list2.begin(); it != ap_list2.end(); it++){
    std::list<Predicate> l = it->get_predicate()->bind(it->get_argv()).conjuncts();
    for(std::list<Predicate>::iterator it = l.begin(); it != l.end(); it++){
      t0.second.push_back(*it);
    }
  }
  task_queue.push(t0);
  bool found_inconsistent = false;
  std::list<Predicate> the_subset; // Collect the result here
  while(!found_inconsistent && !task_queue.empty()){
    task_t t = task_queue.front();
    task_queue.pop();
    while(!t.second.empty()){
      task_t t_next;
      t_next.first = t.first;
      t_next.first.push_front(t.second.front());
      t.second.pop_front();
      t_next.second = t.second;

      std::list<AppliedPredicate> l(ap_list1.begin(),ap_list1.end());
      
      for(std::list<Predicate>::const_iterator it = t_next.first.begin();
          it != t_next.first.end(); it++){
        l.push_back(AppliedPredicate(&*it,std::vector<TsoVar>()));
      }
      if(!APList<TsoVar>::is_consistent(l)){
        found_inconsistent = true;
        the_subset = t_next.first;
      }else{
        task_queue.push(t_next);
      }
    }
  }
  if(!found_inconsistent){
    throw new std::logic_error("PbConstraint::least_inconsistent: Union of predicate sets is not inconsistent.");
  }
  return the_subset;
}


Trace *PbCegar::conflict_trace(const Trace &trace, const PbConstraint::Common *trace_common, PbConstraint::Common **res_common){

  /* Produce a Common for exact tracing */
  PbConstraint::pred_set new_preds;
  for(unsigned i = 0; i < trace_common->predicates.size(); i++){
    new_preds.push_back(new Predicate(*trace_common->predicates[i]));
  }
  *res_common = new PbConstraint::Common(trace_common->k,trace_common->machine,new_preds,false);


  /* Start following trace backwards without abstraction */
  PbConstraint *exact = new PbConstraint(trace.constraint(trace.size())->get_control_states(),**res_common);
  Trace *res = new Trace(exact);

  bool found_conflict = false;
  for(int pos = trace.size(); !found_conflict && pos > 0; pos--){
    Machine::PTransition trans = *trace[pos];
    std::list<Constraint*> ps = exact->pre(trans);
    /* Find the constraint in ps which intersects with
     * it->get_before().  Note that there should be exactly 1 or
     * exactly 0 such constraints in ps. */
    PbConstraint *exact_pre = 0;
    for(std::list<Constraint*>::iterator psit = ps.begin(); psit != ps.end(); psit++){
      if((*psit)->intersects(*trace.constraint(pos-1))){
        assert(exact_pre == 0);
        exact_pre = static_cast<PbConstraint*>(*psit);
      }else{
        delete *psit;
      }
    }
    if(exact_pre){
      res->push_front(exact_pre,trans);
      exact = exact_pre;
    }else{
      Log::debug << " * PbCegar::conflict_trace: Found conflict at this transition\n";
      Log::debug << *trace.constraint(pos-1) 
                 << "####" << trace[pos]->to_string(trace_common->machine) << "\n" 
                 << *trace.constraint(pos) << "\n\n";
      found_conflict = true;
    }
  }
  if(!found_conflict){
    if(exact->is_init_state()){
      delete res;
      res = 0;
      delete *res_common;
      *res_common = 0;
    }
  }
  return res;
}

PbConstraint::Predicate PbCegar::interpolate(const Trace &trace, 
                                              const Trace &ctrace){
  if(trace.size() == ctrace.size()){
    /* Must interpolate with the initial states */
    const PbConstraint *c = static_cast<const PbConstraint*>(ctrace.constraint(0));
    assert(!c->is_init_state());
    Predicate is_init = c->common.is_init.get_predicate()->bind(c->common.is_init.get_argv());
    return APList<TsoVar>::interpolate(c->ap_list,is_init);
  }else{
    assert(trace.size() > ctrace.size());
    /* Interpolate ctrace.front with the corresponding constraint in trace */
    const PbConstraint *c = static_cast<const PbConstraint*>(ctrace.constraint(0));
    /* Find the corresponding TraceElement in trace */
    int trace_pos = trace.size() - ctrace.size();
    /* Now it->get_after() should correspond to c */
    assert(static_cast<const PbConstraint*>(trace.constraint(trace_pos))->pcs == c->pcs);
    assert(static_cast<const PbConstraint*>(trace.constraint(trace_pos))->channels == c->channels);

    /* Start interpolation */
    Machine::PTransition trans = *trace[trace_pos];
    std::list<Constraint*> rs = trace.constraint(trace_pos)->range(trans);
    assert(rs.size() == 1);
    PbConstraint *exact_range = static_cast<PbConstraint*>(rs.front());
    Predicate pred = Predicate::tt();
    for(std::list<AppliedPredicate>::const_iterator it = exact_range->ap_list.begin();
        it != exact_range->ap_list.end(); it++){
      pred = pred && it->get_predicate()->bind(it->get_argv());
    }

    Predicate interpolant = APList<TsoVar>::interpolate(c->ap_list,pred);
    delete exact_range;
    return interpolant;
  }
}

bool PbCegar::can_simulate(PbConstraint::Common &common,
                            const Trace &trace){
  PbConstraint *pbc = new PbConstraint(static_cast<const PbConstraint*>(trace.constraint(trace.size()))->pcs,common);
  bool can_simulate = true;
  for(int pos = trace.size(); can_simulate && pos > 0; pos--){
    Machine::PTransition trans = *trace[pos];
    std::list<Constraint*> prel = pbc->pre(trans);
    PbConstraint *pbc2 = 0;
    for(std::list<Constraint*>::iterator prelit = prel.begin(); prelit != prel.end(); prelit++){
      if(trace.constraint(pos-1)->intersects(**prelit)){
        assert(pbc2 == 0);
        pbc2 = static_cast<PbConstraint*>(*prelit);
      }else{
        delete *prelit;
      }
    }
    if(pbc2 == 0){
      can_simulate = false;
    }else{
      pbc2->abstract();
      delete pbc;
      pbc = pbc2;
    }
  }
  if(can_simulate){
    /* We have managed to simulate through all of the trace. */
    /* Have we arrived at a valid init state? */
    if(!pbc->is_init_state()){
      can_simulate = false;
    }
  }
  delete pbc;
  return can_simulate;
}

std::list<PbCegar::Predicate> PbCegar::get_interpolant_helpers(const Trace &trace,
                                                                 const Trace &ctrace,
                                                                 Predicate interpolant){
  std::list<Predicate> working_set;
  std::list<Predicate> complete_set;

  int trace_pos = trace.size() - ctrace.size();

  assert(trace.constraint(trace_pos));
  assert(ctrace.constraint(0));
  assert(*trace[trace_pos+1] == *ctrace[1]);

  working_set = least_implying(static_cast<const PbConstraint*>(trace.constraint(trace_pos))->ap_list,
                               static_cast<const PbConstraint*>(ctrace.constraint(0))->ap_list,
                               interpolant);

  for(int ctrace_pos = 1; !working_set.empty() && ctrace_pos <= ctrace.size(); ctrace_pos++){
    const PbConstraint *cpbc = static_cast<const PbConstraint*>(ctrace.constraint(ctrace_pos));
    std::set<Predicate> added; // All conjuncts added to ap_list
    std::map<Predicate,Predicate> conj_src;
    {
      Machine::PTransition trans = *ctrace[ctrace_pos];
      std::list<Predicate> after_conjs; // All conjuncts occurring in cit->get_after()->ap_list
      /* Build after_conjs */
      const std::list<AppliedPredicate> &apl = cpbc->ap_list;
      for(std::list<AppliedPredicate>::const_iterator it = apl.begin(); it != apl.end(); it++){
        std::list<Predicate> conjs = it->get_predicate()->bind(it->get_argv()).conjuncts();
        std::copy(conjs.begin(),conjs.end(),std::back_inserter(after_conjs));
      }
      
      /* Build added */
      const PbConstraint::Common &ec = static_cast<const PbConstraint*>(trace.constraint(0))->common;
      PbConstraint::Common exact_common(ec.k,ec.machine,PbConstraint::pred_set(),false);
      PbConstraint tmp_pbc(cpbc->dirty_bit,cpbc->channels,cpbc->pcs,cpbc->cycle_locks,exact_common);
      std::list<Constraint*> empty_pres = tmp_pbc.pre(trans);
      assert(!empty_pres.empty());
      PbConstraint *empty_pre = static_cast<PbConstraint*>(empty_pres.front());
      for(std::list<AppliedPredicate>::const_iterator it = empty_pre->ap_list.begin(); it != empty_pre->ap_list.end(); it++){
        std::list<Predicate> conjs = it->get_predicate()->bind(it->get_argv()).conjuncts();
        for(std::list<Predicate>::iterator it = conjs.begin(); it != conjs.end(); it++){
          added.insert(*it);
        }
      }
      for(std::list<Constraint*>::iterator it = empty_pres.begin(); it != empty_pres.end(); it++) delete *it;

      /* Build conj_src */
      for(std::list<Predicate>::iterator conj_it = after_conjs.begin(); conj_it != after_conjs.end(); conj_it++){
        tmp_pbc.ap_list.push_back(AppliedPredicate(&*conj_it,std::vector<TsoVar>()));
        std::list<Constraint*> pres = tmp_pbc.pre(trans);
        assert(!empty_pres.empty());
        PbConstraint *p = static_cast<PbConstraint*>(pres.front());
        for(std::list<AppliedPredicate>::iterator it = p->ap_list.begin(); it != p->ap_list.end(); it++){
          std::list<Predicate> conjs = it->get_predicate()->bind(it->get_argv()).conjuncts();
          for(std::list<Predicate>::iterator it = conjs.begin(); it != conjs.end(); it++){
            if(added.count(*it) == 0){
              conj_src.insert(std::pair<Predicate,Predicate>(*it,*conj_it));
            }
          }
        }
        for(std::list<Constraint*>::iterator it = pres.begin(); it != pres.end(); it++) delete *it;
        tmp_pbc.ap_list.clear();
      }
    }

    std::list<Predicate> next_working_set;
    for(std::list<Predicate>::iterator wit = working_set.begin(); wit != working_set.end(); wit++){
      if(std::count(added.begin(),added.end(),*wit)){ // *wit was added in by this transition
        complete_set.push_back(*wit);
      }else{
        Predicate src = conj_src.at(*wit);
        next_working_set.push_back(src);
        if(src != *wit){
          complete_set.push_back(*wit);
        }
      }
    }
    working_set = next_working_set;
  }

  /* Generalise complete_set, and remove duplicates */
  std::list<Predicate> helpers;
  for(std::list<Predicate>::iterator it = complete_set.begin(); it != complete_set.end(); it++){
    Predicate g = it->generalise().simplify();
    if(!std::count(helpers.begin(),helpers.end(),g)){
      helpers.push_back(g);
    }
  }

  return helpers;
}


std::string PbCegar::refinement_to_string(const Reachability::Arg *refinement) const{
  if(dynamic_cast<const ExactBwd::Arg*>(refinement)){
    return static_cast<const ExactBwd::Arg*>(refinement)->common->to_string();
  }else{
    return "  (Refinement)\n";
  }
};
