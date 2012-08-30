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


#include <stdexcept>
#include <map>
#include <cassert>
#include "cmsat.h"

#if HAVE_LIBMATHSAT == 1

template<class Var> bool APList<Var>::is_consistent(const std::list<AppliedPredicate> &l){
#if MATHSAT_VERSION == 4
  MSat::msat_env env = MSat::msat_create_env();
  MSat::msat_add_theory(env,MSat::MSAT_IDL);
#elif MATHSAT_VERSION == 5
  MSat::msat_config cfg = MSat::msat_create_config();
  MSat::msat_env env = MSat::msat_create_env(cfg);
#endif
  std::map<Var,MSat::msat_decl> vd_map;

  for(typename std::list<AppliedPredicate>::const_iterator it = l.begin(); it != l.end(); it++){
    MSat::msat_term t = it->get_predicate()->bind(it->get_argv()).to_msat_term(env,vd_map,std::mem_fun_ref(&Var::to_raw_string));
    MSat::msat_assert_formula(env,t);
  }

  MSat::msat_result res = MSat::msat_solve(env);
  MSat::msat_destroy_env(env);
#if MATHSAT_VERSION == 5
  MSat::msat_destroy_config(cfg);
#endif

  switch(res){
  case MSat::MSAT_UNKNOWN:
    throw new MSatFailure("Error in APList::is_consistent");
  case MSat::MSAT_UNSAT: return false;
  case MSat::MSAT_SAT: return true;
  default:
    throw new MSatFailure("Invalid return value in APList::is_consistent");
  }
}

template<class Var> std::list<typename APList<Var>::AppliedPredicate> 
APList<Var>::expand(const std::list<AppliedPredicate> &l, const pred_set &preds){

  /* Collect all variables mentioned in l */
  std::set<Var> vars;
  for(typename std::list<AppliedPredicate>::const_iterator it = l.begin(); it != l.end(); it++){
    std::set<Var> vs = it->get_variables();
    for(typename std::set<Var>::const_iterator vit = vs.begin(); vit != vs.end(); vit++){
      vars.insert(*vit);
    }
  }

  /* Create the set of candidate predicates */
  std::vector<AppliedPredicate> *aps = apply_variables(vars,preds);
  std::list<AppliedPredicate> implied;
  
  /* Create an environment corresponding to l */
  std::map<Var,MSat::msat_decl> vd_map;
#if MATHSAT_VERSION == 4
  MSat::msat_env env = MSat::msat_create_env();
  MSat::msat_add_theory(env,MSat::MSAT_IDL);
#elif MATHSAT_VERSION == 5
  MSat::msat_config cfg = MSat::msat_create_config();
  MSat::msat_env env = MSat::msat_create_env(cfg);
#endif
  for(typename std::list<AppliedPredicate>::const_iterator it = l.begin(); it != l.end(); it++){
    MSat::msat_term t = it->get_predicate()->bind(it->get_argv()).
      to_msat_term(env,vd_map,std::mem_fun_ref(&Var::to_raw_string));
    MSat::msat_assert_formula(env,t);
  }

  /* Add candidates to implied if they are non-trivial and implied  */
  for(unsigned i = 0; i < aps->size(); i++){
    const AppliedPredicate &ap = (*aps)[i];
    if(trivial(ap) == UNKNOWN){
      MSat::msat_term not_ap = 
        Predicate::neg(ap.get_predicate()->bind(ap.get_argv())).
        to_msat_term(env,vd_map,std::mem_fun_ref(&Var::to_raw_string));

      MSat::msat_push_backtrack_point(env);
      MSat::msat_assert_formula(env,not_ap);
      switch(MSat::msat_solve(env)){
      case MSat::MSAT_UNSAT:
        implied.push_back(ap);
        break;
      case MSat::MSAT_SAT: // Do nothing
      case MSat::MSAT_UNKNOWN: // Do nothing
        break;
      }
      MSat::msat_pop_backtrack_point(env);
    }
  }

  /* Cleanup and return */
  MSat::msat_destroy_env(env);
#if MATHSAT_VERSION == 5
  MSat::msat_destroy_config(cfg);
#endif
  delete aps;
  return implied;
}


template<class Var> typename APList<Var>::Predicate APList<Var>::interpolate(const std::list<AppliedPredicate> &l, const Predicate &p){

#if MATHSAT_VERSION == 4
  MSat::msat_env env = MSat::msat_create_env();
  MSat::msat_add_theory(env,MSat::MSAT_IDL);
  MSat::msat_init_interpolation(env);
#elif MATHSAT_VERSION == 5
  MSat::msat_config cfg = MSat::msat_create_config();
  MSat::msat_set_option(cfg,"interpolation","true");
  MSat::msat_env env = MSat::msat_create_env(cfg);
  assert(!MSAT_ERROR_ENV(env));
#endif
  std::map<Var,MSat::msat_decl> vd_map;

  int groups[2];
  groups[0] = MSat::msat_create_itp_group(env);
  groups[1] = MSat::msat_create_itp_group(env);
  if(groups[0] == -1 || groups[1] == -1){
#if MATHSAT_VERSION == 4
    throw new MSatFailure("Failed to create interpolation groups.");
#elif MATHSAT_VERSION == 5
    const char *err = MSat::msat_last_error_message(env);
    std::stringstream ss;
    ss << "Failed to create interpolation groups: " << err;
    throw new MSatFailure(ss.str());
#endif
  }

  Log::debug << "Interpolating:\n"
             << "Group A:\n";
  /* First formula group */
  MSat::msat_set_itp_group(env,groups[0]);
  for(typename std::list<AppliedPredicate>::const_iterator it = l.begin(); it != l.end(); it++){
    Log::debug << "  " << it->to_string(std::mem_fun_ref(&Var::to_raw_string)) << "\n";
    MSat::msat_term t = it->get_predicate()->bind(it->get_argv()).
      to_msat_term(env,vd_map,std::mem_fun_ref(&Var::to_raw_string));
    MSat::msat_assert_formula(env,t);
  }

  /* Second formula group */
  Log::debug << "Group B:\n";
  Log::debug << "  " << p.to_string(std::mem_fun_ref(&Var::to_raw_string)) << "\n\n";

  MSat::msat_set_itp_group(env,groups[1]);
  MSat::msat_assert_formula(env,p.to_msat_term(env,vd_map,std::mem_fun_ref(&Var::to_raw_string)));


  MSat::msat_result res = MSat::msat_solve(env);
  if(res == MSat::MSAT_SAT){
    throw new ItpSatisfiable("APList::interpolate: Failed to interpolate due to satisfiable environment.");
  }
  if(res == MSat::MSAT_UNKNOWN){
    throw new MSatFailure("APList::interpolate: Failed to interpolate due to unknown satisfiability of environment.");
  }
  assert(res == MSat::MSAT_UNSAT);

  /* Interpolate */
  MSat::msat_term itp = MSat::msat_get_interpolant(env,groups,1);
  char *s = MSat::msat_term_repr(itp);
  Log::debug << "APList::interpolate: Found interpolant: " << s << "\n";
#if MATHSAT_VERSION == 4
  free(s);
#elif MATHSAT_VERSION == 5
  MSat::msat_free(s);
#endif

  Predicate p_itp = Predicate::from_msat_term(env,itp,Var::from_string);

  MSat::msat_destroy_env(env);
#if MATHSAT_VERSION == 5
  MSat::msat_destroy_config(cfg);
#endif

  return p_itp;
};


template<class Var> void APList<Var>::apply_variables_to_one(const std::set<Var> &vs,
                                                            const Predicate *p,
                                                            std::vector<AppliedPredicate > *vec){
  if(p->get_argc() == 0){
    vec->push_back(AppliedPredicate(p,std::vector<Var>()));
  }else if(vs.size() == 0){
    /* Do nothing. */
    /* There are no applied predicates that can be added. */
  }else{
    std::vector<typename std::set<Var>::const_iterator> argv_it(p->get_argc(),vs.begin());
    std::vector<Var> argv(p->get_argc(),*vs.begin());
    while(argv_it[0] != vs.end()){
      /* create new */
      vec->push_back(AppliedPredicate(p,argv));

      /* increase iterators */
      for(int i = p->get_argc()-1; i >= 0; i--){
        argv_it[i]++;
        if(i > 0 && argv_it[i] == vs.end()){
          argv_it[i] = vs.begin(); // ... and increase the next iterator
          argv[i] = *vs.begin();
        }else{
          argv[i] = *argv_it[i];
          break;
        }
      }
    }
      
  }
}

template<class Var> std::vector<typename APList<Var>::AppliedPredicate> *
APList<Var>::apply_variables(const std::set<Var> &vs,const pred_set &ps){

  std::vector<AppliedPredicate > *vec = new std::vector<AppliedPredicate>();

  for(unsigned i = 0; i < ps.size(); i++){
    apply_variables_to_one(vs,ps[i],vec);
  }

  return vec;
}


template<class Var> typename APList<Var>::TrivialType APList<Var>::trivial(const Predicate &p){

  static typename std::map<Predicate,TrivialType> trivial_cache;

  typename std::map<Predicate,TrivialType>::iterator it = trivial_cache.find(p);

  if(it == trivial_cache.end()){

#if MATHSAT_VERSION == 4
    MSat::msat_env env = MSat::msat_create_env();
    MSat::msat_add_theory(env,MSat::MSAT_IDL);
#elif MATHSAT_VERSION == 5
    MSat::msat_config cfg = MSat::msat_create_config();
    MSat::msat_env env = MSat::msat_create_env(cfg);
#endif
    std::map<Var,MSat::msat_decl> vd_map;


    MSat::msat_push_backtrack_point(env);
    MSat::msat_assert_formula(env,p.to_msat_term(env,vd_map,std::mem_fun_ref(&Var::to_raw_string)));

    MSat::msat_result res = MSat::msat_solve(env);

    if(res == MSat::MSAT_UNSAT){
      MSat::msat_destroy_env(env);
#if MATHSAT_VERSION == 5
      MSat::msat_destroy_config(cfg);
#endif
      trivial_cache.insert(std::pair<Predicate,TrivialType>(p,CONTRADICTION));
      return CONTRADICTION;
    }else{
      MSat::msat_pop_backtrack_point(env);
      MSat::msat_assert_formula(env,Predicate::neg(p).to_msat_term(env,vd_map,std::mem_fun_ref(&Var::to_raw_string)));
      res = MSat::msat_solve(env);
      MSat::msat_destroy_env(env);
#if MATHSAT_VERSION == 5
      MSat::msat_destroy_config(cfg);
#endif
      if(res == MSat::MSAT_UNSAT){
        trivial_cache.insert(std::pair<Predicate,TrivialType>(p,TAUTOLOGY));
        return TAUTOLOGY;
      }else{
        trivial_cache.insert(std::pair<Predicate,TrivialType>(p,UNKNOWN));
        return UNKNOWN;
      }
    }
  }else{
    return it->second;
  }
};

#else // HAVE_LIBMATHSAT != 1

template<class Var> bool APList<Var>::is_consistent(const std::list<AppliedPredicate> &l){
  throw new MSatFailure("Program is not compiled with MathSAT.");
}

template<class Var> std::list<typename APList<Var>::AppliedPredicate> 
APList<Var>::expand(const std::list<AppliedPredicate> &l, const pred_set &preds){
  throw new MSatFailure("Program is not compiled with MathSAT.");
}


template<class Var> typename APList<Var>::Predicate APList<Var>::interpolate(const std::list<AppliedPredicate> &l, const Predicate &p){
  throw new MSatFailure("Program is not compiled with MathSAT.");
};


template<class Var> void APList<Var>::apply_variables_to_one(const std::set<Var> &vs,
                                                            const Predicate *p,
                                                            std::vector<AppliedPredicate > *vec){
  throw new MSatFailure("Program is not compiled with MathSAT.");
}

template<class Var> std::vector<typename APList<Var>::AppliedPredicate> *
APList<Var>::apply_variables(const std::set<Var> &vs,const pred_set &ps){
  throw new MSatFailure("Program is not compiled with MathSAT.");
}


template<class Var> typename APList<Var>::TrivialType APList<Var>::trivial(const Predicate &p){
  throw new MSatFailure("Program is not compiled with MathSAT.");
};

#endif // HAVE_LIBMATHSAT == 1
