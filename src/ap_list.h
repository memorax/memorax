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

#ifndef __AP_LIST_H__
#define __AP_LIST_H__

#include "predicates.h"
#include <vector>
#include <list>

/* TODO: Test performance impact of using Z3 for is_consistent and expand. */

template<class Var> class APList{
public:
  typedef Predicates::Predicate<Var> Predicate;
  typedef Predicates::AppliedPredicate<Var> AppliedPredicate;
  typedef std::vector<Predicate*> pred_set;

  /* Thrown when MathSAT fails to answer a query. */
  class MSatFailure : public std::exception{
  public:
    MSatFailure(std::string s) : msg(s) {};
    virtual ~MSatFailure() throw() {};
    const char *what() const throw(){
      return ("MSatFailure: "+msg).c_str();
    };
  private:
    std::string msg;
  };

  /* Returns true iff the set of applied predicates in l defines a
   * consistent theory according to MathSAT.
   */
  static bool is_consistent(const std::list<AppliedPredicate> &l);
  /* Wrapper
   *
   * Pre: p is a nullary predicate
   */
  static bool is_consistent(const Predicate &p){
    return is_consistent(std::list<AppliedPredicate>(1,AppliedPredicate(&p,std::vector<Var>())));
  };

  /* Returns a list of all applied predicates ap which are implied by
   * l, and which can be produced by applying a predicate from preds
   * to some arguments, and which is not trivial according to
   * AppliedPredicate::trivial.
   *
   * Pre: l is sorted.
   */
  static std::list<AppliedPredicate> expand(const std::list<AppliedPredicate> &l, const pred_set &preds);
  /* Wrapper
   * 
   * Pre: p is a nullary predicate
   */
  static std::list<AppliedPredicate> expand(const Predicate &p, const pred_set &preds){
    return expand(std::list<AppliedPredicate>(1,AppliedPredicate(&p,std::vector<Var>())),preds);
  };

  class ItpSatisfiable : public std::exception{
  public:
    ItpSatisfiable(std::string m) : msg("APList::ItpSatisfiable: "+m) {};
    virtual ~ItpSatisfiable() throw() {};
    const char *what() const throw() { return msg.c_str(); };
  private:
    std::string msg;
  };
  /* Returns an interpolant of l and p. 
   *
   * Pre: l is not consistent with p. Otherwise ItpSatisfiable is thrown.
   *      p is a nullary predicate
   */
  static Predicate interpolate(const std::list<AppliedPredicate> &l, const Predicate &p);
  /* Wrapper */
  static Predicate interpolate(const AppliedPredicate &ap, const Predicate &p){
    return interpolate(std::list<AppliedPredicate>(1,ap),p);
  };

  enum TrivialType { TAUTOLOGY, CONTRADICTION, UNKNOWN };
  /* Uses SMT solver to see whether this applied
   * predicate is a tautology or a contradiction. If it can
   * guarantee that the applied predicate is a tautology or
   * contradiction, respectively TAUTOLOGY or CONTRADICTION is
   * returned. Otherwise UNKNOWN is returned.
   */
  static TrivialType trivial(const AppliedPredicate &ap){
    return trivial(ap.get_predicate()->bind(ap.get_argv()));
  };
  /* Pre: p is a nullary predicate */
  static TrivialType trivial(const Predicate &p);

private:
  /* A vector (in heap) containing precisely the applied predicates
   * that can be constructed by applying a predicate in ps to
   * variables from vs. */
  static std::vector<AppliedPredicate> *apply_variables(const std::set<Var> &vs,
                                                                     const pred_set &ps);
  /* Adds into vec all applied predicates that can be constructed by
   * applying the predicate p to variables from vs. */
  static void apply_variables_to_one(const std::set<Var> &vs,
                                     const Predicate  *p,
                                     std::vector<AppliedPredicate> *vec);
};

#include "ap_list.tcc"

#endif
