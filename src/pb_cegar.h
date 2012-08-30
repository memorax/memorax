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

#ifndef __PB_CEGAR_2_H__
#define __PB_CEGAR_2_H__

#include "reachability.h"
#include "cegar_reachability.h"
#include "pb_constraint.h"
#include "tso_var.h"
#include "trace.h"

/* PbCegar implements reachability analysis using PbConstraints while
 * gradually refining the set of predicates used for abstraction.
 */
class PbCegar : public CegarReachability{
public:
  virtual CegarReachability::refinement_result_t 
  refine(Reachability::Result *result, 
         Reachability::Arg    *prev_arg,
         CegarReachability::Arg *cegarg,
         Reachability::Arg **next_arg) const;

  class Arg : public CegarReachability::Arg{
  public:
    typedef std::function<Reachability::Arg*(Reachability::Arg*,PbConstraint::Common*)> init_arg_t;
    /* Takes ownership of abstract_reach and first
     *
     * The function init_arg should compose the previous argument to
     * the underlying reachability algorithm and a refined Common
     * object into the next argument to the underlying reachability
     * algorithm.
     */
    Arg(const Machine &m,Reachability *abstract_reach, Reachability::Arg *first,int max_loop_count,
        init_arg_t init_arg)
      : CegarReachability::Arg(m,abstract_reach,first,max_loop_count), init_arg(init_arg) {};
    virtual ~Arg() {};
    init_arg_t init_arg;
  };

  virtual std::string refinement_to_string(const Reachability::Arg *refinement) const;
private:
  typedef Predicates::Term<TsoVar> Term;
  typedef Predicates::Predicate<TsoVar> Predicate;
  typedef Predicates::AppliedPredicate<TsoVar> AppliedPredicate;


  /* Analyzes the trace trace. If the trace is a valid trace under
   * TSO, null is returned. Otherwise a Common object is returned,
   * which refines the abstraction used in trace.
   */
  static PbConstraint::Common *cegar(const Trace &trace);

  /* If trace is consistent with TSO, null is returned and *res_common
   * is assigned null. Otherwise, a non-abstracted conflict trace for
   * trace is returned, and *res_common is set to point to the Common
   * object used in the returned trace. (The returned trace and
   * **res_common are allocated on heap and ownership is given to the
   * caller.)
   *
   * trace_common should be the Common object used by constraints in
   * trace.
   */
  static Trace *conflict_trace(const Trace &trace, const PbConstraint::Common *trace_common, PbConstraint::Common **res_common);

  /* Returns a predicate corresponding to Delta for the conflict trace
   * ctrace of trace.
   *
   * Pre: ctrace is a conflict trace of trace.
   */
  static Predicate interpolate(const Trace &trace,
                               const Trace &ctrace);

  /* Searches for and returns, a smallest subset, ss, of the applied
   * predicates in ap_list2, such that the union of ss and ap_list1 is
   * inconsistent.
   *
   * Pre: The union of ap_list1 and ap_list2 is inconsistent.
   */
  static std::list<Predicate> least_inconsistent(const std::list<AppliedPredicate> &ap_list1,
                                                 const std::list<AppliedPredicate> &ap_list2);

  /* Searches for and returns, a smallest subset, ss, of the applied
   * predicates in ap_list2, such that the union of ss and ap_list1
   * implies p. Note that the union of ss and ap_list1 may be inconsistent.
   *
   * Pre: The union of ap_list1 and ap_list2 implies p.
   */
  static std::list<Predicate> least_implying(const std::list<AppliedPredicate> &ap_list1,
                                             const std::list<AppliedPredicate> &ap_list2,
                                             Predicate p);  

  /* Returns true if it is possible to simulate the trace trace under
   * the abstraction specified by common. Returns false otherwise.
   */
  static bool can_simulate(PbConstraint::Common &common,
                           const Trace &trace);

  static std::list<Predicate> get_interpolant_helpers(const Trace &trace,
                                                      const Trace &ctrace,
                                                      Predicate interpolant);

};

#endif
