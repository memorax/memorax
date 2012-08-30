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

#ifndef __PREDICATES_H__
#define __PREDICATES_H__

#include "lang.h"
#include "cmsat.h"
#include "log.h"
#include "syntax_string.h"
#include <map>
#include <list>

namespace Predicates {

  class MSatTermError : public std::exception{
  public:
    MSatTermError(std::string m) : msg("Predicates::MSatTermError: "+m) {};
    virtual ~MSatTermError() throw() {};
    const char *what() const throw() { return msg.c_str(); };
  private:
    std::string msg;
  };


  /* Creates a function which replaces one variable for another, and
   * leaves all other variables untouched.
   */
  template<class Var> std::function<Var(const Var&)> subst_translator(Var replacement, Var old){
    return [&replacement,&old](const Var &v)->Var{
      if(v == old){
        return replacement;
      }else{
        return v;
      }
    };
  };

  /* Dummy class which can be used to instantiate terms, predicates,
   * applied predicates.
   */
  class DummyVar{
  public:
    static DummyVar reg(int r, int p) {
      DummyVar dv;
      dv.rg = r;
      dv.pid = p;
      return dv;
    };
    bool operator>(const DummyVar &dv) const throw(){
      return rg > dv.rg || (rg == dv.rg && pid > dv.pid);
    };
    bool operator==(const DummyVar &dv) const throw(){
      return rg == dv.rg && pid == dv.pid;
    };
    bool operator<(const DummyVar &dv) const throw(){
      return rg < dv.rg || (rg == dv.rg && pid < dv.pid);
    };
    std::string to_string() const throw(){
      return "(DummyVar)";
    };
  private:
    int rg;
    int pid;
  };

  template<class Var> class Term : private SyntaxString<Var>{
  public:
    /* constructors */
    static Term from_expr(const Lang::Expr<int> &e,int pid);
    static Term integer(int i) { return Term(SyntaxString<Var>::integer(i)); };
    static Term variable(const Var &v) { return Term(SyntaxString<Var>::variable(v)); };
    static Term argument(int a) { return Term(SyntaxString<Var>::argument(a)); }; // Projection function for argument a
    static Term plus(Term a, Term b) { return Term(SyntaxString<Var>::plus(a,b)); };
    static Term minus(Term a, Term b) { return Term(SyntaxString<Var>::minus(a,b)); };
    static Term minus(Term t) { return Term(SyntaxString<Var>::minus(t)); }; // Unary minus
    std::string to_string(const std::function<std::string(const Var&)> &vts) const throw(){
      return SyntaxString<Var>::to_string(vts);
    };
    std::string to_string(const std::function<std::string(int,int)> &regts,
                          const std::function<std::string(Lang::NML)> &nmlts) const throw(){
      return SyntaxString<Var>::to_string([&regts,&nmlts](const Var &v){ return v.to_string(regts,nmlts); });
    };
  private:
    Term(const SyntaxString<Var> &ss) : SyntaxString<Var>(ss) {};
    template<class Var2> friend class Predicate;
    template<class Var2> friend class AppliedPredicate;
  };

  template<class Var> inline Term<Var> operator+(const Term<Var> &a, const Term<Var> &b){ return Term<Var>::plus(a,b); }
  template<class Var> inline Term<Var> operator-(const Term<Var> &a, const Term<Var> &b){ return Term<Var>::minus(a,b); }
  template<class Var> inline Term<Var> operator-(const Term<Var> &t){ return Term<Var>::minus(t); }

  /* Wrong number of arguments to predicate */
  class ArgcError : public std::exception{
  public:
    ArgcError(){};
    virtual ~ArgcError() throw() {};
    virtual const char *what() const throw() {
      return "Wrong number of arguments given to predicate.";
    };
  };

  template<class Var> class AppliedPredicate;

  template<class Var> class Predicate : private SyntaxString<Var>{
  public:
    /* A predicate (of no arguments) stating that b holds for process
     * pid. */
    static Predicate from_bexpr(const Lang::BExpr<int> &b, int pid);
    static Predicate tt; // true
    static Predicate ff; // false
    static Predicate eq(const Term<Var> &a, const Term<Var> &b) { return Predicate(SyntaxString<Var>::eq(a,b)); };
    static Predicate neq(const Term<Var> &a, const Term<Var> &b) { return Predicate(SyntaxString<Var>::neq(a,b)); };
    static Predicate lt(const Term<Var> &a, const Term<Var> &b) { return Predicate(SyntaxString<Var>::lt(a,b)); };
    static Predicate leq(const Term<Var> &a, const Term<Var> &b) { return Predicate(SyntaxString<Var>::leq(a,b)); };
    static Predicate gt(const Term<Var> &a, const Term<Var> &b) { return Predicate(SyntaxString<Var>::gt(a,b)); };
    static Predicate geq(const Term<Var> &a, const Term<Var> &b) { return Predicate(SyntaxString<Var>::geq(a,b)); };
    static Predicate neg(const Predicate &p) { return Predicate(SyntaxString<Var>::neg(p)); };
    static Predicate conj(const Predicate &a, const Predicate &b) { return Predicate(SyntaxString<Var>::conj(a,b)); };
    static Predicate disj(const Predicate &a, const Predicate &b) { return Predicate(SyntaxString<Var>::disj(a,b)); };
#if HAVE_LIBMATHSAT == 1
    static Predicate from_msat_term(MSat::msat_env &env, MSat::msat_term t,
                                    const std::function<Var(const std::string&)> &stv) {
      return Predicate<Var>(SyntaxString<Var>::from_msat_term(env,t,stv));
    };
#endif
    /* Creates and returns a MathSat term corresponding to this
     * predicate in environment env. All occurrences of Var v will be
     * declared as var_decl_map[v] if there is such an entry in
     * var_decl_map. Otherwise an entry var_decl_map[v] = dcl will be
     * created where dcl is a declaration for "v#" where # is a number
     * between var_decl_map.size() (inclusive) and var_decl_map.size()
     * + (the number of variables occuring in this predicate but not
     * in var_decl_map) (exclusive).
     *
     * Pre: For all Var v with an entry dcl in var_decl_map, dcl should
     * be a declaration in env for integer constant "v#" where # is a
     * number in the interval 0 <= # < var_decl_map.size().
     * This Predicate is nullary.
     */
#if HAVE_LIBMATHSAT == 1
    MSat::msat_term to_msat_term(MSat::msat_env env,std::map<Var,MSat::msat_decl> var_decl_map,
                                 const std::function<std::string(const Var&)> &vts) const{
      assert(this->arg_count == 0);
      return SyntaxString<Var>::to_msat_term(env,var_decl_map,vts);
    };
#endif

    /* Converts this Predicate<Var> to a Predicate<Var2> by replacing
     * each variable v with vc(v).
     */
    template<class Var2> Predicate<Var2> convert(std::function<Var2(const Var&)> &vc) const throw(){
      return Predicate<Var2>(SyntaxString<Var>::convert(vc));
    };

    Predicate substitute(const Term<Var> &t,const Var &v) const throw();
    int get_argc() const throw() { return this->arg_count; };
    /* Apply predicate to arguments argv. */
    AppliedPredicate<Var> operator()(const std::vector<Var> &argv) const throw(ArgcError*);
    /* Bind arguments to variables given in argv.
     * The result is a nullary predicate. 
     * 
     * Pre: argv.size() == get_argc()*/
    Predicate<Var> bind(const std::vector<Var> &argv) const throw();
    /* Returns a predicate, logically equivalent to this one, but with
     * no negations. */
    Predicate<Var> drive_in_negations() const throw(){
      return Predicate<Var>(SyntaxString<Var>::drive_in_negations());
    };
    std::string to_string(const std::function<std::string(const Var&)> &vts) const throw(){
      return SyntaxString<Var>::to_string(vts);
    };
    std::string to_string(const std::function<std::string(int,int)> &regts,
                          const std::function<std::string(Lang::NML)> &nmlts) const throw(){
      return SyntaxString<Var>::to_string([&regts,&nmlts](const Var &v){ return v.to_string(regts,nmlts); });
    };
    std::string to_string(const std::function<std::string(const Var&)> &vts, const std::vector<std::string> &arg_names) const throw(){
      return SyntaxString<Var>::to_string(vts,arg_names);
    };
    std::string to_string(const std::function<std::string(int,int)> &regts,
                          const std::function<std::string(Lang::NML)> &nmlts, 
                          const std::vector<std::string> &arg_names) const throw(){
      return SyntaxString<Var>::to_string([&regts,&nmlts](const Var &v){ return v.to_string(regts,nmlts); },arg_names);
    };
    
    std::set<Var> get_variables() const throw();

    /* Comparison of predicates.
     * Defines a total order on predicates.
     * Return value:
     *   0 if this and p are identical
     *  -1 if this is smaller than p
     *   1 if this is greater than p
     */
    int compare(const Predicate &p) const throw(){ return SyntaxString<Var>::compare(p); };
    bool operator==(const Predicate &p) const throw() { return this->compare(p) == 0; };
    bool operator!=(const Predicate &p) const throw() { return this->compare(p) != 0; };
    bool operator<(const Predicate &p) const throw() { return this->compare(p) < 0; };
    /* Replaces each constant c with t(c). */
    void translate(std::function<Var(const Var&)> &t){
      SyntaxString<Var>::translate(t);
    };
    /* Returns true if this predicate contains constant variables. */
    bool has_constants() const throw() { return this->const_count > 0; };
    /* Returns a predicate identical to this one, except that each
     * variable occuring in the predicate has been replaced by a fresh
     * argument.
     * The returned predicate p will satisfy:
     * p.get_variables() == (the empty set).
     * p.get_argc() ==  this.get_argc() + this.get_variables().size()*/
    Predicate generalise() const throw(){
      return Predicate<Var>(SyntaxString<Var>::generalise());
    };
    /* Performs a few translations of this predicate which preserve
     * the meaning of the predicate but most likely makes it more
     * appealing to the eye.
     */
    Predicate simplify() const throw(){
      return Predicate<Var>(SyntaxString<Var>::simplify());
    };
    /* Recursively returns all conjuncts of the outermost conjunction
     * of this predicate. I.e. If this predicate is (p0 && p1) then
     * the union of p0.conjuncts() and p1.conjuncts() is returned. If
     * this predicate is not a conjunction, the singleton list
     * containing this predicate is returned.
     *
     * Pre: this->get_argc() == 0
     */
    std::list<Predicate<Var> > conjuncts() const throw();
  private:
    Predicate(const SyntaxString<Var> &ss) : SyntaxString<Var>(ss) {};
    template<class V> friend class AppliedPredicate;
    template<class V> friend class Predicate;
  };

  template<class Var> inline Predicate<Var> operator&&(const Predicate<Var> &a, const Predicate<Var> &b){
    return Predicate<Var>::conj(a,b);
  };
  template<class Var> inline Predicate<Var> operator||(const Predicate<Var> &a, const Predicate<Var> &b){
    return Predicate<Var>::disj(a,b);
  };
  template<class Var> inline Predicate<Var> operator!(const Predicate<Var> &p){
    return Predicate<Var>::neg(p);
  };

  template<class Var> class AppliedPredicate{
  public:
    AppliedPredicate<Var> (const Predicate<Var>  *p,const std::vector<Var> &argv) 
    throw(ArgcError*);
    std::string to_string(const std::function<std::string(const Var&)> &vts) const throw();
    std::string to_string(const std::function<std::string(int,int)> &regts,
                          const std::function<std::string(Lang::NML)> &nmlts) const throw(){
      return to_string([&regts,&nmlts](const Var &v){return v.to_string(regts,nmlts);});
    };
    /* The set of all variables in the predicate of this applied
     * predicate and all variables in the arguments to this applied
     * predicate.
     */
    std::set<Var> get_variables() const throw();
    const Predicate<Var>  *get_predicate() const throw() { return pred; };
    const std::vector<Var> &get_argv() const throw() { return argv; };
    bool operator==(const AppliedPredicate<Var> &) const throw();
    bool operator!=(const AppliedPredicate<Var>  &ap) const throw() { return !(*this == ap); };
    bool operator<(const AppliedPredicate<Var> &) const throw();
    /* Replaces each argument a with t(a).
     *
     * Pre: t does not throw an exception for any variable occuring as
     * an argument in this applied predicate.
     */
    void translate_args(std::function<Var(const Var&)> &t);
  private:
    const Predicate<Var>  *pred; // Not owned
    std::vector<Var> argv;
  };

};

#include "predicates.tcc"

#endif
