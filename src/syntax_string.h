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

#ifndef __SYNTAX_STRING_H__
#define __SYNTAX_STRING_H__

#include <config.h>
#include <string>
#include <vector>
#include <list>
#include <map>
#include <stdexcept>
#include <sstream>
#include "cmsat.h"

template<class Var> class SyntaxString{
public:
  SyntaxString(const SyntaxString &ss);
  virtual ~SyntaxString();
  SyntaxString &operator=(const SyntaxString &ss);

  /* Term constructors */
  static SyntaxString integer(int i);
  static SyntaxString variable(const Var &v);
  static SyntaxString argument(int a); // Projection function for argument a
  static SyntaxString plus(const SyntaxString &a, const SyntaxString &b);
  static SyntaxString minus(const SyntaxString &a, const SyntaxString &b);
  static SyntaxString minus(const SyntaxString &t); // Unary minus

  /* Predicate constructors */
  static SyntaxString tt;
  static SyntaxString ff;
  static SyntaxString eq(const SyntaxString &a, const SyntaxString &b);
  static SyntaxString neq(const SyntaxString &a, const SyntaxString &b);
  static SyntaxString lt(const SyntaxString &a, const SyntaxString &b);
  static SyntaxString leq(const SyntaxString &a, const SyntaxString &b);
  static SyntaxString gt(const SyntaxString &a, const SyntaxString &b);
  static SyntaxString geq(const SyntaxString &a, const SyntaxString &b);
  static SyntaxString neg(const SyntaxString &a);
  static SyntaxString conj(const SyntaxString &a, const SyntaxString &b);
  static SyntaxString disj(const SyntaxString &a, const SyntaxString &b);

  /* Getters */
  bool is_integer() const throw() { return symbols[0] == INT; };
  int get_integer() const throw() { return symbols[1]; }; // Pre: is_integer()

  /* Limits */
  static const int min_int; // The smallest allowed integer literal
  static const int max_int; // The largest allowed integer literal

  /* Compares this to ss by some linear, total order.
   *
   * Returns -1 if this is smaller than ss, 0 if this equals ss or 1
   * if this is greater than ss.
   */
  int compare(const SyntaxString<Var> &ss) const throw();

  std::string to_string(const std::function<std::string(const Var&)> &vts) const;
  std::string to_string(const std::function<std::string(const Var&)> &vts, const std::vector<std::string> &arg_names) const;

  /* Converts all constants c and arguments a to respectively vc(c)
   * and vc(a). */
  template<class Var2> SyntaxString<Var2> convert(std::function<Var2(const Var&)> &vc) const;

  /* Evaluates this expression. Constant variables v are evaluated to
   * c[v]. Arguments v are evaluated to A[v]. If this expression is
   * boolean, then a return value of 1 represents true, and a return
   * value of 0 represents false.
   */
  template<typename C, typename A> int eval(C c, A a) const;

protected:

  /* Makes a nullary term of this term by substituting each occurrence
   * of argument i for a constant argv[i].
   *
   * Pre: argv.size() == this->arg_count
   */
  SyntaxString bind(const std::vector<Var> &argv) const throw();

  /* Substitutes every occurrence of the variable v for the term t in
   * this term/predicate.
   *
   * Pre: t is a term.
   */
  SyntaxString substitute(const SyntaxString<Var> &t, const Var &v) const throw();

  /* Returns a syntax string which is equivalent to this one, but
   * contains no negations.
   */
  SyntaxString drive_in_negations() const throw();

  /* Replaces each variable with a fresh argument. Such that the
   * resulting syntax string contains no constants and as many
   * arguments as the original syntax string plus const_count of the
   * original syntax string. Remaps variables to the next fresh
   * argument in the order they first occur in this syntax string from
   * left to right.
   */
  SyntaxString generalise() const throw();

  /* Performs a few translations of this syntax string which preserve
   * the meaning of the string but most likely makes it more appealing
   * to the eye.
   */
  SyntaxString simplify() const throw();

  /* Recursively returns all conjuncts of the outermost conjunction
   * of this syntax string. I.e. If this syntax string is (p0 && p1) then
   * the union of p0.conjuncts() and p1.conjuncts() is returned. If
   * this predicate is not a conjunction, the singleton list
   * containing this predicate is returned.
   *
   * Pre: this is a nullary predicate.
   */
  std::list<SyntaxString> conjuncts() const throw();

  /* Replaces each constant c with t(c). */
  void translate(std::function<Var(const Var &)> &t);

  /* Creates and returns a MathSat term corresponding to this syntax
   * string in environment env. All occurrences of Var v will be
   * declared as var_decl_map[v] if there is such an entry in
   * var_decl_map. Otherwise an entry var_decl_map[v] = dcl will be
   * created where dcl is a declaration for vts(v).
   *
   * Pre: For all Var v with an entry dcl in var_decl_map, dcl should
   * be a declaration in env for integer constant vts(v).
   * This SyntaxString is nullary.
   * 
   * Pre: For Var v, w, it holds that vts(v) == vts(w) implies v == w.
   */
#if HAVE_LIBMATHSAT == 1
  MSat::msat_term to_msat_term(MSat::msat_env &env, std::map<Var,MSat::msat_decl> &var_decl_map,
                               const std::function<std::string(const Var&)> &vts) const throw();
#endif

  /* Creates and returns a syntax string which corresponds to the term
   * t. A constant in t, with the string representation s will be
   * translated into the variable stv(s).
   *
   * Pre: t is a term containing no literals or constants other than
   * boolean and integer such.
   */
#if HAVE_LIBMATHSAT == 1
  static SyntaxString from_msat_term(MSat::msat_env &env, MSat::msat_term t,
                                     const std::function<Var(const std::string&)> &stv);
#endif

  std::string debug_to_string() const throw();

  /* Defines the names of symbols. The symbols will be stored as
   * symbol_t, however because symbol_t may be smaller than
   * symbol_t_name.
   */
  enum symbol_t_name {
    /* Symbols of abstract type term */
    INT,     // An integer literal INT(value)
    VAR,     // A variable VAR(var index into consts)
    ARG,     // An argument ARG(arg index into argv)
    PLUS,    // PLUS(term,term)
    MINUS,   // MINUS(term,term)
    UNMINUS, // UNMINUS(term)

    /* Symbols of abstract type predicate */
    TRUE,    // TRUE
    FALSE,   // FALSE
    EQ,      // EQ(term,term)
    NEQ,     // NEQ(term,term)
    LT,      // LT(term,term)
    LEQ,     // LEQ(term,term)
    GT,      // GT(term,term)
    GEQ,     // GEQ(term,term)
    NOT,     // NOT(predicate)
    AND,     // AND(predicate,predicate)
    OR       // OR(predicate,predicate)
  };
  typedef short symbol_t;

  typedef int ptr_count_t;

  /* Invariant: Control symbols (the ones in symbol_t_name) are
   * always on even indices. Symbols which are not control are
   * always on odd indices.
   */
  symbol_t *symbols; // A string of symbols
  int symbol_count; // The length of symbols
   
  int arg_count; // The number of arguments
  int const_count;
  /* Variables which are always in the term or predicate. consts ==
   * 0 iff const_count == 0.
   *
   * consts is sorted and distinct
   */
  Var *consts;

  /* Pointer counters */
  ptr_count_t *symbols_ptr_count;
  ptr_count_t *consts_ptr_count; // consts_ptr_count == 0 iff consts == 0

  /* Debug function, which checks various aspects of the invariant.
   * Returns true iff all tests turn out correct.
   */
  bool check_invariant() const;

private:
  /* Returns an inner string representation (without function head)
   * of the term or predicate that starts at position i in
   * symbols. Uses vts to represent constants. If this is an applied
   * predicate, vts is also used to represent arguments, otherwise
   * arg_names[argi] represents argument argi.
   */
  std::string to_inner_string(int i, const std::function<std::string(const Var&)> &vts, std::vector<std::string> arg_names) const;
  void init(const SyntaxString &ss); // To be called by copy constructor and copy operator
  void init(int sc, int cc, int ac); // To be called by constructor(sc,cc,ac)
  void self_destruct(); // To be called by destructor and copy operator

  SyntaxString drive_in_negations(bool sign) const throw();

  SyntaxString(int sc, int cc = 0, int ac = 0);
  SyntaxString(bool b);

  /*
   *
   * Pre: Neither a nor b is applied.
   */
  static SyntaxString<Var> combine(symbol_t op, const SyntaxString<Var> &a, const SyntaxString<Var> &b);
  static SyntaxString<Var> combine(symbol_t op, const SyntaxString<Var> &a);

  inline bool binary_symbol(symbol_t s) const throw(){
    return (s == PLUS) || (s == MINUS) || 
      (s == EQ) || (s == NEQ) ||
      (s == LT) || (s == LEQ) ||
      (s == GT) || (s == GEQ) ||
      (s == AND) || (s == OR);
  };

  /* Note that unary here refers to symbols that take another
   * SyntaxString as an argument, not symbols like INT, VAR and ARG.
   */
  inline bool unary_symbol(symbol_t s) const throw(){
    return (s == UNMINUS) || (s == NOT);
  };

  inline bool predicate_symbol(symbol_t s) const throw(){
    return (s == TRUE) || (s == FALSE) ||
      (s == EQ) || (s == NEQ) ||
      (s == LT) || (s == LEQ) ||
      (s == GT) || (s == GEQ) ||
      (s == NOT) ||
      (s == AND) || (s == OR);
  };

  inline bool term_symbol(symbol_t s) const throw(){
    return (s == INT) || (s == VAR) ||
      (s == ARG) || (s == PLUS) ||
      (s == MINUS) || (s == UNMINUS);
  };

  /* Pre: binary_symbol(symbols[0])
   *
   * Creates a new syntax string which corresponds to the left
   * argument of the outermost (binary) symbol of this syntax string.
   */
  SyntaxString<Var> separate_left_argument() const throw();

  /* Pre: binary_symbol(symbols[0])
   *
   * Creates a new syntax string which corresponds to the right
   * argument of the outermost (binary) symbol of this syntax string.
   */
  SyntaxString<Var> separate_right_argument() const throw();

  /* Pre: unary_symbol(symbols[0])
   *
   * Creates a new syntax string which corresponds to the (only)
   * argument of the outermost (unary) symbol of this syntax string.
   */
  SyntaxString<Var> separate_only_argument() const throw();

  SyntaxString<Var> separate_interval(int offset, int new_symbol_count) const throw();

  /* Same as to_msat_term(env,var_decl_map), but works for the 
   * sub expression starting at symbol i.
   */
#if HAVE_LIBMATHSAT == 1
  MSat::msat_term to_msat_term(int i, MSat::msat_env &env, 
                               std::map<Var,MSat::msat_decl> &var_decl_map,
                               const std::function<std::string(const Var&)> &vts) const throw();
#endif

  /* Used to make calls to MSat::msat_term_is_* uniform in both
   * version 4 and 5 of MathSAT. */
#if HAVE_LIBMATHSAT == 1 && MATHSAT_VERSION == 4
  inline static bool msat_query(int (*q)(MSat::msat_term), MSat::msat_env &env, MSat::msat_term &t){
    return q(t);
  };
#elif HAVE_LIBMATHSAT == 1 && MATHSAT_VERSION == 5
  inline static bool msat_query(int (*q)(MSat::msat_env, MSat::msat_term), MSat::msat_env &env, MSat::msat_term &t){
    return q(env,t);
  };
#endif

  template<class Var2> friend class SyntaxString;
};

#include "syntax_string.tcc"

#endif
