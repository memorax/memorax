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

#ifndef __LANG_H__
#define __LANG_H__

#include <iostream>
#include <memory>
#include <set>
#include <vector>
#include <utility>
#include <sstream>
#include <stdexcept>
#include "log.h"
#include "syntax_string.h"
#include "lexer.h"
#include "vecset.h"

namespace Predicates {
  template<class Var> class Term;
  template<class Var> class Predicate;
};

namespace Lang {

  class Exception : public std::exception{
  public:
    Exception(std::string m) : msg(m) {};
    virtual const char *what() const throw() { return msg.c_str(); };
    virtual ~Exception() throw() {};
  private:
    std::string msg;
  };

  /* A data value. Either an integer value or a wildcard. */
  class Value{
  public:
    Value() throw() : wild(true), value(0) {};       // Wildcard
    Value(int v) throw() : wild(false), value(v) {}; // Integer v
    bool is_wild() const throw() { return wild; };
    bool is_star() const throw() { return is_wild(); };
    int get_value() const throw() { return value; };
    std::string to_string() const throw(){
      std::stringstream ss;
      if(wild){
        return "*";
      }else{
        ss << value;
        return ss.str();
      }
    };
  private:
    bool wild; // If true, the value is a wildcard, and the member value is irrelevant.
    int value;
  };

  /* Describes variable declarations.
   *
   * A variable is associated with a name, an initial value and a
   * domain.
   */
  class VarDecl{
  public:
    class Domain;
    VarDecl(std::string name, Lang::Value value, Domain domain)
      : name(name), value(value), domain(domain) {};
    class Domain{
    public:
      // Z
      Domain() : dom_is_int(true) {};
      // [lb,ub]
      Domain(int lb, int ub) : dom_is_int(false), lb(lb), ub(ub) {
        if(ub < lb) throw new std::logic_error("Lang::Domain: Invalid interval.");
      };
      // Is this domain Z?
      bool is_int() const { return dom_is_int; };
      bool is_finite() const { return !dom_is_int; };
      int get_lower_bound() const {
        if(dom_is_int)
          throw new std::logic_error("Lang::Domain::get_lower_bound: Integer domain has no lower bound.");
        return lb;
      };
      int get_upper_bound() const {
        if(dom_is_int)
          throw new std::logic_error("Lang::Domain::get_upper_bound: Integer domain has no upper bound.");
        return ub;
      };
      // Is i inside the domain?
      bool member(int i) const{ return dom_is_int || (lb <= i && i <= ub); };
      class const_iterator{
      public:
        const_iterator(int cur, int lb, int ub) : cur(cur), lb(lb), ub(ub) {
          if(cur < lb || cur > ub){
            cur = ub+1; // Canonical form for end iterator
          }
        };
        const_iterator(const const_iterator &) = default;
        const_iterator &operator=(const const_iterator&) = default;
        int operator*() const{
          if(cur < lb || cur > ub)
            throw new std::logic_error("Lang::Domain::operator*: Invalid iterator.");
          return cur;
        };
        bool operator==(const const_iterator &it) const{
          assert(lb == it.lb && ub == it.ub);
          return cur == it.cur;
        };
        bool operator!=(const const_iterator &it) const{
          assert(lb == it.lb && ub == it.ub);
          return cur != it.cur;
        };
        bool operator<(const const_iterator &it) const{
          assert(lb == it.lb && ub == it.ub);
          return cur < it.cur;
        };
        bool operator<=(const const_iterator &it) const{
          assert(lb == it.lb && ub == it.ub);
          return cur <= it.cur;
        };
        const_iterator operator++(int){ // postfix
          const_iterator cp(*this);
          ++*this;
          return cp;
        };
        const_iterator &operator++(){ // prefix
          if(cur <= ub) cur++;
          return *this;
        };
        const_iterator operator--(int){ // postfix
          const_iterator cp(*this);
          --*this;
          return cp;
        };
        const_iterator &operator--(){ // prefix
          if(lb < cur) cur--;
          return *this;
        };
      private:
        int cur, lb, ub;
      };
      const_iterator begin() const{
        if(dom_is_int)
          throw new std::logic_error("Lang::Domain::begin: Cannot give iterator to infinite domain.");
        return const_iterator(lb,lb,ub);
      };
      const_iterator end() const{
        if(dom_is_int)
          throw new std::logic_error("Lang::Domain::end: Cannot give iterator to infinite domain.");
        return const_iterator(ub+1,lb,ub);
      };
      std::string to_string() const{
        std::stringstream ss;
        if(dom_is_int){
          ss << "Z";
        }else{
          ss << "[" << lb << ":" << ub << "]";
        }
        return ss.str();
      };
    private:
      bool dom_is_int; // True iff the domain is Z
      /* If !dom_is_int, then the domain is [lb,ub].
       *
       * Invariant: if !dom_is_int, then lb <= ub.
       */
      int lb, ub;
    };
    std::string name;   // The name of the variable
    Lang::Value value;  // Initial value of the variable
    Domain domain;      // The domain of the variable
  };

  inline std::function<std::string(const int&)> int_reg_to_string(){
    return [](const int &i)->std::string{
      std::stringstream ss;
      ss << "$reg:" << i;
      return ss.str();
    };
  };

  template<class RegId> class Expr : private SyntaxString<RegId> {
  public:
    std::string to_string(const std::function<std::string(const RegId&)> &rt) const throw(){
      return SyntaxString<RegId>::to_string(rt);
    };
    /* Constructors */
    static Expr integer(int i) { return Expr(SyntaxString<RegId>::integer(i)); };
    static Expr reg(const RegId &r) { return Expr(SyntaxString<RegId>::variable(r)); };
    static Expr plus(const Expr &a, const Expr &b) { return Expr(SyntaxString<RegId>::plus(a,b)); };
    static Expr minus(const Expr &a, const Expr &b) { return Expr(SyntaxString<RegId>::minus(a,b)); };
    static Expr minus(const Expr &a) { return Expr(SyntaxString<RegId>::minus(a)); }; // Unary minus

    bool is_integer() const throw() { return SyntaxString<RegId>::is_integer(); };

    /* Pre: this->is_integer() */
    int get_integer() const throw() { return SyntaxString<RegId>::get_integer(); };

    template<class RegId2> Expr<RegId2> convert(std::function<RegId2(const RegId&)> &rc) const;

    template<typename C, typename A> int eval(C c, A a) const{
      return SyntaxString<RegId>::eval(c,a);
    };

    std::set<RegId> get_registers() const throw();
    /* Compares *this to e. Returns -1 if *this is smaller than e, 0
     * if *this is equal to e, and 1 if *this is greater than e.
     *
     * The comparison implements a total order which takes into
     * account only the shapes and values of the expressions, not
     * where in memory they reside.
     */
    int compare(const Expr<RegId> &e) const throw(){
      return SyntaxString<RegId>::compare(e);
    };

    /* Returns true iff the expression is arithmetic, i.e., an
     * integer, a register or an arithmetic operator applied to
     * arithmetic arguments. Pointers are not arithmetic. Neither are
     * 'me' and 'other(n)'.
     *
     * Note that pointers, 'me' and 'other' are not currently
     * supported.
     */
    bool is_arithmetic() const throw() { return true; };
  private:
    Expr(const SyntaxString<RegId> &ss) : SyntaxString<RegId>(ss) {};
    template<class RegId2> friend class Expr;
    template<class RegId2> friend class BExpr;
    template<class Var> friend class Predicates::Term;
  };

  template<class RegId> Expr<RegId> operator+(const Expr<RegId> &a, const Expr<RegId> &b){
    return Expr<RegId>::plus(a,b);
  };
  template<class RegId> Expr<RegId> operator-(const Expr<RegId> &a, const Expr<RegId> &b){
    return Expr<RegId>::minus(a,b);
  };
  template<class RegId> Expr<RegId> operator-(const Expr<RegId> &a){
    return Expr<RegId>::minus(a);
  };

  template<class RegId> class BExpr : private SyntaxString<RegId> {
  public:
    virtual std::string to_string(const std::function<std::string(const RegId&)> &rt) const throw(){
      return SyntaxString<RegId>::to_string(rt);
    };
    virtual std::set<RegId> get_registers() const throw();
    /* Compares *this to e. Returns -1 if *this is smaller than e, 0
     * if *this is equal to e, and 1 if *this is greater than e.
     *
     * The comparison implements a total order which takes into
     * account only the shapes and values of the expressions, not
     * where in memory they reside.
     */
    virtual int compare(const BExpr<RegId> &e) const throw(){
      return SyntaxString<RegId>::compare(e);
    };

    template<class RegId2> BExpr<RegId2> convert(std::function<RegId2(const RegId&)> &rc) const;

    template<typename C, typename A> bool eval(C c, A a) const{
      return SyntaxString<RegId>::eval(c,a);
    };

    /* Constructors */
    static BExpr tt(){
      static BExpr<RegId> t(SyntaxString<RegId>::tt());
      return t;
    };
    static BExpr ff(){
      static BExpr<RegId> f(SyntaxString<RegId>::ff());
      return f;
    };
    static BExpr eq(const Expr<RegId> &a, const Expr<RegId> &b){
      return BExpr(SyntaxString<RegId>::eq(a,b));
    };
    static BExpr neq(const Expr<RegId> &a, const Expr<RegId> &b){
      return BExpr(SyntaxString<RegId>::neq(a,b));
    };
    static BExpr lt(const Expr<RegId> &a, const Expr<RegId> &b){
      return BExpr(SyntaxString<RegId>::lt(a,b));
    };
    static BExpr leq(const Expr<RegId> &a, const Expr<RegId> &b){
      return BExpr(SyntaxString<RegId>::leq(a,b));
    };
    static BExpr gt(const Expr<RegId> &a, const Expr<RegId> &b){
      return BExpr(SyntaxString<RegId>::gt(a,b));
    };
    static BExpr geq(const Expr<RegId> &a, const Expr<RegId> &b){
      return BExpr(SyntaxString<RegId>::geq(a,b));
    };
    static BExpr neg(const BExpr<RegId> &a){
      return BExpr(SyntaxString<RegId>::neg(a));
    };
    static BExpr conj(const BExpr<RegId> &a, const BExpr<RegId> &b){
      return BExpr(SyntaxString<RegId>::conj(a,b));
    };
    static BExpr disj(const BExpr<RegId> &a, const BExpr<RegId> &b){
      return BExpr(SyntaxString<RegId>::disj(a,b));
    };
  private:
    BExpr(const SyntaxString<RegId> &ss) : SyntaxString<RegId>(ss) {};
    template<class RegId2> friend class BExpr;
    template<class Var> friend class Predicates::Predicate;
  };

  template<class RegId> BExpr<RegId> operator!(const BExpr<RegId> &a){
    return BExpr<RegId>::neg(a);
  };
  template<class RegId> BExpr<RegId> operator&&(const BExpr<RegId> &a, const BExpr<RegId> &b){
    return BExpr<RegId>::conj(a,b);
  };
  template<class RegId> BExpr<RegId> operator||(const BExpr<RegId> &a, const BExpr<RegId> &b){
    return BExpr<RegId>::disj(a,b);
  };

  typedef std::string label_t; // "" means no label.

  template<class Id> class MemLoc{
  public:
    /* Constructors */
    static MemLoc<Id> global(Id) throw();
    static MemLoc<Id> int_deref(int i) throw(); /* pre: i >= 0 */
    static MemLoc<Id> reg_deref(Id) throw();
    static MemLoc<Id> local(Id) throw(); // Id[my]
    static MemLoc<Id> local(Id,int p) throw(); /* Id[p], pre: p >= 0 */
    /* Functions */
    enum Type {
      GLOBAL_ID,        // Global variable by id
      GLOBAL_INT_DEREF, // Global variable by literal pointer
      GLOBAL_REG_DEREF, // Global variable by in-register pointer
      LOCAL             // Local variable given by id and owner
    };
    Type get_type() const throw();
    bool is_global() const throw();
    bool is_local() const throw();
    Id get_id() const throw();  // pre: type in { GLOBAL_ID, LOCAL }
    Id get_reg() const throw(); // pre: type == GLOBAL_REG_DEREF
    int get_pointer() const throw(); // pre: type == GLOBAL_INT_DEREF
    /* pre: type == LOCAL
     * post: Provided that caller is the PID of the calling process,
     * the PID of the process owning this local variable. */
    int get_owner(int caller) const throw();
    /* Provided that this MemLoc is from the perspective of process
     * old_caller, then returns a new MemLoc describing the same
     * memory location from the perspective of process new_caller.
     */
    MemLoc<Id> change_caller(int old_caller, int new_caller) const throw();
    std::string to_string() const throw();
    std::set<Id> get_registers() const throw();
    bool operator==(const MemLoc<Id>&) const throw();
    bool operator!=(const MemLoc<Id> &ml) const throw() { return !(*this == ml); };
    bool operator<(const MemLoc<Id>&) const throw();
  private:
    MemLoc();
    Type type;
    Id id; // Takes the role of the register if type == GLOBAL_REG_DEREF
    /* -1 represents me. Otherwise, if owner < caller_pid, the owner
     * process has PID == owner and if owner >= caller_pid then the
     * owner process has PID == owner+1.
     */
    int owner;
    int ptr;
  };

  template<class RegId> inline std::ostream &operator<<(std::ostream &os,
                                                        const MemLoc<RegId> &ml){
    return os << ml.to_string();
  }

  inline std::function<std::string(const MemLoc<int>&)> int_memloc_to_string(){
    return [](const MemLoc<int> &ml){
      return ml.to_string();
    };
  };

  /* Identifies a memory location from a global perspective, rather
   * than from the perspective of a particular caller process.
   *
   * NML - Normalized Memory Location
   */
  class NML{
  public:
    /* Identifies the same memory location as ml would from the
     * perspective of caller.
     *
     * Pre: ml is not a register dereference.
     */
    NML(const MemLoc<int> &ml,int caller) throw(std::logic_error*);
    /* Global memory location id. */
    static NML global(int id) { return NML(id); };
    /* Local memory location id of process owner. */
    static NML local(int id, int owner) { return NML(id,owner); };
    bool is_global() const throw() { return owner == -1; };
    bool is_local() const throw() { return owner != -1; };
    int get_id() const throw() { return id; };
    int get_owner() const throw(){ return owner; };
    std::string to_string() const throw();
    bool operator==(const NML&) const throw();
    bool operator!=(const NML &nml) const throw() { return !(*this == nml); };
    bool operator<(const NML&) const throw();
    /* The memory location from the perspective of process caller. */
    MemLoc<int> localize(int caller) const{
      if(owner == -1){
        return MemLoc<int>::global(id);
      }else if(owner == caller){
        return MemLoc<int>::local(id);
      }else if(owner < caller){
        return MemLoc<int>::local(id,owner);
      }else{
        return MemLoc<int>::local(id,owner-1);
      }
    };
  private:
    /* Global memory location id. */
    NML(int id);
    /* Local memory location id of process owner. */
    NML(int id, int owner);
    /* The pid of the owning process, if the memory location is
     * local. -1 if the memory location is global.
     */
    int owner;
    int id;
  };

  inline std::ostream &operator<<(std::ostream &os,
                                  const NML &nml){
    return os << nml.to_string();
  }

  /* Tags identifying the type of statement represented by a Stmt object.
   *
   * Also defines the interpretation of the private members of the object.
   */
  enum stmt_t {
    /* Nop */
    NOP,
    /* Assignment: reg := e0 */
    ASSIGNMENT,
    /* Assume: assume: b */
    ASSUME,
    /* Blocking read: read: reads[0] = e0 */
    READASSERT,
    /* Assigning read: read: reg := reads[0] */
    READASSIGN,
    /* Write: write: writes[0] := e0 */
    WRITE,
    /* Locked block: 
     * locked {
     *   stmts[0]
     * or
     *   ...
     * or
     *   stmts[stmt_count-1]
     * }
     * Invariant: 
     *   No labels occur in stmts.
     *   The only kinds statements which may occur in stmts are
     *   nop, assignment, assume, readassert, readassign, write, locked, sequence
     */
    LOCKED,
    /* Store-store locked block: 
     * slocked {
     *   stmts[0]
     * or
     *   ...
     * or
     *   stmts[stmt_count-1]
     * }
     * Invariant: 
     *   No labels occur in stmts.
     *   The only kind of statement which may occur in stmts are write.
     */
    SLOCKED,
    /* Goto: goto lbl */
    GOTO,
    /* Update: An update concerning memory location writes[0], and a
     * write performed by process writer. */
    UPDATE,
    /* Serialise: A serialisation of a write to memory location writes[0] */
    SERIALISE,
    /* If statement:
     * If stmt_count == 1 then: if b then stmts[0]
     * If stmt_count == 2 then: if b then stmts[0] else stmts[1]
     */
    IF,
    /* While statement: while b do stmts[0] */
    WHILE,
    /* Either statement: 
     * either{
     *   stmts[0]
     * or 
     *   ...
     * or
     *   stmts[stmt_count-1]
     * }
     *
     * Invariant: For all i: stmts[i].lbl == ""
     */
    EITHER,
    /* Sequence of statements:
     * { stmts[0], ..., stmts[stmt_count-1] }
     */
    SEQUENCE
  };

  /* Class of statements.
   *
   * A statement object has one of several types, as defined by the
   * its type field. The type defines which additional fields are
   * relevant.
   */
  template<class RegId> class Stmt {
  public:
    /**********************************
     *        Constructors            *
     **********************************/
    Stmt(const Stmt &);           // Deep copy
    Stmt &operator=(const Stmt&); // Deep copy
    virtual ~Stmt() throw();
    /* Nop */
    static Stmt<RegId> nop(const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Assignment: reg := e */
    static Stmt<RegId> assignment(RegId reg, const Expr<RegId> &e,
                                  const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Assume: assume: b */
    static Stmt<RegId> assume(const BExpr<RegId> &b,
                              const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Blocking read: read: ml = e */
    static Stmt<RegId> read_assert(MemLoc<RegId> ml, const Expr<RegId> &e,
                                   const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Assigning read: read: reg := ml */
    static Stmt<RegId> read_assign(RegId reg, MemLoc<RegId> ml,
                                   const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Non-locked write: write: ml := e */
    static Stmt<RegId> write(MemLoc<RegId> ml, const Expr<RegId> &e,
                             const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Locked block: 
     * locked{
     *   ss[0]
     * or
     *   ...
     * or
     *   ss[ss.size()-1]
     * }
     *
     * Pre: ss contains no labels
     * All statements in ss (recursively) are of types
     * nop, assignment, assume, read_assert, read_assign, write, locked or sequence
     * ss.size() > 0
     */
    static Stmt<RegId> locked_block(const std::vector<Stmt> &ss,
                                    const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Locked write: locked write: ml := e */
    static Stmt<RegId> locked_write(MemLoc<RegId> ml, const Expr<RegId> &e,
                                    const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Cas: cas(ml,e0,e1) */
    static Stmt<RegId> cas(MemLoc<RegId> ml, const Expr<RegId> &e0, const Expr<RegId> &e1,
                           const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Store-store locked block:
     * slocked {
     *   ss[0]
     * or
     *   ...
     * or
     *   ss[ss.size()-1]
     * }
     *
     * Pre: ss contains no labels
     *      All statements in ss are of type write.
     *      ss.size() > 0
     */ 
    static Stmt<RegId> slocked_block(const std::vector<Stmt> &ss,
                                     const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Store-store locked write: slocked write: ml := e */
    static Stmt<RegId> slocked_write(MemLoc<RegId> ml, const Expr<RegId> &e,
                                    const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Goto: goto lbl */
    static Stmt<RegId> goto_stmt(label_t lbl,
                                 const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Update: An update concerning memory locations mls, and a
     * write performed by process writer. 
     * 
     * Memory locations in mls should be from the perspective of the
     * owner of the update, not from the process writer (unless the
     * owner and the writer are the same).
     *
     * Note that in pure TSO the only kind of update(w,mls) that can
     * be performed by a process p is one where w == p and mls is a
     * singleton. Support for other kinds of updates is provided for
     * SB.
     */
    static Stmt<RegId> update(int writer, VecSet<MemLoc<RegId> > mls,
                              const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Serialise: A serialisation of a write to memory location writes[0] 
     * This operation concern only the PSO memory model abstraction PWS.
     */
    static Stmt<RegId> serialise(VecSet<MemLoc<RegId>> mls,
                              const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Type of a labeled statement. */
    struct labeled_stmt_t{
      labeled_stmt_t() : lbl(""), stmt() {};
      labeled_stmt_t(label_t l, const Stmt<RegId> &s) : lbl(l), stmt(s) {};
      labeled_stmt_t(const Stmt<RegId> &s) : lbl(""), stmt(s) {};
      label_t lbl; // lbl == "" represents an unlabeled statement
      Stmt<RegId> stmt;
      /* Defines a total order over labeled statements.
       * 
       * Returns 0 if *this is equal to lstmt, -1 if *this is smaller
       * than lstmt and 1 if *this is greater than lstmt.
       */
      int compare(const labeled_stmt_t &lstmt) const throw();
    };
    /* If statement: if b then s */
    static Stmt<RegId> if_stmt(const BExpr<RegId> &b,
                               const labeled_stmt_t &s,
                               const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* If statement: if b then s0 else s1 */
    static Stmt<RegId> if_stmt(const BExpr<RegId> &b,
                               const labeled_stmt_t &s0,
                               const labeled_stmt_t &s1,
                               const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* While statement: while b do s */
    static Stmt<RegId> while_stmt(const BExpr<RegId> &b,
                                  const labeled_stmt_t &s,
                                  const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Either statement: 
     * either{
     *   ss[0]
     * or
     *   ...
     * or
     *   ss[ss.size()-1]
     * }
     * 
     * Pre: ss.size() > 0
     */
    static Stmt<RegId> either(const std::vector<Stmt> &ss,
                              const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));
    /* Sequence statement: { ss[0], ..., ss[ss.size()-1] } */
    static Stmt<RegId> sequence(const std::vector<labeled_stmt_t> &ss,
                                const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));

    /**********************************
     *          Getters               *
     **********************************/
    stmt_t get_type() const throw() { return type; };
    /* Defined for read assert, read assign, write and update */
    MemLoc<RegId> get_memloc() const;
    RegId get_reg() const throw() { return reg; };
    const Expr<RegId> &get_expr() const throw() { return *e0; };
    const Expr<RegId> &get_expr0() const throw() { return *e0; };
    const Expr<RegId> &get_expr1() const throw() { return *e1; };
    const BExpr<RegId> &get_condition() const throw() { return *b; };
    label_t get_goto_target() const throw() { return lbl; };
    bool is_fence() const throw() { return fence; };
    int get_writer() const throw() { return writer; };
    const Stmt<RegId> *get_then_statement() const throw() { return &stmts[0].stmt; };
    /* 0 if there is no else statement */
    const Stmt<RegId> *get_else_statement() const throw(){
      if(stmt_count == 2) return &stmts[1].stmt;
      else return 0;
    };
    label_t get_then_label() const throw() { return stmts[0].lbl; };
    label_t get_else_label() const throw() { return stmts[1].lbl; };
    const Stmt<RegId> *get_statement() const throw() { return &stmts[0].stmt; };
    /* Get the label of the statement s in the while statement while b do s. */
    label_t get_label() const throw() { return stmts[0].lbl; };
    /* In a sequence statement or either statement, get the number of
     * sub statements. */
    int get_statement_count() const throw() { return stmt_count; };
    /* In a sequence statement or either statement, get the i:th sub
     * statement.
     */
    const Stmt<RegId> *get_statement(int i) const throw() { return &stmts[i].stmt; };
    /* In a sequence statement or either statement, get the label of
     * the i:th sub statement.
     */
    label_t get_label(int i) const throw() { return stmts[i].lbl; };
    /* Returns a vector (sorted and without duplicates) of all memory
     * locations ml where this statement contains (or is) a write to
     * ml.
     *
     * An update counts as a write.
     */
    const std::vector<MemLoc<RegId> > &get_writes() const throw() { return writes.get_vector(); };
    /* Returns a vector (sorted and without duplicates) of all memory
     * locations ml where this statement contains (or is) a read to
     * ml.
     */
    const std::vector<MemLoc<RegId> > &get_reads() const throw() { return reads.get_vector(); };
    /* Returns the set of all sets S, such that executing this
     * statement *may* write to precisely the memory locations in
     * S. Here all branching conditions are ignored and all branches
     * are considered.
     */
    VecSet<VecSet<MemLoc<RegId> > > get_write_sets() const throw();
    /* Returns true if this statement contains (or is) a read. */
    bool contains_read() const throw(){ return reads.size() > 0; };
    /* Returns the set of registers occurring in this statement. */
    std::set<RegId> get_registers() const;
    /* Returns the position in the source code at which this statement occurs.
     * (Defaults to (-1,-1) if no position is given.) */
    Lexer::TokenPos get_pos() const { return pos; };

    /**********************************
     *            Misc                *
     **********************************/
    /* Returns a new statement, identical to this one, except in that
     * every register r and memory location ml in this statement has
     * been replaced by the register rc(r) and the memory location
     * mlc(ml).
     */
    template<class RegId2> Stmt<RegId2> 
    convert(std::function<RegId2(const RegId&)> &rc,
            std::function<MemLoc<RegId2>(const MemLoc<RegId>&)> &mlc) const;
    /* Returns an Stmt equal to this one, but if this Stmt is a locked
     * statement, then the contained tree-structure has been flattened
     * as follows:
     *
     * The returned locked statement has a sequence of unique
     * alternatives a0,...,an, such that no ai contains a locked
     * statement. Sequences ai = {s0,...,{s0',...,sk'},...,sm} are
     * flattened into {s0,...,s0',...,sk',...sm}. Sequences ai = {s0}
     * are replaced by s0.
     *
     * Pre: This Stmt is an instruction.
     */
    Stmt flatten() const;
    /* Returns a string representation of this statement.
     * 
     * Registers r and memory locations ml will be represented with
     * respectively regts(r) and mlts(ml).
     *
     * If indentation < 0 then the representation will be on one
     * line. If indentation >= 0, then the representation will be
     * indented by indentation spaces and possibly on multiple lines.
     *
     * If label != "" then the statement will be labeled by label in
     * the representation.
     */
    std::string to_string(const std::function<std::string(const RegId&)> &regts, 
                          const std::function<std::string(const MemLoc<RegId> &)> &mlts,
                          int indentation = -1,std::string label = "") const;

    /* Defines a total ordering <= on statements.
     *
     * Returns 0 iff this <= stmt and stmt <= this
     * Otherwise returns -1 iff this <= stmt and
     * returns 1 iff stmt <= this.
     */
    int compare(const Stmt &stmt) const throw();
    bool operator==(const Stmt &stmt) const throw() { return compare(stmt) == 0; };
    bool operator<(const Stmt &stmt) const throw() { return compare(stmt) < 0; };
    bool operator>(const Stmt &stmt) const throw() { return compare(stmt) > 0; };
    bool operator<=(const Stmt &stmt) const throw() { return compare(stmt) <= 0; };
    bool operator>=(const Stmt &stmt) const throw() { return compare(stmt) <= 0; };

    /* Checks if stmt is an acceptable statement as an element of
     * stmts for an LOCKED statement. Returns true if so, false
     * otherwise.
     *
     * If comment != 0 and stmt is not acceptable, then *comment is
     * assigned a human-readable explanation of the failure.
     */
    static bool check_locked_invariant(const Stmt &stmt, std::string *comment);
    /* Checks if stmt is an acceptable statement as an element of
     * stmts for an SLOCKED statement. Returns true if so, false
     * otherwise.
     *
     * If comment != 0 and stmt is not acceptable, then *comment is
     * assigned a human-readable explanation of the failure.
     */
    static bool check_slocked_invariant(const Stmt &stmt, std::string *comment);
    /* Applies f to all substatements of this statement (including
     * this statement itself) in some order. If there is a
     * substatement that satisfies f, then true is returned, otherwise
     * false.
     *
     * If ss is not null, and there is a substatement that satisfies f
     * then *ss is assigned some substatement that satisfies f.
     */
    bool find_substmt(std::function<bool(const Stmt<RegId>&)> &f,Stmt *ss = 0) const throw();
  private:
    /* Creates a nop statement .
     * Sets all fields to default values. */
    Stmt(const Lexer::TokenPos &p = Lexer::TokenPos(-1,-1));

    stmt_t type;
    /* Fields used by the different types of statements.
     *
     * Fields interpretations are given by the type of statement.
     *
     * Fields which are not mentioned by the description of the
     * statement type are null.
     */
    RegId reg;
    VecSet<Lang::MemLoc<RegId> > writes;
    VecSet<Lang::MemLoc<RegId> > reads;
    const Expr<RegId> *e0;
    const Expr<RegId> *e1;
    const BExpr<RegId> *b;
    labeled_stmt_t *stmts;
    bool fence; // true iff type == LOCKED and there is some write substatement
    label_t lbl; // Goto label
    int writer; // For update
    /* If stmt_count > 0, then stmts points to an array of exactly
     * stmt_count statements. Else, stmt_count == 0 and stmts == 0.
     */
    int stmt_count;
    /* The position in the source code at which this statement occurs.
     * (Defaults to (-1,-1) if no position is given.) */
    Lexer::TokenPos pos;

    template<class RegId2> friend class Stmt;

    /* Delete all owned objects */
    void self_destruct();
    /* Clears reads and writes.
     * Then fills them with reads and writes that are in stmts.
     * reads and writes will be sorted and distinct */
    void populate_reads_writes();
    std::vector<std::vector<Stmt> > flatten_aux() const;
  };

  template<class RegId> inline std::ostream &operator<<(std::ostream &os,Expr<RegId> &e){
    return os << e.to_string();
  };

  template<class RegId> inline std::ostream &operator<<(std::ostream &os,BExpr<RegId> &e){
    return os << e.to_string();
  };

  inline std::ostream &operator<<(std::ostream &os,const Value &v){
    return os << v.to_string();
  }

}

#include "lang.tcc"

#endif // __LANG_H__
