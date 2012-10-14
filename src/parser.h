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

#ifndef __PARSER_H__
#define __PARSER_H__

#include <iostream>
#include <vector>
#include "lang.h"
#include "predicates.h"
#include "lexer.h"

namespace Parser{
  class SyntaxError : public std::exception{
  public:
    SyntaxError(std::string s,Lexer::TokenPos pos) : msg("Syntax Error: "+s), pos(pos) {};
    Lexer::TokenPos get_pos() const { return pos; };
    virtual ~SyntaxError() throw() {};
    const char *what() const throw() { return msg.c_str(); };
  private:
    std::string msg;
    Lexer::TokenPos pos;
  };

  /***************************************/
  /*         Type definitions            */
  /***************************************/
  typedef Lang::Expr<std::string> expr_t;
  typedef Lang::BExpr<std::string> bexpr_t;
  typedef Lang::MemLoc<std::string> memloc_t;
  typedef Lang::Stmt<std::string> stmt_t;
  struct memloc_or_pointer_t{
    memloc_or_pointer_t(const expr_t &e, const Lexer::TokenPos &p) 
      : is_pointer(true), memloc(memloc_t::global("")), pointer(e), pos(p) {};
    memloc_or_pointer_t(const memloc_t &ml, const Lexer::TokenPos &p)
      : is_pointer(false), memloc(ml), pointer(expr_t::integer(0)), pos(p) {};
    bool is_pointer;
    memloc_t memloc;
    expr_t pointer;
    Lexer::TokenPos pos;
  };

  typedef Predicates::Predicate<Predicates::DummyVar> predicate_t;
  /* Each element v in a forbidden_t is a vector specifying an illegal
   * combination of program locations. The illegal combination is that
   * where for each process p, either v[p].first is true (meaning any
   * program location) or v[p].second describes the current location
   * of p.
   */
  typedef std::vector<std::vector<std::pair<bool,Lang::label_t> > > forbidden_t;

  /* A Proc represents a process clause from the pseudo code: variable
   * initialisations and code. */
  class Proc{
  public:
    Proc(std::vector<Lang::VarDecl> vi,std::vector<Lang::VarDecl> ri,stmt_t s) : vars(vi), regs(ri), code(s) {};
    const stmt_t &get_code() const throw() { return code; };
    std::vector<Lang::VarDecl> vars;
    std::vector<Lang::VarDecl> regs;
  private:
    stmt_t code;
  };

  /* A Test represents a whole pseudo code file: initialisation of
   * global variables and a number of processes.  Processes specified
   * with "process(n)" occur repeated n times in the vector
   * processes. */
  struct Test{
    /* global_vars[i] gives the initial value for the global variable i. */
    std::vector<Lang::VarDecl> global_vars;
    std::vector<Proc> processes;
    forbidden_t forbidden;
    std::vector<predicate_t> predicates;
  };

  /* Contains various information that may be necessary while parsing
   * statements.
   */
  struct Context{
    Context() {};
    Context(const std::vector<Lang::VarDecl> &gvars) 
      : global_vars(gvars) {};
    Context(const std::vector<Lang::VarDecl> &gvars, const std::vector<Lang::VarDecl> &regs) 
      : global_vars(gvars), regs(regs) {};
    /* Declarations for all global variables. */
    std::vector<Lang::VarDecl> global_vars;
    /* Declarations for all registers of the currently parsed process */
    std::vector<Lang::VarDecl> regs;
  };

  /***************************************/
  /*          parsing functions          */
  /***************************************/
  expr_t p_expr(Lexer&) throw(SyntaxError*,Lang::Exception*,Lexer::BadToken*);
  bexpr_t p_bexpr(Lexer&) throw(SyntaxError*,Lang::Exception*,Lexer::BadToken*);
  memloc_or_pointer_t p_memloc(Lexer&,const Context&) throw(SyntaxError*,Lang::Exception*,Lexer::BadToken*);
  stmt_t p_stmt(Lexer&,const Context&) throw(SyntaxError*,Lang::Exception*,Lexer::BadToken*);
  Test p_test(Lexer&) throw(SyntaxError*,Lang::Exception*,Lexer::BadToken*);
};

#endif // __PARSER_H__
