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

#include "parser.h"
#include <sstream>
#include <memory>
#include <set>
#include <vector>
#include "log.h"
#include <functional>
#include "zstar.h"

namespace Parser{
  typedef std::string rettype;
  void force(Lexer&,Lexer::TokenType,std::string = "",std::string = ".");
  expr_t p_expr_arith(Lexer&);
  expr_t p_expr_arith_r(Lexer&,const expr_t&);
  expr_t p_expr_arith_unit(Lexer&);
  bexpr_t p_bexpr_r(Lexer&,const bexpr_t&);
  bexpr_t p_bexpr_and(Lexer&);
  bexpr_t p_bexpr_and_l(Lexer&);
  bexpr_t p_bexpr_and_r(Lexer&,const bexpr_t&);
  bexpr_t p_bexpr_atom(Lexer&);
  bexpr_t p_bexpr_atom_r(Lexer&,const expr_t&);
  stmt_t::labeled_stmt_t p_lstmt(Lexer&,const Context&);
  stmt_t p_stmt_list(Lexer&,const Context&); // Returns a Lang::SequenceStatement.
  stmt_t resolve_pointer(const memloc_or_pointer_t&,
                         const std::function<stmt_t(const memloc_t&)>&,
                         const Context&);
  Lang::label_t p_label(Lexer&);
  std::pair<Proc,int> p_proc(Lexer&,const Context&); // Returns (process,multiplier)
  std::vector<Proc> p_proc_list(Lexer&,const Context&);
  enum declaration_type{
    DCL_ML, // declaration for memory location
    DCL_REG // declaration for register
  };
  Lang::VarDecl p_var_decl(Lexer&,declaration_type);
  std::vector<Lang::VarDecl> p_var_decl_list(Lexer&,declaration_type);
  forbidden_t p_forbidden(Lexer&);
  int p_int(Lexer&);
};

void Parser::force(Lexer &lex, Lexer::TokenType toktyp, 
                   std::string complaint_pre,std::string complaint_post){
  Lexer::Token tok;
  lex >> tok;
  if(tok.type != toktyp){
    if(complaint_pre == "")
      complaint_pre = "Expected "+Lexer::token_type_to_string(toktyp)+" at ";
    throw new  SyntaxError(complaint_pre+tok.pos.to_long_string()+complaint_post,tok.pos);
  }
}

Parser::expr_t Parser::p_expr_arith_unit(Lexer &lex){
  Lexer::Token tok;

  lex >> tok;

  switch(tok.type){
  case Lexer::REG: 
    {
      return expr_t::reg(tok.value);
    }
  case Lexer::NAT:
    {
      int i;
      std::stringstream ss;
      ss << tok.value;
      ss >> i;
      return expr_t::integer(i);
    }
  case Lexer::MINUS:
    {
      expr_t arg(p_expr_arith_unit(lex));
      if(arg.is_integer()){
        return expr_t::integer(-arg.get_integer());
      }
      return -arg;
    }
  case Lexer::LPAREN:
    {
      expr_t e(p_expr_arith(lex));
      force(lex,Lexer::RPAREN,
            "Expected ')' at ",
            " to match '(' at "+tok.pos.to_long_string()+".");
      return e;
    }
  default:
    throw new SyntaxError("Expected expression unit at "+tok.pos.to_long_string()+".",tok.pos);
  }
}

Parser::expr_t Parser::p_expr_arith(Lexer &lex){
  return p_expr_arith_r(lex,p_expr_arith_unit(lex));
}

Parser::expr_t Parser::p_expr_arith_r(Lexer &lex,const expr_t &left){
  Lexer::Token tok;

  lex >> tok;

  switch(tok.type){
  case Lexer::PLUS:
    {
      return p_expr_arith_r(lex,left + p_expr_arith_unit(lex));
    }
  case Lexer::MINUS:
    {
      return p_expr_arith_r(lex,left - p_expr_arith_unit(lex));
    }
  default:
    lex.putback(tok);
    return left;
  }
}

Parser::expr_t Parser::p_expr(Lexer &lex) 
  throw(SyntaxError*,Lang::Exception*,Lexer::BadToken*){
  Lexer::Token tok,tok2;

  lex >> tok;

  switch(tok.type){
  case Lexer::ME:
    throw new std::logic_error("Parser: Literal 'me' not supported.");
  case Lexer::OTHER:
    throw new std::logic_error("Parser: Literal 'other' not supported.");
  case Lexer::AT:
    throw new std::logic_error("Parser: Literal '@' not supported.");
  default:
    lex.putback(tok);
    return p_expr_arith(lex);
  }
}

Parser::stmt_t Parser::p_stmt_list(Lexer &lex,const Context &ctx){
  Lexer::Token tok;
  Lexer::TokenPos pos0;

  lex >> tok;
  lex.putback(tok);
  pos0 = tok.pos;

  std::vector<stmt_t::labeled_stmt_t> seq;

  seq.push_back(p_lstmt(lex,ctx));
  
  lex >> tok;
  while(tok.type == Lexer::SEMICOLON){
    seq.push_back(p_lstmt(lex,ctx));
    lex >> tok;
  }
  lex.putback(tok);

  return stmt_t::sequence(seq,pos0);
}

Parser::stmt_t::labeled_stmt_t Parser::p_lstmt(Lexer &lex,const Context &ctx){
  Lexer::Token tok;
  lex >> tok;

  Lang::label_t lbl;
  if(tok.type == Lexer::ID){
    force(lex,Lexer::COLON,"Expected ':' after label at ");
    lbl = tok.value;
  }else{
    lex.putback(tok);
    lbl = "";
  }
  return stmt_t::labeled_stmt_t(lbl,p_stmt(lex,ctx));
}

Parser::stmt_t Parser::resolve_pointer(const memloc_or_pointer_t &ml,
                                       const std::function<stmt_t(const memloc_t&)> &f,
                                       const Context &ctx){
  if(ml.is_pointer){
    const expr_t &e = ml.pointer;
    if(e.is_integer()){
      if(e.get_integer() >= 0 && e.get_integer() < int(ctx.global_vars.size())){
        return f(memloc_t::global(ctx.global_vars[e.get_integer()].name));
      }else{
        throw new SyntaxError("Invalid pointer value at "+ml.pos.to_long_string()+".",ml.pos);
      }
    }else{
      std::function<int(const std::string&)> regids = 
        [&ctx,&ml](const std::string &s)->int{
        for(unsigned i = 0; i < ctx.regs.size(); ++i){
          if(ctx.regs[i].name == s){
            return int(i);
          }
        }
        throw new SyntaxError("Undeclared register '"+s+"' in pointer at "+ml.pos.to_long_string()+".",ml.pos);
      };
      Lang::Expr<int> e_i = e.convert(regids);
      ZStar<int>::Vector regstars(ctx.regs.size());
      
      /* Check if e contains unbounded registers */
      bool e_has_unbounded_reg = false;
      std::set<int> regs = e_i.get_registers();
      for(auto it = regs.begin(); !e_has_unbounded_reg && it != regs.end(); ++it){
        if(ctx.regs[*it].domain.is_int()){
          e_has_unbounded_reg = true;
        }
      }
      
      std::vector<stmt_t> v;
      for(unsigned i = 0; i < ctx.global_vars.size(); ++i){
        /* Check whether this variable can be pointed to by e */
        if(e_has_unbounded_reg || regstars.possible_values(e_i,ctx.regs).count(i) > 0){
          stmt_t s = f(memloc_t::global(ctx.global_vars[i].name));
          std::vector<stmt_t::labeled_stmt_t> seq;
          seq.push_back(stmt_t::assume(bexpr_t::eq(e,expr_t::integer(i)),s.get_pos()));
          seq.push_back(s);
          v.push_back(stmt_t::sequence(seq,s.get_pos()));
        }
      }
      if(v.size() == 0){
        throw new SyntaxError("Invalid pointer value at "+ml.pos.to_long_string()+". (Cannot point to any memory location.)",ml.pos);
      }else{
        return stmt_t::either(v,v[0].get_pos());
      }
    }
  }else{
    return f(ml.memloc);
  }
};

Parser::stmt_t Parser::p_stmt(Lexer &lex,const Context &ctx) 
  throw(SyntaxError*,Lang::Exception*,Lexer::BadToken*){
  Lexer::Token tok,tok1,tok2;
  lex >> tok;

  switch(tok.type){
  case Lexer::NOP:
    return stmt_t::nop(tok.pos);
  case Lexer::READ: 
    {
      force(lex,Lexer::COLON);
      lex >> tok1;
      if(tok1.type == Lexer::REG){
        force(lex,Lexer::ASSIGNMENT);
        return resolve_pointer(p_memloc(lex,ctx),[&tok1,&tok](const memloc_t &ml){ 
            return stmt_t::read_assign(tok1.value,ml,tok.pos);
          },ctx);
      }else{
        lex.putback(tok1);
        Parser::memloc_or_pointer_t ml = p_memloc(lex,ctx);
        force(lex,Lexer::EQ);
        expr_t e = p_expr(lex);
        return resolve_pointer(ml,[&e,&tok](const memloc_t &ml){
            return stmt_t::read_assert(ml,e,tok.pos);
          },ctx);
      }
    }
  case Lexer::WRITE:
    {
      force(lex,Lexer::COLON);
      Parser::memloc_or_pointer_t ml = p_memloc(lex,ctx);
      force(lex,Lexer::ASSIGNMENT);
      expr_t e = p_expr(lex);
      return resolve_pointer(ml,[&e,&tok](const memloc_t &ml){
          return stmt_t::write(ml,e,tok.pos);
        },ctx);
    }
  case Lexer::CAS: 
    {
      force(lex,Lexer::LPAREN);
      Parser::memloc_or_pointer_t ml = p_memloc(lex,ctx);
      force(lex,Lexer::COMMA);
      expr_t v0(p_expr(lex));
      force(lex,Lexer::COMMA);
      expr_t v1(p_expr(lex));
      force(lex,Lexer::RPAREN);
      return resolve_pointer(ml,[&v0,&v1,&tok](const memloc_t &ml){
          return stmt_t::cas(ml,v0,v1,tok.pos);
        },ctx);
    }
  case Lexer::REG: 
    {
      force(lex,Lexer::ASSIGNMENT);
      return stmt_t::assignment(tok.value,p_expr(lex),tok.pos);
    }
  case Lexer::IF: 
    {
      bexpr_t b(p_bexpr(lex));

      force(lex,Lexer::THEN);
      stmt_t::labeled_stmt_t s0 = p_lstmt(lex,ctx);
      lex >> tok1;
      if(tok1.type == Lexer::ELSE){
        stmt_t::labeled_stmt_t s1 = p_lstmt(lex,ctx);
        return stmt_t::if_stmt(b,s0,s1,tok.pos);
      }else{
        lex.putback(tok1);
        return stmt_t::if_stmt(b,s0,tok.pos);
      }
    }
  case Lexer::WHILE:
    {
      bexpr_t b(p_bexpr(lex));
      force(lex,Lexer::DO);
      return stmt_t::while_stmt(b,p_lstmt(lex,ctx),tok.pos);
    }
  case Lexer::GOTO:
    {
      return stmt_t::goto_stmt(p_label(lex),tok.pos);
    }
  case Lexer::EITHER:
    {
      Lexer::TokenPos pos0 = tok.pos;
      std::vector<stmt_t> seq;
      Lexer::Token lctok;
      Lexer::Token tok;
      lex >> lctok;
      lex.putback(lctok);
      force(lex,Lexer::LCURL);

      seq.push_back(p_stmt_list(lex,ctx));
      lex >> tok;
      while(tok.type == Lexer::EITHER_OR){
        seq.push_back(p_stmt_list(lex,ctx));
        lex >> tok;
      }
      lex.putback(tok);
      force(lex,Lexer::RCURL,"Expected '}' at "," to match '{' at "+
            lctok.pos.to_long_string()+".");

      return stmt_t::either(seq,pos0);
    }
  case Lexer::LOCKED:
    {
      Lexer::TokenPos pos0 = tok.pos;
      std::vector<stmt_t> seq;
      Lexer::Token lctok;
      Lexer::Token tok;
      lex >> lctok;
      if(lctok.type == Lexer::WRITE){
        force(lex,Lexer::COLON);
        Parser::memloc_or_pointer_t ml = p_memloc(lex,ctx);
        force(lex,Lexer::ASSIGNMENT);
        expr_t e = p_expr(lex);
        return resolve_pointer(ml,[&e,&pos0](const memloc_t &ml){
            return stmt_t::locked_write(ml,e,pos0);
          },ctx);
      }else{
        lex.putback(lctok);
        force(lex,Lexer::LCURL);

        stmt_t s = p_stmt_list(lex,ctx);
        std::string cmt;
        if(!stmt_t::check_locked_invariant(s,&cmt)){
          throw new SyntaxError("Error in locked block at "+lctok.pos.to_long_string()+": "+cmt,lctok.pos);
        }
        seq.push_back(s);
        lex >> tok;
        while(tok.type == Lexer::EITHER_OR){
          s = p_stmt_list(lex,ctx);
          if(!stmt_t::check_locked_invariant(s,&cmt)){
            throw new SyntaxError("Error in locked block at "+tok.pos.to_long_string()+": "+cmt,tok.pos);
          }
          seq.push_back(s);
          lex >> tok;
        }
        lex.putback(tok);
        force(lex,Lexer::RCURL,"Expected '}' at "," to match '{' at "+
              lctok.pos.to_long_string()+".");

        return stmt_t::locked_block(seq,pos0);
      }
    }
 case Lexer::SLOCKED:
    {
      Lexer::TokenPos pos0 = tok.pos;
      force(lex,Lexer::WRITE);
      force(lex,Lexer::COLON);
      Parser::memloc_or_pointer_t ml = p_memloc(lex,ctx);
      force(lex,Lexer::ASSIGNMENT);
      expr_t e = p_expr(lex);
      return resolve_pointer(ml,[&e,&pos0](const memloc_t &ml){
          return stmt_t::slocked_write(ml,e,pos0);
        },ctx);
    }
  case Lexer::LCURL:
    {
      stmt_t sl = p_stmt_list(lex,ctx);
      force(lex,Lexer::RCURL,"Expected '}' at "," to match '{' at "+
            tok.pos.to_long_string()+".");
      return sl;
    }
  case Lexer::ASSUME:
    {
      force(lex,Lexer::COLON);
      return stmt_t::assume(p_bexpr(lex),tok.pos);
    }
  default:
    throw new SyntaxError("Expected statement at "+tok.pos.to_long_string()+".",tok.pos);
  }
}

Parser::bexpr_t Parser::p_bexpr(Lexer &lex) 
  throw(SyntaxError*,Lang::Exception*,Lexer::BadToken*){
  return p_bexpr_r(lex,p_bexpr_and(lex)); 
}

Parser::bexpr_t Parser::p_bexpr_r(Lexer &lex,const bexpr_t &left){
  Lexer::Token tok;
  lex >> tok;

  switch(tok.type){
  case Lexer::OR:
    {
      return p_bexpr_r(lex,bexpr_t::disj(left, p_bexpr_and(lex)));
    }
  default:
    lex.putback(tok);
    return left;
  }
}

Parser::bexpr_t Parser::p_bexpr_and(Lexer &lex){
  return p_bexpr_and_r(lex,p_bexpr_and_l(lex));
}

Parser::bexpr_t Parser::p_bexpr_and_l(Lexer &lex){
  Lexer::Token tok;

  lex >> tok;

  switch(tok.type){
  case Lexer::NOT:
    {
      return !p_bexpr_atom(lex);
    }
  default:
    lex.putback(tok);
    return p_bexpr_atom(lex);
  }
}

Parser::bexpr_t Parser::p_bexpr_and_r(Lexer &lex,const bexpr_t &left){
  Lexer::Token tok;
  lex >> tok;

  switch(tok.type){
  case Lexer::AND:
    {
      return p_bexpr_and_r(lex,left && p_bexpr_and_l(lex));
    }
  default:
    lex.putback(tok);
    return left;
  }
}

Parser::bexpr_t Parser::p_bexpr_atom(Lexer &lex){
  Lexer::Token tok;

  lex >> tok;

  switch(tok.type){
  case Lexer::TRUE:
    return bexpr_t::tt();
  case Lexer::FALSE:
    return bexpr_t::ff();
  case Lexer::LBRAK:
    {
      bexpr_t b(p_bexpr(lex));
      force(lex,Lexer::RBRAK,"Expected ']' at "," to match '[' at "+
            tok.pos.to_long_string()+".");
      return b;
    }
  default:
    {
      lex.putback(tok);
      return p_bexpr_atom_r(lex,p_expr(lex));
    }
  }
}

Parser::bexpr_t Parser::p_bexpr_atom_r(Lexer &lex,const expr_t &left){
  Lexer::Token tok;

  lex >> tok;

  switch(tok.type){
  case Lexer::EQ:
    {
      return bexpr_t::eq(left,p_expr(lex));
    }
  case Lexer::NEQ:
    {
      return bexpr_t::neq(left, p_expr(lex));
    }
  case Lexer::LT:
    {
      return bexpr_t::lt(left, p_expr(lex));
    }
  case Lexer::GT:
    {
      return bexpr_t::gt(left, p_expr(lex));
    }
  case Lexer::LEQ:
    {
      return bexpr_t::leq(left, p_expr(lex));
    }
  case Lexer::GEQ:
    {
      return bexpr_t::geq(left, p_expr(lex));
    }
  default:
    throw new SyntaxError("Expected comparison operator (=,!=,<,>,<=,>=) at "+
                          tok.pos.to_long_string()+
                          ".",tok.pos);
  }
}

Parser::memloc_or_pointer_t Parser::p_memloc(Lexer &lex, const Context &ctx) 
  throw(SyntaxError*,Lang::Exception*,Lexer::BadToken*){
  Lexer::Token tok0, tok1, tok2, tok3;
  lex >> tok0;

  switch(tok0.type){
  case Lexer::ID:
    {
      lex >> tok1;
      if(tok1.type == Lexer::LBRAK){
        lex >> tok2;
        if(tok2.type == Lexer::MY){
          lex >> tok3;
          if(tok3.type == Lexer::RBRAK){
            return memloc_or_pointer_t(Lang::MemLoc<std::string>::local(tok0.value),tok0.pos);
          }else{
            throw new SyntaxError("Expected ']' at "+tok3.pos.to_long_string()+
                                  " to match '[' at "+tok1.pos.to_long_string()+".",tok3.pos);
          }
        }else if(tok2.type == Lexer::NAT){
          lex >> tok3;
          if(tok3.type == Lexer::RBRAK){
            int i;
            std::stringstream ss;
            ss << tok2.value;
            ss >> i;
            return memloc_or_pointer_t(Lang::MemLoc<std::string>::local(tok0.value,i),tok0.pos);
          }else{
            throw new SyntaxError("Expected ']' at "+tok3.pos.to_long_string()+
                                  " to match '[' at "+tok1.pos.to_long_string()+".",tok3.pos);
          }
        }else{
          throw new SyntaxError("Expected process identifier ('my' or numeric) at "+
                                tok2.pos.to_long_string()+".",tok2.pos);
        }
      }else{
        lex.putback(tok1);
        return memloc_or_pointer_t(Lang::MemLoc<std::string>::global(tok0.value),tok0.pos);
      }
    }
  case Lexer::LBRAK:
    {
      lex >> tok1;
      lex.putback(tok1);
      expr_t e = p_expr(lex);
      force(lex,Lexer::RBRAK);
      return memloc_or_pointer_t(e,tok0.pos);
    }
  default:
    throw new SyntaxError("Expected memory location at "+tok0.pos.to_long_string()+".",tok0.pos);
  }
}

Lang::label_t Parser::p_label(Lexer &lex){
  Lexer::Token tok;
  lex >> tok;

  if(tok.type == Lexer::ID)
    return tok.value;

  throw new SyntaxError("Expected label at "+tok.pos.to_long_string()+".",tok.pos);
}

std::pair<Parser::Proc,int> Parser::p_proc(Lexer &lex, const Context &ctx){
  force(lex,Lexer::PROCESS);
  Lexer::Token tok0,tok1,tok2;
  lex >> tok0;

  int proc_count;
  if(tok0.type == Lexer::LPAREN){
    lex >> tok1;
    if(tok1.type == Lexer::NAT){
      force(lex,Lexer::RPAREN);
      std::stringstream(tok1.value) >> proc_count;
    }else
      throw new SyntaxError("Expected natural number at "+tok1.pos.to_long_string()+".",tok1.pos);
  }else{
    lex.putback(tok0);
    proc_count = 1;
  }

  std::vector<Lang::VarDecl> vi;
  std::vector<Lang::VarDecl> ri;
  bool data_declared = false;
  bool registers_declared = false;

  lex >> tok0;
  while(tok0.type == Lexer::DATA || tok0.type == Lexer::REGISTERS){
    if(tok0.type == Lexer::DATA){
      if(data_declared){
        throw new SyntaxError("Duplicate data declaration at "+tok0.pos.to_long_string()+".",tok0.pos);
      }
      vi = p_var_decl_list(lex,DCL_ML);
    }else{
      if(registers_declared){
        throw new SyntaxError("Duplicate register declaration at "+tok0.pos.to_long_string()+".",tok0.pos);
      }
      ri = p_var_decl_list(lex,DCL_REG);
    }
    lex >> tok0;
  }
  lex.putback(tok0);

  Context pctx(ctx.global_vars,ri);

  force(lex,Lexer::TEXT);

  return std::pair<Proc,int>(Proc(vi,ri,p_stmt_list(lex,pctx)),proc_count);
}

std::vector<Parser::Proc> Parser::p_proc_list(Lexer &lex, const Context &ctx){

  std::vector<Proc> vec;

  std::pair<Proc,int> p = p_proc(lex,ctx);
  for(int i = 0; i < p.second; i++)
    vec.push_back(p.first);
  
  Lexer::Token tok;
  lex >> tok;
  while(tok.type == Lexer::PROCESS){
    lex.putback(tok);
    std::pair<Proc,int> p = p_proc(lex,ctx);
    for(int i = 0; i < p.second; i++)
      vec.push_back(p.first);
    lex >> tok;
  }
  lex.putback(tok);
  
  return vec;
}

Lang::VarDecl Parser::p_var_decl(Lexer &lex, declaration_type dcl_type){
  Lexer::Token tok0,tok1,tok2;

  lex >> tok0;
  
  if(dcl_type == DCL_ML){
    if(tok0.type != Lexer::ID)
      throw new SyntaxError("Expected variable initialization at "+tok0.pos.to_long_string()+".",tok0.pos);
  }else{
    if(tok0.type != Lexer::REG)
      throw new SyntaxError("Expected register initialization at "+tok0.pos.to_long_string()+".",tok0.pos);
  }
  
  force(lex,Lexer::EQ);
  lex >> tok1;

  Lang::Value val;

  switch(tok1.type){
  case Lexer::NAT:
    {
      int i;
      std::stringstream ss;
      ss << tok1.value;
      ss >> i;
      val = Lang::Value(i);
      // return std::pair<std::string,Lang::Value>(tok0.value,Lang::Value(i));
      break;
    }
  case Lexer::MINUS:
    {
      lex >> tok2;
      if(tok2.type != Lexer::NAT)
        throw new SyntaxError("Expected value for variable at "+tok1.pos.to_long_string()+".",tok1.pos);
      int i;
      std::stringstream ss;
      ss << tok2.value;
      ss >> i;
      val = Lang::Value(-i);
      // return std::pair<std::string,Lang::Value>(tok0.value,Lang::Value(-i));
      break;
    }
  case Lexer::STAR:
    val = Lang::Value();
    // return std::pair<std::string,Lang::Value>(tok0.value,Lang::Value());
    break;
  default:
    throw new SyntaxError("Expected value for variable at "+tok1.pos.to_long_string()+".",tok1.pos);
  }

  Lang::VarDecl::Domain dom;

  lex >> tok1;
  if(tok1.type == Lexer::COLON){
    lex >> tok1;
    switch(tok1.type){
    case Lexer::LBRAK:
      {
        int lb = p_int(lex);
        force(lex,Lexer::COLON);
        int ub = p_int(lex);
        force(lex,Lexer::RBRAK);
        if(ub < lb){
          throw new SyntaxError("In data domain specification at "+tok1.pos.to_long_string()+": Empty domain.",tok1.pos);
        }
        dom = Lang::VarDecl::Domain(lb,ub);
        break;
      }
    case Lexer::ID:
      if(tok1.value == "Z"){
        dom = Lang::VarDecl::Domain();
      }else{
        throw new SyntaxError("Expected data domain specification at "+tok1.pos.to_long_string()+".",tok1.pos);
      }
      break;
    default:
      throw new SyntaxError("Expected data domain specification at "+tok1.pos.to_long_string()+".",tok1.pos);
    }
  }else{
    dom = Lang::VarDecl::Domain();
    lex.putback(tok1);
  }

  if(!val.is_wild() && !dom.member(val.get_value())){
    throw new SyntaxError("At declaration of variable "+tok0.value+" at "+tok0.pos.to_long_string()+": Initial value not in domain.",
                          tok0.pos);
  }

  return Lang::VarDecl(tok0.value,val,dom);
}

int Parser::p_int(Lexer &lex){
  Lexer::Token tok;
  std::stringstream ss;
  int coeff = 1;
  lex >> tok;
  if(tok.type == Lexer::MINUS){
    coeff = -1;
    lex >> tok;
  }

  if(tok.type == Lexer::NAT){
    int i;
    ss << tok.value;
    ss >> i;
    return i*coeff;
  }else{
    throw new SyntaxError("Expected integer literal at "+tok.pos.to_long_string()+".",tok.pos);
  }
}

std::vector<Lang::VarDecl> Parser::p_var_decl_list(Lexer &lex, declaration_type dcl_type){
  Lexer::Token tok;
  std::vector<Lang::VarDecl> vec;

  lex >> tok;

  if(dcl_type == DCL_ML){
    if(tok.type != Lexer::ID)
      throw new SyntaxError("Expected variable initialization at "+tok.pos.to_long_string()+".",tok.pos);
  }else{
    if(tok.type != Lexer::REG)
      throw new SyntaxError("Expected register initialization at "+tok.pos.to_long_string()+".",tok.pos);
  }

  lex.putback(tok);
  vec.push_back(p_var_decl(lex,dcl_type));

  lex >> tok;
  lex.putback(tok);
  while((dcl_type == DCL_ML && tok.type == Lexer::ID) || 
        (dcl_type == DCL_REG && tok.type == Lexer::REG)){
    Lang::VarDecl dcl = p_var_decl(lex,dcl_type);
    for(unsigned i = 0; i < vec.size(); i++){
      if(vec[i].name == dcl.name){
        throw new SyntaxError("Duplicate register declaration at "+tok.pos.to_long_string()+".",tok.pos);
      }
    }
    vec.push_back(dcl);
    lex >> tok;
    lex.putback(tok);
  }

  return vec;
}

Parser::forbidden_t Parser::p_forbidden(Lexer &lex){
  forbidden_t fb;

  force(lex,Lexer::FORBIDDEN);

  Lexer::Token tok;
  lex >> tok;
  if(tok.type != Lexer::ID && tok.type != Lexer::STAR){
    throw new SyntaxError("Expected label or '*' at "+tok.pos.to_long_string()+".",tok.pos);
  }

  std::vector<std::pair<bool,Lang::label_t> > tmpv;

  while(tok.type == Lexer::ID || tok.type == Lexer::STAR){

    while(tok.type == Lexer::ID || tok.type == Lexer::STAR){
      if(tok.type == Lexer::ID){
        tmpv.push_back(std::pair<bool,Lang::label_t>(false,tok.value));
      }else{
        tmpv.push_back(std::pair<bool,Lang::label_t>(true,"*"));
      }
      lex >> tok;
    }
    fb.push_back(tmpv);
    tmpv.clear();

    if(tok.type == Lexer::SEMICOLON){
      lex >> tok;
      if(tok.type != Lexer::ID && tok.type != Lexer::STAR){
        throw new SyntaxError("Expected label or '*' at "+tok.pos.to_long_string()+".",tok.pos);
      }
    }

  }
  
  lex.putback(tok);

  return fb;
}

Parser::Test Parser::p_test(Lexer &lex) throw(SyntaxError*,Lang::Exception*,Lexer::BadToken*){
  Test test;

  test.forbidden = p_forbidden(lex);
  
  Lexer::Token tok;
  lex >> tok;

  lex.putback(tok);
  if(tok.type == Lexer::PREDICATES){
    lex >> tok; // Get rid of 'predicates'

    std::map<std::string,int> m;
    std::function<int(const std::string&)> rc = 
      [&m](const std::string &s)->int{
      if(m.count(s) == 0){
        int i = m.size();
        m[s] = i;
      }
      return m[s];
    };

    bexpr_t b = p_bexpr(lex);
    test.predicates.push_back(predicate_t::from_bexpr(b.convert(rc),0).generalise());

    lex >> tok;
    while(tok.type == Lexer::SEMICOLON){
      bexpr_t b = p_bexpr(lex);
      test.predicates.push_back(predicate_t::from_bexpr(b.convert(rc),0).generalise());
      lex >> tok;
    }
    lex.putback(tok);
  }

  lex >> tok;
  lex.putback(tok);
  if(tok.type == Lexer::PROCESS){
    test.processes = p_proc_list(lex,Context());
  }else{
    force(lex,Lexer::DATA);
    test.global_vars = p_var_decl_list(lex,DCL_ML);
    test.processes = p_proc_list(lex,Context(test.global_vars));
  }

  force(lex,Lexer::TOKEOF);
  
  return test;
}
