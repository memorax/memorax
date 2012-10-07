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

#include "preprocessor.h"
#include <cassert>
#include <set>

PPLexer::PPLexer(std::istream &is) : Lexer(is){
}

PPLexer::~PPLexer(){
}

PPLexer &PPLexer::operator>>(Token &tok) throw(BadToken*){
  tok = get_next();
  /* Consume macro definitions and calls */
  while(tok.type == ID && 
        (tok.value == macro_start || macros.count(tok.value))){
    if(tok.value == macro_start){
      // Define macro
      putback(tok);
      read_macro();
      tok = get_next();
    }else{
      // Call macro
      putback(tok);
      call_macro();
      assert(macro_stack.size());
      tok = get_next();
    }
  }
  return *this;
};

Lexer::Token PPLexer::get_next(){
  Token tok;
  if(prefix.size()){
    tok = prefix.back();
    prefix.pop_back();
  }else{
    /* Remove finished macro frames */
    while(macro_stack.size() && macro_stack.back().next_tok >= macro_stack.back().macro.tokens.size()){
      macro_stack.pop_back();
    }
    if(macro_stack.size()){
      tok = macro_stack.back().macro.tokens[macro_stack.back().next_tok];
      ++macro_stack.back().next_tok;
      if(tok.type == ID && macro_stack.back().args.count(tok.value)){
        /* Parameter */
        const std::vector<Token> &arg = macro_stack.back().args[tok.value];
        assert(arg.size());
        for(int i = int(arg.size())-1; i >= 0; --i){
          prefix.push_back(arg[i]);
        }
        tok = prefix.back();
        prefix.pop_back();
      }
    }else{
      this->Lexer::operator>>(tok);
    }
  }
  return tok;
}

PPLexer &PPLexer::putback(Token &tok){
  prefix.push_back(tok);
  return *this;
};

PPLexer::operator bool() const{
  return prefix.size() || this->Lexer::operator bool();
};

const std::string PPLexer::macro_start = "macro";
const std::string PPLexer::macro_end = "endmacro";

void PPLexer::read_macro(){
  macro_t macro;
  Token tok;
  tok = get_next();
  assert(tok.type == ID && tok.value == macro_start);

  /* Read macro name */
  tok = get_next();
  if(tok.type != ID){
    throw new BadToken("Expected macro name.",tok.pos);
  }
  if(macros.count(tok.value) > 0){
    throw new BadToken("Macro \""+tok.value+"\" defined twice.",tok.pos);
  }
  macro.name = tok.value;
  
  /* Read macro parameters */
  tok = get_next();
  if(tok.type != LPAREN) throw new BadToken("Expected '('.",tok.pos);
  {
    std::set<std::string> params;
    tok = get_next();
    bool cont = tok.type != RPAREN;
    while(cont){
      if(tok.type != ID) throw new BadToken("Expected parameter name.",tok.pos);
      if(params.count(tok.value)) throw new BadToken("Duplicate parameter name \""+tok.value+"\".",tok.pos);
      params.insert(tok.value);
      macro.parameters.push_back(tok.value);
      tok = get_next();
      if(tok.type == COMMA){
        tok = get_next();
        cont = true;
      }else if(tok.type == RPAREN){
        cont = false;
      }else{
        throw new BadToken("Expected ')'.",tok.pos);
      }
    }    
  }

  /* Read macro content */
  tok = get_next();
  while(tok.type != TOKEOF &&
        !(tok.type == ID && tok.value == macro_end)){
    if(tok.type == ID && tok.value == macro_start){
      throw new BadToken("Macro definition inside macro.",tok.pos);
    }
    macro.tokens.push_back(tok);
    tok = get_next();
  }
  if(tok.type == TOKEOF){
    throw new BadToken("EOF before end of definition of macro \""+macro.name+"\".",tok.pos);
  }
  macros[macro.name] = macro;
};

void PPLexer::call_macro(){
  Token tok = get_next();
  assert(tok.type == ID && macros.count(tok.value));
  TokenPos call_pos = tok.pos;
  const macro_t &macro = macros[tok.value];
  /* Check if we are looping */
  for(unsigned i = 0; i < macro_stack.size(); ++i){
    if(macro_stack[i].macro.name == macro.name){
      throw new BadToken("Cyclic call to macro \""+macro.name+"\".",tok.pos);
    }
  }

  macro_frame_t frame(macro);
  /* Read arguments */
  tok = get_next();
  if(tok.type != LPAREN) throw new BadToken("Expected '('.",tok.pos);
  {
    int paren_nesting = 1;
    std::vector<Token> cur_arg;
    std::vector<std::vector<Token> > args;
    bool allow_end = true;
    while(paren_nesting > 0){
      tok = get_next();
      if(tok.type == LPAREN){
        allow_end = true;
        ++paren_nesting;
        cur_arg.push_back(tok);
      }else if(tok.type == RPAREN){
        --paren_nesting;
        if(paren_nesting == 0 && cur_arg.size() > 0){
          args.push_back(cur_arg);
        }
        if(paren_nesting > 0){
          cur_arg.push_back(tok);
        }
        if(paren_nesting == 0 && !allow_end){
          throw new BadToken("Empty macro argument.",tok.pos);
        }
        allow_end = true;
      }else if(paren_nesting == 1 && tok.type == COMMA){
        allow_end = false;
        if(cur_arg.empty()) throw new BadToken("Empty macro argument.",tok.pos);
        args.push_back(cur_arg);
        cur_arg.clear();
      }else if(tok.type == TOKEOF){
        throw new BadToken("EOF inside arguments for macro call at "+call_pos.to_long_string()+".",tok.pos);
      }else if(tok.type == ID && tok.value == macro_start){
        throw new BadToken("Macro definition inside arguments for macro call at "+call_pos.to_long_string()+".",tok.pos);
      }else{
        allow_end = true;
        cur_arg.push_back(tok);
      }
    }
    if(args.size() != macro.parameters.size()){
      throw new BadToken("Wrong number of arguments given to macro.",tok.pos);
    }
    for(unsigned i = 0; i < macro.parameters.size(); ++i){
      frame.args[macro.parameters[i]] = args[i];
    }
  }

  macro_stack.push_back(frame);
};
