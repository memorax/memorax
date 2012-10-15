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

#ifndef __PREPROCESSOR_H__
#define __PREPROCESSOR_H__

#include "lexer.h"
#include <vector>

class PPLexer : public Lexer{
public:
  PPLexer(std::istream&);
  virtual ~PPLexer();
  virtual PPLexer &operator>>(Token&) throw(BadToken*);
  virtual PPLexer &putback(Token&);
  virtual operator bool() const;
private:
  struct macro_t{
    std::string name;
    std::vector<std::string> parameters;
    std::vector<Token> tokens;
    std::string to_string() const;
  };
  std::map<std::string,macro_t> macros;
  void read_macro();
  void call_macro();
  Token get_next();
  std::vector<Token> prefix;
  struct macro_frame_t{
    macro_frame_t(const macro_t &m, const Lexer::TokenPos::LineChar &cf)
      : macro(m), next_tok(0), called_from(cf) {};
    const macro_t &macro;
    std::map<std::string,std::vector<Token> > args;
    unsigned next_tok;
    Lexer::TokenPos::LineChar called_from;
  };
  std::vector<macro_frame_t> macro_stack;
  static const std::string macro_start;
  static const std::string macro_end;
};

#endif
