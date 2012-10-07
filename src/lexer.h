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

#ifndef __LEXER_H__
#define __LEXER_H__

#include <iostream>
#include <list>
#include <map>

class Lexer{
public:
  Lexer(std::istream&);
  virtual ~Lexer();

  enum TokenType {
    UNDEF, 
    TOKEOF, 
    ID,
    REG,
    NAT,
    
    IF, THEN, ELSE, WHILE, DO, READ, WRITE, LOCKED, CAS, GOTO,
    TRUE, FALSE, MY, ME, OTHER, PROCESS, NOT, NOP, TEXT, DATA,
    EITHER, EITHER_OR /* 'or' in an either clause */, 
    ASSUME, FORBIDDEN, PREDICATES, REGISTERS,

    EQ, LPAREN, RPAREN, COMMA, MINUS, STAR, SEMICOLON, COLON,
    ASSIGNMENT, LCURL, RCURL, OR, AND, NEQ, LT, GT, PLUS, 
    LBRAK, RBRAK, AT
  };

  static std::string token_type_to_string(TokenType);

  class TokenPos{
  public:
    TokenPos() : lineno(0), charno(0) {};
    TokenPos(int ln,int cn) : lineno(ln), charno(cn) {};
    int lineno;
    int charno;
    std::string to_short_string() const;
    std::string to_long_string() const;
  };

  class Token{
  public:
    Token() : type(UNDEF), value("") {};
    Token(TokenType t,std::string v) : type(t), value(v) {};
    TokenType type;
    TokenPos pos;
    std::string value;
    std::string to_string() const;
  };

  class BadToken : public std::exception{
    std::string value;
    std::string msg;
    TokenPos pos;
  public:
    BadToken(std::string,TokenPos);
    virtual ~BadToken() throw() {};
    std::string to_string() const;
    const char *what() const throw(){ return msg.c_str(); };
  };

  virtual Lexer& operator>>(Token&) throw(BadToken*);
  virtual Lexer& putback(Token&);
  virtual operator bool() const;
private:
  class PosIStream{
    std::istream &is;
    TokenPos cur_pos;
    std::list<int> line_lengths;
  public:
    PosIStream(std::istream&);
    char get();
    PosIStream &get(char &c);
    PosIStream &putback(char);
    bool good() const;
    bool eof() const;
    bool bad() const;
    TokenPos pos() const;
  };
  PosIStream is;
  bool eof;
  std::list<Token> prefix;
  std::map<std::string,TokenType> reserved_names;
  std::map<std::string,TokenType> operators;
  unsigned int op_max_len;
  Token read_name();
  Token read_nat();
  Token read_op();
  Token read_reg();
};;

std::ostream& operator<<(std::ostream&, const Lexer::Token&);
std::ostream& operator<<(std::ostream&, const Lexer::BadToken&);

#endif // __LEXER_H__
