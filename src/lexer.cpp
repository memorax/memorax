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

#include "lexer.h"
#include <sstream>

Lexer::~Lexer(){
};

Lexer::PosIStream::PosIStream(std::istream &iss) : is(iss) {
  cur_pos.lineno = 1;
  cur_pos.charno = 0;
}

char Lexer::PosIStream::get(){
  char c = is.get();
  if(c == '\n'){
    line_lengths.push_front(cur_pos.charno);
    cur_pos.lineno++;
    cur_pos.charno=0;
  }else if(is.good()){
    cur_pos.charno++;
  }
  return c;
}

Lexer::PosIStream &Lexer::PosIStream::get(char &c){
  c = get();
  return *this;
}

Lexer::PosIStream &Lexer::PosIStream::putback(char c){
  if(c == '\n'){
    cur_pos.lineno--;
    cur_pos.charno = line_lengths.front();
    line_lengths.pop_front();
  }else{
    cur_pos.charno--;
  }
  is.putback(c);
  return *this;
}

bool Lexer::PosIStream::good() const{ return is.good();}
bool Lexer::PosIStream::eof() const{ return is.eof();}
bool Lexer::PosIStream::bad() const{ return is.bad();}
Lexer::TokenPos Lexer::PosIStream::pos() const{ return cur_pos;}

Lexer::Lexer(std::istream &i) : is(i), eof(false) {
  reserved_names["if"] = IF;
  reserved_names["then"] = THEN;
  reserved_names["else"] = ELSE;
  reserved_names["while"] = WHILE;
  reserved_names["do"] = DO;
  reserved_names["read"] = READ;
  reserved_names["write"] = WRITE;
  reserved_names["locked"] = LOCKED;
  reserved_names["slocked"] = SLOCKED;
  reserved_names["cas"] = CAS;
  reserved_names["goto"] = GOTO;
  reserved_names["true"] = TRUE;
  reserved_names["false"] = FALSE;
  reserved_names["my"] = MY;
  reserved_names["me"] = ME;
  reserved_names["other"] = OTHER;
  reserved_names["process"] = PROCESS;
  reserved_names["not"] = NOT;
  reserved_names["nop"] = NOP;
  reserved_names["text"] = TEXT;
  reserved_names["data"] = DATA;
  reserved_names["either"] = EITHER;
  reserved_names["or"] = EITHER_OR;
  reserved_names["forbidden"] = FORBIDDEN;
  reserved_names["assume"] = ASSUME;
  reserved_names["predicates"] = PREDICATES;
  reserved_names["registers"] = REGISTERS;

  operators["="]  = EQ;
  operators["("]  = LPAREN;
  operators[")"]  = RPAREN;
  operators[","]  = COMMA;
  operators["-"]  = MINUS;
  operators["*"]  = STAR;
  operators[";"]  = SEMICOLON;
  operators[":"]  = COLON;
  operators[":="] = ASSIGNMENT;
  operators["{"]  = LCURL;
  operators["}"]  = RCURL;
  operators["||"] = OR;
  operators["&&"] = AND;
  operators["!="] = NEQ;
  operators["<"]  = LT;
  operators[">"]  = GT;
  operators["<="]  = LEQ;
  operators[">="]  = GEQ;
  operators["+"]  = PLUS;
  operators["["]  = LBRAK;
  operators["]"]  = RBRAK;
  operators["@"]  = AT;

  op_max_len = 0;
  for(std::map<std::string,TokenType>::iterator it = operators.begin();
      it != operators.end(); it++){
    if((*it).first.length() > op_max_len)
      op_max_len = (*it).first.length();
  }
}

Lexer::operator bool() const{
  return !eof;
}

std::ostream& operator<<(std::ostream& os, const Lexer::Token& tok){
  os << tok.to_string();
  return os;
}

Lexer::BadToken::BadToken(std::string s, TokenPos p) : value(s), pos(p) {
  msg = "Bad token "+(value != "" ? "'"+value+"' " : "")+"at "+pos.to_long_string();
}

std::string Lexer::BadToken::to_string() const{
  return msg;
}

std::ostream& operator<<(std::ostream& os, const Lexer::BadToken& btok){
  os << btok.to_string();
  return os;
}

Lexer& Lexer::putback(Token &tok){
  if(tok.type == TOKEOF) eof = false;
  prefix.push_front(tok);
  return *this;
}

std::string Lexer::token_type_to_string(TokenType typ){
  switch(typ){
  case UNDEF: return "UNDEF";
  case TOKEOF: return "TOKEOF";
  case ID: return "ID";
  case REG: return "REG";
  case IF: return "if";
  case THEN: return "then";
  case ELSE: return "else";
  case WHILE: return "while";
  case DO: return "do";
  case READ: return "read";
  case WRITE: return "write";
  case LOCKED: return "locked";
  case SLOCKED: return "slocked";
  case CAS: return "cas";
  case GOTO: return "goto";
  case TRUE: return "true";
  case FALSE: return "false";
  case MY: return "my";
  case ME: return "me";
  case OTHER: return "other";
  case PROCESS: return "process";
  case NOT: return "not";
  case NAT: return "NAT";
  case EQ: return "=";
  case LPAREN: return "(";
  case RPAREN: return ")";
  case COMMA: return ",";
  case MINUS: return "-";
  case STAR: return "*";
  case SEMICOLON: return ";";
  case COLON: return ":";
  case ASSIGNMENT: return ":=";
  case LCURL: return "{";
  case RCURL: return "}";
  case OR: return "||";
  case AND: return "&&";
  case NEQ: return "!=";
  case LT: return "<";
  case GT: return ">";
  case LEQ: return "<=";
  case GEQ: return ">=";
  case PLUS: return "+";
  case LBRAK: return "[";
  case RBRAK: return "]";
  case AT: return "@";
  case NOP: return "nop";
  case TEXT: return "text";
  case DATA: return "data";
  case EITHER: return "either";
  case EITHER_OR: return "or";
  case FORBIDDEN: return "forbidden";
  case ASSUME: return "assume";
  case PREDICATES: return "predicates";
  case REGISTERS: return "registers";
  default: return "(unknown type)";
  }
}

std::string Lexer::Token::to_string() const{
  switch(type){
  case ID: 
  case REG: 
  case NAT: 
    return token_type_to_string(type)+"("+value+") @"+pos.to_short_string();
  default: return token_type_to_string(type)+" @"+pos.to_short_string();
  }
}

Lexer& Lexer::operator>>(Token &tok) throw(BadToken*) {
  if(!prefix.empty()){
    tok = prefix.front();
    prefix.pop_front();
    return *this;
  }

  char c = ' ';
  while(isspace(c) && is.good())
    is.get(c);

  if(is.eof()){
    tok.type = TOKEOF;
    tok.value = "";
    tok.pos = is.pos();
    eof = true;
    return *this;
  }

  /* Skip comments */
  if(c == '/'){
    is.get(c);
    if(c == '*'){
      while(true){
        is.get(c);
        if(is.eof()){
          tok.type = TOKEOF;
          tok.value = "";
          tok.pos = is.pos();
          eof = true;
          return *this;
        }
        while(c == '*'){
          is.get(c);
          if(is.eof()){
            tok.type = TOKEOF;
            tok.value = "";
            tok.pos = is.pos();
            eof = true;
            return *this;
          }
          if(c == '/'){
            return *this >> tok;
          }
        }
      }
    }else{
      is.putback(c);
      c = '/';
    }
  }

  is.putback(c);
  TokenPos p = is.pos();

  if(isalpha(c) || c == '_') tok = read_name();
  else if(isdigit(c)) tok = read_nat();
  else if(c == '$') tok = read_reg();
  else tok = read_op();

  tok.pos = p;

  return *this;
}

Lexer::Token Lexer::read_name(){
  std::string s = "";
  TokenPos p = is.pos();
  char c = is.get();
  
  while((isalnum(c) || c == '_') && is.good()){
    s += c;
    c = is.get();
  }
  
  if(is.good() && s.length() > 0)
    is.putback(c);
  
  if(s.length() == 0){
    if(is.good()){
      throw new BadToken(std::string(1,c),p);
    }else{
      throw new BadToken("(TOKEOF)",p);
    }
  }

  Token tok;
  tok.value = s;
  if(reserved_names.count(s))
    tok.type = reserved_names[s];
  else
    tok.type = ID;

  return tok;
}

Lexer::Token Lexer::read_reg(){
  TokenPos p = is.pos();
  char c = is.get();
  
  if(is.good() && c == '$'){
    Token tok = read_name();
    tok.type = REG;
    tok.value = '$'+tok.value;
    return tok;
  }
  throw new BadToken(std::string(1,c),p);
}

Lexer::Token Lexer::read_nat(){
  std::string s = "";
  TokenPos p = is.pos();
  char c = is.get();

  while(isdigit(c) && is.good()){
    s += c;
    c = is.get();
  }

  if(is.good()) is.putback(c);

  if(s.length() == 0){
    throw new BadToken("",p);
  }

  Token tok(NAT,s);

  return tok;
}

Lexer::Token Lexer::read_op(){
  std::string s = "";
  TokenPos p = is.pos();
  char c = '='; // '=' used as dummy operator character

  int match_len = -1;  
  while(s.length() < op_max_len && is.good() && !isalnum(c) && !isspace(c) && c != '_'){
    c = is.get();
    if(is.good())
      s += c;
    if(operators.count(s))
      match_len = s.length();
  }

  for(int i = s.length()-1; i >= 0 && i >= match_len; i--)
    is.putback(s[i]);

  if(match_len == -1)
    throw new BadToken(s,p);

  s = s.substr(0,match_len);
  Token tok;
  tok.value = s;
  tok.type = operators[s];
  return tok;
}

std::string Lexer::TokenPos::LineChar::to_short_string() const{
  std::stringstream ss;

  if(lineno >= 0 && charno >= 0){
    ss << "(" << lineno << "," << charno << ")";
  }else{
    ss << "(?,?)";
  }


  return ss.str();
}

std::string Lexer::TokenPos::LineChar::to_long_string() const{
  std::stringstream ss;

  if(lineno >= 0 && charno >= 0){
    ss << "line " << lineno << ", character " << charno;
  }else{
    ss << "line ?, character ?";
  }

  return ss.str();
}

int Lexer::TokenPos::LineChar::compare(const LineChar &lc) const{
  if(lineno < lc.lineno ||
     (lineno == lc.lineno && charno < lc.charno)){
    return -1;
  }else if(lineno == lc.lineno && charno == lc.charno){
    return 0;
  }else{
    return 1;
  }
}

std::string Lexer::TokenPos::to_short_string() const{
  if(pos.size() == 0){
    return "(?,?)";
  }else if(pos.size() == 1){
    return pos[0].to_short_string();
  }else{
    std::string s = "[";
    for(unsigned i = 0; i < pos.size(); ++i){
      if(i != 0) s += ",";
      s += pos[i].to_short_string();
    }
    s += "]";
    return s;
  }
};

std::string Lexer::TokenPos::to_short_line_string() const{
  if(pos.size() == 0){
    return "L?";
  }else{
    std::stringstream ss;
    ss << "L" << pos[0].lineno;
    for(unsigned i = 1; i < pos.size(); ++i){
      ss << " by L" << pos[i].lineno;
    }
    return ss.str();
  }
};

std::string Lexer::TokenPos::to_long_string() const{
  if(pos.size() == 0){
    return "line ?, character ?";
  }else{
    std::string s = pos[0].to_long_string();
    for(unsigned i = 1; i < pos.size(); ++i){
      s += ", called from "+pos[i].to_long_string();
    }
    return s;
  }
};

std::string Lexer::TokenPos::to_json() const{
  std::stringstream ss;
  ss << "[";
  for(unsigned i = 0; i < pos.size(); ++i){
    if(i != 0) ss << ",";
    ss << "{\"lineno\":" << pos[i].lineno << ", \"charno\":" << pos[i].charno << "}";
  }
  ss << "]";
  return ss.str();
};

int Lexer::TokenPos::compare(const TokenPos &tp) const{
  if(pos < tp.pos){
    return -1;
  }else if(pos == tp.pos){
    return 0;
  }else{
    return 1;
  }
};
