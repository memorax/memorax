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

#include <string>
#include <cstring>
#include <vector>
#include <set>
#include <map>
#include <sstream>
#include <stdexcept>
#include <iostream>
#include <cassert>
#include <limits>
#include <cstdlib>

template<class Var> const int 
SyntaxString<Var>::min_int(std::numeric_limits<symbol_t>::min());
template<class Var> const int 
SyntaxString<Var>::max_int(std::numeric_limits<symbol_t>::max());

template<class Var> void SyntaxString<Var>::init(int sc, int cc, int ac){
  symbols = new symbol_t[sc];
  symbol_count = sc;
  symbols_ptr_count = new ptr_count_t(1);
  const_count = cc;
  if(cc > 0){
    consts = new Var[cc];
    consts_ptr_count = new ptr_count_t(1);
  }else{
    consts = 0;
    consts_ptr_count = 0;
  }
  arg_count = ac;
}

template<class Var> SyntaxString<Var>::SyntaxString(int sc, int cc, int ac){
  init(sc,cc,ac);
};

template<class Var> SyntaxString<Var>::SyntaxString(bool b){
  symbols = new symbol_t[2];
  symbol_count = 2;
  symbols_ptr_count = new ptr_count_t(1);
  if(b){
    symbols[0] = TRUE;
  }else{
    symbols[0] = FALSE;
  }
  symbols[1] = 0; // Padding. Must be 0 for symbols to be canonical
  const_count = 0;
  consts = 0;
  consts_ptr_count = 0;
  arg_count = 0;

  assert(check_invariant());
};

template<class Var> SyntaxString<Var>::SyntaxString(const SyntaxString &ss){
  init(ss);
  assert(check_invariant());
}

template<class Var> void SyntaxString<Var>::init(const SyntaxString<Var> &ss){
  symbols = ss.symbols;
  symbol_count = ss.symbol_count;
  arg_count = ss.arg_count;
  const_count = ss.const_count;
  symbols_ptr_count = ss.symbols_ptr_count;
  (*symbols_ptr_count)++;
  consts = ss.consts;
  consts_ptr_count = ss.consts_ptr_count;
  if(consts_ptr_count) (*consts_ptr_count)++;
}

template<class Var> void SyntaxString<Var>::self_destruct(){
  (*symbols_ptr_count)--;
  if(*symbols_ptr_count == 0){
    delete symbols_ptr_count;
    delete[] symbols;
  }
  if(consts_ptr_count){
    (*consts_ptr_count)--;
    if(*consts_ptr_count == 0){
      delete consts_ptr_count;
      delete[] consts;
    }
  }
}

template<class Var> SyntaxString<Var>::~SyntaxString(){
  self_destruct();
}

template<class Var> SyntaxString<Var> &SyntaxString<Var>::operator=(const SyntaxString<Var> &ss){
  if(&ss != this){
    self_destruct();
    init(ss);
  }
  assert(check_invariant());
  return *this;
}

template<class Var> template<class Var2> SyntaxString<Var2> SyntaxString<Var>::convert(std::function<Var2(const Var&)> &vc) const{
  SyntaxString<Var2> ss(symbol_count,const_count,arg_count);

  int *const_map = new int[const_count];

  /* Copy consts into ss.consts, while keeping ss.consts ordered.
   * Assign const_map[i] == j where vc(consts[i]) == ss.consts[j]. */
  int cc = 0;
  for(int i = 0; i < const_count; i++){
    Var2 v2 = vc(consts[i]);
    int j = cc;
    while(j > 0 && (ss.consts[j-1] == v2 || v2 < ss.consts[j-1]))
      j--;
    if(j < i && ss.consts[j] == v2){
      const_map[i] = j;
    }else{
      for(int k = cc; k > j; k--){
        ss.consts[k] = ss.consts[k-1];
      }
      for(int k = 0; k < i; k++){
        if(const_map[k] >= j)
          const_map[k]++;
      }
      ss.consts[j] = v2;
      const_map[i] = j;
      cc++;
    }
  }
  ss.const_count = cc;

  /* Copy symbols, while remapping constants */
  for(int i = 0; i < symbol_count;){
    ss.symbols[i] = symbols[i];
    if(symbols[i] == VAR){
      ss.symbols[i+1] = const_map[int(symbols[i+1])];
    }else{
      ss.symbols[i+1] = symbols[i+1];
    }
    i += 2;
  }

  delete[] const_map;

  assert(ss.check_invariant());
  return ss;
};

template<class Var> std::string SyntaxString<Var>::to_string(const std::function<std::string(const Var&)> &vts) const{
  /* Setup argument names */
  std::vector<std::string> arg_names(arg_count);
  {
    int i = 0;
    std::set<std::string> const_names;
    for(i = 0; i < const_count; i++){
      const_names.insert(vts(consts[i]));
    }
    i = 0;
    int j = 0;
    while(i < arg_count){
      std::stringstream ss;
      ss << "arg" << j;
      if(const_names.count(ss.str())){
        j++; // Pick a new name for the same argument
      }else{
        arg_names[i] = ss.str();
        i++;
        j++;
      }
    }
  }

  std::string s = "";
  if(arg_count > 0){
    s = "fun ";
    for(int i = 0; i < arg_count; i++){
      s = s+arg_names[i]+" ";
    }
    s += "=> ";
  }

  s += to_inner_string(0,vts,arg_names);
  return s;
};

template<class Var> std::string SyntaxString<Var>::to_string(const std::function<std::string(const Var&)> &vts, const std::vector<std::string> &arg_names) const{
  return to_inner_string(0,vts,arg_names);
};

template<class Var> std::string SyntaxString<Var>::to_inner_string(int i, const std::function<std::string(const Var&)> &vts, 
                                                                   std::vector<std::string> arg_names) const {
  switch(symbols[i]){
  case INT:
    {
      std::stringstream ss;
      ss << int(symbols[i+1]);
      return ss.str();
    }
  case VAR:
    return vts(consts[int(symbols[i+1])]);
  case ARG:
    return arg_names[int(symbols[i+1])];
  case PLUS:
    return "("+to_inner_string(i+2,vts,arg_names)+") + ("+to_inner_string(i+(symbols[i+1]),vts,arg_names)+")";
  case MINUS:
    return "("+to_inner_string(i+2,vts,arg_names)+") - ("+to_inner_string(i+(symbols[i+1]),vts,arg_names)+")";
  case UNMINUS:
    return "-("+to_inner_string(i+2,vts,arg_names)+")";
  case TRUE:
    return "true";
  case FALSE:
    return "false";
  case EQ:
    return to_inner_string(i+2,vts,arg_names)+" == "+to_inner_string(i+(symbols[i+1]),vts,arg_names);
  case NEQ:
    return to_inner_string(i+2,vts,arg_names)+" != "+to_inner_string(i+(symbols[i+1]),vts,arg_names);
  case LT:
    return to_inner_string(i+2,vts,arg_names)+" < "+to_inner_string(i+(symbols[i+1]),vts,arg_names);
  case LEQ:
    return to_inner_string(i+2,vts,arg_names)+" <= "+to_inner_string(i+(symbols[i+1]),vts,arg_names);
  case GT:
    return to_inner_string(i+2,vts,arg_names)+" > "+to_inner_string(i+(symbols[i+1]),vts,arg_names);
  case GEQ:
    return to_inner_string(i+2,vts,arg_names)+" >= "+to_inner_string(i+(symbols[i+1]),vts,arg_names);
  case NOT:
    return "not["+to_inner_string(i+2,vts,arg_names)+"]";
  case AND:
    return "["+to_inner_string(i+2,vts,arg_names)+"] && ["+to_inner_string(i+(symbols[i+1]),vts,arg_names)+"]";
  case OR:
    return "["+to_inner_string(i+2,vts,arg_names)+"] || ["+to_inner_string(i+(symbols[i+1]),vts,arg_names)+"]";
      
  default:
    throw new std::logic_error("SyntaxString::to_inner_string: Symbol not supported.");
  }
}

template<class Var> SyntaxString<Var> SyntaxString<Var>::combine(symbol_t op, const SyntaxString<Var> &a, 
                                                                 const SyntaxString<Var> &b){
  static std::vector<int> a_map(8);
  static std::vector<int> b_map(8);
  if(a.const_count > int(a_map.size())) a_map.resize(a.const_count*2);
  if(b.const_count > int(b_map.size())) b_map.resize(b.const_count*2);

  int ac = std::max(a.arg_count, b.arg_count);

  /* Calculate cc */
  int cc = 0;
  {
    int i = 0;
    int j = 0;
    while(i < a.const_count || j < b.const_count){
      if(i >= a.const_count){
        b_map[j] = cc;
        j++;
      }else if(j >= b.const_count){
        a_map[i] = cc;
        i++;
      }else if(a.consts[i] < b.consts[j]){
        a_map[i] = cc;
        i++;
      }else if(a.consts[i] == b.consts[j]){
        a_map[i] = cc;
        b_map[j] = cc;
        i++;
        j++;
      }else{ // a.consts[i] > b.consts[j]
        b_map[j] = cc;
        j++;
      }
      cc++;
    }
  }

  SyntaxString<Var> ss(2+a.symbol_count+b.symbol_count,cc,ac);
    
  /* Copy symbols */
  ss.symbols[0] = op;
  ss.symbols[1] = a.symbol_count+2; // index at which the symbols from b start
  {
    int i = 0;
    while(i < a.symbol_count){
      ss.symbols[i+2] = a.symbols[i];
      if(i%2 == 0 && a.symbols[i] == VAR){
        ss.symbols[i+3] = a_map[a.symbols[i+1]];
        i++;
      }
      i++;
    }
    i = 0;
    while(i < b.symbol_count){
      ss.symbols[i+a.symbol_count+2] = b.symbols[i];
      if(i%2 == 0 && b.symbols[i] == VAR){
        ss.symbols[i+a.symbol_count+3] = b_map[b.symbols[i+1]];
        i++;
      }
      i++;
    }
  }

  /* Copy constants */
  {
    int i = 0;
    int j = 0;
    cc = 0;
    while(i < a.const_count || j < b.const_count){
      if(i >= a.const_count){
        ss.consts[cc] = b.consts[j];
        j++;
      }else if(j >= b.const_count){
        ss.consts[cc] = a.consts[i];
        i++;
      }else if(a.consts[i] < b.consts[j]){
        ss.consts[cc] = a.consts[i];
        i++;
      }else if(a.consts[i] == b.consts[j]){
        ss.consts[cc] = b.consts[j];
        i++;
        j++;
      }else{ // a.consts[i] > b.consts[j]
        ss.consts[cc] = b.consts[j];
        j++;
      }
      cc++;
    }
  }

  assert(ss.check_invariant());
  return ss;
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::combine(symbol_t op, const SyntaxString<Var> &a){
  SyntaxString<Var> ss(2+a.symbol_count,0,a.arg_count);
  ss.symbols[0] = op;
  ss.symbols[1] = 0; // Padding. Must be 0 for symbols to be canonical
  std::memcpy(ss.symbols+2,a.symbols,sizeof(symbol_t)*a.symbol_count);
  ss.const_count = a.const_count;
  ss.consts = a.consts;
  ss.consts_ptr_count = a.consts_ptr_count;
  if(ss.consts_ptr_count){
    (*ss.consts_ptr_count)++;
  }
  assert(ss.check_invariant());
  return ss;
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::integer(int i){
  if(i < min_int || i > max_int){
    std::stringstream ss;
    ss << "SyntaxString::integer: Integer constant out of bounds" << i;
    throw new std::logic_error(ss.str());
  }
  SyntaxString<Var> ss(2);
  ss.symbols[0] = INT;
  ss.symbols[1] = i;
  assert(ss.check_invariant());
  return ss;
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::variable(const Var &v){
  SyntaxString<Var> ss(2,1,0);
  ss.symbols[0] = VAR;
  ss.symbols[1] = 0;
  ss.consts[0] = v;
  assert(ss.check_invariant());
  return ss;
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::argument(int a){
  SyntaxString<Var> ss(2,0,a+1);
  ss.symbols[0] = ARG;
  ss.symbols[1] = a;
  assert(ss.check_invariant());
  return ss;
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::minus(const SyntaxString<Var> &a){
  return combine(UNMINUS,a);
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::plus(const SyntaxString<Var> &a, const SyntaxString<Var> &b){
  return combine(PLUS,a,b);
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::minus(const SyntaxString<Var> &a, const SyntaxString<Var> &b){
  return combine(MINUS,a,b);
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::tt(true);
template<class Var> SyntaxString<Var> SyntaxString<Var>::ff(false);

template<class Var> SyntaxString<Var> SyntaxString<Var>::eq(const SyntaxString<Var> &a, const SyntaxString<Var> &b){
  return combine(EQ,a,b);
};
template<class Var> SyntaxString<Var> SyntaxString<Var>::neq(const SyntaxString<Var> &a, const SyntaxString<Var> &b){
  return combine(NEQ,a,b);
};
template<class Var> SyntaxString<Var> SyntaxString<Var>::lt(const SyntaxString<Var> &a, const SyntaxString<Var> &b){
  return combine(LT,a,b);
};
template<class Var> SyntaxString<Var> SyntaxString<Var>::leq(const SyntaxString<Var> &a, const SyntaxString<Var> &b){
  return combine(LEQ,a,b);
};
template<class Var> SyntaxString<Var> SyntaxString<Var>::gt(const SyntaxString<Var> &a, const SyntaxString<Var> &b){
  return combine(GT,a,b);
};
template<class Var> SyntaxString<Var> SyntaxString<Var>::geq(const SyntaxString<Var> &a, const SyntaxString<Var> &b){
  return combine(GEQ,a,b);
};
template<class Var> SyntaxString<Var> SyntaxString<Var>::neg(const SyntaxString<Var> &a){
  return combine(NOT,a);
};
template<class Var> SyntaxString<Var> SyntaxString<Var>::conj(const SyntaxString<Var> &a, const SyntaxString<Var> &b){
  return combine(AND,a,b);
};
template<class Var> SyntaxString<Var> SyntaxString<Var>::disj(const SyntaxString<Var> &a, const SyntaxString<Var> &b){
  return combine(OR,a,b);
};


template<class Var> SyntaxString<Var> SyntaxString<Var>::bind(const std::vector<Var> &argv) const throw(){
  assert(int(argv.size()) == arg_count);

  if(arg_count == 0){
    /* Cheap way out; nothing to bind anyway */
    return *this;
  }

  SyntaxString<Var> ss(symbol_count,const_count+argv.size(),0);

  /* Build new consts and remember index remappings */
  static std::vector<int> const_map(8);
  static std::vector<int> arg_map(8);
  if(int(const_map.size()) < const_count) const_map.resize(const_count*2);
  if(int(arg_map.size()) < arg_count) arg_map.resize(arg_count*2);

  for(int i = 0; i < const_count; i++){
    ss.consts[i] = consts[i];
    const_map[i] = i;
  }

  int cc = const_count; // Number of unique constants added so far

  /* Insert variables from argv orderly */
  for(int i = 0; i < arg_count; i++){
    /* Check if this argument is ever used */
    bool used = false;
    {
      int k = 0;
      while(k < symbol_count){
        if(symbols[k] == ARG && symbols[k+1] == i){
          used = true;
          break;
        }
        k += 2;
      }
    }

    if(used){ // Otherwise ignore
      int j = 0;
      while(j < cc && ss.consts[j] < argv[i])
        j++;
      if(j == cc || ss.consts[j] != argv[i]){
        /* Move larger constants one step to the right */
        for(int k = cc; k > j; k--){
          ss.consts[k] = ss.consts[k-1];
        }
        /* Update const_map to mirror the one-step move */
        for(int k = const_count-1; k >= 0 && const_map[k] >= j; k--){
          const_map[k]++;
        }
        /* Update arg_map to mirror the one-step move */
        for(int k = 0; k < arg_count; k++){
          if(arg_map[k] >= j)
            arg_map[k]++;
        }

        ss.consts[j] = argv[i];
        cc++;
      }
      arg_map[i] = j;
    }
  }

  ss.const_count = cc;

  /* Copy symbols while translating constant indices */
  int i = 0;
  while(i < symbol_count){
    if(symbols[i] == VAR){
      ss.symbols[i] = symbols[i];
      ss.symbols[i+1] = const_map[int(symbols[i+1])];
    }else if(symbols[i] == ARG){
      ss.symbols[i] = VAR;
      ss.symbols[i+1] = arg_map[int(symbols[i+1])];
    }else{
      ss.symbols[i] = symbols[i];
      ss.symbols[i+1] = symbols[i+1];
    }
    i += 2;
  }

  assert(ss.check_invariant());
  return ss;
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::substitute(const SyntaxString<Var> &t, const Var &v) const throw(){
  /* Find v in consts */
  int v_i; // consts[v_i] == v
  for(v_i = 0; v_i < const_count; v_i++){
    if(consts[v_i] == v)
      break;
  }
  if(v_i == const_count){
    /* v is not in consts, null-substitution */
    assert(check_invariant());
    return *this;
  }

  /* Count the occurrences of v in this */
  int v_count = 0;
  {
    int i = 0;
    while(i < symbol_count){
      if(symbols[i] == VAR && symbols[i+1] == v_i)
        v_count++;
      i += 2;
    }
  }

  int ac = std::max(arg_count,t.arg_count);

  SyntaxString<Var> ss(symbol_count + v_count*(t.symbol_count-2),const_count+t.const_count,ac);

  /* Construct the new consts */
  int cc = 0; // Number of constants inserted so far
  static std::vector<int> c_map(8);
  static std::vector<int> tc_map(8);
  if(int(c_map.size()) < const_count) c_map.resize(const_count*2);
  if(int(c_map.size()) < const_count) c_map.resize(const_count*2);
  {
    int i = 0; // Indexing into consts
    int j = 0; // Indexing into t.consts
    while(i < const_count || j < t.const_count){
      if(i == v_i){
        /* Ignore v from consts */
        /* Note that v may still be included in ss.consts if it is present in t.consts. */
        i++;
      }else{
        if(i >= const_count){
          ss.consts[cc] = t.consts[j];
          tc_map[j] = cc;
          j++;
        }else if(j >= t.const_count){
          ss.consts[cc] = consts[i];
          c_map[i] = cc;
          i++;
        }else if(consts[i] < t.consts[j]){
          ss.consts[cc] = consts[i];
          c_map[i] = cc;
          i++;
        }else if(consts[i] == t.consts[j]){
          ss.consts[cc] = consts[i];
          c_map[i] = cc;
          tc_map[j] = cc;
          i++;
          j++;
        }else{ // consts[i] > t.consts[j]
          ss.consts[cc] = t.consts[j];
          tc_map[j] = cc;
          j++;
        }
        cc++;
      }
    }
    ss.const_count = cc;
  }

  /* Copy symbols while remapping variables */
  {
    static std::vector<int> ref_map(32); // ref_map[i] == j if ss.symbols[j] is a pointer to symbols[i]
    if(ss.symbol_count > int(ref_map.size())) ref_map.resize(ss.symbol_count*2);
    for(int a = 0; a < ss.symbol_count; a++){
      ref_map[a] = -1; // ref_map[i] == -1 if there is no pointer to symbols[i]
    }
    int i = 0; // Indexing into symbols
    int j = 0; // Indexing into ss.symbols
    while(i < symbol_count){
      if(ref_map[i] != -1){
        ss.symbols[ref_map[i]] = j-ref_map[i]+1;
        assert((i-ref_map[i]+1) % 2 == 0);
      }
      if(symbols[i] == VAR){
        if(symbols[i+1] == v_i){
          /* copy t into ss.symbols */
          int k = 0; // Indexing into t.symbols
          while(k < t.symbol_count){
            ss.symbols[j] = t.symbols[k];
            if(t.symbols[k] == VAR){
              ss.symbols[j+1] = tc_map[int(t.symbols[k+1])];
            }else{
              ss.symbols[j+1] = t.symbols[k+1];
            }
            j+=2;
            k+=2;
          }
        }else{
          /* Remap the constant */
          ss.symbols[j] = VAR;
          ss.symbols[j+1] = c_map[int(symbols[i+1])];
          j+=2;
        }
      }else{
        ss.symbols[j] = symbols[i];
        ss.symbols[j+1] = symbols[i+1];
        if(binary_symbol(symbols[i])){
          assert(ref_map[i+int(symbols[i+1])] == -1);
          ref_map[i+int(symbols[i+1])] = j+1;
        }
        j+=2;
      }
      i+=2;
    }
  }

  assert(ss.check_invariant());
  return ss;
};


template<class Var> std::string SyntaxString<Var>::debug_to_string() const throw(){
  std::stringstream ss;
  ss << "argc: " << arg_count << "\n";
  ss << "consts: ";
  for(int i = 0; i < const_count; i++){
    ss << " \"" << consts[i] << "\"";
  }
  ss << "\n";

  ss << "symbols: ";
  for(int i = 0; i < symbol_count; i+=2){
    switch(symbols[i]){
    case INT: ss << "INT "; break;
    case VAR: ss << "VAR "; break;
    case ARG: ss << "ARG "; break;
    case PLUS: ss << "PLUS "; break;
    case MINUS: ss << "MINUS "; break;
    case UNMINUS: ss << "UNMINUS "; break;
    case TRUE: ss << "TRUE "; break;
    case FALSE: ss << "FALSE "; break;
    case EQ: ss << "EQ "; break;
    case NEQ: ss << "NEQ "; break;
    case LT: ss << "LT "; break;
    case LEQ: ss << "LEQ "; break;
    case GT: ss << "GT "; break;
    case GEQ: ss << "GEQ "; break;
    case NOT: ss << "NOT "; break;
    case AND: ss << "AND "; break;
    case OR: ss << "OR "; break;
    }
    ss << int(symbols[i+1]) << " ";
  }
  return ss.str();
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::separate_left_argument() const throw(){
  assert(binary_symbol(symbols[0]));
  return separate_interval(2,int(symbols[1])-2);
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::separate_right_argument() const throw(){
  assert(binary_symbol(symbols[0]));
  return separate_interval(int(symbols[1]),symbol_count - int(symbols[1]));
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::separate_only_argument() const throw(){
  assert(unary_symbol(symbols[0]));
  return separate_interval(2,symbol_count-2);
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::separate_interval(int offset, int new_symbol_count) const throw(){
  SyntaxString<Var> ss(new_symbol_count,const_count,0);

  /* Copy constants and setup arg_count */
  static std::vector<int> c_map(8);
  if(const_count > int(c_map.size())) c_map.resize(const_count*2);
  {
    int cc = 0; // Constants added hitherto
    for(int i = 0; i < ss.symbol_count; i+=2){
      if(symbols[offset+i] == VAR){
        int v_i = symbols[offset+i+1];
        Var v = consts[v_i];
        /* Find position in ss.consts for the new constant */
        int j;
        for(j = 0; j < cc && ss.consts[j] < v; j++) ; 
        if(j < cc && ss.consts[j] == v){
          /* Repetition of previous variable; ignore */
        }else{
          for(int k = cc; k > j; k--){
            ss.consts[k] = ss.consts[k-1];
          }
          for(int k = 0; k < const_count; k++){
            if(c_map[k] >= j)
              c_map[k]++;
          }
          ss.consts[j] = v;
          c_map[v_i] = j;
          cc++;
        }
      }else if(symbols[offset+i] == ARG){
        ss.arg_count = std::max(ss.arg_count,int(symbols[offset+i+1])+1);
      }
    }
    ss.const_count = cc;
  }

  /* Copy symbols while remapping constants */
  for(int i = 0; i < ss.symbol_count; i+=2){
    ss.symbols[i] = symbols[offset+i];
    if(symbols[offset+i] == VAR){
      ss.symbols[i+1] = c_map[int(symbols[offset+i+1])];
    }else{
      ss.symbols[i+1] = symbols[offset+i+1];
    }
  }

  assert(ss.check_invariant());
  return ss;
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::drive_in_negations() const throw(){
  assert(predicate_symbol(symbols[0]));
  return drive_in_negations(true);
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::drive_in_negations(bool sign) const throw(){
  switch(symbols[0]){
  case TRUE:
    if(sign){
      return *this;
    }else{
      return ff;
    }
  case FALSE:
    if(sign){
      return *this;
    }else{
      return tt;
    }
  case EQ:
    if(sign){
      return *this;
    }else{
      return neq(separate_left_argument(),separate_right_argument());
    }
  case NEQ:
    if(sign){
      return *this;
    }else{
      return eq(separate_left_argument(),separate_right_argument());
    }
  case LT:
    if(sign){
      return *this;
    }else{
      return geq(separate_left_argument(),separate_right_argument());
    }
  case LEQ:
    if(sign){
      return *this;
    }else{
      return gt(separate_left_argument(),separate_right_argument());
    }
  case GT:
    if(sign){
      return *this;
    }else{
      return leq(separate_left_argument(),separate_right_argument());
    }
  case GEQ:
    if(sign){
      return *this;
    }else{
      return lt(separate_left_argument(),separate_right_argument());
    }
  case NOT:
    return separate_only_argument().drive_in_negations(!sign);
  case AND:
    if(sign){
      return conj(separate_left_argument().drive_in_negations(true),
                  separate_right_argument().drive_in_negations(true));
    }else{
      return disj(separate_left_argument().drive_in_negations(false),
                  separate_right_argument().drive_in_negations(false));
    }
  case OR:
    if(sign){
      return disj(separate_left_argument().drive_in_negations(true),
                  separate_right_argument().drive_in_negations(true));
    }else{
      return conj(separate_left_argument().drive_in_negations(false),
                  separate_right_argument().drive_in_negations(false));
    }
  };
  throw new std::logic_error("SyntaxString::drive_in_negations: Illegal symbol");
};

template<class Var> SyntaxString<Var> SyntaxString<Var>::generalise() const throw(){
  assert(check_invariant());
  SyntaxString<Var> ss(symbol_count,0,arg_count+const_count);

  std::vector<int> var_to_arg(const_count,-1);
  int vars_remapped = 0;
  // Copy symbols while remapping constants to arguments
  for(int i = 0; i < symbol_count; i+=2){
    if(symbols[i] == VAR){
      ss.symbols[i] = ARG;
      if(var_to_arg[int(symbols[i+1])] == -1){
        var_to_arg[int(symbols[i+1])] = arg_count + vars_remapped;
        vars_remapped++;
      }
      ss.symbols[i+1] = var_to_arg[int(symbols[i+1])];
    }else{
      ss.symbols[i] = symbols[i];
      ss.symbols[i+1] = symbols[i+1];
    }
  }
  assert(vars_remapped == const_count);

  assert(ss.check_invariant());
  return ss;
};

template<class Var> void SyntaxString<Var>::translate(std::function<Var(const Var&)> &t){
  static std::vector<int> c_map(8);
  if(int(c_map.size()) < const_count) c_map.resize(const_count*2);
  bool identity_map;

  if(*symbols_ptr_count == 1 && *consts_ptr_count == 1){
    /* Exclusive ownership, allows for translation without reallocation */
    
    /* translate arguments while keeping c_map updated */
    identity_map = true;
    int cc = 0;
    for(int i = 0; i < const_count; i++){
      Var v = t(consts[i]);
      int j = cc;
      while(j > 0 && !(consts[j-1] < v)) j--;
      c_map[i] = j;
      if(j < cc && consts[j] == v){
        identity_map = false;
      }else{
        if(j != cc){
          identity_map = false;
          for(int k = cc; k > j; k--){
            consts[k] = consts[k-1];
          }
          for(int k = 0; k < i; k++){
            if(c_map[k] >= j)
              c_map[k]++;
          }
        }
        consts[j] = v;
        cc++;
      }
    }
    const_count = cc;

    /* Translate symbols if necessary */
    if(!identity_map){
      for(int i = 0; i < symbol_count; i+=2){
        if(symbols[i] == VAR){
          symbols[i+1] = c_map[int(symbols[i+1])];
        }
      }
    }
    assert(check_invariant());
  }else{
    /* Need to reallocate */
    SyntaxString<Var> tmp(*this);
    self_destruct();
    init(tmp.symbol_count, tmp.const_count, tmp.arg_count);

    /* Copy and translate constants */
    identity_map = true;
    int cc = 0;
    for(int i = 0; i < tmp.const_count; i++){
      Var v = t(tmp.consts[i]);
      int j = cc;
      while(j > 0 && !(consts[j-1] < v)) j--;
      c_map[i] = j;
      if(j < cc && consts[j] == v){
        identity_map = false;
      }else{
        if(j != cc){
          identity_map = false;
          for(int k = cc; k > j; k--){
            consts[k] = consts[k-1];
          }
          for(int k = 0; k < i; k++){
            if(c_map[k] >= j)
              c_map[k]++;
          }
        }
        consts[j] = v;
        cc++;
      }
    }
    const_count = cc;

    
    /* Copy and translate symbols */
    for(int i = 0; i < symbol_count; i+=2){
      symbols[i] = tmp.symbols[i];
      if(tmp.symbols[i] == VAR){
        symbols[i+1] = c_map[int(tmp.symbols[i+1])];
      }else{
        symbols[i+1] = tmp.symbols[i+1];
      }
    }
    assert(check_invariant());
  }
  assert(check_invariant());
};


template<class Var> std::list<SyntaxString<Var> > SyntaxString<Var>::conjuncts() const throw(){
  if(symbols[0] == AND){
    std::list<SyntaxString<Var> > l1(separate_left_argument().conjuncts());
    std::list<SyntaxString<Var> > l2(separate_right_argument().conjuncts());
    for(typename std::list<SyntaxString<Var> >::iterator it = l2.begin(); it != l2.end(); it++){
      l1.push_back(*it);
    }
    return l1;
  }else{
    return std::list<SyntaxString<Var> >(1,*this);
  }
};

template<class Var> int SyntaxString<Var>::compare(const SyntaxString<Var> &ss) const throw(){
  /* Compare argument count */
  if(arg_count < ss.arg_count){
    return -1;
  }else if(arg_count > ss.arg_count){
    return 1;
  }

  /* Compare symbols */
  std::vector<int> vars;
  if(symbols != ss.symbols){
    if(symbol_count < ss.symbol_count){
      return -1;
    }else if(symbol_count > ss.symbol_count){
      return 1;
    }

    /* symbol_count == ss.symbol_count */
    for(int i = 0; i < symbol_count; i++){
      if(i % 2 == 0 && symbols[i] == VAR){
        vars.push_back(symbols[i+1]);
      }
      if(symbols[i] < ss.symbols[i]){
        return -1;
      }else if(symbols[i] > ss.symbols[i]){
        return 1;
      }
      assert(symbols[i] == ss.symbols[i]);
    }
  }

  /* Compare constants */
  if(consts != ss.consts){
    assert(const_count == ss.const_count); // symbols are equal -> const_counts are equal

    for(int i = 0; i < const_count; i++){
      if(consts[i] < ss.consts[i]){
        return -1;
      }else if(!(consts[i] == ss.consts[i])){ // consts[i] > ss.consts[i]
        return 1;
      }
    }
  }

  return 0;
};


#if HAVE_LIBMATHSAT == 1
template<class Var> MSat::msat_term SyntaxString<Var>::to_msat_term(MSat::msat_env &env, 
                                                                    std::map<Var,MSat::msat_decl> &var_decl_map,
                                                                    const std::function<std::string(const Var&)> &vts) const throw(){
  return to_msat_term(0,env,var_decl_map,vts);
};
#endif

#if HAVE_LIBMATHSAT == 1
template<class Var> MSat::msat_term SyntaxString<Var>::to_msat_term(int i, MSat::msat_env &env, 
                                                                    std::map<Var,MSat::msat_decl> &var_decl_map,
                                                                    const std::function<std::string(const Var&)> &vts) const throw(){
  assert(i % 2 == 0);
  switch(symbols[i]){
  case INT:
    {
      std::stringstream ss;
      ss << int(symbols[i+1]);
      return MSat::msat_make_number(env,ss.str().c_str());
    }
  case VAR:
    {
      typename std::map<Var,MSat::msat_decl>::iterator it = var_decl_map.find(consts[int(symbols[i+1])]);
      if(it == var_decl_map.end()){
        const Var &v = consts[int(symbols[i+1])];
#if MATHSAT_VERSION == 4
        MSat::msat_decl dcl = MSat::msat_declare_variable(env,vts(v).c_str(),MSat::MSAT_INT);
#elif MATHSAT_VERSION == 5
        MSat::msat_decl dcl = MSat::msat_declare_function(env,vts(v).c_str(),MSat::msat_get_integer_type(env));
#endif
        var_decl_map.insert(typename std::pair<Var,MSat::msat_decl>(v,dcl));
#if MATHSAT_VERSION == 4
        return MSat::msat_make_variable(env,dcl);
#elif MATHSAT_VERSION == 5
        return MSat::msat_make_constant(env,dcl);
#endif
      }else{
#if MATHSAT_VERSION == 4
        return MSat::msat_make_variable(env,it->second);
#elif MATHSAT_VERSION == 5
        return MSat::msat_make_constant(env,it->second);
#endif
      }
    }
  case ARG:
    {
      Log::warning << "SyntaxString::to_msat_term: Applied to non-nullary SyntaxString\n";
      throw new std::logic_error("SyntaxString::to_msat_term: Applied to non-nullary SyntaxString");
    }
  case PLUS:
    {
      return MSat::msat_make_plus(env,
                                  to_msat_term(i+2,env,var_decl_map,vts),
                                  to_msat_term(i+symbols[i+1],env,var_decl_map,vts));
    }
  case MINUS:
    {
#if MATHSAT_VERSION == 4
      return MSat::msat_make_minus(env,to_msat_term(i+2,env,var_decl_map,vts),
                                   to_msat_term(i+symbols[i+1],env,var_decl_map,vts));
#elif MATHSAT_VERSION == 5
      return MSat::msat_make_plus(env,
                                  to_msat_term(i+2,env,var_decl_map,vts),
                                  MSat::msat_make_times(env,
                                                        MSat::msat_make_number(env,"-1"),
                                                        to_msat_term(i+symbols[i+1],env,var_decl_map,vts)));
#endif
    }
  case UNMINUS:
    {
#if MATHSAT_VERSION == 4
      return MSat::msat_make_negate(env,to_msat_term(i+2,env,var_decl_map,vts));
#elif MATHSAT_VERSION == 5
      return MSat::msat_make_times(env,
                                   MSat::msat_make_number(env,"-1"),
                                   to_msat_term(i+2,env,var_decl_map,vts));
#endif
    }
  case TRUE: return MSat::msat_make_true(env);
  case FALSE: return MSat::msat_make_false(env);
  case EQ:
    {
      return MSat::msat_make_equal(env,
                                  to_msat_term(i+2,env,var_decl_map,vts),
                                  to_msat_term(i+symbols[i+1],env,var_decl_map,vts));
    }
  case NEQ:
    {
      return MSat::msat_make_not(env,
                                 MSat::msat_make_equal(env,
                                                       to_msat_term(i+2,env,var_decl_map,vts),
                                                       to_msat_term(i+symbols[i+1],env,var_decl_map,vts)));
    }
  case LT:
    {
#if MATHSAT_VERSION == 4
      return MSat::msat_make_lt(env,to_msat_term(i+2,env,var_decl_map,vts),
                                to_msat_term(i+symbols[i+1],env,var_decl_map,vts));
#elif MATHSAT_VERSION == 5
      MSat::msat_term a = to_msat_term(i+2,env,var_decl_map,vts);
      MSat::msat_term b = to_msat_term(i+symbols[i+1],env,var_decl_map,vts);
      return MSat::msat_make_and(env,
                                 MSat::msat_make_leq(env,a,b),
                                 MSat::msat_make_not(env,MSat::msat_make_equal(env,a,b)));
#endif
    }
  case LEQ:
    {
      return MSat::msat_make_leq(env,
                                 to_msat_term(i+2,env,var_decl_map,vts),
                                 to_msat_term(i+symbols[i+1],env,var_decl_map,vts));
    }
  case GT:
    {
#if MATHSAT_VERSION == 4
      MSat::msat_make_gt(env,to_msat_term(i+2,env,var_decl_map,vts),
                         to_msat_term(i+symbols[i+1],env,var_decl_map,vts));
#elif MATHSAT_VERSION == 5
      MSat::msat_term a = to_msat_term(i+2,env,var_decl_map,vts);
      MSat::msat_term b = to_msat_term(i+symbols[i+1],env,var_decl_map,vts);
      return MSat::msat_make_and(env,
                                 MSat::msat_make_leq(env,b,a),
                                 MSat::msat_make_not(env,MSat::msat_make_equal(env,a,b)));
#endif
    }
  case GEQ:
    {
#if MATHSAT_VERSION == 4
      return MSat::msat_make_geq(env,to_msat_term(i+symbols[i+1],env,var_decl_map,vts),
                                 to_msat_term(i+2,env,var_decl_map,vts));
#elif MATHSAT_VERSION == 5
      return MSat::msat_make_leq(env,
                                 to_msat_term(i+symbols[i+1],env,var_decl_map,vts),
                                 to_msat_term(i+2,env,var_decl_map,vts));
#endif
    }
  case NOT:
    {
      return MSat::msat_make_not(env,to_msat_term(i+2,env,var_decl_map,vts));
    }
  case AND:
    {
      return MSat::msat_make_and(env,
                                 to_msat_term(i+2,env,var_decl_map,vts),
                                 to_msat_term(i+symbols[i+1],env,var_decl_map,vts));
    }
  case OR:
    {
      return MSat::msat_make_or(env,
                                to_msat_term(i+2,env,var_decl_map,vts),
                                to_msat_term(i+symbols[i+1],env,var_decl_map,vts));
    }
  default:
    Log::warning << "SyntaxString::to_msat_term: Unsupported symbol: symbols[" 
                 << i << "] == " << int(symbols[i]) << "\n";
    throw new std::logic_error("SyntaxString::to_msat_term: Unsupported symbol.");
  }
};
#endif

#if HAVE_LIBMATHSAT == 1
template<class Var> SyntaxString<Var> 
SyntaxString<Var>::from_msat_term(MSat::msat_env &env, MSat::msat_term t,
                                  const std::function<Var(const std::string&)> &stv){

  if(msat_query(MSat::msat_term_is_true, env, t)){
    return tt;
  }else if(msat_query(MSat::msat_term_is_false,env,t)){
    return ff;
  }else if(msat_query(MSat::msat_term_is_number, env, t)){
    char *s = MSat::msat_term_repr(t);
    int i;
#ifndef NDEBUG
    int res = std::sscanf(s,"%d",&i);
    assert(res == 1);
#else
    std::sscanf(s,"%d",&i);
#endif
#if MATHSAT_VERSION == 4
    std::free(s);
#elif MATHSAT_VERSION == 5
    MSat::msat_free(s);
#endif
    if(i < min_int || i > max_int){
      std::stringstream ss;
      ss << "SyntaxString::from_msat_term: Integer constant out of bounds" << i;
      throw new std::logic_error(ss.str());
    }
    return integer(i);
  }else if(msat_query(MSat::msat_term_is_and, env, t)){
    return conj(from_msat_term(env,MSat::msat_term_get_arg(t,0),stv),
                from_msat_term(env,MSat::msat_term_get_arg(t,1),stv));
  }else if(msat_query(MSat::msat_term_is_or, env, t)){
    return disj(from_msat_term(env,MSat::msat_term_get_arg(t,0),stv),
                from_msat_term(env,MSat::msat_term_get_arg(t,1),stv));
  }else if(msat_query(MSat::msat_term_is_not, env, t)){
    return neg(from_msat_term(env,MSat::msat_term_get_arg(t,0),stv));
  }else if(msat_query(MSat::msat_term_is_iff, env, t)){
    SyntaxString<Var> a = from_msat_term(env,MSat::msat_term_get_arg(t,0),stv);
    SyntaxString<Var> b = from_msat_term(env,MSat::msat_term_get_arg(t,1),stv);
    return disj(conj(a,b),conj(neg(a),neg(b)));
  }else if(msat_query(MSat::msat_term_is_term_ite, env, t)){
    SyntaxString<Var> a = from_msat_term(env,MSat::msat_term_get_arg(t,0),stv);
    SyntaxString<Var> b = from_msat_term(env,MSat::msat_term_get_arg(t,1),stv);
    SyntaxString<Var> c = from_msat_term(env,MSat::msat_term_get_arg(t,2),stv);
    return disj(conj(a,b),conj(neg(a),c));
#if MATHSAT_VERSION == 4
  }else if(msat_query(MSat::msat_term_is_variable, env, t)){
#elif MATHSAT_VERSION == 5
  }else if(msat_query(MSat::msat_term_is_constant, env, t)){
#endif
    char *s = MSat::msat_term_repr(t);
    Var v = stv(std::string(s));
#if MATHSAT_VERSION == 4
    std::free(s);
#elif MATHSAT_VERSION == 5
    MSat::msat_free(s);
#endif
    return variable(v);
  }else if(msat_query(MSat::msat_term_is_equal, env, t)){
    if(msat_query(MSat::msat_term_is_atom, env, t)){
      // Integer equality
      SyntaxString<Var> a = from_msat_term(env,MSat::msat_term_get_arg(t,0),stv);
      SyntaxString<Var> b = from_msat_term(env,MSat::msat_term_get_arg(t,1),stv);
      return eq(a,b);
    }else{ // Boolean equality
      SyntaxString<Var> a = from_msat_term(env,MSat::msat_term_get_arg(t,0),stv);
      SyntaxString<Var> b = from_msat_term(env,MSat::msat_term_get_arg(t,1),stv);
      return disj(conj(a,b),conj(neg(a),neg(b)));
    }
  }else if(msat_query(MSat::msat_term_is_leq, env, t)){
    SyntaxString<Var> a = from_msat_term(env,MSat::msat_term_get_arg(t,0),stv);
    SyntaxString<Var> b = from_msat_term(env,MSat::msat_term_get_arg(t,1),stv);
    return leq(a,b);
  }else if(msat_query(MSat::msat_term_is_plus, env, t)){
    SyntaxString<Var> a = from_msat_term(env,MSat::msat_term_get_arg(t,0),stv);
    SyntaxString<Var> b = from_msat_term(env,MSat::msat_term_get_arg(t,1),stv);
    return plus(a,b);
  }else if(msat_query(MSat::msat_term_is_times, env, t)){
    SyntaxString<Var> a = from_msat_term(env,MSat::msat_term_get_arg(t,0),stv);
    SyntaxString<Var> b = from_msat_term(env,MSat::msat_term_get_arg(t,1),stv);
    int n;
    bool minus;
    SyntaxString<Var> m = a;
    if(a.is_integer()){
      n = (a.get_integer() >= 0 ? a.get_integer() : -a.get_integer());
      minus = a.get_integer() < 0;
      m = b;
    }else if(b.is_integer()){
      n = (b.get_integer() >= 0 ? b.get_integer() : -b.get_integer());
      minus = b.get_integer() < 0;
      m = a;
    }else{
      std::stringstream ss;
      char *s = MSat::msat_term_repr(t);
      ss << "SyntaxString::from_msat_term: Unsupported term: " << s;
#if MATHSAT_VERSION == 4
      std::free(s);
#elif MATHSAT_VERSION == 5
      MSat::msat_free(s);
#endif
      throw new std::logic_error(ss.str());
    }
    if(n == 0){
      return integer(0);
    }
    SyntaxString<Var> res = m;
    n--;
    while(n > 0){
      res = plus(res,m);
      n--;
    }
    if(minus){
      res = SyntaxString<Var>::minus(res);
    }
    return res;
#if MATHSAT_VERSION == 4
  }else if(MSat::msat_term_is_lt(t)){
    SyntaxString<Var> a = from_msat_term(env,MSat::msat_term_get_arg(t,0),stv);
    SyntaxString<Var> b = from_msat_term(env,MSat::msat_term_get_arg(t,1),stv);
    return lt(a,b);
  }else if(MSat::msat_term_is_gt(t)){
    SyntaxString<Var> a = from_msat_term(env,MSat::msat_term_get_arg(t,0),stv);
    SyntaxString<Var> b = from_msat_term(env,MSat::msat_term_get_arg(t,1),stv);
    return gt(a,b);
  }else if(MSat::msat_term_is_geq(t)){
    SyntaxString<Var> a = from_msat_term(env,MSat::msat_term_get_arg(t,0),stv);
    SyntaxString<Var> b = from_msat_term(env,MSat::msat_term_get_arg(t,1),stv);
    return geq(a,b);
  }else if(MSat::msat_term_is_minus(t)){
    SyntaxString<Var> a = from_msat_term(env,MSat::msat_term_get_arg(t,0),stv);
    SyntaxString<Var> b = from_msat_term(env,MSat::msat_term_get_arg(t,1),stv);
    return minus(a,b);
  }else if(MSat::msat_term_is_implies(t)){
    SyntaxString<Var> a = from_msat_term(env,MSat::msat_term_get_arg(t,0),stv);
    SyntaxString<Var> b = from_msat_term(env,MSat::msat_term_get_arg(t,1),stv);
    return disj(neg(a),b);
  }else if(MSat::msat_term_is_xor(t)){
    SyntaxString<Var> a = from_msat_term(env,MSat::msat_term_get_arg(t,0),stv);
    SyntaxString<Var> b = from_msat_term(env,MSat::msat_term_get_arg(t,1),stv);
    return disj(conj(a,neg(b)),conj(neg(a),b));
  }else if(MSat::msat_term_is_negate(t)){
    SyntaxString<Var> a = from_msat_term(env,MSat::msat_term_get_arg(t,0),stv);
    return minus(a);
#endif
  }else{
    std::stringstream ss;
    char *s = MSat::msat_term_repr(t);
    ss << "SyntaxString::from_msat_term: Unsupported term: " << s;
#if MATHSAT_VERSION == 4
    std::free(s);
#elif MATHSAT_VERSION == 5
    MSat::msat_free(s);
#endif
    throw new std::logic_error(ss.str());
  }
};
#endif

template<class Var> bool SyntaxString<Var>::check_invariant() const{
#ifdef NDEBUG
  Log::warning << "SyntaxString::check_invariant: Called when NDEBUG is defined.";
  assert(false);
#endif
  if(arg_count < 0){
    Log::warning << "SyntaxString::check_invariant: Negative argument count.\n";
    return false;
  }
  if(const_count < 0){
    Log::warning << "SyntaxString::check_invariant: Negative constant count.\n";
    return false;
  }
  if(symbol_count < 0){
    Log::warning << "SyntaxString::check_invariant: Negative symbol count.\n";
    return false;
  }
  if(symbol_count % 2 != 0){
    Log::warning << "SyntaxString::check_invariant: Odd symbol count.\n";
    return false;
  }

  /* Check that all variables in consts are used and that there is no
   * variable reference to outside of consts. */
  std::vector<bool> used(const_count,false);
  for(int i = 0; i < symbol_count; i+=2){
    if(symbols[i] == VAR){
      if(symbols[i+1] < 0 || symbols[i+1] >= const_count){
        Log::warning << "SyntaxString::check_invariant: variable index out of bound.\n";
        return false;
      }
      used[int(symbols[i+1])] = true;
    }
  }
  for(unsigned i = 0; i < used.size(); i++){
    if(!used[i]){
      Log::warning << "SyntaxString::check_invariant: Unused variable in consts.\n";
      return false;
    }
  }

  /* Check symbols, padding and that arguments are between 0 and arg_count, and that the last argument (if present) is used */
  int max_arg = -1;
  for(int i = 0; i < symbol_count; i+=2){
    if(symbols[i] < INT || symbols[i] > OR){
      Log::warning << "SyntaxString::check_invariant: Invalid symbol: " << symbols[i] << "\n";
      return false;
    }
    if(symbols[i] == UNMINUS || symbols[i] == TRUE || symbols[i] == FALSE || symbols[i] == NOT){
      if(symbols[i+1] != 0){
        Log::warning << "SyntaxString::check_invariant: Missing padding.\n";
        return false;
      }
    }
    if(symbols[i] == ARG && (symbols[i+1] < 0 || symbols[i+1] >= arg_count)){
      Log::warning << "SyntaxString::check_invariant: Argument out of bounds.\n";
      return false;
    }
    if(symbols[i] == ARG){
      max_arg = std::max(max_arg,int(symbols[i+1]));
    }
  }
  if(arg_count > 0 && max_arg != arg_count-1){
    Log::warning << "SyntaxString::check_invariant: Unused argument.\n";
    return false;
  }


  return true;
};

/*
template<class Var> SyntaxString<Var> SyntaxString<Var>::simplify() const throw(){
  switch(symbols[0]){
  case INT: return *this;
  case VAR: return *this;
  case ARG: return *this;
  case TRUE: return *this;
  case FALSE: return *this;
  case PLUS:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      if(b.symbols[0] == UNMINUS){
        return minus(a,b.separate_only_argument()).simplify();
      }else if(b.symbols[0] == INT && a.symbols[0] == PLUS && a.separate_right_argument().symbols[0] == INT){
        int sum = a.separate_right_argument().get_integer() + b.get_integer();
        return plus(a.separate_left_argument(),SyntaxString::integer(sum)).simplify();
      }else if(b.symbols[0] == INT && b.get_integer() == 0){
        return a;
      }else if(b.symbols[0] == INT && b.get_integer() < 0){
        return minus(a,integer(-b.get_integer())).simplify();
      }else{
        return plus(a,b);
      }
    }
  case MINUS:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      return minus(a,b);
    }
  case UNMINUS:
    {
      SyntaxString a = separate_only_argument().simplify();
      return minus(a);
    }
  case EQ:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      if(a.symbols[0] == PLUS && b.symbols[0] == PLUS &&
         a.separate_right_argument().symbols[0] == INT &&
         b.separate_right_argument().symbols[0] == INT){
        int sum = b.separate_right_argument().get_integer() - a.separate_right_argument().get_integer();
        return eq(a.separate_left_argument(),plus(b.separate_left_argument(),integer(sum))).simplify();
      }else if(a.symbols[0] == PLUS && b.symbols[0] == INT &&
               a.separate_right_argument().symbols[0] == INT){
        int sum = b.get_integer() - a.separate_right_argument().get_integer();
        return eq(a.separate_left_argument(),integer(sum)).simplify();
      }else{
        return eq(a,b);
      }
      return eq(a,b);
    }
  case NEQ:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      if(a.symbols[0] == PLUS && b.symbols[0] == PLUS &&
         a.separate_right_argument().symbols[0] == INT &&
         b.separate_right_argument().symbols[0] == INT){
        int sum = b.separate_right_argument().get_integer() - a.separate_right_argument().get_integer();
        return neq(a.separate_left_argument(),plus(b.separate_left_argument(),integer(sum))).simplify();
      }else if(a.symbols[0] == PLUS && b.symbols[0] == INT &&
               a.separate_right_argument().symbols[0] == INT){
        int sum = b.get_integer() - a.separate_right_argument().get_integer();
        return neq(a.separate_left_argument(),integer(sum)).simplify();
      }else{
        return neq(a,b);
      }
    }
  case LT:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      if(a.symbols[0] == PLUS && b.symbols[0] == PLUS &&
         a.separate_right_argument().symbols[0] == INT &&
         b.separate_right_argument().symbols[0] == INT){
        int sum = b.separate_right_argument().get_integer() - a.separate_right_argument().get_integer();
        return lt(a.separate_left_argument(),plus(b.separate_left_argument(),integer(sum))).simplify();
      }else if(a.symbols[0] == PLUS && b.symbols[0] == INT &&
               a.separate_right_argument().symbols[0] == INT){
        int sum = b.get_integer() - a.separate_right_argument().get_integer();
        return lt(a.separate_left_argument(),integer(sum)).simplify();
      }else{
        return lt(a,b);
      }
    }
  case LEQ:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      return leq(a,b);
    }
  case GT:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      return gt(a,b);
    }
  case GEQ:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      if(a.symbols[0] == PLUS && b.symbols[0] == PLUS &&
         a.separate_right_argument().symbols[0] == INT &&
         b.separate_right_argument().symbols[0] == INT){
        int sum = b.separate_right_argument().get_integer() - a.separate_right_argument().get_integer();
        return geq(a.separate_left_argument(),plus(b.separate_left_argument(),integer(sum))).simplify();
      }else if(a.symbols[0] == PLUS && b.symbols[0] == INT &&
               a.separate_right_argument().symbols[0] == INT){
        int sum = b.get_integer() - a.separate_right_argument().get_integer();
        return geq(a.separate_left_argument(),integer(sum)).simplify();
      }else{
        return geq(a,b);
      }
    }
  case NOT:
    {
      return drive_in_negations().simplify();
    }
  case AND:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      return conj(a,b);
    }
  case OR:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      return disj(a,b);
    }
  default:
    throw new std::logic_error("SyntaxString::simplify: Unknown symbol.");
  }
}
*/

template<class Var> SyntaxString<Var> SyntaxString<Var>::simplify() const throw(){
  switch(symbols[0]){
  case PLUS:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      if(b.symbols[0] == UNMINUS){
        return minus(a,b.separate_only_argument()).simplify();
      }else if(b.symbols[0] == INT && a.symbols[0] == PLUS && a.separate_right_argument().symbols[0] == INT){
        int sum = a.separate_right_argument().get_integer() + b.get_integer();
        return plus(a.separate_left_argument(),SyntaxString::integer(sum)).simplify();
      }else if(b.symbols[0] == INT && b.get_integer() == 0){
        return a;
      }else if(b.symbols[0] == INT && b.get_integer() < 0){
        return minus(a,integer(-b.get_integer())).simplify();
      }else{
        return plus(a,b);
      }
    }
  case INT:
  case VAR:
  case ARG:
  case TRUE:
  case FALSE:
    return *this;
  case UNMINUS:
    {
      SyntaxString a = separate_only_argument().simplify();
      return minus(a);
    }
  case NOT:
    {
      // return drive_in_negations().simplify();
      SyntaxString a = separate_only_argument().simplify();
      return neg(a);
    }
  case MINUS:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      return minus(a,b);
    }
  case EQ:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      if(a.symbols[0] == PLUS && b.symbols[0] == PLUS &&
         a.separate_right_argument().symbols[0] == INT &&
         b.separate_right_argument().symbols[0] == INT){
        int sum = b.separate_right_argument().get_integer() - a.separate_right_argument().get_integer();
        return eq(a.separate_left_argument(),plus(b.separate_left_argument(),integer(sum))).simplify();
      }else{
        return eq(a,b);
      }
    }
  case NEQ:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      return neq(a,b);
    }
  case LT:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      if(a.symbols[0] == PLUS && b.symbols[0] == PLUS &&
         a.separate_right_argument().symbols[0] == INT &&
         b.separate_right_argument().symbols[0] == INT){
        int sum = b.separate_right_argument().get_integer() - a.separate_right_argument().get_integer();
        return lt(a.separate_left_argument(),plus(b.separate_left_argument(),integer(sum))).simplify();
      }else{
        return lt(a,b);
      }
    }
  case LEQ:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      return leq(a,b);
    }
  case GT:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      return gt(a,b);
    }
  case GEQ:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      return geq(a,b);
    }
  case AND:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      return conj(a,b);
    }
  case OR:
    {
      SyntaxString a = separate_left_argument().simplify();
      SyntaxString b = separate_right_argument().simplify();
      return disj(a,b);
    }
  default:
    throw new std::logic_error("SyntaxString::simplify: Unknown symbol.");
  }
}

template<class Var> 
template<typename C, typename A> int SyntaxString<Var>::eval(C c, A a) const{
  static std::vector<int> e(16);
  int sc = symbol_count/2;
  if(sc > int(e.size())){
    e.resize(sc);
  }
  for(int i = sc-1; i >= 0; i--){
    int l = i + 1;
    int r = i + symbols[i*2+1]/2;
    switch(symbols[i*2]){
    case INT:
      e[i] = symbols[i*2+1];
      break;
    case VAR:
      e[i] = c[consts[symbols[i*2+1]]];
      break;
    case ARG:
      e[i] = a[symbols[i*2+1]];
      break;
    case PLUS:
      e[i] = e[l] + e[r];
      break;
    case MINUS:
      e[i] = e[l] - e[r];
      break;
    case UNMINUS:
      e[i] = -e[i+1];
      break;
    case TRUE:
      e[i] = 1;
      break;
    case FALSE:
      e[i] = 0;
      break;
    case EQ:
      if(e[l] == e[r]) e[i] = 1;
      else e[i] = 0;
      break;
    case NEQ:
      if(e[l] != e[r]) e[i] = 1;
      else e[i] = 0;
      break;
    case LT:
      if(e[l] < e[r]) e[i] = 1;
      else e[i] = 0;
      break;
    case LEQ:
      if(e[l] <= e[r]) e[i] = 1;
      else e[i] = 0;
      break;
    case GT:
      if(e[l] > e[r]) e[i] = 1;
      else e[i] = 0;
      break;
    case GEQ:
      if(e[l] >= e[r]) e[i] = 1;
      else e[i] = 0;
      break;
    case NOT:
      e[i] = 1 - e[i+1];
      break;
    case AND:
      if(e[l] == 1 && e[r] == 1) e[i] = 1;
      else e[i] = 0;
      break;
    case OR:
      if(e[l] == 1 || e[r] == 1) e[i] = 1;
      else e[i] = 0;
      break;
    default:
      throw new std::logic_error("SyntaxString::eval: Unknown symbol.");
    }
  }
  return e[0];
};
