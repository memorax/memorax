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

#include "log.h"
#include <functional>
#include <sstream>
#include "lang.h"

template<class Z> const ZStar<Z> ZStar<Z>::STAR;

template<class Z> bool inline ZStar<Z>::operator==(const ZStar<Z> &zs) const throw(){
  return wild == zs.wild && (wild || z == zs.z);
};

template<class Z> bool inline ZStar<Z>::operator!=(const ZStar<Z> &zs) const throw(){
  return !(*this == zs);
};

template<class Z> bool inline ZStar<Z>::operator<(const ZStar<Z> &zs) const throw(){
  return !wild && (zs.wild || z < zs.z);
};

template<class Z> bool inline ZStar<Z>::operator<=(const ZStar<Z> &zs) const throw(){
  return (*this < zs) || (*this == zs);
};

template<class Z> bool inline ZStar<Z>::operator>(const ZStar<Z> &zs) const throw(){
  return !zs.wild && (wild || z > zs.z);
};

template<class Z> bool inline ZStar<Z>::operator>=(const ZStar<Z> &zs) const throw(){
  return (*this > zs) || (*this == zs);
};

template<class Z> ZStar<Z> inline ZStar<Z>::operator+(const ZStar<Z> &zs) const throw(){
  assert(!wild && !zs.wild);
  return z+zs.z;
};

template<class Z> ZStar<Z> inline ZStar<Z>::operator-(const ZStar<Z> &zs) const throw(){
  assert(!wild && !zs.wild);
  return z-zs.z;
};

template<class Z> ZStar<Z> inline ZStar<Z>::operator*(const ZStar<Z> &zs) const throw(){
  assert(!wild && !zs.wild);
  return z*zs.z;
};

template<class Z> ZStar<Z> inline ZStar<Z>::operator/(const ZStar<Z> &zs) const throw(){
  assert(!wild && !zs.wild);
  assert(zs.z != 0);
  return z/zs.z;
};

template<class Z> std::string inline ZStar<Z>::to_string() const throw(){
  std::stringstream ss;
  if(wild){
    ss << "*";
  }else{
    ss << z;
  }
  return ss.str();
};

template<class Z> inline Z ZStar<Z>::get_int() const throw(){
  assert(!wild);
  return z;
};

template<class Z> inline ZStar<Z>::Vector::Vector(int sz){
  vec = new ZStar<Z>[sz+2];
  vec[0] = 1;
  vec[1] = sz;
  for(int i = 0; i < sz; i++){
    vec[i+2] = STAR;
  }
};

template<class Z> inline ZStar<Z>::Vector::Vector(const std::vector<ZStar<Z> > &v){
  vec = new ZStar<Z>[v.size()+2];
  vec[0] = 1;
  vec[1] = v.size();
  for(unsigned i = 0; i < v.size(); ++i){
    vec[i+2] = v[i];
  }
};

template<class Z> inline ZStar<Z>::Vector::Vector(const ZStar<Z>::Vector &v){
  vec = v.vec;
  vec[0] = vec[0] + ZStar<Z>(1);
  assert(int(vec[0]) > 1);
};

template<class Z> inline ZStar<Z>::Vector::Vector(int sz, std::function<ZStar(int)> &f){
  vec = new ZStar<Z>[sz+2];
  vec[0] = 1;
  vec[1] = sz;
  for(int i = 0; i < sz; ++i){
    vec[i+2] = f(i);
  }
};

template<class Z> inline typename ZStar<Z>::Vector &
ZStar<Z>::Vector::operator=(const ZStar<Z>::Vector &v){
  if(&v != this){
    assert(int(vec[0]) > 0);
    assert(int(v.vec[0]) > 0);
    release_vec();
    vec = v.vec;
    vec[0] = vec[0] + ZStar<Z>(1);
    assert(int(vec[0]) > 1);
  }
  return *this;
};

template<class Z> inline ZStar<Z>::Vector::~Vector(){
  release_vec();
};

template<class Z> inline const ZStar<Z> &ZStar<Z>::Vector::operator[](int i) const {
  return vec[i+2];
};

template<class Z> inline typename ZStar<Z>::Vector 
ZStar<Z>::Vector::assign(int i, const ZStar<Z> &val) const{
  Vector v(size());
  for(int j = 0; j < size(); ++j){
    v.vec[j+2] = vec[j+2];
  }
  v.vec[i+2] = val;
  return v;
};

template<class Z> inline typename ZStar<Z>::Vector
ZStar<Z>::Vector::unify(const ZStar<Z>::Vector &v, bool *unifiable) const{
  assert(size() == v.size());
  ZStar<Z>::Vector lub(size());
  for(int i = 0; i < size(); ++i){
    if(vec[i+2] == v.vec[i+2]){
      lub.vec[i+2] = vec[i+2];
    }else if(vec[i+2] == STAR){
      lub.vec[i+2] = v.vec[i+2];
    }else if(v.vec[i+2] == STAR){
      lub.vec[i+2] = vec[i+2];
    }else{
      *unifiable = false;
      return lub;
    }
  }
  *unifiable = true;
  return lub;
};

template<class Z> inline int ZStar<Z>::Vector::size() const{
  return vec[1].get_int();
};

template<class Z> inline int ZStar<Z>::Vector::compare(const ZStar<Z>::Vector &v) const{
  if(vec[1] < v.vec[1]){
    return -1;
  }else if(vec[1] > v.vec[1]){
    return 1;
  }
  for(int i = 0; i < vec[1].get_int(); ++i){
    if(vec[i+2] < v.vec[i+2]){
      return -1;
    }else if(vec[i+2] > v.vec[i+2]){
      return 1;
    }
  }
  return 0;
};

template<class Z> inline void ZStar<Z>::Vector::release_vec(){
  assert(vec != 0);
  assert(int(vec[0]) > 0);
  vec[0] = vec[0] - ZStar<Z>(1);
  if(vec[0].get_int() == 0){
    delete[] vec;
  }
  vec = 0;
};

template<class Z> Constraint::Comparison 
ZStar<Z>::Vector::entailment_compare(const ZStar<Z>::Vector &v) const{
  if(size() != v.size()){
    return Constraint::INCOMPARABLE;
  }

  Constraint::Comparison cmp = Constraint::EQUAL;
  for(int i = 0; cmp != Constraint::INCOMPARABLE && i < size(); ++i){
    if(vec[i+2] != v.vec[i+2]){
      if(vec[i+2] == STAR){
        cmp = Constraint::comb_comp(cmp,Constraint::LESS);
      }else if(v.vec[i+2] == STAR){
        cmp = Constraint::comb_comp(cmp,Constraint::GREATER);
      }else{
        cmp = Constraint::INCOMPARABLE;
      }
    }
  }
  return cmp;
};

template<class Z> std::string ZStar<Z>::Vector::to_string() const throw(){
  std::string s = "[";
  for(int i = 0; i < vec[1].get_int(); ++i){
    if(i != 0) s += ",";
    s += vec[i+2].to_string();
  }
  return s+"]";
};

template<class Z> int ZStar<Z>::test(){
  int res = 0;
  std::function<void(std::string,bool)> tst = 
    [&res](std::string t, bool r){
    Log::debug << "  " << t << ": ";
    if(r){
      Log::debug << "Ok\n";
    }else{
      res = 1;
      Log::debug << "FAILURE\n";
    }
  };

  Log::debug << "Testing ZStar\n";
  /* Comparisons */
  tst("0 < STAR",ZStar(0) < ZStar::STAR);
  tst("INTMAX < STAR",ZStar(std::numeric_limits<int>::max()) < ZStar::STAR);
  tst("INTMIN < STAR",ZStar(std::numeric_limits<int>::min()) < ZStar::STAR);
  tst("!(STAR < 1)",!(ZStar::STAR < ZStar(1)));
  tst("!(2 == 4)",!(ZStar(2) == ZStar(4)));
  tst("0 < 1",ZStar(0) < ZStar(1));
  tst("0 <= 1",ZStar(0) <= ZStar(1));
  tst("STAR == STAR",ZStar::STAR == ZStar());
  tst("STAR != 0",ZStar::STAR != ZStar(0));
  tst("0 != STAR",ZStar(0) != ZStar::STAR);
  tst("STAR >= 0",ZStar::STAR >= ZStar(0));
  tst("STAR >= STAR",ZStar::STAR >= ZStar::STAR);
  tst("!(0 >= STAR)",!(ZStar(0) >= ZStar::STAR));

  /* Arithmetic */
  tst("1+3 == 4",ZStar(1) + ZStar(3) == ZStar(4));
  tst("2-4 == -2",ZStar(2) - ZStar(4) == ZStar(-2));
  tst("3*7 == 21",ZStar(3)*ZStar(7) == ZStar(21));
  tst("5/2 == 2",ZStar(5)/ZStar(2) == ZStar(2));

  /* Entailment & unify */
  {
    Vector v0(3);
    std::vector<ZStar<int> > v1v;
    v1v.push_back(0);
    v1v.push_back(1);
    v1v.push_back(2);
    Vector v1(v1v);
    std::function<ZStar<int>(int)> v2f = [](int i){
      if(i == 1) return ZStar<int>(1);
      else return ZStar<int>::STAR;
    };
    Vector v2(3,v2f);
    std::function<ZStar<int>(int)> v3f = [](int i){
      if(i == 2) return ZStar<int>(2);
      else return ZStar<int>::STAR;
    };
    Vector v3(3,v3f);
    std::function<ZStar<int>(int)> v2u3f = [](int i){
      if(i == 1) return ZStar<int>(1);
      else if(i == 2) return ZStar<int>(2);
      else return ZStar<int>::STAR;
    };
    Vector v2u3(3,v2u3f);
    std::function<ZStar<int>(int)> v4f = [](int i){
      if(i == 2) return ZStar<int>(4);
      else return ZStar<int>::STAR;
    };
    Vector v4(3,v4f);
    
    tst(v0.to_string()+" lesser than "+v1.to_string(),
        v0.entailment_compare(v1) == Constraint::LESS);
    tst(v1.to_string()+" greater than "+v0.to_string(),
        v1.entailment_compare(v0) == Constraint::GREATER);
    tst(v1.to_string()+" equal to "+v1.to_string(),
        v1.entailment_compare(v1) == Constraint::EQUAL);
    tst(v0.to_string()+" equal to "+v0.to_string(),
        v0.entailment_compare(v0) == Constraint::EQUAL);
    tst(v0.to_string()+" lesser than "+v2.to_string(),
        v0.entailment_compare(v2) == Constraint::LESS);
    tst(v2.to_string()+" greater than "+v0.to_string(),
        v2.entailment_compare(v0) == Constraint::GREATER);
    tst(v2.to_string()+" lesser than "+v1.to_string(),
        v2.entailment_compare(v1) == Constraint::LESS);
    tst(v1.to_string()+" greater than "+v2.to_string(),
        v1.entailment_compare(v2) == Constraint::GREATER);
    bool u;
    tst(v2.to_string()+" unify "+v3.to_string()+" == "+v2u3.to_string(),
        v2.unify(v3,&u) == v2u3 && u);
    tst(v2.to_string()+" unify "+v3.to_string()+" != "+v2.to_string(),
        v2.unify(v3,&u) != v2 && u);
    tst(v2.to_string()+" unify "+v3.to_string()+" != "+v3.to_string(),
        v2.unify(v3,&u) != v3 && u);
    tst(v2.to_string()+" unify "+v3.to_string()+" == "+
        v3.to_string()+" unify "+v2.to_string(),
        v2.unify(v3,&u) == v3.unify(v2,&u) && u);
    tst(v0.to_string()+" unify "+v2u3.to_string()+" == "+v2u3.to_string(),
        v0.unify(v2u3,&u) == v2u3 && u);
    v4.unify(v3,&u);
    tst(v4.to_string()+" not unifiable with "+v3.to_string(),!u);
    v3.unify(v4,&u);
    tst(v3.to_string()+" not unifiable with "+v4.to_string(),!u);
  }

  /* Assignment */
  {
    Vector v0(4);
    Vector v0copy(v0); /* Shares the representation of v0 */
    Vector v1(4); /* Same content as v0, different representation */
    std::vector<ZStar<int> > v2v;
    v2v.push_back(1);
    v2v.push_back(2);
    v2v.push_back(3);
    v2v.push_back(4);
    Vector v2(v2v); /* Different content from v0 */
    Vector v22(v2v); /* Same content as v2, different representation*/
    v0 = v2; /* Change the representation of v0 to that of v2 */
    /* Check that v0copy and v2 are not disturbed, and that v0 gets the right values */
    tst("Assignment of vectors non-destructive",
        v0 == v22 && v2 == v22 && v0copy == v1 && v1 != v0);
  }

  /* eval */
  {
    std::vector<ZStar<int> > vv;
    vv.push_back(1);
    vv.push_back(2);
    vv.push_back(3);
    vv.push_back(4);
    ZStar<int>::Vector v(vv);
    
    Lang::Expr<int> e = Lang::Expr<int>::reg(1) + Lang::Expr<int>::reg(2);

    tst("("+e.to_string(Lang::int_reg_to_string())+").eval("+v.to_string()+") == 5",
        e.eval(v,v) == 5);
  }

  return res;
};
