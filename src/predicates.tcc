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

#include "predicates.h"

namespace Predicates{

  template<class Var> Term<Var> Term<Var>::from_expr(const Lang::Expr<int> &e,int pid){
    std::function<Var(const int&)> vc = 
      [pid](const int &i){
      return Var::reg(i,pid);
    };
    return Term(e.convert(vc));
  };

  template<class Var> class IntToReg : public std::function<Var(const int&)>{
  public:
    IntToReg(int pid) : pid(pid) {};
    Var operator()(const int &i) const{ return Var::reg(i,pid); };
    Var operator()(const int &i){ return Var::reg(i,pid); };
    int pid;
  };

  template<class Var> Predicate<Var> Predicate<Var>::from_bexpr(const Lang::BExpr<int> &e,int pid){
    IntToReg<Var> f(pid);
    std::function<Var(const int&)> g(f);
    return Predicate(e.convert(g));
  };

  template<class Var> std::set<Var> Predicate<Var>::get_variables() const throw(){
    std::set<Var> s;
    for(int i = 0; i < this->const_count; i++){
      s.insert(this->consts[i]);
    }
    return s;
  };

  template<class Var> AppliedPredicate<Var> Predicate<Var>::operator()(const std::vector<Var> &argv) 
    const throw(ArgcError*){
    return AppliedPredicate<Var>(this,argv);
  }

  template<class Var> Predicate<Var> Predicate<Var>::bind(const std::vector<Var> &argv) const throw(){
    return Predicate<Var>(SyntaxString<Var>::bind(argv));
  };

  template<class Var> Predicate<Var> Predicate<Var>::substitute(const Term<Var> &t,const Var &v) const throw(){
    return Predicate<Var>(SyntaxString<Var>::substitute(t,v));
  };

  template<class Var> std::list<Predicate<Var> > Predicate<Var>::conjuncts() const throw(){
    std::list<Predicate<Var> > l;
    std::list<SyntaxString<Var> > lss = SyntaxString<Var>::conjuncts();
    for(typename std::list<SyntaxString<Var> >::iterator it = lss.begin(); it != lss.end(); it++){
      l.push_back(Predicate<Var>(*it));
    }
    return l;
  };

  template<class Var> std::string AppliedPredicate<Var>::to_string(const std::function<std::string(const Var&)> &vts) const throw(){
    std::vector<std::string> arg_names(argv.size());
    for(unsigned i = 0; i < argv.size(); i++){
      arg_names[i] = vts(argv[i]);
    }
    return pred->to_string(vts,arg_names);
  };

  template<class Var> std::set<Var> AppliedPredicate<Var>::get_variables() const throw(){
    std::set<Var> set = pred->get_variables();
    for(int i = 0; i < int(argv.size()); i++)
      set.insert(argv[i]);
    return set;
  }

  template<class Var> AppliedPredicate<Var>::AppliedPredicate(const Predicate<Var> *p, const std::vector<Var> &av) 
    throw(ArgcError*) :
    pred(p), argv(av) {
    if(pred->get_argc() != int(argv.size()))
      throw new ArgcError();
  }

  template<class Var> void AppliedPredicate<Var>::translate_args(std::function<Var(const Var&)> &t){
    for(unsigned i = 0; i < argv.size(); i++)
      argv[i] = t(argv[i]);
  }

  template<class Var> bool AppliedPredicate<Var>::operator==(const AppliedPredicate<Var> &ap) const throw(){
    return (pred->compare(*ap.pred) == 0) && argv == ap.argv;
  };

  template<class Var> bool AppliedPredicate<Var>::operator<(const AppliedPredicate<Var> &ap) const throw(){
    int pcmp = pred->compare(*ap.pred);
    return (pcmp < 0) || ((pcmp == 0) && (argv < ap.argv));
  };

};
