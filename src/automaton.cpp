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

#include "automaton.h"
#include "lexer.h"
#include "machine.h"
#include "parser.h"
#include "test.h"

#include <functional>
#include <sstream>

Automaton::Automaton(){
  states.resize(1);
}

Automaton::Automaton(const Lang::Stmt<int> &ast) :
  states(), label_map() {
  states.push_back(State());
  unsat_goto_t unsat_goto; // Collect forward gotos here
  construct_from_ast(ast,unsat_goto);

  /* Attempt to satisfy gotos */
  for(unsat_goto_t::iterator it = unsat_goto.begin(); it != unsat_goto.end(); it++){
    if(label_map.count(it->second)){
      int tgt = label_map[it->second];
      for(std::set<Transition*>::iterator tit = it->first.begin();
          tit != it->first.end(); tit++){
        (*tit)->target = tgt;
        states[tgt].bwd_transitions.insert(*tit);
      }
    }else{
      std::string s = "Label ";
      throw new UnDefinedLabel(s+it->second+" in statement goto "+it->second+
                               " is not defined.");
    }
  }

  /* The last state may be superfluous. If so, remove it. */
  if(states.size() > 1 &&
     states.back().bwd_transitions.empty() &&
     states.back().fwd_transitions.empty()){
    states.resize(states.size()-1);
  }

  check_fwd_bwd_consistency();
}

void Automaton::clear_and_copy(const Automaton &a){
  /* clear */
  dealloc();
  label_map.clear();
  states.clear();

  /* copy */
  label_map = a.label_map;
  for(unsigned i = 0; i < a.states.size(); i++){
    State st;
    for(std::set<Transition*>::const_iterator it = a.states[i].fwd_transitions.begin();
        it != a.states[i].fwd_transitions.end(); it++){
      Transition *tr = new Transition((*it)->source,(*it)->instruction,(*it)->target);
      st.fwd_transitions.insert(tr);
    }
    states.push_back(st);
  }
  for(unsigned i = 0; i < states.size(); i++){
    for(std::set<Transition*>::const_iterator it = states[i].fwd_transitions.begin();
        it != states[i].fwd_transitions.end(); it++){
      int j = (*it)->target;
      states[j].bwd_transitions.insert(*it);
    }
  }

  check_fwd_bwd_consistency();
}

Automaton::Automaton(const Automaton &a){
  clear_and_copy(a);
}

Automaton &Automaton::operator=(const Automaton &a){
  if(&a != this)
    clear_and_copy(a);
  return *this;
}

void Automaton::construct_from_ast(const Lang::Stmt<int> &ast,unsat_goto_t &unsat_goto,
                                   int source){
  switch(ast.get_type()){
  case Lang::NOP: case Lang::ASSIGNMENT: case Lang::ASSUME:
  case Lang::READASSERT: case Lang::READASSIGN: case Lang::WRITE:
  case Lang::LOCKED: case Lang::SYNCWR: case Lang::FENCE:
  case Lang::SYNCRDASSERT: case Lang::SYNCRDASSIGN:
  case Lang::SSFENCE: case Lang::LLFENCE:
    /* Ordinary instructions */
    {
      int i = states.size() - 1;
      if(source < 0) source = i;
      int target = i;
      if(source == i){
        states.push_back(State());
        target = i+1;
      }
      Transition *trans = new Transition(source,ast,target);
      states[source].fwd_transitions.insert(trans);
      states[target].bwd_transitions.insert(trans);
      break;
    }
  case Lang::GOTO:
    {
      int i = states.size() - 1;
      /* Check that there is no label at this statement */
      for(std::map<Lang::label_t,int>::iterator it = label_map.begin();
          it != label_map.end(); it++){
        if(it->second == i){
          dealloc();
          std::string str = "Label ";
          throw new LabeledGoto(str+(it->first)+" at goto statement "+
                                ast.to_string(Lang::int_reg_to_string(),
                                              Lang::int_memloc_to_string())+
                                ". (Labeled goto statements are forbidden.)");
        }
      }

      if(states.size() == 1){
        /* Goto is the first statement in the AST.
         * Add a nop from state 0, before goto
         */
        states.push_back(State());
        ++i;
        source = 0;
        Transition *nopt = new Transition(0,Lang::Stmt<int>::nop(),i);
        states[0].fwd_transitions.insert(nopt);
        states[1].bwd_transitions.insert(nopt);
      }else if(source >= 0 && source != i){ // Insert a nop before goto
        Transition *nopt = new Transition(source,Lang::Stmt<int>::nop(),i);
        states[source].fwd_transitions.insert(nopt);
        states[i].bwd_transitions.insert(nopt);
      }

      /* Ok, try to link states */
      Lang::label_t label = ast.get_goto_target();
      if(label_map.count(label)){ // Backward goto

        int j = label_map[label];
        for(std::set<Transition*>::iterator it = states[i].bwd_transitions.begin();
            it != states[i].bwd_transitions.end(); it++){
          (*it)->target = j;
          states[j].bwd_transitions.insert(*it);
        }
        states[i].bwd_transitions = std::set<Transition*>(); // empty set
      }else{ // Forward goto (or goto to undefined label)
        std::pair<std::set<Transition*>,Lang::label_t> pr;
        pr.first = states[i].bwd_transitions;
        pr.second = label;
        unsat_goto.insert(pr);
        states[i].bwd_transitions.clear();
      }
      break;
    }
  case Lang::UPDATE: case Lang::FETCH: case Lang::EVICT: case Lang::WRLLC:
    throw new std::logic_error("Automaton: Pseudo-instruction in AST: "+
                               ast.to_string(Lang::int_reg_to_string(),
                                             Lang::int_memloc_to_string()));
  case Lang::IF:
    {
      int i = states.size() - 1;
      if(source < 0) source = i;
      Lang::Stmt<int> iftrue = Lang::Stmt<int>::assume(ast.get_condition(),ast.get_pos());
      Lang::BExpr<int> fcond = !ast.get_condition();
      Lang::Stmt<int> iffalse = Lang::Stmt<int>::assume(fcond,ast.get_pos());
      int then_state;
      if(source == i){
        states.push_back(State());
        then_state = i+1;
      }else
        then_state = i;
      Transition *trans = new Transition(source,iftrue,then_state);
      states[source].fwd_transitions.insert(trans);
      states[then_state].bwd_transitions.insert(trans);
      Lang::label_t then_label = ast.get_then_label();
      if(then_label != ""){
        if(label_map.count(then_label)){
          dealloc();
          throw new RedefinedLabel("Label "+then_label+" redefined in the same process.");
        }
        label_map[then_label] = then_state;
      }
      construct_from_ast(*ast.get_then_statement(),unsat_goto);

      int else_state = states.size() - 1; // New last state

      if(ast.get_else_statement()){
        Lang::label_t else_label = ast.get_else_label();
        if(else_label != ""){
          if(label_map.count(else_label)){
            dealloc();
            throw new RedefinedLabel("Label "+else_label+" redefined in the same process.");
          }
          label_map[else_label] = else_state;
        }
        std::set<Transition*> skip_else = states[else_state].bwd_transitions;
        states[else_state].bwd_transitions.clear();
        construct_from_ast(*ast.get_else_statement(),unsat_goto);
        int k = states.size() - 1; // New last state
        for(std::set<Transition*>::iterator it = skip_else.begin();
            it != skip_else.end(); it++){
          (*it)->target = k;
          states[k].bwd_transitions.insert(*it);
        }
      }

      trans = new Transition(source,iffalse,else_state);
      states[source].fwd_transitions.insert(trans);
      states[else_state].bwd_transitions.insert(trans);
      break;
    }
  case Lang::WHILE:
    {
      int i = states.size() - 1;
      Lang::Stmt<int> iftrue = Lang::Stmt<int>::assume(ast.get_condition(),ast.get_pos());
      Lang::BExpr<int> fcond = !ast.get_condition();
      Lang::Stmt<int> iffalse = Lang::Stmt<int>::assume(fcond,ast.get_pos());
      Transition *trans = new Transition(i,iftrue,i+1);
      states.push_back(State());
      states[i].fwd_transitions.insert(trans);
      states[i+1].bwd_transitions.insert(trans);
      Lang::label_t label = ast.get_label();
      if(label != ""){
        if(label_map.count(label)){
          dealloc();
          throw new RedefinedLabel("Label "+label+" redefined in the same process.");
        }
        label_map[label] = i+1;
      }
      construct_from_ast(*ast.get_statement(),unsat_goto);

      int j = states.size() - 1; // New last state

      /* Loop */
      for(std::set<Transition*>::iterator it = states[j].bwd_transitions.begin();
          it != states[j].bwd_transitions.end(); it++){
        (*it)->target = i;
        states[i].bwd_transitions.insert(*it);
      }
      states[j].bwd_transitions.clear();

      /* End loop */
      trans = new Transition(i,iffalse,j);
      states[i].fwd_transitions.insert(trans);
      states[j].bwd_transitions.insert(trans);

      if(source >= 0 && source != i){ /* Insert nop before while */
        Transition *nopt = new Transition(source,Lang::Stmt<int>::nop(),i);
        states[source].fwd_transitions.insert(nopt);
        states[i].bwd_transitions.insert(nopt);
      }
      break;
    }
  case Lang::EITHER:
    {
      int i = states.size() - 1;
      if(source < 0) source = i;
      std::set<Transition *> to_end;
      if(source == i)
        states.push_back(State());

      for(int cs = 0; cs < ast.get_statement_count(); cs++){
        /* Add statements for case cs */
        construct_from_ast(*ast.get_statement(cs),unsat_goto,source);

        /* Collect outgoing transitions */
        int k = states.size() - 1; // Last state
        for(std::set<Transition*>::iterator it = states[k].bwd_transitions.begin();
            it != states[k].bwd_transitions.end(); it++)
          to_end.insert(*it);
        states[k].bwd_transitions.clear();
      }

      /* Relink outgoing transitions */
      int j = states.size() - 1;
      for(std::set<Transition*>::iterator it = to_end.begin(); it != to_end.end(); it++){
        (*it)->target = j;
        states[j].bwd_transitions.insert(*it);
      }
      break;
    }
  case Lang::SEQUENCE:
    {
      for(int j = 0; j < ast.get_statement_count(); j++){
        Lang::label_t lbl = ast.get_label(j);
        if(lbl != ""){
          if(label_map.count(lbl)){ // Already defined
            dealloc();
            throw new RedefinedLabel("Label "+lbl+" redefined in the same process.");
          }

          int i = states.size() - 1;
          if(j == 0 && source >= 0 && source != i){
            /* Insert nop before labeled state */
            Transition *nopt = new Transition(source,Lang::Stmt<int>::nop(),i);
            states[source].fwd_transitions.insert(nopt);
            states[i].bwd_transitions.insert(nopt);

            source = -1; // reset source
          }

          // Add label to state
          label_map[lbl] = i;
        }
        int new_source = (j == 0) ? source : -1;
        construct_from_ast(*ast.get_statement(j),unsat_goto,new_source);
      }
      break;
    }
  }
}

void Automaton::dealloc(){
  for(unsigned i = 0; i < states.size(); i++){
    for(std::set<Transition*>::iterator it = states[i].fwd_transitions.begin();
        it != states[i].fwd_transitions.end(); it++)
      delete(*it);
  }
}

Automaton::~Automaton(){
  dealloc();
}

std::string Automaton::Transition::to_string(const std::function<std::string(const int&)> &regts,
                                             const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const throw(){
  std::stringstream ss;
  ss << "(Q" << source << ", "
     << instruction.to_string(regts,mlts)
     << ", Q" << target << ")";
  return ss.str();
}

std::string Automaton::to_string(const std::function<std::string(const int&)> &regts,
                                 const std::function<std::string(const Lang::MemLoc<int> &)> &mlts,
                                 int indentation) const throw(){
  std::string s;
  std::string ind(indentation,' ');
  for(auto it = label_map.begin(); it != label_map.end(); ++it){
    std::stringstream ss;
    ss << ind << it->first << ": Q" << it->second << "\n";
    s += ss.str();
  }
  for(unsigned i = 0; i < states.size(); i++){
    for(std::set<Transition*>::const_iterator it = states[i].fwd_transitions.begin();
        it != states[i].fwd_transitions.end(); it++){
      s += ind + (*it)->to_string(regts,mlts) + "\n";
    }
  }
  return s;
}

void Automaton::check_fwd_bwd_consistency() const throw(Exception*){
  std::string complaint = "Automaton::check_fwd_bwd_consistency: "
    "Forward and backward transitions are inconsistent.";
  for(unsigned i = 0; i < states.size(); i++){
    for(std::set<Transition*>::const_iterator it = states[i].fwd_transitions.begin();
        it != states[i].fwd_transitions.end(); it++){
      int j = (*it)->target;
      if(states[j].bwd_transitions.count(*it) == 0)
        throw new Exception(complaint);
    }
    for(std::set<Transition*>::const_iterator it = states[i].bwd_transitions.begin();
        it != states[i].bwd_transitions.end(); it++){
      int j = (*it)->source;
      if(states[j].fwd_transitions.count(*it) == 0)
        throw new Exception(complaint);
    }
  }
}

int Automaton::state_index_of_label(Lang::label_t lbl) const throw(UnDefinedLabel*){
  if(label_map.count(lbl))
    return (label_map.find(lbl))->second;
  else
    throw new UnDefinedLabel(lbl);
}

std::string Automaton::to_dot(const std::function<std::string(const int&)> &regts,
                              const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const throw(){
  static int gv_id = 0; // Used in names for nodes to ensure uniqueness

  gv_id++;
  std::stringstream ss;
  for(unsigned i = 0; i < states.size(); i++){
    std::string labels = "";
    for(std::map<std::string,int>::const_iterator it = label_map.begin();
        it != label_map.end(); it++){
      if(it->second == int(i))
        labels += it->first + ": ";
    }
    ss << "automaton_node_" << gv_id << "_" << i << " [label=\"" << labels << i << "\"]\n";
    for(std::set<Transition*>::const_iterator it = states[i].fwd_transitions.begin();
        it != states[i].fwd_transitions.end(); it++){
      ss << "  automaton_node_" << gv_id << "_" << i << " -> " <<
        "automaton_node_" << gv_id << "_" << (*it)->target <<
        " [label=\"" << (*it)->instruction.to_string(regts,mlts,-1,"") << "\"]\n";
    }
  }
  return ss.str();
}

std::list<Lang::MemLoc<int> > Automaton::get_possible_writes() const{
  std::list<Lang::MemLoc<int> > mls;
  for(unsigned i = 0; i < states.size(); i++){
    for(std::set<Transition*>::const_iterator it = states[i].fwd_transitions.begin();
        it != states[i].fwd_transitions.end(); it++){
      std::vector<Lang::MemLoc<int> > mls2 = (*it)->instruction.get_writes();
      mls.insert(mls.end(),mls2.begin(),mls2.end());
    }
  }

  mls.sort();
  mls.unique();
  return mls;
}

bool Automaton::add_transition(const Transition &t){
  int max_state = std::max(t.source,t.target);
  if(int(states.size()) <= max_state){
    states.resize(max_state+1);
  }

  bool found = false;
  for(auto it = states[t.source].fwd_transitions.begin(); !found && it != states[t.source].fwd_transitions.end(); it++){
    if(**it == t){
      found = true;
    }
  }

  if(found){
    return false;
  }else{
    Transition *tp = new Transition(t);
    states[t.source].fwd_transitions.insert(tp);
    states[t.target].bwd_transitions.insert(tp);
    return true;
  }
}

bool Automaton::del_transition(const Transition &targ){
  Transition t = targ;

  if(t.source >= int(states.size()) || t.target >= int(states.size())) return false;
  bool found = false;
  for(auto it = states[t.source].fwd_transitions.begin(); it != states[t.source].fwd_transitions.end(); ){
    if(**it == t){
      states[t.source].fwd_transitions.erase(it);
      it = states[t.source].fwd_transitions.end();
      found = true;
    }else{
      ++it;
    }
  }

  if(!found) return false;

  found = false;
  for(auto it = states[t.target].bwd_transitions.begin(); it != states[t.target].bwd_transitions.end(); ){
    if(**it == t){
      delete *it;
      states[t.target].bwd_transitions.erase(it);
      it = states[t.target].bwd_transitions.end();
      found = true;
    }else{
      ++it;
    }
  }

  /* If !found then the transition t was present in
   * states[t.source].fwd_transitions, but not in
   * states[t.target].bwd_transitions. This violates the invariant of
   * Automaton. Also it causes a memory leak in this method.
   */
  assert(found);

  return true;
};

int Automaton::get_transition_count() const{
  int tc = 0;
  for(unsigned i = 0; i < states.size(); ++i){
    tc += states[i].fwd_transitions.size();
  }
  return tc;
}

bool Automaton::same_automaton(const Automaton &a2, bool cmp_pos) const{
  if(states.size() != a2.states.size()) return false;

  class state_t{
  public:
    state_t(const Automaton *a1, const Automaton *a2, bool cmp_pos) : a1(a1), a2(a2), cmp_pos(cmp_pos) {
      std::set<int> all;
      for(int q = 0; q < int(a1->states.size()); ++q){
        all.insert(q);
      }
      qmap.resize(a1->states.size(),all);
      limit_to(0,0);
      label_limit();
      edge_limit();
      propagate();
    };
    const Automaton *a1;
    const Automaton *a2;
    bool cmp_pos;
    /* Maps control states q in a1 to sets of control states in a2
     * that are currently considered possible matches for q. */
    std::vector<std::set<int> > qmap;
    std::vector<int> prop_stack;
    void propagate(){
      while(prop_stack.size()){
        int q = prop_stack.back();
        const State &qstate = a1->states[q];
        prop_stack.pop_back();
        assert(qmap[q].size() <= 1);
        if(qmap[q].size() == 0){
          fail();
          return;
        }else{
          int q2 = *qmap[q].begin();
          const State &qstate2 = a2->states[q2];
          /* Eliminate the state q2 from all other qmap[q'] */
          for(int qq = 0; qq < int(qmap.size()); ++qq){
            if(qq != q){
              limit_rem(qq,q2);
            }
          }

          /* Consider in- and out-going transitions to limit other
           * state maps. */
          for(auto it = qstate.bwd_transitions.begin(); it != qstate.bwd_transitions.end(); ++it){
            int src = (*it)->source;
            /* Find possibilities for src2 */
            std::set<int> newqmap;
            for(auto it2 = qstate2.bwd_transitions.begin(); it2 != qstate2.bwd_transitions.end(); ++it2){
              if(stmt_eq((*it)->instruction,(*it2)->instruction)){
                newqmap.insert((*it2)->source);
              }
            }
            limit_to(src,newqmap);
          }
          for(auto it = qstate.fwd_transitions.begin(); it != qstate.fwd_transitions.end(); ++it){
            int tgt = (*it)->target;
            /* Find possibilities for tgt2 */
            std::set<int> newqmap;
            for(auto it2 = qstate2.fwd_transitions.begin(); it2 != qstate2.fwd_transitions.end(); ++it2){
              if(stmt_eq((*it)->instruction,(*it2)->instruction)){
                newqmap.insert((*it2)->target);
              }
            }
            limit_to(tgt,newqmap);
          }
        }
      }
    };
    void fail(){
      qmap[0].clear();
    };
    void limit_to(int q, int q2){
      if(qmap[q].count(q2)){
        if(qmap[q].size() > 1){
          prop_stack.push_back(q);
        }
        qmap[q].clear();
        qmap[q].insert(q2);
      }else{
        fail();
        return;
      }
    };
    void limit_to(int q, const std::set<int> &q2s){
      /* Find the intersection of q2s and qmap[q] */
      std::set<int> q2sq;
      for(auto it = q2s.begin(); it != q2s.end(); ++it){
        if(qmap[q].count(*it)){
          q2sq.insert(*it);
        }
      }

      /* Set qmap[q] = q2sq */
      if(qmap[q] != q2sq){
        assert(q2sq.size() < qmap[q].size());
        qmap[q] = q2sq;
        if(q2sq.size() == 0){
          fail();
        }else if(q2sq.size() == 1){
          prop_stack.push_back(q);
        }
      }
    };
    void limit_rem(int q, int q2){
      if(qmap[q].count(q2)){
        qmap[q].erase(q2);
        if(qmap[q].size() == 0){
          fail();
        }else if(qmap[q].size() == 1){
          prop_stack.push_back(q);
        }
      }
    }
    bool stmt_eq(const Lang::Stmt<int> &a, const Lang::Stmt<int> &b) const{
      return a.compare(b,cmp_pos) == 0;
    };
    bool same_instrs(const std::set<Transition*> &s0, const std::set<Transition*> &s1){
      if(s0.size() != s1.size()) return false;

      /* Compare the number of occurrences of each statement in
       * transitions in s0 and s1 */
      std::set<Transition*> s1cpy = s1;
      for(auto it = s0.begin(); it != s0.end(); ++it){
        bool found = false;
        for(auto it2 = s1cpy.begin(); !found && it2 != s1cpy.end(); ++it2){
          if(stmt_eq((*it)->instruction,(*it2)->instruction)){
            s1cpy.erase(it2);
            found = true;
          }
        }
        if(!found){
          return false;
        }
      }
      assert(s1cpy.empty());
      return true;
    }
    void edge_limit(){
      for(int q = 0; q < int(qmap.size()); ++q){
        std::set<int> newqmap;
        for(auto it = qmap[q].begin(); it != qmap[q].end(); ++it){
          int q2 = *it;
          const State &qstate = a1->states[q];
          const State &q2state = a2->states[q2];
          if(same_instrs(qstate.bwd_transitions,q2state.bwd_transitions) &&
             same_instrs(qstate.fwd_transitions,q2state.fwd_transitions)){
            newqmap.insert(q2);
          }
        }
        limit_to(q,newqmap);
      }
    };
    void label_limit(){
      /* Check that both automata have the same labels */
      if(a1->label_map.size() != a2->label_map.size()){
        fail();
        return;
      }
      for(auto it = a1->label_map.begin(); it != a1->label_map.end(); ++it){
        if(a2->label_map.count(it->first) == 0){
          fail();
          return;
        }
      }
      /* Limit the control state mapping */
      for(auto it = a1->label_map.begin(); it != a1->label_map.end(); ++it){
        int q = it->second;
        assert(a2->label_map.count(it->first));
        int q2 = a2->label_map.at(it->first);
        limit_to(q,q2);
      }
    };
    std::string to_string() const{
      std::stringstream ss;
      for(unsigned i = 0; i < a1->states.size(); ++i){
        ss << "Q" << i << ": {";
        for(auto it = qmap[i].begin(); it != qmap[i].end(); ++it){
          if(it != qmap[i].begin()) ss << ", ";
          ss << "q" << *it;
        }
        ss << "}\n";
      }
      return ss.str();
    };
    enum search_t {
      SEARCH, // Need to search
      SUCCESS, // The automata are equal
      FAILURE // The automata are different
    };
    search_t search(int *searchvar,std::set<int> *A, std::set<int> *B) const{
      for(int q = 0; q < int(qmap.size()); ++q){
        if(qmap[q].empty()){
          return FAILURE;
        }else if(qmap[q].size() > 1){
          // Pick this q for search variable
          *searchvar = q;
          // Divide the search space
          // Let the first set have one option
          A->clear();
          A->insert(*qmap[q].begin());
          // Let the second set have all other options
          *B = qmap[q];
          B->erase(*qmap[q].begin());
          return SEARCH;
        }
      }
      return SUCCESS;
    };
  };

  std::vector<state_t> stack;
  stack.push_back(state_t(this,&a2,cmp_pos));
  while(stack.size()){
    state_t &st = stack.back();
    int searchvar;
    std::set<int> A, B;
    switch(st.search(&searchvar,&A,&B)){
    case state_t::FAILURE:
      // Try the next search branch
      stack.pop_back();
      break;
    case state_t::SUCCESS:
      return true;
    case state_t::SEARCH:
      // Split st into two states
      state_t st2 = st;
      st.limit_to(searchvar,A);
      st.propagate();
      st2.limit_to(searchvar,B);
      st2.propagate();
      stack.push_back(st2);
      break;
    }
  }

  return false; // All search branches failed
};

void Automaton::test(){
  /* Test same_automaton */
  {
    std::function<Automaton(std::string)> auto_stmt =
      [](std::string s){
      std::stringstream ss;
      ss << "forbidden\n"
      << "  *\n"
      << "data\n"
      << "  s = *\n"
      << "  t = *\n"
      << "  u = *\n"
      << "  v = *\n"
      << "  w = *\n"
      << "  x = *\n"
      << "  y = *\n"
      << "  z = *\n"
      << "process\n"
      << "registers\n"
      << "  $r0 = *\n"
      << "  $r1 = *\n"
      << "  $r2 = *\n"
      << "  $r3 = *\n"
      << "  $r4 = *\n"
      << "text\n"
      << s;
      Lexer lex(ss);
      Machine m(Parser::p_test(lex));
      return m.automata[0];
    };

    std::function<bool(std::string,std::string,bool)> tst =
      [&auto_stmt](std::string A, std::string B, bool expected_res){
      Automaton a = auto_stmt(A);
      Automaton b = auto_stmt(B);
      return a.same_automaton(b,false) == expected_res;
    };

    /* Test labels */
    Test::inner_test("same_automaton #1",
                     tst("write: x := 1; A: write: y := 2; write: z := 3",
                         "write: x := 1; A: write: y := 2; write: z := 3",true));
    Test::inner_test("same_automaton #2",
                     tst("write: x := 1; A: write: y := 2; write: z := 3",
                         "write: x := 1; write: y := 2; A: write: z := 3",false));
    Test::inner_test("same_automaton #3",
                     tst("write: x := 1; A: write: y := 2; write: z := 3",
                         "write: x := 1; A: write: y := 2; B: write: z := 3",false));
    Test::inner_test("same_automaton #4",
                     tst("write: x := 1; A: write: y := 2; write: z := 3",
                         "write: x := 1; write: y := 2; write: z := 3",false));

    /* Test searching */
    Test::inner_test("same_automaton #5",
                     tst("either{write: x := 1; nop or write: x := 1; nop}",
                         "either{write: x := 1; nop or write: x := 1; nop}",true));
    Test::inner_test("same_automaton #6",
                     tst("either{write: x := 1; nop; nop or write: x := 1; nop; nop; nop}",
                         "either{write: x := 1; nop; nop or write: x := 1; nop; nop; nop}",true));
    Test::inner_test("same_automaton #7",
                     tst("either{write: x := 1; A: nop; nop or write: x := 1; nop; nop; nop}",
                         "either{write: x := 1; nop; nop or write: x := 1; A: nop; nop; nop}",false));

    /* Various */
    Test::inner_test("same_automaton #8",
                     tst(std::string("write: x := 1; write: x := 1; either{write: x := 1 or write: y := 2}; ")+
                         "write: x := 1; either{write: x := 1 or write: y := 2}; write: x := 1; write: x := 1",
                         std::string("write: x := 1; write: x := 1; either{write: x := 1 or write: y := 2}; ")+
                         "write: x := 1; either{write: x := 1 or write: y := 2}; write: x := 1; write: x := 1",true));
    Test::inner_test("same_automaton #9",
                     tst("nop","nop",true));
    Test::inner_test("same_automaton #10",
                     tst("$r0 := 0","$r0 := 1",false));
    Test::inner_test("same_automaton #11",
                     /* Disconnected parts of automata */
                     tst("nop; L0: $r0 := 0; $r0 := 1; $r0 := 2; goto L0; L1: $r1 := 0; $r1 := 1; $r1 := 2; goto L1",
                         "goto L0; L1: $r1 := 0; $r1 := 1; $r1 := 2; goto L1; L0: $r0 := 0; $r0 := 1; $r0 := 2; goto L0",true));
    Test::inner_test("same_automaton #12",
                     /* Test obscure gotos */
                     tst("goto L0; goto L1; $r1 := 1; L0: $r0 := 0; L1: $r2 := 2",
                         "nop; goto L0; $r1 := 1; L0: $r0 := 0; L1: $r2 := 2",true));
    Test::inner_test("same_automaton #13",
                     /* More obscure gotos */
                     tst("L0: $r0:=0; goto L0; $r1:=1; goto L1; goto L0; $r2:=2; L1: $r3:=3",
                         "L0: $r0:=0; goto L0; $r2:=2; goto L1; $r1:=1; L1: $r3:=3",true));

  }
};
