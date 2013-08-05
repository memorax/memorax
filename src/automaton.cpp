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
  case Lang::LOCKED: case Lang::SLOCKED:
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

      if(source >= 0 && source != i){ // Insert a nop before goto
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
  case Lang::UPDATE:
    throw new std::logic_error("Automaton: \"Update\" instruction in AST.");
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

int Automaton::get_transition_count() const{
  int tc = 0;
  for(unsigned i = 0; i < states.size(); ++i){
    tc += states[i].fwd_transitions.size();
  }
  return tc;
}
