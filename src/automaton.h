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

#ifndef __AUTOMATON_H__
#define __AUTOMATON_H__

#include <iostream>
#include <vector>
#include <map>
#include <utility>
#include <memory>
#include <stdexcept>
#include <list>
#include "lang.h"

/* An automaton representing the program of one process. */
class Automaton {
public:
  class Exception : public std::exception{
  public:
    Exception(std::string m) : msg(m) {};
    virtual ~Exception() throw() {};
    virtual const char *what() const throw() { return msg.c_str(); };
  private:
    std::string msg;
  };
  class LabeledGoto : public Exception{
  public:
    LabeledGoto(std::string m) : Exception(m) {};
    virtual ~LabeledGoto() throw() {};
    virtual const char *what() const throw() {
      std::string s = "LabeledGoto: ";
      return (s+Exception::what()).c_str();
    };
  };
  class UnDefinedLabel : public Exception{
  public:
    UnDefinedLabel(std::string m) : Exception(m) {};
    virtual ~UnDefinedLabel() throw() {};
    virtual const char *what() const throw() {
      std::string s = "UnDefinedLabel: ";
      return (s+Exception::what()).c_str();
    };
  };
  class RedefinedLabel : public Exception{
  public:
    RedefinedLabel(std::string m) : Exception(m) {};
    virtual ~RedefinedLabel() throw() {};
    virtual const char *what() const throw() {
      std::string s = "RedefinedLabel: ";
      return (s+Exception::what()).c_str();
    };
  };
  /* An automaton with a single state (0) and no transitions. */
  Automaton();
  Automaton(const Lang::Stmt<int>&);
  Automaton(const Automaton&); // Deep copy
  virtual ~Automaton();
  Automaton &operator=(const Automaton&); // Deep copy
  std::string to_string(const std::function<std::string(const int&)> &regts, 
                        const std::function<std::string(const Lang::MemLoc<int> &)> &mlts,
                        int indentation = 0) const throw();
  std::string to_dot(const std::function<std::string(const int&)> &regts, 
                     const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const throw(); // A graphviz representation usable with dot
  struct Transition{
    Transition(int s,const Lang::Stmt<int> &i,int t) :
      source(s), instruction(i), target(t) {};
    virtual ~Transition() {};
    virtual bool operator==(const Transition &t) const throw(){ return compare(t) == 0; };
    virtual bool operator<(const Transition &t) const throw(){ return compare(t) < 0; };
    virtual bool operator>(const Transition &t) const throw(){ return compare(t) > 0; }
    virtual int compare(const Transition &t) const throw(){
      if(source < t.source){
        return -1;
      }else if(source > t.source){
        return 1;
      }else if(target < t.target){
        return -1;
      }else if(target > t.target){
        return 1;
      }else{
        return instruction.compare(t.instruction);
      }
    };
    int source; // Source state index
    Lang::Stmt<int> instruction;
    int target; // Target state index
    virtual std::string to_string(const std::function<std::string(const int&)> &regts, 
                                  const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const throw();
  };
  struct State{
    State() : 
      fwd_transitions(std::set<Transition*>()),
      bwd_transitions(std::set<Transition*>()) {};
    /* Transitions leading from this state.
     * fwd_transitions owns the transitions it points to. */
    std::set<Transition*> fwd_transitions;
    /* Transitions leading to this state.
     * bwd_transitions does *not* own the transitions it points to.
     * The transitions are shared with fwd_transitions. */
    std::set<Transition*> bwd_transitions;
  };
  const std::vector<State> &get_states() const throw() { return states; };
  std::vector<State> &get_states() throw() { return states; };
  const State &operator[](int i) const throw(std::out_of_range) { return states.at(i); };
  /* Returns the state labeled by lbl if there is such a
   * label. */
  int state_index_of_label(Lang::label_t lbl) const throw(UnDefinedLabel*);
  const std::map<Lang::label_t,int> &get_labels() const throw() { return label_map; };
  /* Returns a list with distinct elements ml such that this automaton
   * contains a transition writing to the memory location ml.
   */
  std::list<Lang::MemLoc<int> > get_possible_writes() const;
  /* If there is a transition identical to t in this automaton, then
   * false is returned. Otherwise t is added to this automaton and
   * true is returned. If t.source or t.target do not exist, then
   * those states are created.
   */
  bool add_transition(const Transition &t);
  /* Sets the label lbl to the state i. If lbl was previously present
   * as a label in this automaton, the old label is removed. */
  void set_label(Lang::label_t lbl, int i) { label_map[lbl] = i; };
  /* Returns the total number of transitions in this automaton. */
  int get_transition_count() const;
private:
  void clear_and_copy(const Automaton &); // Used for copy constructor and assignment
  /* Throw Exception if fwd and bwd transitions are inconsistent */
  void check_fwd_bwd_consistency() const throw(Exception*);
  /* The states of the automaton, labeled by their indices.
   * Execution starts in the state states[0].
   */
  std::vector<State> states;
  std::map<Lang::label_t,int> label_map; // Maps labels to state indices
  // Recursive function, used in constructor
  typedef std::set<std::pair<std::set<Transition *>,Lang::label_t> > unsat_goto_t;
  class ConstructFromAst;
  friend class ConstructFromAst;
  void construct_from_ast(const Lang::Stmt<int>&,unsat_goto_t&,int src = -1);
  void dealloc(); // Delete all instructions owned by this automaton
};

#endif // __AUTOMATON_H__
