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

#ifndef __MACHINE_H__
#define __MACHINE_H__

#include "automaton.h"
#include "parser.h"
#include "lang.h"
#include "predicates.h"
#include <vector>
#include <sstream>
#include "vecset.h"

class Machine{
public:
  class Exception : public std::exception{};
  class VariableUnDeclared : public Exception{
  public:
    VariableUnDeclared(Lang::MemLoc<std::string> v) : var(v) {};
    virtual ~VariableUnDeclared() throw() {};
    const char *what() const throw() {
      return ("VariableUnDeclared: Variable '"+
              var.to_string()+"' used without declaration.").c_str();
    };
  private:
    Lang::MemLoc<std::string> var;
  };
  class RegisterUnDeclared : public Exception{
  public:
    RegisterUnDeclared(std::string r) : reg(r) {};
    virtual ~RegisterUnDeclared() throw() {};
    const char *what() const throw(){
      return ("RegisterUnDeclared: Register '"+reg+"' used without declaration.").c_str();
    };
  private:
    std::string reg;
  };
  class PointerOutOfRange : public Exception{
  public:
    PointerOutOfRange(int p) : pointer(p) {};
    virtual ~PointerOutOfRange() throw() {};
    const char *what() const throw() {
      std::stringstream ss;
      ss << "PointerOutOfRange: Literal pointer @" << 
        pointer << " points outside global memory.";
      return ss.str().c_str();
    };
  private:
    int pointer;
  };
  class InvalidPid : public Exception{
  public:
    InvalidPid(std::string m, int p) : pid(p),msg(m) {};
    virtual ~InvalidPid() throw() {};
    virtual const char *what() const throw() {
      std::stringstream ss;
      ss << "InvalidPid(" << pid << "): " << msg;
      return ss.str().c_str();
    }
    int pid;
  private:
    std::string msg;
  };
  class ErroneousForbidden : public Exception{
  public:
    ErroneousForbidden(std::string m) : msg(m) {};
    virtual ~ErroneousForbidden() throw() {};
    virtual const char *what() const throw(){
      return ("ErroneousForbidden: "+msg).c_str();
    };
  private:
    std::string msg;
  };
  Machine(const Parser::Test &);
  Machine(const Machine&); // Deep copy
  virtual ~Machine() throw();

  /* A transition augmented with a pid identifying the process
   * performing the transition.
   */
  class PTransition : public Automaton::Transition{
  public:
    PTransition(const Automaton::Transition &t, int pid) : 
      Automaton::Transition(t), pid(pid) {};
    PTransition(int src, const Lang::Stmt<int> &i, int tgt, int pid) :
      Automaton::Transition(src,i,tgt), pid(pid) {};
    PTransition(const PTransition &pt) :
      Automaton::Transition(pt), pid(pt.pid) {};
    int pid; // The process that performs the transition

    virtual PTransition &operator=(const PTransition &pt){
      if(&pt != this){
        Automaton::Transition::operator=(pt);
        pid = pt.pid;
      }
      return *this;
    };

    virtual bool operator==(const PTransition &pt) const throw(){
      return Automaton::Transition::operator==(pt) && pid == pt.pid;
    };

    virtual bool operator<(const PTransition &pt) const throw(){
      return (pid < pt.pid) ||
        (pid == pt.pid && Automaton::Transition::operator<(pt));
    };
    virtual bool operator>(const PTransition &pt) const throw(){
      return (pid > pt.pid) ||
        (pid == pt.pid && Automaton::Transition::operator>(pt));
    };
    int compare(const PTransition &pt) const throw(){
      if(pid < pt.pid){
        return -1;
      }else if(pid > pt.pid){
        return 1;
      }else{
        return Automaton::Transition::compare(pt);
      }
    }

    /* Describes the style in which a PTransition is written as a
     * string. */
    enum str_style_t {
      SS_CONTROL_STATES, // Write as "Pp: (src,instr,tgt)"
      SS_LINE_NUMBERS    // Write as "Lln: Pp: instr"
    };
    virtual std::string to_string(const std::function<std::string(const int&)> &regts, 
                                  const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const throw(){
      return to_string(regts,mlts,SS_LINE_NUMBERS);
    };
    virtual std::string to_raw_string(str_style_t style = SS_CONTROL_STATES) const throw(){
      return to_string(Lang::int_reg_to_string(),[](const Lang::MemLoc<int> &ml){ return ml.to_string(); }, style);
    };
    virtual std::string to_string(const Machine &m, str_style_t style = SS_LINE_NUMBERS) const throw(){
      return to_string(m.reg_pretty_vts(pid),m.ml_pretty_vts(pid),style);
    };
    virtual std::string to_string(const std::function<std::string(const int&)> &regts, 
                                  const std::function<std::string(const Lang::MemLoc<int> &)> &mlts,
                                  str_style_t style) const throw(){
      std::stringstream ss;
      switch(style){
      case SS_CONTROL_STATES:
        ss << "P" << pid << ": " << Automaton::Transition::to_string(regts,mlts);
        break;
      case SS_LINE_NUMBERS:
        if(instruction.get_pos().get_line_no() >= 0){
          ss << instruction.get_pos().to_short_line_string() << " ";
        }// Else no line number given, write nothing
        ss << "P" << pid << ": " << instruction.to_string(regts,mlts);
        break;
      }
      return ss.str();
    };
  };

  std::string to_string() const throw();
  std::string to_dot() const throw(); // A graphviz representation usable with dot
  int proc_count() const throw() { return automata.size(); };
  int glob_var_count() const throw() { return gvars.size(); };
  int var_count() const throw(); // Number of all variables, global and local
  int var_count(int pid) const throw() { return lvars[pid].size(); };
  Lang::VarDecl get_declaration(const Lang::NML &nml) const;
  Lang::VarDecl get_declaration(const Lang::MemLoc<int> &ml, int pid) const;
  Lang::VarDecl get_declaration(int reg, int pid) const;
  Machine &operator=(const Machine&); // Deep copy
  /* Returns a list with distinct elements (p,ml) such that process
   * p has a transition that writes to the memory location (from the
   * perspective of p) ml. */
  const std::list<std::pair<int,Lang::MemLoc<int> > > &get_possible_writes() const;
  /* Returns the total number of transitions in the automata of this machine */
  int get_transition_count() const;

  /* Report an overapproximation of all possible happens-before cycles
   * in this machine.
   */
  std::list<std::list<PTransition> > find_cycles() const;

  std::function<std::string(const int&)> reg_pretty_vts(int pid) const{
    return [pid,this](const int &r){
      return this->pretty_string_reg.at(std::pair<int,int>(r,pid));
    };
  };

  std::function<std::string(const Lang::MemLoc<int>&)> ml_pretty_vts(int pid) const{
    return [pid,this](const Lang::MemLoc<int> &ml){
      return this->pretty_string_nml.at(Lang::NML(ml,pid));
    };
  };

  /* Returns a new machine where all usage of registers have been
   * removed. Instead the values of registers have been encoded into
   * control states and registers have been replaced by integer
   * literals in transitions.
   *
   * Pre: All registers have finite domains.
   */
  Machine *remove_registers() const;

  /* Returns a new machine where unnecessary nops have been removed.
   * Self-looping read-asserts are also removed (since they do nothing).
   */
  Machine *remove_superfluous_nops() const;

  /* Returns a new machine where all states s have been removed from
   * all automata a, where there is no path from s to any state s' in
   * a such that forbidden[i][a] = s' for some i.
   *
   * In other words, removes all control states from which no
   * forbidden state can be reached by any path (possible or
   * otherwise).
   */
  Machine *forbidden_shave() const;

  /* Returns a machine that has precisely the same behaviour as this
   * one. The returned machine is augmented with assume statements
   * that ensure that no instruction violates the domain of a memory
   * location or register.
   */
  Machine *add_domain_assumes() const;

  /* Returns a new machine where every transition "locked write..." has been
   * converted into two transitions "write..." followed by "mfence" and
   * similarly for transitions "slocked write...".
   *
   * Locked blocks containing a sequence of a read and a write are left as-is.
   * Other contents of locked blocks are not allowed */
  Machine *convert_locks_to_fences() const;

  const Lang::VarDecl &get_var_decl(const Lang::NML &nml) const{
    if(nml.is_global()){
      return gvars[nml.get_id()];
    }else{
      return lvars[nml.get_owner()][nml.get_id()];
    }
  };

  /*****************************************/
  /*                  Data                 */
  /*****************************************/
  /* Indexed by PIDs. */
  std::vector<Automaton> automata;

  /* lvars[pid][var] - declaration of variable var in process pid. */
  std::vector<std::vector<Lang::VarDecl> > lvars;
  /* gvars[var] - declaration of global variable var. */
  std::vector<Lang::VarDecl> gvars;
  /* regs[pid][reg] - declaration of register reg in process pid. */
  std::vector<std::vector<Lang::VarDecl> > regs;
  /* Each element v in forbidden specifies a forbidden combination of
   * program locations. The combination is that where each process p
   * is at state v[p]. */
  std::vector<std::vector<int> > forbidden;
  /* Contains predicates given together with the definition of the
   * machine (usually as a part of the .rmm file). The vector
   * predicates is non-empty if there were any predicates given. All
   * predicates in predicates are nullary (generalised).
   */
  std::vector<Predicates::Predicate<Predicates::DummyVar> > predicates;
  /* Maps NMLs to names that are based on the names in the original
   * rmm-file, and hopefully easier to understand for the user than
   * the output of NML::to_string.
   */
  std::map<Lang::NML,std::string> pretty_string_nml;
  /* Analoguely to pretty_string_nml, maps (reg,pid) to a string
   * representation of register reg of process pid.
   */
  std::map<std::pair<int,int>,std::string> pretty_string_reg;
protected:
private:

  /* Initializes this->forbidden from fb. 
   * Pre: this->automata is fully populated. */
  void init_forbidden(const Parser::forbidden_t &fb) 
    throw(Automaton::UnDefinedLabel*,ErroneousForbidden*);

  /* Helper for remove_registers() */
  std::vector<std::pair<Lang::Stmt<int>, std::vector<int> > >
  remove_registers(const Lang::Stmt<int> &stmt,
                   const std::vector<int> &v,
                   int pid) const;

  /* Investigates in which control states for process pid the value of
   * each register for process pid may be relevant. 
   *
   * Returns a vector v, such that v[pc][reg] is true iff there is a
   * path for process pid from control state pc to a transition that
   * uses the value of register reg, such that the path does not
   * contain any transition that is guaranteed to overwrite the value
   * of register reg.
   */
  std::vector<std::vector<bool> > get_reg_relevant(int pid) const;
  /* Investigates the effect of stmt on register reg.
   *
   * If there may be a way to execute stmt such that stmt reads the
   * value of reg, without first overwriting it, then *may_read is
   * assigned true, otherwise false.
   *
   * If all ways to execute stmt will overwrite the value of reg, then
   * *overwrites is assigned true, otherwise false.
   */
  void get_reg_relevant_aux(int reg, const Lang::Stmt<int> &stmt,
                            bool *may_read, bool *overwrites) const;

  /* Changes forbidden such that each vector f in forbidden is
   * replaced by a vector f' per element q' in m[f[pid]]. f' is
   * identical to f, except that f'[pid] == q'.
   *
   * Pre: For all vectors f in forbidden, m[f[pid]] is defined.
   */
  void update_forbidden(const std::vector<VecSet<int> > &m, int pid);

  /* Helper to remove_superfluous_nops()
   * This function will be executed until a fix-point is reached. */
  Machine *remove_superfluous_nops_one_pass() const;

  /* Helper to add_domain_assumes()
   *
   * Returns the sequence of statements that should replace the
   * statement s occuring in a transition. The returned sequence will
   * contain s and also assume statements as necessary to ensure that
   * s does not violate any domains when executing.
   */
  std::vector<Lang::Stmt<int> > add_domain_assumes(const Lang::Stmt<int> &s, int pid) const;

  /* Helper to add_domain_assumes()
   *
   * Tries to assert whether the expression e, when executed by
   * process pid is guaranteed to have a value within dom. This method
   * is conservative: If it returns true then e is guaranteed to have
   * a value in dom. If it returns false then it is possible that e
   * still has a value within dom.
   */
  bool expr_always_in_domain(const Lang::Expr<int> &e, int pid, const Lang::VarDecl::Domain &dom) const;

  /* Helper to convert_locks_to_fences()
   * This performs the conversion on a single automaton. */
  void convert_locks_to_fences(Automaton &) const;

};

inline std::ostream &operator<<(std::ostream &os,const Machine &m){
  return os << m.to_string();
};

#endif // __MACHINE_H__
