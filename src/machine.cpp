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

#include "machine.h"
#include <utility>
#include <cassert>
#include <queue>

Machine::Machine(const Parser::Test &test){
  class MLC{
  public:
    MLC(const Parser::Test &t, int p,std::function<int(const std::string&)> &rc) : test(t), pid(p), regc(rc) {};
    virtual Lang::MemLoc<int> operator()(const Lang::MemLoc<std::string> &ml){
      switch(ml.get_type()){
      case Lang::MemLoc<std::string>::GLOBAL_ID:
        return Lang::MemLoc<int>::global(get_index(test.global_vars,ml.get_id(),ml));
      case Lang::MemLoc<std::string>::GLOBAL_INT_DEREF:
        if(unsigned(ml.get_pointer()) >= test.global_vars.size())
          throw new PointerOutOfRange(ml.get_pointer());
        return Lang::MemLoc<int>::global(ml.get_pointer());
      case Lang::MemLoc<std::string>::GLOBAL_REG_DEREF:
        return Lang::MemLoc<int>::reg_deref(regc(ml.get_reg()));
      case Lang::MemLoc<std::string>::LOCAL:
        {
          int p = ml.get_owner(pid);
          if(p < 0 || p >= int(test.processes.size())){
            std::stringstream ss;
            ss << "Invalid process id " << p << " in memory location " << ml.to_string();
            throw new InvalidPid(ss.str(),p);
          }
          int index = get_index(test.processes[p].vars,ml.get_id(),ml);
          if(p == pid){
            return Lang::MemLoc<int>::local(index);
          }else{
            int p2 = p;
            if(p2 >= pid) p2--;
            return Lang::MemLoc<int>::local(index,p2);
          }
        }
      default:
        throw new std::logic_error("Machine::MLC: Invalid value for MemLoc type.");
      }
    };
  private:
    int get_index(const std::vector<Lang::VarDecl> &vec,
                  std::string key,
                  Lang::MemLoc<std::string> ml){
      unsigned i;
      for(i = 0; i < vec.size(); i++)
        if(vec[i].name == key)
          return int(i);
      throw new VariableUnDeclared(ml);
    };
    const Parser::Test &test;
    int pid;
    std::function<int(const std::string&)> &regc;
  };

  for(unsigned i = 0; i < test.global_vars.size(); i++){
    gvars.push_back(test.global_vars[i]);
    pretty_string_nml[Lang::NML::global(i)] = test.global_vars[i].name;
  }
  
  for(int pid = 0; pid < int(test.processes.size()); pid++){
    std::function<int(const std::string&)> regc = 
      [&test,pid](const std::string &r)->int{
      for(unsigned i = 0; i < test.processes[pid].regs.size(); i++){
        if(test.processes[pid].regs[i].name == r){
          return i;
        }
      }
      return -1;
    };
    MLC mlc(test,pid,regc);
    std::function<Lang::MemLoc<int>(const Lang::MemLoc<std::string>&)> mlcf = 
      [&mlc](const Lang::MemLoc<std::string> &ml){
      return mlc(ml);
    };

    /* Check that all registers used in the code are declared */
    std::set<std::string> used_regs = test.processes[pid].get_code().get_registers();
    for(auto it = used_regs.begin(); it != used_regs.end(); ++it){
      if(regc(*it) < 0){
        for(unsigned i = 0; i < test.processes[pid].regs.size(); ++i){
          std::cout << "Declared: " << test.processes[pid].regs[i].name << "\n";
        }
        throw new RegisterUnDeclared(*it);
      }
    }

    automata.push_back(Automaton(test.processes[pid].get_code().convert(regc,mlcf)));

    for(unsigned i = 0; i < test.processes[pid].regs.size(); i++){
      pretty_string_reg[std::pair<int,int>(i,pid)] = test.processes[pid].regs[i].name;
    }
    
    for(unsigned i = 0; i < test.processes[pid].vars.size(); i++){
      std::stringstream ss;
      ss << test.processes[pid].vars[i].name << "[P" << pid << "]";
      pretty_string_nml[Lang::NML::local(i,pid)] = ss.str();
    }
    lvars.push_back(test.processes[pid].vars);
    regs.push_back(test.processes[pid].regs);
  }

  init_forbidden(test.forbidden);
  predicates = test.predicates;

}

Machine::Machine(const Machine &m){
  automata = m.automata;
  lvars = m.lvars;
  gvars = m.gvars;
  regs = m.regs;
  forbidden = m.forbidden;
  predicates = m.predicates;
  pretty_string_nml = m.pretty_string_nml;
  pretty_string_reg = m.pretty_string_reg;
}

Machine &Machine::operator=(const Machine &m){
  automata = m.automata;
  lvars = m.lvars;
  gvars = m.gvars;
  regs = m.regs;
  forbidden = m.forbidden;
  pretty_string_nml = m.pretty_string_nml;
  pretty_string_reg = m.pretty_string_reg;
  return *this;
}

Machine::~Machine() throw(){
}

int Machine::var_count() const throw(){
  int ack = glob_var_count();
  for(unsigned i = 0; i < lvars.size(); i++)
    ack += lvars[i].size();
  return ack;
}

std::string Machine::to_string() const throw(){
  std::stringstream ss;

  for(unsigned i = 0; i < gvars.size(); i++)
    ss << "var:" << i << " = " << gvars[i].value.to_string() << " : " << gvars[i].domain.to_string() << "\n";

  for(unsigned i = 0; i < automata.size(); i++){
    ss << "process\n";
    if(lvars[i].size()){
      ss << "data\n";
      for(unsigned j = 0; j < lvars[i].size(); j++)
        ss << "  var:" << j << " = " << lvars[i][j].value.to_string() << " : " << lvars[i][j].domain.to_string() << "\n";
    }
    ss << "text\n";
    ss << automata[i].to_string(reg_pretty_vts(i),ml_pretty_vts(i),2);
  }

  return ss.str();
}

std::string Machine::to_dot() const throw(){
  std::string ack;
  ack = "digraph machine{\n";
  for(unsigned i = 0; i < automata.size(); i++)
    ack += automata[i].to_dot(reg_pretty_vts(i),ml_pretty_vts(i)) + "\n\n";
  ack += "}";
  return ack;
}


void Machine::init_forbidden(const Parser::forbidden_t &fb) 
  throw(Automaton::UnDefinedLabel*,ErroneousForbidden*){

  if(fb.size() == 0){
    throw new ErroneousForbidden("No forbidden state specified.");
  }

  std::list<std::vector<int> > tmp_fb;
  for(unsigned i = 0; i < fb.size(); i++){
    if(int(fb[i].size()) != proc_count()){
      throw new ErroneousForbidden("Wrong number of labels in forbidden state. "
                                   "There should be one per process.");
    }
    std::vector<int> v(fb[i].size(),0);
    for(unsigned j = 0; j < fb[i].size(); j++){
      if(fb[i][j].first){
        v[j] = -1;
      }else{
        try{
          v[j] = automata[j].state_index_of_label(fb[i][j].second);
        }catch(Automaton::UnDefinedLabel *exc){
          std::string m = exc->what();
          delete exc;
          throw new ErroneousForbidden(m+" in definition of forbidden state.");
        }
        assert(v[j] >= 0);
      }
    }
    tmp_fb.push_back(v);
  }

  for(int p = 0; p < proc_count(); p++){
    for(std::list<std::vector<int> >::iterator it = tmp_fb.begin(); it != tmp_fb.end(); it++ ){
      if((*it)[p] < 0){
        std::vector<int> v = *it;
        it = tmp_fb.erase(it);
        for(unsigned i = 0; i < automata[p].get_states().size(); i++){
          v[p] = i;
          tmp_fb.push_front(v);
        }
      }
    }
  }

  std::copy(tmp_fb.begin(),tmp_fb.end(),std::back_inserter(forbidden));

}

const std::list<std::pair<int,Lang::MemLoc<int> > > &Machine::get_possible_writes() const{
  static std::list<std::pair<int,Lang::MemLoc<int> > > writes; // Cache result
  static bool computed = false; // Cache result
  if(!computed){
    for(int p = 0; p < proc_count(); p++){
      std::list<Lang::MemLoc<int> > mls = automata[p].get_possible_writes();
      for(std::list<Lang::MemLoc<int> >::iterator it = mls.begin(); it != mls.end(); it++){
        writes.push_back(std::pair<int,Lang::MemLoc<int> >(p,*it));
      }
    }
    computed = true;
  }
  return writes;
}

int Machine::get_transition_count() const{
  int tc = 0;
  for(unsigned i = 0; i < automata.size(); ++i){
    tc += automata[i].get_transition_count();
  }
  return tc;
};

std::list<std::list<Machine::PTransition> > Machine::find_cycles() const{

  throw new std::logic_error("Machine::find_cycles: Not implemented.");

  return std::list<std::list<PTransition> >();
};

Machine *Machine::remove_registers() const{
  Machine *m = new Machine(*this);
  m->pretty_string_reg.clear();

  Log::msg << "Transforming machine into register free form.\n";

  /* Make sure that all registers have finite domains */
  for(unsigned pid = 0; pid < regs.size(); ++pid){
    for(unsigned r = 0; r < regs[pid].size(); ++r){
      if(regs[pid][r].domain.is_int()){
        delete m;
        throw new std::logic_error("Cannot remove registers with infinite domain (P0."+regs[pid][r].name+")");
      }
    }
  }

  for(unsigned pid = 0; pid < automata.size(); ++pid){
    if(m->regs[pid].size()){
      const int rc = m->regs[pid].size();
      m->regs[pid].clear();
      m->automata[pid] = Automaton(); // Clear automata[pid]
      Automaton &new_atm = m->automata[pid];
      const Automaton &old_atm = automata[pid];

      /* Represent a state as an integer vector v
       * where v[rc] is the original control state,
       * and v[r] for all r is the value of register r.
       */
      const int v_size = rc+1;
      int next_cs = 0;
      std::vector<std::vector<int> > queue;
      std::map<std::vector<int>, int> new_cs; // Maps states to new control states
      // Maps old control states to new control states
      std::vector<VecSet<int> > old_cs_to_new_cs(old_atm.get_states().size());
      old_cs_to_new_cs[0].insert(0);
      int domain_size = 1;

      /* Investigate in which old control states the values of each register is relevant */
      /* reg_relevant[pc][reg] */
      std::vector<std::vector<bool> > reg_relevant = get_reg_relevant(pid);

      /* Compute the initial states */
      {
        std::vector<int> v(v_size);
        v[rc] = 0;
        queue.push_back(v);
        for(int r = 0; r < rc; ++r){
          domain_size *= regs[pid][r].domain.get_upper_bound() - regs[pid][r].domain.get_lower_bound() + 1;
          if(regs[pid][r].value.is_wild()){
            std::vector<std::vector<int> > q = queue;
            queue.clear();
            for(auto dit = regs[pid][r].domain.begin(); dit != regs[pid][r].domain.end(); ++dit){
              for(unsigned i = 0; i < q.size(); ++i){
                queue.push_back(q[i]);
                queue.back()[r] = *dit;
              }
            }
          }else{
            for(unsigned i = 0; i < queue.size(); ++i){
              queue[i][r] = regs[pid][r].value.get_value();
            }
          }
        }
        
        /* Canonize registers whose values are irrelevant */
        {
          VecSet<std::vector<int> > vs;
          while(queue.size()){
            for(int j = 0; j < rc; ++j){
              if(!reg_relevant[0][j]){
                queue.back()[j] = 0;
              }
            }
            vs.insert(queue.back());
            queue.pop_back();
          }
          for(int i = 0; i < vs.size(); ++i){
            queue.push_back(vs[i]);
          }
        }

        if(queue.size() == 1){
          new_cs[queue.back()] = next_cs++;
        }else{
          assert(queue.size() > 1);
          ++next_cs;
          for(unsigned i = 0; i < queue.size(); ++i){
            new_atm.add_transition(Automaton::Transition(0,Lang::Stmt<int>::nop(),next_cs));
            new_cs[queue[i]] = next_cs++;
          }
        }
      }
      
      /* Explore the automaton */
      const std::vector<Automaton::State> &old_states = old_atm.get_states();
      while(queue.size()){
        std::vector<int> v = queue.back();
        queue.pop_back();
        int pc = v[rc];
        for(auto trit = old_states[pc].fwd_transitions.begin(); trit != old_states[pc].fwd_transitions.end(); ++trit){
          auto vs = remove_registers((*trit)->instruction,v,pid);
          for(auto vit = vs.begin(); vit != vs.end(); ++vit){
            /* Canonize registers whose values are irrelevant */
            for(int i = 0; i < rc; ++i){
              if(!reg_relevant[(*trit)->target][i]){
                vit->second[i] = 0;
              }
            }
            vit->second[rc] = (*trit)->target;
            auto pcit = new_cs.find(vit->second);
            if(pcit == new_cs.end()){
              /* New state */
              new_atm.add_transition(Automaton::Transition(new_cs[v],vit->first,next_cs));
              old_cs_to_new_cs[(*trit)->target].insert(next_cs);
              new_cs[vit->second] = next_cs++;
              queue.push_back(vit->second);
            }else{
              /* Old state */
              new_atm.add_transition(Automaton::Transition(new_cs[v],vit->first,pcit->second));
            }
          }
        }
      }
      /* Resetup labels */
      const std::map<Lang::label_t,int> &lbls = old_atm.get_labels();
      for(auto it = lbls.begin(); it != lbls.end(); ++it){
        int old_pc = it->second;
        for(int i = 0; i < old_cs_to_new_cs[old_pc].size(); ++i){
          std::stringstream lbl;
          lbl << it->first;
          if(old_cs_to_new_cs[old_pc].size() > 1) lbl << "." << i;
          new_atm.set_label(lbl.str(),old_cs_to_new_cs[old_pc][i]);
        }
      }
      /* Resetup forbidden states */
      m->update_forbidden(old_cs_to_new_cs,pid);
    }

  }

  return m;
};


std::vector<std::pair<Lang::Stmt<int>, std::vector<int> > >
Machine::remove_registers(const Lang::Stmt<int> &stmt,
                          const std::vector<int> &v,
                          int pid) const{
  typedef std::pair<Lang::Stmt<int>, std::vector<int> > pr_t;
  VecSet<pr_t> s;
  Lexer::TokenPos pos = stmt.get_pos();

  switch(stmt.get_type()){
  case Lang::NOP: case Lang::GOTO: case Lang::UPDATE:
    s.insert(pr_t(stmt,v));
    break;
  case Lang::ASSIGNMENT:
    {
      int val = stmt.get_expr().eval<std::vector<int>,int*>(v,0);
      if(regs[pid][stmt.get_reg()].domain.member(val)){
        std::vector<int> v2 = v;
        v2[stmt.get_reg()] = val;
        s.insert(pr_t(Lang::Stmt<int>::nop(pos),v2));
      }
      break;
    }
  case Lang::ASSUME:
    if(stmt.get_condition().eval<std::vector<int>,int*>(v,0)){
      s.insert(pr_t(Lang::Stmt<int>::nop(pos),v));
    } // else insert nothing
    break;
  case Lang::READASSERT:
    {
      int val = stmt.get_expr().eval<std::vector<int>,int*>(v,0);
      if(get_var_decl(Lang::NML(stmt.get_memloc(),pid)).domain.member(val)){
        s.insert(pr_t(Lang::Stmt<int>::read_assert(stmt.get_memloc(),Lang::Expr<int>::integer(val),pos),v));
      }
      break;
    }
  case Lang::READASSIGN:
    {
      const Lang::VarDecl &vd = get_var_decl(Lang::NML(stmt.get_memloc(),pid));
      for(auto dit = regs[pid][stmt.get_reg()].domain.begin(); dit != regs[pid][stmt.get_reg()].domain.end(); ++dit){
        if(vd.domain.member(*dit)){
          std::vector<int> v2 = v;
          v2[stmt.get_reg()] = *dit;
          s.insert(pr_t(Lang::Stmt<int>::read_assert(stmt.get_memloc(),Lang::Expr<int>::integer(*dit),pos),v2));
        }
      }
      break;
    }
  case Lang::WRITE:
    {
      int val = stmt.get_expr().eval<std::vector<int>,int*>(v,0);
      if(get_var_decl(Lang::NML(stmt.get_memloc(),pid)).domain.member(val)){
        s.insert(pr_t(Lang::Stmt<int>::write(stmt.get_memloc(),Lang::Expr<int>::integer(val),pos),v));
      }
      break;
    }
  case Lang::SLOCKED:
  case Lang::LOCKED:
    {
      for(int i = 0; i < stmt.get_statement_count(); ++i){
        std::vector<pr_t> vs = remove_registers(*stmt.get_statement(i),v,pid);
        for(unsigned j = 0; j < vs.size(); ++j){
          std::vector<Lang::Stmt<int> > seq;
          seq.push_back(vs[j].first);
          if (stmt.get_type() == Lang::SLOCKED)
            s.insert(pr_t(Lang::Stmt<int>::slocked_block(seq,pos),vs[j].second));
          else
            s.insert(pr_t(Lang::Stmt<int>::locked_block(seq,pos),vs[j].second));
        }
      }
      break;
    }
  case Lang::SEQUENCE:
    {
      typedef std::pair<std::vector<Lang::Stmt<int> >, std::vector<int> > pr2_t;
      std::vector<pr2_t> s2;
      s2.push_back(pr2_t(std::vector<Lang::Stmt<int> >(), v));
      for(int i = 0; i < stmt.get_statement_count(); ++i){
        std::vector<pr2_t> s3;
        for(unsigned j = 0; j < s2.size(); ++j){
          std::vector<pr_t> s4 = remove_registers(*stmt.get_statement(i),s2[j].second,pid);
          for(auto sit = s4.begin(); sit != s4.end(); ++sit){
            s3.push_back(pr2_t(s2[j].first,sit->second));
            s3.back().first.push_back(sit->first);
          }
        }
        s2 = s3;
      }
      for(unsigned i = 0; i < s2.size(); ++i){
        std::vector<Lang::Stmt<int>::labeled_stmt_t> seq;
        for(unsigned j = 0; j < s2[i].first.size(); ++j){
          seq.push_back(Lang::Stmt<int>::labeled_stmt_t(s2[i].first[j]));
        }
        s.insert(pr_t(Lang::Stmt<int>::sequence(seq,pos),s2[i].second));
      }
      break;
    }
  default:
    throw new std::logic_error("Machine::remove_registers: Unsupported statement: "+
                               stmt.to_string(reg_pretty_vts(pid),
                                              ml_pretty_vts(pid)));
  }

  return s.get_vector();
};

std::vector<std::vector<bool> > Machine::get_reg_relevant(int pid) const{
  std::vector<std::vector<bool> > rr(automata[pid].get_states().size(),
                                     std::vector<bool>(regs[pid].size(),false));

  // Populate by fixpoint iteration
  bool changed = true;
  while(changed){
    changed = false;
    const std::vector<Automaton::State> &states = automata[pid].get_states();
    for(unsigned i = 0; i < states.size(); ++i){
      for(auto trit = states[i].fwd_transitions.begin(); trit != states[i].fwd_transitions.end(); ++trit){
        for(unsigned reg = 0; reg < regs[pid].size(); ++reg){
          if(!rr[i][reg]){
            bool may_read, overwrites;
            get_reg_relevant_aux(reg,(*trit)->instruction,&may_read,&overwrites);
            if(may_read || (!overwrites && rr[(*trit)->target][reg])){
              rr[i][reg] = true;
              changed = true;
            }
          }
        }
      }
    }
  }
  return rr;
};

void Machine::get_reg_relevant_aux(int reg, const Lang::Stmt<int> &stmt,
                                   bool *may_read, bool *overwrites) const{
  *may_read = false;
  *overwrites = false;
  switch(stmt.get_type()){
  case Lang::NOP: case Lang::GOTO: case Lang::UPDATE:
    return;
  case Lang::ASSIGNMENT:
    *may_read = stmt.get_expr().get_registers().count(reg);
    *overwrites = (stmt.get_reg() == reg);
    break;
  case Lang::ASSUME:
    *may_read = stmt.get_condition().get_registers().count(reg);
    break;
  case Lang::READASSERT:
    *may_read = stmt.get_expr().get_registers().count(reg);
    break;
  case Lang::READASSIGN:
    *overwrites = (stmt.get_reg() == reg);
    break;
  case Lang::WRITE:
    *may_read = stmt.get_expr().get_registers().count(reg);
    break;
  case Lang::SLOCKED:
  case Lang::LOCKED:
    {
      bool mr;
      bool ow;
      *overwrites = true;
      for(int i = 0; i < stmt.get_statement_count(); ++i){
        get_reg_relevant_aux(reg,*stmt.get_statement(i),&mr,&ow);
        *overwrites = *overwrites && ow;
        *may_read = *may_read || mr;
      }
      break;
    }
  case Lang::SEQUENCE:
    {
      bool mr = false;
      bool ow = false;
      for(int i = 0; i < stmt.get_statement_count(); ++i){
        get_reg_relevant_aux(reg,*stmt.get_statement(i),&mr,&ow);
        if(ow){
          *overwrites = true;
          return; // No more chance to read the old value
        }
        *may_read = *may_read || mr;
      }
      break;
    }
  default:
    throw new std::logic_error("Machine::get_reg_relevant_aux: Unsupported statement.");
  }
};

void Machine::update_forbidden(const std::vector<VecSet<int> > &m, int pid){
  std::vector<std::vector<int> > old_forbidden = forbidden;
  forbidden.clear();
  VecSet<std::vector<int> > new_forbidden;

  for(unsigned i = 0; i < old_forbidden.size(); ++i){
    const VecSet<int> &vs = m[old_forbidden[i][pid]];
    for(auto vit = vs.begin(); vit != vs.end(); ++vit){
      std::vector<int> f = old_forbidden[i];
      f[pid] = *vit;
      new_forbidden.insert(f);
    }
  }
  
  forbidden = new_forbidden.get_vector();
};

Machine *Machine::remove_superfluous_nops_one_pass() const{
  Machine *m = new Machine(*this);

  Log::msg << "Removing superfluous nop instructions from machine.\n";

  for(unsigned pid = 0; pid < automata.size(); ++pid){
    Automaton new_atm = m->automata[pid];
    /* Store removed states and map them to the state that they were
     * merged with. */
    std::map<int,int> removed_states;

    VecSet<std::vector<int> > forbidden_vs;
    for(unsigned i = 0; i < forbidden.size(); ++i){
      forbidden_vs.insert(forbidden[i]);
    }

    /* Rules for removing states:
     *
     * - State Q0 may never be removed.
     * - A state q with exactly one backward transition (q',nop,q) may
     *   be merged into q'.
     * - A state q with exactly one forward transition (q,nop,q') may 
     *   be merged into q', provided that for every v in forbidden such 
     *   that v[pid] == q, there is also a vector v[pid := q'] in 
     *   forbidden.
     */
    const std::vector<Automaton::State> &new_states = new_atm.get_states();
    for(unsigned i = 1; i < new_states.size(); ++i){
      if(new_states[i].bwd_transitions.size() == 1 &&
         (*new_states[i].bwd_transitions.begin())->instruction.get_type() == Lang::NOP){
        Automaton::Transition *t = *new_states[i].bwd_transitions.begin();
        int new_pc = t->source;
        if(new_pc == int(i)){
          // Just remove the nop and do nothing more
        }else{
          removed_states[i] = new_pc;
          /* Move all out-bound transitions to the new state */
          for(auto trit = new_states[i].fwd_transitions.begin(); trit != new_states[i].fwd_transitions.end(); ++trit){
            Automaton::Transition t2(**trit);
            t2.source = new_pc;
            new_atm.add_transition(t2);
          }
        }
      }else if(new_states[i].fwd_transitions.size() == 1 &&
         (*new_states[i].fwd_transitions.begin())->instruction.get_type() == Lang::NOP){
        Automaton::Transition *t = *new_states[i].fwd_transitions.begin();
        int new_pc = t->target;
        if(new_pc == int(i)){
          // Just remove the nop and do nothing more
        }else{
          /* Check that state i is at least as forbidden as state new_pc */
          bool ok_forbidden = true;
          for(int j = 0; j < forbidden_vs.size(); ++j){
            if(forbidden_vs[j][pid] == int(i)){
              std::vector<int> f = forbidden_vs[j];
              f[pid] = new_pc;
              if(forbidden_vs.count(f) == 0){
                ok_forbidden = false;
              }
            }
          }
          if(ok_forbidden){
            removed_states[i] = new_pc;
            /* Move all in-bound transitions to the new state */
            for(auto trit = new_states[i].bwd_transitions.begin(); trit != new_states[i].bwd_transitions.end(); ++trit){
              Automaton::Transition t2(**trit);
              t2.target = new_pc;
              new_atm.add_transition(t2);
            }
          }
        }
      }
    }


    /* Complete removed_states to a complete mapping of old states to new */
    std::map<int,int> old_cs_to_new_cs;
    {
      int next = 0;
      for(unsigned i = 0; i < new_states.size(); ++i){
        if(removed_states.count(i) == 0){
          old_cs_to_new_cs[i] = next;
          ++next;
        }
      }
    }
    for(unsigned i = 0; i < new_states.size(); ++i){
      if(removed_states.count(i)){
        int new_pc = i;
        while(removed_states.count(new_pc)){
          new_pc = removed_states[new_pc];
        }
        old_cs_to_new_cs[i] = new_pc;
      }
    }
      
    /* Create a new automaton with the "removed" states actually removed */
    m->automata[pid] = Automaton();
    for(unsigned i = 0; i < new_states.size(); ++i){
      /* Copy all transitions except for those that originate in or
       * transitions to a removed state, and those that are
       * self-looping read-asserts.
       */
      if(removed_states.count(i) == 0){
        for(auto it = new_states[i].fwd_transitions.begin(); it != new_states[i].fwd_transitions.end(); ++it){
          if(removed_states.count((*it)->target) == 0 &&
             !((*it)->target == (*it)->source && (*it)->instruction.get_type() == Lang::READASSERT)){
            Automaton::Transition t = **it;
            t.source = old_cs_to_new_cs[t.source];
            t.target = old_cs_to_new_cs[t.target];
            m->automata[pid].add_transition(t);
          }
        }
      }
    }

    /* Reset labels */
    const std::map<Lang::label_t,int> &lbls = automata[pid].get_labels();
    for(auto it = lbls.begin(); it != lbls.end(); ++it){
      m->automata[pid].set_label(it->first,old_cs_to_new_cs[it->second]);
    }

    /* Reset forbidden */
    VecSet<std::vector<int> > fvs;
    for(unsigned i = 0; i < m->forbidden.size(); ++i){
      std::vector<int> f = m->forbidden[i];
      f[pid] = old_cs_to_new_cs[f[pid]];
      fvs.insert(f);
    }
    m->forbidden = fvs.get_vector();
  }

  return m;
};

Machine *Machine::remove_superfluous_nops() const{
  const Machine *m0 = this;
  Machine *m1 = remove_superfluous_nops_one_pass();
  Log::debug << "Transition count before: " << m0->get_transition_count() << "\n";
  Log::debug << "Transition count after: " << m1->get_transition_count() << "\n";
  while(m0->get_transition_count() > m1->get_transition_count()){
    Log::debug << "Transition count: " << m0->get_transition_count() << " -> " << m1->get_transition_count() << "\n";
    if(m0 != this) delete m0;
    m0 = m1;
    m1 = m0->remove_superfluous_nops_one_pass();
  }

  Log::debug << "Forbidden:\n";
  for(unsigned i = 0; i < m1->forbidden.size(); ++i){
    Log::debug << "  [";
    for(unsigned p = 0; p < m1->forbidden[i].size(); ++p){
      if(p != 0) Log::debug << ", ";
      Log::debug << m1->forbidden[i][p];
    }
    Log::debug << "]\n";
  }

  if(m0 != this) delete m0;
  return m1;

};

Machine *Machine::forbidden_shave() const{
  Machine *m = new Machine(*this);

  for(unsigned pid = 0; pid < automata.size(); ++pid){
    const std::vector<Automaton::State> &states = automata[pid].get_states();
    std::vector<bool> can_reach(states.size(),false);
    std::vector<int> q;
    for(unsigned i = 0; i < forbidden.size(); ++i){
      if(!can_reach[forbidden[i][pid]]){
        q.push_back(forbidden[i][pid]);
        can_reach[forbidden[i][pid]] = true;
      }
    }
    while(q.size()){
      int s = q.back();
      q.pop_back();
      for(auto it = states[s].bwd_transitions.begin(); it != states[s].bwd_transitions.end(); ++it){
        if(!can_reach[(*it)->source]){
          can_reach[(*it)->source] = true;
          q.push_back((*it)->source);
        }
      }
    }
    
    m->automata[pid] = Automaton();
    if(can_reach[0]){
      std::map<int,int> state_remap;
      int next = 0;
      for(unsigned i = 0; i < states.size(); ++i){
        if(can_reach[i]){
          state_remap[i] = next;
          ++next;
        }
      }
      for(unsigned i = 0; i < states.size(); ++i){
        if(can_reach[i]){
          int src = state_remap[i];
          for(auto it = states[i].fwd_transitions.begin(); it != states[i].fwd_transitions.end(); ++it){
            int tgt = state_remap[(*it)->target];
            m->automata[pid].add_transition(Automaton::Transition(src,(*it)->instruction,tgt));
          }
        }
      }
      for(auto it = automata[pid].get_labels().begin(); it != automata[pid].get_labels().end(); ++it){
        m->automata[pid].set_label(it->first,state_remap[it->second]);
      }
      for(unsigned i = 0; i < m->forbidden.size(); ++i){
        m->forbidden[i][pid] = state_remap[m->forbidden[i][pid]];
      }
    }
  }

  return m;
};

Machine *Machine::add_domain_assumes() const{
  Machine *m = new Machine(*this);

  /* Returns a BExpr asserting that the expression e is within the finite domain dom. */
  std::function<Lang::BExpr<int>(const Lang::Expr<int>&,const Lang::VarDecl::Domain&)> in_domain = 
    [](const Lang::Expr<int> &e,const Lang::VarDecl::Domain &dom)->Lang::BExpr<int>{
    assert(dom.is_finite());
    return Lang::BExpr<int>::leq(Lang::Expr<int>::integer(dom.get_lower_bound()),e) &&
    Lang::BExpr<int>::leq(e,Lang::Expr<int>::integer(dom.get_upper_bound()));
  };
  /* Wraps in_domain(e,dom) in an assume statement. */
  std::function<Lang::Stmt<int>(const Lang::Expr<int>&,const Lang::VarDecl::Domain&)> assume_in_domain = 
    [&in_domain](const Lang::Expr<int> &e,const Lang::VarDecl::Domain &dom)->Lang::Stmt<int>{
    return Lang::Stmt<int>::assume(in_domain(e,dom));
  };

  for(unsigned pid = 0; pid < m->automata.size(); ++pid){
    /* Check whether conversion is necessary. I.e. whether there are any
     * finite domains. */
    int tmp_reg = regs[pid].size();
    bool use_tmp_reg = false;
    bool has_finite = false;
    bool has_finite_with_wild_init = false;
    std::vector<Lang::Stmt<int> > init_assumes;
    for(unsigned i = 0; i < gvars.size(); ++i){
      if(gvars[i].domain.is_finite()){
        has_finite = true;
        if(gvars[i].value.is_wild()){
          has_finite_with_wild_init = true;
          init_assumes.push_back(Lang::Stmt<int>::read_assign(tmp_reg,Lang::MemLoc<int>::global(i)));
          init_assumes.push_back(assume_in_domain(Lang::Expr<int>::reg(tmp_reg),gvars[i].domain));
          use_tmp_reg = true;
        }
      }
    }
    for(unsigned pid2 = 0; pid2 < automata.size(); ++pid2){
      for(unsigned i = 0; i < lvars[pid2].size(); ++i){
        if(lvars[pid2][i].domain.is_finite()){
          has_finite = true;
          if(lvars[pid2][i].value.is_wild()){
            has_finite_with_wild_init = true;
            Lang::MemLoc<int> ml = Lang::NML::local(i,pid2).localize(pid);
            init_assumes.push_back(Lang::Stmt<int>::read_assign(tmp_reg,ml));
            init_assumes.push_back(assume_in_domain(Lang::Expr<int>::reg(tmp_reg),lvars[pid2][i].domain));
            use_tmp_reg = true;
          }
        }
      }
    }
    for(unsigned r = 0; r < regs[pid].size(); ++r){
      if(regs[pid][r].domain.is_finite()){
        has_finite = true;
        if(regs[pid][r].value.is_wild()){
          has_finite_with_wild_init = true;
          init_assumes.push_back(assume_in_domain(Lang::Expr<int>::reg(r),regs[pid][r].domain));
        }
      }
    }

    if(use_tmp_reg){
      m->regs[pid].push_back(Lang::VarDecl("$_tmp",Lang::Value(),Lang::VarDecl::Domain()));
      m->pretty_string_reg[std::pair<int,int>(tmp_reg,pid)] = "$_tmp";
      init_assumes.push_back(Lang::Stmt<int>::assignment(tmp_reg,Lang::Expr<int>::integer(0)));
    }

    if(has_finite){
      m->automata[pid] = Automaton();
      Automaton &atm = m->automata[pid];

      /* Maps control states in the old automaton to control states in
       * the new one. */
      std::function<int(int)> new_q = [](int i){ return i; };

      if(has_finite_with_wild_init){
        new_q = [](int i){ return i+1; }; // Need an extra state between the initial state and the rest of the automaton

        /* Construct an initial assume statement that will ensure that
         * all registers and memory location start with an initial
         * value in their domain */
        {
          std::vector<Lang::Stmt<int>::labeled_stmt_t> lv;
          for(unsigned i = 0; i < init_assumes.size(); ++i){
            lv.push_back(init_assumes[i]);
          }
          std::vector<Lang::Stmt<int> > v;
          v.push_back(Lang::Stmt<int>::sequence(lv));
          atm.add_transition(Automaton::Transition(0,Lang::Stmt<int>::locked_block(v),1));
        }
      }

      /* Recreate the automaton, while adding assume statements as
       * necessary */
      const std::vector<Automaton::State> &states = automata[pid].get_states();
      int next_tmp_state = states.size();
      for(unsigned i = 0; i < states.size(); ++i){
        for(auto trit = states[i].fwd_transitions.begin(); trit != states[i].fwd_transitions.end(); ++trit){
          std::vector<Lang::Stmt<int> > ss = add_domain_assumes((*trit)->instruction,pid);
          int src = new_q((*trit)->source);
          for(unsigned j = 0; j < ss.size(); ++j){
            int tgt;
            if(j == ss.size() - 1){
              tgt = new_q((*trit)->target);
            }else{
              tgt = new_q(next_tmp_state);
              ++next_tmp_state;
            }
            atm.add_transition(Automaton::Transition(src,ss[j],tgt));
            src = tgt;
          }
        }
      }

      /* Update labels & forbidden */
      {
        // labels
        const std::map<Lang::label_t,int> &lbls = automata[pid].get_labels();
        for(auto it = lbls.begin(); it != lbls.end(); ++it){
          atm.set_label(it->first,new_q(it->second));
        }
        // forbidden
        for(unsigned i = 0; i < m->forbidden.size(); ++i){
          m->forbidden[i][pid] = new_q(m->forbidden[i][pid]);
        }
      }
    }
  }

  return m;
};

std::vector<Lang::Stmt<int> > Machine::add_domain_assumes(const Lang::Stmt<int> &s, int pid) const{
  std::vector<Lang::Stmt<int> > ss;

  /* Returns a BExpr asserting that the expression e is within the finite domain dom. */
  std::function<Lang::BExpr<int>(const Lang::Expr<int>&,const Lang::VarDecl::Domain&)> in_domain = 
    [](const Lang::Expr<int> &e,const Lang::VarDecl::Domain &dom)->Lang::BExpr<int>{
    assert(dom.is_finite());
    return Lang::BExpr<int>::leq(Lang::Expr<int>::integer(dom.get_lower_bound()),e) &&
    Lang::BExpr<int>::leq(e,Lang::Expr<int>::integer(dom.get_upper_bound()));
  };
  /* Wraps in_domain(e,dom) in an assume statement. */
  std::function<Lang::Stmt<int>(const Lang::Expr<int>&,const Lang::VarDecl::Domain&)> assume_in_domain = 
    [&s,&in_domain](const Lang::Expr<int> &e,const Lang::VarDecl::Domain &dom)->Lang::Stmt<int>{
    return Lang::Stmt<int>::assume(in_domain(e,dom),s.get_pos());
  };

  switch(s.get_type()){
  case Lang::NOP: case Lang::ASSUME: case Lang::READASSERT:
    ss.push_back(s);
    break;
  case Lang::WRITE:
    {
      Lang::VarDecl::Domain dom = get_declaration(s.get_memloc(),pid).domain;
      if(dom.is_finite() && !expr_always_in_domain(s.get_expr(),pid,dom)){
        ss.push_back(assume_in_domain(s.get_expr(),dom));
      }
      ss.push_back(s);
      break;
    }
  case Lang::ASSIGNMENT:
    {
      std::vector<Lang::Stmt<int>::labeled_stmt_t> lv;
      Lang::VarDecl::Domain dom = get_declaration(s.get_reg(),pid).domain;
      if(dom.is_finite() && !expr_always_in_domain(s.get_expr(),pid,dom)){
        lv.push_back(assume_in_domain(s.get_expr(),dom));
      }
      lv.push_back(s);
      std::vector<Lang::Stmt<int> > v;
      v.push_back(Lang::Stmt<int>::sequence(lv,s.get_pos()));
      ss.push_back(Lang::Stmt<int>::locked_block(v,s.get_pos()));
      break;
    }
  case Lang::READASSIGN:
    {
      std::vector<Lang::Stmt<int>::labeled_stmt_t> lv;
      Lang::VarDecl::Domain dom = get_declaration(s.get_reg(),pid).domain;
      Lang::VarDecl::Domain dom_ml = get_declaration(s.get_memloc(),pid).domain;
      lv.push_back(s);
      if(dom.is_finite() && 
         (dom_ml.is_int() || dom_ml.get_lower_bound() < dom.get_lower_bound() || dom_ml.get_upper_bound() > dom.get_upper_bound())){
        lv.push_back(assume_in_domain(Lang::Expr<int>::reg(s.get_reg()),dom));
      }
      std::vector<Lang::Stmt<int> > v;
      v.push_back(Lang::Stmt<int>::sequence(lv,s.get_pos()));
      ss.push_back(Lang::Stmt<int>::locked_block(v,s.get_pos()));
      break;
    }
  case Lang::SLOCKED:
  case Lang::LOCKED:
    {
      std::vector<Lang::Stmt<int> > v;
      for(int i = 0; i < s.get_statement_count(); ++i){
        std::vector<Lang::Stmt<int> > v2 = add_domain_assumes(*s.get_statement(i),pid);
        std::vector<Lang::Stmt<int>::labeled_stmt_t> lv2;
        for(unsigned j = 0; j < v2.size(); ++j){
          lv2.push_back(v2[j]);
        }
        v.push_back(Lang::Stmt<int>::sequence(lv2,s.get_statement(i)->get_pos()));
      }
      if (s.get_type() == Lang::SLOCKED)
        ss.push_back(Lang::Stmt<int>::slocked_block(v,s.get_pos()));
      else
        ss.push_back(Lang::Stmt<int>::locked_block(v,s.get_pos()));
      break;
    }
  case Lang::SEQUENCE:
    {
      std::vector<Lang::Stmt<int>::labeled_stmt_t> lv;
      for(int i = 0; i < s.get_statement_count(); ++i){
        std::vector<Lang::Stmt<int> > v = add_domain_assumes(*s.get_statement(i),pid);
        for(unsigned j = 0; j < v.size(); ++j){
          lv.push_back(v[j]);
        }
      }
      ss.push_back(Lang::Stmt<int>::sequence(lv,s.get_pos()));
      break;
    }
  case Lang::GOTO: case Lang::UPDATE: case Lang::IF: case Lang::WHILE: case Lang::EITHER:
  default:
    throw new std::logic_error("Machine::add_domain_assumes: Unsupported statement: "+
                               s.to_string(reg_pretty_vts(pid),ml_pretty_vts(pid)));
  }
  return ss;
};

Lang::VarDecl Machine::get_declaration(const Lang::NML &nml) const{
  if(nml.is_global()){
    return gvars[nml.get_id()];
  }else{
    return lvars[nml.get_owner()][nml.get_id()];
  }
};
Lang::VarDecl Machine::get_declaration(const Lang::MemLoc<int> &ml, int pid) const{
  return get_declaration(Lang::NML(ml,pid));
};
Lang::VarDecl Machine::get_declaration(int reg, int pid) const{
  return regs[pid][reg];
};

bool Machine::expr_always_in_domain(const Lang::Expr<int> &e, int pid, const Lang::VarDecl::Domain &dom) const{
  if(dom.is_int()){
    return true;
  }

  std::vector<int> rvals(regs[pid].size());
  std::vector<bool> rused(regs[pid].size(),false);
  {
    std::set<int> rs = e.get_registers();
    for(auto it = rs.begin(); it != rs.end(); ++it){
      rused[*it] = true;
      if(regs[pid][*it].domain.is_int()){
        /* Actually e may still be inside dom, but we settle for a
         * conservative guess. */
        return false;
      }
      rvals[*it] = regs[pid][*it].domain.get_lower_bound();
    }
  }
  
  bool cont = true;
  while(cont){
    if(!dom.member(e.eval<const std::vector<int>&,int*>(rvals,0))){
      return false;
    }
    /* Check the next valuation of rvals */
    unsigned i = 0;
    while(i < regs[pid].size()){
      if(rused[i] && rvals[i] < regs[pid][i].domain.get_upper_bound()){
        ++rvals[i];
        for(unsigned j = 0; j < i; ++j){
          if(rused[j]) rvals[j] = regs[pid][j].domain.get_lower_bound();
        }
        break;
      }
      ++i;
    }
    if(i == regs[pid].size()){
      cont = false; // Tried all valuations
    }
  }
  return true;
};
