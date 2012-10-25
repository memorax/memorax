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

#include "pb_constraint.h"
#include <sstream>

PbConstraint::Common::Common(int k, const Machine &m, pred_set preds, bool auto_abstract)
  : machine(m), predicates(preds), k(k), auto_abstract(auto_abstract), is_init(build_is_init(m)) {

#ifndef NDEBUG
  abstraction_cache_calls = abstraction_cache_hits = 0;
#endif

  last_msg = std::vector<std::vector<std::vector<Lang::MemLoc<int> > > >(machine.proc_count());
  for(int pid = 0; pid < machine.proc_count(); pid++){
    last_msg[pid] = std::vector<std::vector<Lang::MemLoc<int> > >(machine.automata[pid].get_states().size());
    
    /* Populate last_msg by fixpoint computation */
    bool changed = true;
    while(changed){
      changed = false;
      for(unsigned q = 0; q < machine.automata[pid].get_states().size(); q++){
        const Automaton::State &state = machine.automata[pid].get_states()[q];
        for(std::set<Automaton::Transition*>::iterator it = state.fwd_transitions.begin();
            it != state.fwd_transitions.end(); it++){
          int src = (*it)->source;
          int tgt = (*it)->target;
          bool carry = true;
          if((*it)->instruction.get_type() == Lang::WRITE){
            carry = false;
            bool found = false;
            for(unsigned j = 0; j < last_msg[pid][tgt].size(); j++){
              if(last_msg[pid][tgt][j] == (*it)->instruction.get_memloc())
                found = true;
            }
            if(!found){
              last_msg[pid][tgt].push_back((*it)->instruction.get_memloc());
              changed = true;
            }
          }else if((*it)->instruction.is_fence()){
            carry = false;
          }
          if(carry){
            for(unsigned i = 0; i < last_msg[pid][src].size(); i++){
              bool found = false;
              for(unsigned j = 0; j < last_msg[pid][tgt].size(); j++){
                if(last_msg[pid][tgt][j] == last_msg[pid][src][i])
                  found = true;
              }
              if(!found){
                last_msg[pid][tgt].push_back(last_msg[pid][src][i]);
                changed = true;
              }
            }
          }
        }
      }
    }
  }

  /* debug print last_msg */
  Log::debug << "last_msg:\n"; 
  for(int pid = 0; pid < machine.proc_count(); pid++){
    Log::debug << "P" << pid << "\n";
    for(unsigned q = 0; q < machine.automata[pid].get_states().size(); q++){
      Log::debug << "  Q" << q << ": [";
      for(unsigned i = 0; i < last_msg[pid][q].size(); i++){
        Log::debug << last_msg[pid][q][i] << " ";
      }
      Log::debug << "]\n";
    }
  }

  init_all_transitions();
  init_trans_to_pc();

  /* Setup has_read */
  for(int pid = 0; pid < machine.proc_count(); pid++){
    const std::vector<Automaton::State> &states = machine.automata[pid].get_states();
    has_read.push_back(std::vector<bool>(states.size(),false));
    for(unsigned i = 0; i < states.size(); i++){
      for(std::set<Automaton::Transition*>::const_iterator it = states[i].bwd_transitions.begin();
          it != states[i].bwd_transitions.end(); it++){
        if((*it)->instruction.get_reads().size() && !(*it)->instruction.is_fence()){
          has_read[pid][i] = true;
          break;
        }
      }
    }
  }
}

void PbConstraint::Common::init_all_transitions(){
  /* Add all instructions in the machine */
  for(int pid = 0; pid < machine.proc_count(); pid++){
    const std::vector<Automaton::State> &states = machine.automata[pid].get_states();
    for(unsigned i = 0; i < states.size(); i++){
      for(std::set<Automaton::Transition*>::const_iterator it = states[i].fwd_transitions.begin();
          it != states[i].fwd_transitions.end(); it++){
        all_transitions.push_back(Machine::PTransition(**it,pid));
        if((*it)->instruction.get_type() == Lang::WRITE){
          /* Also add an locked version */
          all_transitions.push_back(Machine::PTransition((*it)->source,
                                                         Lang::Stmt<int>::locked_write((*it)->instruction.get_memloc(),
                                                                                       (*it)->instruction.get_expr(),
                                                                                       (*it)->instruction.get_pos()),
                                                         (*it)->target,pid));
        }
      }
    }
  }
  
  /* Add all updates */
  typedef std::list<std::pair<int,Lang::MemLoc<int> > > pmsglist;
  const pmsglist &pml = machine.get_possible_writes();
  for(pmsglist::const_iterator it = pml.begin(); it != pml.end(); it++){
    for(unsigned i = 0; i < machine.automata[it->first].get_states().size(); i++){
      VecSet<Lang::MemLoc<int> > ml = VecSet<Lang::MemLoc<int> >::singleton(it->second);
      all_transitions.push_back(Machine::PTransition(i,Lang::Stmt<int>::update(it->first,ml),i,it->first));
    }
  }

  std::function<bool(const Machine::PTransition&,const Machine::PTransition&)> ptlt = 
    [](const Machine::PTransition &pt0, const Machine::PTransition &pt1){
    return pt0 < pt1;
  };
  std::sort(all_transitions.begin(),all_transitions.end(),ptlt);

  typedef std::pair<int,Lang::MemLoc<int> > pml_t;
  std::map<pml_t,int> pmlatw;
  for(unsigned i = 0; i < all_transitions.size(); i++){
    if(all_transitions[i].instruction.get_type() == Lang::LOCKED &&
       all_transitions[i].instruction.get_statement_count() == 1 &&
       all_transitions[i].instruction.get_statement(0)->get_type() == Lang::WRITE){
      pmlatw[pml_t(all_transitions[i].pid,
                   all_transitions[i].instruction.get_statement(0)->get_memloc())] = i;
    }
  }
  for(unsigned i = 0; i < all_transitions.size(); i++){
    if(all_transitions[i].instruction.get_type() == Lang::WRITE){
      pml_t pml(all_transitions[i].pid,
                all_transitions[i].instruction.get_memloc());
      atomized_writes[&all_transitions[i]] = &all_transitions[pmlatw[pml]];
    }
  }
}

void PbConstraint::Common::init_trans_to_pc(){
  for(unsigned i = 0; i < machine.automata.size(); i++){
    trans_to_pc.push_back(std::vector<std::vector<const Machine::PTransition*> >(machine.automata[i].get_states().size()));
  }
  
  for(unsigned i = 0; i < all_transitions.size(); i++){
    trans_to_pc[all_transitions[i].pid][all_transitions[i].target].push_back(&all_transitions[i]);
  }
}

std::list<const Machine::PTransition*> PbConstraint::Common::all_enabled_by_pc(const PbConstraint *pbc) const{
  std::list<const Machine::PTransition*> l;

  for(unsigned pid = 0; pid < pbc->pcs.size(); pid++){
    const std::vector<const Machine::PTransition*> &v = trans_to_pc[pid][pbc->pcs[pid]];
    l.insert(l.end(),v.begin(),v.end());
  }

  return l;
}

std::string PbConstraint::Common::to_string() const throw(){
  std::stringstream ss;
  ss << " *** Refinement Specification ***\n"
     << "  k: " << k << "\n"
     << "  auto abstract: " << (auto_abstract ? "yes" : "no") << "\n"
     << "  predicates:\n";
  for(unsigned i = 0; i < predicates.size(); i++){
    ss << "    " << predicates[i]->to_string([this](int r, int p)->std::string
                                             { return this->machine.pretty_string_reg.at(std::pair<int,int>(r,p)); },
                                             [this](Lang::NML nml)->std::string
                                             { return this->machine.pretty_string_nml.at(nml); }) << "\n";
  }
  return ss.str();
}

PbConstraint::AppliedPredicate PbConstraint::Common::build_is_init(const Machine &machine){
  Predicate is_init_p(Predicate::tt());

  // Global variables
  for(unsigned i = 0; i < machine.gvars.size(); i++){
    if(!machine.gvars[i].value.is_wild()){
      is_init_p = is_init_p && 
        Predicate::eq(Term::variable(TsoVar(Lang::NML::global(i))),
                      Term::integer(machine.gvars[i].value.get_value()));
    }
  }

  // Local variables & registers
  for(unsigned p = 0; p < machine.lvars.size(); p++){
    // Variables
    for(unsigned i = 0; i < machine.lvars[p].size(); i++){
      if(!machine.lvars[p][i].value.is_wild()){
        Term v = Term::variable(TsoVar(Lang::NML::local(i,p)));
          
        is_init_p = is_init_p && 
          Predicate::eq(v,Term::integer(machine.lvars[p][i].value.get_value()));
      }
    }
    // Registers
    for(unsigned i = 0; i < machine.regs[p].size(); ++i){
      if(!machine.regs[p][i].value.is_wild()){
        Term r = Term::variable(TsoVar::reg(i,p));
        is_init_p = is_init_p &&
          Predicate::eq(r,Term::integer(machine.regs[p][i].value.get_value()));
      }
    }
  }

  Predicate *ppred = new Predicate(is_init_p);

  return AppliedPredicate(ppred,std::vector<TsoVar>());
}

PbConstraint::Common::Common(const Common &c)
  : machine(c.machine), predicates(std::vector<Predicate*>()), k(0), is_init(c.is_init)
{
  throw new std::logic_error("Copy constructor for PbConstraint::Common: Should not be used.");
  /* The reason is that each Common owns its predicates, and the exact
   * memory locations of the predicates are used to identify
   * them. Therefore allowing easy copying of Common objects could
   * lead to mistakes where predicates which are structurally equal
   * but located at different places are considered distinct.
   */
}

PbConstraint::Common &PbConstraint::Common::operator=(const Common &c){
  throw new std::logic_error("Assignment operator for PbConstraint::Common: Should not be used.");
}

PbConstraint::Common::~Common(){
  delete is_init.get_predicate();
#ifndef NDEBUG
  Log::debug << " == PbConstraint::Common stats ==\n"
             << "  Abstraction cache hits: " << abstraction_cache_hits << "/" << abstraction_cache_calls << "\n\n";
#endif
  for(std::list<Predicate*>::iterator it = abstraction_cache_predicates.begin();
      it != abstraction_cache_predicates.end(); it++){
    delete *it;
  }
  for(unsigned i = 0; i < predicates.size(); i++){
    delete predicates[i];
  }
}

PbConstraint::Common::AbstractionResult PbConstraint::Common::abstract(const std::list<AppliedPredicate> &apl){
#ifndef NDEBUG
  abstraction_cache_calls++;
#endif

  std::map<std::list<AppliedPredicate>, AbstractionResult>::iterator aplit = abstraction_cache.find(apl);
  if(aplit == abstraction_cache.end()){
    Predicate pred = Predicate::tt();
    std::list<AppliedPredicate> apl_copy;
    for(std::list<AppliedPredicate>::const_iterator it = apl.begin();
        it != apl.end(); it++){
      pred = pred && it->get_predicate()->bind(it->get_argv());
      if(ap_is_abstract(*it)){
        apl_copy.push_back(*it);
      }else{
        Predicate *pcopy = new Predicate(*it->get_predicate());
        apl_copy.push_back(AppliedPredicate(pcopy,it->get_argv()));
        abstraction_cache_predicates.push_back(pcopy);
      }
    }

    AbstractionResult ar;
    if(APList<TsoVar>::is_consistent(pred)){
      ar.consistent = true;
      std::list<AppliedPredicate> exp = APList<TsoVar>::expand(pred,predicates);
      for(std::list<AppliedPredicate>::const_iterator it = exp.begin(); it != exp.end(); it++){
        ar.abstract.push_back(*it);
      }
      ar.abstract.sort();
    }else{
      ar.consistent = false;
    }
    abstraction_cache[apl_copy] = ar;
    return ar;
  }else{
#ifndef NDEBUG
    abstraction_cache_hits++;
#endif
    return aplit->second;
  }
}

PbConstraint::PbConstraint(const std::vector<int> &pcs, Common &c) 
  : common(c), channels(pcs.size(),sharinglist<Lang::MemLoc<int> >()), 
    pcs(pcs) {
#ifndef NDEBUG
  if(c.k < 1){
    throw new std::logic_error("PbConstraint initialized with k < 1.");
  }
#endif
  dirty_bit = false;
}

PbConstraint::PbConstraint(const PbConstraint &pbc) 
  : common(pbc.common), channels(pbc.channels), pcs(pbc.pcs), dirty_bit(pbc.dirty_bit),
    cycle_locks(pbc.cycle_locks) {
  if(pbc.is_abstracted()){
    ap_list = pbc.ap_list;
  }else{
    for(std::list<AppliedPredicate>::const_iterator it = pbc.ap_list.begin(); 
        it != pbc.ap_list.end(); it++){
      if(common.ap_is_abstract(*it)){
        ap_list.push_back(*it);
      }else{
        Predicate *p = new Predicate(*it->get_predicate());
        temporary_predicates.push_back(p);
        ap_list.push_back(AppliedPredicate(p,it->get_argv()));
      }
    }
  }
}

PbConstraint::PbConstraint(bool dirty_bit, std::vector<sharinglist<Lang::MemLoc<int> > > channels, std::vector<int> pcs, 
                           const std::list<TsoCycleLock> &cls,
                           Common &c)
  : common(c), channels(channels), pcs(pcs), dirty_bit(dirty_bit), cycle_locks(cls)
{
}

PbConstraint::~PbConstraint() throw(){
  for(std::list<Predicate*>::iterator it = temporary_predicates.begin();
      it != temporary_predicates.end(); it++){
    delete *it;
  }
}

PbConstraint &PbConstraint::operator=(const PbConstraint &pbc){
  throw new std::logic_error("PbConstraint::operator=: Not implemented.");
}

const std::vector<int> &PbConstraint::get_control_states() const throw(){
  return pcs;
}

void PbConstraint::abstract(){
  if(!is_abstracted()){
    assert(ap_list_is_sorted());
    abstract(common.abstract(ap_list));
    assert(ap_list_is_abstract());
  }
}

void PbConstraint::abstract(const Common::AbstractionResult &ar){
  ap_list.clear();
  for(std::list<Predicate*>::iterator it = temporary_predicates.begin();
      it != temporary_predicates.end(); it++){
    delete *it;
  }
  temporary_predicates.clear();
  ap_list = ar.abstract;
}

bool PbConstraint::is_init_state() const{
  for(unsigned i = 0; i < pcs.size(); i++){
    if(pcs[i] != 0 || channels[i].size() != 0){
      return false;
    }
  }

  /* Check satisfiability */
  /* TODO: Cache is_init in abstraction cache */
  std::list<AppliedPredicate> l(ap_list.begin(),ap_list.end());
  l.push_back(common.is_init);
  return APList<TsoVar>::is_consistent(l);
}

Constraint::Comparison PbConstraint::entailment_compare(const Constraint &c) const{
  if(const PbConstraint *pc = dynamic_cast<const PbConstraint*>(&c)){
    return entailment_compare(*pc);
  }else{
    throw new std::logic_error("PbConstraint::entailment_compare applied to Constraint"
                               " other than PbConstraint.");
  }
}

Constraint::Comparison PbConstraint::entailment_compare(const PbConstraint &c) const{
  assert(is_abstracted() && c.is_abstracted());
  /* Check fields where equality is required for comparability */
  if(pcs != c.pcs || channels != c.channels || cycle_locks != c.cycle_locks){
    return INCOMPARABLE;
  }

  /* Compare cycle_locks, ap_set and dirty_bit, pointwise */
  std::list<AppliedPredicate>::const_iterator my_it = ap_list.begin();
  std::list<AppliedPredicate>::const_iterator c_it = c.ap_list.begin();
  Comparison cmp = EQUAL;
  if(dirty_bit == c.dirty_bit){
    // Change nothing
  }else if(dirty_bit){
    if(cmp == EQUAL || cmp == GREATER)
      cmp = GREATER;
    else
      cmp = INCOMPARABLE;
  }else{
    if(cmp == EQUAL || cmp == LESS)
      cmp = LESS;
    else
      cmp = INCOMPARABLE;
  }
  while(cmp != INCOMPARABLE && my_it != ap_list.end() && c_it != c.ap_list.end()){
    if(*my_it < *c_it){
      if(cmp == LESS)
        cmp = INCOMPARABLE;
      else
        cmp = GREATER;
      my_it++;
    }else if(!(*my_it == *c_it)){ // *my_it > *c_it
      if(cmp == GREATER)
        cmp = INCOMPARABLE;
      else
        cmp = LESS;
      c_it++;
    }else{ // *my_it == *c_it
      my_it++;
      c_it++;
    }
  }
  if(cmp != INCOMPARABLE){
    if(my_it != ap_list.end()){
      if(cmp == LESS)
        cmp = INCOMPARABLE;
      else
        cmp = GREATER;
    }else if(c_it != c.ap_list.end()){
      if(cmp == GREATER)
        cmp = INCOMPARABLE;
      else
        cmp = LESS;
    }
  }
  
  return cmp;
}

std::string PbConstraint::to_string() const throw(){
  std::stringstream ss;
  for(unsigned pid = 0; pid < pcs.size(); pid++){
    ss << "P" << pid << ": Q" << pcs[pid] << " [";
    for(sharinglist<Lang::MemLoc<int> >::const_iterator it = channels[pid].begin();
        it != channels[pid].end(); it++){
      if(it != channels[pid].begin())
        ss << " ";
      ss << it->to_string();
    }
    ss << "]\n";
  }
  if(dirty_bit){
    ss << "  *** Dirty ***\n";
  }

  for(std::list<AppliedPredicate>::const_iterator it = ap_list.begin(); it != ap_list.end(); it++){
    ss << "  " << it->to_string([this](int r, int p)->std::string
                                { return this->common.machine.pretty_string_reg.at(std::pair<int,int>(r,p)); },
                                [this](Lang::NML nml)->std::string
                                { return this->common.machine.pretty_string_nml.at(nml); }) << "\n";
  }

  for(std::list<TsoCycleLock>::const_iterator it = cycle_locks.begin();
      it != cycle_locks.end(); it++){
    ss << "  " << it->to_long_string(Lang::int_reg_to_string(),
                                     Lang::int_memloc_to_string()) << "\n";
  }

  return ss.str();
}

#define USE_TSO_CYCLE_LOCKS

std::list<Constraint*> PbConstraint::pre(const Machine::PTransition &t) const{
  std::list<Constraint*> l = pre(t,false);
#ifdef USE_TSO_CYCLE_LOCKS  
  bool is_read = t.instruction.get_reads().size() > 0;
  bool is_update = t.instruction.get_type() == Lang::UPDATE;
  const Machine::PTransition *pt = common.find_transition(t); // Persistent version of t
  assert(pt);
  for(auto it = l.begin(); it != l.end(); it++){
    PbConstraint *pbc = static_cast<PbConstraint*>(*it);
    if(is_read){
      pbc->clear_cycle_locks(t.pid);
    }else if(is_update){
      pbc->clear_cycle_locks(t.pid);
      
      std::list<const Machine::PTransition*> ts = common.all_enabled_by_pc(this);
      for(auto trit = ts.begin(); trit != ts.end(); trit++){
        /* Check if trit is a reading transition that can overtake the write of the update t */
        if((*trit)->pid == t.pid && (*trit)->instruction.get_reads().size() > 0 && !(*trit)->instruction.is_fence()){
          pbc->add_cycle_lock(TsoCycleLock(*trit,pt,common.machine.automata.size()));
        }
      }
    }
    pbc->cycle_locks_execute(pt);
  }
#endif
  return l;
}

std::list<Constraint*> PbConstraint::pre(const Machine::PTransition &t,bool locked) const{
  std::list<Constraint*> result;
  int next_pc = t.source; // The control state where process pid will end up if the instruction is performed
  const Lang::Stmt<int> &s = t.instruction;
  int pid = t.pid;

  if(pcs[t.pid] == t.target){
    switch(t.instruction.get_type()){
    case Lang::NOP:
      {
        PbConstraint *p = new PbConstraint(*this);
        p->last_trans = last_trans_t(true,pid);
        result.push_back(p);
        break;
      }
    case Lang::READASSERT:
      {
        int msg = this->index_of(pid,s.get_memloc());

        PbConstraint *p = new PbConstraint(*this);
        p->last_trans = last_trans_t(false,pid);

        Term e(Term::from_expr(s.get_expr(),pid));
        TsoVar v = (msg == 0 ? TsoVar(Lang::NML(s.get_memloc(),pid)) : TsoVar::msg(msg,pid));
        Predicate *cp = new Predicate(Predicate::eq(Term::variable(v), e));
        std::list<Predicate*> tmp_preds(1,cp);
        //std::list<AppliedPredicate> apl(1,AppliedPredicate(cp,std::vector<TsoVar>()));
        std::list<AppliedPredicate> apl;
        apl.push_back(AppliedPredicate(cp,std::vector<TsoVar>()));

        if(p->add_to_ap_list(&apl,&tmp_preds)){
          assert(p->ap_list_is_sorted());
          result.push_front(p);
        }else{
          delete p;
        }
        break;
      }
    case Lang::READASSIGN:
      {
        int msg = this->index_of(pid,s.get_memloc());

        PbConstraint *p = new PbConstraint(this->dirty_bit,this->channels,this->pcs,
                                           this->cycle_locks,this->common);
        p->last_trans = last_trans_t(false,pid);

        TsoVar v = (msg == 0 ? TsoVar(Lang::NML(s.get_memloc(),pid)) : TsoVar::msg(msg,pid));
        TsoVar r = TsoVar::reg(s.get_reg(),pid);

        std::list<AppliedPredicate> apl;
        std::list<Predicate*> tmp_preds;
        std::function<TsoVar(const TsoVar&)> trans = Predicates::subst_translator(v,TsoVar::reg(s.get_reg(),pid));
        for(std::list<AppliedPredicate>::const_iterator it = this->ap_list.begin();
            it != this->ap_list.end(); it++){
          if(it->get_variables().count(r)){
            Predicate *ppred = new Predicate(it->get_predicate()->substitute(Term::variable(v),r));
            tmp_preds.push_back(ppred);
            AppliedPredicate ap(ppred,it->get_argv());
            ap.translate_args(trans);
            apl.push_back(ap);
          }else{
            if(this->common.ap_is_abstract(*it)){
              p->ap_list.push_back(*it);
            }else{
              Predicate *ppred = new Predicate(*it->get_predicate());
              p->temporary_predicates.push_back(ppred);
              p->ap_list.push_back(AppliedPredicate(ppred,std::vector<TsoVar>()));
            }
          }
        }
        if(p->add_to_ap_list(&apl,&tmp_preds)){
          assert(p->ap_list_is_sorted());
          result.push_front(p);
        }else{
          delete p;
        }
        break;
      }
    case Lang::WRITE:
      {
        if(locked){
          if(this->channels[pid].size() == 0){

            PbConstraint *p = new PbConstraint(this->dirty_bit,this->channels,this->pcs,
                                               this->cycle_locks,this->common);
            p->last_trans = last_trans_t(false,pid);
            Term t(Term::from_expr(s.get_expr(),pid));
            TsoVar v = TsoVar(Lang::NML(s.get_memloc(),pid));
            std::list<AppliedPredicate> apl;
            std::list<Predicate*> tmp_preds;
            for(std::list<AppliedPredicate>::const_iterator it = this->ap_list.begin();
                it != this->ap_list.end(); it++){
              if(it->get_variables().count(v)){
                Predicate *ppred = new Predicate(it->get_predicate()->bind(it->get_argv()).substitute(t,v));
                tmp_preds.push_back(ppred);
                apl.push_back(AppliedPredicate(ppred,std::vector<TsoVar>()));
              }else{
                if(this->common.ap_is_abstract(*it)){
                  p->ap_list.push_back(*it);
                }else{
                  Predicate *ppred = new Predicate(*it->get_predicate());
                  p->temporary_predicates.push_back(ppred);
                  p->ap_list.push_back(AppliedPredicate(ppred,it->get_argv()));
                }
              }
            }

            if(p->add_to_ap_list(&apl,&tmp_preds)){
              assert(p->ap_list_is_sorted());
              result.push_front(p);
            }else{
              delete p;
            }
          }
      
          /* Non-locked: Check the last message in the channel */
        }else if(this->channels[pid].size() > 0 && this->channels[pid].back() == s.get_memloc()){

          Term t(Term::from_expr(s.get_expr(),pid));
          TsoVar v = TsoVar::msg(this->channels[pid].size(),pid);

          std::vector<sharinglist<Lang::MemLoc<int> > > new_channels(this->channels);
          new_channels[pid].pop_back();
          PbConstraint *p = new PbConstraint(this->dirty_bit,new_channels,this->pcs,
                                             this->cycle_locks,this->common);
          p->last_trans = last_trans_t(true,pid);
          if(p->ok_last_msg(pid,next_pc)){
            std::list<AppliedPredicate> apl;
            std::list<Predicate*> tmp_preds;
            for(std::list<AppliedPredicate>::const_iterator it = this->ap_list.begin(); it != this->ap_list.end(); it++){
              if(it->get_variables().count(v)){
                Predicate *ppred = new Predicate(it->get_predicate()->bind(it->get_argv()).substitute(t,v));
                tmp_preds.push_back(ppred);
                apl.push_back(AppliedPredicate(ppred,std::vector<TsoVar>()));
              }else{
                if(this->common.ap_is_abstract(*it)){
                  p->ap_list.push_back(*it);
                }else{
                  Predicate *ppred = new Predicate(*it->get_predicate());
                  p->temporary_predicates.push_back(ppred);
                  p->ap_list.push_back(AppliedPredicate(ppred,it->get_argv()));
                }
              }
            }
            if(p->add_to_ap_list(&apl,&tmp_preds)){
              assert(p->ap_list_is_sorted());
              result.push_back(p);
            }else{
              delete p;
            }
          }else{
            delete p;
          }

          /* Do we have the option to overflow? */
          int act_k = 0;
          for(sharinglist<Lang::MemLoc<int> >::const_iterator it = this->channels[pid].begin(); 
              it != this->channels[pid].end(); it++){
            if(*it == s.get_memloc())
              act_k++;
          }
    
          if(act_k >= this->common.k){
            assert(act_k == this->common.k);
            /* We may overflow */
            /* Construct all possible new channels */
            bool finish = false;
            for(unsigned i = 1; !finish && i <= this->channels[pid].size(); i++){
              std::vector<sharinglist<Lang::MemLoc<int> > > new_new_channels(this->channels);
              sharinglist<Lang::MemLoc<int> >::iterator it = new_new_channels[pid].begin();
              for(unsigned j = 1; j < i; j++) it++; // Skip the messages which are already checked
              assert(it != new_new_channels[pid].end());
              if(*it == s.get_memloc()){
                finish = true;
              }
              new_new_channels[pid].insert(it,s.get_memloc());
              new_new_channels[pid].pop_back();
              PbConstraint *p = new PbConstraint(true,new_new_channels,this->pcs,
                                                 this->cycle_locks,this->common);
              p->last_trans = last_trans_t(true,pid);

              if(p->ok_last_msg(pid,next_pc)){
                std::function<TsoVar(const TsoVar&)> trans = 
                  [i,pid](const TsoVar &tv)->const TsoVar{
                  if(tv.get_type() == TsoVar::MSG && tv.get_process() == pid && tv.get_msg() >= int(i)){
                    return TsoVar::msg(tv.get_msg()+1,pid);
                  }else{
                    return tv;
                  }
                };
                std::list<AppliedPredicate> apl;
                std::list<Predicate*> tmp_preds;
                for(std::list<AppliedPredicate>::const_iterator it = this->ap_list.begin(); it != this->ap_list.end(); it++){
                  std::set<TsoVar> vars = it->get_variables();
                  bool modify = false;
                  for(std::set<TsoVar>::iterator vit = vars.begin(); vit != vars.end(); vit++){
                    if(*vit == v || (vit->get_type() == TsoVar::MSG && vit->get_process() == pid)){
                      modify = true;
                      break;
                    }
                  }
                  if(modify){
                    Predicate *ppred = new Predicate(it->get_predicate()->bind(it->get_argv()).substitute(t,v));
                    ppred->translate(trans);
                    tmp_preds.push_back(ppred);
                    apl.push_back(AppliedPredicate(ppred,std::vector<TsoVar>()));
                  }else{
                    if(this->common.ap_is_abstract(*it)){
                      p->ap_list.push_back(*it);
                    }else{
                      Predicate *ppred = new Predicate(*it->get_predicate());
                      p->temporary_predicates.push_back(ppred);
                      p->ap_list.push_back(AppliedPredicate(ppred,it->get_argv()));
                    }
                  }
                }

                if(p->add_to_ap_list(&apl,&tmp_preds)){
                  assert(p->ap_list_is_sorted());
                  result.push_back(p);
                }else{
                  delete p;
                }
              }else{
                delete p;
              }
            }
          }
        } // end if(locked)
        break;
      }
    case Lang::ASSIGNMENT:
      {
        PbConstraint *p = new PbConstraint(this->dirty_bit,this->channels,this->pcs,
                                           this->cycle_locks,this->common);
        p->last_trans = last_trans_t(true,pid);

        Term t(Term::from_expr(s.get_expr(),pid));
        TsoVar v = TsoVar::reg(s.get_reg(),pid);
        std::list<AppliedPredicate> apl;
        std::list<Predicate*> tmp_preds;
        for(std::list<AppliedPredicate>::const_iterator it = this->ap_list.begin(); it != this->ap_list.end(); it++){
          if(it->get_variables().count(v)){
            Predicate *ppred = new Predicate(it->get_predicate()->bind(it->get_argv()).substitute(t,v));
            tmp_preds.push_back(ppred);
            apl.push_back(AppliedPredicate(ppred,std::vector<TsoVar>()));
          }else{
            if(this->common.ap_is_abstract(*it)){
              p->ap_list.push_back(*it);
            }else{
              Predicate *ppred = new Predicate(*it->get_predicate());
              p->temporary_predicates.push_back(ppred);
              p->ap_list.push_back(AppliedPredicate(ppred,it->get_argv()));
            }
          }
        }
        if(p->add_to_ap_list(&apl,&tmp_preds)){
          assert(p->ap_list_is_sorted());
          result.push_front(p);
        }else{
          delete p;
        }
        break;
      }
    case Lang::ASSUME:
      {
        Predicate *c = new Predicate(Predicate::from_bexpr(s.get_condition(),pid));
        std::list<Predicate*> tmp_preds(1,c);
        std::list<AppliedPredicate> apl(1,AppliedPredicate(c,std::vector<TsoVar>()));
        PbConstraint *p = new PbConstraint(*this);
        p->last_trans = last_trans_t(true,pid);
        assert(this->ap_list_is_sorted());
        assert(p->ap_list_is_sorted());
        if(p->add_to_ap_list(&apl,&tmp_preds)){
          assert(p->ap_list_is_sorted());
          result.push_front(p);
        }else{
          delete p;
        }
        break;
      }
    case Lang::LOCKED:
      {
        if(!s.is_fence() || channels[pid].empty()){
          bool local = s.get_reads().empty() && s.get_writes().empty();
          for(int i = 0; i < s.get_statement_count(); i++){
            Machine::PTransition t2(t.source,*s.get_statement(i),t.target,pid);
            std::list<Constraint*> r = pre(t2,true);
            for(auto it = r.begin(); it != r.end(); it++){
              PbConstraint *p = static_cast<PbConstraint*>(*it);
              p->last_trans = last_trans_t(local,pid);
              result.push_back(p);
            }
          }
        }
        break;
      }
    case Lang::SEQUENCE:
      {
        if(!locked){
          throw new std::logic_error("PbConstraint::pre: Illegal statement: (non-locked sequence).");
        }
        std::list<Constraint*> a(1,new PbConstraint(*this));
        std::list<Constraint*> b;
        std::list<Constraint*> *after = &a;
        std::list<Constraint*> *before = &b;
        for(int i = s.get_statement_count()-1; i >= 0; i--){
          for(auto it = after->begin(); it != after->end(); it++){
            Machine::PTransition t2(t.target,*s.get_statement(i),t.target,pid);
            std::list<Constraint*> l = static_cast<PbConstraint*>(*it)->pre(t2,locked);
            delete *it;
            before->insert(before->end(),l.begin(),l.end());
          }
          after->clear();
          std::list<Constraint*> *tmp = after;
          after = before;
          before = tmp;
        }
        for(auto ait = after->begin(); ait != after->end(); ait++){
          static_cast<PbConstraint*>(*ait)->pcs[pid] = t.source;
          result.push_back(*ait);
        }
        break;
      }
    case Lang::UPDATE:
      {
        assert(s.get_writes().size() == 1);
        /* Check that pid and writer is the same process */
        bool executable = s.get_writer() == pid;

        if(this->channels[pid].size() == 0){
          /* Check that the new message is acceptable at this control state */
          unsigned sz = this->common.last_msg[pid][this->pcs[pid]].size();
          unsigned i;
          for(i = 0; i < sz; i++){
            if(s.get_memloc() == this->common.last_msg[pid][this->pcs[pid]][i])
              break;
          }
          if(i == this->common.last_msg[pid][this->pcs[pid]].size()){
            executable = false;
          }
        }

        if(locked) executable = false;

        if(executable){

          /* Count the number of occurences of s.get_memloc() in the channel. */
          int act_k = 0;
          for(sharinglist<Lang::MemLoc<int> >::const_iterator it = this->channels[pid].begin();
              it != this->channels[pid].end(); it++){
            if(*it == s.get_memloc()){
              act_k++;
            }
          }

          if(act_k < this->common.k){
            PbConstraint *p = new PbConstraint(this->dirty_bit,this->channels,this->pcs,
                                               this->cycle_locks,this->common);
            p->last_trans = last_trans_t(false,pid);
            p->channels[pid].push_front(s.get_memloc());
            std::function<TsoVar(const TsoVar&)> trans = 
              [pid,&s](const TsoVar &tv)->const TsoVar{
              if(tv == TsoVar(Lang::NML(s.get_memloc(),pid))){
                return TsoVar::msg(1,pid);
              }else if(tv.get_type() == TsoVar::MSG && tv.get_process() == pid){
                return TsoVar::msg(tv.get_msg()+1,pid);
              }else{
                return tv;
              }
            };
            p->get_ap_set_from(*this,trans);

            assert(p->ap_list_is_sorted());
            result.push_back(p);
          }else{
            assert(act_k == this->common.k);
            /* act_k is too great, we cannot insert a new message */
            /* Remove all constraints on s.get_memloc() in memory */
#ifndef NDEBUG
            if(!this->is_abstracted()){
              std::cerr << " * PbConstraint::pre: Beware: Original constraint is not abstracted.\n" 
                        << " *                    Dirty update is very aggressive.\n";
            }
#endif
            PbConstraint *p = new PbConstraint(this->dirty_bit,this->channels,this->pcs,
                                               this->cycle_locks,this->common);
            p->last_trans = last_trans_t(false,pid);
            TsoVar to_remove(Lang::NML(s.get_memloc(),pid));
            for(std::list<AppliedPredicate>::const_iterator it = this->ap_list.begin(); it != this->ap_list.end(); it++){
              if(!(it->get_variables().count(to_remove))){
                p->ap_list.push_back(*it);
              }
            }
            p->dirty_bit = true;

            assert(p->ap_list_is_sorted());
            result.push_back(p);
          }
        }
        break;
      }
    case Lang::GOTO:
      throw new std::logic_error("PbConstraint::pre: Illegal statement 'goto'.");
    case Lang::IF:
      throw new std::logic_error("PbConstraint::pre: Illegal statement 'if'.");
    case Lang::WHILE:
      throw new std::logic_error("PbConstraint::pre: Illegal statement 'while'.");
    case Lang::EITHER:
      throw new std::logic_error("PbConstraint::pre: Illegal statement 'either'.");
    default:
      throw new std::logic_error("PbConstraint::pre: Unknown statement type.");
    }
  }

  /* Update program counters */
  for(std::list<Constraint*>::iterator it = result.begin(); it != result.end(); it++){
    PbConstraint *pbc = static_cast<PbConstraint*>(*it);
    pbc->pcs[t.pid] = t.source;
    assert(pbc->ap_list_is_sorted());
    assert(!common.auto_abstract || pbc->is_abstracted());
    assert(pbc->ap_list_is_sorted());
  }
  return result;
}

int PbConstraint::index_of(int pid, const Lang::MemLoc<int> &ml) const{
  int index = 0;
  int i = 1;
  for(sharinglist<Lang::MemLoc<int> >::const_iterator it = channels[pid].begin();
      it != channels[pid].end(); it++){
    if(*it == ml){
      index = i;
    }
    i++;
  }
  return index;
}

VecSet<PbConstraint::Predicate> PbConstraint::extract_predicates(const Lang::Stmt<int> &stmt, int pid){
  VecSet<Predicate> ps;
  switch(stmt.get_type()){
  case Lang::LOCKED: case Lang::SEQUENCE:
    {
      for(int i = 0; i < stmt.get_statement_count(); ++i){
        ps.insert(extract_predicates(*stmt.get_statement(i),pid));
      }
      break;
    }
  case Lang::ASSUME:
    ps.insert(Predicate::from_bexpr(stmt.get_condition(),pid).generalise().drive_in_negations());
    break;
  case Lang::WRITE:
    ps.insert(Predicate::eq(Term::argument(0), 
                            Term::from_expr(stmt.get_expr(),pid)).
              generalise().drive_in_negations());
    break;
  case Lang::READASSERT:
    ps.insert(Predicate::eq(Term::argument(0),Term::from_expr(stmt.get_expr(),pid))
              .generalise().drive_in_negations());
    break;
  default:
    // Do nothing
    break;
  }
  return ps;
};

PbConstraint::pred_set PbConstraint::extract_predicates(const Machine &m){
  VecSet<Predicate> pvs;
  for(int pid = 0; pid < m.proc_count(); pid++){
    const std::vector<Automaton::State> &states = m.automata[pid].get_states();
    for(unsigned q = 0; q < states.size(); q++){
      for(auto it = states[q].fwd_transitions.begin();
          it != states[q].fwd_transitions.end(); it++){
        pvs.insert(extract_predicates((*it)->instruction,pid));
      }
    }
  }

  pred_set ps;
  for(int i = 0; i < pvs.size(); ++i){
    ps.push_back(new Predicate(pvs[i]));
  }
  
  return ps;
};

bool PbConstraint::ok_last_msg(int pid, int pc) const {  
  if(channels[pid].size() == 0){
    return true;
  }

  /* Check that the new message is acceptable at this control state */
  unsigned sz = common.last_msg[pid][pc].size();
  unsigned i;
  for(i = 0; i < sz; i++){
    if(channels[pid].back() == common.last_msg[pid][pc][i])
      break;
  }
  return i < sz;
}

void PbConstraint::get_ap_set_from(const PbConstraint &pbc, std::function<TsoVar(const TsoVar&)> &t){

  if(pbc.is_abstracted()){
    for(std::list<AppliedPredicate>::const_iterator it = pbc.ap_list.begin(); it != pbc.ap_list.end(); it++){
      if(it->get_predicate()->has_constants()){
        Predicate *ppred = new Predicate(it->get_predicate()->bind(it->get_argv()));
        ppred->translate(t);
        temporary_predicates.push_back(ppred);
        ap_list.push_back(AppliedPredicate(ppred,std::vector<TsoVar>()));
      }else{
        AppliedPredicate ap(*it);
        ap.translate_args(t);
        ap_list.push_back(ap);
      }
    }
  }else{
    for(std::list<AppliedPredicate>::const_iterator it = pbc.ap_list.begin(); it != pbc.ap_list.end(); it++){
      Predicate *ppred = new Predicate(it->get_predicate()->bind(it->get_argv()));
      ppred->translate(t);
      temporary_predicates.push_back(ppred);
      ap_list.push_back(AppliedPredicate(ppred,std::vector<TsoVar>()));
    }
  }
  ap_list.sort();

}

#ifndef NDEBUG
bool PbConstraint::ap_list_is_sorted() const throw(){
  if(!ap_list.empty()){
    std::list<AppliedPredicate>::const_iterator it0 = ap_list.begin();
    std::list<AppliedPredicate>::const_iterator it1 = ap_list.begin();
    it1++;
    while(it1 != ap_list.end()){
      if(!(*it0 < *it1 || *it0 == *it1)){
        return false;
      }
      it0++;
      it1++;
    }
  }
  return true;
}

bool PbConstraint::ap_list_is_abstract() const throw(){
  for(std::list<AppliedPredicate>::const_iterator it = ap_list.begin(); it != ap_list.end(); it++){
    if(!common.ap_is_abstract(*it)){
      return false;
    }
  }
  return true;
}
#endif

template<class E> void PbConstraint::sorted_insert(std::list<E> *l, const E &e) throw(){
  typename std::list<E>::iterator it = l->begin();
  while(it != l->end() && *it < e)
    it++;
  l->insert(it,e);
}

template<class E> bool PbConstraint::set_intersects(const std::set<E> &a, const std::set<E> &b) throw(){
  const std::set<E> *x, *y;
  /* Set up x and y such that x->size() <= y->size(). */
  if(a.size() < b.size()){
    x = &a;
    y = &b;
  }else{
    x = &b;
    y = &a;
  }

  for(typename std::set<E>::const_iterator it = x->begin(); it != x->end(); it++){
    if(y->count(*it))
      return true;
  }
  return false;
}

void PbConstraint::transfer_related(std::list<AppliedPredicate> *src, std::list<AppliedPredicate> *dst) throw(){
  std::set<TsoVar> vars;
  for(std::list<AppliedPredicate>::iterator it = dst->begin(); it != dst->end(); it++){
    std::set<TsoVar> vars2 = it->get_variables();
    for(std::set<TsoVar>::iterator it2 = vars2.begin(); it2 != vars2.end(); it2++)
      vars.insert(*it2);
  }
  unsigned vars_sz = vars.size();
  std::list<AppliedPredicate>::iterator it = src->begin();
  while(it != src->end()){
    std::set<TsoVar> vars2 = it->get_variables();
    if(set_intersects(vars,vars2)){
      for(std::set<TsoVar>::iterator it2 = vars2.begin(); it2 != vars2.end(); it2++)
        vars.insert(*it2);
      dst->push_back(*it);
      it = src->erase(it);
      if(vars_sz != vars.size()) /* New variables added, recheck earlier applied predicates */
        it = src->begin();
      vars_sz = vars.size();
    }else{
      it++;
    }
  }
}

bool PbConstraint::add_to_ap_list(std::list<AppliedPredicate> *apl, std::list<Predicate*> *tmp_preds){
  Common::AbstractionResult ar;
  if(common.auto_abstract){
    assert(is_abstracted());
    transfer_related(&ap_list,apl);
    ar = common.abstract(*apl);
    apl->clear();
    for(std::list<Predicate*>::iterator it = tmp_preds->begin(); it != tmp_preds->end(); it++){
      delete *it;
    }
    assert(ap_list_is_sorted());
    ap_list.merge(ar.abstract);
    assert(ap_list_is_sorted());
  }else{
    apl->sort();
    ap_list.merge(*apl);
    std::copy(tmp_preds->begin(),tmp_preds->end(),std::back_inserter(temporary_predicates));
    ar = common.abstract(ap_list);
  }
  apl->clear();
  tmp_preds->clear();
  return ar.consistent;
}

bool PbConstraint::intersects(const Constraint &c) const{
  return intersects(static_cast<const PbConstraint&>(c));
}

bool PbConstraint::intersects(const PbConstraint &c) const{
  if(pcs == c.pcs && channels == c.channels){
    std::list<AppliedPredicate> l(ap_list.begin(),ap_list.end());
    for(std::list<AppliedPredicate>::const_iterator it = c.ap_list.begin(); it != c.ap_list.end(); it++){
      l.push_back(*it);
    }
    return APList<TsoVar>::is_consistent(l);
  }else{
    return false;
  }
}

std::list<Constraint*> PbConstraint::post(const Machine::PTransition &t, bool locked) const{
  std::list<Constraint*> result;
  const Lang::Stmt<int> &s = t.instruction;

  if(pcs[t.pid] != t.source){
    return result;
  }

  Predicate thisstate = ap_list.empty() ? Predicate::tt() : ap_list.front().get_predicate()->bind(ap_list.front().get_argv());
  {
    auto apit = ap_list.begin();
    if(apit != ap_list.end()){
      apit++;
      for( ; apit != ap_list.end(); apit++){
        thisstate = thisstate && apit->get_predicate()->bind(apit->get_argv());
      }
    }
  }

  switch(s.get_type()){
  case Lang::LOCKED:
    {
      if(!s.is_fence() || channels[t.pid].empty()){
        for(int i = 0; i < s.get_statement_count(); i++){
          Machine::PTransition t2(t.source,*s.get_statement(i),t.target,t.pid);
          std::list<Constraint*> r = post(t2,true);
          result.insert(result.end(),r.begin(),r.end());
        }
      }
      break;
    }
  case Lang::SEQUENCE:
    {
      if(!locked){
        throw new std::logic_error("PbConstraint::post: Illegal statement: (non-locked sequence)");
      }
      
      std::list<Constraint*> a;
      std::list<Constraint*> b(1,new PbConstraint(*this));
      std::list<Constraint*> *after = &a;
      std::list<Constraint*> *before = &b;
      for(int i = 0; i < s.get_statement_count(); i++){
        for(auto it = before->begin(); it != before->end(); it++){
          Machine::PTransition t2(t.source,*s.get_statement(i),t.source,t.pid);
          std::list<Constraint*> l = static_cast<PbConstraint*>(*it)->post(t2,locked);
          delete *it;
          after->insert(after->end(),l.begin(),l.end());
        }
        before->clear();
        std::list<Constraint*> *tmp = after;
        after = before;
        before = tmp;
      }
      for(auto bit = before->begin(); bit != before->end(); bit++){
        static_cast<PbConstraint*>(*bit)->pcs[t.pid] = t.target;
        result.push_back(*bit);
      }      

      break;
    }
  case Lang::ASSUME:
    {
      PbConstraint *pbc2 = new PbConstraint(dirty_bit,channels,pcs,cycle_locks,common);
      Predicate p = thisstate && Predicate::from_bexpr(s.get_condition(),t.pid);
      Predicate *pp = new Predicate(p);
      pbc2->temporary_predicates.push_back(pp);
      pbc2->ap_list.push_back(AppliedPredicate(pp,std::vector<TsoVar>()));
      result.push_back(pbc2);
      break;
    }
  case Lang::READASSERT:
    {
      int msg = this->index_of(t.pid,s.get_memloc());
      TsoVar v = (msg == 0 ? TsoVar(Lang::NML(s.get_memloc(),t.pid)) : TsoVar::msg(msg,t.pid));
      Predicate p = thisstate && Predicate::eq(Term::variable(v),Term::from_expr(s.get_expr(),t.pid));
      PbConstraint *pbc2 = new PbConstraint(dirty_bit,channels,pcs,cycle_locks,common);
      Predicate *pp = new Predicate(p);
      pbc2->temporary_predicates.push_back(pp);
      pbc2->ap_list.push_back(AppliedPredicate(pp,std::vector<TsoVar>()));
      result.push_back(pbc2);

      break;
    }
  case Lang::ASSIGNMENT:
    {
      TsoVar r = TsoVar::reg(s.get_reg(),t.pid);
      TsoVar tmpv = TsoVar::fresh_tmp();
      Predicate p = thisstate;
      std::function<TsoVar(const TsoVar&)> tf = [&r,&tmpv](const TsoVar &tv)->const TsoVar{
        if(tv == r){
          return tmpv;
        }else{
          return tv;
        }
      };
      p.translate(tf);
      p = p && Predicate::eq(Term::variable(r),Term::from_expr(s.get_expr(),t.pid));
      PbConstraint *pbc2 = new PbConstraint(dirty_bit,channels,pcs,cycle_locks,common);
      Predicate *pp = new Predicate(p);
      pbc2->temporary_predicates.push_back(pp);
      pbc2->ap_list.push_back(AppliedPredicate(pp,std::vector<TsoVar>()));
      result.push_back(pbc2);
      break;
    }
  case Lang::READASSIGN:
    {
      int msg = this->index_of(t.pid,s.get_memloc());
      TsoVar v = (msg == 0 ? TsoVar(Lang::NML(s.get_memloc(),t.pid)) : TsoVar::msg(msg,t.pid));
      TsoVar r = TsoVar::reg(s.get_reg(),t.pid);
      TsoVar tmpv = TsoVar::fresh_tmp();
      Predicate p = thisstate;
      std::function<TsoVar(const TsoVar&)> tf = [&r,&tmpv](const TsoVar &tv)->const TsoVar{
        if(tv == r){
          return tmpv;
        }else{
          return tv;
        }
      };
      p.translate(tf);
      p = p && Predicate::eq(Term::variable(v),Term::variable(r));
      PbConstraint *pbc2 = new PbConstraint(dirty_bit,channels,pcs,cycle_locks,common);
      Predicate *pp = new Predicate(p);
      pbc2->temporary_predicates.push_back(pp);
      pbc2->ap_list.push_back(AppliedPredicate(pp,std::vector<TsoVar>()));
      result.push_back(pbc2);
      break;
    }
  case Lang::WRITE:
    {
      if(locked){
        if(channels[t.pid].empty()){
          TsoVar v(Lang::NML(s.get_memloc(),t.pid));
          TsoVar tmpv = TsoVar::fresh_tmp();
          Predicate p = thisstate;
          std::function<TsoVar(const TsoVar&)> tf = [&v,&tmpv](const TsoVar &tv)->const TsoVar{
            if(tv == v){
              return tmpv;
            }else{
              return tv;
            }
          };
          p.translate(tf);
          p = p && Predicate::eq(Term::variable(v),Term::from_expr(s.get_expr(),t.pid));
          PbConstraint *pbc2 = new PbConstraint(dirty_bit,channels,pcs,cycle_locks,common);
          Predicate *pp = new Predicate(p);
          pbc2->temporary_predicates.push_back(pp);
          pbc2->ap_list.push_back(AppliedPredicate(pp,std::vector<TsoVar>()));
          result.push_back(pbc2);
        }
      }else{
        throw new std::logic_error("PbConstraint::post: Unhandled instruction (non-locked write).");
      }
      break;
    }
  default:
    throw new std::logic_error("PbConstraint::post: Unhandled instruction: "+
                               s.to_string(common.machine.reg_pretty_vts(t.pid),
                                           common.machine.ml_pretty_vts(t.pid)));
  }

  /* Change pids */
  for(auto it = result.begin(); it != result.end(); it++){
    PbConstraint *pbc = static_cast<PbConstraint*>(*it);
    pbc->pcs[t.pid] = t.target;
    assert(pbc->ap_list_is_sorted());
    // assert(!common.auto_abstract || pbc->is_abstracted());
    assert(pbc->ap_list_is_sorted());
  }

  return result;
};

std::list<Constraint*> PbConstraint::post(const Machine::PTransition &t) const{
  return post(t,false);
};

std::list<Constraint*> PbConstraint::range(const Machine::PTransition &trans) const{
  std::list<Constraint*> result;
  int pid = trans.pid;
  const Lang::Stmt<int> &s = trans.instruction;

  if(pcs[trans.pid] != trans.target){
    return result;
  }

  switch(trans.instruction.get_type()){
  case Lang::LOCKED:
    {
      PbConstraint pbc2(dirty_bit,channels,pcs,
                        std::list<TsoCycleLock>(),common);
      pbc2.pcs[trans.pid] = trans.source;
      std::list<Constraint*> r = pbc2.post(trans);
      result.insert(result.end(),r.begin(),r.end());
      break;
    }
  case Lang::NOP:
    {
      PbConstraint *pbc2 = new PbConstraint(*this);
      result.push_back(pbc2);
      break;
    }
  case Lang::ASSUME:
    {
      PbConstraint *pbc2 = new PbConstraint(*this);
      Predicate *cp = new Predicate(Predicate::from_bexpr(s.get_condition(),pid));

      pbc2->temporary_predicates.push_back(cp);
      sorted_insert(&pbc2->ap_list, AppliedPredicate(cp,std::vector<TsoVar>()));

      result.push_back(pbc2);
      break;
    }
  case Lang::READASSERT:
    {
      int msg = index_of(pid,s.get_memloc());

      PbConstraint *pbc2 = new PbConstraint(*this);

      Term e(Term::from_expr(s.get_expr(),pid));
      TsoVar v = (msg == 0 ? TsoVar(Lang::NML(s.get_memloc(),pid)) : TsoVar::msg(msg,pid));
      Predicate *cp = new Predicate(Predicate::eq(Term::variable(v), e));

      pbc2->temporary_predicates.push_back(cp);
      sorted_insert(&pbc2->ap_list, AppliedPredicate(cp,std::vector<TsoVar>()));

      result.push_back(pbc2);
      break;
    }
  case Lang::READASSIGN:
    {
      int msg = index_of(pid,s.get_memloc());

      PbConstraint *pbc2 = new PbConstraint(*this);

      TsoVar v = (msg == 0 ? TsoVar(Lang::NML(s.get_memloc(),pid)) : TsoVar::msg(msg,pid));
      TsoVar r = TsoVar::reg(s.get_reg(),pid);

      Predicate *cp = new Predicate(Predicate::eq(Term::variable(v), Term::variable(r)));

      pbc2->temporary_predicates.push_back(cp);
      sorted_insert(&pbc2->ap_list, AppliedPredicate(cp,std::vector<TsoVar>()));

      result.push_back(pbc2);    
      break;
    }
  case Lang::WRITE:
    {
      if(channels[pid].size() > 0 && channels[pid].back() == s.get_memloc()){
        Predicate *ppred = new Predicate(Predicate::eq(Term::variable(TsoVar::msg(channels[pid].size(),pid)),
                                                       Term::from_expr(s.get_expr(),pid)));
        PbConstraint *pbc2 = new PbConstraint(*this);
        pbc2->temporary_predicates.push_back(ppred);
        PbConstraint::sorted_insert(&pbc2->ap_list,AppliedPredicate(ppred,std::vector<TsoVar>()));
        result.push_back(pbc2);
      }
      break;
    }
  case Lang::ASSIGNMENT:
    {
      PbConstraint *pbc2 = new PbConstraint(*this);

      TsoVar r = TsoVar::reg(s.get_reg(),pid);

      Predicate *cp = new Predicate(Predicate::eq(Term::variable(r), Term::from_expr(s.get_expr(),pid)));

      pbc2->temporary_predicates.push_back(cp);
      sorted_insert(&pbc2->ap_list, AppliedPredicate(cp,std::vector<TsoVar>()));

      result.push_back(pbc2);
      break;
    }
  default:
    throw new std::logic_error("PbConstraint::range: Unhandled instruction: "+
                               s.to_string(common.machine.reg_pretty_vts(pid),
                                           common.machine.ml_pretty_vts(pid)));
  }

  return result;
}

std::list<const Machine::PTransition*> PbConstraint::partred() const{
  std::list<const Machine::PTransition*> l = common.all_enabled_by_pc(this);

  /* Remove updates which are not immediately after a read
   * And change writes into locked writes if the buffer is empty. */
  std::list<const Machine::PTransition*>::iterator it = l.begin();
  while(it != l.end()){
    int pid = (*it)->pid;
    int pc = pcs[pid];
    const Lang::Stmt<int> &i = (*it)->instruction;
    if(i.get_type() == Lang::UPDATE && !common.has_read[pid][pc]){
      it = l.erase(it);
    }else if(i.get_type() == Lang::WRITE && channels[pid].empty()){
      assert(common.atomized_writes.count(*it) == 1);
      *it = common.atomized_writes.at(*it);
    }else{
      it++;
    }
  }

  /* Remove transitions which are locked by a cycle lock */
#ifdef USE_TSO_CYCLE_LOCKS  
  static std::vector<bool> has_cycle_lock(pcs.size());
  static std::vector<bool> is_cycle_locked(pcs.size()); // Has at least one cycle lock, and all cycle locks are locked.
  for(unsigned i = 0; i < has_cycle_lock.size(); i++){
    has_cycle_lock[i] = false;
    is_cycle_locked[i] = true;
  }
  for(auto clit = cycle_locks.begin(); clit != cycle_locks.end(); clit++){
    has_cycle_lock[clit->get_pid()] = true;
    if(clit->is_unlocked()) is_cycle_locked[clit->get_pid()] = false;
  }
  for(unsigned i = 0; i < has_cycle_lock.size(); i++){
    if(!has_cycle_lock[i]) is_cycle_locked[i] = false;
  }
  it = l.begin();
  while(it != l.end()){
    if((*it)->instruction.get_type() == Lang::UPDATE){
      /* Updates are allowed */
      it++;
    }else if(is_cycle_locked[(*it)->pid]){
      it = l.erase(it);
    }else if(has_cycle_lock[(*it)->pid]){
      /* Check if this transition is guarded by an unlocked lock */
      bool locked = true;
      for(auto clit = cycle_locks.begin(); clit != cycle_locks.end(); clit++){
        if(clit->get_read() == *it){
          if(clit->is_unlocked())
            locked = false;
          break;
        }
      }
      if(locked){
        it = l.erase(it);
      }else{
        it++;
      }
    }else{
      /* This transition is not locked */
      it++;
    }
  }
#endif

#ifdef USE_CYCLE_LOCKS
  throw new std::logic_error("PbConstraint::partred(): CycleLocks are not supported!");

  static std::vector<bool> has_cycle_lock(pcs.size());
  static std::vector<bool> is_cycle_locked(pcs.size()); // Has at least one cycle lock, and all cycle locks are locked.
  if(pcs.size() != 2){
    // TODO: Fix this
    Log::warning << "Warning: Only 2 processes are supported!\n";
  }
  for(unsigned i = 0; i < has_cycle_lock.size(); i++){
    has_cycle_lock[i] = false;
    is_cycle_locked[i] = true;
  }
  for(std::list<CycleLock>::const_iterator clit = cycle_locks.begin(); 
      clit != cycle_locks.end(); clit++){
    has_cycle_lock[clit->pid] = true;
    if(clit->state == CycleLock::UNLOCKED)
      is_cycle_locked[clit->pid] = false;
  }
  for(unsigned i = 0; i < has_cycle_lock.size(); i++){
    if(!has_cycle_lock[i])
      is_cycle_locked[i] = false;
  }
  it = l.begin();
  while(it != l.end()){
    const Lang::Stmt<int> &s = (*it)->instruction;
    if(has_cycle_lock[(*it)->pid]){
      if(s.get_type() == Lang::READASSERT  || s.get_type() == Lang::READASSIGN){
        std::list<CycleLock>::const_iterator clit = cycle_locks.begin();
        bool locked = true;
        bool found = false;
        while(!found && clit != cycle_locks.end()){
          if(clit->pid == (*it)->pid && clit->v_read == Lang::NML(s.get_memloc(),(*it)->pid)){
            found = true;
            locked = (clit->state != CycleLock::UNLOCKED);
          }else{
            clit++;
          }
        }
        if(locked){
          // std::cout << "CycleLock: Blocked read.\n";
          it = l.erase(it);
        }else{
          // std::cout << "CycleLock: Allowed read through.\n";
          it++;
        }
      }else if(s.get_type() == Lang::UPDATE){
        // Let the update through
        it++;
      }else{
        // std::cout << "CycleLock: Blocked non-read.\n";
        it = l.erase(it);
      }
    }else{
      assert(pcs.size() == 2); // Or the below code will not work
      if(s.get_type() == Lang::UPDATE && 
         is_cycle_locked[1 - (*it)->pid]){
        it = l.erase(it);
      }else{
        it++;
      }
    }
  }
#endif
  

  /* Force a process to execute alone if the previous transition was
   * local.
   */
  if(last_trans.defined && last_trans.local && pcs[last_trans.pid] != 0){
    it = l.begin();
    while(it != l.end()){
      if((*it)->pid != last_trans.pid){
        it = l.erase(it);
      }else{
        it++;
      }
    }
  }
  
  return l;
};

void PbConstraint::clear_cycle_locks(int pid){
  std::list<TsoCycleLock>::iterator it = cycle_locks.begin();

  while(it != cycle_locks.end()){
    if(it->get_pid() == pid){
      it = cycle_locks.erase(it);
    }else{
      it++;
    }
  }
};

void PbConstraint::add_cycle_lock(const TsoCycleLock &cl){
  std::list<TsoCycleLock>::iterator it = cycle_locks.begin();

  while(it != cycle_locks.end() && *it < cl){
    it++;
  }

  assert(it == cycle_locks.end() || cl < *it);

  cycle_locks.insert(it,cl);
};

void PbConstraint::cycle_locks_execute(const Machine::PTransition *pt){
  for(auto it = cycle_locks.begin(); it != cycle_locks.end(); it++){
    it->execute(pt);
  }
};

const Machine::PTransition *PbConstraint::Common::find_transition(const Machine::PTransition &t) const{
  if(&all_transitions[0] <= &t && &t <= &all_transitions[all_transitions.size()-1]){
    /* t is already a reference into all_transitions */
    return &t;
  }else{
    /* Binary search */
    int a = 0;
    int b = all_transitions.size();
    while(b-a > 0){
      int c = all_transitions[(a+b)/2].compare(t);
      if(c < 0){
        a = (a+b)/2 + 1;
      }else if(c == 0){
        return &all_transitions[(a+b)/2];
      }else{
        b = (a+b)/2;
      }
    }
    return 0;
  }
}
