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

#include "tso_fencins.h"

#include <stdexcept>
#include <vector>
#include <algorithm>
#include <cassert>

namespace TsoFencins{

  /* Removes all cycles c from cycles, where c.write == w. */
  void remove_cycles_by_write(std::list<cycle_t> *cycles,const Machine::PTransition *w){
    auto it = cycles->begin();
    while(it != cycles->end()){
      if(it->write == w){
        it = cycles->erase(it);
      }else{
        it++;
      }
    }
  }

  Machine::PTransition get_transition_from_machine(const Machine &m, const Machine::PTransition &w){
    const std::set<Automaton::Transition*> &ft = m.automata[w.pid].get_states()[w.source].fwd_transitions;
    Lang::Stmt<int> wi = w.instruction;
    Lang::Stmt<int> wi2 = Lang::Stmt<int>::nop();
    Lang::Stmt<int> wi3 = Lang::Stmt<int>::nop();
    if(wi.get_type() == Lang::WRITE){
      wi2 = Lang::Stmt<int>::locked_block(std::vector<Lang::Stmt<int> >(1,wi),wi.get_pos());
      wi3 = Lang::Stmt<int>::slocked_block(std::vector<Lang::Stmt<int> >(1,wi),wi.get_pos());
    }else if(wi.get_type() == Lang::SLOCKED){
      assert(wi.get_statement_count() == 1);
      assert(wi.get_statement(0)->get_type() == Lang::WRITE);
      wi2 = *wi.get_statement(0);
      wi3 = Lang::Stmt<int>::locked_block(std::vector<Lang::Stmt<int> >(1,wi2),wi2.get_pos());
    }else{
      assert(wi.get_type() == Lang::LOCKED);
      assert(wi.get_statement_count() == 1);
      assert(wi.get_statement(0)->get_type() == Lang::WRITE);
      wi2 = *wi.get_statement(0);
      wi3 = Lang::Stmt<int>::slocked_block(std::vector<Lang::Stmt<int> >(1,wi2),wi2.get_pos());
    }
    int encountered = 0; // Number of times we have encountered a write matching w
    Machine::PTransition res = w;
    for(std::set<Automaton::Transition*>::const_iterator it = ft.begin(); it != ft.end(); it++){
      if((*it)->instruction == wi || (*it)->instruction == wi2 || (*it)->instruction == wi3){
        encountered++;
        if(encountered > 1){
          throw new std::logic_error("More than one identical write transition in machine.");
        }
        res = Machine::PTransition(**it,w.pid);
      }
    }
    assert(encountered == 1);
    return res;
  }

  /* Returns true iff the transition w in m is locked in m.
   *
   * Pre: w is a write (locked or non-locked) from m. w may differ
   * from the corresponding transition in m with respect to
   * lockedness.
   */
  bool is_locked(const Machine &m, const Machine::PTransition &w){
    return get_transition_from_machine(m,w).instruction.get_type() == Lang::LOCKED;
  }

  /* Iter should be an iterator to FenceSet.
   *
   * Returns true iff there is some FenceSet fs' in the interval
   * [beg,end) such that fs.includes(fs').
   */
  template<class Iter> bool subsumed(const FenceSet &fs,Iter beg, Iter end){
    return std::find_if(beg,end,
                        [&fs](const FenceSet &fs2){ return fs.includes(fs2); }) != end;
  }

  std::list<FenceSet> fencins(const Machine &m, Reachability &r,
                              reach_arg_init_t reach_arg_init, 
                              bool only_one){
    std::list<FenceSet> queue;
    queue.push_back(FenceSet(m));
    std::list<FenceSet> complete;

    Reachability::Result *result = 0;
    while(!queue.empty()){

      Log::msg << "Currently examining fence set:\n";
      queue.front().print(Log::msg,Log::null);
      Log::msg << std::endl;
      
      Reachability::Arg *next_arg = reach_arg_init(queue.front().get_atomized_machine(),result);
      Reachability::Result *tmp_result = r.reachability(next_arg);
      delete next_arg;
      if(result) delete result;
      result = tmp_result;
      Log::msg << result->to_string() << "\n" << std::flush;

      switch(result->result){
      case Reachability::REACHABLE:
        {
          Log::debug << " *** Error Trace (TSO) ***\n";
          result->trace->print(Log::debug,Log::extreme,Log::json,queue.front().get_atomized_machine());
          Log::debug << "\n";
          std::list<cycle_t> cycs = find_cycles(*result->trace);

          Log::msg << "Cycles found in trace:\n";
          for(const cycle_t &cycle : cycs){
            Log::msg << cycle.cycle.to_string(m) << "\n";
          }

          for(auto cycit = cycs.begin(); cycit != cycs.end(); cycit++){
            std::set<Machine::PTransition> cws = get_critical_writes(*cycit,*result->trace,queue.front().get_atomized_machine());
            cws.insert(*cycit->write);

            FenceSet fs = queue.front().atomize(*cycit,*result->trace);
            if(!subsumed(fs,++queue.begin(),queue.end()) && !subsumed(fs,complete.begin(),complete.end())){
              queue.push_back(fs);
            }
          }
        }
        break;
      case Reachability::UNREACHABLE:
        complete.push_back(queue.front());
        if(only_one){
          /* Remove all subsequente fence sets, thereby breaking the fencins loop */
          auto it = queue.begin();
          it++;
          while(it != queue.end()){
            it = queue.erase(it);
          }
        }
        break;
      case Reachability::FAILURE:
        throw new std::logic_error("TsoFencins::fencins: FAILURE in underlying reachability analysis.");
      }
      queue.pop_front();
    }
    assert(result);
    delete result;

    return complete;
  };

  std::map<const Machine::PTransition*,const Machine::PTransition*>
  pair_writes_with_updates(const Trace &trace){
    std::map<const Machine::PTransition*,const Machine::PTransition*> pairs;

    /* For each process p, pending_writes[p] contains the writes which
     * have been issued, but not yet updated for that process.
     *
     * Writes closer to the front are older.
     */
    std::vector<std::list<const Machine::PTransition*> > pending_writes(trace.get_proc_count());
    for(int t = 1; t <= trace.size(); t++){
      const Machine::PTransition *trans = trace.transition(t);
      const Lang::Stmt<int> &s = trans->instruction;
      int pid = trans->pid;
      
      switch(s.get_type()){
      case Lang::SLOCKED:
      case Lang::WRITE:
        pending_writes[pid].push_back(trans);
        break;
      case Lang::UPDATE: {
        for (auto write_iter = pending_writes[pid].begin(); ; ++write_iter) {
          if (write_iter == pending_writes[pid].end()) {
            throw new std::logic_error("TsoFencins: FAILURE: The given trace is not valid under PSO. (Try CEGAR?)");
          }
          if((*write_iter)->instruction.get_memloc() == s.get_memloc() && (*write_iter)->pid == trans->pid){
            pairs[*write_iter] = trans;
            pending_writes[pid].erase(write_iter);
            break;
          }
        }
      } break;
      default:
        // Do nothing
        break;
      }
    }
    for(unsigned p = 0; p < pending_writes.size(); p++){
      if(!pending_writes[p].empty()){
        throw new std::logic_error("TsoFencins: FAILURE: The given trace is not valid under PSO. (Try CEGAR?)");
      }
    }
    return pairs;
  }

  std::list<cycle_t> find_cycles(const Trace &trace){
    int proc_count = trace.get_proc_count();
    std::list<cycle_t> cycles;
    std::map<const Machine::PTransition*,const Machine::PTransition*> wrupdates = pair_writes_with_updates(trace);

    /* For each process p, pending_writes[p] contains the writes which
     * have been issued, but not yet updated for that process.
     *
     * Writes closer to the front are older.
     */
    std::vector<std::list<const Machine::PTransition*> > pending_writes(proc_count);
    /* Cycles c which are not complete and which are such that c.write
     * is in pending_writes.
     */
    std::list<cycle_t> pending_cycles;

    std::function<void(const Machine::PTransition*)> non_locked_read = 
      [&wrupdates,&pending_writes,&pending_cycles,proc_count](const Machine::PTransition *read){
      for(auto wit = pending_writes[read->pid].begin(); wit != pending_writes[read->pid].end(); wit++){
        TsoCycle tc(proc_count);
        assert(wrupdates.count(*wit));
        tc.push_back(wrupdates[*wit]);
        tc.push_back(read);
        cycle_t c(tc, *wit, read);
        pending_cycles.push_back(c);
      }
    };

    for(int t = 1; t <= trace.size(); t++){
      const Machine::PTransition *trans = trace.transition(t);
      const Lang::Stmt<int> &s = trans->instruction;
      int pid = trans->pid;

      auto cit = pending_cycles.begin();
      while(cit != pending_cycles.end()){
        if(cit->cycle.can_push_back(trans)){
          cycle_t c = *cit;
          c.cycle.push_back(trans);
          if(c.cycle.is_complete()){
            cycles.push_back(c);
            cit++;
          }else{
            cit = pending_cycles.insert(cit,c);
            cit++;
            cit++;
          }
        }else{
          cit++;
        }
      }

      switch(s.get_type()){
      case Lang::NOP: case Lang::ASSIGNMENT: case Lang::ASSUME:
      case Lang::SFENCE: case Lang::MFENCE:
        // Do nothing
        break;
      case Lang::READASSIGN: case Lang::READASSERT:
        non_locked_read(trans);
        break;
      case Lang::SLOCKED:
      case Lang::WRITE:
        pending_writes[pid].push_back(trans);
        break;
      case Lang::UPDATE:
        for (auto write_iter = pending_writes[pid].begin(); ; ++write_iter) {
          assert(write_iter != pending_writes[pid].end());
          if((*write_iter)->instruction.get_memloc() == s.get_memloc()){
            const Machine::PTransition *w = *write_iter;
            pending_writes[pid].erase(write_iter);
            remove_cycles_by_write(&pending_cycles,w);
            break;
          }
        }
        break;
      case Lang::LOCKED:
        if(s.get_reads().size() > 0 && !s.is_fence()){
          non_locked_read(trans);
        }
        break;
      default:
        throw new std::logic_error("TsoFencins::find_cycles: Illegal statement.");
      }
    }

    return cycles;
  }

  bool can_overtake(const Trace &trace, const Machine &m, int wi, int ri){
    /* Check order of wi and ri */
    if(ri <= wi) return false;
    /* Check that trace[ri] is a non-locked read. */
    if(trace[ri]->instruction.get_reads().empty() || trace[ri]->instruction.is_fence()){
      return false;
    }
    if(trace[wi]->instruction.get_type() == Lang::WRITE){
      /* Consider the corresponding update instead of w */
      std::map<const Machine::PTransition*,const Machine::PTransition*> wrupdates = pair_writes_with_updates(trace);
      const Machine::PTransition *upd = wrupdates[trace[wi]];
      for(wi = 1; trace[wi] != upd; wi++) ;
      if(wi > ri){
        /* Already overtaken */
        return true;
      }
    }else if(trace[wi]->instruction.get_type() == Lang::UPDATE){
      // Ok
    }else if(trace[wi]->instruction.get_type() == Lang::LOCKED){
      if(trace[wi]->instruction.get_statement_count() != 1 ||
         trace[wi]->instruction.get_statement(0)->get_type() != Lang::WRITE ||
         is_locked(m,*trace[wi])){
        return false;
      }
    }else{
      // Not a write or update
      return false;
    }

    int pid = trace[wi]->pid;
    if(trace[ri]->pid != pid){
      return false;
    }

    /* wi is the point in trace when the write w reaches memory.
     * ri is a non-locked read
     * wi < ri
     * w and r are by the same process pid
     */

    /* The set of memory locations where a value written by process
     * pid reaches memory between positions wi and ri in the trace 
     */
    std::set<Lang::NML> written;
    /* Check that there are no interfering instructions between wi and ri.
     * The following are considered interfering:
     *  - read by other process from memory location in written
     *  - read from process pid
     *  - fence from process pid
     */
    for(int i = wi+1; i < ri; i++){
      const Lang::Stmt<int> &s = trace[i]->instruction;
      if(trace[i]->pid == pid){
        if(s.get_type() == Lang::UPDATE){
          written.insert(Lang::NML(s.get_memloc(),pid));
        }else if(s.get_reads().size() > 0){
          return false;
        }else if(s.is_fence()){
          if(s.get_type() == Lang::LOCKED && 
             s.get_statement_count() == 1 &&
             s.get_statement(0)->get_type() == Lang::WRITE &&
             !is_locked(m,*trace[i])){
            /* This write is not a fence in m */
            written.insert(Lang::NML(s.get_statement(0)->get_memloc(),pid));
          }else{
            return false;
          }
        }
      }else{
        for(auto rit = s.get_reads().begin(); rit != s.get_reads().end(); rit++){
          if(written.count(Lang::NML(*rit,trace[i]->pid)) > 0){
            return false;
          }
        }
      }
    }
    return true;
   
  };
  
  std::set<Machine::PTransition>
  get_critical_writes(const cycle_t &cycle, const Trace &trace, const Machine &m){
    std::set<Machine::PTransition> s;
    s.insert(*cycle.write);
    if(cycle_no_extra_conflict(cycle,trace,m) && cycle_pairs_1_reordering(cycle,trace,m)){
      std::list<std::pair<const Machine::PTransition *,const Machine::PTransition*> > cws = cycle.cycle.get_critical_pairs();
      for(auto cwit = cws.begin(); cwit != cws.end(); ++cwit){
        const Machine::PTransition *w = trace[committed_index(trace,cwit->first)];
        if(s.count(*w) == 0 && cwit->first->instruction.get_writes().size() && 
           cwit->second->instruction.get_writes().size() == 0){
          VecSet<Lang::MemLoc<int> > wmls(cwit->first->instruction.get_writes());
          VecSet<Lang::MemLoc<int> > rmls(cwit->second->instruction.get_reads());
          if(!wmls.intersects(rmls) && !fence_between(trace,m,index(trace,cwit->first),index(trace,cwit->second))){
            s.insert(*w);
          }
        }
      }
    }
    return s;
  };

  FenceSet::FenceSet(const Machine &m,const std::set<Machine::PTransition> f)
    : machine(m), atomized_machine(m)
  {
    for(auto it = f.begin(); it != f.end(); it++){
      insert(*it);
    }
  };
  
  FenceSet::FenceSet(const Machine &m)
    : machine(m), atomized_machine(m)
  { 
  };

  void FenceSet::insert(const Machine::PTransition &t){
    if(t.instruction.get_type() != Lang::WRITE){
      throw new std::logic_error("TsoFencins::FenceSet::insert: Transition not a non-locked write.");
    }
    if(writes.count(t) == 0){
      /* Find the transition in atomized_machine */
      const std::vector<Automaton::State> &states = atomized_machine.automata[t.pid].get_states();
      const std::set<Automaton::Transition*> &ts = states[t.source].fwd_transitions;
#ifndef NDEBUG
      int changed = 0;
#endif
      for(auto it = ts.begin(); it != ts.end(); it++){
        if((*it)->target == t.target && (*it)->instruction == t.instruction){
          (*it)->instruction = Lang::Stmt<int>::locked_write(t.instruction.get_memloc(),t.instruction.get_expr(),
                                                             t.instruction.get_pos());
          assert(++changed == 1);
        }
      }
      assert(changed == 1);
      writes.insert(t);
    }// else we are done
  };

  FenceSet FenceSet::atomize(const cycle_t &cycle, const Trace &trace) const{
    std::set<Machine::PTransition> cws = get_critical_writes(cycle,trace,atomized_machine);
    cws.insert(*cycle.write);
    FenceSet fs(*this);
    for(auto it = cws.begin(); it != cws.end(); it++){
      Machine::PTransition t = *it;
      if(t.instruction.get_type() != Lang::WRITE){
        t = get_transition_from_machine(atomized_machine,t);
      }
      fs.insert(t);
    }
    return fs;
  };

  void FenceSet::print(Log::redirection_stream &text, Log::redirection_stream &json) const throw(){
    if(writes.empty()){
      text << "  (No fences)\n";
    }else{
      for(auto it = writes.begin(); it != writes.end(); it++){
        text << "  " << it->to_string(machine) << "\n";
        json << "json: {\"action\":\"Link Fence\", \"pos\":" << it->instruction.get_pos().to_json() << "}\n";
      }
    }
  };

  bool FenceSet::includes(const FenceSet &fs) const{
    return std::includes(writes.begin(),writes.end(),
                         fs.writes.begin(),fs.writes.end());
  };

  int index(const Trace &trace, const Machine::PTransition *t){
    for(int i = 1; i <= trace.size(); ++i){
      if(trace[i] == t)
        return i;
    }
    throw new std::logic_error("TsoFencins::index(trace,transition): Transition does not occur in trace.");
  };

  bool fence_between(const Trace &trace, const Machine &m, int wi, int ri){
    int pid = trace[wi]->pid;
    for(int i = wi; i <= ri; ++i){
      if(trace[i]->pid == pid && trace[i]->instruction.is_fence()){
        if(trace[i]->instruction.get_type() == Lang::LOCKED &&
           trace[i]->instruction.get_statement_count() == 1 &&
           trace[i]->instruction.get_statement(0)->get_type() == Lang::WRITE){
          if(is_locked(m,*trace[i])){
            return true;
          }
        }else{
          return true;
        }
      }
    }
    return false;
  };

  int committed_index(const Trace &trace, const Machine::PTransition *t){
    if(t->instruction.get_type() == Lang::UPDATE){
      auto wrupdates = pair_writes_with_updates(trace);
      for(auto it = wrupdates.begin(); it != wrupdates.end(); ++it){
        if(it->second == t){
          return index(trace,it->first);
        }
      }
      throw new std::logic_error("TsoFencins::committed_index: Update does not occur in trace.");
    }else{
      return index(trace,t);
    }
  };

  bool cycle_pairs_1_reordering(const cycle_t &cycle, const Trace &trace, const Machine &m){
    std::list<std::pair<const Machine::PTransition *,const Machine::PTransition*> > cps = cycle.cycle.get_critical_pairs();
    int reorderings = 0;
    for(auto cpit = cps.begin(); cpit != cps.end(); ++cpit){
      if((index(trace,cpit->first) < index(trace,cpit->second)) != 
         (committed_index(trace,cpit->first) < committed_index(trace,cpit->second))){
        reorderings++;
      }
    }
    return reorderings == 1;
  };

  std::pair<int,int> get_cycle_bounds(const cycle_t &cycle, const Trace &trace){
    int cbegin = trace.size() + 1;
    int cend = -1;
    for(int i = 0; i < cycle.cycle.size(); ++i){
      cbegin = std::min(cbegin,committed_index(trace,cycle.cycle[i]));
      cend = std::max(cend,index(trace,cycle.cycle[i]));
    }
    return std::pair<int,int>(cbegin,cend);
  };

  bool cycle_no_extra_conflict(const cycle_t &cycle, const Trace &trace, const Machine &m){
    std::pair<int,int> cbeginend = get_cycle_bounds(cycle,trace);
    int cbegin = cbeginend.first, cend = cbeginend.second;
    for(int i = cbegin+1; i < cend; ++i){
      /* is trace[i] in cycle? */
      bool in_cycle = false;
      for(int j = 0; j < cycle.cycle.size() && !in_cycle; ++j){
        if(cycle.cycle[j] == trace[i]){
          in_cycle = true;
        }
      }
      if(!in_cycle){
        /* Identify read and written memory locations */
        VecSet<Lang::NML> wi;
        VecSet<Lang::NML> ri;
        {
          std::vector<Lang::MemLoc<int> > wiml = trace[i]->instruction.get_writes();
          std::vector<Lang::MemLoc<int> > riml = trace[i]->instruction.get_reads();
          for(unsigned k = 0; k < wiml.size(); ++k){
            wi.insert(Lang::NML(wiml[k],trace[i]->pid));
          }
          for(unsigned k = 0; k < riml.size(); ++k){
            ri.insert(Lang::NML(riml[k],trace[i]->pid));
          }
        }
        /* Search for conflict */
        for(int j = cbegin; j <= cend; ++j){
          if(trace[j]->pid != trace[i]->pid){
            std::vector<Lang::MemLoc<int> > wj = trace[j]->instruction.get_writes();
            for(unsigned k = 0; k < wj.size(); ++k){
              Lang::NML nml(wj[k],trace[j]->pid);
              if(wi.count(nml) || ri.count(nml)) return false;
            }
            std::vector<Lang::MemLoc<int> > rj = trace[j]->instruction.get_reads();
            for(unsigned k = 0; k < rj.size(); ++k){
              if(wi.count(Lang::NML(rj[k],trace[j]->pid))) return false;
            }
          }
        }
      }
    }
    return true;
  };

}
