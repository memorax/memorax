/*
 * Copyright (C) 2013 Magnus Lång
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

#include "pso_fencins.h"

namespace PsoFencins{

  /* Removes all cycles c from cycles, where c.write1 == w. */
  void remove_cycles_by_write(std::list<cycle_t> *cycles,const Machine::PTransition *w){
    auto it = cycles->begin();
    while(it != cycles->end()){
      if(it->write1 == w){
        it = cycles->erase(it);
      }else{
        it++;
      }
    }
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
                              TsoFencins::reach_arg_init_t reach_arg_init,
                              bool only_one){
    Log::debug << "Only one: " << only_one << "\n";
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
      if (result) delete result;
      result = tmp_result;
      Log::msg << result->to_string() << "\n" << std::flush;

      switch (result->result) {
      case Reachability::REACHABLE:
        {
          Log::debug << " *** Error Trace (PSO) ***\n";
          result->trace->print(Log::debug,Log::extreme,Log::json,queue.front().get_atomized_machine());
          Log::debug << "\n";
          std::list<cycle_t> cycs = find_cycles(*result->trace);

          Log::msg << "Cycles found in trace:\n";
          for(const cycle_t &cycle : cycs){
            Log::msg << cycle.cycle.to_string(m) << "\n";
          }

          for (const cycle_t &cycle : cycs) {
            FenceSet fs = queue.front().atomize(cycle, *result->trace);
            if (!subsumed(fs, ++queue.begin(), queue.end()) && !subsumed(fs, complete.begin(), complete.end())) {
              queue.push_back(fs);
            }
          }
        }
        break;
      case Reachability::UNREACHABLE:
        complete.push_back(queue.front());
        if (only_one) {
          /* Remove all subsequente fence sets, thereby breaking the fencins loop */
          for (auto it = ++queue.begin(); it != queue.end(); )
            it = queue.erase(it);
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

  std::list<cycle_t> find_cycles(const Trace &trace){
    int proc_count = trace.get_proc_count();
    std::list<cycle_t> cycles;
    {
      std::list<TsoFencins::cycle_t> rwcycles = TsoFencins::find_cycles(trace);
      std::copy(rwcycles.begin(), rwcycles.end(), std::back_inserter(cycles));
    }
    std::map<const Machine::PTransition*,const Machine::PTransition*> wrupdates = TsoFencins::pair_writes_with_updates(trace);

    /* For each process p, pending_writes[p] contains the writes which
     * have been issued, but not yet updated for that process.
     *
     * Writes closer to the front are older.
     */
    std::vector<std::list<const Machine::PTransition*> > pending_writes(proc_count);

    /* For each memory location ml, last_writes[ml] = (w,u) is the last write in
     * trace to ml that so far has updated. w is the write transition and u is
     * the update transition.
     */
    std::map<Lang::NML, std::pair<const Machine::PTransition*,const Machine::PTransition*>> last_writes;

    /* Cycles c which are not complete and which are such that c.write
     * is in pending_writes.
     */
    std::list<cycle_t> pending_cycles;

    std::function<void(const Machine::PTransition*)> any_read =
      [&wrupdates,&pending_writes,&pending_cycles,&last_writes,proc_count,&trace](const Machine::PTransition *read){
      assert(read->instruction.get_reads().size() == 1);
      Lang::NML nml(read->instruction.get_reads()[0], read->pid);
      if (last_writes.count(nml)) {
        for (auto inner : pending_writes) for (const Machine::PTransition *w1 : inner) {
          if (w1->pid == last_writes[nml].second->pid &&
              TsoFencins::index(trace, w1) < TsoFencins::index(trace, last_writes[nml].second)) {
            TsoCycle tc(proc_count);
            tc.push_back(wrupdates[w1]);
            tc.push_back(last_writes[nml].second);
            tc.push_back(read);
            cycle_t c(tc, w1, 0, last_writes[nml].first, read);
            pending_cycles.push_back(c);
          }
        }
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
        // Do nothing
        break;
      case Lang::READASSIGN: case Lang::READASSERT:
        any_read(trans);
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
            last_writes[Lang::NML(s.get_memloc(), pid)] = {w, trans};
            break;
          }
        }
        break;
      case Lang::LOCKED:
        assert(!s.is_fence() || pending_writes[pid].empty());
        if(s.get_reads().size() > 0){
          any_read(trans);
        }
        if (s.get_writes().size() > 0) {
          assert(s.get_writes().size() == 1);
          for (const Lang::MemLoc<int> ml : s.get_writes()) {
            last_writes[Lang::NML(ml, pid)] = {trans, trans};
          }
        }
        break;
      default:
        throw new std::logic_error("PsoFencins::find_cycles: Illegal statement.");
      }
    }

    return cycles;
  }

  FenceSet FenceSet::atomize(const cycle_t &cycle, const Trace &trace) const {
    std::set<Machine::PTransition> cws =
      TsoFencins::get_critical_writes(cycle.to_tso(), trace, atomized_machine);
    cws.insert(*cycle.write1);
    FenceSet fs(*this);
    for (Machine::PTransition t : cws) {
      if (t.instruction.get_type() != Lang::WRITE) {
        t = TsoFencins::get_transition_from_machine(atomized_machine, t);
      }
      if (cycle.is_write_write)
        fs.insert_slock(t);
      else
        fs.insert_mlock(t);
    }
    return fs;
}

  void FenceSet::print(Log::redirection_stream &text, Log::redirection_stream &json) const {
    if (slocks.empty() && mlocks.empty()) {
      text << "  (No fences)\n";
    } else {
      if (!slocks.empty()) text << "  Store-Store fences:\n";
      for (const Machine::PTransition &trans : slocks) {
        text <<  "    " + trans.to_string(machine) + "\n";
        json << "json: {\"action\":\"Link Fence\", \"type\":\"store-store\", \"pos\":"
             << trans.instruction.get_pos().to_json() << "}\n";;
      }
      if (!mlocks.empty()) text << "  Load-Store fences:\n";
      for (const Machine::PTransition &trans : mlocks) {
        text << "    " + trans.to_string(machine) + "\n";
        json << "json: {\"action\":\"Link Fence\", \"type\":\"load-store\", \"pos\":"
             << trans.instruction.get_pos().to_json() << "}\n";;
      }
    }
  }

  bool FenceSet::includes(const FenceSet &fs) const {
    for (const Machine::PTransition &slock : fs.slocks)
      if (!slocks.count(slock) && !mlocks.count(slock)) return false;
    for (const Machine::PTransition &mlock : fs.mlocks)
      if (!mlocks.count(mlock)) return false;
    return true;
  }

  void FenceSet::insert_slock(const Machine::PTransition &t) {
    if (t.instruction.get_type() != Lang::WRITE) {
      throw new std::logic_error("PsoFencins::FenceSet::insert_slock: Transition not a non-locked write.");
    }
    Machine::PTransition baset = TsoFencins::get_transition_from_machine(machine, t);
    if (slocks.count(baset) == 0 && mlocks.count(baset) == 0) {
      /* Find the transition in atomized_machine */
      const std::vector<Automaton::State> &states = atomized_machine.automata[t.pid].get_states();
      const std::set<Automaton::Transition*> &ts = states[t.source].fwd_transitions;
#ifndef NDEBUG
      int changed = 0;
#endif
      for (Automaton::Transition *mt : ts) {
        if (mt->target == t.target && mt->instruction == t.instruction) {
          const Lang::Stmt<int> *write = &t.instruction;
          mt->instruction = Lang::Stmt<int>::slocked_write(write->get_memloc(),write->get_expr(),
                                                           write->get_pos());
          assert(++changed == 1);
        }
      }
      assert(changed == 1);
      slocks.insert(baset);
    } // else we are done
  }

  void FenceSet::insert_mlock(const Machine::PTransition &t) {
    if (t.instruction.get_type() != Lang::WRITE && t.instruction.get_type() != Lang::SLOCKED) {
      throw new std::logic_error("PsoFencins::FenceSet::insert_slock: Transition not a non-locked write.");
    }
    Machine::PTransition baset = TsoFencins::get_transition_from_machine(machine, t);
    if (mlocks.count(baset) == 0) {
      /* Find the transition in atomized_machine */
      const std::vector<Automaton::State> &states = atomized_machine.automata[t.pid].get_states();
      const std::set<Automaton::Transition*> &ts = states[t.source].fwd_transitions;
#ifndef NDEBUG
      int changed = 0;
#endif
      for (Automaton::Transition *mt : ts) {
        if (mt->target == t.target && mt->instruction == t.instruction) {
          const Lang::Stmt<int> *write = &t.instruction;
          if (t.instruction.get_type() == Lang::SLOCKED) {
            assert(t.instruction.get_statement_count() == 1);
            write = t.instruction.get_statement(0);
          }
          mt->instruction = Lang::Stmt<int>::locked_write(write->get_memloc(),write->get_expr(),
                                                          write->get_pos());
          assert(++changed == 1);
        }
      }
      assert(changed == 1);
      slocks.erase(baset);
      mlocks.insert(baset);
    } // else we are done
  }
}
