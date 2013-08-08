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

#include "sb_tso_bwd.h"

Trace *SbTsoBwd::convert_trace(Trace *trace, SbConstraint::Common *common) const{

  /* Check the order of writes */
  /* For each writing instruction w in trace, writes contains an entry
   * (i,m) where i is the index of w in trace and m is the
   * corresponding SB channel message.
   *
   * writes is ordered w.r.t. the indices.
   */
  std::vector<std::pair<int,SbConstraint::Msg> > writes;
  for(int i = 1; i <= trace->size(); ++i){
    if(produces_message(trace->transition(i)->instruction)){
      if(trace->constraint(i) == 0){
        throw new std::logic_error("SbTsoBwd::convert_trace: Trace is incomplete: Missing constraints.");
      }
      assert(dynamic_cast<const SbConstraint*>(trace->constraint(i)));
      const SbConstraint *sbc = static_cast<const SbConstraint*>(trace->constraint(i));
      writes.push_back(std::pair<int,SbConstraint::Msg>(i,sbc->channel[sbc->channel.size()-1]));
    }
  }

  /* Associate updates with writes */
  std::map<std::pair<int,int>,int> write_to_update; // write_to_update[(p,w)] is the update of process p corresponding to write w
  {
    std::vector<int> channel; // channel[i] is the index of the write producing message i
    std::vector<int> proc_seen_until(common->machine.automata.size(),-1); // Pointer into writes
    channel.push_back(-1); // The dummy message
    for(int i = 1; i <= trace->size(); ++i){
      if(trace->constraint(i-1) == 0 || trace->constraint(i) == 0){
        throw new std::logic_error("SbTsoBwd::convert_trace: Trace is incomplete: Missing constraints.");
      }
      assert(dynamic_cast<const SbConstraint*>(trace->constraint(i-1)));
      assert(dynamic_cast<const SbConstraint*>(trace->constraint(i)));
      const SbConstraint *sbc0 = static_cast<const SbConstraint*>(trace->constraint(i-1));
      const SbConstraint *sbc1 = static_cast<const SbConstraint*>(trace->constraint(i));
      if(trace->transition(i)->instruction.get_type() == Lang::UPDATE){
        if(sbc1->channel.size() != sbc0->channel.size()){
          assert(sbc1->channel.size() == sbc0->channel.size()-1);
          channel.erase(channel.begin());
        }
        int pid = trace->transition(i)->pid;
        int w = proc_seen_until[pid] + 1;
        int tgt_w = channel[sbc1->cpointers[pid]];
        while(writes[w].first != tgt_w){
          Log::debug << "Lost message (for process " << pid << "): " 
                     << trace->transition(writes[w].first)->to_string(common->machine) 
                     << "\n";
          write_to_update[std::pair<int,int>(pid,writes[w].first)] = i;
          ++w;
        }
        write_to_update[std::pair<int,int>(pid,writes[w].first)] = i;
        proc_seen_until[pid] = w;
      }else if(produces_message(trace->transition(i)->instruction)){
        if(sbc0->channel.size() + 1 == sbc1->channel.size()){
          /* No messages lost */
          channel.push_back(i);
        }else{
          assert(sbc1->channel.size() <= sbc0->channel.size());
          /* Messages lost */
          messages_lost(sbc0->channel, sbc1->channel, &channel, i, common);
        }
        if(trace->transition(i)->instruction.is_fence()){
          int pid = trace->transition(i)->pid;
          int w = proc_seen_until[pid] + 1;
          int tgt_w = channel[sbc1->cpointers[pid]];
          while(writes[w].first != tgt_w){
            Log::debug << "Lost message (for process " << pid << "): " 
                       << trace->transition(writes[w].first)->to_string(common->machine) 
                       << "\n";
            write_to_update[std::pair<int,int>(pid,writes[w].first)] = i;
            ++w;
          }
          assert(i == writes[w].first);
          write_to_update[std::pair<int,int>(pid,i)] = i;
          proc_seen_until[pid] = w;
        }
      }
      assert(channel.size() == sbc1->channel.size());
    }
#ifndef NDEBUG
    for(unsigned p = 0; p < common->machine.automata.size(); ++p){
      assert(proc_seen_until[p] == int(writes.size())-1);
    }
#endif
  }

  /* Produce a TSO trace */

  /* For each write produce first the section of the TSO trace that
   * preceeds the corresponding update. */
  Trace *tso_trace = new Trace(0);
  std::vector<int> proc_pos(common->machine.automata.size(),1);
  for(unsigned w = 0; w <= writes.size(); ++w){
    for(unsigned p = 0; p < common->machine.automata.size(); ++p){
      while((w == writes.size() && proc_pos[p] <= trace->size()) ||
            (w < writes.size() && write_to_update[std::pair<int,int>(p,writes[w].first)] != proc_pos[p])){
        if(trace->transition(proc_pos[p])->pid == int(p) &&
           !(trace->transition(proc_pos[p])->instruction.get_type() == Lang::UPDATE ||
             trace->transition(proc_pos[p])->instruction.get_type() == Lang::LOCKED)){
          tso_trace->push_back(*trace->transition(proc_pos[p]),0);
        }
        ++proc_pos[p];
      }
    }
    if (w < writes.size()) { /* Produce the update transition */
      int p = trace->transition(writes[w].first)->pid;
      tso_trace->push_back(*trace->transition(write_to_update[std::pair<int,int>(p,writes[w].first)]), 0);
    }
  }

  Log::extreme << " *** SB trace ***\n";
  trace->print(Log::extreme,Log::extreme,Log::json,common->machine);
  Log::extreme << "\n\n";
  Log::extreme << " *** TSO trace ***\n";
  tso_trace->print(Log::extreme,Log::extreme,Log::json,common->machine);
  Log::extreme << "\n";

  return tso_trace;
};

void SbTsoBwd::messages_lost(const std::vector<SbConstraint::Msg> &ch0,
                             const std::vector<SbConstraint::Msg> &ch1,
                             std::vector<int> *ch,
                             int w,
                             const SbConstraint::Common *common) const{
  assert(ch1.size() <= ch0.size());
  VecSet<int> to_remove; // Indices into ch
  for(auto it = common->messages.begin(); it != common->messages.end(); ++it){
    /* Count the number of occurrences of *it in ch0 and ch1 */
    int ch0_count = 0;
    int ch1_count = 0;
    int ch0_rmi = -1; // Rightmost index of *it in ch0
    for(unsigned i = 0; i < ch0.size(); ++i){
      if(ch0[i].wpid == it->wpid && ch0[i].nmls == it->nmls){
        ++ch0_count;
        ch0_rmi = i;
      }
    }
    for(unsigned i = 0; i < ch1.size(); ++i){
      if(ch1[i].wpid == it->wpid && ch1[i].nmls == it->nmls){
        ++ch1_count;
      }
    }
    bool is_w_msg = 
      (ch1.back().wpid == it->wpid && ch1.back().nmls == it->nmls);
    /* Has a message corresponding to *it been lost? */
    bool is_lost;
    if(is_w_msg){
      is_lost = (ch1_count <= ch0_count);
      if(is_lost) assert(ch1_count == ch0_count);
    }else{
      is_lost = (ch1_count < ch0_count);
      if(is_lost) assert(ch1_count == ch0_count - 1);
    }

    /* The lost message is the rightmost occurrence of *it in ch0 */
    to_remove.insert(ch0_rmi);
  }

  /* Remove messages in to_remove */
  int j = 0;
  for(unsigned i = 0; i < ch->size(); ++i){
    if(to_remove.count(i) > 0){
      // Do nothing
    }else{
      (*ch)[j] = (*ch)[i];
      ++j;
    }
  }
  assert(j + int(to_remove.size()) == int(ch->size()));
  ch->resize(j);

  ch->push_back(w);
  throw new std::logic_error("SbTsoBwd::messages_lost: Insufficiently tested.");
};

bool SbTsoBwd::produces_message(const Lang::Stmt<int> &s) const{
  return (s.get_writes().size() > 0 &&
          s.get_type() != Lang::UPDATE);
}
