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

#include "parser.h"
#include "preprocessor.h"
#include "sb_tso_bwd.h"
#include "test.h"

#include <cctype>
#include <functional>
#include <sstream>

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
    if(trace->transition(i)->instruction.get_writes().size() > 0 &&
       trace->transition(i)->instruction.get_type() != Lang::UPDATE){
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
      }else if(trace->transition(i)->instruction.get_writes().size() > 0){
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
    /* Run all processes up to the point where they update w.r.t. writes[w] */
    for(unsigned p = 0; p < common->machine.automata.size(); ++p){
      while((w == writes.size() && proc_pos[p] <= trace->size()) ||
            (w < writes.size() && write_to_update[std::pair<int,int>(p,writes[w].first)] != proc_pos[p])){
        if(trace->transition(proc_pos[p])->pid == int(p) &&
           trace->transition(proc_pos[p])->instruction.get_type() != Lang::UPDATE){
          tso_trace->push_back(*trace->transition(proc_pos[p]),0);
        }
        ++proc_pos[p];
      }
    }
    if(w < writes.size()){
      /* Perform the update */
      int wpid = writes[w].second.wpid;
      int uindex = write_to_update[std::pair<int,int>(wpid,writes[w].first)];
      assert((*trace)[uindex]->instruction.get_type() == Lang::UPDATE);
      int q = (*trace)[uindex]->source;
      VecSet<Lang::MemLoc<int> > mls;
      for(Lang::NML nml : writes[w].second.nmls){
        mls.insert(nml.localize(wpid));
      }
      tso_trace->push_back({q,Lang::Stmt<int>::update(wpid,mls),q,wpid},0);
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

    if(is_lost){
      /* The lost message is the rightmost occurrence of *it in ch0 */
      to_remove.insert(ch0_rmi);
    }
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
};

void SbTsoBwd::test(){
  std::function<Machine*(std::string)> get_machine =
    [](std::string rmm){
    std::stringstream ss(rmm);
    PPLexer lex(ss);
    return new Machine(Parser::p_test(lex));
  };
  struct step_t{
    Machine::PTransition trans;
    SbConstraint *sbc;
  };
  struct msg_t{
    int pid; // writer
    int ml; // NML::global(ml), -1 for initial message
    int val; // -1 for wild value
  };
  std::function<Machine::PTransition(const Machine*,std::string)> trans =
    [&get_machine](const Machine *machine, std::string strans)->Machine::PTransition{
    while(strans.size() && std::isspace(strans[0])) strans = strans.substr(1);
    while(strans.size() && std::isspace(strans[strans.size()-1])) strans = strans.substr(0,strans.size()-1);
    assert(strans.size() && strans[0] == 'P');
    strans = strans.substr(1);
    std::stringstream ss(strans);
    int pid;
    ss >> pid;
    assert(ss);
    std::string src_lbl, tgt_lbl;
    ss >> src_lbl >> tgt_lbl;
    assert(machine->automata[pid].get_labels().count(src_lbl));
    int q_src = machine->automata[pid].get_labels().at(src_lbl);
    if(tgt_lbl == "update"){
      std::string swpid, svar;
      int wpid;
      ss >> swpid >> svar;
      assert(swpid.size()>1 && swpid[0] == 'P');
      {
        std::stringstream swpidss(swpid.substr(1));
        swpidss >> wpid;
        assert(swpidss);
      }
      assert(svar.size() == 1);
      assert('u' <= svar[0] && svar[0] <= 'z');
      Lang::MemLoc<int> ml = Lang::MemLoc<int>::global(svar[0] - 'u');
      return {q_src,Lang::Stmt<int>::update(wpid,VecSet<Lang::MemLoc<int> >::singleton(ml)),q_src,pid};
    }else{
      std::string sinstr;
      std::getline(ss,sinstr);
      std::string prefix = R"(
forbidden *
data
  u = *
  v = *
  w = *
  x = *
  y = *
  z = *
process
text
)";
      Machine *m2 = get_machine(prefix+sinstr);
      Lang::Stmt<int> instr = (*m2->automata[0].get_states()[0].fwd_transitions.begin())->instruction;
      std::string instr_ts = instr.to_string(Lang::int_reg_to_string(),
                                             Lang::int_memloc_to_string());
      delete m2;
      assert(machine->automata[pid].get_labels().count(tgt_lbl));
      int q_tgt = machine->automata[pid].get_labels().at(tgt_lbl);
      for(auto t : machine->automata[pid].get_states()[q_src].fwd_transitions){
        std::string s = t->instruction.to_string(Lang::int_reg_to_string(),
                                                 Lang::int_memloc_to_string());
        if(t->target == q_tgt && instr_ts == s){
          return Machine::PTransition(*t,pid);
        }
      }
      throw new std::logic_error("SbTsoBwd::test::trans: No such transition.");
    }
  };
  std::function<step_t(SbConstraint::Common&,const SbConstraint*,
                       std::string, std::vector<msg_t>)> step =
    [&trans](SbConstraint::Common &common,const SbConstraint *sbc,
             std::string strans, std::vector<msg_t> tgt_buf){
    const Machine *machine = &common.machine;
    Machine::PTransition pt = trans(machine,strans);
    SbConstraint *sbc2 = 0;

    auto sbcs = sbc->pre(pt);
    for(auto c : sbcs){
      SbConstraint *sc = static_cast<SbConstraint*>(c);
      /* Check if sc->channel matches tgt_buf */
      bool match = true;
      if(sc->channel.size() == tgt_buf.size()){
        for(unsigned i = 0; i < sc->channel.size(); ++i){
          if(tgt_buf[i].ml == -1){
            if(sc->channel[i].nmls.size()) match = false;
          }else{
            if(sc->channel[i].wpid != tgt_buf[i].pid) match = false;
            if(sc->channel[i].nmls.size() != 1 ||
               sc->channel[i].nmls[0] != Lang::NML::global(tgt_buf[i].ml)) match = false;
            if(match){
              ZStar<int> val = sc->channel[i].store[common.index(Lang::NML::global(tgt_buf[i].ml))];
              if(!val.is_star() && tgt_buf[i].val != -1 && val.get_int() != tgt_buf[i].val) match = false;
            }
          }
        }
      }else{
        match = false;
      }
      if(match){
        assert(sbc2 == 0);
        sbc2 = sc;
      }else{
        delete sc;
      }
    }

    if(sbc2 == 0){
      throw new std::logic_error("SbTsoBwd::test::step: No such constraint.");
    }

    step_t st = {pt,sbc2};
    return st;
  };

  std::function<SbConstraint*(SbConstraint::Common&,std::vector<std::string>,int,int)> get_sbc =
    [](SbConstraint::Common &common,std::vector<std::string> lbls,int wpid,int ml){
    std::vector<int> pcs;
    {
      for(unsigned p = 0; p < lbls.size(); ++p){
        pcs.push_back(common.machine.automata[p].get_labels().at(lbls[p]));
      }
    }
    SbConstraint::Common::MsgHdr msg(wpid,VecSet<Lang::NML>::singleton(Lang::NML::global(ml)));
    return new SbConstraint(pcs,msg,common);
  };

  std::function<Trace*(SbConstraint*,
                       std::vector<std::pair<std::string,std::vector<msg_t> > >)> get_sb_trace =
    [&step](SbConstraint *end_sbc,
            std::vector<std::pair<std::string,std::vector<msg_t> > > tvec){
    SbConstraint::Common &common = end_sbc->common;
    Trace *trace = new Trace(end_sbc);
    SbConstraint *sbc = end_sbc;
    for(int i = int(tvec.size())-1; 0 <= i; --i){
      auto st = step(common,sbc,tvec[i].first,tvec[i].second);
      trace->push_front(st.sbc,st.trans);
      sbc = st.sbc;
    }
    return trace;
  };

  std::function<Trace*(const Machine&,std::vector<std::string>)> get_tso_trace =
    [&trans](const Machine &machine,std::vector<std::string> tvec){
    Trace *trace = new Trace(0);
    for(std::string s : tvec){
      trace->push_back(trans(&machine,s),0);
    }
    return trace;
  };

  std::function<bool(const Trace *,const Trace *)> eq_trace =
    [](const Trace *t0, const Trace *t1){
    if(t0->size() != t1->size()){
      return false;
    }

    for(int i = 1; i <= t0->size(); ++i){
      if((*t0)[i]->to_raw_string(Machine::PTransition::SS_CONTROL_STATES) !=
         (*t1)[i]->to_raw_string(Machine::PTransition::SS_CONTROL_STATES)){
        return false;
      }
    }

    return true;
  };

  /* Test 1 */
  {
    Machine *m = get_machine(R"(
forbidden CS CS

data
  u = 0 : [0:1]
  v = 0 : [0:1]
  w = 0 : [0:1]
  x = 0 : [0:1]
  y = 0 : [0:1]
  z = 0 : [0:1]

process
text
  L0: write: x := 1;
  L1: read: y = 0;
  CS: write: x := 0;
  goto L0

process
text
  L0: write: y := 1;
  L1: read: x = 0;
  CS: write: y := 0;
  goto L0
)");
    Log::loglevel_t llvl = Log::get_primary_loglevel();
    Log::set_primary_loglevel(Log::loglevel_t(llvl-1));
    SbConstraint::Common c(*m);
    Log::set_primary_loglevel(llvl);

    SbConstraint *sbc = get_sbc(c,{"CS","CS"},0,3);

    Trace *sb_trace =
      get_sb_trace(sbc,
                   {{"P1 L0 L1 write: y := 1",{{0,-1,-1}}},
                    {"P1 L1 update P1 y",{{0,-1,-1},{1,4,-1}}},
                    {"P0 L0 L1 write: x := 1",{{0,-1,-1},{1,4,-1}}},
                    {"P0 L1 CS read: y = 0",{{0,-1,-1},{1,4,-1},{0,3,-1}}},
                    {"P0 CS update P1 y",{{0,-1,-1},{1,4,-1},{0,3,-1}}},
                    {"P1 L1 CS read: x = 0",{{1,4,-1},{0,3,-1}}},
                    {"P0 CS update P0 x",{{1,4,-1},{0,3,-1}}},
                    {"P1 CS update P0 x",{{1,4,-1},{0,3,-1}}}});

    Trace *tso_trace = get_tso_trace(*m,
                                     {"P0 L0 L1 write: x := 1",
                                      "P0 L1 CS read: y = 0",
                                      "P1 L0 L1 write: y := 1",
                                      "P1 L1 update P1 y",
                                      "P1 L1 CS read: x = 0",
                                      "P0 CS update P0 x"
                                      });

    SbTsoBwd stb;
    Trace *sb_tso_trace = stb.convert_trace(sb_trace,&c);

    Test::inner_test("convert_trace #1",eq_trace(tso_trace,sb_tso_trace));

    delete sb_trace;
    delete sb_tso_trace;
    delete tso_trace;
    delete m;
  }

  /* Test 2: One process, more writes than updates */
  {
    Machine *m = get_machine(R"(
forbidden END

data
  u = 0 : [0:1]
  v = 0 : [0:1]
  w = 0 : [0:1]
  x = 0 : [0:1]
  y = 0 : [0:1]
  z = 0 : [0:1]

process
text
  L0: write: x := 1;
  L1: write: y := 2;
  L2: write: x := 3;
  END: nop
)");
    Log::loglevel_t llvl = Log::get_primary_loglevel();
    Log::set_primary_loglevel(Log::loglevel_t(llvl-1));
    SbConstraint::Common c(*m);
    Log::set_primary_loglevel(llvl);

    SbConstraint *sbc = get_sbc(c,{"END"},0,3);

    Trace *sb_trace =
      get_sb_trace(sbc,
                   {{"P0 L0 L1 write: x := 1",{{0,-1,-1}}},
                    {"P0 L1 L2 write: y := 2",{{0,-1,-1},{0,3,-1}}},
                    {"P0 L2 END write: x := 3",{{0,-1,-1},{0,3,-1},{0,4,-1}}},
                    {"P0 END update P0 y",{{0,-1,-1},{0,4,-1},{0,3,-1}}},
                    {"P0 END update P0 x",{{0,4,-1},{0,3,-1}}}});

    Trace *tso_trace = get_tso_trace(*m,
                                     {"P0 L0 L1 write: x := 1",
                                      "P0 L1 L2 write: y := 2",
                                      "P0 L2 END write: x := 3",
                                      "P0 END update P0 x",
                                      "P0 END update P0 y",
                                      "P0 END update P0 x"
                                      });

    SbTsoBwd stb;
    Trace *sb_tso_trace = stb.convert_trace(sb_trace,&c);

    Test::inner_test("convert_trace #2",eq_trace(tso_trace,sb_tso_trace));

    delete sb_trace;
    delete sb_tso_trace;
    delete tso_trace;
    delete m;
  }

  /* Test 3 */
  {
    Machine *m = get_machine(R"(
forbidden END END

data
  u = 0 : [0:1]
  v = 0 : [0:1]
  w = 0 : [0:1]
  x = 0 : [0:1]
  y = 0 : [0:1]
  z = 0 : [0:1]

process
text
  L0: read: x = 1;
  END: nop

process
text
  L0: write: x := 1;
  END: nop
)");
    Log::loglevel_t llvl = Log::get_primary_loglevel();
    Log::set_primary_loglevel(Log::loglevel_t(llvl-1));
    SbConstraint::Common c(*m);
    Log::set_primary_loglevel(llvl);

    SbConstraint *sbc = get_sbc(c,{"END","END"},1,3);

    Trace *sb_trace =
      get_sb_trace(sbc,
                   {
                     {"P1 L0 END write: x := 1",{{0,-1,-1}}},
                     {"P0 L0 update P1 x",{{0,-1,-1},{1,3,-1}}},
                     {"P0 L0 END read: x = 1",{{0,-1,-1},{1,3,-1}}},
                     {"P1 END update P1 x",{{0,-1,-1},{1,3,-1}}}
                   });

    Trace *tso_trace = get_tso_trace(*m,
                                     {"P1 L0 END write: x := 1",
                                      "P1 END update P1 x",
                                      "P0 L0 END read: x = 1"
                                      });

    SbTsoBwd stb;
    Trace *sb_tso_trace = stb.convert_trace(sb_trace,&c);

    Test::inner_test("convert_trace #3",eq_trace(tso_trace,sb_tso_trace));

    delete sb_trace;
    delete sb_tso_trace;
    delete tso_trace;
    delete m;
  }
};
