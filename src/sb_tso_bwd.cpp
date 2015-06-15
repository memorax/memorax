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

Trace *SbTsoBwd::convert_trace(Trace *trace, ChannelConstraint::Common *common) const{
  Trace *tso_trace = ChannelBwd::convert_trace(trace, common);

  Log::extreme << " *** SB trace ***\n";
  trace->print(Log::extreme,Log::extreme,Log::json,common->machine);
  Log::extreme << "\n\n";
  Log::extreme << " *** TSO trace ***\n";
  tso_trace->print(Log::extreme,Log::extreme,Log::json,common->machine);
  Log::extreme << "\n";

  return tso_trace;
};

bool SbTsoBwd::produces_message(const Lang::Stmt<int> &s) const{
  return (s.get_writes().size() > 0 &&
          s.get_type() != Lang::UPDATE);
};

bool SbTsoBwd::consumes_message(const Lang::Stmt<int> &s) const{
  return (s.get_type() == Lang::UPDATE ||
          (s.get_type() == Lang::LOCKED && s.get_writes().size() > 0));
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
  u = 0 : [0:10]
  v = 0 : [0:10]
  w = 0 : [0:10]
  x = 0 : [0:10]
  y = 0 : [0:10]
  z = 0 : [0:10]

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

    Log::set_primary_loglevel(llvl);
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
  u = 0 : [0:10]
  v = 0 : [0:10]
  w = 0 : [0:10]
  x = 0 : [0:10]
  y = 0 : [0:10]
  z = 0 : [0:10]

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

    Log::set_primary_loglevel(llvl);
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
  u = 0 : [0:10]
  v = 0 : [0:10]
  w = 0 : [0:10]
  x = 0 : [0:10]
  y = 0 : [0:10]
  z = 0 : [0:10]

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

    Log::set_primary_loglevel(llvl);
    Test::inner_test("convert_trace #3",eq_trace(tso_trace,sb_tso_trace));

    delete sb_trace;
    delete sb_tso_trace;
    delete tso_trace;
    delete m;
  }

  /* Test 4 */
  {
    Machine *m = get_machine(R"(
forbidden END END

data
  u = 0 : [0:10]
  v = 0 : [0:10]
  w = 0 : [0:10]
  x = 0 : [0:10]
  y = 0 : [0:10]
  z = 0 : [0:10]

process
text
  L0: read: x = 1;
  END: nop

process
text
  L0: locked write: x := 1;
  END: nop
)");
    Log::loglevel_t llvl = Log::get_primary_loglevel();
    Log::set_primary_loglevel(Log::loglevel_t(llvl-1));
    SbConstraint::Common c(*m);

    SbConstraint *sbc = get_sbc(c,{"END","END"},1,3);

    Trace *sb_trace =
      get_sb_trace(sbc,
                   {
                     {"P1 L0 END locked write: x := 1",{{0,-1,-1}}},
                     {"P0 L0 update P1 x",{{0,-1,-1},{1,3,-1}}},
                     {"P0 L0 END read: x = 1",{{1,3,-1}}},
                   });

    Trace *tso_trace = get_tso_trace(*m,
                                     {"P1 L0 END locked write: x := 1",
                                      "P0 L0 END read: x = 1"
                                      });

    SbTsoBwd stb;
    Trace *sb_tso_trace = stb.convert_trace(sb_trace,&c);

    Log::set_primary_loglevel(llvl);
    Test::inner_test("convert_trace #4",eq_trace(tso_trace,sb_tso_trace));

    delete sb_trace;
    delete sb_tso_trace;
    delete tso_trace;
    delete m;
  }

  /* Test 5: Lost messages, also locked writes */
  {
    Machine *m = get_machine(R"(
forbidden END END

data
  u = 0 : [0:10]
  v = 0 : [0:10]
  w = 0 : [0:10]
  x = 0 : [0:10]
  y = 0 : [0:10]
  z = 0 : [0:10]

process
text
  L0: write: x := 1;
  L1: write: y := 2;
  L2: locked write: x := 3;
  END: nop

process
text
  L0: read: y = 2;
  L1: read: x = 3;
  END: nop
)");
    Log::loglevel_t llvl = Log::get_primary_loglevel();
    Log::set_primary_loglevel(Log::loglevel_t(llvl-1));
    SbConstraint::Common c(*m);

    SbConstraint *sbc = get_sbc(c,{"END","END"},0,3);

    Trace *sb_trace =
      get_sb_trace(sbc,
                   {{"P0 L0 L1 write: x := 1",{{0,-1,-1}}},
                    {"P0 L1 update P0 x",{{0,-1,-1},{0,3,-1}}},
                    {"P0 L1 L2 write: y := 2",{{0,-1,-1},{0,3,-1}}},
                    {"P0 L2 update P0 y",{{0,-1,-1},{0,3,-1},{0,4,-1}}},
                    {"P0 L2 END locked write: x := 3",{{0,-1,-1},{0,3,-1},{0,4,-1}}},
                    {"P1 L0 update P0 y",{{0,-1,-1},{0,4,-1},{0,3,-1}}},
                    {"P1 L0 L1 read: y = 2",{{0,4,-1},{0,3,-1}}},
                    {"P1 L1 update P0 x",{{0,4,-1},{0,3,-1}}},
                    {"P1 L1 END read: x = 3",{{0,3,-1}}}});

    Trace *tso_trace = get_tso_trace(*m,
                                     {"P0 L0 L1 write: x := 1",
                                      "P0 L1 update P0 x",
                                      "P0 L1 L2 write: y := 2",
                                      "P0 L2 update P0 y",
                                      "P1 L0 L1 read: y = 2",
                                      "P0 L2 END locked write: x := 3",
                                      "P1 L1 END read: x = 3",
                                      });

    SbTsoBwd stb;
    Trace *sb_tso_trace = stb.convert_trace(sb_trace,&c);

    Log::set_primary_loglevel(llvl);
    Test::inner_test("convert_trace #5",eq_trace(tso_trace,sb_tso_trace));

    delete sb_trace;
    delete sb_tso_trace;
    delete tso_trace;
    delete m;
  }
};
