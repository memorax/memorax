/*
 * Copyright (C) 2014 Carl Leonardsson
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

#include "fencins.h"
#include "log.h"
#include "preprocessor.h"
#include "test.h"
#include "test_vips_fencins.h"
#include "vips_bit_reachability.h"
#include "vips_simple_fencer.h"

#include <functional>
#include <sstream>

namespace TestVipsFencins{

  Machine *get_machine(std::string rmm){
    std::stringstream ss(rmm);
    PPLexer lex(ss);
    return new Machine(Parser::p_test(lex));
  };

  int get_cs(const Machine *m, int pid, std::string lbl){
    return m->automata[pid].state_index_of_label(lbl);
  };

  struct fshort{
    int pid;
    int q;
    std::string fence;
    bool operator==(const fshort &f) const{
      return pid == f.pid && q == f.q && fence == f.fence;
    };
    bool operator<(const fshort &f) const{
      if(pid != f.pid) return pid < f.pid;
      if(q != f.q) return q < f.q;
      return fence < f.fence;
    };
  };
  typedef std::set<std::set<fshort> > fsets_t;

  fsets_t summarize_syncs(const Machine *m, const std::set<std::set<Sync*> > &syncs){
    fsets_t FS;
    for(auto it = syncs.begin(); it != syncs.end(); ++it){
      std::set<fshort> F;
      for(auto s : *it){
        if(dynamic_cast<VipsFullFenceSync*>(s)){
          VipsFullFenceSync *f = static_cast<VipsFullFenceSync*>(s);
          F.insert({f->get_pid(),f->get_q(),"fence"});
        }else if(dynamic_cast<VipsSSFenceSync*>(s)){
          VipsSSFenceSync *f = static_cast<VipsSSFenceSync*>(s);
          F.insert({f->get_pid(),f->get_q(),"ssfence"});
        }else if(dynamic_cast<VipsLLFenceSync*>(s)){
          VipsLLFenceSync *f = static_cast<VipsLLFenceSync*>(s);
          F.insert({f->get_pid(),f->get_q(),"llfence"});
        }else if(dynamic_cast<VipsSyncwrSync*>(s)){
          VipsSyncwrSync *f = static_cast<VipsSyncwrSync*>(s);
          F.insert({f->get_pid(),f->get_write().source,"syncwr"});
        }else{
          assert(dynamic_cast<VipsSyncrdSync*>(s));
          VipsSyncrdSync *f = static_cast<VipsSyncrdSync*>(s);
          F.insert({f->get_pid(),f->get_read().source,"syncrd"});
        }
      }
      FS.insert(F);
    }
    return FS;
  };

  int s2i(const std::string &s){
    int i;
    std::stringstream ss(s);
    ss >> i;
    return i;
  };

  fsets_t summarize_syncs(const Machine *m, const std::set<std::set<std::string> > &syncs){
    fsets_t FS;
    for(auto it = syncs.begin(); it != syncs.end(); ++it){
      std::set<fshort> F;
      for(std::string s : *it){
        int pid, q;
        if(s[0] != 'P'){
          throw new std::logic_error("summarize_syncs: Syntax error in"+s+
                                     ". Expected P.");
        }
        int i = 1;
        while(i < int(s.size()) && '0' <= s[i] && s[i] <= '9') ++i;
        if(i == 1){
          throw new std::logic_error("summarize_syncs: Syntax error in "+s+
                                     ". Expected pid.");
        }
        pid = s2i(s.substr(1,i-1));
        if(int(s.size()) <= i || s[i] != ' '){
          throw new std::logic_error("summarize_syncs: Syntax error in "+s+
                                     ". Expected space.");
        }
        ++i;
        int j = i;
        while(j < int(s.size()) && s[j] != ' ') ++j;
        std::string lbl = s.substr(i,j-i);
        try{
          q = get_cs(m,pid,lbl);
        }catch(Automaton::UnDefinedLabel *exc){
          delete exc;
          throw new std::logic_error("summarize_syncs: Undefined label '"+lbl+"'.");
        }
        F.insert({pid,q,s.substr(j+1)});
      }
      FS.insert(F);
    }
    return FS;
  };

  void print_summary(const fsets_t &FS){
    if(FS.empty()){
      Log::debug << "(No fence sets)\n";
      return;
    }
    int ctr = 0;
    for(auto it = FS.begin(); it != FS.end(); ++it){
      ++ctr;
      Log::debug << "Fence set #" << ctr << "\n";
      for(auto pr : *it){
        Log::debug << "  P" << pr.pid << " Q" << pr.q
                   << ": " << pr.fence << "\n";
      }
    }
  };

  bool check_syncs(const Machine *m,
                   const std::set<std::set<Sync*> > &syncs,
                   const std::set<std::set<std::string> > &expected){
    fsets_t A = summarize_syncs(m,syncs);
    fsets_t B = summarize_syncs(m,expected);
    bool retval = (A == B);
    if(!retval){
      Log::debug << "\nWrong value in fence insertion:\n";
      Log::debug << "\nExpected:\n";
      print_summary(B);
      Log::debug << "\nReceived:\n";
      print_summary(A);
      Log::debug << "\n";
    }
    return retval;
  };

  bool test_fencins(const std::string &rmm,
                    const std::set<std::set<std::string> > &fsets,
                    const std::vector<int> &costvec = {10,5,5,1,1}){
      Machine *m = get_machine(rmm);

      VipsBitReachability reach;

      Fencins::reach_arg_init_t arg_init =
        [](const Machine &m, const Reachability::Result *)->Reachability::Arg*{
        return new Reachability::Arg(m);
      };

      VipsSimpleFencer tf(*m);

      Fencins::cost_fn_t costfn =
        [&costvec](const Sync *s){
        if(dynamic_cast<const VipsFullFenceSync*>(s)){
          return costvec[0];
        }else if(dynamic_cast<const VipsSSFenceSync*>(s)){
          return costvec[1];
        }else if(dynamic_cast<const VipsLLFenceSync*>(s)){
          return costvec[2];
        }else if(dynamic_cast<const VipsSyncwrSync*>(s)){
          return costvec[3];
        }else{
          assert(dynamic_cast<const VipsSyncrdSync*>(s));
          return costvec[4];
        }
      };

      Log::loglevel_t llvl = Log::get_primary_loglevel();
      Log::set_primary_loglevel(Log::SILENT);
      std::set<std::set<Sync*> > syncs =
        Fencins::fencins(*m,reach,arg_init,tf,Fencins::COST,0,costfn);
      Log::set_primary_loglevel(llvl);

      bool retval = check_syncs(m,syncs,fsets);

      for(auto it = syncs.begin(); it != syncs.end(); ++it){
        for(auto sp : *it){
          delete sp;
        }
      }

      delete m;
      return retval;
  };

  void test(){

    Test::inner_test("Small Dekker 10 5 5 1 1",
                     test_fencins(R"(
forbidden CS CS

data
  x = 0 : [0:1]
  y = 0 : [0:1]

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
)",{{"P0 L0 syncwr","P1 L0 syncwr","P0 L1 syncrd","P1 L1 syncrd"}},
   {10,5,5,1,1}));

    Test::inner_test("Small Dekker 1 1 1 1 1",
                     test_fencins(R"(
forbidden CS CS

data
  x = 0 : [0:1]
  y = 0 : [0:1]

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
)",{{"P0 L1 fence","P1 L1 fence"}},{1,1,1,1,1}));

    Test::inner_test("Small Dekker 2 1 1 1 1",
                     test_fencins(R"(
forbidden CS CS

data
  x = 0 : [0:1]
  y = 0 : [0:1]

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
)",{{"P0 L1 fence","P1 L1 fence"},
    {"P0 L0 syncwr","P0 L1 syncrd","P1 L1 fence"},
    {"P0 L0 syncwr","P0 L1 llfence","P1 L1 fence"},
    {"P0 L1 ssfence","P0 L1 syncrd","P1 L1 fence"},
    {"P0 L1 fence","P1 L0 syncwr","P1 L1 syncrd"},
    {"P0 L0 syncwr","P0 L1 syncrd","P1 L0 syncwr","P1 L1 syncrd"},
    {"P0 L0 syncwr","P0 L1 llfence","P1 L0 syncwr","P1 L1 syncrd"},
    {"P0 L1 ssfence","P0 L1 syncrd","P1 L0 syncwr","P1 L1 syncrd"},
    {"P0 L1 fence","P1 L0 syncwr","P1 L1 llfence"},
    {"P0 L0 syncwr","P0 L1 syncrd","P1 L0 syncwr","P1 L1 llfence"},
    {"P0 L0 syncwr","P0 L1 llfence","P1 L0 syncwr","P1 L1 llfence"},
    {"P0 L1 ssfence","P0 L1 syncrd","P1 L0 syncwr","P1 L1 llfence"},
    {"P0 L1 fence","P1 L1 ssfence","P1 L1 syncrd"},
    {"P0 L0 syncwr","P0 L1 syncrd","P1 L1 ssfence","P1 L1 syncrd"},
    {"P0 L0 syncwr","P0 L1 llfence","P1 L1 ssfence","P1 L1 syncrd"},
    {"P0 L1 ssfence","P0 L1 syncrd","P1 L1 ssfence","P1 L1 syncrd"}
   },{2,1,1,1,1}));

    Test::inner_test("Small Dekker 10 5 5 1 10",
                     test_fencins(R"(
forbidden CS CS

data
  x = 0 : [0:1]
  y = 0 : [0:1]

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
)",{{"P0 L0 syncwr","P0 L1 llfence","P1 L0 syncwr","P1 L1 llfence"}},{10,5,5,1,10}));


  }; // End test

};
