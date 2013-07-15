/*
 * Copyright (C) 2013 Carl Leonardsson
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

#include "fence_sync.h"
#include "log.h"
#include "parser.h"
#include "preprocessor.h"
#include "test.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>

FenceSync::FenceSync(Lang::Stmt<int> f, int pid, int q, 
                     std::set<Automaton::Transition> IN, 
                     std::set<Automaton::Transition> OUT)
  : f(f), pid(pid), q(q), IN(IN), OUT(OUT) {
};

std::string FenceSync::to_raw_string() const{
  return to_string_aux(Lang::int_reg_to_string(),
                       Lang::int_memloc_to_string());
};

std::string FenceSync::to_string(const Machine &m) const{
  return to_string_aux(m.reg_pretty_vts(pid),m.ml_pretty_vts(pid));
};

Machine *FenceSync::insert(const Machine &m, std::vector<const Sync::InsInfo*> m_infos, Sync::InsInfo **info) const{
  assert(applies_to(m));

  *info = new InsInfo(clone());

  Log::warning << "FenceSync::insert: Unclean handling of forbidden\n";
  /* Consider forbidden specifications using '*'.
   * The new control states should also be forbidden.
   */
  Machine *m2 = new Machine(m);

  const Automaton::State &state = m.automata[pid].get_states()[q];

  /* Get the transition in m2 which is identical modulo statement
   * position to t. */
  std::function<const Automaton::Transition*(const Automaton::Transition&)> find_transition = 
    [m2,this](const Automaton::Transition &t)->const Automaton::Transition*{
    const Automaton::State &state = m2->automata[this->pid].get_states()[t.source];
    for(auto it = state.fwd_transitions.begin(); it != state.fwd_transitions.end(); ++it){
      if((*it)->compare(t,false) == 0){
        return *it;
      }
    }
    return 0;
  };

  if(IN.size() == state.bwd_transitions.size()){
    /* All incoming transitions should go through the fence. Relink as
     * follows:
     * 1) Introduce a new state q'
     * 2) Change each OUT transition (q,i,q2) into (q',i,q2)
     * 3) Add transition (q,fence,q')
     */
    int qp = m2->automata[pid].get_states().size(); // q'
    /* Change transitions */
    for(auto it = OUT.begin(); it != OUT.end(); ++it){
      const Automaton::Transition *t = find_transition(*it);
      assert(t);
      m2->automata[pid].add_transition(Automaton::Transition(qp,t->instruction,t->target));
      m2->automata[pid].del_transition(*t);
    }
    /* Insert fence */
    m2->automata[pid].add_transition(Automaton::Transition(q,f,qp));
  }else if(OUT.size() == state.fwd_transitions.size()){
    /* All outgoing transitions should go through the fence. Relink as
     * follows:
     * 1) Introduce a new state q'
     * 2) Change each IN transition (q2,i,q) into (q2,i,q')
     * 3) Add transition (q',fence,q)
     */
    int qp = m2->automata[pid].get_states().size(); // q'
    /* Change transitions */
    for(auto it = IN.begin(); it != IN.end(); ++it){
      const Automaton::Transition *t = find_transition(*it);
      assert(t);
      m2->automata[pid].add_transition(Automaton::Transition(t->source,t->instruction,qp));
      m2->automata[pid].del_transition(*t);
    }
    /* Insert fence */
    m2->automata[pid].add_transition(Automaton::Transition(qp,f,q));
  }else{
    /* Some incoming and some outgoing transitions should go through
     * the fence. Relink as follows:
     * 1) Introduce new states q' and q''
     * 2) Add transition (q',fence,q'')
     * 3) Change each IN transition (q2,i,q) into (q2,i,q')
     * 4) Change each OUT transition (q,i,q2) into (q'',i,q2)
     */

    /* What happens with labels? */
    /* What about forbidden control states? */
    /* What about other Syncs that depend on control states? */
    delete m2;
    throw new std::logic_error("FenceSync: Both IN and OUT are partial. Not supported.");
  }

  return m2;
};

bool FenceSync::applies_to(const Machine &m) const{
  if(pid < 0){
    std::stringstream ss;
    ss << "FenceSync::insert: Invalid process: " << pid;
    throw new std::logic_error(ss.str());
  }
  if((int)m.automata.size() <= pid){
    return false;
  }
  if(q < 0){
    std::stringstream ss;
    ss << "FenceSync::insert: Invalid control state: " << q;
    throw new std::logic_error(ss.str());
  }
  if((int)m.automata[pid].get_states().size() <= q){
    return false;
  }

  /* Returns true iff each transitions in S equals some transition
   * pointed to by a member of T. */
  std::function<bool(const std::set<Automaton::Transition> &,const std::set<Automaton::Transition*> &)> is_subset = 
    [](const std::set<Automaton::Transition> &S,
       const std::set<Automaton::Transition*> &T){
    for(auto it = S.begin(); it != S.end(); ++it){
      if(!std::any_of(T.begin(),T.end(),
                      [&it](const Automaton::Transition *t){
                        return it->compare(*t,false) == 0;
                      })){
        return false;
      }
    }
    return true;
  };

  if(!is_subset(IN,m.automata[pid].get_states()[q].bwd_transitions) ||
     !is_subset(OUT,m.automata[pid].get_states()[q].fwd_transitions)){
    return false;
  }

  return true;
};

std::string FenceSync::to_string_aux(const std::function<std::string(const int&)> &regts, 
                                     const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const{
  std::stringstream ss;
  ss << "FenceSync(P" << pid << ",Q" << q << ",f:" << f.to_string(regts,mlts) << ")\n";
  for(auto it = IN.begin(); it != IN.end(); ++it){
    ss << "IN: " << it->to_string(regts,mlts) << "\n";
  }
  for(auto it = OUT.begin(); it != OUT.end(); ++it){
    ss << "OUT: " << it->to_string(regts,mlts) << "\n";
  }
  return ss.str();
};

std::set<Sync*> FenceSync::get_all_possible(const Machine &m,
                                            const std::set<Lang::Stmt<int> > &fs,
                                            const fs_init_t &fsinit){
  std::set<Sync*> ss;
  for(unsigned p = 0; p < m.automata.size(); ++p){
    const std::vector<Automaton::State> &states = m.automata[p].get_states();
    for(unsigned i = 0; i < states.size(); ++i){
      std::set<Automaton::Transition> IN, OUT;
      for(auto it = states[i].bwd_transitions.begin(); it != states[i].bwd_transitions.end(); ++it){
        IN.insert(**it);
      }
      for(auto it = states[i].fwd_transitions.begin(); it != states[i].fwd_transitions.end(); ++it){
        OUT.insert(**it);
      }
      std::set<std::set<Automaton::Transition> > INS = powerset(IN);
      std::set<std::set<Automaton::Transition> > OUTS = powerset(OUT);
      for(auto init = INS.begin(); init != INS.end(); ++init){
        if(init->empty()) continue; // Skip empty set
        for(auto outit = OUTS.begin(); outit != OUTS.end(); ++outit){
          if(outit->empty()) continue; // Skip empty set
          for(auto fit = fs.begin(); fit != fs.end(); ++fit){
            ss.insert(fsinit(*fit,p,i,*init,*outit));
          }
        }
      }
    }
  }
  return ss;
};

template<class T>
std::set<std::set<T> > FenceSync::powerset(const std::set<T> &s){
  std::vector<std::set<T> > v;
  v.push_back(std::set<T>());
  int cur = 1;

  for(auto it = s.begin(); it != s.end(); ++it){
    for(int i = 0; i < cur; ++i){
      std::set<T> subs = v[i];
      subs.insert(*it);
      v.push_back(subs);
    }
    cur *= 2;
  }

  return std::set<std::set<T> >(v.begin(),v.end());
};

void FenceSync::test(){
  /* Test powerset */
  {
    /* Test 1: empty set */
    {
      std::set<std::set<int> > ps = powerset(std::set<int>());
      Test::inner_test("Powerset #1",
                       ps.size() == 1 &&
                       ps.begin()->size() == 0);
    }

    /* Test 2: singleton set */
    {
      std::set<int> s;
      s.insert(1);
      std::set<std::set<int> > tgt;
      {
        std::set<int> ss;
        tgt.insert(ss);
        ss.insert(1);
        tgt.insert(ss);
      }
      std::set<std::set<int> > ps = powerset(s);
      Test::inner_test("Powerset #2",ps == tgt);
    }

    /* Test 3: larger set */
    {
      std::set<int> s;
      s.insert(1);
      s.insert(2);
      s.insert(3);
      std::set<std::set<int> > tgt;
      {
        std::set<int> ss1, ss2, ss3;
        tgt.insert(ss1); // {}
        ss1.insert(1);
        tgt.insert(ss1); // {1}
        ss2.insert(2);
        tgt.insert(ss2); // {2}
        ss3.insert(3);
        tgt.insert(ss3); // {3}
        ss1.insert(2);
        tgt.insert(ss1); // {1,2}
        ss2.insert(3);
        tgt.insert(ss2); // {2,3}
        ss3.insert(1);
        tgt.insert(ss3); // {1,3}
        ss1.insert(3);
        tgt.insert(ss1); // {1,2,3}
      }
      std::set<std::set<int> > ps = powerset(s);
      Test::inner_test("Powerset #3",ps == tgt);
    }
  }

  std::function<Machine*(std::string)> get_machine = 
    [](std::string rmm){
    std::stringstream ss(rmm);
    PPLexer pp(ss);
    return new Machine(Parser::p_test(pp));
  };

  class StmtCmp{
  public:
    bool operator()(const Lang::Stmt<int> &a, const Lang::Stmt<int> &b) const{
      return a.compare(b,false) < 0;
    }
  };
  /* Dummy concrete implementation of FenceSync. */
  class Dummy : public FenceSync{
  public:
    Dummy(Lang::Stmt<int> f, int pid, int q, 
          std::set<Automaton::Transition> IN, 
          std::set<Automaton::Transition> OUT)
      : FenceSync(f,pid,q,IN,OUT) {};
    ~Dummy() {};
    virtual bool prevents(const Trace &t) const{return false;};
    virtual FenceSync *clone() const{
      return new Dummy(f,pid,q,IN,OUT);
    };
    bool operator==(const Dummy &d) const{
      return f == d.f && pid == d.pid && q == d.q &&
        IN == d.IN && OUT == d.OUT;
    };
    bool operator<(const Dummy &d) const{
      return 
        (f < d.f) ||
        (f == d.f && pid < d.pid) ||
        (f == d.f && pid == d.pid && q < d.q) ||
        (f == d.f && pid == d.pid && q == d.q && IN < d.IN) ||
        (f == d.f && pid == d.pid && q == d.q && IN == d.IN && OUT < d.OUT);
    };
    static Dummy parse_dummy(const Machine *m, std::string s){
      int pid;
      {
        std::stringstream ss(s);
        ss >> pid;
        s = s.substr(s.find('{')); // Skip until after pid
      }
      std::set<Lang::Stmt<int>,StmtCmp> IN_stmts, OUT_stmts;
      {
        std::string pre = 
          "forbidden *\n"
          "data\n"
          "  fnc = *\n"
          "  x = *\n"
          "  y = *\n"
          "process\n"
          "registers\n"
          "  $r0 = *\n"
          "  $r1 = *\n"
          "  $r2 = *\n"
          "text\n";
        std::string IN_s, OUT_s;
        int i = s.find("to");
        IN_s = s.substr(0,i);
        OUT_s = s.substr(i+2);
        {
          std::stringstream ss(pre+IN_s);
          PPLexer lex(ss);
          Machine m(Parser::p_test(lex));
          for(unsigned j = 0; j < m.automata[0].get_states().size(); ++j){
            const std::set<Automaton::Transition*> &s = 
              m.automata[0].get_states()[j].fwd_transitions;
            for(auto it = s.begin(); it != s.end(); ++it){
              IN_stmts.insert((*it)->instruction);
            }
          }
        }
        {
          std::stringstream ss(pre+OUT_s);
          PPLexer lex(ss);
          Machine m(Parser::p_test(lex));
          for(unsigned j = 0; j < m.automata[0].get_states().size(); ++j){
            const std::set<Automaton::Transition*> &s = 
              m.automata[0].get_states()[j].fwd_transitions;
            for(auto it = s.begin(); it != s.end(); ++it){
              OUT_stmts.insert((*it)->instruction);
            }
          }
        }
      }
      int q = -1;
      {
        /* Find the right control location */
        const std::vector<Automaton::State> &states = m->automata[pid].get_states();
        for(unsigned i = 0; i < states.size(); ++i){
          if(std::any_of(states[i].fwd_transitions.begin(),states[i].fwd_transitions.end(),
                         [&OUT_stmts](const Automaton::Transition *pt){
                           return OUT_stmts.count(pt->instruction) > 0;
                         })){
            q = i;
            break;
          }
        }
      }

      assert(q >= 0);

      std::set<Automaton::Transition> IN, OUT;
      {
        for(auto it = m->automata[pid].get_states()[q].bwd_transitions.begin();
            it != m->automata[pid].get_states()[q].bwd_transitions.end(); ++it){
          if(IN_stmts.count((*it)->instruction)){
            IN.insert(**it);
          }
        }
        for(auto it = m->automata[pid].get_states()[q].fwd_transitions.begin();
            it != m->automata[pid].get_states()[q].fwd_transitions.end(); ++it){
          if(OUT_stmts.count((*it)->instruction)){
            OUT.insert(**it);
          }
        }
      }
      Lang::Stmt<int> fence = Lang::Stmt<int>::locked_write(Lang::MemLoc<int>::global(0),Lang::Expr<int>::integer(0));
      return Dummy(fence,pid,q,IN,OUT);
    };
  };

  /* Test get_all_possible */
  {

    std::set<Lang::Stmt<int> > fs0;
    /* Under TSO, a locked write to the dummy memory location
     * (global(0)) acts as a total fence. */
    fs0.insert(Lang::Stmt<int>::locked_write(Lang::MemLoc<int>::global(0),
                                             Lang::Expr<int>::integer(0)));

    fs_init_t fsinit = 
      [](Lang::Stmt<int> f, int pid, int q, 
         std::set<Automaton::Transition> IN, 
         std::set<Automaton::Transition> OUT){ return new Dummy(f,pid,q,IN,OUT); };

    /* tgt_s specifies one Dummy per line. The format is
     * P{a;b;c;...}to{A;B;C;...}
     * where P is a natural number; the pid of the Dummy
     * a, b, c, ... are the statements of IN
     * A, B, C, ... are the statements of OUT
     *
     * All statements in m should be unique and may reference global
     * memory locations fnc, x, y, and registers $r0, $r1, $r2.
     */
    std::function<bool(const Machine*,const std::set<Sync*>&,std::string)> tst = 
      [](const Machine *m,const std::set<Sync*> &syncset,std::string tgt_s){
      /* Parse tgt_s, create target Dummys */
      std::set<Dummy> dummys;
      {
        std::size_t lnbegin = 0;
        std::size_t lnend;
        while(lnbegin < tgt_s.size()){
          lnend = tgt_s.find('\n',lnbegin);

          if(lnend == std::string::npos){
            dummys.insert(Dummy::parse_dummy(m,tgt_s.substr(lnbegin)));
          }else{
            dummys.insert(Dummy::parse_dummy(m,tgt_s.substr(lnbegin,lnend - lnbegin)));
          }

          lnbegin = (lnend == std::string::npos) ? tgt_s.size() : lnend+1;
        }
      }

      /* Perform the actual test */
      return syncset.size() == dummys.size() &&
      std::all_of(syncset.begin(),syncset.end(),
                  [&dummys](const Sync *sync){
                    const Dummy *d = static_cast<const Dummy*>(sync);
                    return 
                    std::any_of(dummys.begin(),dummys.end(),
                                [d](const Dummy &d2){
                                  return
                                    d->pid == d2.pid &&
                                    d->q == d2.q &&
                                    d->IN == d2.IN &&
                                    d->OUT == d2.OUT;
                                });
                  });
    };

    /* Test 1: Minimal */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "text\n"
         "  nop"
         );
      std::set<Sync*> s = get_all_possible(*m,fs0,fsinit);
      Test::inner_test("get_all #1",s.empty());

      delete m;
    }

    /* Test 2: Small */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = * : [0:1]"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1"
         );

      std::set<Sync*> s = get_all_possible(*m,fs0,fsinit);
      Test::inner_test("get_all #2",tst(m,s,"0{$r0 := 0}to{$r0 := 1}"));

      for(auto it = s.begin(); it != s.end(); ++it){
        delete *it;
      }

      delete m;
    }

    /* Test 3: Small 2 proc */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = * : [0:1]"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1\n"
         "process\n"
         "registers\n"
         "  $r0 = * : [0:1]"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1"
         );

      std::set<Sync*> s = get_all_possible(*m,fs0,fsinit);
      Test::inner_test("get_all #3",tst(m,s,
                                        "0{$r0 := 0}to{$r0 := 1}\n"
                                        "1{$r0 := 0}to{$r0 := 1}"));

      for(auto it = s.begin(); it != s.end(); ++it){
        delete *it;
      }

      delete m;
    }

    /* Test 4: Multiple in/out transitions */
    {
      Machine *m = get_machine
        ("forbidden *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = *"
         "text\n"
         "  L0:\n"
         "  $r0 := 0;\n"
         "  either{\n"
         "    $r0 := 1\n"
         "  or\n"
         "    $r0 := 2\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 3\n"
         "  or\n"
         "    $r0 := 4\n"
         "  };\n"
         "  $r0 := 5;"
         "  goto L0"
         );

      /*     .<-.
       *     0   \
       *     .    .
       *    1 2   |
       *     .    |
       *    3 4   |
       *     .    ,
       *     5   /
       *     .--'
       */

      std::set<Sync*> s = get_all_possible(*m,fs0,fsinit);
      Test::inner_test("get_all #4",tst(m,s,
                                        "0{$r0:=0}to{$r0:=1}\n"
                                        "0{$r0:=0}to{$r0:=2}\n"
                                        "0{$r0:=0}to{$r0:=1;$r0:=2}\n"

                                        "0{$r0:=1}to{$r0:=3}\n"
                                        "0{$r0:=1}to{$r0:=4}\n"
                                        "0{$r0:=1}to{$r0:=3;$r0:=4}\n"
                                        "0{$r0:=2}to{$r0:=4}\n"
                                        "0{$r0:=2}to{$r0:=3}\n"
                                        "0{$r0:=2}to{$r0:=3;$r0:=4}\n"
                                        "0{$r0:=1;$r0:=2}to{$r0:=3}\n"
                                        "0{$r0:=1;$r0:=2}to{$r0:=4}\n"
                                        "0{$r0:=1;$r0:=2}to{$r0:=3;$r0:=4}\n"

                                        "0{$r0:=3}to{$r0:=5}\n"
                                        "0{$r0:=4}to{$r0:=5}\n"
                                        "0{$r0:=3;$r0:=4}to{$r0:=5}\n"

                                        "0{$r0:=5}to{$r0:=0}"
                                        ));

      for(auto it = s.begin(); it != s.end(); ++it){
        delete *it;
      }

      delete m;
    }
  }

  /* Test insert */
  {
    /* Test 1: Simple */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 1"
         );

      Dummy d = Dummy::parse_dummy(m,"0{$r0 := 0}to{$r0 := 1}");
      Sync::InsInfo *info;
      Machine *m2 = d.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #1",m2->automata[1].same_automaton(m2->automata[0],false));

      delete info;
      delete m2;
      delete m;
    }

    /* Test 2: Simple */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  $r0 := 2"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 1;\n"
         "  $r0 := 2"
         );

      Dummy d = Dummy::parse_dummy(m,"0{$r0 := 0}to{$r0 := 1}");
      Sync::InsInfo *info;
      Machine *m2 = d.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #2",m2->automata[1].same_automaton(m2->automata[0],false));

      delete info;
      delete m2;
      delete m;
    }

    /* Test 3: Multiple transitions */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0\n"
         "  or\n"
         "    $r0 := 1\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 2\n"
         "  or\n"
         "    $r0 := 3\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0\n"
         "  or\n"
         "    $r0 := 1\n"
         "  };\n"
         "  locked write: fnc := 0;\n"
         "  either{\n"
         "    $r0 := 2\n"
         "  or\n"
         "    $r0 := 3\n"
         "  }\n"
         );

      Dummy d = Dummy::parse_dummy(m,"0{$r0 := 0; $r0 := 1}to{$r0 := 2; $r0 := 3}");
      Sync::InsInfo *info;
      Machine *m2 = d.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #3",m2->automata[1].same_automaton(m2->automata[0],false));

      delete info;
      delete m2;
      delete m;
    }

    /* Test 4: Multiple transitions */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0\n"
         "  or\n"
         "    $r0 := 1\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 2\n"
         "  or\n"
         "    $r0 := 3\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    locked write: fnc := 0\n"
         "  or\n"
         "    $r0 := 1\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 2\n"
         "  or\n"
         "    $r0 := 3\n"
         "  }\n"
         );

      Dummy d = Dummy::parse_dummy(m,"0{$r0 := 0}to{$r0 := 2; $r0 := 3}");
      Sync::InsInfo *info;
      Machine *m2 = d.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #4",m2->automata[1].same_automaton(m2->automata[0],false));

      delete info;
      delete m2;
      delete m;
    }

    /* Test 5: Multiple transitions */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = 0 : [0:0]\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0\n"
         "  or\n"
         "    $r0 := 1\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 2\n"
         "  or\n"
         "    $r0 := 3\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0\n"
         "  or\n"
         "    $r0 := 1\n"
         "  };\n"
         "  either{\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 2\n"
         "  or\n"
         "    $r0 := 3\n"
         "  }\n"
         );

      Dummy d = Dummy::parse_dummy(m,"0{$r0 := 0; $r0 := 1}to{$r0 := 2}");
      Sync::InsInfo *info;
      Machine *m2 = d.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #5",m2->automata[1].same_automaton(m2->automata[0],false));

      delete info;
      delete m2;
      delete m;
    }
  }
};
