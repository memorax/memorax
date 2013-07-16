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

Machine *FenceSync::insert(const Machine &m, const std::vector<const Sync::InsInfo*> &m_infos, Sync::InsInfo **info) const{
  assert(applies_to(m,m_infos));

  InsInfo *myinfo = new InsInfo(clone());
  *info = myinfo;

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

  /* Setup info->tchanges as the identity map */
  {
    for(unsigned p = 0; p < m2->automata.size(); ++p){
      const std::vector<Automaton::State> &states = m2->automata[p].get_states();
      for(unsigned i = 0; i < states.size(); ++i){
        for(auto it = states[i].fwd_transitions.begin();
            it != states[i].fwd_transitions.end(); ++it){
          myinfo->bind(Machine::PTransition(**it,p),Machine::PTransition(**it,p));
        }
      }
    }
  }

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
      Automaton::Transition it_i = InsInfo::all_tchanges(m_infos,Machine::PTransition(*it,pid));
      const Automaton::Transition *t = find_transition(it_i);
      assert(t);
      Automaton::Transition t2(qp,t->instruction,t->target);
      m2->automata[pid].add_transition(t2);
      myinfo->bind(Machine::PTransition(*t,pid), Machine::PTransition(t2,pid));
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
      Automaton::Transition it_i = InsInfo::all_tchanges(m_infos,Machine::PTransition(*it,pid));
      const Automaton::Transition *t = find_transition(it_i);
      assert(t);
      Automaton::Transition t2(t->source,t->instruction,qp);
      m2->automata[pid].add_transition(t2);
      myinfo->bind(Machine::PTransition(*t,pid), Machine::PTransition(t2,pid));
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

bool FenceSync::applies_to(const Machine &m, const std::vector<const Sync::InsInfo*> &m_infos) const{
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

  for(unsigned i = 0; i < m_infos.size(); ++i){
    const FenceSync *fs = dynamic_cast<const FenceSync*>(m_infos[i]->sync);
    if(fs && fs->pid == pid && fs->q == q){
      throw new std::logic_error("FenceSync: Attempt to apply multiple fences at the same control location. Not supported (yet).");
      /* Perhaps it should be supported though... */
    }
  }

  /* Returns true iff each transitions in S equals some transition
   * pointed to by a member of T, after changes described by m_infos
   * have been applied to transitions in S.
   *
   * All transitions in S and T are assumed to belong to process pid.
   */
  std::function<bool(const std::set<Automaton::Transition> &,const std::set<Automaton::Transition*> &)> is_subset = 
    [this,&m_infos](const std::set<Automaton::Transition> &S,
       const std::set<Automaton::Transition*> &T){
    for(auto it = S.begin(); it != S.end(); ++it){
      Automaton::Transition s = InsInfo::all_tchanges(m_infos,Machine::PTransition(*it,this->pid));
      if(!std::any_of(T.begin(),T.end(),
                      [&s](const Automaton::Transition *t){
                        return s.compare(*t,false) == 0;
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

void FenceSync::InsInfo::bind(const Machine::PTransition &a,const Machine::PTransition &b){
  auto res = tchanges.insert(std::pair<Machine::PTransition,Machine::PTransition>(a,b));
  if(!res.second){
    tchanges.at(a) = b;
  }
};

const Machine::PTransition &FenceSync::InsInfo::operator[](const Machine::PTransition &t) const{
  return tchanges.at(t);
};

Machine::PTransition FenceSync::InsInfo::all_tchanges(const std::vector<const Sync::InsInfo*> &ivec,
                                                      const Machine::PTransition &t){
  Machine::PTransition t2 = t;
  for(unsigned i = 0; i < ivec.size(); ++i){
    assert(dynamic_cast<const InsInfo*>(ivec[i]));
    const InsInfo *ii = static_cast<const InsInfo*>(ivec[i]);
    t2 = ii->tchanges.at(t2);
  }
  return t2;
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
    virtual bool prevents(const Trace &t, const std::vector<const Sync::InsInfo*> &m_infos) const{return false;};
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

    /* Test 5,6: Multiple transitions */
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

      InsInfo *finfo = static_cast<InsInfo*>(info);
      std::function<bool(const std::pair<Machine::PTransition,Machine::PTransition>&)> iiall =
        [m,m2](const std::pair<Machine::PTransition,Machine::PTransition> &pr){
            Lang::Stmt<int> instr = pr.first.instruction;
            if(pr.first.instruction.compare(pr.second.instruction,false) != 0){
              return false;
            }
            if(pr.first.pid != pr.second.pid){
              return false;
            }
            int pid = pr.first.pid;
            int src0 = pr.first.source;
            int src1 = pr.second.source;
            Automaton::Transition t0 = pr.first;
            Automaton::Transition t1 = pr.second;
            const std::set<Automaton::Transition*> fwd0 = 
              m->automata[pid].get_states()[src0].fwd_transitions;
            const std::set<Automaton::Transition*> fwd1 = 
              m2->automata[pid].get_states()[src1].fwd_transitions;
            if(!std::any_of(fwd0.begin(),fwd0.end(),
                            [&t0](const Automaton::Transition *t){
                              return t->compare(t0,false) == 0;
                            })){
              return false;
            }
            if(!std::any_of(fwd1.begin(),fwd1.end(),
                            [&t1](const Automaton::Transition *t){
                              return t->compare(t1,false) == 0;
                            })){
              return false;
            }
            return true;
          };

      std::function<bool(const std::pair<Machine::PTransition,Machine::PTransition>&)> iiany =
        [m,m2](const std::pair<Machine::PTransition,Machine::PTransition> &pr){
            return pr.first.source != pr.second.source;
          };
        
      Test::inner_test("insert #6 (InsInfo)",
                       std::all_of(finfo->tchanges.begin(),finfo->tchanges.end(),iiall) &&
                       std::any_of(finfo->tchanges.begin(),finfo->tchanges.end(),iiany));

      delete info;
      delete m2;
      delete m;
    }

    /* Test 7: Multiple inserts */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  $r0 := 2;\n"
         "  $r0 := 3;\n"
         "  $r0 := 4\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 2;\n"
         "  $r0 := 3;\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 4"
         );

      Dummy d0 = Dummy::parse_dummy(m,"0{$r0:=1}to{$r0:=2}");
      Dummy d1 = Dummy::parse_dummy(m,"0{$r0:=3}to{$r0:=4}");

      InsInfo *d0info, *d1info;
      Machine *md0 = d0.insert(*m,std::vector<const Sync::InsInfo*>(),(Sync::InsInfo**)&d0info);
      Machine *md1 = d1.insert(*m,std::vector<const Sync::InsInfo*>(),(Sync::InsInfo**)&d1info);
      std::vector<const Sync::InsInfo*> d0ivec(1,d0info), d1ivec(1,d1info);

      Machine *md01 = d1.insert(*md0,d0ivec,(Sync::InsInfo**)&d1info);
      Machine *md10 = d0.insert(*md1,d1ivec,(Sync::InsInfo**)&d0info);

      Test::inner_test("insert #7 (multiple inserts)",
                       /* Both orders of insertion yields identical automata */
                       md01->automata[0].same_automaton(md10->automata[0],false) &&
                       /* It's the correct automaton */
                       m->automata[1].same_automaton(md01->automata[0],false));

      delete d0info;
      delete d1info;
      for(unsigned i = 0; i < d0ivec.size(); ++i){
        delete d0ivec[i];
      }
      for(unsigned i = 0; i < d1ivec.size(); ++i){
        delete d1ivec[i];
      }
      delete md01;
      delete md10;
      delete md0;
      delete md1;
      delete m;
    }

    /* Test 8: Multiple inserts */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  $r0 := 2\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 1;\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 2\n"
         );

      Dummy d0 = Dummy::parse_dummy(m,"0{$r0:=0}to{$r0:=1}");
      Dummy d1 = Dummy::parse_dummy(m,"0{$r0:=1}to{$r0:=2}");

      InsInfo *d0info, *d1info;
      Machine *md0 = d0.insert(*m,std::vector<const Sync::InsInfo*>(),(Sync::InsInfo**)&d0info);
      Machine *md1 = d1.insert(*m,std::vector<const Sync::InsInfo*>(),(Sync::InsInfo**)&d1info);
      std::vector<const Sync::InsInfo*> d0ivec(1,d0info), d1ivec(1,d1info);

      Machine *md01 = d1.insert(*md0,d0ivec,(Sync::InsInfo**)&d1info);
      Machine *md10 = d0.insert(*md1,d1ivec,(Sync::InsInfo**)&d0info);

      Test::inner_test("insert #8 (multiple inserts)",
                       /* Both orders of insertion yields identical automata */
                       md01->automata[0].same_automaton(md10->automata[0],false) &&
                       /* It's the correct automaton */
                       m->automata[1].same_automaton(md01->automata[0],false));

      delete d0info;
      delete d1info;
      for(unsigned i = 0; i < d0ivec.size(); ++i){
        delete d0ivec[i];
      }
      for(unsigned i = 0; i < d1ivec.size(); ++i){
        delete d1ivec[i];
      }
      delete md01;
      delete md10;
      delete md0;
      delete md1;
      delete m;
    }

    /* Test 9: Multiple inserts */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  L2: $r0 := 2;\n"
         "  goto L2\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  $r0 := 0;\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 1;\n"
         "  locked write: fnc := 0;\n"
         "  L2: $r0 := 2;\n"
         "  goto L2\n"
         );

      Dummy d0 = Dummy::parse_dummy(m,"0{$r0:=0}to{$r0:=1}");
      Dummy d1 = Dummy::parse_dummy(m,"0{$r0:=1}to{$r0:=2}");

      InsInfo *d0info, *d1info;
      Machine *md0 = d0.insert(*m,std::vector<const Sync::InsInfo*>(),(Sync::InsInfo**)&d0info);
      Machine *md1 = d1.insert(*m,std::vector<const Sync::InsInfo*>(),(Sync::InsInfo**)&d1info);
      std::vector<const Sync::InsInfo*> d0ivec(1,d0info), d1ivec(1,d1info);

      Machine *md01 = d1.insert(*md0,d0ivec,(Sync::InsInfo**)&d1info);
      Machine *md10 = d0.insert(*md1,d1ivec,(Sync::InsInfo**)&d0info);

      Test::inner_test("insert #9 (multiple inserts)",
                       /* Both orders of insertion yields identical automata */
                       md01->automata[0].same_automaton(md10->automata[0],false) &&
                       /* It's the correct automaton */
                       m->automata[1].same_automaton(md01->automata[0],false));

      delete d0info;
      delete d1info;
      for(unsigned i = 0; i < d0ivec.size(); ++i){
        delete d0ivec[i];
      }
      for(unsigned i = 0; i < d1ivec.size(); ++i){
        delete d1ivec[i];
      }
      delete md01;
      delete md10;
      delete md0;
      delete md1;
      delete m;
    }

    /* Test 10: Multiple inserts */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 1;\n"
         "    $r0 := 3\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 4\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 5;\n"
         "    $r0 := 7\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    either{\n"
         "      $r0 := 8\n"
         "    or\n"
         "      $r0 := 9\n"
         "    }\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 1;\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 3\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 4\n"
         "  };\n"
         "  locked write: fnc := 0;\n"
         "  either{\n"
         "    $r0 := 5;\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 7\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    either{\n"
         "      $r0 := 8\n"
         "    or\n"
         "      locked write: fnc := 0;\n"
         "      $r0 := 9\n"
         "    }\n"
         "  }\n"
         );

      /*      .
       *     / \
       *    1   2
       * -> .   . <-
       *    3   4
       *     \ /
       *      .   <-
       *     / \
       *    5   6
       * -> .    \
       *    |     .
       *    |    / \ <-
       *    7   8   9
       *    |   |  /
       *     \ /  /
       *      Y  /
       *      | /
       *      |/
       *      .
       */

      Dummy d13 = Dummy::parse_dummy(m,"0{$r0 := 1}to{$r0 := 3}");
      Dummy d24 = Dummy::parse_dummy(m,"0{$r0 := 2}to{$r0 := 4}");
      Dummy d3456 = Dummy::parse_dummy(m,"0{$r0 := 3; $r0 := 4}to{$r0 := 5; $r0 := 6}");
      Dummy d57 = Dummy::parse_dummy(m,"0{$r0 := 5}to{$r0 := 7}");
      Dummy d69 = Dummy::parse_dummy(m,"0{$r0 := 6}to{$r0 := 9}");

      /* ma# order: d13, d24, d57, d69, d3456 */
      /* mb# order: d3456, d13, d24, d57, d69 */
      std::vector<const Sync::InsInfo*> a_infos, b_infos;
      Sync::InsInfo *info;
      Machine *ma1 = d13.insert(*m,a_infos,&info); a_infos.push_back(info);
      Machine *ma2 = d24.insert(*ma1,a_infos,&info); a_infos.push_back(info);
      Machine *ma3 = d57.insert(*ma2,a_infos,&info); a_infos.push_back(info);
      Machine *ma4 = d69.insert(*ma3,a_infos,&info); a_infos.push_back(info);
      Machine *ma5 = d3456.insert(*ma4,a_infos,&info); a_infos.push_back(info);

      Machine *mb1 = d3456.insert(*m,b_infos,&info); b_infos.push_back(info);
      Machine *mb2 = d13.insert(*mb1,b_infos,&info); b_infos.push_back(info);
      Machine *mb3 = d24.insert(*mb2,b_infos,&info); b_infos.push_back(info);
      Machine *mb4 = d57.insert(*mb3,b_infos,&info); b_infos.push_back(info);
      Machine *mb5 = d69.insert(*mb4,b_infos,&info); b_infos.push_back(info);

      Test::inner_test("insert #10 (multiple inserts)",
                       m->automata[1].same_automaton(ma5->automata[0],false) && 
                       m->automata[1].same_automaton(mb5->automata[0],false));

      for(unsigned i = 0; i < a_infos.size(); ++i) delete a_infos[i];
      for(unsigned i = 0; i < b_infos.size(); ++i) delete b_infos[i];
      delete ma1;
      delete ma2;
      delete ma3;
      delete ma4;
      delete ma5;
      delete mb1;
      delete mb2;
      delete mb3;
      delete mb4;
      delete mb5;
      delete m;
    }

    /* Test 11: Multiple inserts */
    {
      Machine *m = get_machine
        ("forbidden * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 1;\n"
         "    $r0 := 3\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 4\n"
         "  or\n"
         "    $r0 := 10;\n"
         "    goto L6\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 5;\n"
         "    $r0 := 7\n"
         "  or\n"
         "    $r0 := 6;\n"
         "  L6:\n"
         "    either{\n"
         "      $r0 := 8\n"
         "    or\n"
         "      $r0 := 9\n"
         "    }\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 1;\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 3\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 4\n"
         "  or\n"
         "    $r0 := 10;\n"
         "    goto L6\n"
         "  };\n"
         "  locked write: fnc := 0;\n"
         "  either{\n"
         "    $r0 := 5;\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 7\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    locked write: fnc := 0;\n"
         "  L6:\n"
         "    either{\n"
         "      $r0 := 8\n"
         "    or\n"
         "      $r0 := 9\n"
         "    }\n"
         "  }\n"
         );

      /*      .----.
       *     / \    \
       *    1   2    \
       * -> .   . <-  .
       *    3   4     |
       *     \ /      10
       *      .   <-  |
       *     / \     /
       *    5   6   /
       * -> . -> \ /
       *    |     .
       *    |    / \
       *    7   8   9
       *    |   |  /
       *     \ /  /
       *      Y  /
       *      | /
       *      |/
       *      .
       */

      Dummy d13 = Dummy::parse_dummy(m,"0{$r0 := 1}to{$r0 := 3}");
      Dummy d24 = Dummy::parse_dummy(m,"0{$r0 := 2}to{$r0 := 4}");
      Dummy d3456 = Dummy::parse_dummy(m,"0{$r0 := 3; $r0 := 4}to{$r0 := 5; $r0 := 6}");
      Dummy d57 = Dummy::parse_dummy(m,"0{$r0 := 5}to{$r0 := 7}");
      Dummy d689 = Dummy::parse_dummy(m,"0{$r0 := 6}to{$r0 := 8; $r0 := 9}");
      // Dummy d6109 = Dummy::parse_dummy(m,"0{$r0 := 6; $r0 := 10}to{$r0 := 9}");

      /* ma# order: d13, d24, d57, d689, d6109, d3456 */
      /* mb# order: d3456, d13, d24, d57, d6109, d689 */
      std::vector<const Sync::InsInfo*> a_infos, b_infos;
      Sync::InsInfo *info;
      Machine *ma1 = d13.insert(*m,a_infos,&info); a_infos.push_back(info);
      Machine *ma2 = d24.insert(*ma1,a_infos,&info); a_infos.push_back(info);
      Machine *ma3 = d57.insert(*ma2,a_infos,&info); a_infos.push_back(info);
      Machine *ma4 = d689.insert(*ma3,a_infos,&info); a_infos.push_back(info);
      Machine *ma5 = d3456.insert(*ma4,a_infos,&info); a_infos.push_back(info);

      Machine *mb1 = d3456.insert(*m,b_infos,&info); b_infos.push_back(info);
      Machine *mb2 = d13.insert(*mb1,b_infos,&info); b_infos.push_back(info);
      Machine *mb3 = d24.insert(*mb2,b_infos,&info); b_infos.push_back(info);
      Machine *mb4 = d57.insert(*mb3,b_infos,&info); b_infos.push_back(info);
      Machine *mb5 = d689.insert(*mb4,b_infos,&info); b_infos.push_back(info);

      Test::inner_test("insert #11 (multiple inserts)",
                       m->automata[1].same_automaton(ma5->automata[0],false) && 
                       m->automata[1].same_automaton(mb5->automata[0],false));

      for(unsigned i = 0; i < a_infos.size(); ++i) delete a_infos[i];
      for(unsigned i = 0; i < b_infos.size(); ++i) delete b_infos[i];
      delete ma1;
      delete ma2;
      delete ma3;
      delete ma4;
      delete ma5;
      delete mb1;
      delete mb2;
      delete mb3;
      delete mb4;
      delete mb5;
      delete m;
    }

    /* Test 12: Insertion at initial state */
    {
      Machine *m = get_machine
        ("forbidden * * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  goto L0\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  goto L0\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  $r0 := 0;\n"
         "  $r0 := 1;\n"
         "  locked write: fnc := 0;\n"
         "  goto L0\n"
         );

      Dummy d = Dummy::parse_dummy(m,"0{$r0:=1}to{$r0:=0}");

      Sync::InsInfo *info;
      Machine *m2 = d.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #12",
                       m->automata[1].same_automaton(m2->automata[0],false) ||
                       m->automata[2].same_automaton(m2->automata[0],false));

      delete info;
      delete m2;
      delete m;
    }

    /* Test 13: Insertion at initial state (one-transition self loop) */
    {
      Machine *m = get_machine
        ("forbidden * * *\n"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  $r0 := 0;\n"
         "  goto L0\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  locked write: fnc := 0;\n"
         "  $r0 := 0;\n"
         "  goto L0\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  L0:\n"
         "  $r0 := 0;\n"
         "  locked write: fnc := 0;\n"
         "  goto L0\n"
         );

      Dummy d = Dummy::parse_dummy(m,"0{$r0:=0}to{$r0:=0}");

      Sync::InsInfo *info;
      Machine *m2 = d.insert(*m,std::vector<const Sync::InsInfo*>(),&info);

      Test::inner_test("insert #13",
                       m->automata[1].same_automaton(m2->automata[0],false) ||
                       m->automata[2].same_automaton(m2->automata[0],false));

      delete info;
      delete m2;
      delete m;
    }

    /* Test ?: Test multiple insertions to the same control state */

  }
};
