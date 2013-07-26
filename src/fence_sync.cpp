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
#include "tso_lock_sync.h"

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

  InsInfo *myinfo = new InsInfo((FenceSync*)clone(),f);
  *info = myinfo;
  myinfo->new_q = m.automata[pid].get_states().size();

  Machine *m2 = new Machine(m);

  const Automaton::State &state = m.automata[pid].get_states()[q];

  if(IN.size() != state.bwd_transitions.size() && OUT.size() != state.fwd_transitions.size()){
    delete m2;
    throw new std::logic_error("FenceSync: Both IN and OUT are partial. Not supported.");
  }

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

  /* Get the transition in m2 which is identical modulo statement
   * position to t. */
  std::function<Automaton::Transition*(const Machine &,const Automaton::Transition&)> find_transition = 
    [this](const Machine &machine,const Automaton::Transition &t)->Automaton::Transition*{
    const Automaton::State &state = machine.automata[this->pid].get_states()[t.source];
    for(auto it = state.fwd_transitions.begin(); it != state.fwd_transitions.end(); ++it){
      if((*it)->compare(t,false) == 0){
        return *it;
      }
    }
    return 0;
  };

  /* Calculate the transitions of IN and OUT as they appear in m */
  std::set<Automaton::Transition*> IN_m, OUT_m;
  for(auto it = IN.begin(); it != IN.end(); ++it){
    Automaton::Transition it_i = InsInfo::all_tchanges(m_infos,Machine::PTransition(*it,pid));
    Automaton::Transition *t = find_transition(m,it_i);
    assert(t);
    IN_m.insert(t);
  }
  for(auto it = OUT.begin(); it != OUT.end(); ++it){
    Automaton::Transition it_i = InsInfo::all_tchanges(m_infos,Machine::PTransition(*it,pid));
    Automaton::Transition *t = find_transition(m,it_i);
    assert(t);
    OUT_m.insert(t);
  }

  /* Check if some previous fence has been inserted at the same
   * control location */
  int qp; // The new control location
  {
    bool ex_prev_fence = false;
    for(unsigned i = 0; i < m_infos.size(); ++i){
      const InsInfo *ii = dynamic_cast<const InsInfo*>(m_infos[i]);
      if(ii){
        assert(dynamic_cast<const FenceSync*>(m_infos[i]->sync));
        const FenceSync *fs = static_cast<const FenceSync*>(m_infos[i]->sync);
        if(fs->get_pid() == pid && fs->get_q() == q){
          if(ex_prev_fence){
            /* If there are more than one previous fence at the same
             * location, they should all use the same new_q. */
            assert(qp == ii->new_q);
          }
          ex_prev_fence = true;
          qp = ii->new_q;
          /* IN := IN union fs->IN */
          for(auto it = fs->get_IN().begin(); it != fs->get_IN().end(); ++it){
            Automaton::Transition it_i = InsInfo::all_tchanges(m_infos,Machine::PTransition(*it,pid));
            Automaton::Transition *t = find_transition(m,it_i);
            assert(t);
            IN_m.insert(t);
          }
          /* OUT := OUT union fs->OUT */
          for(auto it = fs->get_OUT().begin(); it != fs->get_OUT().end(); ++it){
            Automaton::Transition it_i = InsInfo::all_tchanges(m_infos,Machine::PTransition(*it,pid));
            Automaton::Transition *t = find_transition(m,it_i);
            assert(t);
            OUT_m.insert(t);
          }
        }
      }
    }
    if(!ex_prev_fence){
      qp = m.automata[pid].get_states().size();
      // Add fence
      m2->automata[pid].add_transition(Automaton::Transition(qp,f,q));
    }
    myinfo->new_q = qp;
  }

  /* Find transitions where change should be considered */
  std::set<Automaton::Transition*> FWD_m = state.fwd_transitions;
  std::set<Automaton::Transition*> BWD_m = state.bwd_transitions;
  if(qp < m.automata[pid].get_states().size()){
    const Automaton::State &qp_state = m.automata[pid].get_states()[qp];
    FWD_m.insert(qp_state.fwd_transitions.begin(),qp_state.fwd_transitions.end());
    BWD_m.insert(qp_state.bwd_transitions.begin(),qp_state.bwd_transitions.end());
  }

  /* Change sources for outgoing transitions */
  for(auto it = FWD_m.begin(); it != FWD_m.end(); ++it){
    int src;
    if(OUT_m.count(*it)){
      /* Should go through fence. Set source == q */
      src = q;
    }else{
      /* Should not go through fence. Change source from q to qp. */
      src = qp;
    }
    Automaton::Transition t2(src,(*it)->instruction,(*it)->target);
    myinfo->bind(Machine::PTransition(**it,pid),Machine::PTransition(t2,pid));
    Automaton::Transition *t_m2 = find_transition(*m2,**it);
    assert(t_m2);
    m2->automata[pid].del_transition(*t_m2);
    m2->automata[pid].add_transition(t2);
  }
  /* Change targets for incoming transitions */
  for(auto it = BWD_m.begin(); it != BWD_m.end(); ++it){
    int tgt;
    if(IN_m.count(*it)){
      /* Should go through fence. Hence change target from q to qp */
      tgt = qp;
    }else{
      /* Should not go through fence. Set target == q */
      tgt = q;
    }
    Automaton::Transition t2((*it)->source,(*it)->instruction,tgt);
    myinfo->bind(Machine::PTransition(**it,pid),Machine::PTransition(t2,pid));
    Automaton::Transition *t_m2 = find_transition(*m2,**it);
    assert(t_m2);
    m2->automata[pid].del_transition(*t_m2);
    m2->automata[pid].add_transition(t2);
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
    if(dynamic_cast<const FenceSync::InsInfo*>(m_infos[i])){
      const FenceSync::InsInfo *info = dynamic_cast<const FenceSync::InsInfo*>(m_infos[i]);
    
      if(info->fence.compare(f,false) != 0){
        throw new std::logic_error("Attempt to insert a different fence instruction at the same control location. "
                                   "Not supported (yet).");
      }
    }else if(dynamic_cast<const TsoLockSync::InsInfo*>(m_infos[i])){
      // Ok
    }else{
      throw new std::logic_error("FenceSync: Unsupported kind of Sync has been inserted before.");
    }
  }

  /* Returns true iff t exists in m (after being subjected to the
   * changes described by m_infos).
   */
  std::function<bool(const Automaton::Transition &)> trans_exists = 
    [&m,this,&m_infos](const Automaton::Transition &t){
    Automaton::Transition s = InsInfo::all_tchanges(m_infos,Machine::PTransition(t,this->pid));
    const Automaton::State &state = m.automata[this->pid].get_states()[s.source];
    return std::any_of(state.fwd_transitions.begin(),state.fwd_transitions.end(),
                       [&s](const Automaton::Transition *t2){
                         return s.compare(*t2,false) == 0;
                       });
  };

  return std::all_of(IN.begin(),IN.end(),trans_exists) &&
    std::all_of(OUT.begin(),OUT.end(),trans_exists);
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
    if(dynamic_cast<const InsInfo*>(ivec[i])){
      const InsInfo *ii = static_cast<const InsInfo*>(ivec[i]);
      t2 = ii->tchanges.at(t2);
    }else{
      assert(dynamic_cast<const TsoLockSync::InsInfo*>(ivec[i]));
      const TsoLockSync::InsInfo *ii = static_cast<const TsoLockSync::InsInfo*>(ivec[i]);
      t2 = ii->tchanges.at(t2);
    }
  }
  return t2;
};

int FenceSync::InsInfo::original_q(const std::vector<const Sync::InsInfo*> &ivec, int q){
  for(int i = (int)ivec.size() - 1; i >= 0; --i){
    if(dynamic_cast<const InsInfo*>(ivec[i])){
      const InsInfo *info = static_cast<const InsInfo*>(ivec[i]);
      assert(dynamic_cast<const FenceSync*>(info->sync));
      const FenceSync *fs = static_cast<const FenceSync*>(info->sync);
      if(q == info->new_q){
        q = fs->get_q();
      }
    }else{
      assert(dynamic_cast<const TsoLockSync::InsInfo*>(ivec[i]));
    }
  }
  return q;
};

int FenceSync::compare(const Sync &s) const{
  assert(dynamic_cast<const FenceSync*>(&s));
  const FenceSync *fs = static_cast<const FenceSync*>(&s);

  int f_cmp = f.compare(fs->f,false);
  if(f_cmp != 0) return f_cmp;

  if(pid < fs->pid) return -1;
  if(pid > fs->pid) return 1;

  if(q < fs->q) return -1;
  if(q > fs->q) return 1;

  std::function<int(const std::set<Automaton::Transition> &,
                    const std::set<Automaton::Transition> &)> cmp_sets = 
    [](const std::set<Automaton::Transition> &S,
       const std::set<Automaton::Transition> &T){

    typedef std::function<bool(const Automaton::Transition&,
                               const Automaton::Transition&)> 
    cmp_fn_t;

    cmp_fn_t cmp_fn = [](const Automaton::Transition &a,
                         const Automaton::Transition &b){
      return a.compare(b,false) < 0;
    };

    std::set<Automaton::Transition,cmp_fn_t> SS(cmp_fn), TT(cmp_fn);
    SS.insert(S.begin(),S.end());
    TT.insert(T.begin(),T.end());

    if(SS < TT){
      return -1;
    }
    if(TT > SS){
      return 1;
    }
    return 0;
  };

  int c = cmp_sets(IN,fs->IN);
  if(c != 0) return c;

  return cmp_sets(OUT,fs->OUT);
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
            return pr.first.source != pr.second.source || pr.first.target != pr.second.target;
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

    /* Test 14,15,16: Multiple insertions to the same control state */
    {
      Machine *m = get_machine
        ("forbidden * * * *"
         "data\n"
         "  fnc = *\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    $r0 := 1\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 3\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 4;\n"
         "    $r0 := 5\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    $r0 := 7\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    $r0 := 1\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 3\n"
         "  };\n"
         "  either{\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 4;\n"
         "    $r0 := 5\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    $r0 := 7\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    $r0 := 1\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 3\n"
         "  };\n"
         "  either{\n"
         "    $r0 := 4;\n"
         "    $r0 := 5\n"
         "  or\n"
         "    locked write: fnc := 0;\n"
         "    $r0 := 6;\n"
         "    $r0 := 7\n"
         "  }\n"
         "process\n"
         "registers\n"
         "  $r0 = *\n"
         "text\n"
         "  either{\n"
         "    $r0 := 0;\n"
         "    $r0 := 1\n"
         "  or\n"
         "    $r0 := 2;\n"
         "    $r0 := 3\n"
         "  };\n"
         "  locked write: fnc := 0;\n"
         "  either{\n"
         "    $r0 := 4;\n"
         "    $r0 := 5\n"
         "  or\n"
         "    $r0 := 6;\n"
         "    $r0 := 7\n"
         "  }\n"
         );

      Dummy d0 = Dummy::parse_dummy(m,"0{$r0:=1;$r0:=3}to{$r0:=4}");
      Dummy d1 = Dummy::parse_dummy(m,"0{$r0:=1;$r0:=3}to{$r0:=6}");
      Dummy d2 = Dummy::parse_dummy(m,"0{$r0:=1;$r0:=3}to{$r0:=4;$r0:=6}");
      Dummy d3 = Dummy::parse_dummy(m,"0{$r0:=1}to{$r0:=4;$r0:=6}");
      Dummy d4 = Dummy::parse_dummy(m,"0{$r0:=3}to{$r0:=4;$r0:=6}");
      
      Sync::InsInfo *info;
      std::vector<const Sync::InsInfo*> m_infos_01, m_infos_10;
      Machine *m0 = d0.insert(*m,m_infos_01,&info); m_infos_01.push_back(info);
      Machine *m1 = d1.insert(*m,m_infos_10,&info); m_infos_10.push_back(info);
      Machine *m2 = d2.insert(*m,std::vector<const Sync::InsInfo*>(),&info); delete info;
      Machine *m01 = d1.insert(*m0,m_infos_01,&info); delete info;
      Machine *m10 = d0.insert(*m1,m_infos_10,&info); delete info;

      Test::inner_test("insert #14 (multiple on same control state)",
                       m->automata[1].same_automaton(m0->automata[0],false) &&
                       m->automata[2].same_automaton(m1->automata[0],false) &&
                       m->automata[3].same_automaton(m2->automata[0],false) &&
                       m->automata[3].same_automaton(m01->automata[0],false) &&
                       m->automata[3].same_automaton(m10->automata[0],false));

      std::vector<const Sync::InsInfo*> m_infos_34, m_infos_43;
      Machine *m3 = d3.insert(*m,m_infos_34,&info); m_infos_34.push_back(info);
      Machine *m4 = d4.insert(*m,m_infos_43,&info); m_infos_43.push_back(info);
      Machine *m34 = d4.insert(*m3,m_infos_34,&info); delete info;
      Machine *m43 = d3.insert(*m4,m_infos_43,&info); delete info;

      Test::inner_test("insert #15 (multiple on same control state)",
                       m->automata[3].same_automaton(m34->automata[0],false) &&
                       m->automata[3].same_automaton(m43->automata[0],false));

      std::vector<const Sync::InsInfo*> m_infos_03, m_infos_30;
      Machine *m03 = d3.insert(*m0,m_infos_01,&info); delete info;
      Machine *m30 = d0.insert(*m3,m_infos_34,&info); delete info;

      Test::inner_test("insert #16 (multiple on same control state)",
                       m->automata[3].same_automaton(m03->automata[0],false) &&
                       m->automata[3].same_automaton(m30->automata[0],false));

      for(unsigned i = 0; i < m_infos_01.size(); ++i){
        delete m_infos_01[i];
      }
      for(unsigned i = 0; i < m_infos_10.size(); ++i){
        delete m_infos_10[i];
      }
      for(unsigned i = 0; i < m_infos_34.size(); ++i){
        delete m_infos_34[i];
      }
      for(unsigned i = 0; i < m_infos_43.size(); ++i){
        delete m_infos_43[i];
      }
      for(unsigned i = 0; i < m_infos_03.size(); ++i){
        delete m_infos_03[i];
      }
      for(unsigned i = 0; i < m_infos_30.size(); ++i){
        delete m_infos_30[i];
      }
      delete m0;
      delete m1;
      delete m2;
      delete m01;
      delete m10;
      delete m3;
      delete m4;
      delete m34;
      delete m43;
      delete m03;
      delete m30;
      delete m;
    }

  }
};
