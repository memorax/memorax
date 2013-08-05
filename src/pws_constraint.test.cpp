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

#include <iostream>
#include <iterator>
#include "pws_constraint.h"
#include "intersection_iterator.h"
#include "preprocessor.h"

using namespace std;
using namespace Lang;

static unique_ptr<Machine> get_machine(istream &is) {
  PPLexer lex(is);
  Machine m0(Parser::p_test(lex));
  unique_ptr<Machine> m1(m0.remove_registers());
  unique_ptr<Machine> m2(m1->remove_superfluous_nops());
  return m2;
}

static Machine::PTransition get_transition(const Machine &m, int pid, int from, int to) {
  for (Automaton::Transition *t : m.automata[pid].get_states()[from].fwd_transitions) {
    if (t->target == to)
      return Machine::PTransition(*t, pid);
  }
  throw new std::logic_error("PwsConstraint.test.cpp::get_transition: Couldn't find transition");
}

static void test_pre_sequence(string name, const Machine& m, vector<Machine::PTransition> trans) {
  PwsConstraint::Common common(m);
  function<string(Machine::PTransition)> pttostr = [m](Machine::PTransition pt) 
    { return pt.to_string(m.reg_pretty_vts(pt.pid), m.ml_pretty_vts(pt.pid)); };
  list<unique_ptr<PwsConstraint>> constraints;
  {
    list<Constraint*> bs = common.get_bad_states();
    transform(bs.begin(), bs.end(), back_inserter(constraints),
              [](Constraint *c) { return unique_ptr<PwsConstraint>(dynamic_cast<PwsConstraint*>(c)); });
  }

  for (Machine::PTransition pt : trans) {
    bool in_any_partred = false;
    list<unique_ptr<PwsConstraint>> news;
    for (const unique_ptr<PwsConstraint> &c : constraints) {
      {
        auto trans = c->partred();
        if (!any_of(trans.begin(), trans.end(), [pt](const Machine::PTransition *t) { return *t == pt; }))
          continue;
      }
      in_any_partred = true;
      list<Constraint*> pres = c->pre(pt);
      transform(pres.begin(), pres.end(), back_inserter(news),
                [](Constraint *c) { return unique_ptr<PwsConstraint>(dynamic_cast<PwsConstraint*>(c)); });
    }
    if (!in_any_partred) {
      Log::debug << "  " << name << ": Transition \"" << pttostr(pt) << "\" was not suggested by any PwsConstraint::partred " 
                 << (news.empty() ? "and produced no constraints\n" : "but produced new constraints\n");
      Test::inner_test(name, false);
      return;
    }
    if (news.empty()) {
      Log::debug << "  " << name << ": Transition \"" << pttostr(pt) << "\" produced zero new constraints\n";
      Test::inner_test(name, false);
      return;
    }
    Log::extreme << "  " << name << ": Transition \"" << pttostr(pt) << "\" produced the following constraints\n";
    for (const unique_ptr<PwsConstraint> &c : news) Log::extreme << c->to_string() << "\n";
    swap(constraints, news);
  }
  Test::inner_test(name, any_of(constraints.begin(), constraints.end(), 
                                [](const unique_ptr<PwsConstraint> &c) { return c->is_init_state(); }));
}

void PwsConstraint::test_pre() {
  {
    stringstream ss;
    ss << R"(
forbidden
  CS CS
data
  a = 0 : [0:1]
  b = 0 : [0:1]
  
process
text
    write: a := 1;
    write: b := 1;
CS: nop

process
text
    read:  b  = 1;
    read:  a  = 0;
CS: nop
)";
    Machine m = *get_machine(ss);
    vector<Machine::PTransition> test{
      {2, Stmt<int>::update(0, VecSet<MemLoc<int>>::singleton(MemLoc<int>::global(0))), 2, 0}, // P0: update(a, P0)
      {2, Stmt<int>::update(0, VecSet<MemLoc<int>>::singleton(MemLoc<int>::global(0))), 2, 1}, //   P1: update(a, P0)
      {2, Stmt<int>::serialise(VecSet<MemLoc<int>>::singleton(MemLoc<int>::global(0))), 2, 0}, // P0: serialise: a
      {**m.automata[1].get_states()[1].fwd_transitions.begin(),                            1}, //   P1: read: a = 0
      {**m.automata[1].get_states()[0].fwd_transitions.begin(),                            1}, //   P1: read: b = 1
      {2, Stmt<int>::update(0, VecSet<MemLoc<int>>::singleton(MemLoc<int>::global(1))), 2, 0}, // P0: update(b, P0)
      {0, Stmt<int>::update(0, VecSet<MemLoc<int>>::singleton(MemLoc<int>::global(1))), 0, 1}, //   P1: update(b, P0)
      {2, Stmt<int>::serialise(VecSet<MemLoc<int>>::singleton(MemLoc<int>::global(1))), 2, 0}, // P0: serialise: b
      {**m.automata[0].get_states()[1].fwd_transitions.begin(),                            0}, // P0: write: b := 1
      {**m.automata[0].get_states()[0].fwd_transitions.begin(),                            0}  // P0: write: a := 1
    };
    test_pre_sequence("simple reorder", m, test);
  }
  {
    stringstream ss;
    ss << R"(
forbidden
  CS CS
data
  turn = * : [0:1]

process(1)
data
  flag = 0 : [0:1]
registers
  $r0 = * : [0:1]
  $r1 = * : [0:1]
text
  L0: slocked write: flag[my] := 1;
  read: $r0 := flag[0];
  while $r0 = 1 do {
    read: $r1 := turn;
    if $r1 != 0 then{
      slocked write: flag[my] := 0;
      while $r1 != 0 do
        read: $r1 := turn;
      goto L0
    };
    read: $r0 := flag[0]
  };
  CS: slocked write: turn := 1;
  slocked write: flag[my] := 0;
  goto L0

process(1)
data
  flag = 0 : [0:1]
registers
  $r0 = * : [0:1]
  $r1 = * : [0:1]
text
  L0: slocked write: flag[my] := 1;
  read: $r0 := flag[0];
  while $r0 = 1 do {
    read: $r1 := turn;
    if $r1 != 1 then{
      slocked write: flag[my] := 0;
      while $r1 != 1 do
        read: $r1 := turn;
      goto L0
    };
    read: $r0 := flag[0]
  };
  CS: slocked write: turn := 0;
  slocked write: flag[my] := 0;
  goto L0
)";
    Machine m = *get_machine(ss);
    vector<Machine::PTransition> test{
      {6, Stmt<int>::update(0, VecSet<MemLoc<int>>::singleton(MemLoc<int>::local(0))),    6, 0}, // P0: update(flag[P0], P0)
      {6, Stmt<int>::update(0, VecSet<MemLoc<int>>::singleton(MemLoc<int>::local(0, 0))), 6, 1}, //   P1: update(flag[P0], P0)
      {6, Stmt<int>::update(1, VecSet<MemLoc<int>>::singleton(MemLoc<int>::local(0, 0))), 6, 0}, // P0: update(flag[P1], P1)
      get_transition(m, 0, 1, 6),                                                                // P0: read: flag[P1] = 0
      get_transition(m, 0, 0, 1),                                                                // P0: slocked write: flag[P0] := 1
      get_transition(m, 1, 1, 6),                                                                //   P1: read: flag[P0] = 0
      {1, Stmt<int>::update(1, VecSet<MemLoc<int>>::singleton(MemLoc<int>::local(0))),    1, 1}, //   P1: update(flag[P1], P1)
      get_transition(m, 1, 0, 1)                                                                 //   P1: socked write: flag[P1] := 1
    };
    test_pre_sequence("slocked dekker", m, test);
  }
}

void PwsConstraint::test() {
  PwsConstraint::test_pre();
}
