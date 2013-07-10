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

#include "test.h"
#include "vips_bit_reachability.h"

#include <functional>
#include <stdexcept>

Reachability::Result *VipsBitReachability::reachability(Arg *arg) const{
  const Machine &machine = arg->machine;
  Result *result = new Result(machine);
  result->timer.start();
  result->result = UNREACHABLE;

  bool found_forbidden = false;

  VipsBitConstraint::Common common(machine);

  /* stack contains constraints that have been found but not explored */
  std::vector<const VipsBitConstraint*> stack;

  /* The set of keys of visited is the set of visited constraints.
   * Each visited constraint maps to a description of its parent.
   */
  std::map<const VipsBitConstraint*,parent_t,vbcmp_t> visited(get_comparator(common));

  /* Find the initial constraints */
  {
    std::set<VipsBitConstraint*> init = common.get_initial_constraints();
    result->generated_constraints = result->stored_constraints = init.size();
    for(auto it = init.begin(); it != init.end(); ++it){
      if((*it)->is_forbidden(common)){
        result->trace = new Trace(0);
        result->result = REACHABLE;
        found_forbidden = true;
      }
      stack.push_back(*it);
      visited[*it] = parent_t();
    }
  }

  while(!found_forbidden && stack.size()){
    const VipsBitConstraint *vbc = stack.back();
    stack.pop_back();

    VecSet<const Machine::PTransition*> transes = vbc->partred(common);

    for(int i = 0; !found_forbidden && i < transes.size(); ++i){
      VipsBitConstraint *child = vbc->post(common,*transes[i]);
      if(child){
        ++result->generated_constraints;
        if(visited.count(child)){
          delete child;
        }else{
          visited[child] = parent_t(transes[i],vbc);
          stack.push_back(child);
          if(child->is_forbidden(common)){
            result->result = REACHABLE;
            found_forbidden = true;
            /* Construct trace */
            {
              result->trace = new Trace(0);
              const parent_t *pt = &visited[child];
              while(pt->parent){
                result->trace->push_front(0,*pt->trans);
                pt = &visited[pt->parent];
              }
            }
          }
        }
      }
    }
  }

  result->stored_constraints = visited.size();

  /* Cleanup */
  for(auto it = visited.begin(); it != visited.end(); ++it){
    delete it->first;
  }

  result->timer.stop();
  return result;
};

VipsBitReachability::vbcmp_t VipsBitReachability::get_comparator(const VipsBitConstraint::Common &common){
  return [&common](const VipsBitConstraint *a,const VipsBitConstraint *b){
    return common.compare(*a,*b) < 0;
  };
};

void VipsBitReachability::test(){
  std::function<Machine*(std::string)> get_machine = 
    [](std::string rmm){
    std::stringstream ss(rmm);
    Lexer lex(ss);
    return new Machine(Parser::p_test(lex));
  };

  /* Test 1: Dekker */
  {
    Machine *m = get_machine
      ("forbidden CS CS\n"
       "data\n"
       "  x = 0 : [0:1]\n"
       "  y = 0 : [0:1]\n"
       "process\n"
       "text\n"
       "L1:\n"
       "  write: x := 1;\n"
       "  read: y = 0;\n"
       "CS:\n"
       "  write: x := 0;\n"
       "  goto L1\n"
       "process\n"
       "text\n"
       "L1:\n"
       "  write: y := 1;\n"
       "  read: x = 0;\n"
       "CS:\n"
       "  write: y := 0;\n"
       "  goto L1\n"
       );

    VipsBitReachability reach;

    Arg arg(*m);
    Result *res = reach.reachability(&arg);
    Test::inner_test("#1 Simple Dekker",res->result == REACHABLE);

    delete res;
    delete m;
  }

  /* Test 2: Dekker 1 syncwr */
  {
    Machine *m = get_machine
      ("forbidden CS CS\n"
       "data\n"
       "  x = 0 : [0:1]\n"
       "  y = 0 : [0:1]\n"
       "process\n"
       "text\n"
       "L1:\n"
       "  syncwr: x := 1;\n"
       "  read: y = 0;\n"
       "CS:\n"
       "  write: x := 0;\n"
       "  goto L1\n"
       "process\n"
       "text\n"
       "L1:\n"
       "  write: y := 1;\n"
       "  read: x = 0;\n"
       "CS:\n"
       "  write: y := 0;\n"
       "  goto L1\n"
       );

    VipsBitReachability reach;

    Arg arg(*m);
    Result *res = reach.reachability(&arg);
    Test::inner_test("#2 Simple Dekker",res->result == REACHABLE);

    delete res;
    delete m;
  }

  /* Test 3: Dekker 1 syncwr (other process) */
  {
    Machine *m = get_machine
      ("forbidden CS CS\n"
       "data\n"
       "  x = 0 : [0:1]\n"
       "  y = 0 : [0:1]\n"
       "process\n"
       "text\n"
       "L1:\n"
       "  write: x := 1;\n"
       "  read: y = 0;\n"
       "CS:\n"
       "  write: x := 0;\n"
       "  goto L1\n"
       "process\n"
       "text\n"
       "L1:\n"
       "  syncwr: y := 1;\n"
       "  read: x = 0;\n"
       "CS:\n"
       "  write: y := 0;\n"
       "  goto L1\n"
       );

    VipsBitReachability reach;

    Arg arg(*m);
    Result *res = reach.reachability(&arg);
    Test::inner_test("#3 Simple Dekker",res->result == REACHABLE);

    delete res;
    delete m;
  }

  /* Test 4: Dekker syncwr in both processes */
  {
    Machine *m = get_machine
      ("forbidden CS CS\n"
       "data\n"
       "  x = 0 : [0:1]\n"
       "  y = 0 : [0:1]\n"
       "process\n"
       "text\n"
       "L1:\n"
       "  syncwr: x := 1;\n"
       "  read: y = 0;\n"
       "CS:\n"
       "  write: x := 0;\n"
       "  goto L1\n"
       "process\n"
       "text\n"
       "L1:\n"
       "  syncwr: y := 1;\n"
       "  read: x = 0;\n"
       "CS:\n"
       "  write: y := 0;\n"
       "  goto L1\n"
       );

    VipsBitReachability reach;

    Arg arg(*m);
    Result *res = reach.reachability(&arg);
    Test::inner_test("#4 Simple Dekker",res->result == REACHABLE);

    delete res;
    delete m;
  }

  /* Test 5: Dekker fence in one process */
  {
    Machine *m = get_machine
      ("forbidden CS CS\n"
       "data\n"
       "  x = 0 : [0:1]\n"
       "  y = 0 : [0:1]\n"
       "process\n"
       "text\n"
       "L1:\n"
       "  write: x := 1;\n"
       "  fence;\n"
       "  read: y = 0;\n"
       "CS:\n"
       "  write: x := 0;\n"
       "  goto L1\n"
       "process\n"
       "text\n"
       "L1:\n"
       "  write: y := 1;\n"
       "  read: x = 0;\n"
       "CS:\n"
       "  write: y := 0;\n"
       "  goto L1\n"
       );

    VipsBitReachability reach;

    Arg arg(*m);
    Result *res = reach.reachability(&arg);
    Test::inner_test("#5 Simple Dekker",res->result == REACHABLE);

    delete res;
    delete m;
  }
  /* Test 6: Dekker fence in one process */
  {
    Machine *m = get_machine
      ("forbidden CS CS\n"
       "data\n"
       "  x = 0 : [0:1]\n"
       "  y = 0 : [0:1]\n"
       "process\n"
       "text\n"
       "L1:\n"
       "  write: x := 1;\n"
       "  read: y = 0;\n"
       "CS:\n"
       "  write: x := 0;\n"
       "  goto L1\n"
       "process\n"
       "text\n"
       "L1:\n"
       "  write: y := 1;\n"
       "  fence;\n"
       "  read: x = 0;\n"
       "CS:\n"
       "  write: y := 0;\n"
       "  goto L1\n"
       );

    VipsBitReachability reach;

    Arg arg(*m);
    Result *res = reach.reachability(&arg);
    Test::inner_test("#6 Simple Dekker",res->result == REACHABLE);

    delete res;
    delete m;
  }

  /* Test 7: Dekker fence in both processes */
  {
    Machine *m = get_machine
      ("forbidden CS CS\n"
       "data\n"
       "  x = 0 : [0:1]\n"
       "  y = 0 : [0:1]\n"
       "process\n"
       "text\n"
       "L1:\n"
       "  write: x := 1;\n"
       "  fence;\n"
       "  read: y = 0;\n"
       "CS:\n"
       "  write: x := 0;\n"
       "  goto L1\n"
       "process\n"
       "text\n"
       "L1:\n"
       "  write: y := 1;\n"
       "  fence;\n"
       "  read: x = 0;\n"
       "CS:\n"
       "  write: y := 0;\n"
       "  goto L1\n"
       );

    VipsBitReachability reach;

    Arg arg(*m);
    Result *res = reach.reachability(&arg);
    Test::inner_test("#7 Simple Dekker",res->result == UNREACHABLE);

    delete res;
    delete m;
  }
  
};
