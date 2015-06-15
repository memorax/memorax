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

#include "fencins.h"
#include "log.h"
#include "min_coverage.h"
#include "preprocessor.h"      // for testing
#include "sb_constraint.h"     // for testing
#include "channel_container.h" // for testing
#include "sb_tso_bwd.h"        // for testing
#include "test.h"              // for testing
#include "tso_simple_fencer.h" // for testing
#include "vecset.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace Fencins{

  void add_disj_to_cnf(const VecSet<Sync*> &disj, std::set<VecSet<Sync*> > *cnf){
    for(auto it = cnf->begin(); it != cnf->end(); ){
      if(std::includes(it->begin(),it->end(),
                       disj.begin(),disj.end())){
        it = cnf->erase(it);
      }else{
        ++it;
      }
    }
    cnf->insert(disj);
  };

  typedef bool (*sync_ptr_less_t)(Sync * const a, Sync * const b);
  bool sync_ptr_less(Sync * const a, Sync * const b){
    return *a < *b;
  };

  void deep_delete(const std::set<VecSet<Sync*> > &S){
    std::set<Sync*> deleted;
    for(auto it = S.begin(); it != S.end(); ++it){
      for(auto it2 = it->begin(); it2 != it->end(); ++it2){
        if(deleted.count(*it2) == 0){
          delete *it2;
          deleted.insert(*it2);
        }
      }
    }
  };

  void deep_delete(const std::set<std::set<Sync*> > &S){
    std::set<Sync*> deleted;
    for(auto it = S.begin(); it != S.end(); ++it){
      for(auto it2 = it->begin(); it2 != it->end(); ++it2){
        if(deleted.count(*it2) == 0){
          delete *it2;
          deleted.insert(*it2);
        }
      }
    }
  };

  void deep_delete(const VecSet<Sync*> &S){
    for(auto it = S.begin(); it != S.end(); ++it){
      delete *it;
    }
  };

  void delete_and_clear(std::vector<const Sync::InsInfo*> *v){
    for(unsigned i = 0; i < v->size(); ++i){
      delete (*v)[i];
    }
    v->clear();
  };

  /* Returns a set SS' which is identical to SS, but where Sync
   * objects are cloned such that no Sync* pointers in sets in SS'
   * point to the same address.
   *
   * SS' still shares pointers with SS.
   */
  std::set<std::set<Sync*> > separate_pointers(const std::set<VecSet<Sync*> > &SS){
    std::set<Sync*> seen_syncs;
    std::set<std::set<Sync*> > SS2;

    for(auto it = SS.begin(); it != SS.end(); ++it){
      std::set<Sync*> S;
      for(auto it2 = it->begin(); it2 != it->end(); ++it2){
        if(seen_syncs.count(*it2)){
          /* Duplicate pointer. Need to clone. */
          S.insert((*it2)->clone());
        }else{
          /* First occurrence of *it2 */
          S.insert(*it2);
        }
      }
      SS2.insert(S);
    }

    return SS2;
  };

  Machine *insert_syncs(const Machine &m,
                        const VecSet<Sync*> syncs,
                        std::vector<const Sync::InsInfo*> *m_infos){
    assert(m_infos->empty());

    Machine *cur_m = new Machine(m);
    for(auto it = syncs.begin(); it != syncs.end(); ++it){
      Sync::InsInfo *info;
      Machine *new_m = (*it)->insert(*cur_m,*m_infos,&info);
      m_infos->push_back(info);
      delete cur_m;
      cur_m = new_m;
    }

    return cur_m;
  };

  std::set<std::set<Sync*> > fencins(const Machine &m,
                                     Reachability &r,
                                     reach_arg_init_t reach_arg_init,
                                     TraceFencer &tf,
                                     min_aspect_t ma,
                                     int max_solutions,
                                     cost_fn_t cost){

    /* Each set S in syncs is such that some synchronization in S is
     * necessary.
     *
     * Invariant: For S0,S1 in syncs and s0 in S0 and s1 in S1, if *s0
     * and *s1 are the same synchronization (according to
     * Sync::operator==), then they are the very same object (the
     * pointers are the same: s0 == s1).
     */
    std::set<VecSet<Sync *> > syncs;
    /* If mcs_up_to_date is set, then mcs is the min coverage sets of
     * syncs.
     */
    bool mcs_up_to_date = false;
    std::pair<MinCoverage::sol_iterator<Sync*>,
              MinCoverage::sol_iterator<Sync*> > mcs;
    /* Maps Sync objects o to a pointer p to a Sync object o' such
     * that o == o' and p is in some set in syncs.
     */
    std::map<Sync*,Sync*,sync_ptr_less_t> sync_ptr(sync_ptr_less);

    /* fence_sets is the set of hitherto found sufficient fence sets.
     *
     * All Syncs in fence_sets are unique objects, that have been
     * cloned from the ones in syncs. fence_sets_uncloned contains the
     * same sets as fence_sets, but in fence_sets_uncloned all Sync
     * pointers are pointers into syncs.
     */
    std::set<VecSet<Sync*> > fence_sets;
    std::set<VecSet<Sync*> > fence_sets_uncloned;

    Reachability::Result *prev_result = 0;
    bool done = false;
    do{
      assert(fence_sets.size() == fence_sets_uncloned.size());
      VecSet<Sync*> mc; // The next Sync set to check
      std::vector<const Sync::InsInfo*> m_infos; // Log from inserting mc
      const Machine *m_synced = 0; // The machine with mc inserted
      // Find the next Sync set to check (mc)
      {
        {
          int sync_count = 0;
          for(auto it = syncs.begin(); it != syncs.end(); ++it){
            sync_count += it->size();
          }
          Log::debug << "Total sync occurrence count: " << sync_count
                     << " in " << syncs.size() << " disjunctions";
          if(syncs.size()){
            Log::debug << ", avg. count: " << (double(sync_count) / double(syncs.size())) << "\n";
          }
          Log::debug << "\n";
        }
        if(!mcs_up_to_date){
          Timer tm;
          tm.start();
          if(ma == COST){
            mcs = MinCoverage::min_coverage_all<Sync*>(syncs,cost);
          }else{
            assert(ma == SUBSET);
            mcs = MinCoverage::subset_min_coverage_all<Sync*>(syncs);
          }
          mcs_up_to_date = true;
          tm.stop();
          Log::debug << "min_coverage time: " << tm.get_time() << " s.\n";
        }
        // Otherwise find one which is not in fence_sets
        for(; mcs.first != mcs.second; ++mcs.first){
          if(fence_sets_uncloned.count(*mcs.first) == 0){
            // Try this one
            try{
              mc = *mcs.first;
              m_infos.clear();
              m_synced = insert_syncs(m,mc,&m_infos);
              break;
            }catch(Sync::Incompatible *exc){
              // mc contains some incompatible Syncs
              // Skip this mc and try the next one
              // (Note that break is not executed in this case.)
              delete exc;
            }
          }
        }
        if(mcs.first == mcs.second){
          /* There are no more fence sets. */
          assert(fence_sets.size());
          break;
        }
      }
      assert(m_synced != 0);

      Log::msg << "Trying the following synchronization set:\n";
      if(mc.empty()){
        Log::msg << "  (No synchronization)\n";
      }
      for(auto s : mc){
        s->print(m,Log::msg,Log::json);
      }
      Log::msg << "\n";

      Log::msg << "Current solution count: " << fence_sets.size() << "\n";
      Reachability::Arg *rarg = reach_arg_init(*m_synced,prev_result);
      Reachability::Result *res = r.reachability(rarg);
      if(prev_result) delete prev_result;
      prev_result = res;

      Log::msg << res->to_string() << "\n";

      if(res->result == Reachability::REACHABLE){
        if(!res->trace){
          delete m_synced;
          delete rarg;
          delete res;
          deep_delete(fence_sets);
          deep_delete(syncs);
          delete_and_clear(&m_infos);
          throw new std::logic_error("Fencins: Received no trace from underlying reachability analysis.");
        }
        std::set<std::set<Sync*> > new_syncs = tf.fence(*res->trace,m_infos);
        if(new_syncs.empty()){
          /* There is no solution for m */
          assert(fence_sets.empty());
          done = true;
        }
        for(auto it = new_syncs.begin(); it != new_syncs.end(); ++it){
          VecSet<Sync*> disj;
          for(auto it2 = it->begin(); it2 != it->end(); ++it2){
            if(sync_ptr.count(*it2)){
              disj.insert(sync_ptr.at(*it2));
            }else{
              Sync *p = (*it2)->clone();
              disj.insert(p);
              sync_ptr[p] = p;
            }
          }
          add_disj_to_cnf(disj,&syncs);
          mcs_up_to_date = false;
        }
        deep_delete(new_syncs);
      }else if(res->result == Reachability::UNREACHABLE){
        /* mc is a solution for m */
        /* Clone mc */
        Log::msg << "Synchronization set shown to be a solution.\n\n";
        VecSet<Sync*> fs;
        for(auto it = mc.begin(); it != mc.end(); ++it){
          fs.insert((*it)->clone());
        }
        fence_sets.insert(fs);
        fence_sets_uncloned.insert(mc);
        assert(max_solutions == 0 || int(fence_sets.size()) <= max_solutions);
        if(int(fence_sets.size()) == max_solutions){
          done = true;
        }
      }else{
        assert(res->result == Reachability::FAILURE);
        delete m_synced;
        delete rarg;
        delete res;
        deep_delete(fence_sets);
        deep_delete(syncs);
        delete_and_clear(&m_infos);
        throw new std::logic_error("Fencins: FAILURE from underlying reachability analysis.");
      }

      delete_and_clear(&m_infos);
      delete m_synced;
      delete rarg;
    }while(!done);

    assert(prev_result);
    delete prev_result;

    deep_delete(syncs);

    return separate_pointers(fence_sets);
  };

  void test(){

    std::function<Machine*(std::string)> get_machine =
      [](std::string rmm){
      std::stringstream ss(rmm);
      PPLexer lex(ss);
      return new Machine(Parser::p_test(lex));
    };

    std::function<int(const Machine&,int,std::string)> cs =
      [](const Machine &m, int pid, std::string lbl){
      return m.automata[pid].state_index_of_label(lbl);
    };

    std::function<bool(std::string,std::string)> test_sb_only_one =
      [&get_machine,&cs](std::string rmm, std::string fence_poses){
      Machine *m = get_machine(rmm);
      SbTsoBwd reach;
      reach_arg_init_t arg_init =
        [](const Machine &m, const Reachability::Result *)->Reachability::Arg*{
        SbConstraint::Common *common = new SbConstraint::Common(m);
        return new ExactBwd::Arg(m,common->get_bad_states(),common,new ChannelContainer());
      };
      TsoSimpleFencer fencer(*m,TsoSimpleFencer::FENCE);
      Log::loglevel_t ll = Log::get_primary_loglevel();
      Log::set_primary_loglevel(Log::loglevel_t(std::max(0,int(ll) - 1)));
      std::set<std::set<Sync*> > fence_sets =
      fencins(*m,reach,arg_init,fencer,COST,1);
        Log::set_primary_loglevel(ll);

      if(fence_sets.empty()){
        deep_delete(fence_sets);
        delete m;
        return fence_poses == "";
      }

      if(fence_sets.size() > 1 || fence_poses == ""){
        deep_delete(fence_sets);
        delete m;
        return false;
      }

      Log::result << "Fence Sets:\n";
      for(auto it = fence_sets.begin(); it != fence_sets.end(); ++it){
        Log::result << "  * Fence Set *\n";
        for(auto fsit = it->begin(); fsit != it->end(); ++fsit){
          Log::result << (*fsit)->to_string(*m) << "\n";
        }
      }

      std::vector<std::set<int> > act_fences(m->automata.size());
      for(auto it = fence_sets.begin()->begin(); it != fence_sets.begin()->end(); ++it){
        TsoFenceSync *p = dynamic_cast<TsoFenceSync*>(*it);
        assert(p);
        act_fences[p->get_pid()].insert(p->get_q());
      }

      std::set<std::vector<std::set<int> > > allowed_fences;
      {
        std::size_t i = 0, j;
        while(i != std::string::npos){
          j = fence_poses.find("\n",i);
          std::stringstream ss(fence_poses.substr(i,(j == std::string::npos) ? std::string::npos : (j - i)));
          i = (j == std::string::npos) ? std::string::npos : (j+1);
          std::vector<std::set<int> > fences(m->automata.size());
          int pid = 0;
          std::string s;
          while(ss >> s){
            if(s == "|"){
              ++pid;
            }else{
              fences[pid].insert(cs(*m,pid,s));
            }
          }
          allowed_fences.insert(fences);
        }
      }

      deep_delete(fence_sets);
      delete m;

      return bool(allowed_fences.count(act_fences));
    };

    std::function<bool(std::string,std::string,
                       min_aspect_t,
                       std::function<int(const Sync*)>*)> test_sb_all =
      [&get_machine,&cs](std::string rmm, std::string fence_poses,
                         min_aspect_t ma,
                         std::function<int(const Sync*)> *cost){
      Machine *m = get_machine(rmm);
      SbTsoBwd reach;
      reach_arg_init_t arg_init =
        [](const Machine &m, const Reachability::Result *)->Reachability::Arg*{
        SbConstraint::Common *common = new SbConstraint::Common(m);
        return new ExactBwd::Arg(m,common->get_bad_states(),common,new ChannelContainer());
      };
      TsoSimpleFencer fencer(*m,TsoSimpleFencer::FENCE);
      std::set<std::set<Sync*> > fence_sets;
      Log::loglevel_t ll = Log::get_primary_loglevel();
      Log::set_primary_loglevel(Log::loglevel_t(std::max(0,int(ll) - 1)));
      if(cost){
        fence_sets = fencins(*m,reach,arg_init,fencer,COST,0,*cost);
      }else{
        fence_sets = fencins(*m,reach,arg_init,fencer,ma,0);
      }
      Log::set_primary_loglevel(ll);

      if(fence_sets.empty()){
        deep_delete(fence_sets);
        delete m;
        return fence_poses == "";
      }

      if(fence_poses == ""){
        deep_delete(fence_sets);
        delete m;
        return false;
      }

      Log::result << "Fence Sets:\n";
      for(auto it = fence_sets.begin(); it != fence_sets.end(); ++it){
        Log::result << "  * Fence Set *\n";
        for(auto fsit = it->begin(); fsit != it->end(); ++fsit){
          Log::result << (*fsit)->to_string(*m) << "\n";
        }
      }

      std::set<std::vector<std::set<int> > > act_fences;
      for(auto it = fence_sets.begin(); it != fence_sets.end(); ++it){
        std::vector<std::set<int> > v(m->automata.size());
        for(auto it2 = it->begin(); it2 != it->end(); ++it2){
          TsoFenceSync *p = dynamic_cast<TsoFenceSync*>(*it2);
          assert(p);
          v[p->get_pid()].insert(p->get_q());
        }
        act_fences.insert(v);
      }

      std::set<std::vector<std::set<int> > > allowed_fences;
      {
        std::size_t i = 0, j;
        while(i != std::string::npos){
          j = fence_poses.find("\n",i);
          std::stringstream ss(fence_poses.substr(i,(j == std::string::npos) ? std::string::npos : (j - i)));
          i = (j == std::string::npos) ? std::string::npos : (j+1);
          std::vector<std::set<int> > fences(m->automata.size());
          int pid = 0;
          std::string s;
          while(ss >> s){
            if(s == "|"){
              ++pid;
            }else{
              fences[pid].insert(cs(*m,pid,s));
            }
          }
          allowed_fences.insert(fences);
        }
      }

      deep_delete(fence_sets);
      delete m;

      return allowed_fences == act_fences;
    };

    /* Test 1 */
    {
      std::string rmm =
        "forbidden CS CS\n"
        "data\n"
        "  x = 0 : [0:1]\n"
        "  y = 0 : [0:1]\n"
        "process\n"
        "text\n"
        "  write: x := 1;\n"
        "  L1: read: y = 0;\n"
        "  CS: nop\n"
        "process\n"
        "text\n"
        "  write: y := 1;\n"
        "  L1: nop;\n"
        "  L2: read: x = 0;\n"
        "  CS: nop\n";
      Test::inner_test("fencins only_one #1 (small Dekker variant)",
                       test_sb_only_one(rmm,"L1 | L1\nL1 | L2"));
    }

    /* Test 2 */
    {
      std::string rmm =
        "forbidden CS CS\n"
        "data\n"
        "  x = 0 : [0:1]\n"
        "  y = 0 : [0:1]\n"
        "process\n"
        "text\n"
        "  L0: write: x := 1;\n"
        "  L1: read: y = 0;\n"
        "  CS: write: x := 0;\n"
        "  goto L0\n"
        "process\n"
        "text\n"
        "  L0: write: y := 1;\n"
        "  L1: nop;\n"
        "  L2: read: x = 0;\n"
        "  CS: write: y := 0;\n"
        "  goto L0\n";
      Test::inner_test("fencins only_one #2 (small Dekker variant)",
                       test_sb_only_one(rmm,"L1 | L1\nL1 | L2"));
    }

    /* Test 3 */
    {
      std::string rmm =
        "forbidden CS CS\n"
        "data\n"
        "  x = 0 : [0:1]\n"
        "  y = 0 : [0:1]\n"
        "process\n"
        "text\n"
        "  L0: write: x := 1;\n"
        "  L1: read: y = 0;\n"
        "  CS: write: x := 0;\n"
        "  goto L1\n"
        "process\n"
        "text\n"
        "  L0: write: y := 1;\n"
        "  L1: nop;\n"
        "  L2: read: x = 0;\n"
        "  CS: write: y := 0;\n"
        "  goto L0\n";
      Test::inner_test("fencins only_one #3 (no solution)",
                       test_sb_only_one(rmm,""));
    }

    /* Test 4 */
    {
      std::string rmm =
        "forbidden\n"
        "  END0 END0 ;\n"
        "  END1 END1\n"
        "\n"
        "data\n"
        "  x0 = 0 : [0:1]\n"
        "  x1 = 0 : [0:1]\n"
        "  y0 = 0 : [0:1]\n"
        "  y1 = 0 : [0:1]\n"
        "\n"
        "process\n"
        "text\n"
        "  write: x0 := 1;\n"
        "  write: x1 := 1;\n"
        "  L2:\n"
        "  either{\n"
        "    read: y1 = 0;\n"
        "    END1: nop\n"
        "  or\n"
        "    read: y0 = 0;\n"
        "    END0: nop\n"
        "  }\n"
        "\n"
        "process\n"
        "text\n"
        "  write: y0 := 1;\n"
        "  write: y1 := 1;\n"
        "  L2:\n"
        "  either{\n"
        "    read: x1 = 0;\n"
        "    END1: nop\n"
        "  or\n"
        "    read: x0 = 0;\n"
        "    END0: nop\n"
        "  }\n";
      Test::inner_test("fencins only_one #4",
                       test_sb_only_one(rmm,"L2 | L2"));
      Test::inner_test("fencins all #4.2",
                       test_sb_all(rmm,"L2 | L2",COST,0));
    }

    /* Test 5 */
    {
      std::string rmm =
        "forbidden CS CS\n"
        "data\n"
        "  x = 0 : [0:1]\n"
        "  y = 0 : [0:1]\n"
        "process\n"
        "text\n"
        "  L0: write: x := 1;\n"
        "  L1: read: y = 0;\n"
        "  CS: write: x := 0;\n"
        "  goto L0\n"
        "process\n"
        "text\n"
        "  L0: write: y := 1;\n"
        "  L1: nop;\n"
        "  L2: read: x = 0;\n"
        "  CS: write: y := 0;\n"
        "  goto L0\n";
      Test::inner_test("fencins all #5 (small Dekker variant)",
                       test_sb_all(rmm,"L1 | L1\nL1 | L2",COST,0));
    }

    /* Test 6,7: empty set is solution */
    {
      std::string rmm =
        "forbidden CS CS\n"
        "data\n"
        "  x = 0 : [0:1]\n"
        "  y = 0 : [0:1]\n"
        "process\n"
        "text\n"
        "  L0: locked write: x := 1;\n"
        "  read: y = 0;\n"
        "  CS: write: x := 0;\n"
        "  goto L0\n"
        "process\n"
        "text\n"
        "  L0: locked write: y := 1;\n"
        "  read: x = 0;\n"
        "  CS: write: y := 0;\n"
        "  goto L0\n";
      Test::inner_test("fencins all #6",
                       test_sb_all(rmm,"|",COST,0));
      Test::inner_test("fencins only_one #7",
                       test_sb_only_one(rmm,"|"));
    }

    /* Test 8,9: disjunct solutions */
    {
      std::string rmm =
        "forbidden CS CS\n"
        "data\n"
        "  x0 = 0 : [0:1]\n"
        "  x1 = 0 : [0:1]\n"
        "  y0 = 0 : [0:1]\n"
        "  y1 = 0 : [0:1]\n"
        "process\n"
        "text\n"
        "  L0: write: x0 := 1;\n"
        "  L1: read: y0 = 0;\n"
        "  L2: write: x1 := 1;\n"
        "  L3: read: y1 = 0;\n"
        "  CS: nop\n"
        "process\n"
        "text\n"
        "  L0: write: y0 := 1;\n"
        "  L1: read: x0 = 0;\n"
        "  L2: write: y1 := 1;\n"
        "  L3: read: x1 = 0;\n"
        "  CS: nop\n";
      Test::inner_test("fencins all #8",
                       test_sb_all(rmm,
                                   "L1 | L1\n"
                                   "L3 | L3",COST,0));
      Test::inner_test("fencins all #9",
                       test_sb_all(rmm,
                                   "L1 | L1\n"
                                   "L3 | L3",SUBSET,0));
    }

    /* Test 10,11,12: disjunct solutions with different costs */
    {
      std::string rmm =
        "forbidden CS CS\n"
        "data\n"
        "  x0 = 0 : [0:1]\n"
        "  x1 = 0 : [0:1]\n"
        "  y0 = 0 : [0:1]\n"
        "  y1 = 0 : [0:1]\n"
        "process\n"
        "text\n"
        "  L0: write: x0 := 1;\n"
        "  L1: read: y0 = 0;\n"
        "  L2: either{\n"
        "    write: x1 := 1;\n"
        "    L21: read: y1 = 0\n"
        "  or\n"
        "    write: x1 := 1;\n"
        "    L22: read: y1 = 0\n"
        "  };"
        "  CS: nop\n"
        "process\n"
        "text\n"
        "  L0: write: y0 := 1;\n"
        "  L1: read: x0 = 0;\n"
        "  L2: either{\n"
        "    write: y1 := 1;\n"
        "    L21: read: x1 = 0\n"
        "  or\n"
        "    write: y1 := 1;\n"
        "    L22: read: x1 = 0\n"
        "  };"
        "  CS: nop\n";
      Test::inner_test("fencins all #10",
                       test_sb_all(rmm, "L1 | L1",COST,0));
      /* Should not return the set "L21 L22 | L21 L22" since it is
       * more expensive than "L1 | L1".
       *
       * Next we increase the price of L1 so we get both sets.
       */
      std::function<int(const Sync*)> expensive_L1 =
        [](const Sync *s){
        const FenceSync *p = dynamic_cast<const FenceSync*>(s);
        if(p->get_q() == 1){
          return 2;
        }else{
          return 1;
        }
      };
      Test::inner_test("fencins all #11",
                       test_sb_all(rmm,
                                   "L1 | L1\n"
                                   "L21 L22 | L21 L22",
                                   COST,
                                   &expensive_L1));

      Test::inner_test("fencins all #12",
                       test_sb_all(rmm,
                                   "L1 | L1\n"
                                   "L21 L22 | L21 L22",
                                   SUBSET,
                                   0));
    }
  };

};
