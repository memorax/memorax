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

#include "min_coverage.h"
#include "test.h"

namespace MinCoverage{

  SubsetIVSGenerator::SubsetIVSGenerator(const std::vector<VecSet<int> > &T,
                                         const std::map<int,VecSet<int> > &cmap)
    : Tvec(T), cov_map(cmap), dummy_cost([](const int&){ return 0; }) {
    candidates.push(CandSet<int>(Tvec,cov_map,dummy_cost));
  };

  bool SubsetIVSGenerator::generate(int i){
    while(int(mcs.size()) <= i && !candidates.empty()){
      CandSet<int> C = candidates.front();
      if(!is_subsumed(C)){
        int i = C.get_uncovered_Ti();
        if(i == -1){
          // C is a subset minimal coverage set
          mcs.push_back(C.get_set());
        }else{
          for(auto it = Tvec[i].begin(); it != Tvec[i].end(); ++it){
            CandSet<int> C2(C);
            C2.insert(*it);
            candidates.push(C2);
          }
        }
      }
      candidates.pop();
    }
    return int(mcs.size()) > i;
  };

  void test(){

    std::function<int(const int&)>
      unit_cost =
      [](const int&){ return 1; };

    std::function<int(const int&)>
      id_cost =
      [](const int &i){ return i; };

    std::function<std::set<VecSet<int> >(const std::pair<sol_iterator<int>,sol_iterator<int> > &)> setofpr =
      [](const std::pair<sol_iterator<int>,sol_iterator<int> > &pr){
      return std::set<VecSet<int> >(pr.first,pr.second);
    };

    {
      std::vector<VecSet<int> > T(3);
      T[0].insert(1); T[0].insert(2); T[0].insert(3);
      T[1].insert(2);
      T[2].insert(3); T[2].insert(4);
      const std::map<int,VecSet<int> > &cov_map = get_coverage_map(T);

      {
        CandSet<int> cs0(T,cov_map,unit_cost);
        Test::inner_test("Initial cost",cs0.get_cost() == 0);
        {
          int sug = cs0.get_uncovered_Ti();
          Test::inner_test("Initial get uncovered",sug == 0 || sug == 1 || sug == 2);
        }
        Test::inner_test("Initially not covered",!cs0.total_coverage());

        cs0.insert(2);

        Test::inner_test("Single cost",cs0.get_cost() == 1);
        {
          int sug = cs0.get_uncovered_Ti();
          Test::inner_test("Intermediate get uncovered",sug == 2);
        }

        cs0.insert(3);

        Test::inner_test("Double cost",cs0.get_cost() == 2);
        {
          int sug = cs0.get_uncovered_Ti();
          Test::inner_test("Covered get covered",sug == -1);
        }
        Test::inner_test("Covered",cs0.total_coverage());
      }

      {
        std::set<VecSet<int> > res_unit;
        VecSet<int> res0; res0.insert(2); res0.insert(3);
        VecSet<int> res1; res1.insert(2); res1.insert(4);
        res_unit.insert(res0); res_unit.insert(res1);

        std::set<VecSet<int> > Tset(T.begin(),T.end());
        Test::inner_test("#1 Min coverage (unit)",res_unit.count(min_coverage<int>(Tset)));
        Test::inner_test("#1 Min coverage all (unit)",res_unit == setofpr(min_coverage_all<int>(Tset)));

        std::set<VecSet<int> > res_id;
        VecSet<int> res2; res2.insert(2); res2.insert(3);
        res_id.insert(res2);
        Test::inner_test("#1 Min coverage (id)",res_id.count(min_coverage<int>(Tset,id_cost)));
        Test::inner_test("#1 Min coverage all (id)",res_id == setofpr(min_coverage_all<int>(Tset,id_cost)));
        std::set<VecSet<int> > ss_res = {{2,3},{2,4}};
        Test::inner_test("#1 Min coverage subset all",
                         setofpr(subset_min_coverage_all<int>(Tset)) == ss_res);

      }
    }

    {
      std::vector<VecSet<int> > T(3);
      T[0].insert(3); T[0].insert(4); T[0].insert(5);
      T[1].insert(5); T[1].insert(1);
      T[2].insert(2); T[2].insert(6);

      std::set<VecSet<int> > res_unit;
      VecSet<int> res0; res0.insert(5); res0.insert(2);
      VecSet<int> res1; res1.insert(5); res1.insert(6);
      res_unit.insert(res0); res_unit.insert(res1);

      std::set<VecSet<int> > Tset(T.begin(),T.end());
      Test::inner_test("#2 Min coverage (unit)",res_unit.count(min_coverage<int>(Tset)));
      Test::inner_test("#2 Min coverage all (unit)",res_unit == setofpr(min_coverage_all<int>(Tset)));

      std::set<VecSet<int> > res_id;
      VecSet<int> res2; res2.insert(1); res2.insert(2); res2.insert(3);
      res_id.insert(res2);

      Test::inner_test("#2 Min coverage (id)",res_id.count(min_coverage<int>(Tset,id_cost)));
      Test::inner_test("#2 Min coverage all (id)",res_id == setofpr(min_coverage_all<int>(Tset,id_cost)));
      std::set<VecSet<int> > ss_res = {{5,2},{3,1,2},{5,6},{3,1,6},{4,1,2},{4,1,6}};
      Test::inner_test("#2 Min coverage subset all",
                       setofpr(subset_min_coverage_all<int>(Tset)) == ss_res);
    }

    {
      /* Test #3: Empty T */
      std::set<VecSet<int> > Tset;

      std::set<VecSet<int> > res;
      res.insert(VecSet<int>());
      Test::inner_test("#3 Min coverage (unit)",res.count(min_coverage<int>(Tset)));
      Test::inner_test("#3 Min coverage all (unit)",res == setofpr(min_coverage_all<int>(Tset)));

      Test::inner_test("#3 Min coverage (id)",res.count(min_coverage<int>(Tset,id_cost)));
      Test::inner_test("#3 Min coverage all (id)",res == setofpr(min_coverage_all<int>(Tset,id_cost)));
      Test::inner_test("#3 Min coverage subset all",
                       setofpr(subset_min_coverage_all<int>(Tset)) == res);
    }

    {
      /* Test #4: Singleton T */
      std::set<VecSet<int> > Tset;
      VecSet<int> T0; T0.insert(1); T0.insert(2); T0.insert(3);
      Tset.insert(T0);

      std::set<VecSet<int> > res_unit;
      VecSet<int> res1; res1.insert(1);
      VecSet<int> res2; res2.insert(2);
      VecSet<int> res3; res3.insert(3);
      res_unit.insert(res1); res_unit.insert(res2); res_unit.insert(res3);
      std::set<VecSet<int> > res_id;
      res_id.insert(res1);

      Test::inner_test("#4 Min coverage (unit)",res_unit.count(min_coverage<int>(Tset)));
      Test::inner_test("#4 Min coverage all (unit)",res_unit == setofpr(min_coverage_all<int>(Tset)));

      Test::inner_test("#4 Min coverage (id)",res_id.count(min_coverage<int>(Tset,id_cost)));
      Test::inner_test("#4 Min coverage all (id)",res_id == setofpr(min_coverage_all<int>(Tset,id_cost)));
      Test::inner_test("#4 Min coverage subset all",
                       setofpr(subset_min_coverage_all<int>(Tset)) == res_unit);
    }

    /* Test sol_iterator */
    {
      std::function<std::set<std::set<int> >(const sol_iterator<int> &)> setof =
        [](const sol_iterator<int> &begin){
        sol_iterator<int> end;
        std::set<std::set<int> > res;
        for(auto it = begin; it != end; ++it){
          res.insert(std::set<int>(it->begin(),it->end()));
        }
        return res;
      };

      std::function<bool(const std::set<std::set<int> >&,const std::set<std::set<int> >&)> eq_sets =
        [](const std::set<std::set<int> > &S,const std::set<std::set<int> > &T){
        return S == T;
      };

      /* Test 1-10 */
      {

        typedef std::set<VecSet<int> > vs;

        Test::inner_test("sol_iterator #1",
                         eq_sets(setof(sol_iterator<int>({{0,1},{0,2}},{{1,2},{3},{4,5}})),
                                 {{1,3},{2,3},{1,4},{1,5},{2,4},{2,5}}));

        Test::inner_test("sol_iterator #2",
                         eq_sets(setof(sol_iterator<int>(vs({{0}}),{{0}})),{{0}}));

        Test::inner_test("sol_iterator #3",
                         eq_sets(setof(sol_iterator<int>(vs({{0}}),{{0,1}})),{{0},{1}}));

        Test::inner_test("sol_iterator #4",
                         eq_sets(setof(sol_iterator<int>(vs({}),{})),{}));

        Test::inner_test("sol_iterator #5",
                         eq_sets(setof(sol_iterator<int>(vs({}),{{1,2},{3,4}})),{}));
      }
    }

  };

};
