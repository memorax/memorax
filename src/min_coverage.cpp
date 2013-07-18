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

  void test(){

    std::function<int(const int&)>
      unit_cost = 
      [](const int&){ return 1; };

    std::function<int(const int&)>
      id_cost = 
      [](const int &i){ return i; };

    {
      std::vector<std::set<int> > T(3);
      T[0].insert(1); T[0].insert(2); T[0].insert(3);
      T[1].insert(2);
      T[2].insert(3); T[2].insert(4);
      const std::map<int,std::set<int> > &cov_map = get_coverage_map(T);

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
        std::set<std::set<int> > res_unit;
        std::set<int> res0; res0.insert(2); res0.insert(3);
        std::set<int> res1; res1.insert(2); res1.insert(4);
        res_unit.insert(res0); res_unit.insert(res1);

        std::set<std::set<int> > Tset(T.begin(),T.end());
        Test::inner_test("#1 Min coverage (unit)",res_unit.count(min_coverage<int>(Tset)));
        Test::inner_test("#1 Min coverage all (unit)",res_unit == min_coverage_all<int>(Tset));

        std::set<std::set<int> > res_id;
        std::set<int> res2; res2.insert(2); res2.insert(3);
        res_id.insert(res2);
        Test::inner_test("#1 Min coverage (id)",res_id.count(min_coverage<int>(Tset,id_cost)));
        Test::inner_test("#1 Min coverage all (id)",res_id == min_coverage_all<int>(Tset,id_cost));

      }
    }

    {
      std::vector<std::set<int> > T(3);
      T[0].insert(3); T[0].insert(4); T[0].insert(5);
      T[1].insert(5); T[1].insert(1);
      T[2].insert(2); T[2].insert(6);

      std::set<std::set<int> > res_unit;
      std::set<int> res0; res0.insert(5); res0.insert(2);
      std::set<int> res1; res1.insert(5); res1.insert(6);
      res_unit.insert(res0); res_unit.insert(res1);

      std::set<std::set<int> > Tset(T.begin(),T.end());
      Test::inner_test("#2 Min coverage (unit)",res_unit.count(min_coverage<int>(Tset)));
      Test::inner_test("#2 Min coverage all (unit)",res_unit == min_coverage_all<int>(Tset));

      std::set<std::set<int> > res_id;
      std::set<int> res2; res2.insert(1); res2.insert(2); res2.insert(3);
      res_id.insert(res2);

      Test::inner_test("#2 Min coverage (id)",res_id.count(min_coverage<int>(Tset,id_cost)));
      Test::inner_test("#2 Min coverage all (id)",res_id == min_coverage_all<int>(Tset,id_cost));
    }

    {
      /* Test #3: Empty T */
      std::set<std::set<int> > Tset;

      std::set<std::set<int> > res;
      res.insert(std::set<int>());
      Test::inner_test("#3 Min coverage (unit)",res.count(min_coverage<int>(Tset)));
      Test::inner_test("#3 Min coverage all (unit)",res == min_coverage_all<int>(Tset));

      Test::inner_test("#3 Min coverage (id)",res.count(min_coverage<int>(Tset,id_cost)));
      Test::inner_test("#3 Min coverage all (id)",res == min_coverage_all<int>(Tset,id_cost));
    }

    {
      /* Test #4: Singleton T */
      std::set<std::set<int> > Tset;
      std::set<int> T0; T0.insert(1); T0.insert(2); T0.insert(3);
      Tset.insert(T0);

      std::set<std::set<int> > res_unit;
      std::set<int> res1; res1.insert(1);
      std::set<int> res2; res2.insert(2);
      std::set<int> res3; res3.insert(3);
      res_unit.insert(res1); res_unit.insert(res2); res_unit.insert(res3);
      std::set<std::set<int> > res_id;
      res_id.insert(res1);

      Test::inner_test("#4 Min coverage (unit)",res_unit.count(min_coverage<int>(Tset)));
      Test::inner_test("#4 Min coverage all (unit)",res_unit == min_coverage_all<int>(Tset));

      Test::inner_test("#4 Min coverage (id)",res_id.count(min_coverage<int>(Tset,id_cost)));
      Test::inner_test("#4 Min coverage all (id)",res_id == min_coverage_all<int>(Tset,id_cost));
    }

  };

};
