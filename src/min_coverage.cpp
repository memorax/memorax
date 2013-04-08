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

      {
        CandSet<int> cs0(T,unit_cost);
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
        std::set<std::set<int> > Tset(T.begin(),T.end());
        std::set<int> mcset = min_coverage<int>(Tset);
        Test::inner_test("Min coverage (unit)",
                         mcset.size() == 2 && 
                         ((mcset.count(2) && mcset.count(3)) || 
                          (mcset.count(2) && mcset.count(4))));

        std::set<int> mcset_id = min_coverage<int>(Tset,id_cost);
        Test::inner_test("Min coverage (id)",
                         mcset.size() == 2 && mcset.count(2) && mcset.count(3));
      }
    }

    {
      std::vector<std::set<int> > T(3);
      T[0].insert(3); T[0].insert(4); T[0].insert(5);
      T[1].insert(5); T[1].insert(1);
      T[2].insert(2); T[2].insert(6);

      std::set<std::set<int> > Tset(T.begin(),T.end());
      std::set<int> mcset = min_coverage<int>(Tset);
      Test::inner_test("Min coverage (unit)",
                       mcset.size() == 2 && 
                       ((mcset.count(5) && mcset.count(2)) || 
                        (mcset.count(5) && mcset.count(6))));

      std::set<int> mcset_id = min_coverage<int>(Tset,id_cost);
      Test::inner_test("Min coverage (id)",
                       mcset_id.size() == 3 && 
                       mcset_id.count(1) && mcset_id.count(2) && mcset_id.count(3));
    }

  };

};
