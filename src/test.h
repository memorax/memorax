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

#ifndef __TEST_H__
#define __TEST_H__

#include <functional>
#include <list>
#include <string>

class Test{
public:

  static bool inner_test(std::string testname, bool success);

  static int run_tests();
  static void add_test(std::string suitename,
                       const std::function<void()> &suite);

  static void add_all_tests(){
    add_test("Test",test_testing);
  };

  static void test_testing(){
    inner_test("Testing works?",true);
  };
private:
  static std::list<std::pair<std::string,const std::function<void()> > > the_tests;
  static int inner_failures; // Number of inner failures since for the current suite
  static int total_inner_failures; // Total number of inner failures
  static int total_inner_test_count; // Total number of inner tests executed
};

#endif
