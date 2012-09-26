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

#include "test.h"
#include "log.h"

std::list<std::pair<std::string,const std::function<void()> > > Test::the_tests;
int Test::inner_failures;
int Test::total_inner_failures;
int Test::total_inner_test_count;

void Test::add_test(std::string suitename, const std::function<void()> &suite){
  the_tests.push_back(std::pair<std::string,const std::function<void()> >(suitename,suite));
};

int Test::run_tests(){
  int result = 0;

  total_inner_failures = 0;
  total_inner_test_count = 0;

  for(auto it = the_tests.begin(); it != the_tests.end(); ++it){
    inner_failures = 0;
    Log::debug << "Running test suite: " << it->first << "\n";
    (it->second)();
    if(inner_failures == 0){
      Log::msg << it->first << ": Ok\n";
    }else{
      Log::msg << it->first << ": FAILURE\n";
      ++result;
    }
  }

  if(result){
    Log::result << "Test summary: " << result << " FAILED suites\n";
  }else{
    Log::result << "Test summary: Ok\n";
  }
  Log::msg << "  Total number of test suites:        " << the_tests.size() << "\n";
  Log::msg << "  Total number of failed inner tests: " << total_inner_failures << "/" << total_inner_test_count << "\n";

  return result;
}

bool Test::inner_test(std::string testname, bool success){
  ++total_inner_test_count;
  if(success){
    Log::debug << "  " << testname << ": Ok\n";
  }else{
    Log::debug << "  " << testname << ": FAILURE\n";
    ++inner_failures;
    ++total_inner_failures;
  }
  return success;
};
