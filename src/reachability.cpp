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

#include "reachability.h"

Reachability::Timer::Timer(){
  running = false;
  offset = 0.0;
};

void Reachability::Timer::start(){
  if(!running){
    start_time = time(0);
    start_time_c = clock();
    running = true;
  }
};

void Reachability::Timer::stop(){
  if(running){
    int stop_time = time(0);
    clock_t stop_time_c = clock();
    if(stop_time - start_time > 2){
      offset += stop_time - start_time;
    }else{
      offset += double(stop_time_c - start_time_c) / CLOCKS_PER_SEC;
    }
    running = false;
  }
};

void Reachability::Timer::reset(){
  offset = 0.0;
  start_time = time(0);
  start_time_c = clock();
};

double Reachability::Timer::get_time() const{
  if(running){
    int stop_time = time(0);
    clock_t stop_time_c = clock();
    if(stop_time - start_time > 2){
      return offset + double(stop_time - start_time);
    }else{
      return offset + double(stop_time_c - start_time_c) / CLOCKS_PER_SEC;
    }
  }else{
    return offset;
  }
};

bool Reachability::Timer::is_running() const {
  return running;
};

Reachability::Result::Result(const Machine &m)
  : result(FAILURE), trace(0), generated_constraints(0), stored_constraints(0), machine(m){
};

std::string Reachability::Result::to_string() const{
  std::stringstream ss;
  std::string reach;
  switch(result){
  case REACHABLE: reach = "Yes"; break;
  case UNREACHABLE: reach = "No"; break;
  case FAILURE: reach = "(Failure)"; break;
  }
  ss << "Reachability analysis results:\n" 
     << "  Reachable:             " << reach << "\n"
     << "  Generated constraints: " << generated_constraints << "\n"
     << "  Size of visited set:   " << stored_constraints << "\n"
     << "  Time consumption:      " << timer.get_time() << " s\n";
  return ss.str();
};

