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

#include "timer.h"
#include <sys/time.h>
#include <cassert>

class Timer::Timer_impl{
public:
  Timer_impl(){
    running = false;
    offset = 0.0;
  };
  Timer_impl(const Timer_impl &) = default;
  Timer_impl &operator=(const Timer_impl &) = default;
  void start(){
    if(!running){
#ifndef NDEBUG
      int res = 
#endif
        gettimeofday(&start_time,0);
      assert(res == 0);
      running = true;
    }
  };
  void stop(){
    if(running){
      struct timeval stop_time;
#ifndef NDEBUG
      int res = 
#endif
        gettimeofday(&stop_time,0);
      assert(res == 0);
      offset += double(stop_time.tv_sec - start_time.tv_sec) + 
        double(stop_time.tv_usec - start_time.tv_usec)*1e-6;
      running = false;
    }
  };
  void reset(){
    offset = 0.0;
#ifndef NDEBUG
      int res = 
#endif
        gettimeofday(&start_time,0);
      assert(res == 0);
  };
  void add(double d){
    offset += d;
  };
  double get_time() const{
    if(running){
      struct timeval stop_time;
#ifndef NDEBUG
      int res = 
#endif
        gettimeofday(&stop_time,0);
      assert(res == 0);
      return offset + double(stop_time.tv_sec - start_time.tv_sec) + 
        double(stop_time.tv_usec - start_time.tv_usec)*1e-6;
    }else{
      return offset;
    }
  };
  bool is_running() const{
    return running;
  };
  struct timeval start_time; // Start time in seconds since the Epoch
  double offset; // The complete time consumption measured at the last stopping of the timer
  bool running; // Is the timer running?
};

Timer::Timer(){
  the_timer = new Timer_impl();
};

Timer::~Timer(){
  delete the_timer;
};

Timer::Timer(const Timer &tim){
  the_timer = new Timer_impl(*tim.the_timer);
};

Timer &Timer::operator=(const Timer &tim){
  if(this != &tim){
    delete the_timer;
    the_timer = new Timer_impl(*tim.the_timer);
  }
  return *this;
};

void Timer::start(){ the_timer->start(); };
void Timer::stop(){ the_timer->stop(); };
void Timer::reset(){ the_timer->reset(); };
void Timer::add(double d){ the_timer->add(d); };
double Timer::get_time() const { return the_timer->get_time(); };
bool Timer::is_running() const { return the_timer->is_running(); };
