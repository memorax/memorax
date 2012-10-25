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

#ifndef __TIMER_H__
#define __TIMER_H__


/* A stop-watch-like timer.
 * Used to measure the time consumption of reachability analysis.
 */
class Timer{
public:
  Timer();
  virtual ~Timer();
  Timer(const Timer &);
  Timer &operator=(const Timer&);
  /* Start the timer 
   * If the timer is already running, do nothing */
  virtual void start();
  /* Stop the timer
   * If the timer is not running, do nothing */
  virtual void stop();
  /* Set the timer to 0.0 */
  virtual void reset();
  /* Add d seconds to the current time of the timer. */
  virtual void add(double d);
  /* Get the current time of the timer */
  virtual double get_time() const;
  /* Is the timer running? */
  virtual bool is_running() const;
private:
  class Timer_impl;
  Timer_impl *the_timer;
};


#endif
