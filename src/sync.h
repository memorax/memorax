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

#ifndef __SYNC_H__
#define __SYNC_H__

#include "machine.h"
#include "trace.h"

/* Sync is an abstract class representing a piece of synchronization
 * that can be inserted into a machine. Derived classes could
 * implement particular pieces of synchronization, e.g. a fence at a
 * given position.
 */
class Sync{
public:
  virtual ~Sync() {};
  virtual Machine *insert(const Machine &m) const = 0;
  virtual bool prevents(const Trace &t) const = 0;
  virtual std::string to_raw_string() const = 0;
  virtual std::string to_string() const { return to_raw_string(); };
  virtual std::string to_string(const Machine &m) const { return to_string(); };
};

#endif
