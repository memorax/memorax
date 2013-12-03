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

#include "log.h"
#include "machine.h"
#include "sync.h"

#include <set>

namespace SyncSetPrinter{

  /* Prints a human-readable representation of S, interpreted as a set
   * of solutions provided by Fencins. All Syncs in S should be for
   * the machine m. Human-readable output is printed to os. Json
   * annotation is printed to json_os.
   */
  void print(const std::set<std::set<Sync*> > &S,
             const Machine &m,
             Log::redirection_stream &os,
             Log::redirection_stream &json_os);
};
