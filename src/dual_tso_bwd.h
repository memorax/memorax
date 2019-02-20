/*
 * Copyright (C) 2018 Tuan Phong Ngo
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

#ifndef __DUAL_TSO_BWD_H__
#define __DUAL_TSO_BWD_H__

#include "reachability.h"
#include "exact_bwd.h"
#include "trace.h"
#include "dual_constraint.h"
#include "dual_channel_bwd.h"

/* This class is a wrapper for Bwd when used for DualConstraints. It
 * executes exactly as Bwd, but converts the Dual trace into a TSO trace
 * before returning the result.
 */

class DualTsoBwd : public DualChannelBwd{
public:
  static void test();
protected:
  virtual Trace *convert_trace(Trace *trace,DualChannelConstraint::Common *common) const;
private:
  /* Returns true iff executing the statement would insert a message in the SB
   * channel.
   */
  virtual bool produces_message(const Lang::Stmt<int>&) const;
  /* Returns true iff executing the statement would advance the cpointer of the
   *  executing process.
   */
  virtual bool consumes_message(const Lang::Stmt<int>&) const;
};

#endif
