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

#ifndef __DUAL_CHANNEL_BWD_H__
#define __DUAL_CHANNEL_BWD_H__

#include "reachability.h"
#include "exact_bwd.h"
#include "trace.h"
#include "dual_constraint.h"

/* This class is a wrapper for Bwd when used for DualChannelConstraints. It executes
 * exactly as Bwd, but converts the trace into a channel-less trace by
 * sectioning the trace of each process by transitions that advance the
 * cpointer, as defined by consumes_message, and then for each section, adds the
 * transitions of every process in that section to the new trace, and between
 * each such section, it adds the cpointer-advancing transition of the process
 * that generated the message. The other cpointer-advancing transitions are
 * skipped.
 */

class DualChannelBwd : public Reachability{
public:
  /* Pre: arg should be of type Bwd::Arg. */
  virtual Result *reachability(Arg *arg) const{
    ExactBwd bwd;
    Result *res = bwd.reachability(arg);
    if(res->result == Reachability::REACHABLE){
      assert(res->trace);
      assert(dynamic_cast<ExactBwd::Result*>(res));
      assert(static_cast<ExactBwd::Result*>(res)->common);
      assert(dynamic_cast<DualChannelConstraint::Common*>(static_cast<ExactBwd::Result*>(res)->common));
      Trace *old_trace = res->trace;
      res->trace = convert_trace(old_trace,
                                 static_cast<DualChannelConstraint::Common*>(static_cast<ExactBwd::Result*>(res)->common));
      //delete old_trace; // uncomment if we implement convert_trace function
    }
    return res;
  };

protected:
  virtual Trace *convert_trace(Trace *trace,DualChannelConstraint::Common *common) const;
private:
  /* Returns true iff executing the statement would insert a message in the SB
   * channel.
   */
  virtual bool produces_message(const Lang::Stmt<int>&) const = 0;
  /* Returns true iff executing the statement would advance the cpointer of the
   *  executing process.
   */
  virtual bool consumes_message(const Lang::Stmt<int>&) const = 0;
};

#endif
