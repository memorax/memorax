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

#ifndef __CHANNEL_BWD_H__
#define __CHANNEL_BWD_H__

#include "reachability.h"
#include "exact_bwd.h"
#include "trace.h"
#include "sb_constraint.h"

/* This class is a wrapper for Bwd when used for ChannelConstraints. It executes
 * exactly as Bwd, but converts the trace into a channel-less trace by
 * sectioning the trace of each process by transitions that advance the
 * cpointer, as defined by consumes_message, and then for each section, adds the
 * transitions of every process in that section to the new trace, and between
 * each such section, it adds the cpointer-advancing transition of the process
 * that generated the message. The other cpointer-advancing transitions are
 * skipped.
 */

class ChannelBwd : public Reachability{
public:
  /* Pre: arg should be of type Bwd::Arg. */
  virtual Result *reachability(Arg *arg) const{
    ExactBwd bwd;
    Result *res = bwd.reachability(arg);
    if(res->result == Reachability::REACHABLE){
      assert(res->trace);
      assert(dynamic_cast<ExactBwd::Result*>(res));
      assert(static_cast<ExactBwd::Result*>(res)->common);
      assert(dynamic_cast<ChannelConstraint::Common*>(static_cast<ExactBwd::Result*>(res)->common));
      Trace *old_trace = res->trace;
      res->trace = convert_trace(old_trace,
                                 static_cast<ChannelConstraint::Common*>(static_cast<ExactBwd::Result*>(res)->common));
      delete old_trace;
    }
    return res;
  };

protected:
  virtual Trace *convert_trace(Trace *trace,ChannelConstraint::Common *common) const;
private:
  /* Returns true iff executing the statement would insert a message in the SB
   * channel.
   */
  virtual bool produces_message(const Lang::Stmt<int>&) const = 0;
  /* Returns true iff executing the statement would advance the cpointer of the
   *  executing process.
   */
  virtual bool consumes_message(const Lang::Stmt<int>&) const = 0;
  /* Helper to convert_trace.
   *
   * ch0 should be the channel of the pre, and ch1 the channel of the
   * post to a write. w should be the index of that write in the
   * trace. ch should correspond to ch0.
   *
   * Afterwards, ch will correspond to ch1.
   */
  void messages_lost(const std::vector<ChannelConstraint::Msg> &ch0,
                     const std::vector<ChannelConstraint::Msg> &ch1,
                     std::vector<int> *ch,
                     int w,
                     const ChannelConstraint::Common *common) const;
};

#endif
