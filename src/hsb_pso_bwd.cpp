/*
 * Copyright (C) 2013 Magnus Lång
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

#include "hsb_pso_bwd.h"
#include "hsb_constraint.h"

Trace *HsbPsoBwd::convert_trace(Trace *trace, ChannelConstraint::Common *cmn) const {
  HsbConstraint::Common &common = dynamic_cast<HsbConstraint::Common&>(*cmn);

 Log::extreme << " *** HSB trace ***\n";
 trace->print(Log::extreme,Log::extreme,Log::json,common.machine);
 Log::extreme << "\n\n";

 typedef std::pair<int, int> p;

  /* Find values lost in the buffer which we convert to messages lost in the
   * channel, which ChannelBwd::convert_trace then can complete the repairs of
   * the trace on. */
  std::unique_ptr<Trace> temp(new Trace(dynamic_cast<const ChannelConstraint*>(trace->constraint(0))->clone()));
  /* each element in lost_values[pid, nmli][*] corresponds to an element in process
   * pid's buffer to memory location nmli and contains the values of all writes lost */
  std::map<std::pair<int, int>, std::list<std::vector<ZStar<int> > > > lost_values;
  for (int trace_pos = 1; trace_pos <= trace->size(); ++trace_pos) {
    const HsbConstraint &hsbc1 = dynamic_cast<const HsbConstraint&>(*trace->constraint(trace_pos-1));
    const HsbConstraint &hsbc2 = dynamic_cast<const HsbConstraint&>(*trace->constraint(trace_pos));
    const Machine::PTransition *trans = trace->transition(trace_pos);
    const Lang::Stmt<int> &s = trans->instruction;
    int pid = trans->pid;
    switch (s.get_type()) {
    case Lang::WRITE: {
      assert(s.get_writes().size() == 1);
      Lang::NML nml(s.get_writes()[0], pid);
      int nmli = common.index(nml);
      if (hsbc1.write_buffers[pid][nmli].size() + 1 != hsbc2.write_buffers[pid][nmli].size()) {
        /* Value lost in the buffer. */
        assert(hsbc1.write_buffers[pid][nmli].size() == hsbc2.write_buffers[pid][nmli].size());
        lost_values[p(pid, nmli)].back().push_back(hsbc1.write_buffers[pid][nmli].back());
      } else {
        /* No values lost in the buffer. */
        lost_values[p(pid, nmli)].push_back({});
      }
    } break;
    case Lang::SERIALISE: {
      assert(s.get_writes().size() == 1);
      Lang::NML nml(s.get_writes()[0], pid);
      int nmli = common.index(nml);
      assert(hsbc1.write_buffers[pid][nmli].size() == hsbc2.write_buffers[pid][nmli].size() + 1);
      for (ZStar<int> lost_value : lost_values[p(pid, nmli)].front()) {
        std::unique_ptr<HsbConstraint> clone(hsbc2.clone());
        clone->channel.back().store = clone->channel.back().store.assign(nmli, lost_value);
        temp->push_back(*trans, clone.release());
      }
      lost_values[p(pid, nmli)].pop_front();
    } break;
    default:
      assert(hsbc1.write_buffers == hsbc2.write_buffers);
    }
    temp->push_back(*trans, hsbc2.clone());
  }

  Log::extreme << " *** HSB trace (no buffer value loss) ***\n";
  temp->print(Log::extreme,Log::extreme,Log::json,common.machine);
  Log::extreme << "\n\n";

  temp = std::unique_ptr<Trace>(ChannelBwd::convert_trace(temp.get(), cmn));
  // Filter out serialise transitions
  Trace *result = new Trace(0);
  for (int i = 1; i <= temp->size(); ++i) {
    if (temp->transition(i)->instruction.get_type() != Lang::SERIALISE)
      result->push_back(*temp->transition(i), 0);
  }

  Log::extreme << " *** PSO trace ***\n";
  result->print(Log::extreme,Log::extreme,Log::json,common.machine);
  Log::extreme << "\n";

  return result;
}

bool HsbPsoBwd::produces_message(const Lang::Stmt<int> &s) const{
  assert(s.get_writes().size() == 0    ||
         s.get_type() == Lang::UPDATE  ||
         s.get_type() == Lang::WRITE   ||
         s.get_type() == Lang::LOCKED  ||
         s.get_type() == Lang::SLOCKED ||
         s.get_type() == Lang::SERIALISE);
  return (s.get_writes().size() > 0 &&
          s.get_type() != Lang::UPDATE &&
          s.get_type() != Lang::WRITE);
}

bool HsbPsoBwd::consumes_message(const Lang::Stmt<int> &s) const{
  return (s.get_type() == Lang::UPDATE ||
          (s.get_type() == Lang::LOCKED && s.get_writes().size() > 0));
}
