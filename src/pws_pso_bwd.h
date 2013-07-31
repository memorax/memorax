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

#ifndef __PWS_PSO_BWD_H__
#define __PWS_PSO_BWD_H__

#include "sb_tso_bwd.h"
#include "pws_constraint.h"

/* This class is a wrapper for Bwd when used for PwsConstraints. It executes
 * exactly as Bwd, but converts the PWS trace into a PSO trace before returning
 * the result.
 * 
 * It is implemented on top of SbTsoBwd because the brunt of the work
 * (associating transitions that produce messages with their corresponding
 * updates and then reordering the process interleaving of the trace so it
 * becomes a TSO/PSO trace) is the same for SB and PWS traces, and the only
 * additional work needed for PWS is to filter out serialise transitions.
 */

class PwsPsoBwd : public SbTsoBwd {
protected:
  virtual Trace *convert_trace(Trace *trace, SbConstraint::Common *common) const;
private:
  virtual bool produces_message(const Lang::Stmt<int>&) const;
};

#endif
