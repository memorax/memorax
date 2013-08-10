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

#ifndef __PWS_CONTAINER__
#define __PWS_CONTAINER__

#include "channel_container.h"
#include "pws_constraint.h"

class PwsContainer : public ChannelContainer {
public:
  PwsContainer() : ChannelContainer() {};
  virtual void clear();
protected:
  virtual std::vector<CWrapper*> &get_F_set(CWrapper *);
  virtual void visit_F(std::function<void(std::vector<CWrapper*>&)>);
private:
/* F[pcs][chr][bx] maps to the set of all constraints in F that have program
 * counters pcs, channel characterization chr, and filled buffers bx
 */
std::map<std::vector<int>,
         std::map<std::vector<ChannelConstraint::MsgCharacterization>,
                  std::map<std::vector<std::pair<int, Lang::NML> >,
                           std::vector<CWrapper *> > > > F;

};

#endif
