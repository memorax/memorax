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

#include "pws_container.h"

void PwsContainer::clear() {
  ChannelContainer::clear();
  F.clear();
};

std::vector<ChannelContainer::CWrapper*> &PwsContainer::get_F_set(CWrapper *cw) {
  return F[cw->sbc->get_control_states()][cw->sbc->characterize_channel()]
    [static_cast<PwsConstraint*>(cw->sbc)->get_filled_buffers()];
}

void PwsContainer::visit_F(std::function<void(std::vector<CWrapper*>&)> f) {
  for(auto FPerPcs : F) {
    for (auto FPerChar : FPerPcs.second) {
      for (auto subset : FPerChar.second) {
        f(subset.second);
      }
    }
  }
}
