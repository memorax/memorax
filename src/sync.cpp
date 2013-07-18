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

#include "sync.h"

#include <typeinfo>

bool Sync::operator<(const Sync &s) const{
  const std::type_info &my_type = typeid(*this);
  const std::type_info &s_type = typeid(s);
  if(my_type.before(s_type)){
    return true;
  }
  if(s_type.before(my_type)){
    return false;
  }
  return compare(s) < 0;
};

bool Sync::operator==(const Sync &s) const{
  return typeid(*this) == typeid(s) && compare(s) == 0;
};
