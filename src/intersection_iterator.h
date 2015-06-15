/*
 * Copyright (C) 2013
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

#ifndef __INTERSECTION_ITERATOR_H__
#define __INTERSECTION_ITERATOR_H__

#include <vector>
/* Represents a intersection of two sorted sets which can only be iterated.
 */
template<class SortedSet,
         class T =              typename SortedSet::T,
         class inner_iterator = typename SortedSet::const_iterator>
class Intersection {
public:
  Intersection(SortedSet A, SortedSet B) : A(A), B(B) {};

  /* Invariants:
   *  Either current_A == A_end AND current_B == B_end
   *      or current_A != A_end AND current_B != B_end AND *current_A == *current_B
   */
  class const_iterator {
  public:
    const_iterator &operator++() {
      if (current_A == A_end) return *this; // We are at the end
      ++current_A;
      ++current_B;
      find_next();
      return *this;
    }
    const_iterator operator++(int) {
      const_iterator old = *this;
      ++this;
      return old;
    }
    bool operator ==(const const_iterator &other) const {
      assert(A_end == other.A_end && B_end == other.B_end); // Ensure it's the same sets
      assert(!((other.current_A == current_A) ^ (other.current_B == current_B))); // Invariant      
      return other.current_A == current_A;
    }
    bool operator !=(const const_iterator &other) const { return !(*this == other); }
    const T &operator *() const { return *current_A; }

  private:
    void find_next() {
      while (true) {
        if (current_A == A_end) { while(current_B != B_end) ++current_B; return; }
        if (current_B == B_end) { while(current_A != A_end) ++current_A; return; }
        if      (*current_B < *current_A) ++current_B;
        else if (*current_A < *current_B) ++current_A;
        else                              break;
      }
    }
    inner_iterator current_A, current_B, A_end, B_end;
    const_iterator(inner_iterator A_begin,
                   inner_iterator B_begin,
                   inner_iterator A_end,
                   inner_iterator B_end) :
      current_A(A_begin), current_B(B_begin), A_end(A_end), B_end(B_end) {
      find_next();
    }
    const_iterator(inner_iterator A_end, inner_iterator B_end) :
      current_A(A_end), current_B(B_end), A_end(A_end), B_end(B_end) {}
    friend class Intersection;
  };

  const_iterator begin() { return const_iterator(A.begin(), B.begin(), A.end(), B.end()); }
  const_iterator end() { return const_iterator(A.end(), B.end()); }

private:
  SortedSet A, B;
};

#endif
