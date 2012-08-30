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

#ifndef __TICKET_QUEUE_H__
#define __TICKET_QUEUE_H__

#include <vector>
#include <cassert>
#include <algorithm>

/* A ticket queue is a FIFO queue where each element is assigned a
 * unique ticket number. Ticket numbers increase at each element
 * pushed. Elements that have been pushed, but not yet popped can be
 * accessed in constant time by ticket number. Pushing and popping are
 * done in amortized constant time.
 *
 * Implemented as a vector.
 */
template<class T> class TicketQueue{
public:
  /* Creates an empty ticket queue with original capacity bufsize. The
   * capacity will be automatically extended as necessary.
   */
  TicketQueue(long bufsize = 1000) 
  : first(0), sz(0), offset(0) {
    buf.resize(std::max(bufsize,1L));
  };
  /* Returns the number of elements in the queue. */
  long size() const { return sz; };
  /* Pushes c onto the end of the queue. Returns the ticket number
   * assigned to c. */
  long push(T c){
    if(first+sz >= long(buf.size())){
      assert(first+sz == long(buf.size()));
      buf.resize(buf.size()*2);
      offset += first;
      for(long i = 0; i < sz; i++){
        buf[i] = buf[i+first];
      }
      first = 0;
    }
    buf[first+sz] = c;
    sz++;
    return offset+first+sz-1;
  };
  /* Returns the first element in the queue, and removes it from the
   * queue.
   */
  T pop(){
    if(sz == 0){
      throw new std::logic_error("TicketQueue::pop: empty");
    }
    first++;
    sz--;
    return buf[first-1];
  };
  /* Returns a reference to the element with ticket tck.
   *
   * Pre: The element with ticket tck is still in the queue.
   */
  T &at(long tck){
    assert(in_queue(tck));
    return buf[tck - offset];
  };
  /* Returns true iff the element with ticket tck is still in the
   * queue.
   */
  bool in_queue(long tck){
    tck -= offset;
    return first <= tck && tck < first+sz;
  };
  /* Removes all elements from the queue. */
  void clear(){
    offset = offset+first+sz; // The next ticket number
    first = 0;
    sz = 0;
  };
private:
  /* The queue is in buf from and including index first, to but not
   * including index first+sz. Elements closer to the front of buf are
   * older than elements closer to the back.
   */
  std::vector<T> buf;
  /* Index in buf of the first element in the queue */
  long first;
  /* The number of elements in the queue */
  long sz;
  /* The ticket number of the element at index i in buf is
   * offset+i. */
  long offset;
};

#endif
