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

#ifndef __TSO_CYCLE_H__
#define __TSO_CYCLE_H__

#include "machine.h"
#include <vector>

/* A TsoCycle represents a critical cycle (Shasha, Snir, 88), or a
 * sequence which is a fragment of a critical cycle.
 * 
 * TsoCycle is machine-oblivious. This means that our definition of
 * "critical cycle" and "critical cycle fragment" does not involve the
 * program order.
 *
 * Tsocycle also does not consider actual reorderability. Any
 * conflict/process-cycle of at least three instructions is reported
 * as a cycle.
 *
 * By "critical cycle" we mean a sequence seq of transitions such that
 * there is a way to construct a parallel program P, such that for
 * each transition (_,instr,_) of process pid in seq there is a
 * transition (_,instr,_) for process pid in P, and seq is a critical
 * cycle in P. Furthermore there is at least one process pid in seq
 * that has two consecutive transitions in order (_,i0,_) and (_,i1,_)
 * where i0 is a non-fence containing a read and i1 is an update to a
 * variable that is not read by i0.
 *
 * By "critical cycle fragment" we mean a sequence seq such that there
 * exists a sequence seq' such that the concatenation seq.seq' is a
 * critical cycle.
 *
 * By "cycle" we mean "critical cycle". By "cycle fragment" we mean
 * "critical cycle fragment".
 *
 * By "variable" we mean memory location.
 *
 * The Shasha Snir 88 definition of critical cycles is interpreted in
 * the following way, to accomodate TSO:
 *
 * ReadAssert, ReadAssign and locked statements containing ReadAssert
 * or ReadAssign are considered reads.
 *
 * Locked Write and Update are considered writes.
 *
 * Note that non-locked Write is *not* considered a write, since it
 * does not update the memory.
 */
class TsoCycle{
public:
  /* Constructs an empty fragment of a cycle containing at most
   * proc_count processes, all with pid in the interval [0,proc_count).
   */
  TsoCycle(int proc_count);
  /* Adds pt at the end of this cycle fragment.
   *
   * Does *not* take ownership of pt.
   *
   * Pre: can_push_back(pt)
   */
  void push_back(const Machine::PTransition *pt);
  /* Adds pt at the beginning of this cycle fragment.
   *
   * Does *not* take ownership of pt.
   *
   * Pre: can_push_front(pt)
   */
  void push_front(const Machine::PTransition *pt);
  /* Returns true iff this->push_back(pt) would return true. */
  bool can_push_back(const Machine::PTransition *pt) const;
  /* Returns true iff this->push_front(pt) would return true. */
  bool can_push_front(const Machine::PTransition *pt) const;
  /* Returns true iff this object is a valid cycle. */
  bool is_complete() const { return complete; };
  /* Returns all pairs (a,b) of consecutive transitions a,b where
   * a->pid == b->pid */
  std::list<std::pair<const Machine::PTransition *,const Machine::PTransition*> > get_critical_pairs() const;
  /* Returns the i:th transition in this fragment. */
  const Machine::PTransition *operator[](const int &i) const { return frag[i]; };
  /* Returns the number of transitions in this fragment */
  int size() const { return frag_len; };
  std::string to_string(const Machine &m) const;
  std::string to_string(const std::function<std::string(const int&)> &regts, 
                        const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const;

  /* Defines a total order over TsoCycles 
   * 
   * Returns -1 if this TsoCycle is less than tc,
   * 0 if this TsoCycle is identical to tc,
   * 1 if this TsoCycle is greater than tc. */
  int compare(const TsoCycle &tc) const throw();
  bool operator==(const TsoCycle &tc) const throw() { return compare(tc) == 0; };
  bool operator!=(const TsoCycle &tc) const throw() { return compare(tc) != 0; };
  bool operator<(const TsoCycle &tc) const throw() { return compare(tc) < 0; };
  bool operator>(const TsoCycle &tc) const throw() { return compare(tc) > 0; };
private:
  /* Limit on the number of processes that may participate in the cycle.
   *
   * Only processes with pid in the interval 0 to (proc_count-1) may
   * participate.
   */
  int proc_count;
  /* The length of this cycle fragment. */
  int frag_len;
  /* true iff this cycle fragment is a complete critical cycle */
  bool complete;
  /* The cycle fragment is contained in the prefix of frag of length frag_len.
   *
   * Invariant:
   * frag.size() == proc_count*2
   * For all i>=frag_len . frag[i] == 0
   * For all 1 <= i < frag_len . connects(frag[i-1],frag[i])
   * For all 0 <= i < frag_len . frag[i] accesses some variable
   */
  std::vector<const Machine::PTransition*> frag;

  /* Checks whether either 
   *
   * 1) a->pid == b->pid
   *
   * or
   *
   * 2) There is a variable that is accessed by both a and b, and
   * written by at least one of them.
   */
  static bool connects(const Machine::PTransition *a, const Machine::PTransition *b);

  /* Pre: Both a and b are sorted and distinct.
   *
   * Post: true iff there is some element that occurs both in a and b.
   */
  static bool overlaps(const std::vector<Lang::MemLoc<int> > &a, int a_pid,
                       const std::vector<Lang::MemLoc<int> > &b, int b_pid);

  /* Checks whether the cycle is complete. Assumes that this is a
   * valid cycle fragment.
   */
  bool compute_is_complete() const{
    return frag_len > 2 && connects(frag[0],frag[frag_len-1]);
  };

  /* Returns true iff pt->instruction accesses memory. Note that
   * non-locked write is not considered as memory accessing, since it
   * only writes to the store buffer.
   */
  static bool mem_access(const Machine::PTransition *pt){
    return (pt->instruction.get_reads().size() > 0 || pt->instruction.get_writes().size() > 0) &&
      pt->instruction.get_type() != Lang::WRITE;
  };
  
};

#endif
