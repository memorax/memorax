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

#ifndef __FENCE_SYNC_H__
#define __FENCE_SYNC_H__

#include "sync.h"

#include <functional>
#include <set>

/* A FenceSync is an abstract class that represents a fence
 * instruction to be inserted at a particular position in a Machine.
 *
 * FenceSync should be inherited by implementations targeting specific
 * memory models.
 *
 * A FenceSync is a tuple: (f,pid,q,IN,OUT), where f is the fence
 * instruction, and pid is the process where the fence should be
 * inserted. The natural number q is a control state. IN and OUT are
 * subsets (non-strict) of respectively the transitions leading to q
 * and the transitions originating in q. The fence should be inserted
 * such that whenever control flow goes through transitions i in IN
 * and then j in OUT, then control flow should go through f between i
 * and j.
 *
 * Invariant: IN and OUT are non-empty.
 */
class FenceSync : public Sync{
public:
  /* Construct the FenceSync (f,pid,q,IN,OUT).
   *
   * (See above.)
   */
  FenceSync(Lang::Stmt<int> f, int pid, int q, 
            std::set<Automaton::Transition> IN, 
            std::set<Automaton::Transition> OUT);
  virtual ~FenceSync() {};
  virtual Machine *insert(const Machine &m) const;
  virtual bool prevents(const Trace &t) const = 0;
  virtual std::string to_raw_string() const;
  virtual std::string to_string(const Machine &m) const;
  static void test();
protected:
  /* Returns a set S of Sync objects, such that for each FenceSync
   * object (f,pid,q,IN,OUT) corresponding to some fence f in fs that
   * can be inserted into m, the object fsinit(f,pid,q,IN,OUT) is in
   * S.
   *
   * Hint: Use from deriving concrete class FS as follows:
   * get_all_possible(m,fs,[](f,pid,q,IN,OUT){ return new FS(f,pid,q,IN,OUT); })
   */
  typedef std::function<Sync*(Lang::Stmt<int> f, int pid, int q, 
                              std::set<Automaton::Transition> IN, 
                              std::set<Automaton::Transition> OUT)> fs_init_t;
  static std::set<Sync*> get_all_possible(const Machine &m,
                                          const std::set<Lang::Stmt<int> > &fs,
                                          const fs_init_t &fsinit);
private:
  /* This FenceSync is (f,pid,q,IN,OUT) as described at the main
   * description of FenceSync at the top of this file. */
  Lang::Stmt<int> f;
  int pid;
  int q;
  std::set<Automaton::Transition> IN;
  std::set<Automaton::Transition> OUT;

  std::string to_string_aux(const std::function<std::string(const int&)> &regts, 
                            const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const;

  /* Returns the power set of s. */
  template<class T>
  static std::set<std::set<T> > powerset(const std::set<T> &s);
};

#endif
