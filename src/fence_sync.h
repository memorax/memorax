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

  class InsInfo : public Sync::InsInfo{
  public:
    InsInfo(const FenceSync *creator_copy) : Sync::InsInfo(creator_copy) {};
    virtual ~InsInfo(){};
    /* When a FenceSync is inserted into a Machine m, creating the
     * Machine m', for each transition t in m, tchanges[t] is that
     * transition as it occurs in tchanges.
     *
     * This means that some transitions will map to themselves, while
     * others on the form (q0,i,q1) will map to (q0',i,q1') where
     * either q0' != q0 or q1' != q1.
     *
     * tchanges[t] is defined for all transitions t in m.
     */
    std::map<Machine::PTransition,Machine::PTransition> tchanges;
    /* Insert a->b into tchanges. */
    void bind(const Machine::PTransition &a,const Machine::PTransition &b);
    /* Shorthand for tchanges[t]. */
    const Machine::PTransition &operator[](const Machine::PTransition &t) const;

    /* If ivec = [&a,&b,...,&z] then the returned transition is
     *
     * z[...b[a[t]]...]
     *
     * Pre: All elements in ivec are pointers to FenceSync::InsInfo
     * objects (or derivatives).
     */
    static Machine::PTransition all_tchanges(const std::vector<const Sync::InsInfo*> &ivec,
                                             const Machine::PTransition &t);
  };

  virtual Machine *insert(const Machine &m, const std::vector<const Sync::InsInfo*> &m_infos, Sync::InsInfo **info) const;
  virtual bool prevents(const Trace &t, const std::vector<const Sync::InsInfo*> &m_infos) const = 0;
  /* Return a deep copy of this object. */
  virtual FenceSync *clone() const = 0;
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

  /* Checks if this FenceSync is correct with respect to m.
   * I.e. that process pid exists in m, and has a control state q.
   * IN is a (non-strict) subset of the transitions targeting q, and
   * OUT is a (non-strict) subset of the transitions originating in q.
   *
   * m is assumed to be the result of applying the changes described
   * in m_infos to to the Machine that this FenceSync was created for.
   */
  bool applies_to(const Machine &m,const std::vector<const Sync::InsInfo*> &m_infos) const;
};

#endif
