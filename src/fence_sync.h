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
#include "trace.h"

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
 * TODO: Beskriv tillståndsuppdelning.
 *
 * TODO: Beskriv kombination och begränsningar på kombination.
 * - För alla två Syncs s0 och s1 gäller att s0.IN och s1.IN
 *   (resp. s0.OUT och s1.OUT) är antingen disjunkta eller relaterade
 *   genom delmängdsjämförelse.
 *
 * Invariant: TODO: Beskriv invariant. (En av IN och OUT är lika med
 * kontrolltillståndets IN resp. OUT.)
 */
class FenceSync : public Sync{
public:
  struct StmtCmp {
    bool operator()(const Lang::Stmt<int> &a, const Lang::Stmt<int> &b) const{
      return a.compare(b,false) < 0;
    };
  };
  struct TransCmp {
    bool operator()(const Automaton::Transition &a, const Automaton::Transition &b) const{
      return a.compare(b,false) < 0;
    };
  };
  typedef std::set<Automaton::Transition,TransCmp> TSet;
  typedef const std::vector<const Sync::InsInfo*> & m_infos_t;

  /* Construct the FenceSync (f,pid,q,IN,OUT).
   *
   * (See above.)
   */
  FenceSync(Lang::Stmt<int> f, int pid, int q, 
            TSet IN, TSet OUT);
  virtual ~FenceSync() {};

  class InsInfo : public Sync::InsInfo{
  public:
    InsInfo(const FenceSync *creator_copy)
    : Sync::InsInfo(creator_copy), fence(-1,Lang::Stmt<int>::nop(),-1,-1) {};
    InsInfo(const InsInfo &) = default;
    virtual InsInfo &operator=(const InsInfo &) = default;
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
    /* All control states in the new machine that correspond to
     * control state q in the original machine. I.e. the control
     * states into which q was split by this FenceSync and previous
     * FenceSyncs to the same control location.
     */
    std::set<int> new_qs;
    /* The fence transition inserted by this FenceSync. (Note that
     * this is the transition as it occurred immediately after this
     * insertion, and that the transition may later be changed
     * according to tchanges of InsInfos corresponding to later
     * insertions.)
     */
    Machine::PTransition fence;
    /* Insert a->b into tchanges. */
    void bind(const Machine::PTransition &a,const Machine::PTransition &b);
    /* Shorthand for tchanges[t]. */
    const Machine::PTransition &operator[](const Machine::PTransition &t) const;

    /* If ivec = [&i0,&i1,...,&in] then the returned transition is
     *
     * in[...ij[ik[t]]...] where k == first and j == first+1
     *
     * Pre: All elements in ik,ij,...,in are pointers to
     * FenceSync::InsInfo objects (or derivatives) or TsoLockSync.
     */
    static Machine::PTransition all_tchanges(const std::vector<const Sync::InsInfo*> &ivec,
                                             const Machine::PTransition &t,
                                             int first = 0);
    /* If q is a new control state that was created during the
     * insertion corresponding to any element of ivec, then the
     * control state qorg that was split into qorg and q in that
     * insertion is returned. Otherwise q is returned.
     */
    static int original_q(m_infos_t ivec, int q);

    /* Make tchanges an identity map with keys precisely the
     * transitions occurring in m.
     */
    void setup_id_tchanges(const Machine &m);
  };

  virtual Machine *insert(const Machine &m, m_infos_t m_infos, Sync::InsInfo **info) const;
  virtual Sync *clone() const = 0;
  virtual std::string to_raw_string() const;
  virtual std::string to_string(const Machine &m) const;
  /* Getters for the parts in (f,pid,q,IN,OUT) */
  Lang::Stmt<int> get_f() const { return f; };
  virtual int get_pid() const { return pid; };
  virtual int get_q() const { return q; };
  virtual const TSet &get_IN() const { return IN; };
  virtual const TSet &get_OUT() const { return OUT; };
  static void test();
protected:
  /* This FenceSync is (f,pid,q,IN,OUT) as described at the main
   * description of FenceSync at the top of this file. */
  Lang::Stmt<int> f;
  int pid;
  int q;
  TSet IN;
  TSet OUT;

  /* Returns a set S of Sync objects, such that for each FenceSync
   * object (f,pid,q,IN,OUT) corresponding to some fence f in fs that
   * can be inserted into m, the object fsinit(f,pid,q,IN,OUT) is in
   * S.
   *
   * Hint: Use from deriving concrete class FS as follows:
   * get_all_possible(m,fs,[](f,pid,q,IN,OUT){ return new FS(f,pid,q,IN,OUT); })
   */
  typedef std::function<Sync*(Lang::Stmt<int> f, int pid, int q, 
                              TSet IN, 
                              TSet OUT)> fs_init_t;
  static std::set<Sync*> get_all_possible(const Machine &m,
                                          const std::set<Lang::Stmt<int> > &fs,
                                          const fs_init_t &fsinit);

  virtual int compare(const Sync &s) const;
private:
  /* Returns true iff IN (and OUT) is subset related to fs.IN
   * (resp. fs.OUT) */
  bool subset_related(const FenceSync &fs) const;
  /* Returns true iff this FenceSync and fs may coexist in the same
   * machine. I.e.:
   *
   * If both FenceSyncs are for the same control location and process,
   * then IN (and OUT) and fs.IN (resp. fs.OUT) are either disjunct or
   * subset related.
   */
  bool compatible(const FenceSync &fs) const;
  /* Returns true iff S is a subset of T. */
  static bool subset(const TSet &S, const TSet &T);
  /* Returns true iff S and T are subset related. */
  static bool subset_related(const TSet &S, const TSet &T);
  /* Returns true iff S and T are disjunct. */
  static bool disjunct(const TSet &S, const TSet &T);
  /* Returns S\T */
  static TSet set_minus(const TSet &S, const TSet &T);

  std::string to_string_aux(const std::function<std::string(const int&)> &regts, 
                            const std::function<std::string(const Lang::MemLoc<int> &)> &mlts) const;

  /* Returns the power set of s. */
  template<class T,class L = std::less<T> >
  static std::set<std::set<T,L> > powerset(const std::set<T,L> &s);

  /* Checks if this FenceSync is correct with respect to m.
   * I.e. that process pid exists in m, and has a control state q.
   * IN is a (non-strict) subset of the transitions targeting q, and
   * OUT is a (non-strict) subset of the transitions originating in q.
   *
   * m is assumed to be the result of applying the changes described
   * in m_infos to to the Machine that this FenceSync was created for.
   */
  bool applies_to(const Machine &m,m_infos_t m_infos) const;

  /* Get a pair (I,O) where I (and O) is the set of all incoming
   * (resp. outgoing) transitions for q in m, as they occur in the
   * original machine.
   */
  std::pair<TSet,TSet> get_orig_q_IN_OUT(const Machine &m,
                                         m_infos_t m_infos) const;

  /* Get a pair (I,O) where I (and O) is the set of all incoming
   * (resp. outgoing) transitions for q in m, as they occur in m.
   */
  std::pair<TSet,TSet> get_m_q_IN_OUT(const Machine &m,
                                      m_infos_t m_infos) const;
};

#endif
