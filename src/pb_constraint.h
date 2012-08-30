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

#ifndef __PB_CONSTRAINT__
#define __PB_CONSTRAINT__

#include "constraint.h"
#include "machine.h"
#include "predicates.h"
#include "ap_list.h"
#include "sharinglist.h"
#include "tso_var.h"
#include "tso_cycle_lock.h"

class PbConstraint : public Constraint{
public:
  typedef Predicates::Term<TsoVar> Term;
  typedef Predicates::Predicate<TsoVar> Predicate;
  typedef Predicates::AppliedPredicate<TsoVar> AppliedPredicate;
  typedef APList<TsoVar>::pred_set pred_set;
  class Common : public Constraint::Common{
  public:
    /* Note: Common takes ownership of all predicates in preds. */
    Common(int k, const Machine &m, pred_set preds, bool auto_abstract);
    ~Common();
    const Machine &machine;
    /* The predicates which are allowed in ap_set when the constraint is
     * abstracted.
     */
    pred_set predicates;
    /* The maximum number of messages to keep in the channel for each
     * combination (writer,location). */
    const int k;
    /* last_msg[p][q] is a vector containing all the possible messages
     * which can be the rightmost message in the channel of process p
     * when process p is at control state q.
     */
    std::vector<std::vector<std::vector<Lang::MemLoc<int> > > > last_msg;
    /* If this flag is set, then constraints returned from pre or post
     * will be abstracted by default.
     */
    bool auto_abstract;
    struct AbstractionResult{
      std::list<AppliedPredicate> abstract;
      bool consistent;
    };
    /* If l is a key in abstraction_cache then the corresponding value
     * is returned. Otherwise an AbstractionResult ar is calculated
     * and returned, and (l maps to ar) is inserted into
     * abstraction_cache and abstraction_cache_predicates. This will
     * not claim ownership of any predicates. Copies will be made.
     *
     * Pre: l is sorted.
     */
    AbstractionResult abstract(const std::list<AppliedPredicate> &l);
    bool predicate_is_abstract(const Predicate *p) const throw(){
      for(unsigned i = 0; i < predicates.size(); i++){
        if(predicates[i] == p) return true;
      }
      return false;
    };
    bool ap_is_abstract(const AppliedPredicate &ap) const throw() { return predicate_is_abstract(ap.get_predicate()); };
    /* An applied predicate which states that each variable has the
     * value required at the initial state of machine.
     */
    AppliedPredicate is_init;
    std::string to_string() const throw();
    /* has_read[pid][pc] == true if there is a read transition
     * (ReadAssign or ReadAssert) in machine for process pid with
     * target pc
     *
     * Used by partred.
     */
    std::vector<std::vector<bool> > has_read;
    /* Maps nonlocked write transitions (pointer into
     * all_transitions) to transitions which are identical,
     * except that they are locked (also pointer into all_transitions).
     *
     */
    std::map<const Machine::PTransition*,const Machine::PTransition*> atomized_writes;
    /* Returns a pointer to the (copy of) transition t in this->all_transitions.
     *
     * If t does not occur in this->all_transitions then 0 is returned.
     */
    const Machine::PTransition *find_transition(const Machine::PTransition &t) const;
    /* Returns a list containing precisely the transitions t of
     * process p in the system such that t.target == pc where pc is
     * the program counter of the process p in pbc.
     */
    std::list<const Machine::PTransition*> all_enabled_by_pc(const PbConstraint *pbc) const;
  private:
    Common(const Common &);
    Common &operator=(const Common &);
    /* Stores all transitions that can be taken within the system. The
     * transitions can be identified by their pointers.
     *
     * The reason for keeping transitions here rather than just using
     * the ones in the machine is that the transitions which are
     * possible within a system depends on the memory
     * model. Specifically, for TSO all_transitions will contain
     * copies of all transitions in the machine, and also updates.
     */
    std::vector<Machine::PTransition> all_transitions;
    /* Initializes all_transitions. */
    void init_all_transitions();
    /* trans_to_pc[pid][pc] is a vector containing pointers to all
     * transitions t for process pid, in all_transitions with t.target
     * == pc.
     */
    std::vector<std::vector<std::vector<const Machine::PTransition*> > > trans_to_pc;
    /* Initializes trans_to_pc. 
     * Should be called after init_all_transitions() */
    void init_trans_to_pc();
    /* Maps lists of applied predicates to the result of their
     * abstraction.  The key list must be sorted. All non-abstract
     * predicates pointed to by some applied predicate in the key list
     * should occur in abstraction_cache_predicates, and nowhere else.
     */
    std::map<std::list<AppliedPredicate>, AbstractionResult> abstraction_cache;
    /* Contains the predicates which are owned by
     * abstraction_cache. They are the ones that have to be deleted.
     */
    std::list<Predicate*> abstraction_cache_predicates;
#ifndef NDEBUG
    /* Statistics about the usage of the abstraction cache */
    int abstraction_cache_calls;
    int abstraction_cache_hits;
#endif
    /* Used to initialize is_init. */
    static AppliedPredicate build_is_init(const Machine &machine);
  };
  PbConstraint(const std::vector<int> &pcs, Common &c);
  PbConstraint(const PbConstraint&);
  ~PbConstraint() throw();
  PbConstraint &operator=(const PbConstraint &);
  const std::vector<int> &get_control_states() const throw();
  /* Same as pre(t), but if locked == true, then updates are not
   * allowed and writes become locked.
   */
  std::list<Constraint*> pre(const Machine::PTransition &t, bool locked) const;
  std::list<Constraint*> pre(const Machine::PTransition &) const;
  std::list<Constraint*> post(const Machine::PTransition &, bool locked) const;
  std::list<Constraint*> post(const Machine::PTransition &) const;
  void abstract();
  void abstract(const Common::AbstractionResult &ar);
  bool is_abstracted() const { return temporary_predicates.empty(); };
  bool is_init_state() const;
  /* pre: c is of type PbConstraint 
   * pre: c.is_abstracted() */
  Comparison entailment_compare(const Constraint &c) const;
  /* pre: c.is_abstracted() */
  Comparison entailment_compare(const PbConstraint &c) const;
  int proc_count() const throw() { return common.machine.proc_count(); };
  std::string to_string() const throw();
  /* Based on the structure of the machine m, attempts to guess a set
   * of predicates which is sufficient to prove reachability or
   * nonreachability for m.
   */
  static pred_set extract_predicates(const Machine &m);
  /* Pre: c is a PbConstraint. */
  bool intersects(const Constraint &c) const;
  bool intersects(const PbConstraint &c) const;
  std::list<Constraint*> range(const Machine::PTransition &trans) const;
  bool is_dirty() const throw() { return dirty_bit; };
  std::list<const Machine::PTransition*> partred() const;
private:
  PbConstraint(bool dirty_bit, std::vector<sharinglist<Lang::MemLoc<int> > > channels, 
               std::vector<int> pcs, const std::list<TsoCycleLock> &cls, Common &c);
  Common &common;
  /* Each message msg contains *only* the variable given by
   * msg. Messages are inserted by writes at the back of the
   * channel, and updated to memory from the front of the
   * channel. Indexes goes from 1 at the front of the channel and
   * increasing towards the back of the channel.
   */
  std::vector<sharinglist<Lang::MemLoc<int> > > channels;
  /* Contains the applied predicates asserted by this constraint. Some
   * may be abstracted, others not. The Predicates of the ones which
   * are not abstract should be in temporary_predicates.
   *
   * Invariant: ap_list is sorted.
   */
  std::list<AppliedPredicate> ap_list;
  std::vector<int> pcs;
  /* The dirty bit is set iff this constraint has been reached only by
   * exploiting the overapproximation of bounding the buffers. I.e. if
   * the channels have at some point overflowed, or been assumed to be
   * overflowed.
   */
  bool dirty_bit;
  /* Non-empty iff this constraint is non-abstracted. Contains
   * pointers to predicates which are local to this constraint. The
   * predicates are owned by this constraint and should be deleted
   * when not needed.
   */
  std::list<Predicate*> temporary_predicates;

  /* Describes the transition by the pre of which this constraint was
   * created.
   */
  struct last_trans_t {
    last_trans_t() : defined(false) {};
    last_trans_t(bool lc, int pd) : defined(true), local(lc), pid(pd) {};
    bool defined; // Is there such a transition? Do we know it?
    bool local;   // The transition is local
    int pid;      // The process who performed the transition
  } last_trans;

  /* All locks which are in power in this constraint.
   * 
   * cycle_locks is sorted contains no two locks to the same (pid, r). */
  std::list<TsoCycleLock> cycle_locks;

  /* Removes all locks for process pid from cycle_locks */
  void clear_cycle_locks(int pid);

  /* Inserts cl into cycle_locks in the correct position w.r.t. the
   * ordering of cycle_locks.
   *
   * Pre: There is no lock in cycle_locks to the same
   * (pid,r) as cl.
   */
  void add_cycle_lock(const TsoCycleLock &cl);

  /* For each cycle lock cl in cycle_locks, runs cl.execute(pt) */
  void cycle_locks_execute(const Machine::PTransition *pt);

  class PreCF;
  friend class PreCF;
  class RangeCF;
  friend class RangeCF;

  /* Returns the index of the message in the channel of process pid,
   * from which process pid would read the memory location ml
   * (nml). Returns 0, if process pid would read from memory.
   */
  int index_of(int pid, const Lang::MemLoc<int> &ml) const;
  /* Returns true if the channel of process pid is either empty or has
   * a rightmost message which is correct in control state pc
   * according to common.last_msg. Returns false otherwise.
   */
  bool ok_last_msg(int pid, int pc) const;

  /* Copies the ap_set of pbc into the ap_set (and
   * temporary_predicates if necessary) of this constraint, with all
   * variables occurring in pbc.ap_set translated by t. On return this
   * constraint will be abstracted precisely if it was abstracted at
   * the call, pbc is abstracted and no predicates in pbc.ap_set
   * contain constant variables.
   */
  void get_ap_set_from(const PbConstraint &pbc, std::function<TsoVar(const TsoVar&)> &t);

  /* Performs the fixpoint computation of moving all applied
   * predicates in src which share a variable with an applied
   * predicate in dst from src to dst.
   *
   * src will remain sorted if it is sorted from the start. No
   * guarantees for dst.
   */
  static void transfer_related(std::list<AppliedPredicate> *src, std::list<AppliedPredicate> *dst) throw();

  /* Inserts e into l such that l remains sorted.
   *
   * Pre: l is sorted.
   */
  template<class E> static void sorted_insert(std::list<E> *l, const E &e) throw();

  /* Returns true if there is an element which occurs in both a and b,
   * false otherwise.
   */
  template<class E> static bool set_intersects(const std::set<E> &a, const std::set<E> &b) throw();

  /* If common.auto_abstract == false, then apl and tmp_preds are
   * merged with ap_list and temporary predicates. If
   * common.auto_abstract == true, then apl is abstracted and then
   * merged with ap_list. If common.auto_abstract == true, then all
   * predicates in tmp_preds are freed.
   *
   * apl and tmp_preds will be cleared at return time.
   *
   * Post: If the resulting ap_list is consistent, true is
   * returned. Otherwise false is returned.
   *
   * Pre: ap_list is sorted and consistent. If common.auto_abstract,
   * then ap_list must be abstracted. The non-abstract predicates in
   * apl should be precisely the predicates pointed to from tmp_preds.
   */
  bool add_to_ap_list(std::list<AppliedPredicate> *apl, std::list<Predicate*> *tmp_preds);

  /* Helper to extract_predicates(const Machine&) */
  static VecSet<Predicate> extract_predicates(const Lang::Stmt<int> &stmt, int pid);

  /* Give private access to containers */
  friend class PbContainer1;
  friend class PbContainer2;

  friend class PbCegar;

#ifndef NDEBUG
  bool ap_list_is_sorted() const throw();
  bool ap_list_is_abstract() const throw();
#endif
};

#endif // __PB_CONSTRAINT__


