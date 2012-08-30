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

#ifndef __SB_CONSTRAINT_H__
#define __SB_CONSTRAINT_H__

#include "constraint.h"
#include "machine.h"
#include "vecset.h"

class SbConstraint : public Constraint{
public:
  class Common;
private:  
  /* The type used to represent integer values in registers and memory
   * locations. Contains a reserved value STAR, which represents "any
   * value".
   */
  typedef int value_t;
  static const value_t STAR;
  /* A reference counted array of values. Used to represent valuations
   * over memory locations or registers.
   */
  class Store {
  public:
    /* Constructs a new store with sz entries (apart from the
     * reference counter). All entries will be set to STAR. */
    Store(int sz){
      store = new value_t[sz+2];
      store[0] = 1;
      store[1] = sz;
      for(int i = 0; i < sz; i++){
        store[i+2] = STAR;
      }
    };
    /* Constructs a new store with v.size() entries where entry i has
     * the value v[i].
     */
    Store(const std::vector<value_t> &v){
      store = new value_t[v.size()+2];
      store[0] = 1;
      store[1] = v.size();
      for(unsigned i = 0; i < v.size(); ++i){
        store[i+2] = v[i];
      }
    };
    Store(const Store &s){
      store = s.store;
      ++store[0];
      assert(store[0] > 1);
    };
    Store &operator=(const Store &s){
      if(&s != this){
        assert(store[0] > 0);
        assert(s.store[0] > 0);
        release_store();
        store = s.store;
        ++store[0];
        assert(store[0] > 1);
      }
      return *this;
    };
    ~Store(){
      release_store();
    };
    const value_t &operator[](int i) const { return store[i+2]; };
    /* Return a new store which is identical to this one, except that
     * element i is set to val.
     */
    Store assign(int i, value_t val) const;
    /* Return a new store which is identical to this one, except that
     * for each pair (i,v) in assv, the i:th element of the store is
     * assigned v.
     */
    Store assign(std::vector<std::pair<int,value_t> > assv) const;
    /* If there is a least upper bound lub (by entailment_compare) of
     * this store and s, then *unifiable is set to true and lub is
     * returned. Otherwise *unifiable is set to false and an arbitrary
     * store is returned.
     *
     * Pre: this->size() == s.size()
     */
    Store unify(const Store &s, bool *unifiable) const;
    /* Returns the number of elements in this store. */
    int size() const { return store[1]; };
    /* Implements a total order over Stores. The order takes into
     * account only values, reference counters are not considered.
     *
     * Returns -1 if this is smaller than st
     * Returns 0 if this equals st
     * Returns 1 if this is greater than st
     */
    int compare(const Store &st) const;
    bool operator<(const Store &st) const { return compare(st) < 0; };
    bool operator==(const Store &st) const { return compare(st) == 0; };
    bool operator>(const Store &st) const { return compare(st) > 0; };
    bool operator<=(const Store &st) const { return compare(st) <= 0; };
    bool operator!=(const Store &st) const { return compare(st) != 0; };
    bool operator>=(const Store &st) const { return compare(st) >= 0; };
    std::string to_string() const;
    Constraint::Comparison entailment_compare(const Store &s) const;
  private:
    /* store[0] is the reference counter. store[1] is the number of
     * values in the store. All subsequent entries in store are
     * values.
     */
    value_t *store;

    void release_store(){
      assert(store != 0);
      assert(store[0] > 0);
      --store[0];
      if(store[0] == 0){
        delete[] store;
      }
      store = 0;
    };
  };

  /* The class of SB channel messages. */
  class Msg{
  public:
    Msg(Store s, int pid, VecSet<Lang::NML> ms)
      : store(s), wpid(pid), nmls(ms) {};
    Msg(const Msg &) = default;
    Store store;
    int wpid;      // The pid of the process that wrote
    /* A distinct, sorted vector of all the written memory locations. */
    VecSet<Lang::NML> nmls;
    std::string to_short_string(const Common &common) const;
    /* A total order on messages */
    int compare(const Msg &) const;
    bool operator<(const Msg &msg) const { return compare(msg) < 0; };
    bool operator==(const Msg &msg) const { return compare(msg) == 0; };
    bool operator>(const Msg &msg) const { return compare(msg) > 0; };
    bool operator<=(const Msg &msg) const { return compare(msg) <= 0; };
    bool operator!=(const Msg &msg) const { return compare(msg) != 0; };
    bool operator>=(const Msg &msg) const { return compare(msg) >= 0; };
    Constraint::Comparison entailment_compare(const Msg &msg) const{
      if(wpid != msg.wpid || nmls != msg.nmls){
        return Constraint::INCOMPARABLE;
      }else{
        return store.entailment_compare(msg.store);
      }
    };
  };

public:
  class Common : public Constraint::Common{
  public:
    Common(const Machine &m);
    virtual ~Common();
    virtual std::string to_string() const {
      return "<SbConstraint::Common>";
    };
    const Machine &machine;

    static void test();

    /* Constructs and returns a list of bad states based on the
     * machine and possible initial messages in the channel.
     */
    std::list<Constraint*> get_bad_states();
  private:
    /**************************/
    /* Computed from machine: */
    /**************************/
    // The number of entries in a memory store 
    // (not necessarily equal to the number of memory locations)
    int mem_size;
    // The number of global variables
    int gvar_count;
    // The maximum number of local variables of any process
    int max_lvar_count;
    // All memory locations that occur in machine
    VecSet<Lang::NML> nmls;
    // Gives the index in a memory location store of the memory location given by nml
    int index(const Lang::NML &nml) const{
      if(nml.is_global()){
        return nml.get_id();
      }else{
        return gvar_count + nml.get_owner()*max_lvar_count + nml.get_id();
      }
    };
    // reg_count[pid] is the number of registers of process pid
    std::vector<int> reg_count;
    /* Copies of all transitions occurring in machine, and also all
     * possible update transitions. */
    std::vector<Machine::PTransition> all_transitions;
    /* transitions_by_pc[pid][pc] is a vector containing pointers to
     * all transitions in all_transitions that belong to process pid
     * and has target state pc.
     */
    std::vector<std::vector<std::vector<const Machine::PTransition*> > > transitions_by_pc;
    /* A MsgHdr mh identifies the set of messages where the writing
     * process is mh.wpid and the written variables are mh.nmls.
     */
    struct MsgHdr{
      MsgHdr(int wpid, const VecSet<Lang::NML> nmls) : wpid(wpid), nmls(nmls) {};
      int wpid;
      VecSet<Lang::NML> nmls;
      bool operator==(const MsgHdr &mh) const {
        return wpid == mh.wpid && nmls == mh.nmls;
      };
      bool operator<(const MsgHdr &mh) const{
        return wpid < mh.wpid || 
          (wpid == mh.wpid && nmls < mh.nmls);
      };
    };
    /* The set of all message headers that can possibly occur in the
     * channel of a constraint from this->machine.
     *
     * Will contain a dummy message in case no writes occur in this->machine. */
    VecSet<MsgHdr> messages;
    
    /* last_msgs[pid][s] is a set S of messages such that it is only
     * possible for process pid to be in a local state s and have the
     * message m as rightmost message in the channel written by
     * process pid, if there is some message m2 in S such that m2.nmls
     * == m.nmls && m.wpid == m2.wpid == pid and m.store is unifiable
     * with m2.store.
     */
    std::vector<std::vector<VecSet<Msg> > > last_msgs;
    std::vector<std::vector<VecSet<std::vector<Msg> > > > last_msgs_vec;
    /* can_have_pending[pid][s] is true iff it is possible for process
     * pid to be at local state s and have a non-locked message in the
     * channel to the right of its cpointer. Equivalently iff process
     * pid can be at local state s having executed at least one write,
     * out of which the last one is still pending.
     */
    std::vector<std::vector<bool> > can_have_pending;

    /* If t performs writes deterministically and such that all
     * written values are given as integer literals, then returns a
     * memory store with those values set to the corresponding memory
     * locations. Otherwise returns a memory store with all stars.
     */
    Store store_of_write(const Machine::PTransition &t) const;

    friend class SbConstraint;
    friend class SbTsoBwd;

    /* Returns true iff a is a suffix of b */
    template<class T>
    bool vector_is_suffix(const std::vector<T> &a, const std::vector<T> &b) const;
  };
  /* Constructs a constraint where process pid is at control state
   * pcs[pid], all registers and memory locations are unrestricted,
   * and the channel consists of exactly one message with an
   * unrestricted memory snapshot and writer and written memory
   * locations as specified by msg. */
  SbConstraint(std::vector<int> pcs, const Common::MsgHdr &msg, Common &c);
  SbConstraint(const SbConstraint &) = default;
  SbConstraint &operator=(const SbConstraint&) = default;
  virtual ~SbConstraint() throw();
  virtual const std::vector<int> &get_control_states() const throw() { return pcs; };
  virtual void abstract(){};
  virtual bool is_abstracted() const { return true; };
  virtual bool is_init_state() const;
  virtual std::list<const Machine::PTransition*> partred() const;
  virtual std::list<Constraint*> pre(const Machine::PTransition &) const;
  virtual std::string to_string() const throw();
  virtual Comparison entailment_compare(const Constraint &c) const;
  virtual Comparison entailment_compare(const SbConstraint &sbc) const;
  int get_channel_length() const { return channel.size(); };

  static void test();
  static void test_possible_values();
  static void test_pre();
  static void test_comparison();
private:
  Common &common;
  /* pcs[pid] is the program counter of process pid. */
  std::vector<int> pcs;
  
  /* The SB channel
   *
   * Messages at lower indices are older. */
  std::vector<Msg> channel;

  /* cpointers[pid] is an index into channel, which is the buffer
   * pointer of process pid.
   */
  std::vector<int> cpointers;

  /* reg_stores[pid] is the register valuation of process pid. */
  std::vector<Store> reg_stores;

  /* Returns the set of values that can be held by the memory location
   * nml according to the valuation mem. This will be either the
   * unique valuation given by mem, or if mem maps nml to STAR, the
   * whole domain of nml.
   */
  VecSet<int> possible_values(const Store &mem, const Lang::NML &nml) const;
  /* Returns the set of values that e can evaluate to when its
   * accessed registers are valuated as indicated by reg_store.
   *
   * pid should be the process evaluating e, i.e., the owner of reg_store.
   */
  VecSet<int> possible_values(const Store &reg_store, int pid, const Lang::Expr<int> &e) const;
  /* Returns true iff there is some assignment to the registers
   * accessed by b that is admitted by reg_store, and which satisfies
   * b.
   *
   * pid should be the process evaluating b, i.e., the owner of reg_store.
   */
  bool possibly_holds(const Store &reg_store, int pid, const Lang::BExpr<int> &b) const;
  /* Returns the set of stores which are entailed by reg_store and
   * where the expression e of process pid evaluates to the unique
   * value value.
   *
   * I.e. tries to instantiate any STAR in reg_store such that e will
   * evaluate to value.
   */
  VecSet<Store> possible_reg_stores(const Store &reg_store, int pid, const Lang::Expr<int> &e, int value) const;
  /* Returns the set of stores which are entailed by reg_store and
   * where the expression b of process pid evaluates to true.
   */
  VecSet<Store> possible_reg_stores(const Store &reg_store, int pid, const Lang::BExpr<int> &b) const;
  /* Returns the index into channel of the message from which process
   * pid would read the value of memory location nml if it were to
   * read in this constraint.
   *
   * I.e. the index i = max{j | j = cpointers[pid] || 
   *                            (channel[j].wpid == pid && nml in channel[j].nmls)}
   */
  int index_of_read(Lang::NML nml, int pid) const;
  /* Returns the set S of SbConstraints sbc such that sbc is this
   * SbConstraint but with the channel replaced by a channel c such
   * that c.m is entailed by this->channel where m is the last message
   * in this->channel, and the upward closure of S is the set of
   * precisely all such SbConstraints sbc.
   *
   * Pre: channel.size() > 1
   */
  std::vector<SbConstraint*> channel_pop_back() const;

  /* Helper for the public pre */
  struct pre_constr_t{
    pre_constr_t(SbConstraint *sbc) : sbc(sbc), pop_back(false), written_nmls() {};
    pre_constr_t(SbConstraint *sbc, bool pop_back, VecSet<Lang::NML> wn)
      : sbc(sbc), pop_back(pop_back), written_nmls(wn) {};
    SbConstraint *sbc;
    bool pop_back; // true iff the last message in sbc should be popped
    VecSet<Lang::NML> written_nmls; // NMLs that were written
  };
  virtual std::list<pre_constr_t> pre(const Machine::PTransition &, bool locked) const;
  /* Entailment compare this->channel with sbc.channel. Return the
   * combination (Constraint::comb_comp) of that comparison result and
   * cmp.
   */
  virtual Constraint::Comparison entailment_compare_channels(const SbConstraint &sbc, Constraint::Comparison cmp) const;

  /* An MsgCharacterization keeps some information about a particular
   * message in an SB channel: The writing process, the written memory
   * locations and the set of processes whose cpointers point to this
   * message.
   */
  class MsgCharacterization{
  public:
    MsgCharacterization(int wpid, VecSet<Lang::NML> nmls, VecSet<int> cpointers)
      : wpid(wpid), nmls(nmls), cpointers(cpointers) {};
    /* The writing process */
    int wpid;
    /* The written memory locations */
    VecSet<Lang::NML> nmls;
    /* The processes whose cpointers point to this message */
    VecSet<int> cpointers;
    bool operator==(const MsgCharacterization &mc) const{
      return wpid == mc.wpid && nmls == mc.nmls && cpointers == mc.cpointers;
    };
    bool operator<(const MsgCharacterization &mc) const{
      if(wpid < mc.wpid){
        return true;
      }
      if(wpid > mc.wpid){
        return false;
      }
      return cpointers < mc.cpointers || 
        (cpointers == mc.cpointers && nmls < mc.nmls);
    };
  };

  /* Returns a vector characterizing the channel of this constraint.
   *
   * Let S be the set of messages in this->channel such that either
   * there is some cpointer pointing to the message or the message is
   * the rightmost write of some process to some variable. The
   * returned vector v is such that v[i] is the characterization of
   * the i:th leftmost message in S.
   *
   * Interestingly: 
   *   sbc0.characterize_channel() != sbc1.characterize_channel()
   *   implies that
   *   sbc0.entailment_compare(sbc1) == INCOMPARABLE
   */
  std::vector<MsgCharacterization> characterize_channel() const;

  /* Checks that the channel is possible according to
   * common.last_msgs. Will constrain the possible values in message
   * stores where possible.
   *
   * Messages where this does not hold can be discarded, since they
   * will never reach an initial state.
   */
  bool ok_channel();
  /* Update the stores in the channel such that all stores to the
   * right of and including message i in the channel have a consistent
   * value for the memory location nml. Here i is the index of the
   * rightmost message in the channel that writes to the memory
   * location nml. If there are messages in the channel to the right
   * of and including message i that have inconsistent (and non-STAR)
   * values for memory location nml, then no changes are made and
   * false is returned. Otherwise true is returned.
   *
   * If nmli >= 0, then nmli is assumed to equal common.index(nml).
   */
  bool propagate_value_in_channel(const Lang::NML &nml, int nmli = -1);
  /* Same as propagate_value_in_channel(nml,nmli), but propagates in
   * ch instead of in this->channel.
   *
   * Note: In this function nmli must be a proper index into the
   * stores of messages in ch.
   */
  static bool propagate_value_in_channel(std::vector<Msg> *ch, const Lang::NML &nml, int nmli);
  /* Tries to unify messages in channel with messages in lmv. If
   * possible, then the unified channel is returned and *unifiable is
   * set to true. If not possible, then an empty vector is returned
   * and *unifiable is set to false.
   *
   * Messages are unified in the following manner:
   *
   * Let pch be the result of removing from channel all messages
   * except the ones written by pid and which are the last (rightmost)
   * message for pid and some memory location. For all i, the message
   * in channel which is the i:th from the right in pch will be
   * unified with the i:th message from the right in lmv. After all
   * messages have been unified in such a manner, if
   * use_channel_suffix_equality, then the new values in message
   * stores are propagated with propagate_value_in_channel.
   *
   * This function is a helper to ok_channel when use_last_msgs_vec is
   * set.
   */
  static std::vector<Msg> unify_last_msgs_vec(const std::vector<Msg> &channel, 
                                              const std::vector<Msg> &lmv, int pid,
                                              const Common &common,
                                              bool *unifiable);

  friend class SbContainer;
  friend class SbTsoBwd;
  friend class Common;

  /*****************/
  /* Configuration */
  /*****************/
  static const bool use_last_msg;
  static const bool use_last_msgs_vec; // Note: will nullify use_last_msg
  static const bool use_updates_only_after_reads;
  static const bool use_channel_suffix_equality;
  static const bool use_can_have_pending;
  static const bool use_limit_other_updates;
};

#endif
