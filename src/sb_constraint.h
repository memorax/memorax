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

#include "channel_constraint.h"
#include "constraint.h"
#include "machine.h"
#include "vecset.h"
#include "zstar.h"

class SbConstraint : public ChannelConstraint{
private:  
  
  typedef ZStar<int> value_t;
  typedef ZStar<int>::Vector Store;

public:
  class Common : public ChannelConstraint::Common{
  public:
    Common(const Machine &m);
    virtual ~Common() noexcept {};
    virtual std::string to_string() const {
      return "<SbConstraint::Common>";
    };

    static void test();

    /* Constructs and returns a list of bad states based on the
     * machine and possible initial messages in the channel.
     */
    virtual std::list<Constraint*> get_bad_states();
  private:
    /* Copies of all transitions occurring in machine, and also all
     * possible update transitions. */
    std::vector<Machine::PTransition> all_transitions;
    /* transitions_by_pc[pid][pc] is a vector containing pointers to
     * all transitions in all_transitions that belong to process pid
     * and has target state pc.
     */
    std::vector<std::vector<std::vector<const Machine::PTransition*> > > transitions_by_pc;

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

    friend class SbConstraint;

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
  virtual SbConstraint *clone() const { return new SbConstraint(*this); }
  virtual std::list<const Machine::PTransition*> partred() const;
  virtual std::list<Constraint*> pre(const Machine::PTransition &) const;

  static void test();
  static void test_possible_values();
  static void test_pre();
  static void test_comparison();

private:
  Common &common;

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

  /* Checks that the channel is possible according to
   * common.last_msgs. Will constrain the possible values in message
   * stores where possible.
   *
   * Messages where this does not hold can be discarded, since they
   * will never reach an initial state.
   */
bool ok_channel();

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
