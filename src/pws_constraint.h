/*
 * Copyright (C) 2013 Magnus Lång
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

#ifndef __PWS_CONSTRAINT_H__
#define __PWS_CONSTRAINT_H__

#include "sb_constraint.h"
#include "machine.h"
#include "vecset.h"
#include "zstar.h"
#include <sstream>

class PwsConstraint : public ChannelConstraint{
private:
  typedef ZStar<int> value_t;
  typedef ZStar<int>::Vector Store;

public:
  class Common : public ChannelConstraint::Common {
  public:
    Common(const Machine &);
    virtual std::string to_string() const {
      return "<PwsConstraint::Common>";
    };
    /* Constructs and returns a list of bad states based on the
     * machine and possible initial messages in the channel. */
    virtual std::list<Constraint*> get_bad_states();

  private:
    /* Copies of all transitions occurring in machine, and also all
     * possible update transitions. */
    std::vector<Machine::PTransition> all_transitions;
    /* transitions_by_pc[pid][pc] is a vector containing pointers to
     * all transitions in all_transitions that belong to process pid
     * and has target state pc. */
    std::vector<std::vector<std::vector<const Machine::PTransition*> > > transitions_by_pc;

    /* A process p in state q may only have a write m:=v for a memory location m
     * and a value v pending (that is, in it's buffer or in the channel ahead of
     * it's cpointer), if (m, v) is a member of pending_set[p][q] or (m, *) is a
     * memver of pending_set[p][q]. */
    std::vector<std::vector<std::map<Lang::NML, value_t> > > pending_set,
    /* Likewise, but only for writes in the buffer. */
                                                             pending_buffers;

    /* Helpers to compute pending_set and pending_buffers */
    void init_pending(std::function<bool(const Lang::Stmt<int>&, Lang::MemLoc<int>)> ,
                      const std::vector<Automaton::State> &,
                      std::vector<std::vector<std::map<Lang::NML, value_t> > > &);
    void iterate_pending(std::function<bool(const Lang::Stmt<int>&, Lang::MemLoc<int>)>,
                         const std::vector<Automaton::State>&, int, int,
                         std::vector<std::map<Lang::NML, ZStar<int> > > &, bool &);
    friend class PwsConstraint;
    friend class PwsPsoBwd;
  };
  /* Constructs a constraint where process pid is at control state
   * pcs[pid], all registers and memory locations are unrestricted,
   * and the channel consists of exactly one message with an
   * unrestricted memory snapshot and writer and written memory
   * locations as specified by msg. */
  PwsConstraint(std::vector<int> pcs, const Common::MsgHdr &msg, Common &c);
  virtual PwsConstraint *clone() const { return new PwsConstraint(*this); }
  virtual bool is_init_state() const;
  virtual std::list<const Machine::PTransition*> partred() const;
  virtual std::list<Constraint*> pre(const Machine::PTransition &) const;
  virtual Comparison entailment_compare(const Constraint &c) const;
  virtual int get_weight() const;

  static void test();
  static void test_pre();

  /* Return the set S such that the buffer of process p to memory location nml
   * is non-empty iff (p, nml) is a member of S. S is represented as an ordered
   * vector.
   *
   * Note that a.get_filled_buffers() != b.get_filled_buffers()
   * implies that a.entailment_compare(b) == Comparison::UNCOMPARABLE */
  std::vector<std::pair<int, Lang::NML> > get_filled_buffers() const;

protected:
  virtual bool propagate_value_in_channel(const Lang::NML &nml, int nmli = -1);

  /* Appends a string describing the local state of process p to ss */
  virtual void process_to_string(int p, std::stringstream &ss) const noexcept;

private:
  Common &common;

  /* write_buffers[pid][nml] is the write buffer of process pid to memory location nml */
  std::vector<std::vector<Store>> write_buffers;

  Comparison entailment_compare_impl(const PwsConstraint &sbc) const;
  Comparison entailment_compare_buffers(const PwsConstraint &sbc) const;
  Comparison entailment_compare_buffer(const Store &a, const Store& b) const;
  void pretty_print_buffer(std::stringstream &ss, const std::vector<Store> &buffer, Lang::NML nml) const;

  inline VecSet<int> possible_values(const ZStar<int> &buffer_value, const Lang::NML &nml) const;
  /* Checks using various heuristics if this constraint can be reached from an
   * initial state and returns true only if this state is unreachable.
   *
   * Analogous to (the negation of) SbConstraint::ok_channel. Constraints
   * failing these can be safely discarded. */
  bool unreachable();

  /* Returns the set S of PwsConstraints pwsc such that pwsc is this
   * PwsConstraint but with the buffer of pid to nml replaced by a buffer b such
   * that b . v is entailed by this->write_buffers[pid][nmli] where v is the
   * last value in this->write_buffers[pid][nmli], and the upward closure of S
   * is the set of precisely all such PwsConstraints pwsc.
   *
   * Pre: write_buffers[pid][nmli].size() > 0
   */
  std::vector<PwsConstraint*> buffer_pop_back(int pid, Lang::NML nml) const;

  /* Checks if all writes of a process are serialised, which is a requirement in
   * order to take a s- or mfence transition in the PWS model. */
  bool is_fully_serialised(int pid) const;

  /* Checks if all writes to any memory location in nmls of a process are
   * serialised, which is a requirement in order to take a cas transition in the
   * PWS model. */
  bool is_fully_serialised(int pid, const std::vector<Lang::MemLoc<int>> nmls) const;

  /* Helper for the public pre */
  struct pre_constr_t{
    pre_constr_t(PwsConstraint *pwsc) : pwsc(pwsc), channel_pop_back(false),
                                        buffer_pop_back(false), written_nmls() {};
    pre_constr_t(PwsConstraint *pwsc, bool channel_pop_back,
                 bool buffer_pop_back, VecSet<Lang::NML> wn)
      : pwsc(pwsc), channel_pop_back(channel_pop_back),
        buffer_pop_back(buffer_pop_back), written_nmls(wn) {};
    PwsConstraint *pwsc;
    bool channel_pop_back; // true iff the last message in pwsc should be popped
    bool buffer_pop_back; // true iff the last value in the buffer of written_nmls[0] in pwsc should be popped
    VecSet<Lang::NML> written_nmls; // NMLs that were written. only one if buffer_pop_back
  };

  /* locked = true means that we are inside a cas. */
  std::list<pre_constr_t> pre(const Machine::PTransition &, bool locked) const;

  friend class Common;
  friend class PwsPsoBwd;

  /*****************/
  /* Configuration */
  /*****************/
  static const bool use_channel_suffix_equality;
  static const bool use_limit_other_updates;
  static const bool use_pending_sets;
  static const bool use_serialisations_only_after_writes;
};

// Implementations
// -----------------------------------------------------------------------------

inline VecSet<int> PwsConstraint::possible_values(const ZStar<int> &buffer_value, const Lang::NML &nml) const {
    Lang::VarDecl var_decl = common.machine.get_var_decl(nml);
    if (buffer_value.is_wild()) {
      std::vector<int> vec;
      for (int possible_value : var_decl.domain)
        vec.push_back(possible_value);
      return VecSet<int>(vec);
    } else return VecSet<int>::singleton(buffer_value.get_int());
  };

#endif // __PWS_CONSTRAINT_H__
