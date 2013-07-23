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

class PwsConstraint : public SbConstraint{
private:  
  typedef ZStar<int> value_t;
  typedef ZStar<int>::Vector Store;

public:
  class Common : public SbConstraint::Common { 
  public:
    Common(const Machine &);
    virtual std::string to_string() const {
      return "<PwsConstraint::Common>";
    };
    /* Constructs and returns a list of bad states based on the
     * machine and possible initial messages in the channel. */
    std::list<Constraint*> get_bad_states();
  };
  /* Constructs a constraint where process pid is at control state
   * pcs[pid], all registers and memory locations are unrestricted,
   * and the channel consists of exactly one message with an
   * unrestricted memory snapshot and writer and written memory
   * locations as specified by msg. */
  PwsConstraint(std::vector<int> pcs, const SbConstraint::Common::MsgHdr &msg, Common &c);
  PwsConstraint(const PwsConstraint &) = default;
  PwsConstraint(const SbConstraint &s, Common &c);
  PwsConstraint &operator=(const PwsConstraint&) = default;
  // virtual ~PwsConstraint() throw();
  // virtual void abstract(){};
  // virtual bool is_abstracted() const { return true; };
  // virtual bool is_init_state() const;
  // virtual std::list<const Machine::PTransition*> partred() const;
  virtual std::list<Constraint*> pre(const Machine::PTransition &) const;
  virtual std::string to_string() const throw();
  // virtual Comparison entailment_compare(const Constraint &c) const;
  virtual Comparison entailment_compare(const SbConstraint &sbc) const;
  virtual Comparison entailment_compare(const PwsConstraint &sbc) const;

private:
  Common &common;

  /* write_buffers[pid][nml] is the write buffer of process pid to memory location nml */
  std::vector<std::vector<Store>> write_buffers;

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

  /* Does the same thing as SbConstraint::channel_pop_back, but with
   * PwsConstraints instead */
  std::vector<PwsConstraint*> channel_pop_back() const;
  /* Returns the set S of PwsConstraints pwsc such that pwsc is this
   * PwsConstraint but with the buffer of pid to nml replaced by a buffer b such
   * that b . v is entailed by this->write_buffers[pid][nmli] where v is the
   * last value in this->write_buffers[pid][nmli], and the upward closure of S
   * is the set of precisely all such PwsConstraints pwsc.
   *
   * Pre: write_buffers[pid][nmli].size() > 0
   */
  std::vector<PwsConstraint*> buffer_pop_back(int pid, Lang::NML nml) const;

  /* Checks if a set of memory locations are fully serialised, which is a
   * requirement in order to take a s- or mfence transition in the PWS model,
   * and equivalently be able to do a write in a LOCKED or SLOCKED block. */
  bool is_fully_serialised(const std::vector<Lang::MemLoc<int>> &mls, int pid) const;

    // Helper for the public pre
  struct pre_constr_t{
    pre_constr_t(PwsConstraint *pwsc) : pwsc(pwsc), channel_pop_back(false),
                                        buffer_pop_back(false), written_nmls() {};
    pre_constr_t(PwsConstraint *pwsc, bool channel_pop_back, 
                 bool buffer_pop_back, VecSet<Lang::NML> wn)
      : pwsc(pwsc), channel_pop_back(channel_pop_back),
        buffer_pop_back(buffer_pop_back), written_nmls(wn) {};
    PwsConstraint *pwsc;
    bool channel_pop_back; // true iff the last message in pwsc should be popped
    bool buffer_pop_back; // true iff the last message in pwsc should be popped
    VecSet<Lang::NML> written_nmls; // NMLs that were written. only one if buffer_pop_bacl
  };
  // PRE: mlocked => slocked
  std::list<pre_constr_t> pre(const Machine::PTransition &, bool mlocked, bool slocked) const;

  friend class Common;
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
