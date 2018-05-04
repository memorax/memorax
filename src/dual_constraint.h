/*
 * Copyright (C) 2018 Tuan Phong Ngo
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

#ifndef __DUAL_CONSTRAINT_H__
#define __DUAL_CONSTRAINT_H__

#include "dual_channel_constraint.h"
#include "constraint.h"
#include "machine.h"
#include "vecset.h"
#include "dual_zstar.h"

class DualConstraint : public DualChannelConstraint{
private:  
  
  typedef DualZStar<int> value_t;
  typedef DualZStar<int>::Vector Store;

public:
  class Common : public DualChannelConstraint::Common{
  public:
    Common(const Machine &m);
    virtual ~Common() {};
    virtual std::string to_string() const {
      return "<DualConstraint::Common>";
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
    
    friend class DualConstraint;
    friend class DualTsoBwd;

    /* Returns true iff a is a suffix of b */
    template<class T>
    bool vector_is_suffix(const std::vector<T> &a, const std::vector<T> &b) const;
  };
  /* Constructs a constraint where process pid is at control state
   * pcs[pid], all registers and memory locations are unrestricted,
   * and the channel consists of exactly one message with an
   * unrestricted memory snapshot and writer and written memory
   * locations as specified by msg. */
  DualConstraint(std::vector<int> pcs, const Common::MsgHdr &msg, Common &c);
  DualConstraint(std::vector<int> pcs, Common &c);
  virtual DualConstraint *clone() const { return new DualConstraint(*this); }
  virtual std::list<const Machine::PTransition*> partred() const;
  virtual std::list<Constraint*> pre(const Machine::PTransition &) const;

  static void test();
  static void test_possible_values();
  static void test_pre();
  static void test_comparison();

private:
  Common &common;

  /* Helper for the public pre */
  /* Using one nml instead of several nml */
  struct pre_constr_t{
    pre_constr_t(DualConstraint *sbc) : sbc(sbc), pop_back(false), written_nmls() {};
    pre_constr_t(DualConstraint *sbc, bool pop_back, VecSet<Lang::NML> wn)
      : sbc(sbc), pop_back(pop_back), written_nmls(wn) {};
    DualConstraint *sbc;
    bool pop_back; // true iff the last message in sbc should be popped
    VecSet<Lang::NML> written_nmls; // NML that were written
  };

  virtual std::list<pre_constr_t> pre(const Machine::PTransition &, bool locked) const;

  friend class Common;
  friend class DualTsoBwd;

  /*****************/
  /* Configuration */
  /*****************/
  static const bool use_limit_other_delete_propagate;
  static const bool use_propagate_only_after_write;
  static const bool use_allow_all_delete;
  static const bool use_allow_all_propagate;
};

#endif
