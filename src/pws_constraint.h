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

class PwsConstraint : public SbConstraint{
private:  
  typedef ZStar<int> value_t;
  typedef ZStar<int>::Vector Store;

public:
  class Common : public SbConstraint::Common { 
  public:
    Common(const Machine &m) : SbConstraint::Common(m) {};
  };
  /* Constructs a constraint where process pid is at control state
   * pcs[pid], all registers and memory locations are unrestricted,
   * and the channel consists of exactly one message with an
   * unrestricted memory snapshot and writer and written memory
   * locations as specified by msg. */
  PwsConstraint(std::vector<int> pcs, const SbConstraint::Common::MsgHdr &msg, Common &c) :
    SbConstraint(pcs, msg, c), common(c) {};
  PwsConstraint(const PwsConstraint &) = default;
  PwsConstraint &operator=(const PwsConstraint&) = default;
  // virtual ~PwsConstraint() throw();
  // virtual void abstract(){};
  // virtual bool is_abstracted() const { return true; };
  // virtual bool is_init_state() const;
  virtual std::list<const Machine::PTransition*> partred() const;
  virtual std::list<Constraint*> pre(const Machine::PTransition &) const;
  virtual std::string to_string() const throw();
  // virtual Comparison entailment_compare(const Constraint &c) const;
  virtual Comparison entailment_compare(const SbConstraint &sbc) const;
  virtual Comparison entailment_compare(const PwsConstraint &sbc) const;

private:
  Common &common;

  Comparison entailment_compare_buffers(const PwsConstraint &sbc) const;
  Comparison entailment_compare_buffer(const Store &a, const Store& b) const;
  /* write_buffers[pid][nml] is the write buffer of process pid to memory location nml */
  std::vector<std::vector<Store>> write_buffers;

  friend class Common;
};

#endif // __PWS_CONSTRAINT_H__
