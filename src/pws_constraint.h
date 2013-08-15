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
    std::vector<std::vector<std::map<Lang::NML, value_t> > > pending_set;

    /* Helpers to compute pending_set and pending_buffers */
    void init_pending(std::function<bool(const Lang::Stmt<int>&, Lang::MemLoc<int>)> ,
                      const std::vector<Automaton::State> &,
                      std::vector<std::vector<std::map<Lang::NML, value_t> > > &);
    void iterate_pending(std::function<bool(const Lang::Stmt<int>&, Lang::MemLoc<int>)>,
                         const std::vector<Automaton::State>&, int, int,
                         std::vector<std::map<Lang::NML, ZStar<int> > > &, bool &);

    typedef std::pair<std::list<std::map<Lang::NML, value_t> >,
                      std::list<std::map<Lang::NML, value_t> > > map_sequence;

    /* A last write sequence of a process p is a sequence of writes, for
     * example, z=1, |, y=3, x=2, that are the last writes that p has done to
     * each memory location. Additionally, the sequence contains one instance of
     * the symbol "|", that marks which of these writes are ahead of p's
     * cpointer. We will use this more compact syntax for a last write sequence:
     *   z=1|y=3:x=2
     *
     * last_write_sets[pid][pc] describes all last write sequences that are
     * possible for a process pid in state pc. It is a tuple, where the the
     * first element is the part of the sequence that is definitely behind the
     * "|" symbol, and the second element is the part of the sequence that may
     * lie on either side of the "|" symbol.
     *
     * The sequences are represented as a list of maps, for example,
     * {z=1, y=3}, {x=2}. If more than one write are present in a map, the map
     * describes any sequence that is some ordering of it's elements. Thus, the
     * above example describes both z=1:y=3:x=2 and y=3:z=1:x=2, but not
     * z=1:x=2:y=3. Also, the maps may contain the value "*" (represented by
     * value_t::STAR), which describes any write to that memory location.
     *
     * Invariants: In each sequence pair, each memory location may only appear
     *             in one map.
     *             Only the last map in each sequence in a pair may be empty. */
    std::vector<std::vector<std::set<map_sequence> > > last_write_sets;

    /* Returns all maps that have some subset of the key-value pairs of map and their complements. */
    std::list<std::pair<std::map<Lang::NML, value_t>,
                        std::map<Lang::NML, value_t> > >
    powermaps(std::map<Lang::NML, value_t> map) const;

    /* Returns a value that subsumes any value written by trans.
     * Pre: trans only writes a single memory location. */
    inline value_t value_of_write(const Machine::PTransition &trans) const;

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

  inline VecSet<int> possible_values(const value_t &buffer_value, const Lang::NML &nml) const;

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
  static const bool use_last_write_sets;
};

// Implementations
// -----------------------------------------------------------------------------

inline VecSet<int> PwsConstraint::possible_values(const ZStar<int> &buffer_value,
                                                  const Lang::NML &nml) const {
  Lang::VarDecl var_decl = common.machine.get_var_decl(nml);
  if (buffer_value.is_wild()) {
    std::vector<int> vec;
    for (int possible_value : var_decl.domain)
      vec.push_back(possible_value);
    return VecSet<int>(vec);
  } else return VecSet<int>::singleton(buffer_value.get_int());
};

inline ZStar<int> PwsConstraint::Common::value_of_write(const Machine::PTransition &trans) const {
  assert(trans.instruction.get_writes().size() == 1);
  Store s = store_of_write(trans);
  return s[index(Lang::NML(trans.instruction.get_writes()[0], trans.pid))];
};

#endif // __PWS_CONSTRAINT_H__
