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

#ifndef __VIPS_BIT_CONSTRAINT_H__
#define __VIPS_BIT_CONSTRAINT_H__

#include "automaton.h"
#include "machine.h"
#include "lang.h"
#include "vecset.h"

#include <cstdint>
#include <map>
#include <vector>

/* VipsBitConstraint is an explicit representation of a VIPS-M
 * configuration. It is meant for explicit state forward model
 * checking under VIPS-M. 
 *
 * VipsBitConstraint uses bit-packing to achieve small configurations
 * with fast comparison. For this reason it does not inherit from
 * Constraint, and does not include a reference to a Common object in
 * each configuration. For this reason, most operations will require a
 * Common object as a parameter. In general, a pre-condition is that
 * the Common object sent as argument to VipsBitConstraint methods
 * needs to be the same object that was given as an argument in the
 * construction of the VipsBitConstraint object.
 *
 * The VIPS-M semantics used by this Constraint are those of
 * formalization.VIPS-M.simplified.3.alt.pdf. However, the L1 cache of
 * each process will always have an entry for each memory
 * location. Invisible evicts are assumed before each instruction or
 * event that depends on a memory location not being in L1.
 */
class VipsBitConstraint{
private:
  typedef uintptr_t data_t;
public:
  /* A Common object contains information common to all
   * VipsBitConstraints for a particular Machine.
   *
   * The Common object provides information about the Machine, which
   * is necessary to interpret VipsBitConstraints.
   */
  class Common{
  public:
    /* Construct a Common object for constraints of m. */
    Common(const Machine &m);
    Common(const Common&) = delete;
    Common &operator=(const Common&) = delete;
    ~Common();
    /* Compute and return some initial constraint of this->machine */
    VipsBitConstraint *get_initial_constraint();
    /* Compute and return all initial constraints of this->machine. */
    std::set<VipsBitConstraint*> get_initial_constraints();
    /* Return a clone of vbc. */
    VipsBitConstraint *clone(const VipsBitConstraint &vbc);
    /* Deallocate vbc.
     *
     * For efficient use, vbc should be the latest VipsBitConstraint
     * that has been allocated by this Common object and has not yet
     * been deallocated.
     *
     * Pre: vbc was created by this Common object.
     *      vbc has not been deallocated.
     */
    void dealloc(VipsBitConstraint *vbc);
    /* Compare a and b by some total order.
     * Return 0 if a = b, -1 if a < b and 1 if b < a.
     */
    int compare(const VipsBitConstraint &a, const VipsBitConstraint &b) const;
    /* The Machine which this Common object corresponds to. */
    const Machine &machine;
  private:
    /*********************/
    /* Memory management */
    /*********************/
    static const int buffer_multiplier;
    /* Each element cp in vbc_buffers is an array of
     * sizeof(VipsBitConstraint)*buffer_multiplier bytes. The buffers
     * are used for keeping allocated VipsBitConstraints.
     */
    std::vector<char*> vbc_buffers;
    /* Each element cp in data_buffers is an array of
     * bits_len*sizeof(data_t)*buffer_multiplier bytes. The buffers
     * are used for keeping allocated data_t arrays pointed to by
     * VipsBitConstraints.
     */
    std::vector<char*> data_buffers;
    /* (next_buffer,next_elem) describes which elements in the
     * vbc/data buffers are used, and which are available for
     * allocation. An element vbc_buffers[i][j] is used if (i,j) <lex
     * (next_buffer,next_elem*sizeof(VipsBitConstraint)). An element
     * data_buffers[i][j] is used if (i,j) <lex
     * (next_buffer,next_elem*sizeof(data_t)*bits_len).
     *
     * If pointer_pack is set, then data_buffers is empty, otherwise
     * the bits member of the VipsBitConstraint
     * vbc_buffers[i][j*sizeof(VipsBitConstraint)] points to
     * data_buffers[i][j*bits_len*sizeof(data_t)].
     *
     * Invariant:
     *   0 <= next_buffer <= vbc_buffers.size()
     *   0 <= next_elem <= buffer_multiplier
     *   If pointer_pack then data_buffers.empty()
     *   Otherwise vbc_buffers.size() == data_buffers.size()
     */
    int next_buffer, next_elem;
    /* Allocate space for a new VipsBitConstraint (and its
     * data). Return the VipsBitConstraint. If !pointer_pack, then the
     * bits member is set to point to freshly allocated data,
     * otherwise bits is uninitialized.
     */
    VipsBitConstraint *alloc();
    /* Counts the number of times a VipsBitConstraint has been
     * dealloc'ed in this Common object without it being possible to
     * recycle its memory. Only used for debugging.
     */ 
#ifndef NDEBUG
    int bad_dealloc_count;
#endif

    /*****************************/
    /* Information about machine */
    /*****************************/

    /* The number of processes in machine.
     */
    int proc_count;

    /* Calculates the amount of data necessary to store a
     * configuration of m. Returns true if this amount is small enough
     * to be pointer packed. Returns false otherwise.
     */
    static bool possible_to_pointer_pack(const Machine &m);

    /* All transitions of machine, instructions and fetches and
     * wrllcs.
     *
     * Pointers into this vector will be returned by
     * VipsBitConstraint::partred.
     */
    std::vector<Machine::PTransition> all_transitions;

    /* all_transitions_per_cs[p][cs] is a vector containing pointers
     * to all transitions in all_transitions with source control state
     * cs of process p.
     */
    std::vector<std::vector<std::vector<const Machine::PTransition*> > > all_transitions_per_cs;

    /* Initialize all_transitions and all_transitions_per_cs, based on
     * m.
     *
     * Called from Common constructor.
     */
    void init_all_transitions(const Machine &m);

    /**************************************************************/
    /* Information about how to interpret VipsBitConstraint::bits */
    /**************************************************************/

    /* All data in a VipsBitConstraint is packed into
     * VipsBitConstraint::bits. These members describe how to
     * interpret bits.
     *
     * The short notation vbcbits is used to denote
     * VipsBitConstraint::bits.
     */
    
    /* Should data be packed directly into vbcbits, as opposed to into
     * the array pointed to by bits? See VipsBitConstraint::bits.
     */
    bool pointer_pack;
    /* The number of elements in the array pointed to by vbcbits. */
    int bits_len;
    
    /* A bitfield represents a certain part of vbcbits.
     *
     * A bitfield is entirely contained within one element of the
     * vbcbits array (or in the vbcbits pointer if pointer_pack is
     * set).
     *
     * A bitfield must be strictly shorter than a data_t.
     */
    struct bitfield{
      /* Pre:
       * e >= 0
       * 0 < d < std::numeric_limits<data_t>::max()
       * 0 < m <= std::numeric_limits<data_t>::max()
       */
      bitfield(int e, data_t d, data_t m, int off);
      /* Which element of the array pointed to by vbcbits contains
       * this bitfield?
       *
       * This field is undefined if pointer_pack is set.
       */
      int element;
      /* To get the bitfield from an element e, compute e / div % mod. */
      data_t div, mod;
      /* A value v is stored in the bitfield as v - offset.
       *
       * offset should be such that the domain of values to be stored
       * is some range [0,n], with 0 as its lowest element.
       */
      int offset;
      int get_el(data_t e) const{ return e / div % mod + offset; };
      int get_vec(const data_t *vec) const{ return get_el(vec[element]); };
      /* Returns e, with this bitfield set to v. */
      data_t set_el(data_t e,int val) const{
        val -= offset;
        assert(0 <= val);
        assert((data_t)val < mod);
        return (e - e%(div*mod)) + val*div + e%div;
      };
      /* Updates the part of the element in vec described by this
       * bitfield, with the value val */
      void set_vec(data_t *vec,data_t val) const{ vec[element] = set_el(vec[element],val); };
      std::string to_string() const;
    };

    /* For each process pid, pcs[pid].get_vec(vbcbits) is its control
     * state. */
    std::vector<bitfield> pcs;

    /* ml_offsets is a vector with one entry per process. For each
     * process p, ml_offsets[p] is the total number of global memory
     * locations + the total number of local memory locations
     * belonging to any process p' with lower pid: p' < p.
     *
     * ml_offsets is used internally to find the position
     * corresponding to a certain NML in vectors such as mem_vec. The
     * global NML with id i, will have index i. The local NML with id
     * i, belonging to process p, will have index ml_offsets[p] + i.
     */
    std::vector<int> ml_offsets;

    /* All NMLs in machine, in the order described by the
     * documentation for ml_offsets.
     */
    std::vector<Lang::NML> all_nmls;

    /* mem_vec[i].get_vec(vbcbits) is the value in memory of the
     * memory location corresponding to index i (as described in the
     * documentation of ml_offsets).
     *
     * Use through the member function mem.
     */
    std::vector<bitfield> mem_vec;

    /* l1_vec[p][i].get_vec(vbcbits) is (v*2 + d) where
     *
     * v is the value in the L1 of process p of the memory location
     * corresponding to index i (as described in the documentation of
     * ml_offsets)
     *
     * d is 1 if the entry for i in the L1 of process p is dirty, 0
     * otherwise.
     */
    std::vector<std::vector<bitfield> > l1_vec;

    /* reg_vec[p][r].get_vec(vbcbits) is the value of the r:th
     * register of process p.
     */
    std::vector<std::vector<bitfield> > reg_vec;

    /* Returns the index of nml according to ml_offsets. */
    int nml_index(const Lang::NML &nml) const{
      if(nml.is_global()){
        return nml.get_id();
      }else{
        return ml_offsets[nml.get_owner()] + nml.get_id();
      }
    }

    /* mem(nml).get_vec(vbcbits) is the value in memory of nml. */
    const bitfield &mem(const Lang::NML &nml) const{
      return mem_vec[nml_index(nml)];
    };

    /* Shorthand for l1_vec[pid][i] where i is the index of nml
     * according to ml_offsets.
     */
    const bitfield &l1(int pid, const Lang::NML &nml) const{
      return l1_vec[pid][nml_index(nml)];
    };

    /* Get the bitfield corresponding to the L1 entry of process
     * caller for the memory location ml from the perspective of
     * process caller.
     */
    const bitfield &l1(int caller, const Lang::MemLoc<int> &ml) const{
      return l1(caller,Lang::NML(ml,caller));
    };

    /* Shorthand for reg_vec[pid][reg]. */
    const bitfield &reg(int pid, int reg) const{
      return reg_vec[pid][reg];
    };

    /* For an L1 value vd, 
     * l1val_is_dirty(vd) returns true iff vd is dirty.
     * l1val_valof(vd) returns the value of vd.
     *
     * For an integer value val,
     * l1val_clean(val) returns an L1 value corresponding to (val,clean)
     * l1val_dirty(val) returns an L1 value corresponding to (val,dirty)
     */
    static bool l1val_is_dirty(int vd){ return vd % 2; }
    static int l1val_valof(int vd){ return (vd - (vd % 2 ? 1 : 0)) / 2; }
    static int l1val_clean(int val){ return val*2; };
    static int l1val_dirty(int val){ return val*2+1; };

    /* Returns the value held in field bf of vbcbits.
     *
     * Takes pointer_pack into account.
     */
    int bfget(const data_t *vbcbits,const bitfield &bf) const{
      if(pointer_pack) return bf.get_el((data_t)vbcbits);
      else return bf.get_vec(vbcbits);
    };

    /* Assigns the value val to the field bf in vbcbits.
     *
     * Takes pointer_pack into account.
     */
    void bfset(data_t **vbcbits,const bitfield &bf,int val) const{
      if(pointer_pack) *vbcbits = (data_t*)bf.set_el((data_t)(*vbcbits),val);
      else bf.set_vec(*vbcbits,val);
    };

    friend class VipsBitConstraint;
  };

  /* A RegVal object regval(pid,common,bits) provides an operator [],
   * such that regval[r] is the value in bits (interpreted by common)
   * of the r:th register of process pid.
   *
   * Use this when calling Expr::eval, BExpr::eval.
   */
  class RegVal{
  public:
    RegVal(int pid,const Common &common,const data_t *bits)
      : pid(pid), common(common), bits(bits) {};
    int operator[](int r) const{
      return common.bfget(bits,common.reg(pid,r));
    };
  private:
    int pid;
    const Common &common;
    const data_t *bits;
  };

  /* To construct VipsBitConstraint objects, use
   * Common::get_initial_constraint or
   * Common::get_initial_constraints.
   */
  VipsBitConstraint() = delete;
  VipsBitConstraint(const VipsBitConstraint&) = delete;
  VipsBitConstraint &operator=(const VipsBitConstraint&) = delete;
  void *operator new(size_t) = delete;
  /* To delete VipsBitConstraint objects, use Common::dealloc with the
   * same Common object that created this constraint.
   *
   * All, not yet deallocated, VipsBitConstraints will be
   * automatically deallocated upon destruction of the Common object
   * that created them.
   */
  void operator delete(void*,size_t) = delete;

  /* Returns the set of transitions that should be explored from this
   * constraint.
   *
   * The transitions are owned by common.
   *
   * The pointers can be used as identifiers of the transitions:
   * For PTransition pointers p and q, we have *p == *q implies p == q,
   * even if p and q were obtained in different calls with the same 
   * common object.
   */
  VecSet<const Machine::PTransition*> partred(const Common &common) const;

  /* Returns the result of applying t to this constraint.
   * Returns null if t cannot be applied to this constraint.
   */
  VipsBitConstraint *post(Common &common, 
                          const Machine::PTransition &t) const;

  /* A vector v such that for each process pid, v[pid] is the control
   * state of pid in this constraint.
   */
  std::vector<int> get_control_states(const Common &common) const throw();

  /* Returns true iff this constraint is forbidden by common.machine.
   */
  bool is_forbidden(const Common &common) const throw();

  /* Return a multi-line, human-readable representation of this Constraint.
   */
  std::string to_string(const Common &common) const;

  static void test();
private:
  /* All data of the constraint are bit-packed here.
   *
   * If common.pointer_pack is set, then the pointer bits should not
   * be interpreted as a pointer. Instead the data is all packed into
   * the field bits itself. So bits should be cast into data_t and
   * treated as data.
   *
   * If common.pointer_pack is unset, then bits points to an array
   * containing common.bits_len elements.
   *
   * General overview of format of the data stored in bits:
   * ppack - Is common.pointer_pack set? (1 bit)
   * pcs - control states of all processes
   * mem - main memory / LLC
   * per process:
   * - L1
   * -- per memory location:
   * --- value
   * --- dirty/clean
   * per process
   * - per register
   * -- value
   */
  data_t *bits;

  bool use_pointer_pack() const{
    return ((ulong)bits) % 2;
  };

  /* Dump the representation of this Constraint in a low-level fashion. */
  std::string debug_dump(const Common &) const;

  /* Returns true iff s is a CAS statement */
  static bool is_cas(const Lang::Stmt<int> &s);

  friend class Common;
};

#endif
