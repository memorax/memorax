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
 */

#ifndef __DUAL_ZStar_H__
#define __DUAL_ZStar_H__

/* The class DualZStar represents the set Z union {STAR} where Z should be
 * some set of integers (e.g. int) and STAR is a wild-card value that
 * represents the notion of "any integer".
 * 
 * Values of type DualZStar are immutable.
 */

#include "constraint.h"
#include "lang.h"
#include "vecset.h"

template<class Z> class DualZStar{
public:
  /* The integer i. */
  DualZStar(Z i) : wild(false), z(i) {};
  /* STAR */
  DualZStar() : wild(true) {};
  /* STAR */
  static const DualZStar STAR;
  /* Returns the integer represented by this DualZStar.
   * Undefined for STAR. */
  Z get_int() const throw();
  operator int() const throw() { return get_int(); };
  /* Three synonymes for checking whether this DualZStar is STAR. */
  bool is_wild() const throw() { return wild; };
  bool is_star() const throw() { return wild; };
  bool is_STAR() const throw() { return wild; };
  /* Comparisons */
  /* STAR is considered greater than all integers. */
  bool operator==(const DualZStar &zs) const throw();
  bool operator!=(const DualZStar &zs) const throw();
  bool operator<(const DualZStar &zs) const throw();
  bool operator<=(const DualZStar &zs) const throw();
  bool operator>(const DualZStar &zs) const throw();
  bool operator>=(const DualZStar &zs) const throw();

  /* Arithmetic */
  /* Simply executes the operation on Z */
  /* Undefined if any argument is STAR */
  DualZStar operator+(const DualZStar &zs) const throw();
  DualZStar operator-(const DualZStar &zs) const throw();
  DualZStar operator*(const DualZStar &zs) const throw();
  DualZStar operator/(const DualZStar &zs) const throw();

  /* Return true iff the cut of the sets of values represented by this DualZStar and
   * zs is nonempty. */
  inline bool unifiable(const DualZStar &zs) const noexcept {
    return wild || zs.wild || z == zs.z;
  };

  /* If the cut of the sets of values represented by this DualZStar and zs is
   * nonempty, return a DualZStar representing that cut. Otherwise, set unifiable to
   * false and return an arbitrary DualZStar.
   */
  DualZStar unify(const DualZStar &zs, bool *unifiable) const;


  std::string to_string() const throw();

  /* An unmutable vector of DualZStar values.
   * Duplicate Vectors will share their representation.
   */
  class Vector {
  public:
    /* A new Vector with sz entries, all set to STAR. */
    Vector(int sz);
    /* A new Vector with sz entries, where entry i has the value f(i). */
    Vector(int sz, std::function<DualZStar(int)> &f);
    /* A new Vector with v.size() entries, where entry i has the value v[i]. */
    Vector(const std::vector<DualZStar> &v);
    /* A new Vector with v.size() entries, where entry i is the integer v[i]. */
    Vector(const std::vector<Z> &v);
    /* A duplicate of v, sharing the same representation. */
    Vector(const Vector &v);
    /* Replaces this vector by v.
     * This does not affect any duplicates of this vector.
     */
    Vector &operator=(const Vector &v);
    ~Vector();
    /* The ZValue at index i in this vector */
    const DualZStar &operator[](int i) const;
    /* The last ZValue in this vector */
    const DualZStar &back() const { return (*this)[size() - 1]; }
    /* Return a new Vector which is identical to this one, except that
     * element i is set to val.
     */
    Vector assign(int i, const DualZStar &val) const;
    /* Return a new Vector which has val as its first element, and the elements
     * of this one as the rest.
     */
    Vector push_front(DualZStar val) const;
    /* Return a new Vector which is identical to this one, except the very last
     * element is removed.
     */
    Vector pop_back() const;
    /* If there is a least upper bound lub (by entailment_compare) of
     * this store and s, then *unifiable is set to true and lub is
     * returned. Otherwise *unifiable is set to false and an arbitrary
     * store is returned.
     *
     * Pre: this->size() == s.size()
     */
    Vector unify(const Vector &s, bool *unifiable) const;
    /* Compares this Vector with v. If this vector and v have
     * different lengths, then they are considered uncomparable,
     * otherwise the comparison is pointwise on the elements. When
     * comparing the DualZStar values a and b,
     * - they are considered equal iff a == b
     * - a is considered lesser than b iff a == STAR && b != STAR
     * - b is considered lesser than a iff b == STAR && a != STAR
     * - they are considered uncomparable otherwise
     */
    Constraint::Comparison entailment_compare(const Vector &v) const;
    /* Returns the number of elements in this vector. */
    int size() const;
    /* Implements a total order over Vectors.
     *
     * Returns -1 if this is smaller than st
     * Returns 0 if this equals st
     * Returns 1 if this is greater than st
     */
    int compare(const Vector &v) const;
    bool operator<(const Vector &v) const { return compare(v) < 0; };
    bool operator==(const Vector &v) const { return compare(v) == 0; };
    bool operator>(const Vector &v) const { return compare(v) > 0; };
    bool operator<=(const Vector &v) const { return compare(v) <= 0; };
    bool operator!=(const Vector &v) const { return compare(v) != 0; };
    bool operator>=(const Vector &v) const { return compare(v) >= 0; };
    std::string to_string() const throw();

    /* Methods for non-deterministic evaluation */

    /* Returns the set of vectors which are entailed by this vector
     * and where the expression e evaluates to the unique value value,
     * where (*this)[r] is the value of register r and decls[r] is the
     * declaration of register r.
     *
     * Pre: For each register r, either (*this)[r] is not STAR or the
     *      domain of register r in decls is finite.
     *
     * I.e. tries to instantiate any STAR in this vector such that e
     * will evaluate to value.
     */
    VecSet<Vector> possible_regs(const Lang::Expr<int> &e, int value,
                                 const std::vector<Lang::VarDecl> &decls) const;
    VecSet<Vector> possible_regs(const std::vector<Lang::VarDecl> &decls) const;
    VecSet<Vector> possible_stores(const std::vector<Lang::VarDecl> &decls) const;
    /* Returns the set of vectors which are entailed by this vector
     * and where the expression b evaluates to true, where (*this)[r]
     * is the value of register r and decls[r] is the declaration of
     * register r.
     *
     * Pre: For each register r, either (*this)[r] is not STAR or the
     *      domain of register r in decls is finite.
     *
     * I.e. tries to instantiate any STAR in this vector such that b
     * will evaluate to true.
     */
    VecSet<Vector> possible_regs(const Lang::BExpr<int> &b,
                                 const std::vector<Lang::VarDecl> &decls) const;
    /* Returns the set of possible values for (*this)[i] given that
     * decl is the declaration for (*this)[i]. I.e. if (*this)[i] is
     * an integer j, then {j} is returned, otherwise the domain given
     * in decl is returned.
     *
     * Pre: Either (*this)[i] is not STAR or decl.domain is finite.
     */
    VecSet<Z> possible_values(int i, const Lang::VarDecl &decl) const;
    /* Returs the set of possible values to which the expression e may
     * valuate given that (*this)[r] is the value of register r, and
     * that decls[r] is the declaration of register r.
     *
     * Pre: For each register r, either (*this)[r] is not STAR or
     * decls[r] is finite.
     */
    VecSet<Z> possible_values(const Lang::Expr<int> &e, 
                              const std::vector<Lang::VarDecl> &decls) const;
  private:
    /* vec[0] is the reference counter. vec[1] is the number of
     * values in the vector. All subsequent entries in store are
     * values.
     */
    DualZStar *vec;

    void release_vec();
  };

  static void test();
private:
  /* This DualZStar is STAR iff wild == true.
   * Otherwise this DualZStar is the integer z.
   */
  bool wild;
  Z z;  
};

template<class Z>
inline std::ostream &operator<<(std::ostream &os, const DualZStar<Z> &zs){
  return os << zs.to_string();
};

#include "dual_zstar.tcc"

#endif
