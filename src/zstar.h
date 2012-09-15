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
 */

/* The class ZStar represents the set Z union {STAR} where Z should be
 * some set of integers (e.g. int) and STAR is a wild-card value that
 * represents the notion of "any integer".
 * 
 * Values of type ZStar are immutable.
 */

#include "constraint.h"

template<class Z> class ZStar{
public:
  /* The integer i. */
  ZStar(Z i) : wild(false), z(i) {};
  /* STAR */
  ZStar() : wild(true) {};
  /* STAR */
  static const ZStar STAR;
  /* Returns the integer represented by this ZStar.
   * Undefined for STAR. */
  Z get_int() const throw();
  /* Three synonymes for checking whether this ZStar is STAR. */
  bool is_wild() { return wild; };
  bool is_star() { return wild; };
  bool is_STAR() { return wild; };
  /* Comparisons */
  /* STAR is considered greater than all integers. */
  bool operator==(const ZStar &zs) const throw();
  bool operator!=(const ZStar &zs) const throw();
  bool operator<(const ZStar &zs) const throw();
  bool operator<=(const ZStar &zs) const throw();
  bool operator>(const ZStar &zs) const throw();
  bool operator>=(const ZStar &zs) const throw();

  /* Arithmetic */
  /* Simply executes the operation on Z */
  /* Undefined if any argument is STAR */
  ZStar operator+(const ZStar &zs) const throw();
  ZStar operator-(const ZStar &zs) const throw();
  ZStar operator*(const ZStar &zs) const throw();
  ZStar operator/(const ZStar &zs) const throw();

  std::string to_string() const throw();

  /* An unmutable vector of ZStar values.
   * Duplicate Vectors will share their representation.
   */
  class Vector {
  public:
    /* A new Vector with sz entries, all set to STAR. */
    Vector(int sz);
    /* A new Vector with sz entries, where entry i has the value f(i). */
    Vector(int sz, std::function<ZStar(int)> &f);
    /* A new Vector with sz entries, where entry i has the value v[i]. */
    Vector(const std::vector<ZStar> &v);
    /* A duplicate of v, sharing the same representation. */
    Vector(const Vector &v);
    /* Replaces this vector by v.
     * This does not affect any duplicates of this vector.
     */
    Vector &operator=(const Vector &v);
    ~Vector();
    /* The ZValue at index i in this vector */
    const ZStar &operator[](int i) const;
    /* Return a new Vector which is identical to this one, except that
     * element i is set to val.
     */
    Vector assign(int i, const ZStar &val) const;
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
     * comparing the ZStar values a and b,
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
  private:
    /* vec[0] is the reference counter. vec[1] is the number of
     * values in the vector. All subsequent entries in store are
     * values.
     */
    ZStar *vec;

    void release_vec();
  };

  static int test();
private:
  /* This ZStar is STAR iff wild == true.
   * Otherwise this ZStar is the integer z.
   */
  bool wild;
  Z z;  
};

#include "zstar.tcc"
