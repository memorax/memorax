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

/* This file declares the min-coverage algorithm. 
 *
 * The problem solved by the algorithm is the following: Let a set S
 * be given as well as a cost function c : S -> nat . Now let T =
 * {T1,T2,...,Tn} be a set of finite, non-empty subsets of S. Define
 * the extension of c to finite subsets of S as follows: c(M) =
 * Sigma_{m in M} c(m) .
 *
 * A coverage set M for (T,c) is a subset of S, such that for each set
 * Ti in T, the intersection of Ti and M is non-empty. A min-coverage
 * set M for (T,c) is a coverage set for (T,c) such that for all
 * coverage sets M' for (T,c), we have c(M) <= c(M').
 */

#ifndef __MIN_COVERAGE_H__
#define __MIN_COVERAGE_H__

#include <functional>
#include <set>

namespace MinCoverage {

  /* Returns a min-coverage set for (T,cost). */
  template<typename S>
  std::set<S>
  min_coverage(const std::set<std::set<S> > &T,
               const std::function<int(const S&)> &cost);

  /* Returns a min-coverage set for (T,unit), where unit assigns the
   * cost 1 to each element. */
  template<typename S>
  std::set<S>
  min_coverage(const std::set<std::set<S> > &T);

  /* Returns all min-coverage sets for (T,cost). */
  template<typename S>
  std::set<std::set<S> >
  min_coverage_all(const std::set<std::set<S> > &T,
                   const std::function<int(const S&)> &cost);

  /* Returns all min-coverage sets for (T,unit), where unit assigns
   * the cost 1 to each element. */
  template<typename S>
  std::set<std::set<S> >
  min_coverage_all(const std::set<std::set<S> > &T);

  /* Returns all coverage sets for (T,unit) which are minimal with
   * respect to set inclusion.
   */
  template<typename S>
  std::set<std::set<S> >
  subset_min_coverage_all(const std::set<std::set<S> > &T);

  void test();
};

#include "min_coverage.tcc"

#endif
