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

#include <cassert>
#include <map>
#include <queue>
#include <vector>
#include <sstream>

namespace MinCoverage{

  /* A subset of S that is a candidate for being a coverage set.
   */
  template<typename S>
  class CandSet{
  public:
    /* Constructs an empty candidate for (T,cost_fun)
     * cm should be a coverage map for T (See get_coverage_map below). */
    CandSet(const std::vector<std::set<S> > &T,
            const std::map<S,std::set<int> > &cm,
            const std::function<int(const S&)> &cost_fun);
    CandSet(const CandSet&) = default;
    CandSet &operator=(const CandSet&);
    ~CandSet() = default;
    /* Insert s into this candidate */
    void insert(const S &s);
    /* If there is a Ti that is not covered by this candidate, then
     * such an i is returned. Otherwise -1 is returned.
     */
    int get_uncovered_Ti() const;
    /* Returns true iff this candidate covers all sets in T. */
    bool total_coverage() const;
    /* Returns the cost of this candidate */
    int get_cost() const;
    /* Returns the candidate set */
    std::set<S> get_set() const;
    /* A total order on candidate sets.
     * Compares lexicographically (cost,set). */
    bool operator<(const CandSet&) const;
    bool operator>(const CandSet&) const;
    bool operator==(const CandSet&) const;

    std::string to_string() const;
  private:
    /* The set itself */
    std::set<S> set;
    /* coverage[i] is true iff T[i] is covered by this set
     *
     * By "T[i] is covered" we mean that this->set contains some element
     * from T[i].
     */
    std::vector<bool> coverage;
    /* The total cost of this set. */
    int cost;
    /* The set of sets T */
    const std::vector<std::set<S> > &T;
    /* coverage_map is get_coverage_map(T) */
    const std::map<S,std::set<int> > &coverage_map;
    /* The cost function */
    const std::function<int(const S&)> &cost_fun;
  };

  template<typename S>
  std::string CandSet<S>::to_string() const{
    std::stringstream ss;
    ss << "<{";
    for(auto it = set.begin(); it != set.end(); ++it){
      if(it != set.begin()) ss << ", ";
      ss << *it;
    }
    ss << "},cost:" << cost << ">";
    return ss.str();
  };

  template<typename S>
  bool CandSet<S>::operator<(const CandSet &cs) const{
    return cost < cs.cost || 
      (cost == cs.cost && set < cs.set);
  };

  template<typename S>
  bool CandSet<S>::operator>(const CandSet &cs) const{
    return cost > cs.cost || 
      (cost == cs.cost && set > cs.set);
  };

  template<typename S>
  bool CandSet<S>::operator==(const CandSet &cs) const{
    return cost == cs.cost && set == cs.set;
  };

  template<typename S>
  std::set<S> CandSet<S>::get_set() const{
    return set;
  };

  template<typename S>
  CandSet<S>::CandSet(const std::vector<std::set<S> > &T,
                      const std::map<S,std::set<int> > &cm,
                      const std::function<int(const S&)> &cost_fun)
    : T(T), coverage_map(cm), cost_fun(cost_fun){
    coverage.resize(T.size(),false);
    cost = 0;
  };

  template<typename S>
  CandSet<S> &CandSet<S>::operator=(const CandSet &cs){
    assert(&T == &cs.T);
    assert(&cost_fun == &cs.cost_fun);
    if(&cs != this){
      set = cs.set;
      coverage = cs.coverage;
      cost = cs.cost;
    }
    return *this;
  }

  template<typename S>
  int CandSet<S>::get_uncovered_Ti() const{
    int i = 0;
    while(i < int(coverage.size())){
      if(!coverage[i]) return i;
      ++i;
    }
    return -1;
  };

  template<typename S>
  bool CandSet<S>::total_coverage() const{
    return get_uncovered_Ti() == -1;
  };

  template<typename S>
  int CandSet<S>::get_cost() const{
    return cost;
  };

  template<typename S>
  void CandSet<S>::insert(const S &s){
    set.insert(s);
    const std::set<int> &is = coverage_map.at(s);
    for(auto it = is.begin(); it != is.end(); ++it){
      coverage[*it] = true;
    }
    cost += cost_fun(s);
  };

  /* Returns a map m such that for each s contained in any set in T,
   * m[s] is the set of all indices i where s is in T[i].
   */
  template<typename S> 
  std::map<S,std::set<int> > 
  get_coverage_map(const std::vector<std::set<S> > &T){
    std::map<S,std::set<int> > m;
    for(unsigned i = 0; i < T.size(); ++i){
      for(auto it = T[i].begin(); it != T[i].end(); ++it){
        m[*it].insert(i);
      }
    }
    return m;
  };

  /* Returns a set of min-coverage sets for (T,cost).
   *
   * If get_all, then the returned set contains all min-coverage sets
   * for (T,cost), otherwise it contains exactly one min-coverage set
   * for (T,cost).
   */
  template<typename S> 
  std::set<std::set<S> >
  min_coverage_impl(const std::set<std::set<S> > &T,
                    const std::function<int(const S&)> &cost,
                    bool get_all){
    
    std::vector<std::set<S> > Tvec(T.begin(),T.end());
    const std::map<S,std::set<int> > &cov_map = get_coverage_map(Tvec);

    std::priority_queue<CandSet<S>,
                        std::vector<CandSet<S> >,
                        std::greater<CandSet<S> > > queue;

    queue.push(CandSet<S>(Tvec,cov_map,cost));

    std::set<std::set<S> > res;
    bool found_first = false; // Have we found the first set?
    int cost_res = -1; // If found_first, then cost_res is the cost of each individual set in res

    while(!queue.empty() &&
          (!get_all || !found_first || queue.top().get_cost() == cost_res) &&
          (get_all || !found_first)){
      CandSet<S> cs = queue.top();
      queue.pop();

      int i = cs.get_uncovered_Ti();
      if(i == -1){ // Total coverage for cs
        found_first = true;
        cost_res = cs.get_cost();
        res.insert(cs.get_set());
      }else{ // Increase coverage
        assert(i >= 0 && i < int(Tvec.size()));

        for(auto it = Tvec[i].begin(); it != Tvec[i].end(); ++it){
          CandSet<S> cs2 = cs;
          cs2.insert(*it);
          queue.push(cs2);
        }
      }
    }

    return res;
  };

  template<typename S> 
  std::set<S>
  min_coverage(const std::set<std::set<S> > &T,
               const std::function<int(const S&)> &cost){
    std::set<std::set<S> > mcs = min_coverage_impl(T,cost,false);
    assert(mcs.size() == 1);
    return *mcs.begin();
  };

  template<typename S> 
  std::set<S>
  min_coverage(const std::set<std::set<S> > &T){
    std::function<int(const S&)> unit_cost = 
      [](const S&){ return 1; };
    return min_coverage(T,unit_cost);
  };

  template<typename S>
  std::set<std::set<S> >
  min_coverage_all(const std::set<std::set<S> > &T,
                   const std::function<int(const S&)> &cost){
    return min_coverage_impl(T,cost,true);
  };

  template<typename S>
  std::set<std::set<S> >
  min_coverage_all(const std::set<std::set<S> > &T){
    std::function<int(const S&)> unit_cost = 
      [](const S&){ return 1; };
    return min_coverage_all(T,unit_cost);
  };

};
