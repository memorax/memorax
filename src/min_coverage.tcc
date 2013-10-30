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

#include <algorithm>
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
    const std::set<S> &get_set() const;
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
  const std::set<S> &CandSet<S>::get_set() const{
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

  /* A Preprocessed(T,cost) object holds a preprocessed, simpler
   * version of the problem of min coverage for (T,cost). Instead of
   * keeping all S elements in T, the simpler version keeps track only
   * of equivalence classes of S elements. An equivalence class is
   * identified by an integer.
   *
   * The equivalence relation considered by Preprocessed is that s ~ t
   * iff s and t occur in precisely the same sets in T, and cost(s) ==
   * cost(t).
   */
  template<typename S>
  class Preprocessed{
  public:
    Preprocessed(const std::set<std::set<S> > &T,
                 const std::function<int(const S&)> &cost);
    /* get_coverage_map(T') where T' is the preprocessed T. */
    const std::map<int,std::set<int> > &get_coverage_map() const { return cov_map_i; };
    /* A vector v such that the elements in v are precisely the sets
     * in T', where T' is the preprocessed T.
     */
    const std::vector<std::set<int> > &get_Tvec() const { return Tvec_i; };
    /* The cost function f such that f(i) = cost(s) where s is some
     * element in the equivalence class i, and cost is the original
     * cost function over S.
     */
    const std::function<int(const int&)> &get_cost() const { return cost_i; };
    /* Returns the set U of all sets Ui such that there is a set Ti in
     * T that can be translated to Ui by replacing each i in Ti by
     * some s which is in the equivalence class i.
     */
    std::set<std::set<S> > translate_back(const std::set<std::set<int> > &T) const;
    /* Returns a singleton set {U} where U is a member of
     * translate_back(T), if T contains at least one
     * element. Otherwise returns the empty set. */
    std::set<std::set<S> > translate_back_one(const std::set<std::set<int> > &T) const;
  private:
    std::map<int,std::set<int> > cov_map_i;
    std::vector<std::set<int> > Tvec_i;
    /* trans_i2S[i] is the set of elements s in the equivalence class i. */
    std::vector<std::set<S> > trans_i2S;
    std::function<int(const int&)> cost_i;

    void translate_back(const std::set<int> &T,
                        std::set<std::set<S> > &TGT) const;
  };

  template<typename S>
  void Preprocessed<S>::translate_back(const std::set<int> &T,
                                       std::set<std::set<S> > &TGT) const{
    std::set<std::set<S> > *Ss = new std::set<std::set<S> >();
    Ss->insert(std::set<S>());

    for(int i : T){
      std::set<std::set<S> > *Ss2 = new std::set<std::set<S> >();
      for(auto it = trans_i2S[i].begin(); it != trans_i2S[i].end(); ++it){
        for(std::set<S> S2 : *Ss){
          S2.insert(*it);
          Ss2->insert(S2);
        }
      }
      delete Ss;
      Ss = Ss2;
    }

    TGT.insert(Ss->begin(),Ss->end());
    delete Ss;
  };

  template<typename S>
  std::set<std::set<S> > Preprocessed<S>::translate_back(const std::set<std::set<int> > &T) const{
    std::set<std::set<S> > T2;
    for(auto it = T.begin(); it != T.end(); ++it){
      translate_back(*it,T2);
    }
    return T2;
  };

  template<typename S>
  std::set<std::set<S> > Preprocessed<S>::translate_back_one(const std::set<std::set<int> > &T) const{
    if(T.empty()){
      return {};
    }

    std::set<S> U;
    for(int i : *T.begin()){
      assert(0 <= i && i < int(trans_i2S.size()));
      assert(trans_i2S[i].size());
      U.insert(*trans_i2S[i].begin());
    }
    return {U};
  };

  template<typename S>
  Preprocessed<S>::Preprocessed(const std::set<std::set<S> > &T,
                             const std::function<int(const S&)> &cost)
    : cost_i([](const int&){return 0;}) {

    /* An id_t (c,U) is the identifying parts of the equivalence class
     * of some S s: c is the cost of s, U is a set of integers
     * identifying which sets Ti in T contain s.
     */
    typedef std::pair<int,std::set<int> > id_t;

    std::map<S,id_t> s_ids;
    {
      int i = 0;
      for(auto it = T.begin(); it != T.end(); ++it){
        for(auto it2 = it->begin(); it2 != it->end(); ++it2){
          s_ids[*it2].first = cost(*it2);
          s_ids[*it2].second.insert(i);
        }
        ++i;
      }
    }

    std::map<id_t,std::set<S> > ids_s; // Inverse of s_ids
    for(auto it = s_ids.begin(); it != s_ids.end(); ++it){
      ids_s[it->second].insert(it->first);
    }

    std::map<S,int> S2i;
    for(auto it = ids_s.begin(); it != ids_s.end(); ++it){
      for(auto it2 = it->second.begin(); it2 != it->second.end(); ++it2){
        S2i[*it2] = trans_i2S.size();
      }
      trans_i2S.push_back(it->second);
    }

    Tvec_i.resize(T.size());
    {
      int i = 0;
      for(auto it = T.begin(); it != T.end(); ++it){
        for(auto it2 = it->begin(); it2 != it->end(); ++it2){
          Tvec_i[i].insert(S2i[*it2]);
        }
        ++i;
      }
    }

    cov_map_i = MinCoverage::get_coverage_map(Tvec_i);

    std::vector<int> costs(trans_i2S.size());
    for(unsigned i = 0; i < trans_i2S.size(); ++i){
      assert(trans_i2S[i].size());
      costs[i] = cost(*trans_i2S[i].begin());
    }
    cost_i =
      [costs](const int &i){
      return costs[i];
    };
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

    Preprocessed<S> pp(T,cost);

    const std::vector<std::set<int> > &Tvec = pp.get_Tvec();
    const std::map<int,std::set<int> > &cov_map = pp.get_coverage_map();

    std::priority_queue<CandSet<int>,
                        std::vector<CandSet<int> >,
                        std::greater<CandSet<int> > > queue;

    queue.push(CandSet<int>(Tvec,cov_map,pp.get_cost()));

    std::set<std::set<int> > res;
    bool found_first = false; // Have we found the first set?
    int cost_res = -1; // If found_first, then cost_res is the cost of each individual set in res

    while(!queue.empty() &&
          (!get_all || !found_first || queue.top().get_cost() == cost_res) &&
          (get_all || !found_first)){
      CandSet<int> cs = queue.top();
      queue.pop();

      int i = cs.get_uncovered_Ti();
      if(i == -1){ // Total coverage for cs
        found_first = true;
        cost_res = cs.get_cost();
        res.insert(cs.get_set());
      }else{ // Increase coverage
        assert(i >= 0 && i < int(Tvec.size()));

        for(auto it = Tvec[i].begin(); it != Tvec[i].end(); ++it){
          CandSet<int> cs2 = cs;
          cs2.insert(*it);
          queue.push(cs2);
        }
      }
    }

    if(get_all){
      return pp.translate_back(res);
    }else{
      return pp.translate_back_one(res);
    }
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

  template<typename S>
  std::set<std::set<S> >
  subset_min_coverage_all(const std::set<std::set<S> > &T){

    Preprocessed<S> pp(T,[](const S&){return 0;}); // Dummy cost
    const std::vector<std::set<int> > Tvec = pp.get_Tvec();
    const std::map<int,std::set<int> > &cov_map = pp.get_coverage_map();

    std::set<std::set<int> > mcs;

    std::queue<CandSet<int> > candidates;
    candidates.push(CandSet<int>(Tvec,cov_map,
                                 [](const int&){return 0;})); // Dummy cost

    /* Check whether C is a superset of some set in mcs. */
    std::function<bool(const CandSet<int>&)> is_subsumed =
      [&mcs](const CandSet<int> &C){
      const std::set<int> &cs = C.get_set();
      for(auto it = mcs.begin(); it != mcs.end(); ++it){
        if(std::includes(cs.begin(),cs.end(),
                         it->begin(),it->end())){
          return true;
        }
      }
      return false;
    };

    while(!candidates.empty()){
      const CandSet<int> &C = candidates.front();
      if(!is_subsumed(C)){
        int i = C.get_uncovered_Ti();
        if(i == -1){
          // C is a subset minimal coverage set
          mcs.insert(C.get_set());
        }else{
          for(auto it = Tvec[i].begin(); it != Tvec[i].end(); ++it){
            CandSet<int> C2(C);
            C2.insert(*it);
            candidates.push(C2);
          }
        }
      }
      candidates.pop();
    }

    return pp.translate_back(mcs);
  };

};
