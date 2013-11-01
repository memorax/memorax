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

#include "vqueue.h"

#include <algorithm>
#include <cassert>
#include <limits>
#include <map>
#include <queue>
#include <vector>
#include <sstream>
#include <stdexcept>

namespace MinCoverage{

  /* An IVSGenerator represents an indexed set of VecSet<int>. The
   * elements may or may not be computed at demand.
   */
  class IVSGenerator {
  public:
    virtual ~IVSGenerator() {};
    /* Returns the i:th VecSet<int> in the set.
     *
     * Pre: has_index(i)
     */
    virtual VecSet<int> operator[](int i) = 0;
    /* Returns true iff i >= 0 and there exists at least i+1 elements
     * in the set.
     *
     * A call to has_index(i) may trigger much
     * computation. Particularly if i is an index which is much larger
     * than any previously requested index.
     */
    virtual bool has_index(int i) = 0;
    virtual bool empty() { return !has_index(0); };
  };

  /* A VecIVSGenerator is a simple IVSGenerator, where the set is kept
   * as a precomputed vector.
   */
  class VecIVSGenerator : public IVSGenerator{
  public:
    /* Let the set be the one associated with the range [begin,end). */
    template<typename ITER>
    VecIVSGenerator(ITER begin, ITER end) : v(begin,end) {};
    virtual ~VecIVSGenerator() {};
    virtual VecSet<int> operator[](int i){ return v[i]; };
    virtual bool has_index(int i) { return 0 <= i && i < int(v.size()); };
  private:
    std::vector<VecSet<int> > v;
  };

  /* A sol_iterator<S> i is a forward input iterator over min coverage
   * solutions. The value of *i is a VecSet<S>. The iterator contains
   * sufficient data to compute solutions on demand, without relying
   * on an external solution set.
   *
   * Technically: A sol_iterator<S> i contains a complete set of
   * solutions to the smaller min_coverage problem where only
   * equivalence classes of S elements are considered, as opposed to
   * considering each S element separately. The iterator translates
   * such solutions of the smaller problem to solutions of the larger
   * problem on demand. This has the benefit of not having to compute
   * or keep in memory the entire set of solutions to the larger
   * problem, which may be exponential in the size of the set of
   * solutions to the smaller problem.
   */
  template<typename S>
  class sol_iterator{
  public:
    typedef VecSet<S> value_type;
    typedef std::input_iterator_tag iterator_category;
    typedef void difference_type;
    typedef VecSet<S>& reference;
    typedef VecSet<S>* pointer;

    /* An end iterator matching any other end iterator. */
    sol_iterator();
    /* An iterator to the beginning of the set of larger solutions,
     * corresponding to the set of T smaller solutions.
     *
     * A larger solution s is said to correspond to a smaller solution
     * t iff s == {f(e) | e in t} for some function f such that f(e)
     * is in trans[e] for all e in t.
     *
     * Pre: For all Ti in T, and all i in Ti, trans[i] is defined and
     * non-empty.
     *
     * Pre: For all distinct 0 <= i,j < trans.size(), we have that
     * trans[i] is disjunct from trans[j].
     */
    sol_iterator(const std::set<VecSet<int> > &T,
                 const std::vector<VecSet<S> > &trans);
    /* (Takes ownership of T.) */
    sol_iterator(IVSGenerator *T,
                 const std::vector<VecSet<S> > &trans);
    sol_iterator(const sol_iterator&);
    ~sol_iterator();
    sol_iterator<S> &operator=(const sol_iterator&);
    bool operator==(const sol_iterator&) const;
    bool operator!=(const sol_iterator &it) const { return !(*this == it); };
    const VecSet<S> &operator*() const;
    const VecSet<S> *operator->() const;
    sol_iterator<S> operator++(int); // postfix
    sol_iterator<S> &operator++(); // prefix
  private:
    /* The set of smaller solutions.
     *
     * Shared between copies of the iterator.
     *
     * T is null iff this is an end iterator.
     */
    IVSGenerator *T;
    /* A vector representation of the possible translations from
     * smaller solutions to larger.
     *
     * Shared between copies of the iterator.
     *
     * trans is null iff T is null.
     */
    std::vector<std::vector<S> > *trans;
    /* Reference counter for T and itself. */
    int *ref_count;
    /* We are currently considering the smaller solution (*T)[T_i]. */
    int T_i;
    /* We are currently considering the tc:th translation of
     * (*T)[T_i] */
    typedef int tc_type;
    tc_type tc;
    /* The current solution. */
    VecSet<S> v;
    /* Is the current translation of (*T)[T_i] the last one?
     * Updated by update_v. */
    bool last_tc;

    void release_T();
    void update_v();
  };

  template<typename S>
  sol_iterator<S>::sol_iterator(){
    T = 0;
    trans = 0;
    ref_count = 0;
  };

  template<typename S>
  sol_iterator<S>::sol_iterator(const std::set<VecSet<int> > &T,
                                const std::vector<VecSet<S> > &trans){
    if(T.empty()){
      // End iterator, because the set is empty
      this->T = 0;
      this->trans = 0;
      this->ref_count = 0;
    }else{
      this->T = new VecIVSGenerator(T.begin(),T.end());
      this->trans = new std::vector<std::vector<S> >(trans.size());
      for(unsigned i = 0; i < trans.size(); ++i){
        (*this->trans)[i].insert((*this->trans)[i].begin(),trans[i].begin(),trans[i].end());
      }
      ref_count = new int(1);
      T_i = 0;
      tc = 0;
      update_v();
    }
  };

  template<typename S>
  sol_iterator<S>::sol_iterator(IVSGenerator *T,
                                const std::vector<VecSet<S> > &trans){
    if(T->empty()){
      // End iterator, because the set is empty
      delete T;
      this->T = 0;
      this->trans = 0;
      this->ref_count = 0;
    }else{
      this->T = T;
      this->trans = new std::vector<std::vector<S> >(trans.size());
      for(unsigned i = 0; i < trans.size(); ++i){
        (*this->trans)[i].insert((*this->trans)[i].begin(),trans[i].begin(),trans[i].end());
      }
      ref_count = new int(1);
      T_i = 0;
      tc = 0;
      update_v();
    }
  };

  template<typename S>
  sol_iterator<S>::sol_iterator(const sol_iterator<S> &it){
    T = it.T;
    trans = it.trans;
    ref_count = it.ref_count;
    if(T) ++*ref_count;
    T_i = it.T_i;
    tc = it.tc;
    last_tc = it.last_tc;
    v = it.v;
  };

  template<typename S>
  void sol_iterator<S>::release_T(){
    if(T){
      --*ref_count;
      if(*ref_count == 0){
        delete T;
        delete trans;
        delete ref_count;
      }
    }
  };

  template<typename S>
  sol_iterator<S>::~sol_iterator(){
    release_T();
  };

  template<typename S>
  sol_iterator<S> &sol_iterator<S>::operator=(const sol_iterator<S> &it){
    if(this != &it){
      if(it.ref_count) ++*it.ref_count;
      release_T();
      T = it.T;
      trans = it.trans;
      ref_count = it.ref_count;
      T_i = it.T_i;
      tc = it.tc;
      last_tc = it.last_tc;
      v = it.v;
    }
    return *this;
  };

  template<typename S>
  bool sol_iterator<S>::operator==(const sol_iterator<S> &it) const{
    if(T == 0) return it.T == 0;
    if(it.T == 0) return false;
    return T_i == it.T_i && tc == it.tc;
  };

  template<typename S>
  void sol_iterator<S>::update_v(){
    assert(T != 0);
    v.clear();
    int tc_rem = tc;
    last_tc = true;
    for(unsigned i = 0; i < (*T)[T_i].size(); ++i){
      const std::vector<S> &tv = (*trans)[(*T)[T_i][i]];
      assert(tv.size());
      int tvi = tc_rem % tv.size();
      if(tvi != int(tv.size())-1){
        last_tc = false;
      }
      v.insert(tv[tvi]);
      tc_rem /= tv.size();
    }
    assert(tc_rem == 0);
  };

  template<typename S>
  const VecSet<S> &sol_iterator<S>::operator*() const{
    return v;
  };

  template<typename S>
  const VecSet<S> *sol_iterator<S>::operator->() const{
    return &v;
  };

  template<typename S>
  sol_iterator<S> sol_iterator<S>::operator++(int){
    sol_iterator<S> oldz(*this);
    ++*this;
    return oldz;
  };

  template<typename S>
  sol_iterator<S> &sol_iterator<S>::operator++(){
    if(T){
      if(last_tc){
        /* Go to next element in T */
        tc = 0;
        ++T_i;
        if(!T->has_index(T_i)){
          /* Become end iterator */
          assert(T->has_index(T_i-1));
          release_T();
          T = 0;
          trans = 0;
          ref_count = 0;
        }
      }else{
        /* Go to next translation of (*T)[T_i]. */
        if(tc == std::numeric_limits<tc_type>::max()){
          throw new std::logic_error("MinCoverage::sol_iterator: Unable to increase iterator due to numeric overflow.");
        }
        ++tc;
      }
      if(T){
        update_v();
      }
    }
    return *this;
  };

  /* A subset of S that is a candidate for being a coverage set.
   */
  template<typename S>
  class CandSet{
  public:
    /* Constructs an empty candidate for (T,cost_fun)
     * cm should be a coverage map for T (See get_coverage_map below). */
    CandSet(const std::vector<VecSet<S> > &T,
            const std::map<S,VecSet<int> > &cm,
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
    const VecSet<S> &get_set() const;
    /* A total order on candidate sets.
     * Compares lexicographically (cost,set). */
    bool operator<(const CandSet&) const;
    bool operator>(const CandSet&) const;
    bool operator==(const CandSet&) const;

    std::string to_string() const;
  private:
    /* The set itself */
    VecSet<S> set;
    /* coverage[i] is true iff T[i] is covered by this set
     *
     * By "T[i] is covered" we mean that this->set contains some element
     * from T[i].
     */
    std::vector<bool> coverage;
    /* The total cost of this set. */
    int cost;
    /* The set of sets T */
    const std::vector<VecSet<S> > &T;
    /* coverage_map is get_coverage_map(T) */
    const std::map<S,VecSet<int> > &coverage_map;
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
  const VecSet<S> &CandSet<S>::get_set() const{
    return set;
  };

  template<typename S>
  CandSet<S>::CandSet(const std::vector<VecSet<S> > &T,
                      const std::map<S,VecSet<int> > &cm,
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
    const VecSet<int> &is = coverage_map.at(s);
    for(auto it = is.begin(); it != is.end(); ++it){
      coverage[*it] = true;
    }
    cost += cost_fun(s);
  };

  /* Returns a map m such that for each s contained in any set in T,
   * m[s] is the set of all indices i where s is in T[i].
   */
  template<typename S>
  std::map<S,VecSet<int> >
  get_coverage_map(const std::vector<VecSet<S> > &T){
    std::map<S,VecSet<int> > m;
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
    Preprocessed(const std::set<VecSet<S> > &T,
                 const std::function<int(const S&)> &cost);
    /* get_coverage_map(T') where T' is the preprocessed T. */
    const std::map<int,VecSet<int> > &get_coverage_map() const { return cov_map_i; };
    /* A vector v such that the elements in v are precisely the sets
     * in T', where T' is the preprocessed T.
     */
    const std::vector<VecSet<int> > &get_Tvec() const { return Tvec_i; };
    /* The cost function f such that f(i) = cost(s) where s is some
     * element in the equivalence class i, and cost is the original
     * cost function over S.
     */
    const std::function<int(const int&)> &get_cost() const { return cost_i; };
    /* Returns the set U of all sets Ui such that there is a set Ti in
     * T that can be translated to Ui by replacing each i in Ti by
     * some s which is in the equivalence class i.
     */
    std::pair<sol_iterator<S>,sol_iterator<S> >
    translate_back(const std::set<VecSet<int> > &T) const;
    const std::vector<VecSet<S> > &get_trans_i2S() const { return trans_i2S; };
  private:
    std::map<int,VecSet<int> > cov_map_i;
    std::vector<VecSet<int> > Tvec_i;
    /* trans_i2S[i] is the set of elements s in the equivalence class i. */
    std::vector<VecSet<S> > trans_i2S;
    std::function<int(const int&)> cost_i;
  };

  template<typename S>
  std::pair<sol_iterator<S>,sol_iterator<S> >
  Preprocessed<S>::translate_back(const std::set<VecSet<int> > &T) const{
    return {sol_iterator<S>(T,trans_i2S),sol_iterator<S>()};
  };

  template<typename S>
  Preprocessed<S>::Preprocessed(const std::set<VecSet<S> > &T,
                             const std::function<int(const S&)> &cost)
    : cost_i([](const int&){return 0;}) {

    /* An id_t (c,U) is the identifying parts of the equivalence class
     * of some S s: c is the cost of s, U is a set of integers
     * identifying which sets Ti in T contain s.
     */
    typedef std::pair<int,VecSet<int> > id_t;

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

    std::map<id_t,VecSet<S> > ids_s; // Inverse of s_ids
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
  std::pair<sol_iterator<S>,sol_iterator<S> >
  min_coverage_impl(const std::set<VecSet<S> > &T,
                    const std::function<int(const S&)> &cost,
                    bool get_all){

    Preprocessed<S> pp(T,cost);

    const std::vector<VecSet<int> > &Tvec = pp.get_Tvec();
    const std::map<int,VecSet<int> > &cov_map = pp.get_coverage_map();

    std::priority_queue<CandSet<int>,
                        std::vector<CandSet<int> >,
                        std::greater<CandSet<int> > > queue;

    queue.push(CandSet<int>(Tvec,cov_map,pp.get_cost()));

    std::set<VecSet<int> > res;
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

    return pp.translate_back(res);
  };

  template<typename S>
  VecSet<S>
  min_coverage(const std::set<VecSet<S> > &T,
               const std::function<int(const S&)> &cost){
    auto mcs = min_coverage_impl(T,cost,false);
    return *mcs.first;
  };

  template<typename S>
  VecSet<S>
  min_coverage(const std::set<VecSet<S> > &T){
    std::function<int(const S&)> unit_cost =
      [](const S&){ return 1; };
    return min_coverage(T,unit_cost);
  };

  template<typename S>
  std::pair<sol_iterator<S>,sol_iterator<S> >
  min_coverage_all(const std::set<VecSet<S> > &T,
                   const std::function<int(const S&)> &cost){
    return min_coverage_impl(T,cost,true);
  };

  template<typename S>
  std::pair<sol_iterator<S>,sol_iterator<S> >
  min_coverage_all(const std::set<VecSet<S> > &T){
    std::function<int(const S&)> unit_cost =
      [](const S&){ return 1; };
    return min_coverage_all(T,unit_cost);
  };

  class SubsetIVSGenerator : public IVSGenerator{
  public:
    SubsetIVSGenerator(const std::vector<VecSet<int> > &Tvec,
                       const std::map<int,VecSet<int> > &cov_map);
    virtual ~SubsetIVSGenerator() {};
    SubsetIVSGenerator(const SubsetIVSGenerator&) = delete;
    SubsetIVSGenerator &operator=(const SubsetIVSGenerator&) = delete;
    virtual VecSet<int> operator[](int i){
      generate(i);
      return mcs[i];
    };
    virtual bool has_index(int i){
      return generate(i);
    };
  private:
    bool generate(int i);
    std::vector<VecSet<int> > mcs;
    vqueue<CandSet<int> > candidates;
    std::vector<VecSet<int> > Tvec;
    std::map<int,VecSet<int> > cov_map;
    std::function<int(const int&)> dummy_cost;
    /* Check whether C is a superset of some set in mcs. */
    bool is_subsumed(const CandSet<int> &C){
      const VecSet<int> &cs = C.get_set();
      for(auto it = mcs.begin(); it != mcs.end(); ++it){
        if(std::includes(cs.begin(),cs.end(),
                         it->begin(),it->end())){
          return true;
        }
      }
      return false;
    };
  };

  template<typename S>
  std::pair<sol_iterator<S>,sol_iterator<S> >
  subset_min_coverage_all(const std::set<VecSet<S> > &T){

    Preprocessed<S> pp(T,[](const S&){return 0;}); // Dummy cost

    SubsetIVSGenerator *G =
      new SubsetIVSGenerator(pp.get_Tvec(),
                             pp.get_coverage_map());

    return {sol_iterator<S>(G,pp.get_trans_i2S()),
        sol_iterator<S>()};
  };

};
