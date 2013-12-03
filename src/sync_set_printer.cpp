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

#include "sync_set_printer.h"
#include "vecset.h"

#include <string>

namespace SyncSetPrinter{

  /* Compare Sync pointers using Sync::operator< */
  class SyncCmp{
  public:
    bool operator()(const Sync *a, const Sync *b) const{
      return *a < *b;
    };
  };

  typedef VecSet<std::string> salt_t;

  typedef std::set<salt_t> conj_t;

  typedef std::set<conj_t> dnf_t;
  typedef std::set<conj_t> char_t;

  typedef std::map<const Sync*,std::string,SyncCmp> names_t;

  /* For each synchronization in S, map it to a unique, short string name. */
  names_t name_syncs(const std::set<std::set<Sync*> > &S){
    std::set<const Sync*,SyncCmp> syncs;
    for(auto it = S.begin(); it != S.end(); ++it){
      for(const Sync *s : *it){
        syncs.insert(s);
      }
    }
    names_t m;
    std::string name = "A";
    for(const Sync *s : syncs){
      m[s] = name;
      int i;
      for(i = name.size()-1; i >= 0; --i){
        if(name[i] == 'Z'){
          name[i] = 'a';
          break;
        }else if(name[i] == 'z'){
          name[i] = 'A';
          // Do not break
        }else{
          ++name[i];
          break;
        }
      }
      if(i < 0){
        name = "A"+name;
      }
    }
    return m;
  };

  void print_sync_names(const names_t &names,
                        const Machine &m,
                        Log::redirection_stream &os,
                        Log::redirection_stream &json_os){
    os << "Totally " << names.size() << " unique synchronizations:\n";
    for(auto it = names.begin(); it != names.end(); ++it){
      os << it->second << ": ";
      it->first->print(m,os,json_os);
      os << "\n";
    }
  };

  std::string salt_to_string(const salt_t &salt, const names_t &names){
    std::string s;
    for(std::string sync : salt){
      if(s != "") s += "|";
      s+=sync;
    }
    if(salt.size() == 1){
      return s;
    }else{
      return "("+s+")";
    }
  };

  std::string conj_to_string(const conj_t &conj, const names_t &names){
    std::string s;
    for(auto it = conj.begin(); it != conj.end(); ++it){
      if(s != "") s += ", ";
      s += salt_to_string(*it,names);
    }
    return "{"+s+"}";
  };

  dnf_t merge_sync_sets(const std::set<std::set<Sync*> > &S,
                        const names_t &names,
                        Log::redirection_stream &os,
                        Log::redirection_stream &json_os){
    dnf_t S2;

    std::map<salt_t,char_t> characterization;
    for(auto it = S.begin(); it != S.end(); ++it){
      conj_t conj;
      for(auto it2 = it->begin(); it2 != it->end(); ++it2){
        conj.insert({names.at(*it2)});
      }
      S2.insert(conj);
      for(auto it2 = it->begin(); it2 != it->end(); ++it2){
        salt_t sgt; sgt.insert(names.at(*it2));
        conj.erase(sgt);
        characterization[sgt].insert(conj);
        conj.insert(sgt);
      }
    }

    bool done = false;
    while(!done){
      done = true;
      std::map<conj_t,salt_t> conj_repr;
      for(auto it = characterization.begin(); done && it != characterization.end(); ++it){
        salt_t a = it->first;
        for(auto it2 = it->second.begin(); done && it2 != it->second.end(); ++it2){
          if(conj_repr.count(*it2)){
            salt_t b = conj_repr[*it2];
            /* Merge a and b */
            salt_t ab = a; ab.insert(b);

            /* Compute intersection between characterizations of a and b */
            char_t ab_char;
            std::set_intersection(characterization[a].begin(),characterization[a].end(),
                                  characterization[b].begin(),characterization[b].end(),
                                  std::inserter(ab_char,ab_char.end()));

            for(auto it = ab_char.begin(); it != ab_char.end(); ++it){
              conj_t cnj = *it;
              characterization[a].erase(cnj);
              characterization[b].erase(cnj);
              for(auto it2 = it->begin(); it2 != it->end(); ++it2){
                cnj.erase(*it2);
                cnj.insert(a);
                assert(characterization[*it2].count(cnj));
                characterization[*it2].erase(cnj);
                cnj.erase(a);
                cnj.insert(b);
                assert(characterization[*it2].count(cnj));
                characterization[*it2].erase(cnj);
                cnj.erase(b);
                cnj.insert(ab);
                characterization[*it2].insert(cnj);
                cnj.erase(ab);
                cnj.insert(*it2);
              }
              cnj.insert(a);
              assert(S2.count(cnj));
              S2.erase(cnj);
              cnj.erase(a);
              cnj.insert(b);
              assert(S2.count(cnj));
              S2.erase(cnj);
              cnj.erase(b);
              cnj.insert(ab);
              assert(S2.count(cnj) == 0);
              S2.insert(cnj);
            }

            assert(characterization.count(ab) == 0);
            characterization[ab] = ab_char;

            done = false;
          }else{
            conj_repr[*it2] = a;
          }
        }
      }
    }

    return S2;
  };

  void print(const std::set<std::set<Sync*> > &S,
             const Machine &m,
             Log::redirection_stream &os,
             Log::redirection_stream &json_os){
    Log::result << "Found " << S.size() << " synchronization set";
    if(S.size() == 0){
      Log::result << "s.\n";
      Log::result << "\nNOTICE: This means that the program is unsafe regardless of fences!\n\n";
    }else{
      if(S.size() == 1){
        Log::result << ":\n";
      }else{
        Log::result << "s:\n";
      }

      names_t names = name_syncs(S);
      print_sync_names(names,m,os,json_os);

      dnf_t S2 = merge_sync_sets(S,names,os,json_os);
      int ctr = 0;
      os << "Synchronization sets:\n";
      for(auto it = S2.begin(); it != S2.end(); ++it){
        os << "#" << ctr << ": " << conj_to_string(*it,names) << "\n";
        ++ctr;
      }
    }
  };

}
