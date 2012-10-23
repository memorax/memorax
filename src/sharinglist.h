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

#ifndef __SHARING_LIST_H__
#define __SHARING_LIST_H__

#include <cstddef>
#include <iterator>
#include <stdexcept>


template<class T> class sharinglist{
private:
  class consbox;
public:
  typedef size_t size_type;
  class const_iterator;
  class SharingListError : public std::exception{
  public: 
    SharingListError(std::string m) : msg("SharingListError: "+m) {};
    ~SharingListError() throw() {};
    const char *what() const throw() { return msg.c_str(); };
  private:
    std::string msg;
  };
  /* Iterator types */
  class iterator{
  public:
    const T &operator*() const;
    T &operator*();
    const T *operator->() const;
    T *operator->();
    iterator &operator++();
    iterator operator++(int);
    bool operator==(const iterator &) const;
    bool operator!=(const iterator &iter) const { return !(*this == iter); };
  private:
    iterator(int i, consbox *it, sharinglist<T> *parent);
    sharinglist<T> *parent;
    int i;
    consbox *it;
    friend class sharinglist<T>;
    friend class sharinglist<T>::const_iterator;
#ifndef NDEBUG
    int version;
#endif
  };
  class const_iterator{
  public:
    const_iterator(const iterator &);
    const T &operator*() const;
    const T *operator->() const;
    const_iterator &operator++();
    const_iterator operator++(int);
    bool operator==(const const_iterator &) const;
    bool operator!=(const const_iterator &iter) const { return !(*this == iter); };
  private:
    const_iterator(int i,const consbox *it,const sharinglist<T> *parent);
    const sharinglist<T> *parent;
    int i;
    const consbox *it;
    friend class sharinglist<T>;
#ifndef NDEBUG
    int version;
#endif
  };

  /* Constructors */
  sharinglist();
  sharinglist(size_type n, const T &value = T());
  template<class InputIterator> sharinglist(InputIterator first, InputIterator last);
  sharinglist(iterator first, iterator last); // Uses sharing
  sharinglist(const sharinglist &sl);
  sharinglist &operator=(const sharinglist &sl);
  ~sharinglist();
  
  /* Iterators */
  iterator begin();
  const_iterator begin() const;
  iterator end();
  const_iterator end() const;

  /* Capacity */
  bool empty() const;
  size_type size() const;
  void resize(size_type sz,const T &c = T());

  /* Element access */
  T &front();
  const T &front() const;
  T &back();                   // Inefficient
  const T &back() const;       // Inefficient

  /* Modifiers */
  void push_front(const T &x);
  void pop_front();
  void push_back(const T &x);  // Inefficient
  void pop_back();
  iterator insert(iterator position, const T &x);
  iterator erase(iterator position);
  void clear();

  /* Comparison */
  /* First compares lists by length, then lexicographically if they
   * have the same length. */
  bool operator==(const sharinglist &sl) const;
  bool operator!=(const sharinglist &sl) const { return !(*this == sl); };
  bool operator<(const sharinglist &sl) const;

private:
  consbox *list;
  int *ptr_count;
  int sz;
#ifndef NDEBUG
  int version;
#endif
};

#include "sharinglist.tcc"

#endif // __SHARING_LIST_H__
