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

template<class T> class sharinglist<T>::consbox{
public:
  consbox(const T &v);
  consbox(const T &v, consbox *next, int *next_ptr_count);
  consbox(const consbox &cb);
  consbox &operator=(const consbox &cb);
  ~consbox();
  T v;
  consbox *next;
  int *ptr_count; // Refers to next
#ifndef NDEBUG
  static int consbox_count;
#endif
};

#ifndef NDEBUG
template<class T> int sharinglist<T>::consbox::consbox_count = 0;
#endif

template<class T> sharinglist<T>::consbox::consbox(const T &v) : v(v) {
  next = 0;
  ptr_count = 0;
#ifndef NDEBUG
  consbox_count++;
#endif
};

template<class T> sharinglist<T>::consbox::consbox(const T &v, consbox *next, int *next_ptr_count) 
  : v(v), next(next), ptr_count(next_ptr_count) {
  assert(next);
  assert(next_ptr_count);
  (*ptr_count)++;
#ifndef NDEBUG
  consbox_count++;
#endif
};

template<class T> sharinglist<T>::consbox::consbox(const consbox &cb) 
  : v(cb.v), next(cb.next), ptr_count(cb.ptr_count){
  if(next){
    assert(ptr_count);
    (*ptr_count)++;
  }
#ifndef NDEBUG
  consbox_count++;
#endif
}

template<class T> typename sharinglist<T>::consbox &sharinglist<T>::consbox::operator=(const consbox &cb){
  if(this != &cb){
    if(next){
      assert(ptr_count);
      (*ptr_count)--;
      if(*ptr_count == 0){
        delete next;
        delete ptr_count;
      }
    }
    v = cb.v;
    next = cb.next;
    ptr_count = cb.ptr_count;
    if(next){
      assert(ptr_count);
      (*ptr_count)++;
    }
  }
  return *this;
}

template<class T> sharinglist<T>::consbox::~consbox(){
  if(next){
    assert(ptr_count);
    (*ptr_count)--;
    if(*ptr_count == 0){
      delete next;
      delete ptr_count;
    }
  }
#ifndef NDEBUG
  consbox_count--;
#endif
}

template<class T> sharinglist<T>::iterator::iterator(int i, consbox *it, sharinglist<T> *parent)
  : parent(parent), i(i), it(it) {
#ifndef NDEBUG
  version = parent->version;
#endif
};

template<class T> const T &sharinglist<T>::iterator::operator*() const{
#ifndef NDEBUG
  if(version != parent->version){
    throw new SharingListError("Use of outdated iterator.");
  }
#endif
  return it->v;
};

template<class T> T &sharinglist<T>::iterator::operator*(){
#ifndef NDEBUG
  if(version != parent->version){
    throw new SharingListError("Use of outdated iterator.");
  }
#endif
  return it->v;
};

template<class T> const T *sharinglist<T>::iterator::operator->() const{
#ifndef NDEBUG
  if(version != parent->version){
    throw new SharingListError("Use of outdated iterator.");
  }
#endif
  return &it->v;
};

template<class T> T *sharinglist<T>::iterator::operator->(){
#ifndef NDEBUG
  if(version != parent->version){
    throw new SharingListError("Use of outdated iterator.");
  }
#endif
  return &it->v;
};

template<class T> typename sharinglist<T>::iterator &sharinglist<T>::iterator::operator++(){
#ifndef NDEBUG
  if(version != parent->version){
    throw new SharingListError("Use of outdated iterator.");
  }
#endif
  if(i < parent->sz){
    assert(it);
    i++;
    it = it->next;
  }
  return *this;
};

template<class T> typename sharinglist<T>::iterator sharinglist<T>::iterator::operator++(int){
#ifndef NDEBUG
  if(version != parent->version){
    throw new SharingListError("Use of outdated iterator.");
  }
#endif
  iterator copy(*this);
  ++(*this);
  return copy;
};

template<class T> bool sharinglist<T>::iterator::operator==(const iterator &iter) const {
#ifndef NDEBUG
  if(version != parent->version || iter.version != iter.parent->version){
    throw new SharingListError("Use of outdated iterator.");
  }
#endif
  return i == iter.i;
}

template<class T> sharinglist<T>::const_iterator::const_iterator(int i,const consbox *it,const sharinglist<T> *parent)
  : parent(parent), i(i), it(it) {
#ifndef NDEBUG
  version = parent->version;
#endif
};

template<class T> sharinglist<T>::const_iterator::const_iterator(const iterator &iter)
  : parent(iter.parent), i(iter.i), it(iter.i){
#ifndef NDEBUG
  version = iter.version;
#endif
}

template<class T> const T &sharinglist<T>::const_iterator::operator*() const{
#ifndef NDEBUG
  if(version != parent->version){
    throw new SharingListError("Use of outdated iterator.");
  }
#endif
  return it->v;
};

template<class T> const T *sharinglist<T>::const_iterator::operator->() const{
#ifndef NDEBUG
  if(version != parent->version){
    throw new SharingListError("Use of outdated iterator.");
  }
#endif
  return &it->v;
};

template<class T> typename sharinglist<T>::const_iterator &sharinglist<T>::const_iterator::operator++(){
#ifndef NDEBUG
  if(version != parent->version){
    throw new SharingListError("Use of outdated iterator.");
  }
#endif
  if(i < parent->sz){
    assert(it);
    i++;
    it = it->next;
  }
  return *this;
};

template<class T> typename sharinglist<T>::const_iterator sharinglist<T>::const_iterator::operator++(int){
#ifndef NDEBUG
  if(version != parent->version){
    throw new SharingListError("Use of outdated iterator.");
  }
#endif
  const_iterator copy(*this);
  ++(*this);
  return copy;
};

template<class T> bool sharinglist<T>::const_iterator::operator==(const const_iterator &iter) const {
#ifndef NDEBUG
  if(version != parent->version || iter.version != iter.parent->version){
    throw new SharingListError("Use of outdated iterator.");
  }
#endif
  return i == iter.i;
}

template<class T> typename sharinglist<T>::iterator sharinglist<T>::begin(){
  return iterator(0,list,this);
}

template<class T> typename sharinglist<T>::iterator sharinglist<T>::end(){
  return iterator(sz,0,this);
}

template<class T> typename sharinglist<T>::const_iterator sharinglist<T>::begin() const{
  return const_iterator(0,list,this);
}

template<class T> typename sharinglist<T>::const_iterator sharinglist<T>::end() const{
  return const_iterator(sz,0,this);
}

template<class T> sharinglist<T>::sharinglist(){
  list = 0;
  ptr_count = 0;
  sz = 0;
#ifndef NDEBUG
  version = 0;
#endif
};

template<class T> sharinglist<T>::sharinglist(const sharinglist<T> &sl)
  : list(sl.list), ptr_count(sl.ptr_count), sz(sl.sz) {
  if(list){
    assert(ptr_count);
    (*ptr_count)++;
  }
#ifndef NDEBUG
  version = 0;
#endif
};

template<class T> sharinglist<T>::sharinglist(size_type n, const T &value){
  list = 0;
  ptr_count = 0;
  for(int i = 0; i < int(n); i++){
    list = new consbox(value,list,ptr_count);
    ptr_count = new int(1);
  }
  sz = n;
#ifndef NDEBUG
  version = 0;
#endif
}

template<class T> template<class InputIterator> sharinglist<T>::sharinglist(InputIterator first, InputIterator last){
  if(first == last){
    list = 0;
    ptr_count = 0;
    sz = 0;
    return;
  }

  list = new consbox(*first);
  ptr_count = new int(1);
  sz = 1;
  consbox *end = list;
  first++;
  while(first != last){
    end->next = new consbox(*first);
    end->ptr_count = new int(1);
    end = end->next;
    sz++;
    first++;
  }
#ifndef NDEBUG
  version = 0;
#endif
}

template<class T> sharinglist<T>::sharinglist(iterator first, iterator last){
#ifndef NDEBUG
  if(first.version != first.parent->version || last.version != last.parent->version){
    throw new SharingListError("Use of outdated iterator.");
  }
#endif
  if(first == last){
    list = 0;
    ptr_count = 0;
    sz = 0;
    return;
  }
  
  /* Share */
  list = first.parent->list;
  ptr_count = first.parent->ptr_count;
  (*ptr_count)++;
  sz = last.i - first.i;
#ifndef NDEBUG
  version = 0;
#endif
}

template<class T> sharinglist<T> &sharinglist<T>::operator=(const sharinglist<T> &sl){
  if(this != &sl){
    (*ptr_count)--;
    if(*ptr_count == 0){
      delete ptr_count;
      delete list;
    }
    ptr_count = sl.ptr_count;
    list = sl.list;
    sz = sl.sz;
    (*ptr_count)++;
#ifndef NDEBUG
    version++;
#endif
  }
  return *this;
}

template<class T> sharinglist<T>::~sharinglist(){
  if(list){
    assert(ptr_count);
    (*ptr_count)--;
    if(*ptr_count == 0){
      delete list;
      delete ptr_count;
    }
  }
};

template<class T> typename sharinglist<T>::size_type sharinglist<T>::size() const{
  return sz;
};

template<class T> bool sharinglist<T>::empty() const{
  return sz == 0;
}

template<class T> void sharinglist<T>::push_front(const T &x){
  consbox *ncb = new consbox(x);
  ncb->next = list;
  ncb->ptr_count = ptr_count;
  list = ncb;
  ptr_count = new int(1);
  sz++;
#ifndef NDEBUG
  version++;
#endif
}

template<class T> void sharinglist<T>::pop_front(){
  if(sz == 0){
    throw new SharingListError("Empty list.");
  }
  assert(list && ptr_count);
  consbox *next = list->next;
  int *next_ptr_count = list->ptr_count;
  if(next_ptr_count){
    (*next_ptr_count)++;
  }
  (*ptr_count)--;
  if(*ptr_count == 0){
    delete ptr_count;
    delete list;
  }
  ptr_count = next_ptr_count;
  list = next;
  sz--;
#ifndef NDEBUG
  version++;
#endif
}

template<class T> void sharinglist<T>::pop_back(){
  if(sz == 0){
    throw new SharingListError("Empty list.");
  }
  assert(list && ptr_count);
  sz--;
  if(sz == 0){
    (*ptr_count)--;
    if(*ptr_count == 0){
      delete ptr_count;
      delete list;
    }
    ptr_count = 0;
    list = 0;
  }
#ifndef NDEBUG
  version++;
#endif
}

template<class T> void sharinglist<T>::push_back(const T &x){
#ifndef NDEBUG
  version++;
#endif
  if(sz == 0){
    assert(!list && !ptr_count);
    list = new consbox(x);
    ptr_count = new int(1);
    sz++;
    return;
  }

  /* Optimistically go to the end of the list and see if we can
   * push_back without copying */
  consbox *l = list;
  assert(list);
  for(int s = 1; s < sz; s++){
    assert(l->next);
    l = l->next;
  }
  if(l->next == 0){
    l->next = new consbox(x);
    l->ptr_count = new int(1);
    sz++;
    return;
  }else if(l->next->v == x){
    sz++;
    return;
  }

  /* No luck this time: We need to copy the list */
  int *new_ptr_count = new int(1);
  consbox *new_list = new consbox(list->v);
  consbox *nl = new_list;
  l = list;
  for(int s = 1; s < sz; s++){
    l = l->next;
    nl->ptr_count = new int(1);
    nl->next = new consbox(l->v);
    nl = nl->next;
  }
  nl->ptr_count = new int(1);
  nl->next = new consbox(x);
  sz++;

  (*ptr_count)--;
  if(*ptr_count == 0){
    delete ptr_count;
    delete list;
  }
  ptr_count = new_ptr_count;
  list = new_list;
  
}

template<class T> void sharinglist<T>::clear(){
#ifndef NDEBUG
  version++;
#endif
  sz = 0;
  if(list){
    assert(ptr_count);
    (*ptr_count)--;
    if(*ptr_count == 0){
      delete ptr_count;
      delete list;
    }
    ptr_count = 0;
    list = 0;
  }
}

template<class T> T &sharinglist<T>::front(){
  if(sz == 0){
    throw new SharingListError("Empty list.");
  }
  assert(list);
  return list->v;
}

template<class T> const T &sharinglist<T>::front() const {
  if(sz == 0){
    throw new SharingListError("Empty list.");
  }
  assert(list);
  return list->v;
}

template<class T> T &sharinglist<T>::back(){
  if(sz == 0){
    throw new SharingListError("Empty list.");
  }
  assert(list);
  consbox *l = list;
  for(int s = 1; s < sz; s++){
    assert(l->next);
    l = l->next;
  }
  return l->v;
}

template<class T> const T &sharinglist<T>::back() const {
  if(sz == 0){
    throw new SharingListError("Empty list.");
  }
  assert(list);
  consbox *l = list;
  for(int s = 1; s < sz; s++){
    assert(l->next);
    l = l->next;
  }
  return l->v;
}

template<class T> typename sharinglist<T>::iterator sharinglist<T>::insert(iterator position, const T &x){
#ifndef NDEBUG
  if(position.version != version){
    throw new SharingListError("Use of outdated iterator.");
  }
  if(position.parent != this){
    throw new SharingListError("Use of iterator into wrong list.");
  }
#endif
#ifndef NDEBUG
  version++;
#endif
  if(position.i == 0){
    push_front(x);
    return iterator(0,list,this);
  }

  consbox *new_list = new consbox(list->v);
  int *new_ptr_count = new int(1);

  consbox *to_copy = list;
  consbox *end = new_list;

  for(int i = 1; i < position.i; i++){
    to_copy = to_copy->next;
    end->ptr_count = new int(1);
    end->next = new consbox(to_copy->v);
    end = end->next;
  }

  end->next = new consbox(x);
  end->ptr_count = new int(1);
  sharinglist<T>::iterator retit(position.i,end->next,this);
  end = end->next;
  end->next = to_copy->next;
  end->ptr_count = to_copy->ptr_count;
  if(end->ptr_count){
    (*end->ptr_count)++;
  }

  (*ptr_count)--;
  if(*ptr_count == 0){
    delete ptr_count;
    delete list;
  }
  
  list = new_list;
  ptr_count = new_ptr_count;
  sz++;

  return retit;
}

template<class T> typename sharinglist<T>::iterator sharinglist<T>::erase(iterator position){
#ifndef NDEBUG
  if(position.version != version){
    throw new SharingListError("Use of outdated iterator.");
  }
  if(position.parent != this){
    throw new SharingListError("Use of iterator into wrong list.");
  }
#endif
#ifndef NDEBUG
  version++;
#endif
  if(position.i == 0){
    pop_front();
    return begin();
  }
  if(position.i == sz-1){
    pop_back();
    return end();
  }

  consbox *new_list = new consbox(list->v);
  int *new_ptr_count = new int(1);

  consbox *to_copy = list;
  consbox *end = new_list;

  for(int i = 1; i < position.i; i++){
    to_copy = to_copy->next;
    end->ptr_count = new int(1);
    end->next = new consbox(to_copy->v);
    end = end->next;
  }

  // To remove: to_copy->next

  end->next = to_copy->next->next;
  end->ptr_count = to_copy->next->ptr_count;
  if(end->ptr_count){
    (*end->ptr_count)++;
  }

  (*ptr_count)--;
  if(*ptr_count == 0){
    delete ptr_count;
    delete list;
  }

  list = new_list;
  ptr_count = new_ptr_count;
  sz--;

  return iterator(position.i,end->next,this);  
}

template<class T> void sharinglist<T>::resize(size_type new_sz,const T &c){
#ifndef NDEBUG
  version++;
#endif
  if(int(new_sz) == 0){
    clear();
    return;
  }
  if(int(new_sz) < sz){
    sz = new_sz;
    return;
  }
  while(sz < int(new_sz)){
    push_back(c);
  }
}

template<class T> bool sharinglist<T>::operator==(const sharinglist<T> &sl) const{
  if(sl.sz != sz){
    return false;
  }
  if(list == sl.list){
    return true;
  }
  consbox *x = list;
  consbox *y = sl.list;
  for(int i = 0; i < sz; i++){
    if(x->v != y->v){
      return false;
    }
    x = x->next;
    y = y->next;
  }
  return true;
}

template<class T> bool sharinglist<T>::operator<(const sharinglist<T> &sl) const{
  if(sl.sz != sz){
    return sz < sl.sz;
  }
  assert(sz == sl.sz);
  if(list == sl.list){
    return false;
  }
  consbox *x = list;
  consbox *y = sl.list;
  for(int i = 0; i < sz; i++){
    if(x->v < y->v){
      return true;
    }
    if(!(x->v == y->v)){ // x->v > y->v
      return false;
    }
    x = x->next;
    y = y->next;
  }
  return false;
}

#ifndef NDEBUG
#include <iostream>
#include <list>
inline void sharinglist_test0(){
  sharinglist<int> sl;
  assert(sl.size() == 0);
}

inline void sharinglist_test1(){
  sharinglist<int> sl;
  sl.push_front(1);
  sl.push_front(2);
  sl.push_front(5);
  assert(sl.size() == 3);
  sl.pop_front();
  sl.pop_front();
  assert(sl.size() == 1);
  sl.pop_front();
  assert(sl.size() == 0);
  try{
    sl.pop_front();
    assert(false);
  }catch(sharinglist<int>::SharingListError *exc){
    delete exc;
  }
}

inline void sharinglist_test2(){
  sharinglist<int> sl;
  sl.push_front(1);
  sl.push_front(2);
  sharinglist<int> sl2 = sl;
  sl.push_front(3);
  sharinglist<int>::iterator it = sl.begin();
  assert(*it == 3);

  it = sl2.begin();
  assert(*it == 2);

  sl2.push_front(4);
  it = sl2.begin();
  assert(*it == 4);

  it = sl.begin();
  assert(*it == 3);
}

inline void sharinglist_test3(){
  sharinglist<std::pair<int,int> > sl;
  sl.push_front(std::pair<int,int>(42,13));
  sharinglist<std::pair<int,int> >::iterator it = sl.begin();
  assert(it->first == 42 && it->second == 13);
}

inline void sharinglist_test4(){
  sharinglist<int> sl;
  std::list<int> l;
  sl.push_front(1); l.push_front(1);
  sl.push_front(2); l.push_front(2);
  sl.push_front(5); l.push_front(5);
  sl.push_front(13); l.push_front(13);

  sharinglist<int>::iterator slit = sl.begin();
  std::list<int>::iterator lit = l.begin();
  while(slit != sl.end()){
    assert(lit != l.end());
    assert(*slit == *lit);
    lit++;
    slit++;
  }
  assert(lit == l.end());
}

inline void sharinglist_test5(){
  sharinglist<int> sl;
  assert(sl.begin() == sl.end());
}

inline void sharinglist_test6(){
  sharinglist<int> sl;
  sl.push_front(1);
  sl.push_front(2);
  sl.push_front(3);
  sl.push_front(4);
  sl.push_front(5);

  sl.pop_back();
  sl.pop_back();

  std::list<int> l;
  l.push_front(3);
  l.push_front(4);
  l.push_front(5);

  sharinglist<int>::iterator slit = sl.begin();
  std::list<int>::iterator lit = l.begin();
  while(slit != sl.end()){
    assert(lit != l.end());
    assert(*slit == *lit);
    lit++;
    slit++;
  }
  assert(lit == l.end());
}

inline void sharinglist_test7(){
  sharinglist<int> sl;
  sl.push_front(1);
  sl.push_front(2);
  sl.push_front(3);
  sl.push_front(4);
  sl.push_front(5);

  sl.pop_back();
  sl.pop_back();
  sl.clear();
  assert(sl.empty());
  assert(sl.begin() == sl.end());
}

inline void sharinglist_test8(){
  sharinglist<int> sl;
  sl.push_front(1);
  assert(sl.front() == sl.back());
  assert(sl.front() == 1);
  sl.push_front(2);
  assert(sl.back() == 1);
  assert(sl.front() == 2);
  sl.push_front(3);
  assert(sl.back() == 1);
  assert(sl.front() == 3);
  sl.push_front(4);
  assert(sl.back() == 1);
  assert(sl.front() == 4);
  sl.push_front(5);
  assert(sl.back() == 1);
  assert(sl.front() == 5);
  sl.pop_back();
  assert(sl.back() == 2);
  assert(sl.front() == 5);
  assert(sl.size() == 4);
  
}

inline void sharinglist_test9(){
  sharinglist<int> sl;
  std::list<int> l;
  sl.push_back(1); l.push_back(1);
  sl.push_back(2); l.push_back(2);
  sl.push_back(5); l.push_back(5);
  sl.push_back(13); l.push_back(13);

  sharinglist<int>::iterator slit = sl.begin();
  std::list<int>::iterator lit = l.begin();
  while(slit != sl.end()){
    assert(lit != l.end());
    assert(*slit == *lit);
    lit++;
    slit++;
  }
  assert(lit == l.end());
}

inline void sharinglist_test10(){
  sharinglist<int> sl;
  std::list<int> l;
  sl.push_back(1); l.push_back(1);
  sl.push_back(2); l.push_back(2);
  sl.push_back(5); l.push_back(5);
  sl.push_back(13); l.push_back(13);

  sharinglist<int> sl2(sl);
  sl2.pop_back();
  sl2.pop_back();
  sl2.push_back(53);
  sl2.push_back(102);
  sl2.pop_back();
  sl2.push_back(666);
  std::list<int> l2;
  l2.push_back(1);
  l2.push_back(2);
  l2.push_back(53);
  l2.push_back(666);

  sharinglist<int>::iterator slit = sl.begin();
  std::list<int>::iterator lit = l.begin();
  while(slit != sl.end()){
    assert(lit != l.end());
    assert(*slit == *lit);
    lit++;
    slit++;
  }
  assert(lit == l.end());

  slit = sl2.begin();
  lit = l2.begin();
  while(slit != sl2.end()){
    assert(lit != l2.end());
    assert(*slit == *lit);
    lit++;
    slit++;
  }
  assert(lit == l2.end());
}

inline void sharinglist_test11(){
  sharinglist<int> sl;
  std::list<int> l;
  sl.push_back(1); l.push_back(1);
  sl.push_back(2); l.push_back(2);
  sl.push_back(5); l.push_back(5);
  sl.push_back(13); l.push_back(13);

  sharinglist<int> sl2(sl);
  sl2.push_back(53);
  sl2.push_back(102);
  sl2.pop_back();
  sl2.push_back(666);
  std::list<int> l2;
  l2.push_back(1);
  l2.push_back(2);
  l2.push_back(5);
  l2.push_back(13);
  l2.push_back(53);
  l2.push_back(666);

  sharinglist<int>::iterator slit = sl.begin();
  std::list<int>::iterator lit = l.begin();
  while(slit != sl.end()){
    assert(lit != l.end());
    assert(*slit == *lit);
    lit++;
    slit++;
  }
  assert(lit == l.end());

  slit = sl2.begin();
  lit = l2.begin();
  while(slit != sl2.end()){
    assert(lit != l2.end());
    assert(*slit == *lit);
    lit++;
    slit++;
  }
  assert(lit == l2.end());
}

inline void sharinglist_test12(){
  sharinglist<int> sl;
  std::list<int> l;
  sl.push_back(1); l.push_back(1);
  sl.push_back(2); l.push_back(2);
  sl.push_back(5); l.push_back(5);
  sl.push_back(13); l.push_back(13);

  sharinglist<int> sl2(sl);
  sl2.push_back(53);
  sl2.push_back(666);
  std::list<int> l2;
  l2.push_back(1);
  l2.push_back(2);
  l2.push_back(5);
  l2.push_back(13);
  l2.push_back(53);
  l2.push_back(666);

  sharinglist<int>::iterator slit = sl.begin();
  std::list<int>::iterator lit = l.begin();
  while(slit != sl.end()){
    assert(lit != l.end());
    assert(*slit == *lit);
    lit++;
    slit++;
  }
  assert(lit == l.end());

  slit = sl2.begin();
  lit = l2.begin();
  while(slit != sl2.end()){
    assert(lit != l2.end());
    assert(*slit == *lit);
    lit++;
    slit++;
  }
  assert(lit == l2.end());

  assert(&sl.front() == &sl2.front());
}

template<class T> bool sharinglist_test_equal(sharinglist<T> sl, std::list<T> l){
  typename sharinglist<T>::iterator slit = sl.begin();
  typename std::list<T>::iterator lit = l.begin();
  while(slit != sl.end()){
    if(lit == l.end() || *slit != *lit)
      return false;
    lit++;
    slit++;
  }
  return lit == l.end();
}

inline void sharinglist_test13(){
  sharinglist<int> sl;
  sl.push_back(1);
  sl.push_back(2);
  sl.push_back(3);
  sl.push_back(4);
  sharinglist<int>::iterator slit = sl.insert(sl.begin(),0);
  assert(*slit == 0);

  std::list<int> l;
  l.push_back(0);
  l.push_back(1);
  l.push_back(2);
  l.push_back(3);
  l.push_back(4);
  assert(sharinglist_test_equal(sl,l));

  slit = sl.insert(sl.end(),5);
  assert(*slit == 5);
  l.push_back(5);
  assert(sharinglist_test_equal(sl,l));

  sharinglist<int>::iterator it = sl.begin();
  ++(++(++it)); // it points to 3
  slit = sl.insert(it,25);
  assert(*slit == 25);


  l.clear();
  l.push_back(0);
  l.push_back(1);
  l.push_back(2);
  l.push_back(25);
  l.push_back(3);
  l.push_back(4);
  l.push_back(5);
  assert(sharinglist_test_equal(sl,l));
}

inline void sharinglist_test14(){
  sharinglist<int> sl;
  sl.push_back(0);
  sl.push_back(1);
  sl.push_back(2);
  sl.push_back(3);
  sl.push_back(4);
  sl.push_back(5);
  sharinglist<int>::iterator slit = sl.erase(sl.begin());
  assert(*slit == 1);

  std::list<int> l;
  l.push_back(1);
  l.push_back(2);
  l.push_back(3);
  l.push_back(4);
  l.push_back(5);
  assert(sharinglist_test_equal(sl,l));
}

inline void sharinglist_test15(){
  sharinglist<int> sl;
  sl.push_back(0);
  sl.push_back(1);
  sl.push_back(2);
  sl.push_back(3);
  sl.push_back(4);
  sl.push_back(5);
  sharinglist<int>::iterator slit = sl.begin();
  ++slit;
  ++slit;
  ++slit;
  ++slit;
  ++slit; // slit points to 5
  slit = sl.erase(slit);
  assert(slit == sl.end());

  std::list<int> l;
  l.push_back(0);
  l.push_back(1);
  l.push_back(2);
  l.push_back(3);
  l.push_back(4);
  assert(sharinglist_test_equal(sl,l));
}

inline void sharinglist_test16(){
  sharinglist<int> sl;
  sl.push_back(0);
  sl.push_back(1);
  sl.push_back(2);
  sl.push_back(3);
  sl.push_back(4);
  sl.push_back(5);
  sharinglist<int>::iterator slit = sl.begin();
  ++slit;
  ++slit;
  ++slit; // slit points to 3
  slit = sl.erase(slit);
  assert(*slit == 4);

  std::list<int> l;
  l.push_back(0);
  l.push_back(1);
  l.push_back(2);
  l.push_back(4);
  l.push_back(5);
  assert(sharinglist_test_equal(sl,l));

  slit = sl.erase(slit);
  assert(*slit == 5);
  l.clear();
  l.push_back(0);
  l.push_back(1);
  l.push_back(2);
  l.push_back(5);
  assert(sharinglist_test_equal(sl,l));
}

inline void sharinglist_test17(){
  sharinglist<int> sl;
  sl.push_back(1);
  sl.push_back(2);
  sl.resize(7,7);
  
  std::list<int> l;
  l.push_back(1);
  l.push_back(2);
  l.push_back(7);
  l.push_back(7);
  l.push_back(7);
  l.push_back(7);
  l.push_back(7);
  assert(sharinglist_test_equal(sl,l));

  sl.resize(5);
  l.resize(5);
  assert(sharinglist_test_equal(sl,l));
}

inline void sharinglist_test18(){
  int a[5] = {0,1,2,32,4};
  sharinglist<int> sl(a,a+5);

  std::list<int> l(a,a+5);
  assert(sharinglist_test_equal(sl,l));

  sharinglist<int> sl2(sl.begin(),++(++(++(sl.begin()))));
  std::list<int> l2(a,a+3);
  assert(sharinglist_test_equal(sl2,l2));
}

inline void sharinglist_test19(){
  int a[5] = {0,1,2,32,4};
  sharinglist<int> sl(a,a+5);
  sharinglist<int>::iterator slit = ++sl.begin();
  sl.insert(slit,42);
  try{
    sl.insert(slit,666);
    assert(false);
  }catch(sharinglist<int>::SharingListError *exc){
    delete exc;
  }
}

inline void sharinglist_run_tests(){
  std::cout << " *** sharinglist test ***\n";
  sharinglist_test0();
  sharinglist_test1();
  sharinglist_test2();
  sharinglist_test3();
  sharinglist_test4();
  sharinglist_test5();
  sharinglist_test6();
  sharinglist_test7();
  sharinglist_test8();
  sharinglist_test9();
  sharinglist_test10();
  sharinglist_test11();
  sharinglist_test12();
  sharinglist_test13();
  sharinglist_test14();
  sharinglist_test15();
  sharinglist_test16();
  sharinglist_test17();
  sharinglist_test18();
  sharinglist_test19();
  std::cout << "\nSuccess!\n";
}
#endif
