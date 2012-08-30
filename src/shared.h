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

#ifndef __SHARED_H__
#define __SHARED_H__

template<class T> class shared{
public:
  shared(T *t) : ptr(t) {
    ptr_count = new int(1);
  };
  shared(const shared<T> &s){
    ptr = s.ptr;
    ptr_count = s.ptr_count;
    (*ptr_count)++;
  };
  shared &operator=(const shared<T> &s){
    if(this != &s){
      (*ptr_count)--;
      if(*ptr_count == 0){
        delete ptr;
        delete ptr_count;
      }
      ptr = s.ptr;
      ptr_count = s.ptr_count;
      (*ptr_count)++;
    }
    return *this;
  };
  ~shared() {
    (*ptr_count)--;
    if(*ptr_count == 0){
      delete ptr;
      delete ptr_count;
    }
  };
  operator T*() const{
    return ptr;
  };
  T *operator->() const{
    return ptr;
  };
  const T &operator*() const{
    return *ptr;
  };
  T &operator*(){
    return *ptr;
  };
private:
  T *ptr;
  int *ptr_count;
};

#endif // __SHARED_H__
