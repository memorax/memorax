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

template<typename T>
vqueue<T>::vqueue(){
  start = end = 0;
};

template<typename T>
T vqueue<T>::front() const{
  return v[start];
};

template<typename T>
void vqueue<T>::pop(){
  start = (start+1)%v.size();
};

template<typename T>
void vqueue<T>::push(const T &t){
  if(v.size() == 0){
    v.resize(2,t);
  }else if((end+1)%int(v.size()) == start){
    /* Reallocate */
    int old_sz = v.size();
    v.resize(2*old_sz,t);
    if(end < start){
      for(int i = 0; i < end; ++i){
        v[old_sz+i] = v[i];
      }
      end = end + old_sz;
    }
  }
  v[end] = t;
  end = (end+1)%v.size();
};

template<typename T>
bool vqueue<T>::empty() const{
  return start == end;
};

