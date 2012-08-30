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

#include "tso_var.h"
#include <cstdio>

int TsoVar::next_tmp_id = 0;

TsoVar::TsoVar()
  : nml(Lang::NML::global(0))
{
#ifndef NDEBUG
  initialized = false;
#endif
};

TsoVar::TsoVar(Lang::NML nml)
  : type(MEMLOC), nml(nml)
{
#ifndef NDEBUG
  initialized = true;
#endif
}

TsoVar TsoVar::reg(int r, int p){
  TsoVar tv;
  tv.type = REG;
  tv.pid = p;
  tv.reg_msg = r;
#ifndef NDEBUG
  tv.initialized = true;
#endif
  return tv;
}

TsoVar TsoVar::msg(int m, int p){
  TsoVar tv;
  tv.type = MSG;
  tv.pid = p;
  tv.reg_msg = m;
#ifndef NDEBUG
  tv.initialized = true;
#endif
  return tv;
}

TsoVar TsoVar::fresh_tmp(){
  TsoVar tv;
  tv.type = TMP;
  tv.reg_msg = next_tmp_id;
#ifndef NDEBUG
  tv.initialized = true;
#endif
  next_tmp_id++;
  return tv;
}

bool TsoVar::operator==(const TsoVar &tv) const throw(){
  assert(initialized);
  return compare(tv) == 0;
}

bool TsoVar::operator<(const TsoVar &tv) const throw(){
  assert(initialized);
  return compare(tv) < 0;
}

bool TsoVar::operator>(const TsoVar &tv) const throw(){
  assert(initialized);
  return compare(tv) > 0;
}

std::string TsoVar::to_raw_string() const throw(){
  assert(initialized);
  std::stringstream ss;
  switch(type){
  case REG:
    ss << "P" << pid << ":reg" << reg_msg;
    break;
  case MSG:
    ss << "P" << pid << ":msg" << reg_msg;
    break;
  case MEMLOC:
    ss << "var:" << nml;
    break;
  case TMP:
    ss << "tmp#" << reg_msg;
    break;
  }
  return ss.str();
}

std::string TsoVar::to_string(const std::function<std::string(int,int)> &regts,
                              const std::function<std::string(Lang::NML)> &nmlts) const throw(){
  assert(initialized);
  std::stringstream ss;
  switch(type){
  case REG:
    ss << "P" << pid << ":" << regts(reg_msg,pid);
    break;
  case MSG:
    ss << "P" << pid << ":msg" << reg_msg;
    break;
  case MEMLOC:
    ss << nmlts(nml);
    break;
  case TMP:
    ss << "tmp#" << reg_msg;
    break;
  }
  return ss.str();
}

int TsoVar::compare(const TsoVar &tv) const throw(){
  assert(initialized);
  if(type < tv.type){
    return -1;
  }else if(type > tv.type){
    return 1;
  }else{
    switch(type){
    case MSG: // Use the same comparison as REG
    case REG:
      if(pid < tv.pid){
        return -1;
      }else if(pid > tv.pid){
        return 1;
      }else{
        if(reg_msg < tv.reg_msg){
          return -1;
        }else if(reg_msg > tv.reg_msg){
          return 1;
        }else{
          return 0;
        }
      }
    case MEMLOC:
      if(nml < tv.nml){
        return -1;
      }else if(nml == tv.nml){
        return 0;
      }else{
        return 1;
      }
    case TMP:
      if(reg_msg < tv.reg_msg){
        return -1;
      }else if(reg_msg > tv.reg_msg){
        return 1;
      }else{
        return 0;
      }
    }
  }
  /* gcc claims this is reachable... */
  assert(false);
  return 42;
}

TsoVar TsoVar::from_string(std::string s) throw(StringReprError*){
  int pid, reg, msg, id;
  if(sscanf(s.c_str(),"P%d:reg%d",&pid,&reg) == 2 && pid >= 0 && reg >= 0){
    return TsoVar::reg(reg,pid);
  }else if(sscanf(s.c_str(),"P%d:msg%d",&pid,&msg) == 2 && pid >= 0 && msg > 0){
    return TsoVar::msg(msg,pid);
  }else if(sscanf(s.c_str(),"var:gvar:%d",&id) == 1 && id >= 0){
    return TsoVar(Lang::NML::global(id));
  }else if(sscanf(s.c_str(),"var:lvar:P%d:%d",&pid,&id) == 2 && pid >= 0 && id >= 0){
    return TsoVar(Lang::NML::local(id,pid));
  }else if(sscanf(s.c_str(),"tmp#%d",&reg)){
    TsoVar tv;
    tv.type = TMP;
    tv.reg_msg = reg;
    return tv;
  }else{
    throw new StringReprError("Incomprehensible string \""+s+"\"");
  }
}
