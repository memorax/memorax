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

#ifndef __TSO_VAR_H__
#define __TSO_VAR_H__

#include "lang.h"
#include "predicates.h"
#include "syntax_string.h"

class TsoVar{
public:
  TsoVar(); // Initializes nothing
  TsoVar(Lang::NML nml); // Memory location in memory
  static TsoVar reg(int r, int p); // Register r of process p
  static TsoVar msg(int m, int p); // Message m in the channel of process p
  static TsoVar fresh_tmp(); // A fresh temporary variable that has never before not previously been returned by this function
  enum Type { REG, MEMLOC, MSG, TMP };
  Type get_type() const throw() { return type; };
  // pre: type in {REG, MSG}
  int get_process() const throw() { return pid; };
  // pre: type == REG
  int get_reg() const throw() { return reg_msg; };
  // pre: type == MSG
  int get_msg() const throw() { return reg_msg; };
  // pre: type == MEMLOC
  Lang::NML get_memloc() const throw() { return nml; };
  bool operator==(const TsoVar &) const throw();
  bool operator!=(const TsoVar &tv) const throw() { return !(*this == tv); };
  bool operator<(const TsoVar &) const throw();
  bool operator>(const TsoVar &) const throw();
  std::string to_raw_string() const throw();
  /* Uses variable names from m for pretty string representation. */
  std::string to_string(const std::function<std::string(int,int)> &regts,
                        const std::function<std::string(Lang::NML)> &nmlts) const throw();
  class StringReprError : public std::exception{
  public:
    StringReprError(std::string m) : msg("StringReprError: "+m) {};
    virtual ~StringReprError() throw() {};
    const char *what() const throw() { return msg.c_str(); };
  private:
    std::string msg;
  };
  static TsoVar from_string(std::string s) throw(StringReprError*);
private:
#ifndef NDEBUG
  bool initialized;
#endif
  int compare(const TsoVar &) const throw(); // Used for operators ==, <, >
  Type type;
  int pid; // Relevant only for types REG and MSG
  /* Relevant only for types REG, MSG and TMP.
   * Names the register if type == REG, and the message if type == MSG.
   * Is an id for the temporary variable if type == TMP.
   */
  int reg_msg;
  static int next_tmp_id; // Counts up from 0
  Lang::NML nml; // Relevant only for type MEMLOC
};

#endif // __TSO_VAR_H__
