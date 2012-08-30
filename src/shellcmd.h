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

#ifndef __SHELLCMD_H__
#define __SHELLCMD_H__

#include <stdexcept>

namespace ShellCmd{

  class ShCmdException : public std::exception{
  public:
    ShCmdException(std::string m) : msg(m) {};
    virtual ~ShCmdException() throw() {};
    virtual const char *what() throw() { return ("ShCmd: "+msg).c_str(); };
  private:
    std::string msg;
  };

  /* Execute cmd via 'sh -c'.
   * Return what is outputted to stdout.
   * If return_status is non-null, it will be assigned the return status of cmd. */
  std::string exec(std::string cmd, int *return_status = 0) throw(ShCmdException*);

};


#endif // __SHELLCMD_H__
