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

#include "shellcmd.h"
#include <iostream>
#include <cstdio>
#include <cerrno>

std::string ShellCmd::exec(std::string cmd,int *return_status) throw(ShCmdException*){
  FILE *f = popen(cmd.c_str(),"r");
  if(f == 0){
    /* Allocation error or
     * Error in pipe or fork
     */
    if(errno != 0){
      std::string s = "Error while running command '"+cmd+"'";
      perror(s.c_str());
    }
    throw new ShCmdException("Error (allocation, fork or pipe) while running command '"+cmd+"'");
  }

  std::string retval;
  char buf[101];
  int c;
  while((c = fread(buf,1,100,f)) != 0){
    buf[c] = '\0';
    retval += buf;
  }

  if(ferror(f)){
    pclose(f);
    throw new ShCmdException("Error while reading output from '"+cmd+"'");
  }

  int rstatus = pclose(f);
  if(return_status) *return_status = rstatus;
  return retval;
}

/*
// Test
int main(int argc, char *argv[]){

  std::cout << "Shell Command Test.\n";

  try{
    std::string s = ShellCmd::exec("ls");

    std::cout << s;
  }catch(std::exception *exc){
    std::cerr << exc->what() << std::endl;
  }

  return 0;
}
*/
