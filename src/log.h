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

#ifndef __LOG_H__
#define __LOG_H__

#include <iostream>
#include <stdexcept>

namespace Log{

  class redirection_stream{
  public:
    redirection_stream(std::ostream **os) : os(os) {};

    /* Stream operations */
    redirection_stream& operator<< (std::ostream& ( *pf )(std::ostream&)){
      if(*os) **os << pf;
      return *this;
    };
    template<typename T> redirection_stream& operator<< (const T &t){
      if(*os) **os << t;
      return *this;
    };

    std::ostream **os;

  private:
    redirection_stream(const redirection_stream &) : os(0) {
      throw new std::logic_error("redirection_stream: copy constructor");
    };
    redirection_stream &operator=(const redirection_stream &){
      throw new std::logic_error("redirection_stream: copy operator");
    };
  };

  enum loglevel_t {
    SILENT = 0,  // No output
    RESULTS = 1, // Print results
    MSG = 2,     // Print messages about the procedure during computation
    DEBUG = 3,   // Print messages verbosely during computation
    EXTREME = 4  // Print a ridiculous amount of messages 
  };

  /* There are three streams (primary, secondary and tertiary) to
   * which output is sent.
   * 
   * All output at a level lower than or equal to primary_loglevel is
   * sent to the primary stream (std::cout by default). All output at
   * a level greater than primary_loglevel but lower than or equal to
   * secondary_loglevel is sent to the secondary stream (quiet by
   * default). All other output is sent to the tertiary stream (quiet
   * by default).
   */
  loglevel_t get_primary_loglevel();
  void set_primary_loglevel(loglevel_t lv);
  loglevel_t get_secondary_loglevel();
  void set_secondary_loglevel(loglevel_t lv);

  /* Use these to print messages. */
  extern redirection_stream result;
  extern redirection_stream msg;
  extern redirection_stream debug;
  extern redirection_stream extreme;

  /* Use this to print warnings */
  extern redirection_stream warning;

  /* This stream is used to output json directives to downstream tools
   * (notably GUI) */
  extern redirection_stream json;

  /* This stream never prints */
  extern redirection_stream null;

  /* Redirect output to the stream pointed to by os. If os == 0, then
   * the output to the corresponding log levels will be silent.
   */
  void set_primary_stream(std::ostream *os);
  void set_secondary_stream(std::ostream *os);
  void set_tertiary_stream(std::ostream *os);
  void set_warning_stream(std::ostream *os);
  void set_json_stream(std::ostream *os);

  /* Redirect output to the file given by filename. If unsuccesful in
   * opening the file, the stream goes silent.
   *
   * Returns true iff the file was succesfully opened.
   */
  bool set_primary_stream_file(std::string filename);
  bool set_secondary_stream_file(std::string filename);
  bool set_tertiary_stream_file(std::string filename);

};

#endif
