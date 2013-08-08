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

#include "log.h"

#include <fstream>

namespace Log{

  std::ostream *primary_stream = &std::cout;
  bool own_primary_stream = false; // True if Log has ownership of *primary_stream
  std::ostream *secondary_stream = 0;
  bool own_secondary_stream = false; // True if Log has ownership of *secondary_stream
  std::ostream *tertiary_stream = 0;
  bool own_tertiary_stream = false; // True if Log has ownership of *tertiary_stream
  std::ostream *warning_stream = &std::cerr;
  std::ostream *json_stream = 0; // By default, do not print json directives
  std::ostream *null_stream = 0;

  class StreamLiberator{
  public:
    StreamLiberator(std::ostream **os, bool *own) : os(os), own(own) {};
    ~StreamLiberator(){
      if(*own){
        delete *os;
      }
    };
  private:
    std::ostream **os;
    bool *own;
  };

  StreamLiberator sl_primary(&primary_stream,&own_primary_stream);
  StreamLiberator sl_secondary(&secondary_stream,&own_secondary_stream);
  StreamLiberator sl_tertiary(&tertiary_stream,&own_tertiary_stream);

  redirection_stream result(&primary_stream);
  redirection_stream msg(&secondary_stream);
  redirection_stream debug(&secondary_stream);
  redirection_stream extreme(&secondary_stream);
  redirection_stream warning(&warning_stream);
  redirection_stream json(&json_stream);
  redirection_stream null(&null_stream);

  void set_primary_stream(std::ostream *os){
    if(own_primary_stream){
      delete primary_stream;
      own_primary_stream = false;
    }
    primary_stream = os;
  };
  void set_secondary_stream(std::ostream *os){
    if(own_secondary_stream){
      delete secondary_stream;
      own_secondary_stream = false;
    }
    secondary_stream = os;
  };
  void set_tertiary_stream(std::ostream *os){
    if(own_tertiary_stream){
      delete tertiary_stream;
      own_tertiary_stream = false;
    }
    tertiary_stream = os;
  };
  void set_warning_stream(std::ostream *os){
    warning_stream = os;
  };
  void set_json_stream(std::ostream *os){
    json_stream = os;
  };

  loglevel_t primary_loglevel = RESULTS;
  loglevel_t secondary_loglevel = DEBUG;

  void update_redirections(){
    result.os = &tertiary_stream;
    msg.os = &tertiary_stream;
    debug.os = &tertiary_stream;
    extreme.os = &tertiary_stream;

    switch(secondary_loglevel){ // Note that there should be no breaks in this switch
    case EXTREME: extreme.os = &secondary_stream;
    case DEBUG: debug.os = &secondary_stream;
    case MSG: msg.os = &secondary_stream;
    case RESULTS: result.os = &secondary_stream;
    case SILENT:
      break;
    }
    switch(primary_loglevel){ // Note that there should be no breaks in this switch
    case EXTREME: extreme.os = &primary_stream;
    case DEBUG: debug.os = &primary_stream;
    case MSG: msg.os = &primary_stream;
    case RESULTS: result.os = &primary_stream;
    case SILENT:
      break;
    };
  };

  loglevel_t get_primary_loglevel(){
    return primary_loglevel;
  };
  void set_primary_loglevel(loglevel_t lv){
    primary_loglevel = lv;
    update_redirections();
  };
  loglevel_t get_secondary_loglevel(){
    return secondary_loglevel;
  };
  void set_secondary_loglevel(loglevel_t lv){
    secondary_loglevel = lv;
    update_redirections();
  };

  class clofstream : public std::ofstream{
  public:
    clofstream(std::string filename) : std::ofstream(filename.c_str()) {};
    ~clofstream(){
      close();
    };
  };

  bool set_stream_file(std::string filename, std::ostream **os, bool *own){
    if(*own){
      delete *os;
    }
    
    *os = new clofstream(filename);
    if(**os){
      *own = true;
      return true;
    }else{
      std::cerr << "wtf?\n";
      delete *os;
      *os = 0;
      *own = false;
      return false;
    }
  }

  bool set_primary_stream_file(std::string filename){
    return set_stream_file(filename,&primary_stream,&own_primary_stream);
  };
  bool set_secondary_stream_file(std::string filename){
    return set_stream_file(filename,&secondary_stream,&own_secondary_stream);
  };
  bool set_tertiary_stream_file(std::string filename){
    return set_stream_file(filename,&tertiary_stream,&own_tertiary_stream);
  };

};
