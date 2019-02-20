/*
 * Copyright (C) 2012, 2014 Carl Leonardsson
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

#include "constraint.h"
#include "exact_bwd.h"
#include "fence_sync.h"
#include "fencins.h"
#include "lexer.h"
#include "machine.h"
#include "min_coverage.h"
#include "pb_cegar.h"
#include "pb_constraint.h"
#include "tso_fencins.h"
#include "pso_fencins.h"
#include <cerrno>
#include "pb_container2.h"
#include "predicates.h"
#include "preprocessor.h"
#include "sb_constraint.h"
#include "channel_container.h"
#include "sb_tso_bwd.h"
#include "hsb_constraint.h"
#include "hsb_container.h"
#include "hsb_pso_bwd.h"
#include "dual_constraint.h"
#include "dual_channel_container.h"
#include "dual_tso_bwd.h"
#include "pdual_constraint.h"
#include "pdual_channel_container.h"
#include "pdual_tso_bwd.h"
#include "shellcmd.h"
#include "sync_set_printer.h"
#include "test.h"
#include "test_vips_fencins.h"
#include "timer.h"
#include "tso_fence_sync.h"
#include "tso_fencins.h"
#include "tso_lock_sync.h"
#include "tso_simple_fencer.h"
#include "vips_bit_constraint.h"
#include "vips_bit_reachability.h"
#include "vips_simple_fencer.h"
#include "vips_syncwr_sync.h"
#include "vips_syncrd_sync.h"
#include "zstar.h"

#include <cerrno>
#include <config.h>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <regex>
#include <set>
#include <stdexcept>
#include <unistd.h>

struct Flag{
  Flag() {};
  Flag(std::string name, std::string given_name, bool given_by_user)
    : name(name), given_name(given_name), given_by_user(given_by_user), has_argument(false) {};
  Flag(std::string name, std::string given_name, bool given_by_user, std::string argument)
    : name(name), given_name(given_name), given_by_user(given_by_user), has_argument(true), argument(argument) {};
  std::string name;
  std::string given_name; // The name by which the user specified the flag
  bool given_by_user; // The alternative would be that the default value is used
  bool has_argument;
  std::string argument;
};

template<typename ITER> void inform_ignore(ITER begin, ITER end,
                                           const std::map<std::string,Flag> flags){
  for(std::map<std::string,Flag>::const_iterator it = flags.begin(); it != flags.end(); it++){
    if(it->second.given_by_user && std::find(begin,end,it->first) == end){
      Log::warning << "Ignoring flag '" << it->second.given_name << "'";
      if(it->second.has_argument){
        Log::warning << " (" << it->second.argument << ")\n";
      }else{
        Log::warning << "\n";
      }
    }
  }
}

/* Read and return a machine from input_stream.
 *
 * If flags["rff"], then convert the machine to register free form
 * before returning it.
 *
 * If flags["a"].argument is "hsb", additionaly convert locks to
 * fences before returning.
 */
Machine *get_machine(const std::map<std::string,Flag> flags, std::istream &input_stream){
  PPLexer lex(input_stream);
  std::unique_ptr<Machine> machine(new Machine(Parser::p_test(lex)));

  std::set<std::string> abstractions_requiring_fences{"hsb"};
  std::set<std::string> finite_bounds{"sb", "hsb", "dual", "pdual"};
  int reg_count = 0;
  for (const auto &pregs : machine->regs) reg_count += pregs.size();

  if(flags.count("rff")){
    machine = std::unique_ptr<Machine>(machine->remove_registers());
    machine = std::unique_ptr<Machine>(machine->remove_superfluous_nops());
  } else if (flags.count("a") && finite_bounds.count(flags.at("a").argument) && reg_count > 0) {
    Log::msg << "Warning: You are using an abstraction for finite data bounds without register "
             << "free form (--rff). Performance is commonly much better with register free "
             << "form." << std::endl;
  }
  if(flags.count("a") && abstractions_requiring_fences.count(flags.at("a").argument)){
    machine = std::unique_ptr<Machine>(machine->convert_locks_to_fences());
  }
  return machine.release();
};

template<class FenceSet>
void print_fence_sets(const Machine &machine, const std::list<FenceSet> &fence_sets){
  std::set<std::set<Sync*> > sync_sets;
  for(auto it = fence_sets.begin(); it != fence_sets.end(); ++it){
    sync_sets.insert(it->to_sync_set());
  }

  SyncSetPrinter::print(sync_sets,machine,Log::result,Log::json);

  for(auto it = sync_sets.begin(); it != sync_sets.end(); ++it){
    for(auto s : *it){
      delete s;
    }
  }
};

int fencins(const std::map<std::string,Flag> flags, std::istream &input_stream){
  std::set<std::string> used_flags =
    {"a","k","cegar","max-refinements","max-solutions","rff","fmin","fence-cost",
     "dismiss-fence","fence-full-branch-only"};
  inform_ignore(used_flags.begin(),used_flags.end(),flags);
  std::unique_ptr<Machine> machine(get_machine(flags,input_stream));
  int max_refinements = -1;
  if(flags.count("max-refinements")){
    std::stringstream ss(flags.find("max-refinements")->second.argument);
    if(!(ss >> max_refinements) || !ss.eof()){
      std::cerr << "Invalid value '" << flags.find("max-refinements")->second.argument << "' given for max-refinements.\n";
      return 1;
    }
  }
  int max_solutions = 0;
  if(flags.count("max-solutions")){
    std::stringstream ss(flags.find("max-solutions")->second.argument);
    if(!(ss >> max_solutions) || !ss.eof() || max_solutions < 0){
      std::cerr << "Invalid value '" << flags.find("max-solutions")->second.argument << "' given for max-solutions.\n";
      return 1;
    }
  }

  int retval;

  Timer fencins_timer;
  fencins_timer.start();

  if(flags.find("a")->second.argument == "pb"){
    machine = std::unique_ptr<Machine>(machine->add_domain_assumes());
    PbConstraint::pred_set preds;
    int k = 1;
    if(flags.count("k")){
      std::stringstream ss(flags.find("k")->second.argument);
      if(!(ss >> k) || !ss.eof() || k < 1){
        std::cerr << "Invalid value '" << flags.find("k")->second.argument << "' given for k.\n";
        return 1;
      }
    }

    Reachability *reach = 0;
    TsoFencins::reach_arg_init_t *arg_init = 0;
    if(flags.count("cegar")){
      preds.clear();
      if(machine->predicates.size()){
        Log::msg << "Starting CEGAR from predicates given in .rmm file.\n";
        std::function<TsoVar(const Predicates::DummyVar&)> cv =
          [](const Predicates::DummyVar&)->TsoVar{
          throw new std::logic_error("Fencins: Non-nullary predicate in predicates of machine.");
        };
        for(unsigned i = 0; i < machine->predicates.size(); i++){
          preds.push_back(new Predicates::Predicate<TsoVar>(machine->predicates[i].convert(cv)));
        }
      }
      reach = new PbCegar();
      arg_init = new TsoFencins::reach_arg_init_t([&preds,k,max_refinements](const Machine &m,const Reachability::Result *prev_res)->Reachability::Arg*{
          if(prev_res){
            const PbCegar::Result *pres = static_cast<const PbCegar::Result*>(prev_res);
            const ExactBwd::Result *eres = static_cast<const ExactBwd::Result*>(pres->last_result);
            const APList<TsoVar>::pred_set &preds2 = static_cast<const PbConstraint::Common*>(eres->common)->predicates;
            if(preds2.size() > preds.size()){
              /* Replace preds with new predicates */
              for(unsigned i = 0; i < preds.size(); i++){
                delete preds[i];
              }
              preds.clear();
              for(unsigned i = 0; i < preds2.size(); i++){
                preds.push_back(new Predicates::Predicate<TsoVar>(*preds2[i]));
              }
            }
          }
          /* Using preds in constructor for Common will let go of ownership.
           * Therefore we need to copy predicates. */
          PbConstraint::pred_set preds_copy;
          for(unsigned i = 0; i < preds.size(); i++){
            preds_copy.push_back(new Predicates::Predicate<TsoVar>(*preds[i]));
          }
          PbConstraint::Common *common = new PbConstraint::Common(k,m,preds_copy,true);
          return new PbCegar::Arg(m,new ExactBwd(),new ExactBwd::Arg(m,common,new PbContainer2(m)),
                                  max_refinements,ExactBwd::pb_init_arg);
        });
    }else{
      reach = new ExactBwd();
      preds = PbConstraint::extract_predicates(*machine);
      arg_init = new TsoFencins::reach_arg_init_t([&preds,k](const Machine &m,const Reachability::Result *)->Reachability::Arg*{
          PbConstraint::pred_set preds_copy;
          for(unsigned i = 0; i < preds.size(); i++){
            preds_copy.push_back(new Predicates::Predicate<TsoVar>(*preds[i]));
          }
          PbConstraint::Common *common = new PbConstraint::Common(k,m,preds_copy,true);
          return new ExactBwd::Arg(m,common,new PbContainer2(m));
        });
    }
    std::string fmin = "cheap";
    if(flags.count("fmin")){
      fmin = flags.find("fmin")->second.argument;
    }
    if(fmin == "cheap"){
      Log::msg << "Searching for cheap synchronization sets.\n";
      if(max_solutions != 0 && max_solutions != 1){
        Log::warning << "Warning: Solution limiting (other than 0 and 1) is not supported for cheap fencins for TSO. Ignoring flag --max-solutions.\n";
      }
      std::list<TsoFencins::FenceSet> fence_sets =
        TsoFencins::fencins(*machine,*reach,*arg_init,max_solutions == 1);
      print_fence_sets(*machine,fence_sets);
      retval = 0;
    }else if(fmin == "subset" || fmin == "cost"){
      Fencins::min_aspect_t min_aspect;
      if(fmin == "cost"){
        min_aspect = Fencins::COST;
        Log::msg << "Searching for cost minimal synchronization sets.\n";
      }else{
        min_aspect = Fencins::SUBSET;
        Log::msg << "Searching for subset minimal synchronization sets.\n";
      }
      TsoSimpleFencer fencer(*machine,TsoSimpleFencer::LOCKED);
      auto sync_sets = Fencins::fencins(*machine,*reach,*arg_init,fencer,min_aspect,max_solutions);
      SyncSetPrinter::print(sync_sets,*machine,Log::result,Log::json);
      for(auto ss : sync_sets){
        for(auto s : ss){
          delete s;
        }
      }
      retval = 0;
    }else{
      Log::warning << "Fencins minimality criterion '" << fmin
                   << "' is not supported for SB.\n";
      retval = 1;
    }
    for(unsigned i = 0; i < preds.size(); i++){
      delete preds[i];
    }
    delete reach;
    delete arg_init;
  }else if(flags.find("a")->second.argument == "sb"){
    SbTsoBwd reach;
    TsoFencins::reach_arg_init_t arg_init =
      [](const Machine &m, const Reachability::Result *)->Reachability::Arg*{
      SbConstraint::Common *common = new SbConstraint::Common(m);
      return new ExactBwd::Arg(m,common->get_bad_states(),common,new ChannelContainer());
    };
    std::string fmin = "cheap";
    if(flags.count("fmin")){
      fmin = flags.find("fmin")->second.argument;
    }
    if(fmin == "cheap"){
      Log::msg << "Searching for cheap synchronization sets.\n";
      if(max_solutions != 0){
        Log::warning << "Warning: Solution limiting (other than 0 and 1) is not supported for cheap fencins for TSO. Ignoring flag --max-solutions.\n";
      }
      std::list<TsoFencins::FenceSet> fence_sets =
        TsoFencins::fencins(*machine,reach,arg_init,max_solutions == 1);
      print_fence_sets(*machine,fence_sets);
      retval = 0;
    }else if(fmin == "subset" || fmin == "cost"){
      Fencins::min_aspect_t min_aspect;
      if(fmin == "cost"){
        min_aspect = Fencins::COST;
        Log::msg << "Searching for cost minimal synchronization sets.\n";
      }else{
        min_aspect = Fencins::SUBSET;
        Log::msg << "Searching for subset minimal synchronization sets.\n";
      }
      TsoSimpleFencer fencer(*machine,TsoSimpleFencer::LOCKED);
      auto sync_sets = Fencins::fencins(*machine,reach,arg_init,fencer,min_aspect,max_solutions);
      SyncSetPrinter::print(sync_sets,*machine,Log::result,Log::json);
      for(auto ss : sync_sets){
        for(auto s : ss){
          delete s;
        }
      }
      retval = 0;
    }else{
      Log::warning << "Fencins minimality criterion '" << fmin
                   << "' is not supported for SB.\n";
      return 1;
    }
  }else if(flags.find("a")->second.argument == "vips"){
    Fencins::cost_fn_t cost =
      [](const Sync*){
      return 1;
    };
    if(flags.count("fence-cost")){
      int full, ss, ll, syncwr, syncrd;
      std::stringstream fcss(flags.find("fence-cost")->second.argument);
      if(!(fcss >> full >> ss >> ll >> syncwr >> syncrd) || !fcss.eof() ||
         full < 0 || ss < 0 || ll < 0 || syncwr < 0 || syncrd < 0){
        Log::warning << "Invalid cost specification given with flag --fence-cost.\n";
        return 1;
      }
      cost = [full,ss,ll,syncwr,syncrd](const Sync *s){
        if(dynamic_cast<const VipsFullFenceSync*>(s)){
          return full;
        }else if(dynamic_cast<const VipsSSFenceSync*>(s)){
          return ss;
        }else if(dynamic_cast<const VipsLLFenceSync*>(s)){
          return ll;
        }else if(dynamic_cast<const VipsSyncwrSync*>(s)){
          return syncwr;
        }else{
          assert(dynamic_cast<const VipsSyncrdSync*>(s));
          return syncrd;
        }
      };
    }
    Fencins::min_aspect_t min_aspect = Fencins::SUBSET;
    if(flags.count("fmin")){
      if(flags.find("fmin")->second.argument == "cost"){
        min_aspect = Fencins::COST;
      }else if(flags.find("fmin")->second.argument == "subset"){
        min_aspect = Fencins::SUBSET;
      }else{
        Log::warning << "Fencins minimality criterion '" << flags.find("fmin")->second.argument
                     << "' is not supported for VIPS.\n";
        return 1;
      }
    }
    switch(min_aspect){
    case Fencins::SUBSET: Log::msg << "Searching for subset minimal synchronization sets.\n"; break;
    case Fencins::COST: Log::msg << "Searching for cost minimal synchronization sets.\n"; break;
    default: break;
    }
    VipsBitReachability reach;
    Fencins::reach_arg_init_t reach_arg_init =
      [](const Machine &m,const Reachability::Result*)->Reachability::Arg*{
      return new Reachability::Arg(m);
    };
    std::function<bool(const Sync*)> accept =
      [](const Sync*){ return true; };
    try{
      if(flags.count("dismiss-fence")){
        std::regex dismiss_regex(flags.at("dismiss-fence").argument);
        const Machine *m = machine.get();
        accept = [m,dismiss_regex](const Sync *s){
          return !std::regex_match(s->to_string(*m),dismiss_regex);
        };
      }
    }catch(std::regex_error err){
      Log::warning << "Regexp error in argument to --dismiss-fence.\n";
      return 1;
    }
    VipsSimpleFencer fencer(*machine,flags.count("fence-full-branch-only"),accept);
    auto sync_sets = Fencins::fencins(*machine,reach,reach_arg_init,fencer,min_aspect,max_solutions,cost);
    SyncSetPrinter::print(sync_sets,*machine,Log::result,Log::json);
    for(auto ss : sync_sets){
      for(auto s : ss){
        delete s;
      }
    }
    retval = 0;
  }else if(flags.find("a")->second.argument == "hsb"){
    std::list<PsoFencins::FenceSet> fence_sets;
    HsbPsoBwd reach;
    TsoFencins::reach_arg_init_t arg_init =
      [](const Machine &m, const Reachability::Result *)->Reachability::Arg*{
      HsbConstraint::Common *common = new HsbConstraint::Common(m);
      return new ExactBwd::Arg(m,common->get_bad_states(),common,new HsbContainer());
    };
    fence_sets = PsoFencins::fencins(*machine,reach,arg_init,flags.count("only-one"));
    print_fence_sets(*machine,fence_sets);
    retval = 0;  }else{
    Log::warning << "Abstraction '" << flags.find("a")->second.argument << "' is not supported.\nSorry.\n";
    return 1;
  }

  {
    fencins_timer.stop();
    std::stringstream ss;
    ss << "Total time to insert fences: "
       << std::setprecision(1) << std::fixed << fencins_timer.get_time() << " s\n";
    Log::result << ss.str();
  }

  return retval;
}

int reachability(const std::map<std::string,Flag> flags, std::istream &input_stream){
  std::string used_flags[] = {"a","k","cegar","rff"};
  inform_ignore(used_flags,used_flags+4,flags);
  std::unique_ptr<Machine> machine(get_machine(flags,input_stream));

  Reachability *reach = 0;
  Reachability::Arg *rarg = 0;

  if(flags.find("a")->second.argument == "pb"){
    Machine *tmp_machine = machine.release();
    machine = std::unique_ptr<Machine>(tmp_machine->add_domain_assumes());
    delete tmp_machine;
    if(flags.count("cegar") > 0){
      PbConstraint::Common *common = new PbConstraint::Common(1,*machine,PbConstraint::pred_set(),true);
      reach = new PbCegar();
      rarg = new PbCegar::Arg(*machine,new ExactBwd(),new ExactBwd::Arg(*machine,common,new PbContainer2(*machine)),
                              -1,ExactBwd::pb_init_arg);
    }else{
      PbConstraint::pred_set preds;
      if(machine->predicates.size()){
        Log::msg << "Using predicates given in .rmm file.\n";
        /* Use given predicates. */
        std::function<TsoVar(const Predicates::DummyVar&)> cv =
          [](const Predicates::DummyVar&)->TsoVar{
          throw new std::logic_error("Fencins: Non-nullary predicate in predicates of machine.");
        };
        for(unsigned i = 0; i < machine->predicates.size(); i++){
          preds.push_back(new Predicates::Predicate<TsoVar>(machine->predicates[i].convert(cv)));
        }
      }else{
        Log::msg << "Extracting predicates from source code in .rmm file.\n";
        /* Try to extract predicates from the automata of the machine. */
        preds = PbConstraint::extract_predicates(*machine);
      }
      int k = 1;
      if(flags.count("k")){
        std::stringstream ss(flags.find("k")->second.argument);
        if(!(ss >> k) || !ss.eof() || k < 1){
          std::cerr << "Invalid value '" << flags.find("k")->second.argument << "' given for k.\n";
          return 1;
        }
      }
      Log::msg  << "Abstraction: pb\n"
                << "k: " << k << "\n"
                << "Predicates:\n";
      for(unsigned i = 0; i < preds.size(); i++){
        Log::msg << "  " << preds[i]->to_string([&machine](int r,int p){ return machine->pretty_string_reg.at(std::pair<int,int>(r,p)); },
                                                [&machine](Lang::NML nml){ return machine->pretty_string_nml.at(nml); }) << "\n";
      }
      std::list<Constraint*> bad_states;
      PbConstraint::Common *common = new PbConstraint::Common(k,*machine,preds,true);
      for(unsigned i = 0; i < machine->forbidden.size(); i++){
        bad_states.push_back(new PbConstraint(machine->forbidden[i],*common));
      }
      reach = new ExactBwd();
      rarg = new ExactBwd::Arg(*machine,bad_states,common,new PbContainer2(*machine));
    }
  }else if(flags.find("a")->second.argument == "sb"){
    SbConstraint::Common *common = new SbConstraint::Common(*machine);
    reach = new SbTsoBwd();
    rarg = new ExactBwd::Arg(*machine,common->get_bad_states(),common,new ChannelContainer());
  }else if(flags.find("a")->second.argument == "vips"){
    reach = new VipsBitReachability();
    rarg = new Reachability::Arg(*machine);
  }else if(flags.find("a")->second.argument == "hsb"){
    HsbConstraint::Common *common = new HsbConstraint::Common(*machine);
    reach = new HsbPsoBwd();
    rarg = new ExactBwd::Arg(*machine,common->get_bad_states(),common,new HsbContainer());
  }else if(flags.find("a")->second.argument == "dual"){
    DualConstraint::Common *common = new DualConstraint::Common(*machine);
    reach = new DualTsoBwd();
    rarg = new ExactBwd::Arg(*machine,common->get_bad_states(),common,new DualChannelContainer());
  }else if(flags.find("a")->second.argument == "pdual"){
    PDualConstraint::Common *common = new PDualConstraint::Common(*machine);
    reach = new PDualTsoBwd();
    rarg = new ExactBwd::Arg(*machine,common->get_bad_states(),common,new PDualChannelContainer());
  }else{
    Log::warning << "Abstraction '" << flags.find("a")->second.argument << "' is not supported.\nSorry.\n";
    return 1;
  }

  Log::msg << "Running reachability analysis...\n" << std::flush;
  Reachability::Result *result = reach->reachability(rarg);

  if(result->result == Reachability::REACHABLE){
    if(flags.find("a")->second.argument == "vips"){
      /* Rewrite trace to improve readability. */
      Trace *t2 = VipsSimpleFencer::decrease_reorderings(*result->trace);
      delete result->trace;
      result->trace = t2;
    }
    Log::msg << "\n *** Witness trace ***\n";
    result->trace->print(Log::msg,Log::debug,Log::json,*machine);
  }

  Log::result << result->to_string() << "\n";

  delete result;
  delete reach;
  delete rarg;
  return 0;
}

/* Produce a pdf showing the automata generated from the code inputted on cin. */
int dotify(const std::map<std::string,Flag> flags, std::istream &input_stream){
  std::string used_flags[] = {"o","rff","a"};
  inform_ignore(used_flags,used_flags+3,flags);
  if(flags.count("o") == 0){
    Log::warning << "For command dotify. Specify an output file.pdf using the flag -o.\n";
    return 1;
  }

  std::string outputfile = flags.find("o")->second.argument;

  std::string dot_repr;
  std::unique_ptr<Machine> m(get_machine(flags,input_stream));
  dot_repr = m->to_dot();

  char tmp_file_name[] = "dotifytmpXXXXXX";
  int tmpfile = mkstemp(tmp_file_name);
  if(tmpfile == -1){
    perror("Dotify failed to open temporary file");
    return 1;
  }
  int written_chars = write(tmpfile,dot_repr.c_str(),dot_repr.length());
  close(tmpfile);
  if(written_chars != int(dot_repr.length())){
    Log::warning << "Failed to write to file.\n";
  }

  int status;
  ShellCmd::exec("dot -Tpdf "+std::string(tmp_file_name)+" > "+outputfile,&status);
  if(status == 0){
    Log::result << "Wrote pdf to " << outputfile << std::endl;
  }else{
    Log::result << "Failed to dotify automata.\n";
  }

  ShellCmd::exec("rm "+std::string(tmp_file_name));

  return 0;
}

void print_version(int argc, char *argv[]){
  std::cout << PACKAGE_STRING << "\n"
            << "Copyright (C) 2012 Carl Leonardsson\n"
            << "This program comes with ABSOLUTELY NO WARRANTY. This is free software and you\n"
            << "are welcome to redistribute it under certain conditions. See the full text of\n"
            << "the GNU General Public License Version 3 (http://www.gnu.org/licenses/).\n";
}

void print_help(int argc, char *argv[]){
  print_version(argc,argv);
  std::cout << "\n";
  std::cout << "Usage: " << argv[0] << " [OPTIONS] COMMAND [FILE]\n"
            << std::endl
            << "  Commands:\n"
            << "    reach            - Read a rmm specification on stdin. Check reachability.\n"
            << "    fencins          - Read a rmm specification on stdin. Insert fences.\n"
            << "    dotify           - Produce a pdf file representing the compiled automata.\n"
            << std::endl
            << "  Options:\n"
            << "    -o <filename> / --output <filename>\n"
            << "        Write output to <filename>.\n"
            << "    -a <abstraction> / --abstraction <abstraction>\n"
            << "        Use abstraction <abstraction>.\n"
            << "    -k <int>\n"
            << "        Use k as buffer bound. (Used only for abstraction pb.)\n"
            << "    --cegar\n"
            << "        Use CEGAR refinement in reachability analysis.\n"
            << "    --dismiss-fence <regex>\n"
            << "        For fence insertion, ignore all synchronizations that\n"
            << "        match <regex>. Uses ECMAScript regex syntax.\n"
            << "    --fence-cost <int:a> <int:b> <int:c> <int:d> <int:e>\n"
            << "        (only vips, minimality criterion cost)\n"
            << "        Instead of counting all kinds of fences as equally expensive,\n"
            << "        use cost <int:a> for full fences, <int:b> for ssfences,\n"
            << "        <int:c> for llfences, <int:d> for syncwrs, and <int:e> for syncrds.\n"
            << "    --fence-full-branch-only / --ffbo\n"
            << "        In fence insertion, only consider fences between all incoming\n"
            << "        and all outgoing transitions for a given control location.\n"
            << "    --max-refinements <int>\n"
            << "        Perform at most <int> many refinements. (Used only in cegar.)\n"
            << "    --max-solutions <int>\n"
            << "        During fence insertion, stop searching after finding <int>\n"
            << "        sufficient, minimal fence sets.\n"
            << "    --fencins-minimality <M> / --fmin <M>\n"
            << "        Use minimality criterion <M> for fence insertion.\n"
            << "        Possible values are cheap, cost, subset.\n"
            << "    -v / --verbose\n"
            << "        Print output verbosely.\n"
            << "    -vv / --very-verbose\n"
            << "        Print output very verbosely.\n"
            << "    -vvv / --very-very-verbose\n"
            << "        Print output very very verbosely.\n"
            << "    --rff\n"
            << "        Convert machine to Register Free Form before using it.\n"
            << "    --version / -V\n"
            << "        Print version and quit.\n"
            << std::endl
            << "  Abstractions:\n"
            << "    pb\n"
            << "      TSO with bounded number of buffer messages per process and variable.\n"
            << "      Uses predicate abstraction.\n"
            << "      Overapproximation of TSO.\n"
            << "      Sound, but incomplete with CEGAR.\n"
            << "    sb (default)\n"
            << "      The Single Buffer model.\n"
            << "      Equivalent to TSO w.r.t. control state reachability.\n"
            << "      Sound and complete for finite data domains.\n"
            << "    hsb\n"
            << "      The Hierarchy Single Buffer model.\n"
            << "      Equivalent to PSO w.r.t. control state reachability.\n"
            << "      Sound and complete for finite data domains.\n"
            << "    dual\n"
            << "      The Dual TSO Buffer model.\n"
            << "      Equivalent to TSO w.r.t. control state reachability.\n"
            << "      Sound and complete for finite data domains.\n"
            << "    pdual\n"
            << "      The parameterized Dual TSO Buffer model.\n"
            << "      Equivalent to TSO w.r.t. parameterized control state reachability.\n"
            << "      Sound and complete for finite data domains.\n"
            << "    vips\n"
            << "      VIPS-M. Explicit state forward analysis.\n"
            << "      Sound and complete for finite data domains.\n"
            << std::endl
            << "  Fencins minimality criteria:\n"
            << "    subset\n"
            << "      Find sets of synchronization which are subset minimal.\n"
            << "    cost\n"
            << "      Find sets of synchronization with the least cost.\n"
            << "      All kinds of synchronization is considered equally expensive\n"
            << "      by default. (See --fence-cost.)\n"
            << "    cheap (sb/pb only)\n"
            << "      Cheaper fence insertion. Only considers synchronization by locking writes.\n"
            << "      Usually gives subset minimal synchronization sets, but will occasionally\n"
            << "      yield larger sets.\n";
}

int main(int argc, char *argv[]){
  enum command { UNDEF, DOTIFY, TEST, REACHABILITY, FENCINS };
  command cmd = UNDEF;
  std::map<std::string,Flag> flags;
  std::set<int> needs_input_stream; // Set of all commands that require an input stream
  needs_input_stream.insert(REACHABILITY);
  needs_input_stream.insert(DOTIFY);
  needs_input_stream.insert(FENCINS);
  std::istream *input_stream = &std::cin;
  if(argc > 1){
    for(int i = 1; i < argc; i++){
      if(argv[i] == std::string("dotify")){
        if(cmd == UNDEF){
          cmd = DOTIFY;
        }else{
          Log::warning << "Can't specify more than one command.\n";
          print_help(argc, argv);
          return 1;
        }
      }else if(argv[i] == std::string("reach")){
        if(cmd == UNDEF){
          cmd = REACHABILITY;
        }else{
          Log::warning << "Can't specify more than one command.\n";
          print_help(argc, argv);
          return 1;
        }
      }else if(argv[i] == std::string("fencins")){
        if(cmd == UNDEF){
          cmd = FENCINS;
        }else{
          Log::warning << "Can't specify more than one command.\n";
          print_help(argc, argv);
          return 1;
        }
      }else if(argv[i] == std::string("test")){
        if(cmd == UNDEF){
          cmd = TEST;
        }else{
          Log::warning << "Can't specify more than one command.\n";
          print_help(argc, argv);
          return 1;
        }
      }else if(argv[i] == std::string("--cegar")){
        flags["cegar"] = Flag("cegar",argv[i],true);
      }else if(argv[i] == std::string("--dismiss-fence")){
        if(flags.count("dismiss-fence")){
          Log::warning << "Flag --dismiss-fence specified twice.\n";
          print_help(argc,argv);
          return 1;
        }else if(i < argc-1){
          flags["dismiss-fence"] = Flag("dismiss-fence",argv[i],true,argv[i+1]);
          i++; // Do not account for the next argv twice.
        }else{
          Log::warning << "--dismiss-fence must have an argument.\n";
          print_help(argc,argv);
          return 1;
        }
      }else if(argv[i] == std::string("--fence-full-branch-only") || argv[i] == std::string("--ffbo")){
        flags["fence-full-branch-only"] = Flag("fence-full-branch-only",argv[i],true);
      }else if(argv[i] == std::string("--max-solutions")){
        if(flags.count("max-solutions")){
          Log::warning << "Flag --max-solutions specified twice.\n";
          print_help(argc,argv);
          return 1;
        }else if(i < argc-1){
          flags["max-solutions"] = Flag("max-solutions",argv[i],true,argv[i+1]);
          i++; // Do not account for the next argv twice.
        }else{
          Log::warning << "--max-solutions must have an argument.\n";
          print_help(argc,argv);
          return 1;
        }
      }else if(argv[i] == std::string("--fence-cost")){
        if(flags.count("fence-cost")){
          Log::warning << "Flag --fence-cost specified twice.\n";
          print_help(argc,argv);
          return 1;
        }else if(i < argc-5){
          std::string args =
            std::string(argv[i+1])+" "+argv[i+2]+" "+
            argv[i+3]+" "+argv[i+4]+" "+argv[i+5];
          flags["fence-cost"] = Flag("fence-cost",argv[i],true,args);
          i+=5; // Do not account for the next 5 argvs twice.
        }else{
          Log::warning << "--fence-cost must have 5 arguments.\n";
          print_help(argc,argv);
          return 1;
        }
      }else if(argv[i] == std::string("-v") || argv[i] == std::string("--verbose")){
        flags["verbose"] = Flag("verbose",argv[i],true);
      }else if(argv[i] == std::string("-vv") || argv[i] == std::string("--very-verbose")){
        flags["very-verbose"] = Flag("very-verbose",argv[i],true);
      }else if(argv[i] == std::string("-vvv") || argv[i] == std::string("--very-very-verbose")){
        flags["very-very-verbose"] = Flag("very-very-verbose",argv[i],true);
      }else if(argv[i] == std::string("-o") || argv[i] == std::string("--output")){
        if(flags.count("o")){
          Log::warning << "Flag " << argv[i] << " specified twice.\n";
          print_help(argc,argv);
          return 1;
        }else if(i < argc-1){
          flags["o"] = Flag("o",argv[i],true,argv[i+1]);
          i++; // Do not account for the next argv twice.
        }else{
          Log::warning << argv[i] << " must have an argument.\n";
          print_help(argc,argv);
          return 1;
        }
      }else if(argv[i] == std::string("--rff")){
        flags["rff"] = Flag("rff",argv[i],true);
      }else if(argv[i] == std::string("--max-refinements")){
        if(flags.count("max-refinements")){
          Log::warning << "Flag " << argv[i] << " specified twice.\n";
          print_help(argc,argv);
          return 1;
        }else if(i < argc-1){
          flags["max-refinements"] = Flag("max-refinements",argv[i],true,argv[i+1]);
          i++; // Do not account for the next argv twice.
        }else{
          Log::warning << argv[i] << " must have an argument.\n";
          print_help(argc,argv);
          return 1;
        }
      }else if(argv[i] == std::string("-k")){
        if(flags.count("k")){
          Log::warning << "Flag -k specified twice.\n";
          print_help(argc,argv);
          return 1;
        }else if(i < argc-1){
          flags["k"] = Flag("k",argv[i],true,argv[i+1]);
          i++; // Do not account for the next argv twice.
        }else{
          Log::warning << "-k must have an argument.\n";
          print_help(argc,argv);
          return 1;
        }
      }else if(argv[i] == std::string("--fencins-minimality") || argv[i] == std::string("--fmin")){
        if(flags.count("fmin")){
          Log::warning << "Flag " << argv[i] << " specified twice.\n";
          print_help(argc,argv);
          return 1;
        }else if(i < argc-1){
          flags["fmin"] = Flag("fmin",argv[i],true,argv[i+1]);
          i++; // Do not account for the next argv twice.
        }else{
          Log::warning << argv[i] << " must have an argument.\n";
          print_help(argc,argv);
          return 1;
        }
      }else if(argv[i] == std::string("-a") || argv[i] == std::string("--abstraction")){
        if(flags.count("a")){
          Log::warning << "Flag " << argv[i] << " specified twice.\n";
          print_help(argc,argv);
          return 1;
        }else if(i < argc-1){
          if(argv[i+1] == std::string("sb") ||
             argv[i+1] == std::string("pb") ||
             argv[i+1] == std::string("hsb") ||
             argv[i+1] == std::string("dual") ||
             argv[i+1] == std::string("pdual") ||
             argv[i+1] == std::string("vips")){
            flags["a"] = Flag("a",argv[i],true,argv[i+1]);
            i++;
          }else{
            Log::warning << argv[i+1] << " is not a supported abstraction.\n";
            print_help(argc,argv);
            return 1;
          }
        }else{
          Log::warning << "-a must have an argument.\n";
          print_help(argc,argv);
          return 1;
        }
      }else if(argv[i] == std::string("--version") || argv[i] == std::string("-V")){
        flags["version"] = Flag("version",argv[i],true);
      }else if(argv[i] == std::string("--json")){
        // Activate printing of json directives
        Log::set_json_stream(&std::cout);
      }else if(i == argc-1 && needs_input_stream.count(cmd)){
        errno = 0;
        input_stream = new std::ifstream(argv[i]);
        if(!input_stream->good()){
          Log::warning << "Unable to open file '" << argv[i] << "' for reading.\n";
          std::perror(0);
          return 1;
        }
      }else{
        Log::warning << "Unknown flag: " << argv[i] << std::endl;
        print_help(argc,argv);
        return 1;
      }
    }
  }

  if(flags.count("version")){
    print_version(argc,argv);
    return 0;
  }

  if(cmd == UNDEF){
    print_help(argc,argv);
    return 1;
  }
  /* Set defaults */
  if(flags.count("a") == 0) flags["a"] = Flag("a","-a",false,"sb");

  if(flags.count("verbose")){
    Log::set_primary_loglevel(Log::MSG);
  }
  if(flags.count("very-verbose")){
    Log::set_primary_loglevel(Log::DEBUG);
  }
  if(flags.count("very-very-verbose")){
    Log::set_primary_loglevel(Log::EXTREME);
  }
  flags.erase("verbose");
  flags.erase("very-verbose");
  flags.erase("very-very-verbose");

  int retval = 1;
  try{
    switch(cmd){
    case REACHABILITY:
      retval = reachability(flags,*input_stream);
      break;
    case FENCINS:
      retval = fencins(flags,*input_stream);
      break;
    case DOTIFY:
      retval = dotify(flags,*input_stream);
      break;
    case TEST:
      Test::add_test("Automaton",Automaton::test);
      Test::add_test("Fencins",Fencins::test);
      Test::add_test("FenceSync",FenceSync::test);
      Test::add_test("Machine",Machine::test);
      Test::add_test("MinCoverage",MinCoverage::test);
      Test::add_test("SbTsoBwd",SbTsoBwd::test);
      Test::add_test("Test",Test::test_testing);
      Test::add_test("TestVipsFencins",TestVipsFencins::test);
      Test::add_test("TsoFenceSync",TsoFenceSync::test);
      Test::add_test("TsoLockSync",TsoLockSync::test);
      Test::add_test("TsoSimpleFencer",TsoSimpleFencer::test);
      Test::add_test("VIPS-M Bit",VipsBitConstraint::test);
      Test::add_test("VIPS-M Bit Reachability",VipsBitReachability::test);
      Test::add_test("VipsFenceSync",VipsFenceSync::test);
      Test::add_test("VipsSimpleFencer",VipsSimpleFencer::test);
      Test::add_test("VipsSyncrdSync",VipsSyncwrSync::test);
      Test::add_test("VipsSyncwrSync",VipsSyncwrSync::test);
      Test::add_test("ZStar",ZStar<int>::test);
      Test::add_test("HsbConstraint",HsbConstraint::test);
      Test::add_test("DualZStar",DualZStar<int>::test);
      Test::add_test("DualConstraint",DualConstraint::test);
      Test::add_test("PDualConstraint",PDualConstraint::test);
      retval = Test::run_tests();
      break;
    default:
      break;
    }
  }catch(Parser::SyntaxError *exc){
    Log::warning << "Error: " << exc->what() << std::endl << std::flush;
    Log::json << "json: {\"action\":\"Syntax Error\", \"pos\":" << exc->get_pos().to_json() << "}\n";
    retval = 1;
    delete exc;
  }catch(std::exception *exc){
    Log::warning << "Error: " << exc->what() << std::endl << std::flush;
    retval = 1;
    delete exc;
  }
  if(input_stream != &std::cin){
    delete input_stream;
  }
  return retval;
}
