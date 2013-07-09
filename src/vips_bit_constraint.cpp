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

#include "lexer.h"
#include "log.h"
#include "parser.h"
#include "test.h"
#include "vips_bit_constraint.h"

#include <cassert>
#include <functional>
#include <limits>
#include <sstream>
#include <stdexcept>

VipsBitConstraint::Common::Common(const Machine &m) : machine(m) {
  proc_count = m.automata.size();

  /* Note that possible_to_pointer_pack also checks that the domains
   * of memory locations and registers are finite, and sufficiently
   * small.
   */
  if(possible_to_pointer_pack(m)){
    pointer_pack = true;
    Log::debug << "VipsBitConstraint::Common: Using pointer packing.\n";
  }else{
    pointer_pack = false;
    Log::debug << "VipsBitConstraint::Common: Not using pointer packing.\n";
  }

  /* Set up ml_offsets */
  {
    int c = machine.gvars.size();
    for(int p = 0; p < proc_count; ++p){
      ml_offsets.push_back(c);
      c += machine.lvars[p].size();
    }
  }

  /* Set up all_nmls */
  {
    /* Global */
    for(unsigned i = 0; i < machine.gvars.size(); ++i){
      all_nmls.push_back(Lang::NML::global(i));
    }
    /* Local */
    for(unsigned p = 0; p < machine.lvars.size(); ++p){
      for(unsigned i = 0; i < machine.lvars[p].size(); ++i){
        all_nmls.push_back(Lang::NML::local(i,p));
      }
    }
  }

  /* A class computing a sequence of non-overlapping bitfields while
   * moving to new elements as necessary.
   */
  class BFSeq{
  public:
    BFSeq(bool pointer_pack){
      if(pointer_pack){
        div = 2;
      }else{
        div = 1;
      }
      element = 0;
    };
    /* Get the next bitfield with size mod and offset off. */
    bitfield next(data_t mod, int off){
      if((std::numeric_limits<data_t>::max() - mod + 1) / mod < div){
        ++element;
        div = 1;
      }
      bitfield bf(element,div,mod,off);
      div *= mod;
      return bf;
    };
    /* Get the total number of elements that are used by this sequence. */
    int get_element_count() const{
      return element+1;
    };
  private:
    data_t div;
    int element;
  };

  /* Set up bitfields */
  BFSeq bfseq(pointer_pack);
  /* pcs */
  for(int p = 0; p < proc_count; ++p){
    pcs.push_back(bfseq.next((int)m.automata[p].get_states().size() + 1,0));
  }

  /* mem */
  {
    for(unsigned i = 0; i < all_nmls.size(); ++i){
      Lang::VarDecl::Domain dom = machine.get_var_decl(all_nmls[i]).domain;
      int mod = 1 + dom.get_upper_bound() - dom.get_lower_bound();
      int off = dom.get_lower_bound();
      mem_vec.push_back(bfseq.next(mod,off));
    }
  }

  /* L1s */
  {
    for(int p = 0; p < proc_count; ++p){
      l1_vec.push_back(std::vector<bitfield>());
      for(unsigned i = 0; i < all_nmls.size(); ++i){
        Lang::VarDecl::Domain dom = machine.get_var_decl(all_nmls[i]).domain;
        int mod = 2*(1 + dom.get_upper_bound() - dom.get_lower_bound());
        int off = 2*dom.get_lower_bound();
        l1_vec[p].push_back(bfseq.next(mod,off));
      }
    }
  }

  /* Registers */
  {
    for(int p = 0; p < proc_count; ++p){
      reg_vec.push_back(std::vector<bitfield>());
      for(unsigned r = 0; r < machine.regs[p].size(); ++r){
        Lang::VarDecl::Domain dom = machine.regs[p][r].domain;
        int mod = 1 + dom.get_upper_bound() - dom.get_lower_bound();
        int off = dom.get_lower_bound();
        reg_vec[p].push_back(bfseq.next(mod,off));
      }
    }
  }

  bits_len = bfseq.get_element_count();

};

VipsBitConstraint::VipsBitConstraint(const Common &common){
  if(common.pointer_pack){
    bits = (data_t*)1;
  }else{
    bits = new data_t[common.bits_len];
    for(int i = 0; i < common.bits_len; ++i){
      bits[i] = 0;
    }
    assert((data_t)bits % 2 == 0);
  }

  /* Setup pcs */
  /* Vacuously set all pcs to 0 */

  /* Setup mem and L1s */
  {
    for(unsigned i = 0; i < common.all_nmls.size(); ++i){
      int val;
      if(common.machine.get_var_decl(common.all_nmls[i]).value.is_wild()){
        val = common.machine.get_var_decl(common.all_nmls[i]).domain.get_lower_bound();
      }else{
        val = common.machine.get_var_decl(common.all_nmls[i]).value.get_value();
      }
      common.bfset(&bits,common.mem(common.all_nmls[i]),val);
      for(int pid = 0; pid < common.proc_count; ++pid){
        common.bfset(&bits,common.l1(pid,common.all_nmls[i]),common.l1val_clean(val));
      }
    }
  }

  /* Setup registers */
  {
    for(int p = 0; p < common.proc_count; ++p){
      for(unsigned r = 0; r < common.machine.regs[p].size(); ++r){
        int val;
        if(common.machine.regs[p][r].value.is_wild()){
          val = common.machine.regs[p][r].domain.get_lower_bound();
        }else{
          val = common.machine.regs[p][r].value.get_value();
        }
        common.bfset(&bits,common.reg(p,r),val);
      }
    }
  }
};

VipsBitConstraint::VipsBitConstraint(const Common &common, const VipsBitConstraint &vbc){
  if(common.pointer_pack){
    bits = vbc.bits;
  }else{
    bits = new data_t[common.bits_len];
    for(int i = 0; i < common.bits_len; ++i){
      bits[i] = vbc.bits[i];
    }
  }
};

VipsBitConstraint::~VipsBitConstraint(){
  if(!use_pointer_pack()){
    delete[] bits;
  }
};

VipsBitConstraint *VipsBitConstraint::post(const Common &c, 
                                           const Machine::PTransition &t) const{
  int pid = t.pid;
  const Lang::Stmt<int> &s = t.instruction;

  /* Check control state */
  if(c.bfget(bits,c.pcs[pid]) != t.source){
    return 0;
  }

  VipsBitConstraint *vbc = new VipsBitConstraint(c,*this);
  c.bfset(&vbc->bits,c.pcs[pid],t.target);

  switch(s.get_type()){
  case Lang::NOP:
    return vbc;
  case Lang::ASSIGNMENT:
    {
      int reg = s.get_reg();
      RegVal regval(pid,c,bits);
      int v = s.get_expr().eval<RegVal,int*>(regval,0);
      if(c.machine.regs[pid][reg].domain.member(v)){
        c.bfset(&vbc->bits,c.reg(pid,reg),v);
        return vbc;
      }else{
        /* t cannot be executed; computed value outside of domain */
        delete vbc;
        return 0;
      }
    }
  case Lang::ASSUME:
    {
      RegVal regval(pid,c,bits);
      if(s.get_condition().eval<RegVal,int*>(regval,0)){
        return vbc;
      }else{ /* Assume failed. */
        delete vbc;
        return 0;
      }
    }
  case Lang::FETCH:
    {
      int vd = c.bfget(bits,c.l1(pid,s.get_memloc()));
      if(c.l1val_is_dirty(vd)){
        /* Cannot perform necessary implicit evict */
        /* Cannot fetch */
        delete vbc;
        return 0;
      }
      int memval = c.bfget(bits,c.mem(Lang::NML(s.get_memloc(),pid)));
      c.bfset(&vbc->bits,c.l1(pid,s.get_memloc()),c.l1val_clean(memval));
      return vbc;
    }
  case Lang::WRLLC:
    {
      int vd = c.bfget(bits,c.l1(pid,s.get_memloc()));
      if(!c.l1val_is_dirty(vd)){
        /* Cannot write back value; it is clean */
        delete vbc;
        return 0;
      }
      int val = c.l1val_valof(vd);
      c.bfset(&vbc->bits,c.mem(Lang::NML(s.get_memloc(),pid)),val);
      c.bfset(&vbc->bits,c.l1(pid,s.get_memloc()),c.l1val_clean(val));
      return vbc;
    }
  case Lang::WRITE:
    {
      RegVal regval(pid,c,bits);
      int val = s.get_expr().eval<RegVal,int*>(regval,0);
      Lang::NML nml(s.get_memloc(),pid);
      if(!c.machine.get_var_decl(nml).domain.member(val)){
        /* Cannot write; value outside of domain */
        delete vbc;
        return 0;
      }
      c.bfset(&vbc->bits,c.l1(pid,s.get_memloc()),c.l1val_dirty(val));
      return vbc;
    }
  case Lang::READASSERT:
    {
      RegVal regval(pid,c,bits);
      int e_val = s.get_expr().eval<RegVal,int*>(regval,0);
      int v_val = c.l1val_valof(c.bfget(bits,c.l1(pid,s.get_memloc())));
      if(e_val == v_val){
        return vbc;
      }else{
        /* Read the wrong value */
        delete vbc;
        return 0;
      }
    }
  case Lang::READASSIGN:
    {
      int v_val = c.l1val_valof(c.bfget(bits,c.l1(pid,s.get_memloc())));
      if(c.machine.regs[pid][s.get_reg()].domain.member(v_val)){
        c.bfset(&vbc->bits,c.reg(pid,s.get_reg()),v_val);
        return vbc;
      }else{
        /* The read value is not in the register domain. */
        /* Cannot proceed with the read */
        delete vbc;
        return 0;
      }
    }
  case Lang::SYNCWR:
    {
      if(c.l1val_is_dirty(c.bfget(bits,c.l1(pid,s.get_memloc())))){
        /* Cannot execute implicit evict before syncwr */
        delete vbc;
        return 0;
      }
      RegVal regval(pid,c,bits);
      int val = s.get_expr().eval<RegVal,int*>(regval,0);
      Lang::NML nml(s.get_memloc(),pid);
      if(!c.machine.get_var_decl(nml).domain.member(val)){
        /* Cannot write; value outside of domain */
        delete vbc;
        return 0;
      }
      c.bfset(&vbc->bits,c.l1(pid,s.get_memloc()),c.l1val_clean(val));
      c.bfset(&vbc->bits,c.mem(nml),val);
      return vbc;
    }
  case Lang::FENCE:
  case Lang::LOCKED:
  case Lang::EVICT:
  default:
    delete vbc;
    throw new std::logic_error("VipsBitConstraint::post: Unsupported transition: "+t.to_string(c.machine));
  }

  delete vbc;
  throw new std::logic_error("VipsBitConstraint::post: Not implemented");
};

std::vector<int> VipsBitConstraint::get_control_states(const Common &common) const throw(){
  std::vector<int> pcs(common.proc_count);
  for(int p = 0; p < common.proc_count; ++p){
    pcs[p] = common.bfget(bits,common.pcs[p]);
  }
  return pcs;
};

VecSet<const Machine::PTransition*> VipsBitConstraint::partred(const Common &common) const{
  throw new std::logic_error("VipsBitConstraint::partred: Not implemented");
};

std::string VipsBitConstraint::to_string(const Common &common) const{
  throw new std::logic_error("VipsBitConstraint::to_string: Not implemented");
};

std::string VipsBitConstraint::debug_dump(const Common &common) const{
  std::stringstream ss;
  ss << "VBC(";
  if(common.pointer_pack){
    ss << "0x" << std::hex << (data_t)bits;
  }else{
    for(int i = 0; i < common.bits_len; ++i){
      if(i > 0) ss << " ";
      ss << "0x" << std::hex << (data_t)bits[i];
    }
  }
  ss << ")";
  return ss.str();
};

VipsBitConstraint::Common::bitfield::bitfield(int e, data_t d, data_t m, int off)
  : element(e), div(d), mod(m), offset(off) {
  assert(0 <= e);
  assert(0 < d);
  assert(d < std::numeric_limits<data_t>::max());
  assert(0 < m);
  assert(m <= std::numeric_limits<data_t>::max());
};

std::string VipsBitConstraint::Common::bitfield::to_string() const{
  std::stringstream ss;
  ss << "BF(";
  if(element != 0){
    ss << "e:" << element << ", ";
  }
  ss << "div:" << div << ", mod:" << mod;
  if(offset != 0){
    ss << ", off:" << offset;
  }
  ss << ")";
  return ss.str();
};

bool VipsBitConstraint::Common::possible_to_pointer_pack(const Machine &machine){
  data_t remains = std::numeric_limits<data_t>::max() / 2 + 1;

  /* The maximum allowed domain size for a memory location or register. */
  /* Sufficiently small to fit both value and dirty/clean state into one data_t. */
  data_t max_sz = std::numeric_limits<data_t>::max() / 2 + 1;
  
  /* pcs */
  for(unsigned p = 0; p < machine.automata.size(); ++p){
    if((data_t)machine.automata[p].get_states().size() > remains) return false;
    remains /= (data_t)machine.automata[p].get_states().size();
  }

  /* mem and L1s */
  {
    /* Global */
    for(unsigned i = 0; i < machine.gvars.size(); ++i){
      Lang::VarDecl::Domain dom = machine.gvars[i].domain;
      if(!dom.is_finite()){
        throw new std::logic_error("VipsBitConstraint: Global memory location "+machine.gvars[i].name+
                                   " has an infinite domain. Not supported.");
      }
      data_t sz = 1 + dom.get_upper_bound() - dom.get_lower_bound();
      if(sz > max_sz){
        throw new std::logic_error("VipsBitConstraint: Global memory location "+machine.gvars[i].name+
                                   " has a too large domain. Not supported.");
      }
      /* mem */
      if(sz > remains) return false;
      remains /= sz;
      /* L1s */
      for(unsigned l1 = 0; l1 < machine.automata.size(); ++l1){
        if(sz*2 > remains) return false;
        remains /= (2*sz);
      }
    }
    /* Local */
    for(unsigned p = 0; p < machine.lvars.size(); ++p){
      for(unsigned i = 0; i < machine.lvars[p].size(); ++i){
        Lang::VarDecl::Domain dom = machine.lvars[p][i].domain;
        if(!dom.is_finite()){
          std::stringstream ss;
          ss << "VipsBitConstraint: Local memory location " << machine.lvars[p][i].name
             << "[P" << p << "] has an infinite domain. Not supported.";
          throw new std::logic_error(ss.str());
        }
        data_t sz = 1 + dom.get_upper_bound() - dom.get_lower_bound();
        if(sz > max_sz){
          std::stringstream ss;
          ss << "VipsBitConstraint: Local memory location " << machine.lvars[p][i].name
             << "[P" << p << "] has a too large domain. Not supported.";
          throw new std::logic_error(ss.str());
        }
        /* mem */
        if(sz > remains) return false;
        remains /= sz;
        /* L1s */
        for(unsigned l1 = 0; l1 < machine.automata.size(); ++l1){
          if(sz*2 > remains) return false;
          remains /= (2*sz);
        }
      }
    }
  }

  /* Registers */
  {
    for(unsigned p = 0; p < machine.regs.size(); ++p){
      for(unsigned r = 0; r < machine.regs[p].size(); ++r){
        Lang::VarDecl::Domain dom = machine.regs[p][r].domain;
        if(!dom.is_finite()){
          std::stringstream ss;
          ss << "VipsBitConstraint: Register " << machine.regs[p][r].name
             << " of P" << p << " has an infinite domain. Not supported.";
          throw new std::logic_error(ss.str());
        }
        data_t sz = 1 + dom.get_upper_bound() - dom.get_lower_bound();
        if(sz > max_sz){
          std::stringstream ss;
          ss << "VipsBitConstraint: Register " << machine.regs[p][r].name
             << " of P" << p << " has a too large domain. Not supported.";
          throw new std::logic_error(ss.str());
        }
        if(sz > remains) return false;
        remains /= sz;
      }
    }
  }

  Log::debug << "VipsBitConstraint: remains: " << remains << "\n";

  return true;
};

void VipsBitConstraint::test(){
  /* Test bitfields */
  /* Test 1 */
  {
    std::stringstream ss;
    data_t e = 981290183;
    Common::bitfield bf(0,4,4,0);
    ss << bf.to_string() << ": get(set(" << e << ",2)) == 2";
    Test::inner_test("#1.1: "+ss.str(),bf.get_el(bf.set_el(e,2)) == 2);
  }

  /* Test 2 */
  {
    std::stringstream ss;
    data_t e = 18288972349823;
    Common::bitfield bf(0,13,27,0);
    ss << bf.to_string() << ": get(set(" << e << ",7)) == 7";
    Test::inner_test("#1.2: "+ss.str(),bf.get_el(bf.set_el(e,7)) == 7);
  }

  /* Test 3: lowest bit */
  {
    std::stringstream ss;
    data_t e = 0;
    Common::bitfield bf(0,1,8,0);
    ss << bf.to_string() << ": set(" << e << ",7) == 7";
    Test::inner_test("#1.3: "+ss.str(),bf.set_el(e,7) == 7);
  }

  /* Test 4,5: highest bits */
  {
    std::stringstream ss;
    data_t e = 0;
    Common::bitfield bf(0,(std::numeric_limits<data_t>::max()/4)+1,4,0);
    ss << bf.to_string() << ": get(set(" << e << ",3)) == 3";
    Test::inner_test("#1.4: "+ss.str(),bf.get_el(bf.set_el(e,3)) == 3);
    Test::inner_test("#1.5: "+ss.str()+" - no overflow",bf.set_el(e,3) % 2 == 0);
  }

  /* Test 6: longest possible field */
  {
    std::stringstream ss;
    data_t e = 0;
    Common::bitfield bf(0,1,std::numeric_limits<data_t>::max(),0);
    int v = std::numeric_limits<int>::max() - 1;
    ss << bf.to_string() << ": get(set(" << e << "," << v << ")) == " << v;
    Test::inner_test("#1.6: "+ss.str(),bf.get_el(bf.set_el(e,v)) == v);
  }

  /* Test 7,8: empty field */
  {
    std::stringstream ss, ss2;
    data_t e = 18288972349823;
    Common::bitfield bf(0,13,1,0);
    ss << bf.to_string() << ": get(set(" << e << ",0)) == 0";
    ss2 << bf.to_string() << ": set(" << e << ",0) == " << e;
    Test::inner_test("#1.7: "+ss.str(),bf.get_el(bf.set_el(e,0)) == 0);
    Test::inner_test("#1.8: "+ss2.str(),bf.set_el(e,0) == e);
  }

  /* Test 9,10,11: Negative values */
  {
    std::stringstream ss, ss2, ss3;
    data_t e = 23409;
    Common::bitfield bf(0,11,20,-10); /* range: [-10,9] */
    ss << bf.to_string() << ": get(set(" << e << ",-10)) == -10";
    ss2 << bf.to_string() << ": get(set(" << e << ",9)) == 9";
    ss3 << bf.to_string() << ": set(" << e << ",get(" << e << ")) == " << e;
    Test::inner_test("#1.9: "+ss.str(),bf.get_el(bf.set_el(e,-10)) == -10);
    Test::inner_test("#1.10: "+ss2.str(),bf.get_el(bf.set_el(e,9)) == 9);
    Test::inner_test("#1.11: "+ss3.str(),bf.set_el(e,bf.get_el(e)) == e);
  }

  /* Test fields in VipsBitConstraint */
  std::function<Machine*(std::string)> get_machine = 
    [](std::string rmm){
    std::stringstream ss(rmm);
    Lexer lex(ss);
    return new Machine(Parser::p_test(lex));
  };

  /* Test 1-11: bit packing (pointer_pack) */
  {
    /* Machine s.t. configurations (barely) fits into a 64-bit data_t. */
    Machine *m = get_machine
      ("forbidden * * *\n"
       "data\n"
       "  x = 0 : [0:1]\n"
       "  y = 1 : [-1:1]\n"
       "process\n"
       "data\n"
       "  flag = 1 : [1:1]\n"
       "registers\n"
       "  $r0 = 0 : [0:1]\n"
       "  $r1 = 2 : [2:3]\n"
       "text\n"
       "  nop\n\n"
       "process\n"
       "registers\n"
       "  $r0 = -1 : [-2:-1]\n"
       "data\n"
       "  a = 11 : [10:20]\n"
       "  b = * : [2:5]\n"
       "text\n"
       "  nop;nop;nop;nop\n\n"
       "process\n"
       "data\n"
       "  a = 1 : [0:1]\n"
       "text\n"
       "  nop;nop;nop\n");

    Common common(*m);

    /* Test 1,2: pcs */
    Test::inner_test("#2.1: Common pcs (init, basic)",
                     common.pcs.size() == 3 &&
                     common.pcs[0].mod == 3 &&
                     common.pcs[1].mod == 6 &&
                     common.pcs[2].mod == 5);

    VipsBitConstraint vbc(common);

    Log::result << vbc.debug_dump(common) << "\n";

    std::vector<int> pcs = vbc.get_control_states(common);
    Test::inner_test("#2.2: VBC pcs (init, basic)",
                     pcs.size() == 3 &&
                     pcs[0] == 0 &&
                     pcs[1] == 0 &&
                     pcs[2] == 0);

    /* Test 3,4,5: mem */
    Test::inner_test("#2.3: Common ml_offsets",
                     common.ml_offsets.size() == 3 &&
                     common.ml_offsets[0] == 2 &&
                     common.ml_offsets[1] == 3 &&
                     common.ml_offsets[2] == 5);

    Test::inner_test("#2.4: Common mem (init, basic)",
                     common.mem_vec.size() == 6 &&
                     common.mem_vec[0].mod == 2 &&
                     common.mem_vec[1].mod == 3 &&
                     common.mem_vec[2].mod == 1 &&
                     common.mem_vec[3].mod == 11 &&
                     common.mem_vec[4].mod == 4 &&
                     common.mem_vec[5].mod == 2);

    Test::inner_test("#2.5: VBC mem (init,basic)",
                     common.bfget(vbc.bits,common.mem(Lang::NML::global(0))) == 0 &&
                     common.bfget(vbc.bits,common.mem(Lang::NML::global(1))) == 1 &&
                     common.bfget(vbc.bits,common.mem(Lang::NML::local(0,0))) == 1 &&
                     common.bfget(vbc.bits,common.mem(Lang::NML::local(0,1))) == 11 &&
                     common.bfget(vbc.bits,common.mem(Lang::NML::local(1,1))) >= 2 &&
                     common.bfget(vbc.bits,common.mem(Lang::NML::local(1,1))) <= 5 &&
                     common.bfget(vbc.bits,common.mem(Lang::NML::local(0,2))) == 1);

    /* Test 6-10: L1 */
    Test::inner_test("#2.6: VBC L1 (init,basic)",
                     !common.l1val_is_dirty(common.bfget(vbc.bits,common.l1(0,Lang::NML::local(1,1)))) &&
                     !common.l1val_is_dirty(common.bfget(vbc.bits,common.l1(2,Lang::NML::global(0)))) &&
                     common.l1val_valof(common.bfget(vbc.bits,common.l1(1,Lang::NML::local(0,1)))) == 11 &&
                     common.l1val_valof(common.bfget(vbc.bits,common.l1(0,Lang::NML::local(0,2)))) == 1);

    Test::inner_test("#2.7: l1val_valof/l1val_clean",
                     common.l1val_valof(common.l1val_clean(-3)) == -3 &&
                     common.l1val_valof(common.l1val_clean(-1)) == -1 &&
                     common.l1val_valof(common.l1val_clean(0)) == 0 &&
                     common.l1val_valof(common.l1val_clean(1)) == 1 &&
                     common.l1val_valof(common.l1val_clean(2)) == 2);

    Test::inner_test("#2.8: l1val_valof/l1val_dirty",
                     common.l1val_valof(common.l1val_dirty(-3)) == -3 &&
                     common.l1val_valof(common.l1val_dirty(-1)) == -1 &&
                     common.l1val_valof(common.l1val_dirty(0)) == 0 &&
                     common.l1val_valof(common.l1val_dirty(1)) == 1 &&
                     common.l1val_valof(common.l1val_dirty(2)) == 2);

    Test::inner_test("#2.9: l1val_is_dirty/l1val_dirty",
                     common.l1val_is_dirty(common.l1val_dirty(-3)) &&
                     common.l1val_is_dirty(common.l1val_dirty(-1)) &&
                     common.l1val_is_dirty(common.l1val_dirty(0)) &&
                     common.l1val_is_dirty(common.l1val_dirty(1)) &&
                     common.l1val_is_dirty(common.l1val_dirty(2)));

    Test::inner_test("#2.10: l1val_is_dirty/l1val_clean",
                     !common.l1val_is_dirty(common.l1val_clean(-3)) &&
                     !common.l1val_is_dirty(common.l1val_clean(-1)) &&
                     !common.l1val_is_dirty(common.l1val_clean(0)) &&
                     !common.l1val_is_dirty(common.l1val_clean(1)) &&
                     !common.l1val_is_dirty(common.l1val_clean(2)));


    /* Test 11: Registers */
    Test::inner_test("#2.11: VBC registers (init,basic)",
                     common.bfget(vbc.bits,common.reg(0,0)) == 0 && 
                     common.bfget(vbc.bits,common.reg(0,1)) == 2 && 
                     common.bfget(vbc.bits,common.reg(1,0)) == -1);

    delete m;
  }

  /* Test 12-22: bit packing (pointer_pack) */
  {
    /* Machine s.t. configurations do not fit into a 64-bit data_t. */
    Machine *m = get_machine
      ("forbidden * * *\n"
       "data\n"
       "  x = 0 : [0:5]\n" /* Changed domain [0:1] -> [0:5] */
       "  y = 1 : [-1:1]\n"
       "process\n"
       "data\n"
       "  flag = 1 : [1:1]\n"
       "registers\n"
       "  $r0 = 0 : [0:1]\n"
       "  $r1 = 2 : [2:3]\n"
       "text\n"
       "  nop\n\n"
       "process\n"
       "registers\n"
       "  $r0 = -1 : [-2:-1]\n"
       "data\n"
       "  a = 11 : [10:20]\n"
       "  b = * : [2:5]\n"
       "text\n"
       "  nop;nop;nop;nop\n\n"
       "process\n"
       "data\n"
       "  a = 1 : [0:1]\n"
       "text\n"
       "  nop;nop;nop\n");

    Common common(*m);

    /* Test 12,13: pcs */
    Test::inner_test("#2.12: Common pcs (init, basic)",
                     common.pcs.size() == 3 &&
                     common.pcs[0].mod == 3 &&
                     common.pcs[1].mod == 6 &&
                     common.pcs[2].mod == 5);

    VipsBitConstraint vbc(common);

    Log::result << vbc.debug_dump(common) << "\n";

    std::vector<int> pcs = vbc.get_control_states(common);
    Test::inner_test("#2.13: VBC pcs (init, basic)",
                     pcs.size() == 3 &&
                     pcs[0] == 0 &&
                     pcs[1] == 0 &&
                     pcs[2] == 0);

    /* Test 14,15,16: mem */
    Test::inner_test("#2.14: Common ml_offsets",
                     common.ml_offsets.size() == 3 &&
                     common.ml_offsets[0] == 2 &&
                     common.ml_offsets[1] == 3 &&
                     common.ml_offsets[2] == 5);

    Test::inner_test("#2.15: Common mem (init, basic)",
                     common.mem_vec.size() == 6 &&
                     common.mem_vec[0].mod == 6 &&
                     common.mem_vec[1].mod == 3 &&
                     common.mem_vec[2].mod == 1 &&
                     common.mem_vec[3].mod == 11 &&
                     common.mem_vec[4].mod == 4 &&
                     common.mem_vec[5].mod == 2);

    Test::inner_test("#2.16: VBC mem (init,basic)",
                     common.bfget(vbc.bits,common.mem(Lang::NML::global(0))) == 0 &&
                     common.bfget(vbc.bits,common.mem(Lang::NML::global(1))) == 1 &&
                     common.bfget(vbc.bits,common.mem(Lang::NML::local(0,0))) == 1 &&
                     common.bfget(vbc.bits,common.mem(Lang::NML::local(0,1))) == 11 &&
                     common.bfget(vbc.bits,common.mem(Lang::NML::local(1,1))) >= 2 &&
                     common.bfget(vbc.bits,common.mem(Lang::NML::local(1,1))) <= 5 &&
                     common.bfget(vbc.bits,common.mem(Lang::NML::local(0,2))) == 1);

    /* Test 17-21: L1 */
    Test::inner_test("#2.17: VBC L1 (init,basic)",
                     !common.l1val_is_dirty(common.bfget(vbc.bits,common.l1(0,Lang::NML::local(1,1)))) &&
                     !common.l1val_is_dirty(common.bfget(vbc.bits,common.l1(2,Lang::NML::global(0)))) &&
                     common.l1val_valof(common.bfget(vbc.bits,common.l1(1,Lang::NML::local(0,1)))) == 11 &&
                     common.l1val_valof(common.bfget(vbc.bits,common.l1(0,Lang::NML::local(0,2)))) == 1);

    Test::inner_test("#2.18: l1val_valof/l1val_clean",
                     common.l1val_valof(common.l1val_clean(-3)) == -3 &&
                     common.l1val_valof(common.l1val_clean(-1)) == -1 &&
                     common.l1val_valof(common.l1val_clean(0)) == 0 &&
                     common.l1val_valof(common.l1val_clean(1)) == 1 &&
                     common.l1val_valof(common.l1val_clean(2)) == 2);

    Test::inner_test("#2.19: l1val_valof/l1val_dirty",
                     common.l1val_valof(common.l1val_dirty(-3)) == -3 &&
                     common.l1val_valof(common.l1val_dirty(-1)) == -1 &&
                     common.l1val_valof(common.l1val_dirty(0)) == 0 &&
                     common.l1val_valof(common.l1val_dirty(1)) == 1 &&
                     common.l1val_valof(common.l1val_dirty(2)) == 2);

    Test::inner_test("#2.20: l1val_is_dirty/l1val_dirty",
                     common.l1val_is_dirty(common.l1val_dirty(-3)) &&
                     common.l1val_is_dirty(common.l1val_dirty(-1)) &&
                     common.l1val_is_dirty(common.l1val_dirty(0)) &&
                     common.l1val_is_dirty(common.l1val_dirty(1)) &&
                     common.l1val_is_dirty(common.l1val_dirty(2)));

    Test::inner_test("#2.21: l1val_is_dirty/l1val_clean",
                     !common.l1val_is_dirty(common.l1val_clean(-3)) &&
                     !common.l1val_is_dirty(common.l1val_clean(-1)) &&
                     !common.l1val_is_dirty(common.l1val_clean(0)) &&
                     !common.l1val_is_dirty(common.l1val_clean(1)) &&
                     !common.l1val_is_dirty(common.l1val_clean(2)));


    /* Test 22,23: Registers */
    Test::inner_test("#2.22: VBC registers (init,basic)",
                     common.bfget(vbc.bits,common.reg(0,0)) == 0 && 
                     common.bfget(vbc.bits,common.reg(0,1)) == 2 && 
                     common.bfget(vbc.bits,common.reg(1,0)) == -1);

    RegVal regval(0,common,vbc.bits);
    Lang::Expr<int> e = Lang::Expr<int>::reg(0) + Lang::Expr<int>::reg(1) + Lang::Expr<int>::integer(1);
    Test::inner_test("#2.23: VBC register, expr evaluation",
                     e.eval<RegVal,int*>(regval,0) == 3);

    delete m;
  }

  /* Test semantics */
  {
    Machine *m = get_machine
      ("forbidden * *\n"
       "data\n"
       "  x = 0 : [0:5]\n"
       "  y = 0 : [0:5]\n"
       "process\n"
       "registers\n"
       "  $r0 = 0 : [0:5]\n"
       "  $r1 = 0 : [0:5]\n"
       "text\n"
       "  Lnop: nop;\n"
       "  Lnop1: either{\n"
       "    write: x := 1;\n"
       "    Lreads:\n"
       "    either{\n"
       "      read: x = 1; goto A\n"
       "    or\n"
       "      read: x = 0; goto B\n"
       "    or\n"
       "      read: $r0 := x; goto C\n"
       "    }\n"
       "  or\n"
       "    syncwr: x := 2; goto D\n"
       "  };\n"
       "  END: nop;\n"
       "  A: nop;\n"
       "  B: nop;\n"
       "  C: nop;\n"
       "  D: nop;\n"
       "  FAIL: nop\n"
       "process\n"
       "registers\n"
       "  $r0 = 0 : [0:5]\n"
       "  $r1 = 0 : [0:5]\n"
       "text\n"
       "  Lass0: $r0 := 1;\n"
       "  Lass1: $r1 := 2;\n"
       "  Lass2: $r1 := $r0 + $r1;\n"
       "  Lass3: either{\n"
       "    assume: $r0 + $r1 = 4\n"
       "  or\n"
       "    $r1 := $r1 + $r1;\n" /* outside domain */
       "    goto FAIL\n"
       "  };\n"
       "  Lassu1: either{\n"
       "    assume: $r1 = 2;\n"
       "    goto FAIL\n"
       "  or\n"
       "    nop"
       "  };\n"
       "  END: nop;\n"
       "  FAIL: nop\n"
       );

    Common common(*m);
    VipsBitConstraint vbc(common);

    Lang::NML x = Lang::NML::global(0);
    Lang::NML y = Lang::NML::global(1);

    /* Return some PTransition from m which originates in the control
     * state of process pid which is labelled lbl.
     */
    std::function<Machine::PTransition(int,std::string)> trans = 
      [m](int pid, std::string lbl){
      int s = m->automata[pid].state_index_of_label(lbl);
      Automaton::Transition *t = *m->automata[pid].get_states()[s].fwd_transitions.begin();
      return Machine::PTransition(*t,pid);
    };

    /* Return some PTransition from m which originates in the control
     * state of process pid which is labelled srclbl, and targets the
     * control state labelled tgtlbl.
     */
    std::function<Machine::PTransition(int,std::string,std::string)> trans2 = 
      [m](int pid, std::string srclbl, std::string tgtlbl){
      int src = m->automata[pid].state_index_of_label(srclbl);
      int tgt = m->automata[pid].state_index_of_label(tgtlbl);
      Automaton::Transition *t = 0;
      for(auto it = m->automata[pid].get_states()[src].fwd_transitions.begin(); 
          it != m->automata[pid].get_states()[src].fwd_transitions.end();
          ++it){
        if((*it)->target == tgt){
          t = *it;
        }
      }
      return Machine::PTransition(*t,pid);
    };

    /* Return a fetch for process pid and memory location nml,
     * originating in the control state labelled lbl
     */
    std::function<Machine::PTransition(int,std::string,Lang::NML)> fetch =
      [m](int pid, std::string lbl, Lang::NML nml){
      int src = m->automata[pid].state_index_of_label(lbl);
      return Machine::PTransition(src,Lang::Stmt<int>::fetch(nml.localize(pid)),src,pid);
    };

    /* Return a wrllc for process pid and memory location nml,
     * originating in the control state labelled lbl
     */
    std::function<Machine::PTransition(int,std::string,Lang::NML)> wrllc =
      [m](int pid, std::string lbl, Lang::NML nml){
      int src = m->automata[pid].state_index_of_label(lbl);
      return Machine::PTransition(src,Lang::Stmt<int>::wrllc(nml.localize(pid)),src,pid);
    };

    /* Test nop */
    {
      VipsBitConstraint *vbc2 = vbc.post(common,trans(0,"Lnop"));
      Test::inner_test("#3.1: nop",
                       vbc2 != 0 &&
                       common.bfget(vbc2->bits,common.pcs[0]) == 1 &&
                       common.bfget(vbc2->bits,common.pcs[1]) == 0);

      delete vbc2;

      vbc2 = vbc.post(common,trans(0,"Lnop1"));
      Test::inner_test("#3.2: nop", vbc2 == 0);

      // Do not delete vbc2 (it is null)
    }

    /* Test assignment */
    {
      VipsBitConstraint *vbc2 = vbc.post(common,trans(1,"Lass0"));
      Test::inner_test("#3.3: assignment",
                       vbc2 != 0 &&
                       common.bfget(vbc2->bits,common.pcs[0]) == 0 &&
                       common.bfget(vbc2->bits,common.pcs[1]) == 1 &&
                       common.bfget(vbc2->bits,common.reg(1,0)) == 1 &&
                       common.bfget(vbc2->bits,common.reg(1,1)) == 0 && 
                       common.bfget(vbc2->bits,common.reg(0,0)) == 0 &&
                       common.bfget(vbc2->bits,common.reg(0,1)) == 0);

      VipsBitConstraint *vbc3 = vbc2->post(common,trans(1,"Lass1"));
      Test::inner_test("#3.4: assignment",
                       vbc3 != 0 &&
                       common.bfget(vbc3->bits,common.reg(1,1)) == 2);

      VipsBitConstraint *vbc4 = vbc3->post(common,trans(1,"Lass2"));
      Test::inner_test("#3.5: assignment",
                       vbc4 != 0 &&
                       common.bfget(vbc4->bits,common.reg(1,1)) == 3);

      VipsBitConstraint *vbc5 = vbc4->post(common,trans2(1,"Lass3","FAIL"));
      Test::inner_test("#3.6: assignment", vbc5 == 0);

      delete vbc2;
      delete vbc3;
      delete vbc4;
      // Do not delete vbc5
    }

    /* Test assume */
    {
      
      VipsBitConstraint *vbc2 = vbc.post(common,trans(1,"Lass0"));
      VipsBitConstraint *vbc3 = vbc2->post(common,trans(1,"Lass1"));
      VipsBitConstraint *vbc4 = vbc3->post(common,trans(1,"Lass2"));
      VipsBitConstraint *vbc5 = vbc4->post(common,trans2(1,"Lass3","Lassu1"));
      Test::inner_test("#3.7: assume", vbc5 != 0);
      VipsBitConstraint *vbc6 = vbc5->post(common,trans2(1,"Lassu1","FAIL"));
      Test::inner_test("#3.8: assume",vbc6 == 0);

      delete vbc2;
      delete vbc3;
      delete vbc4;
      delete vbc5;
      // Do not delete vbc6
    }

    /* Test write, fetch, wrllc */
    {
      VipsBitConstraint *vbc2 = vbc.post(common,trans(0,"Lnop"));
      VipsBitConstraint *vbc3 = vbc2->post(common,fetch(0,"Lnop1",x));
      Test::inner_test("#3.9: fetch",
                       vbc3 != 0 &&
                       !common.l1val_is_dirty(common.bfget(vbc3->bits,common.l1(0,x))) &&
                       common.l1val_valof(common.bfget(vbc3->bits,common.l1(0,x))) == 0);
      VipsBitConstraint *vbc5 = vbc2->post(common,wrllc(0,"Lnop1",x));
      Test::inner_test("#3.10: wrllc", vbc5 == 0);
      VipsBitConstraint *vbc4 = vbc2->post(common,trans2(0,"Lnop1","Lreads"));
      Test::inner_test("#3.11: write",
                       vbc4 != 0 &&
                       common.l1val_is_dirty(common.bfget(vbc4->bits,common.l1(0,x))) &&
                       common.l1val_valof(common.bfget(vbc4->bits,common.l1(0,x))) == 1 &&
                       !common.l1val_is_dirty(common.bfget(vbc4->bits,common.l1(1,x))) &&
                       common.l1val_valof(common.bfget(vbc4->bits,common.l1(1,x))) == 0 &&
                       common.bfget(vbc4->bits,common.mem(x)) == 0);
      VipsBitConstraint *vbc6 = vbc4->post(common,fetch(0,"Lreads",x));
      Test::inner_test("#3.12: fetch",vbc6 == 0);
      VipsBitConstraint *vbc7 = vbc4->post(common,fetch(0,"Lreads",y));
      Test::inner_test("#3.13: fetch",
                       vbc7 != 0 &&
                       !common.l1val_is_dirty(common.bfget(vbc7->bits,common.l1(0,y))) &&
                       common.l1val_valof(common.bfget(vbc7->bits,common.l1(0,y))) == 0);
      VipsBitConstraint *vbc8 = vbc4->post(common,wrllc(0,"Lreads",x));
      Test::inner_test("#3.14: wrllc",
                       vbc8 != 0 &&
                       !common.l1val_is_dirty(common.bfget(vbc8->bits,common.l1(0,x))) &&
                       common.l1val_valof(common.bfget(vbc8->bits,common.l1(0,x))) == 1 &&
                       !common.l1val_is_dirty(common.bfget(vbc8->bits,common.l1(1,x))) &&
                       common.l1val_valof(common.bfget(vbc8->bits,common.l1(1,x))) == 0 &&
                       common.bfget(vbc8->bits,common.mem(x)) == 1);

      delete vbc2;
      delete vbc3;
      delete vbc4;
      // Do not delete vbc5
      // Do not delete vbc6
      delete vbc7;
      delete vbc8;
    }

    /* Test read */
    {
      VipsBitConstraint *vbc2 = vbc.post(common,trans(0,"Lnop"));
      VipsBitConstraint *vbc3 = vbc2->post(common,trans2(0,"Lnop1","Lreads")); // write: x := 1
      VipsBitConstraint *vbc4 = vbc3->post(common,trans2(0,"Lreads","A")); // read: x = 1
      Test::inner_test("#3.15: read assert", vbc4 != 0);
      VipsBitConstraint *vbc5 = vbc3->post(common,wrllc(0,"Lreads",x));
      VipsBitConstraint *vbc6 = vbc5->post(common,trans2(0,"Lreads","A")); // read: x = 1
      Test::inner_test("#3.16: read assert", vbc6 != 0);
      VipsBitConstraint *vbc7 = vbc3->post(common,trans2(0,"Lreads","B")); // read: x = 0
      Test::inner_test("#3.17: read assert", vbc7 == 0);
      VipsBitConstraint *vbc8 = vbc3->post(common,trans2(0,"Lreads","C")); // read: $r0 := x
      Test::inner_test("#3.18: read assign",
                       vbc8 != 0 &&
                       common.bfget(vbc8->bits,common.reg(0,0)) == 1);
      

      delete vbc2;
      delete vbc3;
      delete vbc4;
      delete vbc5;
      delete vbc6;
      // Do not delete vbc7
      delete vbc8;
    }

    /* Test syncwr */
    {
      VipsBitConstraint *vbc2 = vbc.post(common,trans(0,"Lnop"));
      VipsBitConstraint *vbc3 = vbc2->post(common,trans2(0,"Lnop1","D")); // syncwr: x := 2

      Test::inner_test("#3.19: syncwr",
                       vbc3 != 0 &&
                       common.bfget(vbc3->bits,common.mem(x)) == 2 &&
                       !common.l1val_is_dirty(common.bfget(vbc3->bits,common.l1(0,x))) &&
                       common.l1val_valof(common.bfget(vbc3->bits,common.l1(0,x))) == 2);

      delete vbc2;
      delete vbc3;
    }

    delete m;
  }
  
};
