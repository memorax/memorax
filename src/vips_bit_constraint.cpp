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

const VipsBitConstraint::data_t VipsBitConstraint::Common::ptr_max = VipsBitConstraint::Common::calc_ptr_max();

VipsBitConstraint::Common::Common(const Machine &m) : machine(m) {
  proc_count = m.automata.size();
  if(possible_to_pointer_pack(m)){
    pointer_pack = true;
    bits_len = 0;
    Log::debug << "VipsBitConstraint::Common: Using pointer packing.\n";
  }else{
    pointer_pack = false;
    bits_len = 1;
    Log::debug << "VipsBitConstraint::Common: Not using pointer packing.\n";
  }

  /* Set up bitfields */
  int div = 2;
  int element = 0;
  for(int p = 0; p < proc_count; ++p){
    int mod = (int)m.automata[p].get_states().size() + 1;
    pcs.push_back(bitfield(element,div,mod,0));
    div *= mod;
  }
};

VipsBitConstraint::VipsBitConstraint(const Common &common){
  if(common.pointer_pack){
    bits = (data_t*)1;
  }else{
    bits = new data_t[common.bits_len];
    for(int i = 0; i < common.bits_len; ++i){
      bits[i] = 0;
    }
    assert((ulong)bits % 2 == 0);
  }

  /* Setup pcs */
  /* Vacuously set all pcs to 0 */
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

VipsBitConstraint VipsBitConstraint::post(const Common &common, 
                                          const Machine::PTransition &t) const{
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
  throw new std::logic_error("VipsBitConstraint::debug_dump: Not implemented");
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
  data_t remains = ptr_max / 2 + 1;
  
  /* pcs */
  for(unsigned p = 0; p < machine.automata.size(); ++p){
    if((data_t)machine.automata[p].get_states().size() > remains) return false;
    remains /= (data_t)machine.automata[p].get_states().size();
  }

  return true;
};

VipsBitConstraint::data_t VipsBitConstraint::Common::calc_ptr_max(){
  if(sizeof(data_t*) >= sizeof(data_t)){
    return std::numeric_limits<data_t>::max();
  }else{
    // Bits per unit used by sizeof
    int bpc = std::numeric_limits<unsigned char>::digits / sizeof(unsigned char);

    /* Calculate 2^(bpc*sizeof(data_t*))-1 */
    data_t d = 1;
    for(int i = 0; i < (int)sizeof(data_t*)*bpc - 1; ++i){
      d *= 2;
    }
    d -= 1;
    d *= 2;
    d += 1;
    return d;
  }
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

  /* Test 1: pcs */
  {
    Machine *m = get_machine
      ("forbidden * * *\n"
       "process\n"
       "text\n"
       "  nop\n\n"
       "process\n"
       "text\n"
       "  nop;nop;nop;nop\n\n"
       "process\n"
       "text\n"
       "  nop;nop;nop\n");

    Common common(*m);
    Test::inner_test("#2.1: Common pcs (init, basic)",
                     common.pcs.size() == 3 &&
                     common.pcs[0].mod == 3 &&
                     common.pcs[1].mod == 6 &&
                     common.pcs[2].mod == 5);

    VipsBitConstraint vbc(common);
    std::vector<int> pcs = vbc.get_control_states(common);
    Test::inner_test("#2.2: VBC pcs (init, basic)",
                     pcs.size() == 3 &&
                     pcs[0] == 0 &&
                     pcs[1] == 0 &&
                     pcs[2] == 0);
    delete m;
  }
  
};
