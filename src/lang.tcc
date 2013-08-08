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

#include <sstream>
#include <exception>
#include <algorithm>

template<class RegId> std::set<RegId> Lang::Expr<RegId>::get_registers() const throw(){
  std::set<RegId> s;
  for(int i = 0; i < this->const_count; i++){
    s.insert(this->consts[i]);
  }
  return s;
};

template<class RegId> template<class RegId2> Lang::Expr<RegId2> Lang::Expr<RegId>::convert(std::function<RegId2(const RegId&)> &rc) const{
  return Expr<RegId2>(SyntaxString<RegId>::template convert<RegId2>(rc));
};

template<class RegId> std::set<RegId> Lang::BExpr<RegId>::get_registers() const throw(){
  std::set<RegId> s;
  for(int i = 0; i < this->const_count; i++){
    s.insert(this->consts[i]);
  }
  return s;
};

template<class RegId> template<class RegId2> 
Lang::BExpr<RegId2> Lang::BExpr<RegId>::convert(std::function<RegId2(const RegId&)> &rc) const{
  return BExpr<RegId2>(SyntaxString<RegId>::template convert<RegId2>(rc));
};

template<class Id> Lang::MemLoc<Id> Lang::MemLoc<Id>::change_caller(int old_caller, int new_caller) const throw(){
  switch(type){
  case GLOBAL_ID:
    return *this;
  case LOCAL:
    {
      int owner_pid;
      if(owner == -1){
        owner_pid = old_caller;
      }else if(owner < old_caller){
        owner_pid = owner;
      }else{
        owner_pid = owner+1;
      }
      if(owner_pid == new_caller){
        return local(id);
      }else if(owner_pid < new_caller){
        return local(id,owner_pid);
      }else{
        return local(id,owner_pid-1);
      }
    }
  default:
    throw new std::logic_error("Lang::MemLoc::change_caller: Unsupported MemLoc type.");
  }
};

template<class Id> bool Lang::MemLoc<Id>::operator==(const MemLoc<Id> &ml) const throw(){
  if(type == ml.type){
    switch(type){
    case GLOBAL_ID: return id == ml.id;
    case GLOBAL_INT_DEREF: return ptr == ml.ptr;
    case GLOBAL_REG_DEREF: return id == ml.id;
    case LOCAL: return id == ml.id && owner == ml.owner;
    }
    throw new std::logic_error("Lang::MemLoc::operator==: Invalid value for type.");
  }else{
    return false;
  }
}

template<class Id> bool Lang::MemLoc<Id>::operator<(const MemLoc<Id> &ml) const throw(){
  if(type < ml.type){
    return true;
  }else if(type > ml.type){
    return false;
  }else{
    switch(type){
    case GLOBAL_ID:
      return id < ml.id;
    case GLOBAL_INT_DEREF:
      return ptr < ml.ptr;
    case GLOBAL_REG_DEREF:
      return id < ml.id;
    case LOCAL:
      return (id < ml.id) || (id == ml.id && owner < ml.owner);
    }
    throw new std::logic_error("Lang::MemLoc::operator<: Invalid value for type.");
  }
}

template<class Id> Lang::MemLoc<Id> Lang::MemLoc<Id>::global(Id i) throw(){
  MemLoc<Id> ml;
  ml.type = GLOBAL_ID;
  ml.id = i;
  return ml;
}

template<class Id> Lang::MemLoc<Id> Lang::MemLoc<Id>::int_deref(int i) throw(){
  MemLoc<Id> ml;
  ml.type = GLOBAL_INT_DEREF;
  ml.ptr = i;
  return ml;
}

template<class Id> Lang::MemLoc<Id> Lang::MemLoc<Id>::reg_deref(Id reg) throw(){
  MemLoc<Id> ml;
  ml.type = GLOBAL_REG_DEREF;
  ml.id = reg;
  return ml;
}

template<class Id> Lang::MemLoc<Id> Lang::MemLoc<Id>::local(Id id) throw(){
  MemLoc<Id> ml;
  ml.type = LOCAL;
  ml.id = id;
  ml.owner = -1;
  return ml;
}

template<class Id> Lang::MemLoc<Id> Lang::MemLoc<Id>::local(Id id,int p) throw(){
  MemLoc<Id> ml;
  ml.type = LOCAL;
  ml.id = id;
  ml.owner = p;
  return ml;
}

template<class Id> typename Lang::MemLoc<Id>::Type Lang::MemLoc<Id>::get_type() const throw(){
  return type;
}

template<class Id> bool Lang::MemLoc<Id>::is_global() const throw(){
  return type != LOCAL;
}

template<class Id> bool Lang::MemLoc<Id>::is_local() const throw(){
  return type == LOCAL;
}

template<class Id> Id Lang::MemLoc<Id>::get_id() const throw(){
  return id;
}

template<class Id> Id Lang::MemLoc<Id>::get_reg() const throw(){
  return id;
}

template<class Id> int Lang::MemLoc<Id>::get_pointer() const throw(){
  return ptr;
}

template<class Id> int Lang::MemLoc<Id>::get_owner(int caller) const throw(){
  if(owner == -1)
    return caller;
  if(owner < caller)
    return owner;
  return owner+1;
}

template<class Id> Lang::MemLoc<Id>::MemLoc() {}

template<class Id> std::string Lang::MemLoc<Id>::to_string() const throw(){
  std::stringstream ss;
  switch(type){
  case GLOBAL_ID: ss << "var:" << id; break;
  case GLOBAL_INT_DEREF: ss << "[@" << ptr << "]"; break;
  case GLOBAL_REG_DEREF: ss << "[$" << id << "]"; break;
  case LOCAL: 
    if(owner == -1)
      ss << "var:" << id << "[my]";
    else
      ss << "var:" << id << "[" << owner << "]";
  }
  return ss.str();
}

template<class Id> std::set<Id> Lang::MemLoc<Id>::get_registers() const throw(){
  std::set<Id> s;
  if(type == GLOBAL_REG_DEREF)
    s.insert(id);
  return s;
}

inline Lang::NML::NML(const Lang::MemLoc<int> &ml, int caller) throw(std::logic_error*){
  switch(ml.get_type()){
  case MemLoc<int>::GLOBAL_ID:
    id = ml.get_id();
    owner = -1;
    break;
  case MemLoc<int>::GLOBAL_INT_DEREF:
    id = ml.get_pointer();
    owner = -1;
    break;
  case MemLoc<int>::GLOBAL_REG_DEREF:
    throw new std::logic_error("Lang::NML: Attempt to construct Normalized MemLoc "
                               "from MemLoc which is register dereference.");
  case MemLoc<int>::LOCAL:
    id = ml.get_id();
    owner = ml.get_owner(caller);
    break;
  }
}

inline Lang::NML::NML(int id) : owner(-1), id(id) {}

inline Lang::NML::NML(int id, int owner) : owner(owner), id(id) {}

inline std::string Lang::NML::to_string() const throw(){
  std::stringstream ss;
  if(is_global()){
    ss << "gvar:" << id;
  }else{
    ss << "lvar:P" 
       << owner << ":" 
       << id;
  }
  return ss.str();
}

inline bool Lang::NML::operator==(const NML &nml) const throw(){
  return owner == nml.owner && id == nml.id;
}

inline bool Lang::NML::operator<(const NML &nml) const throw(){
  return (owner < nml.owner) || ((owner == nml.owner) && (id < nml.id));
}

template<class RegId> Lang::Stmt<RegId>::Stmt(const Lexer::TokenPos &p) :
  type(NOP), e0(0), e1(0), b(0), 
  stmts(0), fence(false), lbl(""), 
  writer(-1), stmt_count(0), pos(p)
{
};

template<class RegId> void Lang::Stmt<RegId>::self_destruct(){
  writes.clear();
  reads.clear();
  if(e0){
    delete e0;
    e0 = 0;
  }
  if(e1){
    delete e1;
    e1 = 0;
  }
  if(b){
    delete b;
    b = 0;
  }
  if(stmts){
    delete[] stmts;
    stmts = 0;
    stmt_count = 0;
  }
};

template<class RegId> Lang::Stmt<RegId>::~Stmt() throw(){
  self_destruct();
};

template<class RegId> Lang::Stmt<RegId>::Stmt(const Stmt &s) :
type(s.type), reg(s.reg), writes(s.writes), reads(s.reads), e0(0), e1(0), b(0),
stmts(0), fence(s.fence), lbl(s.lbl), writer(s.writer), stmt_count(s.stmt_count), pos(s.pos)
{
  if(s.e0) e0 = new Expr<RegId>(*s.e0);
  if(s.e1) e1 = new Expr<RegId>(*s.e1);
  if(s.b) b = new BExpr<RegId>(*s.b);
  if(s.stmts){
    stmts = new labeled_stmt_t[stmt_count];
    for(int i = 0; i < stmt_count; i++){
      stmts[i] = s.stmts[i];
    }
  }
};

template<class RegId> Lang::Stmt<RegId> &Lang::Stmt<RegId>::operator=(const Stmt<RegId> &s){
  if(&s != this){
    self_destruct();
    type = s.type;
    reg = s.reg;
    writes = s.writes;
    reads = s.reads;
    if(s.e0) e0 = new Expr<RegId>(*s.e0);
    if(s.e1) e1 = new Expr<RegId>(*s.e1);
    if(s.b) b = new BExpr<RegId>(*s.b);
    if(s.stmts){
      stmts = new labeled_stmt_t[s.stmt_count];
      for(int i = 0; i < s.stmt_count; i++){
        stmts[i] = s.stmts[i];
      }
    }
    fence = s.fence;
    lbl = s.lbl;
    writer = s.writer;
    stmt_count = s.stmt_count;
    pos = s.pos;
  }
  return *this;
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::nop(const Lexer::TokenPos &p)
{
  return Stmt<RegId>(p);
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::assignment(RegId reg, const Expr<RegId> &e,
                                                const Lexer::TokenPos &p){
  Stmt<RegId> s;
  s.type = ASSIGNMENT;
  s.reg = reg;
  s.e0 = new Expr<RegId>(e);
  s.pos = p;
  return s;
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::assume(const BExpr<RegId> &b,
                                            const Lexer::TokenPos &p){
  Stmt<RegId> s;
  s.type = ASSUME;
  s.b = new BExpr<RegId>(b);
  s.pos = p;
  return s;
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::read_assert(MemLoc<RegId> ml, const Expr<RegId> &e,
                                                 const Lexer::TokenPos &p){
  Stmt<RegId> s;
  s.type = READASSERT;
  s.reads.insert(ml);
  s.e0 = new Expr<RegId>(e);
  s.pos = p;
  return s;
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::read_assign(RegId reg, MemLoc<RegId> ml,
                                                 const Lexer::TokenPos &p){
  Stmt<RegId> s;
  s.type = READASSIGN;
  s.reg = reg;
  s.reads.insert(ml);
  s.pos = p;
  return s;
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::write(MemLoc<RegId> ml, const Expr<RegId> &e,
                                           const Lexer::TokenPos &p){
  Stmt<RegId> s(p);
  s.type = WRITE;
  s.writes.insert(ml);
  s.e0 = new Expr<RegId>(e);
  return s;
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::locked_block(const std::vector<Stmt> &ss,
                                                  const Lexer::TokenPos &p){
  if(ss.empty()){
    throw new std::logic_error("Lang::Stmt::locked: Empty statement set.");
  }
  /* Check that ss satisfies the invariant for stmts */
  unsigned i;
  for(i = 0; i < ss.size(); i++){
    std::string cmt;
    if(!check_locked_invariant(ss[i],&cmt)){
      throw new std::logic_error("Lang::Stmt::locked: "+cmt);
    }
  }

  Stmt<RegId> s(p);
  s.type = LOCKED;
  s.stmt_count = ss.size();
  s.stmts = new labeled_stmt_t[ss.size()];
  for(i = 0; i < ss.size(); i++){
    s.stmts[i].lbl = "";
    s.stmts[i].stmt = ss[i];
  }
  s.populate_reads_writes();
  s.fence = s.writes.size() > 0;
  return s;
}

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::locked_write(MemLoc<RegId> ml, const Expr<RegId> &e,
                                                  const Lexer::TokenPos &p){
  return locked_block(std::vector<Stmt<RegId> >(1,write(ml,e,p)),p);
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::cas(MemLoc<RegId> ml, const Expr<RegId> &e0,
                                         const Expr<RegId> &e1,
                                         const Lexer::TokenPos &p){
  std::vector<labeled_stmt_t> ss;
  ss.push_back(read_assert(ml,e0,p));
  ss.push_back(write(ml,e1,p));
  return locked_block(std::vector<Stmt<RegId> >(1,sequence(ss,p)),p);
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::slocked_block(const std::vector<Stmt> &ss,
                                                   const Lexer::TokenPos &p){
  if(ss.empty()){
    throw new std::logic_error("Lang::Stmt::slocked_block: Empty statement set.");
  }
  /* Check that ss satisfies the invariant for stmts */
  unsigned i;
  for(i = 0; i < ss.size(); i++){
    std::string cmt;
    if(!check_slocked_invariant(ss[i],&cmt)){
      throw new std::logic_error("Lang::Stmt::slocked_block: "+cmt);
    }
  }

  Stmt<RegId> s(p);
  s.type = SLOCKED;
  s.stmt_count = ss.size();
  s.stmts = new labeled_stmt_t[ss.size()];
  for(i = 0; i < ss.size(); i++){
    s.stmts[i].lbl = "";
    s.stmts[i].stmt = ss[i];
  }
  s.populate_reads_writes();
  return s;
}

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::slocked_write(MemLoc<RegId> ml, const Expr<RegId> &e,
                                                   const Lexer::TokenPos &p){
  return slocked_block(std::vector<Stmt<RegId> >(1,write(ml,e,p)),p);
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::goto_stmt(label_t lbl,
                                               const Lexer::TokenPos &p){
  Stmt<RegId> s(p);
  s.type = GOTO;
  s.lbl = lbl;
  return s;
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::update(int writer, VecSet<MemLoc<RegId> > mls,
                                            const Lexer::TokenPos &p){
  Stmt<RegId> s(p);
  s.type = UPDATE;
  s.writer = writer;
  s.writes = mls;
  return s;
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::serialise(VecSet<MemLoc<RegId>> mls,
                                               const Lexer::TokenPos &p) {
  Stmt<RegId> s(p);
  s.type = SERIALISE;
  s.writes = mls;
  return s;
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::if_stmt(const BExpr<RegId> &b, 
                                             const labeled_stmt_t &s0,
                                             const Lexer::TokenPos &p){
  Stmt<RegId> s(p);
  s.type = IF;
  s.b = new BExpr<RegId>(b);
  s.stmts = new labeled_stmt_t[1];
  s.stmts[0] = s0;
  s.stmt_count = 1;
  s.populate_reads_writes();
  return s;
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::if_stmt(const BExpr<RegId> &b, 
                                             const labeled_stmt_t &s0,
                                             const labeled_stmt_t &s1,
                                             const Lexer::TokenPos &p){
  Stmt<RegId> s(p);
  s.type = IF;
  s.b = new BExpr<RegId>(b);
  s.stmts = new labeled_stmt_t[2];
  s.stmts[0] = s0;
  s.stmts[1] = s1;
  s.stmt_count = 2;
  s.populate_reads_writes();
  return s;
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::while_stmt(const BExpr<RegId> &b,
                                                const labeled_stmt_t &s0,
                                                const Lexer::TokenPos &p){
  Stmt<RegId> s(p);
  s.type = WHILE;
  s.b = new BExpr<RegId>(b);
  s.stmts = new labeled_stmt_t[1];
  s.stmts[0] = s0;
  s.stmt_count = 1;
  s.populate_reads_writes();
  return s;
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::either(const std::vector<Stmt> &ss,
                                            const Lexer::TokenPos &p){
  if(ss.empty()){
    throw new std::logic_error("Lang::Stmt::either: Empty statement set.");
  }
  Stmt<RegId> s(p);
  s.type = EITHER;
  s.stmts = new labeled_stmt_t[ss.size()];
  for(unsigned i = 0; i < ss.size(); i++){
    s.stmts[i].lbl = "";
    s.stmts[i].stmt = ss[i];
  }
  s.stmt_count = ss.size();
  s.populate_reads_writes();
  return s;
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::sequence(const std::vector<labeled_stmt_t> &ss,
                                              const Lexer::TokenPos &p){
  Stmt<RegId> s(p);
  s.type = SEQUENCE;
  s.stmts = new labeled_stmt_t[ss.size()];
  for(unsigned i = 0; i < ss.size(); i++){
    s.stmts[i] = ss[i];
  }
  s.stmt_count = ss.size();
  s.populate_reads_writes();
  return s;
};

template<class RegId> template<class RegId2> Lang::Stmt<RegId2> 
Lang::Stmt<RegId>::convert(std::function<RegId2(const RegId&)> &rc,
                           std::function<MemLoc<RegId2>(const MemLoc<RegId>&)> &mlc) const{
  Stmt<RegId2> s(pos);
  s.type = type;
  s.reg = rc(reg);
  for(int i = 0; i < reads.size(); i++){
    s.reads.insert(mlc(reads[i]));
  }
  for(int i = 0; i < writes.size(); i++){
    s.writes.insert(mlc(writes[i]));
  }
  if(e0) s.e0 = new Expr<RegId2>(e0->convert(rc));
  if(e1) s.e1 = new Expr<RegId2>(e1->convert(rc));
  if(b) s.b = new BExpr<RegId2>(b->convert(rc));
  if(stmts){
    s.stmts = new typename Stmt<RegId2>::labeled_stmt_t[stmt_count];
    for(int i = 0; i < stmt_count; i++){
      s.stmts[i].lbl = stmts[i].lbl;
      s.stmts[i].stmt = stmts[i].stmt.convert(rc,mlc);
    }
  }
  s.fence = fence;
  s.lbl = lbl;
  s.writer = writer;
  s.stmt_count = stmt_count;
  return s;
};

template<class RegId> std::string 
Lang::Stmt<RegId>::to_string(const std::function<std::string(const RegId&)> &regts, 
                             const std::function<std::string(const MemLoc<RegId> &)> &mlts,
                             int indentation,std::string label) const{
  std::string ind;
  for(int i = 0; i < indentation; i++){
    ind += " ";
  }
  std::string indlbl = ind;
  if(label != ""){
    indlbl = ind+label+": ";
  }
  switch(type){
  case NOP: return indlbl+"nop";
  case ASSIGNMENT: return indlbl+regts(reg)+" := "+e0->to_string(regts);
  case ASSUME: return indlbl+"assume: "+b->to_string(regts);
  case READASSERT: return indlbl+"read: "+mlts(reads[0])+" = "+e0->to_string(regts);
  case READASSIGN: return indlbl+"read: "+regts(reg)+" := "+mlts(reads[0]);
  case WRITE: 
    return indlbl+"write: "+mlts(writes[0])+" := "+e0->to_string(regts);
  case GOTO: return indlbl+"goto "+lbl;
  case SERIALISE: return indlbl+"serialise: "+mlts(writes[0]);
  case UPDATE:
    {
      std::stringstream ss;
      ss << indlbl << "update(";
      if(writes.size() == 1){
        ss << mlts(writes[0]);
      }else{
        ss << "[";
        for(int i = 0; i < writes.size(); ++i){
          if(i != 0) ss << ", ";
          ss << mlts(writes[i]);
        }
        ss << "]";
      }
      ss << ", P" << writer << ")";
      return ss.str();
    }
  case IF:
    if(indentation >= 0){
      return indlbl+"if "+b->to_string(regts)+" then\n"+
        stmts[0].stmt.to_string(regts,mlts,indentation+2,stmts[0].lbl)+
        (stmt_count == 2 
         ? "\n"+ind+"else\n"+ stmts[1].stmt.to_string(regts,mlts,indentation+2,stmts[1].lbl)
         : "");
    }else{
      return indlbl+"if "+b->to_string(regts)+" then "+
        stmts[0].stmt.to_string(regts,mlts,indentation,stmts[0].lbl)+
        (stmt_count == 2 
         ? " else "+ stmts[1].stmt.to_string(regts,mlts,indentation,stmts[1].lbl)
         : "");
    }
  case WHILE:
    if(indentation >= 0){
      return indlbl+"while "+b->to_string(regts)+" do\n"+
        stmts[0].stmt.to_string(regts,mlts,indentation+2,stmts[0].lbl);
    }else{
      return indlbl+"while "+b->to_string(regts)+" do "+
        stmts[0].stmt.to_string(regts,mlts,indentation,stmts[0].lbl);
    }
  case EITHER:
    if(indentation >= 0){
      std::string s = indlbl+"either{\n";
      for(int i = 0; i < stmt_count; i++){
        s += stmts[i].stmt.to_string(regts,mlts,indentation+2,stmts[i].lbl)+"\n";
        if(i < stmt_count-1) s += ind+"or\n";
      }
      s += ind+"}";
      return s;
    }else{
      std::string s = indlbl+"either{ ";
      for(int i = 0; i < stmt_count; i++){
        s += stmts[i].stmt.to_string(regts,mlts,indentation,stmts[i].lbl);
        if(i < stmt_count-1) s += " or ";
      }
      s += " }";
      return s;
    }
  case SLOCKED: case LOCKED:
    if(stmt_count == 1 && stmts[0].stmt.get_type() == Lang::WRITE && stmts[0].lbl==""){
      return indlbl+(type==SLOCKED?"slocked ":"locked ")+
        stmts[0].stmt.to_string(regts,mlts,0,"");
    }else if(indentation >= 0){
      std::string s = indlbl+(type==SLOCKED?"slocked{\n":"locked{\n");
      for(int i = 0; i < stmt_count; i++){
        s += stmts[i].stmt.to_string(regts,mlts,indentation+2,stmts[i].lbl)+"\n";
        if(i < stmt_count-1) s += ind+"or\n";
      }
      s += ind+"}";
      return s;
    }else{
      std::string s = indlbl+(type==SLOCKED?"slocked{\n":"locked{\n");
      for(int i = 0; i < stmt_count; i++){
        s += stmts[i].stmt.to_string(regts,mlts,indentation,stmts[i].lbl);
        if(i < stmt_count-1) s += " or ";
      }
      s += " }";
      return s;
    }
  case SEQUENCE:
    if(indentation >= 0){
      std::string s = indlbl+"{\n";
      for(int i = 0; i < stmt_count; i++){
        s += stmts[i].stmt.to_string(regts,mlts,indentation+2,stmts[i].lbl);
        if(i < stmt_count-1) s += ";";
        s += "\n";
      }
      s += ind+"}";
      return s;
    }else{
      std::string s = indlbl+"{ ";
      for(int i = 0; i < stmt_count; i++){
        s += stmts[i].stmt.to_string(regts,mlts,indentation,stmts[i].lbl);
        if(i < stmt_count-1) s += "; ";
      }
      s += " }";
      return s;
    }
  }
  throw new std::logic_error("Lang::Stmt::to_string: Invalid statement type.");
};

template<class RegId> int Lang::Stmt<RegId>::compare(const Stmt<RegId> &stmt) const throw(){
  {
    int c = pos.compare(stmt.pos);
    if(c != 0) return c;
  }
  if(type < stmt.type){
    return -1;
  }else if(type > stmt.type){
    return 1;
  }else{
    switch(type){
    case NOP: return 0;
    case ASSIGNMENT: 
      if(reg < stmt.reg){
        return -1;
      }else if(reg > stmt.reg){
        return 1;
      }else{
        return e0->compare(*stmt.e0);
      }
    case ASSUME: return b->compare(*stmt.b);
    case READASSERT:
      if(reads[0] < stmt.reads[0]){
        return -1;
      }else if(stmt.reads[0] < reads[0]){
        return 1;
      }else{
        return e0->compare(*stmt.e0);
      }
    case READASSIGN:
      if(reg < stmt.reg){
        return -1;
      }else if(reg > stmt.reg){
        return 1;
      }else if (reads[0] < stmt.reads[0]){
        return -1;
      }else if(stmt.reads[0] < reads[0]){
        return 1;
      }else{
        return 0;
      }
    case WRITE:
      if(writes[0] < stmt.writes[0]){
        return -1;
      }else if(stmt.writes[0] < writes[0]){
        return 1;
      }else{
        return e0->compare(*stmt.e0);
      }
    case GOTO:
      if(lbl < stmt.lbl){
        return -1;
      }else if(lbl > stmt.lbl){
        return 1;
      }else{
        return 0;
      }
    case UPDATE:
      if(writes[0] < stmt.writes[0]){
        return -1;
      }else if(stmt.writes[0] < writes[0]){
        return 1;
      }else if(writer < stmt.writer){
        return -1;
      }else if(writer > stmt.writer){
        return 1;
      }else{
        return 0;
      }
    case SERIALISE:
      if(writes[0] < stmt.writes[0]){
        return -1;
      }else if(stmt.writes[0] < writes[0]){
        return 1;
      }else{
        return 0;
      }
    case IF:
      {
        int bc = b->compare(*stmt.b);
        if(bc != 0){
          return bc;
        }else{
          int s0c = stmts[0].compare(stmt.stmts[0]);
          if(s0c != 0){
            return s0c;
          }else{
            if(stmt_count < stmt.stmt_count){
              return -1;
            }else if(stmt_count > stmt.stmt_count){
              return 1;
            }else if(stmt_count == 1){
              return 0;
            }else{
              return stmts[1].compare(stmt.stmts[1]);
            }
          }
        }
      }
    case WHILE:
      {
        int bc = b->compare(*stmt.b);
        if(bc != 0){
          return bc;
        }else{
          return stmts[0].compare(stmt.stmts[0]);
        }
      }
    case SLOCKED:
    case LOCKED:
      {
        if(fence < stmt.fence){
          return -1;
        }else if(fence > stmt.fence){
          return 1;
        }else{
          if(stmt_count < stmt.stmt_count){
            return -1;
          }else if(stmt_count > stmt.stmt_count){
            return 1;
          }else{
            for(int i = 0; i < stmt_count; i++){
              int c = stmts[i].compare(stmt.stmts[i]);
              if(c != 0){
                return c;
              }
            }
            return 0;
          }
        }
      }
    case EITHER: // Same as sequence
    case SEQUENCE:
      {
        if(stmt_count < stmt.stmt_count){
          return -1;
        }else if(stmt_count > stmt.stmt_count){
          return 1;
        }else{
          for(int i = 0; i < stmt_count; i++){
            int c = stmts[i].compare(stmt.stmts[i]);
            if(c != 0){
              return c;
            }
          }
          return 0;
        }
      }
    default:
      throw new std::logic_error("Lang::Stmt::compare: Unknown statement type");
    }
  }
}

template<class RegId> int Lang::Stmt<RegId>::labeled_stmt_t::compare(const labeled_stmt_t &lstmt) const throw(){
  if(lbl < lstmt.lbl){
    return -1;
  }else if(lbl > lstmt.lbl){
    return 1;
  }else{
    return stmt.compare(lstmt.stmt);
  }
}


template<class RegId> bool Lang::Stmt<RegId>::check_locked_invariant(const Stmt &stmt, std::string *comment){
  switch(stmt.get_type()){
  case NOP: case ASSIGNMENT: case ASSUME: case READASSERT: case READASSIGN: case WRITE: case LOCKED: case SLOCKED:
    return true;
  case SEQUENCE:
    {
      for(int i = 0; i < stmt.get_statement_count(); i++){
        if(stmt.get_label(i) != ""){
          if(comment) *comment = "Label (\""+stmt.get_label(i)+"\") in locked block.";
          return false;
        }
        if(!check_locked_invariant(*stmt.get_statement(i),comment)){
          return false;
        }
      }
      return true;
    }
  case GOTO:
    if(comment) *comment = "Illegal type of statement in locked block: goto.";
    return false;
  case UPDATE:
    if(comment) *comment = "Illegal type of statement in locked block: update.";
    return false;
  case IF:
    if(comment) *comment = "Illegal type of statement in locked block: if statement.";
    return false;
  case WHILE:
    if(comment) *comment = "Illegal type of statement in locked block: while statement.";
    return false;
  case EITHER:
    if(comment) *comment = "Illegal type of statement in locked block: either statement.";
    return false;
  default:
    if(comment) *comment = "Illegal type of statement in locked block.";
    return false;
  }
}

template<class RegId> bool Lang::Stmt<RegId>::check_slocked_invariant(const Stmt &stmt, std::string *comment){
  if (stmt.get_type() != WRITE && comment) *comment = "Illegal type of statement in store-store locked block";
  return stmt.get_type() == WRITE;
}

template<class RegId> bool Lang::Stmt<RegId>::find_substmt(std::function<bool(const Stmt&)> &f, Stmt *ss) const throw(){
  if(f(*this)){
    if(ss) *ss = *this;
    return true;
  }else{
    switch(type){
    case NOP: case ASSIGNMENT: case ASSUME: case READASSERT: case READASSIGN: 
    case WRITE: case GOTO: case UPDATE:
      return false;
    case LOCKED: case SLOCKED: case IF: case WHILE: case EITHER: case SEQUENCE:
      {
        for(int i = 0; i < stmt_count; i++){
          if(stmts[i].stmt.find_substmt(f,ss)){
            return true;
          }
        }
        return false;
      }
    default:
      throw new std::logic_error("Lang::Stmt::findsubstmt: Unknown statement type.");
    }
  }
}

template<class RegId> Lang::MemLoc<RegId> Lang::Stmt<RegId>::get_memloc() const{
  switch(type){
  case READASSERT: case READASSIGN:
    return reads[0];
  case WRITE: case UPDATE: case SLOCKED:
    return writes[0];
  default:
    throw new std::logic_error("Lang::Stmt::get_memloc: Statement has no unique memory location.");
  }
}

template<class RegId> void Lang::Stmt<RegId>::populate_reads_writes(){
  if(stmt_count == 0){
    return;
  }else{
    writes = stmts[0].stmt.writes;
    reads = stmts[0].stmt.reads;
    for(int i = 1; i < stmt_count; i++){
      writes.insert(stmts[i].stmt.writes);
      reads.insert(stmts[i].stmt.reads);
    }
  }
}

template<class RegId> std::set<RegId> Lang::Stmt<RegId>::get_registers() const{
  std::set<RegId> set;
  switch(type){
  case NOP: return set;
  case ASSIGNMENT:
    set = e0->get_registers();
    set.insert(reg);
    return set;
  case ASSUME:
    return b->get_registers();
  case READASSERT:
    return e0->get_registers();
  case READASSIGN:
    set.insert(reg);
    return set;
  case WRITE:
    return e0->get_registers();
  case GOTO: return set;
  case UPDATE: return set;
  case IF: case WHILE:
    set = b->get_registers();
    // Note: no break
  case LOCKED: case SLOCKED: case EITHER: case SEQUENCE:
    {
      for(int i = 0; i < stmt_count; i++){
        std::set<RegId> s2 = stmts[i].stmt.get_registers();
        set.insert(s2.begin(),s2.end());
      }
      return set;
    }
  default:
    throw new std::logic_error("Lang::Stmt::get_registers: Unsupported type of statement.");
  }
}

template<class RegId> 
VecSet<VecSet<Lang::MemLoc<RegId> > > Lang::Stmt<RegId>::get_write_sets() const throw(){
  VecSet<VecSet<MemLoc<RegId> > > no_writes = VecSet<VecSet<MemLoc<RegId> > >::singleton(VecSet<MemLoc<RegId> >());
  switch(type){
  case NOP: case ASSIGNMENT: case ASSUME: case READASSERT: case READASSIGN: case GOTO: 
    return no_writes;
  case WRITE: case UPDATE:
    return VecSet<VecSet<MemLoc<RegId> > >::singleton(writes);
  case LOCKED: case SLOCKED: case EITHER: case IF:
    {
      VecSet<VecSet<MemLoc<RegId> > > s;
      for(int i = 0; i < stmt_count; ++i){
        s.insert(stmts[i].stmt.get_write_sets());
      }
      return s;
    }
  case WHILE:
    {
      no_writes.insert(stmts[0].stmt.get_write_sets());
      return no_writes;
    }
  case SEQUENCE:
    {
      VecSet<VecSet<MemLoc<RegId> > > s = no_writes;
      for(int i = 0; i < stmt_count; ++i){
        VecSet<VecSet<MemLoc<RegId> > > si = stmts[i].stmt.get_write_sets();
        VecSet<VecSet<MemLoc<RegId> > > tmp;
        /* For each pair (a,b) in the cross product s x si, add a U b to tmp. */
        for(int a = 0; a < s.size(); ++a){
          for(int b = 0; b < si.size(); ++b){
            VecSet<MemLoc<RegId> > A = s[a];
            A.insert(si[b]);
            tmp.insert(A);
          }
        }

        s = tmp;
      }
      return s;
    }
  default:
    throw new std::logic_error("Lang::Stmt::get_write_sets: Unsupported type of statement.");
  }
};

template<class RegId>
Lang::Stmt<RegId> Lang::Stmt<RegId>::flatten() const{
  switch(get_type()){
  case LOCKED:
    {
      std::vector<std::vector<Stmt> > ss = flatten_aux();
      VecSet<Stmt> vs;
      for(unsigned i = 0; i < ss.size(); ++i){
        if(ss[i].size() == 0){
          vs.insert(nop(get_pos()));
        }else if(ss[i].size() == 1){
          vs.insert(ss[i][0]);
        }else{
          std::vector<labeled_stmt_t> lss;
          for(unsigned j = 0; j < ss[i].size(); ++j){
            lss.push_back(ss[i][j]);
          }
          vs.insert(sequence(lss,get_pos()));
        }
      }
      return locked_block(vs.get_vector(),get_pos());
    }
  case NOP: case ASSIGNMENT: case ASSUME: case READASSERT: case READASSIGN: case WRITE: case UPDATE:
    return *this;
  case IF: case WHILE: case EITHER: case SEQUENCE: case GOTO:
  default:
    throw new std::logic_error("Lang::Stmt::flatten: Not an instruction.");
  }
};

template<class RegId>
std::vector<std::vector<Lang::Stmt<RegId> > > Lang::Stmt<RegId>::flatten_aux() const{
  VecSet<std::vector<Stmt> > res;
  
  switch(get_type()){
  case LOCKED:
    {
      for(int i = 0; i < get_statement_count(); ++i){
        std::vector<std::vector<Stmt> > resi = get_statement(i)->flatten_aux();
        for(unsigned j = 0; j < resi.size(); ++j){
          res.insert(resi[j]);
        }
      }
      break;
    }
  case SEQUENCE:
    {
      res.insert(std::vector<Stmt>());
      for(int i = 0; i < get_statement_count(); ++i){
        std::vector<std::vector<Stmt> > resi = get_statement(i)->flatten_aux();
        VecSet<std::vector<Stmt> > res2;
        for(unsigned j = 0; j < resi.size(); ++j){
          for(auto it = res.begin(); it != res.end(); ++it){
            std::vector<Stmt> ss = *it;
            ss.insert(ss.end(),resi[j].begin(),resi[j].end());
            res2.insert(ss);
          }
        }
        res = res2;
      }
      break;
    }
  case NOP: case ASSIGNMENT: case ASSUME: case READASSERT: case READASSIGN: case WRITE: case UPDATE:
    {
      res.insert(std::vector<Stmt>(1,*this));
      break;
    }
  case IF: case WHILE: case EITHER:case GOTO:
  default:
    throw new std::logic_error("Lang::Stmt::flatten: Not an instruction.");
  }

  return res.get_vector();
};
