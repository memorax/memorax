/*
 * Copyright (C) 2018 Tuan Phong Ngo
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

#ifndef __PDUAL_CHANNEL_CONSTRAINT_H__
#define __PDUAL_CHANNEL_CONSTRAINT_H__

#include "constraint.h"
#include "machine.h"
#include "vecset.h"
#include "dual_zstar.h"

/* Common base class for constraint classes using as PDUAL-style channel. */
class PDualChannelConstraint : public Constraint{
public:
    class Common;
private:
    
    typedef DualZStar<int> value_t;
    typedef DualZStar<int>::Vector Store;
    
protected:
    /* The class of PDUAL channel messages. */
    class Msg{
    public:
        Msg(Store s, int pid, VecSet<Lang::NML> ms)
        : store(s), wpid(pid), nmls(ms) {};
        
        Store store;
        int wpid;      // The pid of the process that wrote
        /* A distinct, sorted vector of all the written memory locations. */
        VecSet<Lang::NML> nmls;
        std::string to_short_string(const Common &common) const;
        /* A total order on messages */
        int compare(const Msg &) const;
        bool operator<(const Msg &msg) const { return compare(msg) < 0; };
        bool operator==(const Msg &msg) const { return compare(msg) == 0; };
        bool operator>(const Msg &msg) const { return compare(msg) > 0; };
        bool operator<=(const Msg &msg) const { return compare(msg) <= 0; };
        bool operator!=(const Msg &msg) const { return compare(msg) != 0; };
        bool operator>=(const Msg &msg) const { return compare(msg) >= 0; };
        Constraint::Comparison entailment_compare(const Msg &msg) const{
            if(wpid != msg.wpid || nmls != msg.nmls){
                return Constraint::INCOMPARABLE;
            }else{
                return store.entailment_compare(msg.store);
            }
        };
    };
    
public:
    class Common : public Constraint::Common{
    public:
        Common(const Machine &m);
        const Machine &machine;
        
        /* Constructs and returns a list of bad states based on the
         * machine and possible initial messages in the channel.
         */
        virtual std::list<Constraint*> get_bad_states() = 0;
    protected:
        /**************************/
        /* Computed from machine: */
        /**************************/
        // The number of entries in a memory store
        // (not necessarily equal to the number of memory locations)
        int mem_size;
        // The number of global variables
        int gvar_count;
        // The maximum number of local variables of any process
        int max_lvar_count;
        // All memory locations that occur in machine
        VecSet<Lang::NML> nmls;
        // Gives the index in a memory location store of the memory location given by nml
        int index(const Lang::NML &nml) const{
            if(nml.is_global()){
                return nml.get_id();
            }else{
                return gvar_count + nml.get_owner()*max_lvar_count + nml.get_id();
            }
        };
        // reg_count[pid] is the number of registers of process pid
        std::vector<int> reg_count;
        
        /* A MsgHdr mh identifies the set of messages where the writing
         * process is mh.wpid and the written variables are mh.nmls.
         */
        struct MsgHdr{
            MsgHdr(int wpid, const VecSet<Lang::NML> nmls) : wpid(wpid), nmls(nmls) {};
            int wpid;
            VecSet<Lang::NML> nmls;
            bool operator==(const MsgHdr &mh) const {
                return wpid == mh.wpid && nmls == mh.nmls;
            };
            bool operator<(const MsgHdr &mh) const{
                return wpid < mh.wpid ||
                (wpid == mh.wpid && nmls < mh.nmls);
            };
        };
        /* The set of all message headers that can possibly occur in the
         * channel of a constraint from this->machine.
         *
         * Will contain a dummy message in case no writes occur in this->machine. */
        VecSet<MsgHdr> messages;
      
        VecSet<MsgHdr> removed_lock_blocks_messages;
      
        /* If t performs writes deterministically and such that all
         * written values are given as integer literals, then returns a
         * memory store with those values set to the corresponding memory
         * locations. Otherwise returns a memory store with all stars.
         */
        virtual Store store_of_write(const Machine::PTransition &t) const;
        
        friend class PDualChannelConstraint;
        friend class PDualChannelBwd;
    };
    /* Constructs a constraint where process pid is at control state
     * pcs[pid], all registers and memory locations are unrestricted,
     * and the channel consists of exactly one message with an
     * unrestricted memory snapshot and writer and written memory
     * locations as specified by msg. */
    PDualChannelConstraint(std::vector<int> pcs, const Common::MsgHdr &msg, Common &c);
    PDualChannelConstraint(std::vector<int> pcs, Common &c);
    /* Returns a copy of the constraint */
    virtual PDualChannelConstraint *clone() const = 0;
    virtual const std::vector<int> &get_control_states() const noexcept { return pcs; };
    virtual void abstract(){};
    virtual bool is_abstracted() const { return true; };
    virtual bool is_init_state() const;
    virtual std::string to_string() const noexcept;
    virtual Comparison entailment_compare(const Constraint &c) const;
    /* The weight is a hint to ChannelContainer as to in which order constraints
     * should be explored; lower weight constraints are explored before higher
     * weight constraints.
     *
     * The weight must not be negative, and should be kept to relatively small
     * numbers. */
    virtual int get_weight() const {
      int sum = 0;
      
      for(int ci=0; ci<channels.size(); ci++) {
        sum += channels[ci].size();
      }
      return sum;
    };
    
    /* An MsgCharacterization keeps some information about a particular
     * message in an SB channel: The writing process, the written memory
     * locations and the set of processes whose cpointers point to this
     * message.
     */
    class MsgCharacterization{
    public:
        MsgCharacterization(int wpid, VecSet<Lang::NML> nmls)
        : wpid(wpid), nmls(nmls) {};
        /* The writing process */
        int wpid;
        /* The written memory locations */
        VecSet<Lang::NML> nmls;

        bool operator==(const MsgCharacterization &mc) const{
            return wpid == mc.wpid && nmls == mc.nmls;
        };
        bool operator<(const MsgCharacterization &mc) const{
            if(wpid < mc.wpid){
                return true;
            }
            if(wpid > mc.wpid){
                return false;
            }
            return nmls < mc.nmls;
        };
    };
    
    /* Returns a vector characterizing the channel of this constraint.
     *
     * Let S be the set of messages in this->channel such that either
     * there is some cpointer pointing to the message or the message is
     * the rightmost write of some process to some variable. The
     * returned vector v is such that v[i] is the characterization of
     * the i:th leftmost message in S.
     *
     * Interestingly:
     *   sbc0.characterize_channel() != sbc1.characterize_channel()
     *   implies that
     *   sbc0.entailment_compare(sbc1) == INCOMPARABLE
     */
    std::vector<MsgCharacterization> characterize_channel(int ci) const;
  
protected:
    /* pcs[pid] is the program counter of process pid. */
    std::vector<int> pcs;
  
    /* ptypes[p] indicates type of process p
     * input contains a set of class of processes
     * type = 0: a process of class 0 ...
     */
    std::vector<int> ptypes;
  
    /* The PDUAL channels
     *
     * Messages at lower indices are older. */
    std::vector<std::vector<Msg>> channels;
    
    std::vector<Store> mems;
    
    /* reg_stores[pid] is the register valuation of process pid. */
    std::vector<Store> reg_stores;
    
    /* Returns the set of values that can be held by the memory location
     * nml according to the valuation mem. This will be either the
     * unique valuation given by mem, or if mem maps nml to STAR, the
     * whole domain of nml.
     */
    VecSet<int> inline possible_values(const Store &mem, const Lang::NML &nml) const{
        return mem.possible_values(common.index(nml),common.machine.get_var_decl(nml));
    };
  

    /* Returns the set of values that e can evaluate to when its
     * accessed registers are valuated as indicated by reg_store.
     *
     * pid should be the process evaluating e, i.e., the owner of reg_store.
     */
    VecSet<int> inline possible_values(const Store &reg_store, int pid, const Lang::Expr<int> &e) const{
        return reg_store.possible_values(e,common.machine.regs[pid]);
    };
    /* Returns true iff there is some assignment to the registers
     * accessed by b that is admitted by reg_store, and which satisfies
     * b.
     *
     * pid should be the process evaluating b, i.e., the owner of reg_store.
     */
    bool inline possibly_holds(const Store &reg_store, int pid, const Lang::BExpr<int> &b) const{
        throw new std::logic_error("PDualChannelConstraint::possibly_holds: Not implemented");
    };
    /* Returns the set of stores which are entailed by reg_store and
     * where the expression e of process pid evaluates to the unique
     * value value.
     *
     * I.e. tries to instantiate any STAR in reg_store such that e will
     * evaluate to value.
     */
    VecSet<Store> inline possible_reg_stores(const Store &reg_store, int pid, const Lang::Expr<int> &e, int value) const{
        return reg_store.possible_regs(e,value,common.machine.regs[pid]);
    };
  
    /* Returns the set of stores which are entailed by reg_store and
     * where the expression b of process pid evaluates to true.
     */
    VecSet<Store> inline possible_reg_stores(const Store &reg_store, int pid, const Lang::BExpr<int> &b) const{
        return reg_store.possible_regs(b,common.machine.regs[pid]);
    };
  
    VecSet<Store> inline possible_reg_stores(const Store &reg_store, int pid) const{
      return reg_store.possible_regs(common.machine.regs[pid]);
    };
  
    VecSet<Store> inline possible_store_stores(const Store &mem,
                                               const VecSet<Lang::NML> &nmls) const{
      assert(mem.size() == nmls.size());
      std::vector<Lang::VarDecl> decls(nmls.size(),common.machine.get_var_decl(nmls[0]));
      decls.reserve(nmls.size());
      for(int nmli=0; nmli<nmls.size(); nmli++) {
        int i = common.index(nmls[nmli]);
        decls[i]=common.machine.get_var_decl(nmls[nmli]);
      }
      return mem.possible_stores(decls);
      
    };
  
    /* Returns the index into channel of the message from which process
     * pid would read the value of memory location nml if it were to
     * read in this constraint.
     *
     * I.e. the index i = max{j | j = cpointers[pid] ||
     *                            (channel[j].wpid == pid && nml in channel[j].nmls)}
     */
    int index_of_read(Lang::NML nml, int pid) const;
    
protected:
    /* Appends a string describing the local state of process p to ss */
    virtual void process_to_string(int p, std::stringstream &ss) const noexcept;
    
private:
    Comparison entailment_compare_impl(const PDualChannelConstraint &sbc) const;
    Common &common;
    /* Entailment compare this->channel with sbc.channel. Return the
     * combination (Constraint::comb_comp) of that comparison result and
     * cmp.
     */
    virtual Constraint::Comparison entailment_compare_channels(const PDualChannelConstraint &sbc, Constraint::Comparison cmp, std::vector<int> cand, int same=0) const;
  
  
    std::vector<std::vector<int>> possible_mappings(int num, int size,
                                                    std::vector<int> occupied) const;
  
    friend class PDualChannelBwd;
    friend class ChannelContainer;
};

/*******************/
/* Implementations */
/*******************/

#endif
