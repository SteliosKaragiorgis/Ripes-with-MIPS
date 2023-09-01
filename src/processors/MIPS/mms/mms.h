#pragma once

#include "VSRTL/core/vsrtl_adder.h"
#include "VSRTL/core/vsrtl_constant.h"
#include "VSRTL/core/vsrtl_design.h"
#include "VSRTL/core/vsrtl_logicgate.h"
#include "VSRTL/core/vsrtl_multiplexer.h"

#include "../../ripesvsrtlprocessor.h"

#include "../../MIPS/mips.h"
#include "../../MIPS/mips_alu.h"
#include "../../MIPS/mips_control_mms.h"
#include "../../MIPS/mips_alu_control.h"
#include "../../MIPS/mips_ecallchecker.h"
#include "../../MIPS/mips_immediate.h"
#include "../../MIPS/mips_memory_mms.h"
#include "../../MIPS/mips_shift.h"
#include "../../MIPS/mips_shift_extend.h"
#include "../../MIPS/mips_registerfile_mms.h"
#include "../../MIPS/mips_jump_address.h"
#include "../../MIPS/mms/mips_decodeMMS.h"
#include "../../MIPS/mips_decode_mms.h"
#include "../../MIPS/mips_multi_fsm_state.h"
#include "../../MIPS/mips_multi_fsm_start_state.h"




namespace vsrtl {
namespace core {
using namespace Ripes;

template <typename XLEN_T>
class MMS : public RipesVSRTLProcessor {
  static_assert(std::is_same<uint32_t, XLEN_T>::value,
                "Only supports 32-bit variants");
  static constexpr unsigned XLEN = sizeof(XLEN_T) * CHAR_BIT;

public:
  enum State {
      InstructionFetch = 0,
      InstructionDecode = 1,
      MemoryAddressComputation = 2,
      MemoryAccessLW = 3,
      WriteBack = 4,
      MemoryAccessSW = 5,
      Execution = 6,
      RTypeCompletion = 7,
      BranchCompletion = 8,
      JumpCompletion = 9,
      STATECOUNT  };
  MMS(const QStringList &extensions)
      : RipesVSRTLProcessor("Multi Cycle MIPS Processor") {
    m_enabledISA = std::make_shared<ISAInfo<XLenToMIPSISA<XLEN>()>>(extensions);
    decode->setISA(m_enabledISA);
    decode->initialState();

    // -----------------------------------------------------------------------
    // Program counter
    0 >> pc_reg->clear;
    pc_reg->out >> pc_4->op2;
    4 >> pc_4->op1;


    // Note: pc_src works uses the PcSrc enum, but is selected by the boolean
    // signal from the controlflow OR gate. PcSrc enum values must adhere to the
    // boolean 0/1 values.

    // -----------------------------------------------------------------------
    // Instruction memory


    // -----------------------------------------------------------------------
    // Decode
    control->irwrite >> decode->irwrite;
    pc_reg->out >> decode->pc;
    control->instr_sel >> decode->instr_sel;

    // -----------------------------------------------------------------------
    // Control signals
    decode->opcode >> control->opcode;
    decode->state >> control->state;


    // -----------------------------------------------------------------------
    // Immediate

    decode->instr_imm_15_0 >> immediate->instr15_0;

    // -----------------------------------------------------------------------
    // Registers
    decode->mt_sel >> registerFile->mt_sel;
    decode->mf_sel >> registerFile->mf_sel;
    decode->mult_div_sel >> registerFile->mult_div_sel;
    decode->j_sel >> registerFile->j_sel;
    decode->instr_rs_25_21 >> registerFile->r1_addr;
    decode->instr_rt_20_16 >> registerFile->r2_addr;
    pc_reg->out >> registerFile->pc_4;

    decode->instr_rt_20_16 >> write_reg->get(MIPS_WrReg::RR);
    decode->instr_rd_15_11 >> write_reg->get(MIPS_WrReg::WR);
    decode->instr_shamt_10_6 >> shamt->in;
    shamt->out >> alu->shamt;
    decode->opcode >> registerFile->opcode;
    control->do_reg_dst >> write_reg->select;
    write_reg->out >> registerFile->wr_addr;
    alu->hi >> hi_out->in;

    control->reg_do_write_ctrl >> registerFile->wr_en;
    reg_wr_src->out >> registerFile->data_in;

    alu->res >> alu_out->in;
    hi_out->out >> registerFile->hi_in;

    decode->state >> registerFile->state;


    mem_data_reg->out >> reg_wr_src->get(MIPS_RegWrSrc::MEMREAD);
    alu_out->out >> reg_wr_src->get(MIPS_RegWrSrc::ALURES);
    control->reg_wr_src_ctrl >> reg_wr_src->select;

    data_mem->data_out >> decode->instr_s0;


    registerFile->setMemory(m_regMem);




    // -----------------------------------------------------------------------
    // Branch

    alu->zero >> *br_and->in[1];
    control->pc_write_cond >> *br_and->in[0];

    br_and->out >> *pc_write_or->in[0];
    control->pc_write >> *pc_write_or->in[1];
    bn_and->out >> *pc_write_or->in[2];
    bl_and->out >> *pc_write_or->in[3];
    bg_and->out >> *pc_write_or->in[4];
    pc_write_or->out >> pc_reg->enable;



    alu->greater >> *bg_and->in[1];
    control->do_bgt_write >> *bg_and->in[0];

    alu->less >> *bl_and->in[1];
    control->do_blt_write >> *bl_and->in[0];

    alu->zero >> *bn_not->in[0];

    control->do_bne_write >> *bn_and->in[1];
    bn_not->out >> *bn_and->in[0];

    bn_and->out >> *br_or->in[0];
    br_and->out >> *br_or->in[1];
    bg_and->out >> *br_or->in[2];
    bl_and->out >> *br_or->in[3];

    control->do_jump >> pc_src->select;

    // -----------------------------------------------------------------------
    // ALU

    registerFile->r1_out >> a->in;

    registerFile->r2_out >> b->in;

    control->alu_op1_ctrl >> alu_op1_src_multi->select;
    pc_reg->out >> alu_op1_src_multi->get(MIPSMulti_AluSrc1::PC);
    a->out >> alu_op1_src_multi->get(MIPSMulti_AluSrc1::REG1);
    alu_op1_src_multi->out >> alu->op1;

    control->alu_op2_ctrl >> alu_op2_src_multi->select;
    b->out >> alu_op2_src_multi->get(MIPSMulti_AluSrc2::REG2);
    4 >> alu_op2_src_multi->get(MIPSMulti_AluSrc2::FOUR);
    immediate->imm >> alu_op2_src_multi->get(MIPSMulti_AluSrc2::IMM);
    shift_branch->out >> alu_op2_src_multi->get(MIPSMulti_AluSrc2::IMMSHIFT);
    alu_op2_src_multi->out >> alu->op2;


    control->alu_ctrl >> alu_control->ctrl;
    decode->instr_imm_5_0 >> alu_control->instr5_0;
    alu_control->res >> alu->ctrl;

    pc_4->out >> alu_branch->op1;
    shift_branch->out >> alu_branch->op2;

    alu_branch->out >> pc_branch->get(MIPS_PcBranch::IN2);
    pc_4->out >> pc_branch->get(MIPS_PcBranch::IN1);
    br_or->out >> pc_branch->select;

    control->pc_source >> pc_src_multi->select;
    alu->res >> pc_src_multi->get(MIPSMulti_PcSrc::ALU);
    alu_out->out >> pc_src_multi->get(MIPSMulti_PcSrc::ALUREG);
    j_address->res >> pc_src_multi->get(MIPSMulti_PcSrc::JADD);
    pc_src_multi->out >> pc_reg->in;


    j_address->res >> pc_src->get(MIPS_PcSrc::ALU);
    pc_branch->out >> pc_src->get(MIPS_PcSrc::PC4);

    // -----------------------------------------------------------------------
    //Jump


    pc_reg->out >> j_address->pc;
    shift_pc->out >> j_address->sl;
    decode->opcode >> j_address->opcode;
    registerFile->r1_out >> j_address->ra;


    // -----------------------------------------------------------------------
    // Shift
    decode->instr_address_25_0 >> shift_pc->in;
    immediate->imm >> shift_branch->in;

    // -----------------------------------------------------------------------
    // Data memory

    control->iord >> address_multi->select;
    pc_reg->out >> address_multi->get(MIPSMulti_Address::PC);
    alu_out->out >> address_multi->get(MIPSMulti_Address::ALUREG);
    address_multi->out >> data_mem->addr;
    pc_reg->out >> data_mem->addr_instr_mem;

    control->mem_do_write_ctrl >> data_mem->wr_en;
    b->out >> data_mem->data_in;
    control->mem_ctrl >> data_mem->op;
    control->mem_do_read_ctrl >> data_mem->mem_read;
    data_mem->mem->setMemory(m_memory);

    data_mem->instr_mem->setMemory(m_memory);

    data_mem->data_out >> mem_data_reg->in;
    data_mem->instr_mem->data_out >> decode->instr;
    data_mem->data_out >> instr_reg->in;
    instr_reg->out >> decode->instr_show;
    0 >> instr_reg->clear;
    decode->instr_reg_enable >> instr_reg->enable;

    // -----------------------------------------------------------------------
    // Ecall checker
    decode->opcode >> ecallChecker->opcode;
    ecallChecker->setSyscallCallback(&trapHandler);
    0 >> ecallChecker->stallEcallHandling;

    // -----------------------------------------------------------------------
    //FSM
    0 >> fsm_s0->start;
    fsm_s0->next_state >> fsm_s1->prev_state;
    fsm_s1->next_state >> fsm_s2->prev_state;
    fsm_s1->next_state >> fsm_s6->prev_state;
    fsm_s1->next_state >> fsm_s8->prev_state;
    fsm_s1->next_state >> fsm_s9->prev_state;
    fsm_s1->next_state >> fsm_s10->prev_state;
    fsm_s1->next_state >> fsm_s12->prev_state;
    fsm_s2->next_state >> fsm_s3->prev_state;
    fsm_s2->next_state >> fsm_s5->prev_state;
    fsm_s3->next_state >> fsm_s4->prev_state;
    fsm_s6->next_state >> fsm_s7->prev_state;
    fsm_s10->next_state >> fsm_s11->prev_state;


    fsm_s4->next_state >> fsm_s0->s4;
    fsm_s5->next_state >> fsm_s0->s5;
    fsm_s7->next_state >> fsm_s0->s7;
    fsm_s8->next_state >> fsm_s0->s8;
    fsm_s9->next_state >> fsm_s0->s9;
    fsm_s11->next_state >> fsm_s0->s11;
    fsm_s12->next_state >> fsm_s0->s12;

    0 >> fsm_show->start;
    fsm_s4->next_state >> fsm_show->s4;
    fsm_s5->next_state >> fsm_show->s5;
    fsm_s7->next_state >> fsm_show->s7;
    fsm_s8->next_state >> fsm_show->s8;
    fsm_s9->next_state >> fsm_show->s9;
    fsm_s11->next_state >> fsm_show->s11;
    fsm_s12->next_state >> fsm_show->s12;





  }

  //FSM
  SUBCOMPONENT(fsm_show, TYPE(MIPS_Multi_FSM_Start<XLEN>));
  SUBCOMPONENT(fsm_s0, TYPE(MIPS_Multi_FSM_Start<XLEN>));
  SUBCOMPONENT(fsm_s1, TYPE(MIPS_Multi_FSM<XLEN>));
  SUBCOMPONENT(fsm_s2, TYPE(MIPS_Multi_FSM<XLEN>));
  SUBCOMPONENT(fsm_s3, TYPE(MIPS_Multi_FSM<XLEN>));
  SUBCOMPONENT(fsm_s4, TYPE(MIPS_Multi_FSM<XLEN>));
  SUBCOMPONENT(fsm_s5, TYPE(MIPS_Multi_FSM<XLEN>));
  SUBCOMPONENT(fsm_s6, TYPE(MIPS_Multi_FSM<XLEN>));
  SUBCOMPONENT(fsm_s7, TYPE(MIPS_Multi_FSM<XLEN>));
  SUBCOMPONENT(fsm_s8, TYPE(MIPS_Multi_FSM<XLEN>));
  SUBCOMPONENT(fsm_s9, TYPE(MIPS_Multi_FSM<XLEN>));
  SUBCOMPONENT(fsm_s10, TYPE(MIPS_Multi_FSM<XLEN>));
  SUBCOMPONENT(fsm_s11, TYPE(MIPS_Multi_FSM<XLEN>));
  SUBCOMPONENT(fsm_s12, TYPE(MIPS_Multi_FSM<XLEN>));


  // Design subcomponents
  SUBCOMPONENT(registerFile, TYPE(MIPS_RegisterFile_MMS<XLEN, false>));
  SUBCOMPONENT(alu, TYPE(MIPS_ALU<XLEN>));
  SUBCOMPONENT(alu_branch, Adder<XLEN>);
  SUBCOMPONENT(control, MIPS_Control_MMS);
  SUBCOMPONENT(alu_control, MIPS_ALU_Control);
  SUBCOMPONENT(immediate, TYPE(MIPS_Immediate<XLEN>));
  SUBCOMPONENT(decode, TYPE(MIPS_DecodeMMS<XLEN>));
  SUBCOMPONENT(pc_4, Adder<XLEN>);

  // Registers
  SUBCOMPONENT(pc_reg, RegisterClEn<XLEN>);
  SUBCOMPONENT(a, Register<XLEN>);
  SUBCOMPONENT(b, Register<XLEN>);
  SUBCOMPONENT(shamt, Register<c_MIPSRegsBits>);
  SUBCOMPONENT(alu_out, Register<XLEN>);
  SUBCOMPONENT(hi_out, Register<XLEN>);
  SUBCOMPONENT(mem_data_reg, Register<XLEN>);
  SUBCOMPONENT(instr_reg, RegisterClEn<XLEN>);

  // Multiplexers
  SUBCOMPONENT(reg_wr_src, TYPE(EnumMultiplexer<MIPS_RegWrSrc, XLEN>));
  SUBCOMPONENT(pc_src, TYPE(EnumMultiplexer<MIPS_PcSrc, XLEN>));
  SUBCOMPONENT(write_reg, TYPE(EnumMultiplexer<MIPS_WrReg, c_MIPSRegsBits>));
  SUBCOMPONENT(pc_branch, TYPE(EnumMultiplexer<MIPS_PcBranch, XLEN>));
  SUBCOMPONENT(alu_op1_src_multi, TYPE(EnumMultiplexer<MIPSMulti_AluSrc1, XLEN>));
  SUBCOMPONENT(alu_op2_src_multi, TYPE(EnumMultiplexer<MIPSMulti_AluSrc2, XLEN>));
  SUBCOMPONENT(pc_src_multi, TYPE(EnumMultiplexer<MIPSMulti_PcSrc, XLEN>));
  SUBCOMPONENT(address_multi, TYPE(EnumMultiplexer<MIPSMulti_Address, XLEN>));



  // Memories
  SUBCOMPONENT(data_mem, TYPE(MIPS_Memory_MMS<XLEN>));

  // Gates
  SUBCOMPONENT(br_and, TYPE(And<1, 2>));
  SUBCOMPONENT(bn_and, TYPE(And<1, 2>));
  SUBCOMPONENT(bg_and, TYPE(And<1, 2>));
  SUBCOMPONENT(bl_and, TYPE(And<1, 2>));
  SUBCOMPONENT(br_or, TYPE(Or<1, 4>));
  SUBCOMPONENT(bn_not, TYPE(Not<1, 1>));
  SUBCOMPONENT(pc_write_or, TYPE(Or<1, 5>));

  // Shift
  SUBCOMPONENT(shift_pc, MIPS_Shift_Extend<26>, ShiftTypeExtend::sl, 2);
  SUBCOMPONENT(shift_branch, MIPS_Shift<32>, ShiftType::sl, 2);

  // Address spaces
  ADDRESSSPACEMM(m_memory);
  ADDRESSSPACE(m_regMem);

  SUBCOMPONENT(ecallChecker, MIPS_EcallChecker);
  SUBCOMPONENT(j_address, TYPE(MIPS_Jump_Address<XLEN>));

  // Ripes interface compliance
  const ProcessorStructure &structure() const override { return m_structure; }
  unsigned int getPcForStage(StageIndex) const override {
    return pc_reg->out.uValue();
  }
  AInt nextFetchedAddress() const override { return pc_src->out.uValue(); }
  QString stageName(StageIndex idx) const override {
  const VSRTL_VT_U &opcode = decode->opcode.uValue();

      pc_reg->out.setActivePath(false);
      address_multi->out.setActivePath(false);
      data_mem->data_out.setActivePath(false);
      mem_data_reg->out.setActivePath(false);
      decode->decode->instr_imm_15_0.setActivePath(false);
      decode->decode->instr_imm_5_0.setActivePath(false);
      decode->decode->instr_rd_15_11.setActivePath(false);
      decode->decode->instr_rt_20_16.setActivePath(false);
      decode->decode->instr_rs_25_21.setActivePath(false);
      decode->decode->opcode.setActivePath(false);
      decode->decode->instr_address_25_0.setActivePath(false);
      shift_pc->out.setActivePath(false);
      j_address->res.setActivePath(false);
      write_reg->out.setActivePath(false);
      reg_wr_src->out.setActivePath(false);
      registerFile->_rd1_mem->data_out.setActivePath(false);
      registerFile->_rd2_mem->data_out.setActivePath(false);
      a->out.setActivePath(false);
      b->out.setActivePath(false);
      alu_control->res.setActivePath(false);
      immediate->imm.setActivePath(false);
      shift_branch->out.setActivePath(false);
      alu_op1_src_multi->out.setActivePath(false);
      alu_op2_src_multi->out.setActivePath(false);
      alu->res.setActivePath(false);
      alu->less.setActivePath(false);
      alu->greater.setActivePath(false);
      alu_out->out.setActivePath(false);
      pc_src_multi->out.setActivePath(false);

      //Control
      control->mem_do_read_ctrl.setActivePath(false);
      control->alu_op1_ctrl.setActivePath(false);
      control->iord.setActivePath(false);
      control->irwrite.setActivePath(false);
      control->alu_op2_ctrl.setActivePath(false);
      control->alu_ctrl.setActivePath(false);
      control->pc_write.setActivePath(false);
      control->pc_source.setActivePath(false);
      control->reg_wr_src_ctrl.setActivePath(false);
      control->reg_do_write_ctrl.setActivePath(false);
      control->do_reg_dst.setActivePath(false);
      control->do_bgt_write.setActivePath(false);
      control->do_blt_write.setActivePath(false);
      control->do_bne_write.setActivePath(false);
      control->pc_write_cond.setActivePath(false);



      //FSM

      fsm_s0->setCompActiveFsm(false);
      fsm_s1->setCompActiveFsm(false);
      fsm_s2->setCompActiveFsm(false);
      fsm_s3->setCompActiveFsm(false);
      fsm_s4->setCompActiveFsm(false);
      fsm_s5->setCompActiveFsm(false);
      fsm_s6->setCompActiveFsm(false);
      fsm_s7->setCompActiveFsm(false);
      fsm_s8->setCompActiveFsm(false);
      fsm_s9->setCompActiveFsm(false);
      fsm_s10->setCompActiveFsm(false);
      fsm_s11->setCompActiveFsm(false);
      fsm_s12->setCompActiveFsm(false);
      fsm_show->setCompActiveFsm(false);

      fsm_s0->setCompActiveFsmCol(false);
      fsm_s1->setCompActiveFsmCol(false);
      fsm_s2->setCompActiveFsmCol(false);
      fsm_s3->setCompActiveFsmCol(false);
      fsm_s4->setCompActiveFsmCol(false);
      fsm_s5->setCompActiveFsmCol(false);
      fsm_s6->setCompActiveFsmCol(false);
      fsm_s7->setCompActiveFsmCol(false);
      fsm_s8->setCompActiveFsmCol(false);
      fsm_s9->setCompActiveFsmCol(false);
      fsm_s10->setCompActiveFsmCol(false);
      fsm_s11->setCompActiveFsmCol(false);
      fsm_s12->setCompActiveFsmCol(false);
      fsm_show->setCompActiveFsmCol(false);

      fsm_s0->next_state.setActivePath(false);
      fsm_s1->next_state.setActiveFsm(false);
      fsm_s2->next_state.setActiveFsm(false);
      fsm_s3->next_state.setActiveFsm(false);
      fsm_s4->next_state.setActiveFsm(false);
      fsm_s5->next_state.setActiveFsm(false);
      fsm_s6->next_state.setActiveFsm(false);
      fsm_s7->next_state.setActiveFsm(false);
      fsm_s8->next_state.setActiveFsm(false);
      fsm_s9->next_state.setActiveFsm(false);
      fsm_s10->next_state.setActiveFsm(false);
      fsm_s11->next_state.setActiveFsm(false);
      fsm_s12->next_state.setActiveFsm(false);


      fsm_s0->setCompActiveFsm(false);
      fsm_s1->setCompActiveFsm(false);


      // Design subcomponents
      registerFile->setCompActivePath(false);
      alu->setCompActivePath(false);
      alu_branch->setCompActivePath(false);
      control->setCompActivePath(true);
      alu_control->setCompActiveFsm(false);
      alu_control->setCompActivePath(false);
      immediate->setCompActivePath(false);
      decode->setCompActivePath(false);
      pc_4->setCompActivePath(false);

      // Registers
      pc_reg->setCompActivePath(false);
      a->setCompActivePath(false);
      b->setCompActivePath(false);
      alu_out->setCompActivePath(false);
      hi_out->setCompActivePath(false);
      mem_data_reg->setCompActivePath(false);
      instr_reg->setCompActivePath(false);

      // Multiplexers
      reg_wr_src->setCompActivePath(false);
      pc_src->setCompActivePath(false);
      write_reg->setCompActivePath(false);
      pc_branch->setCompActivePath(false);
      alu_op1_src_multi->setCompActivePath(false);
      alu_op2_src_multi->setCompActivePath(false);
      pc_src_multi->setCompActivePath(false);
      address_multi->setCompActivePath(false);



      // Memories
      data_mem->setCompActivePath(false);

      // Gates
      br_and->setCompActiveFsm(true);
      bn_and->setCompActiveFsm(true);
      bg_and->setCompActiveFsm(true);
      bl_and->setCompActiveFsm(true);
      br_or->setCompActiveFsm(true);
      bn_not->setCompActiveFsm(true);
      pc_write_or->setCompActiveFsm(true);

      // Shift
      shift_pc->setCompActivePath(false);
      shift_branch->setCompActivePath(false);


      ecallChecker->setCompActivePath(false);
      j_address->setCompActivePath(false);


      switch(decode->state.uValue()){

      //Instruction Fetch
      case MIPSMulti_States::S0:{
          fsm_s0->setCompActiveFsmCol(true);


          pc_reg->setCompActivePath(true);
          address_multi->setCompActivePath(true);
          data_mem->setCompActivePath(true);
          alu_op1_src_multi->setCompActivePath(true);
          alu_op2_src_multi->setCompActivePath(true);
          alu->setCompActivePath(true);
          pc_src_multi->setCompActivePath(true);
          mem_data_reg->setCompActivePath(true);
          alu_control->setCompActiveFsm(true);

          pc_reg->out.setActivePath(true);
          address_multi->out.setActivePath(true);
          data_mem->data_out.setActivePath(true);
          alu_op1_src_multi->out.setActivePath(true);
          alu_op2_src_multi->out.setActivePath(true);
          alu->res.setActivePath(true);
          pc_src_multi->out.setActivePath(true);
          alu_control->res.setActivePath(true);



          //Control
          control->mem_do_read_ctrl.setActivePath(true);
          control->alu_op1_ctrl.setActivePath(true);
          control->iord.setActivePath(true);
          control->irwrite.setActivePath(true);
          control->alu_op2_ctrl.setActivePath(true);
          control->alu_ctrl.setActivePath(true);
          control->pc_write.setActivePath(true);
          control->pc_source.setActivePath(true);


          return "State 0    ";
      }
      case MIPSMulti_States::S1:{
           fsm_s0->setCompActiveFsm(true);
           fsm_s1->setCompActiveFsmCol(true);


           immediate->setCompActivePath(true);
           shift_branch->setCompActivePath(true);
           alu_op1_src_multi->setCompActivePath(true);
           alu_op2_src_multi->setCompActivePath(true);
           alu->setCompActivePath(true);
           decode->setCompActivePath(true);
           mem_data_reg->setCompActivePath(true);
           write_reg->setCompActivePath(true);
           reg_wr_src->setCompActivePath(true);
           registerFile->setCompActivePath(true);
           a->setCompActivePath(true);
           b->setCompActivePath(true);
           alu_control->setCompActiveFsm(true);





          mem_data_reg->out.setActivePath(true);
          pc_reg->out.setActivePath(true);
          decode->decode->instr_imm_15_0.setActivePath(true);
          immediate->imm.setActivePath(true);
          shift_branch->out.setActivePath(true);
          decode->decode->instr_rt_20_16 .setActivePath(true);
          decode->decode->instr_rs_25_21.setActivePath(true);
          decode->decode->opcode.setActivePath(true);
          registerFile->_rd1_mem->data_out.setActivePath(true);
          registerFile->_rd2_mem->data_out.setActivePath(true);
          alu_op1_src_multi->out.setActivePath(true);
          alu_op2_src_multi->out.setActivePath(true);
          alu->res.setActivePath(true);
          alu_control->res.setActivePath(true);


          //Control
          control->alu_op1_ctrl.setActivePath(true);
          control->alu_op2_ctrl.setActivePath(true);
          control->alu_ctrl.setActivePath(true);



          return "State 1    ";
      }
      case MIPSMulti_States::S2:{
          fsm_s0->setCompActiveFsm(true);
          fsm_s1->setCompActiveFsm(true);
          fsm_s2->setCompActiveFsmCol(true);


          immediate->setCompActivePath(true);
          a->setCompActivePath(true);
          alu_op1_src_multi->setCompActivePath(true);
          alu_op2_src_multi->setCompActivePath(true);
          alu->setCompActivePath(true);
          alu_control->setCompActiveFsm(true);


          decode->decode->instr_imm_15_0.setActivePath(true);
          immediate->imm.setActivePath(true);
          a->out.setActivePath(true);
          alu_op1_src_multi->out.setActivePath(true);
          alu_op2_src_multi->out.setActivePath(true);
          alu->res.setActivePath(true);
          alu_control->res.setActivePath(true);


          //Control
          control->alu_op1_ctrl.setActivePath(true);
          control->alu_op2_ctrl.setActivePath(true);
          control->alu_ctrl.setActivePath(true);






          return "State 2    ";
      }
      case MIPSMulti_States::S3:{
          fsm_s0->setCompActiveFsm(true);
          fsm_s1->setCompActiveFsm(true);
          fsm_s2->setCompActiveFsm(true);
          fsm_s3->setCompActiveFsmCol(true);



          address_multi->setCompActivePath(true);
          data_mem->setCompActivePath(true);
          alu_out->setCompActivePath(true);
          data_mem->setCompActivePath(true);


          address_multi->out.setActivePath(true);
          data_mem->data_out.setActivePath(true);
          alu_out->out.setActivePath(true);



          //Control
          control->mem_do_read_ctrl.setActivePath(true);
          control->iord.setActivePath(true);





          return "State 3    ";
      }
      case MIPSMulti_States::S4:{
          fsm_s0->setCompActiveFsm(true);
          fsm_s1->setCompActiveFsm(true);
          fsm_s2->setCompActiveFsm(true);
          fsm_s3->setCompActiveFsm(true);
          fsm_s4->setCompActiveFsmCol(true);


          mem_data_reg->setCompActivePath(true);
          decode->decode->setCompActivePath(true);
          write_reg->setCompActivePath(true);
          reg_wr_src->setCompActivePath(true);
          decode->setCompActivePath(true);


          mem_data_reg->out.setActivePath(true);
          decode->decode->instr_rt_20_16.setActivePath(true);
          write_reg->out.setActivePath(true);
          reg_wr_src->out.setActivePath(true);

          //Control
          control->reg_wr_src_ctrl.setActivePath(true);
          control->reg_do_write_ctrl.setActivePath(true);
          control->do_reg_dst.setActivePath(true);


          return "State 4    ";
      }
      case MIPSMulti_States::S5:{
          fsm_s0->setCompActiveFsm(true);
          fsm_s1->setCompActiveFsm(true);
          fsm_s2->setCompActiveFsm(true);
          fsm_s5->setCompActiveFsmCol(true);



          address_multi->setCompActivePath(true);
          alu_out->setCompActivePath(true);
          b->setCompActivePath(true);
          data_mem->setCompActivePath(true);

          address_multi->out.setActivePath(true);
          alu_out->out.setActivePath(true);

          //Control
          control->mem_do_write_ctrl.setActivePath(true);
          control->iord.setActivePath(true);


          return "State 5    ";
      }
      case MIPSMulti_States::S6:{
          fsm_s0->setCompActiveFsm(true);
          fsm_s1->setCompActiveFsm(true);
          fsm_s6->setCompActiveFsmCol(true);



          a->setCompActivePath(true);
          b->setCompActivePath(true);
          alu->setCompActivePath(true);
          alu_control->setCompActiveFsm(true);
          alu_op1_src_multi->setCompActivePath(true);
          alu_op2_src_multi->setCompActivePath(true);


          decode->decode->instr_imm_5_0.setActivePath(true);
          alu_control->res.setActivePath(true);
          a->out.setActivePath(true);
          b->out.setActivePath(true);
          alu_op1_src_multi->out.setActivePath(true);
          alu_op2_src_multi->out.setActivePath(true);
          alu->res.setActivePath(true);

          //Control
          control->alu_op1_ctrl.setActivePath(true);
          control->alu_op2_ctrl.setActivePath(true);
          control->alu_ctrl.setActivePath(true);


          return "State 6    ";
      }
      case MIPSMulti_States::S7:{
          fsm_s0->setCompActiveFsm(true);
          fsm_s1->setCompActiveFsm(true);
          fsm_s6->setCompActiveFsm(true);
          fsm_s7->setCompActiveFsmCol(true);


          alu_out->setCompActivePath(true);
          reg_wr_src->setCompActivePath(true);
          registerFile->setCompActivePath(true);
          decode->setCompActivePath(true);
          write_reg->setCompActivePath(true);


          decode->decode->instr_rd_15_11.setActivePath(true);
          write_reg->out.setActivePath(true);
          reg_wr_src->out.setActivePath(true);
          alu_out->out.setActivePath(true);

          //Control
          control->reg_wr_src_ctrl.setActivePath(true);
          control->reg_do_write_ctrl.setActivePath(true);
          control->do_reg_dst.setActivePath(true);


          return "State 7    ";
      }
      case MIPSMulti_States::S8:{
          fsm_s0->setCompActiveFsm(true);
          fsm_s1->setCompActiveFsm(true);
          fsm_s8->setCompActiveFsmCol(true);


          a->setCompActivePath(true);
          b->setCompActivePath(true);
          alu_op1_src_multi->setCompActivePath(true);
          alu_op2_src_multi->setCompActivePath(true);
          alu->setCompActivePath(true);
          alu_control->setCompActiveFsm(true);
          alu_out->setCompActivePath(true);
          pc_src_multi->setCompActivePath(true);
          pc_reg->setCompActivePath(true);


          a->out.setActivePath(true);
          b->out.setActivePath(true);
          alu_op1_src_multi->out.setActivePath(true);
          alu_op2_src_multi->out.setActivePath(true);
          alu->res.setActivePath(true);
          alu->zero.setActivePath(true);
          alu_out->out.setActivePath(true);
          pc_src_multi->out.setActivePath(true);
          alu_control->res.setActivePath(true);

          //Control
          control->alu_op1_ctrl.setActivePath(true);
          control->alu_op2_ctrl.setActivePath(true);
          control->alu_ctrl.setActivePath(true);
          control->pc_source.setActivePath(true);
          control->pc_write_cond.setActivePath(true);


          switch(opcode){
               case MIPS_Instr::BGEZ:{
                    control->do_bgt_write.setActivePath(true);
                    alu->greater.setActivePath(true);
               }
               case MIPS_Instr::BGTZ:{
                    control->do_bgt_write.setActivePath(true);
                    control->pc_write_cond.setActivePath(false);
                    alu->greater.setActivePath(true);
                    alu->zero.setActivePath(false);

               }
               case MIPS_Instr::BLEZ:{
                    control->do_blt_write.setActivePath(true);
                    alu->less.setActivePath(true);

               }
               case MIPS_Instr::BLTZ:{
                    control->do_blt_write.setActivePath(true);
                    control->pc_write_cond.setActivePath(false);
                    alu->less.setActivePath(true);
                    alu->zero.setActivePath(false);

               }
               case MIPS_Instr::BNE:{
                    control->do_bne_write.setActivePath(true);
                    control->pc_write_cond.setActivePath(false);
               }
          }


          return "State 8    ";
      }
      case MIPSMulti_States::S9:{
          fsm_s0->setCompActiveFsm(true);
          fsm_s1->setCompActiveFsm(true);
          fsm_s9->setCompActiveFsmCol(true);

          decode->setCompActivePath(true);
          shift_pc->setCompActivePath(true);
          j_address->setCompActivePath(true);
          pc_src_multi->setCompActivePath(true);
          pc_reg->setCompActivePath(true);

          pc_reg->out.setActivePath(true);
          decode->decode->instr_address_25_0.setActivePath(true);
          shift_pc->out.setActivePath(true);
          j_address->res.setActivePath(true);
          pc_src_multi->out.setActivePath(true);

          //Control
          control->pc_write.setActivePath(true);
          control->pc_source.setActivePath(true);



          return "State 9    ";
      }
      case MIPSMulti_States::S10:{
          fsm_s0->setCompActiveFsm(true);
          fsm_s1->setCompActiveFsm(true);
          fsm_s10->setCompActiveFsmCol(true);

          decode->setCompActivePath(true);
          immediate->setCompActivePath(true);
          a->setCompActivePath(true);
          alu_op1_src_multi->setCompActivePath(true);
          alu_op2_src_multi->setCompActivePath(true);
          alu->setCompActivePath(true);
          alu_out->setCompActivePath(true);


          decode->decode->instr_imm_15_0.setActivePath(true);
          decode->decode->instr_imm_5_0.setActivePath(true);
          alu_control->res.setActivePath(true);
          immediate->imm.setActivePath(true);
          a->out.setActivePath(true);
          alu_op1_src_multi->out.setActivePath(true);
          alu_op2_src_multi->out.setActivePath(true);
          alu->res.setActivePath(true);

          //Control
          control->alu_op1_ctrl.setActivePath(true);
          control->alu_op2_ctrl.setActivePath(true);
          control->alu_ctrl.setActivePath(true);


          return "State 10    ";
      }
      case MIPSMulti_States::S11:{

          fsm_s0->setCompActiveFsm(true);
          fsm_s1->setCompActiveFsm(true);
          fsm_s10->setCompActiveFsm(true);
          fsm_s11->setCompActiveFsmCol(true);


          alu_out->setCompActivePath(true);
          reg_wr_src->setCompActivePath(true);
          registerFile->setCompActivePath(true);
          decode->setCompActivePath(true);
          write_reg->setCompActivePath(true);

          decode->decode->instr_rt_20_16.setActivePath(true);
          write_reg->out.setActivePath(true);
          reg_wr_src->out.setActivePath(true);
          alu_out->out.setActivePath(true);

          //Control
          control->reg_wr_src_ctrl.setActivePath(true);
          control->reg_do_write_ctrl.setActivePath(true);
          control->do_reg_dst.setActivePath(true);



          return "State 11    ";
      }
      case MIPSMulti_States::S12:{

          fsm_s0->setCompActiveFsm(true);
          fsm_s1->setCompActiveFsm(true);
          fsm_s12->setCompActiveFsmCol(true);

          decode->setCompActivePath(true);
          shift_pc->setCompActivePath(true);
          j_address->setCompActivePath(true);
          pc_src_multi->setCompActivePath(true);
          pc_reg->setCompActivePath(true);


          pc_reg->out.setActivePath(true);
          decode->decode->instr_address_25_0.setActivePath(true);
          shift_pc->out.setActivePath(true);
          j_address->res.setActivePath(true);
          pc_src_multi->out.setActivePath(true);

          //Control
          control->pc_write.setActivePath(true);
          control->pc_source.setActivePath(true);
          control->reg_wr_src_ctrl.setActivePath(true);
          control->reg_do_write_ctrl.setActivePath(true);


          return "State 12    ";
      }



      default: assert(false && "Processor does not contain state");
      }
        Q_UNREACHABLE();
    // clang-format on
  }
  StageInfo stageInfo(StageIndex) const override {
      if(pc_write_or->out.uValue() == 1  && decode->state.uValue() !=  MIPSMulti_States::S8 &&
             decode->state.uValue() !=  MIPSMulti_States::S9 && decode->state.uValue() !=  MIPSMulti_States::S1 &&
              decode->state.uValue() !=  MIPSMulti_States::S12 && decode->state.uValue() !=  MIPSMulti_States::S7){
          return StageInfo({pc_reg->out.uValue(),
                            isExecutableAddress(pc_reg->out.uValue()),
                            StageInfo::State::None});
      }

      return StageInfo({pc_reg->out.uValue()-4,
                        isExecutableAddress(pc_reg->out.uValue()-4),
                        StageInfo::State::None});


  }
  void setProgramCounter(AInt address) override {
    pc_reg->forceValue(0, address);
    propagateDesign();
  }
  void setPCInitialValue(AInt address) override {
    pc_reg->setInitValue(address);
  }
  AddressSpaceMM &getMemory() override { return *m_memory; }
  VInt getRegister(RegisterFileType, unsigned i) const override {
    return registerFile->getRegister(i);
  }
  void finalize(FinalizeReason fr) override {
    if (fr == FinalizeReason::exitSyscall) {
      // Allow one additional clock cycle to clear the current instruction
      m_finishInNextCycle = true;
    }
  }
  bool finished() const override {
    return m_finished || !stageInfo({0, 0}).stage_valid;
  }
  const std::vector<StageIndex> breakpointTriggeringStages() const override {
    return {{0, 0}};
  }

  MemoryAccess dataMemAccess() const override {
    return memToAccessInfo(data_mem);
  }
  MemoryAccess instrMemAccess() const override {
    auto instrAccess = memToAccessInfo(data_mem->instr_mem);
    instrAccess.type = MemoryAccess::Read;
    return instrAccess;
  }

  void setRegister(RegisterFileType, unsigned i, VInt v) override {
    setSynchronousValue(registerFile->_wr_mem, i, v);
    decode->initialState();
  }

  void clockProcessor() override {
    // Single cycle processor; 1 instruction retired per cycle!
    m_instructionsRetired++;

    // m_finishInNextCycle may be set during Design::clock(). Store the value
    // before clocking the processor, and emit finished if this was the final
    // clock cycle.
    const bool finishInThisCycle = m_finishInNextCycle;
    Design::clock();
    if (finishInThisCycle) {
      m_finished = true;
    }
  }

  void reverse() override {
    m_instructionsRetired--;
    Design::reverse();
    // Ensure that reverses performed when we expected to finish in the
    // following cycle, clears this expectation.
    m_finishInNextCycle = false;
    m_finished = false;
  }

  ///DONE
  void reset() override {
    ecallChecker->setSysCallExiting(false);
    Design::reset();
    m_syscallExitCycle = -1;
  }

  static ProcessorISAInfo supportsISA() {
    return ProcessorISAInfo{
        std::make_shared<ISAInfo<XLenToMIPSISA<XLEN>()>>(QStringList())
    };
  }
  const ISAInfoBase *implementsISA() const override {
    return m_enabledISA.get();
  }
  const std::set<RegisterFileType> registerFiles() const override {
    std::set<RegisterFileType> rfs;
    rfs.insert(RegisterFileType::GPR);

    // @TODO: uncomment when enabling floating-point support
    // if (implementsISA()->extensionEnabled("F")) {
    //     rfs.insert(RegisterFileType::Float);
    // }
    return rfs;
  }

private:
  bool m_finishInNextCycle = false;
  bool m_finished = false;
  long long m_syscallExitCycle = -1;
  std::shared_ptr<ISAInfoBase> m_enabledISA;
  ProcessorStructure m_structure = {{0, 1}};
};

} // namespace core
} // namespace vsrtl
