#pragma once

#include "VSRTL/core/vsrtl_adder.h"
#include "VSRTL/core/vsrtl_constant.h"
#include "VSRTL/core/vsrtl_design.h"
#include "VSRTL/core/vsrtl_logicgate.h"
#include "VSRTL/core/vsrtl_multiplexer.h"
#include "VSRTL/graphics/vsrtl_portgraphic.h"

#include "../../ripesvsrtlprocessor.h"

#include "../../MIPS/mips.h"
#include "../../MIPS/mips_alu.h"
#include "../../MIPS/mips_control.h"
#include "../../MIPS/mips_alu_control.h"
#include "../../MIPS/mips_ecallchecker.h"
#include "../../MIPS/mips_immediate.h"
#include "../../MIPS/mips_memory.h"
#include "../../MIPS/mips_shift.h"
#include "../../MIPS/mips_shift_extend.h"
#include "../../MIPS/mips_registerfile.h"
#include "../../MIPS/mips_jump_address.h"
#include "../../MIPS/mss/mips_decodeRVC.h"
#include <QPalette>
#include <QPen>



namespace vsrtl {
namespace core {
using namespace Ripes;

template <typename XLEN_T>
class MSS : public RipesVSRTLProcessor {
  static_assert(std::is_same<uint32_t, XLEN_T>::value,
                "Only supports 32-bit variants");
  static constexpr unsigned XLEN = sizeof(XLEN_T) * CHAR_BIT;

public:
  MSS(const QStringList &extensions)
      : RipesVSRTLProcessor("Single Cycle MIPS Processor") {
    m_enabledISA = std::make_shared<ISAInfo<XLenToMIPSISA<XLEN>()>>(extensions);
    decode->setISA(m_enabledISA);



    // -----------------------------------------------------------------------
    // Program counter
    pc_reg->out >> pc_4->op2;
    4 >> pc_4->op1;
    pc_src->out >> pc_reg->in;

    // Note: pc_src works uses the PcSrc enum, but is selected by the boolean
    // signal from the controlflow OR gate. PcSrc enum values must adhere to the
    // boolean 0/1 values.

    // -----------------------------------------------------------------------
    // Instruction memory
    pc_reg->out >> instr_mem->addr;
    instr_mem->setMemory(m_memory);

    // -----------------------------------------------------------------------
    // Decode
    instr_mem->data_out >> decode->instr;

    // -----------------------------------------------------------------------
    // Control signals
    decode->instr_opc_31_26 >> control->instr31_26;
    decode->opcode >> control->opcode;

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
    pc_4->out >> registerFile->pc_4;

    decode->instr_rt_20_16 >> write_reg->get(MIPS_WrReg::RR);
    decode->instr_rd_15_11 >> write_reg->get(MIPS_WrReg::WR);
    decode->instr_shamt_10_6 >> alu->shamt;
    decode->opcode >> registerFile->opcode;
    control->do_reg_dst >> write_reg->select;
    write_reg->out >> registerFile->wr_addr;
    alu->hi >> registerFile->hi_in;

    control->reg_do_write_ctrl >> registerFile->wr_en;
    reg_wr_src->out >> registerFile->data_in;


    data_mem->data_out >> reg_wr_src->get(MIPS_RegWrSrc::MEMREAD);
    alu->res >> reg_wr_src->get(MIPS_RegWrSrc::ALURES);
    control->reg_wr_src_ctrl >> reg_wr_src->select;

    registerFile->setMemory(m_regMem);

    // -----------------------------------------------------------------------
    // Branch

    alu->zero >> *br_and->in[1];
    control->do_branch >> *br_and->in[0];

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

    registerFile->r1_out >> alu->op1;

    registerFile->r2_out >> alu_op2_src->get(MIPS_AluSrc2::REG2);
    immediate->imm >> alu_op2_src->get(MIPS_AluSrc2::IMM);
    control->alu_op2_ctrl >> alu_op2_src->select;

    alu_op2_src->out >> alu->op2;

    control->alu_ctrl >> alu_control->ctrl;
    decode->instr_imm_5_0 >> alu_control->instr5_0;
    alu_control->res >> alu->ctrl;

    pc_4->out >> alu_branch->op1;
    shift_branch->out >> alu_branch->op2;

    alu_branch->out >> pc_branch->get(MIPS_PcBranch::IN2);
    pc_4->out >> pc_branch->get(MIPS_PcBranch::IN1);
    br_or->out >> pc_branch->select;

    j_address->res >> pc_src->get(MIPS_PcSrc::ALU);
    pc_branch->out >> pc_src->get(MIPS_PcSrc::PC4);

    // -----------------------------------------------------------------------
    //Jump

    pc_4->out >> j_address->pc;
    shift_pc->out >> j_address->sl;
    decode->opcode >> j_address->opcode;
    registerFile->r1_out >> j_address->ra;


    // -----------------------------------------------------------------------
    // Shift
    decode->instr_address_25_0 >> shift_pc->in;
    immediate->imm >> shift_branch->in;

    // -----------------------------------------------------------------------
    // Data memory
    alu->res >> data_mem->addr;                         //yes
    control->mem_do_write_ctrl >> data_mem->wr_en;
    registerFile->r2_out >> data_mem->data_in;
    control->mem_ctrl >> data_mem->op;
    control->mem_do_read_ctrl >> data_mem->mem_read;
    data_mem->mem->setMemory(m_memory);

    // -----------------------------------------------------------------------
    // Ecall checker
    decode->opcode >> ecallChecker->opcode;
    ecallChecker->setSyscallCallback(&trapHandler);
    0 >> ecallChecker->stallEcallHandling;
  }

  // Design subcomponents
  SUBCOMPONENT(registerFile, TYPE(MIPS_RegisterFile<XLEN, false>));
  SUBCOMPONENT(alu, TYPE(MIPS_ALU<XLEN>));
  SUBCOMPONENT(alu_branch, Adder<XLEN>);
  SUBCOMPONENT(control, MIPS_Control);
  SUBCOMPONENT(alu_control, MIPS_ALU_Control);
  SUBCOMPONENT(immediate, TYPE(MIPS_Immediate<XLEN>));
  SUBCOMPONENT(decode, TYPE(MIPS_DecodeC<XLEN>));
  SUBCOMPONENT(pc_4, Adder<XLEN>);

  // Registers
  SUBCOMPONENT(pc_reg, Register<XLEN>);

  // Multiplexers
  SUBCOMPONENT(reg_wr_src, TYPE(EnumMultiplexer<MIPS_RegWrSrc, XLEN>));
  SUBCOMPONENT(pc_src, TYPE(EnumMultiplexer<MIPS_PcSrc, XLEN>));
  SUBCOMPONENT(alu_op2_src, TYPE(EnumMultiplexer<MIPS_AluSrc2, XLEN>));
  SUBCOMPONENT(write_reg, TYPE(EnumMultiplexer<MIPS_WrReg, c_MIPSRegsBits>));
  SUBCOMPONENT(pc_branch, TYPE(EnumMultiplexer<MIPS_PcBranch, XLEN>));

  // Memories
  SUBCOMPONENT(instr_mem, TYPE(ROM<XLEN, c_MIPSInstrWidth>));
  SUBCOMPONENT(data_mem, TYPE(MIPS_Memory<XLEN, XLEN>));

  // Gates
  SUBCOMPONENT(br_and, TYPE(And<1, 2>));
  SUBCOMPONENT(bn_and, TYPE(And<1, 2>));
  SUBCOMPONENT(bg_and, TYPE(And<1, 2>));
  SUBCOMPONENT(bl_and, TYPE(And<1, 2>));
  SUBCOMPONENT(br_or, TYPE(Or<1, 4>));
  SUBCOMPONENT(bn_not, TYPE(Not<1, 1>));

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
  QString stageName(StageIndex) const override {
    return "•";
  }
  StageInfo stageInfo(StageIndex) const override {
    return StageInfo({pc_reg->out.uValue(),
                      isExecutableAddress(pc_reg->out.uValue()),
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
    auto instrAccess = memToAccessInfo(instr_mem);
    instrAccess.type = MemoryAccess::Read;
    return instrAccess;
  }

  void setRegister(RegisterFileType, unsigned i, VInt v) override {
    setSynchronousValue(registerFile->_wr_mem, i, v);
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

  void reset() override {
    Design::reset();
    m_finishInNextCycle = false;
    m_finished = false;
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
  std::shared_ptr<ISAInfoBase> m_enabledISA;
  ProcessorStructure m_structure = {{0, 1}};
};

} // namespace core
} // namespace vsrtl
