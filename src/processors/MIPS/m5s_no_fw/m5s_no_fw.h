#pragma once

#include "VSRTL/core/vsrtl_adder.h"
#include "VSRTL/core/vsrtl_constant.h"
#include "VSRTL/core/vsrtl_design.h"
#include "VSRTL/core/vsrtl_logicgate.h"
#include "VSRTL/core/vsrtl_multiplexer.h"

#include "../../ripesvsrtlprocessor.h"

#include "../../MIPS/mips.h"
#include "../../MIPS/mips_alu_m5s.h"
#include "../../MIPS/mips_branch.h"
#include "../../MIPS/mips_control_m5s.h"
#include "../../MIPS/mips_alu_control.h"
#include "../../MIPS/mips_ecallchecker.h"
#include "../../MIPS/mips_immediate.h"
#include "../../MIPS/mips_memory.h"
#include "../../MIPS/mips_shift.h"
#include "../../MIPS/mips_shift_extend.h"
#include "../../MIPS/mips_registerfile.h"
#include "../../MIPS/mips_jump_address.h"
#include "../../MIPS/mss/mips_decodeRVC.h"

#include "../m5s_no_fw_hz/m5s_no_fw_hz_ifid.h"
#include "m5s_no_fw_idex.h"
#include "../m5s/m5s_exmem.h"
#include "../m5s/m5s_memwb.h"

#include "../../MIPS/m5s_no_fw/m5s_no_fw_hazardunit.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <typename XLEN_T>
class M5S_NO_FW : public RipesVSRTLProcessor {
  static_assert(std::is_same<uint32_t, XLEN_T>::value,
                "Only supports 32-bit variants");
  static constexpr unsigned XLEN = sizeof(XLEN_T) * CHAR_BIT;

public:
  enum Stage { IF = 0, ID = 1, EX = 2, MEM = 3, WB = 4, STAGECOUNT };
  M5S_NO_FW(const QStringList &extensions)
      : RipesVSRTLProcessor("5-stage MIPS Processor without forwarding") {
    m_enabledISA = std::make_shared<ISAInfo<XLenToMIPSISA<XLEN>()>>(extensions);
    decode->setISA(m_enabledISA);

    // -----------------------------------------------------------------------
    // Program counter
    pc_reg->out >> pc_4->op2;
    4 >> pc_4->op1;
    pc_branch->out >> pc_reg->in;
    hzunit->hazardFEEnable >> pc_reg->enable;
    0 >> pc_reg->clear;

    flush_or->out >> *efsc_or->in[0];
    ecallChecker->syscallExit >> *efsc_or->in[1];

    efsc_or->out >> *efschz_or->in[0];
    hzunit->hazardIDEXClear >> *efschz_or->in[1];

    // Note: pc_src works uses the PcSrc enum, but is selected by the boolean
    // signal from the controlflow OR gate. PcSrc enum values must adhere to the
    // boolean 0/1 values.

    // -----------------------------------------------------------------------
    // IF ID
    pc_reg->out >> ifid_reg->pc_in;

    1 >> ifid_reg->valid_in;

    hzunit->hazardFEEnable >> ifid_reg->enable;
    efsc_or->out >> ifid_reg->clear;

    instr_mem->data_out >> ifid_reg->instr_in;
    pc_4->out >> ifid_reg->pc4_in;

    // -----------------------------------------------------------------------
    // ID EX
    ifid_reg->pc_out >> idex_reg->pc_in;



    hzunit->hazardIDEXEnable >> idex_reg->enable;
    hzunit->hazardIDEXClear >> idex_reg->stalled_in;
    efschz_or->out >> idex_reg->clear;

    ifid_reg->valid_out >> idex_reg->valid_in;
    decode->opcode>> idex_reg->opcode_no_fw_in;

    ifid_reg->pc4_out >> idex_reg->pc4_in;
    registerFile->r1_out >> idex_reg->r1_in;
    registerFile->r2_out >> idex_reg->r2_in;
    immediate->imm >> idex_reg->imm_in;
    decode->instr_rt_20_16 >> idex_reg->instr_20_16_in;
    decode->instr_address_25_0 >> idex_reg->instr_25_0_in;
    decode->instr_rd_15_11 >> idex_reg->instr_15_11_in;
    decode->instr_shamt_10_6 >> idex_reg->instr_shamt_10_6_in;
    decode->instr_imm_5_0 >> idex_reg->instr_5_0_in;
    j_address->res >> idex_reg->j_address_in;
    decode->opcode>> idex_reg->opcode_in;

    decode->mt_sel >> idex_reg->mt_sel_in;
    decode->mf_sel >> idex_reg->mf_sel_in;
    decode->mult_div_sel >> idex_reg->mult_div_sel_in;
    decode->j_sel >> idex_reg->j_sel_in;
    ifid_reg->pc4_out >> idex_reg->pc_4_in;

    idex_reg->mt_sel_out >> exmem_reg->mt_sel_in;
    idex_reg->mf_sel_out >> exmem_reg->mf_sel_in;
    idex_reg->mult_div_sel_out >> exmem_reg->mult_div_sel_in;
    idex_reg->j_sel_out >> exmem_reg->j_sel_in;
    idex_reg->pc_4_out >> exmem_reg->pc_4_in;

    // Control
    control->alu_op2_ctrl >> idex_reg->alu_op2_ctrl_in;
    control->alu_ctrl >> idex_reg->alu_ctrl_in;
    control->do_reg_dst >> idex_reg->do_reg_dst_in;
    control->do_branch >> idex_reg->do_br_in;
    control->do_bne_write >> idex_reg->do_bn_in;
    control->do_bgt_write >> idex_reg->do_bg_in;
    control->do_blt_write >> idex_reg->do_bl_in;
    control->mem_ctrl >> idex_reg->mem_ctrl_in;
    control->mem_do_read_ctrl >> idex_reg->mem_do_read_ctrl_in;
    control->mem_do_write_ctrl >> idex_reg->mem_do_write_ctrl_in;
    control->reg_do_write_ctrl >> idex_reg->reg_do_write_in;
    control->reg_wr_src_ctrl >> idex_reg->reg_wr_src_ctrl_in;
    control->do_jump >> idex_reg->do_jmp_in;

    // -----------------------------------------------------------------------
    // EX MEM
    idex_reg->pc_out >> exmem_reg->pc_in;

    1 >> exmem_reg->enable;
    idex_reg->valid_out >> exmem_reg->valid_in;

    flush_or->out >> *mem_clear_or->in[0];
    hzunit->hazardEXMEMClear >> *mem_clear_or->in[1];
    mem_clear_or->out >> exmem_reg->clear;
    hzunit->hazardEXMEMClear >> *mem_stalled_or->in[0];
    idex_reg->stalled_out >> *mem_stalled_or->in[1];
    mem_stalled_or->out >> exmem_reg->stalled_in;

    alu->zero >> exmem_reg->aluzero_in;
    alu->res >> exmem_reg->alures_in;
    alu->hi >> exmem_reg->aluhi_in;
    alu->greater >> exmem_reg->alugreater_in;
    alu->less >> exmem_reg->aluless_in;

    idex_reg->r2_out >> exmem_reg->r2_in;
    write_reg->out >> exmem_reg->write_reg_in;
    idex_reg->opcode_out >> exmem_reg->opcode_in;

    exmem_reg->mt_sel_out >> memwb_reg->mt_sel_in;
    exmem_reg->mf_sel_out >> memwb_reg->mf_sel_in;
    exmem_reg->mult_div_sel_out >> memwb_reg->mult_div_sel_in;
    exmem_reg->j_sel_out >> memwb_reg->j_sel_in;
    exmem_reg->pc_4_out >> memwb_reg->pc_4_in;

    // Control
    idex_reg->do_br_out >> exmem_reg->do_br_in;
    idex_reg->do_bg_out >> exmem_reg->do_bg_in;
    idex_reg->do_bl_out >> exmem_reg->do_bl_in;
    idex_reg->do_bn_out >> exmem_reg->do_bn_in;
    idex_reg->do_jmp_out >> exmem_reg->do_jmp_in;
    idex_reg->mem_ctrl_out >> exmem_reg->mem_ctrl_in;
    idex_reg->mem_do_read_ctrl_out >> exmem_reg->mem_do_read_ctrl_in;
    idex_reg->mem_do_write_ctrl_out >> exmem_reg->mem_do_write_ctrl_in;
    idex_reg->reg_do_write_out >> exmem_reg->reg_do_write_in;
    idex_reg->reg_wr_src_ctrl_out >> exmem_reg->reg_wr_src_ctrl_in;

    // -----------------------------------------------------------------------
    // MEM WB
    exmem_reg->pc_out >> memwb_reg->pc_in;

    exmem_reg->valid_out >> memwb_reg->valid_in;

    data_mem->data_out >> memwb_reg->mem_read_in;
    exmem_reg->alures_out >> memwb_reg->alures_in;
    exmem_reg->aluhi_out >> memwb_reg->aluhi_in;
    exmem_reg->write_reg_out >> memwb_reg->write_reg_in;
    exmem_reg->opcode_out >> memwb_reg->opcode_in;

    memwb_reg->mt_sel_out >> registerFile->mt_sel;
    decode->mf_sel >>registerFile->mf_sel;
    memwb_reg->mult_div_sel_out >> registerFile->mult_div_sel;
    memwb_reg->j_sel_out >> registerFile->j_sel;
    memwb_reg->pc_4_out >> registerFile->pc_4;

    // Control
    exmem_reg->reg_do_write_out >> memwb_reg->reg_do_write_in;
    exmem_reg->reg_wr_src_ctrl_out >> memwb_reg->reg_wr_src_ctrl_in;

    // -----------------------------------------------------------------------
    // Instruction memory
    pc_reg->out >> instr_mem->addr;
    instr_mem->setMemory(m_memory);

    // -----------------------------------------------------------------------
    // Decode
    ifid_reg->instr_out >> decode->instr;

    // -----------------------------------------------------------------------
    // Control signals
    decode->instr_opc_31_26 >> control->instr31_26;
    decode->opcode >> control->opcode;

    // -----------------------------------------------------------------------
    // Immediate

    decode->instr_imm_15_0 >> immediate->instr15_0;

    // -----------------------------------------------------------------------
    // Registers


    decode->instr_rs_25_21 >> registerFile->r1_addr;
    decode->instr_rt_20_16 >> registerFile->r2_addr;

    idex_reg->instr_20_16_out >> write_reg->get(MIPS_WrReg::RR);
    idex_reg->instr_15_11_out >> write_reg->get(MIPS_WrReg::WR);
    idex_reg->instr_shamt_10_6_out >> alu->shamt;
    memwb_reg->opcode_out >> registerFile->opcode;
    idex_reg->do_reg_dst_out >> write_reg->select;
    memwb_reg->write_reg_out >> registerFile->wr_addr;
    memwb_reg->aluhi_out >> registerFile->hi_in;

    memwb_reg->reg_do_write_out >> registerFile->wr_en;
    reg_wr_src->out >> registerFile->data_in;


    memwb_reg->mem_read_out >> reg_wr_src->get(MIPS_RegWrSrc::MEMREAD);
    memwb_reg->alures_out >> reg_wr_src->get(MIPS_RegWrSrc::ALURES);
    memwb_reg->reg_wr_src_ctrl_out >> reg_wr_src->select;

    registerFile->setMemory(m_regMem);

    // -----------------------------------------------------------------------
    // Branch


    exmem_reg->aluzero_out >> *br_and->in[1];
    exmem_reg->do_br_out >> *br_and->in[0];

    exmem_reg->alugreater_out >> *bg_and->in[1];
    exmem_reg->do_bg_out >> *bg_and->in[0];

    exmem_reg->aluless_out >> *bl_and->in[1];
    exmem_reg->do_bl_out >> *bl_and->in[0];

    exmem_reg->aluzero_out >> *bn_not->in[0];
    bn_not->out >> *bn_and->in[1];
    exmem_reg->do_bn_out >> *bn_and->in[0];



    registerFile->r1_out >> branch_eq->op1;
    registerFile->r2_out >> branch_eq->op2;
    control->comp_ctrl >> branch_eq->comp_op;

    pc_src->out >> exmem_reg->pc_branch_in;
    exmem_reg->pc_branch_out >> pc_branch->get(MIPS_PcBranch::IN2);
    pc_4->out >> pc_branch->get(MIPS_PcBranch::IN1);
    idex_reg->do_jmp_out >> pc_src->select;
    br_and->out >> *flush_or->in[0];
    exmem_reg->do_jmp_out >> *flush_or->in[1];
    bn_and->out >> *flush_or->in[2];
    bl_and->out >> *flush_or->in[3];
    bg_and->out >> *flush_or->in[4];

    flush_or->out >> pc_branch->select;


    j_address->res >> pc_src->get(MIPS_PcSrc::ALU);
    alu_branch->out >> pc_src->get(MIPS_PcSrc::PC4);

    // -----------------------------------------------------------------------
    // ALU
    idex_reg->r1_out >> alu->op1;
    idex_reg->r2_out >> alu_op2_src->get(MIPS_AluSrc2::REG2);
    idex_reg->imm_out >> alu_op2_src->get(MIPS_AluSrc2::IMM);
    idex_reg->alu_op2_ctrl_out >> alu_op2_src->select;

    alu_op2_src->out >> alu->op2;

    idex_reg->alu_ctrl_out >> alu_control->ctrl;
    idex_reg->instr_5_0_out >> alu_control->instr5_0;
    alu_control->res >> alu->ctrl;


    idex_reg->pc4_out >> alu_branch->op1;
    shift_branch->out >> alu_branch->op2;

    exmem_reg->aluhi_out >> alu->hi_in_mem;
    memwb_reg->aluhi_out >> alu->hi_in_wb;

    exmem_reg->opcode_out >> alu->opcode_mem;
    memwb_reg->opcode_out >> alu->opcode_wb;

    // -----------------------------------------------------------------------
    //Jump

    idex_reg->pc_out >> j_address->pc;
    shift_pc->out >> j_address->sl;
    idex_reg->opcode_out >> j_address->opcode;
    idex_reg->r1_out >> j_address->ra;



    // -----------------------------------------------------------------------
    // Shift
    idex_reg->instr_25_0_out >> shift_pc->in;
    idex_reg->imm_out >> shift_branch->in;

    // -----------------------------------------------------------------------
    // Data memory
    exmem_reg->alures_out >> data_mem->addr;
    exmem_reg->mem_do_write_ctrl_out >> data_mem->wr_en;
    exmem_reg->r2_out >> data_mem->data_in;
    exmem_reg->mem_ctrl_out >> data_mem->op;
    exmem_reg->mem_do_read_ctrl_out >> data_mem->mem_read;
    data_mem->mem->setMemory(m_memory);

    // -----------------------------------------------------------------------
    // Ecall checker
    idex_reg->opcode_out >> ecallChecker->opcode;
    ecallChecker->setSyscallCallback(&trapHandler);
    hzunit->stallEcallHandling >> ecallChecker->stallEcallHandling;

    // -----------------------------------------------------------------------
    // Hazard Unit
    decode->instr_rs_25_21 >> hzunit->id_reg1_idx;
    decode->instr_rt_20_16 >> hzunit->id_reg2_idx;
    control->alu_op2_ctrl >> hzunit->id_alu_op_ctrl_2;
    control->do_branch >> hzunit->id_do_branch;
    control->mem_do_write_ctrl >> hzunit->id_mem_do_write;
    decode->opcode >> hzunit->id_opcode;


    idex_reg->mem_do_read_ctrl_out >> hzunit->ex_do_mem_read_en;
    idex_reg->reg_do_write_out >> hzunit->ex_do_reg_write;
    write_reg->out >> hzunit->ex_reg_wr_idx;
    idex_reg->do_jmp_out >> hzunit->ex_do_jump;
    idex_reg->opcode_out >> hzunit->ex_opcode;
    flush_or->out >> hzunit->ex_branch_taken;

    exmem_reg->reg_do_write_out >> hzunit->mem_do_reg_write;
    exmem_reg->write_reg_out >> hzunit->mem_reg_wr_idx;
    exmem_reg->opcode_out >> hzunit->mem_opcode;


    memwb_reg->reg_do_write_out >> hzunit->wb_do_reg_write;
  }

  // Design subcomponents
  SUBCOMPONENT(registerFile, TYPE(MIPS_RegisterFile<XLEN, true>));
  SUBCOMPONENT(alu, TYPE(MIPS_ALU_M5S<XLEN>));
  SUBCOMPONENT(alu_branch, Adder<XLEN>);
  SUBCOMPONENT(control, MIPS_Control_M5S);
  SUBCOMPONENT(alu_control, MIPS_ALU_Control);
  SUBCOMPONENT(immediate, TYPE(MIPS_Immediate<XLEN>));
  SUBCOMPONENT(decode, TYPE(MIPS_DecodeC<XLEN>));
  SUBCOMPONENT(pc_4, Adder<XLEN>);
  SUBCOMPONENT(branch_eq, TYPE(MIPS_Branch<XLEN>));

  // Registers
  SUBCOMPONENT(pc_reg, RegisterClEn<XLEN>);

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
  SUBCOMPONENT(flush_or, TYPE(Or<1, 5>));
  SUBCOMPONENT(efsc_or, TYPE(Or<1, 2>));
  SUBCOMPONENT(efschz_or, TYPE(Or<1, 2>));
  SUBCOMPONENT(mem_stalled_or, TYPE(Or<1, 2>));
  SUBCOMPONENT(mem_clear_or, TYPE(Or<1, 2>));


  SUBCOMPONENT(bn_and, TYPE(And<1, 2>));
  SUBCOMPONENT(bg_and, TYPE(And<1, 2>));
  SUBCOMPONENT(bl_and, TYPE(And<1, 2>));
  SUBCOMPONENT(bn_not, TYPE(Not<1, 1>));

  // Shift
  SUBCOMPONENT(shift_pc, MIPS_Shift_Extend<26>, ShiftTypeExtend::sl, 2);
  SUBCOMPONENT(shift_branch, MIPS_Shift<32>, ShiftType::sl, 2);

  SUBCOMPONENT(hzunit, MIPS_HazardUnit_NO_FW);

  // Stage Registers
  SUBCOMPONENT(ifid_reg, TYPE(MIPS_IFID<XLEN>));
  SUBCOMPONENT(idex_reg, TYPE(M5S_NO_FW_IDEX<XLEN>));
  SUBCOMPONENT(exmem_reg, TYPE(M5S_EXMEM<XLEN>));
  SUBCOMPONENT(memwb_reg, TYPE(MIPS_MEMWB<XLEN>));

  // Address spaces
  ADDRESSSPACEMM(m_memory);
  ADDRESSSPACE(m_regMem);

  SUBCOMPONENT(ecallChecker, MIPS_EcallChecker);
  SUBCOMPONENT(j_address, TYPE(MIPS_Jump_Address<XLEN>));

  // Ripes interface compliance
  const ProcessorStructure &structure() const override { return m_structure; }
  unsigned int getPcForStage(StageIndex idx) const override {
    // clang-format off
        switch (idx.index()) {
            case IF: return pc_reg->out.uValue();
            case ID: return ifid_reg->pc_out.uValue();
            case EX: return idex_reg->pc_out.uValue();
            case MEM: return exmem_reg->pc_out.uValue();
            case WB: return memwb_reg->pc_out.uValue();
            default: assert(false && "Processor does not contain stage");
        }
        Q_UNREACHABLE();
    // clang-format on
  }
  AInt nextFetchedAddress() const override { return pc_src->out.uValue(); }
  QString stageName(StageIndex idx) const override {
    // clang-format off
        switch (idx.index()) {
            case IF: return "IF";
            case ID: return "ID";
            case EX: return "EX";
            case MEM: return "MEM";
            case WB: return "WB";
            default: assert(false && "Processor does not contain stage");
        }
        Q_UNREACHABLE();
    // clang-format on
  }
  StageInfo stageInfo(StageIndex stage) const override {
    bool stageValid = true;
    // Has the pipeline stage been filled?
    stageValid &= stage.index() <= m_cycleCount;

    // clang-format off
        // Has the stage been cleared?
        switch(stage.index()){
        case ID: stageValid &= ifid_reg->valid_out.uValue(); break;
        case EX: stageValid &= idex_reg->valid_out.uValue(); break;
        case MEM: stageValid &= exmem_reg->valid_out.uValue(); break;
        case WB: stageValid &= memwb_reg->valid_out.uValue(); break;
        default: case IF: break;
        }

        // Is the stage carrying a valid (executable) PC?
        switch(stage.index()){
        case ID: stageValid &= isExecutableAddress(ifid_reg->pc_out.uValue()); break;
        case EX: stageValid &= isExecutableAddress(idex_reg->pc_out.uValue()); break;
        case MEM: stageValid &= isExecutableAddress(exmem_reg->pc_out.uValue()); break;
        case WB: stageValid &= isExecutableAddress(memwb_reg->pc_out.uValue()); break;
        default: case IF: stageValid &= isExecutableAddress(pc_reg->out.uValue()); break;
        }

        // Are we currently clearing the pipeline due to a syscall exit? if such, all stages before the EX stage are invalid
        if(stage.index() < EX){
            stageValid &= !ecallChecker->isSysCallExiting();
        }

        /// IF
        if(stage.index() == 0){
            pc_branch->setCompActivePath(false);
            pc_4->setCompActivePath(false);
            pc_reg->setCompActivePath(false);
            instr_mem->setCompActivePath(false);

            pc_reg->out.setActivePath(false);
            pc_branch->out.setActivePath(false);
            pc_4->out.setActivePath(false);
            instr_mem->data_out.setActivePath(false);

            if(stageValid){
                pc_branch->setCompActivePath(true);
                pc_4->setCompActivePath(true);
                pc_reg->setCompActivePath(true);
                instr_mem->setCompActivePath(true);


                pc_reg->out.setActivePath(true);
                pc_branch->out.setActivePath(true);
                pc_4->out.setActivePath(true);
                instr_mem->data_out.setActivePath(true);
            }
        }


        /// ID
        else if(stage.index() == 1){
            ifid_reg->setCompActivePath(false);
            decode->setCompActivePath(false);
            control->setCompActivePath(false);
            registerFile->setCompActivePath(false);
            immediate->setCompActivePath(false);
            hzunit->setCompActivePath(false);

            ifid_reg->instr_reg->out.setActivePath(false);
            ifid_reg->pc4_reg->out.setActivePath(false);
            decode->decode->instr_opc_31_26.setActivePath(false);
            decode->decode->instr_rs_25_21.setActivePath(false);
            decode->decode->instr_rt_20_16.setActivePath(false);
            decode->decode->instr_rd_15_11.setActivePath(false);
            decode->decode->instr_imm_15_0.setActivePath(false);
            decode->decode->instr_imm_5_0.setActivePath(false);
            immediate->imm.setActivePath(false);
            registerFile->r1_out.setActivePath(false);
            registerFile->r2_out.setActivePath(false);

            control->do_branch.setActivePath(false);
            control->mem_do_write_ctrl.setActivePath(false);
            control->do_reg_dst.setActivePath(false);

            if(stageValid){
                ifid_reg->setCompActivePath(true);
                decode->setCompActivePath(true);
                control->setCompActivePath(true);
                registerFile->setCompActivePath(true);
                hzunit->setCompActivePath(true);


                ifid_reg->instr_reg->out.setActivePath(true);
                decode->decode->instr_opc_31_26.setActivePath(true);
                control->do_branch.setActivePath(true);
                control->mem_do_write_ctrl.setActivePath(true);
                control->do_reg_dst.setActivePath(true);

                switch(decode->opcode.uValue()){
                    case MIPS_Instr::ADD: case MIPS_Instr::ADDU: case MIPS_Instr:: AND:
                    case MIPS_Instr::NOR: case MIPS_Instr::OR: case MIPS_Instr:: SLLV:
                    case MIPS_Instr::SLT: case MIPS_Instr::SLTU: case MIPS_Instr:: SRAV:
                    case MIPS_Instr::SRLV: case MIPS_Instr::SUB: case MIPS_Instr:: SUBU:
                    case MIPS_Instr::XOR: case MIPS_Instr::MFHI: case MIPS_Instr::MFLO:{
                        decode->decode->instr_rs_25_21.setActivePath(true);
                        decode->decode->instr_rt_20_16.setActivePath(true);
                        decode->decode->instr_rd_15_11.setActivePath(true);
                        decode->decode->instr_imm_5_0.setActivePath(true);
                        registerFile->r1_out.setActivePath(true);
                        registerFile->r2_out.setActivePath(true);
                        break;
                    }

                    case MIPS_Instr::DIV: case MIPS_Instr::DIVU: case MIPS_Instr::MULT:
                    case MIPS_Instr::MULTU:{
                        decode->decode->instr_rs_25_21.setActivePath(true);
                        decode->decode->instr_rt_20_16.setActivePath(true);
                        decode->decode->instr_imm_5_0.setActivePath(true);
                        registerFile->r1_out.setActivePath(true);
                        registerFile->r2_out.setActivePath(true);
                        break;
                    }

                    case MIPS_Instr::MTHI: case MIPS_Instr::MTLO: {
                        decode->decode->instr_rs_25_21.setActivePath(true);
                        decode->decode->instr_rt_20_16.setActivePath(true);
                        decode->decode->instr_imm_5_0.setActivePath(true);
                        registerFile->r1_out.setActivePath(true);
                        registerFile->r2_out.setActivePath(true);

                        break;
                    }

                    case MIPS_Instr::LB: case MIPS_Instr::LBU: case MIPS_Instr::LH:
                    case MIPS_Instr::LHU: case MIPS_Instr::LW: case MIPS_Instr::LUI:{
                        immediate->setCompActivePath(true);

                        decode->decode->instr_rs_25_21.setActivePath(true);
                        decode->decode->instr_rt_20_16.setActivePath(true);
                        decode->decode->instr_rd_15_11.setActivePath(true);
                        decode->decode->instr_imm_15_0.setActivePath(true);
                        decode->decode->instr_imm_5_0.setActivePath(true);
                        immediate->imm.setActivePath(true);
                        registerFile->r1_out.setActivePath(true);
                        registerFile->r2_out.setActivePath(true);
                        break;
                    }
                    case MIPS_Instr::SB: case MIPS_Instr::SH: case MIPS_Instr::SW:{
                        immediate->setCompActivePath(true);

                        decode->decode->instr_rs_25_21.setActivePath(true);
                        decode->decode->instr_rt_20_16.setActivePath(true);
                        decode->decode->instr_imm_15_0.setActivePath(true);
                        decode->decode->instr_imm_5_0.setActivePath(true);
                        immediate->imm.setActivePath(true);
                        registerFile->r1_out.setActivePath(true);
                        registerFile->r2_out.setActivePath(true);
                        break;
                    }

                    case MIPS_Instr::ADDI: case MIPS_Instr::ADDIU: case MIPS_Instr:: ANDI:
                    case MIPS_Instr::ORI: case MIPS_Instr::SLTI: case MIPS_Instr::SLTIU:
                    case MIPS_Instr::XORI:{
                        immediate->setCompActivePath(true);

                        decode->decode->instr_rs_25_21.setActivePath(true);
                        decode->decode->instr_rt_20_16.setActivePath(true);
                        decode->decode->instr_rd_15_11.setActivePath(true);
                        decode->decode->instr_imm_15_0.setActivePath(true);
                        decode->decode->instr_imm_5_0.setActivePath(true);
                        immediate->imm.setActivePath(true);
                        registerFile->r1_out.setActivePath(true);
                        break;
                    }

                    case MIPS_Instr::BEQ: case MIPS_Instr::BNE: {
                        immediate->setCompActivePath(true);

                        ifid_reg->pc4_reg->out.setActivePath(true);
                        decode->decode->instr_rs_25_21.setActivePath(true);
                        decode->decode->instr_rt_20_16.setActivePath(true);
                        decode->decode->instr_imm_15_0.setActivePath(true);
                        decode->decode->instr_imm_5_0.setActivePath(true);
                        immediate->imm.setActivePath(true);
                        registerFile->r1_out.setActivePath(true);
                        registerFile->r2_out.setActivePath(true);
                        break;

                    }

                    case MIPS_Instr::BGEZ: case MIPS_Instr::BGTZ:
                    case MIPS_Instr::BLEZ: case MIPS_Instr::BLTZ:{
                        immediate->setCompActivePath(true);

                        ifid_reg->pc4_reg->out.setActivePath(true);
                        decode->decode->instr_rs_25_21.setActivePath(true);
                        decode->decode->instr_imm_15_0.setActivePath(true);
                        decode->decode->instr_imm_5_0.setActivePath(true);
                        immediate->imm.setActivePath(true);
                        registerFile->r1_out.setActivePath(true);
                        break;
                    }

                    case MIPS_Instr::JAL: case MIPS_Instr::J: {

                        ifid_reg->pc_reg->out.setActivePath(true);
                        decode->decode->instr_address_25_0.setActivePath(true);
                        break;
                    }

                    case MIPS_Instr::JR: {
                        ifid_reg->pc_reg->out.setActivePath(true);
                        decode->decode->instr_address_25_0.setActivePath(true);
                        decode->decode->instr_rs_25_21.setActivePath(true);
                        registerFile->r1_out.setActivePath(true);

                        break;
                    }

                    case MIPS_Instr::JALR: {
                        ifid_reg->pc_reg->out.setActivePath(true);
                        decode->decode->instr_rs_25_21.setActivePath(true);
                        decode->decode->instr_address_25_0.setActivePath(true);
                        decode->decode->instr_rt_20_16.setActivePath(true);
                        decode->decode->instr_rd_15_11.setActivePath(true);
                        registerFile->r1_out.setActivePath(true);

                        break;
                    }
                }
            }
        }



        /// EX
        else if(stage.index() == 2){
            idex_reg->setCompActivePath(false);
            shift_branch->setCompActivePath(false);
            alu_branch->setCompActivePath(false);
            alu_op2_src->setCompActivePath(false);
            alu->setCompActivePath(false);
            alu_control->setCompActivePath(false);
            write_reg->setCompActivePath(false);

            idex_reg->pc4_reg->out.setActivePath(false);
            idex_reg->r1_reg->out.setActivePath(false);
            idex_reg->r2_reg->out.setActivePath(false);
            idex_reg->instr_20_16_reg->out.setActivePath(false);
            idex_reg->instr_15_11_reg->out.setActivePath(false);
            idex_reg->instr_5_0_reg->out.setActivePath(false);
            idex_reg->imm_reg->out.setActivePath(false);
            write_reg->out.setActivePath(false);
            shift_branch->out.setActivePath(false);
            alu_branch->out.setActivePath(false);
            alu_op2_src->out.setActivePath(false);
            alu_control->res.setActivePath(false);
            alu->res.setActivePath(false);
            alu->zero.setActivePath(false);
            alu->greater.setActivePath(false);
            alu->less.setActivePath(false);
            pc_src->out.setActivePath(false);    //hidden




            idex_reg->reg_do_write_reg->out.setActivePath(false);
            idex_reg->do_br_reg->out.setActivePath(false);
            idex_reg->do_reg_dst_reg->out.setActivePath(false);
            idex_reg->alu_ctrl_reg->out.setActivePath(false);
            idex_reg->alu_op2_ctrl_reg->out.setActivePath(false);


            if(stageValid){
                idex_reg->setCompActivePath(true);


                //control signals
                idex_reg->reg_do_write_reg->out.setActivePath(true);
                idex_reg->do_br_reg->out.setActivePath(true);
                idex_reg->do_reg_dst_reg->out.setActivePath(true);
                idex_reg->alu_ctrl_reg->out.setActivePath(true);
                idex_reg->alu_op2_ctrl_reg->out.setActivePath(true);

                switch(idex_reg->opcode_out.uValue()){
                    case MIPS_Instr::ADD: case MIPS_Instr::ADDU: case MIPS_Instr:: AND:
                    case MIPS_Instr::NOR: case MIPS_Instr::OR: case MIPS_Instr:: SLLV:
                    case MIPS_Instr::SLT: case MIPS_Instr::SLTU: case MIPS_Instr:: SRAV:
                    case MIPS_Instr::SRLV: case MIPS_Instr::SUB: case MIPS_Instr:: SUBU:
                    case MIPS_Instr::XOR: case MIPS_Instr::MFHI: case MIPS_Instr::MFLO:{
                        alu_op2_src->setCompActivePath(true);
                        alu->setCompActivePath(true);
                        alu_control->setCompActivePath(true);
                        write_reg->setCompActivePath(true);

                        idex_reg->r1_reg->out.setActivePath(true);
                        idex_reg->r2_reg->out.setActivePath(true);
                        idex_reg->instr_15_11_reg->out.setActivePath(true);
                        idex_reg->instr_5_0_reg->out.setActivePath(true);
                        write_reg->out.setActivePath(true);
                        alu_op2_src->out.setActivePath(true);
                        alu_control->res.setActivePath(true);
                        alu->res.setActivePath(true);

                        break;
                    }

                    case MIPS_Instr::DIV: case MIPS_Instr::DIVU: case MIPS_Instr::MULT:
                    case MIPS_Instr::MULTU:{
                        alu_op2_src->setCompActivePath(true);
                        alu->setCompActivePath(true);
                        alu_control->setCompActivePath(true);

                        idex_reg->r1_reg->out.setActivePath(true);
                        idex_reg->r2_reg->out.setActivePath(true);
                        idex_reg->instr_5_0_reg->out.setActivePath(true);
                        alu_op2_src->out.setActivePath(true);
                        alu_control->res.setActivePath(true);
                        alu->res.setActivePath(true);
                        break;
                    }

                    case MIPS_Instr::MTHI: case MIPS_Instr::MTLO: {
                        alu_op2_src->setCompActivePath(true);
                        alu->setCompActivePath(true);
                        alu_control->setCompActivePath(true);

                        idex_reg->r1_reg->out.setActivePath(true);
                        idex_reg->r2_reg->out.setActivePath(true);
                        idex_reg->instr_5_0_reg->out.setActivePath(true);
                        alu_op2_src->out.setActivePath(true);
                        alu_control->res.setActivePath(true);
                        alu->res.setActivePath(true);
                        break;
                    }

                    case MIPS_Instr::LB: case MIPS_Instr::LBU: case MIPS_Instr::LH:
                    case MIPS_Instr::LHU: case MIPS_Instr::LW: case MIPS_Instr::LUI:{
                        alu_op2_src->setCompActivePath(true);
                        alu->setCompActivePath(true);
                        alu_control->setCompActivePath(true);
                        write_reg->setCompActivePath(true);

                        idex_reg->r1_reg->out.setActivePath(true);
                        idex_reg->r2_reg->out.setActivePath(true);
                        idex_reg->imm_reg->out.setActivePath(true);
                        idex_reg->instr_20_16_reg->out.setActivePath(true);
                        idex_reg->instr_5_0_reg->out.setActivePath(true);
                        write_reg->out.setActivePath(true);
                        alu_op2_src->out.setActivePath(true);
                        alu_control->res.setActivePath(true);
                        alu->res.setActivePath(true);
                        break;
                    }
                    case MIPS_Instr::SB: case MIPS_Instr::SH: case MIPS_Instr::SW:{
                        alu_op2_src->setCompActivePath(true);
                        alu->setCompActivePath(true);
                        alu_control->setCompActivePath(true);

                        idex_reg->r1_reg->out.setActivePath(true);
                        idex_reg->r2_reg->out.setActivePath(true);
                        idex_reg->imm_reg->out.setActivePath(true);
                        idex_reg->instr_5_0_reg->out.setActivePath(true);
                        alu_op2_src->out.setActivePath(true);
                        alu_control->res.setActivePath(true);
                        alu->res.setActivePath(true);
                        break;
                    }

                    case MIPS_Instr::ADDI: case MIPS_Instr::ADDIU: case MIPS_Instr:: ANDI:
                    case MIPS_Instr::ORI: case MIPS_Instr::SLTI: case MIPS_Instr::SLTIU:
                    case MIPS_Instr::XORI:{
                        alu_op2_src->setCompActivePath(true);
                        alu->setCompActivePath(true);
                        alu_control->setCompActivePath(true);
                        write_reg->setCompActivePath(true);

                        idex_reg->r1_reg->out.setActivePath(true);
                        idex_reg->imm_reg->out.setActivePath(true);
                        idex_reg->instr_20_16_reg->out.setActivePath(true);
                        idex_reg->instr_5_0_reg->out.setActivePath(true);
                        write_reg->out.setActivePath(true);
                        alu_op2_src->out.setActivePath(true);
                        alu_control->res.setActivePath(true);
                        alu->res.setActivePath(true);
                        break;
                    }

                    case MIPS_Instr::BEQ: case MIPS_Instr::BNE: {
                        alu_op2_src->setCompActivePath(true);
                        alu->setCompActivePath(true);
                        alu_control->setCompActivePath(true);
                        shift_branch->setCompActivePath(true);
                        alu_branch->setCompActivePath(true);

                        idex_reg->pc4_reg->out.setActivePath(true);
                        idex_reg->r1_reg->out.setActivePath(true);
                        idex_reg->r2_reg->out.setActivePath(true);
                        idex_reg->imm_reg->out.setActivePath(true);
                        idex_reg->instr_5_0_reg->out.setActivePath(true);
                        shift_branch->out.setActivePath(true);
                        alu_branch->out.setActivePath(true);
                        alu_op2_src->out.setActivePath(true);
                        alu_control->res.setActivePath(true);
                        alu->res.setActivePath(true);
                        alu->zero.setActivePath(true);
                        pc_src->out.setActivePath(true);    //hidden

                        break;

                    }

                    case MIPS_Instr::BGEZ: case MIPS_Instr::BGTZ:
                    case MIPS_Instr::BLEZ: case MIPS_Instr::BLTZ:{
                        alu_op2_src->setCompActivePath(true);
                        alu->setCompActivePath(true);
                        alu_control->setCompActivePath(true);
                        shift_branch->setCompActivePath(true);
                        alu_branch->setCompActivePath(true);

                        idex_reg->pc4_reg->out.setActivePath(true);
                        idex_reg->r1_reg->out.setActivePath(true);
                        idex_reg->imm_reg->out.setActivePath(true);
                        idex_reg->instr_5_0_reg->out.setActivePath(true);
                        shift_branch->out.setActivePath(true);
                        alu_branch->out.setActivePath(true);
                        alu_op2_src->out.setActivePath(true);
                        alu_control->res.setActivePath(true);
                        alu->res.setActivePath(true);
                        pc_src->out.setActivePath(true);    //hidden
                        switch(idex_reg->opcode_reg->out.uValue()){
                            case MIPS_Instr::BGEZ:{
                                alu->greater.setActivePath(true);
                                break;
                            }
                            case MIPS_Instr::BGTZ:{
                                alu->greater.setActivePath(true);
                                alu->zero.setActivePath(false);
                                break;
                            }
                            case MIPS_Instr::BLEZ:{
                                alu->less.setActivePath(true);
                                break;
                            }
                            case MIPS_Instr::BLTZ:{
                                alu->less.setActivePath(true);
                                alu->zero.setActivePath(false);
                                break;
                            }
                        }
                        break;
                    }

                    case MIPS_Instr::JAL: case MIPS_Instr::J: {
                        shift_branch->setCompActivePath(true);
                        alu_branch->setCompActivePath(true);

                        idex_reg->pc4_reg->out.setActivePath(true);
                        idex_reg->imm_reg->out.setActivePath(true);
                        shift_branch->out.setActivePath(true);
                        alu_branch->out.setActivePath(true);
                        pc_src->out.setActivePath(true);    //hidden

                        idex_reg->do_reg_dst_reg->out.setActivePath(false);
                        idex_reg->alu_ctrl_reg->out.setActivePath(false);
                        idex_reg->alu_op2_ctrl_reg->out.setActivePath(false);
                        break;
                    }

                    case MIPS_Instr::JR: {
                        shift_branch->setCompActivePath(true);
                        alu_branch->setCompActivePath(true);

                        idex_reg->pc4_reg->out.setActivePath(true);
                        alu_branch->out.setActivePath(true);
                        pc_src->out.setActivePath(true);    //hidden

                        idex_reg->do_reg_dst_reg->out.setActivePath(false);
                        idex_reg->alu_ctrl_reg->out.setActivePath(false);
                        idex_reg->alu_op2_ctrl_reg->out.setActivePath(false);
                        break;
                    }

                    case MIPS_Instr::JALR: {
                        alu_branch->setCompActivePath(true);
                        write_reg->setCompActivePath(true);

                        idex_reg->pc4_reg->out.setActivePath(true);
                        idex_reg->instr_15_11_reg->out.setActivePath(true);
                        write_reg->out.setActivePath(true);
                        alu_branch->out.setActivePath(true);
                        pc_src->out.setActivePath(true);    //hidden

                        idex_reg->alu_ctrl_reg->out.setActivePath(false);
                        idex_reg->alu_op2_ctrl_reg->out.setActivePath(false);
                        break;
                    }
                }

            }

        }


        /// MEM
        else if(stage.index() == 3){
            exmem_reg->setCompActivePath(false);
            data_mem->setCompActivePath(false);
            bn_not->setCompActivePath(false);
            br_and->setCompActivePath(false);
            bn_and->setCompActivePath(false);
            bg_and->setCompActivePath(false);
            bl_and->setCompActivePath(false);

            exmem_reg->alures_reg->out.setActivePath(false);
            exmem_reg->r2_reg->out.setActivePath(false);
            exmem_reg->aluzero_reg->out.setActivePath(false);
            exmem_reg->alugreater_reg->out.setActivePath(false);
            exmem_reg->aluless_reg->out.setActivePath(false);
            exmem_reg->pc_branch_reg->out.setActivePath(false);


            bn_not->out.setActivePath(false);
            br_and->out.setActivePath(false);
            bn_and->out.setActivePath(false);
            bg_and->out.setActivePath(false);
            bl_and->out.setActivePath(false);

            exmem_reg->write_reg_reg->out.setActivePath(false);
            exmem_reg->reg_do_write_reg->out.setActivePath(false);
            exmem_reg->mem_do_write_ctrl_reg->out.setActivePath(false);
            exmem_reg->mem_do_read_ctrl_reg->out.setActivePath(false);
            exmem_reg->do_br_reg->out.setActivePath(false);
            exmem_reg->do_bn_reg->out.setActivePath(false);
            exmem_reg->do_bl_reg->out.setActivePath(false);
            exmem_reg->do_bg_reg->out.setActivePath(false);

            data_mem->data_out.setActivePath(false);
            flush_or->out.setActivePath(false);

            if(stageValid){
                exmem_reg->setCompActivePath(true);

                exmem_reg->reg_do_write_reg->out.setActivePath(true);

                switch(exmem_reg->opcode_reg->out.uValue()){
                case MIPS_Instr::ADD: case MIPS_Instr::ADDU: case MIPS_Instr:: AND:
                case MIPS_Instr::NOR: case MIPS_Instr::OR: case MIPS_Instr:: SLLV:
                case MIPS_Instr::SLT: case MIPS_Instr::SLTU: case MIPS_Instr:: SRAV:
                case MIPS_Instr::SRLV: case MIPS_Instr::SUB: case MIPS_Instr:: SUBU:
                case MIPS_Instr::XOR: case MIPS_Instr::MFHI: case MIPS_Instr::MFLO:
                case MIPS_Instr::ADDI: case MIPS_Instr::ADDIU: case MIPS_Instr:: ANDI:
                case MIPS_Instr::ORI: case MIPS_Instr::SLTI: case MIPS_Instr::SLTIU:
                case MIPS_Instr::XORI:{
                    exmem_reg->alures_reg->out.setActivePath(true);
                    exmem_reg->write_reg_reg->out.setActivePath(true);


                    break;
                }

                case MIPS_Instr::DIV: case MIPS_Instr::DIVU: case MIPS_Instr::MULT:
                case MIPS_Instr::MULTU: case MIPS_Instr::MTHI: case MIPS_Instr::MTLO:{
                    exmem_reg->alures_reg->out.setActivePath(true);

                    break;
                }


                case MIPS_Instr::LB: case MIPS_Instr::LBU: case MIPS_Instr::LH:
                case MIPS_Instr::LHU: case MIPS_Instr::LW: case MIPS_Instr::LUI:{
                    exmem_reg->alures_reg->out.setActivePath(true);
                    exmem_reg->r2_reg->out.setActivePath(true);
                    exmem_reg->write_reg_reg->out.setActivePath(true);
                    data_mem->data_out.setActivePath(true);

                    exmem_reg->mem_do_read_ctrl_reg->out.setActivePath(true);
                    data_mem->setCompActivePath(true);

                    break;
                }

                case MIPS_Instr::SB:case MIPS_Instr::SH: case MIPS_Instr::SW:{
                    exmem_reg->alures_reg->out.setActivePath(true);
                    exmem_reg->r2_reg->out.setActivePath(true);

                    exmem_reg->mem_do_write_ctrl_reg->out.setActivePath(true);
                    data_mem->setCompActivePath(true);

                    break;
                }


                case MIPS_Instr::BEQ: case MIPS_Instr::BNE: {
                    exmem_reg->aluzero_reg->out.setActivePath(true);
                    exmem_reg->pc_branch_reg->out.setActivePath(true);
                    //flush_or->setCompActivatePath(true);


                    switch(exmem_reg->opcode_reg->out.uValue()){
                        case MIPS_Instr::BEQ:{
                            br_and->setCompActivePath(true);

                            exmem_reg->do_br_reg->out.setActivePath(true);
                            br_and->out.setActivePath(true);

                            break;
                        }
                        case MIPS_Instr::BNE:{
                            bn_not->setCompActivePath(true);
                            bn_and->setCompActivePath(true);

                            bn_not->out.setActivePath(true);
                            exmem_reg->do_bn_reg->out.setActivePath(true);
                            bn_and->out.setActivePath(true);

                            break;
                        }
                    }

                    break;

                }

                case MIPS_Instr::BGEZ: case MIPS_Instr::BGTZ: case MIPS_Instr::BLEZ:
                case MIPS_Instr::BLTZ: case MIPS_Instr::JAL: case MIPS_Instr::J:
                case MIPS_Instr::JR: case MIPS_Instr::JALR:{

                    exmem_reg->pc_branch_reg->out.setActivePath(true);
                    flush_or->out.setActivePath(true);



                    switch(exmem_reg->opcode_reg->out.uValue()){
                        case MIPS_Instr::BGEZ:{

                            br_and->setCompActivePath(true);
                            bg_and->setCompActivePath(true);

                            exmem_reg->aluzero_reg->out.setActivePath(true);
                            exmem_reg->alugreater_reg->out.setActivePath(true);

                            exmem_reg->do_br_reg->out.setActivePath(true);
                            exmem_reg->do_bg_reg->out.setActivePath(true);

                            br_and->out.setActivePath(true);
                            bg_and->out.setActivePath(true);

                            break;
                        }
                        case MIPS_Instr::BGTZ:{

                            bg_and->setCompActivePath(true);

                            exmem_reg->alugreater_reg->out.setActivePath(true);

                            exmem_reg->do_bg_reg->out.setActivePath(true);

                            bg_and->out.setActivePath(true);

                            break;
                        }
                        case MIPS_Instr::BLEZ:{

                            br_and->setCompActivePath(true);
                            bl_and->setCompActivePath(true);

                            exmem_reg->aluzero_reg->out.setActivePath(true);
                            exmem_reg->aluless_reg->out.setActivePath(true);

                            exmem_reg->do_br_reg->out.setActivePath(true);
                            exmem_reg->do_bl_reg->out.setActivePath(true);

                            br_and->out.setActivePath(true);
                            bl_and->out.setActivePath(true);

                            break;
                        }
                        case MIPS_Instr::BLTZ:{
                            bl_and->setCompActivePath(true);

                            exmem_reg->aluless_reg->out.setActivePath(true);

                            exmem_reg->do_bl_reg->out.setActivePath(true);

                            bl_and->out.setActivePath(true);

                            break;
                        }
                        case MIPS_Instr::JALR:{
                            exmem_reg->write_reg_reg->out.setActivePath(true);

                            break;
                        }
                        default:{
                            flush_or->out.setActivePath(false);
                            //flush_or->setCompActivatePath(false);

                            break;
                        }
                    }
                    break;
                }
                default:{
                    exmem_reg->reg_do_write_reg->out.setActivePath(false);
                    exmem_reg->mem_do_read_ctrl_reg->out.setActivePath(false);
                    exmem_reg->mem_do_write_ctrl_reg->out.setActivePath(false);

                    break;
                }

            }
          }

        }


        /// WB
        else {
            memwb_reg->setCompActivePath(false);
            reg_wr_src->setCompActivePath(false);

            memwb_reg->reg_wr_src_ctrl_reg->out.setActivePath(false);
            memwb_reg->alures_reg->out.setActivePath(false);
            memwb_reg->write_reg_reg->out.setActivePath(false);
            memwb_reg->mem_read_reg->out.setActivePath(false);
            memwb_reg->reg_do_write_reg->out.setActivePath(false);

            reg_wr_src->out.setActivePath(false);


            if(stageValid){
                memwb_reg->setCompActivePath(true);
                reg_wr_src->setCompActivePath(true);


                memwb_reg->reg_do_write_reg->out.setActivePath(true);
                memwb_reg->reg_wr_src_ctrl_reg->out.setActivePath(true);
                reg_wr_src->out.setActivePath(true);

                switch(memwb_reg->opcode_reg->out.uValue()){
                    case MIPS_Instr::ADD: case MIPS_Instr::ADDU: case MIPS_Instr:: AND:
                    case MIPS_Instr::NOR: case MIPS_Instr::OR: case MIPS_Instr:: SLLV:
                    case MIPS_Instr::SLT: case MIPS_Instr::SLTU: case MIPS_Instr:: SRAV:
                    case MIPS_Instr::SRLV: case MIPS_Instr::SUB: case MIPS_Instr:: SUBU:
                    case MIPS_Instr::XOR: case MIPS_Instr::MFHI: case MIPS_Instr::MFLO:
                    case MIPS_Instr::ADDI: case MIPS_Instr::ADDIU: case MIPS_Instr:: ANDI:
                    case MIPS_Instr::ORI: case MIPS_Instr::SLTI: case MIPS_Instr::SLTIU:
                    case MIPS_Instr::XORI:{
                        memwb_reg->alures_reg->out.setActivePath(true);
                        memwb_reg->write_reg_reg->out.setActivePath(true);
                        registerFile->setCompActivePath(true);


                        break;
                    }

                    case MIPS_Instr::DIV: case MIPS_Instr::DIVU: case MIPS_Instr::MULT:
                    case MIPS_Instr::MULTU: case MIPS_Instr::MTHI: case MIPS_Instr::MTLO:{
                        memwb_reg->alures_reg->out.setActivePath(true);
                        registerFile->setCompActivePath(true);

                        break;
                    }


                    case MIPS_Instr::LB: case MIPS_Instr::LBU: case MIPS_Instr::LH:
                    case MIPS_Instr::LHU: case MIPS_Instr::LW: case MIPS_Instr::LUI:{
                        memwb_reg->write_reg_reg->out.setActivePath(true);
                        memwb_reg->mem_read_reg->out.setActivePath(true);
                        registerFile->setCompActivePath(true);

                        break;
                    }

                    case MIPS_Instr::JALR: {
                        memwb_reg->write_reg_reg->out.setActivePath(true);
                        reg_wr_src->out.setActivePath(false);
                        registerFile->setCompActivePath(true);

                        break;
                    }

                    default:{

                        memwb_reg->setCompActivePath(false);
                        reg_wr_src->setCompActivePath(false);


                        memwb_reg->reg_do_write_reg->out.setActivePath(false);
                        memwb_reg->reg_wr_src_ctrl_reg->out.setActivePath(false);
                        reg_wr_src->out.setActivePath(false);

                        break;
                    }
            }

        }
       }





    // clang-format on

    // Gather stage state info
    StageInfo::State state = StageInfo ::State::None;
    switch (stage.index()) {
    case IF:
      break;
    case ID:
      if (m_cycleCount > ID && ifid_reg->valid_out.uValue() == 0) {
        state = StageInfo::State::Flushed;
      }
      break;
    case EX: {
      if (m_cycleCount > EX && idex_reg->valid_out.uValue() == 0) {
        state = StageInfo::State::Flushed;
      }
      break;
    }
    case MEM: {
      if (m_cycleCount > MEM && exmem_reg->valid_out.uValue() == 0) {
        state = StageInfo::State::Flushed;
      }
      break;
    }
    case WB: {
      if (m_cycleCount > WB && memwb_reg->valid_out.uValue() == 0) {
        state = StageInfo::State::Flushed;
      }
      break;
    }
    }

    return StageInfo({getPcForStage(stage), stageValid, state});
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
    if ((fr & FinalizeReason::exitSyscall) &&
        !ecallChecker->isSysCallExiting()) {
      // An exit system call was executed. Record the cycle of the execution,
      // and enable the ecallChecker's system call exiting signal.
      m_syscallExitCycle = m_cycleCount;
    }
    ecallChecker->setSysCallExiting(ecallChecker->isSysCallExiting() ||
                                    (fr & FinalizeReason::exitSyscall));
  }
  const std::vector<StageIndex> breakpointTriggeringStages() const override {
    return {{0, IF}};
  };

  MemoryAccess dataMemAccess() const override {
    return memToAccessInfo(data_mem);
  }
  MemoryAccess instrMemAccess() const override {
    auto instrAccess = memToAccessInfo(instr_mem);
    instrAccess.type = MemoryAccess::Read;
    return instrAccess;
  }

  bool finished() const override {
    // The processor is finished when there are no more valid instructions in
    // the pipeline
    bool allStagesInvalid = true;
    for (int stage = IF; stage < STAGECOUNT; stage++) {
      allStagesInvalid &= !stageInfo({0, stage}).stage_valid;
      if (!allStagesInvalid)
        break;
    }
    return allStagesInvalid;
  }
  void setRegister(RegisterFileType, unsigned i, VInt v) override {
    setSynchronousValue(registerFile->_wr_mem, i, v);
  }

  void clockProcessor() override {
    // An instruction has been retired if the instruction in the WB stage is
    // valid and the PC is within the executable range of the program
    if (memwb_reg->valid_out.uValue() != 0 &&
        isExecutableAddress(memwb_reg->pc_out.uValue())) {
      m_instructionsRetired++;
    }

    Design::clock();
  }

  void reverse() override {
    if (m_syscallExitCycle != -1 && m_cycleCount == m_syscallExitCycle) {
      // We are about to undo an exit syscall instruction. In this case, the
      // syscall exiting sequence should be terminate
      ecallChecker->setSysCallExiting(false);
      m_syscallExitCycle = -1;
    }
    Design::reverse();
    if (memwb_reg->valid_out.uValue() != 0 &&
        isExecutableAddress(memwb_reg->pc_out.uValue())) {
      m_instructionsRetired--;
    }
  }

  void reset() override {
    Design::reset();
    ecallChecker->setSysCallExiting(false);
    m_syscallExitCycle = -1;
  }

  static ProcessorISAInfo supportsISA() {
    return ProcessorISAInfo{
        std::make_shared<ISAInfo<XLenToMIPSISA<XLEN>()>>(QStringList()),
        {"M", "C"},
        {"M"}};
  }
  const ISAInfoBase *implementsISA() const override {
    return m_enabledISA.get();
  };

  const std::set<RegisterFileType> registerFiles() const override {
    std::set<RegisterFileType> rfs;
    rfs.insert(RegisterFileType::GPR);

    if (implementsISA()->extensionEnabled("F")) {
      rfs.insert(RegisterFileType::FPR);
    }
    return rfs;
  }

private:
  /**
   * @brief m_syscallExitCycle
   * The variable will contain the cycle of which an exit system call was
   * executed. From this, we may determine when we roll back an exit system call
   * during rewinding.
   */
  long long m_syscallExitCycle = -1;
  std::shared_ptr<ISAInfoBase> m_enabledISA;
  ProcessorStructure m_structure = {{0, 5}};
};
} // namespace core
} // namespace vsrtl
