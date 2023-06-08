#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "VSRTL/core/vsrtl_register.h"

#include "../mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class MIPS_IDEX_UNOPT : public Component {
public:
  MIPS_IDEX_UNOPT(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    setDescription("Instruction decode/execute stage separating register");
    CONNECT_REGISTERED_CLEN_INPUT(pc4, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(pc, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(r1, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(r2, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(imm, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(j_address, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(instr_20_16, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(instr_25_0, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(instr_15_11, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(instr_shamt_10_6, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(instr_5_0, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(opcode, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(pc_4, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(j_sel, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mf_sel, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mt_sel, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mult_div_sel, clear, enable);


    // EX
    CONNECT_REGISTERED_CLEN_INPUT(alu_op2_ctrl, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(alu_ctrl, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(do_reg_dst, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(do_jmp, clear, enable);

    // MEM
    CONNECT_REGISTERED_CLEN_INPUT(do_br, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(do_bn, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(do_bg, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(do_bl, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mem_do_write_ctrl, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mem_ctrl, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mem_do_read_ctrl, clear, enable);

    // WB
    CONNECT_REGISTERED_CLEN_INPUT(reg_do_write, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(reg_wr_src_ctrl, clear, enable);


    CONNECT_REGISTERED_CLEN_INPUT(valid, clear, enable);
  }

  // Data
  REGISTERED_CLEN_INPUT(pc, XLEN);
  REGISTERED_CLEN_INPUT(pc4, XLEN);
  REGISTERED_CLEN_INPUT(r1, XLEN);
  REGISTERED_CLEN_INPUT(r2, XLEN);
  REGISTERED_CLEN_INPUT(imm, XLEN);
  REGISTERED_CLEN_INPUT(j_address, XLEN);
  REGISTERED_CLEN_INPUT(instr_25_0, 26);
  REGISTERED_CLEN_INPUT(instr_20_16, c_MIPSRegsBits);
  REGISTERED_CLEN_INPUT(instr_15_11, c_MIPSRegsBits);
  REGISTERED_CLEN_INPUT(instr_shamt_10_6, c_MIPSRegsBits);
  REGISTERED_CLEN_INPUT(instr_5_0, c_MIPSRegsBits);
  REGISTERED_CLEN_INPUT(opcode, MIPS_Instr::width());
  REGISTERED_CLEN_INPUT(pc_4, XLEN);
  REGISTERED_CLEN_INPUT(j_sel,2);
  REGISTERED_CLEN_INPUT(mf_sel, 2);
  REGISTERED_CLEN_INPUT(mt_sel, 2);
  REGISTERED_CLEN_INPUT(mult_div_sel, 1);

  // Control


  // EX
  REGISTERED_CLEN_INPUT(alu_op2_ctrl, MIPS_AluSrc2::width());
  REGISTERED_CLEN_INPUT(alu_ctrl, MIPS_ALUOp::width());
  REGISTERED_CLEN_INPUT(do_reg_dst, 1);
  REGISTERED_CLEN_INPUT(do_jmp, 1);

  // MEM
  REGISTERED_CLEN_INPUT(do_br, 1);
  REGISTERED_CLEN_INPUT(do_bn, 1);
  REGISTERED_CLEN_INPUT(do_bg, 1);
  REGISTERED_CLEN_INPUT(do_bl, 1);
  REGISTERED_CLEN_INPUT(mem_do_write_ctrl, 1);
  REGISTERED_CLEN_INPUT(mem_ctrl, MIPS_MemOp::width());
  REGISTERED_CLEN_INPUT(mem_do_read_ctrl, 1);

  // WB
  REGISTERED_CLEN_INPUT(reg_wr_src_ctrl, MIPS_RegWrSrc::width());
  REGISTERED_CLEN_INPUT(reg_do_write, 1);


  // Register bank controls
  INPUTPORT(enable, 1);
  INPUTPORT(clear, 1);

  // Valid signal. False when the register bank has been cleared. May be used by
  // UI to determine whether the NOP in the stage is a user-inserted nop or the
  // result of some pipeline action.
  REGISTERED_CLEN_INPUT(valid, 1);
};

} // namespace core
} // namespace vsrtl
