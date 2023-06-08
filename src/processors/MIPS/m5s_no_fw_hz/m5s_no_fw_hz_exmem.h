#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "VSRTL/core/vsrtl_register.h"

#include "../mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class MIPS_EXMEM : public Component {
public:
  MIPS_EXMEM(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    setDescription("Execute/memory stage separating register");
    CONNECT_REGISTERED_CLEN_INPUT(pc, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(pc_branch, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(alures, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(aluhi, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(aluzero, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(r2, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(write_reg, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(opcode, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(pc_4, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(j_sel, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mf_sel, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mt_sel, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mult_div_sel, clear, enable);


    // MEM
    CONNECT_REGISTERED_CLEN_INPUT(do_br, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mem_do_write_ctrl, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mem_ctrl, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mem_do_read_ctrl, clear, enable);

    // WB
    CONNECT_REGISTERED_CLEN_INPUT(reg_do_write, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(reg_wr_src_ctrl, clear, enable);

/*
    CONNECT_REGISTERED_CLEN_INPUT(reg_wr_src_ctrl, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(wr_reg_idx, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(reg_do_write, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mem_do_write, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mem_do_read, clear, enable);
    CONNECT_REGISTERED_CLEN_INPUT(mem_op, clear, enable);
*/
    CONNECT_REGISTERED_CLEN_INPUT(valid, clear, enable);
  }

  // Data
  REGISTERED_CLEN_INPUT(pc, XLEN);
  REGISTERED_CLEN_INPUT(pc_branch, XLEN);
  REGISTERED_CLEN_INPUT(alures, XLEN);
  REGISTERED_CLEN_INPUT(aluhi, XLEN);
  REGISTERED_CLEN_INPUT(aluzero, 1);
  REGISTERED_CLEN_INPUT(r2, XLEN);
  REGISTERED_CLEN_INPUT(write_reg, c_MIPSRegsBits);
  REGISTERED_CLEN_INPUT(opcode, MIPS_Instr::width());
  REGISTERED_CLEN_INPUT(pc_4, XLEN);
  REGISTERED_CLEN_INPUT(j_sel,2);
  REGISTERED_CLEN_INPUT(mf_sel, 2);
  REGISTERED_CLEN_INPUT(mt_sel, 2);
  REGISTERED_CLEN_INPUT(mult_div_sel, 1);



  // Control

  // MEM
  REGISTERED_CLEN_INPUT(do_br, 1);
  REGISTERED_CLEN_INPUT(mem_do_write_ctrl, 1);
  REGISTERED_CLEN_INPUT(mem_ctrl, MIPS_MemOp::width());
  REGISTERED_CLEN_INPUT(mem_do_read_ctrl, 1);

  // WB
  REGISTERED_CLEN_INPUT(reg_wr_src_ctrl, MIPS_RegWrSrc::width());
  REGISTERED_CLEN_INPUT(reg_do_write, 1);

  /*
  REGISTERED_CLEN_INPUT(reg_wr_src_ctrl, MIPS_RegWrSrc::width());
  REGISTERED_CLEN_INPUT(wr_reg_idx, c_MIPSRegsBits);
  REGISTERED_CLEN_INPUT(reg_do_write, 1);
  REGISTERED_CLEN_INPUT(mem_do_write, 1);
  REGISTERED_CLEN_INPUT(mem_do_read, 1);
  REGISTERED_CLEN_INPUT(mem_op, MIPS_MemOp::width());
*/
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
