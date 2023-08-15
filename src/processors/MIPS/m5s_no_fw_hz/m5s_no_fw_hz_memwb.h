#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "VSRTL/core/vsrtl_register.h"

#include "../mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class MIPS_MEMWB : public Component {
public:
  MIPS_MEMWB(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    setDescription("Memory/write-back stage separating register");
    CONNECT_REGISTERED_INPUT(pc);
    //CONNECT_REGISTERED_INPUT(pc4);
    CONNECT_REGISTERED_INPUT(alures);
    CONNECT_REGISTERED_INPUT(mem_read);
    CONNECT_REGISTERED_INPUT(write_reg);
    CONNECT_REGISTERED_INPUT(opcode);
    CONNECT_REGISTERED_INPUT(aluhi);
    CONNECT_REGISTERED_INPUT(pc_4);
    CONNECT_REGISTERED_INPUT(j_sel);
    CONNECT_REGISTERED_INPUT(mf_sel);
    CONNECT_REGISTERED_INPUT(mt_sel);
    CONNECT_REGISTERED_INPUT(mult_div_sel);

/*
    CONNECT_REGISTERED_INPUT(reg_wr_src_ctrl);
    CONNECT_REGISTERED_INPUT(wr_reg_idx);
    CONNECT_REGISTERED_INPUT(reg_do_write);
*/
    // WB
    CONNECT_REGISTERED_INPUT(reg_do_write);
    CONNECT_REGISTERED_INPUT(reg_wr_src_ctrl);

    CONNECT_REGISTERED_INPUT(valid);


  }

  // Data
  REGISTERED_INPUT(pc, XLEN);
  //REGISTERED_INPUT(pc4, XLEN);
  REGISTERED_INPUT(alures, XLEN);
  REGISTERED_INPUT(mem_read, XLEN);
  REGISTERED_INPUT(write_reg, c_MIPSRegsBits);
  REGISTERED_INPUT(opcode, MIPS_Instr::width());
  REGISTERED_INPUT(aluhi, XLEN);
  REGISTERED_INPUT(pc_4, XLEN);
  REGISTERED_INPUT(j_sel,2);
  REGISTERED_INPUT(mf_sel, 2);
  REGISTERED_INPUT(mt_sel, 2);
  REGISTERED_INPUT(mult_div_sel, 1);

  // Control
  /*
  REGISTERED_INPUT(reg_wr_src_ctrl, MIPS_RegWrSrc::width());
  REGISTERED_INPUT(wr_reg_idx, c_MIPSRegsBits);
  REGISTERED_INPUT(reg_do_write, 1);
*/

  // WB
  REGISTERED_INPUT(reg_wr_src_ctrl, MIPS_RegWrSrc::width());
  REGISTERED_INPUT(reg_do_write, 1);

  // Valid signal. False when the register bank has been cleared. May be used by
  // UI to determine whether the NOP in the stage is a user-inserted nop or the
  // result of some pipeline action.
  REGISTERED_INPUT(valid, 1);
};

} // namespace core
} // namespace vsrtl
