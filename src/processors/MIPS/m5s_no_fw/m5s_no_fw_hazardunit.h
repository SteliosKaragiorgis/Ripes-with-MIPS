#pragma once

#include "../mips.h"

#include "VSRTL/core/vsrtl_component.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

class MIPS_HazardUnit_NO_FW : public Component {
public:
  MIPS_HazardUnit_NO_FW(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    hazardFEEnable << [=] { return !hasHazard(); };
    hazardIDEXEnable << [=] { return !hasEcallHazard(); };
    hazardEXMEMClear << [=] { return hasEcallHazard(); };
    hazardIDEXClear << [=] { return hasDataOrLoadUseHazard(); };
    stallEcallHandling << [=] { return hasEcallHazard(); };
  }

  INPUTPORT(id_reg1_idx, c_MIPSRegsBits);
  INPUTPORT(id_reg2_idx, c_MIPSRegsBits);
  INPUTPORT(id_do_branch, 1);
  INPUTPORT(id_mem_do_write, 1);
  INPUTPORT_ENUM(id_alu_op_ctrl_2, MIPS_AluSrc2);
  INPUTPORT_ENUM(id_opcode, MIPS_Instr);


  INPUTPORT(ex_reg_wr_idx, c_MIPSRegsBits);
  INPUTPORT(ex_do_mem_read_en, 1);
  INPUTPORT(ex_do_reg_write, 1);
  INPUTPORT(ex_do_jump, 1);
  INPUTPORT(ex_branch_taken, 1);
  INPUTPORT_ENUM(ex_opcode, MIPS_Instr);

  INPUTPORT(mem_reg_wr_idx, c_MIPSRegsBits);
  INPUTPORT(mem_do_reg_write, 1);
  INPUTPORT_ENUM(mem_opcode, MIPS_Instr);


  INPUTPORT(wb_do_reg_write, 1);

  // Hazard Front End enable: Low when stalling the front end (shall be
  // connected to a register 'enable' input port).
  OUTPUTPORT(hazardFEEnable, 1);

  // Hazard IDEX enable: Low when stalling due to an ECALL hazard
  OUTPUTPORT(hazardIDEXEnable, 1);

  // EXMEM clear: High when an ECALL hazard is detected
  OUTPUTPORT(hazardEXMEMClear, 1);
  // IDEX clear: High when a load-use hazard is detected
  OUTPUTPORT(hazardIDEXClear, 1);

  // Stall Ecall Handling: High whenever we are about to handle an ecall, but
  // have outstanding writes in the pipeline which must be comitted to the
  // register file before handling the ecall.
  OUTPUTPORT(stallEcallHandling, 1);

private:
  bool hasHazard() { return hasDataOrLoadUseHazard() || hasEcallHazard(); }

  bool hasDataOrLoadUseHazard() {
    return hasDataHazardExMem() || hasLoadUseHazard();
  }

  bool hasDataHazardExMem() { return hasDataHazardEx() || hasDataHazardMem(); }

  bool hasLoadUseHazard() const {

    unsigned exidx_temp = ex_reg_wr_idx.uValue();
    if (ex_opcode.uValue() == MIPS_Instr::MTHI) exidx_temp = 32;
    else if (ex_opcode.uValue() == MIPS_Instr::MTLO) exidx_temp = 33;

    unsigned idx1_temp = id_reg1_idx.uValue();
    if (id_opcode.uValue() == MIPS_Instr::MFHI) idx1_temp = 32;
    else if (id_opcode.uValue() == MIPS_Instr::MFLO) idx1_temp = 33;

    const unsigned exidx = exidx_temp;
    const unsigned idx1 = idx1_temp;
    const unsigned idx2 = id_reg2_idx.uValue();
    const bool mrd = ex_do_mem_read_en.uValue();

    return (exidx == idx1 || exidx == idx2) && mrd;
  }

  bool hasEcallHazard() const {
    // Check for ECALL hazard. We are implictly dependent on all registers when
    // performing an ECALL operation. As such, all outstanding writes to the
    // register file must be performed before handling the ecall. Hence, the
    // front-end of the pipeline shall be stalled until the remainder of the
    // pipeline has been cleared and there are no more outstanding writes.
    const bool isEcall = ex_opcode.uValue() == MIPS_Instr::SYSCALL;
    return isEcall && (mem_do_reg_write.uValue() || wb_do_reg_write.uValue());
  }

  // Check if data Hazard between instruction i+1 and i
  bool hasDataHazardEx() const {
    unsigned exidx_temp = ex_reg_wr_idx.uValue();
    if (ex_opcode.uValue() == MIPS_Instr::MTHI) exidx_temp = 32;
    else if (ex_opcode.uValue() == MIPS_Instr::MTLO) exidx_temp = 33;

    const unsigned exWriteIdx = exidx_temp;
    const bool exRegWrite = ex_do_reg_write.uValue();
    return hasDataHazard(exWriteIdx, exRegWrite);
  }

  // Check if data Hazard between instruction i+2 and i
  bool hasDataHazardMem() const {

    unsigned mem_temp = mem_reg_wr_idx.uValue();
    if (mem_opcode.uValue() == MIPS_Instr::MTHI) mem_temp = 32;
    else if (mem_opcode.uValue() == MIPS_Instr::MTLO) mem_temp = 33;

    const unsigned memWriteIdx = mem_temp;
    const bool memRegWrite = mem_do_reg_write.uValue();
    return hasDataHazard(memWriteIdx, memRegWrite);
  }

  bool hasDataHazard(unsigned writeIdx, bool regWrite) const {

    unsigned idx1_temp = id_reg1_idx.uValue();
    if (id_opcode.uValue() == MIPS_Instr::MFHI) idx1_temp = 32;
    else if (id_opcode.uValue() == MIPS_Instr::MFLO) idx1_temp = 33;

    const unsigned idx1 = idx1_temp;
    const unsigned idx2 = id_reg2_idx.uValue();
    const bool idx2isReg = id_alu_op_ctrl_2.uValue() == MIPS_AluSrc2::REG2;

    // Get branch information (when branch, idx2isReg must be ignored, because
    // it is IMM)
    const bool isBranch = id_do_branch.uValue();

    // Get mem write information (when write to memory, idx2isReg must be
    // ignored, because it is IMM)
    const bool isMemWrite = id_mem_do_write.uValue();

    // Get jump information (when jump, no stall needed because instruction will
    // flushed)
    const bool doJump = ex_do_jump.uValue();

    // Check if branch is taken do no stall
    const bool exBranchTaken = ex_branch_taken.uValue();

    // If destination is x0 or jump is taken ignore hazard
    if ((writeIdx == 0) || (doJump)) {
      return false;
    }

    return ((writeIdx == idx1) ||
            ((writeIdx == idx2) && (idx2isReg || isBranch || isMemWrite))) &&
           regWrite && !exBranchTaken;
  }
};
} // namespace core
} // namespace vsrtl
