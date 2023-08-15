#pragma once

#include "../mips.h"

#include "VSRTL/core/vsrtl_component.h"

namespace Ripes {
Enum(MIPS_ForwardingSrc, IdStage, MemStage, WbStage);
}

namespace vsrtl {
namespace core {
using namespace Ripes;

class MIPS_ForwardingUnit : public Component {
public:
  MIPS_ForwardingUnit(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    alu_reg1_forwarding_ctrl << [=] {
      const auto idx = id_reg1_idx.uValue();
      if (idx == 0) {
          if ( (opcode_ex.uValue() ==  MIPS_Instr::MFLO &&
               opcode_mem.uValue() == MIPS_Instr::MTLO)
               || (opcode_ex.uValue() ==  MIPS_Instr::MFHI &&
                   opcode_mem.uValue() == MIPS_Instr::MTHI)
               || (opcode_ex.uValue() ==  MIPS_Instr::MFLO &&
                   opcode_mem.uValue() == MIPS_Instr::MULT)
               || (opcode_ex.uValue() ==  MIPS_Instr::MFLO &&
                   opcode_mem.uValue() == MIPS_Instr::MULTU)
               || (opcode_ex.uValue() ==  MIPS_Instr::MFLO &&
                   opcode_mem.uValue() == MIPS_Instr::DIV)
               || (opcode_ex.uValue() ==  MIPS_Instr::MFLO &&
                   opcode_mem.uValue() == MIPS_Instr::DIVU)) {
              return MIPS_ForwardingSrc::MemStage;
          }
          else if ( (opcode_ex.uValue() ==  MIPS_Instr::MFLO &&
               opcode_wb.uValue() == MIPS_Instr::MTLO)
               || (opcode_ex.uValue() ==  MIPS_Instr::MFHI &&
                   opcode_wb.uValue() == MIPS_Instr::MTHI)
                    || (opcode_ex.uValue() ==  MIPS_Instr::MFLO &&
                        opcode_wb.uValue() == MIPS_Instr::MULT)
                    || (opcode_ex.uValue() ==  MIPS_Instr::MFLO &&
                        opcode_wb.uValue() == MIPS_Instr::MULTU)
                    || (opcode_ex.uValue() ==  MIPS_Instr::MFLO &&
                        opcode_wb.uValue() == MIPS_Instr::DIV)
                    || (opcode_ex.uValue() ==  MIPS_Instr::MFLO &&
                        opcode_wb.uValue() == MIPS_Instr::DIVU)) {
              return MIPS_ForwardingSrc::WbStage;
          }
        return MIPS_ForwardingSrc::IdStage;
      } else if (idx == mem_reg_wr_idx.uValue() && mem_reg_wr_en.uValue()) {
        return MIPS_ForwardingSrc::MemStage;
      } else if (idx == wb_reg_wr_idx.uValue() && wb_reg_wr_en.uValue()) {
        return MIPS_ForwardingSrc::WbStage;
      } else {
        return MIPS_ForwardingSrc::IdStage;
      }
    };

    alu_reg2_forwarding_ctrl << [=] {
      const auto idx = id_reg2_idx.uValue();
      if (idx == 0) {
        return MIPS_ForwardingSrc::IdStage;
      } else if (idx == mem_reg_wr_idx.uValue() && mem_reg_wr_en.uValue()) {
        return MIPS_ForwardingSrc::MemStage;
      } else if (idx == wb_reg_wr_idx.uValue() && wb_reg_wr_en.uValue()) {
        return MIPS_ForwardingSrc::WbStage;
      } else {
        return MIPS_ForwardingSrc::IdStage;
      }
    };
  }

  INPUTPORT(id_reg1_idx, c_MIPSRegsBits);
  INPUTPORT(id_reg2_idx, c_MIPSRegsBits);

  INPUTPORT(mem_reg_wr_idx, c_MIPSRegsBits);
  INPUTPORT(mem_reg_wr_en, 1);

  INPUTPORT(wb_reg_wr_idx, c_MIPSRegsBits);
  INPUTPORT(wb_reg_wr_en, 1);

  INPUTPORT_ENUM(opcode_ex, MIPS_Instr);
  INPUTPORT_ENUM(opcode_mem, MIPS_Instr);
  INPUTPORT_ENUM(opcode_wb, MIPS_Instr);

  OUTPUTPORT_ENUM(alu_reg1_forwarding_ctrl, MIPS_ForwardingSrc);
  OUTPUTPORT_ENUM(alu_reg2_forwarding_ctrl, MIPS_ForwardingSrc);

};
} // namespace core
} // namespace vsrtl
