#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

/// RegDst

class MIPS_ALU_Control : public Component {
public:
  MIPS_ALU_Control(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    res << [=] {
      switch (ctrl.uValue()) {
      case MIPS_ALUOp::NOP:
        return MIPS_ALUOp::NOP;
      case MIPS_ALUOp::ADD:
        return MIPS_ALUOp::ADD;
      case MIPS_ALUOp::ADDU:
        return MIPS_ALUOp::ADDU;
      case MIPS_ALUOp::AND:
        return MIPS_ALUOp::AND;
      case MIPS_ALUOp::DIV:
        return MIPS_ALUOp::DIV;
      case MIPS_ALUOp::DIVU:
        return MIPS_ALUOp::DIVU;
      case MIPS_ALUOp::MULT:
        return MIPS_ALUOp::MULT;
      case MIPS_ALUOp::MULTU:
        return MIPS_ALUOp::MULTU;
      case MIPS_ALUOp::NOR:
        return MIPS_ALUOp::NOR;
      case MIPS_ALUOp::OR:
        return MIPS_ALUOp::OR;
      case MIPS_ALUOp::SLL:
        return MIPS_ALUOp::SLL;
      case MIPS_ALUOp::SLLV:
        return MIPS_ALUOp::SLLV;
      case MIPS_ALUOp::SLT:
        return MIPS_ALUOp::SLT;
      case MIPS_ALUOp::SLTU:
        return MIPS_ALUOp::SLTU;
      case MIPS_ALUOp::SRA:
        return MIPS_ALUOp::SRA;
      case MIPS_ALUOp::SRAV:
        return MIPS_ALUOp::SRAV;
      case MIPS_ALUOp::SRL:
        return MIPS_ALUOp::SRL;
      case MIPS_ALUOp::SRLV:
        return MIPS_ALUOp::SRLV;
      case MIPS_ALUOp::SUB:
        return MIPS_ALUOp::SUB;
      case MIPS_ALUOp::SUBU:
        return MIPS_ALUOp::SUBU;
      case MIPS_ALUOp::XOR:
        return MIPS_ALUOp::XOR;
      case MIPS_ALUOp::LB:
        return MIPS_ALUOp::LB;
      case MIPS_ALUOp::LBU:
        return MIPS_ALUOp::LBU;
      case MIPS_ALUOp::LH:
        return MIPS_ALUOp::LH;
      case MIPS_ALUOp::LHU:
        return MIPS_ALUOp::LHU;
      case MIPS_ALUOp::LT:
        return MIPS_ALUOp::LT;
      case MIPS_ALUOp::LTU:
        return MIPS_ALUOp::LTU;
      case MIPS_ALUOp::SL:
        return MIPS_ALUOp::SL;
      case MIPS_ALUOp::LUI:
        return MIPS_ALUOp::LUI;
      case MIPS_ALUOp::SLTI:
        return MIPS_ALUOp::SLTI;
      case MIPS_ALUOp::SLTIU:
        return MIPS_ALUOp::SLTIU;
      case MIPS_ALUOp::SH:
        return MIPS_ALUOp::SH;
      case MIPS_ALUOp::XORI:
        return MIPS_ALUOp::XORI;
      case MIPS_ALUOp::MFHI:
        return MIPS_ALUOp::MFHI;
      case MIPS_ALUOp::MFLO:
        return MIPS_ALUOp::MFLO;
      case MIPS_ALUOp::MTHI:
        return MIPS_ALUOp::MTHI;
      case MIPS_ALUOp::MTLO:
        return MIPS_ALUOp::MTLO;
      case MIPS_ALUOp::BGLZ:
        return MIPS_ALUOp::BGLZ;
      default:
        throw std::runtime_error("Invalid ALUOp value");
      }
    };
  }
  INPUTPORT_ENUM(ctrl, MIPS_ALUOp); // 2-bit  Used to define Intruction Type
  INPUTPORT(instr5_0,
            c_MIPSRegsBits); // 6-bit  Used for R-Type different ALU operations
  OUTPUTPORT_ENUM(res, MIPS_ALUOp);
};

} // namespace core
} // namespace vsrtl
