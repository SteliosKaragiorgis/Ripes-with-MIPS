#pragma once

#include "limits.h"
#include <math.h>

#include "mips.h"
#include "mips_alu.h"

#include "VSRTL/core/vsrtl_component.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class MIPS_ALU_M5S : public Component {
public:
  SetGraphicsType(ALU);
  MIPS_ALU_M5S(const std::string &name, SimComponent *parent) : Component(name, parent) {
    res << [=] {
      switch (ctrl.uValue()) {
      case MIPS_ALUOp::ADD: case MIPS_ALUOp::BGLZ:
        return op1.uValue() + op2.uValue();

      case MIPS_ALUOp::ADDU:{
          const auto result = static_cast<uint32_t>(op1.uValue()) +
                              static_cast<uint32_t>(op2.uValue());
          return VT_U(result);
      }

      case MIPS_ALUOp::AND:
        return op1.uValue() & op2.uValue();

      case MIPS_ALUOp::DIV: {
        const VSRTL_VT_S overflow = div_mips_overflow32;
        if (op2.sValue() == 0) {
          return VT_U(-1);
        } else if (op1.sValue() == overflow && op2.sValue() == -1) {
          // Overflow
          return VT_U(overflow);
        } else {
          return VT_U(op1.sValue() / op2.sValue());
        }
      }

      case MIPS_ALUOp::DIVU: {
        if (op2.uValue() == 0) {
          return VT_U(-1LL);
        } else {
            const auto result = static_cast<uint32_t>(op1.uValue()) /
                                static_cast<uint32_t>(op2.uValue());
            return VT_U(result);
        }
      }

      case MIPS_ALUOp::MULT:
          return VT_U(op1.sValue() * op2.sValue());

      case MIPS_ALUOp::MULTU: {
        const auto result = static_cast<uint32_t>(op1.uValue()) *
                            static_cast<uint32_t>(op2.uValue());
        return VT_U(result);
      }

      case MIPS_ALUOp::NOR:
        return VT_U(~(op1.uValue() | op2.uValue()));


      case MIPS_ALUOp::OR:
        return op1.uValue() | op2.uValue();

      case MIPS_ALUOp::SL:
        return op2.uValue() << shamt.uValue();

      case MIPS_ALUOp::SLLV:
        return op2.uValue() << op1.uValue();

      case MIPS_ALUOp::LT:
        return VT_U(op1.sValue() < op2.sValue() ? 1 : 0);

      case MIPS_ALUOp::LTU:
        return VT_U(op1.uValue() < op2.uValue() ? 1 : 0);

      case MIPS_ALUOp::SRA:
        return VT_U(op2.sValue() >> shamt.uValue());

      case MIPS_ALUOp::SRAV:
        return VT_U(op2.sValue() >> op1.uValue());

      case MIPS_ALUOp::SRL:
        return op2.uValue() >> shamt.uValue();

      case MIPS_ALUOp::SRLV:
        return op2.uValue() >> op1.uValue();

      case MIPS_ALUOp::SUB:
      case MIPS_ALUOp::SUBU:
        return op1.uValue() - op2.uValue();

      case MIPS_ALUOp::XOR:
        return op1.uValue() ^ op2.uValue();

      case MIPS_ALUOp::MTHI:
      case MIPS_ALUOp::MTLO:
      case MIPS_ALUOp::MFLO:
        return op1.uValue();

      case MIPS_ALUOp::MFHI:{
          if(opcode_mem.uValue() == MIPS_Instr::DIV ||
                  opcode_mem.uValue() == MIPS_Instr::DIVU ||
                  opcode_mem.uValue() == MIPS_Instr::MULT ||
                  opcode_mem.uValue() == MIPS_Instr::MULT ||
                  opcode_mem.uValue() == MIPS_Instr::MTHI )
                    return hi_in_mem.uValue();
          else if(opcode_wb.uValue() == MIPS_Instr::DIV ||
                            opcode_wb.uValue() == MIPS_Instr::DIVU ||
                            opcode_wb.uValue() == MIPS_Instr::MULT ||
                            opcode_wb.uValue() == MIPS_Instr::MULT ||
                            opcode_wb.uValue() == MIPS_Instr::MTHI )
                              return hi_in_wb.uValue();
          else return op1.uValue();
      }


      case MIPS_ALUOp::LUI:{
        return VT_U(op2.uValue() << 16);
      }


      case MIPS_ALUOp::NOP:
        return VT_U(0xDEADBEEF);


      default:
        throw std::runtime_error("Invalid ALU opcode");
      }
    };

    zero << [=] {
        switch(ctrl.uValue()){
            case MIPS_ALUOp::BGLZ: {
                if(op1.sValue() == 0){
                    return 1;
                }
                return 0;
            }

            default:
                break;
        }

        if((op1.uValue() - op2.uValue()) == 0){
            return 1;
        }
        return 0;
    };

    greater << [=] {
        if(op1.sValue() > 0){
            return 1;
        }
        return 0;
    };

    less << [=] {
        if(op1.sValue() < 0){
            return 1;
        }
        return 0;
    };

    hi << [=] {
        switch (ctrl.uValue()) {
            case MIPS_ALUOp::DIV: {
                const VSRTL_VT_S overflow = div_mips_overflow32;

                if (op2.sValue() == 0) {

                  return op1.uValue();
                } else if (op1.sValue() == overflow && op2.sValue() == -1) {
                  // Overflow

                  return VT_U(0);
                } else {

                  return VT_U(op1.sValue() % op2.sValue());
                }
            }

            case MIPS_ALUOp::DIVU: {
                const VSRTL_VT_S overflow = div_mips_overflow32;
                if (op2.uValue() == 0) {
                  return op1.uValue();
                } else {
                  return op1.uValue() % op2.uValue();
                }

                if (op2.sValue() == 0) {
                  return VT_U(-1);
                } else if (op1.sValue() == overflow && op2.sValue() == -1) {
                  // Overflow
                  return VT_U(overflow);
                } else {
                  return VT_U(op1.sValue() / op2.sValue());
                }
            }

            case MIPS_ALUOp::MULT: {
               const auto result = static_cast<int64_t>(op1.sValue()) *
                                static_cast<int64_t>(op2.sValue());
               return VT_U(result >> 32);
            }

            case MIPS_ALUOp::MULTU: {
                const auto result = static_cast<uint64_t>(op1.uValue()) *
                                static_cast<uint64_t>(op2.uValue());
                return VT_U(result >> 32);
            }

            default: return VT_U(0xDEADBEEF);

        }
    };
  }

  INPUTPORT_ENUM(ctrl, MIPS_ALUOp);
  INPUTPORT(op1, XLEN);
  INPUTPORT(op2, XLEN);
  INPUTPORT(shamt, c_MIPSRegsBits);
  INPUTPORT(hi_in_mem, XLEN);
  INPUTPORT(hi_in_wb, XLEN);
  INPUTPORT_ENUM(opcode_mem, MIPS_Instr);
  INPUTPORT_ENUM(opcode_wb, MIPS_Instr);



  OUTPUTPORT(zero, 1);
  OUTPUTPORT(greater, 1);
  OUTPUTPORT(less, 1);
  OUTPUTPORT(res, XLEN);
  OUTPUTPORT(hi, XLEN);

};

} // namespace core
} // namespace vsrtl
