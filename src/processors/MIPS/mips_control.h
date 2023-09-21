#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

class MIPS_Control : public Component {
public:
  /* clang-format off */
    static VSRTL_VT_U do_reg_dest_ctrl(const VSRTL_VT_U& opc) {
        switch(opc){
            // Arithmetic instructions
            case MIPS_Instr::ADD: case MIPS_Instr::ADDU: case MIPS_Instr:: AND: case MIPS_Instr::DIV:
            case MIPS_Instr::DIVU: case MIPS_Instr::XOR: case MIPS_Instr::MFHI: case MIPS_Instr::MFLO:
            case MIPS_Instr::MTHI: case MIPS_Instr::MTLO: case MIPS_Instr::MULT: case MIPS_Instr::MULTU:
            case MIPS_Instr::NOR: case MIPS_Instr::OR: case MIPS_Instr::SLL: case MIPS_Instr::SLLV:
            case MIPS_Instr::SLT: case MIPS_Instr::SLTU: case MIPS_Instr::SRA: case MIPS_Instr::SRAV:
            case MIPS_Instr::SRL: case MIPS_Instr::SRLV: case MIPS_Instr::SUB: case MIPS_Instr::SUBU:
            case MIPS_Instr::JALR:
                return 1;

            default:
                return 0;

        }
    }


    static MIPS_CompOp do_comp_ctrl(const VSRTL_VT_U& opc) {
        switch(opc){
            case MIPS_Instr::BEQ: return MIPS_CompOp::EQ;
            case MIPS_Instr::BGEZ : return MIPS_CompOp::EQ;
            case MIPS_Instr::BGTZ : return MIPS_CompOp::GT;
            case MIPS_Instr::BLEZ : return MIPS_CompOp::LE;
            case MIPS_Instr::BLTZ : return MIPS_CompOp::LT;
            case MIPS_Instr::BNE: return MIPS_CompOp::NE;
            case MIPS_Instr::SLT: return MIPS_CompOp::LT;
            case MIPS_Instr::SLTI: return MIPS_CompOp::LT;
            case MIPS_Instr::SLTU: return MIPS_CompOp::LT;
            case MIPS_Instr::SLTIU: return MIPS_CompOp::LT;
            default: return MIPS_CompOp::NOP;
        }
    }

    static VSRTL_VT_U do_branch_ctrl(const VSRTL_VT_U& opc) {
        switch(opc){
            case MIPS_Instr::BEQ: case MIPS_Instr::BGEZ:
            case MIPS_Instr::BLEZ:
                return 1;
            default:
                return 0;
        }
    }

    static VSRTL_VT_U do_bne_write_ctrl(const VSRTL_VT_U& opc) {
        switch(opc){
        case MIPS_Instr::BNE:
                return 1;
            default:
                return 0;
        }
    }

    static VSRTL_VT_U do_bgt_write_ctrl(const VSRTL_VT_U& opc) {
        switch(opc){
            case MIPS_Instr::BGEZ:case MIPS_Instr::BGTZ:
                return 1;
            default:
                return 0;
        }
    }

    static VSRTL_VT_U do_blt_write_ctrl(const VSRTL_VT_U& opc) {
        switch(opc){
            case MIPS_Instr::BLEZ: case MIPS_Instr::BLTZ:
                return 1;
            default:
                return 0;
        }
    }

    static VSRTL_VT_U do_jump_ctrl(const VSRTL_VT_U& opc) {
        switch(opc){
            case MIPS_Instr::JAL: case MIPS_Instr::JALR: case MIPS_Instr::JR: case MIPS_Instr::J:
                return 1;
            default:
                return 0;
        }
    }

    static MIPS_MemOp do_mem_ctrl(const VSRTL_VT_U& opc) {
        switch(opc){
            case MIPS_Instr::LB: return MIPS_MemOp::LB;
            case MIPS_Instr::LH: return MIPS_MemOp::LH;
            case MIPS_Instr::LW: return MIPS_MemOp::LW;
            case MIPS_Instr::LBU: return MIPS_MemOp::LBU;
            case MIPS_Instr::LHU: return MIPS_MemOp::LHU;
            case MIPS_Instr::SH: return MIPS_MemOp::SH;
            case MIPS_Instr::SB: return MIPS_MemOp::SB;
            case MIPS_Instr::SW: return MIPS_MemOp::SW;
            case MIPS_Instr::LWC1: return MIPS_MemOp::LWC1;
            case MIPS_Instr::SWC1: return MIPS_MemOp::SWC1;
            default:
                return MIPS_MemOp::NOP;
        }
    }

    static VSRTL_VT_U do_reg_do_write_ctrl(const VSRTL_VT_U& opc) {
        switch(opc) {
            case MIPS_Instr::LUI:

            // Arithmetic-immediate instructions
            case MIPS_Instr::ADDI: case MIPS_Instr::ADDIU: case MIPS_Instr::ANDI: case MIPS_Instr::ORI:
            case MIPS_Instr::SLTI: case MIPS_Instr::SLTIU: case MIPS_Instr::XORI:

            // Arithmetic instructions
            case MIPS_Instr::ADD: case MIPS_Instr::ADDU: case MIPS_Instr:: AND: case MIPS_Instr::DIV:
            case MIPS_Instr::DIVU: case MIPS_Instr::XOR: case MIPS_Instr::MFHI: case MIPS_Instr::MFLO:
            case MIPS_Instr::MTHI: case MIPS_Instr::MTLO: case MIPS_Instr::MULT: case MIPS_Instr::MULTU:
            case MIPS_Instr::NOR: case MIPS_Instr::OR: case MIPS_Instr::SLL: case MIPS_Instr::SLLV:
            case MIPS_Instr::SLT: case MIPS_Instr::SLTU: case MIPS_Instr::SRA: case MIPS_Instr::SRAV:
            case MIPS_Instr::SRL: case MIPS_Instr::SRLV: case MIPS_Instr::SUB: case MIPS_Instr::SUBU:


            // Load instructions
            case MIPS_Instr::LB: case MIPS_Instr::LH: case MIPS_Instr::LW: case MIPS_Instr::LBU: case MIPS_Instr::LHU:
            case MIPS_Instr::LWC1: case MIPS_Instr::SWC1:

            // Jump instructions
            case MIPS_Instr::JALR:
            case MIPS_Instr::JAL:
            case MIPS_Instr::JR:
            case MIPS_Instr::J:
                return 1;
            default: return 0;
        }
    }

    static MIPS_RegWrSrc do_reg_wr_src_ctrl(const VSRTL_VT_U& opc) {
        switch(opc){
            // Load instructions
            case MIPS_Instr::LB: case MIPS_Instr::LH: case MIPS_Instr::LW: case MIPS_Instr::LBU: case MIPS_Instr::LHU:
            case MIPS_Instr::LWC1: case MIPS_Instr::SWC1:
                return MIPS_RegWrSrc::MEMREAD;

            // Jump instructions

            default:
                return MIPS_RegWrSrc::ALURES;
        }
    }


    static MIPS_AluSrc2 do_alu_op2_ctrl(const VSRTL_VT_U& opc) {
        switch(opc) {
            case MIPS_Instr::LUI:
                return MIPS_AluSrc2::IMM;

            // Arithmetic-immediate instructions
            case MIPS_Instr::ADDI: case MIPS_Instr::ADDIU: case MIPS_Instr::ANDI: case MIPS_Instr::ORI:
            case MIPS_Instr::SLTI: case MIPS_Instr::SLTIU: case MIPS_Instr::XORI:
                return MIPS_AluSrc2::IMM;

            // Arithmetic instructions
            case MIPS_Instr::ADD: case MIPS_Instr::ADDU: case MIPS_Instr:: AND: case MIPS_Instr::DIV:
            case MIPS_Instr::DIVU: case MIPS_Instr::XOR: case MIPS_Instr::MFHI: case MIPS_Instr::MFLO:
            case MIPS_Instr::MTHI: case MIPS_Instr::MTLO: case MIPS_Instr::MULT: case MIPS_Instr::MULTU:
            case MIPS_Instr::NOR: case MIPS_Instr::OR: case MIPS_Instr::SLL: case MIPS_Instr::SLLV:
            case MIPS_Instr::SLT: case MIPS_Instr::SLTU: case MIPS_Instr::SRA: case MIPS_Instr::SRAV:
            case MIPS_Instr::SRL: case MIPS_Instr::SRLV: case MIPS_Instr::SUB: case MIPS_Instr::SUBU:
                return MIPS_AluSrc2::REG2;


            // Load/Store instructions
            case MIPS_Instr::LB: case MIPS_Instr::LH: case MIPS_Instr::LW: case MIPS_Instr::LBU: case MIPS_Instr::LHU:
            case MIPS_Instr::SH: case MIPS_Instr::SB: case MIPS_Instr::SW: case MIPS_Instr::LWC1: case MIPS_Instr::SWC1:
                return MIPS_AluSrc2::IMM;

            // Branch instructions
            case MIPS_Instr::BEQ: case MIPS_Instr::BGEZ: case MIPS_Instr::BGTZ:
            case MIPS_Instr::BLEZ: case MIPS_Instr::BLTZ: case MIPS_Instr::BNE:
                return MIPS_AluSrc2::REG2;

            // Jump instructions
            case MIPS_Instr::JALR:
            case MIPS_Instr::JAL:
            case MIPS_Instr::JR:
            case MIPS_Instr::J:
                return MIPS_AluSrc2::IMM;

            default:
                return MIPS_AluSrc2::REG2;
        }
    }

    static MIPS_ALUOp do_alu_ctrl(const VSRTL_VT_U& opc) {
        switch(opc) {
            case MIPS_Instr::LB: case MIPS_Instr::LH: case MIPS_Instr::LW: case MIPS_Instr::LBU: case MIPS_Instr::LHU:
            case MIPS_Instr::SH: case MIPS_Instr::SB: case MIPS_Instr::SW: case MIPS_Instr::LWC1: case MIPS_Instr::SWC1:
                return MIPS_ALUOp::ADD;
            case MIPS_Instr::LUI:
                return MIPS_ALUOp::LUI;
            case MIPS_Instr::JAL: case MIPS_Instr::JALR:  case MIPS_Instr::JR:  case MIPS_Instr::J:
            case MIPS_Instr::ADD: case MIPS_Instr::ADDI: case MIPS_Instr::ADDIU:
            case MIPS_Instr::BEQ: case MIPS_Instr::BLTZ: case MIPS_Instr::BNE:
                return MIPS_ALUOp::ADD;
            case MIPS_Instr::BGEZ: case MIPS_Instr::BLEZ:
                return MIPS_ALUOp::BGLZ;
            case MIPS_Instr::SUB: case MIPS_Instr::SUBU: return MIPS_ALUOp::SUB;
            case MIPS_Instr::SLT: case MIPS_Instr::SLTI:
                return MIPS_ALUOp::LT;
            case MIPS_Instr::SLTU: case MIPS_Instr::SLTIU:
                return MIPS_ALUOp::LTU;
            case MIPS_Instr::XOR: case MIPS_Instr::XORI:
                return MIPS_ALUOp::XOR;
            case MIPS_Instr::OR: case MIPS_Instr::ORI:
                return MIPS_ALUOp::OR;
            case MIPS_Instr::AND: case MIPS_Instr::ANDI:
                return MIPS_ALUOp::AND;
            case MIPS_Instr::SLL:
                return MIPS_ALUOp::SL;
            case MIPS_Instr::SLLV:
                return MIPS_ALUOp::SLLV;
            case MIPS_Instr::SRL:
                return MIPS_ALUOp::SRL;
            case MIPS_Instr::SRA:
                return MIPS_ALUOp::SRA;
            case MIPS_Instr::SRAV:
                return MIPS_ALUOp::SRAV;
            case MIPS_Instr::ADDU   : return MIPS_ALUOp::ADDU;
            case MIPS_Instr::DIV  : return MIPS_ALUOp::DIV;
            case MIPS_Instr::DIVU : return MIPS_ALUOp::DIVU;
            case MIPS_Instr::MFHI: return MIPS_ALUOp::MFHI;
            case MIPS_Instr::MFLO   : return MIPS_ALUOp::MFLO;
            case MIPS_Instr::MTHI  : return MIPS_ALUOp::MTHI;
            case MIPS_Instr::MTLO   : return MIPS_ALUOp::MTLO;
            case MIPS_Instr::MULT  : return MIPS_ALUOp::MULT;
            case MIPS_Instr::MULTU : return MIPS_ALUOp::MULTU;
            case MIPS_Instr::NOR : return MIPS_ALUOp::NOR;
            case MIPS_Instr::SRLV : return MIPS_ALUOp::SRLV;


            default: return MIPS_ALUOp::NOP;
        }
    }

    static VSRTL_VT_U do_do_mem_write_ctrl(const VSRTL_VT_U& opc) {
        switch(opc) {
            case MIPS_Instr::SH: case MIPS_Instr::SB: case MIPS_Instr::SW: case MIPS_Instr::LWC1: case MIPS_Instr::SWC1:
                return 1;
            default: return 0;
        }
    }

    static VSRTL_VT_U do_do_read_ctrl(const VSRTL_VT_U& opc) {
        switch(opc) {
            case MIPS_Instr::LB: case MIPS_Instr::LH: case MIPS_Instr::LW: case MIPS_Instr::LBU: case MIPS_Instr::LHU:
                return 1;
            default: return 0;
        }
    }
  /* clang-format on */

public:
  MIPS_Control(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    do_reg_dst << [=] { return do_reg_dest_ctrl(opcode.uValue()); };
    comp_ctrl << [=] { return do_comp_ctrl(opcode.uValue()); };
    do_branch << [=] { return do_branch_ctrl(opcode.uValue()); };
    do_bne_write << [=] { return do_bne_write_ctrl(opcode.uValue()); };
    do_bgt_write << [=] { return do_bgt_write_ctrl(opcode.uValue()); };
    do_blt_write << [=] { return do_blt_write_ctrl(opcode.uValue()); };
    do_jump << [=] { return do_jump_ctrl(opcode.uValue()); };
    mem_ctrl << [=] { return do_mem_ctrl(opcode.uValue()); };
    reg_do_write_ctrl << [=] { return do_reg_do_write_ctrl(opcode.uValue()); };
    reg_wr_src_ctrl << [=] { return do_reg_wr_src_ctrl(opcode.uValue()); };
    alu_op2_ctrl << [=] { return do_alu_op2_ctrl(opcode.uValue()); };
    alu_ctrl << [=] { return do_alu_ctrl(opcode.uValue()); };
    mem_do_write_ctrl << [=] { return do_do_mem_write_ctrl(opcode.uValue()); };
    mem_do_read_ctrl << [=] { return do_do_read_ctrl(opcode.uValue()); };
  }

  INPUTPORT_ENUM(opcode, MIPS_Instr);
  INPUTPORT(instr31_26, c_MIPSRegsBits);
  OUTPUTPORT(do_reg_dst, 1);
  OUTPUTPORT(reg_do_write_ctrl, 1);
  OUTPUTPORT(mem_do_write_ctrl, 1);
  OUTPUTPORT(mem_do_read_ctrl, 1);
  OUTPUTPORT(do_branch, 1);
  OUTPUTPORT(do_jump, 1);
  OUTPUTPORT(do_bne_write, 1);
  OUTPUTPORT(do_bgt_write, 1);
  OUTPUTPORT(do_blt_write, 1);
  OUTPUTPORT_ENUM(comp_ctrl, MIPS_CompOp);
  OUTPUTPORT_ENUM(reg_wr_src_ctrl, MIPS_RegWrSrc);
  OUTPUTPORT_ENUM(mem_ctrl, MIPS_MemOp);
  OUTPUTPORT_ENUM(alu_op2_ctrl, MIPS_AluSrc2);
  OUTPUTPORT_ENUM(alu_ctrl, MIPS_ALUOp);
};

} // namespace core
} // namespace vsrtl
