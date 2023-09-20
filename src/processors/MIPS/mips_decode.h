#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "mips.h"
#include <bitset>

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class MIPS_Decode : public Component {
public:
  void setISA(const std::shared_ptr<ISAInfoBase> &isa) { m_isa = isa; }

  MIPS_Decode(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    opcode << [=] {
      const auto instrValue = instr.uValue();

      const unsigned l7 = (instr.uValue() >> 26) & 0b111111;

      // clang-format off
            switch(l7) {
            case MIPSISA::Opcode::LUI: return MIPS_Instr::LUI;
            case MIPSISA::Opcode::JAL: return MIPS_Instr::JAL;
            case MIPSISA::Opcode::J: return MIPS_Instr::J;

            case MIPSISA::Opcode::RTYPE:{
                const auto fields = MIPS_InstrParser::getParser()->decodeR32Instr(instrValue);
                switch(fields[5]){
                    case MIPSISA::Function::ADD: return MIPS_Instr::ADD;
                    case MIPSISA::Function::ADDU: return MIPS_Instr::ADDU;
                    case MIPSISA::Function::AND: return MIPS_Instr::AND;
                    case MIPSISA::Function::BREAK: return MIPS_Instr::BREAK;
                    case MIPSISA::Function::DIV: return MIPS_Instr::DIV;
                    case MIPSISA::Function::DIVU: return MIPS_Instr::DIVU;
                    case MIPSISA::Function::MFHI: return MIPS_Instr::MFHI;
                    case MIPSISA::Function::MFLO: return MIPS_Instr::MFLO;
                    case MIPSISA::Function::MTHI: return MIPS_Instr::MTHI;
                    case MIPSISA::Function::MTLO: return MIPS_Instr::MTLO;
                    case MIPSISA::Function::MULT: return MIPS_Instr::MULT;
                    case MIPSISA::Function::MULTU: return MIPS_Instr::MULTU;
                    case MIPSISA::Function::NOR: return MIPS_Instr::NOR;
                    case MIPSISA::Function::OR: return MIPS_Instr::OR;
                    case MIPSISA::Function::SLL: return MIPS_Instr::SLL;
                    case MIPSISA::Function::SLLV: return MIPS_Instr::SLLV;
                    case MIPSISA::Function::SLT: return MIPS_Instr::SLT;
                    case MIPSISA::Function::SLTU: return MIPS_Instr::SLTU;
                    case MIPSISA::Function::SRA: return MIPS_Instr::SRA;
                    case MIPSISA::Function::SRAV: return MIPS_Instr::SRAV;
                    case MIPSISA::Function::SRL: return MIPS_Instr::SRL;
                    case MIPSISA::Function::SRLV: return MIPS_Instr::SRLV;
                    case MIPSISA::Function::SUB: return MIPS_Instr::SUB;
                    case MIPSISA::Function::SUBU: return MIPS_Instr::SUBU;
                    case MIPSISA::Function::XOR: return MIPS_Instr::XOR;
                    case MIPSISA::Function::SYSCALL: return MIPS_Instr::SYSCALL;
                    case MIPSISA::Function::JALR: return MIPS_Instr::JALR;
                    case MIPSISA::Function::JR: return MIPS_Instr::JR;

                    default: return MIPS_Instr::NOP;


                }
            }

            case MIPSISA::Opcode::ADDI: return MIPS_Instr::ADDI;
            case MIPSISA::Opcode::ADDIU: return MIPS_Instr::ADDIU;
            case MIPSISA::Opcode::ANDI: return MIPS_Instr::ANDI;
            case MIPSISA::Opcode::BEQ: return MIPS_Instr::BEQ;
            case 0b000001:{
                const auto fields = MIPS_InstrParser::getParser()->decodeI32Instr(instrValue);
                switch(fields[2]){
                    case 0b00001:
                        return MIPS_Instr::BGEZ;
                    default:
                        return MIPS_Instr::BLTZ;
                }
            }
            case MIPSISA::Opcode::BGTZ: return MIPS_Instr::BGTZ;
            case MIPSISA::Opcode::BLEZ: return MIPS_Instr::BLEZ;
            case MIPSISA::Opcode::BNE: return MIPS_Instr::BNE;
            case MIPSISA::Opcode::LB: return MIPS_Instr::LB;
            case MIPSISA::Opcode::LBU: return MIPS_Instr::LBU;
            case MIPSISA::Opcode::LH: return MIPS_Instr::LH;
            case MIPSISA::Opcode::LHI: return MIPS_Instr::LHI;
            case MIPSISA::Opcode::LHU: return MIPS_Instr::LHU;
            case MIPSISA::Opcode::LW: return MIPS_Instr::LW;
            case MIPSISA::Opcode::LWC1: return MIPS_Instr::LWC1;
            case MIPSISA::Opcode::ORI: return MIPS_Instr::ORI;
            case MIPSISA::Opcode::SB: return MIPS_Instr::SB;
            case MIPSISA::Opcode::SLTI: return MIPS_Instr::SLTI;
            case MIPSISA::Opcode::SLTIU: return MIPS_Instr::SLTIU;
            case MIPSISA::Opcode::SH: return MIPS_Instr::SH;
            case MIPSISA::Opcode::SW: return MIPS_Instr::SW;
            case MIPSISA::Opcode::SWC1: return MIPS_Instr::SWC1;
            case MIPSISA::Opcode::XORI: return MIPS_Instr::XORI;





            default:
               return MIPS_Instr::NOP;
            }

            // Fallthrough - unknown instruction.

        };

        instr_address_25_0 << [=] {
            const auto printInstr = instr.uValue() & 0b11111111111111111111111111;
          return instr.uValue() & 0b11111111111111111111111111;

        };

        instr_opc_31_26 << [=] {
            const auto printInstr = (instr.uValue() >> 26) & 0b111111;
          return (instr.uValue() >> 26) & 0b11111;

        };

        instr_rs_25_21 << [=] {
            const auto printInstr = (instr.uValue() >> 21) & 0b11111;
          return (instr.uValue() >> 21) & 0b11111;

        };

        instr_rt_20_16 << [=] {
            const auto printInstr = (instr.uValue() >> 16) & 0b11111;
          return (instr.uValue() >> 16) & 0b11111;

        };

        instr_rd_15_11 << [=] {
            const auto printInstr = (instr.uValue() >> 11) & 0b11111;
          return (instr.uValue() >> 11) & 0b11111;

        };

        instr_imm_15_0 << [=] {
            const auto printInstr = instr.uValue() & 0b1111111111111111;
          return instr.uValue() & 0b1111111111111111;

        };

        instr_imm_5_0 << [=] {
            const auto printInstr = instr.uValue() & 0b111111;
          return instr.uValue() & 0b111111;

        };

        instr_shamt_10_6 << [=] {
            const auto printInstr = (instr.uValue() >> 6) & 0b11111;
          return (instr.uValue() >> 6) & 0b11111;

        };

        wr_reg_idx << [=] {
            const auto printInstr = instr.uValue()  >> 11;
          return (instr.uValue() >> 11) & 0b11111;
        };

        r1_reg_idx << [=] {
            const auto printInstr = instr.uValue()  >> 21;
          return (instr.uValue() >> 21) & 0b11111;
        };

        r2_reg_idx << [=] {
            const auto printInstr = instr.uValue()  >> 16;
          return (instr.uValue() >> 16) & 0b11111;
        };

        mf_sel << [=] {
            const auto instrValue = instr.uValue();
            const unsigned l7 = (instr.uValue() >> 26) & 0b111111;
                  switch(l7) {
                    case MIPSISA::Opcode::RTYPE:{
                      const auto fields = MIPS_InstrParser::getParser()->decodeR32Instr(instrValue);
                      switch(fields[5]){

                            case MIPSISA::Function::MFHI:
                                return 1;
                            case MIPSISA::Function::MFLO:{
                                return 2;
                            }

                            default: return 0;

                      }
                    }
                    default: return 0;
                  }
        };

        mt_sel << [=] {
            const auto instrValue = instr.uValue();
            const unsigned l7 = (instr.uValue() >> 26) & 0b111111;
                  switch(l7) {
                    case MIPSISA::Opcode::RTYPE:{
                      const auto fields = MIPS_InstrParser::getParser()->decodeR32Instr(instrValue);
                      switch(fields[5]){

                            case MIPSISA::Function::MTHI:
                                return 1;
                            case MIPSISA::Function::MTLO:
                            case MIPSISA::Function::MULT:
                            case MIPSISA::Function::MULTU:
                            case MIPSISA::Function::DIV:
                            case MIPSISA::Function::DIVU:{
                                return 2;
                            }

                            default: return 0;

                      }
                    }
                    default: return 0;
                  }
        };

        mult_div_sel << [=] {
            const auto instrValue = instr.uValue();
            const unsigned l7 = (instr.uValue() >> 26) & 0b111111;
                  switch(l7) {
                    case MIPSISA::Opcode::RTYPE:{
                      const auto fields = MIPS_InstrParser::getParser()->decodeR32Instr(instrValue);
                      switch(fields[5]){

                            case MIPSISA::Function::MULT:
                            case MIPSISA::Function::MULTU:
                            case MIPSISA::Function::DIV:
                            case MIPSISA::Function::DIVU:
                                return 1;


                            default: return 0;

                      }
                    }
                    default: return 0;
                  }
        };

        j_sel << [=] {
            const auto instrValue = instr.uValue();
            const unsigned l7 = (instr.uValue() >> 26) & 0b111111;
                  switch(l7) {
                    case MIPSISA::Opcode::JAL: return 1;

                    case MIPSISA::Opcode::RTYPE:{
                        const auto fields = MIPS_InstrParser::getParser()->decodeR32Instr(instrValue);
                        switch(fields[5]){
                          case MIPSISA::Function::JALR:
                              return 2;

                          default: return 0;
                        }
                     }

                     default: return 0;
                  }
        };

    // clang-format on
  }

  INPUTPORT(instr, c_MIPSInstrWidth);
  OUTPUTPORT_ENUM(opcode, MIPS_Instr);
  OUTPUTPORT(instr_address_25_0, 26);
  OUTPUTPORT(instr_opc_31_26, c_MIPSRegsBits);
  OUTPUTPORT(instr_rs_25_21, c_MIPSRegsBits);
  OUTPUTPORT(instr_rt_20_16, c_MIPSRegsBits);
  OUTPUTPORT(instr_rd_15_11, c_MIPSRegsBits);
  OUTPUTPORT(instr_imm_15_0, 16);
  OUTPUTPORT(instr_imm_5_0, c_MIPSRegsBits);
  OUTPUTPORT(instr_shamt_10_6, c_MIPSRegsBits);
  OUTPUTPORT(wr_reg_idx, c_MIPSRegsBits);
  OUTPUTPORT(r1_reg_idx, c_MIPSRegsBits);
  OUTPUTPORT(r2_reg_idx, c_MIPSRegsBits);
  OUTPUTPORT(mf_sel, 2);
  OUTPUTPORT(mt_sel, 2);
  OUTPUTPORT(mult_div_sel, 1);
  OUTPUTPORT(j_sel, 2);

private:
  void unknownInstruction() {}
  std::shared_ptr<ISAInfoBase> m_isa;
};

} // namespace core
} // namespace vsrtl
