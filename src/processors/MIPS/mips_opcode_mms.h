#pragma once

#include "Signals/Signal.h"
#include "VSRTL/core/vsrtl_component.h"

#include "mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

class MIPS_Opcode_MMS : public Component {
public:
  MIPS_Opcode_MMS(const std::string &name, SimComponent *parent)
      : Component(name, parent) {

    opcode_out << [=] {
        auto instrValue = instr_mem.uValue();
        switch(state.uValue()){
          case MIPSMulti_States::S0:
            instrValue = instr_mem.uValue();
            break;
          default:
            instrValue = instr.uValue();
            break;

        }


        const unsigned l7 = (instrValue >> 26) & 0b111111;


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

    };

    instr_out << [=] {
        const auto instrValue = instr.uValue();
        VInt new_instr = instrValue;
        switch(state.uValue()){
          case MIPSMulti_States::S0:
            new_instr = 0;

        }

        return new_instr;


    };



  }



  INPUTPORT(instr, c_MIPSInstrWidth);
  INPUTPORT(instr_mem, c_MIPSInstrWidth);
  INPUTPORT_ENUM(state, MIPSMulti_States);
  OUTPUTPORT(instr_out, c_MIPSInstrWidth);
  OUTPUTPORT_ENUM(opcode_out, MIPS_Instr);



};

} // namespace core
} // namespace vsrtl
