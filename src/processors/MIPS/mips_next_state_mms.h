#pragma once

#include "Signals/Signal.h"
#include "VSRTL/core/vsrtl_component.h"

#include "mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

class MIPS_Next_State_MMS : public Component {
public:
  MIPS_Next_State_MMS(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
      next_state << [=] {
          switch(state_in.uValue()){
              case MIPSMulti_States::S0:{
                  return MIPSMulti_States::S1;
              }

              case MIPSMulti_States::S1:{
                  const auto opc = opcode.uValue();
                  switch(opc){
                      case MIPS_Instr::LB: case MIPS_Instr::LBU: case MIPS_Instr::LH: case MIPS_Instr::LHU:
                      case MIPS_Instr::LW: case MIPS_Instr::LWC1: case MIPS_Instr::SB: case MIPS_Instr::SH:
                      case MIPS_Instr::SW: case MIPS_Instr::SWC1:
                        return MIPSMulti_States::S2;


                      case MIPS_Instr::ADD: case MIPS_Instr::ADDU: case MIPS_Instr:: AND: case MIPS_Instr::DIV:
                      case MIPS_Instr::DIVU: case MIPS_Instr::XOR: case MIPS_Instr::MFHI: case MIPS_Instr::MFLO:
                      case MIPS_Instr::MTHI: case MIPS_Instr::MTLO: case MIPS_Instr::MULT: case MIPS_Instr::MULTU:
                      case MIPS_Instr::NOR: case MIPS_Instr::OR: case MIPS_Instr::SLL: case MIPS_Instr::SLLV:
                      case MIPS_Instr::SLT: case MIPS_Instr::SLTU: case MIPS_Instr::SRA: case MIPS_Instr::SRAV:
                      case MIPS_Instr::SRL: case MIPS_Instr::SRLV: case MIPS_Instr::SUB: case MIPS_Instr::SUBU:
                      case MIPS_Instr::JR: case MIPS_Instr::JALR:
                          return MIPSMulti_States::S6;


                      case MIPS_Instr::BEQ: case MIPS_Instr::BGEZ: case MIPS_Instr::BGTZ:
                      case MIPS_Instr::BLEZ: case MIPS_Instr::BLTZ: case MIPS_Instr::BNE:
                          return MIPSMulti_States::S8;


                      case MIPS_Instr::J:
                            return MIPSMulti_States::S9;


                      case MIPS_Instr::ADDI: case MIPS_Instr::ADDIU: case MIPS_Instr::ANDI: case MIPS_Instr::ORI:
                      case MIPS_Instr::SLTI: case MIPS_Instr::SLTIU: case MIPS_Instr::XORI: case MIPS_Instr::LUI:
                            return MIPSMulti_States::S10;


                       case MIPS_Instr::JAL:
                            return MIPSMulti_States::S12;

                      default: {
                          return MIPSMulti_States::S0;
                      }
                 }
              }

              case MIPSMulti_States::S2:{
                  const auto opc = opcode.uValue();
                  switch(opc){
                      case MIPS_Instr::SB: case MIPS_Instr::SH:
                      case MIPS_Instr::SW: case MIPS_Instr::SWC1:
                        return MIPSMulti_States::S5;

                      case MIPS_Instr::LB: case MIPS_Instr::LBU: case MIPS_Instr::LH: case MIPS_Instr::LHU:
                      case MIPS_Instr::LW: case MIPS_Instr::LWC1:
                        return MIPSMulti_States::S3;
                  }


              }
              case MIPSMulti_States::S3:{
                  return MIPSMulti_States::S4;
              }

              case MIPSMulti_States::S6:{
                  return MIPSMulti_States::S7;
              }
              case MIPSMulti_States::S10:{
                  return MIPSMulti_States::S11;
              }
              default: return MIPSMulti_States::S0;

          }
        };




  }



  INPUTPORT_ENUM(state_in, MIPSMulti_States);
  INPUTPORT_ENUM(opcode, MIPS_Instr);
  OUTPUTPORT_ENUM(next_state, MIPSMulti_States);



};

} // namespace core
} // namespace vsrtl
