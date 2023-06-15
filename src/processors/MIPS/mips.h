#pragma once

#include <functional>

#include "VSRTL/core/vsrtl_enum.h"
#include "VSRTL/interface/vsrtl_binutils.h"
#include "VSRTL/interface/vsrtl_interface.h"

#include "mips_instrparser.h"

#include "../../isa/mips32isainfo.h"
#include "../../isa/mips32isainfo.h"
#include "../../isa/mipsisainfo_common.h"

namespace Ripes {

constexpr int c_MIPSInstrWidth = 32; // Width of instructions
constexpr int c_MIPSRegs = 34;       // Number of registers
constexpr int c_MIPSRegsBits =
    ceillog2(c_MIPSRegs); // Width of operand to index into registers

/** Instruction set enumerations */
Enum(MIPS_InstrType, R, I, J);

Enum(MIPS_Instr, NOP,
     /* MIPS32 R-TYPE Instruction Set */
     ADD, ADDU, AND, DIV, DIVU, MULT, MULTU, NOR, OR, SLL, SLLV, SRA, SRAV,
     SRL, SRLV, SUB, SUBU, XOR, SLT, SLTU, JALR, JR, MFHI, MFLO, MTHI, MTLO,
     BREAK, SYSCALL,

     /* MIPS32 I-TYPE Standard Extension */
     ADDI, ADDIU, ANDI, ORI, XORI, LHI, LLO, SLTI, SLTIU, BEQ, BGEZ, BGTZ,
     BLEZ, BLTZ, BNE, LB, LBU, LH, LHU, LUI, LW, LWC1, SB, SH, SW, SWC1, TRAP,

     /* MIPS32 J-TYPE Instruction Set */
     J, JAL);


/** Datapath enumerations */
Enum(MIPS_RegDst, RT = 0, RD = 1);
Enum(MIPS_Jump, ALU = 0, PC4 = 1);
Enum(MIPS_Branch, BRANCH = 1);
Enum(MIPS_ALUOp, NOP, ADD, ADDU, AND, DIV, DIVU, MULT, MULTU, NOR, OR, SLL, SLLV,
     SLT, SLTU, SRA, SRAV, SRL, SRLV, SUB, SUBU, XOR, LB, LBU, LH, LHU,
     LT, LTU, SL, LUI, SLTI, SLTIU, SH, XORI, MFHI, MFLO, MTHI, MTLO, BGLZ); //BGL-Branch Greater Less Zero
Enum(MIPS_RegWrSrc, ALURES, MEMREAD);
Enum(MIPS_AluSrc2, REG2, IMM);
Enum(MIPS_CompOp, NOP, EQ, NE, LT, LTU, GT, GTU, LE, LEU, GE, GEU);
Enum(MIPS_MemOp, NOP, LB, LH, LW, LBU, LHU, SB, SH, SW, LWC1, SWC1);
Enum(MIPS_ECALL, none, print_int = 1, print_char = 2, print_string = 4, exit = 10);
Enum(MIPS_PcSrc, PC4 = 0, ALU = 1);
Enum(MIPS_PcInc, INC2 = 0, INC4 = 1);
Enum(MIPS_WrReg, RR=0 , WR=1);
Enum(MIPS_Mf, IN1, IN2, IN3);
Enum(MIPS_Mt, IN1, IN2, IN3);
Enum(MIPS_Mf_Mt_En, IN1, IN2);
Enum(MIPS_Mult_Div, IN1, IN2);
Enum(MIPS_Mt_Reg, IN1, IN2, IN3);
Enum(MIPS_WrData, IN1, IN2, IN3);
Enum(MIPS_WrRegInternal, IN1, IN2, IN3);
Enum(MIPS_PcBranch, IN1, IN2);

/** Multi cycle datapath enumerations */
Enum(MIPSMulti_AluSrc1, PC, REG1);
Enum(MIPSMulti_AluSrc2, REG2, FOUR, IMM, IMMSHIFT);
Enum(MIPSMulti_PcSrc, ALU, ALUREG, JADD);
Enum(MIPSMulti_Address, PC, ALUREG);
Enum(MIPSMulti_Instr, S0, OTHER);
Enum(MIPSMulti_States, S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, S12, S13, S14, S15, S16);



/** Instruction field parser */
class MIPS_InstrParser {
public:
  static MIPS_InstrParser *getParser() {
    static MIPS_InstrParser parser;
    return &parser;
  }

  std::vector<uint32_t> decodeR32Instr(const uint32_t &instr) const {
      return m_decodeR32Instr(instr);
    }
    std::vector<uint32_t> decodeI32Instr(const uint32_t &instr) const {
      return m_decodeI32Instr(instr);
    }
    std::vector<uint32_t> decodeJ32Instr(const uint32_t &instr) const {
      return m_decodeJ32Instr(instr);
    }


private:
  MIPS_InstrParser() {
    m_decodeR32Instr = mips_generateInstrParser<uint32_t>(
        std::vector<int>{6, 5, 5, 5, 5, 6}); // from LSB to MSB
    m_decodeI32Instr =
        mips_generateInstrParser<uint32_t>(std::vector<int>{16, 5, 5, 6});
    m_decodeJ32Instr =
        mips_generateInstrParser<uint32_t>(std::vector<int>{26, 6});


  }
  decode_functor<uint32_t> m_decodeR32Instr;
  decode_functor<uint32_t> m_decodeI32Instr;
  decode_functor<uint32_t> m_decodeJ32Instr;


};

} // namespace Ripes
