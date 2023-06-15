#pragma once

#include <QObject>
#include <functional>

#include "assembler.h"
#include "mipsassembler_common.h"

namespace Ripes {
namespace Assembler {

template <typename Reg__T>
struct MIPS_I {
    AssemblerTypes(Reg__T);
    enum class Options {
      shifts64BitVariant, // appends 'w' to 32-bit shift operations, for use in
                          // the 64-bit RISC-V ISA
      LI64BitVariant      // Modifies LI to be able to emit 64-bit constants
    };

    static void enable(const ISAInfoBase *isa, _InstrVec &instructions,
                       _PseudoInstrVec &pseudoInstructions,
                       const std::set<Options> &options = {}) {



        // clang-format off


        pseudoInstructions.push_back(std::shared_ptr<_PseudoInstruction>(
            new _PseudoInstruction(Token("move"), {RegTok, RegTok}, _PseudoExpandFunc(line) {
                return LineTokensVec{ LineTokens{Token("addu"), line.tokens.at(1), Token("$zero"), line.tokens.at(2)}};
            })));

        pseudoInstructions.push_back(std::shared_ptr<_PseudoInstruction>(new _PseudoInstruction(
            Token("li"), {RegTok, ImmTok}, _PseudoExpandFunc(line) {
                return LineTokensVec{LineTokens{Token("ori"), line.tokens.at(1), Token("$zero"), line.tokens.at(2)}};
            })));

        pseudoInstructions.push_back(std::shared_ptr<_PseudoInstruction>(new _PseudoInstruction(
            Token("lw"), {RegTok, ImmTok}, _PseudoExpandFunc(line) {
                return LineTokensVec{LineTokens() << Token("lui") << Token("$at") << Token("0x1001"),
                                     LineTokens() << Token("lw") << line.tokens.at(1) << Token(QString("%1").arg(line.tokens.at(2)), "%pcrel_lo16") << Token("$at") };
            })));


        pseudoInstructions.push_back(std::shared_ptr<_PseudoInstruction>(new _PseudoInstruction(
            Token("la"), {RegTok, ImmTok}, _PseudoExpandFunc(line) {
                return LineTokensVec{LineTokens() << Token("lui") << Token("$at") << Token("0x1001"),
                                     LineTokens() << Token("ori") << line.tokens.at(1) << Token("$at") << Token(QString("%1").arg(line.tokens.at(2)), "%pcrel_lo16") };
            })));

        pseudoInstructions.push_back(std::shared_ptr<_PseudoInstruction>(new _PseudoInstruction(
            Token("blt"), {RegTok, RegTok, ImmTok}, _PseudoExpandFunc(line) {
                return LineTokensVec{LineTokens() << Token("slt") << Token("$at") << line.tokens.at(1) << line.tokens.at(2),
                                     LineTokens() << Token("bne") << Token("$at") << Token("$zero") << line.tokens.at(3) };
            })));

        pseudoInstructions.push_back(std::shared_ptr<_PseudoInstruction>(new _PseudoInstruction(
            Token("ble"), {RegTok, RegTok, ImmTok}, _PseudoExpandFunc(line) {
                return LineTokensVec{LineTokens() << Token("slt") << Token("$at") << line.tokens.at(2) << line.tokens.at(1),
                                     LineTokens() << Token("beq") << Token("$at") << Token("$zero") << line.tokens.at(3) };
            })));

        pseudoInstructions.push_back(std::shared_ptr<_PseudoInstruction>(new _PseudoInstruction(
            Token("bgt"), {RegTok, RegTok, ImmTok}, _PseudoExpandFunc(line) {
                return LineTokensVec{LineTokens() << Token("slt") << Token("$at") << line.tokens.at(2) << line.tokens.at(1),
                                     LineTokens() << Token("bne") << Token("$at") << Token("$zero") << line.tokens.at(3) };
            })));

        pseudoInstructions.push_back(std::shared_ptr<_PseudoInstruction>(new _PseudoInstruction(
            Token("bge"), {RegTok, RegTok, ImmTok}, _PseudoExpandFunc(line) {
                return LineTokensVec{LineTokens() << Token("slt") << Token("$at") << line.tokens.at(1) << line.tokens.at(2),
                                     LineTokens() << Token("beq") << Token("$at") << Token("$zero") << line.tokens.at(3) };
            })));






        instructions.push_back(std::shared_ptr<_Instruction>(new _Instruction(
            _Opcode(Token("syscall"),
                    {OpPart(MIPSISA::Function::SYSCALL, 0, 5), OpPart(0, 6, 25), OpPart(MIPSISA::Opcode::RTYPE,26,31)}),
            {})));

        instructions.push_back(std::shared_ptr<_Instruction>(new _Instruction(
            _Opcode(Token("break"),
                    {OpPart(MIPSISA::Function::BREAK, 0, 5), OpPart(0, 6, 25), OpPart(MIPSISA::Opcode::RTYPE,26,31)}),
            {})));

        //R-TYPE Instructions(rd, rs, rt)
        instructions.push_back(RType(Token("add"), MIPSISA::Function::ADD));
        instructions.push_back(RType(Token("addu"), MIPSISA::Function::ADDU));
        instructions.push_back(RType(Token("and"), MIPSISA::Function::AND));
        instructions.push_back(RType(Token("nor"), MIPSISA::Function::NOR));
        instructions.push_back(RType(Token("or"), MIPSISA::Function::OR));
        instructions.push_back(RType(Token("sub"), MIPSISA::Function::SUB));
        instructions.push_back(RType(Token("subu"), MIPSISA::Function::SUBU));
        instructions.push_back(RType(Token("xor"), MIPSISA::Function::XOR));
        instructions.push_back(RType(Token("slt"), MIPSISA::Function::SLT));
        instructions.push_back(RType(Token("sltu"), MIPSISA::Function::SLTU));


        //R-TYPE Instructions(rd, rt, rs)
        instructions.push_back(RTypeRtRs(Token("sllv"), MIPSISA::Function::SLLV));
        instructions.push_back(RTypeRtRs(Token("srav"), MIPSISA::Function::SRAV));
        instructions.push_back(RTypeRtRs(Token("srlv"), MIPSISA::Function::SRLV));

        //R-TYPE Mult,Div  (rs, rt)
        instructions.push_back(RMultDivType(Token("div"), MIPSISA::Function::DIV));
        instructions.push_back(RMultDivType(Token("divu"), MIPSISA::Function::DIVU));
        instructions.push_back(RMultDivType(Token("mult"), MIPSISA::Function::MULT));
        instructions.push_back(RMultDivType(Token("multu"), MIPSISA::Function::MULTU));

        //R-TYPE Jalr (rd, rs)
        instructions.push_back(JALRType(Token("jalr"), MIPSISA::Function::JALR));


        //R-TYPE Rs  (rs)
        instructions.push_back(RRSType(Token("jr"), MIPSISA::Function::JR));
        instructions.push_back(RRSType(Token("mthi"), MIPSISA::Function::MTHI));
        instructions.push_back(RRSType(Token("mtlo"), MIPSISA::Function::MTLO));

        //R-TYPE Rd  (rd)
        instructions.push_back(RRDType(Token("mfhi"), MIPSISA::Function::MFHI));
        instructions.push_back(RRDType(Token("mflo"), MIPSISA::Function::MFLO));



        //R-TYPE Shift Instructions
        instructions.push_back(RShiftType(Token("sll"), MIPSISA::Function::SLL));
        instructions.push_back(RShiftType(Token("sra"), MIPSISA::Function::SRA));
        instructions.push_back(RShiftType(Token("srl"), MIPSISA::Function::SRL));       /// @bug:the program is not working with this instruction



        //I-TYPE Instructions
        instructions.push_back(IType(Token("addi"), MIPSISA::Opcode::ADDI));
        instructions.push_back(IType(Token("addiu"), MIPSISA::Opcode::ADDIU));          /// @bug:the program is not working with this instruction
        instructions.push_back(IType(Token("andi"), MIPSISA::Opcode::ANDI));
        instructions.push_back(IType(Token("ori"), MIPSISA::Opcode::ORI));
        instructions.push_back(IType(Token("slti"), MIPSISA::Opcode::SLTI));
        instructions.push_back(IType(Token("sltiu"), MIPSISA::Opcode::SLTIU));
        instructions.push_back(IType(Token("xori"), MIPSISA::Opcode::XORI));


        //I-TYPE Lui
        instructions.push_back(ILuiType(Token("lui"), MIPSISA::Opcode::LUI));

        //I-TYPE Branch Instructions
        instructions.push_back(IBranchType(Token("beq"), MIPSISA::Opcode::BEQ));
        instructions.push_back(IBranchType(Token("bne"), MIPSISA::Opcode::BNE));

        //I-TYPE Branch with Zero Instructions
        instructions.push_back(IBranchZeroType(Token("bgez"), MIPSISA::Opcode::BGEZ, 0b00001));
        instructions.push_back(IBranchZeroType(Token("bgtz"), MIPSISA::Opcode::BGTZ, 0b00000));
        instructions.push_back(IBranchZeroType(Token("blez"), MIPSISA::Opcode::BLEZ, 0b00000));
        instructions.push_back(IBranchZeroType(Token("bltz"), MIPSISA::Opcode::BLTZ, 0b00000));

        //I-TYPE Load Store
        instructions.push_back(ILoadStoreType(Token("lb"), MIPSISA::Opcode::LB));
        instructions.push_back(ILoadStoreType(Token("lbu"), MIPSISA::Opcode::LBU));
        instructions.push_back(ILoadStoreType(Token("lh"), MIPSISA::Opcode::LH));
        instructions.push_back(ILoadStoreType(Token("lhu"), MIPSISA::Opcode::LHU));
        instructions.push_back(ILoadStoreType(Token("lw"), MIPSISA::Opcode::LW));
        instructions.push_back(ILoadStoreType(Token("lwc1"), MIPSISA::Opcode::LWC1));
        instructions.push_back(ILoadStoreType(Token("sb"), MIPSISA::Opcode::SB));
        instructions.push_back(ILoadStoreType(Token("sh"), MIPSISA::Opcode::SH));
        instructions.push_back(ILoadStoreType(Token("sw"), MIPSISA::Opcode::SW));
        instructions.push_back(ILoadStoreType(Token("swc1"), MIPSISA::Opcode::SWC1));


        //J-Type
        instructions.push_back(JType(Token("j"), MIPSISA::Opcode::J));
        instructions.push_back(JType(Token("jal"), MIPSISA::Opcode::JAL));



    }


};

} // namespace Assembler
} // namespace Ripes
