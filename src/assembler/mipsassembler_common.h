#pragma once

#include "../isa/mipsisainfo_common.h"
#include "assembler.h"

namespace Ripes {
namespace Assembler {

// The following macros assumes that ASSEMBLER_TYPES(..., ...) has been defined
// for the given assembler.

/// R TYPE
#define RType(name, funct)                                                     \
  std::shared_ptr<_Instruction>(new _Instruction(                              \
      _Opcode(name, {OpPart(funct, 0, 5), OpPart(0b000000 /*shampt*/, 6, 10),  \
                     OpPart(MIPSISA::Opcode::RTYPE /*opcode*/, 26, 31)}),      \
      {std::make_shared<_Reg>(isa, 1, 11, 15, "rd"),                           \
       std::make_shared<_Reg>(isa, 2, 21, 25, "rs"),                           \
       std::make_shared<_Reg>(isa, 3, 16, 20, "rt")}))

#define RTypeRtRs(name, funct)                                                 \
  std::shared_ptr<_Instruction>(new _Instruction(                              \
      _Opcode(name, {OpPart(funct, 0, 5), OpPart(0b000000 /*shampt*/, 6, 10),  \
                     OpPart(MIPSISA::Opcode::RTYPE /*opcode*/, 26, 31)}),      \
      {std::make_shared<_Reg>(isa, 1, 11, 15, "rd"),                           \
       std::make_shared<_Reg>(isa, 2, 16, 20, "rt"),                           \
       std::make_shared<_Reg>(isa, 3, 21, 25, "rs")}))

#define JALRType(name, funct)                                                  \
  std::shared_ptr<_Instruction>(new _Instruction(                              \
      _Opcode(name, {OpPart(funct, 0, 5), OpPart(0b000000 /*shampt*/, 6, 10),  \
                     OpPart(MIPSISA::Opcode::RTYPE /*opcode*/, 26, 31),        \
                     OpPart(0b00000 /*rt*/, 16, 20)}),                         \
      {std::make_shared<_Reg>(isa, 1, 11, 15, "rd"),                           \
       std::make_shared<_Reg>(isa, 2, 21, 25, "rs")}))

#define RMultDivType(name, funct)                                              \
  std::shared_ptr<_Instruction>(new _Instruction(                              \
      _Opcode(name, {OpPart(funct, 0, 5), OpPart(0b000000 /*shampt*/, 6, 10),  \
                     OpPart(0b00000 /*rd*/, 11, 15),                           \
                     OpPart(MIPSISA::Opcode::RTYPE /*opcode*/, 26, 31)}),      \
      {std::make_shared<_Reg>(isa, 1, 21, 25, "rs"),                           \
       std::make_shared<_Reg>(isa, 2, 16, 20, "rt")}))

#define RRSType(name, funct)                                                   \
  std::shared_ptr<_Instruction>(new _Instruction(                              \
      _Opcode(name, {OpPart(funct, 0, 5), OpPart(0b000000 /*shampt*/, 6, 10),  \
                     OpPart(MIPSISA::Opcode::RTYPE /*opcode*/, 26, 31),        \
                     OpPart(0b00000 /*rt*/, 16, 20),                           \
                     OpPart(0b00000 /*rd*/, 11, 15)}),                         \
      {std::make_shared<_Reg>(isa, 1, 21, 25, "rs")}))

#define RRDType(name, funct)                                                   \
  std::shared_ptr<_Instruction>(new _Instruction(                              \
      _Opcode(name, {OpPart(funct, 0, 5), OpPart(0b000000 /*shampt*/, 6, 10),  \
                     OpPart(MIPSISA::Opcode::RTYPE /*opcode*/, 26, 31),        \
                     OpPart(0b00000 /*rt*/, 16, 20),                           \
                     OpPart(0b00000 /*rs*/, 21, 25)}),                         \
      {std::make_shared<_Reg>(isa, 1, 11, 15, "rd")}))

#define RShiftType(name, funct)                                                \
  std::shared_ptr<_Instruction>(new _Instruction(                              \
      _Opcode(name, {OpPart(funct, 0, 5), OpPart(0b00000 /*rs*/, 21, 25),      \
                     OpPart(MIPSISA::Opcode::RTYPE /*opcode*/, 26, 31)}),      \
      {std::make_shared<_Reg>(isa, 1, 11, 15, "rd"),                           \
       std::make_shared<_Reg>(isa, 2, 16, 20, "rt"),                           \
       std::make_shared<_Imm>(3, 5, _Imm::Repr::Signed,                        \
                              std::vector{ImmPart(0, 6, 10)})}))

/// I TYPE
#define IType(name, opcode)                                                    \
  std::shared_ptr<_Instruction>(new _Instruction(                              \
      _Opcode(name, {OpPart(opcode, 26, 31)}),                                 \
      {std::make_shared<_Reg>(isa, 1, 16, 20, "rt"),                           \
       std::make_shared<_Reg>(isa, 2, 21, 25, "rs"),                           \
       std::make_shared<_Imm>(3, 16, _Imm::Repr::Signed,                       \
                              std::vector{ImmPart(0, 0, 15)})}))

#define IBranchType(name, opcode)                                              \
  std::shared_ptr<_Instruction>(                                               \
      new _Instruction(_Opcode(name, {OpPart(opcode, 26, 31)}),                \
                       {std::make_shared<_Reg>(isa, 1, 21, 25, "rs"),          \
                        std::make_shared<_Reg>(isa, 2, 16, 20, "rt"),          \
                        std::make_shared<_Imm>(3, 16, _Imm::Repr::SignedMips,  \
                                               std::vector{ImmPart(0, 0, 15)}, \
                                               _Imm::SymbolType::Relative)}))

#define IBranchZeroType(name, opcode, rt)                                      \
  std::shared_ptr<_Instruction>(new _Instruction(                              \
      _Opcode(name, {OpPart(opcode, 26, 31), OpPart(rt, 16, 20)}),             \
      {std::make_shared<_Reg>(isa, 1, 21, 25, "rs"),                           \
       std::make_shared<_Imm>(2, 16, _Imm::Repr::SignedMips,                   \
                              std::vector{ImmPart(0, 0, 15)},                  \
                              _Imm::SymbolType::Relative)}))

#define ILoadStoreType(name, opcode)                                           \
  std::shared_ptr<_Instruction>(new _Instruction(                              \
      _Opcode(name, {OpPart(opcode, 26, 31)}),                                 \
      {std::make_shared<_Reg>(isa, 1, 16, 20, "rt"),                           \
       std::make_shared<_Imm>(2, 16, _Imm::Repr::Signed,                       \
                              std::vector{ImmPart(0, 0, 15)}),                 \
       std::make_shared<_Reg>(isa, 3, 21, 25, "rs")}))

#define ILuiType(name, opcode)                                                 \
  std::shared_ptr<_Instruction>(new _Instruction(                              \
      _Opcode(name, {OpPart(opcode, 26, 31), OpPart(0b00000, 21, 25)}),        \
      {std::make_shared<_Reg>(isa, 1, 16, 20, "rt"),                           \
       std::make_shared<_Imm>(2, 16, _Imm::Repr::Signed,                       \
                              std::vector{ImmPart(0, 0, 15)})}))

/// J TYPE
#define JType(name, opcode)                                                    \
  std::shared_ptr<_Instruction>(new _Instruction(                              \
      _Opcode(name, {OpPart(opcode, 26, 31)}),                                 \
      {std::make_shared<_Imm>(1, 26, _Imm::Repr::UnsignedMips,                 \
                              std::vector{ImmPart(0, 0, 25)},                  \
                              _Imm::SymbolType::Absolute)}))

#define RegTok _PseudoInstruction::reg()
#define ImmTok _PseudoInstruction::imm()
#define Create_PseudoInstruction
#define _PseudoExpandFuncSyms(line, symbols)                                   \
  [=](const _PseudoInstruction &, const TokenizedSrcLine &line,                \
      const SymbolMap &symbols)

#define _PseudoExpandFunc(line)                                                \
  [](const _PseudoInstruction &, const TokenizedSrcLine &line,                 \
     const SymbolMap &)

#define PseudoLoad(name)                                                       \
  std::shared_ptr<_PseudoInstruction>(new _PseudoInstruction(                  \
      name, {RegTok, ImmTok}, _PseudoExpandFunc(line) {                        \
        LineTokensVec v;                                                       \
        v.push_back(LineTokens() << Token("lui") << Token("x1")                \
                                 << Token(line.tokens.at(2), "%pcrel_hi"));    \
        v.push_back(LineTokens()                                               \
                    << name << line.tokens.at(1)                               \
                    << Token(QString("(%1 + 4)").arg(line.tokens.at(2)),       \
                             "%pcrel_lo")                                      \
                    << line.tokens.at(1));                                     \
        return v;                                                              \
      }))

// The sw is a pseudo-op if a symbol is given as the immediate token. Thus, if
// we detect that a number has been provided, then abort the pseudo-op handling.
#define PseudoStore(name)                                                      \
  std::shared_ptr<_PseudoInstruction>(new _PseudoInstruction(                  \
      name, {RegTok, ImmTok, RegTok}, _PseudoExpandFunc(line) {                \
        bool canConvert;                                                       \
        getImmediate(line.tokens.at(2), canConvert);                           \
        if (canConvert) {                                                      \
          return Result<std::vector<LineTokens>>(                              \
              Error(0, "Unused; will fallback to non-pseudo op sw"));          \
        }                                                                      \
        LineTokensVec v;                                                       \
        v.push_back(LineTokens() << Token("auipc") << line.tokens.at(3)        \
                                 << Token(line.tokens.at(2), "%pcrel_hi"));    \
        v.push_back(LineTokens()                                               \
                    << name << line.tokens.at(1)                               \
                    << Token(QString("(%1 + 4)").arg(line.tokens.at(2)),       \
                             "%pcrel_lo")                                      \
                    << line.tokens.at(3));                                     \
        return Result<std::vector<LineTokens>>(v);                             \
      }))

} // namespace Assembler
} // namespace Ripes
