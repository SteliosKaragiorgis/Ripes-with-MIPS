#pragma once

#include <QObject>
#include <functional>

#include "assembler.h"
#include "mipsassembler_common.h"

namespace Ripes {
namespace Assembler {

class MIPS32I_Assembler : public QObject, public Assembler<uint32_t> {
  Q_OBJECT

public:
  using Reg_T = uint32_t;
  MIPS32I_Assembler(const ISAInfo<ISA::MIPS32I> *isa);

private:
  std::tuple<_InstrVec, _PseudoInstrVec>
  initInstructions(const ISAInfo<ISA::MIPS32I> *isa) const;

protected:
  QChar commentDelimiter() const override { return '#'; }
};

} // namespace Assembler
} // namespace Ripes
