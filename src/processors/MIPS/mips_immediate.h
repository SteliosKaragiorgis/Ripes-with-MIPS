#pragma once

#include "VSRTL/core/vsrtl_component.h"

#include "mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class MIPS_Immediate : public Component {
  static_assert(XLEN == 32 || XLEN == 64, "Only RV32 and RV64 are supported");

public:
  MIPS_Immediate(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    setDescription("Immediate value decoder");
    imm << [=] {
      QString val = QString("%1").arg(instr15_0.uValue(), 16, 2, QChar('0'));

      if(val[0] == '0')
          return (signextend<32>(instr15_0.uValue()) & 0xffffffff);
      else return (signextend<32>(instr15_0.uValue()) | 0xffff0000);

    };
  }



  INPUTPORT(instr15_0, 16);
  OUTPUTPORT(imm, XLEN);
};

} // namespace core
} // namespace vsrtl
