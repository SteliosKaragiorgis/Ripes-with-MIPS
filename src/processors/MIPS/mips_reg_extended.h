#pragma once

#include "VSRTL/core/vsrtl_component.h"

#include "mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class MIPS_Reg_Extended : public Component {
  static_assert(XLEN == 32 || XLEN == 64, "Only RV32 and RV64 are supported");

public:
  MIPS_Reg_Extended(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    setDescription("Immediate value decoder");
    r1_addr << [=] {
      QString val = QString("%1").arg(reg1.uValue(), 5, 2, QChar('0'));
          return (signextend<6>(reg1.uValue()) & 0x0000003f);
    };

    r2_addr << [=] {
      QString val = QString("%1").arg(reg2.uValue(), 5, 2, QChar('0'));
          return (signextend<6>(reg2.uValue()) & 0x0000003f);
    };

    wr_addr << [=] {
      QString val = QString("%1").arg(wreg.uValue(), 5, 2, QChar('0'));
          return (signextend<6>(wreg.uValue()) & 0x0000003f);
    };
  }



  INPUTPORT(reg1, c_MIPSRegsBits);
  INPUTPORT(reg2, c_MIPSRegsBits);
  INPUTPORT(wreg, c_MIPSRegsBits);
  OUTPUTPORT(r1_addr, c_MIPSRegsBits+1);
  OUTPUTPORT(r2_addr, c_MIPSRegsBits+1);
  OUTPUTPORT(wr_addr, c_MIPSRegsBits+1);
};

} // namespace core
} // namespace vsrtl
