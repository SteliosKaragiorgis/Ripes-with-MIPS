#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class MIPS_Branch : public Component {
public:
  MIPS_Branch(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    // clang-format off
        res << [=] {
            switch(comp_op.uValue()){
                case MIPS_CompOp::NOP: return false;
                case MIPS_CompOp::EQ: return op1.uValue() == op2.uValue();
                case MIPS_CompOp::NE: return op1.uValue() != op2.uValue();
                case MIPS_CompOp::LT: return op1.sValue() < 0;
                case MIPS_CompOp::LTU: return op1.uValue() < op2.uValue();
                case MIPS_CompOp::GT: return op1.sValue() > 0;
                case MIPS_CompOp::GTU: return op1.uValue() > op2.uValue();
                case MIPS_CompOp::LE: return op1.sValue() <= 0;
                case MIPS_CompOp::LEU: return op1.uValue() <= op2.uValue();
                case MIPS_CompOp::GE: return op1.sValue() >= 0;
                case MIPS_CompOp::GEU: return op1.uValue() >= op2.uValue();
                default: assert("Comparator: Unknown comparison operator"); return false;
            }
        };
    // clang-format on
  }

  INPUTPORT_ENUM(comp_op, MIPS_CompOp);
  INPUTPORT(op1, XLEN);
  INPUTPORT(op2, XLEN);
  OUTPUTPORT(res, 1);
};

} // namespace core
} // namespace vsrtl
