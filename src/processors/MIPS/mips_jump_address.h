#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "mips.h"


namespace vsrtl {
namespace core {

template <unsigned XLEN>
class MIPS_Jump_Address : public Component {
public:
  MIPS_Jump_Address(const std::string &name, SimComponent *parent) : Component(name, parent) {

  res << [=] {
      switch(opcode.uValue()) {
          case MIPS_Instr::JR: case MIPS_Instr::JALR:
              return ra.uValue();

          default: return ((pc.uValue() & 0xf0000000) | (sl.uValue()));

      }
  };

  }
  INPUTPORT(pc, XLEN);
  INPUTPORT(sl, 28);
  INPUTPORT_ENUM(opcode, MIPS_Instr);
  INPUTPORT(ra, XLEN);

  OUTPUTPORT(res, XLEN);
};

}  // namespace core
}  // namespace vsrtl
