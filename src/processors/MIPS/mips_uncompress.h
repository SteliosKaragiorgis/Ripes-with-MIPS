#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class MIPS_Uncompress : public Component {
public:
  void setISA(const std::shared_ptr<ISAInfoBase> &isa) {
    m_isa = isa;
    m_disabled = !m_isa->extensionEnabled("C");
  }

  MIPS_Uncompress(std::string name, SimComponent *parent)
      : Component(name, parent) {
    setDescription("Uncompresses instructions from the 'C' extension into "
                   "their 32-bit representation.");
    Pc_Inc << [=] {
      if (m_disabled)
        return true;
      return (((instr.uValue() & 0b11) == 0b11) || (!instr.uValue()));
    };

    // only support 32 bit instructions
    exp_instr << [=] {
      const auto instrValue = instr.uValue();
      VInt new_instr = instrValue;

      return new_instr;
    };
  }

  INPUTPORT(instr, c_MIPSInstrWidth);
  OUTPUTPORT(Pc_Inc, 1);
  OUTPUTPORT(exp_instr, c_MIPSInstrWidth);

private:
  std::shared_ptr<ISAInfoBase> m_isa;
  bool m_disabled = true;
};

} // namespace core
} // namespace vsrtl
