#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "VSRTL/core/vsrtl_register.h"

#include "mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class MIPS_STATE_REGISTER : public Component {
public:
  MIPS_STATE_REGISTER(const std::string &name, SimComponent *parent)
      : Component(name, parent) {

    CONNECT_REGISTERED_CLEN_INPUT(state_reg, clear, enable);
  }

  REGISTERED_CLEN_INPUT(state_reg, MIPSMulti_States::width());

  // Register bank controls
  INPUTPORT(enable, 1);
  INPUTPORT(clear, 1);
};

} // namespace core
} // namespace vsrtl
