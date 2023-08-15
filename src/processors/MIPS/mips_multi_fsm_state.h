#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "mips.h"


namespace vsrtl {
namespace core {

template <unsigned XLEN>
class MIPS_Multi_FSM : public Component {
public:
  MIPS_Multi_FSM(const std::string &name, SimComponent *parent) : Component(name, parent) {

      next_state << [=] {
          return prev_state.uValue();
      };

      }
      INPUTPORT(prev_state, XLEN);

      OUTPUTPORT(next_state, XLEN);
    };

}  // namespace core
}  // namespace vsrtl
