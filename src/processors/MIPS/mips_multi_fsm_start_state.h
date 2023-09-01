#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "mips.h"


namespace vsrtl {
namespace core {

template <unsigned XLEN>
class MIPS_Multi_FSM_Start : public Component {
public:
  MIPS_Multi_FSM_Start(const std::string &name, SimComponent *parent) : Component(name, parent) {

      next_state << [=] {
          return s4.uValue();
      };

      }
      INPUTPORT(start, XLEN);
      INPUTPORT(s4, XLEN);
      INPUTPORT(s5, XLEN);
      INPUTPORT(s7, XLEN);
      INPUTPORT(s8, XLEN);
      INPUTPORT(s9, XLEN);
      INPUTPORT(s11, XLEN);
      INPUTPORT(s12, XLEN);


      OUTPUTPORT(next_state, XLEN);
    };

}  // namespace core
}  // namespace vsrtl
