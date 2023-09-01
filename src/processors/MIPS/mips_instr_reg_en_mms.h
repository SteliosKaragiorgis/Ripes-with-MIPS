#pragma once

#include "Signals/Signal.h"
#include "VSRTL/core/vsrtl_component.h"

#include "mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

class MIPS_Reg_En_MMS : public Component {
public:
  MIPS_Reg_En_MMS(const std::string &name, SimComponent *parent)
      : Component(name, parent) {

      instr_reg_enable << [=] {
          switch(state_in.uValue()){
              case MIPSMulti_States::S0: return 1;
              default: return 0;
          }
      };



  }



  INPUTPORT_ENUM(state_in, MIPSMulti_States);
  OUTPUTPORT(instr_reg_enable, 1);

private:

};

} // namespace core
} // namespace vsrtl
