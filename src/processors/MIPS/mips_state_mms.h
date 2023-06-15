#pragma once

#include "Signals/Signal.h"
#include "VSRTL/core/vsrtl_component.h"

#include "mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

class MIPS_State_MMS : public Component {
public:
  MIPS_State_MMS(const std::string &name, SimComponent *parent)
      : Component(name, parent) {

    state_out << [=] {
        switch(state.uValue()){
            case MIPSMulti_States::S0: return MIPSMulti_States::S0;
            case MIPSMulti_States::S1: return MIPSMulti_States::S1;
            case MIPSMulti_States::S2: return MIPSMulti_States::S2;
            case MIPSMulti_States::S3: return MIPSMulti_States::S3;
            case MIPSMulti_States::S4: return MIPSMulti_States::S4;
            case MIPSMulti_States::S5: return MIPSMulti_States::S5;
            case MIPSMulti_States::S6: return MIPSMulti_States::S6;
            case MIPSMulti_States::S7: return MIPSMulti_States::S7;
            case MIPSMulti_States::S8: return MIPSMulti_States::S8;
            case MIPSMulti_States::S9: return MIPSMulti_States::S9;
            case MIPSMulti_States::S10: return MIPSMulti_States::S10;
            case MIPSMulti_States::S11: return MIPSMulti_States::S11;
            case MIPSMulti_States::S12: return MIPSMulti_States::S12;
            default: return MIPSMulti_States::S0;
        }

    };



  }



  INPUTPORT_ENUM(state, MIPSMulti_States);
  OUTPUTPORT_ENUM(state_out, MIPSMulti_States);

private:

};

} // namespace core
} // namespace vsrtl
