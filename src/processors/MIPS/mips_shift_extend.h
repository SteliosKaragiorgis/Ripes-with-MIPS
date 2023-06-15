#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "mips.h"

namespace vsrtl {
namespace core {

enum class ShiftTypeExtend { sl, sra, srl };

template <unsigned int W>
class MIPS_Shift_Extend : public Component {
public:
    MIPS_Shift_Extend(const std::string& name, SimComponent* parent, ShiftTypeExtend t, unsigned int shamt) : Component(name, parent) {
        out << [=] {
            if (t == ShiftTypeExtend::sl) {
                return in.uValue() << shamt;
            } else if (t == ShiftTypeExtend::sra) {
                return VT_U(in.sValue() >> shamt);
            } else if (t == ShiftTypeExtend::srl) {
                return in.uValue() >> shamt;
            } else {
                throw std::runtime_error("Unknown shift type");
            }
        };
    }

    OUTPUTPORT(out, W+2);
    INPUTPORT(in, W);
};

}  // namespace core
}  // namespace vsrtl
