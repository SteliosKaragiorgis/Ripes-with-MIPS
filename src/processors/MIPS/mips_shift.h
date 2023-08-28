#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "mips.h"

namespace vsrtl {
namespace core {

enum class ShiftType { sl, sra, srl };

template <unsigned int W>
class MIPS_Shift : public Component {
public:
    MIPS_Shift(const std::string& name, SimComponent* parent, ShiftType t, unsigned int shamt) : Component(name, parent) {
        out << [=] {
            if (t == ShiftType::sl) {
                return in.uValue() << shamt;
            } else if (t == ShiftType::sra) {
                return VT_U(in.sValue() >> shamt);
            } else if (t == ShiftType::srl) {
                return in.uValue() >> shamt;
            } else {
                throw std::runtime_error("Unknown shift type");
            }
        };
    }

    OUTPUTPORT(out, W);
    INPUTPORT(in, W);
};

}  // namespace core
}  // namespace vsrtl
