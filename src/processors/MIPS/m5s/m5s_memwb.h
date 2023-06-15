#pragma once

#include "VSRTL/core/vsrtl_component.h"
#include "VSRTL/core/vsrtl_register.h"

#include "../mips.h"

#include "../m5s_no_fw_hz/m5s_no_fw_hz_memwb.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class M5S_MEMWB : public MIPS_MEMWB<XLEN> {
public:
  M5S_MEMWB(const std::string &name, SimComponent *parent)
      : MIPS_MEMWB<XLEN>(name, parent) {
    CONNECT_REGISTERED_INPUT(stalled);
  }

  REGISTERED_INPUT(stalled, 1);
};

} // namespace core
} // namespace vsrtl
