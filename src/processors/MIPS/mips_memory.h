#pragma once

#include "VSRTL/core/vsrtl_memory.h"
#include "VSRTL/core/vsrtl_wire.h"
#include "mips.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned int addrWidth, unsigned int dataWidth>
class MIPS_Memory : public Component, public BaseMemory<true> {
public:
  SetGraphicsType(ClockedComponent);
  MIPS_Memory(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    addr >> mem->addr;
    wr_en >> mem->wr_en;
    data_in >> mem->data_in;

    wr_width->setSensitiveTo(&op);
    wr_width->out << [=] {
      switch (op.uValue()) {
      case MIPS_MemOp::SB:
        return 1;
      case MIPS_MemOp::SH:
        return 2;
      case MIPS_MemOp::SW:
        return 4;
      case MIPS_MemOp::SWC1:

        return 8;
      default:
        return 0;
      }
    };
    wr_width->out >> mem->wr_width;

    data_out << [=] {
      const auto &value = mem->data_out.uValue();
      switch (op.uValue()) {
      case MIPS_MemOp::LB:
        return VT_U(signextend<8>(value & 0xFFUL));
      case MIPS_MemOp::LBU:
        return value & 0xFFUL;
      case MIPS_MemOp::LH:
        return VT_U(signextend<16>(value & 0xFFFFUL));
      case MIPS_MemOp::LHU:
        return value & 0xFFFFUL;
      case MIPS_MemOp::LW:
        return VT_U(signextend<32>(value));
      case MIPS_MemOp::LWC1:
        return value;
      default:
        return value;
      }
    };
  }

  void setMemory(AddressSpace *addressSpace) {
    setMemory(addressSpace);
    mem->setMemory(addressSpace);
  }

  // MIPSInstrParser is also a BaseMemory... A bit redundant, but MIPSInstrParser has a notion
  // of the memory operation that is happening, while the underlying
  // MemoryAsyncRd does not.
  VSRTL_VT_U addressSig() const override { return addr.uValue(); };
  VSRTL_VT_U wrEnSig() const override { return wr_en.uValue(); };
  VSRTL_VT_U opSig() const override { return op.uValue(); };
  AddressSpace::RegionType accessRegion() const override {
    return mem->accessRegion();
  }

  SUBCOMPONENT(mem, TYPE(MemoryAsyncRd<addrWidth, dataWidth>));

  WIRE(wr_width, ceillog2(dataWidth / 8 + 1)); // Write width, in bytes

  INPUTPORT(addr, addrWidth);
  INPUTPORT(data_in, dataWidth);
  INPUTPORT(wr_en, 1);
  INPUTPORT_ENUM(op, MIPS_MemOp); // original mem read
  INPUTPORT(mem_read, 1);
  OUTPUTPORT(data_out, dataWidth);
};

} // namespace core
} // namespace vsrtl
