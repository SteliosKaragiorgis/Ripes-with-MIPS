#pragma once

#include "VSRTL/interface/vsrtl_binutils.h"
#include "relocation.h"
#include "QDebug"

namespace Ripes {
namespace Assembler {

// pcrel_lo/hi are restricted to 32-bit absolute addresses, so keep computations
// in this base.
inline uint32_t pcrel_hi16(const uint32_t val, const uint32_t reloc_addr) {
  return ((val - (reloc_addr % 0xFFFFF000) + 0x800) >> 16);
}

template <typename Reg_T>
Relocation<Reg_T> mips_pcrel_hi() {
  return Relocation<Reg_T>("%pcrel_hi16",
                           [](const Reg_T val, const Reg_T reloc_addr)
                               -> HandleRelocationRes<Reg_T> {
                             const uint32_t _hi16 = pcrel_hi16(val, reloc_addr);
                             return {_hi16};
                           });
}

template <typename Reg_T>
Relocation<Reg_T> mips_pcrel_lo() {
  return Relocation<Reg_T>(
      "%pcrel_lo16",
      [](const Reg_T val,
         const Reg_T reloc_addr) -> HandleRelocationRes<Reg_T> {
        using Reg_T_S = typename std::make_signed<Reg_T>::type;
        const uint32_t _hi16 = pcrel_hi16(val, reloc_addr);
        const uint32_t lo16 = val & 0x0000FFF ;
        return {static_cast<Reg_T>(
            static_cast<Reg_T_S>(vsrtl::signextend(lo16, 16)))};
      });
}

template <typename Reg_T>
Relocation<Reg_T> mips_hi() {
  return Relocation<Reg_T>(
      "%hi",
      [](const Reg_T val, const Reg_T /*reloc_addr*/)
          -> HandleRelocationRes<Reg_T> { return {val >> 16 & 0xFFFFFF}; });
}

template <typename Reg_T>
Relocation<Reg_T> mips_lo() {
  return Relocation<Reg_T>(
      "%lo",
      [](const Reg_T val, const Reg_T /*reloc_addr*/)
          -> HandleRelocationRes<Reg_T> { return {val & 0xFFF}; });
}

/** @brief
 * A collection of RISC-V assembler relocations
 */

template <typename Reg_T>
RelocationsVec<Reg_T> mipsRelocations() {
  RelocationsVec<Reg_T> relocations;

  relocations.push_back(
      std::make_shared<Relocation<Reg_T>>(mips_pcrel_hi<Reg_T>()));
  relocations.push_back(
      std::make_shared<Relocation<Reg_T>>(mips_pcrel_lo<Reg_T>()));
  relocations.push_back(std::make_shared<Relocation<Reg_T>>(mips_hi<Reg_T>()));
  relocations.push_back(std::make_shared<Relocation<Reg_T>>(mips_lo<Reg_T>()));

  return relocations;
}
} // namespace Assembler
} // namespace Ripes
