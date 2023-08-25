#include "mips32i_assembler.h"
#include "gnudirectives.h"
#include "mipsrelocations.h"
#include "ripessettings.h"

#include <QByteArray>
#include <algorithm>

#include "rv_c_ext.h"
#include "rv_m_ext.h"

#include "mips_i_ext.h"

namespace Ripes {
namespace Assembler {

MIPS32I_Assembler::MIPS32I_Assembler(const ISAInfo<ISA::MIPS32I> *isa)
    : Assembler<Reg_T>(isa) {
  auto [instrs, pseudos] = initInstructions(isa);

  auto directives = gnuDirectives();
  auto relocations = mipsRelocations<Reg_T>();
  initialize(instrs, pseudos, directives, relocations);

  // Initialize segment pointers and monitor settings changes to segment
  // pointers
  connect(RipesSettings::getObserver(RIPES_SETTING_ASSEMBLER_TEXTSTART),
          &SettingObserver::modified, this, [this](const QVariant &value) {
            setSegmentBase(".text", value.toULongLong() /*4194304*/);
          });
  RipesSettings::getObserver(RIPES_SETTING_ASSEMBLER_TEXTSTART)->trigger();
  connect(RipesSettings::getObserver(RIPES_SETTING_ASSEMBLER_DATASTART),
          &SettingObserver::modified, this, [this](const QVariant &value) {
            setSegmentBase(".data", /*value.toULongLong()*/ 268500992);
          });
  RipesSettings::getObserver(RIPES_SETTING_ASSEMBLER_DATASTART)->trigger();
  connect(RipesSettings::getObserver(RIPES_SETTING_ASSEMBLER_BSSSTART),
          &SettingObserver::modified, this, [this](const QVariant &value) {
            setSegmentBase(".bss", value.toULongLong());
          });
  RipesSettings::getObserver(RIPES_SETTING_ASSEMBLER_BSSSTART)->trigger();
}

std::tuple<MIPS32I_Assembler::_InstrVec, MIPS32I_Assembler::_PseudoInstrVec>
MIPS32I_Assembler::initInstructions(const ISAInfo<ISA::MIPS32I> *isa) const {
  _InstrVec instructions;
  _PseudoInstrVec pseudoInstructions;

  MIPS_I<Reg_T>::enable(isa, instructions, pseudoInstructions);
  for (const auto &extension : isa->enabledExtensions()) {
    switch (extension.unicode()->toLatin1()) {
    case 'M':
      RV_M<Reg_T>::enable(isa, instructions, pseudoInstructions);
      break;
    case 'C':
      RV_C<Reg_T>::enable(isa, instructions, pseudoInstructions);
      break;
    default:
      assert(false && "Unhandled ISA extension");
    }
  }
  return {instructions, pseudoInstructions};
}

} // namespace Assembler
} // namespace Ripes
