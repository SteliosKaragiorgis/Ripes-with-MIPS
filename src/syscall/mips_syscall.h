#pragma once

#include "processorhandler.h"
#include "ripes_syscall.h"

// Syscall headers
#include "control.h"
#include "print.h"
#include "read.h"
#include "syscall_time.h"

#include "isa/mipsisainfo_common.h"

namespace Ripes {

class MIPSSyscall : public Syscall {
public:
  MIPSSyscall(const QString &name, const QString &description = QString(),
              const std::map<ArgIdx, QString> &argumentDescriptions =
                  std::map<ArgIdx, QString>(),
              const std::map<ArgIdx, QString> &returnDescriptions =
                  std::map<ArgIdx, QString>())
      : Syscall(name, description, argumentDescriptions, returnDescriptions) {}
  ~MIPSSyscall() override {}

  VInt getArg(RegisterFileType rfid, ArgIdx i) const override {
    assert(i < 7);
    const int regIdx = 4 + i; // a0 = x4
    return ProcessorHandler::getRegisterValue(rfid, regIdx);
  }

  void setRet(RegisterFileType rfid, ArgIdx i, VInt value) const override {
    assert(i < 7);
    const int regIdx = 2 + i; // v0 = x2
    ProcessorHandler::setRegisterValue(rfid, regIdx, value);
  }
};

class MIPSSyscallManager : public SyscallManagerT<MIPSSyscall> {
public:
  MIPSSyscallManager() {
    // Print syscalls
    emplace<PrintIntSyscall<MIPSSyscall>>(MIPSABI::PrintInt);
    emplace<PrintStrSyscall<MIPSSyscall>>(MIPSABI::PrintStr);
    emplace<PrintCharMipsSyscall<MIPSSyscall>>(MIPSABI::PrintChar);
    emplace<PrintHexSyscall<MIPSSyscall>>(MIPSABI::PrintIntHex);
    emplace<PrintBinarySyscall<MIPSSyscall>>(MIPSABI::PrintIntBinary);
    emplace<PrintUnsignedSyscall<MIPSSyscall>>(MIPSABI::PrintIntUnsigned);

    // Control syscalls
    emplace<ExitMipsSyscall<MIPSSyscall>>(MIPSABI::Exit);
    emplace<Exit2Syscall<MIPSSyscall>>(MIPSABI::Exit2);
    emplace<BrkSyscall<MIPSSyscall>>(MIPSABI::Sbrk);

    // Read syscalls
    emplace<ReadIntSyscall<MIPSSyscall>>(MIPSABI::ReadInt);
    emplace<ReadStringSyscall<MIPSSyscall>>(MIPSABI::ReadString);

    // Time syscalls
  }
};

} // namespace Ripes
