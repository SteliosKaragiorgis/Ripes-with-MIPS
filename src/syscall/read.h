#pragma once

#include <type_traits>

#include "processorhandler.h"
#include "ripes_syscall.h"
#include "systemio.h"

namespace Ripes {

template <typename BaseSyscall>
class ReadIntSyscall : public BaseSyscall {
  static_assert(std::is_base_of<Syscall, BaseSyscall>::value);

public:
  ReadIntSyscall()
      : BaseSyscall("ReadInt", "Read integer", {{0, "integer to print"}},
                    {{0, "contains integer read"}}) {}
  void execute() {
    const int fd = 0;
    int byteAddress = BaseSyscall::getArg(RegisterFileType::GPR, 1);
    QByteArray buffer;

    int retLength = SystemIO::readFromFile(fd, buffer, 255);

    bool ok;
    BaseSyscall::setRet(RegisterFileType::GPR, 0, buffer.toInt(&ok, 10));
  }
};

template <typename BaseSyscall>
class ReadStringSyscall : public BaseSyscall {
  static_assert(std::is_base_of<Syscall, BaseSyscall>::value);

public:
  ReadStringSyscall()
      : BaseSyscall("ReadString", "Read from a file descriptor into a buffer",
                    {{0, "address of null-terminated string to print"},
                     {1, "address of the buffer"}}) {}
  void execute() {
    const int fd = 0;
    int byteAddress = BaseSyscall::getArg(
        RegisterFileType::GPR, 0); // destination of characters read from file
    const int length = BaseSyscall::getArg(RegisterFileType::GPR, 1);
    QByteArray buffer;

    int retLength = SystemIO::readFromFile(fd, buffer, length);
    BaseSyscall::setRet(RegisterFileType::GPR, 0, retLength);

    if (retLength != -1) {
      // copy bytes from returned buffer into memory
      const char *dataptr =
          buffer.constData(); // QString::data contains a possible null
                              // termination '\0' character (present if reading
                              // from stdin and not from a file)
      while (retLength-- > 0) {
        ProcessorHandler::writeMem(byteAddress++, *dataptr++, sizeof(char));
      }
    }
  }
};

} // namespace Ripes
