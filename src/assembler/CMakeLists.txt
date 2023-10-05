create_ripes_lib(assembler LINK_TO_RIPES_LIB)

target_sources(assembler_lib
  PRIVATE
    mips32i_assembler.h mips32i_assembler.cpp
    mipsrelocations.h
    mips_i_ext.h
    mipsassembler_common.h
)
