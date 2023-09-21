#pragma once

#include "../mips.h"
#include "../mips_decode.h"
#include "../mips_uncompress.h"
#include "VSRTL/core/vsrtl_component.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class MIPS_DecodeC : public Component {
public:
  void setISA(const std::shared_ptr<ISAInfoBase> &isa) {
    decode->setISA(isa);
    uncompress->setISA(isa);
  }

  MIPS_DecodeC(std::string name, SimComponent *parent) : Component(name, parent) {
    instr >> uncompress->instr;

    uncompress->Pc_Inc >> Pc_Inc;
    uncompress->exp_instr >> decode->instr;
    uncompress->exp_instr >> exp_instr;

    decode->opcode >> opcode;
    decode->instr_address_25_0 >> instr_address_25_0;
    decode->instr_opc_31_26 >> instr_opc_31_26;
    decode->instr_rs_25_21 >> instr_rs_25_21;
    decode->instr_rt_20_16 >> instr_rt_20_16;
    decode->instr_rd_15_11 >> instr_rd_15_11;
    decode->instr_imm_15_0 >> instr_imm_15_0;
    decode->instr_imm_5_0 >> instr_imm_5_0;
    decode->instr_shamt_10_6 >> instr_shamt_10_6;

    decode->mf_sel >> mf_sel;
    decode->mt_sel >> mt_sel;
    decode->mult_div_sel >> mult_div_sel;
    decode->j_sel >> j_sel;

    decode->wr_reg_idx >> wr_reg_idx;

  }

  SUBCOMPONENT(decode, TYPE(MIPS_Decode<XLEN>));
  SUBCOMPONENT(uncompress, TYPE(MIPS_Uncompress<XLEN>));

  INPUTPORT(instr, c_MIPSInstrWidth);
  OUTPUTPORT_ENUM(opcode, MIPS_Instr);
  OUTPUTPORT(instr_address_25_0, 26);
  OUTPUTPORT(instr_opc_31_26, c_MIPSRegsBits);
  OUTPUTPORT(instr_rs_25_21, c_MIPSRegsBits);
  OUTPUTPORT(instr_rt_20_16, c_MIPSRegsBits);
  OUTPUTPORT(instr_rd_15_11, c_MIPSRegsBits);
  OUTPUTPORT(instr_imm_15_0, 16);
  OUTPUTPORT(instr_shamt_10_6, c_MIPSRegsBits);
  OUTPUTPORT(instr_imm_5_0, c_MIPSRegsBits);
  OUTPUTPORT(wr_reg_idx, c_MIPSRegsBits);
  OUTPUTPORT(mf_sel, 2);
  OUTPUTPORT(mt_sel, 2);
  OUTPUTPORT(mult_div_sel, 1);
  OUTPUTPORT(j_sel, 2);



  OUTPUTPORT(Pc_Inc, 1);                                //MIPS_Uncompress
  OUTPUTPORT(exp_instr, c_MIPSInstrWidth);              //MIPS_Uncompress
};

} // namespace core
} // namespace vsrtl
