#pragma once

#include "../mips.h"
#include "../mips_decode_mms.h"
#include "../mips_uncompress.h"
#include "../mips_uncompress.h"
#include "../mips_state_mms.h"
#include "VSRTL/core/vsrtl_component.h"
#include "../mips_state_register.h"
#include "VSRTL/core/vsrtl_logicgate.h"
#include "VSRTL/core/vsrtl_multiplexer.h"

#include "../mips_state_mms.h"
#include "../mips_next_state_mms.h"
#include "../mips_opcode_mms.h"
#include "../mips_opcode_mms.h"
#include "../mips_instr_reg_en_mms.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN>
class MIPS_DecodeMMS : public Component {
public:
  void setISA(const std::shared_ptr<ISAInfoBase> &isa) {
    decode->setISA(isa);
    uncompress->setISA(isa);
  }

  void initialState(){
      decode->initialState();
  }

  MIPS_DecodeMMS(std::string name, SimComponent *parent) : Component(name, parent) {
      const auto pcValue = pc.uValue() & 0b1;

      0 >> state_reg->clear;
      pcValue >> state_reg->enable;
      decode->next_state >> state_reg->state_reg_in;
      state_reg->state_reg_out >> decode->state_in;

    instr_show >> uncompress->instr;

    uncompress->Pc_Inc >> Pc_Inc;
    opcode_calc->instr_out >> decode->instr;
    uncompress->exp_instr >> exp_instr;

    opcode_calc->opcode_out >> opcode;

    decode->instr_address_25_0 >> instr_address_25_0;
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


    instr_show >> opcode_calc->instr;
    instr >> opcode_calc->instr_mem;

    opcode_calc->opcode_out >> next_state_calc->opcode;
    state_calc->state_out >> next_state_calc->state_in;


    next_state_calc->next_state >> state_calc_reg->state_reg_in;
    1 >> state_calc_reg->enable;
    0 >> state_calc_reg->clear;
    state_calc_reg->state_reg_out >> state_calc->state;

    state_calc->state_out >> opcode_calc->state;
    state_calc->state_out >> state;

    state_calc->state_out >> reg_en->state_in;
    reg_en->instr_reg_enable >> instr_reg_enable;

  }
  SUBCOMPONENT(opcode_calc, MIPS_Opcode_MMS);
  SUBCOMPONENT(state_calc, TYPE(MIPS_State_MMS));                //state
  SUBCOMPONENT(next_state_calc, TYPE(MIPS_Next_State_MMS));    //next state
  SUBCOMPONENT(reg_en, TYPE(MIPS_Reg_En_MMS));              //instr register enable
  SUBCOMPONENT(state_calc_reg, TYPE(MIPS_STATE_REGISTER<XLEN>));


  SUBCOMPONENT(state_reg, TYPE(MIPS_STATE_REGISTER<XLEN>));

  SUBCOMPONENT(decode, TYPE(MIPS_Decode_MMS<XLEN>));

  SUBCOMPONENT(uncompress, TYPE(MIPS_Uncompress<c_MIPSInstrWidth>));
  INPUTPORT(pc, c_MIPSInstrWidth);
  INPUTPORT(instr_show, c_MIPSInstrWidth);
  INPUTPORT(irwrite, 1);
  INPUTPORT(instr, c_MIPSInstrWidth);
  INPUTPORT(instr_s0, c_MIPSInstrWidth);
  INPUTPORT_ENUM(instr_sel, MIPSMulti_Instr);
  OUTPUTPORT_ENUM(opcode, MIPS_Instr);
  OUTPUTPORT_ENUM(state, MIPSMulti_States);
  OUTPUTPORT(instr_address_25_0, 26);
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
  OUTPUTPORT(instr_reg_enable, 1);


  OUTPUTPORT(Pc_Inc, 1);                                //MIPS_Uncompress
  OUTPUTPORT(exp_instr, c_MIPSInstrWidth);              //MIPS_Uncompress
};

} // namespace core
} // namespace vsrtl
