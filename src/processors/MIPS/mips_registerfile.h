#pragma once

#include "VSRTL/core/vsrtl_constant.h"
#include "VSRTL/core/vsrtl_memory.h"
#include "VSRTL/core/vsrtl_wire.h"
#include "VSRTL/core/vsrtl_multiplexer.h"

#include "mips.h"
#include "mips_reg_extended.h"

namespace vsrtl {
namespace core {
using namespace Ripes;

template <unsigned XLEN, bool readBypass>
class MIPS_RegisterFile : public Component {
public:
  SetGraphicsType(ClockedComponent);
  MIPS_RegisterFile(const std::string &name, SimComponent *parent)
      : Component(name, parent) {
    // Writes

    // Disable writes to register 0
    wr_en_0->setSensitiveTo(wr_en);
    wr2_en_0->setSensitiveTo(wr_en);

    wr_en_0->out << [=] {
      int ret = wr_en.uValue() && wr_addr.uValue() != 0;
      switch(opcode.uValue()){
        case MIPS_Instr::MTHI:
        case MIPS_Instr::MTLO:
        case MIPS_Instr::MULT:
        case MIPS_Instr::MULTU:
        case MIPS_Instr::DIV:
        case MIPS_Instr::DIVU:
        case MIPS_Instr::JAL:
        case MIPS_Instr::JALR:
          return 1;
        default:
          return ret;
      }
    };

    wr2_en_0->out << [=] {
      switch(opcode.uValue()){
        case MIPS_Instr::MULT:
        case MIPS_Instr::MULTU:
        case MIPS_Instr::DIV:
        case MIPS_Instr::DIVU:
          return 1;
        default:
          return 0;
      }
    };


    mt_sel >> _write_reg->select;
    reg_extend->wr_addr >> _write_reg->get(MIPS_Mt::IN1);
    0b100000 >> _write_reg->get(MIPS_Mt::IN2);    //hi
    0b100001 >> _write_reg->get(MIPS_Mt::IN3);    //lo
    _write_reg->out >> _wr_mem->addr;


    mult_div_sel >> _write2_reg->select;
    0b100001 >> _write2_reg->get(MIPS_Mult_Div::IN1);
    0b100000 >> _write2_reg->get(MIPS_Mult_Div::IN2);    //mult, multu, div, divu
    _write2_reg->out >> _wr2_mem->addr;

    wr2_en_0->out >> _wr2_mem->wr_en;
    hi_in >> _wr2_mem->data_in;
    (XLEN / CHAR_BIT) >> _wr2_mem->wr_width;




     mf_sel >> _reg_addr->select;
     reg_extend->r1_addr >> _reg_addr->get(MIPS_Mf::IN1);
     0b100000 >> _reg_addr->get(MIPS_Mf::IN2);   //hi
     0b100001 >> _reg_addr->get(MIPS_Mf::IN3);   //lo
     _reg_addr->out >> _rd1_mem->addr;



     r1_addr >> reg_extend->reg1;
     r2_addr >> reg_extend->reg2;





    wr_en_0->out >> _wr_mem->wr_en;

    j_sel >> _write_data->select;
    data_in >> _write_data->get(MIPS_WrData::IN1);
    pc_4 >> _write_data->get(MIPS_WrData::IN2);
    pc_4 >> _write_data->get(MIPS_WrData::IN3);
    _write_data->out >> _wr_mem->data_in;

    j_sel >> _write_reg_internal->select;
    wr_addr >> _write_reg_internal->get(MIPS_WrRegInternal::IN1);
    31 >> _write_reg_internal->get(MIPS_WrRegInternal::IN2);
    wr_addr >> _write_reg_internal->get(MIPS_WrRegInternal::IN3);
    _write_reg_internal->out >> reg_extend->wreg;



    (XLEN / CHAR_BIT) >> _wr_mem->wr_width;

    reg_extend->r2_addr >> _rd2_mem->addr;

    /** If read bypassing is enabled, we may read the next-state register value
     * in the current state. Also note that, given that the RegisterFile is of
     * type Component, all inputs must be propagated before outputs are
     * propagated. Thus, we are sure to have received the next-state write
     * address when we clock the output ports. This would >not< be the case if
     * the RegisterFile was a clocked component.
     */
    if constexpr (readBypass) {
      r1_out << [=] {
        if(mf_sel.uValue() != 0) {
            if(opcode.uValue() == MIPS_Instr::DIV ||
               opcode.uValue() == MIPS_Instr::DIVU ||
               opcode.uValue() == MIPS_Instr::MULT ||
               opcode.uValue() == MIPS_Instr::MULTU ||
               opcode.uValue() == MIPS_Instr::MTHI ||
               opcode.uValue() == MIPS_Instr::MTLO)
                    return data_in.uValue();
            else{
                return _rd1_mem->data_out.uValue();
            }

        }

        const int rd_idx = r1_addr.uValue();
        if (rd_idx == 0) {
          return VT_U(0);
        }

        const unsigned wr_idx = wr_addr.uValue();
        if (wr_en.uValue() && wr_idx == r1_addr.uValue()) {
          return data_in.uValue();
        } else {
          return _rd1_mem->data_out.uValue();
        }
      };

      r2_out << [=] {
        const unsigned rd_idx = r2_addr.uValue();
        if (rd_idx == 0) {
          return VT_U(0);
        }

        const unsigned wr_idx = wr_addr.uValue();
        if (wr_en.uValue() && wr_idx == r2_addr.uValue()) {
          return data_in.uValue();
        } else {
          return _rd2_mem->data_out.uValue();
        }
      };
    } else {
      _rd1_mem->data_out >> r1_out;
      _rd2_mem->data_out >> r2_out;
    }
  }

  SUBCOMPONENT(reg_extend, TYPE(MIPS_Reg_Extended<XLEN>));

  SUBCOMPONENT(_reg_addr, TYPE(EnumMultiplexer<MIPS_Mf, c_MIPSRegsBits+1>));
  SUBCOMPONENT(_write_data, TYPE(EnumMultiplexer<MIPS_WrData, XLEN>));
  SUBCOMPONENT(_write_reg_internal, TYPE(EnumMultiplexer<MIPS_WrRegInternal, c_MIPSRegsBits>));
  SUBCOMPONENT(_write_reg, TYPE(EnumMultiplexer<MIPS_Mt, c_MIPSRegsBits+1>));
  SUBCOMPONENT(_write2_reg, TYPE(EnumMultiplexer<MIPS_Mult_Div, c_MIPSRegsBits+1>));

  SUBCOMPONENT(_wr_mem, TYPE(WrMemory<c_MIPSRegsBits+1, XLEN, false>));
  SUBCOMPONENT(_wr2_mem, TYPE(WrMemory<c_MIPSRegsBits+1, XLEN, false>));
  SUBCOMPONENT(_rd1_mem, TYPE(RdMemory<c_MIPSRegsBits+1, XLEN, false>));
  SUBCOMPONENT(_rd2_mem, TYPE(RdMemory<c_MIPSRegsBits+1, XLEN, false>));

  INPUTPORT(r1_addr, c_MIPSRegsBits);
  INPUTPORT(r2_addr, c_MIPSRegsBits);
  INPUTPORT(wr_addr, c_MIPSRegsBits);

  INPUTPORT(pc_4, XLEN);
  INPUTPORT(j_sel,2);
  INPUTPORT(mf_sel, 2);
  INPUTPORT(mt_sel, 2);
  INPUTPORT(mult_div_sel, 1);
  INPUTPORT(hi_in, XLEN);
  INPUTPORT_ENUM(opcode, MIPS_Instr);

  INPUTPORT(data_in, XLEN);
  WIRE(wr_en_0, 1);
  WIRE(wr2_en_0, 1);
  INPUTPORT(wr_en, 1);
  OUTPUTPORT(r1_out, XLEN);
  OUTPUTPORT(r2_out, XLEN);

  VSRTL_VT_U getRegister(unsigned i) const {
    return m_memory->readMemConst(i << ceillog2(XLEN / CHAR_BIT),
                                  XLEN / CHAR_BIT);
  }

  std::vector<VSRTL_VT_U> getRegisters() {
    std::vector<VSRTL_VT_U> regs;
    for (int i = 0; i < c_MIPSRegs; ++i)
      regs.push_back(getRegister(i));
    return regs;
  }

  void setMemory(AddressSpace *mem) {
    m_memory = mem;
    // All memory components must point to the same memory
    _wr_mem->setMemory(m_memory);
    _wr2_mem->setMemory(m_memory);
    _rd1_mem->setMemory(m_memory);
    _rd2_mem->setMemory(m_memory);
  }

private:
  AddressSpace *m_memory = nullptr;
};

} // namespace core
} // namespace vsrtl
