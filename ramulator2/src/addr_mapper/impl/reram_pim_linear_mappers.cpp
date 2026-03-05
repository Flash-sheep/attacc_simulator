#include <algorithm>
#include <vector>

#include "base/base.h"
#include "dram/dram.h"
#include "addr_mapper/addr_mapper.h"
#include "memory_system/memory_system.h"

namespace Ramulator {

/**
 * ReRAM 线性映射（bit-address 版本）
 *
 * 约定：trace 里传入的地址是“bit 级”地址（精确到 row/column 的 bit），不再是 byte address。
 *
 * 默认地址布局：从低位到高位依次切片给
 *   column <- row <- array <- ag <- pe <- tile <- channel
 *
 * 特例：PIM_MASK
 *   用户约定 MASK 的地址由 (base_addr | mask_value) 组成：
 *     - mask_value: 低 32 位，长度 32，对应一个 block(32 行) 的行选通（0 选通，1 不选通）
 *     - base_addr : 高位部分，仍然是 bit-address，用于指定层级位置（至少到 array / block）
 *   因此对 PIM_MASK，我们对 addr >> 32 做层级切片。
 */
class ReRAMLinearMapperBase : public IAddrMapper {
public:
  IDRAM* m_dram = nullptr;

  int m_num_levels = -1;
  std::vector<int> m_addr_bits;
  int m_ch_level = -1;
  int m_tile_level = -1;
  int m_pe_level = -1;
  int m_ag_level = -1;
  int m_array_level = -1;
  int m_row_level = -1;
  int m_col_level = -1;
  int m_rows_per_block = 32;

protected:
  void setup(IFrontEnd*, IMemorySystem* memory_system) {
    m_dram = memory_system->get_ifce<IDRAM>();

    const auto& count = m_dram->m_organization.count;
    m_num_levels = static_cast<int>(count.size());
    m_addr_bits.resize(m_num_levels);

    for (int level = 0; level < m_num_levels; level++) {
      m_addr_bits[level] = calc_log2(count[level]);
    }

    m_ch_level    = m_dram->m_levels("channel");
    m_tile_level  = m_dram->m_levels("tile");
    m_pe_level    = m_dram->m_levels("pe");
    m_ag_level    = m_dram->m_levels("ag");
    m_array_level = m_dram->m_levels("array");
    m_row_level   = m_dram->m_levels("row");
    m_col_level   = m_dram->m_levels("column");
  }
};

class ReRAMMap final : public ReRAMLinearMapperBase, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IAddrMapper,
      ReRAMMap,
      "ReRAM-Linear",
      "Linear mapping for ReRAM with bit-level address: (Ch, Tile, PE, AG, Array, Row, Col)."
  );

public:
  static constexpr uint64_t kBSBitPos  = 62;
  static constexpr uint64_t kBSBitMask = (1ULL << kBSBitPos);

  void init() override {}

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    ReRAMLinearMapperBase::setup(frontend, memory_system);
  }

  void apply(Request& req) override {
    req.addr_vec.resize(m_num_levels, -1);
    if (req.isa_decoded) {
      auto clamp_mod = [](int val, int dim) -> int {
        if (dim <= 0) return 0;
        int x = val % dim;
        return (x < 0) ? (x + dim) : x;
      };
      const int rows = m_dram->m_organization.count[m_row_level];
      const int cols = m_dram->m_organization.count[m_col_level];

      req.addr_vec[m_ch_level]    = clamp_mod(req.isa_channel, m_dram->m_organization.count[m_ch_level]);
      req.addr_vec[m_tile_level]  = clamp_mod(req.isa_bg,      m_dram->m_organization.count[m_tile_level]);
      req.addr_vec[m_pe_level]    = clamp_mod(req.isa_bank,    m_dram->m_organization.count[m_pe_level]);
      req.addr_vec[m_ag_level]    = clamp_mod(req.isa_ag,      m_dram->m_organization.count[m_ag_level]);
      req.addr_vec[m_array_level] = clamp_mod(req.isa_array,   m_dram->m_organization.count[m_array_level]);

      int row = 0;
      if (req.isa_bs == 1) {
        int row_in_block = 0;
        if (req.type_id == Request::Type::PIM_WRITE_SINGLE ||
            req.type_id == Request::Type::PIM_MV_SINGLE ||
            req.type_id == Request::Type::PIM_RD_SINGLE) {
          row_in_block = std::max(0, req.isa_row);
        }
        row = req.isa_block * m_rows_per_block + row_in_block;
      }
      req.addr_vec[m_row_level] = clamp_mod(row, rows);

      int col = 0;
      switch (req.type_id) {
        case Request::Type::PIM_SET:
        case Request::Type::PIM_NOR:
          col = req.isa_input1;
          break;
        case Request::Type::PIM_WRITE_SINGLE:
        case Request::Type::PIM_MV_SINGLE:
        case Request::Type::PIM_RD_SINGLE:
        case Request::Type::PIM_MUL_RD:
          col = std::max(0, req.isa_word_col) * 16;
          break;
        case Request::Type::PIM_WRITE_MUL:
          col = req.isa_block * 32;
          break;
        default:
          col = 0;
          break;
      }
      req.addr_vec[m_col_level] = clamp_mod(col, cols);
      return;
    }

    // bit-address: 不再做 tx/byte 对齐
    uint64_t addr = static_cast<uint64_t>(req.addr);

    // BS 特例：用于描述访问范围，不参与层级切片
    if (req.type_id == Request::Type::PIM_MUL_RD ||
        req.type_id == Request::Type::PIM_RD_ALL ||
        req.type_id == Request::Type::PIM_WRITE_MUL) {
      addr &= ~kBSBitMask;
    }

    // MASK 特例：低 32 位是 mask_value，不参与层级切片
    // （mask_value 由 controller 从 req.addr 自行解析）
    if (req.type_id == Request::Type::PIM_MASK || req.type_id == Request::Type::PIM_WRITE_MUL) {
      addr = (addr >> 32);
    }

    // 从最低层开始切片（column 是最后一层）
    for (int level = m_num_levels - 1; level >= 0; --level) {
      req.addr_vec[level] = slice_lower_bits(addr, m_addr_bits[level]);
    }
  }
};

} // namespace Ramulator
