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

protected:
  void setup(IFrontEnd*, IMemorySystem* memory_system) {
    m_dram = memory_system->get_ifce<IDRAM>();

    const auto& count = m_dram->m_organization.count;
    m_num_levels = static_cast<int>(count.size());
    m_addr_bits.resize(m_num_levels);

    for (int level = 0; level < m_num_levels; level++) {
      m_addr_bits[level] = calc_log2(count[level]);
    }

    // sanity: row/column 必须存在
    (void)m_dram->m_levels("row");
    (void)m_dram->m_levels("column");
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
  void init() override {}

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    ReRAMLinearMapperBase::setup(frontend, memory_system);
  }

  void apply(Request& req) override {
    req.addr_vec.resize(m_num_levels, -1);

    // bit-address: 不再做 tx/byte 对齐
    Addr_t addr = req.addr;

    // MASK 特例：低 32 位是 mask_value，不参与层级切片
    // （mask_value 由 controller 从 req.addr 自行解析）
    if (req.type_id == Request::Type::PIM_MASK||req.type_id == Request::Type::PIM_WRITE_MUL) {
      addr = (addr >> 32);
    }

    // 从最低层开始切片（column 是最后一层）
    for (int level = m_num_levels - 1; level >= 0; --level) {
      req.addr_vec[level] = slice_lower_bits(addr, m_addr_bits[level]);
    }
  }
};

} // namespace Ramulator
