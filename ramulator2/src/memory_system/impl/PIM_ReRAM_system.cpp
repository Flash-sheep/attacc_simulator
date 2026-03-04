#include "memory_system/memory_system.h"
#include "translation/translation.h"
#include "dram_controller/controller.h"
#include "addr_mapper/addr_mapper.h"
#include "dram/dram.h"

namespace Ramulator {

class ReRAMSystem final : public IMemorySystem, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IMemorySystem,
      ReRAMSystem,
      "ReRAM",                  // 在 config 里用 "ReRAM" 这个名字
      "A ReRAM-PIM memory system."
  );

protected:
  Clk_t m_clk = 0;
  IDRAM*        m_dram        = nullptr;
  IAddrMapper*  m_addr_mapper = nullptr;
  std::vector<IDRAMController*> m_controllers;

public:
  // ===== 统计信息 =====
  int s_num_read_requests      = 0;
  int s_num_write_requests     = 0;
  int s_num_pim_nor_requests   = 0;
  int s_num_pim_set_requests   = 0;
  int s_num_other_requests     = 0;
  int s_num_pim_mask_requests  = 0;
  int s_num_mul_rd_requests    = 0;
  int s_num_mv_single_requests = 0;
  int s_num_rd_single_requests = 0;
  int s_num_rd_all_requests    = 0;


public:
  void init() override {
    // 1. 创建设备（你的 ReRAM::init 会被调用）
    m_dram        = create_child_ifce<IDRAM>();
    m_addr_mapper = create_child_ifce<IAddrMapper>();

    // 2. 根据 "channel" 维度建 controller
    int channel_level = m_dram->m_levels("channel");
    int num_channels  = m_dram->get_level_size("channel");

    for (int i = 0; i < num_channels; i++) {
      IDRAMController* controller = create_child_ifce<IDRAMController>();
      controller->m_impl->set_id(fmt::format("Channel {}", i));
      controller->m_channel_id = i;
      m_controllers.push_back(controller);
    }

    // 3. 时钟比例（和 DRAM 版一致）
    m_clock_ratio = param<uint>("clock_ratio").required();

    // 4. 注册统计量
    register_stat(m_clk).name("memory_system_cycles");
    register_stat(s_num_read_requests).name("reram_num_read_requests");
    register_stat(s_num_write_requests).name("reram_num_write_requests");
    register_stat(s_num_pim_nor_requests).name("reram_num_pim_nor_requests");
    register_stat(s_num_pim_set_requests).name("reram_num_pim_set_requests");
    register_stat(s_num_pim_mask_requests).name("reram_num_pim_mask_requests");
    register_stat(s_num_mul_rd_requests).name("reram_num_mul_rd_requests");
    register_stat(s_num_mv_single_requests).name("reram_num_mv_single_requests");
    register_stat(s_num_rd_single_requests).name("reram_num_rd_single_requests");
    register_stat(s_num_rd_all_requests).name("reram_num_rd_all_requests");
    register_stat(s_num_other_requests).name("reram_num_other_requests");
  };

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    // 目前 ReRAMSystem 不需要额外 setup
    (void)frontend;
    (void)memory_system;
  }

  bool send(Request req) override {
    // 1. 地址映射：填好 req.addr_vec
    m_addr_mapper->apply(req);

    // 2. 取出 channel 号
    // 建议不要硬写 addr_vec[0]，而是用层级索引更稳妥
    int ch_level  = m_dram->m_levels("channel");
    int channel_id = static_cast<int>(req.addr_vec[ch_level]);

    bool is_success = m_controllers[channel_id]->send(req);

    // 3. 统计请求类型
    if (is_success) {
      switch (req.type_id) {
        case Request::Type::Read: {
          s_num_read_requests++;
          break;
        }
        case Request::Type::Write: {
          s_num_write_requests++;
          break;
        }
        case Request::Type::PIM_NOR: {
          s_num_pim_nor_requests++;
          break;
        }
        case Request::Type::PIM_SET: {
          s_num_pim_set_requests++;
          break;
        }
        case Request::Type::PIM_MASK: {
          s_num_pim_mask_requests++;
          break;
        }
        case Request::Type::PIM_MUL_RD: {
          s_num_mul_rd_requests++;
          break;
        }
        case Request::Type::PIM_MV_SINGLE: {
          s_num_mv_single_requests++;
          break;
        }
        case Request::Type::PIM_RD_SINGLE: {
          s_num_rd_single_requests++;
          break;
        }
        case Request::Type::PIM_RD_ALL: {
          s_num_rd_all_requests++;
          break;
        }
        default: {
          s_num_other_requests++;
          break;
        }
      }
    }

    return is_success;
  };

  void tick() override {
    m_clk++;

    // 先推进设备内部时钟
    m_dram->tick();

    // 再推进每个 channel controller
    for (auto controller : m_controllers) {
      controller->tick();
    }
  };

  float get_tCK() override {
    // ReRAM::set_timing_vals 已经把 "tCK_ps" 写进 m_timing_vals 里
    return m_dram->m_timing_vals("tCK_ps") / 1000.0f;  // 转成 ns
  };

  bool is_pending() override {
    bool pending_any = false;
    for (auto controller : m_controllers) {
      pending_any |= controller->is_pending();
    }
    return pending_any;
  };
};

} // namespace Ramulator
