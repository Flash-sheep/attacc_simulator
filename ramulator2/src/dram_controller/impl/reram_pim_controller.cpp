#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "memory_system/memory_system.h"

#include <cstdint>
#include <vector>

namespace Ramulator {

class ReRAMController final : public IDRAMController, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IDRAMController,
      ReRAMController,
      "ReRAM",
      "A ReRAM-PIM controller with per-Arraygroup scheduling + simple block-mask/buffer modeling."
  );

private:
  // ===== 每个 Arraygroup 内部的“局部 controller” =====
  struct AGController {
    int ag_id = -1;

    ReqBuffer active_buffer;    // open-row path (ReRAM 不用也没关系)
    ReqBuffer priority_buffer;  // 高优先级
    ReqBuffer read_buffer;      // 普通读
    ReqBuffer write_buffer;     // 普通写
    ReqBuffer pim_buffer;       // PIM/扩展指令

    bool  is_write_mode     = false;
    float wr_low_watermark  = 0.2f;
    float wr_high_watermark = 0.8f;

    // ===== PIM busy + mask + buffer =====
    Clk_t pim_busy_until = 0;   // 用来模拟“片上 buffer / SA 读取 / 片上搬运”的额外开销

    int num_arrays = 0;
    static constexpr int kBlocksPerArray = 32; // 1024 rows / 32 rows-per-block

    // block mask：每个 array 的每个 block 一个 uint32
    // bit=0 -> 选通；bit=1 -> 不选通
    std::vector<uint32_t> block_masks;

    struct BufState {
      bool  valid        = false;
      int   array_id     = -1;
      bool  whole_array  = false;  // BS=0 -> whole array; BS=1 -> one block
      int   block_id     = -1;     // 仅在 whole_array=false 时有效
      int   fp16_col     = -1;     // buffer 中缓存的是哪个 FP16 列（col/16）
      Clk_t ready_at     = 0;
    } buf;

    inline void set_write_mode() {
      if (!is_write_mode) {
        if ((write_buffer.size() > wr_high_watermark * write_buffer.max_size) ||
            read_buffer.size() == 0) {
          is_write_mode = true;
        }
      } else {
        if ((write_buffer.size() < wr_low_watermark * write_buffer.max_size) &&
            read_buffer.size() != 0) {
          is_write_mode = false;
        }
      }
    }

    inline bool empty() const {
      return active_buffer.size()   == 0 &&
             priority_buffer.size() == 0 &&
             read_buffer.size()     == 0 &&
             write_buffer.size()    == 0 &&
             pim_buffer.size()      == 0 &&
             pending_reads          == 0;
    }

    // simple counter for is_pending()
    size_t pending_reads = 0;

    inline uint32_t& mask_ref(int array_id, int block_id) {
      return block_masks[array_id * kBlocksPerArray + block_id];
    }
  };

  // 一个 AG 选出的“候选命令”
  struct AGIssueCandidate {
    bool valid = false;
    ReqBuffer* buf = nullptr;
    ReqBuffer::iterator it;
  };

private:
  IDRAM*           m_dram      = nullptr;
  IScheduler*      m_scheduler = nullptr;
  IRefreshManager* m_refresh   = nullptr;

  std::vector<AGController> m_ag_ctrls;

  int m_ag_addr_idx    = -1; // "ag" in addr_vec
  int m_array_addr_idx = -1; // "array" in addr_vec
  int m_row_addr_idx   = -1; // "row" in addr_vec
  int m_col_addr_idx   = -1; // "column" in addr_vec

  int m_row_bits = 0;
  int m_col_bits = 0;

  std::deque<Request> pending;   // 等待返回 callback 的读

  int m_last_ag_issued = -1;

  std::vector<IControllerPlugin*> m_plugins;

  // ======= 额外开销参数（可从 YAML 配置覆盖） =======
  // 这些开销用于模拟：MASK 配置、MUL_RD 块读入 buffer、MV_SINGLE/ RD_SINGLE 等。
  int m_mask_cycles             = 1;
  int m_mul_rd_cycles_per_row   = 0;   // 如果为 0，setup 时默认用 DRAM nCL
  int m_mv_single_hit_cycles    = 1;
  int m_mv_single_miss_cycles   = 0;   // miss 时额外 SA 读一次
  int m_rd_single_bus_cycles    = 0;   // RD_SINGLE 通过总线传一个 FP16 的额外占用（建议用 nBL 或 1）

public:
  void init() override {
    // wr watermarks
    (void)param<float>("wr_low_watermark")
        .desc("Threshold for switching back to read mode.")
        .default_val(0.2f);
    (void)param<float>("wr_high_watermark")
        .desc("Threshold for switching to write mode.")
        .default_val(0.8f);

    // extra cycles
    m_mask_cycles = param<int>("mask_cycles")
        .desc("Extra cycles to model MASK configuration.")
        .default_val(1);

    m_mul_rd_cycles_per_row = param<int>("mul_rd_cycles_per_row")
        .desc("Extra cycles per selected row for MUL_RD / RD_ALL (SA+local buffer fill). If 0, defaults to DRAM nCL.")
        .default_val(0);

    m_mv_single_hit_cycles = param<int>("mv_single_hit_cycles")
        .desc("Cycles for MV_SINGLE when data is already in buffer.")
        .default_val(1);

    m_mv_single_miss_cycles = param<int>("mv_single_miss_cycles")
        .desc("Extra cycles for MV_SINGLE miss (need SA read once). If 0, defaults to mul_rd_cycles_per_row.")
        .default_val(0);

    m_rd_single_bus_cycles = param<int>("rd_single_bus_cycles")
        .desc("Extra cycles for RD_SINGLE bus transfer of one FP16.")
        .default_val(0);

    // scheduler / refresh
    m_scheduler = create_child_ifce<IScheduler>();
    m_refresh   = create_child_ifce<IRefreshManager>();

    // plugins
    if (m_config["plugins"]) {
      YAML::Node plugin_configs = m_config["plugins"];
      for (YAML::iterator it = plugin_configs.begin(); it != plugin_configs.end(); ++it) {
        m_plugins.push_back(create_child_ifce<IControllerPlugin>(*it));
      }
    }
  }

  void setup(IFrontEnd*, IMemorySystem* memory_system) override {
    m_dram = memory_system->get_ifce<IDRAM>();

    m_ag_addr_idx    = m_dram->m_levels("ag");
    m_array_addr_idx = m_dram->m_levels("array");
    m_row_addr_idx   = m_dram->m_levels("row");
    m_col_addr_idx   = m_dram->m_levels("column");

    // bits for BS extraction: bs = (base_addr >> (row_bits+col_bits)) & 1
    m_row_bits = calc_log2(m_dram->m_organization.count[m_row_addr_idx]);
    m_col_bits = calc_log2(m_dram->m_organization.count[m_col_addr_idx]);

    const int num_ag = m_dram->m_organization.count[m_ag_addr_idx];
    const int num_arrays = m_dram->m_organization.count[m_array_addr_idx];

    if (num_ag <= 0 || num_arrays <= 0) {
      throw std::runtime_error("ReRAMController: invalid organization counts.");
    }

    // default for mul_rd_cycles_per_row
    if (m_mul_rd_cycles_per_row == 0) {
      // 如果 device 没有 nCL，可以保底用 1
      m_mul_rd_cycles_per_row = std::max(1, m_dram->m_timing_vals("nCL"));
    }
    if (m_mv_single_miss_cycles == 0) {
      m_mv_single_miss_cycles = m_mul_rd_cycles_per_row;
    }
    if (m_rd_single_bus_cycles == 0) {
      // 默认把 RD_SINGLE 的 bus 占用当 1
      m_rd_single_bus_cycles = 1;
    }

    m_ag_ctrls.resize(num_ag);
    for (int i = 0; i < num_ag; ++i) {
      auto& ag = m_ag_ctrls[i];
      ag.ag_id = i;

      ag.read_buffer.max_size     = 128;
      ag.write_buffer.max_size    = 128;
      ag.pim_buffer.max_size      = 256;
      ag.priority_buffer.max_size = 64;
      ag.active_buffer.max_size   = 64;

      ag.wr_low_watermark  = 0.2f;
      ag.wr_high_watermark = 0.8f;

      ag.num_arrays = num_arrays;
      ag.block_masks.assign(num_arrays * AGController::kBlocksPerArray, 0u);
      ag.buf = {};
      ag.pim_busy_until = 0;
      ag.pending_reads = 0;
    }
  }

  // ==================== 请求入口 ====================

  bool send(Request& req) override {
    req.final_command = m_dram->m_request_translations(req.type_id);
    req.arrive        = m_clk;

    if (req.addr_vec.size() <= (size_t)m_ag_addr_idx) {
      throw std::runtime_error("ReRAMController: addr_vec too short for ag.");
    }
    int ag_id = static_cast<int>(req.addr_vec[m_ag_addr_idx]);
    if (ag_id < 0 || ag_id >= (int)m_ag_ctrls.size()) {
      throw std::runtime_error("ReRAMController: invalid ag id.");
    }
    auto& ag = m_ag_ctrls[ag_id];

    bool ok = false;
    switch (req.type_id) {
      case Request::Type::Read:
        ok = ag.read_buffer.enqueue(req);
        break;
      case Request::Type::Write:
        ok = ag.write_buffer.enqueue(req);
        break;

      case Request::Type::PIM_NOR:
      case Request::Type::PIM_SET:
      case Request::Type::PIM_MASK:
      case Request::Type::PIM_MUL_RD:
      case Request::Type::PIM_MV_SINGLE:
        ok = ag.pim_buffer.enqueue(req);
        break;

      case Request::Type::PIM_RD_SINGLE:
      case Request::Type::PIM_RD_ALL:
        // 这两类你也可以丢到 read_buffer；这里为了统一当作 PIM/片上路径
        ok = ag.pim_buffer.enqueue(req);
        break;

      default:
        ok = ag.priority_buffer.enqueue(req);
        break;
    }

    if (!ok) {
      req.arrive = -1;
      return false;
    }
    return true;
  }

  bool priority_send(Request& req) override {
    req.final_command = m_dram->m_request_translations(req.type_id);
    req.arrive        = m_clk;

    if (req.addr_vec.size() <= (size_t)m_ag_addr_idx) {
      throw std::runtime_error("ReRAMController: addr_vec too short for ag in priority_send.");
    }
    int ag_id = static_cast<int>(req.addr_vec[m_ag_addr_idx]);
    if (ag_id < 0 || ag_id >= (int)m_ag_ctrls.size()) {
      throw std::runtime_error("ReRAMController: invalid ag id in priority_send.");
    }

    auto& ag = m_ag_ctrls[ag_id];
    bool ok = ag.priority_buffer.enqueue(req);
    if (!ok) {
      req.arrive = -1;
      return false;
    }
    return true;
  }

  // ==================== tick：核心调度 ====================

  void tick() override {
    m_clk++;

    serve_completed_reads();

    if (m_refresh) {
      m_refresh->tick();
    }

    // 1) 每个 AG 选候选
    std::vector<AGIssueCandidate> cands(m_ag_ctrls.size());
    for (size_t i = 0; i < m_ag_ctrls.size(); ++i) {
      cands[i] = select_candidate_for_ag(m_ag_ctrls[i]);
    }

    // 2) rr 仲裁
    int chosen_ag = -1;
    int n = static_cast<int>(m_ag_ctrls.size());
    for (int offset = 1; offset <= n; ++offset) {
      int idx = (m_last_ag_issued + offset) % n;
      if (cands[idx].valid) {
        chosen_ag = idx;
        break;
      }
    }

    if (chosen_ag < 0) {
      ReqBuffer::iterator dummy;
      for (auto* plugin : m_plugins) {
        plugin->update(false, dummy);
      }
      return;
    }

    m_last_ag_issued = chosen_ag;
    auto& ag   = m_ag_ctrls[chosen_ag];
    auto& cand = cands[chosen_ag];

    Request& req = *(cand.it);

    for (auto* plugin : m_plugins) {
      plugin->update(true, cand.it);
    }

    // 3) 发命令
    m_dram->issue_command(req.command, req.addr_vec);

    // 4) 处理完成
    if (req.command == req.final_command) {
      // --- callback reads ---
      if (req.type_id == Request::Type::Read || req.type_id == Request::Type::PIM_RD_SINGLE) {
        req.depart = m_clk + m_dram->m_read_latency;
        pending.push_back(req);
        ag.pending_reads++;
      }

      // --- extra PIM modeling ---
      handle_pim_side_effects(ag, req);

      cand.buf->remove(cand.it);
      return;
    }

    // 不是 final：ReRAM 当前没有 opening command，但保留逻辑
    if (m_dram->m_command_meta(req.command).is_opening) {
      ag.active_buffer.enqueue(req);
      cand.buf->remove(cand.it);
    }
  }

  bool is_pending() override {
    if (!pending.empty()) return true;
    for (auto& ag : m_ag_ctrls) {
      if (!ag.empty()) return true;
    }
    return false;
  }

  Clk_t get_clk() { return m_clk; }

private:
  static inline bool is_pim_like(Request::Type t) {
    switch (t) {
      case Request::Type::PIM_NOR:
      case Request::Type::PIM_SET:
      case Request::Type::PIM_MASK:
      case Request::Type::PIM_MUL_RD:
      case Request::Type::PIM_MV_SINGLE:
      case Request::Type::PIM_RD_SINGLE:
      case Request::Type::PIM_RD_ALL:
        return true;
      default:
        return false;
    }
  }

  inline int get_bs_bit(const Request& req) const {
    // base_addr: MASK 需要去掉低 32 位 mask
    uint64_t base = (req.type_id == Request::Type::PIM_MASK) ? (uint64_t(req.addr) >> 32) : uint64_t(req.addr);
    return int((base >> (m_row_bits + m_col_bits)) & 0x1ULL);
  }

  inline int get_block_id(const Request& req) const {
    int row = static_cast<int>(req.addr_vec[m_row_addr_idx]);
    return (row >> 5); // 1024 rows => 32 blocks
  }

  inline int get_fp16_col(const Request& req) const {
    int col = static_cast<int>(req.addr_vec[m_col_addr_idx]);
    return (col >> 4); // bit col -> fp16 index
  }

  // ==================== 每个 AG 自身如何选候选命令 ====================

  AGIssueCandidate select_candidate_for_ag(AGController& ag) {
    AGIssueCandidate cand;

    // busy：如果还在做片上 buffer/SA 相关开销，本 AG 的 PIM 类请求先不发
    const bool pim_busy = (ag.pim_busy_until > m_clk);

    // 1) active
    if (ag.active_buffer.size() != 0) {
      auto it = m_scheduler->get_best_request(ag.active_buffer);
      if (it != ag.active_buffer.end() && m_dram->check_ready(it->command, it->addr_vec)) {
        cand.valid = true;
        cand.buf   = &ag.active_buffer;
        cand.it    = it;
        return cand;
      }
    }

    // 2) priority -> pim -> rw
    ReqBuffer* bufs[] = {
        &ag.priority_buffer,
        &ag.pim_buffer,
        &ag.read_buffer,
        &ag.write_buffer,
    };

    for (ReqBuffer* buf : bufs) {
      if (buf->size() == 0) continue;

      auto it = m_scheduler->get_best_request(*buf);
      if (it == buf->end()) continue;

      // PIM-like 且 busy：先不发
      if (pim_busy && is_pim_like(it->type_id)) {
        continue;
      }

      it->command = m_dram->get_preq_command(it->final_command, it->addr_vec);
      if (m_dram->check_ready(it->command, it->addr_vec)) {
        cand.valid = true;
        cand.buf   = buf;
        cand.it    = it;
        return cand;
      }
    }

    return cand;
  }

  // ==================== PIM side effects (mask/buffer modeling) ====================

  void handle_pim_side_effects(AGController& ag, const Request& req) {
    if (!is_pim_like(req.type_id)) return;

    const int array_id = static_cast<int>(req.addr_vec[m_array_addr_idx]);
    const int block_id = get_block_id(req);
    const int bs       = get_bs_bit(req);
    const int fp16_col = get_fp16_col(req);

    // 选通行数（仅在 block 范围内）
    auto selected_rows_in_block = [&](int b) -> int {
      uint32_t mask = ag.mask_ref(array_id, b);
      int not_sel = __builtin_popcount(mask);
      return 32 - not_sel;
    };

    switch (req.type_id) {
      case Request::Type::PIM_MASK: {
        // mask_value: 低 32 位
        uint32_t mask_value = static_cast<uint32_t>(uint64_t(req.addr) & 0xffffffffULL);
        ag.mask_ref(array_id, block_id) = mask_value;
        ag.pim_busy_until = std::max(ag.pim_busy_until, m_clk + (Clk_t)m_mask_cycles);
        break;
      }

      case Request::Type::PIM_MUL_RD:
      case Request::Type::PIM_RD_ALL: {
        // 将指定范围的一列 FP16 读入 buffer
        // BS=1 -> 单个 block (32 行), BS=0 -> whole array (1024 行)
        int rows = (bs == 0) ? 1024 : 32;

        // mask 仅对 block 生效；whole-array 时逐 block 累加
        int selected_rows = 0;
        if (rows == 32) {
          selected_rows = selected_rows_in_block(block_id);
        } else {
          for (int b = 0; b < AGController::kBlocksPerArray; ++b) {
            selected_rows += selected_rows_in_block(b);
          }
        }

        const int extra_cycles = std::max(1, m_mul_rd_cycles_per_row) * std::max(1, selected_rows);
        ag.pim_busy_until = std::max(ag.pim_busy_until, m_clk + (Clk_t)extra_cycles);

        ag.buf.valid       = true;
        ag.buf.array_id    = array_id;
        ag.buf.whole_array = (rows != 32);
        ag.buf.block_id    = (rows == 32) ? block_id : -1;
        ag.buf.fp16_col    = fp16_col;
        ag.buf.ready_at    = ag.pim_busy_until;
        break;
      }

      case Request::Type::PIM_MV_SINGLE: {
        // hit：buffer 覆盖该 array/block 且列一致
        bool hit = false;
        if (ag.buf.valid && ag.buf.array_id == array_id && ag.buf.fp16_col == fp16_col) {
          if (ag.buf.whole_array) {
            hit = true;
          } else {
            hit = (ag.buf.block_id == block_id);
          }
        }
        int extra = hit ? m_mv_single_hit_cycles : (m_mv_single_hit_cycles + m_mv_single_miss_cycles);
        ag.pim_busy_until = std::max(ag.pim_busy_until, m_clk + (Clk_t)std::max(1, extra));
        break;
      }

      case Request::Type::PIM_RD_SINGLE: {
        // RD_SINGLE：模型上把 bus/传输额外算一下
        ag.pim_busy_until = std::max(ag.pim_busy_until, m_clk + (Clk_t)std::max(1, m_rd_single_bus_cycles));
        break;
      }

      case Request::Type::PIM_NOR:
      case Request::Type::PIM_SET: {
        // NOR/SET：按选通行数比例增加开销（可粗略）
        int selected = selected_rows_in_block(block_id);
        int extra = std::max(1, selected) * 1; // 每选通一行 1cycle（你可按需要换成 nWR/nRC 等）
        ag.pim_busy_until = std::max(ag.pim_busy_until, m_clk + (Clk_t)extra);
        break;
      }
      default:
        break;
    }
  }

  // ==================== 完成的读请求回调 ====================

  void serve_completed_reads() {
    if (pending.empty()) return;

    auto& req = pending.front();
    if (req.depart <= m_clk) {
      if (req.callback) {
        req.callback(req);
      }
      int ag_id = static_cast<int>(req.addr_vec[m_ag_addr_idx]);
      if (ag_id >= 0 && ag_id < (int)m_ag_ctrls.size()) {
        if (m_ag_ctrls[ag_id].pending_reads > 0) m_ag_ctrls[ag_id].pending_reads--;
      }
      pending.pop_front();
    }
  }
};

} // namespace Ramulator
