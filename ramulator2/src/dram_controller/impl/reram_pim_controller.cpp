#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "memory_system/memory_system.h"

#include <algorithm>
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
    int blocks_per_array = 0;

    // block mask：每个 array 的每个 block 一个 uint32
    // bit=1 -> 选通；bit=0 -> 不选通
    std::vector<uint32_t> block_masks;

    // Buffer modeling is temporarily disabled.
    // struct BufState {
    //   bool  valid        = false;
    //   int   array_id     = -1;
    //   bool  whole_array  = false;  // BS=0 -> whole array; BS=1 -> one block
    //   int   block_id     = -1;     // 仅在 whole_array=false 时有效
    //   int   fp16_col     = -1;     // buffer 中缓存的是哪个 FP16 列（col/16）
    //   Clk_t ready_at     = 0;
    // } buf;

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
      return block_masks[array_id * blocks_per_array + block_id];
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

  int m_rows_per_array = 0;
  int m_rows_per_block = 32;
  int m_blocks_per_array = 0;

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
  int m_mul_rd_bus_cycles       = 0;   // MUL_RD 直接回主存时的总线占用
  int m_rd_all_bus_cycles       = 0;
  int m_write_single_cycles = 1;
  int m_write_mul_cycles_per_row = 1;
  float m_wr_low_watermark = 0.2f;
  float m_wr_high_watermark = 0.8f;

public:
  void init() override {
    // wr watermarks
    m_wr_low_watermark = param<float>("wr_low_watermark")
        .desc("Threshold for switching back to read mode.")
        .default_val(0.2f);
    m_wr_high_watermark = param<float>("wr_high_watermark")
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

    m_mul_rd_bus_cycles = param<int>("mul_rd_bus_cycles")
        .desc("Extra cycles for MUL_RD bus transfer to memory. If 0, defaults to nBL.")
        .default_val(0);

    m_rd_all_bus_cycles = param<int>("rd_all_bus_cycles")
        .desc("Extra cycles for RD_ALL bus transfer from local buffer to memory. If 0, defaults to rd_single_bus_cycles.")
        .default_val(0);

    m_write_single_cycles = param<int>("write_single_cycles")
        .desc("Extra cycles for PIM_WRITE_SINGLE.")
        .default_val(1);

    m_write_mul_cycles_per_row = param<int>("write_mul_cycles_per_row")
        .desc("Extra cycles per selected row for PIM_WRITE_MUL.")
        .default_val(1);

    m_rows_per_block = param<int>("rows_per_block")
        .desc("Rows represented by one 32-bit MASK value.")
        .default_val(32);

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

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    m_dram = memory_system->get_ifce<IDRAM>();

    m_ag_addr_idx    = m_dram->m_levels("ag");
    m_array_addr_idx = m_dram->m_levels("array");
    m_row_addr_idx   = m_dram->m_levels("row");
    m_col_addr_idx   = m_dram->m_levels("column");

    const int num_ag = m_dram->m_organization.count[m_ag_addr_idx];
    const int num_arrays = m_dram->m_organization.count[m_array_addr_idx];
    m_rows_per_array = m_dram->m_organization.count[m_row_addr_idx];

    if (num_ag <= 0 || num_arrays <= 0 || m_rows_per_array <= 0) {
      throw std::runtime_error("ReRAMController: invalid organization counts.");
    }
    if (m_rows_per_block <= 0 || m_rows_per_block > 32) {
      throw std::runtime_error("ReRAMController: rows_per_block must be in (0, 32].");
    }
    if ((m_rows_per_array % m_rows_per_block) != 0) {
      throw std::runtime_error("ReRAMController: rows_per_array must be divisible by rows_per_block.");
    }
    m_blocks_per_array = m_rows_per_array / m_rows_per_block;

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
    if (m_mul_rd_bus_cycles == 0) {
      m_mul_rd_bus_cycles = std::max(1, m_dram->m_timing_vals("nBL"));
    }
    if (m_rd_all_bus_cycles == 0) {
      m_rd_all_bus_cycles = m_rd_single_bus_cycles;
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

      ag.wr_low_watermark  = m_wr_low_watermark;
      ag.wr_high_watermark = m_wr_high_watermark;

      ag.num_arrays = num_arrays;
      ag.blocks_per_array = m_blocks_per_array;
      const uint32_t default_mask =
          (m_rows_per_block == 32) ? 0xffffffffu : ((1u << m_rows_per_block) - 1u);
      ag.block_masks.assign(num_arrays * m_blocks_per_array, default_mask);
      // Buffer modeling is temporarily disabled.
      // ag.buf = {};
      ag.pim_busy_until = 0;
      ag.pending_reads = 0;
    }

    // ReRAM-specific children depend on m_dram being ready. Wire them explicitly
    // here instead of relying on the global recursive setup order.
    if (m_scheduler != nullptr && m_scheduler->m_impl != nullptr) {
      m_scheduler->m_impl->setup(frontend, memory_system);
    }
    if (m_refresh != nullptr && m_refresh->m_impl != nullptr) {
      m_refresh->m_impl->setup(frontend, memory_system);
    }
    for (auto* plugin : m_plugins) {
      if (plugin != nullptr && plugin->m_impl != nullptr) {
        plugin->m_impl->setup(frontend, memory_system);
      }
    }
  }

  // ==================== 请求入口 ====================

  bool send(Request& req) override {
    if (req.type_id == Request::Type::PIM_RD_ALL) {
      // RD_ALL is temporarily disabled.
      req.arrive = -1;
      return false;
    }
    req.final_command = translate_request_type_to_command(req.type_id);
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
      case Request::Type::PIM_WRITE_SINGLE:
      case Request::Type::PIM_WRITE_MUL:
        ok = ag.pim_buffer.enqueue(req);
        break;

      case Request::Type::PIM_RD_SINGLE:
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
    if (req.type_id == Request::Type::PIM_RD_ALL) {
      // RD_ALL is temporarily disabled.
      req.arrive = -1;
      return false;
    }
    req.final_command = translate_request_type_to_command(req.type_id);
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
      if (req.type_id == Request::Type::Read ||
          req.type_id == Request::Type::PIM_MUL_RD ||
          req.type_id == Request::Type::PIM_RD_SINGLE) {
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
  static inline bool is_pim_like(int t) {
    switch (t) {
      case Request::Type::PIM_NOR:
      case Request::Type::PIM_SET:
      case Request::Type::PIM_MASK:
      case Request::Type::PIM_MUL_RD:
      case Request::Type::PIM_MV_SINGLE:
      case Request::Type::PIM_RD_SINGLE:
      case Request::Type::PIM_WRITE_SINGLE:
      case Request::Type::PIM_WRITE_MUL:
        return true;
      default:
        return false;
    }
  }

  int translate_request_type_to_command(int type_id) const {
    switch (type_id) {
      case Request::Type::Read:             return m_dram->m_commands("RD");
      case Request::Type::Write:            return m_dram->m_commands("WR");
      case Request::Type::PIM_NOR:          return m_dram->m_commands("NOR");
      case Request::Type::PIM_SET:          return m_dram->m_commands("SET");
      case Request::Type::PIM_MASK:         return m_dram->m_commands("MASK");
      case Request::Type::PIM_MUL_RD:       return m_dram->m_commands("MUL_RD");
      case Request::Type::PIM_MV_SINGLE:    return m_dram->m_commands("MV_SINGLE");
      case Request::Type::PIM_RD_SINGLE:    return m_dram->m_commands("RD_SINGLE");
      case Request::Type::PIM_RD_ALL:       return m_dram->m_commands("RD_ALL");
      case Request::Type::PIM_WRITE_SINGLE: return m_dram->m_commands("WRITE_SINGLE");
      case Request::Type::PIM_WRITE_MUL:    return m_dram->m_commands("WRITE_MUL");
      default:
        throw std::runtime_error("ReRAMController: unsupported request type.");
    }
  }

  inline int get_bs_bit(const Request& req) const {
    if (req.isa_decoded) return req.isa_bs;
    return 1;
  }

  inline int get_block_id(const Request& req) const {
    if (req.isa_decoded) return req.isa_block;
    int row = static_cast<int>(req.addr_vec[m_row_addr_idx]);
    return row / m_rows_per_block;
  }

  inline int get_fp16_col(const Request& req) const {
    if (req.isa_decoded && req.isa_word_col >= 0) return req.isa_word_col;
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
    (void)fp16_col;

    // 选通行数（仅在 block 范围内）
    auto selected_rows_in_block = [&](int b) -> int {
      uint32_t mask = ag.mask_ref(array_id, b);
      const uint32_t active_mask = (m_rows_per_block == 32) ? 0xffffffffu : ((1u << m_rows_per_block) - 1u);
      return __builtin_popcount(mask & active_mask);
    };

    switch (req.type_id) {
      case Request::Type::PIM_MASK: {
        // mask_value: 低 32 位
        uint32_t mask_value = static_cast<uint32_t>(uint64_t(req.addr) & 0xffffffffULL);
        ag.mask_ref(array_id, block_id) = mask_value;
        ag.pim_busy_until = std::max(ag.pim_busy_until, m_clk + (Clk_t)m_mask_cycles);
        break;
      }

      case Request::Type::PIM_MUL_RD: {
        // 将指定范围的一列 FP16 从阵列读出并直接回主存（占用总线）
        // BS=1 -> 单个 block, BS=0 -> whole array
        int rows = (bs == 0) ? m_rows_per_array : m_rows_per_block;

        // mask 仅对 block 生效；whole-array 时逐 block 累加
        int selected_rows = 0;
        if (rows == m_rows_per_block) {
          selected_rows = selected_rows_in_block(block_id);
        } else {
          for (int b = 0; b < m_blocks_per_array; ++b) {
            selected_rows += selected_rows_in_block(b);
          }
        }

        const int array_cycles = std::max(1, m_mul_rd_cycles_per_row) * std::max(1, selected_rows);
        const int bus_cycles   = std::max(1, m_mul_rd_bus_cycles);
        const int extra_cycles = array_cycles + bus_cycles;
        ag.pim_busy_until = std::max(ag.pim_busy_until, m_clk + (Clk_t)extra_cycles);

        // Buffer modeling is temporarily disabled.
        // ag.buf.valid       = true;
        // ag.buf.array_id    = array_id;
        // ag.buf.whole_array = (rows != m_rows_per_block);
        // ag.buf.block_id    = (rows == m_rows_per_block) ? block_id : -1;
        // ag.buf.fp16_col    = fp16_col;
        // ag.buf.ready_at    = ag.pim_busy_until;
        break;
      }

      case Request::Type::PIM_RD_ALL: {
        // RD_ALL is temporarily disabled.
        break;
      }

      case Request::Type::PIM_MV_SINGLE: {
        // Buffer-based hit/miss modeling is temporarily disabled.
        // Use a fixed "no-buffer" cost path for now.
        int extra = m_mv_single_hit_cycles + m_mv_single_miss_cycles;
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
      case Request::Type::PIM_WRITE_SINGLE: {

          ag.pim_busy_until =
              std::max(ag.pim_busy_until,
                      m_clk + (Clk_t)m_write_single_cycles);

          // Buffer invalidation is temporarily disabled.
          // if (ag.buf.valid &&
          //     ag.buf.array_id == array_id &&
          //     ag.buf.fp16_col == fp16_col) {
          //
          //     if (ag.buf.whole_array ||
          //         ag.buf.block_id == block_id) {
          //         ag.buf.valid = false;
          //     }
          // }

          break;
      }
      case Request::Type::PIM_WRITE_MUL: {

    int rows = (bs == 0) ? m_rows_per_array : m_rows_per_block;
    int selected_rows = 0;

    if (rows == m_rows_per_block) {
        selected_rows = selected_rows_in_block(block_id);
    } else {
        for (int b = 0; b < m_blocks_per_array; b++) {
            selected_rows += selected_rows_in_block(b);
        }
    }

    int extra_cycles =
        std::max(1, m_write_mul_cycles_per_row) *
        std::max(1, selected_rows);

    ag.pim_busy_until =
        std::max(ag.pim_busy_until,
                 m_clk + (Clk_t)extra_cycles);

    // Buffer invalidation is temporarily disabled.
    // if (ag.buf.valid &&
    //     ag.buf.array_id == array_id &&
    //     ag.buf.fp16_col == fp16_col) {
    //
    //     if (bs == 0 ||
    //         ag.buf.whole_array ||
    //         ag.buf.block_id == block_id) {
    //         ag.buf.valid = false;
    //     }
    // }

    break;
}

      default:
        break;
    }
  }

  // ==================== 完成的读请求回调 ====================

  void serve_completed_reads() {
    while (!pending.empty() && pending.front().depart <= m_clk) {
      auto& req = pending.front();
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
