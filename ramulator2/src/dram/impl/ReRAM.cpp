#include "dram/dram.h"
#include "dram/lambdas.h"

// Considering QDR DQ pins, we double dq pins and halve burst length. So, rate 2 actually means 4 Gbps DQs for HBM3.

namespace Ramulator {

class ReRAM : public IDRAM, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAM, ReRAM, "ReRAM", "ReRAM Device Model")

  public:
    inline static const std::map<std::string, Organization> org_presets = {

      //   name          density    DQ  Ch Tile PE  Ag  Array Ro Co
      {"ReRAM_32GB", {256<<10, 1, {16, 8, 8, 4, 64, 1024, 1024}}}
    };

    inline static const std::map<std::string, std::vector<int>> timing_presets = {
      //   name             rate   nBL  nCL  nRCDRD  nRCDWR  nRP  nRAS  nRC  nWR  nRTPS  nRTPL  nCWL  nCCDS  nCCDL  nCCDAB  nCCDSB  nRRDS  nRRDL  nWTRS  nWTRL  nRTW  nFAW  nRFC  nRFCSB  nREFI  nREFISB  nRREFD  tCK_ps
      {"ReRAM_Leader_4.8Gbps", {
      4800,
      2,
      /* nCL    */ 18,   // 15ns
      /* nRCDRD */ 22,   // 18ns
      /* nRCDWR */ 22,   // 18ns
      /* nRP    */ 17,   // 沿用 HBM3，基本等同于 ~14ns
      /* nRAS   */ 41,   // 沿用 HBM3
      /* nRC    */ 58,   // 沿用 HBM3 (RAS+RP)
      /* nWR    */ 32,   // 26ns (tWR_fast)
      /* nRTPS  */ 5,
      /* nRTPL  */ 8,
      /* nCWL   */ 5,
      /* nCCDS  */ 2,
      /* nCCDL  */ 4,
      /* nRRDS  */ 2,
      /* nRRDL  */ 4,
      /* nWTRS  */ 8,    // 这里先沿用 HBM3 的写读间隔
      /* nWTRL  */ 10,
      /* nRTW   */ 3,
      /* nFAW   */ 36,   // ~30ns，与 Leader 的 tFAW 一致
      /* nRFC   */ -1,   // ReRAM 实际不需要 refresh，可以后续在控制器上关掉 REF
      /* nRFCSB */ 240,
      /* nREFI  */ 4680,
      /* nREFISB*/ -1,
      /* nRREFD */ 10,
      /* tCK_ps */ 833   // 实际上会被 set_timing_vals() 里根据 rate 重算覆盖
  }},

  // 同理可以给 5.2 / 5.6 / 6.0 / 6.4Gbps 也各来一条：
  {"ReRAM_Leader_5.2Gbps", {
      5200,
      2,
      20, 24, 24, 19, 45, 63, 34, 6, 8, 6, 2, 4, 2, 4, 8, 11, 3, 39, -1, 260, 5070, -1, 11, 769
  }},
  {"ReRAM_Leader_5.6Gbps", {
      5600,
      2,
      21, 26, 26, 20, 48, 68, 37, 6, 9, 6, 2, 4, 2, 4, 9, 12, 3, 42, -1, 280, 5460, -1, 12, 714
  }},
  {"ReRAM_Leader_6.0Gbps", {
      6000,
      2,
      23, 27, 27, 21, 51, 72, 39, 6, 9, 6, 2, 4, 2, 4, 9, 12, 3, 45, -1, 300, 5850, -1, 12, 667
  }},
  {"ReRAM_Leader_6.4Gbps", {
      6400,
      2,
      24, 29, 29, 23, 55, 77, 42, 7,10, 7, 2, 4, 2, 4,10, 13, 3, 48, -1, 320, 6240, -1, 13, 625
  }},
    };


  /************************************************
   *                Organization
   ***********************************************/   
    const int m_internal_prefetch_size = 1;

    inline static constexpr ImplDef m_levels = {
      "channel", "tile", "pe", "ag", "array", "row", "column",    
    };


  /************************************************
   *             Requests & Commands
   ***********************************************/
    inline static constexpr ImplDef m_commands = {
  // DRAM commands
  "RD",  "WR",
  // PIM / ReRAM-PIM commands
  "NOR", "SET",
  "MASK", "MUL_RD", "MV_SINGLE", "RD_SINGLE", "RD_ALL",
  "WRITE_SINGLE", "WRITE_MUL",
};


    inline static const ImplLUT m_command_scopes = LUT(
    m_commands, m_levels, {
        // ReRAM 基本读写
        {"RD",        "column"},
        {"WR",        "column"},

        // ReRAM PIM / 扩展指令
        {"NOR",       "column"},
        {"SET",       "column"},
        {"MASK",      "row"},
        {"MUL_RD",    "column"},
        {"MV_SINGLE", "column"},
        {"RD_SINGLE", "column"},
        {"RD_ALL",    "column"},
        {"WRITE_SINGLE", "column"},
        {"WRITE_MUL",    "column"},
    }
);


    inline static const ImplLUT m_command_meta = LUT<DRAMCommandMeta>(
    m_commands, {
            // open?  close? access? refresh?
    {"RD",        {false, false, true,  false}},  // 读阵列
    {"WR",        {false, false, true,  false}},  // 写阵列
    {"NOR",       {false, false, true,  false}},  // PIM NOR
    {"SET",       {false, false, true,  false}},  // PIM SET
    {"MASK",      {false, false, true,  false}},  // 行选通 mask 配置
    {"MUL_RD",    {false, false, true,  false}},  // 块读到 buffer（概念）
    {"MV_SINGLE", {false, false, true,  false}},  // 片上搬运（概念）
    {"RD_SINGLE", {false, false, true,  false}},  // 单个 FP16 读（概念）
    {"RD_ALL",    {false, false, true,  false}},  // 块读（概念）
    {"WRITE_SINGLE", {false, false, true, false}}, // 单个 FP16 写（概念）
    {"WRITE_MUL",    {false, false, true, false}}, // 块写（概念）
    }
);


    inline static constexpr ImplDef m_requests = {
  // DRAM requests
  "read", "write",
  // PIM requests
  "pim-nor", "pim-set",
  // Extended ReRAM-PIM requests
  "pim-mask", "mul-rd", "mv-single", "rd-single", "rd-all",
  "pim-write-single", "pim-write-mul",
};

inline static const ImplLUT m_request_translations = LUT(
    m_requests, m_commands, {
        {"read",      "RD"},
        {"write",     "WR"},
        {"pim-nor",   "NOR"},
        {"pim-set",   "SET"},
        {"pim-mask",  "MASK"},
        {"mul-rd",    "MUL_RD"},
        {"mv-single", "MV_SINGLE"},
        {"rd-single", "RD_SINGLE"},
        {"rd-all",    "RD_ALL"},
        {"pim-write-single", "WRITE_SINGLE"},
        {"pim-write-mul",    "WRITE_MUL"},
    }
);


   
  /************************************************
   *                   Timing
   ***********************************************/
    inline static constexpr ImplDef m_timings = {
      "rate", 
      "nBL", "nCL", "nRCDRD", "nRCDWR", "nRP", "nRAS", "nRC", "nWR", "nRTPS", "nRTPL", "nCWL",
      "nCCDS", "nCCDL", "nCCDAB", "nCCDSB",
      "nRRDS", "nRRDL",
      "nWTRS", "nWTRL",
      "nRTW",
      "nFAW",
      "nRFC", "nRFCSB", "nREFI", "nREFISB", "nRREFD",
      "tCK_ps"
    };


  /************************************************
   *                 Node States
   ***********************************************/
    inline static constexpr ImplDef m_states = {
        "N/A"
    };

    inline static const ImplLUT m_init_states = LUT(
    m_levels, m_states, {
        {"channel", "N/A"},
        {"tile",    "N/A"},
        {"pe",      "N/A"},
        {"ag",      "N/A"},
        {"array",   "N/A"},
        {"row",     "N/A"},
        {"column",  "N/A"},
    }
    );

  public:
    struct Node : public DRAMNodeBase<ReRAM> {
      Node(ReRAM* dram, Node* parent, int level, int id) : DRAMNodeBase<ReRAM>(dram, parent, level, id) {};
    };
    std::vector<Node*> m_channels;
    
    FuncMatrix<ActionFunc_t<Node>>  m_actions;
    FuncMatrix<PreqFunc_t<Node>>    m_preqs;
    FuncMatrix<RowhitFunc_t<Node>>  m_rowhits;
    FuncMatrix<RowopenFunc_t<Node>> m_rowopens;


  public:
    void tick() override {
      m_clk++;
    };

    void init() override {
      RAMULATOR_DECLARE_SPECS();
      set_organization();
      set_timing_vals();

      set_actions();
      set_preqs();
      set_rowhits();
      set_rowopens();
      
      create_nodes();
    };

    void issue_command(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      m_channels[channel_id]->update_timing(command, addr_vec, m_clk);
      m_channels[channel_id]->update_states(command, addr_vec, m_clk);
    };

    int get_preq_command(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->get_preq_command(command, addr_vec, m_clk);
    };

    bool check_ready(int command, const AddrVec_t& addr_vec) override {
      int channel_id = addr_vec[m_levels["channel"]];
      return m_channels[channel_id]->check_ready(command, addr_vec, m_clk);
    };

    bool check_rowbuffer_hit(int command, const AddrVec_t& addr_vec) override {
        (void)command;
        (void)addr_vec;
        return false;
    }


  private:
    void set_organization() {
      // Channel width
      m_channel_width = param_group("org").param<int>("channel_width").default_val(32);

      // Organization
      m_organization.count.resize(m_levels.size(), -1);

      // Load organization preset if provided
      if (auto preset_name = param_group("org").param<std::string>("preset").optional()) {
        if (org_presets.count(*preset_name) > 0) {
          m_organization = org_presets.at(*preset_name);
        } else {
          throw ConfigurationError("Unrecognized organization preset \"{}\" in {}!", *preset_name, get_name());
        }
      }

      // Override the preset with any provided settings
      if (auto dq = param_group("org").param<int>("dq").optional()) {
        m_organization.dq = *dq;
      }

      for (int i = 0; i < m_levels.size(); i++){
        auto level_name = m_levels(i);
        if (auto sz = param_group("org").param<int>(level_name).optional()) {
          m_organization.count[i] = *sz;
        }
      }

      if (auto density = param_group("org").param<int>("density").optional()) {
        m_organization.density = *density;
      }

      // Sanity check: is the calculated channel density the same as the provided one?
      // Sanity check: is the calculated total density the same as the provided one?
        size_t _density = 1;

        // 把所有层级的 count 都乘起来：ch * tile * pe * ag * array * row * column
        for (int i = 0; i < m_levels.size(); i++) {
        int cnt = m_organization.count[i];
        if (cnt > 0) {
            _density *= size_t(cnt);
        }
        }

        // 乘上每个 cell 的位宽 (dq) 和内部预取宽度
        _density *= size_t(m_organization.dq);
        _density *= size_t(m_internal_prefetch_size);

        // 转成 Mb（注意这里还是按 bit→Mb 来算）
        _density >>= 20;  // / 2^20

        if (m_organization.density != _density) {
        throw ConfigurationError(
            "Calculated {} device density {} Mb does not equal the provided density {} Mb!",
            get_name(),
            _density,
            m_organization.density
        );
        }


    };

    void set_timing_vals() {
    // 1. 初始化 timing 数组
    m_timing_vals.resize(m_timings.size(), -1);

    // 2. 先根据 preset 填一整行
    bool preset_provided = false;
    if (auto preset_name = param_group("timing").param<std::string>("preset").optional()) {
        if (timing_presets.count(*preset_name) > 0) {
        m_timing_vals = timing_presets.at(*preset_name);
        preset_provided = true;
        } else {
        throw ConfigurationError("Unrecognized timing preset \"{}\" in {}!", *preset_name, get_name());
        }
    }

    // 3. 如果用户单独覆盖了 rate，就更新 rate，并重算 tCK_ps
    if (auto rate = param_group("timing").param<int>("rate").optional()) {
        if (preset_provided) {
        throw ConfigurationError("Cannot change the transfer rate of {} when using a speed preset !", get_name());
        }
        m_timing_vals("rate") = *rate;
    }

    // ReRAM 接口如果还是沿用 HBM/QDR 风格，可以继续 /4；如果是 SDR，就去掉 /4
    int tCK_ps = 1E6 / m_timing_vals("rate");   // SDR：1e6 / MT/s

    m_timing_vals("tCK_ps") = tCK_ps;

    // 4. 支持在 config 里用 “nXXX”（cycles） 或 “tXXX”（ns） 覆盖单个 timing
    //    注意：这里假定 m_timings[0] 是 "rate"，最后一个是 "tCK_ps"，所以从 1 到 size-2
    for (int i = 1; i < m_timings.size() - 1; i++) {
        auto timing_name = std::string(m_timings(i));

        if (auto provided_cycles = param_group("timing").param<int>(timing_name).optional()) {
        // 直接用周期数，例如 nCL = 20
        m_timing_vals(i) = *provided_cycles;
        } else {
        // 支持用 ns 指定，例如 tCL = 15.0
        auto t_name = timing_name;
        t_name.replace(0, 1, "t");  // nCL -> tCL
        if (auto provided_ns = param_group("timing").param<float>(t_name).optional()) {
            m_timing_vals(i) = JEDEC_rounding(*provided_ns, tCK_ps);
        }
        }
    }

    // 5. 确保所有 timing 都已经被 preset 或 config 填好
    for (int i = 0; i < m_timing_vals.size(); i++) {
        if (m_timing_vals(i) == -1) {
        throw ConfigurationError("In \"{}\", timing {} is not specified!", get_name(), m_timings(i));
        }
    }

    // 6. 读延迟 = nCL + nBL（Ramulator 用这个算 data 返回延时）
    m_read_latency = m_timing_vals("nCL") + m_timing_vals("nBL");

    // 7. 填 ReRAM 的 timing constraints
    #define V(timing) (m_timing_vals(timing))
    // 64B 块读（RD_ALL）在总线上的等效占用：按 channel_width 估算需要多少个 beat
    const int _beat_bytes = m_channel_width / 8;
    const int _beats_all  = (_beat_bytes > 0) ? ( (64 + _beat_bytes - 1) / _beat_bytes ) : 16;
    const int _nBL_ALL    = V("nBL") * _beats_all;

    populate_timingcons(this, {

        /////////////////////////////////////////
        ////  Channel 级：总线 (CAS) 约束   ////
        /////////////////////////////////////////

        {.level = "channel", .preceding = {"RD"}, .following = {"RD"}, .latency = V("nBL")},
        {.level = "channel", .preceding = {"WR"}, .following = {"WR"}, .latency = V("nBL")},
        {.level = "channel", .preceding = {"RD"}, .following = {"WR"}, .latency = V("nRTW")},
        {.level = "channel", .preceding = {"WR"}, .following = {"RD"}, .latency = V("nWTRS")},

// 扩展读指令：RD_SINGLE / RD_ALL 同样占用 channel 总线
{.level = "channel", .preceding = {"RD_SINGLE"}, .following = {"RD_SINGLE"}, .latency = V("nBL")},
{.level = "channel", .preceding = {"RD"},        .following = {"RD_SINGLE"}, .latency = V("nBL")},
{.level = "channel", .preceding = {"RD_SINGLE"}, .following = {"RD"},        .latency = V("nBL")},
{.level = "channel", .preceding = {"MUL_RD"},    .following = {"MUL_RD"},    .latency = V("nBL")},
{.level = "channel", .preceding = {"RD"},        .following = {"MUL_RD"},    .latency = V("nBL")},
{.level = "channel", .preceding = {"MUL_RD"},    .following = {"RD"},        .latency = V("nBL")},
{.level = "channel", .preceding = {"RD_SINGLE"}, .following = {"MUL_RD"},    .latency = V("nBL")},
{.level = "channel", .preceding = {"MUL_RD"},    .following = {"RD_SINGLE"}, .latency = V("nBL")},
{.level = "channel", .preceding = {"MUL_RD"},    .following = {"WR"},        .latency = V("nRTW")},
{.level = "channel", .preceding = {"WR"},        .following = {"MUL_RD"},    .latency = V("nWTRS")},

// RD_ALL：模型上按 64B 传输估算总线占用（_nBL_ALL）
{.level = "channel", .preceding = {"RD_ALL"},    .following = {"RD_ALL"},    .latency = _nBL_ALL},
{.level = "channel", .preceding = {"RD_ALL"},    .following = {"RD"},        .latency = _nBL_ALL},
{.level = "channel", .preceding = {"RD_ALL"},    .following = {"RD_SINGLE"}, .latency = _nBL_ALL},
{.level = "channel", .preceding = {"RD"},        .following = {"RD_ALL"},    .latency = V("nBL")},
{.level = "channel", .preceding = {"RD_SINGLE"}, .following = {"RD_ALL"},    .latency = V("nBL")},


        /////////////////////////////////////////
        ////  Array 级：阵列内部访问约束     ////
        /////////////////////////////////////////

        // 同一 array 内的“读-读 / 读-写 / 写-读”最小间隔，用 nRC 当阵列周期
        {.level = "array", .preceding = {"RD"},  .following = {"RD"},  .latency = V("nRC")},
        {.level = "array", .preceding = {"RD"},  .following = {"WR"},  .latency = V("nRC")},
        {.level = "array", .preceding = {"WR"},  .following = {"RD"},  .latency = V("nRC")},

// 扩展读：按 read 类处理
{.level = "array", .preceding = {"RD_SINGLE"},  .following = {"RD_SINGLE"},  .latency = V("nRC")},
{.level = "array", .preceding = {"RD_ALL"},     .following = {"RD_ALL"},     .latency = V("nRC")},
{.level = "array", .preceding = {"MUL_RD"},     .following = {"MUL_RD"},     .latency = V("nRC")},
{.level = "array", .preceding = {"MV_SINGLE"},  .following = {"MV_SINGLE"},  .latency = V("nRC")},
{.level = "array", .preceding = {"RD"},         .following = {"RD_SINGLE"},  .latency = V("nRC")},
{.level = "array", .preceding = {"RD"},         .following = {"RD_ALL"},     .latency = V("nRC")},
{.level = "array", .preceding = {"RD"},         .following = {"MUL_RD"},     .latency = V("nRC")},
{.level = "array", .preceding = {"RD"},         .following = {"MV_SINGLE"},  .latency = V("nRC")},
{.level = "array", .preceding = {"RD_SINGLE"},  .following = {"RD"},         .latency = V("nRC")},
{.level = "array", .preceding = {"RD_ALL"},     .following = {"RD"},         .latency = V("nRC")},
{.level = "array", .preceding = {"MUL_RD"},     .following = {"RD"},         .latency = V("nRC")},
{.level = "array", .preceding = {"MV_SINGLE"},  .following = {"RD"},         .latency = V("nRC")},

// ------------------------------
// 把 WRITE_SINGLE / WRITE_MUL 纳入 array 级访问（nRC）约束
// 目的：避免调度器把 array 内部访问重叠发射
// ------------------------------
{.level="array", .preceding={"RD"},        .following={"WRITE_SINGLE"}, .latency=V("nRC")},
{.level="array", .preceding={"RD"},        .following={"WRITE_MUL"},    .latency=V("nRC")},
{.level="array", .preceding={"RD_SINGLE"}, .following={"WRITE_SINGLE"}, .latency=V("nRC")},
{.level="array", .preceding={"RD_SINGLE"}, .following={"WRITE_MUL"},    .latency=V("nRC")},
{.level="array", .preceding={"RD_ALL"},    .following={"WRITE_SINGLE"}, .latency=V("nRC")},
{.level="array", .preceding={"RD_ALL"},    .following={"WRITE_MUL"},    .latency=V("nRC")},
{.level="array", .preceding={"MUL_RD"},    .following={"WRITE_SINGLE"}, .latency=V("nRC")},
{.level="array", .preceding={"MUL_RD"},    .following={"WRITE_MUL"},    .latency=V("nRC")},
{.level="array", .preceding={"MV_SINGLE"}, .following={"WRITE_SINGLE"}, .latency=V("nRC")},
{.level="array", .preceding={"MV_SINGLE"}, .following={"WRITE_MUL"},    .latency=V("nRC")},

{.level="array", .preceding={"WRITE_SINGLE"}, .following={"RD"},        .latency=V("nRC")},
{.level="array", .preceding={"WRITE_MUL"},    .following={"RD"},        .latency=V("nRC")},
{.level="array", .preceding={"WRITE_SINGLE"}, .following={"RD_SINGLE"}, .latency=V("nRC")},
{.level="array", .preceding={"WRITE_MUL"},    .following={"RD_SINGLE"}, .latency=V("nRC")},
{.level="array", .preceding={"WRITE_SINGLE"}, .following={"RD_ALL"},    .latency=V("nRC")},
{.level="array", .preceding={"WRITE_MUL"},    .following={"RD_ALL"},    .latency=V("nRC")},
{.level="array", .preceding={"WRITE_SINGLE"}, .following={"MUL_RD"},    .latency=V("nRC")},
{.level="array", .preceding={"WRITE_MUL"},    .following={"MUL_RD"},    .latency=V("nRC")},
{.level="array", .preceding={"WRITE_SINGLE"}, .following={"MV_SINGLE"}, .latency=V("nRC")},
{.level="array", .preceding={"WRITE_MUL"},    .following={"MV_SINGLE"}, .latency=V("nRC")},

{.level="array", .preceding={"WRITE_SINGLE"}, .following={"WRITE_SINGLE"}, .latency=V("nRC")},
{.level="array", .preceding={"WRITE_SINGLE"}, .following={"WRITE_MUL"},    .latency=V("nRC")},
{.level="array", .preceding={"WRITE_MUL"},    .following={"WRITE_SINGLE"}, .latency=V("nRC")},
{.level="array", .preceding={"WRITE_MUL"},    .following={"WRITE_MUL"},    .latency=V("nRC")},

        // 写类 = WR / NOR / SET：互相之间至少间隔 nWR（等价“写一次”的时间）
        {.level = "array", .preceding = {"WR"},  .following = {"WR"},  .latency = V("nWR")},
        {.level = "array", .preceding = {"WR"},  .following = {"NOR"}, .latency = V("nWR")},
        {.level = "array", .preceding = {"WR"},  .following = {"SET"}, .latency = V("nWR")},
{.level = "array", .preceding = {"WR"},  .following = {"MASK"}, .latency = V("nWR")},
{.level = "array", .preceding = {"WR"},  .following = {"RD_ALL"}, .latency = V("nWR")},

        {.level = "array", .preceding = {"NOR"}, .following = {"WR"},  .latency = V("nWR")},
        {.level = "array", .preceding = {"NOR"}, .following = {"NOR"}, .latency = V("nWR")},
        {.level = "array", .preceding = {"NOR"}, .following = {"SET"}, .latency = V("nWR")},

        {.level = "array", .preceding = {"SET"}, .following = {"WR"},  .latency = V("nWR")},
        {.level = "array", .preceding = {"SET"}, .following = {"NOR"}, .latency = V("nWR")},
        {.level = "array", .preceding = {"SET"}, .following = {"SET"}, .latency = V("nWR")},
{.level = "array", .preceding = {"SET"},  .following = {"MASK"}, .latency = V("nWR")},
{.level = "array", .preceding = {"MASK"}, .following = {"WR"},  .latency = V("nWR")},
{.level = "array", .preceding = {"MASK"}, .following = {"NOR"}, .latency = V("nWR")},
{.level = "array", .preceding = {"MASK"}, .following = {"SET"}, .latency = V("nWR")},
{.level = "array", .preceding = {"MASK"}, .following = {"MASK"},.latency = V("nWR")},

// ------------------------------
// 把 WRITE_SINGLE / WRITE_MUL 纳入“写类”nWR 约束
// 目的：写后恢复/编程时间建模（与你已有 WR/NOR/SET 一致）
// ------------------------------
{.level="array", .preceding={"WR"},           .following={"WRITE_SINGLE"}, .latency=V("nWR")},
{.level="array", .preceding={"WR"},           .following={"WRITE_MUL"},    .latency=V("nWR")},
{.level="array", .preceding={"NOR"},          .following={"WRITE_SINGLE"}, .latency=V("nWR")},
{.level="array", .preceding={"NOR"},          .following={"WRITE_MUL"},    .latency=V("nWR")},
{.level="array", .preceding={"SET"},          .following={"WRITE_SINGLE"}, .latency=V("nWR")},
{.level="array", .preceding={"SET"},          .following={"WRITE_MUL"},    .latency=V("nWR")},
{.level="array", .preceding={"MASK"},         .following={"WRITE_SINGLE"}, .latency=V("nWR")},
{.level="array", .preceding={"MASK"},         .following={"WRITE_MUL"},    .latency=V("nWR")},

{.level="array", .preceding={"WRITE_SINGLE"}, .following={"WR"},           .latency=V("nWR")},
{.level="array", .preceding={"WRITE_MUL"},    .following={"WR"},           .latency=V("nWR")},
{.level="array", .preceding={"WRITE_SINGLE"}, .following={"NOR"},          .latency=V("nWR")},
{.level="array", .preceding={"WRITE_MUL"},    .following={"NOR"},          .latency=V("nWR")},
{.level="array", .preceding={"WRITE_SINGLE"}, .following={"SET"},          .latency=V("nWR")},
{.level="array", .preceding={"WRITE_MUL"},    .following={"SET"},          .latency=V("nWR")},
{.level="array", .preceding={"WRITE_SINGLE"}, .following={"MASK"},         .latency=V("nWR")},
{.level="array", .preceding={"WRITE_MUL"},    .following={"MASK"},         .latency=V("nWR")},

{.level="array", .preceding={"WRITE_SINGLE"}, .following={"WRITE_SINGLE"}, .latency=V("nWR")},
{.level="array", .preceding={"WRITE_SINGLE"}, .following={"WRITE_MUL"},    .latency=V("nWR")},
{.level="array", .preceding={"WRITE_MUL"},    .following={"WRITE_SINGLE"}, .latency=V("nWR")},
{.level="array", .preceding={"WRITE_MUL"},    .following={"WRITE_MUL"},    .latency=V("nWR")},
    });
    #undef V
    }



    // There are no actions and prerequisites for WRGB, MVSB, MVGB, SFM, SETM, SETH because they are not related to the state of the DRAM.

    void set_actions() {
      m_actions.resize(m_levels.size(), std::vector<ActionFunc_t<Node>>(m_commands.size()));
    };

    void set_preqs() {
      m_preqs.resize(m_levels.size(), std::vector<PreqFunc_t<Node>>(m_commands.size()));

    };

   void set_rowhits() {
    // 分配表，但不注册任何 rowhit 回调
    m_rowhits.resize(m_levels.size(),
                    std::vector<RowhitFunc_t<Node>>(m_commands.size()));

    // 不填任何条目：表示对 RD / WR / NOR / SET 都不做 row-hit 检查，
    // check_rowbuffer_hit() 会一直得到 false（你上层已经改成直接返回 false 其实也可以）。
    }



   void set_rowopens() {
    // 分配表，但不注册任何 RowOpen 回调
    m_rowopens.resize(m_levels.size(),
                        std::vector<RowhitFunc_t<Node>>(m_commands.size()));

    // 不填任何条目：
    //   表示对 RD / WR / NOR / SET 都没有 “row 打开/关闭” 的状态判断，
    //   上层策略不会基于 row-open 状态做任何优化。
    }



    void create_nodes() {
      int num_channels = m_organization.count[m_levels["channel"]];
      for (int i = 0; i < num_channels; i++) {
        Node* channel = new Node(this, nullptr, 0, i);
        m_channels.push_back(channel);
      }
    };
};


}        // namespace Ramulator
