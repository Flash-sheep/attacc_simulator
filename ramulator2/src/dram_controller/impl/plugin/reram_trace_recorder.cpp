#include <vector>
#include <filesystem>

#include <spdlog/spdlog.h>
#include <spdlog/sinks/basic_file_sink.h>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/plugin.h"
#include "dram/dram.h"

namespace Ramulator {

class ReRAMTraceRecorder : public IControllerPlugin, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IControllerPlugin,
      ReRAMTraceRecorder,
      "ReRAMTraceRecorder",
      "Trace recorder for ReRAM commands (RD/WR/NOR/SET).");

 private:
  IDRAM*   m_dram   = nullptr;
  Logger_t m_tracer;

  std::filesystem::path m_trace_path;
  Clk_t m_clk = 0;

 public:
  void init() override {
    // 从 config 里拿 trace 基础路径，如 "traces/reram_trace"
    m_trace_path = param<std::string>("path")
                       .desc("Path prefix to the ReRAM trace file")
                       .required();

    auto parent_path = m_trace_path.parent_path();
    std::filesystem::create_directories(parent_path);
    if (!(std::filesystem::exists(parent_path) &&
          std::filesystem::is_directory(parent_path))) {
      throw ConfigurationError("Invalid path to trace file: {}",
                               parent_path.string());
    }
  }

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    // 拿到所属的 controller 和 dram 句柄
    m_ctrl = cast_parent<IDRAMController>();
    m_dram = m_ctrl->m_dram;

    // 每个 channel 单独一个文件，例如: traces/reram_trace.ch0
    auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
        fmt::format("{}.ch{}", m_trace_path.string(), m_ctrl->m_channel_id),
        /*truncate=*/true);

    m_tracer = std::make_shared<spdlog::logger>(
        fmt::format("reram_trace_ch{}", m_ctrl->m_channel_id), sink);

    m_tracer->set_pattern("%v");
    m_tracer->set_level(spdlog::level::trace);
  }

  void update(bool request_found, ReqBuffer::iterator& req_it) override {
    // 每个 controller tick 调用一次
    m_clk++;

    if (!request_found) {
      // 本周期没发命令就啥也不记
      return;
    }

    auto& req = *req_it;
    // 命令名字： "RD" / "WR" / "NOR" / "SET"
    auto cmd_name = m_dram->m_commands(req.command);

    // 地址向量： [channel, Tile, PE, Arraygroup, Array, row, column]
    // 你可以简单 trace 出来，后处理时自己解析
    m_tracer->trace(
        "{}, {}, type_id={}, addr_vec=[{}]",
        m_clk,
        cmd_name,
        req.type_id,
        fmt::join(req.addr_vec, ", "));
  }
};

}  // namespace Ramulator
