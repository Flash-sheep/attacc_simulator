#include <filesystem>
#include <iostream>
#include <fstream>

#include "frontend/frontend.h"
#include "base/exception.h"
#include "memory_system/memory_system.h"
#include "dram/dram.h"   // Request

namespace Ramulator {

namespace fs = std::filesystem;

/**
 * ReRAMTrace
 *
 * 每行两个字段：
 *   <OP> <ADDR>
 *
 * ADDR 为 bit 级地址（精确到 row/col 的 bit）。
 * 对 PIM_MASK：ADDR 的低 32 位为 mask_value，高位为 base_addr。
 * 对 PIM_WRITE_MUL：ADDR 的低 32 位为 write_value，高位为 base_addr。
 * 对 PIM_MUL_RD / PIM_RD_ALL / PIM_WRITE_MUL：ADDR 的 bit62 作为 BS（0=whole array, 1=single block）。
 */
class ReRAMTrace : public IFrontEnd, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IFrontEnd,
      ReRAMTrace,
      "ReRAMTrace",
      "ReRAM load/store + PIM trace frontend (bit-address)."
  );

private:
  struct Trace {
    int type;
    Addr_t addr;
  };
  std::vector<Trace> m_trace;

  size_t m_trace_length   = 0;
  size_t m_curr_trace_idx = 0;
  size_t m_trace_count    = 0;

  Logger_t m_logger;

public:
  void init() override {
    std::string trace_path_str =
        param<std::string>("path")
            .desc("Path to the ReRAM trace file.")
            .required();
    m_clock_ratio =
        param<uint>("clock_ratio")
            .desc("CPU cycles per memory system cycle.")
            .required();

    m_logger = Logging::create_logger("ReRAMTrace");
    m_logger->info("Loading ReRAM trace file {} ...", trace_path_str);
    init_trace(trace_path_str);
    m_logger->info("Loaded {} lines.", m_trace.size());
  }

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    (void)frontend;
    (void)memory_system;
  }

  void tick() override {
    if (is_finished()) return;

    bool req_full = false;
    while (!req_full && !is_finished()) {
      const Trace& t = m_trace[m_curr_trace_idx];
      bool sent = m_memory_system->send({t.addr, t.type});
      if (sent) {
        m_curr_trace_idx = (m_curr_trace_idx + 1) % m_trace_length;
        m_trace_count++;
      } else {
        req_full = true;
      }
    }
  }

private:
  static int parse_op(const std::string& op, const std::string& file, size_t line_no) {
    // 兼容别名：NOR/SET 等
    if (op == "LD") return Request::Type::Read;
    if (op == "ST") return Request::Type::Write;

    if (op == "PIM_NOR" || op == "NOR") return Request::Type::PIM_NOR;
    if (op == "PIM_SET" || op == "SET") return Request::Type::PIM_SET;

    if (op == "MASK" || op == "PIM_MASK") return Request::Type::PIM_MASK;
    if (op == "MUL_RD" || op == "PIM_MUL_RD") return Request::Type::PIM_MUL_RD;
    if (op == "MV_SINGLE" || op == "PIM_MV_SINGLE") return Request::Type::PIM_MV_SINGLE;
    if (op == "RD_SINGLE" || op == "PIM_RD_SINGLE") return Request::Type::PIM_RD_SINGLE;
    if (op == "RD_ALL" || op == "PIM_RD_ALL") return Request::Type::PIM_RD_ALL;
    if (op == "PIM_WRITE_SINGLE" || op == "WRITE_SINGLE") return Request::Type::PIM_WRITE_SINGLE;
    if (op == "PIM_WRITE_MUL" || op == "WRITE_MUL") return Request::Type::PIM_WRITE_MUL;

    throw ConfigurationError(
        "Trace {} invalid op '{}' at line {}.",
        file, op, line_no);
  }

  static Addr_t parse_addr(const std::string& s, const std::string& file, size_t line_no) {
    try {
      if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0) {
        return static_cast<Addr_t>(std::stoull(s.substr(2), nullptr, 16));
      }
      return static_cast<Addr_t>(std::stoull(s, nullptr, 10));
    } catch (...) {
      throw ConfigurationError(
          "Trace {} invalid address '{}' at line {}.",
          file, s, line_no);
    }
  }

  void init_trace(const std::string& file_path_str) {
    fs::path trace_path(file_path_str);
    if (!fs::exists(trace_path)) {
      throw ConfigurationError("Trace {} does not exist!", file_path_str);
    }

    std::ifstream trace_file(trace_path);
    if (!trace_file.is_open()) {
      throw ConfigurationError("Trace {} cannot be opened!", file_path_str);
    }

    std::string line;
    size_t line_no = 0;
    while (std::getline(trace_file, line)) {
      line_no++;
      std::vector<std::string> tokens;
      tokenize(tokens, line, " \t");

      if (tokens.empty()) continue;
      if (tokens.size() != 2) {
        throw ConfigurationError(
            "Trace {} format invalid at line {}: expect 2 tokens, got {}.",
            file_path_str, line_no, tokens.size());
      }

      const std::string& op = tokens[0];
      const std::string& addr_str = tokens[1];

      int type = parse_op(op, file_path_str, line_no);
      Addr_t addr = parse_addr(addr_str, file_path_str, line_no);

      m_trace.push_back({type, addr});
    }

    trace_file.close();
    m_trace_length = m_trace.size();
  }

  bool is_finished() override {
    return m_trace_count >= m_trace_length;
  }
};

} // namespace Ramulator
