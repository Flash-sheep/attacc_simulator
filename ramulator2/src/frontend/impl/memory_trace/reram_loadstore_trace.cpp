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
 *   <OP> <PAYLOAD>
 *
 * PAYLOAD 默认按 56-bit 解释为 Addr24|Tail32：
 *   Addr24: BS(1) + Channel(4) + BG(3) + Bank(3) + AG(2) + Array(6) + Block(5)
 *   Tail32: 指令相关字段（input/row/word_col/write/mask 等）
 *
 * 兼容旧格式：若 PAYLOAD 仅有低 24bit，则视作只有 Addr24，Tail32=0。
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
    uint64_t raw_payload;
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
      Request req(static_cast<Addr_t>(t.raw_payload), t.type);
      decode_payload(req, t.raw_payload);
      bool sent = m_memory_system->send(req);
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
    if (op == "PIM_NOT" || op == "NOT") return Request::Type::PIM_NOR;
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

  static uint64_t parse_payload(const std::string& s, const std::string& file, size_t line_no) {
    try {
      if (s.rfind("0x", 0) == 0 || s.rfind("0X", 0) == 0) {
        return std::stoull(s.substr(2), nullptr, 16);
      }
      return std::stoull(s, nullptr, 10);
    } catch (...) {
      throw ConfigurationError(
          "Trace {} invalid payload '{}' at line {}.",
          file, s, line_no);
    }
  }

  static void decode_addr24(Request& req, uint32_t addr24) {
    req.isa_addr24  = addr24;
    req.isa_bs      = (addr24 >> 23) & 0x1;
    req.isa_channel = (addr24 >> 19) & 0xf;
    req.isa_bg      = (addr24 >> 16) & 0x7;
    req.isa_bank    = (addr24 >> 13) & 0x7;
    req.isa_ag      = (addr24 >> 11) & 0x3;
    req.isa_array   = (addr24 >> 5)  & 0x3f;
    req.isa_block   = addr24 & 0x1f;
  }

  static void decode_tail32(Request& req, uint32_t tail32) {
    req.isa_tail32 = tail32;
    req.isa_input1 = static_cast<uint16_t>((tail32 >> 16) & 0xffffu);
    req.isa_input2 = static_cast<uint16_t>(tail32 & 0xffffu);
    req.isa_output1 = req.isa_input2;
    req.isa_write16 = static_cast<uint16_t>(tail32 & 0xffffu);
    req.isa_write32 = tail32;
    req.isa_mask32  = tail32;
    req.isa_row      = (tail32 >> 27) & 0x1f;
    req.isa_word_col = (tail32 >> 16) & 0x7ff;
  }

  static void decode_payload(Request& req, uint64_t raw) {
    req.isa_decoded = true;
    req.isa_raw56   = raw & 0x00ffffffffffffffULL;

    uint32_t addr24 = 0;
    uint32_t tail32 = 0;
    if ((raw >> 24) == 0) {
      // Backward compatibility: payload is only addr24
      addr24 = static_cast<uint32_t>(raw & 0x00ffffffu);
    } else {
      addr24 = static_cast<uint32_t>((raw >> 32) & 0x00ffffffu);
      tail32 = static_cast<uint32_t>(raw & 0xffffffffu);
    }

    decode_addr24(req, addr24);
    decode_tail32(req, tail32);
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
      const std::string& payload_str = tokens[1];

      int type = parse_op(op, file_path_str, line_no);
      uint64_t raw_payload = parse_payload(payload_str, file_path_str, line_no);

      m_trace.push_back({type, raw_payload});
    }

    trace_file.close();
    m_trace_length = m_trace.size();
  }

  bool is_finished() override {
    return m_trace_count >= m_trace_length;
  }
};

} // namespace Ramulator
