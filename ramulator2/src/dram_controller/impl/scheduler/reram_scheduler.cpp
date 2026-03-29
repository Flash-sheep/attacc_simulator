#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/scheduler.h"
#include "memory_system/memory_system.h"

namespace Ramulator {

/**
 * @brief  ReRAM Scheduler
 * 
 * 策略：
 *  1. 先为 buffer 中所有请求计算当前应该发出的 command（get_preq_command）。
 *  2. 比较时优先选择 “当前 cycle 内 ready 的请求”。
 *  3. 若都 ready 或都不 ready，则退化为 FCFS（到达时间越早优先级越高）。
 * 
 * 由于你的 ReRAM 模型没有 row-buffer 概念，这里不再使用 rowhit_list 之类优化。
 */
class ReRAMScheduler : public IScheduler, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IScheduler,
      ReRAMScheduler,
      "ReRAM",
      "Simple readiness-aware FCFS scheduler for ReRAM."
  )

private:
  IDRAM* m_dram = nullptr;
  IMemorySystem* m_memory_system = nullptr;

  void ensure_dram() {
    if (m_dram == nullptr) {
      if (m_memory_system != nullptr) {
        m_dram = m_memory_system->get_ifce<IDRAM>();
      }
      if (m_dram == nullptr) {
        auto* ctrl = cast_parent<IDRAMController>();
        if (ctrl != nullptr) {
          m_dram = ctrl->m_dram;
        }
      }
    }
    if (m_dram == nullptr) {
      throw std::runtime_error("ReRAMScheduler: DRAM interface is not initialized.");
    }
  }

public:
  void init() override { }

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    m_memory_system = memory_system;
    // 从父对象（Controller）拿到 DRAM 设备指针
    m_dram = memory_system->get_ifce<IDRAM>();
  }

  /**
   * @brief 比较两个候选请求，返回“更优”的那个 iterator
   */
  ReqBuffer::iterator compare(ReqBuffer::iterator req1,
                              ReqBuffer::iterator req2) override {
    ensure_dram();
    bool ready1 = m_dram->check_ready(req1->command, req1->addr_vec);
    bool ready2 = m_dram->check_ready(req2->command, req2->addr_vec);

    // 1. 只有一个 ready，则优先选择 ready 的
    if (ready1 ^ ready2) {
      return ready1 ? req1 : req2;
    }

    // 2. 都 ready 或都不 ready，则走 FCFS：arrive 越早优先级越高
    if (req1->arrive <= req2->arrive) {
      return req1;
    } else {
      return req2;
    }
  }

  /**
   * @brief 在一个 ReqBuffer 内挑选“最好”的请求
   *
   * 注意：这里会为 buffer 中所有请求调用一次 get_preq_command，
   *       即把它们的 command 更新成当前应当执行的“下一条子命令”。
   */
  ReqBuffer::iterator get_best_request(ReqBuffer& buffer) override {
    ensure_dram();
    if (buffer.size() == 0) {
      return buffer.end();
    }

    // 先为每个请求计算当前要发出的 command
    for (auto& req : buffer) {
      req.command = m_dram->get_preq_command(req.final_command, req.addr_vec);
    }

    // 初始候选为第一个
    auto candidate = buffer.begin();

    // 不需要 PIM_BARRIER 相关逻辑：ReRAM 目前只支持 RD/WR/NOR/SET
    for (auto next = std::next(buffer.begin(), 1); next != buffer.end(); ++next) {
      candidate = compare(candidate, next);
    }

    return candidate;
  }
};

} // namespace Ramulator
