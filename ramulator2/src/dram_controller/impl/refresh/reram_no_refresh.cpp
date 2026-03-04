#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/refresh.h"

namespace Ramulator {

class ReRAMNoRefresh : public IRefreshManager, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(
      IRefreshManager,
      ReRAMNoRefresh,
      "ReRAMNo",
      "No Refresh scheme for ReRAM (true no-op)."
  );

private:
  Clk_t           m_clk  = 0;
  IDRAM*          m_dram = nullptr;
  IDRAMController* m_ctrl = nullptr;

public:
  void init() override {
    m_ctrl = cast_parent<IDRAMController>();
  };

  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
    m_dram = m_ctrl->m_dram;
    // 不再访问 rank / nREFI / all-bank-refresh
  };

  void tick() override {
    ++m_clk;
    // 完全不发刷新请求
  };
};

} // namespace Ramulator
