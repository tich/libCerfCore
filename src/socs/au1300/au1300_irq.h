#pragma once

#include "../irq_controller.h"

#include <cstdint>
#include <mutex>

namespace cerfengine { class MipsJit; }

namespace cerfcore {

class StateWriter;
class StateReader;

/* Au1300 GPIC's interrupt-delivery half (the MMIO register decode lives in
   the sibling Au1300Gpic Peripheral, same convention as
   Pxa2xxIntc/Pr31x00Intc: one IrqController-derived class owns the
   enable/pending state and drives the CPU's external interrupt level, a
   separate Peripheral forwards register accesses into it).

   128 sources across 4 banks of 32, each bank wired to one MIPS hardware
   interrupt line IP2-IP5 (Linux arch/mips/alchemy/common/gpioint.c routes
   GPIC "int group" N to MIPS_CPU_IRQ_BASE+2+N; this project's own earlier
   grounding of this chip's GPIC, in au1300_gpic.cpp, already names
   "IP2-IP5" for this exact reason). CP0.Cause IP2 is bit 10, so bank N's
   line is bit (10+N). */
class Au1300Irq : public IrqController {
public:
    using IrqController::IrqController;

    bool ShouldRegister() override;
    void OnReady() override;

    void AssertIrq   (int source_bit) override;
    void DeAssertIrq (int source_bit) override;
    void AssertSubIrq(int main_source_bit, int sub_source_bit) override;

    uint32_t ReadIen (uint32_t bank) const;
    void     SetIen  (uint32_t bank, uint32_t bits);
    void     ClearIen(uint32_t bank, uint32_t bits);
    uint32_t ReadIpend(uint32_t bank) const;

    void SaveState(StateWriter& w);
    void RestoreState(StateReader& r);
    void PostRestore();

private:
    static constexpr uint32_t kBanks = 4u;

    void RecomputeLocked();

    mutable std::mutex  mtx_;
    cerfengine::MipsJit* jit_ = nullptr;

    uint32_t ien_[kBanks]   = {};
    uint32_t ipend_[kBanks] = {};
};

}
