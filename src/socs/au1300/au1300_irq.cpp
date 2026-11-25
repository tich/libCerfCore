#include "au1300_irq.h"

#include "au1300_id.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include "../../execution/engine_bridge.h"
#include <cerfengine/mips/mips_jit.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>

namespace cerfcore {

namespace {
constexpr uint32_t kIpBase = 10u;  /* CP0.Cause bit for IP2; bank N -> bit (10+N) */
}

bool Au1300Irq::ShouldRegister() {
    auto* bd = emu_.TryGet<BoardContext>();
    return bd && bd->GetSocId() == SocId::Au1300;
}

void Au1300Irq::OnReady() {
    jit_ = &emu_.Get<EngineBridge>().Engine().Get<cerfengine::MipsJit>();
    emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) {
        std::lock_guard<std::mutex> lk(mtx_);
        for (uint32_t& v : ien_) v = 0;
        for (uint32_t& v : ipend_) v = 0;
        RecomputeLocked();
    });
}

void Au1300Irq::RecomputeLocked() {
    uint32_t ip = 0;
    for (uint32_t bank = 0; bank < kBanks; ++bank) {
        if (ipend_[bank] & ien_[bank]) ip |= 1u << (kIpBase + bank);
    }
    jit_->SetExternalInterruptLevel(ip);
}

void Au1300Irq::AssertIrq(int source_bit) {
    if (source_bit < 0 || source_bit >= static_cast<int>(kBanks * 32u)) {
        emu_.Get<Fatal>().Die("Au1300Irq: AssertIrq source %d out of the GPIC's 128-source range",
                              source_bit);
    }
    std::lock_guard<std::mutex> lk(mtx_);
    ipend_[source_bit / 32] |= 1u << (source_bit % 32);
    RecomputeLocked();
}

void Au1300Irq::DeAssertIrq(int source_bit) {
    if (source_bit < 0 || source_bit >= static_cast<int>(kBanks * 32u)) {
        emu_.Get<Fatal>().Die("Au1300Irq: DeAssertIrq source %d out of the GPIC's 128-source "
                              "range", source_bit);
    }
    std::lock_guard<std::mutex> lk(mtx_);
    ipend_[source_bit / 32] &= ~(1u << (source_bit % 32));
    RecomputeLocked();
}

void Au1300Irq::AssertSubIrq(int /*main_source_bit*/, int sub_source_bit) {
    emu_.Get<Fatal>().Die("Au1300Irq: AssertSubIrq(%d) - the GPIC has no sub-interrupt tier",
                          sub_source_bit);
}

uint32_t Au1300Irq::ReadIen(uint32_t bank) const {
    std::lock_guard<std::mutex> lk(mtx_);
    return ien_[bank];
}

void Au1300Irq::SetIen(uint32_t bank, uint32_t bits) {
    std::lock_guard<std::mutex> lk(mtx_);
    ien_[bank] |= bits;
    RecomputeLocked();
}

void Au1300Irq::ClearIen(uint32_t bank, uint32_t bits) {
    std::lock_guard<std::mutex> lk(mtx_);
    ien_[bank] &= ~bits;
    RecomputeLocked();
}

uint32_t Au1300Irq::ReadIpend(uint32_t bank) const {
    std::lock_guard<std::mutex> lk(mtx_);
    return ipend_[bank];
}

void Au1300Irq::SaveState(StateWriter& w) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (uint32_t v : ien_) w.Write("ien", v);
    for (uint32_t v : ipend_) w.Write("ipend", v);
}

void Au1300Irq::RestoreState(StateReader& r) {
    std::lock_guard<std::mutex> lk(mtx_);
    for (uint32_t& v : ien_) r.Read("ien", v);
    for (uint32_t& v : ipend_) r.Read("ipend", v);
}

void Au1300Irq::PostRestore() {
    std::lock_guard<std::mutex> lk(mtx_);
    RecomputeLocked();
}

REGISTER_SERVICE(Au1300Irq);

}
