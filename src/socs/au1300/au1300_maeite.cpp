#include <cerfcore/peripherals/peripheral_base.h>

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>
#include "../../peripherals/peripheral_dispatcher.h"

#include <cstdint>

namespace cerfcore {

/* Au1300 MAE/ITE (Media Acceleration Engine / Image Transform Engine),
   PA 0x14010000 per Linux arch/mips/include/asm/mach-au1x00/au1000.h
   (the file does not use that exact name for this block, but this
   project's own earlier MEMPOOL trace showed chevy_mylink nk.exe
   naming reserved RAM regions "MAE" and "ITE", and its ITE driver's
   own init string - "ITE: Driver, v1.0 Sep 1 2012.00:19:47" - printed
   right before the first MMIO access to this exact base confirms the
   name). Region size (0x714 = 1812 bytes) is not a guess: it is the
   exact size argument chevy_mylink nk.exe itself passes to its own
   MapIoSpace-style helper for this base (decompiled VA 0xC0B11280-
   0xC0B11290, a2=1812).

   chevy_mylink nk.exe writes 1 to offset +0x700 here (decompiled VA
   0xC0B112FC) after that mapping call and two other setup calls this
   project has not yet decoded (0xC0B11830, 0xC0B11880/0xC0B11890).
   Nothing about this register's bit layout or purpose is confirmed
   from a primary source - this chip's real Media Acceleration Engine
   is not documented in the public Au1200 data book or in Linux's
   mainline Au1300 support, both already used as this project's
   grounding sources elsewhere. It then writes 2 to +0x704 right after
   (same call site, decompiled VA 0xC0B112FC-0xC0B11304). Both are
   modeled as opaque read/write state, no identity asserted, same
   convention as au1300_unk11003.cpp. Every other offset in this
   0x714-byte block halts per FATAL-FIRST until a real access to it is
   seen. */

namespace {

constexpr uint32_t kMaeItePhysAddr = 0x14010000u;
constexpr uint32_t kMaeIteBlockSize = 0x714u;

constexpr uint32_t kReg700 = 0x700u;
constexpr uint32_t kReg704 = 0x704u;

class Au1300MaeIte : public Peripheral {
public:
    using Peripheral::Peripheral;

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetSocId() == SocId::Au1300;
    }

    void OnReady() override {
        emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) {
            reg700_ = 0;
            reg704_ = 0;
        });
        emu_.Get<PeripheralDispatcher>().Register(this);
    }

    uint32_t MmioBase() const override { return kMaeItePhysAddr; }
    uint32_t MmioSize() const override { return kMaeIteBlockSize; }

    uint32_t ReadWord(uint32_t addr) override {
        switch (addr - MmioBase()) {
            case kReg700: return reg700_;
            case kReg704: return reg704_;
            default: HaltUnsupportedAccess("ReadWord", addr, 0);
        }
    }

    void WriteWord(uint32_t addr, uint32_t value) override {
        switch (addr - MmioBase()) {
            case kReg700: reg700_ = value; return;
            case kReg704: reg704_ = value; return;
            default: HaltUnsupportedAccess("WriteWord", addr, value);
        }
    }

    void SaveState(StateWriter& w) override {
        w.Write("reg700", reg700_);
        w.Write("reg704", reg704_);
    }
    void RestoreState(StateReader& r) override {
        r.Read("reg700", reg700_);
        r.Read("reg704", reg704_);
    }

private:
    uint32_t reg700_ = 0;
    uint32_t reg704_ = 0;
};

}  /* namespace */

REGISTER_SERVICE(Au1300MaeIte);

}
