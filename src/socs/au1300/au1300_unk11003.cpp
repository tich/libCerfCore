#include <cerfcore/peripherals/peripheral_base.h>

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>
#include "../../peripherals/peripheral_dispatcher.h"

#include <cstdint>

namespace cerfcore {

/* Au1300 peripheral at PA 0x11003000, identity NOT confirmed. Searched
   Linux's arch/mips/include/asm/mach-au1x00/au1000.h (the "Physical
   base addresses for integrated peripherals" block, which tags every
   entry with a 0..5 per-SoC index) for any 0x1100xxxx constant tagged
   for Au1300 (index 5) - none exists. The only 0x11000000-range entry
   in that file at all is AU1000_I2S_PHYS_ADDR, tagged for Au1000/
   Au1100 only, not Au1300. This device may be hardware Linux's Au1300
   support never touches, or GM/supplier-specific.

   chevy_mylink nk.exe reaches it through the same generic table-driven
   "write this word to that device register" OAL helper this project
   has already seen used for GPIC PINCFG and PSC_SEL writes (decompiled
   VA 0x80106100-0x801061A0, calling the same trampoline region as
   those - 0x8010B1EC vs. 0x80111DDC et al.), writing small constants
   (1, 2, 3, 7, 15, 31) to +0x10 and +0x14, and 0 / 0x01FFFFFF to +0x0C,
   across what looks like two branches of an if/else that both
   converge on the same final write to +0x14 (value 15, the one this
   project's halt actually hit - confirmed by the fatal's own
   "pa=0x11003014" against this file's base of 0x11003000). No bit
   pattern here matches anything
   this project has confirmed (not a PSC_SEL mode+clock field, not a
   GPIC bank mask, not an interrupt source number). Modeled as opaque
   read/write state with no asserted identity - same convention
   au1300_uart.h already uses for its own unconfirmed offsets - because
   guessing a device name or register meaning here would be exactly
   the kind of unverified claim this project's rules forbid. Every
   offset beyond the three confirmed so far halts per FATAL-FIRST until
   a real access to it is seen. */

namespace {

constexpr uint32_t kUnk11003PhysAddr = 0x11003000u;
constexpr uint32_t kUnk11003BlockSize = 0x18u;

constexpr uint32_t kRegC = 0x0Cu;
constexpr uint32_t kReg10 = 0x10u;
constexpr uint32_t kReg14 = 0x14u;

class Au1300Unk11003 : public Peripheral {
public:
    using Peripheral::Peripheral;

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetSocId() == SocId::Au1300;
    }

    void OnReady() override {
        emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) {
            regC_ = 0;
            reg10_ = 0;
            reg14_ = 0;
        });
        emu_.Get<PeripheralDispatcher>().Register(this);
    }

    uint32_t MmioBase() const override { return kUnk11003PhysAddr; }
    uint32_t MmioSize() const override { return kUnk11003BlockSize; }

    uint32_t ReadWord(uint32_t addr) override {
        switch (addr - MmioBase()) {
            case kRegC:  return regC_;
            case kReg10: return reg10_;
            case kReg14: return reg14_;
            default: HaltUnsupportedAccess("ReadWord", addr, 0);
        }
    }

    void WriteWord(uint32_t addr, uint32_t value) override {
        switch (addr - MmioBase()) {
            case kRegC:  regC_ = value;  return;
            case kReg10: reg10_ = value; return;
            case kReg14: reg14_ = value; return;
            default: HaltUnsupportedAccess("WriteWord", addr, value);
        }
    }

    void SaveState(StateWriter& w) override {
        w.Write("regC", regC_);
        w.Write("reg10", reg10_);
        w.Write("reg14", reg14_);
    }
    void RestoreState(StateReader& r) override {
        r.Read("regC", regC_);
        r.Read("reg10", reg10_);
        r.Read("reg14", reg14_);
    }

private:
    uint32_t regC_ = 0;
    uint32_t reg10_ = 0;
    uint32_t reg14_ = 0;
};

}  /* namespace */

REGISTER_SERVICE(Au1300Unk11003);

}
