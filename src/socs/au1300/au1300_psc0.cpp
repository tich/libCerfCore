#include <cerfcore/peripherals/peripheral_base.h>

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>
#include "../../peripherals/peripheral_dispatcher.h"

#include <cstdint>

namespace cerfcore {

/* Au1300 PSC0 (Programmable Serial Controller #0), Linux
   arch/mips/include/asm/mach-au1x00/au1000.h AU1300_PSC0_PHYS_ADDR=
   0x10A00000. Same shared PSC block as the sibling Au1300Psc2/Au1300Psc3
   (PSC_SEL at +0x00, PSC_CTRL at +0x04, status at +0x14 - see
   au1300_psc3.cpp's longer grounding comment for the shared-register
   rationale).

   chevy_mylink nk.exe writes 0x13 to PSC_SEL here (PA 0x10A00000,
   decompiled VA 0xC0452C9C - the same shared generic PSC-init call
   site PSC2/PSC3's SEL writes use). 0x13 = 0b00010011: bits[2:0]=3 is
   Linux's PSC_SEL_PS_I2SMODE, the same protocol PSC2 selects, matching
   the "Initializing PSC0 for I2S2 operation" string this project
   observed immediately before the write - a second, independent I2S
   instance, not a guess. Bit 4 falls in Linux's PSC_SEL_CLK_MASK (bits
   5:3), same field as PSC2's bit 5 - a different clock-source
   selection, plausible since each PSC instance can route its own
   clock source independently and nothing here needs them to match.

   PSC_CTRL enable (3) and the status-bit flip-on-read modeling
   (bit 1 toggling, bits 0+2 reported SET once enabled) are carried
   over unchanged from Au1300Psc2 - same I2S protocol, same
   wait-clear-then-wait-set handshake expected from psc_i2s.dll's
   shared init routine (not yet independently re-confirmed for PSC0's
   own call site, but there is no reason for a second instance of the
   same driver's init routine to behave differently).

   Only PSC_SEL, PSC_CTRL, the status bits above, and the same three
   timing/divisor register offsets PSC2/PSC3 model (stored but not yet
   interpreted) are modeled; every other PSC0 or I2S-core register
   halts per FATAL-FIRST until a real access to one is seen. */

namespace {

constexpr uint32_t kPsc0PhysAddr = 0x10A00000u;
constexpr uint32_t kPsc0BlockSize = 0x24u;  /* through the timing register (+0x20) */

constexpr uint32_t kRegSel   = 0x00u;
constexpr uint32_t kRegCtrl  = 0x04u;
constexpr uint32_t kRegTmr0  = 0x08u;
constexpr uint32_t kRegTmr1  = 0x0Cu;
constexpr uint32_t kRegStat  = 0x14u;
constexpr uint32_t kRegTmr2  = 0x20u;
constexpr uint32_t kCtrlEnable = 3u;
constexpr uint32_t kStatReady  = (1u << 0) | (1u << 2);
constexpr uint32_t kStatBusy   = (1u << 1);

class Au1300Psc0 : public Peripheral {
public:
    using Peripheral::Peripheral;

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetSocId() == SocId::Au1300;
    }

    void OnReady() override {
        emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) {
            sel_ = 0;
            ctrl_ = 0;
            tmr0_ = 0;
            tmr1_ = 0;
            tmr2_ = 0;
        });
        emu_.Get<PeripheralDispatcher>().Register(this);
    }

    uint32_t MmioBase() const override { return kPsc0PhysAddr; }
    uint32_t MmioSize() const override { return kPsc0BlockSize; }

    uint32_t ReadWord(uint32_t addr) override {
        switch (addr - MmioBase()) {
            case kRegSel:  return sel_;
            case kRegCtrl: return ctrl_;
            case kRegTmr0: return tmr0_;
            case kRegTmr1: return tmr1_;
            case kRegTmr2: return tmr2_;
            case kRegStat: {
                if (!(ctrl_ & kCtrlEnable)) return 0u;
                stat_toggle_ ^= kStatBusy;
                return kStatReady | stat_toggle_;
            }
            default: HaltUnsupportedAccess("ReadWord", addr, 0);
        }
    }

    void WriteWord(uint32_t addr, uint32_t value) override {
        switch (addr - MmioBase()) {
            case kRegSel:  sel_ = value;  return;
            case kRegCtrl: ctrl_ = value; return;
            case kRegTmr0: tmr0_ = value; return;
            case kRegTmr1: tmr1_ = value; return;
            case kRegTmr2: tmr2_ = value; return;
            default: HaltUnsupportedAccess("WriteWord", addr, value);
        }
    }

    void SaveState(StateWriter& w) override {
        w.Write("sel", sel_);
        w.Write("ctrl", ctrl_);
        w.Write("tmr0", tmr0_);
        w.Write("tmr1", tmr1_);
        w.Write("tmr2", tmr2_);
    }
    void RestoreState(StateReader& r) override {
        r.Read("sel", sel_);
        r.Read("ctrl", ctrl_);
        r.Read("tmr0", tmr0_);
        r.Read("tmr1", tmr1_);
        r.Read("tmr2", tmr2_);
    }

private:
    uint32_t stat_toggle_ = 0;  /* modeling artifact only, not real register state */
    uint32_t sel_ = 0;
    uint32_t ctrl_ = 0;
    uint32_t tmr0_ = 0;
    uint32_t tmr1_ = 0;
    uint32_t tmr2_ = 0;
};

}  /* namespace */

REGISTER_SERVICE(Au1300Psc0);

}
