#include <cerfcore/peripherals/peripheral_base.h>

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>
#include "../../peripherals/peripheral_dispatcher.h"

#include <cstdint>

namespace cerfcore {

/* Au1300 PSC2 (Programmable Serial Controller #2), Linux
   arch/mips/include/asm/mach-au1x00/au1000.h AU1300_PSC2_PHYS_ADDR=
   0x10A02000. Same shared PSC block as the sibling Au1300Psc3 (PSC_SEL
   at +0x00, PSC_CTRL at +0x04, status at +0x14 - see that file's longer
   grounding comment for the shared-register rationale, which applies
   here unchanged since PSC_CTRL/status are explicitly protocol-
   independent in Linux's au1xxx_psc.h).

   chevy_mylink nk.exe writes 0x23 to PSC_SEL here (PA 0x10A02000,
   decompiled VA 0xC0452C9C - the exact same call site PSC3's SEL write
   uses, confirming this is one shared generic PSC-init routine called
   per-instance, not PSC2-specific code). 0x23 = 0b00100011: bits[2:0]=3
   is Linux's PSC_SEL_PS_I2SMODE (its PSC_SEL_PS_SMBUSMODE=5 is what
   PSC3 selects instead, per that file's comment), and bit 5 falls in
   Linux's PSC_SEL_CLK_MASK (bits 5:3) - a clock-source select field I2S
   needs (an explicit bit/frame clock source) that SMBus has no use for,
   consistent with "Initializing PSC2 for I2S operation" being the UART2
   string this project observed immediately before the write. Only the
   protocol differs from PSC3's grounding; the register is still stored
   verbatim, not decoded, since nothing yet reads individual SEL bits
   back.

   PSC_CTRL enable (3) is carried over unchanged from PSC3 (shared,
   protocol-independent per Linux). The status bits are NOT carried
   over as-is, though: live tracing of psc_i2s.dll's init (decompiled
   VA 0xC0AB8958-0xC0AB8968, called twice back-to-back with different
   $a1 values during PSC2 bring-up) showed it polling +0x14 bit 1 and
   looping while that bit is SET ("BNE $a0,$zero,-20" on
   "$a0 = stat & 2") - i.e. it waits for the bit to go LOW, the
   opposite polarity from PSC3's SMBus bit 1 ("DR", waited HIGH). Same
   bit position, different protocol-specific meaning (most likely a
   busy/FIFO-full flag here, vs. SMBus's data-ready flag there) - both
   plausible given Linux's au1xxx_psc.h documents status bit meanings
   as per-protocol overlays on one shared register, not one fixed
   meaning.

   A few instructions later the SAME function polls the SAME bit again
   with the OPPOSITE test ("BEQ $a0,$zero,-20" - loop while CLEAR,
   i.e. wait for it to go HIGH): a wait-clear-then-wait-set handshake,
   so no constant value can satisfy both polls. Modeled as flipping on
   every read of this register - the real hardware condition behind it
   (FIFO/transfer state advancing) changes on its own timescale this
   project has no need to simulate exactly, and a flip-per-read
   guarantees each poll sees the transition it is waiting for within
   at most one extra read, which is what "negligible real latency
   relative to a boot-time poll" (PSC3's own reasoning) means here.
   Bits 0 and 2 stay reported SET once enabled, same as PSC3.

   Only PSC_SEL, PSC_CTRL, the status bits above, and the same three
   timing/divisor register offsets PSC3 models (stored but not yet
   interpreted) are modeled; every other PSC2 or I2S-core register
   (FIFO, DMA, frame/bit clock dividers) halts per FATAL-FIRST until a
   real access to one is seen. */

namespace {

constexpr uint32_t kPsc2PhysAddr = 0x10A02000u;
constexpr uint32_t kPsc2BlockSize = 0x24u;  /* through the timing register (+0x20) */

constexpr uint32_t kRegSel   = 0x00u;
constexpr uint32_t kRegCtrl  = 0x04u;
constexpr uint32_t kRegTmr0  = 0x08u;
constexpr uint32_t kRegTmr1  = 0x0Cu;
constexpr uint32_t kRegStat  = 0x14u;
constexpr uint32_t kRegTmr2  = 0x20u;
constexpr uint32_t kCtrlEnable = 3u;
constexpr uint32_t kStatReady  = (1u << 0) | (1u << 2);
constexpr uint32_t kStatBusy   = (1u << 1);

class Au1300Psc2 : public Peripheral {
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

    uint32_t MmioBase() const override { return kPsc2PhysAddr; }
    uint32_t MmioSize() const override { return kPsc2BlockSize; }

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

REGISTER_SERVICE(Au1300Psc2);

}
