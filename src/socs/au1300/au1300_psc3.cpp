#include <cerfcore/peripherals/peripheral_base.h>

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>
#include "../../peripherals/peripheral_dispatcher.h"

#include <cstdint>

namespace cerfcore {

/* Au1300 PSC3 (Programmable Serial Controller #3), Linux
   arch/mips/include/asm/mach-au1x00/au1000.h AU1300_PSC3_PHYS_ADDR=
   0x10A03000. Every PSC block shares one "protocol select" register at
   offset +0x00 (Linux au1xxx_psc.h PSC_SEL_OFFSET): the low field picks
   which of SPI/AC97/I2S/SMBus(I2C) the shared hardware implements this
   instance as, before any protocol-specific registers at higher offsets
   become meaningful.

   chevy_mylink nk.exe writes 5 to this register (PA 0x10A03000, decompiled
   VA 0xC0452C9C) before touching anything else here - 5 is Linux's
   PSC_SEL_PS_SMBUSMODE, selecting SMBus/I2C mode for PSC3. This matches
   the chip's own SYS_PINFUNC field for PSC3 (au1000.h SYS_PF_PSC3_SMBUS =
   3 << 20), a named, real hardware configuration this SoC documents -
   not a guess at what PSC3 is wired to on this board.

   Immediately after PSC_SEL, chevy_mylink nk.exe writes 3 to +0x04
   (decompiled VA 0xC0452C9C, same call site) - Linux au1xxx_psc.h's
   PSC_CTRL register, shared by every protocol mode: 3 is PSC_CTRL_ENABLE
   (0=disable, 2=suspend, 3=enable), so this is the driver enabling PSC3
   right after selecting SMBus mode.

   psc_smbus.dll then polls +0x14 (decompiled VA 0xC0A91654-0xC0A91678) in
   a bounded 20-iteration retry loop, testing bit 0 of the read value and
   sleeping between attempts if clear - the standard Au1xxx PSC "State
   Ready" status bit, shared by every protocol's status register, which
   the real hardware sets within a few peripheral-clock cycles of a
   PSC_CTRL enable. Modeled as always-ready once PSC_CTRL has the enable
   bit set, since that enable latency is negligible to a boot-time poll
   and this project has no evidence of a slower real sequencing to model
   instead.

   It then writes -1 (0xFFFFFFFF) to +0x0C and +0x20, and 0xFC000000 to
   +0x08 (VA 0xC0A91690-0xC0A916b0) - SMBus clock-divisor/timing setup
   registers this project does not yet need to interpret, since nothing
   reads them back - and polls +0x14 again (VA 0xC0A916B8-0xC0A916D8,
   50 iterations), this time testing bit 2. Same reasoning as bit 0: the
   real hardware's clock-divisor-applied/stable condition this bit
   reports is immediate relative to a boot-time poll, so it is modeled
   the same way, folded into the same always-ready-once-enabled value.

   Only PSC_SEL, PSC_CTRL, the status bits above, and the three
   timing/divisor registers (stored but not yet interpreted) are modeled;
   every other PSC3 or I2C-core register (FIFO, transaction control,
   address/data) halts per FATAL-FIRST until a real access to one is
   seen. */

namespace {

constexpr uint32_t kPsc3PhysAddr = 0x10A03000u;
constexpr uint32_t kPsc3BlockSize = 0x24u;  /* through the timing register (+0x20) */

constexpr uint32_t kRegSel   = 0x00u;
constexpr uint32_t kRegCtrl  = 0x04u;
constexpr uint32_t kRegTmr0  = 0x08u;
constexpr uint32_t kRegTmr1  = 0x0Cu;
constexpr uint32_t kRegStat  = 0x14u;
constexpr uint32_t kRegTmr2  = 0x20u;
constexpr uint32_t kCtrlEnable = 3u;
constexpr uint32_t kStatReady  = (1u << 0) | (1u << 2);

class Au1300Psc3 : public Peripheral {
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

    uint32_t MmioBase() const override { return kPsc3PhysAddr; }
    uint32_t MmioSize() const override { return kPsc3BlockSize; }

    uint32_t ReadWord(uint32_t addr) override {
        switch (addr - MmioBase()) {
            case kRegSel:  return sel_;
            case kRegCtrl: return ctrl_;
            case kRegTmr0: return tmr0_;
            case kRegTmr1: return tmr1_;
            case kRegTmr2: return tmr2_;
            case kRegStat: return (ctrl_ & kCtrlEnable) ? kStatReady : 0u;
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
    uint32_t sel_ = 0;
    uint32_t ctrl_ = 0;
    uint32_t tmr0_ = 0;
    uint32_t tmr1_ = 0;
    uint32_t tmr2_ = 0;
};

}  /* namespace */

REGISTER_SERVICE(Au1300Psc3);

}
