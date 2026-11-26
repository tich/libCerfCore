#include <cerfcore/peripherals/peripheral_base.h>

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include "au1300_irq.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>
#include "../../peripherals/peripheral_dispatcher.h"

#include <cstdint>

namespace cerfcore {

/* Au1300 GPIC (the chip's combined GPIO + interrupt controller, replacing
   the classic Au1000-style dual IC0/IC1 the rest of the Alchemy family
   uses). Real register offsets, confirmed from Linux arch/mips/include/
   asm/mach-au1x00/au1000.h (AU1300_GPIC_PHYS_ADDR=0x10200000,
   AU1300_GPIC_IEN=0x40 "int_mask in manual", AU1300_GPIC_IDIS=0x50
   "int_maskclr in manual", AU1300_GPIC_DMASEL=0x60, AU1300_GPIC_PINCFG=
   0x1000) and from GPIC_GPIO_BANKOFF(gpio) in gpio-au1300.h, which strides
   IEN/IDIS/DMASEL by 4 bytes per 32-source bank (4 banks, confirmed by
   Linux's own alchemy_gpic_init_irq(): "bank_base = AU1300_GPIC_ADDR +
   (i*4)" for i in 0..3).

   Every register below is further confirmed directly against chevy_mylink
   nk.exe's own OEMInterruptInit (decompiled VA 0x801076A0-0x80107758):
     - It writes 0xFFFFFFFF to IDIS banks 0-3 (+0x50/+0x54/+0x58/+0x5C) -
       "+OEMInterruptInit Disable All..." in the NKDBG trace.
     - It writes two DMASEL (bank 0, +0x60) values for device-specific pin
       routing setup, matching Linux au1300_pinfunc_to_dev()'s DMASEL use.
     - It walks a fixed table of {source index, ?, enable flag, pincfg
       value} entries (terminated by source index -1): for each entry it
       writes the pincfg value to PINCFG[source] (+0x1000 + source*4,
       exactly GPIC_GPIO_TO_BIT/BANKOFF's per-source word stride) and, if
       the enable flag is set, ORs the source's bit into IEN[source>>5].
   IDIS is modeled as a write-only alias that clears bits out of the same
   mask IEN sets (the standard SET/CLR register-pair idiom this same
   family's older IC_CFG0SET/IC_CFG0CLR pattern also uses - not specific
   to this chip, so not a guess at Au1300-specific behavior).

   PINVAL (+0x00, Linux au1000.h AU1300_GPIC_PINVAL) and PINVALCLR (+0x10,
   AU1300_GPIC_PINVALCLR) are modeled too: a single 32-bit GPIO output
   level register plus its write-only "clear bit" alias, the same
   SET/CLR idiom IEN/IDIS already use. chevy_mylink nk.exe writes
   0x00800000 to PINVALCLR (decompiled VA 0xC0AB8770) - clearing GPIO 23 -
   right after the PSC3/SMBus bring-up this file's sibling models,
   consistent with driving a GPIO (e.g. an I2C-attached device's reset or
   enable line) low as part of that same init sequence.

   IEN/IDIS and the new IPEND (+0x20, Linux au1000.h AU1300_GPIC_IPEND)
   register now delegate to the sibling Au1300Irq (au1300_irq.h/.cpp),
   which owns the enable/pending state and routes each bank onto one MIPS
   hardware interrupt line (IP2-IP5), since chevy_mylink's boot settles
   into an idle loop with CP0.Status IE and IM[IP2-5,IP7] all set but no
   interrupt source ever firing - confirmed by instrumented tracing
   showing zero exception dispatches for the ~13+ s the idle loop spins,
   and the only enabled GPIC sources (captured directly from the OAL's
   own IEN writes during OEMInterruptInit) are bank 2 bits 11/20/21 =
   sources 75 (AU1300_DDMA_INT), 84 (AU1300_RTC_INT) and 85
   (AU1300_RTC_MATCH0_INT). The sibling au1300_sys.cpp now drives
   RTC_MATCH0_INT periodically from a real RTC-counter/match comparison,
   the only one of these three with an obvious periodic-tick role.

   PRIENC (+0x30) - the lowest-numbered pending-and-enabled source
   across all 4 banks, or 127 if none - is confirmed directly from
   chevy_mylink nk.exe's own ISR dispatcher at VA 0x8010649C: it reads
   this register, checks it against 127 first (nothing pending), then
   against specific source numbers to pick a handler.

   DEVSEL/DEVCLR and RSTVAL are not implemented yet - no real access to
   either has been seen, so they halt per FATAL-FIRST until one is. */

namespace {

constexpr uint32_t kGpicPhysAddr = 0x10200000u;
constexpr uint32_t kGpicBlockSize = 0x1200u;  /* through PINCFG[127] (+0x1000..+0x11FC) */

constexpr uint32_t kPinVal    = 0x00u;
constexpr uint32_t kPinValClr = 0x10u;
constexpr uint32_t kIpend    = 0x20u;  /* banks 0-3: +0x20, +0x24, +0x28, +0x2C */
constexpr uint32_t kPriEnc   = 0x30u;  /* lowest-numbered pending&enabled source, or 127 */
constexpr uint32_t kIen      = 0x40u;  /* banks 0-3: +0x40, +0x44, +0x48, +0x4C */
constexpr uint32_t kIdis     = 0x50u;  /* banks 0-3: +0x50, +0x54, +0x58, +0x5C */
constexpr uint32_t kDmaSel   = 0x60u;  /* banks 0-3: +0x60, +0x64, +0x68, +0x6C */
constexpr uint32_t kPinCfg   = 0x1000u; /* 128 words: +0x1000 .. +0x11FC */

class Au1300Gpic : public Peripheral {
public:
    using Peripheral::Peripheral;

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetSocId() == SocId::Au1300;
    }

    void OnReady() override {
        irq_ = &emu_.Get<Au1300Irq>();
        emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) {
            pinval_ = 0;
            for (uint32_t& v : dmasel_) v = 0;
            for (uint32_t& v : pincfg_) v = 0;
        });
        emu_.Get<PeripheralDispatcher>().Register(this);
    }

    uint32_t MmioBase() const override { return kGpicPhysAddr; }
    uint32_t MmioSize() const override { return kGpicBlockSize; }

    uint32_t ReadWord(uint32_t addr) override {
        const uint32_t off = addr - MmioBase();
        if (off == kPinVal) return pinval_;
        if (off >= kIpend && off < kIpend + 16u) return irq_->ReadIpend((off - kIpend) / 4u);
        if (off == kPriEnc) return irq_->ReadPrienc();
        if (off >= kIen && off < kIen + 16u)    return irq_->ReadIen((off - kIen) / 4u);
        if (off >= kDmaSel && off < kDmaSel + 16u) return dmasel_[(off - kDmaSel) / 4u];
        if (off >= kPinCfg && off < kPinCfg + kNumSources * 4u) {
            return pincfg_[(off - kPinCfg) / 4u];
        }
        HaltUnsupportedAccess("ReadWord", addr, 0);
    }

    void WriteWord(uint32_t addr, uint32_t value) override {
        const uint32_t off = addr - MmioBase();
        if (off == kPinVal)    { pinval_ = value;      return; }
        if (off == kPinValClr) { pinval_ &= ~value;    return; }
        if (off >= kIpend && off < kIpend + 16u) {
            irq_->ClearIpend((off - kIpend) / 4u, value);
            return;
        }
        if (off >= kIen && off < kIen + 16u) { irq_->SetIen((off - kIen) / 4u, value); return; }
        if (off >= kIdis && off < kIdis + 16u) { irq_->ClearIen((off - kIdis) / 4u, value); return; }
        if (off >= kDmaSel && off < kDmaSel + 16u) { dmasel_[(off - kDmaSel) / 4u] = value; return; }
        if (off >= kPinCfg && off < kPinCfg + kNumSources * 4u) {
            pincfg_[(off - kPinCfg) / 4u] = value;
            return;
        }
        HaltUnsupportedAccess("WriteWord", addr, value);
    }

    void SaveState(StateWriter& w) override {
        w.Write("pinval", pinval_);
        for (uint32_t v : dmasel_) w.Write("dmasel", v);
        for (uint32_t v : pincfg_) w.Write("pincfg", v);
    }

    void RestoreState(StateReader& r) override {
        r.Read("pinval", pinval_);
        for (uint32_t& v : dmasel_) r.Read("dmasel", v);
        for (uint32_t& v : pincfg_) r.Read("pincfg", v);
    }

private:
    static constexpr uint32_t kNumSources = 128u;

    Au1300Irq* irq_ = nullptr;
    uint32_t pinval_ = 0;
    uint32_t dmasel_[4] = {0, 0, 0, 0};
    uint32_t pincfg_[kNumSources] = {};
};

}  /* namespace */

REGISTER_SERVICE(Au1300Gpic);

}
