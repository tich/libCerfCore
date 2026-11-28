#include <cerfcore/peripherals/peripheral_base.h>

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include "au1300_ehci.h"
#include <cerfcore/core/cerf_emulator.h>
#include "../../peripherals/peripheral_dispatcher.h"

#include <cstdint>

namespace cerfcore {

/* Au1300 EHCI capability-register block, PA 0x14020000 - this is the
   "ehci_local0" controller USBware's real EHCI driver discovers and
   uses (confirmed by the NKDBG string "ehci_local0: EHCI version 1.0,
   with 4 ports" appearing right after reading this block's +0x00/+0x04
   words, decompiled VA 0xC0B6AAD8). It reads CAPLENGTH=0x40 here, then
   immediately reads offset +0x40 (0x14020040 = cap_base+CAPLENGTH) -
   the textbook EHCI 1.0 "operational registers start at
   capbase+CAPLENGTH" discovery this project's OTHER EHCI access
   (au1300_ehci.cpp's 0x14022000, confirmed much earlier in boot, via
   a different code path/call site entirely, before USBware even
   loads) evidently does NOT do - that earlier access hardcodes
   0x14022000 as if it were already the operational base with no
   preamble. Rather than guess whether these are the same physical
   core accessed two different ways, or two different cores, this
   class forwards every operational-register access at +0x40 and
   above straight into the existing Au1300Ehci instance (translating
   this block's capbase-relative address into Au1300Ehci's own
   0x14022000-based one) so there is exactly one piece of register/
   schedule state, not two independently-tracked copies that could
   drift apart.

   +0x00 (CAPLENGTH+HCIVERSION) and +0x04 (HCSPARAMS) are placeholders
   inherited from i.MX51's own ChipIdea/EHCI capability block
   (imx51_usboh3.cpp's kCapReset/kHcsparamsOnePort), not independently
   confirmed for this chip beyond the NKDBG string's own "version 1.0,
   with 4 ports" readback confirming HCIVERSION=0x0100 and
   HCSPARAMS.N_PORTS=4 both parse the way this project intended.
   HCCPARAMS (+0x08) and anything else in this small preamble halts
   per FATAL-FIRST until a real access to it is seen. */

namespace {

constexpr uint32_t kBase = 0x14020000u;
constexpr uint32_t kSize = 0x00000098u;  /* through PORTSC4 (CAPLENGTH 0x40 + 0x54 + 4) */
constexpr uint32_t kCapLength = 0x40u;
constexpr uint32_t kOffCapLengthHciVersion = 0x00u;
constexpr uint32_t kOffHcsparams = 0x04u;
constexpr uint32_t kOffHccparams = 0x08u;
constexpr uint32_t kCapLengthHciVersion = 0x01000040u;
constexpr uint32_t kHcsparamsFourPorts = 4u;
/* EHCI 1.0 Spec Table 2-6 (p14): bit 0 (64-bit addressing capability) left
   clear - this project's EmulatedMemory/physical addressing is 32-bit
   throughout, so claiming 64-bit capability would be actively wrong,
   not just unconfirmed. Every other HCCPARAMS field (isochronous
   scheduling threshold, etc.) is left 0 as an unconfirmed placeholder,
   same reasoning as HCSPARAMS. */
constexpr uint32_t kHccparams = 0x00000000u;

class Au1300EhciCap : public Peripheral {
public:
    using Peripheral::Peripheral;

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetSocId() == SocId::Au1300;
    }

    void OnReady() override {
        ehci_ = &emu_.Get<Au1300Ehci>();
        emu_.Get<PeripheralDispatcher>().Register(this);
    }

    uint32_t MmioBase() const override { return kBase; }
    uint32_t MmioSize() const override { return kSize; }

    uint32_t ReadWord(uint32_t addr) override {
        const uint32_t off = addr - kBase;
        switch (off) {
            case kOffCapLengthHciVersion: return kCapLengthHciVersion;
            case kOffHcsparams:           return kHcsparamsFourPorts;
            case kOffHccparams:           return kHccparams;
            default:
                if (off >= kCapLength) return ehci_->ReadWord(Au1300Ehci::kOperationalBase + (off - kCapLength));
                HaltUnsupportedAccess("ReadWord", addr, 0);
        }
    }

    void WriteWord(uint32_t addr, uint32_t value) override {
        const uint32_t off = addr - kBase;
        if (off >= kCapLength) {
            ehci_->WriteWord(Au1300Ehci::kOperationalBase + (off - kCapLength), value);
            return;
        }
        HaltUnsupportedAccess("WriteWord", addr, value);
    }

private:
    Au1300Ehci* ehci_ = nullptr;
};

}  /* namespace */

REGISTER_SERVICE(Au1300EhciCap);

}
