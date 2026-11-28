#include "au1300_ohci.h"

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include "au1300_irq.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include "../../cpu/physical_bus.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>

#include <cstdint>

namespace cerfcore {

namespace {

/* Au1300 OHCI0/OHCI1 - the two DesignWare-shared OHCI blocks sibling to
   the EHCI core this project already modeled (au1300_ehci.cpp/.h).
   OHCI1's base (0x14022800) is confirmed live: chevy_mylink nk.exe
   writes 0xFFFFFFFF (decompiled VA 0xC098441C, the same generic
   MMIO-write trampoline this project has seen reused everywhere) to
   PA 0x14022814 - i.e. base+0x14, and offset 0x14 in the standard
   OHCI 1.0a register map is HcInterruptDisable. Writing all-1s there
   to disable every interrupt before touching anything else is the
   exact same "disable everything first" idiom this project has
   already seen for DBDMA_CONF, USB_CTL's INT_ENABLE, and PSC3's own
   "Disable All" OAL string - independent confirmation this is
   HcInterruptDisable and not some other register, and that
   0x14022800 is a real OHCI base. This is the SAME early, OAL-layer
   access pattern au1300_ehci.cpp's 0x14022000 base came from, before
   USBware's real driver stack loads.

   A SEPARATE, later access from USBware's own OHCI driver
   ("ohci_local0:") reaches OHCI0 via PA 0x14020800 - the EHCI
   capability block's base (0x14020000, au1300_ehci_cap.cpp) + 0x800,
   mirroring exactly how USBware's "ehci_local0" driver separately
   reached EHCI via capbase+CAPLENGTH while an earlier OAL-layer touch
   used a different hardcoded address for the same hardware. Rather
   than track two independent copies of OHCI0's register/schedule
   state, au1300_ohci_cap.cpp forwards that address window into this
   same Au1300Ohci0 instance, the same approach au1300_ehci_cap.cpp
   already uses for Au1300Ehci.

   OHCI0's "early" base (0x14022400) is NOT independently confirmed
   the same way OHCI1's was - it follows by analogy from the +0x400/
   +0x800 EHCI-to-OHCI0/OHCI1 spacing an earlier (Android-tree) Au1300
   header suggested, shifted by the same +0x2000 offset this
   project's own live-traced EHCI base required relative to that
   header. If a real access to it (at this address, not through the
   capbase+0x800 alias) ever halts, re-derive it the same way OHCI1's
   was confirmed rather than trusting this inference further.

   Reuses the project's existing OhciHostController (src/peripherals/
   usb/ohci_host_controller.{h,cpp}) exactly as S3C2410UsbHost does
   (s3c2410_usb_host.cpp) - register I/O through OhciHostController::
   Read/Write, ED/TD bus-master DMA through PhysicalBus, and the
   controller's own interrupt output asserting GPIC source 90 (Linux
   au1000.h AU1300_USB_INT - the SAME shared line au1300_ehci.cpp uses,
   since Au1300UsbCtl's single USB_INT_ENABLE register covers EHCI/
   OHCI0/OHCI1/UDC/OTG together).

   Like PXA27x's own OHCI wrapper, this does not attach any USB device
   to either OHCI instance - OhciHostController itself only tracks
   register/schedule state, it does not walk EDs/TDs against a
   UsbDevice the way au1300_ehci_host.cpp's engine does for EHCI. If a
   real device ever needs to enumerate over Au1300's OHCI ports rather
   than its EHCI ports, OhciHostController would need the same kind of
   UsbDevice-driving engine added - a separate, larger piece of work,
   not attempted here. Port count (NDP) and the OHCI profile constants
   below are NOT confirmed for this specific SoC; they are inherited
   from S3C2410's profile as a starting point (OHCI's HcRevision=0x10
   is spec-fixed across every OHCI 1.0a implementation, so that part is
   safe - the writable masks and NDP are not, and FATAL-FIRST will
   catch a real mismatch). */

constexpr OhciProfile kAu1300Profile = {
    "Au1300",
    0x00000010u,
    0xC000007Fu,
    0x000007FFu,
    0x00000002u,
    0xFF001B00u,
    0x00000628u,
};

constexpr int kUsbIrqSource = 90;  /* shared with au1300_ehci.cpp */

}  /* namespace */

Au1300Ohci::Au1300Ohci(CerfEmulator& emu, uint32_t base)
    : Peripheral(emu), base_(base), ohci_(emu, kAu1300Profile, *this) {}

bool Au1300Ohci::ShouldRegister() {
    auto* bd = emu_.TryGet<BoardContext>();
    return bd && bd->GetSocId() == SocId::Au1300;
}

void Au1300Ohci::OnReady() {
    ohci_.Attach();
    irq_ = &emu_.Get<Au1300Irq>();
    emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) {
        ohci_.HardwareReset();
    });
    emu_.Get<PeripheralDispatcher>().Register(this);
}

uint32_t Au1300Ohci::MmioBase() const { return base_; }
uint32_t Au1300Ohci::MmioSize() const { return kSize; }

uint32_t Au1300Ohci::ReadWord(uint32_t addr) {
    if (addr & 3u) HaltUnsupportedAccess("ReadWord(unaligned)", addr, 0);
    return ohci_.Read(addr - base_);
}

void Au1300Ohci::WriteWord(uint32_t addr, uint32_t value) {
    if (addr & 3u) HaltUnsupportedAccess("WriteWord(unaligned)", addr, value);
    ohci_.Write(addr - base_, value);
}

uint32_t Au1300Ohci::OhciRead32(uint32_t addr) {
    uint32_t value = 0u;
    if ((addr & 3u) != 0u || !emu_.Get<PhysicalBus>().Read(addr, MmioWidth::kWord, &value))
        RejectBusMaster(addr);
    return value;
}

void Au1300Ohci::OhciWrite32(uint32_t addr, uint32_t value) {
    if ((addr & 3u) != 0u || !emu_.Get<PhysicalBus>().Write(addr, MmioWidth::kWord, value))
        RejectBusMaster(addr);
}

void Au1300Ohci::OhciInterrupt(bool asserted) {
    if (asserted) irq_->AssertIrq(kUsbIrqSource);
    else          irq_->DeAssertIrq(kUsbIrqSource);
}

void Au1300Ohci::SaveState(StateWriter& w) { ohci_.SaveState(w); }
void Au1300Ohci::RestoreState(StateReader& r) { ohci_.RestoreState(r); }
void Au1300Ohci::PostRestore() { ohci_.PostRestore(); }

void Au1300Ohci::RejectBusMaster(uint32_t addr) {
    emu_.Get<Fatal>().Die(
        "Au1300 OHCI (base 0x%08X): bus master access to an unbacked or unaligned "
        "address pa=0x%08X", base_, addr);
}

Au1300Ohci0::Au1300Ohci0(CerfEmulator& emu) : Au1300Ohci(emu, 0x14022400u) {}
Au1300Ohci1::Au1300Ohci1(CerfEmulator& emu) : Au1300Ohci(emu, 0x14022800u) {}

REGISTER_SERVICE(Au1300Ohci0);
REGISTER_SERVICE(Au1300Ohci1);

}
