#pragma once

#include <cerfcore/peripherals/peripheral_base.h>
#include "../../peripherals/usb/usb_host_port.h"
#include "../../execution/guest_cycle_clock.h"

#include <array>
#include <cstdint>
#include <mutex>

namespace cerfcore {

class Au1300Irq;
class UsbDevice;

/* Au1300 EHCI host controller (DesignWare core shared with OHCI0/OHCI1/UDC/
   OTG, clocked/enabled through the sibling Au1300UsbCtl - see that file's
   comment for the full USB_DWC_CTRLn layout). Single core, host-only (this
   project has not yet reached anything suggesting chevy_mylink runs this
   core in device mode); unlike i.MX51's Imx51Usboh3 (4 ChipIdea cores, one
   of which runs in device mode for its SBOOT flasher), there is no
   multi-core or OTG-role complexity to model here.

   PA 0x14022000 confirmed live as the EHCI OPERATIONAL register base
   (chevy_mylink nk.exe's first access, decompiled VA 0xC098438C, reads
   offset +0x44 - this is the only EHCI 1.0-standard offset that lands on
   PORTSC1 when the operational register block starts at relative offset 0,
   i.e. this base has no (or a zero-length) capability-register preamble
   visible to software). The operational register offsets below
   (USBCMD=+0x00 through PORTSC1=+0x44) are therefore the plain EHCI 1.0
   Spec Table 2-9/2-16 layout, not a guess specific to this chip - EHCI's
   operational register set is standardized, unlike Au1300's own
   proprietary blocks (PSC/GPIC/MAE-ITE) which needed live-tracing for
   every single register. Further live accesses confirmed PORTSC2 at
   +0x48 and PORTSC4 at +0x50 (the next consecutive EHCI port-register
   slots), so this models 4 ports (N_PORTS=4) even though PORTSC3
   (+0x4C) itself was never directly observed - a reasonable inference
   given software is reading sequential port registers, but worth
   re-confirming if a future halt proves N_PORTS is actually smaller.
   No capability registers (CAPLENGTH/HCSPARAMS/
   HCCPARAMS) are modeled: nothing has touched them, so they halt per
   FATAL-FIRST like every other unconfirmed offset in this project,
   pending a real access that proves where (or whether) they live.

   The async/periodic schedule engine (QH/qTD walking) is implemented in
   au1300_ehci_host.cpp, adapted from Imx51Usboh3's own EHCI-spec-grounded
   engine (imx51_usboh3_host.cpp) - that logic is EHCI-standard, not
   i.MX51-specific, so it transfers directly once the register offsets are
   corrected for this chip's simpler single-core layout.

   The root port is a UsbHostPort, the same attachment point every other
   USB host controller in this project uses (Imx51Usboh3's OTG host root
   port, usb_hub.h's downstream ports): this is deliberate, so that a
   future "pass real physical USB hardware through to the guest" UsbDevice
   implementation (not yet written anywhere in this project) can attach to
   RootPort() exactly like any virtual device would, with no change needed
   here. */
class Au1300Ehci : public Peripheral, public UsbHostPortHost {
public:
    using Peripheral::Peripheral;

    bool ShouldRegister() override;
    void OnReady() override;

    uint32_t MmioBase() const override;
    uint32_t MmioSize() const override;

    uint32_t ReadWord(uint32_t addr) override;
    void     WriteWord(uint32_t addr, uint32_t value) override;

    void SaveState(StateWriter& w) override;
    void RestoreState(StateReader& r) override;
    void PostRestore() override;

    /* The attachment point for a real or virtual USB device on port
       `index` (0 or 1) - see the class comment. */
    UsbHostPort& RootPort(uint32_t index) { return ports_[index]; }

    void OnPortConnectChanged(int port_index) override;

private:
    static constexpr uint32_t kBase = 0x14022000u;
    static constexpr uint32_t kSize = 0x00000100u;  /* generous; only +0x00..+0x48 confirmed */
    static constexpr uint32_t kPortCount = 4u;
    static constexpr uint32_t kOffPortsc1 = 0x44u;

    void ResetController();
    void UpdateIrq();
    void UpdateScheduleTimer();
    void ArmNextFrame();
    void OnFrameTimer();
    void WriteUsbCmd(uint32_t value);
    void WriteUsbSts(uint32_t value);
    void WritePortsc(uint32_t port, uint32_t value);
    UsbDevice* DeviceAtAddress(uint8_t addr);

    void ExecuteAsyncSchedule();
    void ExecutePeriodicSchedule();
    void ExecuteQueueHead(uint32_t qh_addr);
    bool ExecuteQtd(uint32_t qtd_addr, class UsbDevice* dev, uint32_t endpt);

    std::array<uint32_t, kSize / 4> regs_{};
    std::array<UsbHostPort, kPortCount> ports_{UsbHostPort(*this, 0), UsbHostPort(*this, 1),
                                               UsbHostPort(*this, 2), UsbHostPort(*this, 3)};

    std::mutex              mtx_;
    GuestCycleClock*        clock_ = nullptr;
    GuestCycleClock::Event* frame_event_ = nullptr;
    Au1300Irq*               irq_ = nullptr;
    uint32_t                 frindex_ = 0;
    bool                     frames_running_ = false;
};

}
