#include "au1300_ehci.h"

#include "au1300_id.h"
#include "au1300_irq.h"
#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>
#include "../../execution/guest_cycle_clock.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include "../../peripherals/usb/usb_device.h"

namespace cerfcore {

namespace {

/* EHCI 1.0 Spec Table 2-9 (operational registers, p19-20) - standard
   offsets, not Au1300-specific (see the class comment in au1300_ehci.h
   for why no capability-register preamble is modeled). */
constexpr uint32_t kOffUsbCmd    = 0x00u;
constexpr uint32_t kOffUsbSts    = 0x04u;
constexpr uint32_t kOffUsbIntr   = 0x08u;
constexpr uint32_t kOffFrindex   = 0x0Cu;
constexpr uint32_t kOffCtrlDsSeg = 0x10u;
constexpr uint32_t kOffPeriodicListBase = 0x14u;
constexpr uint32_t kOffAsyncListAddr    = 0x18u;
constexpr uint32_t kOffConfigFlag = 0x40u;
/* PORTSC1 is the class's kOffPortsc1 member (+0x44); PORTSC2 = +0x48,
   PORTSC[n] = +0x44 + 4n. */

constexpr uint32_t kCmdRs  = 1u << 0;   /* Run/Stop */
constexpr uint32_t kCmdRst = 1u << 1;   /* Host Controller Reset */
constexpr uint32_t kCmdPse = 1u << 4;   /* Periodic Schedule Enable */
constexpr uint32_t kCmdAse = 1u << 5;   /* Asynchronous Schedule Enable */

constexpr uint32_t kStsUi  = 1u << 0;   /* USB Interrupt (transaction complete) */
constexpr uint32_t kStsUei = 1u << 1;   /* USB Error Interrupt */
constexpr uint32_t kStsPci = 1u << 2;   /* Port Change Detect */
constexpr uint32_t kStsHch = 1u << 12;  /* HCHalted (RO) */
constexpr uint32_t kStsPss = 1u << 14;  /* Periodic Schedule Status (RO==PSE) */
constexpr uint32_t kStsAss = 1u << 15;  /* Asynchronous Schedule Status (RO==ASE) */
constexpr uint32_t kStsRoMask = kStsHch | kStsPss | kStsAss;
constexpr uint32_t kStsW1cMask = kStsUi | kStsUei | kStsPci;

/* EHCI 1.0 Table 2-16 (p26-29): PORTSC bit layout. */
constexpr uint32_t kPortscCcs  = 1u << 0;
constexpr uint32_t kPortscCsc  = 1u << 1;
constexpr uint32_t kPortscPed  = 1u << 2;
constexpr uint32_t kPortscPedc = 1u << 3;
constexpr uint32_t kPortscPr   = 1u << 8;
constexpr uint32_t kPortscPp   = 1u << 12;  /* hard-wired to 1: PPC=0 (no per-port power switch) */

/* GPIC source 90, Linux au1000.h AU1300_USB_INT = AU1300_FIRST_INT+90 -
   one shared level covering EHCI/OHCI0/OHCI1/UDC/OTG per
   au1300_usbctl.cpp's USB_INT_ENABLE bits. This class asserts it
   directly from its own USBSTS&USBINTR state, without gating on
   Au1300UsbCtl's USB_INTEN_EHCI bit - no live access has yet proven
   whether that gating happens in software-visible silicon or is
   purely internal; this is a simplification to flag for future
   refinement, not a confirmed hardware fact. */
constexpr int kUsbIrqSource = 90;

}  /* namespace */

bool Au1300Ehci::ShouldRegister() {
    auto* bd = emu_.TryGet<BoardContext>();
    return bd && bd->GetSocId() == SocId::Au1300;
}

void Au1300Ehci::OnReady() {
    irq_ = &emu_.Get<Au1300Irq>();
    clock_ = &emu_.Get<GuestCycleClock>();
    frame_event_ = clock_->Add([this] { OnFrameTimer(); });
    emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) {
        std::lock_guard<std::mutex> lk(mtx_);
        ResetController();
    });
    emu_.Get<PeripheralDispatcher>().Register(this);
    ResetController();
}

void Au1300Ehci::ResetController() {
    regs_.fill(0u);
    regs_[kOffUsbSts >> 2] = kStsHch;  /* out of reset, controller is halted */
    frindex_ = 0;
    frames_running_ = false;
    clock_->Disarm(frame_event_);
    irq_->DeAssertIrq(kUsbIrqSource);
    for (uint32_t p = 0; p < kPortCount; ++p) {
        if (ports_[p].IsConnected()) OnPortConnectChanged(static_cast<int>(p));
    }
}

uint32_t Au1300Ehci::MmioBase() const { return kBase; }
uint32_t Au1300Ehci::MmioSize() const { return kSize; }

uint32_t Au1300Ehci::ReadWord(uint32_t addr) {
    std::lock_guard<std::mutex> lk(mtx_);
    const uint32_t off = addr - kBase;
    switch (off) {
        case kOffUsbCmd: case kOffUsbSts: case kOffUsbIntr: case kOffCtrlDsSeg:
        case kOffPeriodicListBase: case kOffAsyncListAddr: case kOffConfigFlag:
            return regs_[off >> 2];
        case kOffFrindex:
            return frindex_;
        default:
            if (off >= kOffPortsc1 && off < kOffPortsc1 + kPortCount * 4u &&
                (off - kOffPortsc1) % 4u == 0u) {
                /* EHCI 1.0 Table 2-16 (p27): with HCSPARAMS.PPC=0 (no
                   per-port power switches) PP is RO and hard-wired to 1. */
                return regs_[off >> 2] | kPortscPp;
            }
            HaltUnsupportedAccess("ReadWord", addr, 0);
    }
}

void Au1300Ehci::WriteWord(uint32_t addr, uint32_t value) {
    std::lock_guard<std::mutex> lk(mtx_);
    const uint32_t off = addr - kBase;
    switch (off) {
        case kOffUsbCmd:    WriteUsbCmd(value); return;
        case kOffUsbSts:    WriteUsbSts(value); return;
        case kOffUsbIntr:   regs_[off >> 2] = value; UpdateIrq(); return;
        case kOffFrindex:
            if (!(regs_[kOffUsbCmd >> 2] & kCmdRs)) frindex_ = value & 0x3FFFu;
            return;
        case kOffCtrlDsSeg: regs_[off >> 2] = value; return;
        case kOffPeriodicListBase: regs_[off >> 2] = value; return;
        case kOffAsyncListAddr:    regs_[off >> 2] = value; return;
        case kOffConfigFlag:       regs_[off >> 2] = value; return;
        default:
            if (off >= kOffPortsc1 && off < kOffPortsc1 + kPortCount * 4u &&
                (off - kOffPortsc1) % 4u == 0u) {
                WritePortsc((off - kOffPortsc1) / 4u, value);
                return;
            }
            HaltUnsupportedAccess("WriteWord", addr, value);
    }
}

void Au1300Ehci::WriteUsbCmd(uint32_t value) {
    if (value & kCmdRst) {
        bool connected[kPortCount];
        for (uint32_t p = 0; p < kPortCount; ++p) connected[p] = ports_[p].IsConnected();
        ResetController();
        for (uint32_t p = 0; p < kPortCount; ++p) {
            if (connected[p]) OnPortConnectChanged(static_cast<int>(p));
        }
        return;
    }
    const uint32_t old = regs_[kOffUsbCmd >> 2];
    regs_[kOffUsbCmd >> 2] = value;
    uint32_t sts = regs_[kOffUsbSts >> 2];
    sts = (value & kCmdAse) ? (sts | kStsAss) : (sts & ~kStsAss);
    sts = (value & kCmdPse) ? (sts | kStsPss) : (sts & ~kStsPss);
    sts = (value & kCmdRs)  ? (sts & ~kStsHch) : (sts | kStsHch);
    regs_[kOffUsbSts >> 2] = sts;
    if ((value & kCmdRs) && !(old & kCmdRs)) frindex_ = 0;
    UpdateScheduleTimer();
    UpdateIrq();
}

void Au1300Ehci::WriteUsbSts(uint32_t value) {
    const uint32_t old = regs_[kOffUsbSts >> 2];
    regs_[kOffUsbSts >> 2] = (old & kStsRoMask) | (old & ~kStsRoMask & ~(value & kStsW1cMask));
    UpdateIrq();
}

void Au1300Ehci::WritePortsc(uint32_t port, uint32_t value) {
    const uint32_t idx = (kOffPortsc1 >> 2) + port;
    const uint32_t old = regs_[idx];
    uint32_t next = old;

    if ((value & kPortscPr) && !(old & kPortscPr)) {
        /* EHCI 1.0 §4.2.2 (p33): port reset auto-completes; software never
           writes PR back to 0 itself (the host controller clears it). */
        if (old & kPortscCcs) {
            next = (next & ~kPortscPr) | kPortscPed | kPortscPedc;
            regs_[kOffUsbSts >> 2] |= kStsPci;
            if (UsbDevice* dev = ports_[port].Device()) dev->ResetToDefault();
        } else {
            next &= ~kPortscPr;
        }
    }

    if (value & kPortscCsc)  next &= ~kPortscCsc;
    if (value & kPortscPedc) next &= ~kPortscPedc;
    if (!(value & kPortscPed) && (old & kPortscPed)) next &= ~kPortscPed;

    regs_[idx] = next;
    UpdateIrq();
}

void Au1300Ehci::OnPortConnectChanged(int port_index) {
    if (port_index < 0 || static_cast<uint32_t>(port_index) >= kPortCount) return;
    const uint32_t idx = (kOffPortsc1 >> 2) + static_cast<uint32_t>(port_index);
    if (ports_[port_index].IsConnected()) {
        regs_[idx] |= kPortscCcs | kPortscCsc;
    } else {
        regs_[idx] &= ~(kPortscCcs | kPortscPed);
        regs_[idx] |= kPortscCsc;
    }
    regs_[kOffUsbSts >> 2] |= kStsPci;
    UpdateIrq();
    UpdateScheduleTimer();
}

UsbDevice* Au1300Ehci::DeviceAtAddress(uint8_t addr) {
    for (auto& port : ports_) {
        if (UsbDevice* root = port.Device()) {
            if (UsbDevice* dev = root->FindByAddress(addr)) return dev;
        }
    }
    return nullptr;
}

void Au1300Ehci::UpdateIrq() {
    const uint32_t pending = regs_[kOffUsbSts >> 2] & regs_[kOffUsbIntr >> 2] &
                             (kStsUi | kStsUei | kStsPci);
    if (pending) irq_->AssertIrq(kUsbIrqSource);
    else         irq_->DeAssertIrq(kUsbIrqSource);
}

void Au1300Ehci::UpdateScheduleTimer() {
    const uint32_t cmd = regs_[kOffUsbCmd >> 2];
    bool any_connected = false;
    for (auto& port : ports_) any_connected |= port.IsConnected();
    if (!any_connected || !(cmd & kCmdRs) || !(cmd & (kCmdAse | kCmdPse))) {
        frames_running_ = false;
        clock_->Disarm(frame_event_);
        return;
    }
    if (frames_running_) return;
    frames_running_ = true;
    ArmNextFrame();
}

void Au1300Ehci::ArmNextFrame() {
    /* EHCI micro-frame = 125us. */
    clock_->Arm(frame_event_, clock_->NsToCycles(clock_->NowNs() + 125));
}

void Au1300Ehci::OnFrameTimer() {
    std::lock_guard<std::mutex> lk(mtx_);
    frindex_ = (frindex_ + 1u) & 0x3FFFu;
    ExecuteAsyncSchedule();
    ExecutePeriodicSchedule();
    if (frames_running_) ArmNextFrame();
}

void Au1300Ehci::SaveState(StateWriter& w) {
    std::lock_guard<std::mutex> lk(mtx_);
    w.WriteBytes("regs", regs_.data(), sizeof(regs_));
    w.Write("frindex", frindex_);
    for (auto& port : ports_) port.SaveState(w);
}

void Au1300Ehci::RestoreState(StateReader& r) {
    std::lock_guard<std::mutex> lk(mtx_);
    r.ReadBytes("regs", regs_.data(), sizeof(regs_));
    r.Read("frindex", frindex_);
    for (auto& port : ports_) port.RestoreState(r);
    clock_->Disarm(frame_event_);
    frames_running_ = false;
}

void Au1300Ehci::PostRestore() {
    std::lock_guard<std::mutex> lk(mtx_);
    for (auto& port : ports_) port.PostRestore();
    UpdateIrq();
    UpdateScheduleTimer();
}

REGISTER_SERVICE(Au1300Ehci);

}
