#include <cerfcore/peripherals/peripheral_base.h>

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>
#include "../../peripherals/peripheral_dispatcher.h"

#include <cstdint>

namespace cerfcore {

/* Au1300 USB control block (clocks/PHY/interrupt-enable for the shared
   DesignWare EHCI/OHCI0/OHCI1/UDC/OTG USB blocks - NOT the host
   controllers themselves, which live at separate bases this project
   has not yet reached). Linux arch/mips/include/asm/mach-au1x00/
   au1000.h AU1300_USB_CTL_PHYS_ADDR=0x14021000, register layout and
   bit definitions confirmed verbatim from Linux drivers/usb/host/
   alchemy-common.c's Au1300 path (au1300_usb_init/au1300_usb_control/
   __au1300_usb_phyctl):

     +0x00 USB_DWC_CTRL1  (OTGD=0x04 disable-OTG, HSTRS=0x02 enable-EHCI,
                            DCRS=0x01 enable-UDC)
     +0x04 USB_DWC_CTRL2  (PHY1RS=0x04, PHY0RS=0x02, PHYRS=0x01)
     +0x10 USB_VBUS_TIMER
     +0x14 USB_SBUS_CTRL  (SBCA=0x04 coherent access)
     +0x18 USB_MSR_ERR
     +0x1C USB_DWC_CTRL3  (OHCI1_CKEN=1<<19, OHCI0_CKEN=1<<18,
                            EHCI0_CKEN=1<<17, OTG0_CKEN=1<<16)
     +0x20 USB_DWC_CTRL4  (port 2 routing - boot firmware/platform
                            setting, not something this driver computes)
     +0x28 USB_OTG_STATUS
     +0x2C USB_DWC_CTRL5
     +0x30 USB_DWC_CTRL6
     +0x34 USB_DWC_CTRL7  (written 1 then 0 around OHCI clock start)
     +0xC0 USB_PHY_STATUS
     +0xC4 USB_INT_STATUS
     +0xC8 USB_INT_ENABLE (FORCE=0x20, PHY=0x10, UDC=0x08, EHCI=0x04,
                            OHCI1=0x02, OHCI0=0x01)

   Linux's own au1300_usb_init()/au1300_usb_control() never read back a
   "ready" or "done" status bit anywhere in this block (confirmed by
   reading that function's body directly - every step is a plain
   write followed by a memory barrier, no delay, no poll), so there is
   no real hardware timing to model: every register here is plain
   read/write state, matching what the guest itself expects to read
   back (several of Linux's enable sequences explicitly read-modify-
   write CTRL2/CTRL3/INT_ENABLE). chevy_mylink nk.exe's first access
   is a read of CTRL1 (decompiled VA 0xC09824F4), consistent with that
   same read-modify-write idiom. */

namespace {

constexpr uint32_t kUsbCtlPhysAddr = 0x14021000u;
constexpr uint32_t kUsbCtlBlockSize = 0xCCu;  /* through USB_INT_ENABLE (+0xC8) */

constexpr uint32_t kRegCtrl1    = 0x00u;
constexpr uint32_t kRegCtrl2    = 0x04u;
constexpr uint32_t kRegVbusTmr  = 0x10u;
constexpr uint32_t kRegSbusCtrl = 0x14u;
constexpr uint32_t kRegMsrErr   = 0x18u;
constexpr uint32_t kRegCtrl3    = 0x1Cu;
constexpr uint32_t kRegCtrl4    = 0x20u;
constexpr uint32_t kRegOtgStat  = 0x28u;
constexpr uint32_t kRegCtrl5    = 0x2Cu;
constexpr uint32_t kRegCtrl6    = 0x30u;
constexpr uint32_t kRegCtrl7    = 0x34u;
constexpr uint32_t kRegPhyStat  = 0xC0u;
constexpr uint32_t kRegIntStat  = 0xC4u;
constexpr uint32_t kRegIntEn    = 0xC8u;

class Au1300UsbCtl : public Peripheral {
public:
    using Peripheral::Peripheral;

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetSocId() == SocId::Au1300;
    }

    void OnReady() override {
        emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) { Reset(); });
        emu_.Get<PeripheralDispatcher>().Register(this);
    }

    uint32_t MmioBase() const override { return kUsbCtlPhysAddr; }
    uint32_t MmioSize() const override { return kUsbCtlBlockSize; }

    uint32_t ReadWord(uint32_t addr) override {
        switch (addr - MmioBase()) {
            case kRegCtrl1:    return ctrl1_;
            case kRegCtrl2:    return ctrl2_;
            case kRegVbusTmr:  return vbus_timer_;
            case kRegSbusCtrl: return sbus_ctrl_;
            case kRegMsrErr:   return msr_err_;
            case kRegCtrl3:    return ctrl3_;
            case kRegCtrl4:    return ctrl4_;
            case kRegOtgStat:  return otg_status_;
            case kRegCtrl5:    return ctrl5_;
            case kRegCtrl6:    return ctrl6_;
            case kRegCtrl7:    return ctrl7_;
            case kRegPhyStat:  return phy_status_;
            case kRegIntStat:  return int_status_;
            case kRegIntEn:    return int_enable_;
            default: HaltUnsupportedAccess("ReadWord", addr, 0);
        }
    }

    void WriteWord(uint32_t addr, uint32_t value) override {
        switch (addr - MmioBase()) {
            case kRegCtrl1:    ctrl1_ = value;       return;
            case kRegCtrl2:    ctrl2_ = value;        return;
            case kRegVbusTmr:  vbus_timer_ = value;   return;
            case kRegSbusCtrl: sbus_ctrl_ = value;    return;
            case kRegMsrErr:   msr_err_ = value;      return;
            case kRegCtrl3:    ctrl3_ = value;        return;
            case kRegCtrl4:    ctrl4_ = value;        return;
            case kRegOtgStat:  otg_status_ = value;   return;
            case kRegCtrl5:    ctrl5_ = value;        return;
            case kRegCtrl6:    ctrl6_ = value;        return;
            case kRegCtrl7:    ctrl7_ = value;        return;
            case kRegPhyStat:  phy_status_ = value;   return;
            case kRegIntStat:  int_status_ &= ~value; return;  /* write-1-to-clear */
            case kRegIntEn:    int_enable_ = value;   return;
            default: HaltUnsupportedAccess("WriteWord", addr, value);
        }
    }

    void SaveState(StateWriter& w) override {
        w.Write("ctrl1", ctrl1_);
        w.Write("ctrl2", ctrl2_);
        w.Write("vbus_timer", vbus_timer_);
        w.Write("sbus_ctrl", sbus_ctrl_);
        w.Write("msr_err", msr_err_);
        w.Write("ctrl3", ctrl3_);
        w.Write("ctrl4", ctrl4_);
        w.Write("otg_status", otg_status_);
        w.Write("ctrl5", ctrl5_);
        w.Write("ctrl6", ctrl6_);
        w.Write("ctrl7", ctrl7_);
        w.Write("phy_status", phy_status_);
        w.Write("int_status", int_status_);
        w.Write("int_enable", int_enable_);
    }
    void RestoreState(StateReader& r) override {
        r.Read("ctrl1", ctrl1_);
        r.Read("ctrl2", ctrl2_);
        r.Read("vbus_timer", vbus_timer_);
        r.Read("sbus_ctrl", sbus_ctrl_);
        r.Read("msr_err", msr_err_);
        r.Read("ctrl3", ctrl3_);
        r.Read("ctrl4", ctrl4_);
        r.Read("otg_status", otg_status_);
        r.Read("ctrl5", ctrl5_);
        r.Read("ctrl6", ctrl6_);
        r.Read("ctrl7", ctrl7_);
        r.Read("phy_status", phy_status_);
        r.Read("int_status", int_status_);
        r.Read("int_enable", int_enable_);
    }

private:
    void Reset() {
        ctrl1_ = 0;
        ctrl2_ = 0;
        vbus_timer_ = 0;
        sbus_ctrl_ = 0;
        msr_err_ = 0;
        ctrl3_ = 0;
        ctrl4_ = 0;
        otg_status_ = 0;
        ctrl5_ = 0;
        ctrl6_ = 0;
        ctrl7_ = 0;
        phy_status_ = 0;
        int_status_ = 0;
        int_enable_ = 0;
    }

    uint32_t ctrl1_ = 0;
    uint32_t ctrl2_ = 0;
    uint32_t vbus_timer_ = 0;
    uint32_t sbus_ctrl_ = 0;
    uint32_t msr_err_ = 0;
    uint32_t ctrl3_ = 0;
    uint32_t ctrl4_ = 0;
    uint32_t otg_status_ = 0;
    uint32_t ctrl5_ = 0;
    uint32_t ctrl6_ = 0;
    uint32_t ctrl7_ = 0;
    uint32_t phy_status_ = 0;
    uint32_t int_status_ = 0;
    uint32_t int_enable_ = 0;
};

}  /* namespace */

REGISTER_SERVICE(Au1300UsbCtl);

}
