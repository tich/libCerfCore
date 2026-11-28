#pragma once

#include <cerfcore/peripherals/peripheral_base.h>
#include "../../peripherals/usb/ohci_host_controller.h"

#include <cstdint>

namespace cerfcore {

class Au1300Irq;

/* Au1300 OHCI0/OHCI1 - see au1300_ohci.cpp for the full grounding
   comment. Declared in a header (rather than kept anonymous, like most
   single-instance Au1300 peripherals in this project) so au1300_ohci_cap.cpp
   can forward a second, alias address window into these same instances via
   emu_.Get<Au1300Ohci0>()/emu_.Get<Au1300Ohci1>() - the same pattern
   au1300_ehci_cap.cpp already uses for Au1300Ehci. */
class Au1300Ohci : public Peripheral, public OhciHostBus {
public:
    Au1300Ohci(CerfEmulator& emu, uint32_t base);

    bool ShouldRegister() override;
    void OnReady() override;

    uint32_t MmioBase() const override;
    uint32_t MmioSize() const override;

    uint32_t ReadWord(uint32_t addr) override;
    void     WriteWord(uint32_t addr, uint32_t value) override;

    uint32_t OhciRead32(uint32_t addr) override;
    void     OhciWrite32(uint32_t addr, uint32_t value) override;
    void     OhciInterrupt(bool asserted) override;

    void SaveState(StateWriter& w) override;
    void RestoreState(StateReader& r) override;
    void PostRestore() override;

private:
    static constexpr uint32_t kSize = 0x0000005Cu;

    [[noreturn]] void RejectBusMaster(uint32_t addr);

    uint32_t            base_;
    Au1300Irq*          irq_ = nullptr;
    OhciHostController  ohci_;
};

class Au1300Ohci0 : public Au1300Ohci {
public:
    explicit Au1300Ohci0(CerfEmulator& emu);
};

class Au1300Ohci1 : public Au1300Ohci {
public:
    explicit Au1300Ohci1(CerfEmulator& emu);
};

}
