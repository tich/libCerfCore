#include <cerfcore/peripherals/peripheral_base.h>

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include "au1300_ohci.h"
#include <cerfcore/core/cerf_emulator.h>
#include "../../peripherals/peripheral_dispatcher.h"

#include <cstdint>

namespace cerfcore {

/* Alias window for Au1300Ohci0, reached by USBware's real OHCI driver
   ("ohci_local0:", decompiled VA 0xC0B6AAD8 - the same generic
   register-read helper au1300_ehci_cap.cpp's own alias uses) at PA
   0x14020800 = the EHCI capability block's base (0x14020000,
   au1300_ehci_cap.cpp) + 0x800. This mirrors exactly how that same
   driver stack reaches EHCI via capbase+CAPLENGTH (0x14020040)
   while an earlier, separate OAL-layer touch uses a different
   hardcoded address (0x14022000) for the same EHCI hardware - see
   au1300_ehci_cap.cpp's comment for the full reasoning. Forwards
   every access straight into the existing Au1300Ohci0 instance so
   there is exactly one piece of register/schedule state for OHCI0,
   not two independently-tracked copies. No equivalent alias for
   OHCI1 has been observed yet; add one the same way if a real
   access proves it exists. */

namespace {

constexpr uint32_t kBase = 0x14020800u;
constexpr uint32_t kSize = 0x0000005Cu;  /* same size as Au1300Ohci's own block */

class Au1300Ohci0Alias : public Peripheral {
public:
    using Peripheral::Peripheral;

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetSocId() == SocId::Au1300;
    }

    void OnReady() override {
        ohci0_ = &emu_.Get<Au1300Ohci0>();
        emu_.Get<PeripheralDispatcher>().Register(this);
    }

    uint32_t MmioBase() const override { return kBase; }
    uint32_t MmioSize() const override { return kSize; }

    uint32_t ReadWord(uint32_t addr) override {
        return ohci0_->ReadWord(ohci0_->MmioBase() + (addr - kBase));
    }

    void WriteWord(uint32_t addr, uint32_t value) override {
        ohci0_->WriteWord(ohci0_->MmioBase() + (addr - kBase), value);
    }

private:
    Au1300Ohci0* ohci0_ = nullptr;
};

}  /* namespace */

REGISTER_SERVICE(Au1300Ohci0Alias);

}
