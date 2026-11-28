#include <cerfcore/peripherals/peripheral_base.h>

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>
#include "../../peripherals/peripheral_dispatcher.h"

#include <array>
#include <cstdint>
#include <cstring>

namespace cerfcore {

/* Au1300 LCD/display controller, PA 0x15000000 (Linux's AU1200_LCD_ADDR
   per drivers/video/fbdev/au1200fb.h is 0xB5000000 KSEG1 - PA
   0x15000000). Size (0x900 = 2304 bytes) is not a guess: it is the
   exact size argument chevy_mylink nk.exe's own MapIoSpace-style
   helper passes for this base (decompiled VA 0xC0A3CEAC-0xC0A3CEB8).

   The driver's first access after that mapping call is a zero-init
   loop clearing its own register block (decompiled VA 0xC0A3CEF8
   onward, an unaligned SWL/SWR pair at a runtime-computed offset -
   the same "clear everything before configuring" idiom this whole
   SoC's boot sequence has shown repeatedly for every other
   peripheral). This project's existing au1200fb-style register
   layout is NOT modeled here yet - no individual register name or
   bit has been confirmed, since this zero-init loop writes across
   the whole block generically rather than touching one named
   register at a time. Modeled as plain, opaque read/write word
   storage across the whole confirmed size, with no identity or bit
   layout asserted for any individual offset - once the real register
   layout needs grounding (Linux's drivers/video/fbdev/au1200fb.h/.c),
   do that then rather than guessing it now from the zero-init loop
   alone. */

namespace {

constexpr uint32_t kBase = 0x15000000u;
constexpr uint32_t kSize = 0x00000900u;

class Au1300Lcd : public Peripheral {
public:
    using Peripheral::Peripheral;

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetSocId() == SocId::Au1300;
    }

    void OnReady() override {
        emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) { regs_.fill(0u); });
        emu_.Get<PeripheralDispatcher>().Register(this);
    }

    uint32_t MmioBase() const override { return kBase; }
    uint32_t MmioSize() const override { return kSize; }

    uint32_t ReadWord(uint32_t addr) override { return regs_[(addr - kBase) / 4u]; }

    void WriteWord(uint32_t addr, uint32_t value) override {
        regs_[(addr - kBase) / 4u] = value;
    }

    /* The zero-init loop that first touches this block (see the class
       comment) does so via an unaligned SWL/SWR pair, which this
       project's MIPS engine delivers as direct WriteByte calls rather
       than auto-decomposing through WriteWord - so byte/half access
       needs its own implementation here, not just the word path. */
    uint8_t ReadByte(uint32_t addr) override {
        return reinterpret_cast<const uint8_t*>(regs_.data())[addr - kBase];
    }
    void WriteByte(uint32_t addr, uint8_t value) override {
        reinterpret_cast<uint8_t*>(regs_.data())[addr - kBase] = value;
    }
    uint16_t ReadHalf(uint32_t addr) override {
        uint16_t v;
        std::memcpy(&v, reinterpret_cast<const uint8_t*>(regs_.data()) + (addr - kBase), 2u);
        return v;
    }
    void WriteHalf(uint32_t addr, uint16_t value) override {
        std::memcpy(reinterpret_cast<uint8_t*>(regs_.data()) + (addr - kBase), &value, 2u);
    }

    void SaveState(StateWriter& w) override { w.WriteBytes("regs", regs_.data(), sizeof(regs_)); }
    void RestoreState(StateReader& r) override { r.ReadBytes("regs", regs_.data(), sizeof(regs_)); }

private:
    std::array<uint32_t, kSize / 4u> regs_{};
};

}  /* namespace */

REGISTER_SERVICE(Au1300Lcd);

}
