#include "../mips/mips_core_clock.h"

#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include "../../socs/au1300/au1300_id.h"

namespace cerfcore {

namespace {

constexpr uint64_t kPClockPerCountTick = 2u;
constexpr uint32_t kCoreClockHz        = 500000000u;

class Au1300CoreClock : public MipsCoreClock {
public:
    using MipsCoreClock::MipsCoreClock;

    bool ShouldRegister() override {
        return emu_.Get<BoardContext>().GetSocId() == SocId::Au1300;
    }

    GuestCycleClock::Rate ResetRate() const override {
        return GuestCycleClock::Rate{kCoreClockHz, 1u};
    }

    uint64_t CyclesPerCountTick() const override { return kPClockPerCountTick; }

    uint32_t ConfigEc() const override { return 0u; }
};

}

REGISTER_SERVICE_AS(Au1300CoreClock, MipsCoreClock);

}
