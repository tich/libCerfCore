#include "../mips/mips_cp0_sel_registers.h"

#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfengine/mips/mips_cpu_state.h>
#include "../../socs/au1300/au1300_id.h"

namespace cerfcore {

namespace {

constexpr uint32_t kConfig1 = 0x9e190c8eu;
constexpr uint32_t kConfig2 = 0x80000000u;
constexpr uint32_t kConfig3 = 0x00000000u;

class Au1300Cp0SelRegisters : public MipsCp0SelRegisters {
public:
    using MipsCp0SelRegisters::MipsCp0SelRegisters;

    bool ShouldRegister() override {
        return emu_.Get<BoardContext>().GetSocId() == SocId::Au1300;
    }

    cerfengine::MipsCp0Decision Decide(uint32_t rd, uint32_t sel) override {
        cerfengine::MipsCp0Decision d;
        if (rd != cerfengine::MipsCp0::kConfig) return d;
        switch (sel) {
            case 1: d.present = true; d.read = &ReadConfig1; d.read_context = this; break;
            case 2: d.present = true; d.read = &ReadConfig2; d.read_context = this; break;
            case 3: d.present = true; d.read = &ReadConfig3; d.read_context = this; break;
            default: break;
        }
        return d;
    }

private:
    static uint32_t CERFENGINE_FASTCALL ReadConfig1(void*) { return kConfig1; }
    static uint32_t CERFENGINE_FASTCALL ReadConfig2(void*) { return kConfig2; }
    static uint32_t CERFENGINE_FASTCALL ReadConfig3(void*) { return kConfig3; }
};

}

REGISTER_SERVICE_AS(Au1300Cp0SelRegisters, MipsCp0SelRegisters);

}
