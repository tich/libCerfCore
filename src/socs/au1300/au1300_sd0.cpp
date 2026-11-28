#include <cerfcore/peripherals/peripheral_base.h>

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>
#include "../../peripherals/peripheral_dispatcher.h"

#include <array>
#include <cstdint>

namespace cerfcore {

/* Au1300 SD0 (SD/MMC card controller), PA 0x10600000 - confirmed live:
   chevy_mylink nk.exe writes 0 to +0x0C (decompiled VA 0xC0452C9C, the
   same generic MMIO-write trampoline this project has seen reused
   everywhere), right after "SDIOInitializeSlot:: Slot:0x0" and the
   HalInitDmaChannel calls assigning DBDMA channels 6/7 to SDIO0 TX/RX
   (au1300_dbdma.cpp). +0x0C is SD_ENABLE in Linux's au1100_mmc.h
   (SD0_BASE=0xB0600000, confirmed matching 0x10600000 physical), and
   writing 0 there (CE=0, R=0: controller disabled, not reset) is the
   same "disable before configuring" idiom this project has already
   seen for every other Au1300 peripheral's own init sequence.

   Register layout (SD_TXPORT=0x00 through SD_TIMEOUT=0x38) is Linux's
   au1100_mmc.h verbatim - the Au1100-era data book this header cites
   is the only primary source found for this controller; Au1200/Au1300
   are assumed to share it (not independently confirmed beyond
   SD_ENABLE's offset/bits matching the live access above, since
   nothing else has been touched yet).

   SDIO0's actual data path goes through DBDMA channels 6/7 (already
   modeled), so this class only needs to track SD0's own control/
   status registers, not move real card data - no real SD card is
   modeled behind this (no actual filesystem/card-image mounted), so
   SD_STATUS's card-detect bit (CD) stays clear, matching every other
   USB/peripheral port in this project with nothing attached. Every
   register is stored verbatim with no protocol logic (no command
   actually executes), since nothing so far depends on a real SD
   transaction completing - same minimal philosophy as the OHCI/PSC
   peripherals elsewhere in this project. */

namespace {

constexpr uint32_t kBase = 0x10600000u;
constexpr uint32_t kSize = 0x0000003Cu;  /* through SD_TIMEOUT (+0x38) */

constexpr uint32_t kOffTxPort  = 0x00u;
constexpr uint32_t kOffRxPort  = 0x04u;
constexpr uint32_t kOffConfig  = 0x08u;
constexpr uint32_t kOffEnable  = 0x0Cu;
constexpr uint32_t kOffConfig2 = 0x10u;
constexpr uint32_t kOffBlkSize = 0x14u;
constexpr uint32_t kOffStatus  = 0x18u;
constexpr uint32_t kOffDebug   = 0x1Cu;
constexpr uint32_t kOffCmd     = 0x20u;
constexpr uint32_t kOffCmdArg  = 0x24u;
constexpr uint32_t kOffResp3   = 0x28u;
constexpr uint32_t kOffResp2   = 0x2Cu;
constexpr uint32_t kOffResp1   = 0x30u;
constexpr uint32_t kOffResp0   = 0x34u;
constexpr uint32_t kOffTimeout = 0x38u;

class Au1300Sd0 : public Peripheral {
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

    uint32_t ReadWord(uint32_t addr) override {
        const uint32_t off = addr - kBase;
        switch (off) {
            case kOffTxPort: case kOffRxPort: case kOffConfig: case kOffEnable:
            case kOffConfig2: case kOffBlkSize: case kOffStatus: case kOffDebug:
            case kOffCmd: case kOffCmdArg: case kOffResp3: case kOffResp2:
            case kOffResp1: case kOffResp0: case kOffTimeout:
                return regs_[off / 4u];
            default: HaltUnsupportedAccess("ReadWord", addr, 0);
        }
    }

    void WriteWord(uint32_t addr, uint32_t value) override {
        const uint32_t off = addr - kBase;
        switch (off) {
            case kOffTxPort: case kOffRxPort: case kOffConfig: case kOffEnable:
            case kOffConfig2: case kOffBlkSize: case kOffDebug:
            case kOffCmd: case kOffCmdArg: case kOffResp3: case kOffResp2:
            case kOffResp1: case kOffResp0: case kOffTimeout:
                regs_[off / 4u] = value;
                return;
            case kOffStatus:
                /* Write-1-to-clear, same SET/CLR idiom this project's other
                   status registers (USB_CTL's INT_STATUS, DBDMA) use;
                   confirmed by the live write of 0xFFFFFFFF ("clear all")
                   right after init, decompiled VA 0xC09B2B8C. */
                regs_[off / 4u] &= ~value;
                return;
            default: HaltUnsupportedAccess("WriteWord", addr, value);
        }
    }

    void SaveState(StateWriter& w) override { w.WriteBytes("regs", regs_.data(), sizeof(regs_)); }
    void RestoreState(StateReader& r) override { r.ReadBytes("regs", regs_.data(), sizeof(regs_)); }

private:
    std::array<uint32_t, kSize / 4u> regs_{};
};

}  /* namespace */

REGISTER_SERVICE(Au1300Sd0);

}
