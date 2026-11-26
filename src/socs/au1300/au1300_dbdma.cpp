#include <cerfcore/peripherals/peripheral_base.h>

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>
#include "../../peripherals/peripheral_dispatcher.h"

#include <cstdint>

namespace cerfcore {

/* Au1300 DBDMA (Descriptor-Based DMA controller), Linux arch/mips/
   include/asm/mach-au1x00/au1xxx_dbdma.h (au1x_dma_chan_t) and au1000.h
   (AU1550_DBDMA_PHYS_ADDR=0x14002000, shared with Au1300 per that
   constant's per-SoC mask). 16 channels (Linux's NUM_DBDMA_CHANS), each
   a 7-word register block (ddma_cfg/desptr/statptr/dbell/irq/stat/
   bytecnt at +0x00/+0x04/+0x08/+0x0C/+0x10/+0x14/+0x18) padded to a
   256-byte (0x100) window per that header's own "remainder, up to the
   256 byte boundary, is reserved" comment - 16 * 0x100 = 0x1000 bytes,
   which independently confirms the sibling Au1300DbdmaConf's placement
   right after this block (chevy_mylink nk.exe reaches it via a channel
   handle pointer + 0x1000, see that class's own comment).

   chevy_mylink nk.exe writes 0 to channel 0's ddma_cfg (decompiled VA
   0xC0452C9C, the same shared generic MMIO-write trampoline this
   project has already seen reused across PSC_SEL writes - evidently a
   generic "write this word to that device register" OAL helper, not
   specific to any one driver) immediately before the DBDMA_CONF write
   this file's sibling class models - i.e. clearing/disabling the
   channel as the first step of configuring it.

   This models real DMA data movement nowhere yet: registers are
   stored verbatim, ddma_stat reads back "Halted" (bit 0, DDMA_STAT_H)
   since no transfer has ever been started, and everything beyond the
   7 real per-channel words halts per FATAL-FIRST, same as every
   sibling Au1300 peripheral in this project, until a real access to
   it is seen. */

namespace {

constexpr uint32_t kDbdmaPhysAddr = 0x14002000u;
constexpr uint32_t kNumChannels   = 16u;
constexpr uint32_t kChanStride    = 0x100u;
constexpr uint32_t kDbdmaBlockSize = kNumChannels * kChanStride;

constexpr uint32_t kChanCfg      = 0x00u;
constexpr uint32_t kChanDesptr   = 0x04u;
constexpr uint32_t kChanStatptr  = 0x08u;
constexpr uint32_t kChanDbell    = 0x0Cu;
constexpr uint32_t kChanIrq      = 0x10u;
constexpr uint32_t kChanStat     = 0x14u;
constexpr uint32_t kChanByteCnt  = 0x18u;

constexpr uint32_t kStatHalted = 1u << 0;  /* DDMA_STAT_H */

struct Channel {
    uint32_t cfg     = 0;
    uint32_t desptr   = 0;
    uint32_t statptr  = 0;
    uint32_t dbell    = 0;
    uint32_t irq      = 0;
    uint32_t bytecnt  = 0;
};

class Au1300Dbdma : public Peripheral {
public:
    using Peripheral::Peripheral;

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetSocId() == SocId::Au1300;
    }

    void OnReady() override {
        emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) {
            for (Channel& c : chans_) c = Channel{};
        });
        emu_.Get<PeripheralDispatcher>().Register(this);
    }

    uint32_t MmioBase() const override { return kDbdmaPhysAddr; }
    uint32_t MmioSize() const override { return kDbdmaBlockSize; }

    uint32_t ReadWord(uint32_t addr) override {
        const uint32_t off = addr - MmioBase();
        Channel& c = chans_[off / kChanStride];
        switch (off % kChanStride) {
            case kChanCfg:     return c.cfg;
            case kChanDesptr:  return c.desptr;
            case kChanStatptr: return c.statptr;
            case kChanDbell:   return c.dbell;
            case kChanIrq:     return c.irq;
            case kChanStat:    return kStatHalted;
            case kChanByteCnt: return c.bytecnt;
            default: HaltUnsupportedAccess("ReadWord", addr, 0);
        }
    }

    void WriteWord(uint32_t addr, uint32_t value) override {
        const uint32_t off = addr - MmioBase();
        Channel& c = chans_[off / kChanStride];
        switch (off % kChanStride) {
            case kChanCfg:     c.cfg = value;    return;
            case kChanDesptr:  c.desptr = value;  return;
            case kChanStatptr: c.statptr = value; return;
            case kChanDbell:   c.dbell = value;   return;
            case kChanIrq:     c.irq = value;     return;
            case kChanByteCnt: c.bytecnt = value; return;
            default: HaltUnsupportedAccess("WriteWord", addr, value);
        }
    }

    void SaveState(StateWriter& w) override {
        for (const Channel& c : chans_) {
            w.Write("cfg", c.cfg);
            w.Write("desptr", c.desptr);
            w.Write("statptr", c.statptr);
            w.Write("dbell", c.dbell);
            w.Write("irq", c.irq);
            w.Write("bytecnt", c.bytecnt);
        }
    }
    void RestoreState(StateReader& r) override {
        for (Channel& c : chans_) {
            r.Read("cfg", c.cfg);
            r.Read("desptr", c.desptr);
            r.Read("statptr", c.statptr);
            r.Read("dbell", c.dbell);
            r.Read("irq", c.irq);
            r.Read("bytecnt", c.bytecnt);
        }
    }

private:
    Channel chans_[kNumChannels];
};

/* Au1300 DBDMA's global config region, Linux AU1550_DBDMA_CONF_PHYS_ADDR=
   0x14003000 (shared with Au1300 per that constant's per-SoC mask,
   despite the Au1550-only-looking name) - the dbdma_global_t struct
   (ddma_config/intstat/throttle/inten at +0x00/+0x04/+0x08/+0x0C) that
   sits immediately after the 16-channel block the sibling Au1300Dbdma
   models (16 * 0x100 = 0x1000 bytes, exactly the gap between
   0x14002000 and 0x14003000 - independent confirmation of both
   addresses, since chevy_mylink nk.exe reaches this register via a
   channel handle pointer + 0x1000, decompiled VA 0xC045375C).

   chevy_mylink nk.exe writes 7 to ddma_config (+0x00) here right after
   allocating and clearing a DMA channel - "HalInitDmaChannel: Channel
   0 initialized for I2S TX" / "...Channel 1...I2S RX" confirm this is
   the I2S driver's own DMA channel setup, not a guess at what the two
   channels are for. It then reads ddma_inten (+0x0C, decompiled VA
   0x801067DC). None of these four registers' bit layouts are
   confirmed from a primary source against something this boot
   sequence depends on, so every one is only stored/returned verbatim,
   never interpreted - same FATAL-FIRST-for-anything-unconfirmed
   convention the sibling PSC files use, now satisfied for all four
   dbdma_global_t registers Linux's header names. */

constexpr uint32_t kDbdmaConfPhysAddr = 0x14003000u;
constexpr uint32_t kDbdmaConfBlockSize = 0x10u;  /* through ddma_inten (+0x0C) */

constexpr uint32_t kRegConf    = 0x00u;
constexpr uint32_t kRegIntStat = 0x04u;
constexpr uint32_t kRegThrottle = 0x08u;
constexpr uint32_t kRegInten   = 0x0Cu;

class Au1300DbdmaConf : public Peripheral {
public:
    using Peripheral::Peripheral;

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetSocId() == SocId::Au1300;
    }

    void OnReady() override {
        emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) { conf_ = 0; });
        emu_.Get<PeripheralDispatcher>().Register(this);
    }

    uint32_t MmioBase() const override { return kDbdmaConfPhysAddr; }
    uint32_t MmioSize() const override { return kDbdmaConfBlockSize; }

    uint32_t ReadWord(uint32_t addr) override {
        switch (addr - MmioBase()) {
            case kRegConf:     return conf_;
            case kRegIntStat:  return intstat_;
            case kRegThrottle: return throttle_;
            case kRegInten:    return inten_;
            default: HaltUnsupportedAccess("ReadWord", addr, 0);
        }
    }

    void WriteWord(uint32_t addr, uint32_t value) override {
        switch (addr - MmioBase()) {
            case kRegConf:     conf_ = value;     return;
            case kRegIntStat:  intstat_ = value;  return;
            case kRegThrottle: throttle_ = value; return;
            case kRegInten:    inten_ = value;     return;
            default: HaltUnsupportedAccess("WriteWord", addr, value);
        }
    }

    void SaveState(StateWriter& w) override {
        w.Write("conf", conf_);
        w.Write("intstat", intstat_);
        w.Write("throttle", throttle_);
        w.Write("inten", inten_);
    }
    void RestoreState(StateReader& r) override {
        r.Read("conf", conf_);
        r.Read("intstat", intstat_);
        r.Read("throttle", throttle_);
        r.Read("inten", inten_);
    }

private:
    uint32_t conf_ = 0;
    uint32_t intstat_ = 0;
    uint32_t throttle_ = 0;
    uint32_t inten_ = 0;
};

}  /* namespace */

REGISTER_SERVICE(Au1300Dbdma);
REGISTER_SERVICE(Au1300DbdmaConf);

}
