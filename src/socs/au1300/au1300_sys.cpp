#include <cerfcore/peripherals/peripheral_base.h>

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include "au1300_irq.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>
#include "../../execution/guest_cycle_clock.h"
#include "../../peripherals/peripheral_dispatcher.h"
#include "../oscillator_ticks.h"

#include <cstdint>

namespace cerfcore {

/* Au1300 "sys" block (Linux arch/mips/include/asm/mach-au1x00/au1000.h
   AU1300_SYS_PHYS_ADDR=0x10900000). Decompiled chevy_mylink nk.exe VA
   0x8010B178-0x8010B1E8 reads only SYS_CPUPLL (+0x60, Linux
   AU1000_SYS_CPUPLL) and SYS_POWERCTRL (+0x3C, AU1000_SYS_POWERCTRL) to
   compute a PLL-derived frequency as (cpupll&0x7f)*12,000,000 /
   ((powerctrl&3)+2) / 2 - nk.exe itself never writes either register, so
   their real value comes from board-level PLL straps this ROM dump does not
   contain. Both are modeled as plain read/write state defaulting to the 0
   every other un-strapped Au1000-family config register resets to (Linux
   clock.c: "Early Au1000 have a write-only SYS_CPUPLL register").

   SYS_SCRATCH0 (+0x18) and SYS_SCRATCH1 (+0x1C, Linux au1000.h
   AU1000_SYS_SCRATCH0/1) are general-purpose scratch registers with no
   fixed hardware behavior; chevy_mylink nk.exe's OEMInit (decompiled VA
   0x80107DB0) writes its own image base (0x80100000) to SCRATCH1, the
   classic boot-time handoff-data pattern. Modeled as plain read/write
   state.

   SYS_CNTRCTRL (+0x14, Linux au1000.h AU1000_SYS_CNTRCTRL) is the TOY/RTC
   32kHz-domain counter control/status register: writable enable bits
   (E0/EN1 etc., au1000.h SYS_CNTRL_*) plus read-only "still being written
   across the clock domain" busy bits and a "32kHz oscillator running"
   status bit (SYS_CNTRL_32S = 1<<5). chevy_mylink nk.exe's own code
   (decompiled VA 0x80108580-0x801085AC) confirms the latter precisely: it
   reads CNTRCTRL and tests bit 0x20 (32S); if clear, writes 0x100 (E0,
   SYS_CNTRL_E0 = 1<<8, "enable counter 0") and then busy-waits reading
   CNTRCTRL until bit 0x20 sets. This model has no real clock-domain
   propagation delay, so a write that sets E0 (bit8) also sets 32S (bit5)
   in the stored value immediately - the correct behavior for an
   idealized instantaneous domain crossing (the guest's own wait loop
   demands *some* latency model; zero is the simplest one that does not
   invent a specific real delay figure), not a guess at real timing.

   SYS_RTCTRIM (+0x44, Linux au1000.h AU1000_SYS_RTCTRIM) is the RTC
   clock-domain prescaler trim value; chevy_mylink nk.exe writes 0x1F to
   it right after the CNTRCTRL enable above. SYS_TOYTRIM (+0x00,
   AU1000_SYS_TOYTRIM) is the equivalent trim for the TOY counter;
   chevy_mylink nk.exe writes 0x7FFF to it (the textbook 32768Hz-oscillator
   trim constant). Neither has any runtime behavior beyond storing the
   value implemented, since nothing yet depends on the simulated tick
   rate of either counter. Modeled as plain read/write state.

   SYS_RTCWRITE (+0x48, Linux au1000.h AU1000_SYS_RTCWRITE) loads a new
   RTC counter value; chevy_mylink nk.exe writes 0 to it right after
   setting RTCTRIM. The busy/complete handshake for this write is the
   same CNTRCTRL "*S" bits already modeled above (C0S etc.), which this
   model already always reports clear, so no separate latency mechanism
   is needed. Modeled as plain read/write state; the value is not yet
   wired to any simulated real-time clock.

   SYS_RTCREAD (+0x58, Linux au1000.h AU1000_SYS_RTCREAD) reads the RTC
   counter back; chevy_mylink nk.exe reads it right after the RTCWRITE
   above. This model has no free-running RTC tick simulation, so RTCREAD
   returns exactly the value last loaded via RTCWRITE rather than
   inventing an elapsed-time figure - correct for "read back what you
   just wrote" and honest about not modeling real elapsed time.

   SYS_TOYREAD (+0x40, Linux au1000.h AU1000_SYS_TOYREAD) reads the live
   32-bit TOY (Time-Of-Year) counter, clocked by the same 32768 Hz domain
   SYS_TOYTRIM/SYS_CNTRCTRL already model. Decompiled chevy_mylink nk.exe
   VA 0x801086FC-0x80108760 reads TOYREAD, multiplies it by 10,000,000
   (100ns units per second) via a 64-bit multiply, and adds a 64-bit base
   timestamp - i.e. this is a real "convert the TOY tick count into the
   current time" conversion, not a busy-wait poll, so unlike CNTRCTRL this
   one DOES need to actually advance: per agent_docs/timers_clocks.md ("A
   state that the guest sees advance with time runs on guest time"; "A
   guest clock that does not keep time is a CERF defect until the
   arithmetic says otherwise"), this is backed by OscillatorTicks at the
   real 32768 Hz rate, derived from the guest's own cycle count rather
   than a host wall-clock thread or a frozen constant. No TOYWRITE access
   has been observed yet (nk.exe never writes it on this boot path), so
   the counter simply free-runs from 0 at reset; TOYWRITE stays
   unimplemented (halts per FATAL-FIRST) until a real access grounds it.

   Every other offset in the block is still unimplemented and halts, per
   FATAL-FIRST, until a real access to it is seen. */

namespace {

constexpr uint32_t kSysPhysAddr  = 0x10900000u;
constexpr uint32_t kSysBlockSize = 0x00000080u;  /* up through SYS_SLEEP (+0x7C) */

constexpr uint32_t kRegCpuPll    = 0x60u;

/* nk.exe's InitClock (0x80108114-0x80108124 in this ROM) reads SYS_CPUPLL,
   masks it to the low 7 bits, and multiplies by a hardcoded 12 (confirmed
   live: $s3=12 at the multiply) to print "@ %dMHZ" - i.e. CPUPLL holds a
   12MHz-reference PLL multiplier, not a frequency itself. On real hardware
   this is a board-strapped value already set by an earlier boot stage we
   don't emulate, so a reset value of 0 (silently read as "0MHz") is the
   actual bug, not a faithful reset state. 42*12=504MHz, the closest clean
   multiple of 12 to this project's existing (ungrounded-at-the-time)
   500MHz core clock choice in Au1300CoreClock - kept until real evidence
   gives a different exact multiplier. */
constexpr uint32_t kCpuPllReset  = 42u;
constexpr uint32_t kRegPowerCtrl = 0x3Cu;
constexpr uint32_t kRegScratch0  = 0x18u;
constexpr uint32_t kRegScratch1  = 0x1Cu;
constexpr uint32_t kRegCntrCtrl  = 0x14u;
constexpr uint32_t kCntrCtrlE0   = 1u << 8;
constexpr uint32_t kCntrCtrl32S  = 1u << 5;
constexpr uint32_t kRegRtcTrim   = 0x44u;
constexpr uint32_t kRegToyTrim   = 0x00u;
constexpr uint32_t kRegRtcWrite  = 0x48u;
constexpr uint32_t kRegRtcRead   = 0x58u;
constexpr uint32_t kRegToyRead   = 0x40u;
constexpr uint64_t kToyClockHz   = 32768u;
/* SYS_FREQCTRL0, Linux au1000.h AU1000_SYS_FREQCTRL0 - auxiliary clock
   divider/enable control, read by coredll.dll's au1300 clock driver
   (VA 0xC0A915E0) before it has written anything. Reset default 0
   (dividers disabled) per the field layout's own convention - every
   divider/enable bit defined for this register is 0 = off/bypassed,
   and this project has no evidence of a different hardware reset
   value. */
constexpr uint32_t kRegFreqCtrl0 = 0x20u;
/* SYS_CLKSRC, Linux au1000.h AU1000_SYS_CLKSRC - clock-source mux select,
   read immediately after SYS_FREQCTRL0 by the same coredll.dll clock-init
   routine (VA 0xC0A91608). Reset default 0, same reasoning as
   SYS_FREQCTRL0 above. */
constexpr uint32_t kRegClkSrc    = 0x28u;

/* SYS_RTCMATCH0, Linux au1000.h AU1000_SYS_RTCMATCH0 (+0x4C) - the RTC
   counter/match pair driving AU1300_RTC_MATCH0_INT (GPIC source 85).
   Confirmed as the OS's periodic-tick source by direct observation: the
   chevy_mylink OAL's own OEMInterruptInit enables exactly GPIC sources 75
   (DDMA), 84 (RTC) and 85 (RTC_MATCH0) and nothing else (captured
   directly from its real IEN writes), and with no interrupt source ever
   firing the guest settles into an idle context-switch loop forever
   (confirmed by instrumented tracing: zero further exception dispatches
   across a 13+ s run) despite CP0.Status already enabling IE and
   IM[IP2-5,IP7] - i.e. the CPU is ready to take an interrupt, nothing is
   asserting one. RTC_MATCH0 is the only one of the three enabled sources
   with an obvious periodic role, so it is the one driven here: on each
   write to RTCMATCH0, arm a real GuestCycleClock event for the RTC
   oscillator (same 32768 Hz domain as TOY, Linux's "32kHz routed to
   both") to reach that exact tick count, and on firing assert
   RTC_MATCH0_INT through the GPIC's own Au1300Irq (source bit 85,
   matching the project's own bank-size convention: bank 2 bit 21 =
   64+21). The OS's own ISR is expected to rewrite RTCMATCH0 for the next
   tick, same as the real hardware's free-running counter requires. */
constexpr uint32_t kRegRtcMatch0 = 0x4Cu;
constexpr int       kRtcMatch0IrqSource = 85;

/* AU1300_RTC_INT (GPIC source 84, bank 2 bit 20 = 64+20) - one of the same
   three sources chevy_mylink's OEMInterruptInit enables (see the comment
   above), independently confirmed via Linux's arch/mips/include/asm/
   mach-au1x00/au1000.h: AU1300_RTC_INT = AU1300_FIRST_INT+84, immediately
   followed by AU1300_RTC_MATCH0_INT at +85 - the same bank/bit this file
   already uses for kRtcMatch0IrqSource. Per this data book's Section
   10.2 "Time of Year Clock and Real Time Clock": "Each counter is also
   able to generate an interrupt on every tick," where a tick is one
   increment of the (divided) counter - "Divide = TRIM + 1" from the
   32768 Hz input (Section 10.2.1.5). Modeled as a free-running,
   self-rearming GuestCycleClock event at that same divided rate,
   independent of RTCMATCH0 and requiring no guest configuration to
   start, matching real hardware's "fires on every tick" wording and
   this interrupt's absence of an associated match register. */
constexpr int       kRtcTickIrqSource = 84;

class Au1300Sys : public Peripheral {
public:
    explicit Au1300Sys(CerfEmulator& emu)
        : Peripheral(emu), toy_osc_(emu, /*credits_park=*/true), rtc_osc_(emu, /*credits_park=*/true) {}

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetSocId() == SocId::Au1300;
    }

    void OnReady() override {
        irq_   = &emu_.Get<Au1300Irq>();
        clock_ = &emu_.Get<GuestCycleClock>();
        rtc_match0_event_ = clock_->Add([this] { OnRtcMatch0(); });
        rtc_tick_event_   = clock_->Add([this] { OnRtcTick(); });
        emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) {
            cpu_pll_ = kCpuPllReset;
            power_ctrl_ = 0;
            scratch0_ = 0;
            scratch1_ = 0;
            cntrctrl_ = 0;
            rtctrim_ = 0;
            toytrim_ = 0;
            rtcwrite_ = 0;
            freqctrl0_ = 0;
            clksrc_ = 0;
        });
        emu_.Get<PeripheralDispatcher>().Register(this);
        toy_osc_.Attach(kToyClockHz, 1u);
        rtc_osc_.Attach(kToyClockHz, 1u);
        rtc_osc_.ArmAt(rtc_tick_event_, rtc_osc_.Now() + (rtctrim_ + 1u));
    }

    void OnRtcMatch0() { irq_->AssertIrq(kRtcMatch0IrqSource); }

    void OnRtcTick() {
        irq_->AssertIrq(kRtcTickIrqSource);
        rtc_osc_.ArmAt(rtc_tick_event_, rtc_osc_.Now() + (rtctrim_ + 1u));
    }

    uint32_t MmioBase() const override { return kSysPhysAddr; }
    uint32_t MmioSize() const override { return kSysBlockSize; }

    uint32_t ReadWord(uint32_t addr) override {
        switch (addr - MmioBase()) {
            case kRegCpuPll:    return cpu_pll_;
            case kRegPowerCtrl: return power_ctrl_;
            case kRegScratch0:  return scratch0_;
            case kRegScratch1:  return scratch1_;
            case kRegCntrCtrl:  return cntrctrl_;
            case kRegRtcTrim:   return rtctrim_;
            case kRegToyTrim:   return toytrim_;
            case kRegRtcWrite:  return rtcwrite_;
            case kRegRtcRead:   return static_cast<uint32_t>(rtc_osc_.Now());
            case kRegToyRead:   return static_cast<uint32_t>(toy_osc_.Now());
            case kRegFreqCtrl0: return freqctrl0_;
            case kRegClkSrc:    return clksrc_;
            case kRegRtcMatch0: return rtcmatch0_;
            default: HaltUnsupportedAccess("ReadWord", addr, 0);
        }
    }

    void WriteWord(uint32_t addr, uint32_t value) override {
        switch (addr - MmioBase()) {
            case kRegCpuPll:    cpu_pll_ = value;    return;
            case kRegPowerCtrl: power_ctrl_ = value; return;
            case kRegScratch0:  scratch0_ = value;   return;
            case kRegScratch1:  scratch1_ = value;   return;
            case kRegCntrCtrl:
                cntrctrl_ = value | ((value & kCntrCtrlE0) ? kCntrCtrl32S : 0u);
                return;
            case kRegRtcTrim:   rtctrim_ = value; return;
            case kRegToyTrim:   toytrim_ = value; return;
            case kRegRtcWrite:  rtcwrite_ = value; return;
            case kRegFreqCtrl0: freqctrl0_ = value; return;
            case kRegClkSrc:    clksrc_ = value; return;
            case kRegRtcMatch0:
                rtcmatch0_ = value;
                irq_->DeAssertIrq(kRtcMatch0IrqSource);
                rtc_osc_.ArmAt(rtc_match0_event_, value);
                return;
            default: HaltUnsupportedAccess("WriteWord", addr, value);
        }
    }

    void SaveState(StateWriter& w) override {
        w.Write("cpu_pll", cpu_pll_);
        w.Write("power_ctrl", power_ctrl_);
        w.Write("scratch0", scratch0_);
        w.Write("scratch1", scratch1_);
        w.Write("cntrctrl", cntrctrl_);
        w.Write("rtctrim", rtctrim_);
        w.Write("toytrim", toytrim_);
        w.Write("rtcwrite", rtcwrite_);
        w.Write("freqctrl0", freqctrl0_);
        w.Write("clksrc", clksrc_);
        w.Write("rtcmatch0", rtcmatch0_);
        toy_osc_.Save(w, "toy_");
        rtc_osc_.Save(w, "rtc_");
    }

    void RestoreState(StateReader& r) override {
        toy_osc_.Restore(r, "toy_");
        rtc_osc_.Restore(r, "rtc_");
        r.Read("cpu_pll", cpu_pll_);
        r.Read("power_ctrl", power_ctrl_);
        r.Read("scratch0", scratch0_);
        r.Read("scratch1", scratch1_);
        r.Read("cntrctrl", cntrctrl_);
        r.Read("rtctrim", rtctrim_);
        r.Read("toytrim", toytrim_);
        r.Read("rtcwrite", rtcwrite_);
        r.Read("freqctrl0", freqctrl0_);
        r.Read("clksrc", clksrc_);
        r.Read("rtcmatch0", rtcmatch0_);
        rtc_osc_.ArmAt(rtc_match0_event_, rtcmatch0_);
        rtc_osc_.ArmAt(rtc_tick_event_, rtc_osc_.Now() + (rtctrim_ + 1u));
    }

private:
    OscillatorTicks toy_osc_;
    OscillatorTicks rtc_osc_;
    Au1300Irq*         irq_   = nullptr;
    GuestCycleClock*    clock_ = nullptr;
    GuestCycleClock::Event* rtc_match0_event_ = nullptr;
    GuestCycleClock::Event* rtc_tick_event_   = nullptr;
    uint32_t cpu_pll_ = kCpuPllReset;
    uint32_t power_ctrl_ = 0;
    uint32_t scratch0_ = 0;
    uint32_t scratch1_ = 0;
    uint32_t cntrctrl_ = 0;
    uint32_t rtctrim_ = 0;
    uint32_t toytrim_ = 0;
    uint32_t rtcwrite_ = 0;
    uint32_t freqctrl0_ = 0;
    uint32_t clksrc_ = 0;
    uint32_t rtcmatch0_ = 0;
};

}  /* namespace */

REGISTER_SERVICE(Au1300Sys);

}
