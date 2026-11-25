#pragma once

#include "../../peripherals/uart16550/uart16550.h"

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include <cerfcore/core/cerf_emulator.h>

#include <cstdint>

namespace cerfcore {

/* Au1300 UART0-3. This is NOT the classic 16550 register order at 4-byte
   stride CERF's shared Uart16550/Serial16550 model assumes for its first 8
   indices (Linux's generic serial8250 framework, with
   arch/mips/alchemy/common/platform.c's .regshift = 2, apparently gets away
   with that assumption only because nothing exercises every register it
   misplaces) - two real guests directly disagree with it:
     - chevy_mylink nk.exe's own OAL debug-string routine (decompiled VA
       0x80108CDC-0x80108D50) polls byte offset +0x1C for bit 0x20 (TX
       ready) before writing the character being printed to offset +0x04 -
       not +0x14 (classic-order LSR) and +0x00 (classic-order THR).
     - Linux's own raw hardware helper, arch/mips/include/asm/mach-au1x00/
       au1000.h alchemy_uart_putchar(): "if (__raw_readl(base + 0x1c) &
       0x20) break;" then writes the character via a macro at a non-+0x00
       offset too - the same +0x1C TX-ready convention, independently.
   Given the classic order is wrong for this chip, this class does not
   inherit Uart16550's offset-based dispatch at all: every offset is
   explicit, and only offsets a real guest access has actually confirmed
   are implemented. Everything else halts (FATAL-FIRST) until a future
   halt names it, rather than silently guessing at a uniform renumbering.

   Confirmed so far:
     - +0x04: THR (write) / RBR (read) - chevy_mylink nk.exe writes the
       printed character here (see above).
     - +0x1C: LSR - chevy_mylink nk.exe and Linux's alchemy_uart_putchar
       both poll bit 0x20 (THRE) here before transmitting (see above).
     - +0x28: UART_CLK, an Alchemy-specific combined baud-rate divisor
       register (observed: chevy_mylink nk.exe writes UART2's +0x28 right
       after enabling it via MOD_CNTRL, never touching LCR.DLAB/DLL/DLM).
       Modeled as plain read/write state; it does not yet drive this
       model's baud timing, only accepts the write, since nothing so far
       depends on the simulated baud rate being accurate.
     - +0x100: UART_MOD_CNTRL (Linux au1000.h alchemy_uart_enable/disable:
       bit0 = clock enable, bit1 = block-reset deassert; the guest brings a
       UART up by writing 0, then 1, then 3, and tears it down by writing
       0). Modeled as plain read/write state.
     - +0x10, +0x14 and +0x18: written (0x31, 3, 3) by chevy_mylink nk.exe's
       UART-enable routine immediately after MOD_CNTRL, never read back
       anywhere this ROM's boot path has reached so far. This chip's real
       register layout is confirmed non-uniform (THR sits at +0x04, not
       the classic-order +0x00 a uniform renumbering would predict, while
       LSR sits at +0x1C, not the +0x18 that same renumbering would
       predict) - extrapolating a shift to guess these three offsets are
       "really" LCR/MCR/something would be exactly the guessed-value
       CERF's own rules forbid. They are modeled as plain opaque
       read/write state with no asserted identity, which is sufficient
       for what has been observed (write-only so far).
   +0x28 (UART_CLK)'s real purpose is now confirmed, not just observed:
   the same routine calls the SYS_CPUPLL/SYS_POWERCTRL frequency function
   (au1300_sys.cpp) and divides the result by 1,843,200 (0x1C2000 - the
   classic 16550 1.8432MHz BRG reference) to produce the value it writes
   to +0x28, i.e. UART_CLK is a precomputed baud divisor, not a raw
   register the guest expects particular hardware behavior from. Since
   this model does not yet drive simulated baud timing off it, the
   computed-vs-stored value does not currently matter; au1300_sys.cpp's
   CPUPLL/POWERCTRL defaulting to 0 only means this divisor computes to 0.
   Every other offset chevy_mylink's own boot path has touched so far
   (+0x00, +0x08, +0x0C) is unconfirmed and halts.

   The root oscillator Linux cites for every Alchemy board it has seen is
   12MHz (arch/mips/alchemy/common/clock.c ALCHEMY_ROOTCLK_RATE); the real
   UART baud-generator input is a PLL/divider descendant of that root whose
   exact ratio is board/variant-specific and not confirmed for this car's
   Au1300 variant, so 12MHz/1 is used here as a placeholder baud clock - it
   does not affect whether the MMIO access is accepted, only simulated baud
   timing. */

class Au1300Uart : public Uart16550 {
public:
    explicit Au1300Uart(CerfEmulator& emu)
        : Uart16550(emu, Config{.divisor_reset = 0u}, Clock{12000000u, 1u}) {}

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetSocId() == SocId::Au1300;
    }

    uint32_t MmioSize() const override { return 0x00001000u; }

protected:
    /* Only ReadByte/WriteByte's alignment check still uses this - ReadWord/WriteWord
       below dispatch on raw byte offset directly, not a stride-scaled index. */
    uint32_t RegStride() const override { return 4u; }

    void SetInterruptLine(bool pending) override {
        if (!pending) return;
        HaltUnsupportedAccess("SetInterruptLine(pending=true)", MmioBase(), 0);
    }

    uint32_t ReadWord(uint32_t addr) override {
        switch (addr - MmioBase()) {
            case kOffThr:      return ReadReg8(kBaseRbrThr);
            case kOffLsr:      return ReadReg8(kBaseLsr);
            case kOffUartClk:  return uart_clk_;
            case kOffModCntrl: return mod_cntrl_;
            case kOffUnk10:    return unk10_;
            case kOffUnk14:    return unk14_;
            case kOffUnk18:    return unk18_;
            default: HaltUnsupportedAccess("ReadWord", addr, 0);
        }
    }

    void WriteWord(uint32_t addr, uint32_t value) override {
        switch (addr - MmioBase()) {
            case kOffThr:      WriteReg8(kBaseRbrThr, static_cast<uint8_t>(value & 0xFFu)); return;
            case kOffLsr:      WriteReg8(kBaseLsr,    static_cast<uint8_t>(value & 0xFFu)); return;
            case kOffUartClk:  uart_clk_  = value;              return;
            case kOffModCntrl: mod_cntrl_ = value & kModCntrlMask; return;
            case kOffUnk10:    unk10_ = value; return;
            case kOffUnk14:    unk14_ = value; return;
            case kOffUnk18:    unk18_ = value; return;
            default: HaltUnsupportedAccess("WriteWord", addr, value);
        }
    }

    void SaveState(StateWriter& w) override {
        Uart16550::SaveState(w);
        w.Write("mod_cntrl", mod_cntrl_);
        w.Write("uart_clk", uart_clk_);
        w.Write("unk10", unk10_);
        w.Write("unk14", unk14_);
        w.Write("unk18", unk18_);
    }

    void RestoreState(StateReader& r) override {
        Uart16550::RestoreState(r);
        r.Read("mod_cntrl", mod_cntrl_);
        r.Read("uart_clk", uart_clk_);
        r.Read("unk10", unk10_);
        r.Read("unk14", unk14_);
        r.Read("unk18", unk18_);
    }

private:
    /* Serial16550's internal register indices (serial_16550.cpp kRbrThrDll=0, kLsr=5). */
    static constexpr uint32_t kBaseRbrThr = 0u;
    static constexpr uint32_t kBaseLsr    = 5u;

    static constexpr uint32_t kOffThr      = 0x04u;
    static constexpr uint32_t kOffLsr      = 0x1Cu;
    static constexpr uint32_t kOffUartClk  = 0x28u;
    static constexpr uint32_t kOffModCntrl = 0x100u;
    static constexpr uint32_t kOffUnk10    = 0x10u;
    static constexpr uint32_t kOffUnk14    = 0x14u;
    static constexpr uint32_t kOffUnk18    = 0x18u;
    static constexpr uint32_t kModCntrlMask = 0x3u;  /* CE | E */

    uint32_t mod_cntrl_ = 0;
    uint32_t uart_clk_  = 0;
    uint32_t unk10_     = 0;
    uint32_t unk14_     = 0;
    uint32_t unk18_     = 0;
};

}
