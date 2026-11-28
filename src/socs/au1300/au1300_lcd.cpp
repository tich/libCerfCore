#include "au1300_lcd.h"

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/host/host_presentation.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>
#include "../../peripherals/peripheral_dispatcher.h"

#include <cstring>

namespace cerfcore {

/* Au1300 LCD/display controller, PA 0x15000000 (Linux's AU1200_LCD_ADDR
   per drivers/video/fbdev/au1200fb.h is 0xB5000000 KSEG1 - PA
   0x15000000). Size (0x900 = 2304 bytes) was first confirmed from
   chevy_mylink nk.exe's own MapIoSpace-style call for this base
   (decompiled VA 0xC0A3CEAC-0xC0A3CEB8), and independently matches
   that header's own `struct au1200_lcd` size exactly (reserved0 +
   screen..fifoctrl + reserved2[42] + window[4] + reserved3 +
   palette[256] + cursorpattern[256 bytes] = 0x900) - strong
   confirmation this chip really is the au1200fb-compatible
   controller, not a coincidence of MapIoSpace's size argument alone.

   Register layout below is that struct, field-for-field (Au1200 only
   per the header's own scope - Au1300 compatibility is not stated
   there, but every offset/bit this project actually needs has now
   been exercised by the real guest driver without any mismatch):
     +0x04 screen       (SEN=1<<31, SX=0x7FF<<19 [width-1], SY=0x7FF<<8 [height-1])
     +0x08 backcolor    (SBGR 23:16, SBGG 15:8, SBGB 7:0)
     +0x24 winenable    (WEN0-3, bits 0-3)
     +0x100 + 0x20*n     window[n] (n=0..3), each:
       +0x00 winctrl0   (OX=0x7FF<<21, OY=0x7FF<<10, A=0xFF<<2, AEN=1<<1)
       +0x04 winctrl1   (PRI=3<<30, PIPE=1<<29, FRM=0xF<<25 [13=32bpp,
                         12=24bpp, 6=16bpp565], SZX=0x7FF<<11 [width-1],
                         SZY=0x7FF<<0 [height-1])
       +0x08 winctrl2   (color-key/scaling - not modeled beyond storage)
       +0x0C winbuf0    (DMA framebuffer physical address, buffer 0)
       +0x10 winbuf1    (DMA framebuffer physical address, buffer 1)
       +0x14 winbufctrl (DB=1<<1 double-buffer enable, DBN=1<<0 selects
                         buf1 over buf0)

   Confirmed live: the zero-init loop that first touches this block
   (decompiled VA 0xC0A3CEF8 onward, an unaligned SWL/SWR pair - the
   same "clear everything before configuring" idiom this whole SoC's
   boot sequence shows repeatedly) clears the generic register array;
   subsequent writes (not yet individually traced beyond what boot
   itself required to proceed) configure at least one window well
   enough for "Using panel 10 WVGA_800x480_TFT BPP:32" and
   "LayMgr.cpp: Layout Manager successfully initialized" to print and
   for the guest to then write real pixel data into its allocated
   framebuffer (AllocPhysMem p=0E200000) without halting.

   Every register is still stored as plain, opaque read/write words
   (no side effects on write) - the accessor methods below just give
   au1300_lcd_renderer.cpp named access to the fields it needs to find
   and interpret the active window's pixel data, they do not change
   how writes are handled. Interrupt/HWC-cursor/palette/PWM/timing
   fields are not given named accessors since nothing reads them back
   yet; add them if a real access ever needs their semantics
   interpreted rather than just stored. */

bool Au1300Lcd::ShouldRegister() {
    auto* bd = emu_.TryGet<BoardContext>();
    return bd && bd->GetSocId() == SocId::Au1300;
}

void Au1300Lcd::OnReady() {
    emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) { regs_.fill(0u); });
    emu_.Get<PeripheralDispatcher>().Register(this);
}

uint32_t Au1300Lcd::MmioBase() const { return kBase; }
uint32_t Au1300Lcd::MmioSize() const { return kSize; }

uint32_t Au1300Lcd::ReadWord(uint32_t addr) { return regs_[(addr - kBase) / 4u]; }

void Au1300Lcd::WriteWord(uint32_t addr, uint32_t value) {
    const uint32_t off = addr - kBase;
    regs_[off / 4u] = value;
    NotifyIfGeometryOffset(off);
}

/* winenable and every window's ctrl0 (OX/OY)/ctrl1 (SZX/SZY) affect
   EnabledWindowBounds - see that method's own comment, and
   au1300_lcd_renderer.cpp's, for why this guest's real display size
   has to be derived this way instead of read once from a fixed
   register. Called from every write path (word/half/byte): the
   zero-init loop and, per live tracing, the real window-configuring
   writes too both go through unaligned SWL/SWR, i.e. WriteByte, not
   WriteWord - a word-only hook here would silently never fire. */
void Au1300Lcd::NotifyIfGeometryOffset(uint32_t off) {
    if (off == kOffWinEnable) { NotifyIfRemoded(); return; }
    if (off >= kOffWindowBase && off < kOffWindowBase + kWindowCount * kWindowStride) {
        const uint32_t win_off = (off - kOffWindowBase) % kWindowStride;
        if (win_off == kOffWinCtrl0 || win_off == kOffWinCtrl1) NotifyIfRemoded();
    }
}

void Au1300Lcd::NotifyIfRemoded() {
    uint32_t w = 0, h = 0;
    EnabledWindowBounds(w, h);
    if (w == notified_w_ && h == notified_h_) return;
    notified_w_ = w;
    notified_h_ = h;
    if (w != 0 && h != 0) emu_.Get<HostPresentation>().GuestRemoded(w, h);
}

void Au1300Lcd::EnabledWindowBounds(uint32_t& w, uint32_t& h) const {
    w = 0;
    h = 0;
    for (uint32_t n = 0; n < kWindowCount; ++n) {
        if (!WindowEnabled(n)) continue;
        const uint32_t right = WindowOx(n) + WindowWidth(n);
        const uint32_t bottom = WindowOy(n) + WindowHeight(n);
        if (right > w) w = right;
        if (bottom > h) h = bottom;
    }
}

/* See the class comment: the zero-init loop reaches this peripheral via
   an unaligned SWL/SWR pair, which this project's MIPS engine delivers
   as direct WriteByte calls rather than auto-decomposing through
   WriteWord - so byte/half access needs its own implementation here,
   not just the word path. */
uint8_t Au1300Lcd::ReadByte(uint32_t addr) {
    return reinterpret_cast<const uint8_t*>(regs_.data())[addr - kBase];
}
void Au1300Lcd::WriteByte(uint32_t addr, uint8_t value) {
    const uint32_t off = addr - kBase;
    reinterpret_cast<uint8_t*>(regs_.data())[off] = value;
    NotifyIfGeometryOffset(off & ~3u);
}
uint16_t Au1300Lcd::ReadHalf(uint32_t addr) {
    uint16_t v;
    std::memcpy(&v, reinterpret_cast<const uint8_t*>(regs_.data()) + (addr - kBase), 2u);
    return v;
}
void Au1300Lcd::WriteHalf(uint32_t addr, uint16_t value) {
    const uint32_t off = addr - kBase;
    std::memcpy(reinterpret_cast<uint8_t*>(regs_.data()) + off, &value, 2u);
    NotifyIfGeometryOffset(off & ~3u);
}

void Au1300Lcd::SaveState(StateWriter& w) { w.WriteBytes("regs", regs_.data(), sizeof(regs_)); }
void Au1300Lcd::RestoreState(StateReader& r) { r.ReadBytes("regs", regs_.data(), sizeof(regs_)); }

bool Au1300Lcd::ScreenEnabled() const { return (Reg(kOffScreen) & (1u << 31)) != 0u; }
uint32_t Au1300Lcd::ScreenWidth() const { return ((Reg(kOffScreen) >> 19) & 0x7FFu) + 1u; }
uint32_t Au1300Lcd::ScreenHeight() const { return ((Reg(kOffScreen) >> 8) & 0x7FFu) + 1u; }
uint32_t Au1300Lcd::BackColorBgr() const { return Reg(kOffBackColor) & 0x00FFFFFFu; }

bool Au1300Lcd::WindowEnabled(uint32_t n) const { return (Reg(kOffWinEnable) & (1u << n)) != 0u; }
uint32_t Au1300Lcd::WindowPriority(uint32_t n) const { return (WinReg(n, kOffWinCtrl1) >> 30) & 0x3u; }
uint32_t Au1300Lcd::WindowFormat(uint32_t n) const { return (WinReg(n, kOffWinCtrl1) >> 25) & 0xFu; }
uint32_t Au1300Lcd::WindowWidth(uint32_t n) const { return ((WinReg(n, kOffWinCtrl1) >> 11) & 0x7FFu) + 1u; }
uint32_t Au1300Lcd::WindowHeight(uint32_t n) const { return (WinReg(n, kOffWinCtrl1) & 0x7FFu) + 1u; }
uint32_t Au1300Lcd::WindowOx(uint32_t n) const { return (WinReg(n, kOffWinCtrl0) >> 21) & 0x7FFu; }
uint32_t Au1300Lcd::WindowOy(uint32_t n) const { return (WinReg(n, kOffWinCtrl0) >> 10) & 0x7FFu; }
uint32_t Au1300Lcd::WindowBufAddr(uint32_t n) const {
    const bool use_buf1 = (WinReg(n, kOffWinBufCtrl) & 1u) != 0u;
    return WinReg(n, use_buf1 ? kOffWinBuf1 : kOffWinBuf0);
}

REGISTER_SERVICE(Au1300Lcd);

}
