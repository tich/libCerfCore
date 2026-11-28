#pragma once

#include <cerfcore/peripherals/peripheral_base.h>

#include <array>
#include <cstdint>

namespace cerfcore {

/* Au1300 LCD/display controller - see au1300_lcd.cpp for the full
   grounding comment. Declared in a header (rather than kept
   anonymous, like most single-instance Au1300 peripherals in this
   project) so au1300_lcd_renderer.cpp can read window/scanout state
   via emu_.Get<Au1300Lcd>(), the same pattern au1300_ehci_cap.cpp/
   au1300_ohci_cap.cpp already use for Au1300Ehci/Au1300Ohci0. */
class Au1300Lcd : public Peripheral {
public:
    using Peripheral::Peripheral;

    bool ShouldRegister() override;
    void OnReady() override;

    uint32_t MmioBase() const override;
    uint32_t MmioSize() const override;

    uint32_t ReadWord(uint32_t addr) override;
    void     WriteWord(uint32_t addr, uint32_t value) override;
    uint8_t  ReadByte(uint32_t addr) override;
    void     WriteByte(uint32_t addr, uint8_t value) override;
    uint16_t ReadHalf(uint32_t addr) override;
    void     WriteHalf(uint32_t addr, uint16_t value) override;

    void SaveState(StateWriter& w) override;
    void RestoreState(StateReader& r) override;

    /* Au1200fb struct au1200_lcd field accessors (see the .cpp's
       grounding comment for every offset/bit-field name). Window
       index n is 0-3. */
    static constexpr uint32_t kWindowCount = 4u;

    bool     ScreenEnabled() const;
    uint32_t ScreenWidth() const;   /* SX+1 */
    uint32_t ScreenHeight() const;  /* SY+1 */
    uint32_t BackColorBgr() const;  /* 0x00RRGGBB-ish packed per the header's SBGR/SBGG/SBGB fields */

    bool     WindowEnabled(uint32_t n) const;
    uint32_t WindowPriority(uint32_t n) const;  /* WINCTRL0.PRI equivalent is actually in WINCTRL1 */
    uint32_t WindowFormat(uint32_t n) const;    /* WINCTRL1.FRM, 5 bits */
    uint32_t WindowWidth(uint32_t n) const;     /* WINCTRL1.SZX+1 */
    uint32_t WindowHeight(uint32_t n) const;    /* WINCTRL1.SZY+1 */
    uint32_t WindowOx(uint32_t n) const;        /* WINCTRL0.OX */
    uint32_t WindowOy(uint32_t n) const;        /* WINCTRL0.OY */
    uint32_t WindowBufAddr(uint32_t n) const;   /* winbuf0 or winbuf1, per winbufctrl.DBN */

    /* Bounding box (max over enabled windows of OX+width, OY+height) -
       see au1300_lcd_renderer.cpp's comment on why this, not the
       screen register's own SX/SY, is this guest's real display
       size. Shared between the renderer (for PresentedSize/HasFrame)
       and WriteWord's own GuestRemoded notification below, so both
       always agree. */
    void EnabledWindowBounds(uint32_t& w, uint32_t& h) const;

private:
    static constexpr uint32_t kBase = 0x15000000u;
    static constexpr uint32_t kSize = 0x00000900u;

    static constexpr uint32_t kOffScreen     = 0x04u;
    static constexpr uint32_t kOffBackColor  = 0x08u;
    static constexpr uint32_t kOffWinEnable  = 0x24u;
    static constexpr uint32_t kOffWindowBase = 0x100u;
    static constexpr uint32_t kWindowStride  = 0x20u;
    static constexpr uint32_t kOffWinCtrl0   = 0x00u;
    static constexpr uint32_t kOffWinCtrl1   = 0x04u;
    static constexpr uint32_t kOffWinBuf0    = 0x0Cu;
    static constexpr uint32_t kOffWinBuf1    = 0x10u;
    static constexpr uint32_t kOffWinBufCtrl = 0x14u;

    uint32_t Reg(uint32_t off) const { return regs_[off / 4u]; }
    uint32_t WinReg(uint32_t n, uint32_t off) const {
        return Reg(kOffWindowBase + n * kWindowStride + off);
    }

    void NotifyIfGeometryOffset(uint32_t off);
    void NotifyIfRemoded();

    std::array<uint32_t, kSize / 4u> regs_{};
    uint32_t notified_w_ = 0;
    uint32_t notified_h_ = 0;
};

}
