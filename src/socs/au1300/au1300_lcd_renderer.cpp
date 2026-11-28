#include "au1300_lcd.h"

#include <cerfcore/boards/board_context.h>
#include "au1300_id.h"
#include <cerfcore/core/byte_order.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/cpu/emulated_memory.h>
#include "../../host/panel_frame_renderer.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>

namespace cerfcore {

namespace {

/* au1200fb.h LCD_WINCTRL1_FRM values this renderer understands - see
   au1300_lcd.cpp's grounding comment for the full field table. 32bpp
   (13) and 24bpp (12) are both real, confirmed-live values (window 0
   actually runs FRM=0xC=12 despite the "BPP:32" NKDBG string this
   project first saw - that string evidently describes something
   else, maybe the panel's total addressable depth, not this specific
   window's own format). Any other FRM value halts rather than
   guessing a conversion this project has no live evidence for. */
constexpr uint32_t kFrm24Bpp = 12u;
constexpr uint32_t kFrm32Bpp = 13u;

/* 32bpp is ARGB8888 in memory - a little-endian 32-bit ARGB8888 word
   is already byte-order B,G,R,A, i.e. CERF's own dib_bgra32 layout,
   so no per-pixel conversion is needed there, just a straight word
   copy. 24bpp's exact byte order (B,G,R vs R,G,B) is NOT independently
   confirmed - packed as B,G,R here (3 bytes/pixel, no padding) as the
   first guess; if the rendered image's colors come out visibly wrong
   once actually viewed, swap this order rather than assuming it's
   right. */

class Au1300LcdRenderer : public PanelFrameRenderer {
public:
    using PanelFrameRenderer::PanelFrameRenderer;

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetSocId() == SocId::Au1300;
    }

    /* The "screen" register's own SEN/SX/SY fields are a real,
       confirmed-live register (SEN is set), but this guest never
       writes SX/SY to the actual panel resolution - both read back 0
       (width/height "1"), so they cannot be the size source. The
       enabled window(s)' own OX/OY/SZX/SZY (independently confirmed
       live: window 0 reports 800x480, matching "WVGA_800x480" from
       this project's own earlier NKDBG trace) are reliable, so
       PresentedSize/HasFrame both derive size from Au1300Lcd's own
       EnabledWindowBounds (the same method WriteWord uses to decide
       when to call HostPresentation::GuestRemoded) instead of the
       screen register. */
    void PresentedSize(uint32_t& w, uint32_t& h) override {
        emu_.Get<Au1300Lcd>().EnabledWindowBounds(w, h);
    }

    bool HasFrame() override {
        auto& lcd = emu_.Get<Au1300Lcd>();
        if (!lcd.ScreenEnabled()) return false;
        uint32_t w = 0, h = 0;
        lcd.EnabledWindowBounds(w, h);
        return w != 0 && h != 0;
    }

    void RenderInto(uint32_t* dib_bgra32, uint32_t host_w, uint32_t host_h) override {
        auto& lcd = emu_.Get<Au1300Lcd>();
        const uint32_t bg = lcd.BackColorBgr() | 0xFF000000u;
        for (uint32_t i = 0; i < (size_t)host_w * host_h; ++i) dib_bgra32[i] = bg;

        /* Draw enabled windows in increasing priority order, so a
           higher-priority window ends up on top - no alpha blending
           (WINCTRL0.A/AEN not modeled), matching this project's
           "simplest thing that shows real pixels" first pass. */
        std::array<uint32_t, Au1300Lcd::kWindowCount> order = {0, 1, 2, 3};
        std::sort(order.begin(), order.end(), [&](uint32_t a, uint32_t b) {
            return lcd.WindowPriority(a) < lcd.WindowPriority(b);
        });

        auto& mem = emu_.Get<EmulatedMemory>();
        for (uint32_t n : order) {
            if (!lcd.WindowEnabled(n)) continue;
            const uint32_t fb_pa = lcd.WindowBufAddr(n);
            if (fb_pa == 0) continue;
            const uint32_t fmt = lcd.WindowFormat(n);
            if (fmt != kFrm32Bpp && fmt != kFrm24Bpp) {
                emu_.Get<Fatal>().Die(
                    "Au1300LcdRenderer: window %u FRM=0x%X not modelled (only "
                    "24bpp=12 and 32bpp=13 verified live). Halting.", n, fmt);
            }
            const uint32_t bytes_per_pixel = (fmt == kFrm32Bpp) ? 4u : 3u;

            const uint32_t win_w = lcd.WindowWidth(n);
            const uint32_t win_h = lcd.WindowHeight(n);
            const uint32_t ox = lcd.WindowOx(n);
            const uint32_t oy = lcd.WindowOy(n);
            if (win_w == 0 || win_h == 0 || ox >= host_w || oy >= host_h) continue;

            const uint8_t* src_base = mem.TryTranslate(fb_pa);
            if (!src_base) continue;

            const uint32_t copy_w = std::min(win_w, host_w - ox);
            const uint32_t copy_h = std::min(win_h, host_h - oy);
            for (uint32_t y = 0; y < copy_h; ++y) {
                const uint8_t* src_row = src_base + (size_t)y * win_w * bytes_per_pixel;
                uint32_t* dst_row = dib_bgra32 + (size_t)(oy + y) * host_w + ox;
                if (fmt == kFrm32Bpp) {
                    std::memcpy(dst_row, src_row, (size_t)copy_w * 4u);
                } else {
                    for (uint32_t x = 0; x < copy_w; ++x) {
                        const uint8_t* px = src_row + (size_t)x * 3u;
                        dst_row[x] = 0xFF000000u | (uint32_t(px[2]) << 16) |
                                     (uint32_t(px[1]) << 8) | uint32_t(px[0]);
                    }
                }
            }
        }
    }
};

}  /* namespace */

REGISTER_SERVICE_AS(Au1300LcdRenderer, PanelFrameRenderer);

}
