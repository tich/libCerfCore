#pragma once

#include <cstdint>

namespace cerfcore {

namespace ChevyMylinkDram {

inline constexpr uint32_t kPaBase = 0x00000000u;
/* 256 MiB (0x10000000), not the previous 0x0D700000 (~216 MiB): the
   guest's own MEMPOOL trace (NKDBG "MEMPOOL: Found LCD Memory
   Region(2). Phys:0x0E200000 ... Size:0x01E00000") reserves a
   framebuffer carve-out that runs to 0x10000000 exactly, and a write
   to that carve-out (decompiled VA 0xC0A33EA8, once LCD/display init
   reached the point of actually drawing into the framebuffer) halted
   as unmapped RAM under the old, smaller size. 0x10000000 is also
   exactly where Au1300's own peripheral MMIO region starts (ROM,
   OTP, GPIC, etc. all live at 0x10000000 and above - see every other
   au1300_*.cpp file in this project), so this is the natural RAM/MMIO
   boundary, not an arbitrary round number. */
inline constexpr uint32_t kSize   = 0x10000000u;

}

}
