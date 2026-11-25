#pragma once

#include <cerfcore/core/service.h>

#include <cstdint>

namespace cerfcore {

struct ParsedRom;
struct ParsedXipRegion;

/* RomPlacer - copies each parsed ROM partition's flat bytes into
   emulated DRAM at the PA the SoC's OAT maps each partition's
   flat_base_va to. */
class RomPlacer : public Service {
public:
    using Service::Service;
    bool ShouldRegister() override;
    void OnReady() override;

    const ParsedXipRegion* PlacedPrimaryXip() const { return placed_primary_; }

private:
    bool PlaceRomXips(const ParsedRom& rom, bool volatile_only);

    /* Runs after PlaceRomXips: applies each XIP ROMHDR's copy table
       (ulCopyEntries COPYentry records at VA ulCopyOffset, each
       {ulSource, ulDest, ulCopyLen, ulDestLen}) - the CE1 loader step that
       populates a module's RAM-resident .data/.bss from a short ROM
       prefix, zero-filling the rest. This project's own ROM parser only
       ever validated ulCopyOffset's position, never walked the table, so
       every such destination silently read as whatever PlaceRomXips left
       there (typically zero) instead of its real initial contents. */
    void ApplyCopyTables(const ParsedRom& rom);

    /* Runs after PlaceRomXips and copies only the B000FF section bytes outside
       every XIP physfirst..physlast range - the multi-XIP XIPCHAIN table past
       the kernel ROMHDR's physlast. Only the backed-region VA span is copied;
       volatile_only as in PlaceRomXips. Single-XIP images place nothing. */
    void PlaceB000FFSections(const ParsedRom& rom, bool volatile_only);
    bool IsVolatilePa(uint32_t pa);

    const ParsedXipRegion* placed_primary_ = nullptr;
};

}
