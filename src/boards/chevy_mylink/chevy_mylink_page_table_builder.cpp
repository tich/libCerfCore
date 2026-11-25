#include "../unmapped_window_page_table_builder.h"

#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/boards/board_context.h>
#include "chevy_mylink_dram.h"
#include "chevy_mylink_family_id.h"

#include <cstdint>
#include <vector>

namespace cerfcore {

namespace {

constexpr UnmappedWindowDram kDram = UnmappedWindowPageTableBuilder::CachedWindowDram(
    ChevyMylinkDram::kPaBase, ChevyMylinkDram::kSize, 0u, ChevyMylinkDram::kSize);

class ChevyMylinkPageTableBuilder : public UnmappedWindowPageTableBuilder {
public:
    using UnmappedWindowPageTableBuilder::UnmappedWindowPageTableBuilder;

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetBoardFamilyId() == BoardFamilyId::ChevyMylink;
    }

    std::vector<DramRegion> CachedDramRegions() const override {
        std::vector<DramRegion> r;
        kDram.AddCachedTo(r);
        return r;
    }

    std::vector<BackedRegion> BackedMemoryRegions() const override {
        std::vector<BackedRegion> r;
        kDram.AddBackedTo(r);
        return r;
    }

    std::vector<DramRegion> MappedVaSpans() const override {
        std::vector<DramRegion> r;
        kDram.AddMappedTo(r);
        return r;
    }
};

}  /* namespace */

REGISTER_SERVICE_AS(ChevyMylinkPageTableBuilder, PageTableBuilder);

}
