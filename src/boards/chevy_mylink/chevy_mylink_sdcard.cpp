#include "../../peripherals/mmc/sd_card_base.h"

#include <cerfcore/boards/board_context.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/cerf_paths.h>
#include <cerfcore/core/device_config.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/core/log.h>
#include "../../storage/disk_image.h"
#include "chevy_mylink_family_id.h"

namespace cerfcore {

namespace {

constexpr uint32_t kSdCardSlotIndex = 0u;

/* This card is this project's own emulated removable storage card, not a
   model of a specific real physical part the way EmmcCardBase's existing
   SkHynixH26m52002ckr is (that one cites a real datasheet) - chevy_mylink's
   real hardware takes a genuinely removable SD card, so there is no single
   "the" card to model, and these CID/CSD values are a reasonable synthetic
   identity, not a datasheet-grounded one. The one thing that IS grounded:
   OCR.CCS=1 (SDHC/SDXC, block-addressed) in sd_card_base.cpp's ACMD41
   response, matching this card's real backing image being far larger than
   the 2 GiB SDSC byte-addressing limit. */
constexpr SdCardCid kCid = {
    0x03u,                         // MID (arbitrary, not a registered manufacturer ID)
    'C', 'F',                      // OID
    'C', 'E', 'R', 'F', 'D',       // product name (5 chars)
    0x10u,                         // PRV (revision 1.0)
    0x00u, 0x00u, 0x00u, 0x01u,    // PSN (serial number)
    0x01u, 0x80u,                  // MDT (manufacturing date) + reserved nibble
    0x00u,                         // CRC7 + stop bit, synthesized by the host controller
};

// Physical Layer Simplified Spec 4.10 Table 5-16 field values for this
// card's 512 MiB capacity: C_SIZE=0x3FF gives (0x3FF+1)*512 KiB = 512 MiB,
// i.e. (C_SIZE+1)*1024 sectors = 1,048,576 sectors, matching kSectorCount
// below exactly.
constexpr SdCsdFields kCsd = {
    .taac          = 0x0Eu,
    .nsac          = 0x00u,
    .tran_speed    = 0x32u,  // 25 MHz, the standard SD default speed value
    .ccc           = 0x5B5u,
    .read_bl_len   = 9u,     // 2^9 = 512 bytes
    .c_size        = 0x3FFu,
    .erase_blk_en  = 1u,
    .sector_size   = 0x7Fu,
    .wp_grp_size   = 0u,
    .wp_grp_enable = 0u,     // "shall be set to 0" for SDHC/SDXC (Spec 4.10 Table 5-16)
    .r2w_factor    = 2u,
    .write_bl_len  = 9u,
};

constexpr uint32_t kSectorCount = 0x00100000u;  // 1,048,576 sectors = 512 MiB

class ChevyMylinkSdCard : public SdCardBase {
public:
    using SdCardBase::SdCardBase;

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetBoardFamilyId() == BoardFamilyId::ChevyMylink;
    }

    uint32_t SlotIndex() const override { return kSdCardSlotIndex; }

    void OnReady() override {
        SdCardBase::OnReady();
        OpenStore();
    }

protected:
    SdCardCid   Cid() const override { return kCid; }
    SdCsdFields Csd() const override { return kCsd; }
    uint32_t    SectorCount() const override { return kSectorCount; }

    void ReadBlock(uint32_t sector, uint8_t* out) override {
        if (!store_.ReadSectors(sector, 1u, out)) {
            emu_.Get<Fatal>().Die(
                "SD card in slot %u: sector %u cannot be read from %s",
                kSdCardSlotIndex, sector, store_path_.c_str());
        }
    }

    void WriteBlock(uint32_t sector, const uint8_t* data) override {
        if (!store_.WriteSectors(sector, 1u, data)) {
            emu_.Get<Fatal>().Die(
                "SD card in slot %u: sector %u cannot be written to %s",
                kSdCardSlotIndex, sector, store_path_.c_str());
        }
    }

private:
    void OpenStore() {
        const DeviceConfig& cfg = emu_.Get<DeviceConfig>();
        store_path_ = ResolveDeviceFile(cfg.device_name, cfg.storage_sdcard);
        if (!store_.Open(store_path_, uint64_t(kSectorCount) * DiskImage::kSectorSize,
                         /*create_if_missing=*/false)) {
            emu_.Get<Fatal>().Die(
                "SD card in slot %u: storage.sdcard %s cannot be opened - this "
                "project does not synthesize a FAT filesystem at runtime, so a "
                "real card image must be placed there first", kSdCardSlotIndex,
                store_path_.c_str());
        }
        if (store_.SectorCount() != kSectorCount) {
            emu_.Get<Fatal>().Die(
                "SD card in slot %u: storage.sdcard %s holds %llu sectors, and "
                "this card is fixed at %u", kSdCardSlotIndex, store_path_.c_str(),
                static_cast<unsigned long long>(store_.SectorCount()), kSectorCount);
        }
        LOG(Boot, "SD card in slot %u: using storage.sdcard %s\n", kSdCardSlotIndex,
            store_path_.c_str());
    }

    DiskImage   store_;
    std::string store_path_;
};

}  // namespace

REGISTER_SERVICE_AS(ChevyMylinkSdCard, MmcCard);

}
