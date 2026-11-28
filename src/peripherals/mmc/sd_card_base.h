#pragma once

#include "mmc_card.h"

#include <cerfcore/core/sd_card_cid.h>

#include <cstdint>
#include <vector>

namespace cerfcore {

/* Physical Layer Simplified Specification - SD protocol command set, as
   distinct from the project's existing EmmcCardBase (JEDEC eMMC): SD
   cards use CMD55+ACMD41 (not a raw CMD1) to leave idle state, the CARD
   itself picks its own RCA and returns it in CMD3's response (the host
   never supplies one), and there is no SWITCH/EXT_CSD - only the basic
   CSD (version 2.0/"CSD structure 1", used by every SDHC/SDXC card,
   which this models as block-addressed: CCS=1 in the ACMD41 response,
   so CMD17/18/24/25's argument is a sector number like eMMC's, not a
   byte offset - same ReadBlock/WriteBlock(sector) shape as
   EmmcCardBase for that reason). */
namespace cerf_sd {

constexpr uint8_t kCmdGoIdleState      = 0u;
constexpr uint8_t kCmdAllSendCid       = 2u;
constexpr uint8_t kCmdSendRelativeAddr = 3u;
constexpr uint8_t kCmdSelectCard       = 7u;
constexpr uint8_t kCmdSendIfCond       = 8u;
constexpr uint8_t kCmdSendCsd          = 9u;
constexpr uint8_t kCmdSendCid          = 10u;
constexpr uint8_t kCmdStopTransmission = 12u;
constexpr uint8_t kCmdSendStatus       = 13u;
constexpr uint8_t kCmdSetBlocklen      = 16u;
constexpr uint8_t kCmdReadSingleBlock  = 17u;
constexpr uint8_t kCmdReadMultiBlock   = 18u;
constexpr uint8_t kCmdWriteBlock       = 24u;
constexpr uint8_t kCmdWriteMultiBlock  = 25u;
constexpr uint8_t kAcmdSetBusWidth     = 6u;
constexpr uint8_t kAcmdSdSendOpCond    = 41u;
constexpr uint8_t kCmdAppCmd           = 55u;

constexpr uint32_t kBlockBytes = 512u;

constexpr uint32_t kGoIdleArgument = 0u;
constexpr uint32_t kStopHpi        = 1u << 0;

// Physical Layer Simplified Spec 4.10 Table 4-3: "pattern" is the low byte of
// CMD8's argument, echoed back unchanged in the R7 response.
constexpr uint32_t kIfCondCheckPatternMask = 0xFFu;
constexpr uint32_t kIfCondVoltageMask      = 0xF00u;
constexpr uint32_t kIfCondVoltage27to36    = 0x100u;

// Physical Layer Simplified Spec 4.10 Table 5-1: OCR register.
constexpr uint32_t kOcrBusy    = 1u << 31;
constexpr uint32_t kOcrCcs     = 1u << 30;  // Card Capacity Status: 1 = SDHC/SDXC (block-addressed)
constexpr uint32_t kOcrVoltage = 0x00FF8000u;
constexpr uint32_t kAcmd41Hcs  = 1u << 30;  // Host Capacity Support, set by every modern host

constexpr uint32_t kR1ReadyForData      = 1u << 8;
constexpr uint32_t kR1AppCmd            = 1u << 5;
constexpr uint32_t kR1StateShift        = 9u;
constexpr uint32_t kR1AddressOutOfRange = 1u << 31;

enum class SdState : uint32_t {
    Idle  = 0u,
    Ready = 1u,
    Ident = 2u,
    Stby  = 3u,
    Tran  = 4u,
    Data  = 5u,
    Rcv   = 6u,
};

}  // namespace cerf_sd

/* Physical Layer Simplified Spec 4.10 §5.3.2, Table 5-16: CSD Version 2.0
   ("CSD_STRUCTURE"=1), the format every SDHC/SDXC card uses - much
   simpler than eMMC's: capacity is C_SIZE alone (22 bits), no
   READ_BL_LEN/C_SIZE_MULT multiplication (capacity = (C_SIZE+1) *
   512 KiB, fixed by the spec, not stored in any field). */
struct SdCsdFields {
    uint8_t  taac;
    uint8_t  nsac;
    uint8_t  tran_speed;
    uint16_t ccc;
    uint8_t  read_bl_len;
    uint32_t c_size;  // 22 bits
    uint8_t  erase_blk_en;
    uint8_t  sector_size;   // 7 bits
    uint8_t  wp_grp_size;   // 7 bits
    uint8_t  wp_grp_enable;
    uint8_t  r2w_factor;
    uint8_t  write_bl_len;
};

void EncodeSdCid(const SdCardCid& cid, uint32_t out[4]);
void EncodeSdCsdV2(const SdCsdFields& csd, uint32_t out[4]);

class SdCardBase : public MmcCard {
public:
    using MmcCard::MmcCard;

    void OnReady() override;

    MmcCommandResult Command(uint8_t index, uint32_t argument,
                             uint32_t response[4]) override;

    const std::vector<uint8_t>& ReadData() const override { return read_data_; }

    void NextBlock() override;

    void EndDataPhase() override;

    void ReceiveBlock(const uint8_t* data, uint32_t bytes) override;

    void Reset() override;

    void SaveState(StateWriter& w) override;
    void RestoreState(StateReader& r) override;

protected:
    virtual SdCardCid  Cid() const = 0;
    virtual SdCsdFields Csd() const = 0;
    virtual uint32_t   SectorCount() const = 0;
    virtual void ReadBlock(uint32_t sector, uint8_t* out) = 0;
    virtual void WriteBlock(uint32_t sector, const uint8_t* data) = 0;

private:
    uint32_t StatusWord(cerf_sd::SdState before) const;
    [[noreturn]] void HaltUnmodelledCommand(uint8_t index, uint32_t argument, bool app_cmd);

    cerf_sd::SdState     state_       = cerf_sd::SdState::Idle;
    uint16_t             rca_         = 0u;
    bool                 app_cmd_armed_ = false;
    bool                 multi_read_  = false;
    bool                 multi_write_ = false;
    uint32_t             next_sector_ = 0u;
    std::vector<uint8_t> read_data_;
};

}
