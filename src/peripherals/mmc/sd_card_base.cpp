#include "sd_card_base.h"

#include <cerfcore/core/byte_order.h>
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/socs/guest_cpu_reset.h>
#include <cerfcore/state/state_stream.h>

namespace cerfcore {

using namespace cerf_sd;

/* Physical Layer Simplified Spec 4.10 §5.3, Table 5-13: CID register bit
   layout. CID/CSD are both sent MSB-first as a 128-bit field; the 4
   response words here follow the same "most significant word first"
   convention EncodeEmmcCid/EncodeEmmcCsd already use for the eMMC case
   (R2 response: bits 127:8 of the register, CRC7+stop bit in the low
   8 bits are synthesized by the host controller hardware, not stored
   in these words - same split EncodeEmmcCid already relies on). */
void EncodeSdCid(const SdCardCid& cid, uint32_t out[4]) {
    out[3] = (uint32_t(cid[0]) << 24) | (uint32_t(cid[1]) << 16) | (uint32_t(cid[2]) << 8) | cid[3];
    out[2] = (uint32_t(cid[4]) << 24) | (uint32_t(cid[5]) << 16) | (uint32_t(cid[6]) << 8) | cid[7];
    out[1] = (uint32_t(cid[8]) << 24) | (uint32_t(cid[9]) << 16) | (uint32_t(cid[10]) << 8) | cid[11];
    out[0] = (uint32_t(cid[12]) << 24) | (uint32_t(cid[13]) << 16) | (uint32_t(cid[14]) << 8) | cid[15];
}

// Physical Layer Simplified Spec 4.10 §5.3.2, Table 5-16: CSD Version 2.0.
void EncodeSdCsdV2(const SdCsdFields& csd, uint32_t out[4]) {
    uint8_t b[16] = {};
    b[0]  = (1u << 6);  // CSD_STRUCTURE=1 (Version 2.0), bits 127:126
    b[1]  = csd.taac;
    b[2]  = csd.nsac;
    b[3]  = csd.tran_speed;
    b[4]  = static_cast<uint8_t>(csd.ccc >> 4);
    b[5]  = static_cast<uint8_t>((csd.ccc & 0xFu) << 4) | (csd.read_bl_len & 0xFu);
    // bits [119:118] reserved=0, [117:112]=C_SIZE[21:16]
    b[6]  = static_cast<uint8_t>((csd.c_size >> 16) & 0x3Fu);
    b[7]  = static_cast<uint8_t>((csd.c_size >> 8) & 0xFFu);
    b[8]  = static_cast<uint8_t>(csd.c_size & 0xFFu);
    b[9]  = static_cast<uint8_t>((csd.erase_blk_en & 1u) << 6) | ((csd.sector_size >> 1) & 0x3Fu);
    b[10] = static_cast<uint8_t>((csd.sector_size & 1u) << 7) | (csd.wp_grp_size & 0x7Fu);
    b[11] = static_cast<uint8_t>((csd.wp_grp_enable & 1u) << 7) | ((csd.r2w_factor & 7u) << 2) |
            ((csd.write_bl_len >> 2) & 3u);
    b[12] = static_cast<uint8_t>((csd.write_bl_len & 3u) << 6);
    b[13] = 0u;
    b[14] = 0u;  // COPY/PERM_WRITE_PROTECT/TMP_WRITE_PROTECT/FILE_FORMAT left 0
    b[15] = 0u;  // CRC7+stop bit synthesized by the host controller, not stored here
    out[3] = (uint32_t(b[0]) << 24) | (uint32_t(b[1]) << 16) | (uint32_t(b[2]) << 8) | b[3];
    out[2] = (uint32_t(b[4]) << 24) | (uint32_t(b[5]) << 16) | (uint32_t(b[6]) << 8) | b[7];
    out[1] = (uint32_t(b[8]) << 24) | (uint32_t(b[9]) << 16) | (uint32_t(b[10]) << 8) | b[11];
    out[0] = (uint32_t(b[12]) << 24) | (uint32_t(b[13]) << 16) | (uint32_t(b[14]) << 8) | b[15];
}

void SdCardBase::OnReady() {
    emu_.Get<GuestCpuReset>().RegisterResetListener([this](ResetLineKind) { Reset(); });
}

uint32_t SdCardBase::StatusWord(SdState before) const {
    return kR1ReadyForData | (static_cast<uint32_t>(before) << kR1StateShift);
}

MmcCommandResult SdCardBase::Command(uint8_t index, uint32_t argument, uint32_t response[4]) {
    const SdState  before     = state_;
    const uint16_t arg_rca    = static_cast<uint16_t>(argument >> 16);
    const bool     is_app_cmd = app_cmd_armed_;
    app_cmd_armed_            = false;

    read_data_.clear();

    if (is_app_cmd) {
        switch (index) {
        case kAcmdSetBusWidth:
            // Physical Layer Simplified Spec 4.10 §4.3.4, Table 4-33: 00b=1
            // bit, 10b=4 bit. Accepted and ignored - this project's DBDMA-
            // driven data path does not model bus width.
            if (before != SdState::Tran) break;
            response[0] = StatusWord(before) | kR1AppCmd;
            return MmcCommandResult::Short;

        case kAcmdSdSendOpCond:
            if (before != SdState::Idle && before != SdState::Ready) break;
            state_      = SdState::Ready;
            response[0] = kOcrBusy | kOcrCcs | kOcrVoltage;
            return MmcCommandResult::Short;

        default:
            break;
        }
        HaltUnmodelledCommand(index, argument, true);
    }

    switch (index) {
    case kCmdGoIdleState:
        if (argument != kGoIdleArgument) break;
        multi_read_  = false;
        multi_write_ = false;
        state_       = SdState::Idle;
        rca_         = 0u;
        return MmcCommandResult::NoResponse;

    case kCmdSendIfCond:
        // Physical Layer Simplified Spec 4.10 §4.3.13, Table 4-28 (R7):
        // echo the check pattern and voltage range unchanged.
        if ((argument & kIfCondVoltageMask) != kIfCondVoltage27to36) break;
        response[0] = argument & (kIfCondVoltageMask | kIfCondCheckPatternMask);
        return MmcCommandResult::Short;

    case kCmdAppCmd:
        if (before != SdState::Idle && before != SdState::Ready && before != SdState::Stby &&
            before != SdState::Tran) {
            break;
        }
        app_cmd_armed_ = true;
        response[0]    = StatusWord(before) | kR1AppCmd;
        return MmcCommandResult::Short;

    case kCmdAllSendCid:
        if (before != SdState::Ready) break;
        state_ = SdState::Ident;
        EncodeSdCid(Cid(), response);
        return MmcCommandResult::Long;

    case kCmdSendRelativeAddr:
        // Physical Layer Simplified Spec 4.10 §4.9.5: the card, not the
        // host, publishes the RCA here - fixed rather than randomized,
        // since nothing in this project needs RCA collision handling.
        if (before != SdState::Ident && before != SdState::Stby) break;
        rca_        = 0xAAAAu;
        state_      = SdState::Stby;
        response[0] = (uint32_t(rca_) << 16) | (StatusWord(before) & 0xFFFFu);
        return MmcCommandResult::Short;

    case kCmdSendCsd:
        if (before != SdState::Stby) break;
        if (arg_rca != rca_) return MmcCommandResult::NoResponse;
        EncodeSdCsdV2(Csd(), response);
        return MmcCommandResult::Long;

    case kCmdSendCid:
        if (before != SdState::Stby) break;
        if (arg_rca != rca_) return MmcCommandResult::NoResponse;
        EncodeSdCid(Cid(), response);
        return MmcCommandResult::Long;

    case kCmdSelectCard:
        if (before == SdState::Stby && arg_rca == rca_ && rca_ != 0u) {
            state_      = SdState::Tran;
            response[0] = StatusWord(before);
            return MmcCommandResult::Short;
        }
        if (before == SdState::Tran && arg_rca != rca_) {
            state_ = SdState::Stby;
            return MmcCommandResult::NoResponse;
        }
        break;

    case kCmdSendStatus:
        if (before != SdState::Stby && before != SdState::Tran && before != SdState::Data) break;
        if (arg_rca != rca_) return MmcCommandResult::NoResponse;
        response[0] = StatusWord(before);
        return MmcCommandResult::Short;

    case kCmdSetBlocklen:
        // Physical Layer Simplified Spec 4.10 §4.3.3: for an SDHC/SDXC
        // card (CCS=1, this card), block length is fixed at 512 bytes
        // and this command exists only for backward compatibility -
        // accepted (no-op) for any argument equal to that fixed length.
        if (before != SdState::Tran) break;
        if (argument != kBlockBytes) {
            emu_.Get<Fatal>().Die(
                "SD card in slot %u: SET_BLOCKLEN %u is not modeled (this card is "
                "SDHC/SDXC, fixed at %u-byte blocks)", SlotIndex(), argument, kBlockBytes);
        }
        response[0] = StatusWord(before);
        return MmcCommandResult::Short;

    case kCmdReadSingleBlock:
    case kCmdReadMultiBlock:
        if (before != SdState::Tran) break;
        if (argument >= SectorCount()) {
            response[0] = StatusWord(before) | kR1AddressOutOfRange;
            return MmcCommandResult::Short;
        }
        read_data_.resize(kBlockBytes);
        ReadBlock(argument, read_data_.data());
        multi_read_  = (index == kCmdReadMultiBlock);
        next_sector_ = argument + 1u;
        state_       = SdState::Data;
        response[0]  = StatusWord(before);
        return MmcCommandResult::Short;

    case kCmdWriteBlock:
    case kCmdWriteMultiBlock:
        if (before != SdState::Tran) break;
        if (argument >= SectorCount()) {
            response[0] = StatusWord(before) | kR1AddressOutOfRange;
            return MmcCommandResult::Short;
        }
        multi_write_ = (index == kCmdWriteMultiBlock);
        next_sector_ = argument;
        state_       = SdState::Rcv;
        response[0]  = StatusWord(before);
        return MmcCommandResult::Short;

    case kCmdStopTransmission:
        if (before != SdState::Data && !(before == SdState::Rcv && multi_write_)) break;
        if ((argument & kStopHpi) != 0u) {
            emu_.Get<Fatal>().Die(
                "SD card in slot %u: STOP_TRANSMISSION argument 0x%08X sets a "
                "bit this card does not model", SlotIndex(), argument);
        }
        multi_read_  = false;
        multi_write_ = false;
        state_       = SdState::Tran;
        response[0]  = StatusWord(before);
        return MmcCommandResult::Short;

    default:
        break;
    }

    HaltUnmodelledCommand(index, argument, false);
}

void SdCardBase::EndDataPhase() {
    if (state_ != SdState::Data) {
        emu_.Get<Fatal>().Die(
            "SD card in slot %u: the host finished a data phase while the card "
            "is in state %u", SlotIndex(), static_cast<unsigned>(state_));
    }
    if (!multi_read_) state_ = SdState::Tran;
}

void SdCardBase::NextBlock() {
    if (state_ != SdState::Data || !multi_read_) {
        emu_.Get<Fatal>().Die(
            "SD card in slot %u: the host asked for another block while the "
            "card is in state %u with no multiple block read open", SlotIndex(),
            static_cast<unsigned>(state_));
    }
    if (next_sector_ >= SectorCount()) {
        emu_.Get<Fatal>().Die(
            "SD card in slot %u: a multiple block read runs past the last "
            "sector into sector %u", SlotIndex(), next_sector_);
    }
    read_data_.resize(kBlockBytes);
    ReadBlock(next_sector_, read_data_.data());
    ++next_sector_;
}

void SdCardBase::ReceiveBlock(const uint8_t* data, uint32_t bytes) {
    if (state_ != SdState::Rcv) {
        emu_.Get<Fatal>().Die(
            "SD card in slot %u: the host sent a %u byte data block while the "
            "card is in state %u", SlotIndex(), bytes, static_cast<unsigned>(state_));
    }
    if (bytes != kBlockBytes) {
        emu_.Get<Fatal>().Die(
            "SD card in slot %u: the host sent a %u byte data block, and a write "
            "block length other than %u is not modeled", SlotIndex(), bytes, kBlockBytes);
    }
    if (next_sector_ >= SectorCount()) {
        emu_.Get<Fatal>().Die(
            "SD card in slot %u: a multiple block write runs past the last "
            "sector into sector %u", SlotIndex(), next_sector_);
    }
    WriteBlock(next_sector_, data);
    ++next_sector_;
    if (!multi_write_) state_ = SdState::Tran;
}

void SdCardBase::Reset() {
    state_         = SdState::Idle;
    rca_           = 0u;
    app_cmd_armed_ = false;
    multi_read_    = false;
    multi_write_   = false;
    next_sector_   = 0u;
    read_data_.clear();
}

void SdCardBase::HaltUnmodelledCommand(uint8_t index, uint32_t argument, bool app_cmd) {
    emu_.Get<Fatal>().Die(
        "SD card in slot %u: %s%u with argument 0x%08X in card state %u is not "
        "modeled", SlotIndex(), app_cmd ? "ACMD" : "CMD", static_cast<unsigned>(index),
        argument, static_cast<unsigned>(state_));
}

void SdCardBase::SaveState(StateWriter& w) {
    w.Write<uint32_t>("card_state", static_cast<uint32_t>(state_));
    w.Write<uint32_t>("rca", rca_);
    w.Write<uint32_t>("app_cmd_armed", app_cmd_armed_ ? 1u : 0u);
    w.Write<uint32_t>("multi_read", multi_read_ ? 1u : 0u);
    w.Write<uint32_t>("multi_write", multi_write_ ? 1u : 0u);
    w.Write<uint32_t>("next_sector", next_sector_);
}

void SdCardBase::RestoreState(StateReader& r) {
    uint32_t state    = 0u;
    uint32_t rca      = 0u;
    uint32_t app_cmd  = 0u;
    uint32_t multi    = 0u;
    uint32_t multi_w  = 0u;
    uint32_t next     = 0u;
    r.Read("card_state", state);
    r.Read("rca", rca);
    r.Read("app_cmd_armed", app_cmd);
    r.Read("multi_read", multi);
    r.Read("multi_write", multi_w);
    r.Read("next_sector", next);
    const bool     receiving = state == static_cast<uint32_t>(SdState::Rcv);
    const uint32_t last_next =
        (receiving && multi_w == 0u) ? SectorCount() - 1u : SectorCount();
    if (multi > 1u || multi_w > 1u || app_cmd > 1u || next > last_next ||
        (multi == 1u && state != static_cast<uint32_t>(SdState::Data)) ||
        (multi_w == 1u && !receiving)) {
        r.Reject(
            "SD card in slot %u: restored multiple block read %u, multiple "
            "block write %u at sector %u in state %u is not a transfer this card "
            "can hold", SlotIndex(), multi, multi_w, next, state);
    }
    if (state > static_cast<uint32_t>(SdState::Rcv) || rca > 0xFFFFu) {
        r.Reject(
            "SD card in slot %u: restored state %u rca 0x%X is not a state this "
            "card can reach", SlotIndex(), state, rca);
    }
    state_         = static_cast<SdState>(state);
    rca_           = static_cast<uint16_t>(rca);
    app_cmd_armed_ = app_cmd != 0u;
    multi_read_    = (multi == 1u);
    multi_write_   = (multi_w == 1u);
    next_sector_   = next;
}

}
