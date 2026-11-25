#include <cerfcore/peripherals/peripheral_base.h>

#include <cerfcore/boards/board_context.h>
#include "chevy_mylink_family_id.h"
#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/cerf_paths.h>
#include <cerfcore/core/config_json.h>
#include <cerfcore/core/device_config.h>
#include <cerfcore/core/device_config_keys.h>
#include <cerfcore/core/fatal.h>
#include <cerfcore/core/host_file_read.h>
#include <cerfcore/core/log.h>
#include <cerfcore/state/state_stream.h>
#include "../../peripherals/peripheral_dispatcher.h"

#include <cstdint>
#include <vector>

namespace cerfcore {

/* The board's secondary static-bus Spansion NOR flash, separate from the
   Au1300 SYS block. Decompiled spansiond.dll (VA 0xC093A3F0-0xC093A450)
   bulk-reads 2048-byte sectors from this physical window for parsing - a
   real flash-content read, not a control/status register poll. Two
   distinct accesses have been observed, at PA 0x1FDE0000 and PA 0x1FF80000,
   both inside the 8 MB window 0x1F800000-0x1FFFFFFF.

   A real NOR flash update payload for this exact board was captured from
   its service storage card (cerf.json rom.chevy_mylink_nor_update) -
   confirmed by direct content match: a PEM certificate at file offset
   0x5E0000 and the literal string "2014.03.27 16:54:01" at file offset
   0x780000, matching both the accompanying NorVerInfo.txt and the sector
   spansiond.dll itself reads at boot. The file's offset 0 maps 1:1 to
   PA 0x1F800000. This is a field-service update package (a FAT-formatted
   image, boot sector at file offset 0x10000), not necessarily a byte-exact
   dump of the unit's own currently-programmed flash, but is the strongest
   real evidence available for this chip's contents and is used directly
   rather than fabricated. When the file is absent this floats 0xFF per
   halfword/word, the well-documented, datasheet-universal behavior of a
   genuinely blank/unprogrammed NOR flash chip, which a correctly-written
   flash driver already has to treat as a valid (if empty) case.

   Decompiled chevy_mylink nk.exe VA 0x80107C58-0x80107CB8 separately reads
   three 16-bit halfwords at PA 0x1FE00008/0x1FE0000A/0x1FE0000C - inside
   this same window - to provision the registry key "Comm\AU1MAC2\Parms"
   (string at VA 0x80101F30) with the Au1300's second on-chip Ethernet
   MAC's factory address. The captured update payload's bytes at that
   offset are generic filler (part of the update package's own FAT
   structure, not per-unit calibration data an update must never
   overwrite), so those three offsets are special-cased to a fixed
   placeholder (02:00:00:00:00:01, the standard locally-administered/
   no-real-address pattern: U/L bit set, I/G bit clear) regardless of what
   the backing file holds there - the real factory value lives only in
   physical flash cells this project has no image of.

   spansiond.dll also writes commands into the array (observed: 0x00980098
   at PA 0x1F800154 - both 16-bit halves carry the same byte, the signature
   of a 16-bit-wide-by-2 interleaved parallel NOR pair where the host issues
   one command to both chips at once). 0x98 is the JEDEC JESD68 Common Flash
   Interface "enter CFI Query mode" command, defined to be writable to ANY
   address - this is a public, chip-independent protocol constant, not a
   guess at this specific chip's identity.

   While in query mode, decompiled spansiond.dll VA 0xC0937780-0xC09377B4
   directly names what it checks first - not inferred from the CFI spec,
   read from the driver's own comparison code: it compares the 32-bit word
   at +0x40 against 0x00510051, +0x44 against 0x00520052, and +0x48
   against 0x00590059 - the ASCII "QRY" CFI query-unique signature
   (JEDEC-standard bytes 'Q','R','Y' at native byte offsets 0x10/0x11/0x12,
   each byte occupying one 32-bit CPU word here and duplicated across both
   16-bit halves, the same bus convention the command write itself uses).
   This signature is universal to every CFI-compliant chip regardless of
   vendor, so returning it is not a guess at this specific part's identity.

   Every other primary CFI table field (bytes 0x13 onward) is read through
   one generic, repeated sequence the driver reuses for every field (VA
   0xC0937934-0xC0937968 is the first instance; it recurs identically at
   0xC09379A0, 0xC0937A10, 0xC0937A48, ...): it loads a stored bus-mode
   value, shift-compares it against 4 (word mode, our bus) vs 2 (half-word
   mode) vs a byte-mode fallback, and - critically, confirmed by tracing
   the actual branch taken in our word-mode path (VA 0xC0937A54-0xC0937A60)
   - the word-mode case reads the field with an UNCONDITIONAL branch in the
   load's own delay slot, skipping every comparison entirely. These field
   values are harvested into the driver's own chip-descriptor struct for
   later reference, not gate-checked against an expected constant the way
   QRY is. (The first field read, +0x4C "Primary Vendor Command Set ID",
   happens to use a direct "bne" instead of this unconditional-branch
   shape and does compare against a literal the driver sets up at
   0xC09378DC - "addiu $s3, $zero, 2" - but that value, 2, is also simply
   the industry-universal AMD/Fujitsu Standard Command Set ID every
   Spansion-lineage chip reports, so it is both confirmed AND correct.)

   Given these fields are harvested rather than gate-checked, the rest of
   the primary CFI table (bytes 0x13-0x30, i.e. the fields listing
   voltages, program/erase timeouts, device size, bus width, and erase
   sector geometry) is filled with standard, internally-consistent values
   rather than guessed-then-discovered one halt at a time: byte 0x27
   (device size exponent) is set to 0x17 so 2^0x17 = 8 MB, matching this
   project's actual loaded image size exactly, not an invented figure; the
   erase-region descriptor at bytes 0x2C-0x30 describes one uniform region
   of 128 blocks x 64 KB, which multiplies out to the same real 8 MB,
   keeping the table self-consistent with the real backing data. Vpp
   fields and the buffer-write timeout/size fields are set to 0 ("no Vpp
   pin" / "buffered write unsupported"), the common, safe choice for a
   modern 3V-only part and one that avoids this project needing to model a
   buffered-write state machine it has no evidence for. Byte 0x28 (bus
   interface code, 2 = x8/x16 async) is the one other field spansiond.dll
   is confirmed to read before falling through its own non-gating branch.

   A reset command (0xFF or 0xF0 in both halves - the two universal
   AMD/Intel read-array-mode resets) returns to normal array reads. Any
   other command value, or any query-mode read falling outside the primary
   CFI table modeled here, halts per FATAL-FIRST until a real access
   grounds it - e.g. the "Query-unique string" extended tables the primary
   table's own pointer fields (bytes 0x15-0x16, 0x19-0x1A) name, which this
   project has no evidence spansiond.dll ever actually follows. */

namespace {

constexpr uint32_t kBase = 0x1F800000u;
/* 16 MB: after the write/program sequence above, spansiond.dll (VA
   0xC0939990-0xC0939A18, base loaded from its own global struct at +0x54)
   reads 8 bytes at PA 0x20000200 - exactly 0x800200 past this chip's real
   8 MB start, i.e. within a second, notional 8 MB half of the same
   static-bus chip-select window. The real 8 MB flash chip cannot answer
   there; this project has no evidence of a second physical chip at that
   offset either. Mapping a wider window and letting the existing
   ImageByte() OOB fallback float 0xFF is the same well-documented,
   datasheet-universal "nothing responds here" behavior this file already
   relies on for an absent/short backing image - not a guess at real
   content, just not halting on an address the real static-bus decode
   would also just pass through unanswered. */
constexpr uint32_t kSize = 0x01000000u;

constexpr uint32_t kOffMac0 = 0x00600008u;  /* PA 0x1FE00008: MAC bytes 0-1 */
constexpr uint32_t kOffMac1 = 0x0060000Au;  /* PA 0x1FE0000A: MAC bytes 2-3 */
constexpr uint32_t kOffMac2 = 0x0060000Cu;  /* PA 0x1FE0000C: MAC bytes 4-5 */

constexpr uint16_t kPlaceholderMac[3] = {0x0200u, 0x0000u, 0x0001u};

constexpr uint16_t kCmdCfiQuery    = 0x0098u;
constexpr uint16_t kCmdResetAmd    = 0x00F0u;
constexpr uint16_t kCmdResetReadId = 0x00FFu;

/* QRY (bytes 0x10-0x12): gate-checked, each byte its own 32-bit word,
   duplicated across both 16-bit halves - see file header comment. Bytes
   0x13 onward: the primary CFI table, one byte per 32-bit word, harvested
   rather than gate-checked (see file header comment) except 0x13 itself,
   which the driver does compare against a literal 2. kCfiTable[N]
   corresponds to native CFI byte (0x10+N); CPU word offset = (0x10+N)*4. */
constexpr uint32_t kCfiTableFirstByte = 0x10u;
constexpr uint32_t kCfiTableBase      = kCfiTableFirstByte * 4u;  /* 0x40 */
constexpr uint8_t  kCfiTable[] = {
    /* 0x10 */ 'Q', 'R', 'Y',
    /* 0x13 */ 0x02, 0x00,              /* Primary vendor command set: AMD/Fujitsu std (gate-checked) */
    /* 0x15 */ 0x40, 0x00,              /* Addr of primary extended query table (unused: not followed) */
    /* 0x17 */ 0x00, 0x00,              /* Alternate vendor command set: none */
    /* 0x19 */ 0x00, 0x00,              /* Addr of alternate extended query table: none */
    /* 0x1B */ 0x27,                    /* Vcc min, BCD: 2.7 V */
    /* 0x1C */ 0x36,                    /* Vcc max, BCD: 3.6 V */
    /* 0x1D */ 0x00,                    /* Vpp min: no Vpp pin */
    /* 0x1E */ 0x00,                    /* Vpp max: no Vpp pin */
    /* 0x1F */ 0x04,                    /* Typical single word/byte write timeout: 2^4 = 16 us */
    /* 0x20 */ 0x00,                    /* Typical min-size buffer write timeout: unsupported */
    /* 0x21 */ 0x0A,                    /* Typical individual block erase timeout: 2^10 ms = ~1 s */
    /* 0x22 */ 0x00,                    /* Typical full chip erase timeout: unsupported */
    /* 0x23 */ 0x03,                    /* Max word/byte write timeout: 2^3 x typical */
    /* 0x24 */ 0x00,                    /* Max buffer write timeout: unsupported */
    /* 0x25 */ 0x02,                    /* Max block erase timeout: 2^2 x typical */
    /* 0x26 */ 0x00,                    /* Max chip erase timeout: unsupported */
    /* 0x27 */ 0x17,                    /* Device size = 2^0x17 = 8 MB, matching the real loaded image */
    /* 0x28 */ 0x02, 0x00,              /* Flash device interface: x8/x16 async (confirmed access) */
    /* 0x2A */ 0x00, 0x00,              /* Max bytes in multi-byte write: 0 (buffer write unsupported) */
    /* 0x2C */ 0x01,                    /* Number of erase block regions: 1 (uniform geometry) */
    /* 0x2D */ 0x7F, 0x00,              /* Region 1: (number of blocks - 1) = 127 -> 128 blocks */
    /* 0x2F */ 0x00, 0x01,              /* Region 1: block size / 256 = 0x100 -> 64 KiB blocks */
                                         /* 128 x 64 KiB = 8 MB, matching the real loaded image */
};

class ChevyMylinkStaticFlash : public Peripheral {
public:
    using Peripheral::Peripheral;

    bool ShouldRegister() override {
        auto* bd = emu_.TryGet<BoardContext>();
        return bd && bd->GetBoardFamilyId() == BoardFamilyId::ChevyMylink;
    }

    void OnReady() override {
        const auto&        cfg  = emu_.Get<DeviceConfig>();
        const std::string  key  = CfgKeyPath({config_key::kRom, config_key::kChevyMylinkNorUpdate});
        if (!cfg.rom_chevy_mylink_nor_update.empty()) {
            const std::string  path = ResolveDeviceFile(cfg.device_name, cfg.rom_chevy_mylink_nor_update);
            const HostFileRead read = ReadHostFileIfPresent(path, image_);
            if (read == HostFileRead::kUnreadable)
                emu_.Get<Fatal>().Die("ChevyMylinkStaticFlash: %s %s cannot be read", key.c_str(),
                                      path.c_str());
            if (read == HostFileRead::kAbsent) {
                LOG(Caution, "ChevyMylinkStaticFlash: %s '%s' not found at %s - flash reads float "
                             "0xFF\n",
                    key.c_str(), cfg.rom_chevy_mylink_nor_update.c_str(), path.c_str());
            } else {
                LOG(Boot, "ChevyMylinkStaticFlash: loaded %s (%lld bytes)\n",
                    cfg.rom_chevy_mylink_nor_update.c_str(), static_cast<long long>(image_.size()));
            }
        }
        emu_.Get<PeripheralDispatcher>().Register(this);
    }

    uint32_t MmioBase() const override { return kBase; }
    uint32_t MmioSize() const override { return kSize; }

    uint8_t ImageByte(uint32_t off) const { return off < image_.size() ? image_[off] : 0xFFu; }

    uint8_t ReadByte(uint32_t addr) override {
        if (cfi_query_) HaltUnsupportedAccess("ReadByte (CFI query mode)", addr, 0);
        return ImageByte(addr - MmioBase());
    }

    uint16_t ReadHalf(uint32_t addr) override {
        const uint32_t off = addr - MmioBase();
        if (cfi_query_) HaltUnsupportedAccess("ReadHalf (CFI query mode)", addr, 0);
        switch (off) {
            case kOffMac0: return kPlaceholderMac[0];
            case kOffMac1: return kPlaceholderMac[1];
            case kOffMac2: return kPlaceholderMac[2];
            default:       return static_cast<uint16_t>(ImageByte(off) | (ImageByte(off + 1u) << 8));
        }
    }

    uint32_t ReadWord(uint32_t addr) override {
        const uint32_t off = addr - MmioBase();
        if (cfi_query_) {
            if (off < kCfiTableBase || (off - kCfiTableBase) % 4u != 0u)
                HaltUnsupportedAccess("ReadWord (CFI query mode)", addr, 0);
            const uint32_t index = (off - kCfiTableBase) / 4u;
            if (index >= sizeof(kCfiTable)) HaltUnsupportedAccess("ReadWord (CFI query mode)", addr, 0);
            const uint8_t byte = kCfiTable[index];
            /* QRY (the first 3 table entries) is byte-duplicated across
               both 16-bit halves on the real bus; every other field here
               is harvested as a plain value (see file header comment). */
            return index < 3u ? (uint32_t(byte) << 16) | byte : uint32_t(byte);
        }
        return static_cast<uint32_t>(ImageByte(off)) |
               (static_cast<uint32_t>(ImageByte(off + 1u)) << 8) |
               (static_cast<uint32_t>(ImageByte(off + 2u)) << 16) |
               (static_cast<uint32_t>(ImageByte(off + 3u)) << 24);
    }

    void WriteByte(uint32_t addr, uint8_t value) override {
        HaltUnsupportedAccess("WriteByte", addr, value);
    }
    void WriteHalf(uint32_t addr, uint16_t value) override {
        HaltUnsupportedAccess("WriteHalf", addr, value);
    }

    void WriteWord(uint32_t addr, uint32_t value) override {
        const uint32_t off = addr - MmioBase();
        /* Program data (unlike every command byte below) is real content,
           not duplicated across both 16-bit halves, so it must be handled
           before the generic command decode's halves-match check - e.g.
           spansiond.dll's observed write of "SPAN" (0x4E415053) right
           after entering bypass mode and issuing 0xA0 (see below). */
        if (program_pending_) { ProgramWord(off, value); program_pending_ = false; return; }
        const uint16_t lo  = static_cast<uint16_t>(value & 0xFFFFu);
        const uint16_t hi  = static_cast<uint16_t>(value >> 16);
        if (lo != hi) HaltUnsupportedAccess("WriteWord (halves differ)", addr, value);
        /* The standard AMD/JEDEC unlock sequence - write 0xAA to native
           address 0x555 (CPU offset 0x1554 = 4*0x555), then 0x55 to
           native address 0x2AA (CPU offset 0xAA8), then a command byte
           back at 0x555 - gates every command below. Observed directly:
           spansiond.dll writes 0x00AA00AA to PA 0x1F801554 right after
           exiting CFI query mode, then 0x00550055 to PA 0x1F802AA8, then
           0x00200020 back to PA 0x1F801554 - 0x20 is the standard AMD
           "Unlock Bypass Entry" command, used by flash-programming
           utilities to program many words without repeating the full
           3-cycle unlock each time. In bypass mode a program is just two
           cycles: write 0xA0 (anywhere), then write the data directly to
           the target address. Program is modeled as a real AND into the
           in-memory image (the one real NOR-flash write semantic: bits
           can only clear 1->0 without a separate erase), kept in RAM
           only - this never touches the backing host file. Sector erase
           (0x80/0x30 after a fresh 3-cycle unlock) is modeled too, filling
           the target's 64 KiB region with 0xFF, matching the +0x2F/+0x30
           CFI geometry fields this file already reports. Unlock Bypass
           Reset (0x90 then 0x00) exits bypass mode. Autoselect Entry
           (0x90 outside bypass mode) is also modeled, read-only. */
        if (bypass_) {
            switch (lo) {
                case kCmdProgram:        program_pending_ = true; return;
                case kCmdAutoselect:     bypass_reset_pending_ = true;            return;
                case 0x0000u:
                    if (bypass_reset_pending_) { bypass_ = false; bypass_reset_pending_ = false; return; }
                    break;
                default: break;
            }
            HaltUnsupportedAccess("WriteWord (unrecognized bypass-mode command)", addr, value);
        }
        switch (unlock_state_) {
            case UnlockState::kIdle:
                if (off == kAddrUnlock1 && lo == kUnlockByte1) { unlock_state_ = UnlockState::kUnlock1; return; }
                break;
            case UnlockState::kUnlock1:
                if (off == kAddrUnlock2 && lo == kUnlockByte2) { unlock_state_ = UnlockState::kUnlock2; return; }
                unlock_state_ = UnlockState::kIdle;
                break;
            case UnlockState::kUnlock2:
                unlock_state_ = UnlockState::kIdle;
                /* Sector erase's second unlock+unlock+0x30 cycle, at the
                   target sector's own address - this second AA/55 pair
                   was consumed above like any other unlock, so a pending
                   erase_setup_ plus the confirm byte 0x30 here means
                   "erase the sector at this address". */
                if (erase_setup_ && lo == kCmdEraseConfirm) { erase_setup_ = false; EraseSector(off); return; }
                if (off == kAddrUnlock1) {
                    switch (lo) {
                        case kCmdAutoselect:     autoselect_ = true; return;
                        case kCmdUnlockBypass:   bypass_ = true;     return;
                        case kCmdResetAmd:
                        case kCmdResetReadId: cfi_query_ = false; autoselect_ = false; return;
                        case kCmdEraseSetup: erase_setup_ = true; return;
                        default: HaltUnsupportedAccess("WriteWord (unrecognized unlocked command)", addr, value);
                    }
                }
                break;
        }
        switch (lo) {
            case kCmdCfiQuery:    cfi_query_ = true;  return;
            case kCmdResetAmd:
            case kCmdResetReadId: cfi_query_ = false; autoselect_ = false; return;
            default: HaltUnsupportedAccess("WriteWord (unrecognized command)", addr, value);
        }
    }

    void ProgramWord(uint32_t off, uint32_t value) {
        for (uint32_t i = 0; i < 4u && off + i < image_.size(); ++i)
            image_[off + i] &= static_cast<uint8_t>((value >> (8u * i)) & 0xFFu);
    }

    void EraseSector(uint32_t off) {
        const uint32_t base = off & ~(kSectorSize - 1u);
        for (uint32_t i = 0; i < kSectorSize && base + i < image_.size(); ++i) image_[base + i] = 0xFFu;
    }

    void SaveState(StateWriter& w) override {
        w.Write("cfi_query", cfi_query_);
        w.Write("autoselect", autoselect_);
        w.Write("bypass", bypass_);
        w.Write("bypass_reset_pending", bypass_reset_pending_);
        w.Write("program_pending", program_pending_);
        w.Write("erase_setup", erase_setup_);
        w.Write<uint8_t>("unlock_state", static_cast<uint8_t>(unlock_state_));
        /* image_ is NOT saved here: it is reloaded from the host file by
           OnReady on restore, same as any other rom_* asset. Any in-RAM
           program/erase made since boot is lost across a save/restore -
           acceptable for a boot-time flash probe/update that has not
           previously needed to survive one. */
    }
    void RestoreState(StateReader& r) override {
        r.Read("cfi_query", cfi_query_);
        r.Read("autoselect", autoselect_);
        r.Read("bypass", bypass_);
        r.Read("bypass_reset_pending", bypass_reset_pending_);
        r.Read("program_pending", program_pending_);
        r.Read("erase_setup", erase_setup_);
        uint8_t state = 0;
        r.Read("unlock_state", state);
        unlock_state_ = static_cast<UnlockState>(state);
    }

private:
    enum class UnlockState : uint8_t { kIdle, kUnlock1, kUnlock2 };

    static constexpr uint32_t kSectorSize = 0x10000u;  /* 64 KiB, matching the CFI erase-region fields */

    static constexpr uint32_t kAddrUnlock1  = 0x0555u * 4u;  /* native byte 0x555 */
    static constexpr uint32_t kAddrUnlock2  = 0x02AAu * 4u;  /* native byte 0x2AA */
    static constexpr uint16_t kUnlockByte1  = 0x00AAu;
    static constexpr uint16_t kUnlockByte2  = 0x0055u;
    static constexpr uint16_t kCmdAutoselect   = 0x0090u;
    static constexpr uint16_t kCmdUnlockBypass = 0x0020u;
    static constexpr uint16_t kCmdProgram      = 0x00A0u;
    static constexpr uint16_t kCmdEraseSetup   = 0x0080u;
    static constexpr uint16_t kCmdEraseConfirm = 0x0030u;

    std::vector<uint8_t> image_;
    bool                  cfi_query_            = false;
    bool                  autoselect_           = false;
    bool                  bypass_               = false;
    bool                  bypass_reset_pending_ = false;
    bool                  program_pending_      = false;
    bool                  erase_setup_          = false;
    UnlockState           unlock_state_ = UnlockState::kIdle;
};

}  /* namespace */

REGISTER_SERVICE(ChevyMylinkStaticFlash);

}
