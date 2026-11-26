#include "scripted_debugger.h"

#include <cerfcore/core/cerf_emulator.h>
#include <cerfcore/core/config_json.h>
#include <cerfcore/core/log.h>
#include <cerfengine/mips/mips_cpu_state.h>
#include "../tracing/trace_manager.h"

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace cerfcore {

/* JSON schema read from CERF_DEBUG_SCRIPT:

   { "breakpoints": [
       { "pc": "0x80112818",       // required, hex or decimal
         "label": "idle save",     // optional, for readability in the log
         "max_hits": 10,           // optional, 0 (default) = unlimited
         "dump_gpr": true,         // optional: log all 32 GPRs
         "dump_cp0": true,         // optional: log cp0_status/cause/epc/compare/badvaddr
         "dump_mem": [{"addr": "0x8400A014", "len": 64, "label": "tick struct"}],
         "dump_str": ["a0"],       // read these GPRs as UTF-16LE string pointers
         "dump_astr": ["a0"],      // read these GPRs as ASCII string pointers
         "dump_stack": 64,         // dump this many bytes from $sp
         "find_call_sites": {      // scan the stack for a saved return address
           "targets": ["0x80108cd0"],  // whose call site is "jal <target>"
           "scan_start": 0,            // offset from $sp to start the scan
           "scan_len": 512 }      // how many bytes of stack to scan
         },
         "pause": true           // block the guest CPU here, see below
       }
   ],
   "trace_ranges": [
     { "start": "0x80105000", "end": "0x80107000", // [start, end) VA window
       "label": "initclock-search",
       "max_events": 300 }        // distinct PCs to log before going quiet
   ] }

   trace_ranges logs the opcode word and $ra of every distinct guest PC
   that falls in [start, end) - dev-mode only (CERF_DEV_MODE, via
   TraceManager::OnRunLoopIter, which only fires at JIT block-entry
   points, not every instruction - still useful since a branch target,
   such as each case of a comparison chain, starts a new block). Meant
   for finding an indirectly-called function's body or a comparison's
   constants when the address isn't known up front and find_call_sites
   can't locate it (e.g. the call site uses jalr through a register,
   not a fixed jal).

   This directly replaces the hand-written TraceManager::OnPc() lambdas
   this project's own board-specific nk_trace.cpp files have used for
   one-off reverse-engineering (see e.g. chevy_mylink's): those need a
   rebuild for every new probe address or dump; this needs only a JSON
   edit and a re-run. GPR indices accept either a MIPS ABI name (a0, sp,
   ra, ...) or a bare number (0-31).

   "pause": true turns a breakpoint into a real stop: on hit, after
   logging whatever the breakpoint above it asked for, the guest CPU
   thread blocks and polls two sidecar files next to CERF_DEBUG_SCRIPT
   (<script>.state.json and <script>.cmd.json) every 50ms, instead of
   a socket or FIFO - this process's own tool access is file read/write,
   a FIFO's open() blocking until both ends connect is a real deadlock
   risk against a human-paced command sender, and plain files behave
   the same on the Windows builds this project also targets. The state
   file is overwritten on every poll with the current GPRs and the last
   command's result:
     { "paused": true, "label": "...", "pc": "0x...", "hit": 3,
       "last_processed_seq": 2, "gpr": {"a0": "0x...", ...},
       "last_result": {"seq": 2, "action": "read_mem", ...} }
   Send a command by writing the .cmd.json file with a seq higher than
   last_processed_seq (stale or repeated seqs are ignored, so re-reads
   of a half-written file are harmless):
     {"seq": 1, "action": "continue"}
     {"seq": 1, "action": "read_mem",  "addr": "0xB0900060", "len": 4}
     {"seq": 1, "action": "write_mem", "addr": "0xB0900060", "value": "0x2A"}
   read_mem/write_mem reach RAM first, then MMIO peripheral registers
   on a miss (same side-effect caveat as dump_mem/ReadMmioWord). Only
   "continue" resumes the guest; any other action's result lands in the
   next state-file poll, keeping the CPU blocked. */

namespace {

constexpr const char* kGprNames[32] = {
    "zero", "at", "v0", "v1", "a0", "a1", "a2", "a3",
    "t0",   "t1", "t2", "t3", "t4", "t5", "t6", "t7",
    "s0",   "s1", "s2", "s3", "s4", "s5", "s6", "s7",
    "t8",   "t9", "k0", "k1", "gp", "sp", "fp", "ra",
};

int RegIndex(const std::string& name) {
    static const std::unordered_map<std::string, int> kByName = {
        {"zero", 0},  {"at", 1},  {"v0", 2},  {"v1", 3},  {"a0", 4},  {"a1", 5},
        {"a2", 6},    {"a3", 7},  {"t0", 8},  {"t1", 9},  {"t2", 10}, {"t3", 11},
        {"t4", 12},   {"t5", 13}, {"t6", 14}, {"t7", 15}, {"s0", 16}, {"s1", 17},
        {"s2", 18},   {"s3", 19}, {"s4", 20}, {"s5", 21}, {"s6", 22}, {"s7", 23},
        {"t8", 24},   {"t9", 25}, {"k0", 26}, {"k1", 27}, {"gp", 28}, {"sp", 29},
        {"fp", 30},   {"ra", 31},
    };
    auto it = kByName.find(name);
    if (it != kByName.end()) return it->second;
    try {
        return std::stoi(name, nullptr, 0);
    } catch (...) {
        return -1;
    }
}

uint32_t ParseAddr(const nlohmann::json& j, const char* key, uint32_t fallback = 0) {
    if (!j.contains(key)) return fallback;
    const auto& v = j.at(key);
    if (v.is_string()) return static_cast<uint32_t>(std::stoul(v.get<std::string>(), nullptr, 0));
    if (v.is_number_integer() || v.is_number_unsigned()) return v.get<uint32_t>();
    return fallback;
}

std::string ReadWideString(const TraceContext& ctx, uint32_t addr, size_t max_len = 200) {
    std::string out;
    for (size_t i = 0; i < max_len; ++i) {
        auto ch = ctx.ReadVa16(addr + static_cast<uint32_t>(i) * 2u);
        if (!ch || *ch == 0) break;
        out.push_back(static_cast<char>(*ch & 0xFFu));
    }
    return out;
}

std::string ReadAsciiString(const TraceContext& ctx, uint32_t addr, size_t max_len = 200) {
    std::string out;
    for (size_t i = 0; i < max_len; ++i) {
        auto ch = ctx.ReadVa8(addr + static_cast<uint32_t>(i));
        if (!ch || *ch == 0) break;
        out.push_back(static_cast<char>(*ch));
    }
    return out;
}

struct DumpMemEntry {
    uint32_t    addr = 0;
    uint32_t    len  = 16;
    std::string label;
};

struct FindCallSites {
    std::vector<uint32_t> targets;
    uint32_t               scan_start = 0;
    uint32_t               scan_len   = 256;
};

struct Breakpoint {
    uint32_t                   pc        = 0;
    std::string                label;
    uint64_t                   max_hits  = 0;  /* 0 = unlimited */
    uint64_t                   hits      = 0;
    bool                       dump_gpr  = false;
    bool                       dump_cp0  = false;
    std::vector<DumpMemEntry>  dump_mem;
    std::vector<int>           dump_str_regs;
    std::vector<int>           dump_astr_regs;
    uint32_t                   dump_stack_len = 0;
    FindCallSites               find_calls;
    bool                       pause     = false;
    std::string                script_path;  /* set from CERF_DEBUG_SCRIPT, for the .state.json/.cmd.json sidecar files */
};

#if CERF_DEV_MODE
struct TraceRange {
    uint32_t                     start = 0;
    uint32_t                     end   = 0;
    std::string                  label;
    uint32_t                     max_events = 300;
    uint32_t                     events     = 0;
    std::unordered_map<uint32_t, bool> seen;
};

void FireTraceRange(const TraceContext& ctx, const std::shared_ptr<TraceRange>& tr) {
    if (ctx.pc < tr->start || ctx.pc >= tr->end) return;
    if (tr->seen.count(ctx.pc)) return;
    if (tr->events >= tr->max_events) return;
    tr->seen[ctx.pc] = true;
    ++tr->events;
    auto word = ctx.ReadVa32(ctx.pc);
    LOG(Trace, "scripted_debugger: TRACE '%s' pc=0x%08X opcode=0x%08X ra=0x%08X\n",
        tr->label.empty() ? "?" : tr->label.c_str(), ctx.pc, word.value_or(0u),
        ctx.mips ? static_cast<uint32_t>(ctx.mips->gpr[31]) : 0u);
}
#endif

std::string Hex(uint32_t v) {
    char buf[16];
    std::snprintf(buf, sizeof(buf), "0x%08X", v);
    return buf;
}

struct PauseCommand {
    uint32_t    seq = 0;
    std::string action;
    uint32_t    addr  = 0;
    uint32_t    value = 0;
};

bool ReadPauseCommand(const std::string& path, PauseCommand& out) {
    std::string text;
    if (CfgReadTextFile(path, text) != HostFileRead::kRead) return false;
    std::string error;
    auto parsed = CfgParseJsonObject<nlohmann::json>(text, error);
    if (!parsed) return false;
    const nlohmann::json& j = *parsed;
    out.seq    = j.value("seq", static_cast<uint32_t>(0));
    out.action = j.value("action", std::string());
    out.addr   = ParseAddr(j, "addr");
    out.value  = ParseAddr(j, "value");
    return true;
}

void WriteDebugState(const TraceContext& ctx, const std::shared_ptr<Breakpoint>& bp,
                     uint32_t last_processed_seq, const nlohmann::ordered_json& last_result) {
    nlohmann::ordered_json j;
    j["paused"]             = true;
    j["label"]              = bp->label;
    j["pc"]                 = Hex(bp->pc);
    j["hit"]                = bp->hits;
    j["last_processed_seq"] = last_processed_seq;
    if (ctx.mips) {
        nlohmann::ordered_json gpr;
        for (int i = 0; i < 32; ++i) gpr[kGprNames[i]] = Hex(static_cast<uint32_t>(ctx.mips->gpr[i]));
        j["gpr"] = gpr;
        j["cp0"] = {{"status", Hex(ctx.mips->cp0_status)},
                    {"cause", Hex(ctx.mips->cp0_cause)},
                    {"epc", Hex(ctx.mips->cp0_epc)},
                    {"compare", Hex(ctx.mips->cp0_compare)}};
    }
    if (!last_result.is_null()) j["last_result"] = last_result;
    uint32_t error = 0;
    CfgWriteJsonFile(bp->script_path + ".state.json", j, error);
}

/* Blocks the guest CPU thread, polling <script>.cmd.json every 50ms - see
   the "pause" schema comment above the top of this file for the protocol
   and why this is file-based rather than a FIFO or socket. */
void PauseAndWait(const TraceContext& ctx, const std::shared_ptr<Breakpoint>& bp) {
    const std::string cmd_path = bp->script_path + ".cmd.json";
    uint32_t                last_seq = 0;
    nlohmann::ordered_json  last_result;
    LOG(Boot, "scripted_debugger: PAUSED '%s' pc=0x%08X hit #%llu - write commands to %s\n",
        bp->label.empty() ? "?" : bp->label.c_str(), bp->pc,
        static_cast<unsigned long long>(bp->hits), cmd_path.c_str());
    for (;;) {
        WriteDebugState(ctx, bp, last_seq, last_result);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        PauseCommand cmd;
        if (!ReadPauseCommand(cmd_path, cmd)) continue;
        if (cmd.seq == 0 || cmd.seq <= last_seq) continue;
        last_seq = cmd.seq;
        if (cmd.action == "continue") {
            last_result = nullptr;
            break;
        } else if (cmd.action == "read_mem") {
            auto        w   = ctx.ReadVa32(cmd.addr);
            const char* src = "ram";
            if (!w) {
                w   = ctx.ReadMmioWord(cmd.addr);
                src = "mmio";
            }
            last_result = {{"seq", cmd.seq},
                           {"action", "read_mem"},
                           {"addr", Hex(cmd.addr)},
                           {"value", w ? Hex(*w) : std::string("unreadable")},
                           {"source", src}};
        } else if (cmd.action == "write_mem") {
            const bool ok = ctx.WriteGuestWord(cmd.addr, cmd.value);
            last_result   = {{"seq", cmd.seq},
                            {"action", "write_mem"},
                            {"addr", Hex(cmd.addr)},
                            {"value", Hex(cmd.value)},
                            {"ok", ok}};
        } else {
            last_result = {{"seq", cmd.seq}, {"error", "unknown action '" + cmd.action + "'"}};
        }
    }
    LOG(Boot, "scripted_debugger: RESUMED '%s'\n", bp->label.empty() ? "?" : bp->label.c_str());
}

void FireBreakpoint(const TraceContext& ctx, const std::shared_ptr<Breakpoint>& bp) {
    ++bp->hits;
    if (bp->max_hits != 0 && bp->hits > bp->max_hits) return;
    if (!ctx.mips) return;

    LOG(Trace, "scripted_debugger: BP '%s' pc=0x%08X hit #%llu\n",
        bp->label.empty() ? "?" : bp->label.c_str(), bp->pc,
        static_cast<unsigned long long>(bp->hits));

    if (bp->dump_gpr) {
        for (int i = 0; i < 32; ++i) {
            LOG(Trace, "scripted_debugger:   $%-4s = 0x%08X\n", kGprNames[i],
                static_cast<uint32_t>(ctx.mips->gpr[i]));
        }
    }

    if (bp->dump_cp0) {
        LOG(Trace, "scripted_debugger:   cp0_status  = 0x%08X\n", ctx.mips->cp0_status);
        LOG(Trace, "scripted_debugger:   cp0_cause   = 0x%08X\n", ctx.mips->cp0_cause);
        LOG(Trace, "scripted_debugger:   cp0_epc     = 0x%08X\n", ctx.mips->cp0_epc);
        LOG(Trace, "scripted_debugger:   cp0_compare = 0x%08X\n", ctx.mips->cp0_compare);
        LOG(Trace, "scripted_debugger:   cp0_badvaddr= 0x%08X\n", ctx.mips->cp0_badvaddr);
    }

    for (const auto& m : bp->dump_mem) {
        LOG(Trace, "scripted_debugger:   mem '%s' @0x%08X:\n", m.label.c_str(), m.addr);
        for (uint32_t off = 0; off < m.len; off += 4u) {
            /* RAM/ROM first (no side effects); MMIO peripheral registers
               only if that misses, since reading one can have side effects
               (clearing a status bit, popping a FIFO, ...) same as a real
               guest read would. */
            auto w = ctx.ReadVa32(m.addr + off);
            const char* source = "ram";
            if (!w) {
                w      = ctx.ReadMmioWord(m.addr + off);
                source = "mmio";
            }
            if (w) {
                LOG(Trace, "scripted_debugger:     [0x%08X] = 0x%08X (%s)\n", m.addr + off, *w,
                    source);
            } else {
                LOG(Trace, "scripted_debugger:     [0x%08X] = <unreadable>\n", m.addr + off);
            }
        }
    }

    for (int r : bp->dump_str_regs) {
        if (r < 0 || r > 31) continue;
        const uint32_t addr = static_cast<uint32_t>(ctx.mips->gpr[r]);
        LOG(Trace, "scripted_debugger:   str($%s=0x%08X) = \"%s\"\n", kGprNames[r], addr,
            ReadWideString(ctx, addr).c_str());
    }

    for (int r : bp->dump_astr_regs) {
        if (r < 0 || r > 31) continue;
        const uint32_t addr = static_cast<uint32_t>(ctx.mips->gpr[r]);
        LOG(Trace, "scripted_debugger:   astr($%s=0x%08X) = \"%s\"\n", kGprNames[r], addr,
            ReadAsciiString(ctx, addr).c_str());
    }

    if (bp->dump_stack_len) {
        const uint32_t sp = static_cast<uint32_t>(ctx.mips->gpr[29]);
        for (uint32_t off = 0; off < bp->dump_stack_len; off += 4u) {
            auto w = ctx.ReadVa32(sp + off);
            LOG(Trace, "scripted_debugger:   stack[sp+0x%02X] = 0x%08X\n", off,
                w.value_or(0xDEADDEADu));
        }
    }

    if (!bp->find_calls.targets.empty()) {
        const uint32_t sp    = static_cast<uint32_t>(ctx.mips->gpr[29]);
        const uint32_t start = sp + bp->find_calls.scan_start;
        for (uint32_t off = 0; off < bp->find_calls.scan_len; off += 4u) {
            auto w = ctx.ReadVa32(start + off);
            if (!w) continue;
            auto at = ctx.ReadVa32(*w - 4u);
            if (!at) continue;
            for (uint32_t target : bp->find_calls.targets) {
                /* jal <target>: opcode 3 in bits 31:26, target>>2 in bits 25:0 -
                   only valid within the same 256MB segment as the call site,
                   same restriction the real instruction has. */
                const uint32_t jal = (3u << 26) | ((target >> 2) & 0x3FFFFFFu);
                if (*at == jal) {
                    LOG(Trace,
                        "scripted_debugger:   CALL SITE for 0x%08X: "
                        "stack[sp+0x%02X]=0x%08X (jal at 0x%08X)\n",
                        target, start + off - sp, *w, *w - 4u);
                }
            }
        }
    }

    if (bp->pause) PauseAndWait(ctx, bp);
}

}  /* namespace */

bool ScriptedDebugger::ShouldRegister() {
    return std::getenv("CERF_DEBUG_SCRIPT") != nullptr;
}

void ScriptedDebugger::OnReady() {
    const char* path = std::getenv("CERF_DEBUG_SCRIPT");
    if (!path) return;

    std::string text;
    if (CfgReadTextFile(path, text) != HostFileRead::kRead) {
        LOG(Caution, "scripted_debugger: CERF_DEBUG_SCRIPT=%s could not be read\n", path);
        return;
    }
    std::string error;
    auto parsed = CfgParseJsonObject<nlohmann::json>(text, error);
    if (!parsed) {
        LOG(Caution, "scripted_debugger: %s %s\n", path, error.c_str());
        return;
    }
    const nlohmann::json& j = *parsed;
    if ((!j.contains("breakpoints") || !j["breakpoints"].is_array()) &&
        (!j.contains("trace_ranges") || !j["trace_ranges"].is_array())) {
        LOG(Caution, "scripted_debugger: %s has no \"breakpoints\" or \"trace_ranges\" array\n",
            path);
        return;
    }

    auto& tm      = emu_.Get<TraceManager>();
    size_t loaded = 0;
    if (j.contains("breakpoints") && j["breakpoints"].is_array()) {
    for (const auto& bpj : j["breakpoints"]) {
        auto bp = std::make_shared<Breakpoint>();
        bp->pc       = ParseAddr(bpj, "pc");
        bp->label    = bpj.value("label", std::string());
        bp->max_hits = bpj.value("max_hits", static_cast<uint64_t>(0));
        bp->dump_gpr = bpj.value("dump_gpr", false);
        bp->dump_cp0 = bpj.value("dump_cp0", false);
        bp->dump_stack_len = bpj.value("dump_stack", static_cast<uint32_t>(0));
        bp->pause        = bpj.value("pause", false);
        bp->script_path  = path;

        if (bpj.contains("dump_mem") && bpj["dump_mem"].is_array()) {
            for (const auto& m : bpj["dump_mem"]) {
                DumpMemEntry e;
                e.addr  = ParseAddr(m, "addr");
                e.len   = m.value("len", static_cast<uint32_t>(16));
                e.label = m.value("label", std::string());
                bp->dump_mem.push_back(e);
            }
        }
        if (bpj.contains("dump_str") && bpj["dump_str"].is_array()) {
            for (const auto& r : bpj["dump_str"]) bp->dump_str_regs.push_back(RegIndex(r.get<std::string>()));
        }
        if (bpj.contains("dump_astr") && bpj["dump_astr"].is_array()) {
            for (const auto& r : bpj["dump_astr"]) {
                bp->dump_astr_regs.push_back(RegIndex(r.get<std::string>()));
            }
        }
        if (bpj.contains("find_call_sites")) {
            const auto& fc = bpj["find_call_sites"];
            if (fc.contains("targets") && fc["targets"].is_array()) {
                for (const auto& t : fc["targets"]) {
                    bp->find_calls.targets.push_back(
                        static_cast<uint32_t>(std::stoul(t.get<std::string>(), nullptr, 0)));
                }
            }
            bp->find_calls.scan_start = fc.value("scan_start", static_cast<uint32_t>(0));
            bp->find_calls.scan_len   = fc.value("scan_len", static_cast<uint32_t>(256));
        }

        const uint32_t pc = bp->pc;
        tm.OnPc(pc, [bp](const TraceContext& ctx) { FireBreakpoint(ctx, bp); });
        ++loaded;
    }
    }

#if CERF_DEV_MODE
    size_t ranges_loaded = 0;
    if (j.contains("trace_ranges") && j["trace_ranges"].is_array()) {
        std::vector<std::shared_ptr<TraceRange>> ranges;
        for (const auto& trj : j["trace_ranges"]) {
            auto tr        = std::make_shared<TraceRange>();
            tr->start      = ParseAddr(trj, "start");
            tr->end        = ParseAddr(trj, "end");
            tr->label      = trj.value("label", std::string());
            tr->max_events = trj.value("max_events", static_cast<uint32_t>(300));
            ranges.push_back(tr);
            ++ranges_loaded;
        }
        if (!ranges.empty()) {
            tm.OnRunLoopIter([ranges](const TraceContext& ctx) {
                for (const auto& tr : ranges) FireTraceRange(ctx, tr);
            });
        }
    }
    LOG(Boot, "scripted_debugger: loaded %zu breakpoint(s), %zu trace range(s) from %s\n", loaded,
        ranges_loaded, path);
#else
    LOG(Boot, "scripted_debugger: loaded %zu breakpoint(s) from %s\n", loaded, path);
#endif
}

REGISTER_SERVICE(ScriptedDebugger);

}
