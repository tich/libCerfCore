#pragma once

#include <cerfcore/core/service.h>

namespace cerfcore {

/* A data-driven, rebuild-free alternative to hand-written OnPc() hooks
   for one-off reverse-engineering sessions (see agent_docs/debugging.md
   for why that loop exists at all). Point CERF_DEBUG_SCRIPT at a JSON
   file of breakpoints before running cerf; each one logs a structured
   dump to cerf.log when the guest PC reaches it, with no code change or
   rebuild needed to add, remove, or reconfigure a breakpoint between
   runs - only to change what the *engine* itself exposes to read.
   Inert (ShouldRegister() false) when the env var is unset, so it costs
   nothing in normal use. See scripted_debugger.cpp's header comment for
   the JSON schema. */
class ScriptedDebugger : public Service {
public:
    using Service::Service;

    bool ShouldRegister() override;
    void OnReady() override;
};

}
