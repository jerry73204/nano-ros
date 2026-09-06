#!/usr/bin/env bash
# The crate + feature set the PUBLISHED API docs are built from — issue 1110.
#
# ONE spelling, because there are two consumers and they must not drift:
#
#   just book                 builds and deploys these pages (`docs.yml`)
#   just check rustdoc-links  proves they still build, on a lane a PR runs
#
# The gate exists because nothing on a merge-gating path ran rustdoc, and
# rustdoc's broken-link lint is deny-level here. `3941b569a` deleted
# `ClientTrait::is_server_ready` while the paragraph beside it still linked to
# it, and `just book` — plus every docs deploy — stayed red for three days
# without any lane saying so. A second break (`nros-rmw-zenoh` naming a
# `record_alloc_ceilings` that never existed) was sitting behind it, invisible
# because rustdoc stops at the first crate that fails.
#
# The FEATURES are load-bearing and were learned the hard way (2026-08-21):
# without `std,env,macros` rustdoc drops `ExecutorConfigEnvExt::from_env`, the
# alloc-gated `ExecutorNodeRuntime::spin` and `nros::node!` — and every doc
# comment linking those then fails to resolve. A narrower set does not make the
# gate cheaper, it makes it wrong.
#
# Scope is the DEPLOYED set, not the workspace. A workspace-wide pass reports
# ~70 diagnostics in crates nothing publishes (issue 1116): real, worth fixing,
# and not what keeps the book from deploying. Widening this list is a ratchet
# decision, not a drive-by.
NROS_RUSTDOC_FEATURES="rmw-cffi,platform-posix,ros-humble,safety-e2e,std,env,macros"
NROS_RUSTDOC_CRATES=(
    nros
    nros-rmw
    nros-rmw-cffi
    nros-rmw-zenoh
    nros-platform-api
    nros-platform-cffi
)

# `-p a -p b …`, for a caller that wants the cargo spelling.
nros_rustdoc_package_args() {
    local p
    for p in "${NROS_RUSTDOC_CRATES[@]}"; do
        printf '%s\n' "-p" "$p"
    done
}
