//! Shared library backing the `nros` CLI.
//!
//! `nros-cli` owns the canonical user-facing command surface. Codegen
//! implementation details still live in the `cargo_nano_ros` library until
//! that library is renamed or split.

pub mod abi_guard;
/// phase-383 — `nros build`, the colcon-like builder (RFC-0065).
///
/// Named `builder`, not `build`: `packages/cli/.gitignore` carries a `build/`
/// rule, so a source directory with that name is silently untracked by
/// `git add` AND invisible to ripgrep/fd by default. A negation rule would fix
/// the first and not the second.
pub mod builder;
// Issue 0498 — temp + `rename(2)` for any file a concurrent `nros sync` may
// read. Was a private helper in `cmd/ws.rs` whose doc claimed it was the
// discipline "every other sync-owned file uses"; the metadata sidecar was not
// using it and died mid-sweep on a half-written read.
pub mod atomic_file;
pub mod build_output;
// RFC-0087 D2 / phase-420 W2 — the `<build_type>` vocabulary. Old spellings and
// new resolve to one build path here, so a reader taught `nros_cargo` does not
// stop understanding the 157 `ament_cargo` packages that W3 has not rewritten
// yet. The table is cross-checked against `cmake/NanoRosPackageXml.cmake` by
// `check-build-type-spelling`, because three readers of one rule is how the
// rmw parity map came to disagree with the vtable by 25 symbols.
pub mod build_type;
pub mod cmd;
// phase-403 W9 (issue 0965) — the ENTITY inventory: WHICH entities an image
// creates, the half the BOUND inventory (which prices a TYPE) cannot answer.
// Shaped like `rosidl_codegen::bounds` — one data model, three transports —
// rather than as a second inventory mechanism.
pub mod entity_inventory;
/// Issue 0827 — a cargo leaf derives its pool budgets from the metadata probe,
/// through `entity_inventory`'s rules rather than a second copy of them.
pub mod leaf_entity_env;
// Issue 0363 — the freshness predicate, shared verbatim with `build.rs` via
// `include!`. One implementation: the build embeds a stamp, the runtime
// recomputes it. Replaces the mtime comparison that fired on every rebase.
pub mod interface_package;
pub mod source_stamp;
pub mod stale_guard;
// Issue 0455 — one `scratch_dir` for every unit test in this crate. Nine
// hand-written `temp_dir().join(...)` spellings raced each other; the
// differences between them WERE the bug. Test-only, so it ships nothing.
#[cfg(test)]
mod test_support;
// Phase 219.A — Entry-pkg codegen (`nros codegen entry`). The shared
// pkg-index walk + launch.xml parser also live here so the cmake-fn
// path (`nano_ros_entry(LAUNCH …)`), the Rust proc-macro
// (via `nros-build`'s re-export), and any future C/C++ tooling all
// dispatch through one implementation.
pub mod codegen;
// phase-262 W2 — `launch_parser` extracted to the `nros-launch-parser` leaf crate
// (depends only on nros-pkg-index, not the launch-manifest submodule). Re-exported
// so `nros_cli_core::launch_parser::*` consumers are unchanged.
pub use nros_launch_parser as launch_parser;
pub mod orchestration;
// phase-262 W1 — `pkg_index` extracted to the `nros-pkg-index` leaf crate so the
// nros-macros proc-macro path doesn't pull all of nros-cli-core. Re-exported here
// so every `nros_cli_core::pkg_index::*` consumer (+ `crate::pkg_index` internal
// refs, e.g. launch_parser) is unchanged.
pub use nros_pkg_index as pkg_index;

use eyre::Result;

/// Top-level dispatcher entry point — every binary front-end lands here.
///
/// `argv` is the post-clap parsed command structure. Each variant maps
/// 1:1 to a `nros <verb>` invocation.
/// Issue 0363 B — the stable name used by the stale-guard's allow-list.
///
/// Matched on the ENUM VARIANT, not a user-typed string: a renamed CLI verb
/// then cannot silently fall out of the guarded set, which is the drift that
/// makes allow-lists rot.
fn cmd_name(cmd: &cmd::Cmd) -> &'static str {
    match cmd {
        cmd::Cmd::Sync(_) => "sync",
        cmd::Cmd::Plan(_) => "plan",
        cmd::Cmd::Ws(_) => "ws",
        cmd::Cmd::Codegen(_) => "codegen",
        cmd::Cmd::CodegenSystem(_) => "codegen-system",
        cmd::Cmd::ModelPath(_) => "model-path",
        cmd::Cmd::SdkPath(_) => "sdk-path",
        cmd::Cmd::SdkFront(_) => "sdk-front",
        cmd::Cmd::GenerateRust(_) => "generate-rust",
        cmd::Cmd::Setup(_) => "setup",
        // Everything else stays runnable on a stale binary ON PURPOSE —
        // `doctor` above all, since diagnosing a broken checkout is exactly
        // when you have one.
        _ => "unguarded",
    }
}

pub fn run(cmd: cmd::Cmd) -> Result<()> {
    // Issue 0363 B — self-check BEFORE dispatch. The equivalent shell guard in
    // `scripts/build/cargo.sh` only covers callers that go through `just`;
    // `activate.sh` puts this binary straight on PATH, so `nros sync` — the
    // documented recovery — bypassed it entirely.
    if let Err(msg) = stale_guard::refuse_if_stale(cmd_name(&cmd)) {
        return Err(eyre::eyre!("{msg}"));
    }
    match cmd {
        cmd::Cmd::Build(args) => cmd::build::run(args),
        cmd::Cmd::Materialize(args) => cmd::materialize::run(args),
        cmd::Cmd::ImageFacts(args) => cmd::image_facts::run(args),
        cmd::Cmd::New(args) => cmd::new::run(args),
        cmd::Cmd::Generate(args) => cmd::generate::run(args),
        cmd::Cmd::GenerateRust(args) => cmd::generate::run_rust(args),
        cmd::Cmd::CodegenFingerprint => {
            println!("{}", rosidl_codegen::codegen_fingerprint());
            Ok(())
        }
        cmd::Cmd::SourceStamp => match stale_guard::stamp_pair() {
            None => {
                println!("source-stamp: n/a (not a per-checkout binary, or built outside git)");
                Ok(())
            }
            Some((built, current)) if built == current => {
                println!("source-stamp: fresh ({built})");
                Ok(())
            }
            Some((built, current)) => Err(eyre::eyre!(
                "source-stamp: STALE — built from {built}, sources are now {current}.\n\
                 Rebuild: ./scripts/bootstrap.sh   (contributors: just setup-cli)"
            )),
        },
        cmd::Cmd::Sync(args) => cmd::ws::run_sync(args),
        cmd::Cmd::GeneratePx4Msgs(args) => cmd::generate_px4::run(args),
        cmd::Cmd::Codegen(args) => cmd::codegen::run(args),
        cmd::Cmd::CodegenSystem(args) => cmd::codegen_system::run(args),
        cmd::Cmd::ModelPath(args) => cmd::model_path::run(args),
        cmd::Cmd::SdkPath(args) => cmd::sdk_path::run(args),
        cmd::Cmd::SdkFront(args) => cmd::sdk_front::run(args),
        cmd::Cmd::Profile(args) => cmd::profile::run(args),
        cmd::Cmd::Metadata(args) => cmd::metadata::run(args),
        cmd::Cmd::Plan(args) => cmd::plan::run(args),
        cmd::Cmd::Check(args) => cmd::check::run(args),
        cmd::Cmd::Explain(args) => cmd::explain::run(args),
        cmd::Cmd::Config(args) => cmd::config::run(args),
        cmd::Cmd::Setup(args) => cmd::setup::run(args),
        cmd::Cmd::Init(args) => cmd::init::run(args),
        cmd::Cmd::Doctor(args) => cmd::doctor::run(args),
        cmd::Cmd::Board(args) => cmd::board::run(args),
        cmd::Cmd::Ws(args) => cmd::ws::run(args),
        cmd::Cmd::Version => cmd::version::run(),
        cmd::Cmd::Completions(args) => cmd::completions::run(args),
        #[cfg(feature = "release")]
        cmd::Cmd::Release(args) => cmd::release::run(args),
    }
}
