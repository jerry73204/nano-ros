//! Subcommand dispatch surface.
//!
//! Each verb lives in its own submodule and exposes:
//!   * a clap `Args` struct (when the verb takes options)
//!   * a `run(args) -> Result<()>` function
//!
//! `Cmd` is the clap-derived enum the binary front-ends parse into;
//! [`crate::run`] dispatches it.

use clap::Subcommand;

pub mod board;
pub mod board_facts;
pub mod bringup;
pub mod build;
pub mod check;
pub mod check_workspace;
pub mod codegen;
pub mod codegen_cyclonedds_descriptors;
pub mod codegen_system;
pub mod completions;
pub mod config;
pub mod doctor;
pub mod emit_package_xml;
pub mod entity_facts;
pub mod entity_inventory;
pub mod explain;
pub mod generate;
pub mod generate_px4;
pub mod image_facts;
pub mod init;
pub mod materialize;
pub mod metadata;
pub mod model_path;
pub mod new;
pub mod new_entry;
pub mod new_node;
pub mod new_platform;
pub mod new_system;
pub mod plan;
pub mod profile;
pub mod scaffold_deploy;
pub mod sdk_front;
pub mod sdk_path;
pub mod setup;
pub mod version;
pub mod ws;

#[cfg(feature = "release")]
pub mod release;

#[derive(Debug, Subcommand)]
pub enum Cmd {
    /// Build a nano-ros workspace: discover packages, resolve the image,
    /// preflight, generate the root, and hand off to the native tool
    /// (RFC-0065 / phase-383). The final stage is an `exec`, so compiler
    /// diagnostics are byte-identical to running cargo/cmake/west directly.
    Build(build::Args),

    /// Take ownership of a generated entry package (RFC-0065 D5). The last
    /// resort: `panic` and `profile` are declarations on the image and need no
    /// materialising. One way — `nros build` stops regenerating it.
    Materialize(materialize::Args),

    /// Scaffold a new nano-ros project (talker / listener / service / action)
    New(new::Args),

    /// Generate Rust / C / C++ message bindings from `package.xml`
    Generate(generate::Args),

    /// Generate Rust message bindings from `package.xml`
    #[command(name = "generate-rust")]
    GenerateRust(generate::RustArgs),

    /// Print this binary's CODEGEN FINGERPRINT — a hash of the bytes its
    /// emitters produce for a compiled-in probe corpus (RFC-0061 / phase-318 W1).
    ///
    /// The fixture-freshness signature keys on this instead of the binary's own
    /// hash, so a rebuild that changes no emitted byte invalidates no fixture.
    /// Hidden: it is a build-system seam, not a user verb.
    #[command(name = "codegen-fingerprint", hide = true)]
    CodegenFingerprint,

    /// Report whether this binary matches the CLI sources in its checkout
    /// (issue 0363). Exits non-zero when stale.
    ///
    /// Hidden for the same reason as `codegen-fingerprint`: a build-system
    /// seam. It exists so `scripts/check-cli-fresh.sh` can ASK the binary
    /// rather than reimplement the predicate — the freshness logic lives in
    /// exactly one place, and this is its read-only door.
    ///
    /// Deliberately NOT in the guarded set: a stale binary must still be able
    /// to report that it is stale.
    #[command(name = "source-stamp", hide = true)]
    SourceStamp,

    /// Sync generated msg bindings + the `[patch.crates-io]` config to the
    /// declared deps (`package.xml` / `Cargo.toml`) — for a standalone pkg
    /// or a workspace (picks single-pkg vs colcon mode by layout). Writes the
    /// patch into each Rust consumer's `.cargo/config.toml` (never `Cargo.toml`).
    /// Pre-cargo step; run after editing `*.msg` files. Phase-265 W5 — promoted
    /// from the retired `ws`-scoped `sync` alias. (`nros generate-rust` stays the
    /// codegen-only primitive.)
    Sync(ws::SyncArgs),

    /// Generate CDR `px4_msgs::msg::*` from a PX4-Autopilot `.msg` tree (no
    /// ament dep) for the XRCE companion path (Phase 233 / RFC-0039 Track B).
    #[command(name = "generate-px4-msgs")]
    GeneratePx4Msgs(generate_px4::Args),

    /// Build-tool C/C++ binding generation (`--args-file` / `resolve-deps`).
    /// The interface the cmake / build.rs consumers speak (Phase 195.A — folds
    /// in the former standalone `nros-codegen` binary).
    Codegen(codegen::Args),

    /// Host-time system bake (Phase 212.E) — read `<bringup>/system.toml` +
    /// `<bringup>/launch/system.launch.xml` and emit the baked compile-time
    /// C config + component registration glue consumed by every embedded
    /// RTOS adapter.
    #[command(name = "codegen-system")]
    CodegenSystem(codegen_system::Args),

    /// RFC-0085 D2 — print the RESOLVED image as data, so a west build derives
    /// its cargo invocation from `[image.*]` instead of re-spelling it from
    /// Kconfig. A query, not a build: it cannot recurse into `west build`.
    #[command(name = "image-facts")]
    ImageFacts(image_facts::Args),

    /// phase-330 W7 — print the resolved SystemModel path for
    /// (bringup, launch, args). The cmake bridge for `nano_ros_entry(LAUNCH …)`.
    #[command(name = "model-path")]
    ModelPath(model_path::Args),

    /// phase-365 W2 — print the CONSTRUCTED install path of a provisioned tool.
    /// The cmake/just/shell bridge to `sdk_store::tool_dir`, so the store is
    /// never searched (issue 0625).
    #[command(name = "sdk-path")]
    SdkPath(sdk_path::Args),

    /// phase-431 W3 — point `$NROS_HOME/bin/<name>` at the NEWEST installed
    /// version of a tool. The bridge for `scripts/bootstrap.sh`, which unpacks
    /// the CLI itself and has no `nros setup` to run until it has.
    #[command(name = "sdk-front")]
    SdkFront(sdk_front::Args),

    /// phase-336 — the cargo build-profile table (CMAKE_BUILD_TYPE mapping,
    /// flags, artifact dir, env-injected definitions). The bridge cmake/bash
    /// use so the derivations are not re-spelled per language.
    #[command(subcommand_negates_reqs = true)]
    Profile(profile::Args),

    /// Collect component source metadata for orchestration planning
    Metadata(metadata::Args),

    /// Resolve launch files, manifests, and metadata into nros-plan.json
    Plan(plan::Args),

    /// Validate a generated nros-plan.json
    Check(check::Args),

    /// Render a generated nros-plan.json in human-readable form
    Explain(explain::Args),

    /// Inspect or validate the current project's resolved configuration
    #[command(subcommand)]
    Config(config::Args),

    /// Resolve + fetch a board's toolchain/SDK packages (Phase 187)
    Setup(setup::Args),

    /// Generate a project `CMakePresets.json` including the per-board presets
    /// `nros setup` wrote (RFC-0048 §6). Then `cmake --preset <board>` works.
    Init(init::Args),

    /// Health-check the workspace (SDK paths, toolchains, env)
    Doctor(doctor::Args),

    /// Inspect supported boards
    #[command(subcommand)]
    Board(board::Args),

    /// Workspace-level msg-pkg utilities (Phase 210.B.3 + 210.D.1 — env, list,
    /// status, clean, doctor). `sync` was promoted to the top-level `nros sync`.
    Ws(ws::Args),

    /// Print toolchain + library versions
    Version,

    /// Generate shell completions (bash | zsh | fish | powershell)
    Completions(completions::Args),

    /// Maintainer-only release subcommands (hidden unless built with
    /// `--features release`)
    #[cfg(feature = "release")]
    #[command(subcommand)]
    Release(release::Args),
}

// #186 — the hidden `nros migrate workspace` one-shot (Phase 212.I) is
// retired: its emitter never adopted the post-212.I component sub-table
// spec and pre-212 workspaces have aged out. A tree that still needs the
// migration runs it from the `nros-v0.5.0` tag's CLI.
