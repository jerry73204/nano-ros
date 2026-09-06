//! Data-driven board profiles (Phase 195.C).
//!
//! The `nros` CLI is shipped from a *separate* repo, so it must carry **no**
//! baked-in knowledge of the nano-ros workspace layout. Every per-board fact
//! — which board crate to depend on, the rustc target, the `.cargo/config.toml`
//! body, the kernel-port / libc paths, the generated entry-point shape — lives
//! in a `nros-board.toml` descriptor *in the workspace* and is read at runtime.
//!
//! Discovery is uniform: every `packages/boards/*/nros-board.toml` is loaded
//! (crate-backed boards put the file in their crate dir; the crate-less host
//! boards — `posix`, `zephyr` — get a descriptor-only dir under
//! `packages/boards/`). A file holds a `[[board]]` array so one crate can back
//! several boards (e.g. `nros-board-nuttx-qemu` → arm virt + rv-virt,
//! differing only by `chip`).
//!
//! Layout paths in `cargo_config` are stored **relative** and written with the
//! `${workspace}` placeholder; the CLI substitutes the workspace root it
//! discovered at render time, so the binary stays workspace-agnostic.

use std::path::{Path, PathBuf};

use serde::Deserialize;

/// Resolved platform identity for a `(board, target)` pair.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum PlatformKind {
    Posix,
    Freertos,
    BareMetal,
    Nuttx,
    Zephyr,
    ThreadxLinux,
    ThreadxRiscv64,
    Esp32,
    Stm32,
    OrinSpe,
}

impl PlatformKind {
    /// The kebab-case token this variant deserializes from — the spelling that
    /// appears as `platform = "…"` in `nros-board.toml`.
    ///
    /// phase-341 W2 needs it because a leaf's `[package.metadata.nros.entry]
    /// deploy` is matched against the descriptor's `platform` (the mapping
    /// `scripts/check-board-cargo-config-applied.sh` already uses). Written as
    /// an exhaustive match rather than a serde round-trip so adding a variant
    /// is a compile error here instead of a silently unmatched board;
    /// `platform_kebab_round_trips` proves the two spellings agree.
    pub fn kebab(self) -> &'static str {
        match self {
            PlatformKind::Posix => "posix",
            PlatformKind::Freertos => "freertos",
            PlatformKind::BareMetal => "bare-metal",
            PlatformKind::Nuttx => "nuttx",
            PlatformKind::Zephyr => "zephyr",
            PlatformKind::ThreadxLinux => "threadx-linux",
            PlatformKind::ThreadxRiscv64 => "threadx-riscv64",
            PlatformKind::Esp32 => "esp32",
            PlatformKind::Stm32 => "stm32",
            PlatformKind::OrinSpe => "orin-spe",
        }
    }
}

/// Rust toolchain a generated package pins.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum Toolchain {
    /// Stable rustc with a prebuilt target — no `rust-toolchain.toml`.
    Stable,
    /// Pinned nightly + `rust-src` for `-Z build-std`.
    Nightly,
    /// Xtensa `+esp` espup toolchain (ESP32-S3).
    Esp,
}

/// External libraries the generated `build.rs` must link.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum LinkKind {
    /// Board crate / cargo handles all linking.
    None,
    /// NuttX staging-archive group-link + dramboot linker script.
    NuttxStaging,
}

/// Shape of the generated package's entry point.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum EntryKind {
    /// Hosted Rust `fn main` (posix / threadx-linux host).
    HostedMain,
    /// `<board>::run(cfg, closure)` on a bare-metal / RTOS target.
    BoardRun,
    /// Rust staticlib consumed by zephyr-lang-rust `rust_cargo_application()`.
    ZephyrStaticlib,
}

// phase-351 W6 — `NetStack` (`rtos-owned` / `nanoros-owned`) is GONE. It was
// parsed and never read from the day it was added, and it answered the wrong
// question: "who brings up NIC+IP", not "which stack", so nothing could act on
// it. `supported_netstacks` (W4) answers the question consumers actually ask
// and IS read — by `resolve_netstack`, by `nros ws board-facts`, and by
// `check-site-config`.

/// The per-board pieces the entry-point renderer interpolates into the shared
/// board-run entry shape. `None` path interpolation here — these reference only
/// the board crate name.
#[derive(Debug, Clone, Deserialize)]
pub struct BoardEntry {
    /// Board rlib invoked as `<crate>::run(<crate>::Config::default(), ..)`.
    ///
    /// DERIVED from `board_crate` when omitted — see
    /// [`BoardDescriptor::apply_conventions`].
    #[serde(default)]
    pub crate_name: String,
    /// Attribute(s) + `fn` signature line(s) preceding the fn body.
    ///
    /// DEFAULTS to the `#[unsafe(no_mangle)] extern "C" fn main() -> !` shape
    /// that 4 of 7 boards state verbatim.
    #[serde(default)]
    pub signature: String,
    /// Crate-root `use`s / items pinned above the entry (panic handler, etc.).
    #[serde(default)]
    pub crate_root_extra: String,
    /// Cargo dependencies the [`Self::crate_root_extra`] items need, as
    /// `name = <spec>` lines in manifest spelling.
    ///
    /// **One fact, one place.** `crate_root_extra = "use panic_semihosting as
    /// _;"` names a crate, and a crate-root `use` of a crate nothing depends on
    /// does not compile. While every entry was hand-written a human supplied
    /// both halves and they could not drift apart; the moment the entry became
    /// GENERATED, the descriptor had one half and no way to express the other,
    /// and `nros build freertos` emitted `use panic_semihosting as _;` into a
    /// manifest with no `panic-semihosting` (phase-383 W9.b, found by migrating
    /// `examples/workspaces/rust`).
    ///
    /// Kept as raw manifest lines rather than a structured type because a
    /// dependency spec is already a small language — `"0.6"`, `{ version =
    /// "~0.18.0", features = [...] }` — and re-modelling it here would be a
    /// second grammar to keep in step with cargo's.
    #[serde(default)]
    pub crate_root_deps: Vec<String>,
    /// Builder-chain suffix appended inside the closure; empty for most boards.
    #[serde(default)]
    pub closure_extra: String,
}

/// Declared board capabilities (RFC-0042 D2 / phase-241 wave C). The single
/// source of truth for what a board provides; the generator (241.C.2) lowers
/// each to the right per-platform mechanism — `-D NROS_PLATFORM_HAS_*` for
/// baremetal/threadx, Kconfig (`prj.conf`) for zephyr, etc. — instead of the
/// per-RTOS-header self-`#define`s + the one hand-set cmake `-D` they replace.
#[derive(Debug, Clone, Copy, Deserialize)]
pub struct BoardCapabilities {
    /// Board has a usable heap allocator. Drives the canonical malloc/free +
    /// `NROS_PLATFORM_HAS_MALLOC` (and `!NROS_NO_DYNAMIC_MEMORY` on bare-metal).
    #[serde(default)]
    pub heap: bool,
    /// Board provides atomic load/store. Drives `NROS_PLATFORM_HAS_ATOMICS`.
    #[serde(default)]
    pub atomics: bool,
    /// Board has threads + a mutex. Drives `NROS_FEATURE_THREADS` /
    /// `NROS_PLATFORM_HAS_MUTEX`.
    #[serde(default)]
    pub threads: bool,
}

impl BoardCapabilities {
    /// Conservative defaults inferred from the platform when a board omits the
    /// `[board.capabilities]` block (migration path; a lint flags reliance on
    /// inference). RTOS + hosted platforms have a heap/threads; generic
    /// bare-metal does not (it must opt in — the #38 lesson). Atomics are
    /// assumed everywhere (every supported target provides them today).
    fn inferred(platform: PlatformKind) -> Self {
        use PlatformKind::*;
        match platform {
            Posix | Freertos | Nuttx | Zephyr | ThreadxLinux | ThreadxRiscv64 | Esp32 => {
                BoardCapabilities {
                    heap: true,
                    atomics: true,
                    threads: true,
                }
            }
            // Generic bare-metal / SPE: no heap by default (opt in via board.toml).
            BareMetal | Stm32 | OrinSpe => BoardCapabilities {
                heap: false,
                atomics: true,
                threads: false,
            },
        }
    }
}

/// One board profile. Mirrors the old hardcoded `PlatformProfile` +
/// `BoardEntry`, but every field is owned data read from `nros-board.toml`.
/// RFC-0049 promised this: "Unknown keys fail loud with the valid-key list
/// (`deny_unknown_fields`, the RFC-0033 precedent)" (RFC-0049 §"Schema"). It
/// shipped without it, so a typo'd board key was silently ignored by this
/// reader AND by `BoardKnobsFile`, which `#[serde(flatten)]`s the remainder
/// into a discarded `_rest`. A board descriptor is a hardware SSoT read by
/// four different consumers; a key nobody reads reads exactly like a key that
/// works.
///
/// `priority_plan` below is the reason this could not simply be switched on:
/// six boards declare `[board.priority_plan]` and no field modelled it, so a
/// naive `deny_unknown_fields` would have rejected every one of them.
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BoardDescriptor {
    /// Board name + accepted aliases (the values a user passes as `board`).
    pub names: Vec<String>,
    /// The board id to hand `west build -b`, when it differs from the name the
    /// image authored.
    ///
    /// `[image.*] board` is BOTH the catalog lookup key and, for a Zephyr
    /// image, the string west receives — so a descriptor answering to
    /// `["my-board", "qemu_cortex_m3"]` and an image authored as
    /// `board = "my-board"` reached `west build -b my-board`, which west has
    /// never heard of. The in-tree descriptors hide it by convention: their
    /// name lists carry the Zephyr spelling and the examples author THAT one.
    ///
    /// A workspace-local board makes the convention hard to keep — the natural
    /// name for a package is the friendly one — so the descriptor can say
    /// which of its names west uses. Absent, the authored string is passed
    /// through, which is what every existing descriptor relies on.
    #[serde(default)]
    pub west_board: Option<String>,
    pub platform: PlatformKind,
    /// rustc target triple this board pins, if any (`None` → take from plan).
    #[serde(default)]
    pub target: Option<String>,
    pub toolchain: Toolchain,
    /// The `nros/<feature>` selected (e.g. `platform-posix`).
    pub platform_feature: String,
    /// Extra local default-feature aliases beyond `nros/<feature>`.
    #[serde(default)]
    pub local_aliases: Vec<String>,
    pub link_kind: LinkKind,
    pub entry_kind: EntryKind,
    /// phase-351 W4 — the network stacks this board can actually be built with,
    /// in preference order; the first is the default when a deploy names none.
    ///
    /// A FACT of the board, not a menu: every vendor has already welded its
    /// choice, and the pairing has a validity domain (NetX Duo ships a port
    /// table of 24 arches against ThreadX's 47, so a ThreadX arch with no NetX
    /// counterpart cannot be paired at all). Empty means "this board makes no
    /// stack choice" — the RTOS or the host owns it and a deploy must not try
    /// to select one.
    #[serde(default)]
    pub supported_netstacks: Vec<String>,
    /// esp-hal / stm32 chip feature; `None` for non-chip platforms.
    #[serde(default)]
    pub chip: Option<String>,
    /// Board crate to depend on; `None` for crate-less host boards
    /// (posix / zephyr) that pull static or `nros-platform-cffi` deps.
    #[serde(default)]
    pub board_crate: Option<String>,
    /// Board-crate path relative to the workspace root; defaults to
    /// `packages/boards/<board_crate>` when omitted.
    #[serde(default)]
    pub crate_path: Option<String>,
    /// Extra features to enable on the board crate dependency.
    #[serde(default)]
    pub board_features: Vec<String>,
    /// Phase 252 — the capability-axis features this board crate forwards to its
    /// backend (e.g. `["safety-e2e"]` → the board's `safety-e2e = ["nros-rmw-zenoh?/safety-e2e"]`).
    /// A declared `[safety]` axis lowers to the board feature only when the board
    /// Verbatim `.cargo/config.toml` body, with `${workspace}` placeholders for
    /// any layout path. `None` for boards that need no config (posix/zephyr/…).
    #[serde(default)]
    pub cargo_config: Option<String>,
    /// Generated entry-point pieces; `None` for hosted boards that emit the
    /// default `fn main` shape.
    #[serde(default)]
    pub entry: Option<BoardEntry>,
    /// Disambiguate two descriptors sharing a `names` entry by requiring this
    /// substring in the requested target (e.g. `"riscv64"` for threadx-riscv64,
    /// so `board = "threadx"` picks riscv64 vs linux by target).
    #[serde(default)]
    pub target_contains: Option<String>,
    /// Declared board capabilities (heap/atomics/threads). `None` → inferred from
    /// `platform` via `capabilities()` during the 241.C migration.
    #[serde(default)]
    pub capabilities: Option<BoardCapabilities>,
    /// CMake cross-compile facts for `nros setup`'s CMakePreset emission
    /// (RFC-0048 §6 / phase-287 W5). `None` for host boards (posix) that need no
    /// toolchain file — their preset carries only `nano_ros_ROOT`.
    #[serde(default)]
    pub cmake: Option<BoardCmake>,
    /// `[board.zephyr]` — what a Zephyr build needs to know about this board.
    ///
    /// RFC-0064 R5 D3. Before this block the same facts had two homes and
    /// neither was the descriptor: `fvp-aemv8r-smp` kept them in a private
    /// `board.cmake` (14 `NROS_BOARD_*` variables), and `qemu_cortex_a53` kept
    /// them nowhere at all — an inline `board = "…"` string in
    /// `examples/fixtures.toml` plus a `.conf` copied into every example leaf
    /// that wanted it.
    #[serde(default)]
    pub zephyr: Option<BoardZephyr>,
    /// `[board.provisioning]` — what `nros setup board` must fetch first.
    ///
    /// RFC-0064 R5 D3. Only meaningful for a board a downstream consumer
    /// provisions into its OWN Zephyr workspace, which is why it is separate
    /// from `[board.zephyr]`: the build facts are needed by every consumer, the
    /// provisioning facts only by one that does not inherit nano-ros's tree.
    #[serde(default)]
    pub provisioning: Option<BoardProvisioning>,
    /// phase-341 W2 — the `nros-board.toml` this descriptor was read from,
    /// workspace-relative. Set by [`BoardCatalog::load`], `None` for
    /// in-memory descriptors. Recorded rather than derived from
    /// `crate_path_rel()`: a generated projection of `cargo_config` names its
    /// SSoT in the DO-NOT-EDIT header, and a header that names the wrong file
    /// sends the next reader to edit the wrong descriptor.
    #[serde(skip)]
    pub source: Option<String>,
    /// `[board.priority_plan]` — ACKNOWLEDGED here, interpreted elsewhere.
    ///
    /// The tier/transport priority plan (RFC-0049; `linux/nros-board.toml`
    /// declares `reserved.transport = [90, 99]` for zenoh-pico's read and
    /// lease tasks) is consumed by `check-tier-priority-plan` and by
    /// `nros-platform-config`, not by this reader. It is modelled as an opaque
    /// value ON PURPOSE: `deny_unknown_fields` above must not mean "every
    /// consumer's schema is restated here", which would make this struct the
    /// union of four readers and guarantee drift. Declaring it says "this key
    /// is real and someone else owns it"; omitting it would have said "typo".
    #[serde(default)]
    pub priority_plan: Option<toml::Value>,
}

/// `[board.zephyr]` — the Zephyr facts, and only the ones no convention gives.
///
/// RFC-0064 R5 D3 + D6. What is DERIVED and therefore absent here:
///
/// * `prj.conf` — the file beside the descriptor, if it exists.
/// * `boards/<west_board with `/` and `@` replaced by `_`>.{conf,overlay}` —
///   the per-board Kconfig fragment and DTS overlay. `board.cmake` stated this
///   rule in a comment and then hand-wrote both paths anyway.
/// * `zephyr-rust-support/` — the Kconfig module that enables `RUST_SUPPORTED`
///   for this board's arch. Present-if-exists, and its whole body is
///   `default y if BOARD_<UPPER(first segment of west_board)>`, so
///   `nros board cmake-vars` GENERATES it rather than a board author writing
///   three lines of Kconfig per board.
/// * `requires_rust` — `!rust_targets.is_empty()`.
#[derive(Debug, Clone, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct BoardZephyr {
    /// Zephyr `BOARD` string, hwv2 `<board>/<soc>/<variant>` form.
    ///
    /// The one irreducible fact. Note it is also what `names` was being used to
    /// smuggle: the `zephyr` descriptor declares
    /// `names = ["zephyr", "native_sim/native/64"]`, a real name beside a
    /// Zephyr id, because there was nowhere else to put the second one.
    pub west_board: String,
    /// Zephyr-SDK toolchain ABI target (`aarch64-zephyr-elf`). `None` for a
    /// board built with the SDK's default for its arch.
    #[serde(default)]
    pub sdk_abi: Option<String>,
    /// Default RMW backend. A consumer overrides with `-DNANO_ROS_RMW=<rmw>`.
    #[serde(default)]
    pub default_rmw: Option<String>,
    /// Default transport (`ethernet`, `serial`, …).
    #[serde(default)]
    pub default_transport: Option<String>,
    /// West runner, stated ONLY when it differs from what Zephyr's own board
    /// definition already selects.
    ///
    /// `armfvp` is the case that needs it. `qemu` does not: Zephyr's
    /// `qemu_cortex_a53` board already declares its emulator, and restating it
    /// here would be a second spelling of a fact Zephyr owns.
    #[serde(default)]
    pub runner: Option<String>,
}

/// `[board.provisioning]` — what a downstream consumer's Zephyr tree needs.
///
/// RFC-0064 R5 D3. Drives `nros setup board <name> --zephyr-workspace <dir>`.
/// An `import:false` consumer does not inherit nano-ros's toolchain
/// provisioning, so the board package is the single source for what its tree
/// must gain before it can build.
#[derive(Debug, Clone, Default, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct BoardProvisioning {
    /// Zephyr support line — selects `scripts/zephyr/patches/<line>.sh`.
    #[serde(default)]
    pub zephyr_line: Option<String>,
    /// rustup target triples the board's Zephyr build compiles for. Empty
    /// means the board needs no Rust, which is also how `requires_rust` is
    /// derived — one fact, stated once.
    #[serde(default)]
    pub rust_targets: Vec<String>,
    /// `nros-sdk-index.toml` `[source.*]` name for the board's RMW source tree.
    #[serde(default)]
    pub rmw_source: Option<String>,
    /// `nros-sdk-index.toml` `[gated.*]` packages this board needs. Licence-
    /// gated, so they are declared and never downloaded.
    #[serde(default)]
    pub gated: Vec<String>,
}

/// `[board.cmake]` — CMake toolchain facts for the ament-shape preset flow.
/// Deliberately minimal: only the board-intrinsic toolchain file. The SDK
/// directory cache-vars (`NUTTX_DIR`, `THREADX_DIR`, …) are NOT restated here —
/// the platform CMake modules default them from their own on-disk location, and
/// the store compiler bin flows onto the preset's `environment.PATH` from the
/// provision result. No `${…}` templating (RFC-0048 §6, shape C′).
#[derive(Debug, Clone, Deserialize)]
pub struct BoardCmake {
    /// Repo-relative path to the CMake toolchain file, e.g.
    /// `cmake/toolchain/armv7a-nuttx-eabi.cmake`. `nros setup` resolves it against
    /// the repo root and emits it as the preset's `toolchainFile`.
    pub toolchain_file: String,
}

/// phase-351 W4 — why a `[board_config.<board>].netstack` was refused.
///
/// Both arms name what IS available, because the whole point of declaring the
/// domain is that a user who picked outside it can see the edge.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum NetstackError {
    #[error(
        "board `{board}` does not support netstack `{requested}` — it supports: {}",
        supported.join(", ")
    )]
    Unsupported {
        board: String,
        requested: String,
        supported: Vec<String>,
    },
    #[error(
        "board `{board}` declares no `supported_netstacks`, so `netstack = \"{requested}\"` \
         selects nothing: this board's RTOS (or its host) owns the stack. Drop the key."
    )]
    BoardSelectsNone { board: String, requested: String },
}

impl BoardDescriptor {
    /// Fill in what a convention produces, for fields the descriptor left out.
    ///
    /// RFC-0064 R5 D6. Each rule below was MEASURED across the twelve
    /// `[[board]]` rows in the tree before being made a default, and each one
    /// keeps its override: a model that cannot be escaped is worse than none
    /// (RFC-0064's customization ladder, rung 3).
    ///
    /// Deliberately NOT here, and worth recording because an earlier draft of
    /// R5 said otherwise:
    ///
    /// * **`entry_kind` is not derivable from `platform`.** The draft claimed
    ///   "hosted platform → `hosted-main`, zephyr → `zephyr-staticlib`, else
    ///   `board-run`, zero exceptions". Measured: `freertos-posix` and
    ///   `mps2-an385-freertos` are BOTH `platform = "freertos"` and their
    ///   `entry_kind` values are `hosted-main` and `board-run`. Whether a port
    ///   is hosted is a fact no field in this tree carries, so the derivation
    ///   would need a new one — which is not a simplification.
    /// * **`crate_path` and `board_features` are not dead.** No in-tree
    ///   descriptor authors either, but `builder/entry.rs` reads both, so they
    ///   are an out-of-tree extension point with no in-tree user rather than
    ///   fields to delete.
    fn apply_conventions(&mut self) {
        // `local_aliases` defaults to the platform feature. 8 of 10 stated
        // exactly that; the two real overrides (`platform-esp32-qemu`,
        // `platform-threadx-riscv64`) still say so and still win.
        if self.local_aliases.is_empty() && !self.platform_feature.is_empty() {
            self.local_aliases = vec![self.platform_feature.clone()];
        }
        if let Some(entry) = self.entry.as_mut() {
            // `crate_name` is `snake_case(board_crate)` in 7 of 7 boards that
            // state one — a mechanical transform of a field one line above it.
            if entry.crate_name.is_empty()
                && let Some(board_crate) = self.board_crate.as_deref()
            {
                entry.crate_name = board_crate.replace('-', "_");
            }
            // 4 of 7 signatures are this exact string. The three that differ
            // are real (an `esp_hal::main` attribute, a `cortex-m-rt` entry
            // macro re-exported by the board crate, a plain `fn main`), so it
            // is a default rather than a derivation.
            if entry.signature.is_empty() {
                entry.signature = "#[unsafe(no_mangle)]\nextern \"C\" fn main() -> !".to_string();
            }
        }
    }

    /// Does this board answer to `key`?
    ///
    /// Its declared `names`, plus its Zephyr board id when it has one. The
    /// second is a DERIVATION, not a second place to write a name: a Zephyr
    /// build is configured with `BOARD=<id>`, so an id has to resolve, and
    /// before RFC-0064 R5 the id was smuggled into `names` instead — the
    /// `zephyr` descriptor still reads
    /// `names = ["zephyr", "native_sim/native/64"]`, one real name beside a
    /// board id, because there was nowhere else to put the second.
    ///
    /// It also replaces what `attach_bundle_aliases` used to do by reading a
    /// bundle's `board.cmake` and pushing its `NROS_BOARD_ZEPHYR_ID` onto
    /// ANOTHER board's descriptor as an alias. That file is gone; this is the
    /// same capability, taken from the board's own descriptor.
    pub fn answers_to(&self, key: &str) -> bool {
        self.names.iter().any(|n| n == key)
            || self.zephyr.as_ref().is_some_and(|z| z.west_board == key)
    }

    /// Board-crate path relative to the workspace root, applying the
    /// `packages/boards/<board_crate>` default.
    pub fn crate_path_rel(&self) -> Option<String> {
        self.crate_path.clone().or_else(|| {
            self.board_crate
                .as_ref()
                .map(|c| format!("packages/boards/{c}"))
        })
    }

    /// issue 0606 — the descriptor's DIRECTORY as an alias:
    /// `packages/boards/nros-board-<x>/nros-board.toml` → `<x>`.
    ///
    /// `None` for an in-memory descriptor (no `source`). Not injective: one
    /// directory can hold several witnesses (the two NuttX boards), which is
    /// why the caller treats several matches as an ambiguity rather than
    /// picking one.
    pub fn directory_alias(&self) -> Option<String> {
        let src = self.source.as_deref()?;
        let dir = std::path::Path::new(src).parent()?.file_name()?.to_str()?;
        Some(dir.strip_prefix("nros-board-").unwrap_or(dir).to_string())
    }

    /// Resolved board capabilities — the declared `[board.capabilities]` block,
    /// or platform-inferred conservative defaults when omitted (241.C migration).
    pub fn capabilities(&self) -> BoardCapabilities {
        self.capabilities
            .unwrap_or_else(|| BoardCapabilities::inferred(self.platform))
    }

    /// Whether the board declared its capabilities explicitly (vs relying on the
    /// platform-inferred defaults). Used by the migration lint.
    pub fn has_declared_capabilities(&self) -> bool {
        self.capabilities.is_some()
    }

    /// phase-351 W4 — the netstack this deploy will build with, or an error
    /// naming what the board actually supports.
    ///
    /// `requested` is `[board_config.<board>].netstack`. `None` takes the board's
    /// first declared stack, which is why the list is ordered. A board that
    /// declares NO stacks makes no choice: naming one there is an error too,
    /// because silently ignoring it is how a deploy ends up believing it
    /// selected something.
    pub fn resolve_netstack<'a>(
        &'a self,
        requested: Option<&'a str>,
    ) -> Result<Option<&'a str>, NetstackError> {
        match (requested, self.supported_netstacks.first()) {
            (None, default) => Ok(default.map(String::as_str)),
            (Some(want), None) => Err(NetstackError::BoardSelectsNone {
                board: self.names.first().cloned().unwrap_or_default(),
                requested: want.to_string(),
            }),
            (Some(want), Some(_)) => {
                if self.supported_netstacks.iter().any(|s| s == want) {
                    Ok(Some(want))
                } else {
                    Err(NetstackError::Unsupported {
                        board: self.names.first().cloned().unwrap_or_default(),
                        requested: want.to_string(),
                        supported: self.supported_netstacks.clone(),
                    })
                }
            }
        }
    }

    /// Render `cargo_config` with `${workspace}` resolved to `workspace`.
    pub fn cargo_config_rendered(&self, workspace: &Path) -> Option<String> {
        let ws = path_for_template(workspace);
        self.cargo_config
            .as_ref()
            .map(|body| body.replace("${workspace}", &ws))
    }
}

/// Escape a path for embedding inside a double-quoted TOML string.
fn path_for_template(path: &Path) -> String {
    path.to_string_lossy()
        .replace('\\', "\\\\")
        .replace('"', "\\\"")
}

/// `deny_unknown_fields` here too: the file level is where a whole MISPLACED
/// table lands (a `[bard]` typo, or a `[knobs]` written at top level instead of
/// under the board). Without it the file parses and the table vanishes.
#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct BoardFile {
    #[serde(default, rename = "board")]
    boards: Vec<BoardDescriptor>,
}

/// The `[build]` / `[target.<triple>]` shape of a `cargo_config` template.
#[derive(Debug, Default, Deserialize)]
struct CargoConfigTemplate {
    #[serde(default)]
    build: Option<CargoConfigBuild>,
    #[serde(default)]
    target: std::collections::BTreeMap<String, toml::Value>,
}

#[derive(Debug, Default, Deserialize)]
struct CargoConfigBuild {
    #[serde(default)]
    target: Option<String>,
}

/// The rustc triple a board's `cargo_config` template states.
///
/// Issue 0951 — a board descriptor does not record its triple as DATA. It
/// records a `.cargo/config.toml` template, and the triple is in there: as
/// `[build] target = "..."`, or as the sole `[target.<triple>]` header. Every
/// shipped descriptor uses one of those two and NONE sets the `target` field
/// on `[[board]]` that `BoardDescriptor::target` expects — so that field has
/// been `None` for every board, and `nros build` has been copying the `None`
/// into `ImagePlan.target` (build.rs) without anyone noticing.
///
/// Parsed as TOML rather than matched with a regex: it IS a TOML document, and
/// a regex over it would also match a triple mentioned inside a rustflag.
///
/// Inferred only when UNAMBIGUOUS — `[build].target` wins; otherwise exactly
/// one `[target.*]` key. A template with several is genuinely multi-triple and
/// must not be collapsed to whichever came first.
fn target_from_cargo_config(cargo_config: &str) -> Option<String> {
    let parsed: CargoConfigTemplate = toml::from_str(cargo_config).ok()?;
    if let Some(t) = parsed.build.and_then(|b| b.target) {
        return Some(t);
    }
    let mut keys = parsed.target.into_keys();
    match (keys.next(), keys.next()) {
        (Some(only), None) => Some(only),
        _ => None,
    }
}

impl BoardFile {
    /// Fill each board's `target` from its `cargo_config` template when the
    /// board did not state one. An authored `target` always wins.
    fn with_inferred_targets(mut self) -> Vec<BoardDescriptor> {
        for b in &mut self.boards {
            if b.target.is_none()
                && let Some(cfg) = b.cargo_config.as_deref()
                && let Some(t) = target_from_cargo_config(cfg)
            {
                b.target = Some(t);
            }
            b.apply_conventions();
        }
        self.boards
    }
}

/// Every board descriptor discovered under `<workspace>/packages/boards`.
#[derive(Debug, Default)]
pub struct BoardCatalog {
    descriptors: Vec<BoardDescriptor>,
}

/// Extra board-search roots from `$NROS_EXTRA_BOARD_PATH` (PATH-style
/// separator). Each entry is a directory shaped like `packages/boards/` —
/// immediate subdirectories carrying `nros-board.toml` (or a
/// `boards/<name>/board.cmake` bundle). This is the out-of-tree escape
/// hatch the vendored-tree workflow needs: without it, a consumer board
/// had to be copied INTO the checkout for the catalog to see it.
///
/// Missing entries are skipped silently — the variable rides shell
/// profiles, and a path that only exists on the CI host must not break a
/// laptop run.
pub fn extra_board_roots() -> Vec<PathBuf> {
    match std::env::var_os("NROS_EXTRA_BOARD_PATH") {
        Some(v) => std::env::split_paths(&v)
            .filter(|p| !p.as_os_str().is_empty() && p.is_dir())
            .collect(),
        None => Vec::new(),
    }
}

impl BoardCatalog {
    /// Load every `packages/boards/*/nros-board.toml` under `workspace`,
    /// plus every root named by `$NROS_EXTRA_BOARD_PATH`.
    pub fn load(workspace: &Path) -> Result<Self, BoardLoadError> {
        Self::load_with_extra(workspace, &extra_board_roots())
    }

    /// [`Self::load`] with the extra roots passed explicitly (tests; env
    /// mutation is process-global and racy under a parallel test runner).
    pub fn load_with_extra(
        workspace: &Path,
        extra_roots: &[PathBuf],
    ) -> Result<Self, BoardLoadError> {
        let boards_dir = workspace.join("packages/boards");
        let mut descriptors = Vec::new();
        Self::load_root(&boards_dir, Some(workspace), &mut descriptors)?;
        for root in extra_roots {
            // Extra-root descriptors keep an ABSOLUTE `source`: the
            // workspace-relative invariant exists so IN-TREE paths in
            // committed artifacts don't drift per host, and an out-of-tree
            // board is by definition not in a committed nano-ros artifact.
            // `nano_ros_root.join(abs)` resolves to the absolute path, so
            // `NROS_BOARD_TOML` consumers keep working unchanged.
            Self::load_root(root, None, &mut descriptors)?;
        }
        Ok(Self { descriptors })
    }

    /// [`Self::load`] plus board descriptors carried by the USER's own
    /// workspace packages.
    ///
    /// A board is a package. `src/my_board/nros-board.toml` is discovered the
    /// same way `src/talker_pkg` is, so declaring a board for a workspace
    /// needs no environment variable, no path outside the tree, and nothing
    /// copied into the nano-ros checkout — the colcon property that everything
    /// a workspace needs lives in the workspace.
    ///
    /// `$NROS_EXTRA_BOARD_PATH` stays, and stays useful: a board shared by
    /// SEVERAL workspaces has no single workspace to live in. What changes is
    /// which one is the default answer.
    pub fn load_with_packages(
        workspace: &Path,
        pkg_dirs: &[PathBuf],
    ) -> Result<Self, BoardLoadError> {
        let mut catalog = Self::load_with_extra(workspace, &extra_board_roots())?;
        catalog.absorb_packages(pkg_dirs)?;
        Ok(catalog)
    }

    /// Load `<pkg>/nros-board.toml` for each package that carries one.
    ///
    /// Package dirs are named INDIVIDUALLY rather than scanning their parent:
    /// a workspace's layout is whatever its packages say it is, and a parent
    /// scan would also read directories that discovery deliberately excluded.
    fn absorb_packages(&mut self, pkg_dirs: &[PathBuf]) -> Result<(), BoardLoadError> {
        for dir in pkg_dirs {
            let descriptor_path = dir.join("nros-board.toml");
            if !descriptor_path.is_file() {
                continue;
            }
            let text = std::fs::read_to_string(&descriptor_path)
                .map_err(|e| BoardLoadError::Io(descriptor_path.clone(), e))?;
            let file: BoardFile = toml::from_str(&text)
                .map_err(|e| BoardLoadError::Parse(descriptor_path.clone(), e))?;
            // Absolute, like an extra root and for the same reason: the
            // workspace-relative form exists so IN-TREE paths in committed
            // nano-ros artifacts do not drift per host, and a user's board is
            // not in one.
            let abs = descriptor_path.display().to_string();
            self.descriptors
                .extend(file.with_inferred_targets().into_iter().map(|mut b| {
                    b.source = Some(abs.clone());
                    b
                }));
        }
        Ok(())
    }

    /// Append every `<root>/*/nros-board.toml` to `descriptors`.
    /// `rel_base = Some(ws)` records workspace-relative sources (in-tree);
    /// `None` records absolute ones (extra roots).
    /// Directories under `root` holding an `nros-board.toml`.
    ///
    /// RFC-0064 R5 D2. This used to be a single `read_dir` one level below the
    /// root, which had two consequences worth stating because both were load
    /// bearing:
    ///
    /// * A BUNDLE board could not own a descriptor. `fvp-aemv8r-smp` lives at
    ///   `nros-board-zephyr/boards/fvp-aemv8r-smp/`, one level too deep, and
    ///   resolved only because `attach_bundle_aliases` read its `board.cmake`
    ///   and pushed the bundle name onto the `zephyr` descriptor as an ALIAS.
    ///   The FVP board therefore had no descriptor at all — it borrowed one.
    ///   RFC-0064 R5 D4 deletes `board.cmake`, so the depth is a prerequisite.
    /// * A board could exist without ever announcing itself, because this walk
    ///   found descriptors and `provider_scan` found announcements, and nothing
    ///   compared them. See [`Self::require_announcement`].
    ///
    /// The prune and ignore rules are `cargo_nano_ros::provider_scan`'s, reached
    /// by `pub` rather than copied: a second list is how the two walks diverged
    /// in the first place (issue 0809 is the same defect one lane over).
    ///
    /// The recursion does NOT descend into a directory that already holds a
    /// descriptor. A board is a package, and a package does not contain another
    /// package — colcon's rule, and it keeps a board crate's own `target/`
    /// escapees from reading as nested boards even if a prune rule ever misses.
    /// A descriptor must sit beside a `package.xml` that announces a board.
    ///
    /// RFC-0064 R5 D5. A provider says what it IS in `package.xml` and what it
    /// LOWERS TO in the descriptor; with only the second half it resolves here
    /// and is invisible to `provider_scan` and to every consumer that reaches
    /// providers through it. That asymmetry is not hypothetical — phase-385
    /// landed `nros-board-mps3-an536-freertos` with a descriptor and no
    /// announcement and no gate said so, because the in-tree gate skipped any
    /// descriptor whose `package.xml` was absent (a migration ratchet that
    /// outlived its migration).
    ///
    /// Checked HERE and not only in the gate, because the gate sees the
    /// nano-ros tree and this sees a consumer's own boards and
    /// `$NROS_EXTRA_BOARD_PATH` too — the case the gate structurally cannot
    /// reach.
    ///
    /// Deliberately shallow: it requires the FILE and the `kind="board"` tag,
    /// not that the names match. Name equality is
    /// `check-provider-announcements`'s job and it needs a TOML parse this
    /// walk has not done yet.
    fn require_announcement(dir: &Path, descriptor_path: &Path) -> Result<(), BoardLoadError> {
        let pkg_xml = dir.join("package.xml");
        let announced = std::fs::read_to_string(&pkg_xml)
            .ok()
            .is_some_and(|body| Self::announces_a_board(&body));
        if announced {
            return Ok(());
        }
        Err(BoardLoadError::Unannounced {
            descriptor: descriptor_path.to_path_buf(),
            package_xml: pkg_xml,
        })
    }

    /// Does this `package.xml` body carry a `kind="board"` provision?
    ///
    /// Comments are stripped first, for issue 0516's reason: a provider's
    /// `package.xml` documents the tag in a comment, and a scan that cannot
    /// tell a comment from a declaration counts the documentation as the
    /// announcement. Every board `package.xml` in this tree has such a comment,
    /// so without the strip this check passes on a file that announces nothing.
    fn announces_a_board(body: &str) -> bool {
        let mut stripped = String::with_capacity(body.len());
        let mut rest = body;
        while let Some(start) = rest.find("<!--") {
            stripped.push_str(&rest[..start]);
            match rest[start..].find("-->") {
                Some(end) => rest = &rest[start + end + 3..],
                None => return stripped.contains("<nano_ros_provides") && false,
            }
        }
        stripped.push_str(rest);
        stripped.split("<nano_ros_provides").skip(1).any(|tag| {
            let tag = tag.split('>').next().unwrap_or("");
            tag.contains(r#"kind="board""#) || tag.contains("kind='board'")
        })
    }

    fn collect_board_dirs(
        dir: &Path,
        root: &Path,
        depth: usize,
        out: &mut Vec<PathBuf>,
    ) -> Result<(), BoardLoadError> {
        // A root the caller named is a root the caller meant, so a marker only
        // opts out BELOW it — provider_scan's `depth > 0` rule, same reasoning.
        if depth > 0 && cargo_nano_ros::provider_scan::is_ignored_dir(dir) {
            return Ok(());
        }
        if dir.join("nros-board.toml").is_file() {
            out.push(dir.to_path_buf());
            return Ok(());
        }
        let entries = match std::fs::read_dir(dir) {
            Ok(e) => e,
            Err(e) if depth > 0 => {
                // Unreadable subdirectory: skip it. Only the ROOT being
                // unreadable is an error the caller must hear about.
                let _ = e;
                return Ok(());
            }
            Err(e) => return Err(BoardLoadError::Io(dir.to_path_buf(), e)),
        };
        for entry in entries {
            let entry = entry.map_err(|e| BoardLoadError::Io(dir.to_path_buf(), e))?;
            let path = entry.path();
            if !path.is_dir() {
                continue;
            }
            let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
                continue;
            };
            if name.starts_with('.') || cargo_nano_ros::provider_scan::is_pruned_dir(name) {
                continue;
            }
            Self::collect_board_dirs(&path, root, depth + 1, out)?;
        }
        out.sort();
        Ok(())
    }

    fn load_root(
        root: &Path,
        rel_base: Option<&Path>,
        descriptors: &mut Vec<BoardDescriptor>,
    ) -> Result<(), BoardLoadError> {
        let mut dirs = Vec::new();
        // The in-tree dir must exist; an extra root was existence-checked by
        // `extra_board_roots`, so a race here is still an error.
        Self::collect_board_dirs(root, root, 0, &mut dirs)?;
        for dir in dirs {
            let descriptor_path = dir.join("nros-board.toml");
            Self::require_announcement(&dir, &descriptor_path)?;
            let text = std::fs::read_to_string(&descriptor_path)
                .map_err(|e| BoardLoadError::Io(descriptor_path.clone(), e))?;
            let file: BoardFile = toml::from_str(&text)
                .map_err(|e| BoardLoadError::Parse(descriptor_path.clone(), e))?;
            // Workspace-relative, forward slashes — it goes into a COMMITTED
            // generated header, so an absolute host path would be drift the
            // moment anyone else regenerates it. (Extra roots: absolute, see
            // `load_with_extra`.)
            let rel = match rel_base {
                Some(ws) => descriptor_path
                    .strip_prefix(ws)
                    .unwrap_or(&descriptor_path)
                    .components()
                    .map(|c| c.as_os_str().to_string_lossy().into_owned())
                    .collect::<Vec<_>>()
                    .join("/"),
                None => descriptor_path.display().to_string(),
            };
            descriptors.extend(file.with_inferred_targets().into_iter().map(|mut b| {
                b.source = Some(rel.clone());
                b
            }));
        }
        Ok(())
    }

    /// issue 0729 — conf-bundle boards join the catalog as ALIASES of their
    /// family's descriptor.
    ///
    /// Phase-337 W9.a folded per-board Zephyr crates into
    /// `packages/boards/nros-board-<family>/boards/<name>/` (board.cmake +
    /// confs, no `Cargo.toml`, no `nros-board.toml`), so no descriptor claimed
    /// their names and every consumer of `resolve_deploy` — the site-config
    /// gate, `board-facts` — answered Unknown for them. A bundle is a conf
    /// overlay on its family's PLATFORM lane, so the facts a descriptor
    /// carries (platform, toolchain, entry_kind, capabilities) are the
    /// family's; the per-board knobs live in the bundle's `board.cmake` and in
    /// site config, which the consumers read separately.
    ///
    /// The bundle's dir name and, when parseable, its `NROS_BOARD_ZEPHYR_ID`
    /// are appended to the `names` of the descriptor whose `platform` matches
    /// the family suffix (`nros-board-zephyr` → `zephyr`) — and only when no
    /// existing descriptor already claims that name and exactly ONE descriptor
    /// matches the family, so a hand-authored claim always wins and an
    /// ambiguous family attaches nothing rather than guessing.

    pub fn from_descriptors(descriptors: Vec<BoardDescriptor>) -> Self {
        Self { descriptors }
    }

    pub fn descriptors(&self) -> &[BoardDescriptor] {
        &self.descriptors
    }

    /// Resolve a `(board, target)` pair to its descriptor.
    ///
    /// A board name may be claimed by two descriptors (e.g. `threadx` →
    /// `threadx-riscv64` vs `threadx-linux`); the one whose `target_contains`
    /// matches the requested target wins, else the unconstrained one. As a last
    /// resort an unknown board on a `*-linux*` target resolves to `posix`
    /// (mirrors the old `target.contains("linux")` fallback).
    pub fn resolve(&self, board: &str, target: &str) -> Option<&BoardDescriptor> {
        let named: Vec<&BoardDescriptor> = self
            .descriptors
            .iter()
            .filter(|d| d.answers_to(board))
            .collect();
        if !named.is_empty() {
            // Prefer a target-qualified match, then the unconstrained one.
            return named
                .iter()
                .find(|d| {
                    d.target_contains
                        .as_ref()
                        .is_some_and(|sub| target.contains(sub.as_str()))
                })
                .or_else(|| named.iter().find(|d| d.target_contains.is_none()))
                .copied();
        }
        if target.contains("linux") {
            return self
                .descriptors
                .iter()
                .find(|d| d.platform == PlatformKind::Posix);
        }
        None
    }

    /// phase-341 W2 — resolve a leaf's `[package.metadata.nros.entry] deploy`
    /// token to the descriptor whose `cargo_config` governs that leaf's link.
    ///
    /// [`resolve`] cannot serve here: it takes a `(board, target)` pair and the
    /// target is exactly what the projection is trying to *derive*. Two rules,
    /// in order, each requiring a UNIQUE hit:
    ///
    /// 1. **`names`** — the board's own name/alias list. This is what separates
    ///    the two descriptors that share a triple *and* a platform:
    ///    `nuttx` (armv7a) vs `nuttx-riscv` (riscv32imac) are one `names` entry
    ///    apart, and `target_contains` — [`resolve`]'s discriminator — is
    ///    useless without a target to test it against.
    /// 2. **`platform`** — the mapping
    ///    `scripts/check-board-cargo-config-applied.sh` uses (it matches a
    ///    leaf's `deploy` against each `platform = "…"`). Reached only when no
    ///    board CLAIMS the name, and only when exactly one board declares that
    ///    platform — `nuttx` is declared by two, so a deploy token that reached
    ///    this rule for it is ambiguous rather than "the first one".
    ///
    /// Anything else is [`DeployResolution::Unknown`] / `Ambiguous`, and the
    /// caller must write NOTHING. A projection carrying the wrong board's link
    /// args is worse than no projection: the hand-mirrored block it would
    /// shadow is at least the block that links today (issue 0440).
    pub fn resolve_deploy(&self, deploy: &str) -> DeployResolution<'_> {
        let by_name: Vec<&BoardDescriptor> = self
            .descriptors
            .iter()
            .filter(|d| d.answers_to(deploy))
            .collect();
        match by_name.len() {
            1 => return DeployResolution::Board(by_name[0]),
            0 => {}
            _ => return DeployResolution::Ambiguous(descriptor_labels(&by_name)),
        }
        // issue 0606 — the DIRECTORY is an alias, stated here once.
        //
        // `[deploy.*].board` and a descriptor's `names` grew apart: the field
        // carries the DOWNSTREAM ecosystem's board id (Zephyr's
        // `native_sim/native/64`, PlatformIO's `esp32dev`, NuttX's
        // `qemu-armv7a-nsh`), while `names` is what nano-ros calls the board.
        // Most values happen to be in both. Where they were not, three
        // consumers each grew their own directory fallback — the site-config
        // gate, `board-facts`, and the standalone-leaf path — which is three
        // opinions about what a board is called. This is the one rule they now
        // share; the descriptors carry the downstream ids in `names`.
        let by_dir: Vec<&BoardDescriptor> = self
            .descriptors
            .iter()
            .filter(|d| d.directory_alias().as_deref() == Some(deploy))
            .collect();
        match by_dir.len() {
            1 => return DeployResolution::Board(by_dir[0]),
            0 => {}
            _ => return DeployResolution::Ambiguous(descriptor_labels(&by_dir)),
        }
        let by_platform: Vec<&BoardDescriptor> = self
            .descriptors
            .iter()
            .filter(|d| d.platform.kebab() == deploy)
            .collect();
        match by_platform.len() {
            1 => DeployResolution::Board(by_platform[0]),
            0 => DeployResolution::Unknown,
            _ => DeployResolution::Ambiguous(descriptor_labels(&by_platform)),
        }
    }
}

/// Human-readable identity of each candidate, for an ambiguity diagnostic.
fn descriptor_labels(candidates: &[&BoardDescriptor]) -> Vec<String> {
    candidates
        .iter()
        .map(|d| match &d.source {
            Some(src) => format!("{} ({src})", d.names.join("/")),
            None => d.names.join("/"),
        })
        .collect()
}

/// Outcome of [`BoardCatalog::resolve_deploy`].
#[derive(Debug)]
pub enum DeployResolution<'a> {
    /// Exactly one descriptor claims this deploy token.
    Board(&'a BoardDescriptor),
    /// No descriptor claims it (e.g. a deploy key known only to
    /// `nros_orchestration_ir::board_path_for`, or an out-of-tree board).
    Unknown,
    /// Several descriptors claim it and nothing here can choose between them.
    Ambiguous(Vec<String>),
}

/// Error loading or parsing board descriptors.
#[derive(Debug)]
pub enum BoardLoadError {
    Io(std::path::PathBuf, std::io::Error),
    Parse(std::path::PathBuf, toml::de::Error),
    /// A descriptor with no sibling `package.xml` announcing a board
    /// (RFC-0064 R5 D5). Its own variant rather than an `Io` so the message can
    /// say what to write, not merely that a file was missing.
    Unannounced {
        descriptor: std::path::PathBuf,
        package_xml: std::path::PathBuf,
    },
}

impl std::fmt::Display for BoardLoadError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BoardLoadError::Io(path, e) => write!(f, "reading {}: {e}", path.display()),
            BoardLoadError::Parse(path, e) => write!(f, "parsing {}: {e}", path.display()),
            BoardLoadError::Unannounced {
                descriptor,
                package_xml,
            } => write!(
                f,
                "{} has no sibling package.xml announcing a board.\n  \
                 A provider says what it IS in package.xml and what it LOWERS TO \
                 in the descriptor; with only the second half it is invisible to \
                 `provider_scan`.\n  \
                 Write {} with:\n    \
                 <export><nano_ros_provides kind=\"board\" name=\"...\"/></export>\n  \
                 naming the descriptor's `names`, canonical first.",
                descriptor.display(),
                package_xml.display()
            ),
        }
    }
}

impl std::error::Error for BoardLoadError {}

#[cfg(test)]
mod tests {
    use super::*;

    // ── phase-351 W4 — supported_netstacks ────────────────────────────────

    fn board_with(stacks: &[&str]) -> BoardDescriptor {
        let mut d: BoardDescriptor = toml::from_str::<BoardFile>(STM32_TOML)
            .expect("fixture parses")
            .boards
            .remove(0);
        d.supported_netstacks = stacks.iter().map(|s| s.to_string()).collect();
        d
    }

    /// No request takes the board's FIRST declared stack — which is why the
    /// list is ordered rather than a set.
    #[test]
    fn unrequested_netstack_takes_the_boards_default() {
        assert_eq!(
            board_with(&["lwip", "freertos_plus_tcp"])
                .resolve_netstack(None)
                .unwrap(),
            Some("lwip")
        );
    }

    #[test]
    fn a_supported_netstack_resolves_to_itself() {
        assert_eq!(
            board_with(&["lwip", "freertos_plus_tcp"])
                .resolve_netstack(Some("freertos_plus_tcp"))
                .unwrap(),
            Some("freertos_plus_tcp")
        );
    }

    /// The error must NAME the domain — a refusal that does not say what is
    /// available just moves the guessing.
    #[test]
    fn an_unsupported_netstack_lists_what_is_supported() {
        let err = board_with(&["netxduo"])
            .resolve_netstack(Some("lwip"))
            .expect_err("lwip is not in the board's table");
        let msg = err.to_string();
        assert!(msg.contains("netxduo"), "{msg}");
        assert!(msg.contains("lwip"), "{msg}");
    }

    /// A board that declares none makes no choice, so naming one is an error
    /// rather than a silent no-op: the deploy would otherwise believe it had
    /// selected something.
    #[test]
    fn naming_a_netstack_on_a_board_that_has_none_is_refused() {
        let err = board_with(&[])
            .resolve_netstack(Some("lwip"))
            .expect_err("a board with no table cannot honour a request");
        assert!(err.to_string().contains("owns the stack"), "{err}");
        assert_eq!(board_with(&[]).resolve_netstack(None).unwrap(), None);
    }

    /// The SHIPPED descriptors, so the declarations cannot rot: each board that
    /// claims a stack must resolve it, and the empties must refuse.
    #[test]
    fn shipped_boards_declare_a_resolvable_domain() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            // packages/cli/nros-cli-core -> cli -> packages -> repo root
            .ancestors()
            .nth(3)
            .expect("repo root")
            .to_path_buf();
        // No `Err(_) => return` escape: this test asserts about the SHIPPED
        // descriptors, so a catalog it cannot load is a failure, not a reason
        // to report green having checked nothing (issue 0571's shape).
        let catalog = BoardCatalog::load(&root)
            .unwrap_or_else(|e| panic!("shipped board catalog under {}: {e}", root.display()));
        let mut checked = 0;
        for d in catalog.descriptors() {
            for want in &d.supported_netstacks {
                assert_eq!(
                    d.resolve_netstack(Some(want)).unwrap(),
                    Some(want.as_str()),
                    "board {:?} does not resolve its own declared stack",
                    d.names
                );
                checked += 1;
            }
            assert!(
                d.resolve_netstack(None).is_ok(),
                "board {:?} cannot resolve its default",
                d.names
            );
        }
        assert!(checked > 0, "no shipped board declares a netstack");
    }

    // ── NROS_EXTRA_BOARD_PATH — out-of-tree board roots ──────────────────

    fn repo_root_for_tests() -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("repo root")
            .to_path_buf()
    }

    /// A descriptor in an extra root joins the catalog, resolves by name,
    /// and carries an ABSOLUTE source (out-of-tree paths are consumer-local,
    /// never committed nano-ros artifacts — see `load_with_extra`).
    /// Write the `package.xml` a board package must carry (RFC-0064 R5 D5).
    ///
    /// Carries a COMMENT mentioning the tag, because every real board
    /// `package.xml` in this tree does and issue 0516 is about a scan that
    /// counts the documentation as the declaration.
    fn write_board_package_xml(dir: &Path, names: &[&str]) {
        let tags: String = names
            .iter()
            .map(|n| format!("    <nano_ros_provides kind=\"board\" name=\"{n}\"/>\n"))
            .collect();
        std::fs::write(
            dir.join("package.xml"),
            format!(
                "<?xml version=\"1.0\"?>\n<package format=\"3\">\n  <export>\n\
                 <!-- e.g. <nano_ros_provides kind=\"board\" name=\"documented\"/> -->\n\
                 {tags}  </export>\n</package>\n"
            ),
        )
        .unwrap();
    }

    #[test]
    fn a_descriptor_with_no_announcement_is_refused() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let crate_dir = tmp.path().join("nros-board-silentx");
        std::fs::create_dir_all(&crate_dir).unwrap();
        std::fs::write(
            crate_dir.join("nros-board.toml"),
            STM32_TOML.replace("stm32f4\"", "silentx\""),
        )
        .unwrap();

        let err =
            BoardCatalog::load_with_extra(&repo_root_for_tests(), &[tmp.path().to_path_buf()])
                .expect_err("a descriptor with no package.xml must not resolve");
        assert!(
            matches!(err, BoardLoadError::Unannounced { .. }),
            "expected Unannounced, got {err:?}"
        );
        let msg = err.to_string();
        assert!(
            msg.contains("package.xml") && msg.contains("nano_ros_provides"),
            "the message must say what to write; got {msg}"
        );
    }

    #[test]
    fn a_commented_out_announcement_does_not_count() {
        // Issue 0516's rule, applied here: every board package.xml in this tree
        // documents the tag in a comment, so a check that cannot tell a comment
        // from a declaration passes on a file announcing nothing.
        let tmp = tempfile::tempdir().expect("tempdir");
        let crate_dir = tmp.path().join("nros-board-commentedx");
        std::fs::create_dir_all(&crate_dir).unwrap();
        std::fs::write(
            crate_dir.join("nros-board.toml"),
            STM32_TOML.replace("stm32f4\"", "commentedx\""),
        )
        .unwrap();
        std::fs::write(
            crate_dir.join("package.xml"),
            "<package><export>\n<!-- <nano_ros_provides kind=\"board\" name=\"commentedx\"/> -->\n</export></package>",
        )
        .unwrap();

        let err =
            BoardCatalog::load_with_extra(&repo_root_for_tests(), &[tmp.path().to_path_buf()])
                .expect_err("a commented-out announcement must not satisfy D5");
        assert!(
            matches!(err, BoardLoadError::Unannounced { .. }),
            "got {err:?}"
        );
    }

    /// A bundle board one level deeper than the old walk reached.
    ///
    /// The depth is the point: before RFC-0064 R5 D2 this descriptor was
    /// invisible, and `fvp-aemv8r-smp` only resolved because
    /// `attach_bundle_aliases` read its `board.cmake` and pushed the name onto
    /// the family descriptor as an alias.
    #[test]
    fn a_bundle_board_one_level_deeper_is_found() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let bundle = tmp.path().join("nros-board-familyx/boards/deepx");
        std::fs::create_dir_all(&bundle).unwrap();
        std::fs::write(
            bundle.join("nros-board.toml"),
            STM32_TOML.replace("stm32f4\"", "deepx\""),
        )
        .unwrap();
        write_board_package_xml(&bundle, &["deepx"]);

        let cat =
            BoardCatalog::load_with_extra(&repo_root_for_tests(), &[tmp.path().to_path_buf()])
                .expect("bundle-depth descriptor loads");
        assert!(
            cat.descriptors()
                .iter()
                .any(|d| d.names.iter().any(|n| n == "deepx")),
            "a board one level below the root must be found"
        );
    }

    /// A pruned or opted-out subtree stays invisible at depth.
    ///
    /// The walk got deeper, so the prune rules stopped being decorative. They
    /// are `provider_scan`'s, reached by `pub` rather than copied.
    #[test]
    fn the_deep_walk_honours_prunes_and_ignore_markers() {
        let tmp = tempfile::tempdir().expect("tempdir");
        for (rel, marker) in [
            ("target/nros-board-buildx", None),
            ("vendored/nros-board-optedx", Some("COLCON_IGNORE")),
        ] {
            let dir = tmp.path().join(rel);
            std::fs::create_dir_all(&dir).unwrap();
            let name = dir.file_name().unwrap().to_str().unwrap().to_string();
            let board = name.trim_start_matches("nros-board-").to_string();
            std::fs::write(
                dir.join("nros-board.toml"),
                STM32_TOML.replace("stm32f4\"", &format!("{board}\"")),
            )
            .unwrap();
            // Deliberately NOT announced: if a prune ever regressed, D5 would
            // mask it as an Unannounced error instead of the missed prune it is.
            if let Some(m) = marker {
                std::fs::write(dir.parent().unwrap().join(m), "").unwrap();
            }
        }

        let cat =
            BoardCatalog::load_with_extra(&repo_root_for_tests(), &[tmp.path().to_path_buf()])
                .expect("pruned and ignored subtrees must not even be read");
        for absent in ["buildx", "optedx"] {
            assert!(
                !cat.descriptors()
                    .iter()
                    .any(|d| d.names.iter().any(|n| n == absent)),
                "{absent} sits in a pruned/ignored subtree and must not resolve"
            );
        }
    }

    #[test]
    fn an_extra_root_descriptor_joins_the_catalog_with_absolute_source() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let crate_dir = tmp.path().join("nros-board-vendorx");
        std::fs::create_dir_all(&crate_dir).unwrap();
        std::fs::write(
            crate_dir.join("nros-board.toml"),
            STM32_TOML.replace("stm32f4\"", "vendorx\""),
        )
        .unwrap();
        // RFC-0064 R5 D5 — a board is a package, out of tree exactly as in it,
        // so the fixture announces itself. Before D5 this file was absent and
        // the board resolved anyway, which is the asymmetry D5 closes.
        write_board_package_xml(&crate_dir, &["vendorx"]);

        let cat =
            BoardCatalog::load_with_extra(&repo_root_for_tests(), &[tmp.path().to_path_buf()])
                .expect("load with extra root");
        let d = cat
            .descriptors()
            .iter()
            .find(|d| d.names.iter().any(|n| n == "vendorx"))
            .expect("extra-root board resolves by name");
        let src = d.source.as_deref().expect("source recorded");
        assert!(
            std::path::Path::new(src).is_absolute(),
            "extra-root source must be absolute, got {src}"
        );
    }

    /// An empty / unset extra list is exactly the old behavior.
    #[test]
    fn no_extra_roots_is_the_plain_in_tree_catalog() {
        let root = repo_root_for_tests();
        let plain = BoardCatalog::load_with_extra(&root, &[]).expect("load");
        assert!(
            plain
                .descriptors()
                .iter()
                .all(|d| !d.source.as_deref().unwrap_or("").starts_with('/')),
            "in-tree sources stay workspace-relative"
        );
    }

    const STM32_TOML: &str = r##"
[[board]]
names = ["stm32f4", "stm32f429"]
platform = "stm32"
target = "thumbv7em-none-eabihf"
toolchain = "stable"
platform_feature = "platform-bare-metal"
local_aliases = ["platform-stm32"]
link_kind = "none"
entry_kind = "board-run"
chip = "stm32f429"
board_crate = "nros-board-stm32f4"
cargo_config = """
[build]
target = "thumbv7em-none-eabihf"
"""

[board.entry]
crate_name = "nros_board_stm32f4"
signature = "#[nros_board_stm32f4::entry]\nfn main() -> !"
crate_root_extra = "use panic_probe as _;"
crate_root_deps = ["panic-probe = \"0.3\""]

[[board]]
names = ["stm32f407"]
platform = "stm32"
target = "thumbv7em-none-eabihf"
toolchain = "stable"
platform_feature = "platform-bare-metal"
link_kind = "none"
entry_kind = "board-run"
chip = "stm32f407"
board_crate = "nros-board-stm32f4"

[board.entry]
crate_name = "nros_board_stm32f4"
signature = "#[nros_board_stm32f4::entry]\nfn main() -> !"
"##;

    fn catalog() -> BoardCatalog {
        let file: BoardFile = toml::from_str(STM32_TOML).expect("parse stm32 descriptor");
        BoardCatalog::from_descriptors(file.boards)
    }

    /// phase-241 C.4 — migration lint (merge gate): every in-tree board must
    /// declare `[board.capabilities]` rather than rely on the platform-inferred
    /// defaults. All boards declare today; this catches a future board that
    /// omits the block (which would silently inherit a possibly-wrong heap/
    /// threads default — the issue-0038 footgun).
    /// Phase 252 (issue 0072) — a board descriptor advertises the `safety-e2e`
    /// capability feature, so codegen lowers `[safety]` to that board's
    /// `safety-e2e = ["nros-rmw-zenoh?/safety-e2e"]` forwarding.
    ///
    /// phase-337 W7.a rehomed this from `stm32f4` (deleted with its board) to
    /// `bare-metal` (the mps2-an385 descriptor). The assertion is over the SET rather than one name: the
    /// point is that the forwarding chain has at least one in-tree witness, and
    /// keying it to a single board is what made a board deletion look like a
    /// capability regression.
    /// issue 0729 — a conf-bundle board (no crate, no descriptor file) must
    /// resolve as a deploy token, both by its bundle dir name and by its
    /// Zephyr board id, to its family's descriptor.
    #[test]
    fn a_bundle_board_resolves_to_its_own_descriptor() {
        let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("repo root")
            .to_path_buf();
        let cat = BoardCatalog::load(&root).expect("load real board catalog");
        for token in ["fvp-aemv8r-smp", "fvp_baser_aemv8r/fvp_aemv8r_aarch64/smp"] {
            match cat.resolve_deploy(token) {
                DeployResolution::Board(d) => {
                    assert_eq!(d.platform, PlatformKind::Zephyr);
                    // RFC-0064 R5: it lands on the FVP's OWN descriptor now.
                    // It used to land on the `zephyr` (native_sim) descriptor,
                    // because `attach_bundle_aliases` read the bundle's
                    // `board.cmake` and pushed both these tokens onto it as
                    // aliases -- so the FVP board had no descriptor and
                    // borrowed one, and every fact about it that the borrowed
                    // descriptor also stated was simply the wrong board's.
                    assert!(
                        d.names.iter().any(|n| n == "fvp-aemv8r-smp"),
                        "`{token}` landed on {:?}, not the FVP board",
                        d.names
                    );
                    // And the Zephyr id resolves because the descriptor DECLARES
                    // it in `[board.zephyr] west_board`, not because a name list
                    // restates it.
                    assert_eq!(
                        d.zephyr.as_ref().map(|z| z.west_board.as_str()),
                        Some("fvp_baser_aemv8r/fvp_aemv8r_aarch64/smp")
                    );
                }
                other => panic!("bundle `{token}` did not resolve: {other:?}"),
            }
        }
    }

    /// A hand-authored `names` claim always beats a bundle alias — the alias
    /// attaches only to names nothing else claims.
    #[test]
    fn bundle_aliases_never_shadow_an_authored_name() {
        let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("repo root")
            .to_path_buf();
        let cat = BoardCatalog::load(&root).expect("load real board catalog");
        let claims: Vec<&BoardDescriptor> = cat
            .descriptors()
            .iter()
            .filter(|d| d.names.iter().any(|n| n == "zephyr"))
            .collect();
        assert_eq!(
            claims.len(),
            1,
            "`zephyr` must stay uniquely claimed by the authored descriptor"
        );
    }

    #[test]
    fn every_in_tree_board_declares_capabilities() {
        let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("repo root from packages/cli/nros-cli-core")
            .to_path_buf();
        let cat = BoardCatalog::load(&root).expect("load real board catalog");
        assert!(
            !cat.descriptors().is_empty(),
            "no boards loaded from {}/packages/boards",
            root.display()
        );
        let undeclared: Vec<String> = cat
            .descriptors()
            .iter()
            .filter(|d| !d.has_declared_capabilities())
            .map(|d| d.names.join("/"))
            .collect();
        assert!(
            undeclared.is_empty(),
            "boards relying on inferred capabilities — add [board.capabilities] \
             to their nros-board.toml: {undeclared:?}"
        );
    }

    /// Read a FreeRTOS config header and INLINE its relative `#include "..."`
    /// siblings, so the caller sees the same text the compiler does.
    ///
    /// phase-337 W5.a hoisted the shared body of `FreeRTOSConfig.h` into
    /// `nros-board-freertos/config/`, leaving each board's copy as two
    /// `#define`s plus a relative include. The agreement gate below reads that
    /// file directly, so from W5.a until phase-337 W7.b it saw NO
    /// `configSUPPORT_DYNAMIC_ALLOCATION` at all and read it as `0` — the gate
    /// went red while the thing it guards was fine. A gate that stops at the
    /// first file is narrower than the rule it enforces (the issue-0196 class),
    /// so it follows the include instead.
    ///
    /// Deliberately NOT a C preprocessor: it resolves `#include "relative/path"`
    /// against the including file's directory, depth-first, and ignores
    /// `#include <system>` and any conditional compilation. That is exactly the
    /// shape these config headers use, and anything richer would be a second
    /// implementation of cpp living in a test.
    fn read_freertos_config(path: &std::path::Path) -> Option<String> {
        fn walk(path: &std::path::Path, depth: usize, out: &mut String) {
            // A cycle or a pathological chain must not hang the test suite.
            if depth > 8 {
                return;
            }
            let Ok(src) = std::fs::read_to_string(path) else {
                return;
            };
            let dir = path.parent().unwrap_or(std::path::Path::new("."));
            for line in src.lines() {
                let trimmed = line.trim();
                if let Some(rest) = trimmed.strip_prefix("#include")
                    && let Some(open) = rest.find('"')
                    && let Some(close) = rest[open + 1..].find('"')
                {
                    let rel = &rest[open + 1..open + 1 + close];
                    walk(&dir.join(rel), depth + 1, out);
                    continue;
                }
                out.push_str(line);
                out.push('\n');
            }
        }
        if !path.exists() {
            return None;
        }
        let mut out = String::new();
        walk(path, 0, &mut out);
        Some(out)
    }

    /// `#define <name> <val>` is present with a non-zero `<val>` (the FreeRTOS
    /// idiom for an enabled feature). Absent or `0` → false.
    fn freertos_define_is_one(src: &str, name: &str) -> bool {
        src.lines().any(|line| {
            let line = line.trim();
            let Some(rest) = line.strip_prefix("#define") else {
                return false;
            };
            let mut it = rest.split_whitespace();
            it.next() == Some(name) && it.next().and_then(|v| v.parse::<i64>().ok()) == Some(1)
        })
    }

    /// Phase 241.C.2b — for a FreeRTOS board that co-locates its
    /// `config/FreeRTOSConfig.h`, the declared `[board.capabilities]` must AGREE
    /// with the RTOS config it claims to mirror, not silently override it:
    /// `configSUPPORT_DYNAMIC_ALLOCATION` ↔ `heap`, `configUSE_MUTEXES` ↔
    /// `threads`. Catches the #38-class drift (board.toml says heap-capable but
    /// the FreeRTOS config disabled dynamic allocation) at merge time rather than
    /// in an e2e dispatch. (Zephyr's heap/mutex live in per-app Kconfig, not a
    /// board-local file, so they stay config-derived — see 241.C.2b note.)
    #[test]
    fn freertos_capabilities_agree_with_freertosconfig() {
        let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("repo root from packages/cli/nros-cli-core")
            .to_path_buf();
        let cat = BoardCatalog::load(&root).expect("load real board catalog");
        let mut checked = 0usize;
        for d in cat.descriptors() {
            if d.platform != PlatformKind::Freertos {
                continue;
            }
            let Some(rel) = d.crate_path_rel() else {
                continue;
            };
            let cfg = root.join(&rel).join("config/FreeRTOSConfig.h");
            // Follows the W5.a relative include into `nros-board-freertos`; a
            // board with no co-located config at all yields `None` and is
            // skipped, exactly as before.
            let Some(src) = read_freertos_config(&cfg) else {
                continue; // board without a co-located config — nothing to cross-check
            };
            let caps = d.capabilities();
            let cfg_heap = freertos_define_is_one(&src, "configSUPPORT_DYNAMIC_ALLOCATION");
            let cfg_threads = freertos_define_is_one(&src, "configUSE_MUTEXES");
            let name = d.names.join("/");
            assert_eq!(
                caps.heap,
                cfg_heap,
                "board `{name}`: [board.capabilities] heap={} but \
                 configSUPPORT_DYNAMIC_ALLOCATION={} in {}",
                caps.heap,
                cfg_heap as u8,
                cfg.display()
            );
            assert_eq!(
                caps.threads,
                cfg_threads,
                "board `{name}`: [board.capabilities] threads={} but \
                 configUSE_MUTEXES={} in {}",
                caps.threads,
                cfg_threads as u8,
                cfg.display()
            );
            checked += 1;
        }
        assert!(
            checked > 0,
            "no FreeRTOS board with a co-located config/FreeRTOSConfig.h was \
             cross-checked — the C.2b agreement guard is vacuous"
        );
    }

    #[test]
    fn resolves_board_by_alias() {
        let cat = catalog();
        let d = cat.resolve("stm32f4", "thumbv7em-none-eabihf").unwrap();
        assert_eq!(d.platform, PlatformKind::Stm32);
        assert_eq!(d.chip.as_deref(), Some("stm32f429"));
        // alias of the same descriptor
        assert_eq!(
            cat.resolve("stm32f429", "thumbv7em-none-eabihf")
                .unwrap()
                .chip
                .as_deref(),
            Some("stm32f429")
        );
    }

    #[test]
    fn multi_board_crate_distinguishes_by_name() {
        let cat = catalog();
        // Same crate, different chip.
        let f407 = cat.resolve("stm32f407", "thumbv7em-none-eabihf").unwrap();
        assert_eq!(f407.chip.as_deref(), Some("stm32f407"));
        assert_eq!(f407.board_crate.as_deref(), Some("nros-board-stm32f4"));
    }

    #[test]
    fn crate_path_defaults_under_packages_boards() {
        let cat = catalog();
        let d = cat.resolve("stm32f4", "thumbv7em-none-eabihf").unwrap();
        assert_eq!(
            d.crate_path_rel().as_deref(),
            Some("packages/boards/nros-board-stm32f4")
        );
    }

    #[test]
    fn cargo_config_substitutes_workspace() {
        let descriptor = BoardDescriptor {
            names: vec!["x".into()],
            west_board: None,
            platform: PlatformKind::ThreadxRiscv64,
            target: None,
            toolchain: Toolchain::Stable,
            platform_feature: "platform-threadx".into(),
            local_aliases: vec![],
            link_kind: LinkKind::None,
            entry_kind: EntryKind::BoardRun,
            supported_netstacks: Vec::new(),
            chip: None,
            board_crate: None,
            crate_path: None,
            board_features: vec![],
            priority_plan: None,
            cargo_config: Some("inc = \"${workspace}/third-party/x\"".into()),
            entry: None,
            target_contains: None,
            capabilities: None,
            cmake: None,
            source: None,
            zephyr: None,
            provisioning: None,
        };
        let rendered = descriptor.cargo_config_rendered(Path::new("/ws")).unwrap();
        assert_eq!(rendered, "inc = \"/ws/third-party/x\"");
    }

    /// phase-341 W2 — `PlatformKind::kebab()` must spell each variant exactly
    /// as serde deserializes it, since the deploy→descriptor fallback compares
    /// a leaf's `deploy` token against that spelling. A hand-written match can
    /// drift from `rename_all`; this closes the loop on every variant.
    #[test]
    fn platform_kebab_round_trips() {
        use PlatformKind::*;
        for p in [
            Posix,
            Freertos,
            BareMetal,
            Nuttx,
            Zephyr,
            ThreadxLinux,
            ThreadxRiscv64,
            Esp32,
            Stm32,
            OrinSpe,
        ] {
            let back: PlatformKind = serde_json::from_str(&format!("\"{}\"", p.kebab()))
                .unwrap_or_else(|e| panic!("`{}` does not deserialize back: {e}", p.kebab()));
            assert_eq!(back, p, "kebab() spelling disagrees with serde for {p:?}");
        }
    }

    /// phase-341 W2 — the ambiguity the phase doc names: two NuttX descriptors
    /// share `platform = "nuttx"` (and back the same crate), and are told apart
    /// only by `names`. Resolution must pick the arm board for `deploy =
    /// "nuttx"` and the riscv one for `deploy = "nuttx-riscv"` — a projection
    /// that swapped them would write the wrong triple and link args.
    #[test]
    fn deploy_resolves_the_two_nuttx_boards_apart() {
        let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("repo root")
            .to_path_buf();
        let cat = BoardCatalog::load(&root).expect("load real board catalog");
        let arm = match cat.resolve_deploy("nuttx") {
            DeployResolution::Board(d) => d,
            other => panic!("deploy `nuttx` did not resolve: {other:?}"),
        };
        let riscv = match cat.resolve_deploy("nuttx-riscv") {
            DeployResolution::Board(d) => d,
            other => panic!("deploy `nuttx-riscv` did not resolve: {other:?}"),
        };
        let arm_cfg = arm.cargo_config.as_deref().expect("arm board cargo_config");
        let riscv_cfg = riscv
            .cargo_config
            .as_deref()
            .expect("riscv board cargo_config");
        assert!(
            arm_cfg.contains("armv7a-nuttx-eabihf"),
            "`nuttx` resolved to a board whose cargo_config is not the arm one:\n{arm_cfg}"
        );
        assert!(
            riscv_cfg.contains("riscv"),
            "`nuttx-riscv` resolved to a board whose cargo_config is not the riscv one:\n{riscv_cfg}"
        );
        // Both descriptors are read from the same file; the header a projection
        // writes must still name it.
        assert_eq!(
            arm.source.as_deref(),
            Some("packages/boards/nros-board-nuttx-qemu/nros-board.toml")
        );
    }

    /// A token no descriptor claims resolves to `Unknown` — never to "the first
    /// board that looked close". The caller writes nothing for these.
    #[test]
    fn unclaimed_deploy_token_is_unknown() {
        let cat = catalog();
        assert!(matches!(
            cat.resolve_deploy("some-out-of-tree-board"),
            DeployResolution::Unknown
        ));
    }

    /// Two descriptors sharing a `names` entry (`threadx` → riscv64 vs linux)
    /// are AMBIGUOUS without a target to test `target_contains` against. Real
    /// catalog: the pair exists in tree, and no leaf may silently get one.
    #[test]
    fn deploy_shared_by_two_boards_is_ambiguous() {
        let root = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("repo root")
            .to_path_buf();
        let cat = BoardCatalog::load(&root).expect("load real board catalog");
        match cat.resolve_deploy("threadx") {
            DeployResolution::Ambiguous(names) => {
                assert!(
                    names.len() >= 2,
                    "expected several candidates, got {names:?}"
                );
            }
            other => panic!("`threadx` is claimed by two boards, got {other:?}"),
        }
    }

    /// The `platform` fallback — the mapping
    /// `check-board-cargo-config-applied.sh` uses — resolves a token no board
    /// NAMES, and only when a single board declares that platform.
    #[test]
    fn deploy_falls_back_to_platform_when_unique() {
        let cat = catalog();
        // `stm32` is declared by both stm32 descriptors → ambiguous, not a guess.
        assert!(matches!(
            cat.resolve_deploy("stm32"),
            DeployResolution::Ambiguous(_)
        ));
        let mut boards = catalog().descriptors;
        boards.retain(|d| d.names.iter().any(|n| n == "stm32f407"));
        let cat = BoardCatalog::from_descriptors(boards);
        match cat.resolve_deploy("stm32") {
            DeployResolution::Board(d) => assert_eq!(d.chip.as_deref(), Some("stm32f407")),
            other => panic!("unique platform must resolve, got {other:?}"),
        }
    }

    /// Issue 0951 — every shipped board's triple comes out of its
    /// `cargo_config` template, because no descriptor sets the `target` field
    /// on `[[board]]`. Before this, `BoardDescriptor::target` was `None` for
    /// EVERY board in the tree and both readers silently got nothing.
    #[test]
    fn shipped_boards_resolve_their_rustc_triple() {
        let repo = Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("repo root")
            .to_path_buf();
        let catalog = BoardCatalog::load(&repo).expect("in-tree catalog");
        for (board, want) in [
            ("mps2-an385-freertos", "thumbv7m-none-eabi"),
            ("mps3-an536-freertos", "armv8r-none-eabihf"),
            ("esp32-qemu", "riscv32imc-unknown-none-elf"),
        ] {
            let d = match catalog.resolve_deploy(board) {
                DeployResolution::Board(d) => d,
                other => panic!("{board} does not resolve: {other:?}"),
            };
            assert_eq!(
                d.target.as_deref(),
                Some(want),
                "{board} must state its triple"
            );
        }
        // A host board has no cross triple, and inventing one would be worse
        // than `None` — the caller's default IS the host.
        let host = match catalog.resolve_deploy("threadx-linux") {
            DeployResolution::Board(d) => d,
            other => panic!("threadx-linux does not resolve: {other:?}"),
        };
        assert_eq!(host.target, None);
    }

    /// `[build].target` wins, and several `[target.*]` tables are ambiguous
    /// rather than first-wins — collapsing them is how one board answers for
    /// another.
    #[test]
    fn a_triple_is_inferred_only_when_unambiguous() {
        assert_eq!(
            target_from_cargo_config("[target.thumbv7m-none-eabi]\nrunner = \"q\"\n").as_deref(),
            Some("thumbv7m-none-eabi")
        );
        assert_eq!(
            target_from_cargo_config(
                "[build]\ntarget = \"riscv32imc-unknown-none-elf\"\n\
                 [target.riscv32imc-unknown-none-elf]\nrunner = \"q\"\n"
            )
            .as_deref(),
            Some("riscv32imc-unknown-none-elf")
        );
        assert_eq!(
            target_from_cargo_config(
                "[target.thumbv7m-none-eabi]\nrunner = \"a\"\n\
                 [target.armv8r-none-eabihf]\nrunner = \"b\"\n"
            ),
            None,
            "two triples, no `[build]` — ambiguous"
        );
        // A triple MENTIONED in a rustflag is not a declaration. This is why
        // the template is parsed as TOML rather than matched with a regex.
        assert_eq!(
            target_from_cargo_config(
                "[build]\nrustflags = [\"--sysroot=/x/thumbv7m-none-eabi\"]\n"
            ),
            None
        );
        assert_eq!(target_from_cargo_config("not = valid toml ["), None);
    }

    #[test]
    fn unknown_board_on_linux_target_falls_back_to_posix() {
        let mut boards = catalog().descriptors;
        boards.push(BoardDescriptor {
            names: vec!["native".into(), "posix".into()],
            west_board: None,
            platform: PlatformKind::Posix,
            target: None,
            toolchain: Toolchain::Stable,
            platform_feature: "platform-posix".into(),
            local_aliases: vec![],
            link_kind: LinkKind::None,
            entry_kind: EntryKind::HostedMain,
            supported_netstacks: Vec::new(),
            chip: None,
            board_crate: None,
            crate_path: None,
            board_features: vec![],
            priority_plan: None,
            cargo_config: None,
            entry: None,
            target_contains: None,
            capabilities: None,
            cmake: None,
            source: None,
            zephyr: None,
            provisioning: None,
        });
        let cat = BoardCatalog::from_descriptors(boards);
        let d = cat
            .resolve("some-unknown", "x86_64-unknown-linux-gnu")
            .unwrap();
        assert_eq!(d.platform, PlatformKind::Posix);
    }

    /// Every crate a `crate_root_extra` `use`s must be a declared dependency.
    ///
    /// phase-383 W9.b. While entries were hand-written a human wrote both the
    /// `use` and the `[dependencies]` line, so they could not drift; a
    /// GENERATED entry gets only what the descriptor declares, and
    /// `nros build freertos` emitted `use panic_semihosting as _;` into a
    /// manifest with no `panic-semihosting`. The failure was a compile error in
    /// a generated tree — far from the descriptor that caused it.
    ///
    /// Checks the SHIPPED catalog, and skips `use`s of the board's own crate
    /// (the emitter always depends on that one).
    #[test]
    fn every_crate_root_use_has_a_declared_dependency() {
        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("repo root")
            .to_path_buf();
        let catalog = BoardCatalog::load(&root)
            .unwrap_or_else(|e| panic!("shipped board catalog under {}: {e}", root.display()));
        let mut checked = 0;
        for d in catalog.descriptors() {
            let Some(entry) = d.entry.as_ref() else {
                continue;
            };
            for line in entry.crate_root_extra.lines() {
                let line = line.trim();
                let Some(rest) = line.strip_prefix("use ") else {
                    continue;
                };
                let krate = rest
                    .split([' ', ';', ':'])
                    .next()
                    .unwrap_or_default()
                    .trim();
                if krate.is_empty() || krate == entry.crate_name {
                    continue;
                }
                let dashed = krate.replace('_', "-");
                let declared = entry
                    .crate_root_deps
                    .iter()
                    .any(|dep| dep.split(['=', ' ']).next().unwrap_or_default().trim() == dashed);
                assert!(
                    declared,
                    "board {:?}: `crate_root_extra` uses `{krate}` but \
                     `crate_root_deps` does not declare `{dashed}`. A generated \
                     entry would emit the `use` with no dependency and fail to \
                     compile (phase-383 W9.b).",
                    d.names
                );
                checked += 1;
            }
        }
        assert!(
            checked > 0,
            "no crate-root `use` was checked — the descriptors moved and this \
             gate is now inert"
        );
    }
}

#[cfg(test)]
mod workspace_board_package_tests {
    use super::*;

    fn repo_root_for_tests() -> std::path::PathBuf {
        std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .ancestors()
            .nth(3)
            .expect("repo root")
            .to_path_buf()
    }

    /// A board declared BY the workspace, as one of its packages.
    ///
    /// Before this, a user's board had to live outside their workspace and be
    /// reached through `$NROS_EXTRA_BOARD_PATH` — ambient state, set per shell,
    /// invisible in the command that ran. Everything else a workspace needs is
    /// declared inside it; a board is not special.
    #[test]
    fn a_package_carrying_a_descriptor_joins_the_catalog() {
        let tmp = tempfile::tempdir().unwrap();
        let pkg = tmp.path().join("src").join("my_board");
        std::fs::create_dir_all(&pkg).unwrap();
        std::fs::write(
            pkg.join("nros-board.toml"),
            r#"
[[board]]
names = ["my-board"]
west_board = "qemu_cortex_m3"
platform = "zephyr"
toolchain = "stable"
platform_feature = "platform-zephyr"
link_kind = "none"
entry_kind = "zephyr-staticlib"
"#,
        )
        .unwrap();

        let cat =
            BoardCatalog::load_with_packages(&repo_root_for_tests(), std::slice::from_ref(&pkg))
                .expect("load");
        let d = cat
            .resolve("my-board", "")
            .expect("the workspace's own board");
        assert_eq!(d.platform, PlatformKind::Zephyr);
        // The name west is given, which is NOT the name the workspace uses.
        assert_eq!(d.west_board.as_deref(), Some("qemu_cortex_m3"));
    }

    /// A package with no descriptor is simply not a board — the common case,
    /// and it must cost nothing and raise nothing.
    #[test]
    fn a_package_without_a_descriptor_is_ignored() {
        let tmp = tempfile::tempdir().unwrap();
        let pkg = tmp.path().join("src").join("talker_pkg");
        std::fs::create_dir_all(&pkg).unwrap();

        let with =
            BoardCatalog::load_with_packages(&repo_root_for_tests(), std::slice::from_ref(&pkg))
                .expect("load");
        let without = BoardCatalog::load_with_extra(&repo_root_for_tests(), &[]).expect("load");
        assert_eq!(with.descriptors.len(), without.descriptors.len());
    }

    /// `west_board` is optional, and its absence means "the authored name is
    /// already the one west knows" — which is what every in-tree descriptor
    /// relies on, so a missing field must not change them.
    #[test]
    fn west_board_is_absent_on_the_in_tree_descriptors() {
        let cat = BoardCatalog::load_with_extra(&repo_root_for_tests(), &[]).expect("load");
        let zephyr = cat
            .resolve("native_sim/native/64", "")
            .expect("the zephyr board");
        assert_eq!(zephyr.west_board, None);
    }

    /// `deny_unknown_fields` is only worth having if it is exercised against
    /// the REAL descriptors, not a fixture. Six of them declare
    /// `[board.priority_plan]`, which no field modelled until this change — so
    /// a naive `deny_unknown_fields` would have rejected them and this test is
    /// what says so.
    #[test]
    fn every_in_tree_descriptor_parses_under_deny_unknown_fields() {
        let root = repo_root_for_tests();
        let cat = BoardCatalog::load_with_extra(&root, &[]).expect(
            "every packages/boards/*/nros-board.toml must parse; an unknown-field              error here names the key and the file",
        );
        assert!(
            cat.descriptors.len() >= 12,
            "expected the in-tree board descriptors, found {}",
            cat.descriptors.len()
        );
        assert!(
            cat.descriptors.iter().any(|d| d.priority_plan.is_some()),
            "at least one in-tree board declares [board.priority_plan]; if none              does, this test has stopped covering the case it exists for"
        );
    }

    /// The other half of the same claim: a key nobody reads must now FAIL, and
    /// fail naming itself. Before this change it parsed and vanished.
    #[test]
    fn an_unknown_board_key_is_rejected_and_names_itself() {
        let err = toml::from_str::<BoardFile>(
            "[[board]]\n\
             names = [\"demo\"]\n\
             platform = \"posix\"\n\
             toolchain = \"stable\"\n\
             platform_feature = \"platform-posix\"\n\
             link_kind = \"none\"\n\
             entry_kind = \"hosted-main\"\n\
             supported_netstakcs = []\n",
        )
        .expect_err("a misspelled key must not parse");
        let msg = err.to_string();
        assert!(
            msg.contains("supported_netstakcs"),
            "the error must name the offending key, got: {msg}"
        );
    }
}
