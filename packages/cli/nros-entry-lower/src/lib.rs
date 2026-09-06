//! Stage 2 of the ENTRY pipeline: the facts every language pack renders from.
//!
//! RFC-0091 §4 / phase-432 W2.2. The entry emitters each derived these
//! independently — and the `nros::main!()` proc-macro derived them a third
//! time, because it cannot depend on `nros-cli-core` (issue 0083: that pulled
//! the whole planner into every USER's entry build).
//!
//! ## The dependency budget is the design constraint
//!
//! This crate exists to be adoptable by that proc-macro. Its dependency list
//! is `serde` today and may grow only to what the macro already accepts
//! (`nros-pkg-index`, `nros-launch-parser`, `nros-orchestration-ir`). A heavy
//! dependency here lands in every downstream user's build, which is the force
//! that produced the duplication this crate removes. `eyre` in particular
//! stays out: a leaf carries a plain error type, as `nros-lang` does.
//!
//! ## What belongs here, and what does not
//!
//! COMPUTATION belongs here: which family a board key names, which boot shape
//! that family has, the encodings the ABI fixes. SPELLING does not — RFC-0091
//! §8b found the first draft leaking C++ into this stage as a `board_path`
//! like `::nros::board::LinuxBoard`, which a pure-C or Zig pack cannot use.
//! The neutral fact is the board's IDENTITY; how that becomes a call is the
//! pack's business.

#![forbid(unsafe_code)]
#![no_std]

/// The board families the entry pipeline distinguishes.
///
/// Nineteen board keys collapse onto five families. The KEY is what a user
/// writes and what cmake passes; the FAMILY is what the lowering reasons
/// about, and what a pack turns into a call.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
#[serde(rename_all = "snake_case")]
pub enum BoardFamily {
    /// The host build (`native`, `posix`) — and the fallback for an
    /// unrecognised key, see [`board_family`].
    Native,
    Zephyr,
    Nuttx,
    Freertos,
    Threadx,
}

/// The boot wrapper a generated entry gets.
///
/// Derived from the family, once. It used to be spelled separately in the
/// per-tier and single-executor paths of `emit_cpp`, and the two spellings
/// tested DIFFERENT predicates — they agreed only because a condition seventy
/// lines away excluded the one board they disagree about.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BootShape {
    /// The kernel calls `main(void)` directly (Zephyr).
    Kernel,
    /// The board's `startup.c` owns `main` and dispatches to `nros_app_main`.
    App,
    /// A host process keeping the POSIX `int main(argc, argv)`.
    Host,
}

impl BoardFamily {
    /// Every family, in a stable order — so a consumer that must handle all of
    /// them iterates rather than lists, and cannot go stale.
    pub const ALL: [BoardFamily; 5] = [
        BoardFamily::Native,
        BoardFamily::Zephyr,
        BoardFamily::Nuttx,
        BoardFamily::Freertos,
        BoardFamily::Threadx,
    ];

    /// Does this family boot through the board's own `startup.c`?
    ///
    /// Everything except the host: the board owns `main` and dispatches to the
    /// entry's `app_main`, so a generated `int main` would be a second `main`
    /// in an image whose board already defines one. Zephyr is embedded but the
    /// KERNEL calls `main(void)`, which is why boot shape is three-valued and
    /// not this predicate.
    pub fn is_embedded(self) -> bool {
        self != BoardFamily::Native
    }

    /// The boot wrapper this family's entry needs.
    pub fn boot_shape(self) -> BootShape {
        match self {
            BoardFamily::Zephyr => BootShape::Kernel,
            BoardFamily::Native => BootShape::Host,
            BoardFamily::Nuttx | BoardFamily::Freertos | BoardFamily::Threadx => BootShape::App,
        }
    }

    /// The family's canonical name — the same string the serde repr uses.
    pub fn as_str(self) -> &'static str {
        match self {
            BoardFamily::Native => "native",
            BoardFamily::Zephyr => "zephyr",
            BoardFamily::Nuttx => "nuttx",
            BoardFamily::Freertos => "freertos",
            BoardFamily::Threadx => "threadx",
        }
    }
}

/// Which family a board key names.
///
/// An unrecognised key falls back to [`BoardFamily::Native`], preserving the
/// behaviour the C++ emitter had: cmake has already errored on a missing
/// `BOARD` for a non-native `DEPLOY` by the time codegen runs, so this
/// fallback exists to let unit tests cover the unhappy path without teaching
/// the lowering every embedded board prematurely. It is a deliberate choice
/// and not a silent default — a NEW board key that reaches production without
/// a row here builds as a host image, which fails loudly at link.
pub fn board_family(board: &str) -> BoardFamily {
    match board {
        "native" | "posix" => BoardFamily::Native,
        // Every Phase 215 Zephyr board (FVP, qemu-zephyr, …) compiles with
        // `__ZEPHYR__` and shares the one metadata-driven adapter.
        "zephyr" | "fvp-aemv8r-smp" | "armfvp" => BoardFamily::Zephyr,
        // Phase 238 — network is up at kernel boot; shares the lifecycle
        // adapter.
        "nuttx" | "nuttx-qemu-arm" | "nuttx-qemu-riscv" => BoardFamily::Nuttx,
        // Phase 240.6 / phase-263 C2b, plus phase-370's `freertos-posix` and
        // phase-372/385's S32Z270 and MPS3-AN536. `freertos-posix` is a HOST
        // process whose nodes still run as FreeRTOS TASKS, so it belongs here
        // and not with `native`: it once fell through to the host default,
        // which is a silent wrong answer that only surfaced at link when
        // `app_main` came up undefined.
        "freertos"
        | "mps2-an385-freertos"
        | "freertos-posix"
        | "s32z270-freertos"
        | "s32z270"
        | "mps3-an536-freertos"
        | "an536" => BoardFamily::Freertos,
        // Phase 246 — Azure RTOS ThreadX (host sim + bare-metal qemu-riscv64).
        "threadx" | "threadx-linux" | "threadx-qemu-riscv64" | "qemu-riscv64-threadx" => {
            BoardFamily::Threadx
        }
        _ => BoardFamily::Native,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every key the C++ emitter recognised, and the family it must land in.
    /// This is the table the emitters used to each carry a copy of.
    const KEYS: &[(&str, BoardFamily)] = &[
        ("native", BoardFamily::Native),
        ("posix", BoardFamily::Native),
        ("zephyr", BoardFamily::Zephyr),
        ("fvp-aemv8r-smp", BoardFamily::Zephyr),
        ("armfvp", BoardFamily::Zephyr),
        ("nuttx", BoardFamily::Nuttx),
        ("nuttx-qemu-arm", BoardFamily::Nuttx),
        ("nuttx-qemu-riscv", BoardFamily::Nuttx),
        ("freertos", BoardFamily::Freertos),
        ("mps2-an385-freertos", BoardFamily::Freertos),
        ("freertos-posix", BoardFamily::Freertos),
        ("s32z270-freertos", BoardFamily::Freertos),
        ("s32z270", BoardFamily::Freertos),
        ("mps3-an536-freertos", BoardFamily::Freertos),
        ("an536", BoardFamily::Freertos),
        ("threadx", BoardFamily::Threadx),
        ("threadx-linux", BoardFamily::Threadx),
        ("threadx-qemu-riscv64", BoardFamily::Threadx),
        ("qemu-riscv64-threadx", BoardFamily::Threadx),
    ];

    #[test]
    fn every_known_board_key_lands_in_its_family() {
        for (key, want) in KEYS {
            assert_eq!(board_family(key), *want, "board key `{key}`");
        }
        assert_eq!(KEYS.len(), 19, "a key was added or removed without a row");
    }

    /// `freertos-posix` is the trap this table exists to hold. It is a host
    /// PROCESS, so it reads like `native` — but its nodes run as FreeRTOS
    /// tasks and its `startup.c` owns `main`, so a host `int main` compiles
    /// and then fails at link on an undefined `app_main`.
    #[test]
    fn freertos_posix_is_freertos_not_native() {
        assert_eq!(board_family("freertos-posix"), BoardFamily::Freertos);
        assert_eq!(
            board_family("freertos-posix").boot_shape(),
            BootShape::App,
            "a host `int main` here is a second main"
        );
    }

    /// The boot shape is three-valued for a reason: Zephyr is embedded AND
    /// takes the host-looking `main(void)`, so `is_embedded` cannot decide it.
    #[test]
    fn zephyr_is_embedded_but_boots_through_main() {
        assert!(BoardFamily::Zephyr.is_embedded());
        assert_eq!(BoardFamily::Zephyr.boot_shape(), BootShape::Kernel);
        assert_ne!(BoardFamily::Zephyr.boot_shape(), BootShape::Host);
    }

    /// ThreadX is the board the two former spellings in `emit_cpp` disagreed
    /// about — one tested `freertos || nuttx`, the other `is_embedded`. Pinned
    /// here so the one derivation stays right about it on its own terms.
    #[test]
    fn threadx_boots_through_the_board_not_a_host_main() {
        assert_eq!(BoardFamily::Threadx.boot_shape(), BootShape::App);
        assert_eq!(
            BoardFamily::Threadx.boot_shape(),
            BoardFamily::Nuttx.boot_shape()
        );
    }

    #[test]
    fn an_unknown_key_falls_back_to_native() {
        assert_eq!(board_family("zigos"), BoardFamily::Native);
        assert_eq!(board_family(""), BoardFamily::Native);
    }

    #[test]
    fn only_native_is_not_embedded() {
        for f in BoardFamily::ALL {
            assert_eq!(
                f.is_embedded(),
                f != BoardFamily::Native,
                "{} embeddedness",
                f.as_str()
            );
        }
    }

    #[test]
    fn all_is_exhaustive() {
        for f in BoardFamily::ALL {
            // Exhaustive match: a new family fails to compile here first.
            let named = match f {
                BoardFamily::Native => "native",
                BoardFamily::Zephyr => "zephyr",
                BoardFamily::Nuttx => "nuttx",
                BoardFamily::Freertos => "freertos",
                BoardFamily::Threadx => "threadx",
            };
            assert_eq!(named, f.as_str());
        }
        assert_eq!(BoardFamily::ALL.len(), 5);
    }
}
