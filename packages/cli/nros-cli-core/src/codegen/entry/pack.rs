//! phase-432 W3.2 — the entry pack manifests, and the one routing decision.
//!
//! ## What a manifest is for
//!
//! CMake has to know two things about an entry pack BEFORE codegen runs,
//! because they decide the output path and the compiler: the generated TU's
//! extension, and whether a C-family toolchain builds it. It used to derive
//! both itself, in `NanoRosEntry.cmake`:
//!
//! ```text
//! if(_NRX_LANG STREQUAL "c" AND NOT NANO_ROS_PLATFORM STREQUAL "posix")
//!     set(_ext "cpp")   # an embedded C entry is rendered as C++
//! ```
//!
//! while the CLI made the same decision from a different input:
//!
//! ```text
//! Lang::C if !board_is_embedded(&plan.board) => emit_c,
//! _                                          => emit_cpp,
//! ```
//!
//! One keyed on `NANO_ROS_PLATFORM`, the other on the board key `plan.board`
//! carries (from the bringup's `system.toml`). Those are related and not equal.
//!
//! **No live disagreement was found**, and that is worth saying plainly rather
//! than dressing the change up: every (platform, board) pair the tree
//! exercises gives the same answer both ways. `freertos-posix` looks like the
//! counterexample — a HOST process whose board family is `Freertos` — and is
//! not, because its fixture rows set `NANO_ROS_PLATFORM = "freertos"`, so both
//! sides say embedded.
//!
//! What is wrong is structural: nothing MAKES them agree. They read different
//! inputs, `board_family` answers `Native` for any key it has not learned, and
//! the failure if they ever diverged is a C++ TU written into a `.c` file — a
//! compile error at best, and at worst a `.c` file that happens to compile.
//! The `freertos-posix` comment in `board_family` records that this table has
//! already produced one silent wrong answer of exactly that kind.
//!
//! So the manifest holds the DATA and this module holds the DECISION, and both
//! answers are served to CMake through `nros codegen entry-pack` — one
//! producer, asked rather than re-derived.
//!
//! ## What is deliberately NOT in a manifest
//!
//! Which pack renders for a given (language, board) is a rule with a reason —
//! the RTOS board runners are C++ only, so an embedded C entry must be
//! rendered as C++ — and a rule belongs in reviewed Rust, not in data a
//! template author can edit. RFC-0091 draws the same line for target facts.

use std::collections::BTreeMap;

use nros_lang::Language;

/// One pack's manifest, as declared in `packs/entry/<surface>/pack.toml`.
#[derive(Debug, Clone, PartialEq, Eq, serde::Deserialize, serde::Serialize)]
pub struct PackManifest {
    /// Absent for the shared partial pack, which renders no TU of its own.
    #[serde(default)]
    pub language: Option<Language>,
    /// True only for the shared pack. The two kinds are distinguished by a
    /// declared field rather than by a directory name, so a new shared pack
    /// cannot be created by accident.
    #[serde(default)]
    pub shared: bool,
    #[serde(default)]
    pub extension: Option<String>,
    #[serde(default)]
    pub c_family: Option<bool>,
    #[serde(default)]
    pub entry_template: Option<String>,
    #[serde(default)]
    pub partials: Vec<String>,
}

/// Every manifest, bundled at build time.
///
/// `include_str!` rather than a directory read, for the same reason the
/// templates are bundled: `nros` runs from CMake and from build scripts in
/// trees that do not contain this checkout.
const MANIFESTS: &[(&str, &str)] = &[
    ("c", include_str!("packs/entry/c/pack.toml")),
    ("cpp", include_str!("packs/entry/cpp/pack.toml")),
    ("rust", include_str!("packs/entry/rust/pack.toml")),
    ("shared", include_str!("packs/entry/shared/pack.toml")),
];

/// Parse every manifest, keyed by surface directory name.
///
/// A malformed manifest is a bug in a file that ships INSIDE the binary, so it
/// surfaces as a plain message rather than being folded into a caller's error
/// vocabulary — the same contract `render::render` uses.
pub fn manifests() -> Result<BTreeMap<&'static str, PackManifest>, String> {
    let mut out = BTreeMap::new();
    for (surface, text) in MANIFESTS {
        let m: PackManifest = toml::from_str(text)
            .map_err(|e| format!("bundled pack manifest `{surface}/pack.toml` is invalid: {e}"))?;
        out.insert(*surface, m);
    }
    Ok(out)
}

/// What CMake needs to know about the pack that will render this entry.
#[derive(Debug, Clone, PartialEq, Eq, serde::Serialize)]
pub struct EntryPackInfo {
    /// The surface directory whose pack renders it — NOT necessarily the
    /// language asked for: an embedded C entry is rendered by the C++ pack.
    pub pack: String,
    /// The generated TU's extension.
    pub extension: String,
    /// Whether a C-family compiler builds it.
    pub c_family: bool,
    /// True when the requested language is not the rendering pack's, i.e. the
    /// routing rule fired. Reported so a caller can SAY so rather than having
    /// to notice the extension changed.
    pub routed: bool,
}

/// The one routing decision: which pack renders `language` on `board`.
///
/// The rule and its reason: the RTOS board runners
/// (`ThreadxBoard::run_components` and siblings) are C++ only, so an embedded
/// C entry is rendered by the C++ pack, which drives the C++ runner and calls
/// each C node through its `extern "C"` seam. Native C stays C.
///
/// W3.1 is the item that would delete this branch, by giving every board a
/// C-ABI `run_components`.
pub fn entry_pack_for(language: Language, board: &str) -> Result<EntryPackInfo, String> {
    let embedded = nros_entry_lower::board_family(board).is_embedded();
    let pack = match language {
        Language::C if embedded => "cpp",
        Language::C => "c",
        Language::Cpp => "cpp",
        Language::Rust => "rust",
    };
    let all = manifests()?;
    let m = all
        .get(pack)
        .ok_or_else(|| format!("no pack manifest for surface `{pack}`"))?;
    let extension = m
        .extension
        .clone()
        .ok_or_else(|| format!("pack `{pack}` declares no extension"))?;
    let c_family = m
        .c_family
        .ok_or_else(|| format!("pack `{pack}` declares no c_family"))?;
    Ok(EntryPackInfo {
        pack: pack.to_string(),
        extension,
        c_family,
        routed: m.language != Some(language),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every bundled manifest parses, and declares the fields its KIND
    /// requires. A language pack without an extension is a pack CMake cannot
    /// name an output for; a shared pack WITH one is a pack that looks
    /// renderable and is not.
    #[test]
    fn every_manifest_declares_its_kind_completely() {
        let all = manifests().expect("bundled manifests must parse");
        assert_eq!(all.len(), MANIFESTS.len());
        for (surface, m) in &all {
            if m.shared {
                assert!(
                    m.language.is_none()
                        && m.extension.is_none()
                        && m.c_family.is_none()
                        && m.entry_template.is_none(),
                    "shared pack `{surface}` declares a language pack's fields — the \
                     two kinds must not be confusable"
                );
                assert!(
                    !m.partials.is_empty(),
                    "shared pack `{surface}` declares no partials, so it is nothing"
                );
            } else {
                assert!(
                    m.language.is_some()
                        && m.extension.is_some()
                        && m.c_family.is_some()
                        && m.entry_template.is_some(),
                    "language pack `{surface}` is missing a required field — CMake \
                     cannot name its output or pick its compiler"
                );
            }
        }
    }

    /// Every template a manifest names is registered, and every registered
    /// template is named by some manifest.
    ///
    /// This is the half that makes the manifests LOAD-BEARING rather than
    /// decorative: without it a manifest could name a template that does not
    /// exist, or omit one that does, and nothing would notice until a render
    /// failed on someone's build.
    #[test]
    fn manifests_and_the_registry_describe_the_same_templates() {
        use std::collections::BTreeSet;

        let all = manifests().expect("manifests parse");
        let mut declared: BTreeSet<String> = BTreeSet::new();
        for m in all.values() {
            if let Some(t) = &m.entry_template {
                declared.insert(t.clone());
            }
            declared.extend(m.partials.iter().cloned());
        }
        let registered: BTreeSet<String> = super::super::render::template_keys()
            .into_iter()
            .map(str::to_string)
            .collect();
        assert_eq!(
            declared, registered,
            "the manifests and the registry disagree about which templates exist"
        );
    }

    /// The routing rule, stated as cases rather than re-derived.
    #[test]
    fn an_embedded_c_entry_is_rendered_by_the_cpp_pack() {
        let native = entry_pack_for(Language::C, "native").unwrap();
        assert_eq!(
            (native.pack.as_str(), native.extension.as_str()),
            ("c", "c")
        );
        assert!(!native.routed);

        for board in ["zephyr", "nuttx", "freertos", "threadx"] {
            let got = entry_pack_for(Language::C, board).unwrap();
            assert_eq!(
                (got.pack.as_str(), got.extension.as_str()),
                ("cpp", "cpp"),
                "an embedded C entry on `{board}` must render as C++ — the RTOS \
                 board runners are C++ only"
            );
            assert!(got.routed, "`{board}` must report that routing fired");
        }
    }

    /// A Rust entry is not a C-family TU, so CMake must not link it as one.
    #[test]
    fn a_rust_entry_is_not_c_family() {
        let got = entry_pack_for(Language::Rust, "native").unwrap();
        assert!(!got.c_family);
        assert_eq!(got.extension, "rs");
    }
}
