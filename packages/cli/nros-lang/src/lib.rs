//! The target-language enumeration, declared once.
//!
//! RFC-0091 §3 / phase-432 W2.1. Before this crate the enumeration existed
//! TWICE — `codegen::entry::Lang` and `orchestration::ComponentLanguage`, the
//! same three variants with no relationship between them — so a language added
//! to one was invisible to the other. Issue **#1062** is that already shipped:
//! two language readers disagreeing, "and the loser is a silent `C`".
//!
//! ## Why a crate of its own, with one dependency
//!
//! Placement is FORCED, not chosen. The consumers are `nros-cli-core`,
//! `rosidl-codegen` and — the binding one — `nros-macros`, the `nros::main!()`
//! proc-macro. `rosidl-codegen` does not depend on `nros-pkg-index`, and the
//! proc-macro depends on neither `nros-orchestration-ir` nor `nros-cli-core`:
//! issue **0083** removed its `nros-build` dependency precisely because that
//! pulled the whole planner and orchestration tree into every USER's entry
//! build. No existing crate is reachable by all three.
//!
//! So the dependency list is `serde` and nothing else, and it must stay that
//! way — a heavy dependency here is a dependency in every downstream user's
//! build, which is the force that created the duplication this crate removes.
//!
//! ## One enumeration, many narrowings
//!
//! This is the ENUMERATION. It deliberately does not absorb the types that
//! merely look like it, because they are different concerns wearing one word
//! (RFC-0091 §1):
//!
//! * `cmd::generate::Lang` adds `All`, which is a CLI affordance and not a
//!   language.
//! * `ComponentLang { Rust, Other }` is a binary predicate.
//! * `PayloadLang { Rust, C }` is a genuine NARROWING — only two emitters ask
//!   that question and `Cpp` is not a valid answer.
//!
//! A narrowing should DERIVE from this type rather than re-spell it.

#![forbid(unsafe_code)]
#![cfg_attr(not(feature = "std"), no_std)]

extern crate alloc;

use alloc::string::String;
use core::fmt;

/// A target language nano-ros generates code for.
///
/// The `snake_case` serde representation is a COMPATIBILITY SURFACE, not a
/// detail: `SourceMetadata` writes this field to disk, so the strings `rust`,
/// `c` and `cpp` are what already-written metadata files contain. Changing
/// them silently invalidates every one of them, which is why
/// `serde_repr_is_the_on_disk_contract` pins them.
#[derive(
    Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, serde::Serialize, serde::Deserialize,
)]
#[serde(rename_all = "snake_case")]
pub enum Language {
    Rust,
    C,
    Cpp,
}

impl Language {
    /// Every language, in a stable order.
    ///
    /// Exists so a consumer that must handle all of them — a CLI `All`, a
    /// golden matrix — iterates rather than listing, and therefore cannot go
    /// stale when a variant is added.
    pub const ALL: [Language; 3] = [Language::Rust, Language::C, Language::Cpp];

    /// The canonical spelling — the same string the serde repr uses.
    pub fn as_str(self) -> &'static str {
        match self {
            Language::Rust => "rust",
            Language::C => "c",
            Language::Cpp => "cpp",
        }
    }

    /// Parse a user-supplied language name.
    ///
    /// Accepts the aliases a CLI must accept (`c++`, `cxx`) alongside the
    /// canonical spellings. Returns the offending input in the error so the
    /// caller can phrase its own message — this crate does not depend on an
    /// error library, and should not acquire one.
    pub fn parse(s: &str) -> Result<Self, UnknownLanguage> {
        match s {
            "rust" => Ok(Language::Rust),
            "c" => Ok(Language::C),
            "cpp" | "c++" | "cxx" => Ok(Language::Cpp),
            other => Err(UnknownLanguage {
                input: String::from(other),
            }),
        }
    }
}

impl fmt::Display for Language {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// `Language::parse` was given something that is not a language.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UnknownLanguage {
    pub input: String,
}

impl fmt::Display for UnknownLanguage {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "unknown language `{}` (expected one of: rust, c, cpp)",
            self.input
        )
    }
}

#[cfg(feature = "std")]
impl std::error::Error for UnknownLanguage {}

#[cfg(test)]
mod tests {
    use alloc::{format, string::ToString};

    use super::*;

    /// The on-disk contract. `SourceMetadata.language` is serialized into
    /// metadata files that already exist on users' disks, so these three
    /// strings are not a naming choice — changing one invalidates every file
    /// written before the change, silently, because `deny_unknown_fields` is
    /// about KEYS and an unknown variant fails at the value.
    #[test]
    fn serde_repr_is_the_on_disk_contract() {
        for (lang, spelling) in [
            (Language::Rust, "\"rust\""),
            (Language::C, "\"c\""),
            (Language::Cpp, "\"cpp\""),
        ] {
            let encoded = serde_json::to_string(&lang).expect("serialize");
            assert_eq!(encoded, spelling, "{lang:?} changed its on-disk spelling");
            let decoded: Language = serde_json::from_str(spelling).expect("deserialize");
            assert_eq!(decoded, lang);
        }
    }

    /// `as_str` and the serde repr must not drift apart — they are two
    /// spellings of one fact, which is the defect this crate exists to remove.
    #[test]
    fn as_str_agrees_with_serde() {
        for lang in Language::ALL {
            let encoded = serde_json::to_string(&lang).expect("serialize");
            assert_eq!(encoded, format!("\"{}\"", lang.as_str()));
        }
    }

    #[test]
    fn parse_accepts_the_aliases_a_cli_must_accept() {
        assert_eq!(Language::parse("c++").unwrap(), Language::Cpp);
        assert_eq!(Language::parse("cxx").unwrap(), Language::Cpp);
        assert_eq!(Language::parse("cpp").unwrap(), Language::Cpp);
        assert_eq!(Language::parse("rust").unwrap(), Language::Rust);
        assert_eq!(Language::parse("c").unwrap(), Language::C);
    }

    /// The error names what was passed. A parse failure whose message does not
    /// quote the input makes a typo in a build script unreadable.
    #[test]
    fn parse_rejects_and_names_the_input() {
        let err = Language::parse("zig").expect_err("not a language yet");
        assert_eq!(err.input, "zig");
        assert!(err.to_string().contains("zig"), "{err}");
    }

    /// `ALL` must stay exhaustive. A variant added without extending it makes
    /// every consumer that iterates silently skip the new language — the exact
    /// failure mode (a language invisible to one reader) this crate removes.
    #[test]
    fn all_is_exhaustive() {
        for lang in Language::ALL {
            // Exhaustive match: adding a variant fails to compile here first.
            let named = match lang {
                Language::Rust => "rust",
                Language::C => "c",
                Language::Cpp => "cpp",
            };
            assert_eq!(named, lang.as_str());
        }
        assert_eq!(Language::ALL.len(), 3);
    }
}
