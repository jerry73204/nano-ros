//! RFC-0091 §6b / phase-432 W2.5b — **a language contributes a FILTER SET.**
//!
//! A pack is `.jinja` text and a serializable context; the only Rust a language
//! adds is the handful of filters that turn a neutral fact into that language's
//! syntax. `c_type` takes a [`CTypeSpell`] and returns a C spelling — that IS
//! the neutral-fact-to-language-syntax boundary, and RFC-0091 §6b names it as
//! the model the rest of the pipeline should follow.
//!
//! Until this module the ten filters were registered flat into one
//! `Environment`, so the code said nothing about which language owned which.
//! "Adding a language = a pack plus a filter set" was a claim with no place to
//! put the second half. Here it has one: [`FILTER_SETS`], one entry per
//! [`Language`] plus the neutral set, and [`for_language`] answers the question
//! a newcomer actually asks.
//!
//! ## What "owns" means here
//!
//! A set is keyed by the language whose PACKS call the filters — the language
//! you are adding. It is deliberately not keyed by the syntax a filter emits,
//! and the difference is real rather than pedantic:
//! [`crate::types::cpp_repr_c_type_spelling`] and its `view` sibling return
//! **Rust** (`[u8; 32]`, `u32`, `nros_cpp_heap_str_t`), because the `cpp` pack
//! emits a `repr(C)` Rust mirror beside its header — RFC-0091 §6's "the `cpp`
//! pack emits Rust", visible in the filter names. Keying by emitted syntax
//! would file those two under Rust, and then adding a language that needed its
//! own FFI mirror would grow the *Rust* set, which is exactly backwards for the
//! procedure this module exists to make honest.
//!
//! ## The neutral set
//!
//! `snake_case` is the one filter that belongs to no language. It implements
//! the ROS naming convention (`PoseStamped` -> `pose_stamped`), and the proof
//! that it is neutral is by USE, not by assertion: the same
//! [`crate::utils::to_snake_case`] builds C header names in
//! `generator/srv.rs`, C++ header names in `generator/cpp.rs`, and Rust
//! constant names in the `nros` pack. One convention, three languages.
//!
//! ## What holds the set together
//!
//! Four tests at the bottom of this file, in both directions:
//!
//! * every [`Language`] has a set (adding a variant to `nros-lang` fails here);
//! * every name a set DECLARES is one its `register` actually adds;
//! * every filter a bundled pack CALLS is declared by some set — the check that
//!   would otherwise happen at render time, i.e. at a user's build;
//! * every declared filter is called by some pack — no dead spellings.

use minijinja::{Environment, value::ViaDeserialize};
use nros_lang::Language;

/// The neutral facts a C field carries for the C-type pack filters (RFC-0068
/// step 2). Deserialized from the field's serialized form; extra keys are
/// ignored.
#[derive(serde::Deserialize)]
struct CTypeSpell {
    field_type: rosidl_parser::FieldType,
    is_configurable: bool,
    is_heap: bool,
    cap: usize,
    current_package: String,
}

/// Neutral facts the Rust-type pack filters (`rust_type_rmw` /
/// `rust_type_idiomatic`) compose the Rust type string from. `current_package`
/// drives the self-ref (`crate::` vs `pkg::`) choice inside
/// `rust_type_for_field`.
#[derive(serde::Deserialize)]
struct RustTypeSpell {
    field_type: rosidl_parser::FieldType,
    current_package: String,
}

impl RustTypeSpell {
    fn pkg(&self) -> Option<&str> {
        (!self.current_package.is_empty()).then_some(self.current_package.as_str())
    }
}

/// Neutral facts the `cpp_type` / `cpp_array_suffix` pack filters compose the
/// C++ header type from.
#[derive(serde::Deserialize)]
struct CppTypeSpell {
    field_type: rosidl_parser::FieldType,
    is_borrowed: bool,
    is_heap: bool,
    cap: Option<usize>,
    current_package: String,
}

/// Neutral facts the `cpp_repr_c_type` / `cpp_view_repr_type` pack filters
/// compose the C++ FFI Rust `repr(C)` type from.
#[derive(serde::Deserialize)]
struct CppReprSpell {
    field_type: rosidl_parser::FieldType,
    struct_name: String,
    cap: Option<usize>,
    current_package: String,
    name: String,
    is_string: bool,
    is_sequence: bool,
    is_heap: bool,
    is_borrowed: bool,
}

impl CppReprSpell {
    fn pkg(&self) -> Option<&str> {
        (!self.current_package.is_empty()).then_some(self.current_package.as_str())
    }
}

/// Neutral facts the `nros_type` pack filter composes the nros Rust type from.
#[derive(serde::Deserialize)]
struct NrosTypeSpell {
    field_type: rosidl_parser::FieldType,
    is_configurable: bool,
    is_heap: bool,
    cap: usize,
    mode: crate::types::NrosCodegenMode,
    current_package: String,
}

/// One language's Rust surface area in the codegen — a named set of filters and
/// the function that registers them.
///
/// `filters` is authored beside `register` on purpose and the two are checked
/// against each other (`declared_names_are_the_registered_ones`): an authored
/// list nobody compares is the drift this repo has been bitten by before.
pub struct FilterSet {
    /// The language whose packs call these filters, or `None` for the neutral
    /// set that every pack may call.
    pub language: Option<Language>,
    /// A short human name for the set, used in diagnostics.
    pub name: &'static str,
    /// Every filter name this set registers, in registration order.
    pub filters: &'static [&'static str],
    /// Registers `filters` on an environment.
    pub register: fn(&mut Environment<'_>),
}

/// The one filter that belongs to no language: the ROS `CamelCase` ->
/// `snake_case` naming convention, shared by the C, C++ and Rust surfaces.
const NEUTRAL: FilterSet = FilterSet {
    language: None,
    name: "neutral",
    filters: &["snake_case"],
    register: |env| {
        env.add_filter("snake_case", |s: &str| crate::utils::to_snake_case(s));
    },
};

/// The C pack's spellings (`packs/c`). RFC-0068 step 2 moved these out of the
/// pre-baked `CField.c_type` / `.array_suffix`.
const C: FilterSet = FilterSet {
    language: Some(Language::C),
    name: "c",
    filters: &["c_type", "c_array_suffix"],
    register: |env| {
        env.add_filter("c_type", |v: ViaDeserialize<CTypeSpell>| {
            let c = &v.0;
            let cp = (!c.current_package.is_empty()).then_some(c.current_package.as_str());
            crate::types::c_type_spelling(&c.field_type, c.is_configurable, c.is_heap, c.cap, cp)
        });
        env.add_filter("c_array_suffix", |v: ViaDeserialize<CTypeSpell>| {
            let c = &v.0;
            crate::types::c_array_suffix_spelling(
                &c.field_type,
                c.is_configurable,
                c.is_heap,
                c.cap,
            )
        });
    },
};

/// The C++ pack's spellings (`packs/cpp`). Four, not two: the pack emits a
/// header AND the `repr(C)` Rust mirror the C++ FFI glue needs, so
/// `cpp_repr_c_type` / `cpp_view_repr_type` return Rust syntax while belonging
/// to the C++ pack (see the module docs).
const CPP: FilterSet = FilterSet {
    language: Some(Language::Cpp),
    name: "cpp",
    filters: &[
        "cpp_type",
        "cpp_array_suffix",
        "cpp_repr_c_type",
        "cpp_view_repr_type",
    ],
    register: |env| {
        env.add_filter("cpp_type", |v: ViaDeserialize<CppTypeSpell>| {
            let c = &v.0;
            let cp = (!c.current_package.is_empty()).then_some(c.current_package.as_str());
            crate::types::cpp_type_spelling(&c.field_type, c.is_borrowed, c.is_heap, c.cap, cp)
        });
        env.add_filter("cpp_array_suffix", |v: ViaDeserialize<CppTypeSpell>| {
            crate::types::cpp_array_suffix_spelling(&v.0.field_type, v.0.is_borrowed)
        });
        env.add_filter("cpp_repr_c_type", |v: ViaDeserialize<CppReprSpell>| {
            let c = &v.0;
            crate::types::cpp_repr_c_type_spelling(
                &c.field_type,
                c.is_sequence,
                c.is_heap,
                c.is_string,
                c.cap,
                &c.struct_name,
                &c.name,
                c.pkg(),
            )
        });
        env.add_filter("cpp_view_repr_type", |v: ViaDeserialize<CppReprSpell>| {
            let c = &v.0;
            crate::types::cpp_view_repr_type_spelling(
                &c.field_type,
                c.is_borrowed,
                c.is_sequence,
                c.is_heap,
                c.is_string,
                c.cap,
                &c.struct_name,
                &c.name,
                c.pkg(),
            )
        });
    },
};

/// The Rust packs' spellings. Three filters for three SURFACES of one language
/// (RFC-0091 §6): the ROS-ABI mirror (`packs/rmw`), the idiomatic binding
/// (`packs/rust`) and the embedded `nros` one (`packs/nros`), whose storage
/// decision and codegen mode make it a different spelling of the same field.
const RUST: FilterSet = FilterSet {
    language: Some(Language::Rust),
    name: "rust",
    filters: &["rust_type_rmw", "rust_type_idiomatic", "nros_type"],
    register: |env| {
        env.add_filter("rust_type_rmw", |v: ViaDeserialize<RustTypeSpell>| {
            crate::types::rust_type_for_field(&v.0.field_type, true, v.0.pkg())
        });
        env.add_filter("rust_type_idiomatic", |v: ViaDeserialize<RustTypeSpell>| {
            crate::types::rust_type_for_field(&v.0.field_type, false, v.0.pkg())
        });
        env.add_filter("nros_type", |v: ViaDeserialize<NrosTypeSpell>| {
            let n = &v.0;
            let cp = (!n.current_package.is_empty()).then_some(n.current_package.as_str());
            crate::types::nros_type_spelling(
                &n.field_type,
                n.is_configurable,
                n.is_heap,
                n.cap,
                n.mode,
                cp,
            )
        });
    },
};

/// Every filter set: the neutral one, then one per [`Language`].
///
/// This is THE registry. A language missing from it fails
/// `every_language_contributes_a_filter_set`, which is the whole point — the
/// set has a declared home instead of being ten flat `add_filter` calls.
pub const FILTER_SETS: &[FilterSet] = &[NEUTRAL, C, CPP, RUST];

/// The filter set a language contributes, if it has one.
pub fn for_language(language: Language) -> Option<&'static FilterSet> {
    FILTER_SETS.iter().find(|s| s.language == Some(language))
}

/// The neutral set — the filters no language owns and every pack may call.
pub fn neutral() -> &'static FilterSet {
    FILTER_SETS
        .iter()
        .find(|s| s.language.is_none())
        .expect("the neutral set is a const in this module")
}

/// Every filter name the renderer registers, across all sets.
pub fn all_filter_names() -> impl Iterator<Item = &'static str> {
    FILTER_SETS.iter().flat_map(|s| s.filters.iter().copied())
}

/// Register every set on an environment. The renderer's one call site.
pub fn register_all(env: &mut Environment<'_>) {
    for set in FILTER_SETS {
        (set.register)(env);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Names a bundled pack passes through a `|` filter, found with minijinja's
    /// OWN lexer rather than a regex.
    ///
    /// That distinction is load-bearing: the packs emit Rust and C++, so their
    /// raw text is full of `|item|` closures and `a || b`. A textual scan reads
    /// those as filter calls. The tokenizer only leaves template-expression
    /// regions as tokens — everything else arrives as one `TemplateData` blob —
    /// so `Pipe` followed by `Ident` is exactly a filter call and nothing else.
    fn filters_called_by(source: &str) -> Vec<String> {
        use minijinja::machinery::{Token, tokenize};

        let mut found = Vec::new();
        let mut after_pipe = false;
        for tok in tokenize(source, false, Default::default(), Default::default()) {
            let (tok, _span) = tok.expect("bundled pack must tokenize");
            match tok {
                Token::Pipe => after_pipe = true,
                Token::Ident(name) if after_pipe => {
                    found.push(name.to_string());
                    after_pipe = false;
                }
                _ => after_pipe = false,
            }
        }
        found
    }

    /// Is `name` resolvable as a filter on `env`?
    ///
    /// minijinja has no `get_filter`, and it resolves filters at RENDER time —
    /// which is the defect this file's checks exist to move earlier. So probe:
    /// render a one-filter expression and look at the error KIND. A registered
    /// filter that dislikes the argument fails some other way; only an absent
    /// one is `UnknownFilter`.
    fn resolves(env: &Environment<'_>, name: &str) -> bool {
        match env.render_str(&format!("{{{{ 1|{name} }}}}"), ()) {
            Ok(_) => true,
            Err(e) => e.kind() != minijinja::ErrorKind::UnknownFilter,
        }
    }

    /// The addressability this wave is for. A `Language` variant added to
    /// `nros-lang` with no filter set fails HERE, in the crate that would
    /// otherwise discover it as a missing-filter render error at a user's
    /// build.
    #[test]
    fn every_language_contributes_a_filter_set() {
        for language in Language::ALL {
            let set = for_language(language)
                .unwrap_or_else(|| panic!("language `{language}` contributes no filter set"));
            assert!(
                !set.filters.is_empty(),
                "language `{language}` declares an empty filter set"
            );
        }
        // And no set claims a language twice.
        let mut seen = Vec::new();
        for set in FILTER_SETS {
            assert!(
                !seen.contains(&set.language),
                "two filter sets claim {:?}",
                set.language
            );
            seen.push(set.language);
        }
    }

    /// The authored list and the `register` body are two spellings of one fact,
    /// so they are compared. Each set is registered ALONE, so a name that
    /// resolves only because a sibling set happened to add it fails here.
    #[test]
    fn declared_names_are_the_registered_ones() {
        for set in FILTER_SETS {
            let mut env = Environment::new();
            (set.register)(&mut env);
            for name in set.filters {
                assert!(
                    resolves(&env, name),
                    "set `{}` declares `{name}` but its register does not add it",
                    set.name
                );
            }
        }
    }

    /// No two sets register the same name — a duplicate would mean the LAST
    /// registration silently wins and one language's spelling reaches the
    /// other's pack.
    #[test]
    fn filter_names_are_unique_across_sets() {
        let mut seen: Vec<&str> = Vec::new();
        for name in all_filter_names() {
            assert!(
                !seen.contains(&name),
                "`{name}` is declared by more than one filter set"
            );
            seen.push(name);
        }
    }

    /// The check that would otherwise happen at RENDER time, i.e. at some
    /// user's build: every filter a bundled pack calls is one a set registers.
    ///
    /// A name we do not declare is only an error if minijinja does not have it
    /// either — the packs legitimately use builtins (`upper`). Probing a bare
    /// `Environment` answers that without this file carrying a list of
    /// minijinja's builtins that would go stale on every upgrade.
    #[test]
    fn every_filter_a_pack_calls_is_registered() {
        let ours: Vec<&str> = all_filter_names().collect();
        let builtins = Environment::new();
        for (pack, source) in crate::render::bundled_packs() {
            for name in filters_called_by(source) {
                if ours.contains(&name.as_str()) {
                    continue;
                }
                assert!(
                    resolves(&builtins, &name),
                    "pack `{pack}` calls filter `{name}`, which no filter set \
                     registers and minijinja does not provide"
                );
            }
        }
    }

    /// The other direction: a declared filter no pack calls is a spelling left
    /// behind by a pack rewrite. Nine of the ten are per-language type
    /// spellings and every one of them has a caller today; keeping it that way
    /// is what makes "a filter set" a description of the packs rather than an
    /// accumulating list.
    #[test]
    fn every_registered_filter_is_called_by_a_pack() {
        let mut called: Vec<String> = Vec::new();
        for (_pack, source) in crate::render::bundled_packs() {
            called.extend(filters_called_by(source));
        }
        for name in all_filter_names() {
            assert!(
                called.iter().any(|c| c == name),
                "filter `{name}` is registered but no bundled pack calls it"
            );
        }
    }

    /// The tokenizer must not be fooled by the target-language text the packs
    /// emit. `|item|` closures and `a || b` appear in real packs; if the helper
    /// counted those, `every_filter_a_pack_calls_is_registered` would be
    /// noise-driven and the first person to hit it would delete it.
    #[test]
    fn the_lexer_ignores_target_language_pipes() {
        let source = "let f = |item| item + 1; let b = a || c;\n{{ x|snake_case }}\n";
        assert_eq!(filters_called_by(source), vec!["snake_case".to_string()]);
    }

    /// `resolves` must actually distinguish the two cases, or every check built
    /// on it passes vacuously.
    #[test]
    fn the_probe_tells_absent_from_present() {
        let mut env = Environment::new();
        (NEUTRAL.register)(&mut env);
        assert!(resolves(&env, "snake_case"));
        assert!(!resolves(&env, "no_such_filter_anywhere"));
        // A registered filter that rejects its argument is still registered:
        // `c_type` wants a struct and gets an integer.
        let mut env = Environment::new();
        (C.register)(&mut env);
        assert!(resolves(&env, "c_type"));
    }
}
