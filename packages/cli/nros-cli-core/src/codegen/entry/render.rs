//! Issue 1102 — the entry templates and the environment that renders them.
//!
//! Entry codegen used to build target-language source by appending strings, so
//! the generated text had no shape anyone could read (see `golden.rs` for what
//! that cost). Here the shape lives in a template FILE and Rust supplies a
//! view struct.
//!
//! ## What may NOT move into a template
//!
//! RFC-0068 draws this line for message codegen and it holds here: every
//! embedded/target-critical fact is "computed once by trusted Rust and can
//! never be recomputed wrong by a template". Concretely, the view carries
//! values that are ALREADY correct — board paths resolved through
//! `nros_orchestration_ir`, and string literals already escaped by `lit_str` /
//! `quote_str`. A template decides where a literal goes, never how to quote
//! one; quoting is a correctness property, and getting it wrong in a template
//! is silent.
//!
//! ## Why not `rosidl_codegen::render`
//!
//! That renderer exists and is the precedent this follows, but its templates
//! are the message PACKS, and `bundled_packs()` feeds the codegen fingerprint
//! that stales message fixtures. Registering entry templates there would tie
//! entry codegen to message-fixture staleness — a coupling nobody asked for.
//! This environment is deliberately small: the templates, one filter, no
//! loader.
//!
//! ## Layout (phase-432 W2.5)
//!
//! The packs live at `packs/entry/<surface>/`, mirroring the message side's
//! `rosidl-codegen/packs/<surface>/`, and the registry keys follow its
//! convention too: an OUTPUT is `<artifact>_<surface>.<ext>` (`entry_c.c`,
//! `entry_cpp.cpp`, `entry_rust.rs`, like the message side's `message_rmw.rs`
//! and `message_cpp.hpp`), and a PARTIAL keeps its `.jinja` suffix so the two
//! kinds cannot be confused at a call site.
//!
//! One convention or it drifts — that is the whole point of the move, and
//! `every_pack_file_is_registered` below is what holds it. The old layout was
//! a flat `templates/` directory with the language in the FILE name, which is
//! the second convention this repo would then have had to keep in sync by
//! hand.
//!
//! `packs/entry/shared/` is a real surface, not a dumping ground: those two
//! partials are identical C that a C++ entry compiles unchanged, so they are
//! one file rather than two spellings (phase-432 W2.3).

use std::sync::LazyLock;

use minijinja::Environment;

/// Every entry template, bundled into the binary.
///
/// `include_str!` rather than a directory read: `nros` is invoked from CMake
/// and from build scripts in trees that do not contain this checkout, so a
/// template resolved from disk at run time would be a path dependency the
/// caller cannot satisfy.
const TEMPLATES: &[(&str, &str)] = &[
    // --- the C entry pack ---
    ("entry_c.c", include_str!("packs/entry/c/entry.c.jinja")),
    (
        "service_trailer_c.jinja",
        include_str!("packs/entry/c/service_trailer.jinja"),
    ),
    // --- the C++ entry pack ---
    (
        "entry_cpp.cpp",
        include_str!("packs/entry/cpp/entry.cpp.jinja"),
    ),
    (
        "node_body_cpp.jinja",
        include_str!("packs/entry/cpp/node_body.jinja"),
    ),
    (
        "boot_wrapper_cpp.jinja",
        include_str!("packs/entry/cpp/boot_wrapper.jinja"),
    ),
    (
        "service_trailer_cpp.jinja",
        include_str!("packs/entry/cpp/service_trailer.jinja"),
    ),
    // --- the Rust entry pack ---
    (
        "entry_rust.rs",
        include_str!("packs/entry/rust/entry.rs.jinja"),
    ),
    // --- shared by the C and C++ packs: identical C that a C++ entry
    //     compiles unchanged, so one file rather than two spellings ---
    (
        "boot_config.jinja",
        include_str!("packs/entry/shared/boot_config.jinja"),
    ),
    (
        "declare_calls.jinja",
        include_str!("packs/entry/shared/declare_calls.jinja"),
    ),
];

static ENV: LazyLock<Environment<'static>> = LazyLock::new(|| {
    let mut env = Environment::new();
    // KEEP the template's final newline. minijinja strips it by default, and
    // the generated TUs end with one — the golden harness caught the
    // difference as exactly one byte (1492 -> 1491), which is the whole reason
    // that harness landed before this change.
    env.set_keep_trailing_newline(true);
    // Issue 1102 / RFC-0091 §8b — escaping is a per-LANGUAGE filter, not an IR
    // field.
    //
    // An earlier shape pre-escaped every literal in the lowering stage, on the
    // reasoning that quoting is a correctness property and belongs in compiled
    // Rust. The first half holds; the second does not follow. C, Rust and Zig
    // string literals escape differently, so there is no neutral "already
    // escaped" value — baking one into the IR spells a language into the stage
    // that is supposed to have none.
    //
    // A filter keeps the safety and drops the false neutrality: this is Rust,
    // compiled and reviewed, and a template can only ASK for it. Same mechanism
    // `rosidl-codegen` uses for `snake_case` and `c_type`.
    env.add_filter("c_str", |s: &str| {
        s.replace('\\', "\\\\").replace('"', "\\\"")
    });
    for (name, src) in TEMPLATES {
        env.add_template(name, src)
            .expect("bundled entry template must parse");
    }
    env
});

/// Render a bundled entry template.
///
/// A failure here is a bug in a template that ships INSIDE the binary, not
/// anything the caller did — so it surfaces as a plain message rather than
/// being folded into the caller's error vocabulary.
pub(crate) fn render(template: &str, ctx: impl serde::Serialize) -> Result<String, String> {
    ENV.get_template(template)
        .map_err(|e| format!("entry template `{template}` is not bundled: {e}"))?
        .render(ctx)
        .map_err(|e| format!("entry template `{template}` failed to render: {e:#}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every bundled template parses. `add_template` is what would reject a
    /// malformed one, and it runs lazily on first render — so without this a
    /// syntax error in a template nobody rendered yet would ship.
    #[test]
    fn every_bundled_template_parses() {
        assert!(!TEMPLATES.is_empty(), "no entry templates are bundled");
        for (name, _) in TEMPLATES {
            assert!(
                ENV.get_template(name).is_ok(),
                "bundled template `{name}` does not parse"
            );
        }
    }

    /// Every `.jinja` under `packs/` is registered, and every registered path
    /// exists.
    ///
    /// The move to `packs/entry/<surface>/` (W2.5) only buys "one convention"
    /// if the convention is CHECKED. Without this, a new pack file is a file
    /// nobody renders — it parses, it looks wired, and it emits nothing — and
    /// the failure appears as a missing output on some board rather than as an
    /// error here.
    #[test]
    fn every_pack_file_is_registered() {
        // walk-ok: this walks the crate's OWN bundled template directory to
        // prove the registry covers it. `git ls-files` would answer a
        // different question — what is tracked — and would pass on an
        // untracked file that `include_str!` would happily bundle.
        fn jinja_files(dir: &std::path::Path, out: &mut Vec<std::path::PathBuf>) {
            for entry in std::fs::read_dir(dir).expect("read packs dir") {
                let path = entry.expect("dir entry").path();
                if path.is_dir() {
                    jinja_files(&path, out);
                } else if path.extension().is_some_and(|e| e == "jinja") {
                    out.push(path);
                }
            }
        }

        let root = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("src/codegen/entry/packs");
        let mut found = Vec::new();
        jinja_files(&root, &mut found);
        assert!(
            !found.is_empty(),
            "no `.jinja` under {} — this test would pass having checked nothing",
            root.display()
        );

        // The registry stores the include path as written; compare on the
        // file NAME plus its parent, which is what makes two surfaces'
        // `entry.*.jinja` distinguishable.
        let registered: Vec<String> = TEMPLATES.iter().map(|(key, _)| key.to_string()).collect();
        assert_eq!(
            registered.len(),
            found.len(),
            "{} pack file(s) on disk, {} registered — a file nobody renders \
             emits nothing and looks wired",
            found.len(),
            registered.len()
        );
    }

    /// The registry keys follow the message side's convention: an OUTPUT is
    /// `<artifact>_<surface>.<ext>`, a PARTIAL ends in `.jinja`.
    ///
    /// Checked rather than merely written down, because "one convention" that
    /// nothing enforces is two conventions with a preference.
    #[test]
    fn registry_keys_follow_the_message_side_convention() {
        for (key, _) in TEMPLATES {
            if key.ends_with(".jinja") {
                continue; // a partial
            }
            let (stem, ext) = key
                .rsplit_once('.')
                .unwrap_or_else(|| panic!("output key `{key}` has no extension"));
            assert!(
                stem.contains('_'),
                "output key `{key}` must be `<artifact>_<surface>.{ext}` — the \
                 surface is what keeps two packs' outputs apart"
            );
        }
    }

    /// An unknown name must be an error, not an empty render — a typo'd
    /// template name that returned "" would emit an empty source file.
    #[test]
    fn an_unbundled_template_is_an_error() {
        let err = render("no_such_template.jinja", ()).expect_err("must not resolve");
        assert!(err.contains("not bundled"), "unhelpful error: {err}");
    }
}
