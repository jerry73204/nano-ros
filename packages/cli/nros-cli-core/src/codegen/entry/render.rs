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
//! This environment is deliberately small: the templates, no filters, no
//! loader.

use std::sync::LazyLock;

use minijinja::Environment;

/// Every entry template, bundled into the binary.
///
/// `include_str!` rather than a directory read: `nros` is invoked from CMake
/// and from build scripts in trees that do not contain this checkout, so a
/// template resolved from disk at run time would be a path dependency the
/// caller cannot satisfy.
const TEMPLATES: &[(&str, &str)] = &[
    (
        "rust_entry.rs.jinja",
        include_str!("templates/rust_entry.rs.jinja"),
    ),
    ("c_entry.c.jinja", include_str!("templates/c_entry.c.jinja")),
    (
        "c_service_trailer.c.jinja",
        include_str!("templates/c_service_trailer.c.jinja"),
    ),
];

static ENV: LazyLock<Environment<'static>> = LazyLock::new(|| {
    let mut env = Environment::new();
    // KEEP the template's final newline. minijinja strips it by default, and
    // the generated TUs end with one — the golden harness caught the
    // difference as exactly one byte (1492 -> 1491), which is the whole reason
    // that harness landed before this change.
    env.set_keep_trailing_newline(true);
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

    /// An unknown name must be an error, not an empty render — a typo'd
    /// template name that returned "" would emit an empty source file.
    #[test]
    fn an_unbundled_template_is_an_error() {
        let err = render("no_such_template.jinja", ()).expect_err("must not resolve");
        assert!(err.contains("not bundled"), "unhelpful error: {err}");
    }
}
