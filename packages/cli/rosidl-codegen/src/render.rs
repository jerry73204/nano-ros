//! RFC-0068 Stage 3 — Render.
//!
//! A runtime (`minijinja`) template engine over data packs. A language backend
//! is a set of `.jinja` templates plus a `serde`-serializable data context;
//! nothing about a language lives in Rust here. Templates are bundled at build
//! time via `include_str!` (fast, no I/O) and rendered from a view struct.
//!
//! Every backend AND the per-package scaffolding (Cargo/lib/build) render through
//! this one `Environment` now — askama is fully removed (phase-335 W6). Type
//! spelling that used to be pre-baked in the builders is composed in the pack by
//! filters (RFC-0068 step 2), and those filters are the LANGUAGE's contribution,
//! not this module's: see [`crate::filters`] for the set, who owns each one, and
//! the checks that keep the packs and the registry in agreement (RFC-0091 §6b /
//! phase-432 W2.5b). This module owns the templates, the loader and the globals.

use std::sync::LazyLock;

use minijinja::Environment;

/// Every bundled pack template, keyed by the stable name a `render(name, …)`
/// call and any `{% import %}` use. Adding a language = adding rows here plus its
/// `.jinja` files — no other Rust. `include_str!` bundles them at build time.
const PACKS: &[(&str, &str)] = &[
    // RFC-0090 / phase-429 W1 — shared by the C and C++ packs, because the
    // codegen-version stamp is about the C ABI both of them emit into. Not
    // inside either pack dir for that reason.
    (
        "_codegen_version.jinja",
        include_str!("../packs/_codegen_version.jinja"),
    ),
    // C pack (packs/c)
    ("_field.jinja", include_str!("../packs/c/_field.jinja")),
    ("message.h", include_str!("../packs/c/message.h.jinja")),
    ("message.c", include_str!("../packs/c/message.c.jinja")),
    ("service.h", include_str!("../packs/c/service.h.jinja")),
    ("service.c", include_str!("../packs/c/service.c.jinja")),
    ("action.h", include_str!("../packs/c/action.h.jinja")),
    ("action.c", include_str!("../packs/c/action.c.jinja")),
    // rmw Rust pack (packs/rmw)
    (
        "message_rmw.rs",
        include_str!("../packs/rmw/message.rs.jinja"),
    ),
    (
        "service_rmw.rs",
        include_str!("../packs/rmw/service.rs.jinja"),
    ),
    (
        "action_rmw.rs",
        include_str!("../packs/rmw/action.rs.jinja"),
    ),
    // nros embedded Rust pack (packs/nros)
    (
        "nros_field.jinja",
        include_str!("../packs/nros/nros_field.jinja"),
    ),
    (
        "message_nros.rs",
        include_str!("../packs/nros/message.rs.jinja"),
    ),
    (
        "service_nros.rs",
        include_str!("../packs/nros/service.rs.jinja"),
    ),
    (
        "action_nros.rs",
        include_str!("../packs/nros/action.rs.jinja"),
    ),
    // idiomatic Rust pack (packs/rust)
    (
        "message_idiomatic.rs",
        include_str!("../packs/rust/message.rs.jinja"),
    ),
    (
        "service_idiomatic.rs",
        include_str!("../packs/rust/service.rs.jinja"),
    ),
    (
        "action_idiomatic.rs",
        include_str!("../packs/rust/action.rs.jinja"),
    ),
    // C++ pack (packs/cpp)
    (
        "message_cpp.hpp",
        include_str!("../packs/cpp/message.hpp.jinja"),
    ),
    (
        "message_cpp_types.rs",
        include_str!("../packs/cpp/message_types.rs.jinja"),
    ),
    (
        "message_cpp_exports.rs",
        include_str!("../packs/cpp/message_exports.rs.jinja"),
    ),
    (
        "service_cpp.hpp",
        include_str!("../packs/cpp/service.hpp.jinja"),
    ),
    (
        "action_cpp.hpp",
        include_str!("../packs/cpp/action.hpp.jinja"),
    ),
    // scaffolding pack (packs/scaffold)
    (
        "cargo.toml",
        include_str!("../packs/scaffold/cargo.toml.jinja"),
    ),
    (
        "cargo_nros.toml",
        include_str!("../packs/scaffold/cargo_nros.toml.jinja"),
    ),
    ("build.rs", include_str!("../packs/scaffold/build.rs.jinja")),
    ("lib.rs", include_str!("../packs/scaffold/lib.rs.jinja")),
    (
        "lib_nros.rs",
        include_str!("../packs/scaffold/lib_nros.rs.jinja"),
    ),
];

/// Optional external pack directory (W4). When set (before the first render), a
/// file `<dir>/<name>` or `<dir>/<name>.jinja` OVERRIDES the bundled pack of that
/// name; anything absent falls back to bundled. Lets a language pack be swapped
/// or added with NO rebuild.
static OVERRIDE_DIR: std::sync::OnceLock<std::path::PathBuf> = std::sync::OnceLock::new();

/// Every bundled pack `(name, content)`. The codegen fingerprint (RFC-0061 /
/// phase-335 W4.a) hashes these so ANY pack edit — even one the emit corpus does
/// not exercise (rmw / idiomatic / scaffolding) — marks fixtures stale.
pub fn bundled_packs() -> &'static [(&'static str, &'static str)] {
    PACKS
}

/// Point the renderer at an external pack directory. Call once, before any
/// render (the `Environment` is built lazily on first use). Returns `Err` with
/// the passed dir if a directory was already set.
pub fn set_template_dir(dir: std::path::PathBuf) -> Result<(), std::path::PathBuf> {
    OVERRIDE_DIR.set(dir)
}

/// The environment. A `minijinja` loader resolves each name on demand: the
/// override dir wins (W4), else the bundled `PACKS`. Imports resolve the same way.
static ENV: LazyLock<Environment<'static>> = LazyLock::new(|| {
    let mut env = Environment::new();
    // Generated sources carry their own trailing newline in the template body;
    // do not let the engine append another.
    env.set_keep_trailing_newline(false);
    // RFC-0090 / phase-429 W1 — a GLOBAL, not a per-context field. Every
    // artifact this generator emits was emitted by this generator, so the
    // version is a property of the environment, not of any one message; adding
    // it to the six-odd context structs instead would be six places to forget.
    env.add_global(
        "codegen_version",
        crate::codegen_version::NROS_CODEGEN_VERSION,
    );
    // phase-432 W2.5b — the filters are not registered here any more. A
    // language's Rust surface area is a FILTER SET (`crate::filters`), so the
    // environment asks the registry rather than carrying ten `add_filter`
    // calls that say nothing about which language owns which spelling.
    crate::filters::register_all(&mut env);

    // W4.b — the override dir comes from `set_template_dir` (a CLI flag can call
    // it) or, with no cross-command plumbing, the `NROS_TEMPLATE_DIR` env var.
    let override_dir = OVERRIDE_DIR
        .get()
        .cloned()
        .or_else(|| std::env::var_os("NROS_TEMPLATE_DIR").map(std::path::PathBuf::from));
    env.set_loader(move |name| {
        if let Some(dir) = &override_dir {
            for cand in [dir.join(name), dir.join(format!("{name}.jinja"))] {
                match std::fs::read_to_string(&cand) {
                    Ok(s) => return Ok(Some(s)),
                    Err(e) if e.kind() == std::io::ErrorKind::NotFound => {}
                    Err(e) => {
                        return Err(minijinja::Error::new(
                            minijinja::ErrorKind::InvalidOperation,
                            format!("reading external pack {}: {e}", cand.display()),
                        ));
                    }
                }
            }
        }
        Ok(PACKS
            .iter()
            .find(|(n, _)| *n == name)
            .map(|(_, s)| s.to_string()))
    });
    env
});

/// Render a bundled pack template with the given serializable context.
pub fn render(template: &str, ctx: impl serde::Serialize) -> Result<String, minijinja::Error> {
    ENV.get_template(template)?.render(ctx)
}

/// Back-compat alias for the C call sites (phase-335 W2).
pub fn render_c(template: &str, ctx: impl serde::Serialize) -> Result<String, minijinja::Error> {
    render(template, ctx)
}
