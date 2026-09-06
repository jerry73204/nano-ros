//! Phase 212.N.9 — `nros::main!()` proc-macro implementation.
//!
//! Replaces the legacy Entry pkg `build.rs + include!()` shape with a
//! one-line `main.rs`. Four forms (per design-doc §11.6):
//!
//! ```ignore
//! nros::main!();                                          // single-node self-bringup
//! nros::main!(board = LinuxBoard);                       // single-node, explicit board
//! nros::main!(launch = "demo_bringup");                   // multi-node, default launch
//! nros::main!(launch = "demo_bringup:sim.launch.xml");    // multi-node, explicit file
//! nros::main!(board = X, launch = "Y:Z.xml", args = [("k", "v")]);
//! ```
//!
//! Form 1 reads `[package.metadata.nros.entry] deploy = "<board>"` from
//! the Entry pkg's own `Cargo.toml` and maps the board key to a board
//! crate via a small lookup table. Forms 3+ resolve the bringup pkg
//! through the N.10 workspace pkg-index, walk the N.11 launch.xml
//! parser, and emit one `<pkg_ident>::register(runtime)?;` call per
//! `<node pkg=… exec=…>` entry.
//!
//! ## Rebuild-correctness workaround
//!
//! Stable Rust proc-macros can't use the unstable
//! `proc_macro::tracked_path::path()` API. Instead we emit
//! `const _: &[u8] = include_bytes!("/abs/path");` for every file the
//! macro read (launch.xml, every `package.xml` the pkg-index walked,
//! the bringup's `system.toml`). Cargo's `include_bytes!` is tracked,
//! so touching any of these files invalidates the Entry pkg's
//! compilation cache.

use std::{
    collections::{BTreeMap, BTreeSet},
    path::{Path, PathBuf},
};

use nros_entry_lower::{LoweredNode, NodeIdentity, QosOverride};
use nros_orchestration_ir::{CallbackGroupDecl, ResolvedTierTable, resolve_tiers};
use proc_macro::TokenStream;
use proc_macro2::Span;
use quote::quote;
use syn::{
    Expr, ExprLit, Ident, Lit, LitStr, Path as SynPath, Token,
    parse::{Parse, ParseStream},
    parse_macro_input,
    punctuated::Punctuated,
};

/// Parsed `nros::main!(...)` argument set.
///
/// Each form maps to a different population of these fields:
///   - Form 1 (no args): all `None`.
///   - Form 2 (`board = X`): `board = Some(X)`, launch / args `None`.
///   - Form 3 (`launch = "Y"`): `launch = Some("Y")`, board derived
///     from `[package.metadata.nros.entry] deploy`.
///   - Form 4 (board + launch + args): all populated.
#[derive(Default)]
struct MainArgs {
    /// Explicit board ident — used verbatim when supplied. None
    /// triggers the `Cargo.toml [package.metadata.nros.entry]
    /// deploy = "<board>"` lookup.
    board: Option<SynPath>,
    /// `"<bringup_pkg>"` or `"<bringup_pkg>:<file.launch.xml>"`. None
    /// triggers the single-node self-bringup path (emit
    /// `<this_pkg>::register(runtime)?;`).
    launch: Option<LitStr>,
    /// RFC-0052 / phase-296 R2 — `model = "<bringup_pkg>"` or
    /// `model = "<bringup_pkg>:<file.yaml>"` (default file
    /// `config/system_model.yaml`). The CANONICAL path: bake from a
    /// resolved SystemModel committed in the bringup pkg
    /// instead of re-parsing launch XML + system.toml. Mutually exclusive
    /// with `launch`.
    model: Option<LitStr>,
    /// `args = [("k", "v"), ...]`. Forwarded to the launch parser as
    /// argument overrides.
    args: Vec<(String, String)>,
    /// issue 0274 — `spin = "forever"`: hosted deploys spin unbounded
    /// instead of the `NROS_ENTRY_SPIN_MS`-gated bounded spin (whose
    /// unset default is register-and-exit, a production-entry trap).
    spin_forever: bool,
    /// Phase 216.B.4 — `custom_tasks = [adc_sample, ui_redraw]`. Each
    /// ident becomes an extra `#[task]` trampoline inside the
    /// generated `mod __nros_app` body when the dispatched framework
    /// is RTIC. `None` = not supplied (the default — no extra tasks).
    /// `Some(vec![])` = supplied as an empty list (still no extra
    /// tasks, but the key was present — distinguished so the
    /// OwnedSpin/Embassy misuse error fires even on `[]`).
    custom_tasks: Option<Vec<Ident>>,
    /// Span of the `custom_tasks` key, retained for diagnostics when
    /// rejecting the key under a non-RTIC framework.
    custom_tasks_span: Option<Span>,
    /// phase-366 W7.a / RFC-0077 — `panic = "platform" | "halt" | "own"`.
    ///
    /// The image's ending, declared in the entry's own vocabulary. Resolved
    /// HERE, when the macro expands, not at link time: Rust has no overridable
    /// `#[panic_handler]`, so exactly one provider must be chosen before rustc
    /// runs, and `own` is what suppresses emission at that same point.
    panic: PanicPolicy,
}

/// What ends this image (RFC-0077's three values, identical on the cmake side).
#[derive(Copy, Clone, PartialEq, Eq, Default)]
enum PanicPolicy {
    /// Route to `nros_platform_panic` — the board's honest ending.
    ///
    /// M5 made this the DEFAULT (was `Own` through M1-M4, which kept every step
    /// behaviour-identical while the images migrated). An entry that says
    /// nothing about panics now gets a working ending, which is the ergonomic
    /// goal of the whole surface.
    ///
    /// Safe to flip only because M2/M3 migrated every image and M6 enforces it:
    /// at the time of the flip, ZERO embedded images called `main!()` without an
    /// explicit policy — the staticlib-shaped ones are derived (the lib owns the
    /// item) and hosted ones are excluded by the `cfg` below.
    #[default]
    Platform,
    /// Park the core, for images that must not print.
    Halt,
    /// This image supplies its own provider; emit nothing.
    ///
    /// A POSITIVE declaration, not an absence: an image bringing
    /// `esp-backtrace` or `panic-semihosting` states it, so the build can tell
    /// "deliberate" from "forgot".
    Own,
}

impl Parse for MainArgs {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let mut out = MainArgs::default();
        if input.is_empty() {
            return Ok(out);
        }
        // Parse `key = value` pairs, comma-separated. Allow a trailing
        // comma — matches every other Rust attribute-arg syntax.
        let pairs: Punctuated<KeyValue, Token![,]> = Punctuated::parse_terminated(input)?;
        for KeyValue { key, value } in pairs {
            match key.to_string().as_str() {
                "board" => {
                    let path = match value {
                        KvValue::Path(p) => p,
                        KvValue::Str(s) => {
                            return Err(syn::Error::new(
                                s.span(),
                                "expected board ident (e.g. `LinuxBoard`), got string literal",
                            ));
                        }
                        KvValue::Args(_) | KvValue::IdentList(_) => {
                            return Err(syn::Error::new(
                                key.span(),
                                "`board = ` takes a type path, not a list",
                            ));
                        }
                    };
                    out.board = Some(path);
                }
                "launch" => {
                    let s = match value {
                        KvValue::Str(s) => s,
                        _ => {
                            return Err(syn::Error::new(
                                key.span(),
                                "`launch = ` takes a string literal",
                            ));
                        }
                    };
                    out.launch = Some(s);
                }
                "model" => {
                    let s = match value {
                        KvValue::Str(s) => s,
                        _ => {
                            return Err(syn::Error::new(
                                key.span(),
                                "`model = ` takes a string literal \
                                 (e.g. `\"demo_bringup\"` or \
                                 `\"demo_bringup:config/system_model.yaml\"`)",
                            ));
                        }
                    };
                    out.model = Some(s);
                }
                "host" => {
                    // phase-326 (issue 0364) — the bake-time host filter is
                    // gone with `<node machine=>` (ROS 1 syntax). Fail with
                    // migration guidance rather than the generic unknown-key
                    // error.
                    return Err(syn::Error::new(
                        key.span(),
                        "`host = ` was removed (phase-326 / issue 0364) — \
                         multi-host partitions at RESOLVE time now. Point \
                         `model = ` at the per-host SystemModel instead \
                         (resolved with `host:=<id>`, e.g. \
                         `model = \"demo_bringup:config/multihost_robot1_model.yaml\"`)",
                    ));
                }
                "args" => {
                    let list = match value {
                        KvValue::Args(pairs) => pairs,
                        _ => {
                            return Err(syn::Error::new(
                                key.span(),
                                "`args = ` takes a list of `(\"key\", \"value\")` tuples",
                            ));
                        }
                    };
                    out.args = list;
                }
                "spin" => {
                    // issue 0274 — only the literal "forever" is accepted;
                    // bounded spins stay on the NROS_ENTRY_SPIN_MS env so
                    // test fixtures keep their per-run control.
                    let lit = match value {
                        KvValue::Str(s) => s,
                        _ => {
                            return Err(syn::Error::new(
                                key.span(),
                                "`spin = ` takes a string literal (`spin = \"forever\"`)",
                            ));
                        }
                    };
                    if lit.value() != "forever" {
                        return Err(syn::Error::new(
                            lit.span(),
                            "`spin = ` accepts only \"forever\" — bounded spins \
                             ride the NROS_ENTRY_SPIN_MS env",
                        ));
                    }
                    out.spin_forever = true;
                }
                "panic" => {
                    // phase-366 W7.a — the value set is closed on purpose. A
                    // free-form string here would be a policy this macro cannot
                    // implement, and the error naming the three is how an image
                    // discovers the surface exists.
                    let lit = match value {
                        KvValue::Str(s) => s,
                        _ => {
                            return Err(syn::Error::new(
                                key.span(),
                                "`panic = ` takes a string literal \
                                 (`panic = \"platform\"`, `\"halt\"` or `\"own\"`)",
                            ));
                        }
                    };
                    out.panic = match lit.value().as_str() {
                        "platform" => PanicPolicy::Platform,
                        "halt" => PanicPolicy::Halt,
                        "own" => PanicPolicy::Own,
                        other => {
                            return Err(syn::Error::new(
                                lit.span(),
                                format!(
                                    "`panic = \"{other}\"` is not a policy \
                                     (expected \"platform\" — route to \
                                     `nros_platform_panic`, \"halt\" — park the \
                                     core, or \"own\" — this image supplies its \
                                     own `#[panic_handler]`)"
                                ),
                            ));
                        }
                    };
                }
                "custom_tasks" => {
                    // Phase 216.B.4 — `custom_tasks = [ident, ident,
                    // ...]`. Stored even when empty so the framework-
                    // dispatch error fires regardless of list length.
                    let idents = match value {
                        KvValue::IdentList(v) => v,
                        _ => {
                            return Err(syn::Error::new(
                                key.span(),
                                "`custom_tasks = ` takes a list of fn idents, \
                                 e.g. `custom_tasks = [adc_sample, ui_redraw]`",
                            ));
                        }
                    };
                    out.custom_tasks = Some(idents);
                    out.custom_tasks_span = Some(key.span());
                }
                other => {
                    return Err(syn::Error::new(
                        key.span(),
                        format!(
                            "unknown `nros::main!` argument `{other}` \
                             (expected one of: board, launch, model, args, custom_tasks, spin, panic)"
                        ),
                    ));
                }
            }
        }
        if out.launch.is_some() && out.model.is_some() {
            return Err(syn::Error::new(
                Span::call_site(),
                "nros::main!: `launch` and `model` are mutually exclusive — `launch` names \
                 your input (phase-330 W7, the canonical spelling); `model` names the \
                 resolved artifact directly (expert override, deprecated)",
            ));
        }
        Ok(out)
    }
}

struct KeyValue {
    key: Ident,
    value: KvValue,
}

enum KvValue {
    /// `board = LinuxBoard` / `board = ::nros_board_linux::LinuxBoard`
    Path(SynPath),
    /// `launch = "demo_bringup:sim.launch.xml"`
    Str(LitStr),
    /// `args = [("use_sim", "true"), ...]`
    Args(Vec<(String, String)>),
    /// Phase 216.B.4 — `custom_tasks = [adc_sample, ui_redraw, ...]`.
    /// Empty list is valid (parses to `vec![]`).
    IdentList(Vec<Ident>),
}

impl Parse for KeyValue {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let key: Ident = input.parse()?;
        input.parse::<Token![=]>()?;
        // Try the array form first — two shapes share the bracket:
        // `args = [("k","v"), ...]` (tuple-string pairs) and
        // `custom_tasks = [foo, bar]` (bare idents, Phase 216.B.4).
        // Dispatch on the key name so each form's parser sees only
        // its expected token shape (cleaner diagnostics).
        if input.peek(syn::token::Bracket) {
            let content;
            syn::bracketed!(content in input);
            if key == "custom_tasks" {
                // Bare-ident list; empty `[]` parses to `vec![]`.
                let idents: Punctuated<Ident, Token![,]> = Punctuated::parse_terminated(&content)?;
                let collected: Vec<Ident> = idents.into_iter().collect();
                return Ok(KeyValue {
                    key,
                    value: KvValue::IdentList(collected),
                });
            }
            let pairs: Punctuated<TupleStrPair, Token![,]> =
                Punctuated::parse_terminated(&content)?;
            let collected = pairs
                .into_iter()
                .map(|p| (p.k.value(), p.v.value()))
                .collect();
            return Ok(KeyValue {
                key,
                value: KvValue::Args(collected),
            });
        }
        // A bare string literal -> KvValue::Str. Anything else -> Path.
        if input.peek(LitStr) {
            let s: LitStr = input.parse()?;
            return Ok(KeyValue {
                key,
                value: KvValue::Str(s),
            });
        }
        let path: SynPath = input.parse()?;
        Ok(KeyValue {
            key,
            value: KvValue::Path(path),
        })
    }
}

struct TupleStrPair {
    k: LitStr,
    v: LitStr,
}

impl Parse for TupleStrPair {
    fn parse(input: ParseStream) -> syn::Result<Self> {
        let content;
        syn::parenthesized!(content in input);
        let k: LitStr = content.parse()?;
        content.parse::<Token![,]>()?;
        let v: LitStr = content.parse()?;
        Ok(TupleStrPair { k, v })
    }
}

/// Entry point — emits the `fn main()` body. Errors surface as
/// `compile_error!()` spans pointing at the macro invocation.
pub fn expand(input: TokenStream) -> TokenStream {
    let args = parse_macro_input!(input as MainArgs);
    match build_main(args) {
        Ok(ts) => ts.into(),
        Err(err) => err.to_compile_error().into(),
    }
}

/// Errors carry a span — when we have a relevant token, attach it; when
/// the failure is environmental (Cargo.toml missing, launch parse
/// failed), fall back to `Span::call_site()`.
type MacroResult<T> = std::result::Result<T, syn::Error>;

fn build_main(mut args: MainArgs) -> MacroResult<proc_macro2::TokenStream> {
    // CARGO_MANIFEST_DIR is set by cargo when compiling proc-macro
    // consumers; if missing we fail loud — proc-macros without a
    // manifest dir would have no way to find Cargo.toml or workspace
    // root.
    let manifest_dir = std::env::var_os("CARGO_MANIFEST_DIR").ok_or_else(|| {
        syn::Error::new(
            Span::call_site(),
            "nros::main!: CARGO_MANIFEST_DIR not set (cargo must drive the build)",
        )
    })?;
    let manifest_dir = PathBuf::from(manifest_dir);

    // List of files that participated in the macro's decision so we
    // can emit `include_bytes!` rebuild stamps below. Always
    // canonicalised so the paths survive cargo's relocation tricks.
    let mut tracked: Vec<PathBuf> = Vec::new();

    // --- phase-330 W7: `launch = "<bringup>[:<file.launch.xml>]"` is the
    // INPUT-ADDRESSED spelling — the user names their launch file, never the
    // model artifact. Normalize it here into the `model =` flow: map
    // (launch, args) to the model's bringup-relative path via the ONE shared
    // rule (`nros_orchestration_ir::model_location::launch_to_model_rel` —
    // [[model]] declarations for arg-bound variants, derive rule otherwise)
    // and let the existing model arm resolve/consume it (build dir first,
    // committed fallback until W4.a). This is NOT the phase-296-deleted parse
    // path: no launch XML is parsed here — the launch file only ADDRESSES the
    // resolver output the build layer produced.
    if let Some(launch_lit) = args.launch.take() {
        let launch_value = launch_lit.value();
        let (bringup_name, launch_file) = match launch_value.split_once(':') {
            Some((b, f)) => (b.trim().to_string(), Some(f.trim().to_string())),
            None => (launch_value.trim().to_string(), None),
        };
        if bringup_name.is_empty() {
            return Err(syn::Error::new(
                launch_lit.span(),
                "nros::main!: empty bringup pkg name in `launch = \"...\"`",
            ));
        }
        let workspace_root = nros_pkg_index::detect_workspace_root(&manifest_dir).map_err(|e| {
            syn::Error::new(
                launch_lit.span(),
                format!("nros::main!: detect_workspace_root: {e}"),
            )
        })?;
        let pkg_index = nros_pkg_index::build_pkg_index(&workspace_root).map_err(|e| {
            syn::Error::new(
                launch_lit.span(),
                format!("nros::main!: build_pkg_index: {e}"),
            )
        })?;
        let bringup_dir = pkg_index
            .resolve_pkg(&bringup_name)
            .map_err(|e| syn::Error::new(launch_lit.span(), format!("nros::main!: {e}")))?;
        // Rebuild when the USER'S inputs change: the launch file and
        // system.toml are what the mapping (and the resolver behind it) read.
        if let Some(f) = &launch_file {
            tracked.push(bringup_dir.join("launch").join(f));
        }
        tracked.push(bringup_dir.join("system.toml"));
        let launch_args: Vec<(String, String)> = args.args.clone();
        let model_rel = nros_orchestration_ir::model_location::launch_to_model_rel(
            bringup_dir,
            launch_file.as_deref(),
            &launch_args,
        )
        .map_err(|e| syn::Error::new(launch_lit.span(), format!("nros::main!: {e}")))?;
        args.model = Some(LitStr::new(
            &format!("{bringup_name}:{model_rel}"),
            launch_lit.span(),
        ));
    }

    // --- Board resolution ---
    // `deploy` is `Some(...)` only when the macro had to read the
    // Entry pkg's `[package.metadata.nros.entry] deploy = "..."` key
    // (form 1). When the user passes `board = X` directly we have no
    // deploy string and default to the `OwnedSpin` framework — RTIC
    // requires the `deploy = "rtic-mps2-an385"` opt-in for now.
    let (board_path, deploy_for_framework): (SynPath, Option<String>) = match &args.board {
        Some(p) => (p.clone(), None),
        None => {
            let cargo_toml = manifest_dir.join("Cargo.toml");
            tracked.push(cargo_toml.clone());
            let deploy = read_entry_deploy(&cargo_toml).map_err(|e| {
                syn::Error::new(
                    Span::call_site(),
                    format!(
                        "nros::main!: failed to read `[package.metadata.nros.entry] deploy` \
                         from `{}`: {e}\n  Hint: add `[package.metadata.nros.entry] deploy = \
                         \"<board>\"` (e.g. `\"native\"`, `\"freertos\"`, `\"zephyr\"`) \
                         to your Cargo.toml, or pass `board = MyBoard` to the macro.",
                        cargo_toml.display()
                    ),
                )
            })?;
            let resolved = board_path_for(&deploy).ok_or_else(|| {
                syn::Error::new(
                    Span::call_site(),
                    format!(
                        "nros::main!: unknown board `{deploy}` in \
                         `[package.metadata.nros.entry] deploy`. \
                         Known boards: {}.\n  Pass `board = <YourBoardZst>` explicitly \
                         if your board crate is not in the default table.",
                        known_boards_csv()
                    ),
                )
            })?;
            (resolved, Some(deploy))
        }
    };
    let framework = match deploy_for_framework.as_deref() {
        Some(d) => framework_for(d),
        None => Framework::OwnedSpin,
    };

    // Phase 216.B.4 — `custom_tasks = [...]` only applies to the
    // RTIC emit. OwnedSpin / Embassy have no `mod __nros_app { ... }`
    // body for the macro to splice into, so flag misuse early with a
    // span pointing at the key.
    if args.custom_tasks.is_some() && framework != Framework::Rtic {
        let span = args.custom_tasks_span.unwrap_or_else(Span::call_site);
        let framework_label = match framework {
            Framework::OwnedSpin => "OwnedSpin",
            Framework::Embassy => "Embassy",
            Framework::Zephyr => "Zephyr",
            Framework::Esp32 => "Esp32",
            Framework::Rtic => unreachable!(),
        };
        return Err(syn::Error::new(
            span,
            format!(
                "nros::main!: `custom_tasks = [...]` is only valid for the RTIC framework \
                 (current framework: {framework_label}). \
                 RTIC splices each ident as a `#[task]` inside the generated `mod __nros_app` \
                 body; other frameworks have no equivalent splice point."
            ),
        ));
    }

    // --- Phase 228.G — per-tier resolution state (RFC-0032 §6) ---
    // Populated only in the launch arm (where bringup_dir + node pkgs are in
    // scope). `node_groups` maps a node *instance* name → its declared callback
    // groups; `node_instances` is every launch node name (for the
    // instance-identity check vs `[[component]]`). `resolved_tiers` stays `None`
    // unless `system.toml` declares `[tiers.*]`.
    let mut node_groups: BTreeMap<String, Vec<CallbackGroupDecl>> = BTreeMap::new();
    let mut node_instances: Vec<String> = Vec::new();
    let mut resolved_tiers: Option<ResolvedTierTable> = None;
    // issue 0438 — the tier names the SYSTEM declared, as opposed to the ones
    // that survived membership resolution. Empty when the schedule is derived.
    let mut authored_tier_names: Vec<String> = Vec::new();
    // Phase 264 W2 — `[lifecycle]` boot autostart from `system.toml` (launch arm only).
    let mut lifecycle_code: Option<u8> = None;
    // Phase 264 W4b — `[param_services]` declared in `system.toml` (launch arm only) →
    // register the ROS 2 param services + seed the volatile store from the baked params.
    let mut param_services_enabled = false;
    // issue 0274 — `[param_services] node = "<name>"`: executor identity for
    // the parameter services on multi-node entries (model arm).
    let mut param_services_node: Option<String> = None;
    // phase-267 W1c/C4 — when `system.toml` declares a `[[bridge]]` AND `nros sync`
    // has generated `<bringup>/nros-bridge.toml`, the entry is a cross-RMW bridge:
    // the macro emits a `run_from_config_str(include_str!(<that file>))` main
    // instead of the ordinary register/spin entry. Holds the absolute path.
    let mut bridge_config_path: Option<PathBuf> = None;
    // Issue 0106 — the RMW backends a `[[bridge]]` Entry uses, read from
    // `system.toml` (resolved via `[[bridge]]` endpoints + `[[domain]]` rmws).
    // The macro emits `nros_rmw_<x>::register()` for each so the linker doesn't
    // dead-strip the backend's self-register `.init_array` ctor (the data-driven
    // `run_from_config` path references no backend symbol on its own).
    let mut bridge_rmws: Vec<String> = Vec::new();
    // phase-432 W2.4 — the per-node runtime bake, ONE struct per node.
    //
    // This was FOUR parallel vectors — `node_param_bakes` (phase-264 W4a),
    // `node_qos_bakes` (issue #52), `node_identity_bakes` (phase-268 W1),
    // `node_remap_bakes` (phase-305 W3 / issue 0255) — each indexed
    // positionally against `pkg_idents` and each pushed from a different arm.
    // Two failure modes went with that shape: an arm that pushes to three of
    // four silently shifts every later node's identity, and a fifth feature
    // arriving as a fifth vector is exactly how the CLI's Rust renderer fell
    // four features behind the macro (archived issue 0302).
    //
    // `LoweredNode` also makes the facts SHAREABLE: `nros codegen entry`'s
    // Rust renderer builds the same type from its own `Plan`, and the parity
    // corpus at `packages/cli/nros-entry-lower/testdata/parity/` is rendered
    // by both and compared (see `entry_parity` below). Empty in the
    // self-bringup arm, where the node has no launch facts at all.
    let mut lowered_nodes: Vec<LoweredNode> = Vec::new();
    // Issue 0257 — callback-slot-consuming entities the model declares for the
    // nodes THIS entry deploys (subs + service servers/clients + action
    // servers/clients). A LOWER bound: the model has no timer/guard-condition
    // entity. `0` in the self-bringup arm (no model) and for a model whose
    // launch tree carried no wiring — both keep the pre-0257 emit exactly.
    let mut model_callbacks: usize = 0;

    // --- Launch resolution → list of <pkg_ident> register calls ---
    let pkg_idents: Vec<Ident> = if let Some(model_lit) = &args.model {
        // RFC-0052 / phase-296 R2 — the CANONICAL arm: bake from a
        // resolved SystemModel committed in the bringup pkg. The
        // model's execution layer already carries the resolved tiers +
        // bindings + params, so this arm never touches launch XML or
        // system.toml (the tier resolution reuses the SAME
        // nros-orchestration-ir path the CLI's codegen-system --model does).
        let workspace_root = nros_pkg_index::detect_workspace_root(&manifest_dir).map_err(|e| {
            syn::Error::new(
                model_lit.span(),
                format!("nros::main!: detect_workspace_root: {e}"),
            )
        })?;
        let pkg_index = nros_pkg_index::build_pkg_index(&workspace_root).map_err(|e| {
            syn::Error::new(
                model_lit.span(),
                format!("nros::main!: build_pkg_index: {e}"),
            )
        })?;
        for (_, pkg_dir) in pkg_index.pkgs() {
            tracked.push(pkg_dir.join("package.xml"));
        }

        // `"bringup[:relative/model.yaml]"` (default `config/system_model.yaml`).
        let model_value = model_lit.value();
        let (bringup_name, file_override) = match model_value.split_once(':') {
            Some((b, f)) => (b.trim().to_string(), Some(f.trim().to_string())),
            None => (model_value.trim().to_string(), None),
        };
        if bringup_name.is_empty() {
            return Err(syn::Error::new(
                model_lit.span(),
                "nros::main!: empty bringup pkg name in `model = \"...\"`",
            ));
        }
        let bringup_dir = pkg_index
            .resolve_pkg(&bringup_name)
            .map_err(|e| syn::Error::new(model_lit.span(), format!("nros::main!: {e}")))?;
        let model_rel = file_override.unwrap_or_else(|| "config/system_model.yaml".to_string());
        // The model is a BUILD ARTIFACT (phase-330), so this asks for it by its
        // inputs: `ensure_model` returns a build-produced copy when one exists
        // (cmake / nros-build resolve into `$NROS_MODEL_DIR` or `$OUT_DIR`
        // before invoking cargo) and otherwise RESOLVES it from the bringup's
        // `system.toml` + launch file.
        //
        // Looking only for the artifact was the gap issue 0414 recorded: a
        // plain `cargo build` of an entry crate runs no build system, so the
        // macro failed with "SystemModel not found" for a model it had every
        // input needed to produce. Entry crates deliberately carry no
        // `build.rs`, which also means no `$OUT_DIR` — there is no build-chosen
        // home to fall back to, so `ensure_model` picks one.
        //
        // `inputs` are the files the model DERIVES from. Tracking them (not
        // just the artifact) is what makes editing `system.toml` or the launch
        // XML rebuild the entry.
        let (model_path, inputs) =
            nros_orchestration_ir::model_location::ensure_model(bringup_dir, &model_rel)
                .map_err(|e| syn::Error::new(model_lit.span(), format!("nros::main!: {e}")))?;
        tracked.extend(inputs);
        let yaml = std::fs::read_to_string(&model_path).map_err(|e| {
            syn::Error::new(
                model_lit.span(),
                format!("nros::main!: read `{}`: {e}", model_path.display()),
            )
        })?;
        let model = ros_launch_manifest_model::SystemModel::from_yaml_str(&yaml).map_err(|e| {
            syn::Error::new(
                model_lit.span(),
                format!(
                    "nros::main!: parse SystemModel `{}`: {e}",
                    model_path.display()
                ),
            )
        })?;

        // Bridge entry (phase-267 / R-code.1): the model's `execution.bridges`
        // is the SSoT for "this bringup relays between two RMW sessions". When
        // non-empty AND `nros sync` generated the runtime config
        // (`<bringup>/nros-bridge.toml`), emit the data-driven bridge runner
        // instead of a plain entry, and force-register each bridged RMW
        // backend (issue 0106 anti-dead-strip). Endpoint syntax is
        // `<rmw>:<domain-name>` — the rmw prefix is what we need here.
        if !model.execution.bridges.is_empty() {
            let cfg = bringup_dir.join("nros-bridge.toml");
            if cfg.exists() {
                tracked.push(cfg.clone());
                bridge_config_path = Some(cfg);
                let mut rmws: Vec<String> = model
                    .execution
                    .bridges
                    .iter()
                    .flat_map(|b| [b.from.as_str(), b.to.as_str()])
                    .map(|ep| ep.split(':').next().unwrap_or(ep).trim().to_string())
                    .filter(|s| !s.is_empty())
                    .collect();
                rmws.sort();
                rmws.dedup();
                bridge_rmws = rmws;
            }
        }

        let rtos = derive_target_rtos(deploy_for_framework.as_deref());
        let board_key = deploy_for_framework.as_deref().unwrap_or("native");

        // Slice: keep the nodes this Entry's board deploys. No deploy layer =
        // every node (single-image default). Board match mirrors the CLI's
        // `plan_from_model`: Linux target ⇒ native/posix; Mcu target ⇒ exact
        // board key OR the deploy's `kind` family (extra.kind, e.g. "zephyr").
        // (The bake-time host filter is gone — phase-326 / issue 0364:
        // multi-host partitions at RESOLVE time, so a per-host model already
        // contains only that host's nodes.)
        use ros_launch_manifest_model::{ExtraValue, Target};
        // phase-315 — does the model place ANYTHING on this board?
        //
        // `execution.deploy` is a PARTITION: node -> one target. That is the
        // right shape for machines, and the wrong shape for
        // board build-variants, where every board runs every node and the map
        // simply cannot say so.
        //
        // So a board the map never mentions is not "a board with nothing on it",
        // it is a board the model has no opinion about — exactly the case the
        // `(None, _) => true` arm below already handles per-NODE, lifted to the
        // board level. Before the resolver emitted `execution.deploy` at all,
        // every such entry took this path via the `is_empty()` fallback; adding
        // one `[deploy.native]` block silently narrowed five embedded entries to
        // zero nodes (`places no nodes on board `freertos``).
        //
        // A board the map DOES mention stays partitioned, so a genuine
        // mis-placement is still caught.
        let board_mentioned = model.execution.deploy.values().any(|d| match &d.target {
            Some(Target::Linux) => matches!(board_key, "native" | "posix"),
            Some(Target::Mcu { board: b }) => {
                b == board_key
                    || matches!(
                        d.extra.get("kind"),
                        Some(ExtraValue::Str(k)) if k == board_key
                    )
            }
            None => false,
        });
        let keep = |fqn: &str| -> bool {
            let Some(dep) = model.execution.deploy.get(fqn) else {
                return model.execution.deploy.is_empty();
            };
            if !board_mentioned {
                // Unpartitioned for THIS board.
                return true;
            }
            match (&dep.target, board_key) {
                // Board-agnostic (multi-board system, issue 0356): the model
                // names no board, so THIS entry's board decides — keep on
                // every board.
                (None, _) => true,
                (Some(Target::Linux), "native" | "posix") => true,
                (Some(Target::Mcu { board: b }), key) => {
                    b == key
                        || matches!(
                            dep.extra.get("kind"),
                            Some(ExtraValue::Str(k)) if k == key
                        )
                }
                _ => false,
            }
        };

        // Walk nodes in FQN order (BTreeMap = deterministic, matches the CLI).
        let mut idents = Vec::new();
        for (fqn, inst) in &model.structure.nodes {
            if !keep(fqn) {
                continue;
            }
            let pkg = inst.pkg.clone().ok_or_else(|| {
                syn::Error::new(
                    model_lit.span(),
                    format!("nros::main!: model node `{fqn}` has no `pkg`"),
                )
            })?;
            let node_pkg_dir = pkg_index.resolve_pkg(&pkg).map_err(|e| {
                syn::Error::new(
                    model_lit.span(),
                    format!("nros::main!: node pkg `{pkg}`: {e}"),
                )
            })?;
            tracked.push(node_pkg_dir.join("package.xml"));
            let cargo_toml = node_pkg_dir.join("Cargo.toml");
            if !cargo_toml.exists() {
                return Err(syn::Error::new(
                    model_lit.span(),
                    format!(
                        "nros::main!: node pkg `{pkg}` has no Cargo.toml at `{}`. \
                         C++ Node pkgs are not supported in Rust Entry pkgs.",
                        node_pkg_dir.display()
                    ),
                ));
            }
            idents.push(Ident::new(&pkg_to_crate_ident(&pkg), Span::call_site()));

            // Params (resolved values ride the model — the embedded target has
            // no record.json to read them from).
            // #276 — `params_files` YAML projects under the inline values.
            // `to_bake_string` is the shared rendering (issue-0269-adjacent:
            // `1.0f64.to_string()` is "1", which the runtime's
            // infer_param_value re-types as INTEGER).
            let resolved: Vec<(String, String)> = inst
                .resolved_params(fqn)
                .iter()
                .map(|(k, v)| (k.clone(), v.to_bake_string()))
                .collect();
            // Issue #52 — `qos_overrides.*` are NOT declared parameters: they
            // decompose into the node's QoS-override table. Until this, the
            // Rust path baked them as ordinary params and applied no QoS at
            // all, while the same model configured QoS on a C/C++ image.
            //
            // Issue 0303 — one shared lowering (the same crate the CLI
            // emitters use), and an override it cannot lower is a COMPILE
            // ERROR: an unknown policy or a misspelled role used to vanish,
            // leaving the image with different delivery semantics than the
            // model declares and nothing to read.
            let qos_overrides: Vec<QosOverride> = nros_orchestration_ir::qos_override::lower_all(
                resolved.iter().map(|(k, v)| (k.as_str(), v.as_str())),
            )
            .map_err(|e| {
                syn::Error::new(model_lit.span(), format!("nros::main!: node `{fqn}`: {e}"))
            })?
            .into_iter()
            .map(|o| QosOverride {
                topic: o.topic,
                role: o.role,
                policy: o.policy,
                value: o.value,
            })
            .collect();
            let params: Vec<(String, String)> = resolved
                .into_iter()
                .filter(|(k, _)| !nros_orchestration_ir::qos_override::is_qos_override(k))
                .collect();

            // Remaps (issue 0255) — resolved rules ride the model verbatim; the
            // runtime seam does the `~`/relative expansion + matching.
            let remaps = remap_bakes_for(inst);

            // Identity: bare name + namespace from the FQN.
            let bare = fqn.rsplit('/').next().unwrap_or(fqn).to_string();
            let namespace = {
                let ns = &fqn[..fqn.len() - bare.len()];
                let ns = ns.trim_end_matches('/');
                if ns.is_empty() {
                    "/".to_string()
                } else {
                    ns.to_string()
                }
            };
            lowered_nodes.push(LoweredNode {
                pkg: pkg.clone(),
                params,
                remaps,
                qos_overrides,
                identity: Some(NodeIdentity::new(&bare, &namespace)),
            });

            // Callback groups → tier, straight from the model's resolved
            // bindings (`<fqn>/<group>` → tier). A whole-node binding
            // (`<fqn>` → tier) assigns the node's default group.
            let prefix = format!("{fqn}/");
            let mut decls: Vec<CallbackGroupDecl> = Vec::new();
            for (key, tier) in &model.execution.bindings {
                if let Some(group) = key.strip_prefix(&prefix) {
                    decls.push(CallbackGroupDecl {
                        id: group.to_string(),
                        r#type: "MutuallyExclusive".to_string(),
                        tier: tier.clone(),
                    });
                } else if key == fqn {
                    decls.push(CallbackGroupDecl {
                        id: nros_orchestration_ir::DEFAULT_TIER.to_string(),
                        r#type: "MutuallyExclusive".to_string(),
                        tier: tier.clone(),
                    });
                }
            }
            if !decls.is_empty() {
                node_groups.insert(bare.clone(), decls);
            }
            node_instances.push(bare);
        }
        // Issue 0257 — count the callback slots the SLICED node set needs. Same
        // `keep` predicate as the walk above, so a per-board/per-host entry
        // sizes for the nodes it actually registers, not the whole system.
        // phase-307 W4 (second half) — `max(model_wiring, recorded)` per node,
        // through the SAME shared rule the CLI bake uses. The model cannot see
        // timers; on a board that honors per-entry sizing this derived value IS
        // the executor's capacity, so a one-sub/five-timer node sized from the
        // model alone dies at boot on its third timer. `nros sync` (W2) is what
        // makes the sidecars dependable; absence just falls back to the bound.
        let recorded = crate::source_metadata_sidecars::collect(&manifest_dir);
        tracked.extend(recorded.paths.iter().cloned());
        model_callbacks = nros_orchestration_ir::executor_sizing::count_callbacks_with_recorded(
            &model,
            |fqn| keep(fqn),
            |pkg, exec| {
                recorded
                    .slots
                    .get(&(pkg.to_string(), exec.to_string()))
                    .copied()
                    .unwrap_or(0)
            },
        );
        if idents.is_empty() {
            return Err(syn::Error::new(
                model_lit.span(),
                format!(
                    "nros::main!(model = …): SystemModel `{}` places no nodes on board \
                     `{board_key}` — check execution.deploy targets",
                    model_path.display()
                ),
            ));
        }

        // Tiers: convert the model's execution tiers for this RTOS and resolve
        // the table via the SHARED nros-orchestration-ir path.
        if !model.execution.tiers.is_empty() {
            let tiers: BTreeMap<String, nros_orchestration_ir::TierDef> = model
                .execution
                .tiers
                .iter()
                .map(|(name, t)| {
                    (
                        name.clone(),
                        nros_orchestration_ir::tier_from_model(t, &rtos),
                    )
                })
                .collect();
            let component_names: BTreeSet<&str> =
                node_instances.iter().map(String::as_str).collect();
            let table =
                resolve_tiers(&tiers, &[], &component_names, &node_groups, &rtos).map_err(|e| {
                    syn::Error::new(
                        model_lit.span(),
                        format!("nros::main!: tier resolution: {e}"),
                    )
                })?;
            nros_orchestration_ir::validate_tier_platform_applicability(&table, &rtos)
                .map_err(|e| syn::Error::new(model_lit.span(), format!("nros::main!: {e}")))?;
            // issue 0438 — remember that tiers were AUTHORED, so a collapse to
            // the degenerate default below can be reported instead of taken.
            authored_tier_names = model.execution.tiers.keys().cloned().collect();
            resolved_tiers = Some(table);
        } else {
            // phase-296 W5.13 follow-up — NO authored tiers: DERIVE the schedule
            // from the contract layer via the SAME shared core the CLI's
            // codegen-system uses (`nros-orchestration-ir::derive`), so a
            // pure-cargo Rust entry gets an identical `derived-<node>` table.
            // A tier-less, contract-less model derives nothing and stays
            // tier-less (byte-identical to before).
            let derived = nros_orchestration_ir::derive::derive_tiers_from_contracts(
                &model,
                &rtos,
                &node_groups,
            );
            if !derived.tiers.is_empty() {
                // Fail-loud: surface every recorded weakening at expansion time
                // (build stderr) — the macro has no runtime channel for these.
                for d in &derived.degradations {
                    eprintln!(
                        "nros::main!: derived-schedule degradation — {} [{}]: {}",
                        d.node, d.dim, d.reason
                    );
                }
                for name in &derived.groupless_notes {
                    eprintln!(
                        "nros::main!: derived-schedule note — node '{name}' declares no \
                         callback groups; it stays on the default tier"
                    );
                }
                let component_names: BTreeSet<&str> =
                    node_instances.iter().map(String::as_str).collect();
                let table = resolve_tiers(
                    &derived.tiers,
                    &derived.overrides,
                    &component_names,
                    &node_groups,
                    &rtos,
                )
                .map_err(|e| {
                    syn::Error::new(
                        model_lit.span(),
                        format!("nros::main!: derived tier resolution: {e}"),
                    )
                })?;
                nros_orchestration_ir::validate_tier_platform_applicability(&table, &rtos)
                    .map_err(|e| syn::Error::new(model_lit.span(), format!("nros::main!: {e}")))?;
                resolved_tiers = Some(table);
            }
        }

        // Lifecycle autostart + param-services capability from the model.
        // `lifecycle_code` is the u8 boot-transition level (0 none / 1
        // configure / 2 active), same encoding as `read_lifecycle_autostart`.
        lifecycle_code = model
            .structure
            .nodes
            .values()
            .find_map(|n| n.lifecycle_autostart)
            .map(|a| {
                use ros_launch_manifest_model::Autostart;
                match a {
                    Autostart::None => 0u8,
                    Autostart::Configure => 1,
                    Autostart::Active => 2,
                }
            });
        param_services_enabled = model
            .execution
            .features
            .iter()
            .any(|f| f == "param_services");
        // issue 0274 (walls 2+3) — `play_launch resolve` does not populate
        // `execution.features` yet, so ALSO read `[param_services]` straight
        // from the bringup's system.toml. The optional `node = "<name>"`
        // row names the executor identity the six ROS 2 parameter services
        // hang off (FQN + the liveliness token rmw_zenoh discovery needs) —
        // without it a multi-node entry leaves the executor node_name empty
        // and the services are invisible to `ros2 param`.
        // 0274 follow-up (sentinel 14.5): prefer the system.toml the model
        // was RESOLVED against (recorded in meta.inputs) — per-target models
        // ride per-target system tomls, and reading the fixed
        // `<bringup>/system.toml` leaked the native profile's
        // [param_services] into every MCU entry. Fallback: the fixed name.
        // Issue 0293 — `meta.inputs` paths are recorded RELATIVE to the bringup
        // package so a committed model is portable. Older models carry the
        // resolving machine's absolute path; accept both, absolute first.
        //
        // This matters more than provenance: an unresolvable path falls
        // through to the fixed `<bringup>/system.toml` below, which is exactly
        // the per-target leak recording the input was meant to prevent. With
        // absolute paths that fallback fired on every machine except the one
        // that resolved the model.
        let system_toml_path = model
            .meta
            .inputs
            .iter()
            .filter(|i| {
                std::path::Path::new(&i.path)
                    .extension()
                    .is_some_and(|e| e == "toml")
            })
            .find_map(|i| {
                let raw = std::path::Path::new(&i.path);
                let candidate = if raw.is_absolute() {
                    raw.to_path_buf()
                } else {
                    bringup_dir.join(raw)
                };
                candidate.exists().then_some(candidate)
            })
            .unwrap_or_else(|| bringup_dir.join("system.toml"));
        if system_toml_path.exists() {
            tracked.push(system_toml_path.clone());
            if let Ok(raw) = std::fs::read_to_string(&system_toml_path)
                && let Ok(doc) = raw.parse::<toml::Table>()
                && let Some(ps) = doc.get("param_services")
            {
                param_services_enabled = true;
                param_services_node = ps
                    .get("node")
                    .and_then(|v| v.as_str())
                    .map(|s| s.to_string());
            }
        }

        idents
    } else {
        match &args.launch {
            None => {
                // Form 1 / Form 2 — single-node self-bringup. Use this
                // pkg's own name as the register target. The codegen-
                // emitted ident is snake_case; we read CARGO_PKG_NAME and
                // sanitise via the same `-` → `_` rule the existing
                // `nros::node!()` macro uses (see `lib.rs::sanitize_pkg_name_for_symbol`).
                //
                // The bin target depends on the lib target of the same
                // pkg via Cargo's automatic `extern crate <my_pkg>;`,
                // so `<my_pkg>::register(runtime)?;` resolves at build
                // time.
                let pkg_name = std::env::var("CARGO_PKG_NAME").map_err(|_| {
                    syn::Error::new(Span::call_site(), "nros::main!: CARGO_PKG_NAME not set")
                })?;
                let crate_ident = pkg_to_crate_ident(&pkg_name);
                // Form 1 self-bringup is opt-in: the user's lib crate
                // must expose a `pub fn register(runtime)`. If this
                // Entry pkg is binary-only (no companion lib), the
                // emitted `<this_pkg>::register(...)` will fail to
                // compile with a clear error; we don't try to detect
                // that here.
                vec![Ident::new(&crate_ident, Span::call_site())]
            }
            Some(launch_lit) => {
                // Unreachable: phase-330 W7 normalizes `launch = …` into the
                // `model = …` arm (and `.take()`s it) before this match.
                return Err(syn::Error::new(
                    launch_lit.span(),
                    "nros::main!: internal error — `launch` survived W7 normalization",
                ));
            }
        }
    };

    // De-duplicate the tracked list — pkg-index walks can revisit
    // a pkg dir's `package.xml` from multiple paths.
    tracked.sort();
    tracked.dedup();

    // --- Emit ---
    let tracked_consts = tracked.iter().filter_map(|p| {
        // Skip non-existent paths. The macro may have stuffed a
        // path that the filesystem doesn't surface (e.g. a missing
        // `package.xml` when build_pkg_index synthesised a dir).
        // `include_bytes!` on a missing path is a hard compile
        // error, so be defensive.
        if !p.exists() {
            return None;
        }
        let s = p.to_string_lossy().into_owned();
        let lit = LitStr::new(&s, Span::call_site());
        Some(quote! {
            const _: &[u8] = ::core::include_bytes!(#lit);
        })
    });

    // phase-432 W2.4 — one `LoweredNode` per register call, whatever arm
    // produced the idents. The self-bringup arm has no launch facts, so its
    // node is BARE rather than absent: every field is still written, which is
    // the reset discipline `render_register_calls` documents.
    let entry_nodes: Vec<LoweredNode> = pkg_idents
        .iter()
        .enumerate()
        .map(|(i, ident)| match lowered_nodes.get(i) {
            Some(n) => n.clone(),
            None => LoweredNode::bare(&ident.to_string()),
        })
        .collect();
    let register_calls: Vec<proc_macro2::TokenStream> = render_register_calls(&entry_nodes);
    // Node count for the Zephyr framework boot banner (literal baked at
    // expansion time so the runtime body needs no extra import).
    let num_register_calls = register_calls.len();

    // Phase 264 W2 — `[lifecycle]` wiring: when `system.toml` declares it, register
    // the REP-2002 services + drive boot autostart right after the per-node
    // `register` calls (the executor is built, the nodes are installed). No-op token
    // stream when absent. `apply_lifecycle` is a no-op unless the Entry enabled
    // `nros/lifecycle-services`, so this is inert without the feature.
    let lifecycle_call: proc_macro2::TokenStream = match lifecycle_code {
        Some(code) => quote! {
            // issue 0460 — carry the reason. Not a `log::` call: an entry crate
            // need not depend on `log` (`native_rust_qos_entry` does not).
            runtime.apply_lifecycle(#code).map_err(|reason| {
                ::nros::__macro_support::nros_platform::RuntimeError::Capability {
                    name: "lifecycle",
                    reason,
                }
            })?;
        },
        None => quote! {},
    };

    // Phase 264 W4b — `[param_services]` wiring: when `system.toml` declares it, register
    // the 6 ROS 2 parameter services + seed the volatile param store with the aggregate
    // of every node's launch-baked `<param>` initials, right after the per-node `register`
    // calls. No-op token stream when absent. `apply_param_services` is a no-op unless the
    // Entry enabled `nros/param-services`, so this is inert without the feature. The seed
    // values are the raw launch strings; the runtime infers each `ParameterValue` type.
    // issue 0274 — `spin = "forever"` swaps the env-gated bounded spin for an
    // unbounded loop (production hosted entries opt in at source).
    let hosted_spin_call: proc_macro2::TokenStream = if args.spin_forever {
        quote! { __nros_hosted_spin_forever(runtime)?; }
    } else {
        quote! { __nros_hosted_spin_if_requested(runtime)?; }
    };

    let param_services_call: proc_macro2::TokenStream = if param_services_enabled {
        let seed_lits = entry_nodes
            .iter()
            .flat_map(|n| n.params.iter())
            .map(|(name, value)| {
                let n = LitStr::new(name, Span::call_site());
                let v = LitStr::new(value, Span::call_site());
                quote! { (#n, #v) }
            });
        quote! {
            // phase-314 — the system DECLARED `[param_services]`, so the entry
            // must carry the cargo feature. Without it `apply_param_services`
            // is a no-op: the build succeeds, the image boots, and
            // `ros2 param list` finds nothing. The declaration and the feature
            // used to be kept in sync by hand, and nothing said when they
            // weren't. This guard compiles in the ENTRY crate, so it sees the
            // entry's own feature set.
            const _: () = ::core::assert!(
                ::nros::__macro_support::PARAM_SERVICES_ENABLED,
                "this system declares `[param_services]` but this `nros` build does not \
                 carry the `param-services` feature — the parameter services would be \
                 silently dropped. Add it to this pkg's nros dependency features."
            );
            // issue 0460 — see the lifecycle twin above.
            runtime.apply_param_services(&[ #( #seed_lits ),* ]).map_err(|reason| {
                ::nros::__macro_support::nros_platform::RuntimeError::Capability {
                    name: "param_services",
                    reason,
                }
            })?;
        }
    } else {
        quote! {}
    };

    // Phase 228.G (RFC-0032 §5) — the OwnedSpin entry call. Multi-tier
    // (`[tiers.*]` present, more than the synthesized `default` tier) emits
    // `<Board>::run_tiers(TIERS, register-only-closure)`; the board owns the
    // per-tier spin. Single-tier / no tiers keeps the unchanged
    // `BoardEntry::run` path (`setup` owns the bounded hosted spin) — so the
    // emitted TU is byte-identical to pre-228 for every current example.
    // Issue #48 cause 1 — bake the deploy overlay from
    // `[package.metadata.nros.deploy.<board>]`. Only Form 1 (deploy key present)
    // has a board key to read; Form 2 (explicit `board = X`) gets an all-`None`
    // overlay, so `run_with_deploy` then behaves exactly like `run`.
    let mut deploy_overlay_lit = match deploy_for_framework.as_deref() {
        Some(board_key) => read_deploy_overlay(&manifest_dir.join("Cargo.toml"), board_key),
        None => DeployOverlayLit::default(),
    };
    // Issue #98 — a single-node launch names the primary session (the ROS graph
    // node name) after that node, instead of the board default `"node"`. With
    // multiple nodes they share one primary session, so naming it after one would
    // be wrong — per-node naming is the deferred multi-node piece, so leave the
    // overlay name unset and the board keeps `"node"`.
    if let [only] = node_instances.as_slice() {
        deploy_overlay_lit.node_name = Some(only.clone());
    }
    // issue 0274 — explicit `[param_services] node = "..."` names the primary
    // session / executor identity (multi-node entries otherwise leave it
    // empty and the parameter services get no liveliness attribution).
    if let Some(n) = &param_services_node {
        deploy_overlay_lit.node_name = Some(n.clone());
    }
    let deploy_overlay_ts = deploy_overlay_tokens(&deploy_overlay_lit);

    // Issue #129 (RFC-0031 C5b amendment) — explicit backend register for the
    // Zephyr framework arm. On every OwnedSpin board the BOARD's boot path owns
    // `nros_rmw_<x>::register()` (Phase 248 C5a); Zephyr has no BoardEntry and
    // its board crate is NetworkWait-only, so registration must come from the
    // entry itself. `.init_array` ctors are compiled out on `target_os = "none"`
    // (this includes native_sim's `x86_64-unknown-none`), so without this emit
    // the CFFI registry stays empty and `Executor::open` fails
    // `Transport(ConnectionFailed)` (NoBackend). The entry deps the concrete
    // backend under its `rmw-<x>` feature (C5b), keeping the crate + this call
    // resolvable; unknown rmw names emit nothing (data-driven fallbacks).
    let zephyr_rmw_register_ts: proc_macro2::TokenStream = deploy_for_framework
        .as_deref()
        .and_then(|board_key| read_deploy_rmw(&manifest_dir.join("Cargo.toml"), board_key))
        .and_then(|rmw| rmw_crate_ident(&rmw).map(|ident| (rmw, ident)))
        .map(|(rmw, crate_ident)| {
            let id = Ident::new(crate_ident, Span::call_site());
            // Gate on the entry's matching `rmw-<x>` cargo feature so an entry
            // built with a different `--features rmw-<y>` selection (the
            // Kconfig-driven multi-RMW shape) still compiles — the register
            // call cfg-outs together with the optional backend dep.
            let feature = format!("rmw-{rmw}");
            quote! {
                #[cfg(feature = #feature)]
                {
                    let _ = ::#id::register();
                }
            }
        })
        .unwrap_or_default();

    // W4b — bake the single-node boot identity into a `NROS_BOOT_CONFIG` static
    // placed in the `.nros_boot_config` linker section (bare-metal only; hosted
    // gets a plain `#[unsafe(no_mangle)] #[used]` static that is still referenceable).
    // The static is emitted once, before `body_ts`, so `&NROS_BOOT_CONFIG`
    // resolves in every framework arm's `deploy_overlay_ts` use site.
    let boot_config_static_ts = {
        let node_name_opt = match &deploy_overlay_lit.node_name {
            Some(s) => quote! { ::core::option::Option::Some(#s) },
            None => quote! { ::core::option::Option::None },
        };
        let locator_opt = match &deploy_overlay_lit.locator {
            Some(s) => quote! { ::core::option::Option::Some(#s) },
            None => quote! { ::core::option::Option::None },
        };
        let domain_opt = match deploy_overlay_lit.domain_id {
            Some(d) => quote! { ::core::option::Option::Some(#d) },
            None => quote! { ::core::option::Option::None },
        };
        quote! {
            #[used]
            #[cfg_attr(target_os = "none", unsafe(link_section = ".nros_boot_config"))]
            #[unsafe(no_mangle)]
            static NROS_BOOT_CONFIG: ::nros::BakedBootConfig =
                ::nros::BakedBootConfig::new(
                    #node_name_opt,
                    #locator_opt,
                    #domain_opt,
                    ::core::option::Option::None,
                );
        }
    };

    // Phase 244.D1 — `target_os = "none"` entry shape for the OwnedSpin
    // framework. FreeRTOS / threadx-linux have a C runtime that calls `main`,
    // so they keep the `extern "C" fn main`. A pure bare-metal Cortex-M image
    // has no C runtime; its reset vector needs a `#[cortex_m_rt::entry]`. Both
    // funnel through the shared `__nros_entry_run`.
    let none_entry_ts: proc_macro2::TokenStream =
        if is_baremetal_cortexm_deploy(deploy_for_framework.as_deref()) {
            quote! {
                #[cfg(target_os = "none")]
                #[::cortex_m_rt::entry]
                fn __nros_cortex_m_reset() -> ! {
                    // `run_with_deploy` loops forever on success (firmware
                    // lifetime) and exits via the board on spin error; reaching
                    // here means `setup()` returned `Err` before the spin loop.
                    let _ = __nros_entry_run();
                    loop {
                        ::cortex_m::asm::wfi();
                    }
                }
            }
        } else {
            quote! {
                #[cfg(target_os = "none")]
                #[unsafe(no_mangle)]
                pub extern "C" fn main() -> i32 {
                    match __nros_entry_run() {
                        ::core::result::Result::Ok(()) => 0,
                        ::core::result::Result::Err(_) => 1,
                    }
                }
            }
        };

    // phase-271 (issue #110) — per-entry executor sizing from the entry's own
    // `[package.metadata.nros.entry] max_callbacks`. `None` → default sizing
    // (byte-identical to pre-271); `Some` → emit the `_sized` board entry so the
    // hosted board opens via `Executor::open_sized`.
    let declared_sizing = read_entry_executor_sizing(&manifest_dir.join("Cargo.toml"));
    // Issue 0257 — DERIVE the sizing from the model when the entry declares
    // none. Explicit metadata always wins (the user may know about timers the
    // model cannot see); a derived value only replaces the build default when
    // it is bigger, so every entry whose model has no wiring — i.e. every
    // in-tree example that authors no `<stem>.contract.yaml` beside its launch
    // file, which is 109 of 114 as of 2026-09-06 (issue 0973) — emits exactly
    // what it emitted before.
    // issue 0460 — CAPABILITY services consume callback slots too, and the
    // model does not count them: `[lifecycle]` registers five REP-2002 services
    // and `[param_services]` six, so a system declaring both needs eleven slots
    // beyond its nodes' own callbacks. `DEFAULT_MAX_CBS` is 4.
    //
    // This is why three `entry_matrix` cells died. The features workspace holds
    // ONE `system.toml` for every feature demo (rust cannot hold two systems —
    // phase-315 W1), so its `features = ["param_services", "lifecycle"]` is a
    // UNION and EVERY entry there emits both capabilities regardless of which
    // launch file it selected. Sized for its one node, the params entry then
    // failed `register_lifecycle_services()` — and the Zephyr entry dropped the
    // `Result`, so the image printed nothing after "Network ready".
    // `zephyr/rust/safety` passed throughout because `safety` is a BACKEND
    // feature that registers no services.
    let capability_slots = usize::from(lifecycle_code.is_some())
        * nros_orchestration_ir::executor_sizing::LIFECYCLE_SERVICE_SLOTS
        + usize::from(param_services_enabled)
            * nros_orchestration_ir::executor_sizing::PARAM_SERVICE_SLOTS;
    let exec_sizing = executor_sizing_for(declared_sizing, model_callbacks + capability_slots)?;
    // Issue 0257 — the loud bake-time check. On a board that ignores the
    // per-entry sizing the capacity is whatever `NROS_EXECUTOR_MAX_CBS`
    // compiled in, which the macro cannot read — so assert it in the emitted
    // code against the real const rather than guessing here.
    let capacity_assert = executor_capacity_assert(
        model_callbacks,
        deploy_for_framework.as_deref(),
        exec_sizing,
    );

    // issue 0438 — a system that DECLARES `[tiers.*]` and binds no callback
    // group to any of them resolves to the degenerate single synthesized
    // `default` tier, and the macro then emits the SINGLE-tier path. That is a
    // declaration being discarded, and it used to happen in silence: the
    // orchestration_tiers_native fixture declared `[tiers.high]`/`[tiers.low]`
    // for months, emitted a single-tier binary, and the only visible symptom
    // was an E2E three layers away reporting a missing boot marker — which sent
    // the investigation to the board's print statements instead of here.
    //
    // Membership comes from `[[component]].group_tiers` (phase-273 W2 /
    // RFC-0047 moved it out of the package manifest). The fixture still carried
    // the pre-273 `callback_groups` form, which nothing reads any more, so the
    // tiers had no members. By this point the board slice is known non-empty —
    // an empty one already errored above — so authored-but-unbound really does
    // mean the tiers are dead declarations rather than sliced away.
    if !authored_tier_names.is_empty()
        && resolved_tiers.as_ref().is_some_and(|t| t.is_single_tier())
    {
        return Err(syn::Error::new(
            proc_macro2::Span::call_site(),
            format!(
                "nros::main!: the system declares tier(s) [{}] but no callback group is \
                 bound to any of them, so they resolve to a single default tier and this \
                 entry would silently emit the SINGLE-tier path.\n  \
                 Bind groups with `[[component]] group_tiers = {{ <group> = \"<tier>\" }}` \
                 in `system.toml` (phase-273 W2 / RFC-0047 — `callback_groups` in a node \
                 package's Cargo.toml is the retired form and is not read).\n  \
                 If the tiers are genuinely unused, delete them. issue 0438",
                authored_tier_names.join(", ")
            ),
        ));
    }
    let multi_tier = resolved_tiers.as_ref().filter(|t| !t.is_single_tier());
    // phase-302 W4 (issue 0265) — reject multi-tier systems on targets with
    // no `run_tiers` EARLY with a real diagnostic. Previously the deploy fell
    // back to the posix tier rules and died later with a misleading
    // `MissingRtosSpec("posix")` or a raw missing-method compile error.
    if multi_tier.is_some()
        && let Some(d) = deploy_for_framework.as_deref()
        && [
            "esp32",
            "rtic",
            "embassy",
            "baremetal",
            "bare-metal",
            "orin",
        ]
        .iter()
        .any(|f| d.contains(f))
    {
        return Err(syn::Error::new(
            proc_macro2::Span::call_site(),
            format!(
                "target `{d}` does not support multi-tier execution (no run_tiers): \
                 collapse the system to a single tier or pick a tiered board \
                 (posix/zephyr/freertos/nuttx/threadx families). issue 0265"
            ),
        ));
    }
    let entry_call: proc_macro2::TokenStream = match multi_tier {
        Some(table) => {
            let tiers_ts = tier_specs_tokens(table);
            quote! {
                <#board_path>::run_tiers(
                    // Issue #48 cause 1 — thread the deploy overlay into the
                    // multi-tier path too (firmware boards apply it to their boot
                    // `Config`; hosted boards ignore it).
                    &#deploy_overlay_ts,
                    #tiers_ts,
                    |runtime: &mut ::nros::__macro_support::nros_platform::RuntimeCtx<'_>|
                        -> ::core::result::Result<
                            (),
                            ::nros::__macro_support::nros_platform::RuntimeError,
                        >
                    {
                        // Register-only: the board sets each tier's
                        // `active_groups` filter and owns the spin loop.
                        // W4c — param services BEFORE the node registers, so the store
                        // exists when each cell captures it (cell → `ctx.parameter`).
                        #param_services_call
                        #( #register_calls )*
                        #lifecycle_call
                        ::core::result::Result::Ok(())
                    },
                )
            }
        }
        None => {
            // The register/spin closure — identical for the sized + unsized
            // board calls below.
            let setup_closure = quote! {
                |runtime: &mut ::nros::__macro_support::nros_platform::RuntimeCtx<'_>|
                    -> ::core::result::Result<
                        (),
                        ::nros::__macro_support::nros_platform::RuntimeError,
                    >
                {
                    // W4c — param services BEFORE the node registers, so the store
                    // exists when each cell captures it (cell → `ctx.parameter`).
                    #param_services_call
                    #( #register_calls )*
                    #lifecycle_call
                    #[cfg(not(any(target_os = "none", target_os = "nuttx")))]
                    #hosted_spin_call
                    ::core::result::Result::Ok(())
                }
            };
            // Issue #48 cause 1 — `run_with_deploy{,_sized}` applies the
            // deploy-metadata overlay (locator / ip / gateway / domain) to the
            // board's boot config. The default trait bodies ignore the overlay +
            // sizing and forward to `run`, so hosted / framework boards are
            // byte-identical; the FreeRTOS / bare-metal boards override
            // `run_with_deploy`, and the posix board overrides
            // `run_with_deploy_sized` (phase-271, issue #110) to open at the
            // entry's declared `max_callbacks`.
            match exec_sizing {
                EntrySizing::Declared(max_cbs, max_sc) => quote! {
                    <#board_path as ::nros::__macro_support::nros_platform::BoardEntry>::run_with_deploy_sized(
                        &#deploy_overlay_ts,
                        #max_cbs,
                        #max_sc,
                        #setup_closure,
                    )
                },
                // Issue 0257 — a DERIVED size must never SHRINK the table below
                // the build-time default (a workspace that already raised
                // `NROS_EXECUTOR_MAX_CBS` keeps its value), hence the max.
                EntrySizing::Derived(max_cbs) => quote! {
                    <#board_path as ::nros::__macro_support::nros_platform::BoardEntry>::run_with_deploy_sized(
                        &#deploy_overlay_ts,
                        {
                            const DERIVED: usize = #max_cbs;
                            const BUILD_DEFAULT: usize =
                                ::nros::__macro_support::EXECUTOR_MAX_CBS;
                            if DERIVED > BUILD_DEFAULT { DERIVED } else { BUILD_DEFAULT }
                        },
                        0usize,
                        #setup_closure,
                    )
                },
                EntrySizing::BuildDefault => quote! {
                    <#board_path as ::nros::__macro_support::nros_platform::BoardEntry>::run_with_deploy(
                        &#deploy_overlay_ts,
                        #setup_closure,
                    )
                },
            }
        }
    };

    // Phase 216 final wave — Node-pkg registration for framework
    // targets (RTIC + Embassy). The OwnedSpin branch keeps its
    // existing `<pkg>::register(runtime)?;` flow inside the
    // `BoardEntry::run` closure; the framework branches instead emit
    // `<pkg>::register_dispatch(&mut executor)?;` calls into the
    // generated `#[init]` body, which push the per-Node
    // `(state, on_callback)` pair into the `Executor`'s dispatch-slot
    // registry that the framework dispatch task walks.
    //
    // Source of truth: `[package.metadata.nros.entry] node_pkgs =
    // ["pkg_a", "pkg_b"]` in the Entry pkg's `Cargo.toml`. When the
    // key is absent we fall back to the Entry pkg's own name with a
    // conventional `_entry` suffix stripped — covers the common
    // self-bringup shape where a single Node pkg `foo_pkg` has a
    // sibling Entry pkg `foo_pkg_entry` (though the framework
    // examples today don't follow this — they always declare
    // `node_pkgs = [...]` explicitly).
    let framework_node_pkg_idents: Vec<Ident> = match framework {
        // OwnedSpin (incl. NuttX) + Zephyr + Esp32 all register via the
        // launch-resolved `register_calls` (the `RuntimeCtx`-based
        // `<pkg>::register` flow), NOT the RTIC/Embassy
        // `register_dispatch` splice.
        Framework::OwnedSpin | Framework::Zephyr | Framework::Esp32 => Vec::new(),
        Framework::Rtic | Framework::Embassy => {
            let cargo_toml = manifest_dir.join("Cargo.toml");
            let from_metadata = read_entry_node_pkgs(&cargo_toml).map_err(|e| {
                syn::Error::new(
                    Span::call_site(),
                    format!(
                        "nros::main!: failed to read `[package.metadata.nros.entry] node_pkgs` \
                         from `{}`: {e}",
                        cargo_toml.display()
                    ),
                )
            })?;
            let names: Vec<String> = match from_metadata {
                Some(v) => v,
                None => {
                    // Self-bringup fallback: strip `_entry` suffix
                    // from the Entry pkg's own name.
                    let pkg_name = std::env::var("CARGO_PKG_NAME").map_err(|_| {
                        syn::Error::new(Span::call_site(), "nros::main!: CARGO_PKG_NAME not set")
                    })?;
                    let stripped = pkg_name
                        .strip_suffix("_entry")
                        .or_else(|| pkg_name.strip_suffix("-entry"))
                        .unwrap_or(&pkg_name);
                    vec![stripped.to_string()]
                }
            };
            names
                .into_iter()
                .map(|n| Ident::new(&pkg_to_crate_ident(&n), Span::call_site()))
                .collect()
        }
    };
    let framework_register_dispatch_calls: Vec<proc_macro2::TokenStream> =
        framework_node_pkg_idents
            .iter()
            .map(|ident| {
                quote! {
                    ::#ident::register_dispatch(&mut executor)
                        .expect("nros::main!: register_dispatch — executor dispatch-slot table full");
                }
            })
            .collect();
    // Phase 289 (#178) — Rtic entity registration. `register_dispatch` only
    // installs the on_callback trampoline into the dispatch-slot table; it
    // never runs `Node::register(ctx)` — so an RTIC image opened its session
    // but owned NO node/publisher/timer entities and published nothing (the
    // phase-216 B.3 "per-Node register wiring" follow-up). Route through the
    // same owned-spin `<pkg>::register(&mut RuntimeCtx)` seam every other
    // board uses (`install_node_typed`: entities + tick registry + the
    // component table `dispatch_callback` scans).
    let framework_register_entity_calls: Vec<proc_macro2::TokenStream> = framework_node_pkg_idents
        .iter()
        .map(|ident| {
            quote! {
                ::#ident::register(&mut __nros_rt)
                    .expect("nros::main!: Node register failed on the RTIC entry");
            }
        })
        .collect();

    // Phase 216.B.3 — framework-dispatched emit body. `OwnedSpin`
    // keeps the long-standing `fn __nros_entry_run + fn main` shape
    // (BoardEntry::run owns the spin loop). `Rtic` emits a
    // `#[rtic::app(...)]` skeleton that delegates to
    // `RticBoardEntry::init_hardware` from the framework-generated
    // `#[init]` body. `Embassy` is a hard error pointing at the
    // 216.C.3 sibling that lands the emit body.
    // issue #128 (half 2) — the Zephyr arm's spin-or-tiers tail. Multi-tier
    // systems route through `ZephyrBoard::run_tiers` (one k_thread per tier
    // over the one shared session, RFC-0015 Model 1); single-tier keeps the
    // plain register+spin scaffold, byte-identical to pre-#128-half-2.
    let zephyr_body_tail: proc_macro2::TokenStream = match multi_tier {
        Some(table) => {
            let tiers_ts = tier_specs_tokens(table);
            quote! {
                return ::nros_board_zephyr::ZephyrBoard::run_tiers(
                    &config,
                    #tiers_ts,
                    |runtime: &mut ::nros::__macro_support::nros_platform::RuntimeCtx<'_>|
                        -> ::core::result::Result<
                            (),
                            ::nros::__macro_support::nros_platform::RuntimeError,
                        >
                    {
                        // Same per-tier closure sequence as the OwnedSpin
                        // multi-tier path: param services BEFORE the node
                        // registers (the store must exist when each cell
                        // captures it), lifecycle AFTER; the board sets each
                        // tier's `active_groups` filter and owns the spin.
                        #param_services_call
                        #( #register_calls )*
                        #lifecycle_call
                        ::core::result::Result::Ok(())
                    },
                );
            }
        }
        None => quote! {
            let executor = match ::nros::Executor::open(&config) {
                ::core::result::Result::Ok(executor) => executor,
                ::core::result::Result::Err(e) => {
                    ::log::error!("nros: zephyr entry — executor open failed: {:?}", e);
                    return ::core::result::Result::Err(
                        ::nros::__macro_support::nros_platform::RuntimeError::Spin,
                    );
                }
            };
            let mut node_runtime = ::nros::ExecutorNodeRuntime::from_executor(executor);
            let mut ctx = ::nros::__macro_support::nros_platform::RuntimeCtx::with_runtime(
                &mut node_runtime,
            );
            let runtime = &mut ctx;
            // Issue #128 — OwnedSpin parity for the capability emits. Param
            // services BEFORE the node registers (the store must exist when
            // each cell captures it — W4c), lifecycle AFTER (the executor is
            // built, the nodes are installed). Both are inert token streams
            // when system.toml doesn't declare them, and no-ops without the
            // `nros/param-services` / `nros/lifecycle-services` features, so
            // plain pub/sub Zephyr entries are byte-identical to pre-#128.
            #param_services_call
            #( #register_calls )*
            #lifecycle_call
            ::log::info!(
                "nros: zephyr workspace entry up ({} nodes)",
                #num_register_calls
            );

            // Forever-spin: native_sim is `no_std`, so the OwnedSpin
            // `NROS_ENTRY_*` bounded path (which needs `std::time`) does
            // not apply. The runtime drives the launch node set (the
            // talker's timer publishes); the workspace E2E observes
            // delivery from an external listener and stops the process.
            loop {
                let _ = runtime.runtime.spin_once(10);
            }
        },
    };

    let body_ts: proc_macro2::TokenStream = match framework {
        Framework::OwnedSpin => quote! {
            // Phase 213.C follow-up — emit two cfg-gated entry shapes so
            // both hosted (POSIX / NuttX / threadx-linux) and embedded
            // (FreeRTOS / bare-metal `target_os = "none"`) targets resolve
            // a working `main`. The shared body is factored into a private
            // `__nros_entry_run` returning `Result` so neither arm
            // duplicates the closure logic.
            fn __nros_entry_run() -> ::core::result::Result<
                (),
                ::nros::__macro_support::nros_platform::RuntimeError,
            > {
                // Phase 244.D1 — custom-transport install seam. Install a
                // board custom transport (e.g. the XRCE-over-UART vtable)
                // selected by `deploy.transport`, BEFORE the RMW registers.
                // XRCE's `set_custom_transport_ops` must precede `register`.
                // The default `setup_transport` is a no-op so boards with a
                // built-in transport are byte-identical. This call is always
                // emitted (it is not dead code): see `BoardEntry::setup_transport`
                // for the design rationale and the current override in
                // `nros-board-mps2-an385` (`xrce-transport` feature).
                #[cfg(target_os = "none")]
                <#board_path as ::nros::__macro_support::nros_platform::BoardEntry>::setup_transport(
                    &#deploy_overlay_ts,
                );
                // Phase 249 P1 — the RMW backend register is OWNED BY THE BOARD
                // (Phase 248 C5a: each board's boot path calls its linked
                // `nros_rmw_<x>::register()`, gated on the board's `rmw-<x>` feature),
                // ordered after `setup_transport` inside the board's `run`. The former
                // `#[cfg(target_os="none")] ::nros::__register_linked_rmw()` emit here
                // was a no-op vestige of the retired linkme-walk path — removed.
                #entry_call
            }

            // `#[allow(dead_code)]` — the multi-tier `run_tiers` entry path
            // (Phase 228.G) owns its own spin and never calls these, so they
            // are unused in that emit; harmless in the single-tier path.
            #[cfg(not(any(target_os = "none", target_os = "nuttx")))]
            #[allow(dead_code)]
            fn __nros_env_usize(name: &str, default: usize) -> usize {
                ::std::env::var(name)
                    .ok()
                    .and_then(|v| v.parse::<usize>().ok())
                    .unwrap_or(default)
            }

            // issue 0274 — unbounded hosted spin (`spin = "forever"` macro
            // arg / `NROS_ENTRY_SPIN_MS=forever` env). Runs until a spin
            // error; the process lifetime is the supervisor's problem.
            #[cfg(not(any(target_os = "none", target_os = "nuttx")))]
            #[allow(dead_code)]
            fn __nros_hosted_spin_forever(
                runtime: &mut ::nros::__macro_support::nros_platform::RuntimeCtx<'_>,
            ) -> ::core::result::Result<
                (),
                ::nros::__macro_support::nros_platform::RuntimeError,
            > {
                loop {
                    runtime
                        .runtime
                        .spin_once(10)
                        .map_err(|_| ::nros::__macro_support::nros_platform::RuntimeError::Spin)?;
                }
            }

            #[cfg(not(any(target_os = "none", target_os = "nuttx")))]
            #[allow(dead_code)]
            fn __nros_hosted_spin_if_requested(
                runtime: &mut ::nros::__macro_support::nros_platform::RuntimeCtx<'_>,
            ) -> ::core::result::Result<
                (),
                ::nros::__macro_support::nros_platform::RuntimeError,
            > {
                // issue 0274 — `NROS_ENTRY_SPIN_MS=forever` joins the numeric
                // values as an env-side unbounded opt-in.
                if ::std::env::var("NROS_ENTRY_SPIN_MS").as_deref() == Ok("forever") {
                    return __nros_hosted_spin_forever(runtime);
                }
                let total_ms = __nros_env_usize("NROS_ENTRY_SPIN_MS", 0);
                if total_ms == 0 {
                    return ::core::result::Result::Ok(());
                }

                let step_ms = __nros_env_usize("NROS_ENTRY_SPIN_STEP_MS", 10)
                    .clamp(1, total_ms.max(1));
                let expect_messages = __nros_env_usize("NROS_ENTRY_EXPECT_MESSAGE_CALLBACKS", 0);
                let deadline = ::std::time::Instant::now()
                    + ::std::time::Duration::from_millis(total_ms as u64);

                loop {
                    runtime
                        .runtime
                        .spin_once(step_ms as u32)
                        .map_err(|_| ::nros::__macro_support::nros_platform::RuntimeError::Spin)?;

                    let (callbacks, messages) = runtime.runtime.observed_callback_counts();
                    if expect_messages > 0 && messages >= expect_messages {
                        ::std::println!(
                            "nros: hosted spin complete callbacks={callbacks} message_callbacks={messages}"
                        );
                        return ::core::result::Result::Ok(());
                    }

                    if ::std::time::Instant::now() >= deadline {
                        ::std::println!(
                            "nros: hosted spin complete callbacks={callbacks} message_callbacks={messages}"
                        );
                        if expect_messages > 0 {
                            return ::core::result::Result::Err(
                                ::nros::__macro_support::nros_platform::RuntimeError::Spin,
                            );
                        }
                        return ::core::result::Result::Ok(());
                    }
                }
            }

            // phase-359 W7 — NuttX takes the C-ABI `main`, not libstd's.
            //
            // NuttX is POSIX-shaped, so this used to fall into the hosted arm
            // below and libstd's `lang_start` wrapped the Rust `main`. The
            // family is `no_std` now, which removes `lang_start`, so the entry
            // symbol NuttX's task dispatch calls has to be emitted directly —
            // the same shape the `target_os = "none"` C-runtime boards
            // (FreeRTOS / threadx-linux) already use, for the same reason: a C
            // runtime calls `main`, and nothing is left to wrap it.
            //
            // The status mapping matches what `lang_start` did with a
            // `Result`-returning main: `Ok` -> 0, `Err` -> 1. Entry leaves on
            // this target carry `#![no_std]` + `#![no_main]`, as the other
            // C-runtime families' leaves do.
            //
            // The `(argc, argv)` signature is not decoration: the board's
            // `nsh_main` (nros-board-nuttx-qemu `entry.rs`) declares
            // `fn main(argc: i32, argv: *const *const c_char) -> i32` and calls
            // it with both. Ignoring them would work by accident on these ABIs
            // — the caller passes them in registers the callee may leave alone
            // — but the definition should match the declaration that already
            // exists rather than rely on that.
            #[cfg(target_os = "nuttx")]
            #[unsafe(no_mangle)]
            pub extern "C" fn main(
                _argc: i32,
                _argv: *const *const ::core::ffi::c_char,
            ) -> i32 {
                match __nros_entry_run() {
                    ::core::result::Result::Ok(()) => 0,
                    ::core::result::Result::Err(_) => 1,
                }
            }

            #[cfg(not(any(target_os = "none", target_os = "nuttx")))]
            fn main() {
                if let ::core::result::Result::Err(e) = __nros_entry_run() {
                    ::std::eprintln!("{}: {}", ::core::env!("CARGO_PKG_NAME"), e);
                    ::std::process::exit(1);
                }
            }

            // Phase 244.D1 — `target_os = "none"` entry: `extern "C" fn main`
            // for C-runtime boards (FreeRTOS / threadx-linux), or a
            // `#[cortex_m_rt::entry]` reset for pure bare-metal Cortex-M.
            #none_entry_ts
        },
        // Phase 225.P — Zephyr framework. The RTOS owns boot + the C
        // `main`; the Rust staticlib exports `rust_main`, which
        // `zephyr-lang-rust`'s `rust_cargo_application()` invokes after
        // kernel + net init. There is NO `BoardEntry::run` and NO Rust
        // `fn main` (Zephyr forbids it). The launch file remains the
        // single source of truth for the node set: `register_calls`
        // registers each launch-named Node pkg, identical to the
        // native/freertos OwnedSpin shape — only the boot/spin scaffold
        // differs. Generalises the single-node
        // `nros::zephyr_component_main!` body to N launch-named nodes.
        Framework::Zephyr => quote! {
            // `rust_main` is the only entry symbol Zephyr links (the RTOS
            // owns boot + the C `main`). native_sim builds for
            // `x86_64-unknown-none` — a `no_std` target — so `std` is
            // unavailable; observability goes through the `log` facade,
            // routed to Zephyr's logger. Errors can't cross the C ABI, so
            // they are logged and the `Result` is dropped here.
            #[unsafe(no_mangle)]
            pub extern "C" fn rust_main() {
                // SAFETY: `set_logger` is callable once post-kernel-init.
                unsafe { let _ = ::zephyr::set_logger(); }
                // issue 0460 — this was `let _ = __nros_zephyr_entry_run();`,
                // which is the silent early-return this project bans at
                // runtime. The comment above already SAID errors are "logged
                // and the Result dropped"; only the dropping was implemented.
                //
                // A successful entry never returns (it spins forever), so ANY
                // return is a failure. Returning quietly leaves Zephyr's main
                // thread terminated and only kernel threads alive, so the image
                // idles to the test's timeout having printed nothing after
                // "Network ready" — three entry_matrix cells (zephyr/rust
                // params, qos, lifecycle) presented exactly that way, and the
                // absence of any application thread in a gdb dump is what
                // finally identified it.
                if let Err(e) = __nros_zephyr_entry_run() {
                    ::log::error!("nros: zephyr entry FAILED: {:?}", e);
                    ::core::panic!("nros: zephyr entry failed: {:?}", e);
                }
                ::log::error!(
                    "nros: zephyr entry RETURNED without error — the spin loop \
                     exited, which a running entry never does"
                );
                ::core::panic!("nros: zephyr entry returned unexpectedly");
            }

            fn __nros_zephyr_entry_run() -> ::core::result::Result<
                (),
                ::nros::__macro_support::nros_platform::RuntimeError,
            > {
                // Carrier / link-up gate. Use the `nros_platform::zephyr::
                // wait_network` C-symbol wrapper (Phase 248 C7 step 1 — relocated
                // from `nros::platform::zephyr`) — it exposes a real linkable
                // symbol. (`ZephyrBoard::wait_link_up` calls Zephyr's
                // `net_if_is_up` / `k_msleep`, which are `static inline` header
                // functions with no link symbol, so the native_sim final link
                // fails with undefined references.)
                let _ = ::nros_platform::zephyr::wait_network(2000);

                // Issue #129 (RFC-0031 C5b amendment) — explicit backend register.
                // Zephyr has no BoardEntry boot path to own it (the C5a home) and
                // `.init_array` ctors are compiled out on `target_os = "none"`, so
                // the codegen emits the register from the entry's own backend dep
                // (`rmw-<x> = ["dep:nros-rmw-<x>"]`). Without it the CFFI registry
                // is empty and `Executor::open` fails Transport(ConnectionFailed).
                #zephyr_rmw_register_ts

                // Phase 249 P1 — RMW register is board-owned (Phase 248 C5a); the
                // backend-agnostic `nros` crate cannot register (no backend dep), so
                // the former no-op `::nros::__register_linked_rmw()` emit is removed.

                // Open the executor + wrap it in the dispatch runtime, then
                // register each launch-named Node pkg through a `RuntimeCtx`
                // — exactly the `<pkg>::register(runtime)?` flow the native
                // entry uses, so the launch file stays the single source of
                // truth.
                // phase-427 W9 — the baked identity has ONE source,
                // `nros::Context::baked()`, which `default_from_env()` resolves
                // to on this `no_std` target (the hosted `from_env()` is
                // unavailable). This arm used to read `option_env!("NROS_LOCATOR")`
                // itself — and had lost the `NROS_DOMAIN_ID` half that
                // `nros::zephyr_component_main!` carried (issue 0161's class),
                // so a workspace Zephyr entry ran domain 0 whatever Kconfig
                // said. Kconfig reaches `nros`'s own build through
                // `nros_zephyr_build::bake_nros_config()` in its `build.rs`
                // (`CONFIG_NROS_ZENOH_LOCATOR` / `CONFIG_NROS_DOMAIN_ID`), the
                // same channel the entry's `build.rs` uses for itself, so
                // Kconfig stays the single source of truth for both languages.
                //
                // #166 / phase-286 W1 — native_sim test parallelism. The test
                // harness launches the image with `-testargs --nros-locator=<loc>`
                // and starts a per-test zenohd on that ephemeral port; preferring
                // it over the compile-time bake lets every test dial a DISTINCT
                // router, retiring the shared-baked-port serialization of the
                // ws-entry lane. Provided by `nros-platform-zephyr` (argv-backed,
                // process lifetime); returns NULL on real embedded → the bake
                // stands. The hook is an `extern "C"` only a Zephyr image
                // defines, so it is resolved HERE and handed to the context.
                // Mirrors `nros::zephyr_component_main!`.
                unsafe extern "C" {
                    fn nros_runtime_locator_override() -> *const ::core::ffi::c_char;
                }
                let runtime_locator: ::core::option::Option<&str> = {
                    let p = unsafe { nros_runtime_locator_override() };
                    if p.is_null() {
                        ::core::option::Option::None
                    } else {
                        unsafe { ::core::ffi::CStr::from_ptr(p) }.to_str().ok()
                    }
                };
                let ctx = match ::nros::Context::default_from_env() {
                    ::core::result::Result::Ok(ctx) => ctx.with_locator_override(runtime_locator),
                    ::core::result::Result::Err(e) => {
                        ::core::panic!("nros: zephyr entry: baked Context is invalid: {:?}", e)
                    }
                };
                let config = ctx.config(::core::env!("CARGO_PKG_NAME"));
                // issue #128 (half 2) — spin-or-tiers tail: multi-tier
                // systems route through `ZephyrBoard::run_tiers`; single-tier
                // keeps the plain single-executor register+spin body.
                #zephyr_body_tail
            }
        },
        // Phase 225.O — ESP32-C3 (esp-hal) framework. esp-riscv-rt's
        // `_start` registers + jumps to the esp-hal entry, so the boot
        // symbol must be a `#[::esp_hal::main] fn main() -> !` — the
        // `OwnedSpin` bare `extern "C" fn main` does not boot. The board
        // ZST's real-runtime `BoardEntry::run` builds the `Config`,
        // brings up hardware + transport, opens the executor, registers
        // each launch-named Node pkg through the `RuntimeCtx` closure
        // (identical `<pkg>::register(runtime)?` flow to native/freertos
        // — the launch file stays the single source of truth), and spins
        // forever (`run` never returns on ESP32). The trailing `loop` is
        // defensive (satisfies `-> !`; unreachable in a working build).
        // The Entry crate provides the panic handler (`esp-backtrace`)
        // and app descriptor (`esp_app_desc!`).
        Framework::Esp32 => quote! {
            #[::esp_hal::main]
            fn main() -> ! {
                let _ = __nros_esp32_entry_run();
                #[allow(clippy::empty_loop)]
                loop {
                    ::core::hint::spin_loop();
                }
            }

            fn __nros_esp32_entry_run() -> ::core::result::Result<
                (),
                ::nros::__macro_support::nros_platform::RuntimeError,
            > {
                // Phase 244.D2 — `run_with_deploy` (not `run`) so the
                // `[package.metadata.nros.deploy.<board>]` overlay (locator / ip /
                // domain) reaches `BoardEntry::run_with_deploy`; with `run` the
                // overlay was inert and both esp32 nodes used the board-default
                // net. Boards without an override fall back to `run` via the
                // default trait body, so non-overlay esp32 builds are unchanged.
                <#board_path as ::nros::__macro_support::nros_platform::BoardEntry>::run_with_deploy(
                    &#deploy_overlay_ts,
                    |runtime: &mut ::nros::__macro_support::nros_platform::RuntimeCtx<'_>|
                        -> ::core::result::Result<
                            (),
                            ::nros::__macro_support::nros_platform::RuntimeError,
                        >
                    {
                        // Issue #128 — OwnedSpin parity: param services before
                        // the registers, lifecycle after. Inert without the
                        // system.toml declarations / cargo features.
                        #param_services_call
                        #( #register_calls )*
                        #lifecycle_call
                        ::core::result::Result::Ok(())
                    },
                )
            }
        },
        Framework::Rtic => {
            let deploy = deploy_for_framework.as_deref().ok_or_else(|| {
                syn::Error::new(
                    Span::call_site(),
                    "nros::main!: RTIC framework requires `[package.metadata.nros.entry] deploy`",
                )
            })?;
            let rtic_spec = rtic_board_spec_for(deploy).ok_or_else(|| {
                syn::Error::new(
                    Span::call_site(),
                    format!("nros::main!: missing RTIC board spec for deploy `{deploy}`"),
                )
            })?;
            let rtic_device = rtic_spec.device_path;
            let rtic_dispatchers = rtic_spec.dispatchers;
            let rtic_consumer = rtic_spec.dispatch_consumer_path;
            // Phase 289 (#178) — optional periodic-tick hardware task. The
            // `binds` route wires the handler through RTIC's real vector
            // table (the same mechanism the dispatchers use) — a board-crate
            // `#[exception] SysTick` does NOT get wired (verified in #178).
            let rtic_tick_ts: proc_macro2::TokenStream = match rtic_spec.tick_irq {
                Some(tick_irq) => quote! {
                    /// Phase 289 — periodic tick. Priority 2 so it PREEMPTS
                    /// the priority-1 `__nros_run` task: its whole job is
                    /// waking the `wfi` inside that task's connect/poll
                    /// busy-waits. The board acknowledges the IRQ in
                    /// `on_tick` (an unacknowledged flag is an IRQ storm).
                    #[task(binds = #tick_irq, priority = 2)]
                    fn __nros_tick(_cx: __nros_tick::Context) {
                        <__NrosBoard as ::nros::__macro_support::nros_platform::RticBoardEntry>::on_tick();
                    }
                },
                None => quote! {},
            };

            // Phase 216.B.3 SKELETON — `#[rtic::app(...)]` module that
            // delegates boot to `RticBoardEntry::init_hardware`. The
            // full body (the `__nros_spin` + `__nros_dispatch` software
            // tasks + per-Node register/spawn wiring) lands in a
            // 216.B.3 follow-up. Today's emit only needs to compile so
            // the route through `framework_for(deploy)` is observable
            // — runtime use surfaces the board crate's `todo!()` in
            // `init_hardware` (intentional).
            //
            // Phase 216.B.4 adds the `custom_tasks = [...]` splice:
            // each user-listed ident `f` becomes a thin `#[task]`
            // trampoline that awaits `super::<f>_impl(cx).await`. The
            // user supplies the impl fn (and its `Context` type-alias
            // arg — RTIC generates `<f>::Context` from the task ident)
            // outside the macro; the macro just declares the task.
            //
            // Hardcoded `dispatchers = [UARTRX0, UARTTX0]` matches
            // `RticMps2An385::DISPATCHERS`. A follow-up reads the const
            // from the board crate at macro-expansion time (requires a
            // build-graph fs round-trip we want to defer).
            //
            // The `#![no_std]`/`#![no_main]` inner attrs are NOT emitted
            // here — the Entry pkg's `main.rs` already declares those
            // at file scope (see the talker-rtic example). Emitting
            // them inside the macro would double-declare.
            let custom_task_items: Vec<proc_macro2::TokenStream> = args
                .custom_tasks
                .as_deref()
                .unwrap_or(&[])
                .iter()
                .map(|f| {
                    // Sibling impl fn name: `<f>_impl`. Defined at
                    // module scope outside the `mod __nros_app` body
                    // by the user — the trampoline reaches it via
                    // `super::<f>_impl`.
                    let impl_ident = Ident::new(&format!("{}_impl", f), f.span());
                    quote! {
                        #[task(priority = 1)]
                        async fn #f(cx: #f::Context) {
                            super::#impl_ident(cx).await;
                        }
                    }
                })
                .collect();

            quote! {
                use #board_path as __NrosBoard;

                #[::rtic::app(
                    device = #rtic_device,
                    dispatchers = [#( #rtic_dispatchers ),*]
                )]
                mod __nros_app {
                    use super::*;

                    // rtic 2.3.0 asserts `Send` on late `#[local]` resources
                    // (initialized in `#[init]`, claimed by a task at a
                    // different priority). The `rmw-cffi` `Executor<'static>`
                    // holds a raw `*mut CffiSession`, so it is `!Send`. These
                    // RTIC boards are single-core cortex-m built with
                    // `critical-section-single-core`: the executor/runtime
                    // never cross cores, so the `Send` requirement is a
                    // structural formality. Wrap the resources in a cell that
                    // is unconditionally `Send` to satisfy the bound.
                    struct __NrosLocalCell<T>(T);
                    // SAFETY: single-core cortex-m — no other core can observe
                    // this value; RTIC serializes access via priority ceilings.
                    unsafe impl<T> ::core::marker::Send for __NrosLocalCell<T> {}

                    #[shared]
                    struct Shared {}

                    /// Phase 216.B.3 follow-up — stashes the
                    /// `(Executor, Runtime)` pair returned by
                    /// `RticBoardEntry::init_hardware` so the
                    /// `__nros_spin` / `__nros_dispatch` software
                    /// tasks can take ownership through the RTIC
                    /// `local = [<field>]` attribute. The assoc-type
                    /// projection keeps the macro emit board-agnostic
                    /// — every `RticBoardEntry` impl picks its own
                    /// concrete `Executor` / `Runtime` types.
                    #[local]
                    struct Local {
                        // #178 — the executor's zenoh session open is a BLOCKING
                        // connect (smoltcp poll loop needs the timer + RX IRQ). RTIC
                        // runs `#[init]` with interrupts masked, so opening there
                        // deadlocks the handshake. Instead `#[init]` stashes the
                        // board's `Boot` carrier (hardware already up, no network
                        // I/O) and the `__nros_run` task opens the executor on its
                        // first poll — after `init` returns and interrupts unmask.
                        // `Option` so the task can move the `Boot` out of `#[local]`.
                        boot: __NrosLocalCell<::core::option::Option<<__NrosBoard as ::nros::__macro_support::nros_platform::RticBoardEntry>::Boot>>,
                        runtime: __NrosLocalCell<<__NrosBoard as ::nros::__macro_support::nros_platform::RticBoardEntry>::Runtime>,
                    }

                    #[init]
                    fn init(cx: init::Context) -> (Shared, Local) {
                        // Phase 216.B.3 follow-up — board bring-up
                        // hands back the `(Executor, Runtime)` pair;
                        // we stash both into `Local` and spawn the two
                        // software tasks that will drive them. The
                        // `run_plan(&mut runtime)` per-Node
                        // register-dispatch-slot call still belongs to
                        // a separate follow-up (the trampoline
                        // registration story spans the macro +
                        // `nros::node!()` emit + the runtime trait
                        // surface; same deferred story as the Embassy
                        // sibling in the C.3 follow-up).
                        // Phase 244.D1 — thread the `[deploy.<board>]` overlay
                        // into the RTIC `#[init]` so each Entry pkg pins its own
                        // ip / locator (the default impl ignores it, so boards
                        // without a baked net Config are unchanged).
                        //
                        // #178 — `init_hardware_with_deploy` now brings up the
                        // hardware and returns a `Boot` carrier WITHOUT opening the
                        // executor (no blocking network I/O here — interrupts are
                        // masked). The `__nros_run` task calls `open_executor(boot)`
                        // + `register_dispatch` once interrupts are live.
                        let (boot, runtime) =
                            <__NrosBoard as ::nros::__macro_support::nros_platform::RticBoardEntry>::init_hardware_with_deploy(
                                cx.device,
                                cx.core,
                                &#deploy_overlay_ts,
                            );
                        __nros_run::spawn().unwrap();
                        (
                            Shared {},
                            Local {
                                boot: __NrosLocalCell(::core::option::Option::Some(boot)),
                                runtime: __NrosLocalCell(runtime),
                            },
                        )
                    }

                    /// RTIC run task — collapsed `__nros_spin` +
                    /// `__nros_dispatch` (Phase 216.B.3 follow-up).
                    ///
                    /// The earlier split-task shape had `__nros_spin`
                    /// own the `Executor` half and `__nros_dispatch`
                    /// own the `Runtime` half. RTIC `#[local]` fields
                    /// are claimed by a single task (the
                    /// `local = [<field>]` attribute is exclusive),
                    /// and the dispatch task needs the executor to
                    /// drive the per-Node trampolines that run inside
                    /// the executor's spin loop. Collapsing into one
                    /// task that owns both fields side-steps the
                    /// exclusivity rule and gives the spin / dequeue
                    /// loop a single coherent borrow.
                    ///
                    /// Body:
                    ///   1. Claim the board's SPSC consumer half via
                    ///      `take_dispatch_consumer()` (stashed by
                    ///      `RticBoardEntry::init_hardware`).
                    ///   2. Drive `executor.spin_once(small_dur)`
                    ///      — small budget so the dequeue loop runs
                    ///      between executor iterations.
                    ///   3. Drain whatever the SPSC has for this
                    ///      cycle. Today each dequeued envelope is
                    ///      dropped with a TODO: per-Node trampoline
                    ///      routing needs an `ExecutorNodeRuntime`-
                    ///      wrapped sink that the macro emit hasn't
                    ///      plumbed yet (the `dispatch_callback`
                    ///      entry on `ExecutorNodeRuntime` is wired
                    ///      separately; the trampoline registry that
                    ///      pairs `cb_id` → Node pkg is the next 216
                    ///      follow-up — likely via `linkme`).
                    ///
                    /// Splitting the tasks back apart once the
                    /// `ExecutorNodeRuntime` sink is plumbed (so the
                    /// spin task can run at lower priority than the
                    /// dispatch task) is a separate follow-up.
                    #[task(local = [boot, runtime], priority = 1)]
                    async fn __nros_run(cx: __nros_run::Context) {
                        // Phase 289 (#178 layer 2) — install the board's
                        // idle-yield hooks (e.g. `wfi` on the busy-wait
                        // sites) now that interrupts are unmasked and the
                        // tick IRQ is armed. MUST precede `open_executor`:
                        // the blocking connect below is the busy-wait the
                        // yield exists for.
                        <__NrosBoard as ::nros::__macro_support::nros_platform::RticBoardEntry>::on_interrupts_live();
                        // #178 — open the executor HERE, not in `#[init]`.
                        // `Executor::open` performs the blocking zenoh-pico
                        // session open (a TCP connect driven by the smoltcp poll
                        // loop, which needs the timer tick + RX IRQ). This task
                        // runs after `#[init]` returns and interrupts are unmasked,
                        // so the handshake can complete; opening in `#[init]`
                        // (interrupts masked) deadlocks it.
                        let executor =
                            <__NrosBoard as ::nros::__macro_support::nros_platform::RticBoardEntry>::open_executor(
                                cx.local.boot.0.take().expect("RTIC boot carrier already taken"),
                            );
                        // Phase 289 (#178) — wrap the executor in the SAME
                        // `ExecutorNodeRuntime` every owned-spin board uses and
                        // run each Node pkg's full `register()` (entities +
                        // tick registry + component table) against it. The
                        // earlier `register_dispatch`-only wiring installed the
                        // on_callback trampoline but never created the node /
                        // publisher / timer entities, so the image opened its
                        // session and then published nothing.
                        let mut __nros_crt =
                            ::nros::node_runtime::ExecutorNodeRuntime::from_executor(executor);
                        {
                            let mut __nros_rt =
                                ::nros::__macro_support::nros_platform::RuntimeCtx::with_runtime(&mut __nros_crt);
                            #( #framework_register_entity_calls )*
                        }
                        // The board-side runtime owns the SPSC
                        // producer half. Today's collapse keeps it in
                        // `Local` for symmetry with the planned split
                        // — once `ExecutorNodeRuntime`-wrapped routing
                        // lands the runtime's `signal_callback` will
                        // be the producer-side bridge between executor
                        // callbacks and the SPSC consumer drained
                        // below.
                        let _runtime = &mut cx.local.runtime.0;
                        let mut consumer =
                            #rtic_consumer()
                                .expect("RTIC dispatch consumer take");
                        loop {
                            // Phase 289 — trait spin (`spin_once(ms)` +
                            // `run_ticks`), matching the owned-spin boards, so
                            // registered timers fire and service/action poll
                            // components tick.
                            let _ = ::nros::__macro_support::nros_platform::NodeDispatchRuntime::spin_once(
                                &mut __nros_crt,
                                10,
                            );
                            // Phase 216 final dispatch wiring — drain
                            // every envelope the board's SPSC has
                            // queued this cycle and forward each one
                            // through `Executor::dispatch_callback`.
                            // The layer-clean `(cb_id: &str,
                            // ctx_ptr: *mut c_void)` shape matches both
                            // the dequeued `SignaledCallback<'static>`
                            // payload and the executor's stable entry
                            // point — no type-translation gymnastics.
                            // The executor-side body is a no-op stub
                            // today; the per-Node trampoline registry
                            // (linkme / Phase 216 follow-up) fills it
                            // in by resolving `cb_id` →
                            // `__nros_node_<pkg>_on_callback` and
                            // invoking with the per-pkg state blob.
                            // What this commit closes is the gap
                            // where the dequeued envelope was being
                            // silently dropped with a TODO — values
                            // now flow into the executor's stable
                            // surface and the registry lands as a
                            // body fill, not a macro rewrite.
                            while let Some(envelope) = consumer.dequeue() {
                                let cb = envelope.into_inner();
                                // SAFETY: the board's `signal_callback` enqueues a
                                // `*mut CallbackCtx<'static>` (see the rtic board
                                // crates); single-core, drained on this task only.
                                let ctx = unsafe {
                                    &mut *(cb.ctx_ptr as *mut ::nros::CallbackCtx<'static>)
                                };
                                __nros_crt.dispatch_callback(cb.cb_id, ctx);
                            }
                        }
                    }

                    // Phase 289 (#178) — periodic tick hardware task
                    // (empty when the board spec declares no tick_irq).
                    #rtic_tick_ts

                    // Phase 216.B.4 — user-supplied `#[task]` trampolines.
                    // Each calls a sibling `<name>_impl` fn the user
                    // defines at module scope; signatures are kept
                    // simple to dodge cross-pkg type-alias plumbing.
                    #( #custom_task_items )*
                }
            }
        }
        Framework::Embassy => {
            // Phase 216.C.3 follow-up — sibling of the Rtic emit above.
            // Emits `#[embassy_executor::main] async fn main(spawner)`
            // that delegates to `EmbassyBoardEntry::init_hardware`
            // (which is **sync** — see `embassy_entry.rs`'s "Sync
            // `init_hardware`" note; matches `RticBoardEntry`) and
            // then spawns two `#[embassy_executor::task]` fns:
            //
            // - `__nros_spin_task(executor)` — long-lived task that
            //   drives the executor. The real body will dequeue from
            //   the board's `CALLBACK_CHANNEL` and invoke per-Node
            //   trampolines; today it parks on `Timer::after_secs` so
            //   the macro emit compiles standalone. The dequeue +
            //   trampoline-lookup integration lands alongside the
            //   B.3-equivalent RTIC `__nros_spin` body fill — the
            //   trampoline registration story spans the macro, the
            //   `nros::node!()` emit, and
            //   `register_dispatch_slot_dyn`, which is substantial
            //   integration work for a separate follow-up.
            //
            // - `__nros_dispatch_task(runtime)` — long-lived task that
            //   calls `runtime.spin_once(timeout_ms)` in a loop with an
            //   `embassy_time` yield between iterations. Same
            //   placeholder shape as the spin task — the
            //   `register_dispatch_slot_dyn(...)` registration call
            //   (the `run_plan(&mut runtime)` story) is the
            //   integration work deferred to the same follow-up.
            //
            // Task argument types resolve via the board's
            // `EmbassyBoardEntry::{Executor, Runtime}` associated
            // types; `#[embassy_executor::task]` doesn't accept
            // generic params, so we name them concretely through the
            // assoc-type projection.
            //
            // The `#![no_std]`/`#![no_main]` inner attrs are NOT
            // emitted here — the Entry pkg's `main.rs` already
            // declares those at file scope (mirrors the Rtic branch).
            quote! {
                use #board_path as __NrosBoard;

                /// Embassy run task — collapsed `__nros_spin_task` +
                /// `__nros_dispatch_task` (Phase 216.C.3 follow-up).
                ///
                /// `#[embassy_executor::task]` doesn't accept multiple
                /// generic params and the spin + dispatch loops need
                /// to share the `(Executor, Runtime)` pair so the
                /// per-callback routing (once plumbed via the
                /// `ExecutorNodeRuntime::dispatch_callback` sink in
                /// `packages/api/nros/src/node_runtime.rs`) can drain
                /// the board's static `CALLBACK_CHANNEL` between
                /// executor iterations. Collapsing the two tasks into
                /// one gives the spin + drain loops a single coherent
                /// borrow over both halves.
                ///
                /// Body:
                ///   1. Drive `executor.spin_once(small_dur)` — a
                ///      small budget so the loop can yield between
                ///      iterations.
                ///   2. Yield to the Embassy scheduler via
                ///      `Timer::after_millis(1)` so other tasks
                ///      (Ethernet driver, user-spawned tasks) run.
                ///
                /// What's still placeholder: a dequeue + dispatch
                /// step. `EmbassyRuntime` owns a `&'static` borrow
                /// of the board's private `CALLBACK_CHANNEL`, but
                /// no public accessor exposes the receiver half today
                /// — adding one is the next 216.C follow-up, paired
                /// with the per-Node trampoline registry (linkme) and
                /// the `ExecutorNodeRuntime`-wrapped sink the macro
                /// emit needs to plumb in order to call
                /// `dispatch_callback`. Splitting the tasks back
                /// apart once that lands is a separate follow-up.
                #[::embassy_executor::task]
                async fn __nros_run_task(
                    mut executor: <__NrosBoard as ::nros::__macro_support::nros_platform::EmbassyBoardEntry>::Executor,
                    runtime: <__NrosBoard as ::nros::__macro_support::nros_platform::EmbassyBoardEntry>::Runtime,
                ) {
                    loop {
                        let _ = executor.spin_once(
                            ::core::time::Duration::from_millis(1),
                        );
                        // Phase 216 final dispatch wiring — drain
                        // every envelope the board's
                        // `CALLBACK_CHANNEL` has queued this cycle
                        // and forward each one through
                        // `Executor::dispatch_callback`.
                        // `EmbassyRuntime::take()` (Phase 216
                        // final, sibling of the RTIC SPSC `dequeue`
                        // path) is non-blocking so the spin loop
                        // keeps yielding even when no callback is
                        // signaled — the `Timer::after_millis(1)`
                        // below paces the executor poll without
                        // needing `embassy_futures::select` (not a
                        // current dep). The same layer-clean
                        // `(cb_id: &str, ctx_ptr: *mut c_void)` shape
                        // applies — see the RTIC sibling for the
                        // matching commentary. The executor-side
                        // dispatch body is a no-op stub today; the
                        // per-Node trampoline registry (linkme /
                        // Phase 216 follow-up) fills it in.
                        while let Some(envelope) = runtime.take() {
                            let cb = envelope.into_inner();
                            executor.dispatch_callback(cb.cb_id, cb.ctx_ptr);
                        }
                        ::embassy_time::Timer::after_millis(1).await;
                    }
                }

                #[::embassy_executor::main]
                async fn main(spawner: ::embassy_executor::Spawner) {
                    // Sync `init_hardware_with_deploy` — see the
                    // `EmbassyBoardEntry` trait "Sync `init_hardware`"
                    // note; matches `RticBoardEntry`. Phase 244.D1 /
                    // issue #98 / RFC-0045 — threads the deploy overlay
                    // so the board can resolve the node name from
                    // `deploy.boot_config` (the default impl ignores it,
                    // so boards without a baked boot config are
                    // unchanged).
                    let (mut executor, runtime) =
                        <__NrosBoard as ::nros::__macro_support::nros_platform::EmbassyBoardEntry>::init_hardware_with_deploy(
                            spawner,
                            &#deploy_overlay_ts,
                        );
                    // Phase 216 final wave — per-Node dispatch
                    // registration. Sibling of the RTIC `#[init]`
                    // splice above; same `register_dispatch(&mut
                    // executor)` shape, populating the executor's
                    // dispatch-slot table before the
                    // `__nros_run_task` is spawned to drain the
                    // board's `CALLBACK_CHANNEL`.
                    #( #framework_register_dispatch_calls )*
                    spawner.spawn(__nros_run_task(executor, runtime)).unwrap();
                }
            }
        }
    };

    // phase-267 W1c/C4 — a `[[bridge]]` system is a cross-RMW gateway: the entry
    // `include_str!`s the `nros-bridge.toml` `nros sync` generated and runs the
    // data-driven `nros_bridge::run_from_config_str` (open_multi + a PubSubBridge
    // per `[[bridge]]` + spin/pump). Replaces the ordinary register/spin body. The
    // Entry pkg deps `nros-bridge` (config feature) + both RMW backends.
    if let Some(cfg_path) = &bridge_config_path {
        let cfg_lit = cfg_path.to_string_lossy();
        let cfg_lit = cfg_lit.as_ref();
        // Issue 0106 — explicitly `register()` each bridge RMW backend so the
        // linker can't dead-strip its self-register `.init_array` ctor. The
        // `run_from_config` body references no backend symbol, so without this
        // the backend is dropped and `open_multi` fails `Transport(
        // InvalidArgument)` (null vtable). Mirrors the board boot path's
        // `nros_rmw_<x>::register()` (Phase 248 C5a). Unknown rmw names map to
        // nothing (the data-driven config still drives the actual open).
        let register_calls: Vec<proc_macro2::TokenStream> = bridge_rmws
            .iter()
            .filter_map(|rmw| rmw_crate_ident(rmw))
            .map(|crate_ident| {
                let id = Ident::new(crate_ident, Span::call_site());
                quote! { let _ = ::#id::register(); }
            })
            .collect();
        // phase-267 (non-flat types) — stage each cyclonedds-egress non-flat type's
        // Cyclone descriptor via a typed `register::<M>()` (reuses `M::FIELDS`, so
        // nested / array / sequence work without a flat schema). The Entry deps the
        // forwarded msg crate(s) + `nros-rmw-cyclonedds`. `let _ =` mirrors the
        // backend register: a failure surfaces downstream as the egress pub error.
        let typed_register_calls: Vec<proc_macro2::TokenStream> = read_register_types(cfg_path)
            .into_iter()
            .filter(|(_, rmw)| rmw == "cyclonedds")
            .filter_map(|(rust_path, _)| syn::parse_str::<SynPath>(&rust_path).ok())
            .map(|path| quote! { let _ = ::nros_rmw_cyclonedds::register::<#path>(); })
            .collect();
        let expanded = quote! {
            #( #tracked_consts )*
            fn main() -> ::core::result::Result<(), ::nros_bridge::ConfigError> {
                #( #register_calls )*
                #( #typed_register_calls )*
                ::nros_bridge::run_from_config_str(::core::include_str!(#cfg_lit))
            }
        };
        return Ok(expanded);
    }

    // phase-366 W7.a / RFC-0077 — the image's ending, emitted from the entry
    // because the entry IS part of the final artifact and a dependency is not.
    //
    // Gated on the target NOT being a hosted one, rather than on
    // `target_os = "none"`. The narrow spelling looked right — it is what the
    // emitted `main` above uses — and is wrong here: NuttX builds
    // `armv7a-nuttx-eabihf`, whose `target_os` is `"nuttx"`, so a
    // `target_os = "none"` gate emits NOTHING for a `no_std` family that needs a
    // handler, which is issue 0617's `#[panic_handler] function required` from
    // the other direction. ESP-IDF (`target_os = "espidf"`) is the same shape.
    //
    // The question this gate is really asking is "does libstd already define the
    // lang item here?", and that is answered by the hosted list, which is short
    // and closed for this tree: native and threadx-linux are Linux, and macOS is
    // a supported host for the native family.
    //
    // The *BSDs are on that list too, and were missing — the mirror of the bug
    // the paragraph above describes. `nros-platform-posix` is POSIX-clean and
    // the BSDs are a supported host family, but `target_os = "freebsd"` is not
    // "linux", "macos" or "windows", so this gate was TRUE there and emitted a
    // `#[panic_handler]` into a libstd image: `duplicate lang item panic_impl`,
    // for every `nros::main!()` entry, at the first line of the build. A list
    // spelled as an allowlist of hosted OSes only works if it actually lists
    // them.
    //
    // …unless this package also produces a `staticlib`, in which case the LIB
    // owns the item for BOTH artifacts and emitting here would be a duplicate.
    // Derived, so every image can write the same `panic = …` regardless of shape.
    let lib_owns_panic = package_emits_staticlib(&manifest_dir.join("Cargo.toml"));
    let panic_ts: proc_macro2::TokenStream = if lib_owns_panic {
        quote! {}
    } else {
        match args.panic {
            PanicPolicy::Platform => quote! {
                #[cfg(not(any(
                    target_os = "linux",
                    target_os = "macos",
                    target_os = "windows",
                    target_os = "freebsd",
                    target_os = "netbsd",
                    target_os = "openbsd",
                    target_os = "dragonfly"
                )))]
                ::nros::panic_to_platform!();
            },
            PanicPolicy::Halt => quote! {
                #[cfg(not(any(
                    target_os = "linux",
                    target_os = "macos",
                    target_os = "windows",
                    target_os = "freebsd",
                    target_os = "netbsd",
                    target_os = "openbsd",
                    target_os = "dragonfly"
                )))]
                ::nros::panic_halt!();
            },
            // `own` emits nothing — the image said it brings its own.
            PanicPolicy::Own => quote! {},
        }
    };

    let expanded = quote! {
        // Phase 212.N.9 — rebuild-tracking workaround. Stable Rust
        // proc-macros can't use `proc_macro::tracked_path::path()`;
        // anonymous `const _: &[u8] = include_bytes!(...)` items are
        // tracked by cargo's `include_bytes!` and force a recompile
        // when any tracked file changes.
        #( #tracked_consts )*

        // Issue 0257 — const-assert the model's entity count against the
        // callback capacity that actually compiles in (no-op token stream
        // when the board honors the emitted per-entry sizing).
        #capacity_assert

        // W4b — baked boot-config static; emitted before the framework
        // body so `&NROS_BOOT_CONFIG` is in scope at every overlay use site.
        #boot_config_static_ts

        #panic_ts

        #body_ts
    };

    Ok(expanded)
}

/// Issue 0106 — map an `system.toml` rmw name to the backend crate ident the
/// Entry deps (so the macro can emit `nros_rmw_<x>::register()`). Mirrors the
/// orchestration codegen map (`generate.rs` ~2785). `None` for unknown names.
fn rmw_crate_ident(rmw: &str) -> Option<&'static str> {
    match rmw {
        "zenoh" => Some("nros_rmw_zenoh"),
        "cyclonedds" => Some("nros_rmw_cyclonedds_sys"),
        "xrce" => Some("nros_rmw_xrce_cffi"),
        _ => None,
    }
}

/// Phase 216 final wave — read `[package.metadata.nros.entry]
/// node_pkgs = ["pkg_a", "pkg_b"]` from `Cargo.toml`. Each entry names
/// a Node-pkg crate the Entry pkg depends on; the framework emit
/// (RTIC / Embassy) splices `<pkg>::register_dispatch(&mut executor)?;`
/// calls for each into the generated `#[init]` body.
///
/// Returns `Ok(None)` when the key is absent — callers fall back to
/// self-bringup (Entry pkg's own crate name minus the conventional
/// `_entry` suffix, when present).
fn read_entry_node_pkgs(cargo_toml: &Path) -> Result<Option<Vec<String>>, String> {
    let raw = std::fs::read_to_string(cargo_toml).map_err(|e| format!("read: {e}"))?;
    let v: toml::Value = toml::from_str(&raw).map_err(|e| format!("parse toml: {e}"))?;
    let arr = match v
        .get("package")
        .and_then(|p| p.get("metadata"))
        .and_then(|m| m.get("nros"))
        .and_then(|n| n.get("entry"))
        .and_then(|e| e.get("node_pkgs"))
    {
        Some(a) => a,
        None => return Ok(None),
    };
    let list = arr
        .as_array()
        .ok_or_else(|| "`node_pkgs` must be an array of strings".to_string())?;
    let mut out = Vec::with_capacity(list.len());
    for item in list {
        let s = item
            .as_str()
            .ok_or_else(|| "`node_pkgs` entries must be strings".to_string())?;
        out.push(s.to_string());
    }
    Ok(Some(out))
}

/// Read `[package.metadata.nros.entry] deploy = "<board>"` from
/// `Cargo.toml`. The key is mandatory for form-1 (no-arg) invocations.
fn read_entry_deploy(cargo_toml: &Path) -> Result<String, String> {
    let raw = std::fs::read_to_string(cargo_toml).map_err(|e| format!("read: {e}"))?;
    let v: toml::Value = toml::from_str(&raw).map_err(|e| format!("parse toml: {e}"))?;
    let deploy = v
        .get("package")
        .and_then(|p| p.get("metadata"))
        .and_then(|m| m.get("nros"))
        .and_then(|n| n.get("entry"))
        .and_then(|e| e.get("deploy"))
        .and_then(|d| d.as_str())
        .ok_or_else(|| {
            "missing `[package.metadata.nros.entry] deploy = \"<board>\"`".to_string()
        })?;
    Ok(deploy.to_string())
}

/// phase-271 (issue #110) — read the per-entry executor sizing from
/// `[package.metadata.nros.entry] max_callbacks = N` (+ optional
/// `max_sched_contexts = M`). Returns `Some((max_callbacks, max_sched_contexts))`
/// (`max_sched_contexts` defaulting to `0` = "board uses the build default") or
/// `None` when `max_callbacks` is absent (the executor opens at the build-time
/// default `MAX_CBS`/`ARENA_SIZE`). This is the per-entry, NOT workspace-global,
/// knob (issue #0110 fix-idea 2): a fat native entry declares its own callback
/// table size without a `[env] NROS_EXECUTOR_MAX_CBS` that bloats every lean
/// embedded entry in the same workspace. The hosted (posix) board applies it via
/// [`BoardEntry::run_with_deploy_sized`] → `Executor::open_sized`.
/// phase-366 W7 / RFC-0077 — does this package also produce a `staticlib`?
///
/// If it does, the LIB owns the `#[panic_handler]` and `main!()` must not emit
/// one. rustc demands the lang item wherever it emits an archive, from
/// `lib.rs`'s module tree — a handler emitted here, in the bin target, never
/// reaches the `.a`. One in the lib reaches both, because the bin links the rlib
/// and inherits it.
///
/// Note what this is NOT saying (RFC-0077 amendment 2026-08-18b): not "a
/// staticlib is the image". rustc's notion of a final artifact is not the
/// system's — on Zephyr and the ThreadX CMake path that archive is an INPUT to a
/// link step west or CMake owns. The obligation is real either way, which is the
/// whole reason this is derived here rather than asked of the author.
///
/// So placement is DERIVED from the manifest rather than chosen by the author.
/// The alternative was making those crates say `panic = "own"`, which would
/// overload `own` (it means "I bring my own provider") and ask an example author
/// to know a Rust linkage rule to answer a question about panics.
///
/// Text-scanned, not parsed: this crate reads `Cargo.toml` in three other places
/// the same way, and the shape here is a fixed one-line array.
fn package_emits_staticlib(cargo_toml: &Path) -> bool {
    let Ok(text) = std::fs::read_to_string(cargo_toml) else {
        return false;
    };
    let mut in_lib = false;
    for line in text.lines() {
        let t = line.trim();
        if t.starts_with('[') {
            in_lib = t == "[lib]";
            continue;
        }
        if in_lib && t.starts_with("crate-type") && t.contains("staticlib") {
            return true;
        }
    }
    false
}

fn read_entry_executor_sizing(cargo_toml: &Path) -> Option<(usize, usize)> {
    let raw = std::fs::read_to_string(cargo_toml).ok()?;
    let v: toml::Value = toml::from_str(&raw).ok()?;
    let entry = v
        .get("package")?
        .get("metadata")?
        .get("nros")?
        .get("entry")?;
    let max_cbs = entry.get("max_callbacks")?.as_integer()?;
    if max_cbs <= 0 {
        return None;
    }
    let max_sc = entry
        .get("max_sched_contexts")
        .and_then(|x| x.as_integer())
        .filter(|n| *n > 0)
        .unwrap_or(0);
    Some((max_cbs as usize, max_sc as usize))
}

/// Issue 0257 — how the emitted entry sizes its executor.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum EntrySizing {
    /// No per-entry sizing: the board opens at the build-time `MAX_CBS`.
    BuildDefault,
    /// `[package.metadata.nros.entry] max_callbacks` (+ `max_sched_contexts`).
    Declared(usize, usize),
    /// Derived from the model's entity count (issue 0257).
    Derived(usize),
}

/// Issue 0257 — pick the executor sizing the entry emits.
///
/// Explicit metadata always wins: the author may know about entities the model
/// cannot express (timers, guard conditions) — but it must still be big enough
/// for what the model DOES declare, so an explicit value below the modelled
/// count is a hard bake error rather than a runtime `Full`.
///
/// With no explicit value the model's count derives one
/// ([`nros_orchestration_ir::executor_sizing::derive_max_callbacks`]), and only
/// when that lands above the build default — a lean entry (or any model with no
/// wiring) keeps emitting the pre-0257 unsized call.
fn executor_sizing_for(
    declared: Option<(usize, usize)>,
    model_callbacks: usize,
) -> syn::Result<EntrySizing> {
    use nros_orchestration_ir::executor_sizing as sizing;

    if let Some((cbs, sc)) = declared {
        if model_callbacks > cbs {
            return Err(syn::Error::new(
                Span::call_site(),
                format!(
                    "nros::main!: the SystemModel registers {model_callbacks} callback \
                     entities on this entry but `[package.metadata.nros.entry] \
                     max_callbacks = {cbs}` sizes the executor for {cbs}. Raise it to at \
                     least {} in this pkg's Cargo.toml (the count is a lower bound — the \
                     model cannot see timers). issue 0257",
                    sizing::derive_max_callbacks(model_callbacks)
                ),
            ));
        }
        return Ok(EntrySizing::Declared(cbs, sc));
    }
    let derived = sizing::derive_max_callbacks(model_callbacks);
    if derived > sizing::DEFAULT_MAX_CBS {
        return Ok(EntrySizing::Derived(derived));
    }
    Ok(EntrySizing::BuildDefault)
}

/// Issue 0257 — the loud bake-time capacity check, emitted as a `const` item.
///
/// Only boards that override `BoardEntry::run_with_deploy_sized` honor the
/// sizing the entry emits; every firmware board takes the default trait body
/// and opens at the build-time `NROS_EXECUTOR_MAX_CBS`. For those the model's
/// count is the only advance warning available — and the macro cannot read the
/// compiled `MAX_CBS` (it is materialised by `nros-node`'s build script), so
/// the comparison is emitted as a `const` panic against the real const instead
/// of being decided here.
///
/// Emits nothing when the count is unknown/zero, when the board honors the
/// emitted sizing (which by construction covers the count), or when the board
/// is unknown (`board = <Zst>` form — no deploy key to classify).
fn executor_capacity_assert(
    model_callbacks: usize,
    deploy: Option<&str>,
    sizing: EntrySizing,
) -> proc_macro2::TokenStream {
    use nros_orchestration_ir::executor_sizing as sz;

    let Some(key) = deploy else {
        return quote! {};
    };
    if model_callbacks == 0 || sz::board_honors_entry_sizing(key) {
        return quote! {};
    }
    let advice = match sizing {
        EntrySizing::Declared(cbs, _) => format!(
            "this entry declares `max_callbacks = {cbs}`, but board `{key}` ignores \
             per-entry sizing"
        ),
        _ => format!("board `{key}` ignores per-entry sizing"),
    };
    let msg = format!(
        "nros::main!: the SystemModel registers {model_callbacks} callback entities but the \
         executor callback table compiled in is smaller ({advice}). Raise \
         `NROS_EXECUTOR_MAX_CBS` (e.g. to {}) in the entry's `.cargo/config.toml` `[env]` \
         and rebuild from clean — resizing the executor arena mixes stale objects. \
         issue 0257",
        sz::derive_max_callbacks(model_callbacks)
    );
    quote! {
        const _: () = {
            if #model_callbacks > ::nros::__macro_support::EXECUTOR_MAX_CBS {
                ::core::panic!(#msg);
            }
        };
    }
}

/// Issue #48 cause 1 — the deploy-overlay values read from the Entry pkg's
/// `[package.metadata.nros.deploy.<board>]` block. Every field is `Option`
/// (absent key → `None`), baked into a `DeployOverlay` const by
/// [`deploy_overlay_tokens`] and threaded through `BoardEntry::run_with_deploy`.
#[derive(Default)]
struct DeployOverlayLit {
    locator: Option<String>,
    ip: Option<[u8; 4]>,
    gateway: Option<[u8; 4]>,
    netmask: Option<[u8; 4]>,
    domain_id: Option<u32>,
    transport: Option<String>,
    /// Issue #98 — the ROS graph node name for the primary session, set from the
    /// launch file's single `<node name>` (only when the launch declares exactly
    /// one node). NOT read from `[deploy.*]`: it is a launch identity, threaded
    /// in by the caller after the launch is parsed.
    node_name: Option<String>,
}

/// Parse a dotted IPv4 string (`"10.0.2.15"`) into 4 octets. Returns `None`
/// on any malformed input so a bad deploy value silently keeps the board
/// default rather than baking garbage.
fn parse_ipv4_lit(s: &str) -> Option<[u8; 4]> {
    let mut out = [0u8; 4];
    let mut n = 0usize;
    for part in s.split('.') {
        if n >= 4 {
            return None;
        }
        out[n] = part.parse::<u8>().ok()?;
        n += 1;
    }
    if n == 4 { Some(out) } else { None }
}

/// Read `[package.metadata.nros.deploy.<board>]` from the Entry pkg's
/// `Cargo.toml`. Missing block / keys → all-`None` overlay (the firmware keeps
/// its compiled-in `Config::default()`). Only the network/locator/domain keys
/// are consumed here; `rmw` is handled elsewhere (feature/link wiring).
/// Issue #129 (RFC-0031 C5b amendment) — the entry's deploy RMW key
/// (`[package.metadata.nros.deploy.<board>].rmw`). The Zephyr framework arm
/// uses it to emit the explicit `::nros_rmw_<x>::register()` call: Zephyr has
/// no `BoardEntry` boot path to own registration (the FreeRTOS C5a home), the
/// board crate is NetworkWait-only, and `.init_array` ctors don't run on
/// `target_os = "none"` — so per the C5b amendment the ENTRY carries the
/// direct backend dep and codegen emits the register.
fn read_deploy_rmw(cargo_toml: &Path, board_key: &str) -> Option<String> {
    let raw = std::fs::read_to_string(cargo_toml).ok()?;
    let v = toml::from_str::<toml::Value>(&raw).ok()?;
    v.get("package")?
        .get("metadata")?
        .get("nros")?
        .get("deploy")?
        .get(board_key)?
        .get("rmw")?
        .as_str()
        .map(str::to_string)
}

fn read_deploy_overlay(cargo_toml: &Path, board_key: &str) -> DeployOverlayLit {
    let Ok(raw) = std::fs::read_to_string(cargo_toml) else {
        return DeployOverlayLit::default();
    };
    let Ok(v) = toml::from_str::<toml::Value>(&raw) else {
        return DeployOverlayLit::default();
    };
    let Some(block) = v
        .get("package")
        .and_then(|p| p.get("metadata"))
        .and_then(|m| m.get("nros"))
        .and_then(|n| n.get("deploy"))
        .and_then(|d| d.get(board_key))
    else {
        return DeployOverlayLit::default();
    };
    DeployOverlayLit {
        locator: block
            .get("locator")
            .and_then(|x| x.as_str())
            .map(str::to_string),
        ip: block
            .get("ip")
            .and_then(|x| x.as_str())
            .and_then(parse_ipv4_lit),
        gateway: block
            .get("gateway")
            .and_then(|x| x.as_str())
            .and_then(parse_ipv4_lit),
        netmask: block
            .get("netmask")
            .and_then(|x| x.as_str())
            .and_then(parse_ipv4_lit),
        domain_id: block
            .get("domain_id")
            .and_then(|x| x.as_integer())
            .and_then(|i| u32::try_from(i).ok()),
        transport: block
            .get("transport")
            .and_then(|x| x.as_str())
            .map(str::to_string),
        // Issue #98 — not a `[deploy.*]` key; the caller fills this from the
        // parsed launch when it declares exactly one node.
        node_name: None,
    }
}

/// Bake a [`DeployOverlayLit`] into a `nros_platform::DeployOverlay` struct
/// literal (all fields `Option`, so the board overlays only the present ones).
fn deploy_overlay_tokens(lit: &DeployOverlayLit) -> proc_macro2::TokenStream {
    fn opt_ipv4(v: &Option<[u8; 4]>) -> proc_macro2::TokenStream {
        match v {
            Some([a, b, c, d]) => quote! { ::core::option::Option::Some([#a, #b, #c, #d]) },
            None => quote! { ::core::option::Option::None },
        }
    }
    let locator = match &lit.locator {
        Some(s) => quote! { ::core::option::Option::Some(#s) },
        None => quote! { ::core::option::Option::None },
    };
    let ip = opt_ipv4(&lit.ip);
    let gateway = opt_ipv4(&lit.gateway);
    let netmask = opt_ipv4(&lit.netmask);
    let domain_id = match lit.domain_id {
        Some(d) => quote! { ::core::option::Option::Some(#d) },
        None => quote! { ::core::option::Option::None },
    };
    let transport = match &lit.transport {
        Some(s) => quote! { ::core::option::Option::Some(#s) },
        None => quote! { ::core::option::Option::None },
    };
    let node_name = match &lit.node_name {
        Some(s) => quote! { ::core::option::Option::Some(#s) },
        None => quote! { ::core::option::Option::None },
    };
    quote! {
        ::nros::__macro_support::nros_platform::DeployOverlay {
            locator: #locator,
            ip: #ip,
            gateway: #gateway,
            netmask: #netmask,
            domain_id: #domain_id,
            transport: #transport,
            node_name: #node_name,
            boot_config: ::core::option::Option::Some(&NROS_BOOT_CONFIG),
        }
    }
}

/// phase-267 (non-flat types) — read the `[[register_type]]` entries `nros sync`
/// emits into `nros-bridge.toml` for forwarded messages whose schema can't ride
/// the flat `fields` list. Each is `(rust_path, egress_rmw)`; the macro emits a
/// typed `register::<M>()` per cyclonedds egress so the runtime can stage the
/// Cyclone descriptor from `M::FIELDS` (arbitrary nesting). Best-effort: a parse
/// error yields an empty list.
fn read_register_types(bridge_toml: &Path) -> Vec<(String, String)> {
    let Ok(raw) = std::fs::read_to_string(bridge_toml) else {
        return Vec::new();
    };
    let Ok(v) = toml::from_str::<toml::Value>(&raw) else {
        return Vec::new();
    };
    let Some(arr) = v.get("register_type").and_then(|r| r.as_array()) else {
        return Vec::new();
    };
    arr.iter()
        .filter_map(|e| {
            let path = e.get("rust_path")?.as_str()?.to_string();
            let rmw = e.get("rmw")?.as_str()?.to_string();
            Some((path, rmw))
        })
        .collect()
}

// =============================================================================
// Phase 228.G — per-tier resolution inputs (RFC-0032 §6)
// =============================================================================

/// Map the resolved board deploy string to the RTOS key `resolve_tiers` expects
/// (picks the `[tiers.<name>.<rtos>]` sub-table). `None` (explicit `board = X`)
/// defaults to `posix` — the native dev target.
fn derive_target_rtos(deploy: Option<&str>) -> String {
    match deploy {
        Some(d) if d.contains("freertos") => "freertos",
        Some(d) if d.contains("threadx") => "threadx",
        Some(d) if d.contains("nuttx") => "nuttx",
        Some(d) if d.contains("zephyr") => "zephyr",
        _ => "posix",
    }
    .to_string()
}

/// Emit a `&[TierSpec]` literal from the resolved tier table (Phase 228.G,
/// RFC-0032 §5). `priority` is the raw per-RTOS value; `groups` is the tier's
/// distinct callback-group ids (the executor's `active_groups` filter).
fn tier_specs_tokens(table: &ResolvedTierTable) -> proc_macro2::TokenStream {
    let entries = table.tiers.iter().map(|t| {
        let name = &t.name;
        let mut groups: Vec<&str> = t.members.iter().map(|(_, g)| g.as_str()).collect();
        groups.sort();
        groups.dedup();
        let priority = t.priority;
        let stack_bytes = t.stack_bytes.unwrap_or(0) as usize;
        let spin_period_us = t.spin_period_us.unwrap_or(1000);
        let opt_u32 = |v: Option<u32>| match v {
            Some(x) => quote! { Some(#x) },
            None => quote! { None },
        };
        let opt_i64 = |v: Option<i64>| match v {
            Some(x) => quote! { Some(#x) },
            None => quote! { None },
        };
        let opt_u64 = |v: Option<u64>| match v {
            Some(x) => quote! { Some(#x) },
            None => quote! { None },
        };
        let opt_str = |v: Option<&str>| match v {
            Some(x) => quote! { Some(#x) },
            None => quote! { None },
        };
        let core = opt_u32(t.core);
        let preempt_threshold = opt_i64(t.preempt_threshold);
        let time_slice_us = opt_u64(t.time_slice_us);
        let class = opt_str(t.class.as_deref());
        let period_us = opt_u64(t.period_us);
        let budget_us = opt_u64(t.budget_us);
        let deadline_us = opt_u64(t.deadline_us);
        let deadline_policy = opt_str(t.deadline_policy.as_deref());
        quote! {
            ::nros::__macro_support::nros_platform::TierSpec {
                name: #name,
                groups: &[ #(#groups),* ],
                priority: #priority,
                stack_bytes: #stack_bytes,
                spin_period_us: #spin_period_us,
                core: #core,
                preempt_threshold: #preempt_threshold,
                time_slice_us: #time_slice_us,
                class: #class,
                period_us: #period_us,
                budget_us: #budget_us,
                deadline_us: #deadline_us,
                deadline_policy: #deadline_policy,
            }
        }
    });
    quote! { &[ #(#entries),* ] }
}

/// Map a board key from `[package.metadata.nros.entry] deploy = "X"` to the
/// tier-1 board crate's ZST type path.
///
/// The table is maintained in [`nros_orchestration_ir::board_path_for`]
/// (the single source of truth shared with the CLI codegen path). This
/// wrapper parses the returned string into a [`syn::Path`] for token
/// emission. Adding a new board requires editing the IR crate only.
fn board_path_for(deploy: &str) -> Option<SynPath> {
    let path_str = nros_orchestration_ir::board_path_for(deploy)?;
    syn::parse_str::<SynPath>(path_str).ok()
}

fn known_boards_csv() -> &'static str {
    "native, freertos, threadx-linux, threadx-qemu-riscv64, nuttx, nuttx-riscv, esp32-qemu, \
     zephyr, rtic-mps2-an385, qemu-mps2-an385, mps2-an385"
}

/// Phase 244.D1 — does this deploy key name a pure bare-metal Cortex-M
/// direct-exec board? Such boards run `OwnedSpin` but, unlike the FreeRTOS /
/// threadx-linux `target_os = "none"` boards (whose C runtime calls `main`),
/// have no C runtime — the reset vector needs a `#[cortex_m_rt::entry]`. The
/// macro keys the entry-emit shape off this. RTIC bare-metal boards are NOT
/// here: they route through the RTIC framework, which owns its own entry.
fn is_baremetal_cortexm_deploy(deploy: Option<&str>) -> bool {
    matches!(deploy, Some("qemu-mps2-an385" | "mps2-an385"))
}

struct RticBoardSpec {
    device_path: SynPath,
    dispatchers: Vec<Ident>,
    dispatch_consumer_path: SynPath,
    /// Phase 289 (#178) — interrupt ident of the board's periodic tick
    /// timer. The macro emits a `#[task(binds = <tick_irq>, priority = 2)]`
    /// hardware task calling `RticBoardEntry::on_tick()`; the board arms the
    /// timer in `init_hardware` and acknowledges it in `on_tick`. The tick's
    /// job is waking the `wfi` idle-yield inside `__nros_run`'s blocking
    /// connect/poll busy-waits (priority 2 so it preempts the priority-1 run
    /// task). `None` = no tick task emitted (board runs without wfi-yield).
    tick_irq: Option<Ident>,
}

fn rtic_board_spec_for(deploy: &str) -> Option<RticBoardSpec> {
    let (device, dispatchers, consumer, tick_irq) = match deploy {
        "rtic-mps2-an385" | "qemu-rtic-mps2-an385" => (
            "mps2_an385_pac",
            &["UARTRX0", "UARTTX0"][..],
            // phase-337 W6.a — folded into `nros-board-mps2-an385`'s `rtic`
            // feature, which re-exports the queue accessors at the crate root.
            "::nros_board_mps2_an385::take_dispatch_consumer",
            Some("TIMER0"),
        ),
        _ => return None,
    };
    Some(RticBoardSpec {
        device_path: syn::parse_str::<SynPath>(device).ok()?,
        dispatchers: dispatchers
            .iter()
            .map(|name| Ident::new(name, Span::call_site()))
            .collect(),
        dispatch_consumer_path: syn::parse_str::<SynPath>(consumer).ok()?,
        tick_irq: tick_irq.map(|name| Ident::new(name, Span::call_site())),
    })
}

/// Phase 216.B.3 — boot-framework dispatch for `nros::main!()`.
///
/// Distinct from [`board_path_for`] (which only resolves the board
/// crate ZST). Frameworks are orthogonal to RMW + platform: each
/// board crate carries its own
/// `[package.metadata.nros.board] framework = "<f>"` knob (consumed
/// by 216.D.1 `nros check`). The macro keys off the Entry pkg's
/// `deploy = "..."` value directly to avoid a fs round-trip into the
/// board crate's manifest at proc-macro expansion time; the long-term
/// spec reads the board's manifest, but the skeleton hardcodes the
/// table to match the `board_path_for` row above.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Framework {
    /// Board owns the spin loop (`BoardEntry::run`). Default for
    /// every board key not explicitly routed below.
    OwnedSpin,
    /// RTIC framework owns the spin loop. The macro emits a
    /// `#[rtic::app(...)]` module + `#[init]` body that delegates to
    /// `RticBoardEntry::init_hardware`.
    Rtic,
    /// Embassy framework owns the spin loop. The macro emits a
    /// `#[embassy_executor::main] async fn main(spawner: Spawner)` body
    /// that delegates to `EmbassyBoardEntry::init_hardware`.
    ///
    /// phase-337 W7.a — UNREACHABLE from [`framework_for`] since
    /// `embassy-stm32f4`, the only in-tree deploy key that selected it, left
    /// with its board crate. The variant and its emit branch stay because they
    /// are the seam an out-of-tree Embassy board consumes — `EmbassyBoardEntry`
    /// is public in `nros-platform` and `nros ws check` already routes on the
    /// board crate's `framework = "embassy"` metadata.
    ///
    /// **Issue 0415 FIXED (phase-346 W1) — and this variant is how you can
    /// tell.** It carried `#[expect(dead_code)]` with the note "the expect
    /// fires the day it becomes constructible", because no in-tree deploy key
    /// selected it after phase-337 W7.a. It is constructible now:
    /// [`framework_from_name`] maps the `"embassy"` a board declares in its own
    /// `[package.metadata.nros.board]`, which reaches expansion through
    /// `NROS_BOARD_FRAMEWORK` (emitted by `nros_build::emit_board_framework`).
    /// So an out-of-tree Embassy board selects this branch without any in-tree
    /// table entry — which was the whole point.
    Embassy,
    /// Phase 225.P — Zephyr RTOS owns boot + `main`. The macro emits a
    /// `#[unsafe(no_mangle)] pub extern "C" fn rust_main()` staticlib
    /// export (consumed by `zephyr-lang-rust`'s `rust_cargo_application`)
    /// that gates on `ZephyrBoard::wait_link_up`, opens an `Executor`,
    /// wraps it in `ExecutorNodeRuntime`, registers each launch-named
    /// Node pkg, then spins — bounded on hosted `native_sim`, forever
    /// otherwise. There is NO `BoardEntry::run` (Zephyr forbids a Rust
    /// `fn main`).
    Zephyr,
    /// Phase 225.O — ESP32-C3 (esp-hal). esp-riscv-rt's `_start` calls
    /// the esp-hal entry registration, so a bare `extern "C" fn main`
    /// (the `OwnedSpin` `target_os = "none"` shape) does not boot. The
    /// macro emits `#[::esp_hal::main] fn main() -> !` that delegates to
    /// the real-runtime `BoardEntry::run` (which never returns), then
    /// spins defensively. The Entry crate provides the panic handler
    /// (`esp-backtrace`) + app descriptor (`esp_app_desc!`).
    Esp32,
}

// Phase 225.O follow-up (known-issue #18) — NOTE on NuttX. NuttX does
// NOT get its own `Framework` variant: it rides `Framework::OwnedSpin`.
// The NuttX flat-build init task calls `CONFIG_INIT_ENTRYPOINT="nsh_main"`,
// but the board crate (`nros-board-nuttx-qemu`'s `entry.rs`) already
// exports a `#[no_mangle] nsh_main` that runs `nsh_initialize()` (virtio
// FDT discovery + network bringup) and then calls the Rust `main`
// lang-start symbol. OwnedSpin emits exactly that `fn main()` (NuttX is
// `target_os = "nuttx"`, the `not(target_os = "none")` hosted arm), which
// delegates to `<QemuArmVirt as BoardEntry>::run`. So no NuttX-specific
// emit is needed; emitting our own `nsh_main` would both collide with
// the board's and skip the critical `nsh_initialize()` network bringup.

/// Map a canonical framework NAME (the `nros-orchestration-ir` spelling) to the
/// emit shape. `None` for a name this macro has no branch for — the caller
/// turns that into a compile error rather than a silent `OwnedSpin`.
fn framework_from_name(name: &str) -> Option<Framework> {
    Some(match name {
        "owned-spin" => Framework::OwnedSpin,
        "rtic" => Framework::Rtic,
        "embassy" => Framework::Embassy,
        "zephyr" => Framework::Zephyr,
        "esp32" => Framework::Esp32,
        _ => return None,
    })
}

/// Issue 0415 — resolve the entry shape from the BOARD, not from a table of
/// in-tree deploy strings.
///
/// Order, and the reason for it:
///
/// 1. `NROS_BOARD_FRAMEWORK`, set by the Entry package's build script (see
///    `nros_build::emit_board_framework`, which reads the board crate's
///    `[package.metadata.nros.board] framework`). This is the only route that
///    reaches an OUT-OF-TREE board, and it wins when present.
/// 2. `nros_orchestration_ir::framework_for_board_key` — the in-tree fast path,
///    the same SSoT `nros ws check` and the CLI's Rust emitter consult.
/// 3. `OwnedSpin`, which is what a board with no framework opinion means.
///
/// An UNKNOWN framework name is an error, never a fall-through. That
/// fall-through was the whole of issue 0415: an out-of-tree board declaring
/// `framework = "embassy"` got a plain `fn main()`, which links and then does
/// nothing the framework was for.
fn framework_for(deploy: &str) -> Framework {
    try_framework_for(deploy).unwrap_or_else(|e| {
        // A proc-macro cannot return a Result here without reshaping every
        // caller; the emitted `compile_error!` is produced by the caller that
        // owns the token stream. Panicking with this message surfaces at the
        // same place, with the same text.
        panic!("{e}")
    })
}

fn try_framework_for(deploy: &str) -> Result<Framework, String> {
    if let Ok(name) = std::env::var("NROS_BOARD_FRAMEWORK") {
        let name = name.trim();
        if !name.is_empty() {
            return framework_from_name(name).ok_or_else(|| {
                format!(
                    "nros::main!: the board declares `framework = \"{name}\"`, which this \
                     version of nano-ros has no entry shape for. Known frameworks: {}. \
                     (The value came from NROS_BOARD_FRAMEWORK, emitted by the Entry \
                     package's build script from the board crate's \
                     `[package.metadata.nros.board]`.)",
                    nros_orchestration_ir::FRAMEWORKS.join(", ")
                )
            });
        }
    }
    match nros_orchestration_ir::framework_for_board_key(deploy) {
        Some(name) => framework_from_name(name).ok_or_else(|| {
            format!(
                "nros::main!: internal — the in-tree board table maps deploy `{deploy}` to \
                 framework `{name}`, which this macro has no emit branch for"
            )
        }),
        None => Ok(Framework::OwnedSpin),
    }
}

/// Sanitise a pkg name into a valid Rust crate ident. Cargo allows
/// `-` in pkg names; Rust idents don't. Matches the
/// `sanitize_pkg_name_for_symbol` rule the existing `nros::node!()`
/// macro uses, so the codegen + Entry-pkg sides round-trip.
fn pkg_to_crate_ident(pkg: &str) -> String {
    // phase-432 W2.4 — DELEGATES. This body and
    // `nros_cli_core::codegen::entry::sanitize_pkg` were character-for-
    // character identical, on the two producers of the same generated text,
    // with nothing asserting they agreed.
    nros_entry_lower::sanitize_pkg(pkg)
}

/// Render the per-node runtime bake — the block both Rust producers emit.
///
/// phase-432 W2.4 / RFC-0091 §7. `nros codegen entry`'s Rust renderer emits
/// this same block from the same [`LoweredNode`]s through a `.jinja` template;
/// the `entry_parity` tests below render the shared corpus with THIS function
/// and compare against that renderer's committed goldens, token for token.
/// That is the byte-diff `emit_rust.rs` has claimed since it was written.
///
/// **Every field is written unconditionally, empty included.** `runtime` is
/// reused across the nodes of one entry, so a node with no params must CLEAR
/// the previous node's rather than inherit them. Emitting an assignment only
/// when it has a value leaks state between nodes and still passes any "does
/// the output contain this value" test — which is why the parity corpus
/// carries a bare node.
///
/// Order is load-bearing too: the state is written BEFORE the `register` call
/// it configures. After it would configure the NEXT node, or nothing.
pub(crate) fn render_register_calls(nodes: &[LoweredNode]) -> Vec<proc_macro2::TokenStream> {
    nodes
        .iter()
        .map(|node| {
            let ident = Ident::new(&node.ident(), Span::call_site());
            // Phase 264 W4a — the node's baked launch `<param>` initials (a
            // promoted `&'static` slice), so its `register`/`init` observes
            // launch values via `ctx.param(name)` (RFC-0004 §10).
            let param_lits = node.params.iter().map(|(name, value)| {
                let n = LitStr::new(name, Span::call_site());
                let v = LitStr::new(value, Span::call_site());
                quote! { (#n, #v) }
            });
            // Phase 305 W3 (issue 0255) — `<remap from= to=/>` rules;
            // `ExecutorSink::create_entity` matches entity source names
            // against them (exact-FQN, first rule wins).
            let remap_lits = node.remaps.iter().map(|(from, to)| {
                let f = LitStr::new(from, Span::call_site());
                let t = LitStr::new(to, Span::call_site());
                quote! { (#f, #t) }
            });
            // Issue #52 — the node's QoS-override table, in the same wire form
            // the C/C++ ABIs use.
            let qos_lits = node.qos_overrides.iter().map(|o| {
                let t = LitStr::new(&o.topic, Span::call_site());
                let (role, policy, value) = (o.role, o.policy, o.value);
                quote! { (#t, #role, #policy, #value) }
            });
            // Phase 268 W1 — `<node name= namespace=>` identity, which
            // `ExecutorSink::create_node` uses instead of the `NodeOptions`
            // default (RFC-0046). `None` in the self-bringup arm, so no
            // identity leaks between components.
            let identity_emit = match node.identity_pair() {
                Some((name, ns)) => {
                    let n = LitStr::new(name, Span::call_site());
                    let s = LitStr::new(ns, Span::call_site());
                    quote! {
                        runtime.node_identity = ::core::option::Option::Some((#n, #s));
                    }
                }
                None => quote! {
                    runtime.node_identity = ::core::option::Option::None;
                },
            };
            quote! {
                runtime.params = &[ #( #param_lits ),* ];
                runtime.remaps = &[ #( #remap_lits ),* ];
                runtime.qos_overrides = &[ #( #qos_lits ),* ];
                #identity_emit
                ::#ident::register(runtime)?;
            }
        })
        .collect()
}

/// Silence the unused-import warning in proc_macro2 — Expr / ExprLit /
/// Lit are imported so future extensions (e.g. parsing
/// `args = vec![("k","v")]`) can reach them without re-importing.
#[allow(dead_code)]
fn _unused() {
    let _ = std::marker::PhantomData::<(Expr, ExprLit, Lit)>;
}

/// Phase 216.B.4 — parser-only unit tests for `custom_tasks = [...]`.
///
/// The full `cargo check`-driven round-trip (showing the RTIC splice
/// actually compiles against a board crate) rides the `qemu` lane's
/// mps2-an385 RTIC fixtures.
/// These tests pin the host-side syntax acceptance: the parser takes
/// `custom_tasks = [ident, ident, ...]` (and the empty `[]` form),
/// rejects malformed shapes, and round-trips ident order.
#[cfg(test)]
mod custom_tasks_parser_tests {
    use super::MainArgs;

    fn parse(src: &str) -> syn::Result<MainArgs> {
        syn::parse_str::<MainArgs>(src)
    }

    #[test]
    fn empty_list_is_accepted() {
        // `Some(vec![])` distinguishes "key supplied, list empty" from
        // "key not supplied" so the OwnedSpin / Embassy misuse error
        // can still fire on `[]`.
        let parsed = parse("custom_tasks = []").expect("parse empty list");
        let tasks = parsed.custom_tasks.expect("custom_tasks set");
        assert!(tasks.is_empty(), "expected empty Vec, got {tasks:?}");
        assert!(parsed.custom_tasks_span.is_some(), "span captured");
    }

    #[test]
    fn single_ident_is_accepted() {
        let parsed = parse("custom_tasks = [adc_sample]").expect("parse single");
        let tasks = parsed.custom_tasks.expect("custom_tasks set");
        assert_eq!(tasks.len(), 1);
        assert_eq!(tasks[0].to_string(), "adc_sample");
    }

    #[test]
    fn multi_ident_round_trips_in_order() {
        let parsed =
            parse("custom_tasks = [adc_sample, ui_redraw, watchdog]").expect("parse multi");
        let names: Vec<String> = parsed
            .custom_tasks
            .expect("custom_tasks set")
            .into_iter()
            .map(|i| i.to_string())
            .collect();
        assert_eq!(names, vec!["adc_sample", "ui_redraw", "watchdog"]);
    }

    #[test]
    fn trailing_comma_is_accepted() {
        let parsed =
            parse("custom_tasks = [adc_sample, ui_redraw,]").expect("parse trailing comma");
        assert_eq!(parsed.custom_tasks.expect("set").len(), 2);
    }

    #[test]
    fn combines_with_other_args() {
        let parsed = parse("board = ::nros_board_linux::LinuxBoard, custom_tasks = [foo, bar]")
            .expect("parse combined");
        assert!(parsed.board.is_some(), "board parsed");
        assert_eq!(parsed.custom_tasks.expect("custom_tasks set").len(), 2);
    }

    #[test]
    fn string_literal_in_list_is_rejected() {
        let err = match parse("custom_tasks = [\"adc_sample\"]") {
            Ok(_) => panic!("string literals must not parse as idents"),
            Err(e) => e,
        };
        let msg = err.to_string();
        // syn's default ident-parse error contains "expected identifier"
        // — pin on that rather than the syn version-specific wording so
        // a syn bump doesn't tip the test.
        assert!(
            msg.contains("expected") && msg.contains("identifier"),
            "diagnostic should mention identifier, got: {msg}"
        );
    }

    #[test]
    fn missing_brackets_falls_back_to_path_branch() {
        // Without brackets the parser drops into the Path branch and
        // stores a `KvValue::Path`, which the `custom_tasks` arm
        // rejects with its diagnostic. Pin on the message contents.
        let err = match parse("custom_tasks = adc_sample") {
            Ok(_) => panic!("bare ident must not parse as a list"),
            Err(e) => e,
        };
        let msg = err.to_string();
        assert!(
            msg.contains("custom_tasks") && msg.contains("list of fn idents"),
            "diagnostic should mention custom_tasks list, got: {msg}"
        );
    }
}

/// Phase 305 W3 (issue 0255) — lower one model node's `<remap>` rules into the
/// `(from, to)` slice `nros::main!` bakes into `runtime.remaps` before that
/// node's `register` call. Order preserved (first match wins at runtime).
fn remap_bakes_for(inst: &ros_launch_manifest_model::NodeInstance) -> Vec<(String, String)> {
    inst.remaps
        .iter()
        .map(|r| (r.from.clone(), r.to.clone()))
        .collect()
}

#[cfg(test)]
mod remap_bake_tests {
    //! Phase 305 W3 (issue 0255) — a model node carrying remaps produces the
    //! `(from, to)` bake slice in declaration order; a remap-free node bakes
    //! an empty slice (the `runtime.remaps = &[]` reset shape).
    use super::remap_bakes_for;

    fn node_from_model_yaml(yaml: &str) -> ros_launch_manifest_model::NodeInstance {
        let model = ros_launch_manifest_model::SystemModel::from_yaml_str(yaml)
            .expect("parse SystemModel yaml");
        model
            .structure
            .nodes
            .values()
            .next()
            .expect("one node in model")
            .clone()
    }

    #[test]
    fn model_remaps_produce_ordered_bakes() {
        let inst = node_from_model_yaml(
            "meta:\n  version: 1\nstructure:\n  nodes:\n    /filter:\n      scope: root\n      pkg: filter_pkg\n      exec: filter\n      remaps:\n        - from: \"~/input/points\"\n          to: /points_raw\n        - from: scan\n          to: scan_filtered\n",
        );
        assert_eq!(
            remap_bakes_for(&inst),
            vec![
                ("~/input/points".to_string(), "/points_raw".to_string()),
                ("scan".to_string(), "scan_filtered".to_string()),
            ]
        );
    }

    #[test]
    fn remap_free_node_bakes_empty() {
        let inst = node_from_model_yaml(
            "meta:\n  version: 1\nstructure:\n  nodes:\n    /filter:\n      scope: root\n      pkg: filter_pkg\n      exec: filter\n",
        );
        assert!(remap_bakes_for(&inst).is_empty());
    }
}

#[cfg(test)]
mod bridge_rmw_tests {
    use super::rmw_crate_ident;

    #[test]
    fn rmw_crate_ident_maps_known_backends() {
        assert_eq!(rmw_crate_ident("zenoh"), Some("nros_rmw_zenoh"));
        assert_eq!(
            rmw_crate_ident("cyclonedds"),
            Some("nros_rmw_cyclonedds_sys")
        );
        assert_eq!(rmw_crate_ident("xrce"), Some("nros_rmw_xrce_cffi"));
        assert_eq!(rmw_crate_ident("unknown"), None);
    }
}

#[cfg(test)]
mod entry_sizing_tests {
    //! phase-271 (issue #110) — `[package.metadata.nros.entry] max_callbacks`
    //! parsing that drives the macro's `run_with_deploy_sized` emit.
    use super::read_entry_executor_sizing;
    use std::io::Write;

    fn write_tmp(name: &str, body: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("nros_macros_entry_sizing_{name}"));
        std::fs::create_dir_all(&dir).unwrap();
        let p = dir.join("Cargo.toml");
        std::fs::File::create(&p)
            .unwrap()
            .write_all(body.as_bytes())
            .unwrap();
        p
    }

    #[test]
    fn absent_max_callbacks_is_none() {
        // No knob → macro emits the default `run_with_deploy` (byte-identical).
        let p = write_tmp(
            "absent",
            "[package.metadata.nros.entry]\ndeploy = \"native\"\n",
        );
        assert_eq!(read_entry_executor_sizing(&p), None);
    }

    #[test]
    fn max_callbacks_only_defaults_sc_to_zero() {
        // `0` sched-contexts means "board uses the build default".
        let p = write_tmp(
            "cbs_only",
            "[package.metadata.nros.entry]\nmax_callbacks = 12\n",
        );
        assert_eq!(read_entry_executor_sizing(&p), Some((12, 0)));
    }

    #[test]
    fn max_callbacks_and_sched_contexts() {
        let p = write_tmp(
            "both",
            "[package.metadata.nros.entry]\nmax_callbacks = 8\nmax_sched_contexts = 5\n",
        );
        assert_eq!(read_entry_executor_sizing(&p), Some((8, 5)));
    }

    #[test]
    fn nonpositive_max_callbacks_is_none() {
        let p = write_tmp("zero", "[package.metadata.nros.entry]\nmax_callbacks = 0\n");
        assert_eq!(read_entry_executor_sizing(&p), None);
    }
}

#[cfg(test)]
mod derived_sizing_tests {
    //! Issue 0257 — the sizing the entry emits is DERIVED from the model's
    //! entity count when the entry declares none, and an over-capacity model
    //! fails the bake loudly instead of dying at boot on `code=-6 Full`.
    use super::{EntrySizing, executor_capacity_assert, executor_sizing_for};

    #[test]
    fn no_model_wiring_keeps_the_build_default() {
        // Any in-tree example model with no contract beside its launch file
        // (109 of 114 on 2026-09-06, issue 0973): no wiring ⇒ nothing to
        // derive ⇒ the pre-0257 unsized `run_with_deploy` emit.
        assert_eq!(
            executor_sizing_for(None, 0).unwrap(),
            EntrySizing::BuildDefault
        );
    }

    #[test]
    fn small_model_stays_on_the_build_default() {
        // 1 entity derives 3 — below the build default 4, so still no sized emit.
        assert_eq!(
            executor_sizing_for(None, 1).unwrap(),
            EntrySizing::BuildDefault
        );
    }

    #[test]
    fn fat_model_derives_a_sizing() {
        // The issue's 3-node workspace: 9 entities ⇒ 9 + 25 % = 12 slots.
        assert_eq!(
            executor_sizing_for(None, 9).unwrap(),
            EntrySizing::Derived(12)
        );
    }

    #[test]
    fn declared_sizing_wins_over_the_derivation() {
        // The author may know about timers the model cannot see.
        assert_eq!(
            executor_sizing_for(Some((32, 4)), 9).unwrap(),
            EntrySizing::Declared(32, 4)
        );
    }

    #[test]
    fn declared_sizing_below_the_model_count_fails_the_bake() {
        let err = executor_sizing_for(Some((4, 0)), 9)
            .expect_err("9 entities do not fit a declared 4")
            .to_string();
        assert!(err.contains("registers 9 callback entities"), "got: {err}");
        assert!(err.contains("max_callbacks = 4"), "got: {err}");
        assert!(err.contains("at least 12"), "got: {err}");
        assert!(err.contains("0257"), "got: {err}");
    }

    #[test]
    fn firmware_board_gets_a_const_capacity_assert() {
        // Zephyr ignores the emitted sizing → the only warning available is a
        // const check against the compiled `MAX_CBS`.
        let tokens = executor_capacity_assert(9, Some("zephyr"), EntrySizing::Derived(12));
        // It is spliced at item position in the expansion — it must parse there.
        syn::parse2::<syn::File>(tokens.clone()).expect("assert parses as an item");
        let ts = tokens.to_string();
        assert!(ts.contains("EXECUTOR_MAX_CBS"), "got: {ts}");
        assert!(ts.contains("NROS_EXECUTOR_MAX_CBS"), "got: {ts}");
        assert!(ts.contains("issue 0257"), "got: {ts}");
    }

    #[test]
    fn hosted_board_and_countless_models_emit_no_assert() {
        // posix honors the emitted sizing (which covers the count by
        // construction); a wiring-free model has nothing to assert; an
        // explicit `board = <Zst>` has no key to classify.
        assert!(executor_capacity_assert(9, Some("posix"), EntrySizing::Derived(12)).is_empty());
        assert!(executor_capacity_assert(0, Some("zephyr"), EntrySizing::BuildDefault).is_empty());
        assert!(executor_capacity_assert(9, None, EntrySizing::BuildDefault).is_empty());
    }

    /// Issue #52 — `qos_overrides.<topic>.<role>.<policy>` params decompose into
    /// the primitive codes `RuntimeCtx::qos_overrides` carries, using the same
    /// numbering as `nros_cpp_qos_override_t` and the CLI's C/C++ emitters.
    #[test]
    fn qos_override_params_decompose_into_codes() {
        let params = [
            (
                "qos_overrides./chatter.publisher.reliability".to_string(),
                "best_effort".to_string(),
            ),
            (
                "qos_overrides./chatter.subscription.depth".to_string(),
                "7".to_string(),
            ),
            (
                "qos_overrides./img.publisher.durability".to_string(),
                "transient_local".to_string(),
            ),
            ("use_sim_time".to_string(), "true".to_string()),
        ];
        let got: Vec<(String, u8, u8, u32)> = nros_orchestration_ir::qos_override::lower_all(
            params
                .iter()
                .map(|(k, v): &(String, String)| (k.as_str(), v.as_str())),
        )
        .expect("lower")
        .into_iter()
        .map(|o| (o.topic, o.role, o.policy, o.value))
        .collect();
        assert_eq!(
            got,
            vec![
                ("/chatter".to_string(), 0u8, 0u8, 0u32),
                ("/chatter".to_string(), 1, 3, 7),
                ("/img".to_string(), 0, 1, 1),
            ]
        );
    }

    /// Issue 0303 — an unusable override is REJECTED, not skipped. Each of
    /// these used to vanish from the bake with no diagnostic, leaving the image
    /// with different delivery semantics than the model declared.
    #[test]
    fn unusable_qos_override_params_are_rejected() {
        for (name, value) in [
            // Unknown role (`listener` is not a ROS role).
            ("qos_overrides./t.listener.reliability", "reliable"),
            // Unknown policy.
            ("qos_overrides./t.publisher.bandwidth", "10"),
            // Right policy, unparseable value.
            ("qos_overrides./t.publisher.depth", "lots"),
            // Prefix present, shape wrong.
            ("qos_overrides.", "x"),
        ] {
            let e =
                nros_orchestration_ir::qos_override::lower(name, value).expect_err("must reject");
            assert!(e.to_string().contains(name), "{e}");
        }
    }

    /// The policies issue 0303 ADDED lower rather than being dropped —
    /// `lifespan` here was previously an "unrecognised policy" in this very
    /// test, which is what made the gap easy to miss.
    #[test]
    fn the_duration_policies_lower() {
        use nros_orchestration_ir::qos_override::{lower, policy};
        for (name, value, expect_policy, expect_value) in [
            (
                "qos_overrides./t.publisher.deadline",
                "100",
                policy::DEADLINE,
                100u32,
            ),
            (
                "qos_overrides./t.publisher.lifespan",
                "10",
                policy::LIFESPAN,
                10,
            ),
            (
                "qos_overrides./t.publisher.liveliness_lease_duration",
                "500",
                policy::LIVELINESS_LEASE,
                500,
            ),
        ] {
            let got = lower(name, value).expect("lowers").expect("is an override");
            assert_eq!(
                (got.policy, got.value),
                (expect_policy, expect_value),
                "{name}"
            );
        }
    }
}

#[cfg(test)]
mod framework_ssot_tests {
    use super::*;

    /// Issue 0415 — the shared framework vocabulary and this macro's emit
    /// shapes must stay in step. If `nros-orchestration-ir` gains a framework
    /// and the macro grows no branch for it, a board declaring it would hit the
    /// "no entry shape" error at expansion — better caught here.
    #[test]
    fn every_declared_framework_has_an_emit_shape() {
        for name in nros_orchestration_ir::FRAMEWORKS {
            assert!(
                framework_from_name(name).is_some(),
                "`{name}` is in nros_orchestration_ir::FRAMEWORKS but this macro \
                 has no emit branch for it"
            );
        }
    }

    /// …and the reverse: a name the macro accepts but the vocabulary does not
    /// declare could never be produced by a board, so it would be dead emit.
    #[test]
    fn the_macro_accepts_no_framework_outside_the_vocabulary() {
        for name in ["owned-spin", "rtic", "embassy", "zephyr", "esp32"] {
            assert!(
                nros_orchestration_ir::is_known_framework(name),
                "the macro maps `{name}` but the shared vocabulary does not declare it"
            );
        }
        assert!(framework_from_name("not-a-framework").is_none());
    }

    /// Every in-tree board key must resolve to a framework this macro emits —
    /// the fast path has to stay usable without the build-script route.
    #[test]
    fn in_tree_board_keys_resolve_to_an_emit_shape() {
        for key in [
            "native",
            "freertos",
            "threadx-linux",
            "threadx-qemu-riscv64",
            "nuttx",
            "nuttx-riscv",
            "esp32-qemu",
            "zephyr",
            "rtic-mps2-an385",
            "qemu-mps2-an385",
        ] {
            let fw = nros_orchestration_ir::framework_for_board_key(key).unwrap_or("owned-spin");
            assert!(
                framework_from_name(fw).is_some(),
                "board key `{key}` maps to framework `{fw}`, which has no emit branch"
            );
        }
    }

    /// The regression 0415 names: an unknown framework must be an ERROR, not a
    /// silent fall-through to `OwnedSpin` (an image that links and does nothing
    /// the framework was for).
    #[test]
    fn an_unknown_framework_is_an_error_not_owned_spin() {
        // SAFETY: single-threaded test process; the var is removed below.
        unsafe { std::env::set_var("NROS_BOARD_FRAMEWORK", "embasy") };
        let err = try_framework_for("native").unwrap_err();
        assert!(
            err.contains("embasy"),
            "the error must name the bad value: {err}"
        );
        assert!(err.contains("embassy"), "and list the known ones: {err}");

        unsafe { std::env::set_var("NROS_BOARD_FRAMEWORK", "embassy") };
        assert_eq!(try_framework_for("native").unwrap(), Framework::Embassy);
        unsafe { std::env::remove_var("NROS_BOARD_FRAMEWORK") };
    }
}
