//! `nros new node <name>` — scaffold a node package.
//!
//! WHY THIS VERB EXISTS
//!
//! There was no way to make one. `nros new <name> --platform native` produces a
//! standalone RUNNABLE project, which pins a board crate and a platform port —
//! correct for what it is, and wrong for a package an entry links onto a
//! different platform. Building a workspace with one showed exactly that:
//!
//! ```text
//! error: no matching package named `nros-board-linux` found
//! required by package `talker_pkg`
//!   ... which satisfies path dependency `talker_pkg` of package `zephyr_entry`
//! ```
//!
//! A node package is **board- and RMW-agnostic**: one `nros` dependency with
//! `alloc` and `rmw-cffi`, and nothing that names a platform. `alloc` is the
//! universal baseline (it works on the `no_std` targets, and a `std` target
//! gets `std` from the entry through feature unification); `rmw-cffi` is the
//! vtable seam, never a concrete backend. The entry chooses the platform, the
//! board and the RMW — that is what makes the same node package deployable to
//! Linux and to a Cortex-M without editing it.
//!
//! WHY IT ALSO EDITS THE BRINGUP
//!
//! Same reason `nros new entry` writes the image: a package and its
//! `[[component]]` row are two halves of one declaration. A node nothing
//! declares is dead code, and the failure — the launch file naming a node the
//! system does not carry — appears nowhere near the package that was just
//! created.

use std::{
    fs,
    path::{Path, PathBuf},
};

use eyre::{Result, WrapErr, bail};
use toml_edit::{DocumentMut, Item, Table, value};

pub struct NodeScaffold {
    /// Package directory to create, e.g. `<ws>/src/talker_pkg`.
    pub node_dir: PathBuf,
    /// The bringup whose `[[component]]` list gains this node. `None` skips
    /// that half — for a package being written before any system exists.
    pub bringup_dir: Option<PathBuf>,
}

pub struct NodeScaffoldOut {
    pub node_dir: PathBuf,
    pub files: Vec<PathBuf>,
    pub declared_in: Option<PathBuf>,
}

pub fn scaffold_node(cfg: &NodeScaffold) -> Result<NodeScaffoldOut> {
    let name = cfg
        .node_dir
        .file_name()
        .and_then(|n| n.to_str())
        .ok_or_else(|| eyre::eyre!("invalid node package name"))?
        .to_string();
    if cfg.node_dir.exists() {
        bail!(
            "{} already exists — refusing to overwrite it",
            cfg.node_dir.display()
        );
    }
    // The Rust type name: `talker_pkg` -> `TalkerPkg`.
    let type_name = name
        .split(['_', '-'])
        .filter(|s| !s.is_empty())
        .map(|s| {
            let mut c = s.chars();
            match c.next() {
                Some(f) => f.to_uppercase().collect::<String>() + c.as_str(),
                None => String::new(),
            }
        })
        .collect::<String>();
    // A node NAME is what the launch file says; keep the `_pkg` suffix out of
    // it, since that names the package rather than the node.
    let node_name = name.trim_end_matches("_pkg").to_string();

    fs::create_dir_all(cfg.node_dir.join("src"))
        .wrap_err_with(|| format!("create {}", cfg.node_dir.display()))?;

    let mut files = Vec::new();
    let mut put = |rel: &str, body: String| -> Result<()> {
        let p = cfg.node_dir.join(rel);
        if let Some(parent) = p.parent() {
            fs::create_dir_all(parent)?;
        }
        fs::write(&p, body).wrap_err_with(|| format!("write {}", p.display()))?;
        files.push(p);
        Ok(())
    };

    put("package.xml", render_package_xml(&name))?;
    put("Cargo.toml", render_cargo_toml(&name))?;
    put("src/lib.rs", render_lib_rs(&name, &type_name, &node_name))?;

    let declared_in = match &cfg.bringup_dir {
        Some(b) => {
            let sys = b.join("system.toml");
            let added = add_component(&sys, &name, &type_name, &node_name)?;
            added.then_some(sys)
        }
        None => None,
    };

    Ok(NodeScaffoldOut {
        node_dir: cfg.node_dir.clone(),
        files,
        declared_in,
    })
}

/// Append a `[[component]]` row to the bringup.
///
/// `toml_edit` rather than a serialize round-trip, for the same reason
/// `nros new entry` uses it: a `system.toml` carries comments and an order the
/// user authored, and rewriting the document to add three lines discards them.
/// Returns whether a row was ADDED (`false` = it was already declared).
fn add_component(system_toml: &Path, pkg: &str, type_name: &str, node_name: &str) -> Result<bool> {
    if !system_toml.is_file() {
        bail!(
            "{} carries no `system.toml`, so it is not a bringup package",
            system_toml.display()
        );
    }
    let text = fs::read_to_string(system_toml)?;
    let mut doc: DocumentMut = text
        .parse()
        .wrap_err_with(|| format!("parse {}", system_toml.display()))?;

    let arr = doc
        .entry("component")
        .or_insert(Item::ArrayOfTables(Default::default()));
    let Some(tables) = arr.as_array_of_tables_mut() else {
        bail!(
            "`component` in {} is not an array of tables",
            system_toml.display()
        );
    };
    if tables
        .iter()
        .any(|t| t.get("pkg").and_then(|v| v.as_str()) == Some(pkg))
    {
        // Already declared, so there is nothing to add — NOT an error.
        // `nros new system --components a,b,c` names the components BEFORE
        // their packages exist, which is a normal order to work in, and
        // refusing here would make the two verbs usable in only one sequence.
        return Ok(false);
    }

    let mut t = Table::new();
    t["pkg"] = value(pkg.to_string());
    t["class"] = value(format!("{pkg}::{type_name}"));
    t["name"] = value(node_name.to_string());
    tables.push(t);

    fs::write(system_toml, doc.to_string())?;
    Ok(true)
}

fn render_package_xml(name: &str) -> String {
    format!(
        r#"<?xml version="1.0"?>
<!-- generated by `nros new node {name}` -->
<package format="3">
  <name>{name}</name>
  <version>0.1.0</version>
  <description>A nano-ros node package.</description>
  <maintainer email="you@example.com">you</maintainer>
  <license>Apache-2.0</license>
</package>
"#
    )
}

fn render_cargo_toml(name: &str) -> String {
    format!(
        r#"# Node package — generated by `nros new node {name}`.
#
# Board- and RMW-agnostic ON PURPOSE. The single `nros` dependency below is the
# whole of it: `alloc` is the universal baseline (it works on the `no_std`
# targets, and a `std` target gets `std` from the entry through feature
# unification), and `rmw-cffi` is the vtable seam rather than a backend.
#
# Do NOT add a board crate, a platform port or a concrete `nros-rmw-*` here.
# The ENTRY package chooses all three, which is what lets this same package be
# deployed to Linux and to a Cortex-M without an edit.
#
# nano-ros crates are not published (RFC-0040); `version = "*"` is the
# left-hand side of the patch `nros sync` writes.

[package]
name = "{name}"
version = "0.1.0"
edition = "2024"
publish = false

[lib]
crate-type = ["rlib"]

[dependencies]
nros = {{ version = "*", default-features = false, features = ["alloc", "rmw-cffi"] }}
log = "0.4"
"#
    )
}

fn render_lib_rs(name: &str, type_name: &str, node_name: &str) -> String {
    format!(
        r#"//! `{name}` — a nano-ros node package.
//!
//! The entry package links this crate and its `nros::main!` emits one
//! `{name}::register(runtime)?` call per `<node>` in the launch file — so the
//! launch file is what decides whether this node runs, and how many of it.
//!
//! `#![no_std]` because a node package must build for the embedded targets
//! too. Everything here is available on all of them.

#![no_std]

use nros::{{
    Callback, CallbackCtx, ExecutableNode, Node, NodeContext, NodeOptions, NodeResult,
    TimerDuration,
}};

pub struct {type_name};

impl Node for {type_name} {{
    const NAME: &'static str = "{node_name}";

    /// Exact entity counts, so this node's static registries are sized to it
    /// rather than to a worst case (issue 0857 measured the difference at
    /// 50,824 bytes of `.bss` against 568). In order: publishers, service
    /// servers, service clients, action clients, action servers — raise the
    /// matching one when you add an entity, or the registry refuses it at run
    /// time. Subscriptions and timers need no slot here.
    const ENTITY_BOUNDS: nros::EntityBounds = nros::EntityBounds::exact(0, 0, 0, 0, 0);

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {{
        let mut node = ctx.create_node(NodeOptions::new("{node_name}"))?;
        // A 1 Hz timer, so a fresh package does something observable. A timer
        // needs no registry slot, which is why ENTITY_BOUNDS stays all-zero.
        let _timer =
            node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(1000))?;
        Ok(())
    }}
}}

impl ExecutableNode for {type_name} {{
    /// Whatever this node needs to remember between callbacks.
    type State = u32;

    fn init() -> Self::State {{
        0
    }}

    fn on_callback(state: &mut Self::State, callback: Callback<'_>, _ctx: &mut CallbackCtx<'_>) {{
        if callback.as_str() == "on_tick" {{
            log::info!("{node_name}: tick {{}}", *state);
            *state = state.wrapping_add(1);
        }}
    }}
}}

nros::node!({type_name});
"#
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ws(tag: &str) -> PathBuf {
        let d = std::env::temp_dir().join(format!("nros-new-node-{tag}"));
        std::fs::remove_dir_all(&d).ok();
        std::fs::create_dir_all(d.join("src/demo_bringup")).unwrap();
        std::fs::write(
            d.join("src/demo_bringup/system.toml"),
            "# a comment the user wrote\n[system]\nname = \"demo\"\n",
        )
        .unwrap();
        d
    }

    /// The whole point of the verb: a node package names NO platform.
    ///
    /// `nros new <name> --platform native` pins `nros-board-linux` and a
    /// platform port, so a Zephyr entry linking it fails with `no matching
    /// package named 'nros-board-linux'`. A node package must be deployable
    /// to any board its entry chooses.
    #[test]
    fn a_node_package_names_no_board_platform_or_backend() {
        let d = ws("agnostic");
        let out = scaffold_node(&NodeScaffold {
            node_dir: d.join("src/talker_pkg"),
            bringup_dir: Some(d.join("src/demo_bringup")),
        })
        .expect("scaffolds");

        let manifest = std::fs::read_to_string(out.node_dir.join("Cargo.toml")).unwrap();
        // The DEPENDENCIES, not the whole file: the comment above them names
        // the very crates it tells you not to add, and a file-wide scan reads
        // that prohibition as a violation of itself.
        let deps = manifest
            .split("[dependencies]")
            .nth(1)
            .expect("a [dependencies] section");
        for forbidden in ["nros-board", "nros-platform", "nros-rmw-", "platform-"] {
            assert!(
                !deps.contains(forbidden),
                "a node package must not depend on `{forbidden}`:\n{deps}"
            );
        }
        assert!(
            manifest.contains(r#"features = ["alloc", "rmw-cffi"]"#),
            "{manifest}"
        );
    }

    /// The package and its `[[component]]` row are two halves of one
    /// declaration; a node nothing declares is dead code.
    #[test]
    fn the_bringup_gains_a_component_row() {
        let d = ws("component");
        scaffold_node(&NodeScaffold {
            node_dir: d.join("src/talker_pkg"),
            bringup_dir: Some(d.join("src/demo_bringup")),
        })
        .expect("scaffolds");

        let sys = std::fs::read_to_string(d.join("src/demo_bringup/system.toml")).unwrap();
        assert!(sys.contains("[[component]]"), "{sys}");
        assert!(sys.contains(r#"pkg = "talker_pkg""#), "{sys}");
        assert!(sys.contains(r#"class = "talker_pkg::TalkerPkg""#), "{sys}");
        // `_pkg` names the PACKAGE; the node is what the launch file calls it.
        assert!(sys.contains(r#"name = "talker""#), "{sys}");
        // The user's own text survives — that is why this is a toml_edit
        // insertion and not a serialize round-trip.
        assert!(sys.contains("# a comment the user wrote"), "{sys}");
    }

    /// A component the bringup ALREADY declares is a no-op, not an error.
    ///
    /// `nros new system --components a,b,c` names components before their
    /// packages exist — a normal order to work in. Refusing here would make
    /// the two verbs usable in exactly one sequence, and the row is already
    /// what the caller wanted.
    #[test]
    fn an_already_declared_component_is_not_added_twice() {
        let d = ws("dup");
        let mk = || {
            scaffold_node(&NodeScaffold {
                node_dir: d.join("src/talker_pkg"),
                bringup_dir: Some(d.join("src/demo_bringup")),
            })
        };
        let first = mk().expect("first");
        assert!(first.declared_in.is_some(), "the first call declares it");

        std::fs::remove_dir_all(d.join("src/talker_pkg")).unwrap();
        let second = mk().expect("a declared component is not an error");
        assert!(
            second.declared_in.is_none(),
            "the row existed, so nothing was added"
        );

        let sys = std::fs::read_to_string(d.join("src/demo_bringup/system.toml")).unwrap();
        assert_eq!(sys.matches("[[component]]").count(), 1, "{sys}");
    }
}
