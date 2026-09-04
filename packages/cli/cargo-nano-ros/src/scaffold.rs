//! Project scaffolder — promoted out of `main.rs` so `nros-cli` and any
//! other front-end can share it.
//!
//! v1 emits a colcon-compatible hello-world per `<lang> × <platform>`.
//! Use-case (`talker` / `listener` / `service` / `action`) and RMW-choice
//! diversification arrives once the `templates/` tree lands; until then
//! both fields are accepted for forward-compat but only surfaced in the
//! "Next steps" output.

use crate::rmw_resolver;
use eyre::{Result, bail};
use std::{
    fs,
    path::{Path, PathBuf},
    process::Command,
};

/// Issue 0249 — maintainer line for generated `package.xml`. Sourced from
/// `git config user.name`/`user.email` so scaffolded packages carry the real
/// author; falls back to an instructional placeholder (never `TODO@todo.com`,
/// which shipped verbatim through colcon/bloom surfaces).
fn maintainer_xml() -> String {
    fn git_config(key: &str) -> Option<String> {
        let out = Command::new("git").args(["config", key]).output().ok()?;
        if !out.status.success() {
            return None;
        }
        let v = String::from_utf8(out.stdout).ok()?.trim().to_string();
        (!v.is_empty()).then_some(v)
    }
    let name = git_config("user.name");
    let email = git_config("user.email");
    match (name, email) {
        (Some(n), Some(e)) => format!(r#"<maintainer email="{e}">{n}</maintainer>"#),
        (Some(n), None) => format!(r#"<maintainer email="you@example.com">{n}</maintainer>"#),
        _ => r#"<maintainer email="you@example.com">Your Name</maintainer>"#.to_string(),
    }
}

/// Issue 0333 — how a `--platform` scaffolds a *runnable* Rust project. The
/// blessed shapes are genuinely different per platform (a hosted std binary vs
/// a `#![no_std]` `nros::main!()` self-bringup vs a west/Kconfig build), so this
/// is not one template. Each variant is modeled on a tracked, compiling example.
enum PlatformKind {
    /// Hosted std binary: explicit `register_linked_rmw()` +
    /// `nros::init_with_launch_auto()` + `Executor::open` + `spin_blocking`.
    /// Models `examples/native/rust/talker`. (native, posix)
    Hosted,
    /// `#![no_std]` `nros::main!()` Form-1 self-bringup — `main.rs` is the
    /// one-line entry, `lib.rs` is the node (`nros::node!`), and
    /// `[package.metadata.nros.entry] deploy` names the board. The runtime dep
    /// profile differs: `CortexM` carries the cortex-m trio + a direct
    /// `nros-rmw-zenoh`; `Esp32` carries `esp-hal`/`esp-backtrace` and lets the
    /// board crate own the RMW. Models the baremetal / esp32 single-package
    /// talkers.
    SelfBringup { runtime: SelfBringupRuntime },
    /// Not yet scaffoldable as a single package — `nros new` bails with a
    /// pointer rather than emit a project that cannot run. The #333 follow-up.
    Deferred {
        /// Why, and where the current runnable example lives.
        reason: &'static str,
    },
}

#[derive(PartialEq)]
enum SelfBringupRuntime {
    /// cortex-m target: `cortex-m` + `cortex-m-rt` + `panic-semihosting`, plus a
    /// direct `nros-rmw-zenoh = { features = ["platform-bare-metal", …] }`.
    CortexM,
    /// esp32c3: `esp-hal` + `esp-backtrace` (panic handler); the board crate's
    /// default features own the RMW.
    Esp32,
}

/// The platform → board-crate + entry-shape SSoT (issue 0333). Every
/// `--platform` value the CLI advertises (`nros-cli-core/src/cmd/new.rs`) MUST
/// resolve here, or scaffolding `bail!`s loudly instead of silently emitting a
/// board dep that is really a TOML comment. `board_crate` / `deploy_token` are
/// the values the tracked entry examples declare; `deploy_token` is what bare
/// `nros::main!()` reads (`nros-orchestration-ir::board_path_for`).
struct PlatformSpec {
    board_crate: &'static str,
    /// `[package.metadata.nros.*] deploy = "…"` token. For `Hosted` it is the
    /// `application` deploy; for `SelfBringup` it is the `entry` deploy the
    /// macro resolves to a board.
    deploy_token: &'static str,
    kind: PlatformKind,
}

/// Resolve a `--platform` token, or `bail!` with the full supported set. Called
/// once up front in [`scaffold_package`] so an unsupported platform fails before
/// any file is written, and reused by [`scaffold_rust`].
fn platform_spec(platform: &str) -> Result<PlatformSpec> {
    let spec = match platform {
        // native + posix both resolve to `nros_board_linux::LinuxBoard`
        // (`board_path_for`); `nros-board-linux` is not reachable via any deploy
        // token, so the hosted template uses `nros-board-linux` for both.
        "native" => PlatformSpec {
            board_crate: "nros-board-linux",
            deploy_token: "native",
            kind: PlatformKind::Hosted,
        },
        "posix" => PlatformSpec {
            board_crate: "nros-board-linux",
            deploy_token: "posix",
            kind: PlatformKind::Hosted,
        },
        "baremetal" => PlatformSpec {
            board_crate: "nros-board-mps2-an385",
            deploy_token: "mps2-an385",
            kind: PlatformKind::SelfBringup {
                runtime: SelfBringupRuntime::CortexM,
            },
        },
        "esp32" => PlatformSpec {
            board_crate: "nros-board-esp32-qemu",
            deploy_token: "esp32-qemu",
            kind: PlatformKind::SelfBringup {
                runtime: SelfBringupRuntime::Esp32,
            },
        },
        "freertos" => PlatformSpec {
            board_crate: "nros-board-mps2-an385-freertos",
            deploy_token: "freertos",
            kind: PlatformKind::Deferred {
                reason: "the tracked shape is a split node-lib + `*-entry` bin pair; \
                         see examples/qemu-arm-freertos/rust/ (issue 0333 follow-up)",
            },
        },
        "nuttx" => PlatformSpec {
            board_crate: "nros-board-nuttx-qemu",
            deploy_token: "nuttx",
            kind: PlatformKind::Deferred {
                reason: "the tracked shape is a split node-lib + `*-entry` bin pair; \
                         see examples/qemu-arm-nuttx/rust/ (issue 0333 follow-up)",
            },
        },
        "threadx" => PlatformSpec {
            board_crate: "nros-board-threadx-linux",
            deploy_token: "threadx-linux",
            kind: PlatformKind::Deferred {
                reason: "the tracked shape is a split node-lib + `*-entry` bin pair; \
                         see examples/threadx-linux/rust/ (issue 0333 follow-up)",
            },
        },
        "zephyr" => PlatformSpec {
            board_crate: "nros-board-zephyr",
            deploy_token: "zephyr",
            kind: PlatformKind::Deferred {
                reason: "Zephyr builds through west/Kconfig with \
                         `nros::zephyr_component_main!` in a lib-only crate, not a plain \
                         cargo binary; see examples/zephyr/rust/ (issue 0333 follow-up)",
            },
        },
        other => bail!(
            "nros new: unsupported --platform '{other}'. Supported: \
             native, posix, freertos, baremetal, nuttx, threadx, zephyr, esp32."
        ),
    };
    Ok(spec)
}

#[derive(Debug, Clone)]
pub struct ScaffoldConfig {
    pub name: String,
    pub lang: String,
    pub platform: String,
    pub rmw: String,
    /// ROS edition (`humble`|`iron`|`jazzy`|`rolling`) → the `ros-<edition>`
    /// cargo feature on the scaffolded package (phase-304 W2b, RFC-0056).
    pub ros_edition: String,
    pub use_case: String,
    pub force: bool,
}

pub fn scaffold_package(cfg: &ScaffoldConfig) -> Result<()> {
    let dir = PathBuf::from(&cfg.name);
    if dir.exists() {
        if !cfg.force {
            bail!(
                "Directory '{}' already exists (use --force to overwrite)",
                cfg.name
            );
        }
        fs::remove_dir_all(&dir)?;
    }

    // Phase 227.4 — validate + lower the declared RMW (RFC-0031). A bad
    // `--rmw` fails here with the known-list, not as a broken downstream build.
    let rmw = rmw_resolver::resolve_rmw(&cfg.rmw).map_err(|e| eyre::eyre!("nros new: {e}"))?;

    // phase-304 W2b (RFC-0056) — validate + lower the declared ROS edition to the
    // `ros-<edition>` cargo feature (a bad `--ros-edition` fails loud here).
    let edition_feature = rosidl_codegen::RosEdition::parse(&cfg.ros_edition)
        .ok_or_else(|| {
            eyre::eyre!(
                "nros new: unknown ROS edition '{}' (humble | iron | jazzy | rolling)",
                cfg.ros_edition
            )
        })?
        .cargo_feature();

    // Issue 0333 — validate the platform up front (like `--rmw`/`--ros-edition`
    // above) so an unsupported value fails loud before any file is written,
    // rather than silently degrading into a commented-out board dep or a
    // `deploy=` string nothing recognizes.
    let spec = platform_spec(&cfg.platform)?;

    // Issue 0333 defect 2 — the Rust runnable-template rewrite covers the
    // single-package platforms (native/posix hosted, baremetal/esp32
    // `nros::main!()` self-bringup). The split-package (freertos/nuttx/threadx)
    // and west/Kconfig (zephyr) shapes are a follow-up; bail before writing any
    // file rather than emit a project that cannot run. C/C++ are unaffected —
    // their platform delta rides `package.xml`, no per-platform cargo shape.
    if cfg.lang == "rust"
        && let PlatformKind::Deferred { reason } = spec.kind
    {
        bail!(
            "nros new: single-package Rust scaffolding for --platform {} is not available \
             yet — {reason}. Use native, posix, baremetal, or esp32 for a runnable Rust \
             starter today.",
            cfg.platform
        );
    }

    // RFC-0048 (phase-287) + RFC-0087 D2/D3 (phase-420 W4) — a scaffolded
    // package is nano-ros-owned, so `<build_type>` says so: `nros_cmake` for
    // the C/C++ shape, `nros_cargo` for Rust. Both replace claims that were
    // false in opposite directions — `ament_cmake` said a stock `colcon build`
    // could handle the package, and `nros.<lang>.<platform>` was an improvised
    // spelling whose only registered readers were 30 colcon entry points that
    // W4 deleted.
    //
    // The `<nano_ros deploy= rmw=/>` tuple is now emitted for EVERY language,
    // not just C/C++. It was the CMake reader's input before; since W4 it is
    // also where the colcon task reads the platform from, because the platform
    // no longer rides in the build type. Without it a `--platform freertos`
    // Rust package would declare no deploy, and a colcon build of it would
    // silently produce a host binary.
    let deploy = if cfg.platform == "native" {
        "native"
    } else {
        cfg.platform.as_str()
    };
    let is_cxx = matches!(cfg.lang.as_str(), "c" | "cpp");
    let build_type = if is_cxx { "nros_cmake" } else { "nros_cargo" };
    let nano_ros_export = format!(
        "\n    <nano_ros deploy=\"{deploy}\" rmw=\"{}\"/>",
        rmw.cmake_value
    );

    fs::create_dir_all(dir.join("src"))?;

    let package_xml = format!(
        r#"<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>0.1.0</version>
  <description>{name} — nano-ros {platform} package</description>
  {maintainer}
  <license>Apache-2.0</license>
  <depend>std_msgs</depend>
  <export>
    <build_type>{build_type}</build_type>{nano_ros_export}
  </export>
</package>
"#,
        name = cfg.name,
        platform = cfg.platform,
        maintainer = maintainer_xml(),
    );
    fs::write(dir.join("package.xml"), package_xml)?;

    match cfg.lang.as_str() {
        "rust" => scaffold_rust(
            &cfg.name,
            &cfg.platform,
            rmw.cargo_feature,
            edition_feature,
            &dir,
        )?,
        "c" => scaffold_c(&cfg.name, &cfg.platform, rmw.cmake_value, &dir)?,
        "cpp" => scaffold_cpp(&cfg.name, &cfg.platform, rmw.cmake_value, &dir)?,
        other => bail!("Unknown language: {other}. Use rust, c, or cpp."),
    }

    println!("✓ Created nano-ros package '{}'", cfg.name);
    println!("  Language : {}", cfg.lang);
    println!("  Platform : {}", cfg.platform);
    println!("  RMW      : {}", cfg.rmw);
    println!(
        "  Use case : {} (template diversification: TODO)",
        cfg.use_case
    );
    println!("  Build    : {build_type}");
    println!();
    println!("Next steps:");
    println!("  cd {}", cfg.name);
    println!("  export NROS_REPO_DIR=/path/to/nano-ros   # your nano-ros source checkout");
    println!("  eval \"$(nros ws env)\"   # ROS + interface search path");
    println!("  nros sync          # codegen + write the [patch.crates-io] block (RFC-0040)");
    println!("  cargo build           # or: cmake --build build / west build / idf.py build");

    Ok(())
}

#[derive(Debug, Clone)]
pub struct ComponentScaffoldConfig {
    pub name: String,
    /// Node flavor: `talker` / `listener` / `service` / `action`. Template
    /// diversification is TODO — today every flavor emits the publisher+timer
    /// shape, named after the flavor.
    pub use_case: String,
    /// Source language. `rust` lands the planned-mode `nros::Component` shape;
    /// `c` / `cpp` land the typed component shape (RFC-0043) in the RFC-0048
    /// ament spelling: `find_package(nano_ros REQUIRED)` +
    /// `nano_ros_add_node(...)` + a `configure(::nros::Node&)` (C++) /
    /// `NROS_C_COMPONENT` (C) seam.
    pub lang: String,
    pub force: bool,
}

/// Scaffold a **planned-mode component** — a reusable nano-ros node compiled as
/// a *library* and linked into a system plan. Unlike
/// the direct-mode hello-world binary `scaffold_package` emits (a `[node]`
/// manifest), this produces an `nros::Component` impl plus a *folded*
/// `[component]` table in `nros.toml`. The platform + RMW are chosen later, at
/// Entry-package build time — not baked here.
///
/// The manifest is intentionally minimal: `[linkage]` is omitted (derived —
/// executable ← component short name, `exported_symbol` ← `nros_component_<n>`,
/// `crate_name` ← package) and `[overrides]` defaults to empty (Phase 172 W.3).
pub fn scaffold_component(cfg: &ComponentScaffoldConfig) -> Result<()> {
    match cfg.lang.as_str() {
        "rust" => scaffold_component_rust(cfg),
        "cpp" => scaffold_component_cpp(cfg),
        "c" => scaffold_component_c(cfg),
        other => bail!(
            "`nros new --component --lang {other}` is not supported. Use \
             `rust`, `c`, or `cpp`."
        ),
    }
}

fn scaffold_component_rust(cfg: &ComponentScaffoldConfig) -> Result<()> {
    let dir = PathBuf::from(&cfg.name);
    if dir.exists() {
        if !cfg.force {
            bail!(
                "Directory '{}' already exists (use --force to overwrite)",
                cfg.name
            );
        }
        fs::remove_dir_all(&dir)?;
    }
    fs::create_dir_all(dir.join("src"))?;

    let crate_name = cfg.name.replace('-', "_");
    let module = &cfg.use_case; // constrained by the CLI to a valid Rust ident

    let package_xml = format!(
        r#"<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>0.1.0</version>
  <description>{name} — nano-ros reusable component.</description>
  {maintainer}
  <license>Apache-2.0</license>
  <export>
    <build_type>nros_cargo</build_type>
  </export>
</package>
"#,
        name = cfg.name,
        maintainer = maintainer_xml(),
    );
    fs::write(dir.join("package.xml"), package_xml)?;

    let cargo_toml = format!(
        r#"[package]
name = "{name}"
version = "0.1.0"
edition = "2024"

# Standalone-buildable: an empty [workspace] makes this its own Cargo root even
# when dropped under a colcon workspace's src/.
[workspace]

# A reusable component is a library (rlib); the deployed system links it.
# nano-ros crates are not published to crates.io (RFC-0040) — `version = "*"` is
# only the patched left-hand side. Run `nros sync` (with NROS_REPO_DIR set) to
# write the nros-managed [patch.crates-io] block redirecting `nros` to your
# nano-ros checkout, then `cargo build`.
[dependencies]
nros = {{ version = "*", default-features = false }}
"#,
        name = cfg.name,
    );
    fs::write(dir.join("Cargo.toml"), cargo_toml)?;

    let lib_rs = format!(
        r#"#![no_std]

//! `{name}` — a reusable nano-ros component (planned mode).
//!
//! `nros plan` and Entry codegen link this crate into a system and call
//! `{module}::Component::register`. `nros metadata --build` records its
//! declarations into `metadata/{module}.json`. Platform + RMW are chosen at
//! Entry-package build time, not here.

pub mod {module} {{
    use nros::{{
        CallbackId, CdrReader, CdrWriter, ComponentContext, ComponentResult, DeserError,
        Deserialize, EntityId, NodeId, NodeOptions, RosMessage, SerError, Serialize, TimerDuration,
    }};

    pub struct Component;

    impl nros::Component for Component {{
        const NAME: &'static str = "{module}";

        fn register(context: &mut ComponentContext<'_>) -> ComponentResult<()> {{
            let mut node =
                context.create_node(NodeId::new("node_{module}"), NodeOptions::new("{module}"))?;
            let _publisher =
                node.create_publisher::<StringMsg>(EntityId::new("pub_chatter"), "chatter")?;
            let _timer = node.create_timer(
                EntityId::new("timer_publish"),
                CallbackId::new("cb_timer"),
                TimerDuration::from_millis(100),
            )?;
            Ok(())
        }}
    }}

    /// Minimal hand-rolled `std_msgs/String` stand-in. Replace with a generated
    /// message type (`nros generate-rust`) for real payloads.
    pub struct StringMsg;
    impl Serialize for StringMsg {{
        fn serialize(&self, _writer: &mut CdrWriter) -> Result<(), SerError> {{
            Ok(())
        }}
    }}
    impl Deserialize for StringMsg {{
        fn deserialize(_reader: &mut CdrReader) -> Result<Self, DeserError> {{
            Ok(Self)
        }}
    }}
    impl RosMessage for StringMsg {{
        const TYPE_NAME: &'static str = "std_msgs::msg::dds_::String_";
        const TYPE_HASH: &'static str = "std_msgs/String";
    }}
}}

// The planner links the component via the Rust type path above. To also expose
// the C / dynamic registration symbol (`__nros_component_register`), add:
//     nros::component!({module}::Component);
"#,
        name = cfg.name,
    );
    fs::write(dir.join("src/lib.rs"), lib_rs)?;

    // Folded `[component]` manifest (Phase 172 W.1). Minimal — no `[linkage]`
    // (derived) and no `[overrides]` (defaults to empty). The `crate::module`
    // component id is required by `nros metadata --build`.
    let nros_toml = format!(
        r#"# nano-ros component manifest (planned mode). A reusable node linked into a
# system by `nros plan` and Entry codegen. See
# docs/design/0004-configuration-and-transports.md.

[component]
version = 1
package = "{name}"
component = "{crate_name}::{module}"
language = "rust"

[component.metadata]
source_metadata = "metadata/{module}.json"
"#,
        name = cfg.name,
    );
    fs::write(dir.join("nros.toml"), nros_toml)?;

    println!("✓ Created nano-ros component '{}'", cfg.name);
    println!("  Component : {crate_name}::{module}");
    println!("  Kind      : planned-mode (library, linked by an Entry pkg)");
    println!();
    println!("Next steps:");
    println!("  cd {}", cfg.name);
    println!("  # add this package to a workspace's [system].components, then:");
    println!("  nros metadata --build   # record its source metadata");

    Ok(())
}

/// Scaffold a **C++ Node pkg** — typed component (RFC-0043). Emits the
/// RFC-0048 ament surface (`find_package(nano_ros)` + `nano_ros_add_node`) and a
/// `<UserClass>::configure(::nros::Node&)` real-callback body in the
/// `<pkg>::` namespace per §212.L.4 (class prefix must equal `PROJECT_NAME`).
/// `configure` creates a `Publisher` + binds a member timer callback by
/// identity (no string descriptor, no interpreter) — the typed Entry
/// constructs the object + calls `configure(node)` on the real executor.
fn scaffold_component_cpp(cfg: &ComponentScaffoldConfig) -> Result<()> {
    let dir = PathBuf::from(&cfg.name);
    if dir.exists() {
        if !cfg.force {
            bail!(
                "Directory '{}' already exists (use --force to overwrite)",
                cfg.name
            );
        }
        fs::remove_dir_all(&dir)?;
    }
    // Header lives at `include/<pkg>/<Class>.hpp` so the typed Entry can
    // `#include "<pkg>/<Class>.hpp"` (nano_ros_node_register adds `include/`).
    fs::create_dir_all(dir.join("src"))?;
    fs::create_dir_all(dir.join("include").join(cfg.name.replace('-', "_")))?;

    // Pkg name → namespace + class. `my-talker` → ns `my_talker`, class
    // `Talker` (PascalCase of use_case). §212.L.4 class prefix must equal
    // PROJECT_NAME (sanitised), so the namespace = the sanitised pkg name.
    let pkg_sym = cfg.name.replace('-', "_");
    let class_name = use_case_to_pascal(&cfg.use_case);
    let node_name = &cfg.use_case;

    let package_xml = format!(
        r#"<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>0.1.0</version>
  <description>{name} — nano-ros C++ Node pkg.</description>
  {maintainer}
  <license>Apache-2.0</license>
  <depend>std_msgs</depend>
  <export>
    <build_type>nros_cmake</build_type>
    <nano_ros deploy="native"/>
  </export>
</package>
"#,
        name = cfg.name,
        maintainer = maintainer_xml(),
    );
    fs::write(dir.join("package.xml"), package_xml)?;

    let cmake = format!(
        r#"cmake_minimum_required(VERSION 3.22)
# §212.L.4 — class prefix must equal PROJECT_NAME.
project({pkg_sym} VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)

# Typed component (RFC-0043). No add_executable, no `main()`; the linked
# Entry pkg's typed runtime constructs this class + calls `configure(node)`
# on the real executor. `DEPLOY native` also builds a standalone runnable
# single-node ELF via the typed native carrier.
nano_ros_add_node({node_name} CLASS {pkg_sym}::{class_name}
    SOURCES src/{class_name}.cpp
    DEPLOY  native)

# The verb generated the package.xml `<depend>` closure as INTERFACE libs
# (`std_msgs__nano_ros_cpp` etc.) carrying the generated headers' include
# dirs + FFI-glue link. The typed `Publisher<Int32>` needs them.
target_link_libraries({pkg_sym}_{node_name}_component
    PUBLIC std_msgs__nano_ros_cpp)
"#,
    );
    fs::write(dir.join("CMakeLists.txt"), cmake)?;

    let class_hpp = format!(
        r#"#pragma once

#include <nros/component.hpp>
#include <nros/nros.hpp>

#include "std_msgs.hpp"

namespace {pkg_sym} {{

/// {class_name} — a typed component (RFC-0043). `configure` creates a
/// publisher on `/chatter` and binds the member `on_tick` (by identity, no
/// name) as a 1 Hz timer callback that publishes a real counter. The typed
/// Entry constructs this object + calls `configure(node)`; the executor
/// dispatches `on_tick` during `spin_once`.
class {class_name} {{
    ::nros::Publisher<std_msgs::msg::Int32> pub_;
    ::nros::Timer timer_;
    int count_ = 0;

    void on_tick(); // real body; bound via &{class_name}::on_tick (no name)

  public:
    ::nros::Result configure(::nros::Node& node);
}};

}} // namespace {pkg_sym}
"#,
    );
    fs::write(
        dir.join("include")
            .join(&pkg_sym)
            .join(format!("{class_name}.hpp")),
        class_hpp,
    )?;

    let class_cpp = format!(
        r#"// Generated by `nros new {name} --component --lang cpp`.
//
// Typed component (RFC-0043). `configure` creates a `Publisher<Int32>` on
// `/chatter` + binds the member `on_tick` by identity as a 1 Hz timer; the
// callback publishes a real counter. No string descriptor, no interpreter.

#include "{pkg_sym}/{class_name}.hpp"

#include <cstdio>

namespace {pkg_sym} {{

void {class_name}::on_tick() {{
    std_msgs::msg::Int32 m;
    m.data = count_++;
    if (pub_.publish(m).ok()) {{
        std::printf("Published: %d\n", m.data);
    }}
}}

::nros::Result {class_name}::configure(::nros::Node& node) {{
    std::setvbuf(stdout, nullptr, _IONBF, 0);
    ::nros::Result r = node.create_publisher(pub_, "/chatter");
    if (!r.ok()) return r;
    // Member-fn-pointer-as-template-param → no-alloc trampoline; `this` is ctx.
    return ::nros::bind_timer<{class_name}, &{class_name}::on_tick>(node, timer_, 1000, this);
}}

}} // namespace {pkg_sym}
"#,
        name = cfg.name,
    );
    fs::write(dir.join(format!("src/{class_name}.cpp")), class_cpp)?;

    println!("✓ Created nano-ros C++ Node pkg '{}'", cfg.name);
    println!("  Class     : {pkg_sym}::{class_name}");
    println!("  Node      : {node_name}");
    println!("  Kind      : typed component (RFC-0043)");
    println!();
    println!("Next steps:");
    println!("  cd {}", cfg.name);
    println!("  # Solo build:");
    println!("  cmake -S . -B build -DNANO_ROS_ROOT=<path-to-nano-ros>");
    println!("  cmake --build build");
    println!();
    println!("  # Or add it as a SUBDIR in a workspace root that calls");
    println!("  # nano_ros_workspace(...), then build the workspace.");

    Ok(())
}

/// Scaffold a **C Node pkg** — typed component (RFC-0043). Emits a
/// `NROS_C_COMPONENT(<state>, <configure>)` factory/configure seam: a raw
/// `/chatter` publisher + 1 Hz timer that publishes a CDR-encoded Int32. The
/// typed Entry's `__nros_c_component_<pkg>_{create,configure}` calls run it on
/// the real executor — no declarative descriptor, no interpreter.
fn scaffold_component_c(cfg: &ComponentScaffoldConfig) -> Result<()> {
    let dir = PathBuf::from(&cfg.name);
    if dir.exists() {
        if !cfg.force {
            bail!(
                "Directory '{}' already exists (use --force to overwrite)",
                cfg.name
            );
        }
        fs::remove_dir_all(&dir)?;
    }
    fs::create_dir_all(dir.join("src"))?;

    let pkg_sym = cfg.name.replace('-', "_");
    let class_name = use_case_to_pascal(&cfg.use_case);
    let node_name = &cfg.use_case;
    let state_ty = format!("{pkg_sym}_t");
    let configure_fn = format!("{node_name}_configure");

    let package_xml = format!(
        r#"<?xml version="1.0"?>
<package format="3">
  <name>{name}</name>
  <version>0.1.0</version>
  <description>{name} — nano-ros C Node pkg.</description>
  {maintainer}
  <license>Apache-2.0</license>
  <export>
    <build_type>nros_cmake</build_type>
    <nano_ros deploy="native"/>
  </export>
</package>
"#,
        name = cfg.name,
        maintainer = maintainer_xml(),
    );
    fs::write(dir.join("package.xml"), package_xml)?;

    let cmake = format!(
        r#"cmake_minimum_required(VERSION 3.22)
# §212.L.4 — class prefix must equal PROJECT_NAME.
project({pkg_sym} VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(nano_ros REQUIRED)

# Typed C component (RFC-0043). The raw `/chatter` publisher carries the type
# name as a string, so no generated C bindings are needed. `DEPLOY native`
# also builds a standalone runnable single-node ELF via the typed C carrier.
nano_ros_add_node({node_name} CLASS {pkg_sym}::{class_name} TYPED
    SOURCES src/{class_name}.c
    DEPLOY  native)
"#,
    );
    fs::write(dir.join("CMakeLists.txt"), cmake)?;

    let source = format!(
        r#"// Generated by `nros new {name} --component --lang c`.
//
// Typed C component (RFC-0043). `{configure_fn}` creates a raw `/chatter`
// publisher + a 1 Hz timer publishing a CDR-encoded Int32 counter.
// `NROS_C_COMPONENT` emits the C-ABI factory/configure the typed Entry calls;
// it runs on the real executor — no declarative descriptor, no interpreter.

#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <nros/component.h>

typedef struct {{
    _Alignas(8) uint8_t pub[NROS_C_PUBLISHER_STORAGE_SIZE];
    int32_t count;
}} {state_ty};

static void write_u32_le(uint8_t* p, uint32_t v) {{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}}

static void on_tick(void* ctx) {{
    {state_ty}* self = ({state_ty}*)ctx;
    /* std_msgs/Int32 CDR: 4-byte encapsulation header (CDR_LE) + int32 data. */
    uint8_t buf[8];
    buf[0] = 0x00;
    buf[1] = 0x01;
    buf[2] = 0x00;
    buf[3] = 0x00;
    write_u32_le(buf + 4, (uint32_t)self->count);
    if (nros_cpp_publish_raw(self->pub, buf, sizeof(buf)) == 0) {{
        printf("Published: %d\n", (int)self->count);
    }}
    self->count++;
}}

static nros_ret_t {configure_fn}(const nros_cpp_node_t* node, void* executor, {state_ty}* self) {{
    self->count = 0;
    int32_t rc = nros_cpp_publisher_create(node, "/chatter", "std_msgs::msg::dds_::Int32_", "",
                                           nros_c_qos_default(), self->pub);
    if (rc != 0) {{
        return rc;
    }}
    size_t timer_handle;
    return nros_cpp_timer_create(executor, /*period_ms=*/1000, on_tick, self, &timer_handle);
}}

NROS_C_COMPONENT({state_ty}, {configure_fn})
"#,
        name = cfg.name,
    );
    fs::write(dir.join(format!("src/{class_name}.c")), source)?;

    println!("✓ Created nano-ros C Node pkg '{}'", cfg.name);
    println!("  Class     : {pkg_sym}::{class_name}");
    println!("  Node      : {node_name}");
    println!("  Kind      : typed component (RFC-0043)");
    println!();
    println!("Next steps:");
    println!("  cd {}", cfg.name);
    println!("  # Solo build:");
    println!("  cmake -S . -B build -DNANO_ROS_ROOT=<path-to-nano-ros>");
    println!("  cmake --build build");
    println!();
    println!("  # Or add it as a SUBDIR in a C++ or Rust Entry workspace.");

    Ok(())
}

/// Map `talker` → `Talker`, `service-server` → `ServiceServer`.
fn use_case_to_pascal(s: &str) -> String {
    s.split(['_', '-'])
        .filter(|p| !p.is_empty())
        .map(|p| {
            let mut chars = p.chars();
            match chars.next() {
                Some(c) => c.to_ascii_uppercase().to_string() + chars.as_str(),
                None => String::new(),
            }
        })
        .collect()
}

/// Issue 0333 defect 2 — emit a *runnable* Rust starter in the current blessed
/// idiom, dispatched by platform kind. Deferred platforms are rejected earlier
/// in [`scaffold_package`], so only [`PlatformKind::Hosted`] and
/// [`PlatformKind::SelfBringup`] reach here.
fn scaffold_rust(
    name: &str,
    platform: &str,
    rmw_feature: &str,
    edition_feature: &str,
    dir: &Path,
) -> Result<()> {
    let spec = platform_spec(platform)?;
    match &spec.kind {
        PlatformKind::Hosted => {
            scaffold_rust_hosted(name, &spec, rmw_feature, edition_feature, dir)
        }
        PlatformKind::SelfBringup { runtime } => {
            scaffold_rust_self_bringup(name, &spec, runtime, edition_feature, platform, dir)
        }
        // Rejected before any file is written (see `scaffold_package`).
        PlatformKind::Deferred { .. } => {
            unreachable!("Deferred platforms bail in scaffold_package")
        }
    }
}

/// Shared size/speed profiles + the `nros sync` note appended to every
/// generated `Cargo.toml`.
const CARGO_PROFILES: &str = r#"
# nano-ros crates are not published to crates.io (RFC-0040) — the `version = "*"`
# requirements above are patched, not resolved from crates.io. Run `nros sync`
# (with NROS_REPO_DIR set) to write the nros-managed [patch.crates-io] block here
# (path deps into your nano-ros checkout + any generated msg crates), then build.

# Named size/speed profiles (Phase 204.15): `cargo build --profile size|speed`.
[profile.size]
inherits = "release"
opt-level = "s"
lto = "fat"
codegen-units = 1
strip = true

[profile.speed]
inherits = "release"
opt-level = 3
lto = "fat"
codegen-units = 1
"#;

/// Hosted (native/posix) — a plain `fn main()` that registers the linked RMW,
/// opens an executor, and spins a String talker. Models
/// `examples/native/rust/talker`.
fn scaffold_rust_hosted(
    name: &str,
    spec: &PlatformSpec,
    rmw_feature: &str,
    edition_feature: &str,
    dir: &Path,
) -> Result<()> {
    let deploy = spec.deploy_token;
    let cargo_toml = format!(
        r#"[package]
name = "{name}"
version = "0.1.0"
edition = "2024"

[workspace]

[[bin]]
name = "{name}"
path = "src/main.rs"

# RFC-0048 — hosted application; the deploy target is the host.
[package.metadata.nros.application]
deploy = ["{deploy}"]

[features]
# RMW is a build-time choice (never source). The default is the backend you
# passed to `nros new`; switch with `--no-default-features --features rmw-<x>`.
default = ["{rmw_feature}"]
rmw-zenoh = ["dep:nros-rmw-zenoh", "nros-board-linux/rmw-zenoh"]
rmw-xrce = ["dep:nros-rmw-xrce-cffi", "nros-board-linux/rmw-xrce"]
rmw-cyclonedds = ["nros/rmw-cyclonedds", "dep:nros-rmw-cyclonedds-sys", "nros-board-linux/rmw-cyclonedds"]

[dependencies]
nros = {{ version = "*", default-features = false, features = ["std", "rmw-cffi", "{edition_feature}"] }}
nros-platform-cffi = {{ version = "*", features = ["posix-c-port"] }}
nros-board-linux = {{ version = "*", default-features = false }}
nros-rmw-zenoh = {{ version = "*", default-features = false, features = ["std", "platform-posix", "{edition_feature}"], optional = true }}
nros-rmw-xrce-cffi = {{ version = "*", default-features = false, features = ["std"], optional = true }}
nros-rmw-cyclonedds-sys = {{ version = "*", features = ["platform-posix"], optional = true }}
std_msgs = {{ version = "*", default-features = false }}
log = "0.4"
env_logger = "0.11"
{CARGO_PROFILES}"#
    );
    fs::write(dir.join("Cargo.toml"), cargo_toml)?;

    let main_rs = format!(
        r#"//! Generated by `nros new {name}` — a hosted nano-ros talker.
//!
//! Publishes `std_msgs/String` ("Hello World: N") on `/chatter` once a second.
//! The RMW backend is linked at build time (feature-selected) and registered
//! here before the executor opens — selection is build/config, never source.

use core::fmt::Write as _;

use log::{{error, info}};
use nros::prelude::*;
use std_msgs::msg::String as StringMsg;

fn main() {{
    // Register the RMW backend the build linked (idempotent; before the executor).
    nros_board_linux::register_linked_rmw();
    env_logger::init();

    // Launch-aware init: picks up ROS_DOMAIN_ID / NROS_LOCATOR / RMW_IMPLEMENTATION
    // from the environment, else the standard defaults.
    let ctx = nros::init_with_launch_auto().expect("nros init failed");
    let cfg = ctx.config("{name}");
    let mut executor: Executor = Executor::open(&cfg).expect("failed to open session");

    let publisher = {{
        let mut node = executor.create_node("{name}").expect("failed to create node");
        node.create_publisher::<StringMsg>("/chatter")
            .expect("failed to create publisher")
    }};

    let mut count: i32 = 0;
    executor
        .register_timer(nros::TimerDuration::from_millis(1000), move || {{
            count = count.wrapping_add(1);
            let mut msg = StringMsg::default();
            let _ = write!(msg.data, "Hello World: {{count}}");
            match publisher.publish(&msg) {{
                Ok(()) => info!("Publishing: '{{}}'", msg.data),
                Err(e) => error!("Publish error: {{:?}}", e),
            }}
        }})
        .expect("failed to register publish timer");

    executor
        .spin_blocking(SpinOptions::default())
        .expect("spin_blocking error");
}}
"#
    );
    fs::write(dir.join("src/main.rs"), main_rs)?;
    Ok(())
}

/// Self-bringup (baremetal/esp32) — `main.rs` is the one-line `nros::main!()`
/// Form-1 entry, `lib.rs` is the node. The macro reads
/// `[package.metadata.nros.entry] deploy` to resolve the board. Models the
/// single-package `examples/qemu-arm-baremetal/rust/talker` / esp32 talker.
fn scaffold_rust_self_bringup(
    name: &str,
    spec: &PlatformSpec,
    runtime: &SelfBringupRuntime,
    edition_feature: &str,
    platform: &str,
    dir: &Path,
) -> Result<()> {
    let name_snake = name.replace('-', "_");
    let board_crate = spec.board_crate;
    let deploy = spec.deploy_token;

    // Per-runtime dependency + entry deltas.
    let (runtime_deps, main_rs) = match runtime {
        SelfBringupRuntime::CortexM => (
            format!(
                r#"{board_crate} = {{ version = "*", features = ["board-entry"] }}
nros-rmw-zenoh = {{ version = "*", features = ["platform-bare-metal", "{edition_feature}"] }}
cortex-m = {{ version = "0.7", features = ["critical-section-single-core"] }}
cortex-m-rt = "0.7"
panic-semihosting = {{ version = "0.6", features = ["exit"] }}
"#
            ),
            // The macro emits `#[cortex_m_rt::entry]` for this deploy.
            "#![no_std]\n#![no_main]\nuse panic_semihosting as _;\nnros::main!();\n".to_string(),
        ),
        SelfBringupRuntime::Esp32 => (
            // The esp32 board crate's default features own the RMW backend.
            format!(
                r#"{board_crate} = {{ version = "*" }}
esp-hal = {{ version = "~1.0.0", features = ["esp32c3", "unstable"] }}
esp-backtrace = {{ version = "~0.18.0", features = ["esp32c3", "panic-handler", "println"] }}
"#
            ),
            // esp-backtrace is the panic handler; the macro emits `#[esp_hal::main]`.
            "#![no_std]\n#![no_main]\nuse esp_backtrace as _;\nesp_hal::esp_app_desc!();\nnros::main!();\n"
                .to_string(),
        ),
    };

    let cargo_toml = format!(
        r#"[package]
name = "{name}"
version = "0.1.0"
edition = "2024"
publish = false

[[bin]]
name = "{name}"
path = "src/main.rs"
test = false
bench = false

# Self-bringup shape (issue 0100): `lib.rs` IS the node (`nros::node!(Talker)`
# emits `register`); `main.rs` is the one-line `nros::main!()` Form-1 entry that
# dispatches to this same crate's `register`.
[lib]
path = "src/lib.rs"
crate-type = ["rlib"]

# `nros::main!()` reads this key to resolve the board (nros-orchestration-ir
# board_path_for). Every dep below is what that board needs to link + boot.
[package.metadata.nros.entry]
deploy = "{deploy}"

[package.metadata.nros.node]
class = "{name_snake}::Talker"
name = "{name}"
default_namespace = "/"
dispatch = "deferred"

[dependencies]
nros = {{ version = "*", default-features = false, features = ["alloc", "rmw-cffi", "{edition_feature}"] }}
{runtime_deps}nros-log = {{ version = "*", default-features = false }}
std_msgs = {{ version = "*", default-features = false }}
{CARGO_PROFILES}"#
    );
    fs::write(dir.join("Cargo.toml"), cargo_toml)?;
    fs::write(dir.join("src/main.rs"), main_rs)?;
    fs::write(dir.join("src/lib.rs"), self_bringup_node_lib(name))?;

    write_default_config_toml(dir)?;
    write_cargo_config(dir, platform)?;
    Ok(())
}

/// The shared `lib.rs` node for the self-bringup shape — a `std_msgs/String`
/// talker (`Node` + `ExecutableNode` + `nros::node!`), copied from the tracked
/// `examples/qemu-arm-baremetal/rust/talker/src/lib.rs`. RMW/platform-agnostic:
/// the same node compiles under every board.
fn self_bringup_node_lib(name: &str) -> String {
    format!(
        r#"//! Declarative talker node — RMW/platform-agnostic application logic.
//!
//! `nros::node!(Talker)` emits the `register` fn that `nros::main!()` (in
//! `main.rs`) dispatches to. The boot scaffold (reset -> BoardEntry -> executor
//! -> spin) is owned by `nros::main!()` + the board crate; none of it is here.
//!
//! Publishes `std_msgs/String` ("Hello World: N") on `/chatter` once a second.

#![no_std]

use core::fmt::Write as _;

use nros::{{
    Callback, CallbackCtx, DispatchStrategy, ExecutableNode, Node, NodeContext, NodeResult,
    TickCtx, TimerDuration,
}};
use nros_log::{{Logger, nros_error, nros_info}};
use std_msgs::msg::String as StringMsg;

static LOGGER: Logger = Logger::new("{name}");

pub struct Talker;

impl Node for Talker {{
    const NAME: &'static str = "{name}";
    const DISPATCH: DispatchStrategy = DispatchStrategy::Deferred;

    fn register(ctx: &mut NodeContext<'_>) -> NodeResult<()> {{
        nros_log::register_logger(&LOGGER);
        let mut node = ctx.create_node(nros::NodeOptions::new("{name}"))?;
        node.create_publisher_for_topic::<StringMsg>("/chatter")?;
        node.create_timer_for_callback_name("on_tick", TimerDuration::from_millis(1000))?;
        Ok(())
    }}
}}

impl ExecutableNode for Talker {{
    type State = i32;

    fn init() -> Self::State {{
        0
    }}

    fn on_callback(state: &mut i32, callback: Callback<'_>, ctx: &mut CallbackCtx<'_>) {{
        if callback.as_str() == "on_tick" {{
            *state = state.wrapping_add(1);
            let mut msg = StringMsg::default();
            let _ = write!(msg.data, "Hello World: {{}}", *state);
            match ctx.publish_to_topic::<StringMsg, 64>("/chatter", &msg) {{
                Ok(()) => nros_info!(&LOGGER, "Publishing: '{{}}'", msg.data),
                Err(e) => nros_error!(&LOGGER, "Publish failed: {{:?}}", e),
            }}
        }}
    }}

    fn tick(_state: &mut Self::State, _ctx: &mut TickCtx<'_>) {{}}
}}

nros::node!(Talker);
"#
    )
}

/// Scaffold `.cargo/config.toml` for the cortex-m cargo-built platforms
/// (bare-metal / FreeRTOS on the QEMU mps2-an385). Carries the Phase 204
/// size knobs by default: `--gc-sections` at link (204.8) plus a documented,
/// commented block for the serial-only IP-stack opt-out (204.7) and the
/// per-backend static-heap size (204.5). Other embedded platforms (Zephyr,
/// NuttX, ESP-IDF, ThreadX) build through their own toolchains and don't use
/// a cargo target triple here, so they get no `.cargo/config.toml`.
fn write_cargo_config(dir: &Path, platform: &str) -> Result<()> {
    let triple = match platform {
        "baremetal" | "freertos" => "thumbv7m-none-eabi",
        // Non-cargo-target build flows — leave the build config to the
        // platform's own toolchain integration.
        _ => return Ok(()),
    };

    let config = format!(
        r#"[build]
target = "{triple}"

[target.{triple}]
# QEMU mps2-an385 (cortex-m3) + semihosting. Adjust `-machine`/`-cpu` for your board.
runner = "qemu-system-arm -cpu cortex-m3 -machine mps2-an385 -nographic -semihosting-config enable=on,target=native -kernel"
rustflags = [
    # Phase 204.8 — drop unreferenced fns/data at link. `rust-lld` is invoked
    # directly (no gcc driver), so the bare `--gc-sections`, NOT `-Wl,...`.
    # `cortex-m-rt`'s `link.x` KEEPs the vector table, so gc is safe.
    "-C", "link-arg=--gc-sections",
    "-C", "link-arg=-Tlink.x",
]

[env]
# Phase 204.7 — serial-only node: uncomment to drop the IP link layer
# (zenoh-pico TCP/UDP link C via `Z_FEATURE_LINK_TCP/UDP=0`; `--gc-sections`
# above then strips the smoltcp residue). Also switch the board to its
# `serial` feature and use a serial `locator` in `nros.toml`.
# NROS_LINK_IP = "0"
# ZPICO_NO_SMOLTCP = "1"
#
# Phase 204.5 — size the static heap to the backend's working set
# (zenoh-pico ~24 KB, XRCE ~8 KB); default is the per-board value (64 KB on
# mps2-an385). Decimal bytes.
# NROS_HEAP_SIZE = "24576"
#
# Phase 204.2 — a brokered (zenoh/XRCE) client multiplexes over one session;
# drop the spare smoltcp socket buffers (sized for DDS RTPS by default).
# NROS_SMOLTCP_MAX_SOCKETS = "1"
# NROS_SMOLTCP_MAX_UDP_SOCKETS = "1"
"#
    );

    let cargo_dir = dir.join(".cargo");
    fs::create_dir_all(&cargo_dir)?;
    fs::write(cargo_dir.join("config.toml"), config)?;
    Ok(())
}

fn scaffold_c(name: &str, platform: &str, rmw_value: &str, dir: &Path) -> Result<()> {
    let _ = platform;
    let _ = rmw_value;
    let cmake = format!(
        r#"cmake_minimum_required(VERSION 3.22)
project({name} VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# RFC-0048 — ament shape. Platform + RMW live in package.xml's
# <export><nano_ros deploy= rmw=/>; find_package(nano_ros) reads it.
find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)

nano_ros_add_executable({name} src/main.c)
ament_target_dependencies({name} std_msgs)

install(TARGETS {name} DESTINATION lib/${{PROJECT_NAME}})
ament_package()
"#,
    );
    fs::write(dir.join("CMakeLists.txt"), cmake)?;

    let main_c = format!(
        r#"// Generated by `nros new {name} --lang c`.
//
// Minimal nano-ros C talker — publishes one `std_msgs/Int32` message on
// `/chatter`, then returns. Swap the body for your own logic; see
// `examples/native/c/talker/src/main.c` for a fuller shape (timer,
// executor, signal handler).

#include <stdio.h>

#include <nros/init.h>
#include <nros/node.h>
#include <nros/publisher.h>
#include "std_msgs.h"

int main(int argc, char** argv) {{
    (void)argc;
    (void)argv;

    nros_support_t support = nros_support_get_zero_initialized();
    if (nros_support_init(&support, NULL, 0) != NROS_RET_OK) {{
        fprintf(stderr, "nros_support_init failed\n");
        return 1;
    }}

    nros_node_t node = rcl_get_zero_initialized_node();
    if (nros_node_init(&node, &support, "{name}", "/") != NROS_RET_OK) {{
        fprintf(stderr, "nros_node_init failed\n");
        return 1;
    }}

    nros_publisher_t pub = rcl_get_zero_initialized_publisher();
    if (nros_publisher_init(&pub, &node,
                            std_msgs_msg_int32_get_type_support(),
                            "/chatter") != NROS_RET_OK) {{
        fprintf(stderr, "nros_publisher_init failed\n");
        return 1;
    }}

    std_msgs_msg_int32 msg;
    std_msgs_msg_int32_init(&msg);
    msg.data = 0;
    (void)std_msgs_msg_int32_publish(&pub, &msg);
    printf("{name}: published 0 on /chatter\n");

    nros_publisher_fini(&pub);
    nros_node_fini(&node);
    nros_support_fini(&support);
    return 0;
}}
"#,
    );
    fs::write(dir.join("src/main.c"), main_c)?;

    if needs_scaffolded_nros_toml(platform) {
        write_default_config_toml(dir)?;
    }
    Ok(())
}

fn scaffold_cpp(name: &str, platform: &str, rmw_value: &str, dir: &Path) -> Result<()> {
    let _ = platform;
    let _ = rmw_value;
    let cmake = format!(
        r#"cmake_minimum_required(VERSION 3.22)
project({name} VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# RFC-0048 — ament shape. Platform + RMW live in package.xml's
# <export><nano_ros deploy= rmw=/>; find_package(nano_ros) reads it.
find_package(nano_ros REQUIRED)
find_package(std_msgs REQUIRED)

nano_ros_add_executable({name} src/main.cpp)
ament_target_dependencies({name} std_msgs)

install(TARGETS {name} DESTINATION lib/${{PROJECT_NAME}})
ament_package()
"#,
    );
    fs::write(dir.join("CMakeLists.txt"), cmake)?;

    let main_cpp = format!(
        r#"// Generated by `nros new {name} --lang cpp`.
//
// Minimal nano-ros C++ talker — publishes one `std_msgs/Int32` message on
// `/chatter`, then returns. Swap the body for your own logic; see
// `examples/native/cpp/talker/src/main.cpp` for a fuller shape (timer,
// executor, signal handler).

#include <cstdio>

#include <nros/nros.hpp>
#include "std_msgs.hpp"

int main(int argc, char** argv) {{
    (void)argc;
    (void)argv;

    if (auto r = nros::init(); !r.ok()) {{
        std::fprintf(stderr, "nros::init failed: %d\n", r.raw());
        return 1;
    }}

    nros::Node node;
    if (auto r = nros::create_node(node, "{name}"); !r.ok()) {{
        std::fprintf(stderr, "create_node failed: %d\n", r.raw());
        return 1;
    }}

    nros::Publisher<std_msgs::msg::Int32> pub;
    if (auto r = node.create_publisher(pub, "/chatter"); !r.ok()) {{
        std::fprintf(stderr, "create_publisher failed: %d\n", r.raw());
        return 1;
    }}

    std_msgs::msg::Int32 msg;
    msg.data = 0;
    (void)pub.publish(msg);
    std::printf("{name}: published 0 on /chatter\n");

    nros::shutdown();
    return 0;
}}
"#,
    );
    fs::write(dir.join("src/main.cpp"), main_cpp)?;

    if needs_scaffolded_nros_toml(platform) {
        write_default_config_toml(dir)?;
    }
    Ok(())
}

/// Does this platform need a scaffolded `nros.toml`?
///
/// Only a NON-hosted target does: the file it writes is a static QEMU-slirp
/// network config (`ip`, `mac`, `gateway`, `locator`), which a host build has
/// no use for — the host kernel owns the stack.
///
/// Issue 0916: this was `platform != "native"`, a literal comparison against
/// ONE of the two host spellings. `native` and `posix` are aliases for the same
/// `PlatformKind::Hosted`, differing only in `deploy_token`, so
/// `--platform posix` got an embedded network config and `--platform native`
/// did not — two spellings of one platform producing different trees.
///
/// Asking the KIND is the fix, not adding `&& platform != "posix"`: a third
/// hosted alias would reintroduce it, and the question being asked was never
/// about the name.
fn needs_scaffolded_nros_toml(platform: &str) -> bool {
    !matches!(
        platform_spec(platform).map(|s| s.kind),
        Ok(PlatformKind::Hosted)
    )
}

fn write_default_config_toml(dir: &Path) -> Result<()> {
    // Phase 172.K — scaffold the direct-mode nros.toml shape (one node + one
    // ethernet transport), not the retired config.toml.
    let nros_toml = r#"# nano-ros config (direct mode). See
# docs/design/0004-configuration-and-transports.md.

[node]
domain_id = 0

# CONFIGURE ME: these defaults target QEMU slirp (10.0.2.0/24, gateway/router
# at 10.0.2.2). Set ip/gateway/locator to your board's network + zenoh router.
[[transport]]
kind    = "ethernet"
ip      = "10.0.2.20/24"
mac     = "02:00:00:00:00:00"
gateway = "10.0.2.2"
locator = "tcp/10.0.2.2:7447"
"#;
    fs::write(dir.join("nros.toml"), nros_toml)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp() -> tempfile::TempDir {
        let d = tempfile::tempdir().unwrap();
        fs::create_dir_all(d.path().join("src")).unwrap();
        d
    }

    #[test]
    fn rust_scaffold_hosted_defaults_to_declared_rmw() {
        // Phase 227.4 / issue 0333 — `--rmw xrce` makes the hosted feature set
        // default to rmw-xrce (all three backends are wired, one is the default).
        let d = tmp();
        scaffold_rust("foo", "native", "rmw-xrce", "ros-humble", d.path()).unwrap();
        let toml = fs::read_to_string(d.path().join("Cargo.toml")).unwrap();
        assert!(
            toml.contains(r#"default = ["rmw-xrce"]"#),
            "expected rmw-xrce default in:\n{toml}"
        );
    }

    #[test]
    fn rust_scaffold_hosted_emits_a_runnable_manual_main() {
        // Issue 0333 defect 2 — native is the hosted shape: a real `fn main()`
        // that registers the linked RMW and spins, NOT the retired no_mangle stub.
        let d = tmp();
        scaffold_rust("foo", "native", "rmw-cyclonedds", "ros-humble", d.path()).unwrap();
        let toml = fs::read_to_string(d.path().join("Cargo.toml")).unwrap();
        assert!(toml.contains(r#"default = ["rmw-cyclonedds"]"#), "{toml}");
        assert!(toml.contains("nros-board-linux = {"), "{toml}");
        let main = fs::read_to_string(d.path().join("src/main.rs")).unwrap();
        assert!(
            main.contains("register_linked_rmw()") && main.contains("spin_blocking"),
            "hosted main must be the runnable shape:\n{main}"
        );
        assert!(
            !main.contains("no_mangle"),
            "retired stub still emitted:\n{main}"
        );
        assert!(
            !d.path().join("src/lib.rs").exists(),
            "hosted needs no lib.rs"
        );
    }

    #[test]
    fn c_and_cpp_scaffold_emit_ament_shape() {
        // RFC-0048 — the CMakeLists is the ament shape; RMW moved to the
        // package.xml <nano_ros> tuple (written by scaffold_package), so it no
        // longer appears in the CMakeLists.
        let dc = tmp();
        scaffold_c("bar", "native", "cyclonedds", dc.path()).unwrap();
        let cm = fs::read_to_string(dc.path().join("CMakeLists.txt")).unwrap();
        assert!(
            cm.contains("find_package(nano_ros REQUIRED)"),
            "expected ament shape in:\n{cm}"
        );
        assert!(
            cm.contains("nano_ros_add_executable(bar src/main.c)"),
            "{cm}"
        );
        assert!(
            !cm.contains("nano_ros_entry"),
            "stray old-shape verb:\n{cm}"
        );
        assert!(
            !cm.contains("set(NANO_ROS_RMW"),
            "RMW should be in package.xml:\n{cm}"
        );

        let dpp = tmp();
        scaffold_cpp("baz", "native", "xrce", dpp.path()).unwrap();
        let cm = fs::read_to_string(dpp.path().join("CMakeLists.txt")).unwrap();
        assert!(
            cm.contains("nano_ros_add_executable(baz src/main.cpp)"),
            "{cm}"
        );
    }

    #[test]
    fn scaffold_package_rejects_unknown_rmw() {
        // `resolve_rmw` fails before any filesystem write, so no package dir is
        // created — the unique name keeps this safe without touching cwd.
        let cfg = ScaffoldConfig {
            name: "pkg_unknown_rmw_227_4_fixture".to_string(),
            lang: "rust".to_string(),
            platform: "native".to_string(),
            rmw: "dust-dds".to_string(),
            ros_edition: "humble".to_string(),
            use_case: "talker".to_string(),
            force: false,
        };
        let err = scaffold_package(&cfg).expect_err("unknown rmw must fail");
        assert!(err.to_string().contains("dust-dds"), "{err}");
        assert!(
            !PathBuf::from(&cfg.name).exists(),
            "no package dir on rejected rmw"
        );
    }

    // Issue 0333 — every platform the CLI advertises must resolve; an unknown
    // one bails.
    #[test]
    fn platform_spec_covers_every_advertised_platform() {
        for p in [
            "native",
            "posix",
            "freertos",
            "baremetal",
            "nuttx",
            "threadx",
            "zephyr",
            "esp32",
        ] {
            platform_spec(p).unwrap_or_else(|e| panic!("{p} must resolve: {e}"));
        }
        // native/posix are hosted against nros-board-linux (posix's deploy token
        // resolves to LinuxBoard, not nros-board-linux).
        assert!(matches!(
            platform_spec("posix").unwrap().kind,
            PlatformKind::Hosted
        ));
        assert_eq!(
            platform_spec("posix").unwrap().board_crate,
            "nros-board-linux"
        );
        assert!(matches!(
            platform_spec("baremetal").unwrap().kind,
            PlatformKind::SelfBringup { .. }
        ));
        assert!(matches!(
            platform_spec("threadx").unwrap().kind,
            PlatformKind::Deferred { .. }
        ));
        assert!(platform_spec("bogus").is_err());
    }

    // Issue 0333 defect 2 — the self-bringup shape writes a real board dep, a
    // `[lib]`, a `nros::node!` lib.rs, and a one-line `nros::main!()` entry —
    // never the retired commented-out dep or `no_mangle` stub.
    #[test]
    fn scaffold_rust_self_bringup_emits_node_lib_and_entry() {
        for (platform, crate_name) in [
            ("baremetal", "nros-board-mps2-an385"),
            ("esp32", "nros-board-esp32-qemu"),
        ] {
            let d = tmp();
            scaffold_rust("app", platform, "rmw-zenoh", "ros-humble", d.path()).unwrap();
            let toml = fs::read_to_string(d.path().join("Cargo.toml")).unwrap();
            assert!(
                toml.contains(&format!("{crate_name} = {{")),
                "{platform}: board dep missing:\n{toml}"
            );
            assert!(
                !toml.contains("# TODO: add board crate"),
                "{platform}:\n{toml}"
            );
            assert!(toml.contains("[lib]"), "{platform}: missing [lib]:\n{toml}");
            assert!(
                toml.contains(r#"deploy = ""#),
                "{platform}: missing entry deploy token:\n{toml}"
            );
            let main = fs::read_to_string(d.path().join("src/main.rs")).unwrap();
            assert!(main.contains("nros::main!();"), "{platform}:\n{main}");
            assert!(
                !main.contains("no_mangle"),
                "{platform}: retired stub:\n{main}"
            );
            let lib = fs::read_to_string(d.path().join("src/lib.rs")).unwrap();
            assert!(
                lib.contains("nros::node!(Talker)"),
                "{platform}: node lib missing:\n{lib}"
            );
        }
    }

    // Issue 0333 defect 2 — split-package (freertos/nuttx/threadx) and west
    // (zephyr) platforms have no single-package template yet; `nros new` must
    // bail before writing any file, not emit a project that cannot run.
    #[test]
    fn scaffold_package_rust_defers_split_and_zephyr_platforms() {
        for (i, platform) in ["freertos", "nuttx", "threadx", "zephyr"]
            .into_iter()
            .enumerate()
        {
            let cfg = ScaffoldConfig {
                name: format!("pkg_deferred_0333_fixture_{i}"),
                lang: "rust".to_string(),
                platform: platform.to_string(),
                rmw: "zenoh".to_string(),
                ros_edition: "humble".to_string(),
                use_case: "talker".to_string(),
                force: false,
            };
            let err = scaffold_package(&cfg).expect_err("deferred platform must bail");
            assert!(
                err.to_string().contains("not available yet"),
                "{platform}: {err}"
            );
            assert!(
                !PathBuf::from(&cfg.name).exists(),
                "{platform}: no package dir on deferred platform"
            );
        }
    }
}

#[cfg(test)]
mod hosted_alias_tests {
    use super::*;

    /// `native` and `posix` are two spellings of one platform, so they must
    /// scaffold the same tree.
    ///
    /// Issue 0916: they did not. The C and C++ scaffolds asked
    /// `platform != "native"`, a literal comparison against one of the two
    /// aliases, so `--platform posix` got an `nros.toml` full of static
    /// QEMU-slirp network settings that a host build has no use for.
    #[test]
    fn the_two_hosted_spellings_agree_about_nros_toml() {
        assert!(!needs_scaffolded_nros_toml("native"));
        assert!(!needs_scaffolded_nros_toml("posix"));
    }

    /// …and a cross target still gets one, which is what the file is for.
    #[test]
    fn a_cross_target_still_gets_nros_toml() {
        assert!(needs_scaffolded_nros_toml("baremetal"));
        assert!(needs_scaffolded_nros_toml("freertos"));
    }

    /// An unknown platform is not silently treated as hosted. `platform_spec`
    /// errors for it, and erring toward writing the file means the scaffold
    /// output is inspectable rather than mysteriously missing a config.
    #[test]
    fn an_unknown_platform_is_not_assumed_hosted() {
        assert!(needs_scaffolded_nros_toml("no-such-platform"));
    }
}
