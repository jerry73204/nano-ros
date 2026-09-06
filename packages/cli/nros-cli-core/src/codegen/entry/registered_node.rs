//! phase-432 W2.6 — the entry TU for ONE registered node, from CMake's facts.
//!
//! `nano_ros_node_register()` (the `nano_ros_add_node` verb) builds a bootable
//! image out of a SINGLE component: one class, one node, one board. It has no
//! launch tree and no `SystemModel`, so it cannot reach [`super::plan_from_model`]
//! — and until this wave it reached no emitter at all. It rendered its own copy
//! of the entry TU through `configure_file()` on
//! `cmake/templates/*_entry_main*.cpp.in`.
//!
//! That was a SECOND PRODUCER of one artifact, and it drifted exactly as a
//! second producer does. Issue 1003: ten of those templates called
//! `run_components(&__nros_entry_setup)` with one argument, binding the
//! delegating overload that hard-codes `"node"`. Every image built through the
//! verb registered under that name; the XRCE client key derives from it, so a
//! talker and a listener hashed to ONE client. It lived from 2026-06-13 to
//! 2026-09-03 beside a correct sibling producer.
//!
//! # Why the facts arrive as ARGUMENTS and not as a model
//!
//! The obvious alternative is to have CMake synthesise a `SystemModel` YAML and
//! feed [`super::plan_from_model`]. That trades a duplicated TEMPLATE for a
//! duplicated IR, which is the worse half: a `SystemModel` is a build artifact
//! of `nros sync`, never hand-authored, and a CMake-side YAML writer would be a
//! third producer of model semantics. `plan_from_model` also requires
//! `execution.deploy` to place at least one node on the board and hard-errors on
//! an empty slice — facts a single registered node does not have and would have
//! to invent.
//!
//! So the node's facts cross the boundary as what they are, and the SYNTHESIS
//! lives here, in Rust. That is the same shape
//! [`crate::orchestration::metadata_probe_cmake::render_probe_main`] already
//! uses for the metadata probe: one node's fields in, a one-node [`Plan`] built
//! in Rust, the real entry emitter out. This generalises that path from the
//! probe to the actual entry.
//!
//! # Why every board renders a C++ TU
//!
//! All six retired templates emitted C++, including the C ones: the entry drives
//! `Board::run_components` and holds the `nros::Node`, while the C component is
//! reached through its `NROS_C_COMPONENT` factory/configure seam. The C emitter
//! ([`super::emit_c`]) would emit a `.c` TU calling `nros_board_native_*`, which
//! is both a different boot seam and — for every board but native — the WRONG
//! one (it hardcodes the native runner whatever the board; see the phase-432
//! doc's blocking-site list). Routing unconditionally through
//! [`super::emit_cpp`] preserves the `main.cpp` filename the RTOS link pass
//! matches on (`nros_board_link_app`'s `/main\.cpp$` MAIN_SOURCE rule) and the
//! board-correct runner.

use super::{Plan, PlanNode};

/// One registered node's facts, exactly as `nano_ros_node_register()` knows
/// them.
///
/// Every field here was a `@VAR@` substitution in the retired templates; the
/// mapping is 1:1 on purpose, so a reader can put the old template and this
/// struct side by side.
#[derive(Debug, Clone)]
pub struct RegisteredNode {
    /// Board key — `native`, `zephyr`, `nuttx`, `threadx`, `freertos`. Selects
    /// the board class and the boot shape.
    pub board: String,
    /// The node's own name (`@NROS_ENTRY_NODE_NAME@`), from cmake's
    /// `${_NRC_NAME}`. This is the SESSION name, and issue 1003 is what
    /// happens when it goes missing.
    pub node_name: String,
    /// Sanitized package symbol (`@NROS_ENTRY_PKG_SYM@`) — the infix of the C
    /// component's `__nros_c_component_<pkg>_{create,configure}` seam.
    pub pkg_sym: String,
    /// `"c"` or `"cpp"`.
    pub language: String,
    /// Fully-qualified component class (`@NROS_ENTRY_CLASS@`). C++ only.
    pub class: Option<String>,
    /// Header to include (`@NROS_ENTRY_CLASS_HEADER@`). C++ only.
    pub header: Option<String>,
    /// `"rclcpp"` (the component IS-A node) or `"configure"`. C++ only; this is
    /// `@NROS_ENTRY_SHAPE_RCLCPP@` in its unencoded form — cmake passed a 0/1
    /// that the template turned into a preprocessor branch, and the branch is
    /// now taken in Rust where it can be read.
    pub shape: Option<String>,
}

impl RegisteredNode {
    /// The one-node [`Plan`] this node's image is.
    ///
    /// Everything a launch tree would contribute is empty, and that is a scope
    /// statement rather than an omission: `nano_ros_node_register` carries no
    /// tiers, params, remaps, QoS overrides or services, because the verb has
    /// no syntax for any of them. An image that needs them uses
    /// `nano_ros_entry(BRINGUP … LAUNCH …)`, which resolves a real model.
    pub fn plan(&self) -> Plan {
        Plan {
            board: self.board.clone(),
            // The banner reads `bringup`/`launch`. There is no bringup and no
            // launch file, and saying so plainly beats inventing a path that
            // does not exist — a reader of the generated TU should be able to
            // tell which producer made it.
            bringup: format!("{} (nano_ros_node_register)", self.node_name),
            launch_file: std::path::PathBuf::from("<no launch — single registered node>"),
            nodes: vec![PlanNode {
                // `pkg` is what the C seam's symbol is built from, so it must be
                // the SANITIZED spelling cmake compiled the component with
                // (`-DNROS_PKG_NAME=<pkg_sym>`); anything else emits an extern
                // declaration that resolves to nothing at link time.
                pkg: self.pkg_sym.clone(),
                exec: self.node_name.clone(),
                name: Some(self.node_name.clone()),
                namespace: None,
                class_name: self.class.clone(),
                class_header: self.header.clone(),
                lang: Some(self.language.clone()),
                shape: self.shape.clone(),
                qos_overrides: Vec::new(),
                params: Vec::new(),
                remaps: Vec::new(),
                callback_groups: Vec::new(),
                sched_context: None,
                group_tiers: std::collections::BTreeMap::new(),
            }],
            // The generated TU is a function of the CLI ARGUMENTS alone — it
            // reads no file — so there is nothing for a depfile to name. The
            // tool's own mtime is the edge that matters here, and cmake adds it
            // with `nros_codegen_tool_reconfigure()` (issue 1018).
            depfile_paths: Vec::new(),
            lifecycle: None,
            param_services: false,
            safety: None,
            tiers: Default::default(),
            node_overrides: Vec::new(),
            resolved_tiers: None,
        }
    }

    /// The entry TU.
    ///
    /// Always the C++ emitter — see the module docs for why a C component still
    /// gets a C++ TU.
    pub fn emit(&self) -> Result<String, String> {
        super::emit_cpp::emit_typed(&self.plan())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cpp(board: &str) -> RegisteredNode {
        RegisteredNode {
            board: board.into(),
            node_name: "talker".into(),
            pkg_sym: "talker_pkg".into(),
            language: "cpp".into(),
            class: Some("talker_pkg::Talker".into()),
            header: Some("talker_pkg/Talker.hpp".into()),
            shape: Some("configure".into()),
        }
    }

    fn c(board: &str) -> RegisteredNode {
        RegisteredNode {
            board: board.into(),
            node_name: "listener".into(),
            pkg_sym: "listener_pkg".into(),
            language: "c".into(),
            class: None,
            header: None,
            shape: None,
        }
    }

    /// Issue 1003, as an assertion rather than a template review.
    ///
    /// The defect was a `run_components` call that named no session, so every
    /// image of a language collided on one client key. Every board shape must
    /// carry the node's name into the emitted call — via the boot config blob,
    /// which is where the shared emitter puts it.
    #[test]
    fn every_board_names_the_session() {
        for board in ["native", "zephyr", "nuttx", "threadx", "freertos"] {
            for n in [cpp(board), c(board)] {
                let src = n.emit().expect("emit");
                assert!(
                    src.contains("nros_boot_config_node_name(&NROS_BOOT_CONFIG)"),
                    "{board}/{}: run_components names no session (issue 1003)",
                    n.language
                );
                assert!(
                    src.contains(&format!(".node_name  = \"{}\"", n.node_name)),
                    "{board}/{}: the boot config does not carry the node's own name",
                    n.language
                );
            }
        }
    }

    /// The C seam is built from the SANITIZED package symbol cmake compiled the
    /// component with. A mismatch here links against nothing, and the error
    /// arrives at link time naming a mangled symbol rather than a config.
    #[test]
    fn a_c_node_declares_its_factory_seam() {
        let src = c("nuttx").emit().expect("emit");
        assert!(
            src.contains("__nros_c_component_listener_pkg_create"),
            "the C factory seam is not declared:\n{src}"
        );
        assert!(
            src.contains("__nros_c_component_listener_pkg_configure"),
            "the C configure seam is not declared:\n{src}"
        );
    }

    /// The three boot shapes the retired templates spelled by hand. `native` is
    /// a host `main`, Zephyr owns the C `main` symbol, and the RTOS families
    /// whose board `startup.c` owns `main` get `nros_app_main` +
    /// `NROS_APP_MAIN_REGISTER_VOID`.
    #[test]
    fn the_boot_shape_follows_the_board() {
        let zephyr = cpp("zephyr").emit().expect("emit");
        assert!(zephyr.contains("int main(void)"), "zephyr is a kernel main");
        assert!(
            !zephyr.contains("nros_app_main"),
            "zephyr must NOT emit the kernel-app entry"
        );

        for board in ["nuttx", "threadx", "freertos"] {
            let src = cpp(board).emit().expect("emit");
            assert!(
                src.contains("extern \"C\" int nros_app_main("),
                "{board}: the board's startup.c owns main, so the entry is nros_app_main"
            );
            assert!(
                src.contains("NROS_APP_MAIN_REGISTER_VOID();"),
                "{board}: the kernel-app forwarding symbol is missing"
            );
        }

        let native = cpp("native").emit().expect("emit");
        assert!(
            native.contains("int main(int /*argc*/, char** /*argv*/)"),
            "native is a host main taking argc/argv"
        );
        // The host board resolves its locator at run time, so its
        // `run_components` is the one that takes no locator argument.
        assert!(
            !native.contains("NROS_ENTRY_LOCATOR"),
            "the host board must not reference the compile-time locator"
        );
    }

    /// The rclcpp shape was `@NROS_ENTRY_SHAPE_RCLCPP@` — a 0/1 cmake wrote into
    /// a preprocessor branch, so BOTH arms compiled into every TU and only one
    /// survived the preprocessor. Taken in Rust, only the live arm is emitted.
    #[test]
    fn the_rclcpp_shape_places_the_component_in_an_arena() {
        let mut n = cpp("native");
        n.shape = Some("rclcpp".into());
        let src = n.emit().expect("emit");
        assert!(
            src.contains("__nros_comp_buf_0[sizeof(::talker_pkg::Talker)]"),
            "the rclcpp shape needs its arena slot:\n{src}"
        );
        assert!(
            !src.contains(".configure(__nros_node_0)"),
            "the configure-shape arm must not be emitted for an rclcpp component"
        );
    }
}
