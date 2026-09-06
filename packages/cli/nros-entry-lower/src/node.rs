//! The per-node runtime bake — the entry fact that has actually drifted.
//!
//! phase-432 W2.4. Every generated entry, in every language, writes four
//! pieces of per-node state immediately before the node's `register` call:
//! its launch `<param>` initials, its `<remap>` rules, its QoS overrides and
//! its `<node name= namespace=>` identity. Two producers emit that block —
//! the `nros::main!()` proc-macro (`quote!`) and `nros codegen entry --lang
//! rust` (`emit_rust`, a template) — and archived issue **0302** is the bill
//! for their having derived it twice: params (phase-264 W4a), identity
//! (phase-268 W1), remaps (phase-305 W3 / issue 0255) and QoS overrides
//! (issue #52) each landed in the proc-macro and left the emitter behind, so a
//! CLI-baked entry ran every node with default parameters, no remaps, its own
//! hardcoded name and no QoS overrides — **from the same plan**.
//!
//! So the types here are Stage 2 for that block: computed once, rendered
//! twice, and compared (see the parity corpus under `testdata/parity/`).
//!
//! ## What is a FACT here and what is a SPELLING elsewhere
//!
//! RFC-0091 §8b: this stage carries no rendered text and no language-specific
//! syntax. A parameter is a `(name, value)` PAIR, not `("rate", "10")`; an
//! identity is `Some(name, namespace)`, not
//! `::core::option::Option::Some(…)`; a board is a KEY, not
//! `::nros_board_linux::LinuxBoard`. Quoting in particular stays with the
//! pack — it is a correctness property, and the two packs quote differently
//! on purpose (`emit_rust` matches the proc-macro's `LitStr`, i.e. plain
//! quoted form, for paths where it uses a raw string).
//!
//! The one thing that IS computed here rather than spelled is
//! [`sanitize_pkg`]: "which identifier does this package name become" is a
//! question with one answer, and both producers were answering it separately.

use alloc::{
    string::{String, ToString},
    vec::Vec,
};

/// One QoS override, in the lowered wire form every language ABI uses.
///
/// The codes are produced by `nros_orchestration_ir::qos_override::lower_all`,
/// which BOTH producers already call (issue 0303 — an override it cannot
/// lower is an error rather than a silent drop). They are carried here as
/// numbers, not as a rendered tuple, because a C pack writes
/// `{ "/t", 1, 2, 5 }` where a Rust pack writes `("/t", 1, 2, 5)`.
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct QosOverride {
    pub topic: String,
    pub role: u8,
    pub policy: u8,
    pub value: u32,
}

/// A node's launch identity — the `<node name= namespace=>` pair.
///
/// `None` on [`LoweredNode::identity`] is not the same as an empty pair: it
/// means "keep the node's own name", which is the self-bringup arm. The
/// producers must still WRITE the reset in that case, which is why this is an
/// `Option` on the node rather than two empty strings.
#[derive(Clone, Debug, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct NodeIdentity {
    pub name: String,
    pub namespace: String,
}

/// One node's runtime state, as the entry must set it before `register`.
///
/// **Every field is rendered unconditionally, including when it is empty.**
/// That reset discipline is load-bearing rather than stylistic: the `runtime`
/// context is REUSED across the nodes of one entry, so a node with no params
/// must CLEAR the previous node's rather than inherit them. A producer that
/// emits an assignment only when non-empty leaks state between nodes and still
/// passes any "does the output contain this value" test.
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct LoweredNode {
    /// The ROS package name, AS WRITTEN — `talker-pkg`, not `talker_pkg`.
    ///
    /// Turning it into an identifier is the pack's step, through the shared
    /// [`sanitize_pkg`]: a ROS package name is a NAME, and which identifier it
    /// becomes is a language question (Rust and C agree; a language with raw
    /// identifiers would not have to). Keeping the raw name here is also what
    /// lets the parity corpus carry `talker-pkg` and prove both producers
    /// reach the same sanitiser instead of each carrying the copy they had.
    pub pkg: String,
    /// Launch `<param>` initials, in declaration order.
    #[serde(default)]
    pub params: Vec<(String, String)>,
    /// Launch `<remap from= to=>` rules, in declaration order (first match
    /// wins at runtime, so the order is a fact and not a presentation choice).
    #[serde(default)]
    pub remaps: Vec<(String, String)>,
    #[serde(default)]
    pub qos_overrides: Vec<QosOverride>,
    /// `None` = the node keeps its own name (the self-bringup arm).
    #[serde(default)]
    pub identity: Option<NodeIdentity>,
}

/// The facts a language pack renders an entry from.
///
/// Deliberately small: this is the subset BOTH Rust producers cover, and
/// growing it means teaching both. `board` is the KEY the user wrote —
/// resolving it to a type path is the pack's job, through
/// `nros_orchestration_ir::board_path_for` (Rust) or a board call (C).
#[derive(Clone, Debug, Default, PartialEq, Eq, serde::Serialize, serde::Deserialize)]
pub struct LoweredEntry {
    /// The bringup package name — a provenance comment in every producer.
    pub bringup: String,
    /// The launch file this entry was resolved from.
    pub launch: String,
    /// The board KEY, as written. See [`crate::board_family`].
    pub board: String,
    pub nodes: Vec<LoweredNode>,
    /// Files the generated entry must be REBUILT when they change — the
    /// `include_bytes!` rebuild-correctness workaround both Rust producers
    /// use. Already filtered to paths that exist, because "is this on disk"
    /// is answered where the disk is and `include_bytes!` on a missing path is
    /// a hard compile error. A path is a fact; QUOTING it is the pack's job,
    /// and the two packs quote it differently (raw string vs `LitStr`).
    #[serde(default)]
    pub depfiles: Vec<String>,
}

impl LoweredEntry {
    /// The family this entry's board key names.
    pub fn family(&self) -> crate::BoardFamily {
        crate::board_family(&self.board)
    }
}

/// A package name as an identifier.
///
/// ROS package names are `snake_case` by convention but not by rule, and a
/// launch file may name one with a dash or a dot. Both Rust entry producers
/// turn that into a path segment (`::talker_pkg::register`), and both had
/// their own character-for-character identical copy of this loop —
/// `entry::sanitize_pkg` and `main_macro::pkg_to_crate_ident`. Neither had a
/// test that the two agreed.
///
/// It is a SPELLING, so it is called from the packs rather than applied in
/// lowering ([`LoweredNode::pkg`] stays the name the user wrote) — but it is a
/// spelling with one right answer, so it has one implementation.
///
/// Every character that is not ASCII alphanumeric or `_` becomes `_`. That is
/// the mapping both producers already had; it is not injective, and it does
/// not need to be — a workspace with both `a-b` and `a.b` is already ambiguous
/// to cargo.
pub fn sanitize_pkg(pkg: &str) -> String {
    let mut out = String::with_capacity(pkg.len());
    for c in pkg.chars() {
        if c.is_ascii_alphanumeric() || c == '_' {
            out.push(c);
        } else {
            out.push('_');
        }
    }
    out
}

impl LoweredNode {
    /// A node with nothing but a package — the bare case, which still gets
    /// all four resets.
    pub fn bare(pkg: &str) -> Self {
        LoweredNode {
            pkg: pkg.to_string(),
            ..Default::default()
        }
    }

    /// The package as a path segment, through the one shared [`sanitize_pkg`].
    pub fn ident(&self) -> String {
        sanitize_pkg(&self.pkg)
    }

    /// The identity as a `(name, namespace)` pair, or `None`.
    ///
    /// Both producers branch on exactly this, so the branch is one function
    /// rather than two `match`es that could disagree about what an identity
    /// with an empty namespace means.
    pub fn identity_pair(&self) -> Option<(&str, &str)> {
        self.identity
            .as_ref()
            .map(|i| (i.name.as_str(), i.namespace.as_str()))
    }
}

impl NodeIdentity {
    pub fn new(name: &str, namespace: &str) -> Self {
        NodeIdentity {
            name: name.to_string(),
            namespace: namespace.to_string(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec;

    #[test]
    fn sanitize_pkg_maps_every_non_ident_char() {
        assert_eq!(sanitize_pkg("talker-pkg"), "talker_pkg");
        assert_eq!(sanitize_pkg("a.b/c"), "a_b_c");
        assert_eq!(sanitize_pkg("plain_pkg"), "plain_pkg");
        assert_eq!(sanitize_pkg("n0de1"), "n0de1");
    }

    /// The reset discipline lives in the PRODUCERS, but the type must not make
    /// "absent" and "empty" different shapes — a node with no params carries
    /// an empty vec, never a missing field, so both producers reach the same
    /// branch.
    #[test]
    fn a_bare_node_carries_empty_lists_not_absent_ones() {
        let n = LoweredNode::bare("talker-pkg");
        assert_eq!(n.pkg, "talker-pkg", "the raw name is what is lowered");
        assert_eq!(n.ident(), "talker_pkg", "the spelling is derived");
        assert!(n.params.is_empty());
        assert!(n.remaps.is_empty());
        assert!(n.qos_overrides.is_empty());
        assert_eq!(n.identity_pair(), None);
    }

    #[test]
    fn identity_pair_carries_an_empty_namespace_through() {
        let n = LoweredNode {
            identity: Some(NodeIdentity::new("talker", "")),
            ..LoweredNode::bare("talker_pkg")
        };
        assert_eq!(n.identity_pair(), Some(("talker", "")));
    }

    #[test]
    fn the_board_key_resolves_to_its_family() {
        let e = LoweredEntry {
            board: "freertos-posix".into(),
            nodes: vec![LoweredNode::bare("talker_pkg")],
            ..Default::default()
        };
        assert_eq!(e.family(), crate::BoardFamily::Freertos);
    }
}
