//! phase-403 W6 -- the derived bound has to LEAVE codegen.
//!
//! Codegen is the right place to DERIVE a message type's serialized-size bound.
//! It is the wrong place for the bound to STOP. Until this module existed the
//! number was emitted only as a `#define` inside a generated header, so every
//! later stage that needed it invented a substitute: the arena multiplied
//! `MAX_CBS` by an action-client worst case, the zenoh payload classes were two
//! hand-set constants, and `NROS_MAX_LARGE_SUBSCRIBERS` was a number a human
//! produced by reading generated headers with their eyes. That last one is
//! measured, not hypothetical -- bringing the island up on mr-canhubk344 meant
//! copying `Control 2052` and `Odometry 1804` out of generated C++ headers into
//! a board `.conf`.
//!
//! So this module carries ONE data model -- [`BoundInventory`] -- and renders it
//! into the transports the later stages already speak:
//!
//! * [`BoundInventory::to_json`] -- the canonical artifact, written beside the
//!   generated code as `nros_message_bounds.json`. This is the format; the other
//!   two are projections of it.
//! * [`BoundInventory::to_cmake`] -- a `.cmake` script the CMake/Kconfig lane
//!   `include()`s. Same shape as `nros codegen resolve-deps --output-cmake`,
//!   which is the existing mechanism for handing CMake a fact codegen computed.
//! * [`BoundInventory::to_build_rs`] -- the `build.rs` a generated Rust message
//!   crate ships, whose `cargo:` metadata reaches a dependent's build script as
//!   `DEP_<LINKS>_BOUNDS_JSON`. Same `links` channel `DEP_NROS_NODE_RX_BUF_SIZE`
//!   already uses.
//!
//! # A type NEVER appears with a fabricated number
//!
//! [`BoundState`] has three states, not two, for the reason
//! [`crate::schema_value::TypeBound`] does: "we looked and no bound exists" and
//! "we could not look" license completely different actions. Neither carries a
//! size, and neither emits a `_TX`/`_RX` key on any transport -- a consumer that
//! reads a number gets a number that was derived, or it reads nothing at all.
//!
//! # This is the REAL bound, not the C++ pack's estimate
//!
//! Every number here comes from [`crate::schema_value::bound_message`], i.e.
//! from `nros_serdes::size::max_serialized_size` -- THE size rule, the same
//! function the runtime's `M::MAX_SERIALIZED_SIZE_XCDR*` uses.
//!
//! It is deliberately NOT [`crate::types::compute_serialized_size_max`], which
//! the C++ pack still uses for its in-header `SERIALIZED_SIZE_MAX`. That
//! function ESTIMATES: it charges a flat 512 bytes per nested message and a flat
//! default capacity per string, and it always returns a value, so it can never
//! report "unbounded". A flat 512 for a nested type whose own bound exceeds 512
//! is an UNDER-estimate, which is the direction that matters. Exporting it as
//! authoritative build metadata would make that guess load-bearing across the
//! whole build, which is exactly what this wave exists to stop.

use crate::schema_value::TypeBound;

/// Bumped when the shape of the emitted inventory changes incompatibly.
/// A consumer that does not recognise the version must refuse, never guess.
pub const INVENTORY_SCHEMA_VERSION: u32 = 1;

/// Canonical artifact name, written into the generated package's output dir.
pub const INVENTORY_JSON_NAME: &str = "nros_message_bounds.json";

/// CMake projection of [`INVENTORY_JSON_NAME`], beside it.
pub const INVENTORY_CMAKE_NAME: &str = "nros_message_bounds.cmake";

/// The small/large payload-class split, in bytes: a type whose `rx` bound is
/// STRICTLY GREATER than this is served from the zenoh backend's `large` class.
///
/// POLICY, not a derived fact -- it is `ZPICO_SUBSCRIBER_SIZE_THRESHOLD`'s
/// shipped default, and a caller may pass another. It lives here because two
/// derivations now classify against it and a second literal is how they come to
/// disagree: `NROS_MESSAGE_BOUNDS_DEFAULT_SMALL_CEILING` in
/// `cmake/NanoRosMessageBounds.cmake` for the CMake lane, and
/// [`crate::bounds`]'s Rust consumer (`nros_cli_core::leaf_payload_classes`)
/// for the cargo-leaf one. The cmake spelling cannot read a Rust const, so the
/// two are held together by `check-payload-class-ceiling` rather than by
/// sharing storage.
pub const DEFAULT_SMALL_CLASS_CEILING: usize = 2048;

/// Read the rows back out of an [`INVENTORY_JSON_NAME`] document.
///
/// The inverse of [`BoundInventory::to_json`], and it lives beside it so the
/// schema has ONE reader and ONE writer in one file. A private parser in a
/// consumer is how a producer's field rename becomes a silent miss.
///
/// Refuses (`Err`) rather than skipping on:
///
/// * an unrecognised `schema_version` -- the rule the module header states for
///   every consumer of this artifact;
/// * a row that says `bounded` and carries no numeric `rx_max_serialized_size`.
///   A missing size read as zero would classify a large type small, which is
///   the silent under-sizing the whole inventory exists to prevent.
///
/// A row whose `state` is `unbounded`/`unresolved` is returned as such: "this
/// type has no bound" is an ANSWER a sizing consumer must refuse on, not a row
/// to drop.
pub fn bounds_from_json(doc: &str) -> Result<Vec<TypeBoundEntry>, String> {
    let v: serde_json::Value =
        serde_json::from_str(doc).map_err(|e| format!("not the bound inventory's JSON: {e}"))?;
    let schema = v.get("schema_version").and_then(|s| s.as_u64());
    if schema != Some(INVENTORY_SCHEMA_VERSION as u64) {
        return Err(format!(
            "bound inventory states schema_version {schema:?}; this reader understands \
             {INVENTORY_SCHEMA_VERSION}. Regenerate the `generated/` tree with this \
             checkout's codegen (`nros sync`)."
        ));
    }
    let types = v
        .get("types")
        .and_then(|t| t.as_array())
        .ok_or_else(|| "bound inventory has no `types` array".to_string())?;
    let mut out = Vec::with_capacity(types.len());
    for row in types {
        let type_name = row
            .get("type_name")
            .and_then(|s| s.as_str())
            .ok_or_else(|| "a bound-inventory row has no `type_name`".to_string())?
            .to_string();
        let state = row.get("state").and_then(|s| s.as_str()).unwrap_or("");
        let reason = || {
            row.get("reason")
                .and_then(|s| s.as_str())
                .unwrap_or("no reason recorded")
                .to_string()
        };
        let bound = match state {
            "bounded" => {
                let num = |k: &str| row.get(k).and_then(|n| n.as_u64()).map(|n| n as usize);
                let (Some(tx), Some(rx)) =
                    (num("tx_max_serialized_size"), num("rx_max_serialized_size"))
                else {
                    return Err(format!(
                        "{type_name} is `bounded` in the inventory and carries no numeric \
                         rx/tx size. The artifact is malformed; regenerate it with `nros sync`."
                    ));
                };
                BoundState::Bounded { tx, rx }
            }
            "unbounded" => BoundState::Unbounded { reason: reason() },
            "unresolved" => BoundState::Unresolved { reason: reason() },
            other => {
                return Err(format!(
                    "{type_name} carries an unknown bound state `{other}`"
                ));
            }
        };
        out.push(TypeBoundEntry {
            type_name,
            bound,
            chains: Vec::new(),
            budget: None,
        });
    }
    Ok(out)
}

/// What codegen concluded about one type's serialized-size bound.
///
/// The two encodings are kept apart because they genuinely differ: XCDR2 adds a
/// 4-byte DHEADER and aligns 8-byte primitives to 4 instead of 8. `tx` is the
/// XCDR1 number because this stack WRITES XCDR1; `rx` is the larger of the two,
/// because a receive buffer must hold either, rounded up by
/// [`transport_framed`], because what a transport DELIVERS can exceed the
/// message by its own framing alignment.
///
/// So `rx >= tx` always, and `rx` is not "the message under some encoding" — it
/// is "what a buffer has to be able to accept". Anything sizing a receive buffer
/// wants `rx`; anything reporting how big the message is wants `tx`.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BoundState {
    /// A bound exists. Bytes, encapsulation header included.
    Bounded { tx: usize, rx: usize },
    /// No bound EXISTS. Carries EVERY member that costs it, as `nros_serdes`
    /// names them. Fix by bounding the field (`string<=64`) or capping it
    /// `inline` in `nros-codegen.toml`.
    Unbounded { reason: String },
    /// The bound was not COMPUTED, because a nested type was not reachable
    /// through the resolver. A search-path problem, not a property of the
    /// message. Nothing may be sized from it.
    Unresolved { reason: String },
}

/// Round a serialized size up to the 4-byte multiple a receive buffer must be
/// able to hold.
///
/// The message is `n` bytes. What a transport DELIVERS can be up to three bytes
/// more, and a receive buffer sized to `n` exactly will refuse those bytes
/// rather than truncate them — correctly, and with the message lost.
///
/// Measured, not assumed. A 25-byte `std_msgs/String` published by ROS 2 Humble
/// over stock `rmw_cyclonedds` arrives at the nano-ros Cyclone backend as:
///
/// ```text
/// WIRE=len:28 hdr:00010000 cdr:25
/// ```
///
/// The three extra bytes are the RTPS submessage's own 4-byte alignment, added
/// by the SENDER; the encapsulation options read `0000` rather than `0003`, so
/// the pad is not even discoverable from the header. The backend adds nothing —
/// it hands back exactly what the wire gave it (issues 0969 / 0970) — which is
/// precisely why the pad now reaches the buffer sizing instead of being absorbed
/// by a re-encode.
///
/// This is deliberately NOT folded into the type's own bound. `MAX_SERIALIZED_SIZE`
/// answers "how big is this message", and issue 0964 exists because that number
/// had been fudged before; the answer stays exact. Framing is a property of the
/// transport, so it is applied where a RECEIVE BUFFER is sized and nowhere else.
/// TX keeps the exact figure: we write what we serialise.
///
/// Four is not a Cyclone number. RTPS aligns submessages to 4, and CDR itself
/// aligns to at most 8 within a 4-aligned encapsulation, so 4 is the alignment
/// any DDS peer can impose. A transport that framed more coarsely would need its
/// own allowance, which is why this is a named function and not a `+ 3`.
///
/// The Rust runtime applies the same rounding in
/// `nros_node::rmw_type_registry::subscription_rx_bytes`; both are cited from
/// each other so the two cannot drift into disagreeing.
pub const fn transport_framed(bound: usize) -> usize {
    bound.next_multiple_of(4)
}

impl BoundState {
    /// The one place the two per-encoding answers become a TX/RX pair.
    ///
    /// The C header emitter (`generator::msg`) calls this too, so the constants
    /// in a generated header and the numbers in the inventory cannot drift into
    /// disagreeing about which encoding feeds which direction.
    ///
    /// RX takes the larger of the two encodings AND rounds it up to a 4-byte
    /// multiple — see [`transport_framed`] for why that rounding is not padding
    /// anyone chose. TX stays exact: we write what we serialise, and the framing
    /// underneath is the transport's business.
    pub fn classify(xcdr1: &TypeBound, xcdr2: &TypeBound) -> Self {
        match (xcdr1, xcdr2) {
            (TypeBound::Bounded(a), TypeBound::Bounded(b)) => BoundState::Bounded {
                tx: *a,
                rx: transport_framed(*a.max(b)),
            },
            // Unbounded wins over Unresolved when both appear: "there is no
            // bound" is a fact about the message, and it stays true however the
            // search path is fixed.
            (TypeBound::Unbounded(w), _) | (_, TypeBound::Unbounded(w)) => BoundState::Unbounded {
                reason: Self::unbounded_reason(w),
            },
            (TypeBound::Unresolved(t), _) | (_, TypeBound::Unresolved(t)) => {
                BoundState::Unresolved {
                    reason: format!("nested type `{t}` could not be resolved"),
                }
            }
        }
    }

    /// The one spelling of "why this type has no bound", shared by the exported
    /// inventory and the generated C header.
    ///
    /// Singular for one member so the common case reads as prose, plural with a
    /// comma list otherwise. The list is ordered by declaration, which is the
    /// order the user reads their `.msg` in, not sorted — a sorted list of
    /// members is harder to walk against the file you are editing.
    pub fn unbounded_reason(members: &[String]) -> String {
        match members {
            [one] => format!("unbounded member: {one}"),
            many => format!("unbounded members: {}", many.join(", ")),
        }
    }

    /// The `state` word used on every transport.
    pub fn tag(&self) -> &'static str {
        match self {
            BoundState::Bounded { .. } => "bounded",
            BoundState::Unbounded { .. } => "unbounded",
            BoundState::Unresolved { .. } => "unresolved",
        }
    }
}

/// One type's row in the inventory.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TypeBoundEntry {
    /// ROS fully-qualified name, `pkg/msg/Name` -- the spelling
    /// `rmw_subscription_options_t`'s `type_name` and the vtable's
    /// `required_rx_bytes` already use, so a consumer can key on it without a
    /// second naming convention.
    pub type_name: String,
    pub bound: BoundState,
    /// phase-403 W7b (issue 0961) — the NESTED repeated chains this type's
    /// total is a product over, worst first.
    ///
    /// Exported so the number is legible: a reader sees
    /// `markers.controls.markers.points = 8 x 8 x 8 x 64` and knows which level
    /// to cap, rather than one opaque total. Empty for the overwhelming
    /// majority of types, which nest nothing.
    ///
    /// Derived from the SAME schema the bound is, in one walk
    /// (`schema_value::sequence_chains`) — not a second derivation, which is the
    /// rule this module exists to hold.
    pub chains: Vec<crate::schema_value::SequenceChain>,
    /// The `[types.*] max_serialized` budget the config states, if any.
    ///
    /// Carried so a consumer can see the assertion beside the derived total. It
    /// is NEVER the exported size: a budget under the derived bound is a build
    /// error, and a budget over it changes nothing.
    pub budget: Option<usize>,
}

/// phase-403 W7b (issue 0961) — a derived bound that exceeds the budget its
/// type declared.
///
/// Carries both numbers and the chain, because "too big" alone does not tell a
/// user what to do. The remedy is always to cap one LEVEL of a nesting chain,
/// and which level is a fact only the chain can supply.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct BudgetExceeded {
    pub type_name: String,
    /// XCDR1, what this stack writes.
    pub derived_tx: usize,
    /// The larger of the two encodings — what a receive buffer must hold, and
    /// therefore the number checked against the budget.
    pub derived_rx: usize,
    pub budget: usize,
    pub chains: Vec<crate::schema_value::SequenceChain>,
}

impl std::fmt::Display for BudgetExceeded {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}: derived serialized-size bound {} bytes (RX; TX {}) exceeds the \
             `max_serialized = {}` budget stated for this type in nros-codegen.toml",
            self.type_name, self.derived_rx, self.derived_tx, self.budget
        )?;
        if self.chains.is_empty() {
            // No nesting: the total is a sum of this type's own members, so the
            // remedy is a cap on one of them or a larger budget. Say that,
            // rather than printing an empty chain list and leaving the user to
            // infer it.
            return write!(
                f,
                ".\n  This type nests no repeated members, so the total is a SUM of \
                 its own fields -- cap one of them, or raise the budget."
            );
        }
        writeln!(
            f,
            ".\n  The total is a PRODUCT: `nros_serdes::size` walks a bounded \
             sequence and a fixed array element by element, so nesting \
             MULTIPLIES (issue 0961). Cap ONE level of the worst chain and the \
             whole product divides:"
        )?;
        for c in &self.chains {
            writeln!(f, "    {c} elements")?;
        }
        write!(
            f,
            "  A factor that is a FIXED ARRAY comes from the `.msg` and no cap \
             can change it; a bounded-sequence factor is either a `.msg` bound \
             or a `cap` in nros-codegen.toml."
        )
    }
}

impl std::error::Error for BudgetExceeded {}

/// Check one type's derived bound against the budget its config states.
///
/// `Ok(())` when there is no budget, when the bound does not exist (an
/// unbounded or unresolved type has no total to check, and already fails
/// through its own louder channel), or when the derived total FITS.
///
/// The derived number is never replaced by the budget in any of those cases --
/// see [`rosidl_lower::config::CapacityResolver::max_serialized`].
pub fn check_budget(
    type_name: &str,
    bound: &BoundState,
    chains: &[crate::schema_value::SequenceChain],
    budget: Option<usize>,
) -> Result<(), BudgetExceeded> {
    let (Some(budget), BoundState::Bounded { tx, rx }) = (budget, bound) else {
        return Ok(());
    };
    if *rx <= budget {
        return Ok(());
    }
    Err(BudgetExceeded {
        type_name: type_name.to_string(),
        derived_tx: *tx,
        derived_rx: *rx,
        budget,
        chains: chains.to_vec(),
    })
}

/// The `(package, message)` half of a `pkg/Msg` or `pkg/msg/Msg` type name, as
/// the config keys it.
///
/// The inventory records `pkg/msg/Name` and `[types.*]` is keyed `pkg/Name`, so
/// this is where the two spellings meet. One place, because a second
/// normalisation is how a budget silently stops matching its type.
fn config_key_of(type_name: &str) -> (String, String) {
    let package = type_name.split('/').next().unwrap_or("").to_string();
    let message = type_name
        .rsplit('/')
        .next()
        .unwrap_or(type_name)
        .to_string();
    (package, message)
}

/// Every generated message type of one interface package, with its bound.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct BoundInventory {
    pub package: String,
    entries: Vec<TypeBoundEntry>,
}

impl BoundInventory {
    pub fn new(package: impl Into<String>) -> Self {
        Self {
            package: package.into(),
            entries: Vec::new(),
        }
    }

    /// Record one type. Later records for the same name replace earlier ones so
    /// a driver that regenerates a type in one pass cannot emit it twice.
    pub fn insert(&mut self, type_name: impl Into<String>, bound: BoundState) {
        self.insert_full(type_name, bound, Vec::new(), None)
    }

    /// [`Self::insert`] plus the nesting chains and the declared budget
    /// (phase-403 W7b).
    pub fn insert_full(
        &mut self,
        type_name: impl Into<String>,
        bound: BoundState,
        chains: Vec<crate::schema_value::SequenceChain>,
        budget: Option<usize>,
    ) {
        let type_name = type_name.into();
        match self.entries.iter_mut().find(|e| e.type_name == type_name) {
            Some(existing) => {
                existing.bound = bound;
                existing.chains = chains;
                existing.budget = budget;
            }
            None => self.entries.push(TypeBoundEntry {
                type_name,
                bound,
                chains,
                budget,
            }),
        }
    }

    /// phase-403 W7b (issue 0961) — every recorded type whose derived bound
    /// exceeds the budget it declared.
    ///
    /// A `Vec` and not the first, for the reason `TypeBound::Unbounded` names
    /// every offending member: a driver reports the whole set in one build
    /// rather than making the user re-run codegen per type.
    ///
    /// Empty whenever no type states a budget, which is the default and the
    /// entire stock tree.
    pub fn budget_violations(&self) -> Vec<BudgetExceeded> {
        self.entries()
            .into_iter()
            .filter_map(|e| check_budget(&e.type_name, &e.bound, &e.chains, e.budget).err())
            .collect()
    }

    /// [`Self::budget_violations`] as one error, or `Ok(())`.
    ///
    /// The spelling a driver calls once after recording a package, so the check
    /// happens in ONE place per driver instead of once per message.
    pub fn check_budgets(&self) -> Result<(), String> {
        let v = self.budget_violations();
        if v.is_empty() {
            return Ok(());
        }
        Err(v
            .iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .join("\n"))
    }

    /// Derive and record the bound for one parsed message.
    ///
    /// `lookup` resolves nested types. A lookup that cannot reach a nested type
    /// yields `Unresolved`, never a number.
    ///
    /// `caps` is the SAME `nros-codegen.toml` resolver the emitters were handed.
    /// It is a required argument rather than an optional refinement because the
    /// inventory and the generated header must agree: a field capped `inline`
    /// bounds the type in both, or the exported number and the `#define` say
    /// different things about one type and nothing in the build compares them.
    pub fn record_message(
        &mut self,
        type_name: &str,
        message: &rosidl_parser::Message,
        caps: &crate::CapacityResolver,
        lookup: &crate::schema_value::MsgLookup<'_>,
    ) {
        use crate::schema_value::{bound_message, chains_for};
        use nros_serdes::cdr::EncodingVersion;
        let x1 = bound_message(type_name, message, EncodingVersion::Xcdr1, caps, lookup);
        let x2 = bound_message(type_name, message, EncodingVersion::Xcdr2, caps, lookup);
        // phase-403 W7b (issue 0961) — the chains and the budget travel with the
        // bound. The chain is a property of the SHAPE, so it is encoding-
        // independent and computed once; the budget is read with the same
        // `pkg/Msg` key the capacity entries beside it use.
        let (pkg, msg) = config_key_of(type_name);
        self.insert_full(
            type_name,
            BoundState::classify(&x1, &x2),
            chains_for(type_name, message, caps, lookup),
            caps.max_serialized(&pkg, &msg),
        );
    }

    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Entries in emission order: sorted by type name, so the artifact is
    /// byte-stable across runs and `write_if_changed` keeps mtimes still.
    pub fn entries(&self) -> Vec<&TypeBoundEntry> {
        let mut v: Vec<&TypeBoundEntry> = self.entries.iter().collect();
        v.sort_by(|a, b| a.type_name.cmp(&b.type_name));
        v
    }

    /// The canonical artifact. Pretty-printed for the on-disk file; the
    /// `build.rs` transport uses [`Self::to_json_compact`] because a `cargo:`
    /// metadata value cannot contain a newline.
    pub fn to_json(&self) -> String {
        format!("{}\n", self.json_value_string(true))
    }

    /// One line, no newline. Same document as [`Self::to_json`].
    pub fn to_json_compact(&self) -> String {
        self.json_value_string(false)
    }

    fn json_value_string(&self, pretty: bool) -> String {
        let types: Vec<serde_json::Value> = self
            .entries()
            .into_iter()
            .map(|e| {
                let mut m = serde_json::Map::new();
                m.insert("type_name".into(), e.type_name.clone().into());
                m.insert("state".into(), e.bound.tag().into());
                match &e.bound {
                    BoundState::Bounded { tx, rx } => {
                        m.insert("tx_max_serialized_size".into(), (*tx).into());
                        m.insert("rx_max_serialized_size".into(), (*rx).into());
                    }
                    BoundState::Unbounded { reason } | BoundState::Unresolved { reason } => {
                        m.insert("reason".into(), reason.clone().into());
                    }
                }
                // phase-403 W7b (issue 0961) — the factor chain, so a total with
                // five factors in it is legible instead of opaque. Omitted
                // entirely when the type nests nothing, which is almost all of
                // them: an empty key on every row is noise, and its absence
                // already means "no nesting".
                if !e.chains.is_empty() {
                    m.insert(
                        "sequence_chains".into(),
                        e.chains
                            .iter()
                            .map(|c| {
                                serde_json::json!({
                                    "path": c.path,
                                    "factors": c.factors,
                                    "elements": c.product(),
                                })
                            })
                            .collect::<Vec<_>>()
                            .into(),
                    );
                }
                // The user's assertion, beside the derived number. Never the
                // exported size -- see `CapacityResolver::max_serialized`.
                if let Some(b) = e.budget {
                    m.insert("max_serialized_budget".into(), b.into());
                }
                serde_json::Value::Object(m)
            })
            .collect();
        let doc = serde_json::json!({
            "schema_version": INVENTORY_SCHEMA_VERSION,
            "producer": "nros-codegen",
            "package": self.package,
            // Named so a reader cannot mistake these for the C++ pack's
            // `SERIALIZED_SIZE_MAX`, which is an estimate (see module docs).
            "derivation": "nros_serdes::size::max_serialized_size",
            "types": types,
        });
        if pretty {
            serde_json::to_string_pretty(&doc).unwrap_or_default()
        } else {
            serde_json::to_string(&doc).unwrap_or_default()
        }
    }

    /// The CMake/Kconfig projection.
    ///
    /// Appends to global lists so several packages' fragments compose into one
    /// image-wide inventory, which is the shape W4 needs to turn the zenoh
    /// payload classes into "the distinct sizes this image's types actually
    /// need". `include()` it at the scope you want the variables in -- CMake
    /// function scope does not leak.
    ///
    /// An unbounded or unresolved type gets a `_STATE` and a `_REASON` and NO
    /// `_TX`/`_RX`, so a consumer that reads a size either reads a derived
    /// number or reads nothing.
    pub fn to_cmake(&self) -> String {
        let mut s = String::new();
        s.push_str("# GENERATED by nros codegen (phase-403 W6). Do not edit.\n");
        s.push_str(&format!(
            "# Derived with nros_serdes::size::max_serialized_size -- the same rule the\n\
             # runtime's M::MAX_SERIALIZED_SIZE_XCDR* uses. NOT the C++ pack's estimate.\n\
             set(NROS_MESSAGE_BOUNDS_SCHEMA_VERSION {INVENTORY_SCHEMA_VERSION})\n"
        ));
        s.push_str(&format!(
            "list(APPEND NROS_MESSAGE_BOUND_PACKAGES \"{}\")\n",
            self.package
        ));
        for e in self.entries() {
            let key = cmake_key(&e.type_name);
            s.push_str(&format!(
                "list(APPEND NROS_MESSAGE_BOUND_TYPES \"{}\")\n",
                e.type_name
            ));
            s.push_str(&format!(
                "set(NROS_MESSAGE_BOUND_{key}_STATE \"{}\")\n",
                e.bound.tag()
            ));
            match &e.bound {
                BoundState::Bounded { tx, rx } => {
                    s.push_str(&format!("set(NROS_MESSAGE_BOUND_{key}_TX {tx})\n"));
                    s.push_str(&format!("set(NROS_MESSAGE_BOUND_{key}_RX {rx})\n"));
                }
                BoundState::Unbounded { reason } | BoundState::Unresolved { reason } => {
                    s.push_str(&format!(
                        "set(NROS_MESSAGE_BOUND_{key}_REASON \"{}\")\n",
                        cmake_escape(reason)
                    ));
                }
            }
            // phase-403 W7b (issue 0961) — two PARALLEL cmake lists rather than
            // one packed string: `;` is cmake's list separator, so
            // `foreach(p ${..._CHAIN_PATHS})` iterates them natively and a
            // consumer never parses a delimiter by hand. Indices line up.
            if !e.chains.is_empty() {
                s.push_str(&format!(
                    "set(NROS_MESSAGE_BOUND_{key}_CHAIN_PATHS \"{}\")\n",
                    e.chains
                        .iter()
                        .map(|c| cmake_escape(&c.path))
                        .collect::<Vec<_>>()
                        .join(";")
                ));
                s.push_str(&format!(
                    "set(NROS_MESSAGE_BOUND_{key}_CHAIN_FACTORS \"{}\")\n",
                    e.chains
                        .iter()
                        .map(|c| c.factors_display())
                        .collect::<Vec<_>>()
                        .join(";")
                ));
                s.push_str(&format!(
                    "set(NROS_MESSAGE_BOUND_{key}_CHAIN_ELEMENTS \"{}\")\n",
                    e.chains
                        .iter()
                        .map(|c| c.product().to_string())
                        .collect::<Vec<_>>()
                        .join(";")
                ));
            }
            if let Some(b) = e.budget {
                s.push_str(&format!(
                    "set(NROS_MESSAGE_BOUND_{key}_MAX_SERIALIZED_BUDGET {b})\n"
                ));
            }
        }
        s.push_str("list(REMOVE_DUPLICATES NROS_MESSAGE_BOUND_PACKAGES)\n");
        s.push_str("list(REMOVE_DUPLICATES NROS_MESSAGE_BOUND_TYPES)\n");
        s
    }

    /// The `build.rs` a generated Rust message crate ships.
    ///
    /// The crate declares `links = "<links_key>"`, so cargo hands every
    /// `cargo:KEY=VALUE` line below to the build script of each crate that
    /// depends on it, as `DEP_<LINKS_KEY_UPPERCASE>_<KEY_UPPERCASE>`. That is
    /// the channel `nros-c`'s build script already reads `DEP_NROS_NODE_MAX_CBS`
    /// on; this wave adds a producer to it rather than a second mechanism.
    ///
    /// The value is the same JSON document as the on-disk artifact, compacted,
    /// because a `cargo:` metadata value may not contain a newline.
    pub fn to_build_rs(&self) -> String {
        format!(
            r#"// GENERATED by nros codegen (phase-403 W6). Do not edit; regenerate with
// `nros sync`.
//
// This crate carries `links` purely so these lines reach a dependent's build
// script as `DEP_<LINKS>_BOUNDS_*`. It links no native library. The channel is
// the one `nros-c` already reads `DEP_NROS_NODE_RX_BUF_SIZE` on.
//
// A type whose bound does not exist, or could not be computed, appears with a
// `state` of "unbounded"/"unresolved" and NO size. Never a substituted default:
// phase-380's rule is that `None` means "no bound EXISTS", never "unknown", and
// a receive buffer sized from a fallback is one that silently mismatches the
// wire.
fn main() {{
    println!("cargo:rerun-if-changed=build.rs");
    println!("cargo:bounds_schema={schema}");
    // The payload is a Rust STRING LITERAL, not a format string: the document
    // is full of `"` and `{{`/`}}`, and putting it in the format position emits
    // a file that does not parse.
    //
    // BOUND TO A NAME first, rather than passed as an argument. Identical
    // output either way, and `clippy::print_literal` fires on the argument
    // form -- it suggests inlining the literal, which for this document means
    // doubling every brace in it. A generated file that warns is a file every
    // consumer of a message crate has to look at, so the shape that does not
    // warn is the one to emit; an interpolated NAME is not a literal.
    let bounds_json = "{json}";
    println!("cargo:bounds_json={{bounds_json}}");
}}
"#,
            schema = INVENTORY_SCHEMA_VERSION,
            json = rust_string_literal_body(&self.to_json_compact()),
        )
    }

    /// The `links` key for a generated crate of `package`.
    ///
    /// Cargo requires `links` to be unique across the dependency graph; a
    /// generated crate is named after its ament package, which already is.
    pub fn links_key(package: &str) -> String {
        format!("nros_msgs_{}", package.replace(['-', '.', '/'], "_"))
    }
}

/// A CMake variable name fragment for a ROS type name.
fn cmake_key(type_name: &str) -> String {
    type_name
        .chars()
        .map(|c| if c.is_ascii_alphanumeric() { c } else { '_' })
        .collect()
}

/// CMake `set(... "...")` is quote- and backslash-sensitive; a reason is prose
/// from `nros_serdes` and must not be able to end the string early.
fn cmake_escape(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

/// The body of a Rust `"..."` literal holding `s`.
///
/// The inventory is JSON, so it is ALL quotes; emitting it raw produced a
/// `build.rs` that did not parse. Not a raw string (`r#"..."#`) either -- a
/// reason is arbitrary prose from `nros_serdes` and could in principle carry
/// the closing delimiter.
fn rust_string_literal_body(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 16);
    for c in s.chars() {
        match c {
            '\\' => out.push_str("\\\\"),
            '"' => out.push_str("\\\""),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            _ => out.push(c),
        }
    }
    out
}

#[cfg(test)]
mod json_reader_tests {
    use super::*;

    fn doc(types: &str) -> String {
        format!(
            r#"{{"schema_version":{INVENTORY_SCHEMA_VERSION},"producer":"nros-codegen",
               "package":"p","derivation":"d","types":[{types}]}}"#
        )
    }

    /// The reader is the writer's inverse — checked against a document the
    /// writer produced, not against one hand-written to match the reader.
    #[test]
    fn a_written_inventory_reads_back_with_the_same_bounds() {
        let mut inv = BoundInventory::new("test_msgs");
        inv.insert("test_msgs/msg/Small", BoundState::Bounded { tx: 5, rx: 12 });
        inv.insert(
            "test_msgs/msg/Open",
            BoundState::Unbounded {
                reason: "unbounded member: data (string)".into(),
            },
        );
        let back = bounds_from_json(&inv.to_json()).expect("reads back");
        assert_eq!(back.len(), 2);
        let small = back
            .iter()
            .find(|e| e.type_name.ends_with("Small"))
            .unwrap();
        assert_eq!(small.bound, BoundState::Bounded { tx: 5, rx: 12 });
        let open = back.iter().find(|e| e.type_name.ends_with("Open")).unwrap();
        assert_eq!(open.bound.tag(), "unbounded");
    }

    /// The compact transport is the same document, so it reads too.
    #[test]
    fn the_compact_build_rs_transport_reads_the_same() {
        let mut inv = BoundInventory::new("t");
        inv.insert("t/msg/A", BoundState::Bounded { tx: 1, rx: 4 });
        assert_eq!(
            bounds_from_json(&inv.to_json_compact()).unwrap(),
            bounds_from_json(&inv.to_json()).unwrap()
        );
    }

    /// A version this reader does not understand REFUSES rather than guessing —
    /// the rule the module header states for every consumer of this artifact.
    #[test]
    fn an_unrecognised_schema_version_refuses() {
        let text = doc("").replace(
            &format!("\"schema_version\":{INVENTORY_SCHEMA_VERSION}"),
            "\"schema_version\":99",
        );
        let err = bounds_from_json(&text).expect_err("must refuse");
        assert!(err.contains("99"), "{err}");
    }

    /// `bounded` with no size would classify a large type SMALL if read as
    /// zero. It refuses, and names the type.
    #[test]
    fn a_bounded_row_with_no_size_refuses_and_names_the_type() {
        let err = bounds_from_json(&doc(r#"{"type_name":"t/msg/Broken","state":"bounded"}"#))
            .expect_err("must refuse");
        assert!(err.contains("t/msg/Broken"), "{err}");
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CapacityResolver;
    use rosidl_parser::{Message, parse_message};

    fn no_lookup(_: &str) -> Option<Message> {
        None
    }

    fn inv() -> BoundInventory {
        let mut i = BoundInventory::new("test_msgs");
        let flat = parse_message("int64 b\n").unwrap();
        i.record_message(
            "test_msgs/msg/Flat",
            &flat,
            &CapacityResolver::empty(),
            &no_lookup,
        );
        let open = parse_message("string s\n").unwrap();
        i.record_message(
            "test_msgs/msg/Open",
            &open,
            &CapacityResolver::empty(),
            &no_lookup,
        );
        let nested = parse_message("other_pkg/Thing t\n").unwrap();
        i.record_message(
            "test_msgs/msg/Nested",
            &nested,
            &CapacityResolver::empty(),
            &no_lookup,
        );
        i
    }

    /// The numbers are the ones `nros_serdes` computes, and the two encodings
    /// really do differ for this type (12 vs 16), which is why TX and RX are
    /// separate fields rather than one constant.
    #[test]
    fn a_bounded_type_carries_the_derived_tx_and_rx() {
        let i = inv();
        let e = i
            .entries()
            .into_iter()
            .find(|e| e.type_name == "test_msgs/msg/Flat")
            .unwrap()
            .clone();
        assert_eq!(e.bound, BoundState::Bounded { tx: 12, rx: 16 });
    }

    /// RX carries the transport's framing allowance and TX does not.
    ///
    /// `classify` is a pure function of the two encoding bounds, so this pins
    /// the rule directly rather than hunting a corpus type whose bound happens
    /// to be misaligned. The numbers are chosen so every case is visible: 13
    /// rounds to 16, 16 stays 16, and TX is 13 either way — we write what we
    /// serialise.
    #[test]
    fn rx_allows_for_transport_framing_and_tx_does_not() {
        assert_eq!(
            BoundState::classify(&TypeBound::Bounded(13), &TypeBound::Bounded(13)),
            BoundState::Bounded { tx: 13, rx: 16 },
            "a 13-byte message can arrive as 16 — see `transport_framed`"
        );
        assert_eq!(
            BoundState::classify(&TypeBound::Bounded(16), &TypeBound::Bounded(16)),
            BoundState::Bounded { tx: 16, rx: 16 },
            "an already-aligned bound must not be inflated"
        );
        // The larger encoding still wins, and THEN gets the allowance.
        assert_eq!(
            BoundState::classify(&TypeBound::Bounded(13), &TypeBound::Bounded(17)),
            BoundState::Bounded { tx: 13, rx: 20 },
            "RX is max(xcdr1, xcdr2) framed, not max(framed, framed) of one"
        );
    }

    /// A 25-byte message is the one this was measured on: ROS 2 Humble over
    /// stock `rmw_cyclonedds` delivers it as 28 bytes (`WIRE=len:28 cdr:25`),
    /// and a buffer sized to 25 refuses it.
    #[test]
    fn the_measured_case_is_covered() {
        assert_eq!(transport_framed(25), 28);
        assert_eq!(transport_framed(28), 28);
        assert_eq!(transport_framed(0), 0);
        assert_eq!(transport_framed(1), 4);
    }

    /// The whole point of the wave: an unbounded type is a MARKER plus the
    /// member that costs the bound, never a number a later stage could read as
    /// authoritative.
    #[test]
    fn an_unbounded_type_carries_a_reason_and_no_number() {
        let i = inv();
        let e = i
            .entries()
            .into_iter()
            .find(|e| e.type_name == "test_msgs/msg/Open")
            .unwrap()
            .clone();
        match &e.bound {
            BoundState::Unbounded { reason } => assert!(
                reason.contains('s'),
                "the reason must name the member: {reason}"
            ),
            other => panic!("expected Unbounded, got {other:?}"),
        }
        // No transport may carry a size for it.
        assert!(!i.to_cmake().contains("Open_TX"));
        assert!(!i.to_cmake().contains("Open_RX"));
        assert!(!i.to_json().contains("\"test_msgs/msg/Open\",\n    \"tx"));
    }

    /// "We could not look" is not "there is no bound", and the inventory keeps
    /// them apart because they license different fixes.
    #[test]
    fn an_unreachable_nested_type_is_unresolved_not_unbounded() {
        let i = inv();
        let e = i
            .entries()
            .into_iter()
            .find(|e| e.type_name == "test_msgs/msg/Nested")
            .unwrap()
            .clone();
        assert_eq!(e.bound.tag(), "unresolved");
        assert!(!i.to_cmake().contains("Nested_TX"));
    }

    #[test]
    fn the_cmake_projection_sets_a_size_only_for_a_bounded_type() {
        let c = inv().to_cmake();
        assert!(c.contains("set(NROS_MESSAGE_BOUND_test_msgs_msg_Flat_TX 12)"));
        assert!(c.contains("set(NROS_MESSAGE_BOUND_test_msgs_msg_Flat_RX 16)"));
        assert!(c.contains("set(NROS_MESSAGE_BOUND_test_msgs_msg_Open_STATE \"unbounded\")"));
        assert!(c.contains("list(APPEND NROS_MESSAGE_BOUND_PACKAGES \"test_msgs\")"));
        // Composable: nothing here assigns a list, everything appends.
        assert!(!c.contains("set(NROS_MESSAGE_BOUND_TYPES"));
    }

    /// The payload is EMITTED RUST, and it is a document made almost entirely
    /// of `"` and `{`/`}`. The first version of this emitter interpolated it
    /// straight into the `println!` FORMAT string and produced a `build.rs`
    /// that did not parse; nothing caught it until a real package was generated
    /// and the file was read. So this asserts the escaping, not just that the
    /// bytes are somewhere in the file:
    ///
    /// * the document is a `let`-bound STRING LITERAL, so `{`/`}` need no
    ///   doubling AND `clippy::print_literal` does not fire (phase-403 W8: the
    ///   argument-position form this used to emit warned in every generated
    ///   message crate, which a `-D warnings` lane turns into a build failure);
    /// * every `"` inside it is backslash-escaped, so the literal does not end
    ///   early;
    /// * un-escaping the literal body gives back the exact document.
    #[test]
    fn the_build_rs_payload_is_an_escaped_rust_string_literal() {
        let i = inv();
        let build_rs = i.to_build_rs();
        let line = build_rs
            .lines()
            .find(|l| l.contains("let bounds_json = "))
            .expect("build.rs binds the inventory to a name");

        // Bound to a name, never interpolated into the format string, and
        // never passed as a literal ARGUMENT (`clippy::print_literal`).
        assert!(
            line.trim_start().starts_with(r#"let bounds_json = ""#),
            "the document must be a let-bound literal: {line}"
        );
        assert!(
            build_rs.contains(r#"println!("cargo:bounds_json={bounds_json}");"#),
            "the emitted println must interpolate the NAME: {build_rs}"
        );
        assert!(
            !build_rs.contains(r#"println!("cargo:bounds_json={}", ""#),
            "argument-position literal is back; clippy::print_literal warns on it"
        );
        // A raw, unescaped document would show a bare `{"` here.
        assert!(
            !build_rs.contains("{\"derivation"),
            "the document leaked into the file unescaped: {build_rs}"
        );

        let body = line
            .split_once(r#"let bounds_json = ""#)
            .unwrap()
            .1
            .trim_end()
            .trim_end_matches(';')
            .trim_end_matches('"');
        let unescaped = body.replace("\\\"", "\"").replace("\\\\", "\\");
        assert_eq!(unescaped, i.to_json_compact());
        let parsed: serde_json::Value = serde_json::from_str(&unescaped).expect("valid JSON");
        assert_eq!(parsed["schema_version"], INVENTORY_SCHEMA_VERSION);
        assert_eq!(parsed["types"].as_array().unwrap().len(), 3);

        // A `cargo:` metadata value may not contain a newline.
        assert!(!i.to_json_compact().contains('\n'));
    }

    #[test]
    fn the_json_and_the_compact_json_are_the_same_document() {
        let i = inv();
        let a: serde_json::Value = serde_json::from_str(&i.to_json()).unwrap();
        let b: serde_json::Value = serde_json::from_str(&i.to_json_compact()).unwrap();
        assert_eq!(a, b);
    }

    /// Emission order must not depend on the order the driver walked the
    /// package, or the artifact churns and `write_if_changed` stops keeping
    /// mtimes still -- which re-stales every fixture keyed on it.
    #[test]
    fn emission_is_sorted_so_the_artifact_is_byte_stable() {
        let mut a = BoundInventory::new("p");
        let mut b = BoundInventory::new("p");
        let m = parse_message("int32 x\n").unwrap();
        for n in ["p/msg/C", "p/msg/A", "p/msg/B"] {
            a.record_message(n, &m, &CapacityResolver::empty(), &no_lookup);
        }
        for n in ["p/msg/B", "p/msg/C", "p/msg/A"] {
            b.record_message(n, &m, &CapacityResolver::empty(), &no_lookup);
        }
        assert_eq!(a.to_json(), b.to_json());
        assert_eq!(a.to_cmake(), b.to_cmake());
    }

    #[test]
    fn a_reason_cannot_break_out_of_the_cmake_string() {
        let mut i = BoundInventory::new("p");
        i.insert(
            "p/msg/M",
            BoundState::Unbounded {
                reason: "he said \"x\" and \\ then".to_string(),
            },
        );
        let c = i.to_cmake();
        assert!(c.contains(r#"\"x\""#), "{c}");
        assert!(c.contains(r"\\"), "{c}");
    }

    #[test]
    fn the_links_key_is_unique_per_package_and_a_legal_ident() {
        assert_eq!(
            BoundInventory::links_key("nav_msgs"),
            "nros_msgs_nav_msgs".to_string()
        );
        assert_eq!(
            BoundInventory::links_key("my-msgs"),
            "nros_msgs_my_msgs".to_string()
        );
    }

    /// The C++ pack's in-header `SERIALIZED_SIZE_MAX` is an ESTIMATE, and the
    /// inventory must never carry it. Pinned here rather than left as prose,
    /// because the direction matters: the estimate charges a FLAT 512 bytes per
    /// nested message, so a nested type whose own bound exceeds 512 makes the
    /// C++ constant SMALLER than the real bound. That is not a conservative
    /// over-estimate; it is a number that would under-size a receive buffer.
    #[test]
    fn the_cpp_packs_constant_under_estimates_a_large_nested_type() {
        let inner_src = "float64[100] samples\n";
        let outer_src = "p/Inner inner\n";
        let outer = rosidl_parser::parse_message(outer_src).unwrap();
        let lookup = |fqn: &str| -> Option<Message> {
            fqn.ends_with("Inner")
                .then(|| rosidl_parser::parse_message(inner_src).unwrap())
        };

        let mut i = BoundInventory::new("p");
        i.record_message("p/msg/Outer", &outer, &CapacityResolver::empty(), &lookup);
        let derived = match i.entries()[0].bound {
            BoundState::Bounded { rx, .. } => rx,
            ref other => panic!("expected a derived bound, got {other:?}"),
        };

        let cpp = crate::generate_cpp_message_package(
            "p",
            "Outer",
            &outer,
            "h",
            &crate::CapacityResolver::empty(),
        )
        .unwrap();
        let estimate: usize = cpp
            .header
            .lines()
            .find_map(|l| l.split("SERIALIZED_SIZE_MAX = ").nth(1))
            .and_then(|t| t.trim_end_matches(';').trim().parse().ok())
            .expect("the C++ header states a SERIALIZED_SIZE_MAX");

        // 100 doubles is 800 bytes of payload; the flat 512 cannot cover it.
        assert!(
            derived > 800,
            "the derived bound must actually bound the type: {derived}"
        );
        assert!(
            estimate < derived,
            "phase-403 W6 finding: the C++ pack estimates {estimate} where the \
             derived bound is {derived}. If this ever stops holding, the C++ \
             pack was fixed -- delete the test and say so, do not relax it."
        );
    }

    // -- phase-403 W0 -- a cap reaches the inventory, and BOTH transports agree --

    /// The exported inventory and the generated C header must say the same thing
    /// about the same type, because W6 made `BoundState::classify` shared
    /// between them precisely so they could not drift. Handing one a resolver
    /// and the other none would have reintroduced the drift through the
    /// argument list instead of through a second implementation, so the
    /// agreement is asserted over a type whose bound EXISTS ONLY BECAUSE OF THE
    /// CONFIG.
    #[test]
    fn a_capped_type_gets_the_same_number_in_the_inventory_and_the_header() {
        let m = parse_message("string label\nint64 v\n").unwrap();
        let caps = CapacityResolver::from_toml_str("[fields]\n\"p/M.label\" = 24\n").unwrap();

        let mut i = BoundInventory::new("p");
        i.record_message("p/msg/M", &m, &caps, &no_lookup);
        let (tx, rx) = match i.entries()[0].bound {
            BoundState::Bounded { tx, rx } => (tx, rx),
            ref other => panic!("a capped type must be bounded, got {other:?}"),
        };

        let header = crate::generate_c_message_package("p", "M", &m, "h", &caps)
            .unwrap()
            .header;
        let read = |suffix: &str| -> usize {
            header
                .lines()
                .find_map(|l| l.split(&format!("_{suffix}_MAX_SERIALIZED_SIZE ")).nth(1))
                .and_then(|t| t.trim().parse().ok())
                .unwrap_or_else(|| panic!("the header states a {suffix} bound:\n{header}"))
        };
        assert_eq!(read("TX"), tx);
        assert_eq!(read("RX"), rx);

        // Control: with no config the same `.msg` gets a number from NEITHER, so
        // the agreement above is about the cap and not about a type that was
        // bounded all along.
        let mut plain = BoundInventory::new("p");
        plain.record_message("p/msg/M", &m, &CapacityResolver::empty(), &no_lookup);
        assert_eq!(plain.entries()[0].bound.tag(), "unbounded");
        assert!(
            !crate::generate_c_message_package("p", "M", &m, "h", &CapacityResolver::empty())
                .unwrap()
                .header
                .contains("_TX_MAX_SERIALIZED_SIZE 3")
        );
    }

    /// One member reads as prose, several read as a list. The plural form is
    /// what makes a stock ROS type actionable in ONE build.
    #[test]
    fn the_reason_names_one_member_or_all_of_them() {
        assert_eq!(
            BoundState::unbounded_reason(&["a (string)".to_string()]),
            "unbounded member: a (string)"
        );
        assert_eq!(
            BoundState::unbounded_reason(&[
                "header.frame_id (string)".to_string(),
                "child_frame_id (string)".to_string(),
            ]),
            "unbounded members: header.frame_id (string), child_frame_id (string)"
        );
    }

    // ========================================================================
    // phase-403 W7b (issue 0961) -- the budget and the factor chain
    // ========================================================================

    /// Three nested messages, each level a bounded sequence, so the total is a
    /// product the way `visualization_msgs` is.
    fn nested_lookup(fqn: &str) -> Option<Message> {
        match fqn {
            "p/Outer" => Some(parse_message("Mid[<=8] mids\n").unwrap()),
            "p/Mid" => Some(parse_message("int64[<=8] leaves\n").unwrap()),
            _ => None,
        }
    }

    fn nesting_inventory(config: &str) -> BoundInventory {
        let m = parse_message("Outer[<=8] outers\n").unwrap();
        let caps = CapacityResolver::from_toml_str(config).unwrap();
        let mut i = BoundInventory::new("p");
        i.record_message("p/msg/M", &m, &caps, &nested_lookup);
        i
    }

    /// A type with NO budget behaves exactly as before: no check, no violation,
    /// and no budget key anywhere on any transport. This is the degrade-to-a-
    /// no-op requirement, asserted rather than assumed -- it is the property
    /// that makes the feature safe to add to a tree where nothing uses it.
    #[test]
    fn a_type_with_no_budget_is_untouched() {
        let i = nesting_inventory("");
        assert!(i.budget_violations().is_empty());
        assert!(i.check_budgets().is_ok());
        assert_eq!(i.entries()[0].budget, None);
        assert!(!i.to_json().contains("max_serialized_budget"));
        assert!(!i.to_cmake().contains("MAX_SERIALIZED_BUDGET"));
        // And the bound itself is what an inventory built before this wave
        // produced.
        assert!(matches!(i.entries()[0].bound, BoundState::Bounded { .. }));
    }

    /// A budget the type FITS changes nothing. Specifically: the exported size
    /// is still the DERIVED number, not the budget. A ceiling to check against
    /// is not a value to substitute -- phase-380's rule, and the one thing this
    /// campaign keeps taking back out.
    #[test]
    fn a_budget_the_type_fits_never_becomes_the_bound() {
        let derived = match nesting_inventory("").entries()[0].bound {
            BoundState::Bounded { tx, rx } => (tx, rx),
            ref other => panic!("{other:?}"),
        };
        let i = nesting_inventory("[types.\"p/M\"]\nmax_serialized = 100000000\n");
        assert!(i.check_budgets().is_ok());
        assert_eq!(
            match i.entries()[0].bound {
                BoundState::Bounded { tx, rx } => (tx, rx),
                ref other => panic!("{other:?}"),
            },
            derived,
            "the derived total stands; the budget is a ceiling, not a value"
        );
        assert!(i.to_json().contains("\"max_serialized_budget\": 100000000"));
    }

    /// Over budget is a violation, and the diagnostic names the type, BOTH
    /// numbers, the budget, and the chain WITH ITS FACTORS -- because the useful
    /// question is which level to cap, and a bare "too big" does not answer it.
    #[test]
    fn over_budget_names_the_chain_and_its_factors() {
        let i = nesting_inventory("[types.\"p/M\"]\nmax_serialized = 64\n");
        let v = i.budget_violations();
        assert_eq!(v.len(), 1, "{v:?}");
        let e = &v[0];
        assert_eq!(e.budget, 64);
        assert!(e.derived_rx > 64);
        assert_eq!(e.chains.len(), 1, "{:?}", e.chains);
        assert_eq!(e.chains[0].path, "outers.mids.leaves");
        assert_eq!(e.chains[0].factors, vec![8, 8, 8]);

        let text = e.to_string();
        assert!(text.contains("p/msg/M"), "{text}");
        assert!(text.contains("max_serialized = 64"), "{text}");
        assert!(text.contains(&e.derived_rx.to_string()), "{text}");
        assert!(text.contains(&e.derived_tx.to_string()), "{text}");
        assert!(
            text.contains("outers.mids.leaves = 8 x 8 x 8 = 512"),
            "{text}"
        );
        assert!(text.contains("PRODUCT"), "{text}");
    }

    /// An UNBOUNDED type has no total to check, so a budget on it is silent
    /// here -- it already fails through the louder unbounded channel, and
    /// reporting "0 exceeds 64" would be a number nobody derived.
    #[test]
    fn a_budget_on_an_unbounded_type_is_not_a_budget_violation() {
        let m = parse_message("string s\n").unwrap();
        let caps =
            CapacityResolver::from_toml_str("[types.\"p/M\"]\nmax_serialized = 8\n").unwrap();
        let mut i = BoundInventory::new("p");
        i.record_message("p/msg/M", &m, &caps, &no_lookup);
        assert_eq!(i.entries()[0].bound.tag(), "unbounded");
        assert!(i.check_budgets().is_ok());
    }

    /// The chain reaches all three transports off ONE model, which is what this
    /// module promises. The CMake side is PARALLEL lists so a consumer can
    /// `foreach` them natively instead of parsing a packed delimiter.
    #[test]
    fn the_chain_reaches_every_transport_from_one_model() {
        let i = nesting_inventory("");
        let json = i.to_json();
        assert!(json.contains("\"path\": \"outers.mids.leaves\""), "{json}");
        assert!(json.contains("\"factors\": ["), "{json}");
        assert!(json.contains("\"elements\": 512"), "{json}");

        let cmake = i.to_cmake();
        assert!(
            cmake.contains("_CHAIN_PATHS \"outers.mids.leaves\""),
            "{cmake}"
        );
        assert!(cmake.contains("_CHAIN_FACTORS \"8 x 8 x 8\""), "{cmake}");
        assert!(cmake.contains("_CHAIN_ELEMENTS \"512\""), "{cmake}");

        // build.rs republishes the same document, so the chain rides along with
        // no second rendering.
        assert!(i.to_build_rs().contains("outers.mids.leaves"));
    }

    /// A budget elsewhere than `[types.*]` is rejected at PARSE, because
    /// `sequence`/`string` at a level are per-field capacities that compose and
    /// a total is not.
    #[test]
    fn a_budget_outside_types_is_a_config_error() {
        for body in [
            "[defaults]\nmax_serialized = 8192\n",
            "[packages.p]\nmax_serialized = 8192\n",
        ] {
            assert!(
                CapacityResolver::from_toml_str(body).is_err(),
                "accepted {body:?}"
            );
        }
        assert!(
            CapacityResolver::from_toml_str("[types.\"p/M\"]\nmax_serialized = 8192\n").is_ok()
        );
    }

    /// The budget is checked as a BUILD ERROR on the emitter path, not only in
    /// the inventory -- and the header and the inventory agree about the number
    /// they refuse on, because both come from `BoundState::classify`.
    #[test]
    fn the_c_emitter_refuses_a_type_over_its_budget() {
        let m = parse_message("Outer[<=8] outers\n").unwrap();
        let caps =
            CapacityResolver::from_toml_str("[types.\"p/M\"]\nmax_serialized = 64\n").unwrap();
        let text = match crate::generate_c_message_package_with_lookup(
            "p",
            "M",
            &m,
            "h",
            &caps,
            &nested_lookup,
        ) {
            Err(e) => e.to_string(),
            Ok(_) => panic!("over budget must not generate"),
        };
        assert!(text.contains("p/M"), "{text}");
        assert!(text.contains("outers.mids.leaves = 8 x 8 x 8"), "{text}");

        // Without the budget the same type generates, and states the same number
        // the violation reported.
        let ok = crate::generate_c_message_package_with_lookup(
            "p",
            "M",
            &m,
            "h",
            &CapacityResolver::empty(),
            &nested_lookup,
        )
        .expect("a budget-free type is unaffected");
        let rx = match nesting_inventory("").entries()[0].bound {
            BoundState::Bounded { rx, .. } => rx,
            ref other => panic!("{other:?}"),
        };
        assert!(
            ok.header.contains(&format!("_RX_MAX_SERIALIZED_SIZE {rx}")),
            "{}",
            ok.header
        );
    }
}
