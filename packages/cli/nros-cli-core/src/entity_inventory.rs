//! phase-403 W9 (issue 0965) -- WHICH ENTITIES AN IMAGE CREATES.
//!
//! The bound inventory (`rosidl_codegen::bounds`) prices a TYPE. It cannot say
//! whether an image subscribes to that type, and three consumers need the
//! second question answered: the zenoh payload class boundaries, the executor
//! arena, and `NROS_EXECUTOR_MAX_CBS`. This module is the second source.
//!
//! It follows `bounds.rs`'s shape deliberately -- ONE data model, rendered into
//! the transports the later stages already speak -- rather than inventing a
//! second inventory mechanism:
//!
//! * [`EntityInventory::to_json`] -- canonical, `nros_entity_inventory.json`.
//! * [`EntityInventory::to_cmake`] -- an `include()`able fragment for the
//!   CMake/Kconfig lane, the same projection `nros_message_bounds.cmake` is.
//! * [`EntityInventory::to_env`] -- `KEY=VALUE` lines, the carrier that reaches
//!   a cargo invocation. `bounds.rs` uses a generated crate's `links` key for
//!   this rung; an entity inventory has no generated crate of its own, and the
//!   knob it feeds (`NROS_EXECUTOR_MAX_CBS`) is read from the ENVIRONMENT by
//!   `nros-node/build.rs`. So the env line IS the cargo transport here, and it
//!   is the same one `nros ws entity-facts` already publishes through
//!   `corrosion_set_env_vars` (`cmake/NanoRosEntityFacts.cmake`).
//!
//! # The JOIN KEY (phase-403 step 1)
//!
//! Counting entities answers `NROS_EXECUTOR_MAX_CBS` and nothing else. The two
//! SIZE consumers need which types are received, which is why every transport
//! also carries [`EntityInventory::subscribed_types`] and
//! [`EntityInventory::received_types`].
//!
//! They are two different sets on purpose. `subscribed_types` is what
//! `nros_derive_message_bound_knobs` narrows the zenoh payload classes with,
//! because those pools have exactly one allocation site and it is reached only
//! from `declare_subscriber`. `received_types` is wider -- a service server, a
//! service client and both action roles all carry receive buffers -- and it is
//! what the executor arena needs. Collapsing them would either price a
//! service's request against a pool it never allocates from, or leave the arena
//! blind to four kinds. See [`EntityKind::receives`] for how each was read off
//! the arena entry types rather than off the names.
//!
//! The spellings join because both inventories key on `pkg/msg/Name`. That is
//! true for messages and cannot be true for services and actions:
//! `BoundInventory::record_message` is called for `.msg` files and for nothing
//! else, so `pkg/srv/Name_Request` and `pkg/action/Name_Result` have no bound
//! entry to join against. A consumer that meets one must REFUSE, and the CMake
//! reader does.
//!
//! # Where the declaration comes from, and why it is AUTHOR-STATED
//!
//! RFC-0043/0044 components create their entities in CONSTRUCTORS, at runtime.
//! The registration macros (`NROS_SUBSCRIBE`, `create_publisher`,
//! `NROS_CREATE_WALL_TIMER`) do know the kind and the type `M` -- but anything they
//! emit is a LINK-SECTION fact, and it exists only after linking. The numbers
//! this inventory feeds are `const` sizes compiled INTO `nros-node`, which is
//! built before a single component TU is compiled. A link-section manifest can
//! therefore VERIFY a count and can never SUPPLY one; that is the direction of
//! the build graph, not a gap in the tooling.
//!
//! So the declaration is stated where the component is already declared --
//! `nano_ros_node_register(... ENTITIES ...)`, beside `CLASS`, `SHAPE` and
//! `CALLBACK_GROUPS` -- and travels the channel that declaration already
//! travels, `nros-metadata.json`.
//!
//! # An under-report can never be silent
//!
//! Three layers, in the order they fire:
//!
//! 1. **Composition refuses on INCOMPLETE data.** If any component in the image
//!    states no `ENTITIES` at all, [`EntityInventory::derive`] refuses for the
//!    WHOLE image and no knob is derived -- the same rule
//!    `NanoRosMessageBounds.cmake` holds when any type in the closure is
//!    unbounded. A component that really creates nothing says so explicitly
//!    (`ENTITIES NONE`), so ABSENCE always means "nobody said", never "zero".
//! 2. **The derived value carries NO headroom.** It is exactly the declared
//!    slot demand. That is deliberate: it makes the running image a CHECKER of
//!    its own manifest.
//! 3. **A short manifest is a named boot failure.** Registration past the table
//!    returns `NodeError::ExecutorFull`, which names the knob, and
//!    `ComponentNode`'s `ok()` flag makes the entry halt boot naming the
//!    failing node. `MAX_CBS` is the right FIRST consumer precisely because its
//!    under-size failure is already loud: an under-sized ARENA halts during
//!    entity creation, before the first spin, which is why issue 0900 W1's
//!    advisory cannot cover it.
//!
//! # A publisher claims no callback slot, and that is MEASURED
//!
//! `NROS_EXECUTOR_MAX_CBS` sizes the executor's callback-entry table. Every
//! registration site that claims one calls `Executor::next_entry_slot()`, and
//! the 24 sites that do are subscriptions, timers, services, service clients,
//! action servers, action clients and guard conditions. `create_publisher` is
//! not among them -- on the C++ path it writes an `RmwPublisher` into
//! caller-owned storage (`nros-cpp/src/publisher.rs`) and on the C path there
//! is no `nros_executor_add_publisher` to increment `handle_count`.
//!
//! This matters because the mr-canhubk344 bring-up recorded "33 handles" for
//! the island and set `MAX_CBS=36` from it. 33 is the ENTITY count; 14 of those
//! are publishers, which claim no slot. Both numbers are in the inventory, and
//! [`EntityKind::callback_slots`] is the only place the difference is spelled.
//!
//! # The infrastructure services are NOT a hidden term, and that was checked
//!
//! The obvious way for this derivation to be short is an entity the executor
//! creates that no component declares. There are two candidates and neither
//! claims a slot: `ParamState` is "stored outside the arena so it doesn't
//! consume `MAX_CBS` slots" (`parameter_services.rs`), and the five REP-2002
//! lifecycle servers go through `create_lc_srv`, which calls
//! `session.create_service` directly and never
//! `Executor::register_service_*`. So the declared application entities are the
//! whole demand -- which is what makes rule 2 above (no headroom) a checkable
//! claim rather than a hopeful one.

use std::collections::BTreeMap;

/// Bumped when the shape of the emitted inventory changes incompatibly.
/// A consumer that does not recognise the version must refuse, never guess.
///
/// **2** (phase-403 step 1): the fragment now also carries WHICH TYPES THE
/// IMAGE RECEIVES -- `NROS_ENTITY_SUBSCRIBED_TYPES` and its wider sibling
/// `NROS_ENTITY_RECEIVED_TYPES`, each with its own status. A version-1 fragment
/// carries neither, and a reader that treated its absence as "no type is
/// received" would derive a payload class over an EMPTY set and publish a
/// number smaller than any real sample. That is an incompatible addition even
/// though nothing moved, so it bumps.
///
/// **3** (phase-403 step 2): the declared QoS DEPTHS --
/// `NROS_ENTITY_DECLARED_DEPTHS` and, load-bearing beside it,
/// `NROS_ENTITY_UNDECLARED_DEPTH_COUNT`. Bumps on the same argument: a
/// version-2 fragment carries no depth at all, and a reader that took the
/// absent list for "every endpoint is depth 0" would size an arena an order of
/// magnitude short. Absence has to be distinguishable from zero here too.
pub const ENTITY_INVENTORY_SCHEMA_VERSION: u32 = 3;

/// Canonical artifact name.
pub const ENTITY_INVENTORY_JSON_NAME: &str = "nros_entity_inventory.json";

/// CMake projection of [`ENTITY_INVENTORY_JSON_NAME`], beside it.
pub const ENTITY_INVENTORY_CMAKE_NAME: &str = "nros_entity_inventory.cmake";

/// A kind of entity a component creates.
///
/// The set is closed on purpose: an unrecognised spelling is a REFUSAL, never a
/// row this module skips. A skipped row is exactly an under-report.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum EntityKind {
    Publisher,
    Subscription,
    Timer,
    ServiceServer,
    ServiceClient,
    ActionServer,
    ActionClient,
    GuardCondition,
}

/// Every kind, in emission order. The one list; a second one is how a kind
/// silently stops being counted.
pub const ALL_ENTITY_KINDS: &[EntityKind] = &[
    EntityKind::Publisher,
    EntityKind::Subscription,
    EntityKind::Timer,
    EntityKind::ServiceServer,
    EntityKind::ServiceClient,
    EntityKind::ActionServer,
    EntityKind::ActionClient,
    EntityKind::GuardCondition,
];

impl EntityKind {
    /// The canonical spelling, used on every transport and in the declaration.
    pub fn tag(self) -> &'static str {
        match self {
            EntityKind::Publisher => "publisher",
            EntityKind::Subscription => "subscription",
            EntityKind::Timer => "timer",
            EntityKind::ServiceServer => "service_server",
            EntityKind::ServiceClient => "service_client",
            EntityKind::ActionServer => "action_server",
            EntityKind::ActionClient => "action_client",
            EntityKind::GuardCondition => "guard_condition",
        }
    }

    /// How many `NROS_EXECUTOR_MAX_CBS` callback-entry slots ONE entity of this
    /// kind claims.
    ///
    /// MIRROR of the `Executor::next_entry_slot()` call sites in
    /// `packages/core/nros-node/src/executor/{spin,action}.rs`, held to them by
    /// `scripts/check-entity-slot-costs.py`. The CLI cannot depend on
    /// `nros-node` -- that crate is `no_std`, platform-gated and built for the
    /// target, not the host -- so the mapping is restated here AND gated, which
    /// is the difference between this and a comment that drifts.
    ///
    /// A publisher is 0. See the module docs: it is the number the island's
    /// hand-count got wrong, and it is worth 14 slots on that image.
    pub fn callback_slots(self) -> usize {
        match self {
            EntityKind::Publisher => 0,
            EntityKind::Subscription
            | EntityKind::Timer
            | EntityKind::ServiceServer
            | EntityKind::ServiceClient
            | EntityKind::ActionServer
            | EntityKind::ActionClient
            | EntityKind::GuardCondition => 1,
        }
    }

    /// Does an entity of this kind RECEIVE a serialized payload?
    ///
    /// Read off the arena entry types in
    /// `packages/core/nros-node/src/executor/arena.rs`, which is where a
    /// receive buffer is actually spelled -- not off the names, which mislead
    /// in both directions (a service CLIENT receives; an action CLIENT receives
    /// three different things).
    ///
    /// * `Subscription` -- the topic sample. `SubBufferedRawCEntry` and its
    ///   siblings.
    /// * `ServiceServer` -- the REQUEST. `SrvRawEntry<REQ_BUF, REPLY_BUF>`
    ///   carries a `req_buffer`.
    /// * `ServiceClient` -- the REPLY. `ServiceClientRawArenaEntry<REPLY_BUF>`
    ///   carries a `reply_buffer`.
    /// * `ActionServer` -- three: the SendGoal request, the GetResult request
    ///   and the CancelGoal request.
    ///   `ActionServerRawArenaEntry<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, _>`.
    /// * `ActionClient` -- three: the goal RESPONSE, the result RESPONSE and
    ///   the feedback message. `ActionClientRawArenaEntry` has the same three
    ///   const buffers, which is the clearest statement that "client" says
    ///   nothing about direction.
    /// * `Publisher` -- no. It SERIALIZES into a per-call stack array
    ///   (`DEFAULT_TX_BUF` in `executor/types.rs`), which is a transmit buffer
    ///   and a different question.
    /// * `Timer`, `GuardCondition` -- no payload at all.
    ///
    /// This is the SEMANTIC predicate. It is deliberately wider than
    /// [`Self::receives_topic_sample`], because the two answer different
    /// questions and collapsing them is how a buffer gets sized too small.
    pub fn receives(self) -> bool {
        match self {
            EntityKind::Subscription
            | EntityKind::ServiceServer
            | EntityKind::ServiceClient
            | EntityKind::ActionServer
            | EntityKind::ActionClient => true,
            EntityKind::Publisher | EntityKind::Timer | EntityKind::GuardCondition => false,
        }
    }

    /// Does an entity of this kind draw from the backend's TOPIC PAYLOAD
    /// pools -- the two statically sized classes
    /// `NROS_SUBSCRIBER_BUFFER_SIZE` / `NROS_SUBSCRIBER_LARGE_SIZE` size?
    ///
    /// Only a subscription, and that is MEASURED rather than assumed: in
    /// `packages/rmw/zenoh/nros-rmw-zenoh/src/shim/subscriber.rs` the pools
    /// `SMALL_PAYLOADS` / `LARGE_PAYLOADS` are reached through exactly one
    /// allocation, `alloc_payload_block(rx_buffer_hint)`, and it has exactly
    /// one caller -- the `declare_subscriber` path. A service server's request
    /// buffer and an action client's feedback buffer are real receive buffers
    /// and neither is one of these blocks; they are sized by other knobs.
    ///
    /// So narrowing the payload classes to subscriptions is not an
    /// under-count. Including the other receiving kinds would not make the
    /// number safer -- it would make it describe a pool those entities never
    /// allocate from.
    pub fn receives_topic_sample(self) -> bool {
        matches!(self, EntityKind::Subscription)
    }

    /// Does an entity of this kind have a QoS HISTORY DEPTH at all?
    ///
    /// phase-403 step 2. `@depth=N` is a QoS attribute, and a timer and a guard
    /// condition are not endpoints -- they carry no QoS, so a depth on one is a
    /// statement about nothing. It is REJECTED rather than ignored, for the
    /// reason an unknown kind is: a silently ignored attribute is a declaration
    /// the author believes they made.
    ///
    /// Every other kind does carry one. A publisher's depth sizes no receive
    /// buffer, so nothing reads it yet, but it is a real QoS field and
    /// forbidding it here would make the grammar say something false.
    pub fn carries_qos_depth(self) -> bool {
        !matches!(self, EntityKind::Timer | EntityKind::GuardCondition)
    }

    /// Parse one declared kind.
    ///
    /// Accepts the canonical [`Self::tag`] plus the short spellings a human
    /// writing a CMake argument reaches for. Anything else is an ERROR and not
    /// a skipped row -- see the type docs.
    pub fn parse(s: &str) -> Result<Self, String> {
        let norm = s.trim().to_ascii_lowercase().replace('-', "_");
        Ok(match norm.as_str() {
            "publisher" | "pub" => EntityKind::Publisher,
            "subscription" | "sub" | "subscriber" => EntityKind::Subscription,
            "timer" | "tmr" => EntityKind::Timer,
            "service_server" | "service" | "srv" | "server" => EntityKind::ServiceServer,
            "service_client" | "client" => EntityKind::ServiceClient,
            "action_server" => EntityKind::ActionServer,
            "action_client" => EntityKind::ActionClient,
            "guard_condition" | "guard" => EntityKind::GuardCondition,
            _ => {
                return Err(format!(
                    "unknown entity kind `{s}` -- expected one of: {}",
                    ALL_ENTITY_KINDS
                        .iter()
                        .map(|k| k.tag())
                        .collect::<Vec<_>>()
                        .join(", ")
                ));
            }
        })
    }
}

/// One declared entity.
///
/// `type_name` is the ROS name the bound inventory prices (`pkg/msg/Name`) --
/// the same spelling `TypeBoundEntry::type_name` uses, so the two inventories
/// join without a second naming convention. It is OPTIONAL because a timer and
/// a guard condition carry no type, and because a count is useful before every
/// call site has been annotated. `name` is the topic / service / action name.
/// `depth` is the QoS HISTORY DEPTH the author declared (phase-403 step 2), and
/// it is `Option` for the reason every other field here is: the arena's
/// per-subscription cost is `(depth + 1) * bound + (depth + 1) * 8`, so a
/// DEFAULT is wrong by up to 10x in either direction -- assuming the ROS
/// default 10 inflates an image that states 1 tenfold, and assuming 1
/// UNDER-sizes one that took the default. `None` means NOBODY SAID, and a
/// consumer that needs a depth must refuse on it. It must never read as 0.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EntityDecl {
    pub kind: EntityKind,
    pub type_name: Option<String>,
    pub name: Option<String>,
    pub depth: Option<u32>,
}

impl EntityDecl {
    /// Parse the declaration spelling:
    /// `<kind>[:<type>[:<name>]][@<attr>=<value>...]`.
    ///
    /// `sub:nav_msgs/msg/Odometry:/localization/kinematic_state@depth=10`,
    /// `timer`, `publisher:autoware_vehicle_msgs/msg/GearCommand`.
    ///
    /// A `*N` suffix on the kind repeats it: `timer*3`. A repeat count is the
    /// one concession to brevity, and it is on the KIND rather than a separate
    /// argument so a row can never lose its multiplier in transit.
    ///
    /// # Why attributes are NAMED and split off FIRST
    ///
    /// The positional part is parsed `splitn(3, ':')`, so the NAME takes the
    /// rest of the spec and a fourth positional field would be ambiguous
    /// against a topic containing a colon. `@depth=10` is therefore a named
    /// attribute, which also leaves room for `@reliability=`, `@history=` and
    /// `@durability=` without another grammar change.
    ///
    /// The `@` split runs BEFORE the `:` split, so an attribute attaches to the
    /// whole declaration rather than to whichever field happens to be last.
    /// That is safe because neither a ROS type name nor a ROS topic name may
    /// contain `@` (REP-144 allows alphanumerics, `_`, `/`, `~`, `{`, `}`), so
    /// the character cannot occur in the positional part.
    ///
    /// An UNKNOWN attribute is an error, never a skipped one, on this module's
    /// standing rule: a silently ignored declaration is one the author believes
    /// they made.
    pub fn parse(spec: &str) -> Result<Vec<Self>, String> {
        let spec = spec.trim();
        if spec.is_empty() {
            return Err("empty entity declaration".to_string());
        }
        let mut fields = spec.split('@');
        let positional = fields.next().unwrap_or("").trim();
        let mut depth: Option<u32> = None;
        for attr in fields {
            let attr = attr.trim();
            if attr.is_empty() {
                return Err(format!(
                    "entity declaration `{spec}`: an empty `@` attribute states nothing. \
                     Write `@depth=<N>` or drop the `@`."
                ));
            }
            let (key, value) = attr.split_once('=').ok_or_else(|| {
                format!(
                    "entity declaration `{spec}`: attribute `@{attr}` has no value. \
                     Attributes are `@<name>=<value>`, e.g. `@depth=10`."
                )
            })?;
            match key.trim() {
                "depth" => {
                    if depth.is_some() {
                        return Err(format!(
                            "entity declaration `{spec}`: `@depth=` is stated twice. \
                             One entity has one depth."
                        ));
                    }
                    let n: u32 = value.trim().parse().map_err(|_| {
                        format!(
                            "entity declaration `{spec}`: `{}` is not a QoS depth. \
                             It is a positive whole number of samples, e.g. `@depth=10`.",
                            value.trim()
                        )
                    })?;
                    if n == 0 {
                        return Err(format!(
                            "entity declaration `{spec}`: a QoS depth of 0 states nothing -- \
                             KEEP_LAST(0) holds no sample. Omit `@depth=` to say \"not \
                             declared\", which is a different claim and the one that makes a \
                             size consumer REFUSE rather than guess."
                        ));
                    }
                    depth = Some(n);
                }
                other => {
                    return Err(format!(
                        "entity declaration `{spec}`: unknown attribute `@{other}=` -- \
                         expected one of: depth"
                    ));
                }
            }
        }

        let mut parts = positional.splitn(3, ':');
        let kind_field = parts.next().unwrap_or("");
        let type_name = parts.next().map(str::trim).filter(|s| !s.is_empty());
        let name = parts.next().map(str::trim).filter(|s| !s.is_empty());

        let (kind_str, repeat) = match kind_field.split_once('*') {
            Some((k, n)) => {
                let n: usize = n.trim().parse().map_err(|_| {
                    format!("entity declaration `{spec}`: `{n}` is not a repeat count")
                })?;
                if n == 0 {
                    return Err(format!(
                        "entity declaration `{spec}`: a repeat count of 0 states nothing. \
                         Omit the row, or declare the component `NONE`."
                    ));
                }
                (k, n)
            }
            None => (kind_field, 1),
        };
        let kind = EntityKind::parse(kind_str).map_err(|e| format!("in `{spec}`: {e}"))?;
        if depth.is_some() && !kind.carries_qos_depth() {
            return Err(format!(
                "entity declaration `{spec}`: a `{}` has no QoS, so `@depth=` says nothing \
                 about it. Drop the attribute.",
                kind.tag()
            ));
        }
        Ok((0..repeat)
            .map(|_| EntityDecl {
                kind,
                type_name: type_name.map(str::to_string),
                name: name.map(str::to_string),
                depth,
            })
            .collect())
    }
}

/// What one component said about its entities.
///
/// Three-valued for the reason [`rosidl_codegen::bounds::BoundState`] is:
/// "it creates none" and "it did not say" license completely different actions,
/// and collapsing them is exactly the under-report this module exists to make
/// impossible.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Declaration {
    /// `ENTITIES <spec>...` -- the component named what it creates.
    Stated(Vec<EntityDecl>),
    /// `ENTITIES NONE` -- the component asserts it creates nothing.
    None,
    /// The register call carried no `ENTITIES` at all.
    Absent,
}

impl Declaration {
    pub fn tag(&self) -> &'static str {
        match self {
            Declaration::Stated(_) => "stated",
            Declaration::None => "none",
            Declaration::Absent => "absent",
        }
    }

    /// The declared entities; empty for both `None` and `Absent`. Callers must
    /// distinguish those two through [`Declaration::tag`], never through the
    /// length of this slice.
    pub fn entities(&self) -> &[EntityDecl] {
        match self {
            Declaration::Stated(v) => v,
            Declaration::None | Declaration::Absent => &[],
        }
    }
}

/// One component's row.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ComponentEntities {
    /// ament package the component lives in.
    pub pkg: String,
    /// The `NAME` the register call gave it -- the launch `exec`.
    pub component: String,
    /// The qualified C++ class, so a refusal names something a user can grep.
    pub class: String,
    pub declaration: Declaration,
}

/// Every component in ONE image, with what each declared.
///
/// The unit is the IMAGE, not the package: `MAX_CBS` sizes one executor and an
/// image has one. That is the same reason `nros_derive_message_bound_knobs`
/// composes over the whole linked closure rather than per package.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct EntityInventory {
    /// Where this inventory came from, for the provenance line. Usually the
    /// `nros-metadata.json` path.
    pub source: String,
    components: Vec<ComponentEntities>,
}

/// MIRRORS of the action multipliers in
/// `nros_node::executor::action`. The CLI cannot depend on `nros-node`, so
/// these are copies, and `check-infra-queryable-counts` holds each to the
/// creation calls that decide it -- the same arrangement
/// `ACTION_SERVER_QUERYABLES` already has in `cmd::entity_facts`.
///
/// They exist because a declared action is ONE entity that costs SEVERAL
/// session slots. An author writing `ENTITIES action_server:...` declares one
/// thing; the backend opens three queryables and two publishers for it. A pool
/// sized from the raw per-kind count is short for every image with an action,
/// and short halts the board.
const ACTION_SERVER_QUERYABLES: usize = 3;
const ACTION_SERVER_PUBLISHERS: usize = 2;
const ACTION_CLIENT_SUBSCRIPTIONS: usize = 1;

/// The knobs an entity inventory can answer, plus how it got there.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DerivedEntityKnobs {
    /// `NROS_EXECUTOR_MAX_CBS` -- the total callback-entry slot demand.
    pub max_cbs: usize,
    /// `NROS_EXECUTOR_ACTION_CLIENTS` (issue 0900) -- how many of those slots
    /// the arena must budget at the HEAVY entry size rather than the pub/sub
    /// one. At the defaults that is 18,048 bytes against 3,584, and the arena
    /// is inline on the TASK STACK, so budgeting every slot heavy is 74,240
    /// bytes where a talker needs 16,384.
    ///
    /// Counts action SERVERS as well as clients, though the knob is named for
    /// clients. The knob's real meaning is "slots budgeted at the worst case",
    /// and `build.rs` picked the action client as that worst case when nothing
    /// else was measured. It is not the worst case: the arena demonstrably
    /// stores `ActionServerArenaEntry`, so an action-server image occupies
    /// heavy slots too, and counting only clients would advise it into exactly
    /// the `BufferTooSmall` this derivation exists to avoid. Counting both is
    /// conservative in the safe direction.
    pub heavy_slots: usize,
    /// Every declared entity, slot-claiming or not. NOT the knob: kept because
    /// it is the number a human counts, and because the gap between the two is
    /// the finding.
    pub entity_total: usize,
    /// `NROS_MAX_SUBSCRIBERS` / `NROS_RMW_SUBSCRIBER_SLOTS` -- session
    /// subscriber slots. Declared subscriptions PLUS the feedback subscription
    /// each action client opens.
    ///
    /// Verified against the shim rather than assumed: `ZenohSubscriber::new`
    /// has exactly one caller (`create_subscription`), and the two things that
    /// looked like they might share the pool do not -- the graph cache lives in
    /// its own `graph_cache.sub` field and liveliness tokens in
    /// `liveliness[ZPICO_MAX_LIVELINESS]`. So there is no shim addend.
    pub max_subscribers: usize,
    /// `NROS_MAX_PUBLISHERS` -- declared publishers plus the feedback and
    /// status topics each action server publishes.
    pub max_publishers: usize,
    /// `NROS_MAX_QUERYABLES` -- a service server IS a queryable, and an action
    /// server is [`ACTION_SERVER_QUERYABLES`] of them.
    ///
    /// Does NOT include the parameter or lifecycle service families
    /// (`PARAM_SERVICE_QUERYABLES` 6, `LIFECYCLE_SERVICE_QUERYABLES` 5): those
    /// are per-image infrastructure enabled by a feature this inventory cannot
    /// see, so counting them here would guess. An image carrying them must
    /// still state the knob, and that is why this is a DEFAULT rather than a
    /// ceiling.
    pub max_queryables: usize,
    /// `NROS_EXECUTOR_MAX_NODES` -- one node per declared component.
    ///
    /// A `ComponentNode` constructor is one `Node::create` is one node NAME,
    /// and the executor keys node slots by name ("a repeated name must reuse
    /// its record"), so two components sharing a name share a slot and this
    /// OVER-counts by one. Over-counting is the safe direction.
    ///
    /// UNDER-counting has exactly one source: `nros_pubsub_bridge_create`
    /// creates TWO nodes whose names are RUNTIME strings, declared nowhere.
    /// That path now names this knob when the table is full rather than
    /// returning a bare error code, which is what makes deriving it safe -- the
    /// same argument that let `MAX_CBS` derive, where the shortfall surfaces as
    /// `ExecutorFull` naming the knob. An image that bridges states this knob.
    pub max_nodes: usize,
    /// Per-kind counts across the image, in [`ALL_ENTITY_KINDS`] order.
    pub per_kind: BTreeMap<&'static str, usize>,
    /// Per-component `(pkg, component, entities, slots)`, so the output records
    /// which declaration contributed what.
    pub per_component: Vec<(String, String, usize, usize)>,
}

/// The result of composing an image's declarations.
///
/// `Refused` carries prose and NO number: a consumer either reads a value this
/// module derived or reads nothing at all.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Derivation {
    Derived(Box<DerivedEntityKnobs>),
    Refused { reason: String },
}

impl Derivation {
    pub fn tag(&self) -> &'static str {
        match self {
            Derivation::Derived(_) => "derived",
            Derivation::Refused { .. } => "refused",
        }
    }

    pub fn knobs(&self) -> Option<&DerivedEntityKnobs> {
        match self {
            Derivation::Derived(k) => Some(k),
            Derivation::Refused { .. } => None,
        }
    }
}

/// Which types an image RECEIVES, and how many entities receive each.
///
/// The count is per ENTITY and not per type, because the consumer that needs
/// it counts blocks: `NROS_MAX_LARGE_SUBSCRIBERS` is how many large payload
/// BLOCKS the backend reserves, and two subscriptions on one large type need
/// two. Deduplicating to a type set would under-reserve by exactly the
/// duplicates.
///
/// `Refused` carries prose and NO list, for the reason [`Derivation`] does: a
/// consumer either reads a set this module resolved or reads nothing. An empty
/// list is a legitimate ANSWER ("this image receives nothing of that shape")
/// and must never be confused with "nobody said".
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReceivedTypes {
    /// `(type_name, receiving entity count)`, sorted by `type_name` so the
    /// artifact is byte-stable and a write-if-changed keeps mtimes still.
    Resolved(Vec<(String, usize)>),
    Refused {
        reason: String,
    },
}

impl ReceivedTypes {
    pub fn tag(&self) -> &'static str {
        match self {
            ReceivedTypes::Resolved(_) => "resolved",
            ReceivedTypes::Refused { .. } => "refused",
        }
    }

    pub fn types(&self) -> Option<&[(String, usize)]> {
        match self {
            ReceivedTypes::Resolved(v) => Some(v),
            ReceivedTypes::Refused { .. } => None,
        }
    }
}

/// One endpoint that stated a QoS history DEPTH (phase-403 step 2).
///
/// `type_name` is the ROS spelling the declaration used (`pkg/msg/Name`);
/// [`dds_type_name`] is what a C++ TU sees as `M::TYPE_NAME`. Both travel,
/// because the two consumers key differently: the arena joins on the ROS
/// spelling that the bound inventory also uses, and the compile-time check
/// joins on whatever the generated message class actually carries.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DeclaredDepth {
    pub kind: EntityKind,
    pub type_name: String,
    pub topic: String,
    pub depth: u32,
}

/// Every declared depth in an image, plus how many endpoints did NOT state one.
///
/// The undeclared COUNT is the load-bearing field. `Resolved { rows: [] }` and
/// `Resolved { rows: [...], undeclared: 4 }` are different facts: the first is
/// an image that has not opted in at all, the second one that opted in
/// partially, and a size consumer must refuse on both while a compile-time
/// check must fire on neither. Reporting only the rows would collapse them.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DeclaredDepths {
    Resolved {
        /// Sorted by `(type_name, topic)` so the artifact is byte-stable.
        rows: Vec<DeclaredDepth>,
        /// Endpoints that COULD carry a depth ([`EntityKind::carries_qos_depth`])
        /// and stated none. NOT zero-by-default: this is the number that says
        /// "nobody said" for the rest of the image.
        undeclared: usize,
    },
    Refused {
        reason: String,
    },
}

impl DeclaredDepths {
    pub fn tag(&self) -> &'static str {
        match self {
            DeclaredDepths::Resolved { .. } => "resolved",
            DeclaredDepths::Refused { .. } => "refused",
        }
    }

    pub fn rows(&self) -> Option<&[DeclaredDepth]> {
        match self {
            DeclaredDepths::Resolved { rows, .. } => Some(rows),
            DeclaredDepths::Refused { .. } => None,
        }
    }
}

/// The DDS-mangled spelling a generated C++ message class carries as
/// `static constexpr const char* TYPE_NAME`.
///
/// `nav_msgs/msg/Odometry` -> `nav_msgs::msg::dds_::Odometry_`, which is
/// literally what `packs/cpp/message.hpp.jinja` emits
/// (`{{package_name}}::msg::dds_::{{message_name}}_`). Restated here rather
/// than shared because that emitter is a Tera template in a different crate;
/// `the_dds_spelling_matches_the_cpp_template` holds the two together.
///
/// A spelling that is already mangled (it contains `::`) passes through, so an
/// author who declares the C++ name gets what they wrote. Anything that is not
/// three `/`-separated segments also passes through unchanged: guessing at a
/// mangling for a shape the codegen does not emit would put a key in the table
/// that nothing can ever match, which is worse than no key.
pub fn dds_type_name(ros_name: &str) -> String {
    if ros_name.contains("::") {
        return ros_name.to_string();
    }
    let parts: Vec<&str> = ros_name.split('/').collect();
    if parts.len() != 3 || parts.iter().any(|p| p.is_empty()) {
        return ros_name.to_string();
    }
    format!("{}::{}::dds_::{}_", parts[0], parts[1], parts[2])
}

impl EntityInventory {
    pub fn new(source: impl Into<String>) -> Self {
        Self {
            source: source.into(),
            components: Vec::new(),
        }
    }

    /// phase-412 -- build the inventory from a resolved SystemModel's wiring
    /// instead of from `ENTITIES`.
    ///
    /// `structure.topics` carries, per topic, its message type and the endpoint
    /// refs on each side (`/node/endpoint`). That is exactly a per-node sub/pub
    /// set with types attached, which is what this inventory holds -- so the
    /// authored contract beside the launch file can replace the `ENTITIES` list
    /// duplicated in every component's `CMakeLists.txt`.
    ///
    /// The two sources are not interchangeable, and the difference is the whole
    /// reason this is a separate constructor rather than a swap:
    ///
    /// * The model has NO TIMER ENTITY. A component whose only callback is a
    ///   timer contributes nothing here and one callback slot at runtime. The
    ///   island has four such timers, so a `MAX_CBS` derived from the model
    ///   alone is short by four. Callers must combine this with the declaration
    ///   or the component sidecar -- `model_ingest` states the same rule as
    ///   `max(model_wiring, recorded_metadata)`.
    /// * Service and action wiring lives in `structure.{services,actions}`,
    ///   which this reads too, but a model that describes no wiring at all is
    ///   ABSENT rather than zero -- see `entity_facts::describes_wiring`, and
    ///   the same three-valued rule [`Declaration`] keeps.
    ///
    /// Returns `None` when the model describes no wiring, so a caller cannot
    /// mistake "nobody authored a contract" for "this image creates nothing".
    /// That distinction is the one this module exists to preserve.
    pub fn from_model(
        source: impl Into<String>,
        model: &ros_launch_manifest_model::SystemModel,
    ) -> Option<Self> {
        if model.structure.topics.is_empty()
            && model.structure.services.is_empty()
            && model.structure.actions.is_empty()
        {
            return None;
        }

        // Endpoint refs are `/ns/node/endpoint`; the node FQN is everything but
        // the last segment. Group per node so each becomes one component row.
        fn node_of(ep: &str) -> String {
            ep.rsplit_once('/')
                .map(|(n, _)| n)
                .unwrap_or(ep)
                .to_string()
        }

        let mut per_node: std::collections::BTreeMap<String, Vec<EntityDecl>> =
            std::collections::BTreeMap::new();

        for wiring in model.structure.topics.values() {
            for ep in &wiring.subscribers {
                per_node.entry(node_of(ep)).or_default().push(EntityDecl {
                    kind: EntityKind::Subscription,
                    type_name: Some(wiring.msg_type.clone()),
                    name: Some(ep.clone()),
                    depth: None,
                });
            }
            for ep in &wiring.publishers {
                per_node.entry(node_of(ep)).or_default().push(EntityDecl {
                    kind: EntityKind::Publisher,
                    type_name: Some(wiring.msg_type.clone()),
                    name: Some(ep.clone()),
                    depth: None,
                });
            }
        }

        let mut inv = Self::new(source);
        for (node_fqn, entities) in per_node {
            let component = node_fqn.rsplit('/').next().unwrap_or(&node_fqn).to_string();
            inv.insert(ComponentEntities {
                // The model names nodes, not ament packages. A node FQN is the
                // stable identity here, and stating it as the package rather
                // than inventing one keeps the provenance line honest about
                // where the row came from.
                pkg: node_fqn.clone(),
                component,
                class: String::new(),
                declaration: Declaration::Stated(entities),
            });
        }
        Some(inv)
    }

    /// Record one component. A later record for the same `(pkg, component)`
    /// replaces the earlier one, so a configure that registers a component
    /// twice cannot double-count it.
    pub fn insert(&mut self, row: ComponentEntities) {
        match self
            .components
            .iter_mut()
            .find(|c| c.pkg == row.pkg && c.component == row.component)
        {
            Some(existing) => *existing = row,
            None => self.components.push(row),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.components.is_empty()
    }

    pub fn len(&self) -> usize {
        self.components.len()
    }

    /// Rows in emission order: sorted by `(pkg, component)`, so the artifact is
    /// byte-stable across runs and a write-if-changed keeps mtimes still.
    pub fn components(&self) -> Vec<&ComponentEntities> {
        let mut v: Vec<&ComponentEntities> = self.components.iter().collect();
        v.sort_by(|a, b| (&a.pkg, &a.component).cmp(&(&b.pkg, &b.component)));
        v
    }

    /// Compose the image's declarations into the knobs, or REFUSE.
    ///
    /// Refuses when the image has no components at all, and when ANY component
    /// stated nothing. Partial data never yields a number: an image whose
    /// fourth node has not been annotated would otherwise derive a total three
    /// nodes' worth short, and a short `MAX_CBS` is a failed entity creation on
    /// a board.
    pub fn derive(&self) -> Derivation {
        if self.components.is_empty() {
            return Derivation::Refused {
                reason: "no components were registered in this image, so there is nothing to \
                         compose. `nano_ros_node_register()` is what puts a component here."
                    .to_string(),
            };
        }

        let undeclared: Vec<&ComponentEntities> = self
            .components()
            .into_iter()
            .filter(|c| matches!(c.declaration, Declaration::Absent))
            .collect();
        if !undeclared.is_empty() {
            let block = undeclared
                .iter()
                .map(|c| format!("    {}::{} ({})", c.pkg, c.component, c.class))
                .collect::<Vec<_>>()
                .join("\n");
            return Derivation::Refused {
                reason: format!(
                    "{} of {} components in this image declare no entities:\n{block}\n\
                     Deriving over only the components that did would publish a slot count \
                     smaller than the image needs, and a short NROS_EXECUTOR_MAX_CBS fails \
                     entity creation at boot. Add `ENTITIES ...` to each \
                     `nano_ros_node_register()` above -- `ENTITIES NONE` for a component that \
                     really creates none.",
                    undeclared.len(),
                    self.components.len()
                ),
            };
        }

        let mut per_kind: BTreeMap<&'static str, usize> = BTreeMap::new();
        for k in ALL_ENTITY_KINDS {
            per_kind.insert(k.tag(), 0);
        }
        let mut per_component = Vec::new();
        let mut max_cbs = 0usize;
        let mut heavy_slots = 0usize;
        let mut entity_total = 0usize;
        for c in self.components() {
            let mut slots = 0usize;
            let mut count = 0usize;
            for e in c.declaration.entities() {
                *per_kind.entry(e.kind.tag()).or_insert(0) += 1;
                slots += e.kind.callback_slots();
                if matches!(e.kind, EntityKind::ActionClient | EntityKind::ActionServer) {
                    heavy_slots += e.kind.callback_slots();
                }
                count += 1;
            }
            max_cbs += slots;
            entity_total += count;
            per_component.push((c.pkg.clone(), c.component.clone(), count, slots));
        }

        // phase-412 W1. A declared action is ONE entity that costs SEVERAL
        // session slots, so the session pools are the per-kind count PLUS the
        // multipliers held beside the calls that decide them. Reading the raw
        // count would size every action-carrying image short.
        let n = |tag: &str| per_kind.get(tag).copied().unwrap_or(0);
        let floor1 = |v: usize| v.max(1);
        let max_subscribers = floor1(
            n(EntityKind::Subscription.tag())
                + n(EntityKind::ActionClient.tag()) * ACTION_CLIENT_SUBSCRIPTIONS,
        );
        let max_publishers = floor1(
            n(EntityKind::Publisher.tag())
                + n(EntityKind::ActionServer.tag()) * ACTION_SERVER_PUBLISHERS,
        );
        // Issue 1015 -- FLOOR OF ONE on any pool that backs a fixed C array.
        //
        // phase-403's rule is that a derived value carries NO headroom: it is
        // exactly the declared demand, so the running image checks its own
        // declaration. That is right for a table the executor INDEXES, where
        // registering past the end returns `ExecutorFull` and names the knob.
        //
        // It is wrong here. These three size C arrays in `zpico.c`
        // (`queryable_entry_t queryables[ZPICO_MAX_QUERYABLES]` and its
        // siblings), and a zero-length array is not a smaller pool -- it is a
        // different kind of object, and it is not the last member of that
        // struct. Measured on the reference island, which declares no service
        // servers and so derived exactly 0: the board transmitted NOTHING in
        // 15 seconds, no panic, no log, core in WFI. The same image with the
        // pool at 4 transmitted 110 bytes. Every gate was green either way,
        // because 0 was derived correctly and delivered faithfully.
        //
        // One slot costs a handful of bytes. A zero costs the whole image.
        let max_queryables = floor1(
            n(EntityKind::ServiceServer.tag())
                + n(EntityKind::ActionServer.tag()) * ACTION_SERVER_QUERYABLES,
        );
        let max_nodes = self.components().len();

        Derivation::Derived(Box::new(DerivedEntityKnobs {
            max_cbs,
            heavy_slots,
            entity_total,
            max_subscribers,
            max_publishers,
            max_queryables,
            max_nodes,
            per_kind,
            per_component,
        }))
    }

    /// The types this image receives THROUGH THE TOPIC PAYLOAD POOLS -- the
    /// join key for the zenoh payload classes (phase-403 step 1).
    ///
    /// Filters the declaration to [`EntityKind::receives_topic_sample`], which
    /// is subscriptions, and counts one per ENTITY.
    pub fn subscribed_types(&self) -> ReceivedTypes {
        self.types_received_by(EntityKind::receives_topic_sample, "subscribed")
    }

    /// Every type this image receives, over every receiving kind
    /// ([`EntityKind::receives`]).
    ///
    /// WIDER than [`Self::subscribed_types`] and published beside it because
    /// the two consumers differ: the payload classes size a pool only
    /// subscriptions allocate from, while the executor ARENA charges a receive
    /// buffer for a service server, a service client and both action roles as
    /// well. Emitting only the narrow set would leave the arena's derivation
    /// (step 3) to re-derive it, and a second derivation is how two green
    /// tools come to disagree.
    pub fn received_types(&self) -> ReceivedTypes {
        self.types_received_by(EntityKind::receives, "received")
    }

    /// The one implementation behind the two views above.
    ///
    /// REFUSES in two cases, and both are "the answer would be short":
    ///
    /// 1. The image's own composition refused -- some component declared no
    ///    `ENTITIES` at all. Its subscriptions are then unknown, and a set
    ///    composed over the components that DID answer is a subset of what the
    ///    image receives.
    /// 2. A matching entity carries no `type_name`. A count needs no type and
    ///    `MAX_CBS` derives happily without one, but a SIZE does: an untyped
    ///    receiving entity is a payload of unknown size, and pricing the rest
    ///    would publish a maximum a real sample can exceed.
    fn types_received_by(&self, matches: fn(EntityKind) -> bool, what: &str) -> ReceivedTypes {
        if let Derivation::Refused { reason } = self.derive() {
            return ReceivedTypes::Refused {
                reason: format!(
                    "the entity inventory itself did not compose, so the {what} type set would \
                     be a subset of what this image receives:\n{reason}"
                ),
            };
        }

        let mut untyped: Vec<String> = Vec::new();
        let mut counts: BTreeMap<String, usize> = BTreeMap::new();
        for c in self.components() {
            for e in c.declaration.entities() {
                if !matches(e.kind) {
                    continue;
                }
                match &e.type_name {
                    Some(t) => *counts.entry(t.clone()).or_insert(0) += 1,
                    None => untyped.push(format!(
                        "    {}::{} declares a `{}`{} with no type",
                        c.pkg,
                        c.component,
                        e.kind.tag(),
                        match &e.name {
                            Some(n) => format!(" on `{n}`"),
                            None => String::new(),
                        }
                    )),
                }
            }
        }

        if !untyped.is_empty() {
            untyped.dedup();
            return ReceivedTypes::Refused {
                reason: format!(
                    "{} receiving entities state no type, so the size of what they receive is \
                     unknown:\n{}\nA count does not need the type and NROS_EXECUTOR_MAX_CBS still \
                     derives; a payload SIZE does. State it as `<kind>:<pkg>/msg/<Name>[:<name>]` \
                     in the component's `nano_ros_node_register(... ENTITIES ...)`.",
                    untyped.len(),
                    untyped.join("\n")
                ),
            };
        }

        ReceivedTypes::Resolved(counts.into_iter().collect())
    }

    /// Every declared QoS history depth in this image (phase-403 step 2).
    ///
    /// REFUSES on exactly one condition -- the image's own composition refused,
    /// because some component declared no `ENTITIES` at all. Its endpoints are
    /// then unknown, so a depth table composed over the components that DID
    /// answer would be missing rows that a compile-time check would then read
    /// as "nobody declared this topic" and let through.
    ///
    /// It does NOT refuse on a missing depth. An endpoint that states none is
    /// counted in `undeclared` and is simply absent from the table: that image
    /// has not opted in, which is not an error. The arena (step 3) refuses on a
    /// non-zero `undeclared` the way W8 refuses on an unbounded type; the
    /// compile-time check sees no row and asserts nothing.
    pub fn declared_depths(&self) -> DeclaredDepths {
        if let Derivation::Refused { reason } = self.derive() {
            return DeclaredDepths::Refused {
                reason: format!(
                    "the entity inventory itself did not compose, so the declared-depth table \
                     would be missing whole components -- and a missing row reads as \"nobody \
                     declared this endpoint\", which is the one thing the table must never \
                     say wrongly:\n{reason}"
                ),
            };
        }

        let mut rows: Vec<DeclaredDepth> = Vec::new();
        let mut undeclared = 0usize;
        for c in self.components() {
            for e in c.declaration.entities() {
                if !e.kind.carries_qos_depth() {
                    continue;
                }
                match (e.depth, &e.type_name, &e.name) {
                    (Some(depth), Some(t), Some(n)) => rows.push(DeclaredDepth {
                        kind: e.kind,
                        type_name: t.clone(),
                        topic: n.clone(),
                        depth,
                    }),
                    // A depth with no type or no topic cannot be JOINED to
                    // anything -- the table is keyed `(type, topic)` and the
                    // arena charges a buffer per typed endpoint. It counts as
                    // undeclared rather than being dropped silently, so the
                    // count stays the honest "endpoints this image cannot size".
                    (Some(_), _, _) | (None, _, _) => undeclared += 1,
                }
            }
        }
        rows.sort_by(|a, b| (&a.type_name, &a.topic).cmp(&(&b.type_name, &b.topic)));
        DeclaredDepths::Resolved { rows, undeclared }
    }

    /// The C++ compile-time table: `nros_declared_qos_generated.h`.
    ///
    /// SUBSCRIPTIONS only, and that is the scope of the consumer rather than a
    /// shortcut. `NROS_SUBSCRIBE` is the one macro that asserts against this
    /// table, and keying `(type, topic)` means a publisher and a subscription
    /// on the same pair would be two rows with one key. When the publish side
    /// grows a check the row gains a kind column; until then a row nothing can
    /// consult is a row that can silently be wrong.
    ///
    /// Emitted as an X-MACRO rather than a C++ array so the file is pure
    /// preprocessor: it can then be included in any order, from any language
    /// mode, and `nros/declared_qos.hpp` owns the one definition of the table's
    /// TYPE. A generated header that also declared the struct would have to be
    /// included at exactly one point of exactly one header.
    ///
    /// Both spellings of the type are emitted per row -- the ROS
    /// `pkg/msg/Name` the declaration used and the DDS-mangled
    /// `pkg::msg::dds_::Name_` a generated C++ class carries. The lookup is a
    /// compile-time linear scan, so a second row costs nothing at runtime, and
    /// which spelling a message class carries is a property of the CODEGEN that
    /// produced it, not something this file should have to predict.
    pub fn to_declared_qos_header(&self) -> String {
        let mut s = String::new();
        // Written line by line, NOT as one `\`-continued literal: Rust strips
        // the leading whitespace after a line continuation, which silently ate
        // the ` ` before every `*` and produced a comment block no C formatter
        // would accept.
        for line in [
            "/* GENERATED by `nros ws entity-inventory` (phase-403 step 2). Do not edit.",
            " *",
            " * The QoS history DEPTH each subscription was DECLARED with, in",
            " * `nano_ros_node_register(... ENTITIES sub:<type>:<topic>@depth=N ...)`.",
            " * `nros/declared_qos.hpp` expands this into a `constexpr` table and",
            " * `NROS_SUBSCRIBE` static_asserts the QoS it is handed against it, so a",
            " * declaration and an implementation that disagree fail the BUILD naming",
            " * the topic and both numbers.",
            " *",
            " * An ABSENT row is not depth 0 and not depth 10: it is \"nobody declared",
            " * this endpoint\", and nothing asserts against it.",
            " *",
        ] {
            s.push_str(line);
            s.push('\n');
        }
        s.push_str(&format!(
            " * Source: {}\n */\n",
            self.source.replace("*/", "*_/")
        ));
        s.push_str("#ifndef NROS_DECLARED_QOS_GENERATED_H\n");
        s.push_str("#define NROS_DECLARED_QOS_GENERATED_H\n\n");

        let depths = self.declared_depths();
        match &depths {
            DeclaredDepths::Refused { reason } => {
                // A refusal emits NO rows and says so in the file, on the rule
                // the rest of this module holds: a consumer reads a table this
                // module built or reads nothing. `NROS_DECLARED_QOS_ROWS` stays
                // undefined, so `declared_qos.hpp` compiles an empty table and
                // every call site keeps working unchecked.
                s.push_str("/* NO TABLE. The entity inventory refused to compose:\n *   ");
                s.push_str(&reason.replace('\n', "\n *   ").replace("*/", "*_/"));
                s.push_str("\n */\n");
                s.push_str("#define NROS_DECLARED_QOS_STATUS \"refused\"\n");
            }
            DeclaredDepths::Resolved { rows, undeclared } => {
                let subs: Vec<&DeclaredDepth> = rows
                    .iter()
                    .filter(|r| r.kind == EntityKind::Subscription)
                    .collect();
                s.push_str("#define NROS_DECLARED_QOS_STATUS \"resolved\"\n");
                s.push_str(&format!(
                    "/* {} of this image's depth-carrying endpoints declared no depth. */\n",
                    undeclared
                ));
                s.push_str(&format!(
                    "#define NROS_DECLARED_QOS_UNDECLARED_COUNT {undeclared}\n\n"
                ));
                if subs.is_empty() {
                    s.push_str(
                        "/* No subscription in this image declared a depth, so there is no\n \
                         * table to assert against. NROS_DECLARED_QOS_ROWS stays undefined\n \
                         * -- an empty list and \"nobody said\" must not look alike. */\n",
                    );
                } else {
                    s.push_str(
                        "/* X-macro. `nros/declared_qos.hpp` defines NROS_DECLARED_QOS_ROW\n \
                         * and expands this; nothing else may. */\n",
                    );
                    s.push_str("#define NROS_DECLARED_QOS_ROWS \\\n");
                    for r in &subs {
                        let dds = dds_type_name(&r.type_name);
                        s.push_str(&format!(
                            "    NROS_DECLARED_QOS_ROW(\"{}\", \"{}\", {}) \\\n",
                            c_escape(&dds),
                            c_escape(&r.topic),
                            r.depth
                        ));
                        if dds != r.type_name {
                            s.push_str(&format!(
                                "    NROS_DECLARED_QOS_ROW(\"{}\", \"{}\", {}) \\\n",
                                c_escape(&r.type_name),
                                c_escape(&r.topic),
                                r.depth
                            ));
                        }
                    }
                    s.push_str("    /* end */\n");
                    let n_rows: usize = subs
                        .iter()
                        .map(|r| {
                            if dds_type_name(&r.type_name) == r.type_name {
                                1
                            } else {
                                2
                            }
                        })
                        .sum();
                    s.push_str(&format!("#define NROS_DECLARED_QOS_ROW_COUNT {n_rows}\n"));
                }
            }
        }
        s.push_str("\n#endif /* NROS_DECLARED_QOS_GENERATED_H */\n");
        s
    }

    /// The canonical artifact.
    pub fn to_json(&self) -> String {
        let derivation = self.derive();
        let components: Vec<serde_json::Value> = self
            .components()
            .into_iter()
            .map(|c| {
                let mut m = serde_json::Map::new();
                m.insert("pkg".into(), c.pkg.clone().into());
                m.insert("component".into(), c.component.clone().into());
                m.insert("class".into(), c.class.clone().into());
                m.insert("declaration".into(), c.declaration.tag().into());
                m.insert(
                    "entities".into(),
                    c.declaration
                        .entities()
                        .iter()
                        .map(|e| {
                            let mut r = serde_json::Map::new();
                            r.insert("kind".into(), e.kind.tag().into());
                            r.insert("callback_slots".into(), e.kind.callback_slots().into());
                            if let Some(t) = &e.type_name {
                                r.insert("type_name".into(), t.clone().into());
                            }
                            if let Some(n) = &e.name {
                                r.insert("name".into(), n.clone().into());
                            }
                            // phase-403 step 2 -- present ONLY when declared.
                            // A `"depth": 0` or a `"depth": null` on every row
                            // would make "nobody said" and "said 0" the same
                            // JSON, which is the collapse this whole inventory
                            // is built to avoid.
                            if let Some(d) = e.depth {
                                r.insert("depth".into(), d.into());
                            }
                            serde_json::Value::Object(r)
                        })
                        .collect::<Vec<_>>()
                        .into(),
                );
                serde_json::Value::Object(m)
            })
            .collect();

        let mut doc = serde_json::Map::new();
        doc.insert(
            "schema_version".into(),
            ENTITY_INVENTORY_SCHEMA_VERSION.into(),
        );
        doc.insert("producer".into(), "nros ws entity-inventory".into());
        doc.insert("source".into(), self.source.clone().into());
        doc.insert("status".into(), derivation.tag().into());
        doc.insert("components".into(), components.into());
        match &derivation {
            Derivation::Derived(k) => {
                doc.insert("entity_total".into(), k.entity_total.into());
                doc.insert("max_cbs".into(), k.max_cbs.into());
                doc.insert(
                    "per_kind".into(),
                    serde_json::Value::Object(
                        k.per_kind
                            .iter()
                            .map(|(name, n)| ((*name).to_string(), (*n).into()))
                            .collect(),
                    ),
                );
            }
            Derivation::Refused { reason } => {
                doc.insert("reason".into(), reason.clone().into());
            }
        }
        // The join key, on the canonical transport too. Same three states as
        // the CMake projection: a `status` is always present and the list is
        // present only when it resolved.
        for (key, r) in [
            ("subscribed_types", self.subscribed_types()),
            ("received_types", self.received_types()),
        ] {
            let mut m = serde_json::Map::new();
            m.insert("status".into(), r.tag().into());
            match &r {
                ReceivedTypes::Refused { reason } => {
                    m.insert("reason".into(), reason.clone().into());
                }
                ReceivedTypes::Resolved(v) => {
                    m.insert(
                        "types".into(),
                        serde_json::Value::Object(
                            v.iter().map(|(t, n)| (t.clone(), (*n).into())).collect(),
                        ),
                    );
                    m.insert(
                        "entity_count".into(),
                        v.iter().map(|(_, n)| *n).sum::<usize>().into(),
                    );
                }
            }
            doc.insert(key.into(), serde_json::Value::Object(m));
        }
        // phase-403 step 2 -- the declared depths, as their own view. Same
        // three states as the two above: a `status` is always present, the rows
        // only when it resolved, and `undeclared` says how much of the image
        // stayed silent, which is the number a size consumer refuses on.
        {
            let depths = self.declared_depths();
            let mut m = serde_json::Map::new();
            m.insert("status".into(), depths.tag().into());
            match &depths {
                DeclaredDepths::Refused { reason } => {
                    m.insert("reason".into(), reason.clone().into());
                }
                DeclaredDepths::Resolved { rows, undeclared } => {
                    m.insert("undeclared".into(), (*undeclared).into());
                    m.insert(
                        "endpoints".into(),
                        rows.iter()
                            .map(|r| {
                                let mut o = serde_json::Map::new();
                                o.insert("kind".into(), r.kind.tag().into());
                                o.insert("type_name".into(), r.type_name.clone().into());
                                o.insert(
                                    "dds_type_name".into(),
                                    dds_type_name(&r.type_name).into(),
                                );
                                o.insert("name".into(), r.topic.clone().into());
                                o.insert("depth".into(), r.depth.into());
                                serde_json::Value::Object(o)
                            })
                            .collect::<Vec<_>>()
                            .into(),
                    );
                }
            }
            doc.insert("declared_depths".into(), serde_json::Value::Object(m));
        }
        format!(
            "{}\n",
            serde_json::to_string_pretty(&serde_json::Value::Object(doc)).unwrap_or_default()
        )
    }

    /// The CMake/Kconfig projection.
    ///
    /// A REFUSAL sets a status and a reason and NO `NROS_DERIVED_*` variable, so
    /// a consumer that reads a number reads one this module derived or reads
    /// nothing -- the rule `nros_message_bounds.cmake` holds for a type with no
    /// bound.
    pub fn to_cmake(&self) -> String {
        let derivation = self.derive();
        let mut s = String::new();
        s.push_str("# GENERATED by `nros ws entity-inventory` (phase-403 W9, issue 0965).\n");
        s.push_str("# Do not edit.\n#\n");
        s.push_str(
            "# WHICH ENTITIES THIS IMAGE CREATES, composed from every\n\
             # `nano_ros_node_register(... ENTITIES ...)` in it. The bound inventory\n\
             # (`nros_message_bounds.cmake`) prices a TYPE; this one counts the entities,\n\
             # which is the half `NROS_EXECUTOR_MAX_CBS` needs.\n#\n",
        );
        s.push_str(
            "# The number is a DEFAULT. An environment value or a Kconfig / board `.conf`\n\
             # value states a number and WINS; this only fills in what nobody stated.\n#\n",
        );
        s.push_str(
            "# It carries NO headroom, deliberately: it is exactly the declared slot\n\
             # demand, so a stale declaration makes the image fail entity creation with\n\
             # `ExecutorFull` naming this knob, rather than being absorbed silently.\n",
        );
        s.push_str(&format!(
            "set(NROS_ENTITY_INVENTORY_SCHEMA_VERSION {ENTITY_INVENTORY_SCHEMA_VERSION})\n"
        ));
        s.push_str(&format!(
            "set(NROS_ENTITY_INVENTORY_SOURCE \"{}\")\n",
            cmake_escape(&self.source)
        ));
        s.push_str(&format!(
            "set(NROS_ENTITY_INVENTORY_STATUS \"{}\")\n",
            derivation.tag()
        ));
        s.push_str(&format!(
            "set(NROS_ENTITY_INVENTORY_COMPONENT_COUNT {})\n",
            self.components.len()
        ));
        match &derivation {
            Derivation::Refused { reason } => {
                s.push_str(&format!(
                    "set(NROS_ENTITY_INVENTORY_REASON \"{}\")\n",
                    cmake_escape(reason)
                ));
                s.push_str("# No knob is derived. Every one keeps its configured value.\n");
            }
            Derivation::Derived(k) => {
                s.push_str(&format!(
                    "set(NROS_ENTITY_INVENTORY_ENTITY_TOTAL {})\n",
                    k.entity_total
                ));
                for (name, n) in &k.per_kind {
                    let key = name.to_ascii_uppercase();
                    s.push_str(&format!("set(NROS_ENTITY_COUNT_{key} {n})\n"));
                }
                s.push_str("# Where the slots came from -- pkg::component = entities/slots.\n");
                for (pkg, comp, count, slots) in &k.per_component {
                    s.push_str(&format!(
                        "#   {pkg}::{comp} = {count} entities, {slots} slots\n"
                    ));
                }
                s.push_str(
                    "# A publisher claims NO callback slot (it writes an RmwPublisher into\n\
                     # caller storage and never reaches Executor::next_entry_slot), so the\n\
                     # entity total above is larger than the slot demand below.\n",
                );
                s.push_str(&format!(
                    "set(NROS_DERIVED_EXECUTOR_MAX_CBS {})\n",
                    k.max_cbs
                ));
                // Issue 0900 -- of those slots, how many the arena must budget
                // at the ACTION entry size (18,048 B at the defaults) rather
                // than the pub/sub one (3,584 B). A talker derives 0 here and
                // stops carrying 74,240 bytes of task stack for an entity it
                // never constructs.
                s.push_str(
                    "# Action clients AND action servers: the knob is named for\n                     # clients because build.rs picked one as the worst case, but\n                     # the arena stores ActionServerArenaEntry too (issue 0900).\n",
                );
                s.push_str(&format!(
                    "set(NROS_DERIVED_EXECUTOR_ACTION_CLIENTS {})\n",
                    k.heavy_slots
                ));
                // phase-412 W1 -- the SESSION pools. Separate from the slot
                // demand above: a publisher claims no callback slot but does
                // claim a session slot, and a declared action claims several
                // of these for the one entity it declares.
                s.push_str(
                    "# Session pools. A declared action is ONE entity that costs\n                     # SEVERAL session slots: a server opens 3 queryables and 2\n                     # publishers, a client 1 subscription. The multipliers live\n                     # beside the calls that decide them and are held there by\n                     # check-infra-queryable-counts.\n                     # NOT included: the parameter (6) and lifecycle (5) service\n                     # families, which a feature enables and this inventory cannot\n                     # see. An image carrying them must state the knob -- which is\n                     # why these are DEFAULTS and not ceilings.\n",
                );
                s.push_str(&format!(
                    "set(NROS_DERIVED_MAX_SUBSCRIBERS {})\n",
                    k.max_subscribers
                ));
                s.push_str(&format!(
                    "set(NROS_DERIVED_RMW_SUBSCRIBER_SLOTS {})\n",
                    k.max_subscribers
                ));
                s.push_str(&format!(
                    "set(NROS_DERIVED_MAX_PUBLISHERS {})\n",
                    k.max_publishers
                ));
                s.push_str(&format!(
                    "set(NROS_DERIVED_MAX_QUERYABLES {})\n",
                    k.max_queryables
                ));
                s.push_str(
                    "# One node per declared component. Over-counts if two share\n                     # a name (slots are keyed by name); UNDER-counts only for a\n                     # bridge, whose two nodes are runtime strings declared\n                     # nowhere -- that path names this knob when the table fills.\n",
                );
                s.push_str(&format!(
                    "set(NROS_DERIVED_EXECUTOR_MAX_NODES {})\n",
                    k.max_nodes
                ));
            }
        }

        // phase-403 step 1 -- the JOIN KEY. `nros_derive_message_bound_knobs`
        // prices a type; these two say which types this image RECEIVES, which
        // is the half it cannot know. Emitted in BOTH branches: a refusal here
        // is a fact a reader must act on, and an absent variable would read as
        // "no type is received" -- a payload class derived over an empty set.
        s.push_str(&render_received(
            "SUBSCRIBED",
            "the types SUBSCRIPTIONS receive. These and only these allocate from the\n\
             # backend's two topic payload classes (one `alloc_payload_block` call site,\n\
             # reached only from `declare_subscriber`), so they are the join key for\n\
             # NROS_SUBSCRIBER_BUFFER_SIZE / _LARGE_SIZE / NROS_MAX_LARGE_SUBSCRIBERS.\n\
             # The count is per ENTITY, not per type: two subscriptions on one large type\n\
             # need two blocks.",
            &self.subscribed_types(),
        ));
        s.push_str(&render_received(
            "RECEIVED",
            "every type this image receives, over every receiving kind -- a service\n\
             # SERVER receives requests, a service CLIENT receives replies, and an action\n\
             # server and action client each receive three things. WIDER than the\n\
             # subscribed set above; it is what the executor arena needs, not the payload\n\
             # classes.",
            &self.received_types(),
        ));
        // phase-403 step 2 -- the QoS DEPTHS. The arena's per-subscription cost
        // is `(depth + 1) * bound + (depth + 1) * 8`, so depth is a MULTIPLIER
        // on the type bound the two views above supply, and no arena can derive
        // without it. Emitted in both branches for the same reason they are: an
        // absent variable would read as "every endpoint is depth 0".
        s.push_str(&render_declared_depths(&self.declared_depths()));
        s
    }

    /// The environment projection -- the carrier that reaches a cargo build.
    ///
    /// One `KEY=VALUE` per line, and NOTHING when the derivation refused: an
    /// absent variable leaves `nros-node/build.rs` on its own default, which is
    /// rung 4 of the precedence ladder and the correct outcome for "no answer".
    pub fn to_env(&self) -> String {
        match self.derive() {
            // Issue 0900 -- both, or neither. `NROS_EXECUTOR_ACTION_CLIENTS`
            // is only meaningful against the `MAX_CBS` it is clamped to, and
            // emitting one without the other would size an arena against a
            // slot count from a different rung.
            Derivation::Derived(k) => format!(
                "NROS_EXECUTOR_MAX_CBS={}\nNROS_EXECUTOR_ACTION_CLIENTS={}\n",
                k.max_cbs, k.heavy_slots
            ),
            Derivation::Refused { .. } => String::new(),
        }
    }
}

/// One received-type view, as CMake.
///
/// `NROS_ENTITY_<WHAT>_TYPES_STATUS` is always set, so a reader can tell
/// "resolved to nothing" from "refused" from "this fragment predates the
/// field" -- three states that license three different actions and that an
/// absent list collapses into one.
fn render_received(what: &str, prose: &str, r: &ReceivedTypes) -> String {
    let mut s = format!("# {prose}\n");
    s.push_str(&format!(
        "set(NROS_ENTITY_{what}_TYPES_STATUS \"{}\")\n",
        r.tag()
    ));
    match r {
        ReceivedTypes::Refused { reason } => {
            s.push_str(&format!(
                "set(NROS_ENTITY_{what}_TYPES_REASON \"{}\")\n",
                cmake_escape(reason)
            ));
            s.push_str(&format!(
                "# No {} type set. A consumer that needs one must REFUSE, never fall back\n\
                 # to a wider set -- a wider set is a different question with a different\n\
                 # answer.\n",
                what.to_ascii_lowercase()
            ));
        }
        ReceivedTypes::Resolved(v) => {
            let names: Vec<&str> = v.iter().map(|(t, _)| t.as_str()).collect();
            s.push_str(&format!(
                "set(NROS_ENTITY_{what}_TYPES \"{}\")\n",
                names.join(";")
            ));
            let pairs: Vec<String> = v.iter().map(|(t, n)| format!("{t}={n}")).collect();
            s.push_str(&format!(
                "set(NROS_ENTITY_{what}_TYPE_COUNTS \"{}\")\n",
                pairs.join(";")
            ));
            let total: usize = v.iter().map(|(_, n)| *n).sum();
            s.push_str(&format!("set(NROS_ENTITY_{what}_ENTITY_COUNT {total})\n"));
        }
    }
    s
}

/// The declared-depth view, as CMake (phase-403 step 2).
///
/// Publishes four things, and the last two are the point:
///
///   `NROS_ENTITY_DECLARED_DEPTHS`        `type|topic=depth` triples, `;`-joined
///   `NROS_ENTITY_DECLARED_DEPTH_COUNT`   how many endpoints stated one
///   `NROS_ENTITY_UNDECLARED_DEPTH_COUNT` how many COULD have and did not
///   `NROS_ENTITY_DECLARED_DEPTH_STATUS`  resolved | refused
///
/// A consumer that sizes from depth must read the UNDECLARED count and refuse
/// when it is non-zero. Reading only the list would size an image from the
/// subset of its endpoints that happened to be annotated, which is the exact
/// under-report `ENTITIES NONE` exists to prevent one level up.
fn render_declared_depths(d: &DeclaredDepths) -> String {
    let mut s = String::from(
        "# phase-403 step 2 -- the DECLARED QoS history depths. Depth is a MULTIPLIER on\n\
         # the type bound above: the arena's per-subscription cost is\n\
         # `(depth + 1) * bound + (depth + 1) * 8`, measured at 86108 bytes for ten\n\
         # subscriptions at the ROS default depth 10 and 24516 at depth 1.\n\
         # A consumer that sizes from these must refuse while\n\
         # NROS_ENTITY_UNDECLARED_DEPTH_COUNT is non-zero: an endpoint that stated no\n\
         # depth has not opted in, and BOTH defaults are wrong -- 10 inflates an image\n\
         # that meant 1 tenfold, and 1 under-sizes one that took the default.\n",
    );
    s.push_str(&format!(
        "set(NROS_ENTITY_DECLARED_DEPTH_STATUS \"{}\")\n",
        d.tag()
    ));
    match d {
        DeclaredDepths::Refused { reason } => {
            s.push_str(&format!(
                "set(NROS_ENTITY_DECLARED_DEPTH_REASON \"{}\")\n",
                cmake_escape(reason)
            ));
            s.push_str(
                "# No depth table. A consumer that needs one must REFUSE -- a partial table\n\
                 # is indistinguishable from an image whose endpoints all took the default.\n",
            );
        }
        DeclaredDepths::Resolved { rows, undeclared } => {
            let triples: Vec<String> = rows
                .iter()
                .map(|r| format!("{}|{}={}", r.type_name, r.topic, r.depth))
                .collect();
            s.push_str(&format!(
                "set(NROS_ENTITY_DECLARED_DEPTHS \"{}\")\n",
                triples.join(";")
            ));
            s.push_str(&format!(
                "set(NROS_ENTITY_DECLARED_DEPTH_COUNT {})\n",
                rows.len()
            ));
            s.push_str(&format!(
                "set(NROS_ENTITY_UNDECLARED_DEPTH_COUNT {undeclared})\n"
            ));
        }
    }
    s
}

/// A C string literal's body. The table's keys are ROS type and topic names, so
/// in practice nothing here needs escaping -- which is exactly why it is done
/// anyway: an unescaped quote or backslash reaching a generated header is a
/// compile error in a file nobody edits, and the fix would be invisible.
fn c_escape(s: &str) -> String {
    s.replace('\\', "\\\\").replace('"', "\\\"")
}

/// CMake `set(... "...")` is quote- and backslash-sensitive, and a refusal
/// reason is multi-line prose.
fn cmake_escape(s: &str) -> String {
    s.replace('\\', "\\\\")
        .replace('"', "\\\"")
        .replace('\n', "\\n")
}

#[cfg(test)]
mod tests {
    use super::*;

    fn stated(pkg: &str, comp: &str, specs: &[&str]) -> ComponentEntities {
        let mut decls = Vec::new();
        for s in specs {
            decls.extend(EntityDecl::parse(s).expect("spec parses"));
        }
        ComponentEntities {
            pkg: pkg.to_string(),
            component: comp.to_string(),
            class: format!("{pkg}::{comp}"),
            declaration: Declaration::Stated(decls),
        }
    }

    /// The whole point: a publisher is declared, counted, and claims no slot.
    /// The two numbers differ and both are reported.
    #[test]
    fn a_publisher_is_inventoried_and_claims_no_callback_slot() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated("p", "n", &["pub*3", "sub", "timer"]));
        let d = inv.derive();
        let k = d.knobs().expect("derived");
        assert_eq!(k.entity_total, 5);
        assert_eq!(k.max_cbs, 2, "3 publishers claim no slot; sub + timer do");
        assert_eq!(k.per_kind["publisher"], 3);
    }

    /// The refusal that makes an under-report impossible. One un-annotated
    /// component and the WHOLE image derives nothing -- not a total three
    /// components' worth short.
    #[test]
    fn one_undeclared_component_refuses_the_whole_image() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated("a", "one", &["sub", "timer"]));
        inv.insert(ComponentEntities {
            pkg: "b".into(),
            component: "two".into(),
            class: "b::Two".into(),
            declaration: Declaration::Absent,
        });
        match inv.derive() {
            Derivation::Refused { reason } => {
                assert!(reason.contains("b::two"), "names the component: {reason}");
                assert!(
                    reason.contains("ENTITIES NONE"),
                    "names the remedy: {reason}"
                );
            }
            other => panic!("expected a refusal, got {other:?}"),
        }
        // And no transport may carry a number.
        assert!(!inv.to_cmake().contains("NROS_DERIVED_EXECUTOR_MAX_CBS"));
        assert_eq!(inv.to_env(), "");
    }

    /// Issue 0900 — the heavy-slot count is what stops a talker carrying an
    /// arena sized for an entity it does not have.
    ///
    /// Asserted through `to_env`, not just the struct field: the env projection
    /// is the only thing a build ever reads, and the derivation was correct for
    /// two phases while nothing lowered it.
    #[test]
    fn only_action_entities_claim_a_heavy_arena_slot() {
        // A talker: publisher (no slot at all) + timer. Nothing heavy.
        let mut talker = EntityInventory::new("test");
        talker.insert(stated("a", "talker", &["publisher", "timer"]));
        assert_eq!(
            talker.to_env(),
            "NROS_EXECUTOR_MAX_CBS=1\nNROS_EXECUTOR_ACTION_CLIENTS=0\n",
            "a pub/sub-only image must budget no slot at the action size"
        );

        // An action CLIENT is heavy.
        let mut client = EntityInventory::new("test");
        client.insert(stated("a", "client", &["timer", "action_client"]));
        assert_eq!(client.derive().knobs().expect("derived").heavy_slots, 1);

        // So is an action SERVER, though the knob is named for clients: the
        // arena stores `ActionServerArenaEntry`, so advising a server image to
        // zero the knob would trade the saving for `BufferTooSmall`.
        let mut server = EntityInventory::new("test");
        server.insert(stated("a", "server", &["action_server"]));
        assert_eq!(
            server.derive().knobs().expect("derived").heavy_slots,
            1,
            "an action server occupies a heavy slot too"
        );

        // A service client/server is NOT heavy — the nearest miss, and the one
        // a name-based rule would get wrong.
        let mut svc = EntityInventory::new("test");
        svc.insert(stated("a", "svc", &["service_client", "service_server"]));
        assert_eq!(svc.derive().knobs().expect("derived").heavy_slots, 0);
    }

    /// "Creates nothing" and "did not say" are different claims, and only the
    /// first one lets the image derive.
    #[test]
    fn an_explicit_none_is_an_answer_and_absence_is_not() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated("a", "one", &["sub"]));
        inv.insert(ComponentEntities {
            pkg: "b".into(),
            component: "two".into(),
            class: "b::Two".into(),
            declaration: Declaration::None,
        });
        let d = inv.derive();
        assert_eq!(d.knobs().expect("derived").max_cbs, 1);
    }

    /// An image with no components is a refusal, not a zero. A zero would be a
    /// perfectly plausible `MAX_CBS` and it would fail the first registration.
    #[test]
    fn an_empty_image_refuses_rather_than_deriving_zero() {
        let inv = EntityInventory::new("test");
        assert!(matches!(inv.derive(), Derivation::Refused { .. }));
        assert!(!inv.to_json().contains("\"max_cbs\""));
    }

    /// An unknown kind is an ERROR at parse time, never a skipped row: a
    /// skipped row is exactly an under-report wearing a typo.
    #[test]
    fn an_unknown_kind_is_rejected_rather_than_skipped() {
        let err = EntityDecl::parse("subscribtion:std_msgs/msg/Int32").unwrap_err();
        assert!(err.contains("unknown entity kind"), "{err}");
        assert!(err.contains("subscription"), "names the legal set: {err}");
    }

    #[test]
    fn a_declaration_carries_its_type_and_name() {
        let d = EntityDecl::parse("sub:nav_msgs/msg/Odometry:/localization/kinematic_state")
            .unwrap()
            .remove(0);
        assert_eq!(d.kind, EntityKind::Subscription);
        assert_eq!(d.type_name.as_deref(), Some("nav_msgs/msg/Odometry"));
        assert_eq!(d.name.as_deref(), Some("/localization/kinematic_state"));
    }

    #[test]
    fn a_repeat_count_of_zero_is_rejected() {
        assert!(EntityDecl::parse("timer*0").is_err());
    }

    // -----------------------------------------------------------------
    // phase-403 step 2 -- the QoS DEPTH.
    // -----------------------------------------------------------------

    /// The grammar, in the spelling the design fixed: a NAMED attribute after
    /// the positional fields, so reliability/history/durability can follow
    /// without another grammar change, and so a fourth positional can never be
    /// ambiguous against a topic.
    #[test]
    fn a_depth_attaches_as_a_named_attribute() {
        let d =
            EntityDecl::parse("sub:nav_msgs/msg/Odometry:/localization/kinematic_state@depth=10")
                .unwrap()
                .remove(0);
        assert_eq!(d.kind, EntityKind::Subscription);
        assert_eq!(d.type_name.as_deref(), Some("nav_msgs/msg/Odometry"));
        assert_eq!(
            d.name.as_deref(),
            Some("/localization/kinematic_state"),
            "the attribute must not be left on the end of the topic"
        );
        assert_eq!(d.depth, Some(10));
    }

    /// ABSENCE IS NOT ZERO -- the rule the whole inventory turns on, restated
    /// one level down. `None` means nobody said; a size consumer refuses on it.
    #[test]
    fn an_undeclared_depth_is_none_and_never_zero() {
        let d = EntityDecl::parse("sub:std_msgs/msg/Int32:/t")
            .unwrap()
            .remove(0);
        assert_eq!(d.depth, None);
        // And the spelling that WOULD collapse them is rejected outright: a
        // KEEP_LAST(0) holds no sample, so `@depth=0` is not a smaller queue,
        // it is a typo for "I did not want to say".
        let err = EntityDecl::parse("sub:std_msgs/msg/Int32:/t@depth=0").unwrap_err();
        assert!(err.contains("states nothing"), "{err}");
        assert!(err.contains("REFUSE"), "names what absence buys: {err}");
    }

    /// An unknown attribute is an ERROR, never a skipped one -- the same rule
    /// an unknown KIND follows, and for the same reason: a silently ignored
    /// attribute is a declaration the author believes they made.
    #[test]
    fn an_unknown_attribute_is_rejected_rather_than_ignored() {
        let err = EntityDecl::parse("sub:std_msgs/msg/Int32:/t@dpeth=1").unwrap_err();
        assert!(err.contains("unknown attribute"), "{err}");
        assert!(err.contains("depth"), "names the legal set: {err}");
        // A bare attribute with no value is a typo too, not a flag.
        assert!(EntityDecl::parse("sub:std_msgs/msg/Int32:/t@depth").is_err());
        assert!(EntityDecl::parse("sub:std_msgs/msg/Int32:/t@").is_err());
        assert!(EntityDecl::parse("sub:std_msgs/msg/Int32:/t@depth=ten").is_err());
        // One entity has one depth.
        assert!(EntityDecl::parse("sub:std_msgs/msg/Int32:/t@depth=1@depth=2").is_err());
    }

    /// A timer has no QoS, so a depth on one is a statement about nothing.
    /// Rejected rather than ignored -- an author who wrote it meant something.
    #[test]
    fn a_depth_on_a_kind_with_no_qos_is_rejected() {
        let err = EntityDecl::parse("timer@depth=5").unwrap_err();
        assert!(err.contains("has no QoS"), "{err}");
        assert!(EntityDecl::parse("guard@depth=5").is_err());
        // Every other kind carries one. A publisher's depth sizes no receive
        // buffer today, and forbidding it would make the grammar say something
        // false about the QoS a publisher really has.
        for k in ALL_ENTITY_KINDS {
            assert_eq!(
                k.carries_qos_depth(),
                !matches!(k, EntityKind::Timer | EntityKind::GuardCondition),
                "{} and QoS depth",
                k.tag()
            );
        }
    }

    /// A repeat count multiplies the whole row, depth included -- otherwise
    /// `sub*3:...@depth=1` would declare one sized endpoint and two silent ones.
    #[test]
    fn a_repeat_count_carries_the_depth_to_every_copy() {
        let d = EntityDecl::parse("sub*3:std_msgs/msg/Int32:/t@depth=2").unwrap();
        assert_eq!(d.len(), 3);
        assert!(d.iter().all(|e| e.depth == Some(2)));
    }

    /// The image-wide view, and the field that makes it honest: how many
    /// endpoints COULD have declared a depth and did not. Reporting only the
    /// rows would make a partly-annotated image look fully declared.
    #[test]
    fn the_declared_depth_view_counts_what_stayed_silent() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated(
            "a",
            "one",
            &[
                "sub:std_msgs/msg/Int32:/t@depth=1",
                "sub:nav_msgs/msg/Odometry:/k@depth=10",
                "sub:std_msgs/msg/Bool:/b",
                "pub:std_msgs/msg/Bool:/p",
                // No QoS at all, so not in the population either way.
                "timer*4",
            ],
        ));
        match inv.declared_depths() {
            DeclaredDepths::Resolved { rows, undeclared } => {
                assert_eq!(rows.len(), 2);
                assert_eq!(rows[0].type_name, "nav_msgs/msg/Odometry", "sorted");
                assert_eq!(rows[0].depth, 10);
                assert_eq!(
                    undeclared, 2,
                    "the untyped-depth subscription and the publisher; the four timers \
                     carry no QoS and are not in the population"
                );
            }
            other => panic!("expected resolved, got {other:?}"),
        }
    }

    /// An incomplete image has NO depth table, for the reason it has no type
    /// set: a missing row reads as "nobody declared this endpoint", and that is
    /// the one thing the table must never say wrongly -- it is what the
    /// compile-time check treats as "assert nothing".
    #[test]
    fn an_incomplete_image_has_no_depth_table() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated("a", "one", &["sub:std_msgs/msg/Int32:/t@depth=1"]));
        inv.insert(ComponentEntities {
            pkg: "b".into(),
            component: "two".into(),
            class: "b::Two".into(),
            declaration: Declaration::Absent,
        });
        assert!(matches!(
            inv.declared_depths(),
            DeclaredDepths::Refused { .. }
        ));
        let c = inv.to_cmake();
        assert!(c.contains("set(NROS_ENTITY_DECLARED_DEPTH_STATUS \"refused\")"));
        assert!(
            !c.contains("set(NROS_ENTITY_DECLARED_DEPTHS "),
            "a refusal must publish no depth list at all: {c}"
        );
        // ...and the generated header carries no rows either, so every call
        // site in that image compiles unchecked rather than against a partial
        // table.
        let h = inv.to_declared_qos_header();
        assert!(h.contains("NROS_DECLARED_QOS_STATUS \"refused\""));
        assert!(!h.contains("#define NROS_DECLARED_QOS_ROWS"));
    }

    /// The CMake projection carries the list AND the undeclared count.
    #[test]
    fn the_cmake_projection_carries_the_depths_and_what_stayed_silent() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated(
            "a",
            "one",
            &[
                "sub:std_msgs/msg/Int32:/t@depth=1",
                "sub:std_msgs/msg/Bool:/b",
            ],
        ));
        let c = inv.to_cmake();
        assert!(c.contains("set(NROS_ENTITY_DECLARED_DEPTHS \"std_msgs/msg/Int32|/t=1\")"));
        assert!(c.contains("set(NROS_ENTITY_DECLARED_DEPTH_COUNT 1)"));
        assert!(
            c.contains("set(NROS_ENTITY_UNDECLARED_DEPTH_COUNT 1)"),
            "the count of endpoints that said nothing is what a size consumer \
             refuses on: {c}"
        );
        // The canonical transport carries the same three facts.
        let j = inv.to_json();
        assert!(j.contains("\"declared_depths\""));
        assert!(j.contains("\"undeclared\": 1"));
        assert!(j.contains("\"dds_type_name\": \"std_msgs::msg::dds_::Int32_\""));
        // A row with no depth carries no `depth` key at all -- `"depth": 0` and
        // `"depth": null` would each make "nobody said" look like an answer.
        assert!(!j.contains("\"depth\": 0"));
        assert!(!j.contains("\"depth\": null"));
    }

    /// The C++ table: subscriptions only, both spellings of the type, and NO
    /// `NROS_DECLARED_QOS_ROWS` at all when nothing was declared.
    #[test]
    fn the_generated_header_emits_both_type_spellings_for_subscriptions() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated(
            "a",
            "one",
            &[
                "sub:std_msgs/msg/Int32:/chatter@depth=1",
                // A PUBLISHER with a declared depth is not in the table: it is
                // keyed `(type, topic)`, and a pub and a sub on one pair would
                // be two rows with one key. Nothing consults it yet either.
                "pub:std_msgs/msg/Int32:/chatter@depth=1",
            ],
        ));
        let h = inv.to_declared_qos_header();
        assert!(
            h.contains("NROS_DECLARED_QOS_ROW(\"std_msgs::msg::dds_::Int32_\", \"/chatter\", 1)")
        );
        assert!(h.contains("NROS_DECLARED_QOS_ROW(\"std_msgs/msg/Int32\", \"/chatter\", 1)"));
        assert!(
            h.contains("#define NROS_DECLARED_QOS_ROW_COUNT 2"),
            "one subscription, two spellings, and the publisher contributes none: {h}"
        );

        // Nothing declared: the macro stays UNDEFINED, so `declared_qos.hpp`
        // compiles an empty table. An empty ROWS list would be a different
        // claim, and C++ has no empty array anyway.
        let mut none = EntityInventory::new("test");
        none.insert(stated("a", "one", &["sub:std_msgs/msg/Int32:/t"]));
        let h = none.to_declared_qos_header();
        assert!(!h.contains("#define NROS_DECLARED_QOS_ROWS"));
        assert!(h.contains("#define NROS_DECLARED_QOS_UNDECLARED_COUNT 1"));
    }

    /// [`dds_type_name`] MIRRORS `packs/cpp/message.hpp.jinja`, so it is held
    /// to it. The two live in different crates and the C++ side is a Tera
    /// template, so this is the only thing standing between a mangling change
    /// and a table whose keys match nothing -- which would leave every
    /// `static_assert` in the tree vacuously true.
    #[test]
    fn the_dds_spelling_matches_the_cpp_template() {
        let tpl = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../rosidl-codegen/packs/cpp/message.hpp.jinja");
        let text =
            std::fs::read_to_string(&tpl).unwrap_or_else(|e| panic!("read {}: {e}", tpl.display()));
        assert!(
            text.contains(
                "static constexpr const char* TYPE_NAME = \
                 \"{{ package_name }}::msg::dds_::{{ message_name }}_\";"
            ),
            "the C++ codegen no longer emits the TYPE_NAME spelling `dds_type_name` \
             produces. Update BOTH, or the declared-QoS table keys on a name no \
             message class carries and every NROS_SUBSCRIBE assertion silently \
             passes.\n{}",
            tpl.display()
        );
        assert_eq!(
            dds_type_name("std_msgs/msg/Int32"),
            "std_msgs::msg::dds_::Int32_"
        );
        // Already mangled, or not a three-segment ROS name: passed through.
        // Guessing a mangling for a shape the codegen does not emit would put a
        // key in the table that nothing can ever match.
        assert_eq!(
            dds_type_name("std_msgs::msg::dds_::Int32_"),
            "std_msgs::msg::dds_::Int32_"
        );
        assert_eq!(dds_type_name("Weird"), "Weird");
        assert_eq!(dds_type_name("a/b"), "a/b");
    }

    /// The CMake projection is `include()`able and composes: it sets the knob
    /// only when derived, and records the provenance either way.
    #[test]
    fn the_cmake_projection_sets_the_knob_only_when_derived() {
        let mut inv = EntityInventory::new("build/nros-metadata.json");
        inv.insert(stated("a", "one", &["sub*2", "pub*4", "timer"]));
        let c = inv.to_cmake();
        assert!(c.contains("set(NROS_ENTITY_INVENTORY_STATUS \"derived\")"));
        assert!(c.contains("set(NROS_DERIVED_EXECUTOR_MAX_CBS 3)"));
        assert!(c.contains("set(NROS_ENTITY_INVENTORY_ENTITY_TOTAL 7)"));
        assert!(c.contains("set(NROS_ENTITY_COUNT_PUBLISHER 4)"));
        assert!(c.contains("a::one = 7 entities, 3 slots"));
        assert!(
            c.contains(&format!(
                "set(NROS_ENTITY_INVENTORY_SCHEMA_VERSION {ENTITY_INVENTORY_SCHEMA_VERSION})"
            )),
            "a reader must be able to refuse an unrecognised schema"
        );
    }

    /// A refusal reason is multi-line prose from `derive()`; a raw newline
    /// inside `set(... "...")` is legal CMake but unreadable, and a stray quote
    /// would end the string early.
    #[test]
    fn a_refusal_reason_is_escaped_for_cmake() {
        let mut inv = EntityInventory::new("test");
        inv.insert(ComponentEntities {
            pkg: "b".into(),
            component: "two".into(),
            class: "b::\"Two\"".into(),
            declaration: Declaration::Absent,
        });
        let c = inv.to_cmake();
        let reason_line = c
            .lines()
            .find(|l| l.starts_with("set(NROS_ENTITY_INVENTORY_REASON"))
            .expect("a reason is published");
        assert!(!reason_line.contains("\\n\\n"), "no double escaping");
        assert!(
            reason_line.ends_with("\")"),
            "the string closes: {reason_line}"
        );
        assert!(reason_line.contains("\\\""), "the class quote is escaped");
    }

    /// The env transport is the cargo carrier and it is EMPTY on a refusal:
    /// an absent variable leaves `nros-node/build.rs` on its own default, which
    /// is rung 4 of the ladder.
    #[test]
    fn the_env_transport_is_empty_on_a_refusal() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated("a", "one", &["sub", "timer", "service_server"]));
        // Issue 0900 — both knobs travel together; none of these three kinds
        // is heavy, so the arena budgets no slot at the action size.
        assert_eq!(
            inv.to_env(),
            "NROS_EXECUTOR_MAX_CBS=3\nNROS_EXECUTOR_ACTION_CLIENTS=0\n"
        );
        inv.insert(ComponentEntities {
            pkg: "b".into(),
            component: "two".into(),
            class: "b::Two".into(),
            declaration: Declaration::Absent,
        });
        assert_eq!(inv.to_env(), "");
    }

    /// Registering the same component twice cannot double-count it: cmake
    /// re-runs `nano_ros_node_register` on every configure, and a workspace
    /// that reaches one package through two `add_subdirectory()` paths is a
    /// shape this tree already has.
    #[test]
    fn a_component_recorded_twice_is_recorded_once() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated("a", "one", &["sub*3"]));
        inv.insert(stated("a", "one", &["sub*3"]));
        assert_eq!(inv.len(), 1);
        assert_eq!(inv.derive().knobs().unwrap().max_cbs, 3);
    }

    /// Emission order is stable, so a write-if-changed consumer does not
    /// re-arm a reconfigure on every run.
    #[test]
    fn emission_order_is_stable() {
        let mut a = EntityInventory::new("test");
        a.insert(stated("z", "one", &["sub"]));
        a.insert(stated("a", "two", &["timer"]));
        let mut b = EntityInventory::new("test");
        b.insert(stated("a", "two", &["timer"]));
        b.insert(stated("z", "one", &["sub"]));
        assert_eq!(a.to_cmake(), b.to_cmake());
        assert_eq!(a.to_json(), b.to_json());
    }

    // -----------------------------------------------------------------
    // phase-403 step 1 -- the JOIN KEY.
    // -----------------------------------------------------------------

    /// Which kinds RECEIVE is the decision this step turns on, and the two
    /// predicates are deliberately different sets. Pinning both here is what
    /// stops someone "simplifying" one into the other: widening
    /// `receives_topic_sample` would price a service's request against a pool
    /// it never allocates from, and narrowing `receives` would leave the arena
    /// blind to four kinds that carry receive buffers.
    #[test]
    fn a_client_receives_and_a_publisher_does_not() {
        for k in [
            EntityKind::Subscription,
            EntityKind::ServiceServer,
            EntityKind::ServiceClient,
            EntityKind::ActionServer,
            EntityKind::ActionClient,
        ] {
            assert!(k.receives(), "{} receives a payload", k.tag());
        }
        for k in [
            EntityKind::Publisher,
            EntityKind::Timer,
            EntityKind::GuardCondition,
        ] {
            assert!(!k.receives(), "{} receives nothing", k.tag());
        }
        // Only a subscription draws from the topic payload pools -- one
        // `alloc_payload_block` call site, reached only from
        // `declare_subscriber`.
        for k in ALL_ENTITY_KINDS {
            assert_eq!(
                k.receives_topic_sample(),
                *k == EntityKind::Subscription,
                "{} and the payload pools",
                k.tag()
            );
        }
    }

    /// The join key counts ENTITIES, not distinct types. Two subscriptions on
    /// one large type need two large payload BLOCKS, and a deduplicated type
    /// set would reserve one.
    #[test]
    fn the_subscribed_set_counts_entities_per_type() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated(
            "a",
            "one",
            &[
                "sub:std_msgs/msg/Int32:/a",
                "sub:std_msgs/msg/Int32:/b",
                "sub:nav_msgs/msg/Odometry:/c",
                "pub:sensor_msgs/msg/Image:/d",
                "timer",
            ],
        ));
        let types = inv.subscribed_types();
        assert_eq!(
            types.types().expect("resolved"),
            &[
                ("nav_msgs/msg/Odometry".to_string(), 1),
                ("std_msgs/msg/Int32".to_string(), 2),
            ],
            "two subscriptions on Int32, one on Odometry, and the PUBLISHED \
             Image is not in the set"
        );
    }

    /// A service SERVER receives requests and a service CLIENT receives
    /// replies, so both are in the wider set -- and neither is in the payload
    /// pools' set. The two views are what keep the arena (step 3) from
    /// re-deriving this and disagreeing.
    #[test]
    fn the_received_set_is_wider_than_the_subscribed_one() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated(
            "a",
            "one",
            &[
                "sub:std_msgs/msg/Int32:/t",
                "service_server:demo/srv/Op_Request:/s",
                "service_client:demo/srv/Op_Response:/c",
                "pub:std_msgs/msg/Bool:/p",
            ],
        ));
        let sub: Vec<String> = inv
            .subscribed_types()
            .types()
            .expect("resolved")
            .iter()
            .map(|(t, _)| t.clone())
            .collect();
        assert_eq!(sub, vec!["std_msgs/msg/Int32".to_string()]);
        let recv: Vec<String> = inv
            .received_types()
            .types()
            .expect("resolved")
            .iter()
            .map(|(t, _)| t.clone())
            .collect();
        assert_eq!(
            recv,
            vec![
                "demo/srv/Op_Request".to_string(),
                "demo/srv/Op_Response".to_string(),
                "std_msgs/msg/Int32".to_string(),
            ],
            "a server's request and a client's reply are both received"
        );
        // And the publisher is in neither.
        assert!(!recv.contains(&"std_msgs/msg/Bool".to_string()));
    }

    /// An untyped SUBSCRIPTION refuses the set rather than being skipped. A
    /// skipped row is an under-report, and here it would size a payload class
    /// from the types that happened to be annotated.
    #[test]
    fn an_untyped_subscription_refuses_the_set_but_not_the_slot_count() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated("a", "one", &["sub:std_msgs/msg/Int32:/t", "sub"]));
        match inv.subscribed_types() {
            ReceivedTypes::Refused { reason } => {
                assert!(reason.contains("a::one"), "names the component: {reason}");
                assert!(reason.contains("subscription"), "names the kind: {reason}");
            }
            other => panic!("expected a refusal, got {other:?}"),
        }
        // MAX_CBS does not need the type, so it still derives -- the two
        // questions have different inputs and different answers.
        assert_eq!(inv.derive().knobs().expect("derived").max_cbs, 2);
        // A timer has no type and that is not a refusal.
        let mut ok = EntityInventory::new("test");
        ok.insert(stated(
            "a",
            "one",
            &["sub:std_msgs/msg/Int32:/t", "timer*3"],
        ));
        assert!(ok.subscribed_types().types().is_some());
    }

    /// An image whose composition refused has NO subscribed set either. The
    /// un-annotated component's subscriptions are unknown, so a set composed
    /// over the rest is a subset of what the image receives -- and a payload
    /// class derived from a subset is too small.
    #[test]
    fn an_incomplete_image_has_no_subscribed_set() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated("a", "one", &["sub:std_msgs/msg/Int32:/t"]));
        inv.insert(ComponentEntities {
            pkg: "b".into(),
            component: "two".into(),
            class: "b::Two".into(),
            declaration: Declaration::Absent,
        });
        assert!(matches!(
            inv.subscribed_types(),
            ReceivedTypes::Refused { .. }
        ));
        let c = inv.to_cmake();
        assert!(c.contains("set(NROS_ENTITY_SUBSCRIBED_TYPES_STATUS \"refused\")"));
        assert!(
            !c.contains("set(NROS_ENTITY_SUBSCRIBED_TYPES "),
            "a refusal must publish no set at all: {c}"
        );
    }

    /// An image that declares entities and subscribes to NOTHING resolves to
    /// an EMPTY set, which is an answer and not a refusal: its payload pools
    /// are genuinely unused. The status is what tells the two apart, so the
    /// fragment must always carry one.
    #[test]
    fn a_subscriber_less_image_resolves_to_an_empty_set() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated("a", "one", &["pub:std_msgs/msg/Bool:/p", "timer"]));
        assert_eq!(inv.subscribed_types().types().expect("resolved").len(), 0);
        let c = inv.to_cmake();
        assert!(c.contains("set(NROS_ENTITY_SUBSCRIBED_TYPES_STATUS \"resolved\")"));
        assert!(c.contains("set(NROS_ENTITY_SUBSCRIBED_TYPES \"\")"));
        assert!(c.contains("set(NROS_ENTITY_SUBSCRIBED_ENTITY_COUNT 0)"));
    }

    /// The CMake projection carries the join key in the shape
    /// `_nros_bounds_join_subscribed` parses: a `;` list of names and a
    /// parallel `;` list of `type=count`.
    #[test]
    fn the_cmake_projection_carries_the_join_key() {
        let mut inv = EntityInventory::new("test");
        inv.insert(stated(
            "a",
            "one",
            &[
                "sub:nav_msgs/msg/Odometry:/k",
                "sub:std_msgs/msg/Int32:/t",
                "sub:std_msgs/msg/Int32:/u",
            ],
        ));
        let c = inv.to_cmake();
        assert!(c.contains(
            "set(NROS_ENTITY_SUBSCRIBED_TYPES \"nav_msgs/msg/Odometry;std_msgs/msg/Int32\")"
        ));
        assert!(c.contains(
            "set(NROS_ENTITY_SUBSCRIBED_TYPE_COUNTS \"nav_msgs/msg/Odometry=1;std_msgs/msg/Int32=2\")"
        ));
        assert!(c.contains("set(NROS_ENTITY_SUBSCRIBED_ENTITY_COUNT 3)"));
    }

    /// The measured island. Its four components, exactly as their ctors read
    /// today, and the two numbers the bring-up conflated.
    ///
    /// 33 entities is what a human counts and what
    /// `docs/roadmap/phase-3-canhubk344-real-silicon.md` recorded; 19 is the
    /// callback-slot demand, because the 14 publishers claim no slot. The board
    /// `.conf` pins 36.
    #[test]
    fn the_island_derives_nineteen_slots_from_thirty_three_entities() {
        let mut inv = EntityInventory::new("island");
        inv.insert(stated(
            "autoware_mrm_handler",
            "mrm_handler",
            &["sub*7", "pub*5", "service_client*2", "timer"],
        ));
        inv.insert(stated(
            "autoware_stop_mode_operator",
            "stop_mode_operator",
            &["pub*4", "sub*3", "timer"],
        ));
        inv.insert(stated(
            "autoware_mrm_comfortable_stop_operator",
            "mrm_comfortable_stop_operator",
            &["service_server", "pub*3", "timer"],
        ));
        inv.insert(stated(
            "autoware_mrm_emergency_stop_operator",
            "mrm_emergency_stop_operator",
            &["sub", "service_server", "pub*2", "timer"],
        ));
        let k = inv.derive().knobs().expect("derived").clone();
        assert_eq!(k.entity_total, 33);
        assert_eq!(k.per_kind["publisher"], 14);
        assert_eq!(k.per_kind["subscription"], 11);
        assert_eq!(k.per_kind["timer"], 4);
        assert_eq!(k.per_kind["service_server"], 2);
        assert_eq!(k.per_kind["service_client"], 2);
        assert_eq!(k.max_cbs, 19);
    }
}

#[cfg(test)]
mod from_model_tests {
    use super::*;
    use ros_launch_manifest_model::SystemModel;

    /// phase-412 -- the island's own resolved model, cut down to the shape that
    /// matters: two nodes, three topics, one of them internal.
    ///
    /// Asserts the counts the pool derivation consumes, not the parse: the
    /// question is whether `structure.topics` yields the same per-node sub/pub
    /// sets the hand-written `ENTITIES` lists did.
    fn model_from_yaml(y: &str) -> SystemModel {
        serde_yaml_ng::from_str(y).expect("model fixture parses")
    }

    #[test]
    fn topics_become_per_node_subscriptions_and_publishers() {
        let m = model_from_yaml(
            r#"
meta: { version: 1 }
structure:
  nodes:
    /mrm_handler:
      { scope: s.launch.xml, pkg: autoware_mrm_handler, exec: mrm_handler,
        node_name: mrm_handler }
    /stop_mode_operator:
      { scope: s.launch.xml, pkg: autoware_stop_mode_operator,
        exec: stop_mode_operator, node_name: stop_mode_operator }
  topics:
    /system/mrm/emergency_stop/status:
      type: tier4_system_msgs/msg/MrmBehaviorStatus
      sub: [/mrm_handler/emergency_stop_status]
    /api/operation_mode/state:
      type: autoware_adapi_v1_msgs/msg/OperationModeState
      sub: [/mrm_handler/operation_mode_state]
    /system/stop_mode/control:
      type: autoware_control_msgs/msg/Control
      pub: [/stop_mode_operator/control]
"#,
        );
        let inv = EntityInventory::from_model("test", &m).expect("model describes wiring");
        let d = inv.derive();
        let k = d.knobs().expect("wiring yields knobs");
        assert_eq!(k.max_subscribers, 2, "two subscriptions across the image");
        assert_eq!(k.max_publishers, 1, "one publisher");
    }

    /// A model that describes NO wiring must abstain, never report zero.
    ///
    /// Every launch file in this tree without a contract resolves that way, and
    /// reporting 0 would size each pool to the infrastructure alone and exhaust
    /// it the moment a node registers -- a confident wrong number, which is the
    /// failure shape this module exists to prevent.
    #[test]
    fn a_model_without_wiring_abstains() {
        let m = model_from_yaml(
            r#"
meta: { version: 1 }
structure:
  nodes:
    /talker:
      { scope: s.launch.xml, pkg: demo, exec: talker, node_name: talker }
"#,
        );
        assert!(
            EntityInventory::from_model("test", &m).is_none(),
            "no contract authored means unanswered, not zero"
        );
    }
}
