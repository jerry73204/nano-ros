//! Node — borrows the session to create typed entities.

use core::marker::PhantomData;

use nros_core::{RosAction, RosMessage, RosService};
use nros_rmw::{ActionInfo, QoSProfile, ServiceInfo, Session as _, TopicInfo, TransportError};

use crate::{
    rmw_type_registry::{MessageForRmw, register_type},
    session,
};

use super::{
    handles::{
        ActionClient, ActionClientCallback, ActionServer, EmbeddedPublisher, EmbeddedServiceClient,
        EmbeddedServiceServer, ServiceClientCallback, Subscription,
    },
    types::NodeError,
};

// ============================================================================
// Node
// ============================================================================

/// Backend-agnostic node — borrows the session to create typed entities.
pub struct NodeHandle<'a> {
    name: heapless::String<64>,
    namespace: heapless::String<64>,
    session: &'a mut session::ConcreteSession,
    domain_id: u32,
    /// Phase 211.H — per-node QoS overrides lowered from the launch
    /// `qos_overrides.<topic>.<role>.<policy>` params and baked into a
    /// `&'static` table by the entry codegen. Folded into each entity's
    /// `QoSProfile` at `create_publisher`/`create_subscription` time
    /// (setup-time, no alloc). Empty (`&[]`) by default → zero cost for
    /// systems without overrides.
    qos_overrides: &'static [nros_rmw::QoSOverride],
    /// RFC-0052 W3b.4 — baked contract-monitor table (mirror of
    /// `qos_overrides`): `create_publisher` attaches the matching
    /// endpoint's counter cell so `publish` can bump it lock-free.
    monitors: &'static [crate::executor::monitor::MonitorSpec],
    /// W3b.5 — baked subscriber age-contract table + epoch clock;
    /// `create_subscription` attaches the matching endpoint's age cell.
    age_monitors: &'static [crate::executor::monitor::AgeMonitorSpec],
    epoch_us_fn: Option<fn() -> u64>,
}

impl<'a> NodeHandle<'a> {
    /// Create a new node (called by Executor::create_node).
    pub(crate) fn new(
        name: heapless::String<64>,
        namespace: heapless::String<64>,
        session: &'a mut session::ConcreteSession,
        domain_id: u32,
    ) -> Self {
        Self {
            name,
            namespace,
            session,
            domain_id,
            qos_overrides: &[],
            monitors: &[],
            age_monitors: &[],
            epoch_us_fn: None,
        }
    }

    /// Phase 211.H — install the plan's QoS-override table on this node. Called
    /// by the generated entry BEFORE the component constructs its entities, so
    /// `create_publisher`/`create_subscription` fold the matching overrides in.
    /// The table is `&'static` (codegen bakes it as a `static`), so there is no
    /// lifetime to thread and no runtime allocation. Plan = authority: an
    /// override for a topic the entity creates is applied transparently (the
    /// user's `create_publisher(topic)` call is unchanged, matching rclcpp).
    pub fn set_qos_overrides(&mut self, overrides: &'static [nros_rmw::QoSOverride]) {
        self.qos_overrides = overrides;
    }

    /// RFC-0052 W3b.4 — install the executor's monitor table on this node
    /// (called by the entry glue / fixture alongside `set_qos_overrides`).
    pub fn set_monitors(&mut self, monitors: &'static [crate::executor::monitor::MonitorSpec]) {
        self.monitors = monitors;
    }

    /// W3b.5 — install the subscriber age-contract table + epoch clock
    /// (auto-seeded from the executor's `set_age_table` / config epoch).
    pub fn set_age_monitors(
        &mut self,
        table: &'static [crate::executor::monitor::AgeMonitorSpec],
        epoch_us: Option<fn() -> u64>,
    ) {
        self.age_monitors = table;
        self.epoch_us_fn = epoch_us;
    }

    /// The installed QoS-override table (empty unless the entry set one).
    #[must_use]
    pub fn qos_overrides(&self) -> &'static [nros_rmw::QoSOverride] {
        self.qos_overrides
    }

    /// Get the node name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// RFC-0088 — the serialization format this node's entities speak.
    ///
    /// The counterpart to ROS 2's `rmw_get_serialization_format()`, asked of
    /// the node rather than the process: an `Executor::open_multi` image holds
    /// two sessions and has no single answer, which is the case the reserved
    /// vtable slot was cut for.
    ///
    /// A single-backend image already knows this at compile time — see
    /// [`crate::session::IMAGE_SERIALIZATION_FORMAT`], which is what the
    /// entity-creation assertions compare against. Use this accessor when the
    /// answer must be a value: a bridge, a diagnostic, a tool.
    pub fn serialization_format(&self) -> &'static str {
        nros_rmw::Session::serialization_format(&*self.session)
    }

    /// Phase 88.12 — return the [`nros_log::Logger`] keyed on the
    /// node name.
    ///
    /// Loggers are interned in nros-log's bounded global table
    /// ([`nros_log::MAX_LOGGERS`] slots). If the caller has
    /// pre-registered a `'static Logger` whose name matches this
    /// node's name (via [`nros_log::register_logger`]), this method
    /// returns that exact reference — so subsequent `nros_*!` calls
    /// share per-logger runtime threshold state with any other call
    /// site that resolves the same name. Otherwise the call returns
    /// [`nros_log::DEFAULT_LOGGER`], keeping the API total.
    ///
    /// ```ignore
    /// // Pre-register if you want a dedicated threshold:
    /// static MY_NODE_LOGGER: nros_log::Logger =
    ///     nros_log::Logger::new("my_node");
    /// nros_log::register_logger(&MY_NODE_LOGGER);
    ///
    /// // Inside any node-creating code:
    /// let logger = node.logger();
    /// nros_log::log_info!(logger, "started; domain = {}", node.domain_id());
    /// ```
    #[must_use]
    pub fn logger(&self) -> &'static nros_log::Logger {
        nros_log::get_logger(self.name())
    }

    /// Get the domain ID.
    pub fn domain_id(&self) -> u32 {
        self.domain_id
    }

    /// Set the domain ID.
    pub fn set_domain_id(&mut self, domain_id: u32) {
        self.domain_id = domain_id;
    }

    /// Get a mutable reference to the underlying session.
    pub fn session_mut(&mut self) -> &mut session::ConcreteSession {
        self.session
    }

    // ------------------------------------------------------------------
    // Routing-info builders (Phase 91.F)
    //
    // Every `create_*` below threads the same node identity (domain_id +
    // name + namespace) into a TopicInfo / ServiceInfo / ActionInfo. The
    // shape repeats verbatim ~12 times across this file. Centralised
    // here so a future change to the routing-info shape (e.g. adding a
    // `with_security_context`) updates one site instead of twelve, and
    // so the per-`create_*` function bodies focus on the parts that
    // actually differ between them.
    // ------------------------------------------------------------------

    // Associated fns (NOT `&self` methods) so the returned `*Info`
    // value's borrow tracks only the explicit `&str` arguments, not
    // the whole `Node`. A `&self` form would block the immediately-
    // following `self.session.create_*(&info, …)` mut borrow on the
    // `name` / `namespace` reborrow held inside the returned `*Info`,
    // because going through a method call hides the field-disjoint
    // path that lets `&self.name` + `&mut self.session` coexist.
    fn topic_info<'b>(
        domain_id: u32,
        node_name: &'b str,
        namespace: &'b str,
        topic_name: &'b str,
        type_name: &'b str,
        type_hash: &'b str,
    ) -> TopicInfo<'b> {
        TopicInfo::new(topic_name, type_name, type_hash)
            .with_domain(domain_id)
            .with_node_name(node_name)
            .with_namespace(namespace)
    }

    fn service_info<'b>(
        domain_id: u32,
        node_name: &'b str,
        namespace: &'b str,
        service_name: &'b str,
        type_name: &'b str,
        type_hash: &'b str,
    ) -> ServiceInfo<'b> {
        ServiceInfo::new(service_name, type_name, type_hash)
            .with_domain(domain_id)
            .with_node_name(node_name)
            .with_namespace(namespace)
    }

    fn action_info<'b>(
        domain_id: u32,
        action_name: &'b str,
        type_name: &'b str,
        type_hash: &'b str,
    ) -> ActionInfo<'b> {
        // Action root only needs the domain — per-channel ServiceInfo /
        // TopicInfo derived from action_info.{send_goal,cancel_goal,...}_key()
        // carry the full node identity via service_info() / topic_info().
        ActionInfo::new(action_name, type_name, type_hash).with_domain(domain_id)
    }

    // -- Publishers --

    /// Create a publisher for the given topic.
    pub fn create_publisher<M: MessageForRmw>(
        &mut self,
        topic_name: &str,
    ) -> Result<EmbeddedPublisher<M>, NodeError> {
        self.create_publisher_with_qos::<M>(topic_name, QoSProfile::default())
    }

    /// Create a publisher with custom QoS settings.
    pub fn create_publisher_with_qos<M: MessageForRmw>(
        &mut self,
        topic_name: &str,
        qos: QoSProfile,
    ) -> Result<EmbeddedPublisher<M>, NodeError> {
        // RFC-0088 / phase-421 W1 — the message's declared format must be the
        // one the linked backend speaks. Universal: the const lives on
        // `RosMessage`, which `MessageForRmw` requires under every backend.
        crate::format_check::assert_message_format::<M>();
        // Phase 212.K.7.6.b — under `rmw-cyclonedds`, ensure the runtime
        // type-descriptor exists before the cffi vtable creates the
        // entity. No-op for other RMWs.
        register_type::<M>()?;
        // Phase 211.H — fold any plan qos_overrides for this topic+publisher
        // into the profile (setup-time, no alloc) BEFORE validation, so an
        // override the backend can't honour still errors loudly below.
        let qos = qos.apply_overrides(
            topic_name,
            nros_rmw::QoSOverrideRole::Publisher,
            self.qos_overrides,
        );
        // Phase 108.B — synchronous QoS validation against backend's
        // `supported_qos_policies()` mask. No silent downgrade.
        qos.validate_against(nros_rmw::Session::supported_qos_policies(self.session))
            .map_err(NodeError::Transport)?;
        let topic = Self::topic_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            topic_name,
            <M as RosMessage>::TYPE_NAME,
            <M as RosMessage>::TYPE_HASH,
        );
        let handle = self
            .session
            .create_publisher(&topic, qos)
            .map_err(|_| NodeError::Transport(TransportError::PublisherCreationFailed))?;
        // RFC-0052 W3b.4 — attach the contracted endpoint's counter cell
        // (exact topic-name match against the baked table; None = free).
        let monitor = self
            .monitors
            .iter()
            .find(|m| m.topic == topic_name)
            .map(|m| m.cell);
        Ok(EmbeddedPublisher {
            handle,
            event_regs: crate::executor::handles::empty_event_regs(),
            monitor,
            epoch: self.epoch_us_fn,
            _phantom: PhantomData,
        })
    }

    /// Create a typeless publisher for non-ROS wire formats (e.g. PX4 uORB
    /// raw POD bytes, custom binary protocols). The caller supplies the
    /// `type_name` and `type_hash` strings used by backends that need them
    /// for liveliness/discovery; backends that don't (uORB) can pass any
    /// stable string.
    pub fn create_publisher_raw(
        &mut self,
        topic_name: &str,
        type_name: &str,
        type_hash: &str,
    ) -> Result<crate::executor::handles::EmbeddedRawPublisher, NodeError> {
        self.create_publisher_raw_with_qos(topic_name, type_name, type_hash, QoSProfile::default())
    }

    /// Typeless publisher with custom QoS.
    pub fn create_publisher_raw_with_qos(
        &mut self,
        topic_name: &str,
        type_name: &str,
        type_hash: &str,
        qos: QoSProfile,
    ) -> Result<crate::executor::handles::EmbeddedRawPublisher, NodeError> {
        // Phase 211.H — apply plan qos_overrides (publisher side) before validate.
        let qos = qos.apply_overrides(
            topic_name,
            nros_rmw::QoSOverrideRole::Publisher,
            self.qos_overrides,
        );
        qos.validate_against(nros_rmw::Session::supported_qos_policies(self.session))
            .map_err(NodeError::Transport)?;
        let topic = Self::topic_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            topic_name,
            type_name,
            type_hash,
        );
        let handle = self
            .session
            .create_publisher(&topic, qos)
            .map_err(|_| NodeError::Transport(TransportError::PublisherCreationFailed))?;
        Ok(crate::executor::handles::EmbeddedRawPublisher {
            handle,
            arena: crate::executor::handles::TxArena::new(),
            event_regs: crate::executor::handles::empty_event_regs(),
        })
    }

    /// Phase 189.M1 — the customizable publisher **builder** (the `clone` tier;
    /// see `docs/design/0022-entity-api-tiers.md`). Pick a mode with `.typed::<M>()`
    /// or `.generic(type, hash)`, set knobs (`.qos`), then `.build()`. The
    /// convenient `create_publisher` / `create_publisher_raw` are the `fork`
    /// tier — sugar over this with defaults.
    pub fn publisher<'t>(&mut self, topic: &'t str) -> PublisherBuilder<'_, 'a, 't> {
        PublisherBuilder {
            node: self,
            topic,
            qos: QoSProfile::default(),
        }
    }

    // -- Subscriptions --

    /// Create a subscription for the given topic.
    pub fn create_subscription<M: MessageForRmw>(
        &mut self,
        topic_name: &str,
    ) -> Result<Subscription<M>, NodeError> {
        self.create_subscription_sized::<M, { crate::config::DEFAULT_RX_BUF_SIZE }>(topic_name)
    }

    /// Create a subscription with custom buffer size.
    pub fn create_subscription_sized<M: MessageForRmw, const RX_BUF: usize>(
        &mut self,
        topic_name: &str,
    ) -> Result<Subscription<M, RX_BUF>, NodeError> {
        self.create_subscription_with_qos::<M, RX_BUF>(topic_name, QoSProfile::default())
    }

    /// Create a subscription with custom QoS and buffer size.
    pub fn create_subscription_with_qos<M: MessageForRmw, const RX_BUF: usize>(
        &mut self,
        topic_name: &str,
        qos: QoSProfile,
    ) -> Result<Subscription<M, RX_BUF>, NodeError> {
        // RFC-0088 / phase-421 W1 — the message's declared format must be the
        // one the linked backend speaks. Universal: the const lives on
        // `RosMessage`, which `MessageForRmw` requires under every backend.
        crate::format_check::assert_message_format::<M>();
        // Phase 212.K.7.6.b — see `create_publisher_with_qos`.
        register_type::<M>()?;
        // Phase 211.H — apply plan qos_overrides (subscription side) before validate.
        let qos = qos.apply_overrides(
            topic_name,
            nros_rmw::QoSOverrideRole::Subscription,
            self.qos_overrides,
        );
        qos.validate_against(nros_rmw::Session::supported_qos_policies(self.session))
            .map_err(NodeError::Transport)?;
        let topic = Self::topic_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            topic_name,
            <M as RosMessage>::TYPE_NAME,
            <M as RosMessage>::TYPE_HASH,
        );
        let handle = self
            .session
            .create_subscription(&topic, qos)
            .map_err(NodeError::Transport)?;
        // W3b.5 — attach the contracted endpoint's age cell (stamped
        // types only; needs an epoch source).
        let age_mon = match (<M as RosMessage>::STAMP_OFFSET, self.epoch_us_fn) {
            (Some(_), Some(epoch)) => self
                .age_monitors
                .iter()
                .find(|a| a.topic == topic_name)
                .map(|a| (a.cell, epoch)),
            _ => None,
        };
        Ok(Subscription {
            handle,
            buffer: [0u8; RX_BUF],
            event_regs: crate::executor::handles::empty_event_regs(),
            age_mon,
            _phantom: PhantomData,
        })
    }

    /// Create a typeless subscription. Caller decodes raw bytes themselves.
    pub fn create_subscription_raw(
        &mut self,
        topic_name: &str,
        type_name: &str,
        type_hash: &str,
    ) -> Result<crate::executor::handles::RawSubscription, NodeError> {
        self.create_subscription_raw_sized::<{ crate::config::DEFAULT_RX_BUF_SIZE }>(
            topic_name, type_name, type_hash,
        )
    }

    /// Typeless subscription with custom buffer size.
    pub fn create_subscription_raw_sized<const RX_BUF: usize>(
        &mut self,
        topic_name: &str,
        type_name: &str,
        type_hash: &str,
    ) -> Result<crate::executor::handles::RawSubscription<RX_BUF>, NodeError> {
        // Phase 211.H — apply plan qos_overrides (subscription side) before
        // validate, mirroring `create_publisher_raw_with_qos`. The raw entity
        // paths honour node overrides exactly like the typed ones — an
        // override the active RMW can't meet errors loudly, never silently.
        let qos = QoSProfile::default().apply_overrides(
            topic_name,
            nros_rmw::QoSOverrideRole::Subscription,
            self.qos_overrides,
        );
        qos.validate_against(nros_rmw::Session::supported_qos_policies(self.session))
            .map_err(NodeError::Transport)?;
        let topic = Self::topic_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            topic_name,
            type_name,
            type_hash,
        );
        let handle = self
            .session
            .create_subscription(&topic, qos)
            .map_err(NodeError::Transport)?;
        Ok(crate::executor::handles::RawSubscription {
            handle,
            buffer: [0u8; RX_BUF],
            event_regs: crate::executor::handles::empty_event_regs(),
        })
    }

    // -- Services --

    /// Create a service server.
    pub fn create_service<Svc: RosService>(
        &mut self,
        service_name: &str,
    ) -> Result<EmbeddedServiceServer<Svc>, NodeError>
    where
        Svc::Request: MessageForRmw,
        Svc::Reply: MessageForRmw,
    {
        self.create_service_sized::<Svc, { crate::config::DEFAULT_RX_BUF_SIZE }, { crate::config::DEFAULT_RX_BUF_SIZE }>(service_name, QoSProfile::services_default())
    }

    /// Phase 193.2b — service server with an explicit QoS profile (applied to
    /// both the request + reply endpoints; rclcpp's `create_service(name, qos)`).
    pub fn create_service_with_qos<Svc: RosService>(
        &mut self,
        service_name: &str,
        qos: QoSProfile,
    ) -> Result<EmbeddedServiceServer<Svc>, NodeError>
    where
        Svc::Request: MessageForRmw,
        Svc::Reply: MessageForRmw,
    {
        self.create_service_sized::<Svc, { crate::config::DEFAULT_RX_BUF_SIZE }, { crate::config::DEFAULT_RX_BUF_SIZE }>(service_name, qos)
    }

    /// Create a service server with custom buffer sizes + QoS.
    pub fn create_service_sized<Svc: RosService, const REQ_BUF: usize, const REPLY_BUF: usize>(
        &mut self,
        service_name: &str,
        qos: QoSProfile,
    ) -> Result<EmbeddedServiceServer<Svc, REQ_BUF, REPLY_BUF>, NodeError>
    where
        Svc::Request: MessageForRmw,
        Svc::Reply: MessageForRmw,
    {
        // Phase 212.K.7.6.b — register both halves of the service round-trip
        // under cyclonedds. No-op for other RMWs.
        register_type::<Svc::Request>()?;
        register_type::<Svc::Reply>()?;
        // Phase 193.5 — validate the service profile against the backend's
        // supported policies (mirrors pub/sub); no silent downgrade. RELIABLE is
        // effectively required for request/reply, so a backend that only honours
        // a fixed profile rejects an incompatible request here.
        qos.validate_against(nros_rmw::Session::supported_qos_policies(self.session))
            .map_err(NodeError::Transport)?;
        let info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            service_name,
            Svc::SERVICE_NAME,
            Svc::SERVICE_HASH,
        );
        let handle = self
            .session
            .create_service(&info, qos)
            .map_err(NodeError::Transport)?;
        Ok(EmbeddedServiceServer {
            handle,
            req_buffer: [0u8; REQ_BUF],
            reply_buffer: [0u8; REPLY_BUF],
            _phantom: PhantomData,
        })
    }

    /// Create a service client.
    pub fn create_client<Svc: RosService>(
        &mut self,
        service_name: &str,
    ) -> Result<EmbeddedServiceClient<Svc>, NodeError>
    where
        Svc::Request: MessageForRmw,
        Svc::Reply: MessageForRmw,
    {
        self.create_client_sized::<Svc, { crate::config::DEFAULT_RX_BUF_SIZE }, { crate::config::DEFAULT_RX_BUF_SIZE }>(service_name, QoSProfile::services_default())
    }

    /// Phase 193.2b — service client with an explicit QoS profile.
    pub fn create_client_with_qos<Svc: RosService>(
        &mut self,
        service_name: &str,
        qos: QoSProfile,
    ) -> Result<EmbeddedServiceClient<Svc>, NodeError>
    where
        Svc::Request: MessageForRmw,
        Svc::Reply: MessageForRmw,
    {
        self.create_client_sized::<Svc, { crate::config::DEFAULT_RX_BUF_SIZE }, { crate::config::DEFAULT_RX_BUF_SIZE }>(service_name, qos)
    }

    /// Create a service client with custom buffer sizes + QoS.
    pub fn create_client_sized<Svc: RosService, const REQ_BUF: usize, const REPLY_BUF: usize>(
        &mut self,
        service_name: &str,
        qos: QoSProfile,
    ) -> Result<EmbeddedServiceClient<Svc, REQ_BUF, REPLY_BUF>, NodeError>
    where
        Svc::Request: MessageForRmw,
        Svc::Reply: MessageForRmw,
    {
        // Phase 212.K.7.6.b — see `create_service_sized`.
        register_type::<Svc::Request>()?;
        register_type::<Svc::Reply>()?;
        // Phase 193.5 — validate against the backend's supported policies (no
        // silent downgrade); request/reply effectively requires RELIABLE.
        qos.validate_against(nros_rmw::Session::supported_qos_policies(self.session))
            .map_err(NodeError::Transport)?;
        let info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            service_name,
            Svc::SERVICE_NAME,
            Svc::SERVICE_HASH,
        );
        let handle = self
            .session
            .create_client(&info, qos)
            .map_err(|_| NodeError::Transport(TransportError::ServiceClientCreationFailed))?;
        Ok(EmbeddedServiceClient {
            handle,
            req_buffer: [0u8; REQ_BUF],
            reply_buffer: [0u8; REPLY_BUF],
            in_flight: false,
            _phantom: PhantomData,
        })
    }

    /// Typeless service server. L1 counterpart of [`create_service`]
    /// for the C / C++ FFI shims and callers that own their own
    /// scheduler. Returns a [`crate::executor::handles::RawServiceServer`]
    /// which polls request bytes directly.
    pub fn create_service_raw(
        &mut self,
        service_name: &str,
        type_name: &str,
        type_hash: &str,
    ) -> Result<crate::executor::handles::RawServiceServer, NodeError> {
        self.create_service_raw_sized::<
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
        >(service_name, type_name, type_hash)
    }

    /// Typeless service server with custom buffer sizes.
    pub fn create_service_raw_sized<const REQ_BUF: usize, const RESP_BUF: usize>(
        &mut self,
        service_name: &str,
        type_name: &str,
        type_hash: &str,
    ) -> Result<crate::executor::handles::RawServiceServer<REQ_BUF, RESP_BUF>, NodeError> {
        let info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            service_name,
            type_name,
            type_hash,
        );
        let handle = self
            .session
            .create_service(&info, QoSProfile::services_default())
            .map_err(NodeError::Transport)?;
        Ok(crate::executor::handles::RawServiceServer::new(handle))
    }

    /// Typeless service client. L1 counterpart of [`create_client`].
    pub fn create_client_raw(
        &mut self,
        service_name: &str,
        type_name: &str,
        type_hash: &str,
    ) -> Result<crate::executor::handles::RawServiceClient, NodeError> {
        self.create_client_raw_sized::<
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
        >(service_name, type_name, type_hash)
    }

    /// Typeless service client with custom buffer sizes.
    pub fn create_client_raw_sized<const REQ_BUF: usize, const REPLY_BUF: usize>(
        &mut self,
        service_name: &str,
        type_name: &str,
        type_hash: &str,
    ) -> Result<crate::executor::handles::RawServiceClient<REQ_BUF, REPLY_BUF>, NodeError> {
        let info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            service_name,
            type_name,
            type_hash,
        );
        let handle = self
            .session
            .create_client(&info, QoSProfile::services_default())
            .map_err(|_| NodeError::Transport(TransportError::ServiceClientCreationFailed))?;
        Ok(crate::executor::handles::RawServiceClient::new(handle))
    }

    // -- Actions --

    /// Phase 122.3.c.6 — typeless action server. Builds the 5
    /// transport channels (`send_goal` / `cancel_goal` / `get_result`
    /// services + `feedback` / `status` publishers) and returns the
    /// raw `ActionServerCore` directly. Caller owns scheduling —
    /// drives `try_recv_goal_request` / `publish_feedback_raw` /
    /// `complete_goal_raw` / `try_handle_cancel` /
    /// `try_handle_get_result_raw` on the returned core.
    pub fn create_action_server_raw(
        &mut self,
        action_name: &str,
        type_name: &str,
        type_hash: &str,
    ) -> Result<
        super::action_core::ActionServerCore<
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
            4,
        >,
        NodeError,
    > {
        self.create_action_server_raw_sized::<
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
            4,
        >(action_name, type_name, type_hash)
    }

    /// Typeless action server with custom buffer + goal-slot sizes.
    pub fn create_action_server_raw_sized<
        const GOAL_BUF: usize,
        const RESULT_BUF: usize,
        const FEEDBACK_BUF: usize,
        const MAX_GOALS: usize,
    >(
        &mut self,
        action_name: &str,
        type_name: &str,
        type_hash: &str,
    ) -> Result<
        super::action_core::ActionServerCore<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>,
        NodeError,
    > {
        let action_info = Self::action_info(self.domain_id, action_name, type_name, type_hash);

        // Issue 0454 / phase-354 W3 — per-CHANNEL DDS type, not the bare
        // action type (see `action_channel_type`). These two `_raw_sized`
        // constructors are the last holders of the phase-338 W3 defect: the
        // type name is baked into the keyexpr, so `…Fibonacci_` never matches a
        // peer advertising `…Fibonacci_SendGoal_`.
        let send_goal_type: heapless::String<256> =
            super::action_core::action_channel_type(type_name, "SendGoal");
        let send_goal_keyexpr: heapless::String<256> = action_info.send_goal_key();
        let send_goal_info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &send_goal_keyexpr,
            &send_goal_type,
            type_hash,
        );
        let send_goal_server = self
            .session
            .create_service(&send_goal_info, QoSProfile::services_default())
            .map_err(|_| NodeError::ActionCreationFailed)?;

        let cancel_goal_keyexpr: heapless::String<256> = action_info.cancel_goal_key();
        let cancel_goal_info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &cancel_goal_keyexpr,
            "action_msgs::srv::dds_::CancelGoal_",
            type_hash,
        );
        let cancel_goal_server = self
            .session
            .create_service(&cancel_goal_info, QoSProfile::services_default())
            .map_err(|_| NodeError::ActionCreationFailed)?;

        let get_result_type: heapless::String<256> =
            super::action_core::action_channel_type(type_name, "GetResult");
        let get_result_keyexpr: heapless::String<256> = action_info.get_result_key();
        let get_result_info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &get_result_keyexpr,
            &get_result_type,
            type_hash,
        );
        let get_result_server = self
            .session
            .create_service(&get_result_info, QoSProfile::services_default())
            .map_err(|_| NodeError::ActionCreationFailed)?;

        let feedback_type: heapless::String<256> =
            super::action_core::action_channel_type(type_name, "FeedbackMessage");
        let feedback_keyexpr: heapless::String<256> = action_info.feedback_key();
        let feedback_topic = Self::topic_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &feedback_keyexpr,
            &feedback_type,
            type_hash,
        );
        let feedback_publisher = self
            .session
            .create_publisher(&feedback_topic, QoSProfile::QOS_PROFILE_DEFAULT)
            .map_err(|_| NodeError::ActionCreationFailed)?;

        let status_keyexpr: heapless::String<256> = action_info.status_key();
        let status_topic = Self::topic_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &status_keyexpr,
            "action_msgs::msg::dds_::GoalStatusArray_",
            type_hash,
        );
        let status_publisher = self
            .session
            .create_publisher(&status_topic, QoSProfile::QOS_PROFILE_ACTION_STATUS_DEFAULT)
            .map_err(|_| NodeError::ActionCreationFailed)?;

        Ok(super::action_core::ActionServerCore {
            send_goal_server,
            cancel_goal_server,
            get_result_server,
            feedback_publisher,
            status_publisher,
            active_goals: heapless::Vec::new(),
            completed_results: heapless::Vec::new(),
            pending_get_results: heapless::Vec::new(),
            result_slab: [0u8; RESULT_BUF],
            result_slab_used: 0,
            goal_buffer: [0u8; GOAL_BUF],
            feedback_buffer: [0u8; FEEDBACK_BUF],
            cancel_buffer: [0u8; 256],
        })
    }

    /// Phase 122.3.c.6 — typeless action client. Same shape as
    /// `create_action_server_raw` but builds the 3 service clients
    /// + 1 feedback subscriber, returns the raw `ActionClientCore`.
    pub fn create_action_client_raw(
        &mut self,
        action_name: &str,
        type_name: &str,
        type_hash: &str,
    ) -> Result<
        super::action_core::ActionClientCore<
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
        >,
        NodeError,
    > {
        self.create_action_client_raw_sized::<
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
        >(action_name, type_name, type_hash)
    }

    /// Typeless action client with custom buffer sizes.
    pub fn create_action_client_raw_sized<
        const GOAL_BUF: usize,
        const RESULT_BUF: usize,
        const FEEDBACK_BUF: usize,
    >(
        &mut self,
        action_name: &str,
        type_name: &str,
        type_hash: &str,
    ) -> Result<super::action_core::ActionClientCore<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>, NodeError>
    {
        let action_info = Self::action_info(self.domain_id, action_name, type_name, type_hash);

        // Issue 0454 / phase-354 W3 — per-CHANNEL DDS type, not the bare
        // action type (see `action_channel_type`). These two `_raw_sized`
        // constructors are the last holders of the phase-338 W3 defect: the
        // type name is baked into the keyexpr, so `…Fibonacci_` never matches a
        // peer advertising `…Fibonacci_SendGoal_`.
        let send_goal_type: heapless::String<256> =
            super::action_core::action_channel_type(type_name, "SendGoal");
        let send_goal_keyexpr: heapless::String<256> = action_info.send_goal_key();
        let send_goal_info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &send_goal_keyexpr,
            &send_goal_type,
            type_hash,
        );
        let send_goal_client = self
            .session
            .create_client(&send_goal_info, QoSProfile::services_default())
            .map_err(|_| NodeError::ActionCreationFailed)?;

        let cancel_goal_keyexpr: heapless::String<256> = action_info.cancel_goal_key();
        let cancel_goal_info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &cancel_goal_keyexpr,
            "action_msgs::srv::dds_::CancelGoal_",
            type_hash,
        );
        let cancel_goal_client = self
            .session
            .create_client(&cancel_goal_info, QoSProfile::services_default())
            .map_err(|_| NodeError::ActionCreationFailed)?;

        let get_result_type: heapless::String<256> =
            super::action_core::action_channel_type(type_name, "GetResult");
        let get_result_keyexpr: heapless::String<256> = action_info.get_result_key();
        let get_result_info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &get_result_keyexpr,
            &get_result_type,
            type_hash,
        );
        let get_result_client = self
            .session
            .create_client(&get_result_info, QoSProfile::services_default())
            .map_err(|_| NodeError::ActionCreationFailed)?;

        let feedback_type: heapless::String<256> =
            super::action_core::action_channel_type(type_name, "FeedbackMessage");
        let feedback_keyexpr: heapless::String<256> = action_info.feedback_key();
        let feedback_topic = Self::topic_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &feedback_keyexpr,
            &feedback_type,
            type_hash,
        );
        let feedback_subscriber = self
            .session
            .create_subscription(&feedback_topic, QoSProfile::BEST_EFFORT)
            .map_err(|_| NodeError::ActionCreationFailed)?;

        Ok(super::action_core::ActionClientCore::new(
            send_goal_client,
            cancel_goal_client,
            get_result_client,
            feedback_subscriber,
        ))
    }

    /// Create an action server.
    pub fn create_action_server<A: RosAction>(
        &mut self,
        action_name: &str,
    ) -> Result<ActionServer<A>, NodeError>
    where
        A::Goal: MessageForRmw,
        A::Result: MessageForRmw,
        A::Feedback: MessageForRmw,
        A::SendGoalRequest: MessageForRmw,
        A::SendGoalResponse: MessageForRmw,
        A::GetResultRequest: MessageForRmw,
        A::GetResultResponse: MessageForRmw,
        A::FeedbackMessage: MessageForRmw,
    {
        self.create_action_server_sized::<A, { crate::config::DEFAULT_RX_BUF_SIZE }, { crate::config::DEFAULT_RX_BUF_SIZE }, { crate::config::DEFAULT_RX_BUF_SIZE }, 4>(action_name)
    }

    /// Create an action server with custom buffer sizes.
    pub fn create_action_server_sized<
        A: RosAction,
        const GOAL_BUF: usize,
        const RESULT_BUF: usize,
        const FEEDBACK_BUF: usize,
        const MAX_GOALS: usize,
    >(
        &mut self,
        action_name: &str,
    ) -> Result<ActionServer<A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>, NodeError>
    where
        A::Goal: MessageForRmw,
        A::Result: MessageForRmw,
        A::Feedback: MessageForRmw,
        A::SendGoalRequest: MessageForRmw,
        A::SendGoalResponse: MessageForRmw,
        A::GetResultRequest: MessageForRmw,
        A::GetResultResponse: MessageForRmw,
        A::FeedbackMessage: MessageForRmw,
    {
        // Phase 212.K.7.6.b + K.7.7.c — register the three user-facing
        // message types AND the five action-protocol envelope types under
        // cyclonedds. No-op for other RMWs. The envelopes are needed
        // because the action service shapes (`*_SendGoal_Request`,
        // `*_GetResult_Response`, …) are the actual on-wire CDR types,
        // and the C++ Cyclone bridge auto-prepends a cdds_request_header_t
        // for any TYPE_NAME ending `_Request`/`_Response`/`_Reply`.
        register_type::<A::Goal>()?;
        register_type::<A::Result>()?;
        register_type::<A::Feedback>()?;
        register_type::<A::SendGoalRequest>()?;
        register_type::<A::SendGoalResponse>()?;
        register_type::<A::GetResultRequest>()?;
        register_type::<A::GetResultResponse>()?;
        register_type::<A::FeedbackMessage>()?;
        // issue #234 — also register the fixed `action_msgs` protocol types
        // (`CancelGoal_{Request,Response}`, `GoalStatusArray`) the cancel service
        // + status publisher created below serialize. They are not `RosAction`
        // associated types (they live in `action_msgs`, which `nros-core` cannot
        // name), so the generated `impl RosAction::register_protocol_types` — which
        // routes them through the generic `nros_rmw::register_type_descriptor` seam —
        // registers them. Without this the cancel_goal service + status publisher
        // have no Cyclone descriptor → `ActionCreationFailed`. The callback executor
        // path (`executor/action.rs`) already did this; this node.rs path — the one
        // `create_action_server` materialises through — did not.
        A::register_protocol_types().map_err(|()| NodeError::ActionCreationFailed)?;
        let action_info =
            Self::action_info(self.domain_id, action_name, A::ACTION_NAME, A::ACTION_HASH);

        // Each underlying ServiceInfo / TopicInfo also carries the
        // node identity so the Zenoh shim declares a liveliness token
        // for it. Without `with_node_name` the shim's
        // `declare_entity_liveliness` short-circuits (`node_name.and_then`
        // → None) and `wait_for_action_server` has nothing to find.
        // Advertise the per-channel service / topic types ROS 2 matches on
        // (`<Action>_SendGoal` / `<Action>_GetResult` / `<Action>_FeedbackMessage`),
        // not the bare action type — see `action_core::action_service_base_type`.
        let send_goal_type = super::action_core::action_service_base_type(
            <A::SendGoalRequest as RosMessage>::TYPE_NAME,
            A::ACTION_NAME,
        );
        let get_result_type = super::action_core::action_service_base_type(
            <A::GetResultRequest as RosMessage>::TYPE_NAME,
            A::ACTION_NAME,
        );
        let feedback_type = <A::FeedbackMessage as RosMessage>::TYPE_NAME;

        let send_goal_keyexpr: heapless::String<256> = action_info.send_goal_key();
        let send_goal_info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &send_goal_keyexpr,
            send_goal_type,
            A::SEND_GOAL_SERVICE_HASH,
        );
        let send_goal_server = self
            .session
            .create_service(&send_goal_info, QoSProfile::services_default())
            .map_err(|_| NodeError::ActionCreationFailed)?;

        let cancel_goal_keyexpr: heapless::String<256> = action_info.cancel_goal_key();
        let cancel_goal_info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &cancel_goal_keyexpr,
            "action_msgs::srv::dds_::CancelGoal_",
            A::ACTION_HASH,
        );
        let cancel_goal_server = self
            .session
            .create_service(&cancel_goal_info, QoSProfile::services_default())
            .map_err(|_| NodeError::ActionCreationFailed)?;

        let get_result_keyexpr: heapless::String<256> = action_info.get_result_key();
        let get_result_info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &get_result_keyexpr,
            get_result_type,
            A::GET_RESULT_SERVICE_HASH,
        );
        let get_result_server = self
            .session
            .create_service(&get_result_info, QoSProfile::services_default())
            .map_err(|_| NodeError::ActionCreationFailed)?;

        let feedback_keyexpr: heapless::String<256> = action_info.feedback_key();
        let feedback_topic = Self::topic_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &feedback_keyexpr,
            feedback_type,
            <A::FeedbackMessage as RosMessage>::TYPE_HASH,
        );
        let feedback_publisher = self
            .session
            .create_publisher(&feedback_topic, QoSProfile::QOS_PROFILE_DEFAULT)
            .map_err(|_| NodeError::ActionCreationFailed)?;

        let status_keyexpr: heapless::String<256> = action_info.status_key();
        let status_topic = Self::topic_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &status_keyexpr,
            "action_msgs::msg::dds_::GoalStatusArray_",
            A::ACTION_HASH,
        );
        let status_publisher = self
            .session
            .create_publisher(&status_topic, QoSProfile::QOS_PROFILE_ACTION_STATUS_DEFAULT)
            .map_err(|_| NodeError::ActionCreationFailed)?;

        Ok(ActionServer {
            core: super::action_core::ActionServerCore {
                send_goal_server,
                cancel_goal_server,
                get_result_server,
                feedback_publisher,
                status_publisher,
                active_goals: heapless::Vec::new(),
                completed_results: heapless::Vec::new(),
                pending_get_results: heapless::Vec::new(),
                result_slab: [0u8; RESULT_BUF],
                result_slab_used: 0,
                goal_buffer: [0u8; GOAL_BUF],
                feedback_buffer: [0u8; FEEDBACK_BUF],
                cancel_buffer: [0u8; 256],
            },
            typed_goals: heapless::Vec::new(),
            completed_goals: heapless::Vec::new(),
        })
    }

    /// Create an action client.
    pub fn create_action_client<A: RosAction>(
        &mut self,
        action_name: &str,
    ) -> Result<ActionClient<A>, NodeError>
    where
        A::Goal: MessageForRmw,
        A::Result: MessageForRmw,
        A::Feedback: MessageForRmw,
        A::SendGoalRequest: MessageForRmw,
        A::SendGoalResponse: MessageForRmw,
        A::GetResultRequest: MessageForRmw,
        A::GetResultResponse: MessageForRmw,
        A::FeedbackMessage: MessageForRmw,
    {
        self.create_action_client_sized::<A, { crate::config::DEFAULT_RX_BUF_SIZE }, { crate::config::DEFAULT_RX_BUF_SIZE }, { crate::config::DEFAULT_RX_BUF_SIZE }>(action_name)
    }

    /// Create an action client with custom buffer sizes.
    pub fn create_action_client_sized<
        A: RosAction,
        const GOAL_BUF: usize,
        const RESULT_BUF: usize,
        const FEEDBACK_BUF: usize,
    >(
        &mut self,
        action_name: &str,
    ) -> Result<ActionClient<A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>, NodeError>
    where
        A::Goal: MessageForRmw,
        A::Result: MessageForRmw,
        A::Feedback: MessageForRmw,
        A::SendGoalRequest: MessageForRmw,
        A::SendGoalResponse: MessageForRmw,
        A::GetResultRequest: MessageForRmw,
        A::GetResultResponse: MessageForRmw,
        A::FeedbackMessage: MessageForRmw,
    {
        // Phase 212.K.7.6.b + K.7.7.c — see `create_action_server_sized`.
        register_type::<A::Goal>()?;
        register_type::<A::Result>()?;
        register_type::<A::Feedback>()?;
        register_type::<A::SendGoalRequest>()?;
        register_type::<A::SendGoalResponse>()?;
        register_type::<A::GetResultRequest>()?;
        register_type::<A::GetResultResponse>()?;
        register_type::<A::FeedbackMessage>()?;
        // issue #234 — register the `action_msgs` protocol types the cancel-goal
        // service client below serializes (`CancelGoal_{Request,Response}`; the impl
        // also registers `GoalStatusArray`, harmlessly unused client-side) via the
        // generic seam. Without it the cancel_goal client has no Cyclone descriptor
        // → `ActionCreationFailed`. Mirrors the server path.
        A::register_protocol_types().map_err(|()| NodeError::ActionCreationFailed)?;
        let action_info =
            Self::action_info(self.domain_id, action_name, A::ACTION_NAME, A::ACTION_HASH);

        // Mirror `create_action_server_sized`: thread node identity through
        // each underlying ServiceInfo / TopicInfo so the Zenoh shim
        // declares the matching client-side liveliness tokens (and so the
        // discovery wildcard built from `send_goal_info` ends up in the
        // same domain as the server's tokens).
        // Same per-channel typing as the server side so the client's requesters
        // and feedback reader match a real ROS 2 action server over DDS.
        let send_goal_type = super::action_core::action_service_base_type(
            <A::SendGoalRequest as RosMessage>::TYPE_NAME,
            A::ACTION_NAME,
        );
        let get_result_type = super::action_core::action_service_base_type(
            <A::GetResultRequest as RosMessage>::TYPE_NAME,
            A::ACTION_NAME,
        );
        let feedback_type = <A::FeedbackMessage as RosMessage>::TYPE_NAME;

        let send_goal_keyexpr: heapless::String<256> = action_info.send_goal_key();
        let send_goal_info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &send_goal_keyexpr,
            send_goal_type,
            A::ACTION_HASH,
        );
        let send_goal_client = self
            .session
            .create_client(&send_goal_info, QoSProfile::services_default())
            .map_err(|_| NodeError::ActionCreationFailed)?;

        let cancel_goal_keyexpr: heapless::String<256> = action_info.cancel_goal_key();
        let cancel_goal_info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &cancel_goal_keyexpr,
            "action_msgs::srv::dds_::CancelGoal_",
            A::ACTION_HASH,
        );
        let cancel_goal_client = self
            .session
            .create_client(&cancel_goal_info, QoSProfile::services_default())
            .map_err(|_| NodeError::ActionCreationFailed)?;

        let get_result_keyexpr: heapless::String<256> = action_info.get_result_key();
        let get_result_info = Self::service_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &get_result_keyexpr,
            get_result_type,
            A::ACTION_HASH,
        );
        let get_result_client = self
            .session
            .create_client(&get_result_info, QoSProfile::services_default())
            .map_err(|_| NodeError::ActionCreationFailed)?;

        let feedback_keyexpr: heapless::String<256> = action_info.feedback_key();
        let feedback_topic = Self::topic_info(
            self.domain_id,
            &self.name,
            &self.namespace,
            &feedback_keyexpr,
            feedback_type,
            A::ACTION_HASH,
        );
        let feedback_subscriber = self
            .session
            .create_subscription(&feedback_topic, QoSProfile::BEST_EFFORT)
            .map_err(|_| NodeError::ActionCreationFailed)?;

        Ok(ActionClient {
            core: super::action_core::ActionClientCore {
                send_goal_client,
                cancel_goal_client,
                get_result_client,
                feedback_subscriber,
                goal_buffer: [0u8; GOAL_BUF],
                result_buffer: [0u8; RESULT_BUF],
                feedback_buffer: [0u8; FEEDBACK_BUF],
                goal_counter: 0,
                in_flight_send_goal: false,
                in_flight_cancel: false,
                in_flight_get_result: false,
            },
            _phantom: PhantomData,
        })
    }
}

// ===================================================================
// Phase 189.M1 — entity builders (the `clone` tier)
// ===================================================================

/// Publisher builder — `node.publisher(topic)`. Choose `.typed::<M>()` or
/// `.generic(type, hash)`, optionally `.qos(..)`, then `.build()`.
pub struct PublisherBuilder<'n, 'a, 't> {
    node: &'n mut NodeHandle<'a>,
    topic: &'t str,
    qos: QoSProfile,
}

impl<'n, 'a, 't> PublisherBuilder<'n, 'a, 't> {
    /// Set the QoS (also settable on the typed/generic builder).
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }

    /// Phase 282 (#145) — mark this publisher "express": its samples bypass
    /// transport tx batching (sent immediately even when the batching knob is
    /// on). For control-tier / latency-sensitive topics.
    pub fn tx_express(mut self, express: bool) -> Self {
        self.qos.tx_express = express;
        self
    }

    /// Typed publisher for a ROS message `M` (mirrors rclcpp/rclrs).
    pub fn typed<M: MessageForRmw>(self) -> TypedPublisherBuilder<'n, 'a, 't, M> {
        TypedPublisherBuilder {
            node: self.node,
            topic: self.topic,
            qos: self.qos,
            _phantom: PhantomData,
        }
    }

    /// Generic (type-erased) publisher — the rclcpp `create_generic_publisher`
    /// form; raw CDR bytes via `publish_raw`.
    pub fn generic(
        self,
        type_name: &'t str,
        type_hash: &'t str,
    ) -> GenericPublisherBuilder<'n, 'a, 't> {
        GenericPublisherBuilder {
            node: self.node,
            topic: self.topic,
            type_name,
            type_hash,
            qos: self.qos,
        }
    }
}

/// Typed publisher builder (`.typed::<M>()`).
pub struct TypedPublisherBuilder<'n, 'a, 't, M> {
    node: &'n mut NodeHandle<'a>,
    topic: &'t str,
    qos: QoSProfile,
    _phantom: PhantomData<M>,
}

impl<'n, 'a, 't, M: MessageForRmw> TypedPublisherBuilder<'n, 'a, 't, M> {
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }

    /// Phase 282 (#145) — see [`PublisherBuilder::tx_express`].
    pub fn tx_express(mut self, express: bool) -> Self {
        self.qos.tx_express = express;
        self
    }

    pub fn build(self) -> Result<EmbeddedPublisher<M>, NodeError> {
        self.node
            .create_publisher_with_qos::<M>(self.topic, self.qos)
    }
}

/// Generic (type-erased) publisher builder (`.generic(type, hash)`).
pub struct GenericPublisherBuilder<'n, 'a, 't> {
    node: &'n mut NodeHandle<'a>,
    topic: &'t str,
    type_name: &'t str,
    type_hash: &'t str,
    qos: QoSProfile,
}

impl<'n, 'a, 't> GenericPublisherBuilder<'n, 'a, 't> {
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }

    /// Phase 282 (#145) — see [`PublisherBuilder::tx_express`].
    pub fn tx_express(mut self, express: bool) -> Self {
        self.qos.tx_express = express;
        self
    }

    pub fn build(self) -> Result<crate::executor::handles::EmbeddedRawPublisher, NodeError> {
        self.node.create_publisher_raw_with_qos(
            self.topic,
            self.type_name,
            self.type_hash,
            self.qos,
        )
    }
}

// ============================================================================
// Phase 273 (RFC-0047) — CallbackGroup token (rclcpp/rclrs shape)
// ============================================================================

/// A first-class callback group — a **name-only token** (rclcpp/rclrs shape).
///
/// Created via [`NodeCtx::create_callback_group`].  Passed to the `_in`
/// entity-create variants (`create_timer_in`, `create_subscription_in`,
/// `create_publisher_in`) to label entities with a group name.
///
/// The group is just a name — the actual `SchedContext` binding is seeded at
/// boot by `Executor::bind_group_sched` (from `system.toml group_tiers`, phase
/// 273 W2) and resolved at entity-registration time by
/// `apply_node_default_sched` (phase 273 W1).  No concurrency type (Mutually-
/// Exclusive vs Reentrant) is stored here — that is RFC-0047 OQ1 follow-up.
pub struct CallbackGroup {
    name: heapless::String<32>,
}

impl CallbackGroup {
    /// The group name (e.g. `"ctrl"`, `"telem"`).
    pub fn name(&self) -> &str {
        &self.name
    }
}

/// An executor-borrowing node handle — `exec.node(id)`. Hosts the
/// callback-registering entity builders (subscriptions register into the
/// executor's dispatch arena). It is a **short-lived `&mut Executor` borrow**:
/// create entities, then drop it before acquiring the next node handle; entity
/// handles (`HandleId`, publishers) are owned and outlive it (no `Arc` — see
/// `docs/design/0022-entity-api-tiers.md` §Borrow model).
pub struct NodeCtx<'e, 's> {
    executor: &'e mut super::spin::Executor<'s>,
    node_id: super::node_record::NodeId,
}

impl<'e, 's> NodeCtx<'e, 's> {
    pub(crate) fn new(
        executor: &'e mut super::spin::Executor<'s>,
        node_id: super::node_record::NodeId,
    ) -> Self {
        Self { executor, node_id }
    }

    /// Subscription builder (the `clone` tier). Pick a mode with `.typed::<M>()`
    /// or `.generic(type, hash)`, set knobs (`.qos`), then `.build(callback)`.
    pub fn subscription<'t>(&mut self, topic: &'t str) -> SubscriptionBuilder<'_, 'e, 't, 's> {
        SubscriptionBuilder {
            ctx: self,
            topic,
            qos: QoSProfile::default(),
        }
    }

    /// Publisher builder (the `clone` tier), symmetric with
    /// [`subscription`](Self::subscription). Pick `.typed::<M>()` or
    /// `.generic(type, hash)`, set `.qos()`, then `.build()`. The returned
    /// publisher handle is owned and outlives this `NodeCtx` — the bridge
    /// builds the dest publisher on one ctx, drops it, then registers the
    /// source subscription on another (see `0022-entity-api-tiers.md`).
    pub fn publisher<'t>(&mut self, topic: &'t str) -> CtxPublisherBuilder<'_, 'e, 't, 's> {
        CtxPublisherBuilder {
            ctx: self,
            topic,
            qos: QoSProfile::default(),
        }
    }

    /// Convenient typed publisher (the `fork` tier — rclcpp/rclrs shape).
    pub fn create_publisher<M: MessageForRmw>(
        &mut self,
        topic: &str,
    ) -> Result<EmbeddedPublisher<M>, NodeError> {
        self.executor
            .create_publisher_on::<M>(self.node_id, topic, QoSProfile::default())
    }

    /// Convenient generic (type-erased) publisher — rclcpp `create_generic_*`.
    pub fn create_generic_publisher(
        &mut self,
        topic: &str,
        type_name: &str,
        type_hash: &str,
    ) -> Result<crate::executor::handles::EmbeddedRawPublisher, NodeError> {
        self.create_generic_publisher_with_qos(topic, type_name, type_hash, QoSProfile::default())
    }

    /// Issue 0306 — the same, with an explicit profile. The declarative
    /// component path needs it: a node that declares
    /// `create_publisher_for_topic_with_qos(...)` carries its profile in
    /// `EntityMetadata::qos`, and the runtime used to drop it on the floor by
    /// calling the default-QoS constructor here.
    pub fn create_generic_publisher_with_qos(
        &mut self,
        topic: &str,
        type_name: &str,
        type_hash: &str,
        qos: QoSProfile,
    ) -> Result<crate::executor::handles::EmbeddedRawPublisher, NodeError> {
        self.executor
            .create_publisher_raw_on(self.node_id, topic, type_name, type_hash, qos)
    }

    /// Convenient typed subscription (the `fork` tier — rclcpp/rclrs shape).
    /// Sugar over the builder with default QoS + buffer.
    pub fn create_subscription<M, F>(
        &mut self,
        topic: &str,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError>
    where
        M: MessageForRmw + 'static,
        F: FnMut(&M) + 'static,
    {
        self.executor
            .register_subscription_buffered_on::<M, F, { crate::config::DEFAULT_RX_BUF_SIZE }>(
                self.node_id,
                topic,
                QoSProfile::default(),
                callback,
                None, // no group — node default
                None, // phase-403 W2: the configured default, unchanged
            )
    }

    /// Subscribe `/clock` and install every sample as this image's ROS time —
    /// phase-425 W3, the thing `rclcpp`'s `use_sim_time` parameter switches on.
    ///
    /// After this, `Clock::ros_time().now()` is the simulator's or the bag
    /// player's time, and a timer registered with `TimerClockSource::Ros`
    /// follows it: it stops while the simulation is paused and tracks the replay
    /// rate. Nothing else changes — a wall timer on the same executor keeps its
    /// own cadence, which is what a watchdog needs.
    ///
    /// QoS is `QoSProfile::clock_default()`, i.e. `rclcpp::ClockQoS`: best
    /// effort, keep-last 1, volatile. A late subscriber wants the NEXT sample,
    /// not a replay of the simulation's history.
    ///
    /// Returns the subscription handle. Cancelling it stops the source but does
    /// NOT clear the override: a node that unsubscribes mid-run keeps the last
    /// simulated time rather than jumping back to the wall clock, which every
    /// ROS-time timer would otherwise have to absorb as a jump.
    ///
    /// The override is process-global — one simulated clock per image — so
    /// calling this on a second node in the same image adds a second subscriber
    /// to the same global, which is redundant rather than wrong.
    #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
    pub fn install_ros_time_source(&mut self) -> Result<super::types::HandleId, NodeError> {
        self.install_ros_time_source_on(crate::time_source::CLOCK_TOPIC)
    }

    /// [`install_ros_time_source`](Self::install_ros_time_source) against a
    /// topic other than `/clock` — a remapped or namespaced clock, which is what
    /// a launch file creates when two simulations share a graph.
    #[cfg(all(feature = "sim-time", any(has_rmw, test)))]
    pub fn install_ros_time_source_on(
        &mut self,
        topic: &str,
    ) -> Result<super::types::HandleId, NodeError> {
        // One spelling of the QoS and the sample conversion, on the executor:
        // the `use_sim_time` reconciliation (phase-425 W3b) installs the same
        // subscription, and two copies would be two places for the QoS to drift.
        self.executor.install_ros_time_source(self.node_id, topic)
    }

    // -----------------------------------------------------------------------
    // Phase 273 (RFC-0047) — callback-group API (rclcpp/rclrs shape)
    // -----------------------------------------------------------------------

    /// Create a named callback group — a thin token wrapping the group name.
    ///
    /// Pass the returned [`CallbackGroup`] to the `_in` create variants
    /// (`create_timer_in`, `create_subscription_in`, `create_publisher_in`)
    /// to label entities with this group. The executor binds the entity's
    /// callback to the `SchedContext` seeded via `bind_group_sched` for
    /// `(node_name, namespace, group_name)` (phase 273 W1/W2).
    ///
    /// ```ignore
    /// let ctrl = node.create_callback_group("ctrl");
    /// node.create_timer_in(&ctrl, TimerDuration::from_millis(10), || { /* … */ })?;
    /// ```
    ///
    /// Names longer than 32 bytes are silently truncated.
    pub fn create_callback_group(&self, name: &str) -> CallbackGroup {
        let mut s = heapless::String::<32>::new();
        // Silently truncate if the name is too long (defensive; caller
        // should use short ASCII group names like "ctrl"/"telem").
        for ch in name.chars() {
            if s.push(ch).is_err() {
                break;
            }
        }
        CallbackGroup { name: s }
    }

    /// Create a repeating timer **in** a callback group (phase 273, rclcpp shape).
    ///
    /// The timer's callback is bound to the `SchedContext` associated with
    /// `group` in the executor's `group_sched_table` for this node. If no
    /// entry was seeded for the group, the node's default `SchedContext` applies
    /// (same as `register_timer`). `period` fires the callback repeatedly.
    pub fn create_timer_in<F>(
        &mut self,
        group: &CallbackGroup,
        period: crate::timer::TimerDuration,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError>
    where
        F: FnMut() + 'static,
    {
        self.executor
            .register_timer_on(Some(self.node_id), period, callback, Some(group.name()))
    }

    /// Create a typed subscription **in** a callback group (phase 273, rclcpp shape).
    ///
    /// The subscription's callback is bound to the `SchedContext` associated
    /// with `group` in the `group_sched_table` for this node. If no entry was
    /// seeded, the node default applies.
    pub fn create_subscription_in<M, F>(
        &mut self,
        group: &CallbackGroup,
        topic: &str,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError>
    where
        M: MessageForRmw + 'static,
        F: FnMut(&M) + 'static,
    {
        self.executor
            .register_subscription_buffered_on::<M, F, { crate::config::DEFAULT_RX_BUF_SIZE }>(
                self.node_id,
                topic,
                QoSProfile::default(),
                callback,
                Some(group.name()),
                None, // phase-403 W2: the configured default, unchanged
            )
    }

    /// Create a typed publisher **in** a callback group (phase 273, rclcpp shape).
    ///
    /// Publishers do not have an executor-dispatched callback, so the group
    /// name has no scheduling effect today (publishers are explicitly driven by
    /// the user via `publish()`). The API is provided for symmetry and
    /// forward-compatibility (intra-process / loaned-message knobs may use
    /// it in the future).
    pub fn create_publisher_in<M: MessageForRmw>(
        &mut self,
        _group: &CallbackGroup,
        topic: &str,
    ) -> Result<EmbeddedPublisher<M>, NodeError> {
        // Publishers carry no executor callback slot; group is forward-compat.
        self.executor
            .create_publisher_on::<M>(self.node_id, topic, QoSProfile::default())
    }

    /// RFC-0041 / Phase 239.1 — callback-based service client (rclcpp
    /// `async_send_request(req, cb)` analogue). The reply is delivered to
    /// `callback` at `spin_once` (no `Promise` poll). Returns a
    /// [`ServiceClientCallback`] send handle; dual-mode — the `Promise`-based
    /// [`create_client`](Self::create_client) is unchanged.
    pub fn create_client_with_callback<Svc, F>(
        &mut self,
        service_name: &str,
        callback: F,
    ) -> Result<ServiceClientCallback<Svc>, NodeError>
    where
        Svc: RosService + 'static,
        Svc::Request: MessageForRmw,
        Svc::Reply: MessageForRmw,
        F: FnMut(&Svc::Reply) + 'static,
    {
        self.create_client_with_callback_sized::<
            Svc,
            F,
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
        >(service_name, callback)
    }

    /// Callback-based service client with custom buffer sizes (Phase 239.1).
    pub fn create_client_with_callback_sized<Svc, F, const REQ_BUF: usize, const REPLY_BUF: usize>(
        &mut self,
        service_name: &str,
        callback: F,
    ) -> Result<ServiceClientCallback<Svc, REQ_BUF, REPLY_BUF>, NodeError>
    where
        Svc: RosService + 'static,
        Svc::Request: MessageForRmw,
        Svc::Reply: MessageForRmw,
        F: FnMut(&Svc::Reply) + 'static,
    {
        register_type::<Svc::Request>()?;
        register_type::<Svc::Reply>()?;
        let (_id, hdr) = self
            .executor
            .register_service_client_callback::<Svc, F, REPLY_BUF>(
                Some(self.node_id),
                service_name,
                Svc::SERVICE_NAME,
                Svc::SERVICE_HASH,
                QoSProfile::services_default(),
                callback,
            )?;
        Ok(ServiceClientCallback::new(hdr))
    }

    /// RFC-0041 / Phase 239.2 — callback-based action client (rclcpp
    /// `SendGoalOptions{goal_response_callback, feedback_callback,
    /// result_callback}` analogue). Goal-response / feedback / result are
    /// delivered to the closures at `spin_once`. Returns an
    /// [`ActionClientCallback`] send handle (`send_goal` / `get_result`);
    /// dual-mode — the `Promise`-based [`create_action_client`](Self::create_action_client)
    /// is unchanged.
    #[allow(clippy::type_complexity)]
    pub fn create_action_client_with_callbacks<A, GRespF, FbF, ResF>(
        &mut self,
        action_name: &str,
        on_goal_response: GRespF,
        on_feedback: FbF,
        on_result: ResF,
    ) -> Result<ActionClientCallback<A>, NodeError>
    where
        A: RosAction + 'static,
        A::Goal: MessageForRmw,
        A::Result: MessageForRmw,
        A::Feedback: MessageForRmw,
        GRespF: FnMut(&nros_core::GoalId, bool) + 'static,
        FbF: FnMut(&nros_core::GoalId, &A::Feedback) + 'static,
        ResF: FnMut(&nros_core::GoalId, nros_core::GoalStatus, &A::Result) + 'static,
    {
        self.create_action_client_with_callbacks_sized::<
            A,
            GRespF,
            FbF,
            ResF,
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
        >(action_name, on_goal_response, on_feedback, on_result)
    }

    /// Callback-based action client with custom buffer sizes (Phase 239.2).
    #[allow(clippy::type_complexity)]
    pub fn create_action_client_with_callbacks_sized<
        A,
        GRespF,
        FbF,
        ResF,
        const GOAL_BUF: usize,
        const RESULT_BUF: usize,
        const FEEDBACK_BUF: usize,
    >(
        &mut self,
        action_name: &str,
        on_goal_response: GRespF,
        on_feedback: FbF,
        on_result: ResF,
    ) -> Result<ActionClientCallback<A, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>, NodeError>
    where
        A: RosAction + 'static,
        A::Goal: MessageForRmw,
        A::Result: MessageForRmw,
        A::Feedback: MessageForRmw,
        GRespF: FnMut(&nros_core::GoalId, bool) + 'static,
        FbF: FnMut(&nros_core::GoalId, &A::Feedback) + 'static,
        ResF: FnMut(&nros_core::GoalId, nros_core::GoalStatus, &A::Result) + 'static,
    {
        register_type::<A::Goal>()?;
        register_type::<A::Result>()?;
        register_type::<A::Feedback>()?;
        let (_id, core) = self
            .executor
            .register_action_client_callback::<A, GRespF, FbF, ResF, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>(
                Some(self.node_id),
                action_name,
                A::ACTION_NAME,
                A::ACTION_HASH,
                // Feedback is a stream → buffer a short QoS-depth history (Phase
                // 239.5). Goal-response / result are single-outstanding (gated).
                8u16,
                on_goal_response,
                on_feedback,
                on_result,
            )?;
        Ok(ActionClientCallback::new(core))
    }

    /// Convenient generic (type-erased) subscription — rclcpp `create_generic_*`.
    pub fn create_generic_subscription<F>(
        &mut self,
        topic: &str,
        type_name: &str,
        type_hash: &str,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError>
    where
        F: FnMut(&[u8]) + 'static,
    {
        self.create_generic_subscription_with_qos(
            topic,
            type_name,
            type_hash,
            QoSProfile::default(),
            callback,
        )
    }

    /// Issue 0306 — the same, with an explicit profile (see
    /// [`Self::create_generic_publisher_with_qos`] for why).
    pub fn create_generic_subscription_with_qos<F>(
        &mut self,
        topic: &str,
        type_name: &str,
        type_hash: &str,
        qos: QoSProfile,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError>
    where
        F: FnMut(&[u8]) + 'static,
    {
        self.executor
            .register_subscription_buffered_raw_on::<F, { crate::config::DEFAULT_RX_BUF_SIZE }>(
                self.node_id,
                topic,
                type_name,
                type_hash,
                qos,
                callback,
            )
    }

    /// Phase 250 (Wave 2) — generic (type-erased) subscription that surfaces E2E
    /// [`IntegrityStatus`](nros_rmw::IntegrityStatus) (CRC + sequence gap/dup) to
    /// the callback (`FnMut(&[u8], &IntegrityStatus)`). The declarative-`Node`
    /// analog of the typed `.typed::<M>().safety()` builder: the validator lives
    /// in the `RmwSubscriber`, so the raw bytes + status arrive together without
    /// a typed `M`. Wired by the declarative runtime's `.safety()` opt-in.
    #[cfg(feature = "safety-e2e")]
    pub fn create_generic_subscription_with_integrity<F>(
        &mut self,
        topic: &str,
        type_name: &str,
        type_hash: &str,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError>
    where
        F: FnMut(&[u8], &nros_rmw::IntegrityStatus) + 'static,
    {
        self.executor
            .register_subscription_buffered_raw_safety_on::<F, { crate::config::DEFAULT_RX_BUF_SIZE }>(
                self.node_id,
                topic,
                type_name,
                type_hash,
                QoSProfile::default(),
                callback,
            )
    }

    /// Deprecated spelling of [`create_subscription_viewable`](Self::create_subscription_viewable).
    ///
    /// phase-390 renamed RFC-0033's `borrowed` mode to `view`, because the fact
    /// that mattered was never that the data is borrowed but that NOTHING WAS
    /// DESERIALIZED. A forwarder rather than a hard break: this is a public
    /// Rust API, the rename is cosmetic, and the C ABI break in W2 was accepted
    /// only because a C type name cannot carry a deprecation.
    #[deprecated(
        since = "0.5.0",
        note = "renamed to `create_subscription_viewable` (phase-390: RFC-0033 \
                `borrowed` mode is now `view`)"
    )]
    pub fn create_subscription_borrowed<B, F>(
        &mut self,
        topic: &str,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError>
    where
        B: nros_core::ViewableMessage + 'static,
        F: for<'a> FnMut(&B::View<'a>) + 'static,
    {
        self.create_subscription_viewable::<B, F>(topic, callback)
    }

    /// Convenient zero-copy subscription (Phase 229.6, issue 0007 / RFC-0033
    /// `view` mode).
    ///
    /// `B` is the code-generated VIEWABLE marker (e.g. `ImageViewable`, emitted
    /// alongside the `inline` `Image` for a `.msg` with a `view`-mode field).
    /// The marker is zero-sized and carries NO lifetime, which is the whole
    /// reason it exists: `B::View<'a>` does, and a generic parameter cannot be
    /// the lifetime-carrying type itself. That is also why phase-390 could not
    /// simply rename it to `{Msg}View` — the view struct already owns that
    /// name, and the marker points AT it.
    ///
    /// The callback receives `&B::View<'a>` — a message whose unbounded
    /// sequence/string fields point directly into the receive buffer (no
    /// `heapless::Vec` copy); valid only for the callback's duration.
    ///
    /// Uses `KEEP_LAST(1)` QoS → triple buffer, as view subscriptions require
    /// (a single well-defined slot for the callback). For an explicit deeper
    /// queue use the `inline` [`create_subscription`](Self::create_subscription);
    /// a view subscription registered with `KEEP_LAST(N>1)` is rejected.
    pub fn create_subscription_viewable<B, F>(
        &mut self,
        topic: &str,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError>
    where
        B: nros_core::ViewableMessage + 'static,
        F: for<'a> FnMut(&B::View<'a>) + 'static,
    {
        self.executor
            .register_subscription_buffered_borrowed_on::<B, F, { crate::config::DEFAULT_RX_BUF_SIZE }>(
                self.node_id,
                topic,
                QoSProfile::default().keep_last(1),
                callback,
            )
    }

    /// Service-server builder (the `clone` tier) — `node.service(name)`.
    /// Set `.qos()` (defaults to the services profile = RELIABLE+VOLATILE+
    /// KEEP_LAST(10)), then `.build::<Svc, _>(callback)` (Phase 193.2).
    pub fn service<'t>(&mut self, name: &'t str) -> CtxServiceBuilder<'_, 'e, 't, 's> {
        CtxServiceBuilder {
            ctx: self,
            name,
            qos: QoSProfile::services_default(),
        }
    }

    /// Convenient service server (the `fork` tier — rclrs/rclcpp shape), default
    /// services QoS. Mirror of `create_subscription`.
    pub fn create_service<Svc, F>(
        &mut self,
        name: &str,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError>
    where
        Svc: RosService + 'static,
        Svc::Request: crate::rmw_type_registry::MessageForRmw,
        Svc::Reply: crate::rmw_type_registry::MessageForRmw,
        F: FnMut(&Svc::Request) -> Svc::Reply + 'static,
    {
        self.executor.register_service_sized_on::<
            Svc,
            F,
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
        >(self.node_id, name, QoSProfile::services_default(), callback)
    }
}

/// Service-server builder on a [`NodeCtx`] — `node.service(name)`.
pub struct CtxServiceBuilder<'c, 'e, 't, 's> {
    ctx: &'c mut NodeCtx<'e, 's>,
    name: &'t str,
    qos: QoSProfile,
}

impl<'c, 'e, 't, 's> CtxServiceBuilder<'c, 'e, 't, 's> {
    /// Service QoS (applies to both the request + reply endpoints). Defaults to
    /// `QoSProfile::services_default()`.
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }

    pub fn build<Svc, F>(self, callback: F) -> Result<super::types::HandleId, NodeError>
    where
        Svc: RosService + 'static,
        Svc::Request: crate::rmw_type_registry::MessageForRmw,
        Svc::Reply: crate::rmw_type_registry::MessageForRmw,
        F: FnMut(&Svc::Request) -> Svc::Reply + 'static,
    {
        self.ctx.executor.register_service_sized_on::<
            Svc,
            F,
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
        >(self.ctx.node_id, self.name, self.qos, callback)
    }
}

/// Publisher builder on a [`NodeCtx`] — `node.publisher(topic)`.
pub struct CtxPublisherBuilder<'c, 'e, 't, 's> {
    ctx: &'c mut NodeCtx<'e, 's>,
    topic: &'t str,
    qos: QoSProfile,
}

impl<'c, 'e, 't, 's> CtxPublisherBuilder<'c, 'e, 't, 's> {
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }

    /// Typed publisher for a ROS message `M`.
    pub fn typed<M: MessageForRmw>(self) -> CtxTypedPublisherBuilder<'c, 'e, 't, 's, M> {
        CtxTypedPublisherBuilder {
            ctx: self.ctx,
            topic: self.topic,
            qos: self.qos,
            _phantom: PhantomData,
        }
    }

    /// Generic (type-erased) publisher.
    pub fn generic(
        self,
        type_name: &'t str,
        type_hash: &'t str,
    ) -> CtxGenericPublisherBuilder<'c, 'e, 't, 's> {
        CtxGenericPublisherBuilder {
            ctx: self.ctx,
            topic: self.topic,
            type_name,
            type_hash,
            qos: self.qos,
        }
    }
}

/// Typed publisher builder on a `NodeCtx` (`.typed::<M>()`).
pub struct CtxTypedPublisherBuilder<'c, 'e, 't, 's, M> {
    ctx: &'c mut NodeCtx<'e, 's>,
    topic: &'t str,
    qos: QoSProfile,
    _phantom: PhantomData<M>,
}

impl<'c, 'e, 't, 's, M: MessageForRmw> CtxTypedPublisherBuilder<'c, 'e, 't, 's, M> {
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }

    pub fn build(self) -> Result<EmbeddedPublisher<M>, NodeError> {
        self.ctx
            .executor
            .create_publisher_on::<M>(self.ctx.node_id, self.topic, self.qos)
    }
}

/// Generic publisher builder on a `NodeCtx` (`.generic(type, hash)`).
pub struct CtxGenericPublisherBuilder<'c, 'e, 't, 's> {
    ctx: &'c mut NodeCtx<'e, 's>,
    topic: &'t str,
    type_name: &'t str,
    type_hash: &'t str,
    qos: QoSProfile,
}

impl<'c, 'e, 't, 's> CtxGenericPublisherBuilder<'c, 'e, 't, 's> {
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }

    pub fn build(self) -> Result<crate::executor::handles::EmbeddedRawPublisher, NodeError> {
        self.ctx.executor.create_publisher_raw_on(
            self.ctx.node_id,
            self.topic,
            self.type_name,
            self.type_hash,
            self.qos,
        )
    }
}

/// Subscription builder — `node.subscription(topic)`.
pub struct SubscriptionBuilder<'c, 'e, 't, 's> {
    ctx: &'c mut NodeCtx<'e, 's>,
    topic: &'t str,
    qos: QoSProfile,
}

impl<'c, 'e, 't, 's> SubscriptionBuilder<'c, 'e, 't, 's> {
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }

    /// Typed subscription for a ROS message `M`.
    pub fn typed<M: MessageForRmw + 'static>(self) -> TypedSubscriptionBuilder<'c, 'e, 't, 's, M> {
        TypedSubscriptionBuilder {
            ctx: self.ctx,
            topic: self.topic,
            qos: self.qos,
            sched: None,
            _phantom: PhantomData,
        }
    }

    /// Generic (type-erased) subscription — raw CDR bytes to the callback.
    pub fn generic(
        self,
        type_name: &'t str,
        type_hash: &'t str,
    ) -> GenericSubscriptionBuilder<'c, 'e, 't, 's> {
        GenericSubscriptionBuilder {
            ctx: self.ctx,
            topic: self.topic,
            type_name,
            type_hash,
            qos: self.qos,
            sched: None,
        }
    }
}

/// Typed subscription builder (`.typed::<M>()`). `RX` is the staging-buffer
/// size, set via `.rx_buffer::<N>()` (defaults to `DEFAULT_RX_BUF_SIZE`).
pub struct TypedSubscriptionBuilder<
    'c,
    'e,
    't,
    's,
    M,
    const RX: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    ctx: &'c mut NodeCtx<'e, 's>,
    topic: &'t str,
    qos: QoSProfile,
    sched: Option<super::sched_context::SchedContextId>,
    _phantom: PhantomData<M>,
}

impl<'c, 'e, 't, 's, M: MessageForRmw + 'static, const RX: usize>
    TypedSubscriptionBuilder<'c, 'e, 't, 's, M, RX>
{
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }

    /// Bind the subscription's callback to a scheduling context.
    pub fn sched_context(mut self, sc: super::sched_context::SchedContextId) -> Self {
        self.sched = Some(sc);
        self
    }

    /// Set the staging-buffer size (const-generic).
    pub fn rx_buffer<const N: usize>(self) -> TypedSubscriptionBuilder<'c, 'e, 't, 's, M, N> {
        TypedSubscriptionBuilder {
            ctx: self.ctx,
            topic: self.topic,
            qos: self.qos,
            sched: self.sched,
            _phantom: PhantomData,
        }
    }

    /// Phase 403 W2 -- size the receive buffer from `M`'s OWN serialized bound
    /// instead of the configured default.
    ///
    /// ```ignore
    /// node.subscription("/chatter")
    ///     .typed::<Int32>()
    ///     .rx_buffer_from_type()   // 12 bytes per slot, not 1024
    ///     .build(on_msg)?;
    /// ```
    ///
    /// **Opt-in, and that is the design.** A subscription that does not call
    /// this claims exactly the bytes it claimed before the knob existed, so an
    /// image that does not opt in is byte-identical -- the same rule issue 0900's
    /// arena knob keeps (`the_default_derivation_is_unchanged`). Sizing every
    /// subscription from its type by default would move every image's arena
    /// occupancy at once, which is a decision, not a refactor.
    ///
    /// **What it costs and what it buys.** The buffered path stores `depth <= 1
    /// ? 3 : depth+1` slots of this size in the executor arena, so a bounded
    /// type recovers `slots * (default - bound)` bytes per subscription. The
    /// number is the LARGER of the type's XCDR1 and XCDR2 bounds, because the
    /// stack writes XCDR1 but a peer may send XCDR2 (see
    /// [`subscription_rx_bytes`](crate::rmw_type_registry::subscription_rx_bytes)).
    ///
    /// **An unbounded `M` fails the BUILD.** Every message type is required to
    /// carry a bound -- stated in the `.msg` (`string<=64`) or capped in
    /// `nros-codegen.toml` -- so "this type has no bound" is a defect to report,
    /// not a case to absorb. The alternative, quietly keeping the configured
    /// default, is precisely the substitution phase 380 forbids: `None` means
    /// "no bound EXISTS", never "unknown", and a buffer sized from a fallback is
    /// the failure that rule was written to prevent. The C header already refuses
    /// the same thing the same way -- naming an unbounded type's
    /// `{PREFIX}_RX_MAX_SERIALIZED_SIZE` expands to a deliberate compile error
    /// carrying the member that costs the bound (`unbounded_token`) -- and this
    /// is that refusal on the Rust path. rustc names `M` in the instantiation
    /// note; the member comes from the codegen diagnostic for the same type.
    ///
    /// **The bound never GROWS the buffer.** A type larger than `RX` keeps `RX`;
    /// spending unbudgeted arena silently is the worse failure, and the
    /// too-small case already reports itself (`report_dropped_take`).
    ///
    /// Returns a builder without `.message_info()` / `.safety()` on purpose:
    /// those entries hold a real `[u8; RX]` array, so their size can only come
    /// from a const generic -- `nros::rx_buffer_for!(M)` at the call site -- and
    /// carrying the flag into them would silently drop it.
    pub fn rx_buffer_from_type(self) -> TypedSubBoundBuilder<'c, 'e, 't, 's, M, RX>
    where
        M: nros_serdes::schema::Message,
    {
        // The diagnostic lives HERE, at the opt-in, rather than in `build()`:
        // this is the call that asserts the type has a bound, so it is the call
        // the error should point at.
        const {
            assert!(
                crate::rmw_type_registry::subscription_rx_bytes::<M>(RX).is_some(),
                "this message type has NO maximum serialized size, so its receive \
                 buffer cannot be sized from it. Every message type must carry a \
                 bound: give the unbounded member one in the `.msg` \
                 (`string<=64`, `int32[<=8]`) or a `cap` in `nros-codegen.toml`. \
                 The generated C header for this type names the member that costs \
                 it the bound (`NROS_UNBOUNDED__<type>__field_<member>`)."
            )
        }
        TypedSubBoundBuilder {
            ctx: self.ctx,
            topic: self.topic,
            qos: self.qos,
            sched: self.sched,
            _phantom: PhantomData,
        }
    }

    /// Surface per-message [`MessageInfo`](nros_core::MessageInfo) (seq,
    /// publisher GID, timestamps) to the callback — `FnMut(&M, Option<&MessageInfo>)`,
    /// the rclrs shape. Distinct from the generic builder's `.message_info()`
    /// (which yields a `RawMessageInfo` with the wire attachment).
    pub fn message_info(self) -> TypedSubInfoBuilder<'c, 'e, 't, 's, M, RX> {
        TypedSubInfoBuilder {
            ctx: self.ctx,
            topic: self.topic,
            qos: self.qos,
            sched: self.sched,
            _phantom: PhantomData,
        }
    }

    /// Surface E2E-safety validation (CRC + sequence gap/duplicate) to the
    /// callback — `FnMut(&M, &IntegrityStatus)`.
    #[cfg(feature = "safety-e2e")]
    pub fn safety(self) -> TypedSubSafetyBuilder<'c, 'e, 't, 's, M, RX> {
        TypedSubSafetyBuilder {
            ctx: self.ctx,
            topic: self.topic,
            qos: self.qos,
            sched: self.sched,
            _phantom: PhantomData,
        }
    }

    pub fn build<F: FnMut(&M) + 'static>(
        self,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError> {
        let handle = self
            .ctx
            .executor
            .register_subscription_buffered_on::<M, F, RX>(
                self.ctx.node_id,
                self.topic,
                self.qos,
                callback,
                None, // group threaded via create_subscription_in; builder uses sched override
                None, // phase-403 W2: `RX` verbatim, the pre-knob behaviour
            )?;
        if let Some(sc) = self.sched {
            self.ctx.executor.bind_handle_to_sched_context(handle, sc)?;
        }
        Ok(handle)
    }
}

/// Phase 403 W2 -- a typed subscription whose buffer is sized by `M`'s own
/// serialized bound (`.typed::<M>().rx_buffer_from_type()`).
///
/// Terminal by construction: `.message_info()` and `.safety()` are absent
/// because their entries store `[u8; RX]` and cannot take a runtime size. `RX`
/// survives as the CEILING -- an unbounded `M`, or one whose bound exceeds `RX`,
/// gets `RX` unchanged.
pub struct TypedSubBoundBuilder<
    'c,
    'e,
    't,
    's,
    M,
    const RX: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    ctx: &'c mut NodeCtx<'e, 's>,
    topic: &'t str,
    qos: QoSProfile,
    sched: Option<super::sched_context::SchedContextId>,
    _phantom: PhantomData<M>,
}

impl<'c, 'e, 't, 's, M: MessageForRmw + nros_serdes::schema::Message + 'static, const RX: usize>
    TypedSubBoundBuilder<'c, 'e, 't, 's, M, RX>
{
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }

    /// Bind the subscription's callback to a scheduling context.
    pub fn sched_context(mut self, sc: super::sched_context::SchedContextId) -> Self {
        self.sched = Some(sc);
        self
    }

    pub fn build<F: FnMut(&M) + 'static>(
        self,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError> {
        // Unreachable: `rx_buffer_from_type` is the only constructor of this
        // builder and its `const` assert already refused an unbounded `M`.
        let rx_bytes = crate::rmw_type_registry::subscription_rx_bytes::<M>(RX)
            .expect("rx_buffer_from_type asserts the bound exists at build time");
        let handle = self
            .ctx
            .executor
            .register_subscription_buffered_on::<M, F, RX>(
                self.ctx.node_id,
                self.topic,
                self.qos,
                callback,
                None,
                Some(rx_bytes),
            )?;
        if let Some(sc) = self.sched {
            self.ctx.executor.bind_handle_to_sched_context(handle, sc)?;
        }
        Ok(handle)
    }
}

/// Typed subscription builder with `MessageInfo` (`.typed::<M>().message_info()`).
/// Callback is `FnMut(&M, Option<&MessageInfo>)`.
pub struct TypedSubInfoBuilder<
    'c,
    'e,
    't,
    's,
    M,
    const RX: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    ctx: &'c mut NodeCtx<'e, 's>,
    topic: &'t str,
    qos: QoSProfile,
    sched: Option<super::sched_context::SchedContextId>,
    _phantom: PhantomData<M>,
}

impl<'c, 'e, 't, 's, M: MessageForRmw + 'static, const RX: usize>
    TypedSubInfoBuilder<'c, 'e, 't, 's, M, RX>
{
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }

    pub fn sched_context(mut self, sc: super::sched_context::SchedContextId) -> Self {
        self.sched = Some(sc);
        self
    }

    pub fn rx_buffer<const N: usize>(self) -> TypedSubInfoBuilder<'c, 'e, 't, 's, M, N> {
        TypedSubInfoBuilder {
            ctx: self.ctx,
            topic: self.topic,
            qos: self.qos,
            sched: self.sched,
            _phantom: PhantomData,
        }
    }

    pub fn build<F: FnMut(&M, Option<&nros_core::MessageInfo>) + 'static>(
        self,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError> {
        let handle = self
            .ctx
            .executor
            .register_subscription_with_info_sized_inner::<M, F, RX>(
                Some(self.ctx.node_id),
                self.topic,
                self.qos,
                callback,
            )?;
        if let Some(sc) = self.sched {
            self.ctx.executor.bind_handle_to_sched_context(handle, sc)?;
        }
        Ok(handle)
    }
}

/// Typed subscription builder with E2E-safety validation
/// (`.typed::<M>().safety()`). Callback is `FnMut(&M, &IntegrityStatus)`.
#[cfg(feature = "safety-e2e")]
pub struct TypedSubSafetyBuilder<
    'c,
    'e,
    't,
    's,
    M,
    const RX: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    ctx: &'c mut NodeCtx<'e, 's>,
    topic: &'t str,
    qos: QoSProfile,
    sched: Option<super::sched_context::SchedContextId>,
    _phantom: PhantomData<M>,
}

#[cfg(feature = "safety-e2e")]
impl<'c, 'e, 't, 's, M: MessageForRmw + 'static, const RX: usize>
    TypedSubSafetyBuilder<'c, 'e, 't, 's, M, RX>
{
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }

    pub fn sched_context(mut self, sc: super::sched_context::SchedContextId) -> Self {
        self.sched = Some(sc);
        self
    }

    pub fn rx_buffer<const N: usize>(self) -> TypedSubSafetyBuilder<'c, 'e, 't, 's, M, N> {
        TypedSubSafetyBuilder {
            ctx: self.ctx,
            topic: self.topic,
            qos: self.qos,
            sched: self.sched,
            _phantom: PhantomData,
        }
    }

    pub fn build<F: FnMut(&M, &nros_rmw::IntegrityStatus) + 'static>(
        self,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError> {
        let handle = self
            .ctx
            .executor
            .register_subscription_with_safety_sized_inner::<M, F, RX>(
                Some(self.ctx.node_id),
                self.topic,
                self.qos,
                callback,
            )?;
        if let Some(sc) = self.sched {
            self.ctx.executor.bind_handle_to_sched_context(handle, sc)?;
        }
        Ok(handle)
    }
}

/// Generic (type-erased) subscription builder (`.generic(type, hash)`).
pub struct GenericSubscriptionBuilder<
    'c,
    'e,
    't,
    's,
    const RX: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    ctx: &'c mut NodeCtx<'e, 's>,
    topic: &'t str,
    type_name: &'t str,
    type_hash: &'t str,
    qos: QoSProfile,
    sched: Option<super::sched_context::SchedContextId>,
}

impl<'c, 'e, 't, 's, const RX: usize> GenericSubscriptionBuilder<'c, 'e, 't, 's, RX> {
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }

    pub fn sched_context(mut self, sc: super::sched_context::SchedContextId) -> Self {
        self.sched = Some(sc);
        self
    }

    pub fn rx_buffer<const N: usize>(self) -> GenericSubscriptionBuilder<'c, 'e, 't, 's, N> {
        GenericSubscriptionBuilder {
            ctx: self.ctx,
            topic: self.topic,
            type_name: self.type_name,
            type_hash: self.type_hash,
            qos: self.qos,
            sched: self.sched,
        }
    }

    /// Surface the sample's wire attachment + metadata to the callback
    /// (`FnMut(&[u8], &RawMessageInfo)`). The cross-RMW bridge reads the
    /// `bridge_origin` tag from `info.attachment()` for echo suppression.
    pub fn message_info(self) -> GenericSubInfoBuilder<'c, 'e, 't, 's, RX> {
        GenericSubInfoBuilder {
            ctx: self.ctx,
            topic: self.topic,
            type_name: self.type_name,
            type_hash: self.type_hash,
            qos: self.qos,
            sched: self.sched,
        }
    }

    pub fn build<F: FnMut(&[u8]) + 'static>(
        self,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError> {
        let handle = self
            .ctx
            .executor
            .register_subscription_buffered_raw_on::<F, RX>(
                self.ctx.node_id,
                self.topic,
                self.type_name,
                self.type_hash,
                self.qos,
                callback,
            )?;
        if let Some(sc) = self.sched {
            self.ctx.executor.bind_handle_to_sched_context(handle, sc)?;
        }
        Ok(handle)
    }
}

/// Generic subscription builder with `MessageInfo` surfaced
/// (`.message_info()`). Callback is `FnMut(&[u8], &RawMessageInfo)`.
pub struct GenericSubInfoBuilder<
    'c,
    'e,
    't,
    's,
    const RX: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    ctx: &'c mut NodeCtx<'e, 's>,
    topic: &'t str,
    type_name: &'t str,
    type_hash: &'t str,
    qos: QoSProfile,
    sched: Option<super::sched_context::SchedContextId>,
}

impl<'c, 'e, 't, 's, const RX: usize> GenericSubInfoBuilder<'c, 'e, 't, 's, RX> {
    pub fn qos(mut self, qos: QoSProfile) -> Self {
        self.qos = qos;
        self
    }

    pub fn sched_context(mut self, sc: super::sched_context::SchedContextId) -> Self {
        self.sched = Some(sc);
        self
    }

    pub fn rx_buffer<const N: usize>(self) -> GenericSubInfoBuilder<'c, 'e, 't, 's, N> {
        GenericSubInfoBuilder {
            ctx: self.ctx,
            topic: self.topic,
            type_name: self.type_name,
            type_hash: self.type_hash,
            qos: self.qos,
            sched: self.sched,
        }
    }

    pub fn build<F: FnMut(&[u8], &nros_core::RawMessageInfo) + 'static>(
        self,
        callback: F,
    ) -> Result<super::types::HandleId, NodeError> {
        let handle = self
            .ctx
            .executor
            .register_subscription_buffered_raw_info_on::<F, RX>(
                self.ctx.node_id,
                self.topic,
                self.type_name,
                self.type_hash,
                self.qos,
                callback,
            )?;
        if let Some(sc) = self.sched {
            self.ctx.executor.bind_handle_to_sched_context(handle, sc)?;
        }
        Ok(handle)
    }
}

// `not(feature = "rmw-cffi")` — these tests use the `mock` backend
// (`crate::mock`, itself `cfg(all(test, not(rmw-cffi)))`); a workspace test
// build that unifies `rmw-cffi` on swaps `ConcreteSession` to the cffi session
// and drops `mock`, so the module must drop with it (matches the
// `mock_integration` gate in lifecycle_services.rs).
#[cfg(all(test, feature = "std", not(feature = "rmw-cffi")))]
mod builder_tests {
    use super::*;
    use crate::{executor::Executor, mock::MockSession};
    use nros_core::{CdrReader, CdrWriter, DeserError, Deserialize, SerError, Serialize};

    struct TestMsg;
    impl RosMessage for TestMsg {
        const TYPE_NAME: &'static str = "test/msg/TestMsg";
        const TYPE_HASH: &'static str = "test_hash";
    }
    impl Serialize for TestMsg {
        fn serialize(&self, _w: &mut CdrWriter) -> Result<(), SerError> {
            Ok(())
        }
    }
    impl Deserialize for TestMsg {
        fn deserialize(_r: &mut CdrReader) -> Result<Self, DeserError> {
            Ok(Self)
        }
    }
    // Phase 212.K.7.6.b — minimal single-field `Message` impl so
    // `TypedPublisherBuilder::build` resolves under the cyclonedds-tightened
    // bound AND the runtime register call succeeds. `DescriptorBuilder`
    // rejects empty `FIELDS` with `BuildError::EmptySchema`; pretend
    // there's one byte so the bridge stub returns a non-NULL pointer.
    // phase-380 W4 — was `#[cfg(rmw_needs_type_descriptors)]`: the schema is
    // now required by `MessageForRmw` on EVERY backend, because that is where
    // a subscription's build-time size bound comes from.
    impl nros_serdes::schema::Message for TestMsg {
        const TYPE_NAME: &'static str = "test/msg/TestMsg";
        const FIELDS: &'static [nros_serdes::schema::Field] = &[nros_serdes::schema::Field {
            name: "data",
            ty: nros_serdes::schema::FieldType::Uint8,
            offset: 0,
        }];
    }

    fn s(v: &str) -> heapless::String<64> {
        heapless::String::try_from(v).unwrap()
    }

    #[test]
    fn publisher_builder_typed_and_generic() {
        let mut session = MockSession::new();
        let mut node = NodeHandle::new(s("n"), s("/"), &mut session, 0);

        // typed: node.publisher(t).typed::<M>().qos(..).build()
        let _typed = node
            .publisher("/chatter")
            .typed::<TestMsg>()
            .qos(QoSProfile::default().keep_last(5))
            .build()
            .expect("typed publisher builds");

        // generic: node.publisher(t).qos(..).generic(type, hash).build()
        let _generic = node
            .publisher("/chatter")
            .qos(QoSProfile::default())
            .generic("std_msgs/msg/Int32", "hash")
            .build()
            .expect("generic publisher builds");
    }

    #[test]
    fn subscription_builder_and_convenient() {
        let mut exec: Executor = Executor::from_session(MockSession::new());
        let id = exec.node_builder("n").build().expect("node");

        // builder: typed
        let _h = exec
            .node_mut(id)
            .subscription("/chatter")
            .typed::<TestMsg>()
            .qos(QoSProfile::default().keep_last(5))
            .build(|_m: &TestMsg| {})
            .expect("typed subscription builds");

        // builder: generic (raw bytes)
        let _g = exec
            .node_mut(id)
            .subscription("/raw")
            .generic("std_msgs/msg/Int32", "hash")
            .build(|_b: &[u8]| {})
            .expect("generic subscription builds");

        // builder: sized + sched-context (slice 3 knobs)
        let sc = exec.default_sched_context_id();
        let _s = exec
            .node_mut(id)
            .subscription("/sized")
            .typed::<TestMsg>()
            .rx_buffer::<64>()
            .sched_context(sc)
            .build(|_m: &TestMsg| {})
            .expect("sized + sched subscription builds");

        // convenient (fork tier) — one node-ctx at a time, re-acquired
        let _c = exec
            .node_mut(id)
            .create_subscription::<TestMsg, _>("/conv", |_m: &TestMsg| {})
            .expect("convenient typed subscription builds");
    }

    #[test]
    fn generic_message_info_builder() {
        // slice 3b — the bridge echo path: generic sub whose callback
        // receives the wire attachment via RawMessageInfo.
        let mut exec: Executor = Executor::from_session(MockSession::new());
        let id = exec.node_builder("n").build().expect("node");

        let _i = exec
            .node_mut(id)
            .subscription("/info")
            .generic("std_msgs/msg/Int32", "hash")
            .message_info()
            .rx_buffer::<256>()
            .build(|_payload: &[u8], info: &nros_core::RawMessageInfo| {
                let _ = info.attachment();
            })
            .expect("generic + message_info subscription builds");
    }

    #[test]
    fn typed_message_info_builder() {
        // M2.a — typed .message_info() (rclrs shape FnMut(&M, Option<&MessageInfo>)),
        // replacing register_subscription_with_info.
        let mut exec: Executor = Executor::from_session(MockSession::new());
        let id = exec.node_builder("n").build().expect("node");
        let _h = exec
            .node_mut(id)
            .subscription("/chatter")
            .typed::<TestMsg>()
            .qos(QoSProfile::default().keep_last(5))
            .message_info()
            .build(|_m: &TestMsg, _info: Option<&nros_core::MessageInfo>| {})
            .expect("typed + message_info subscription builds");
    }

    #[cfg(feature = "safety-e2e")]
    #[test]
    fn typed_safety_builder() {
        // M2.a — typed .safety(), replacing register_subscription_with_safety.
        let mut exec: Executor = Executor::from_session(MockSession::new());
        let id = exec.node_builder("n").build().expect("node");
        let _h = exec
            .node_mut(id)
            .subscription("/chatter")
            .typed::<TestMsg>()
            .safety()
            .build(|_m: &TestMsg, _status: &nros_rmw::IntegrityStatus| {})
            .expect("typed + safety subscription builds");
    }

    #[test]
    fn generator_emitted_chain_compiles() {
        // Locks the exact builder chain the orchestration generator emits
        // for a subscriber (replaces register_subscription_raw_with_qos_sized_on).
        let mut exec: Executor = Executor::from_session(MockSession::new());
        let id = exec.node_builder("n").build().expect("node");
        let _h = exec
            .node_mut(id)
            .subscription("/topic")
            .generic("std_msgs/msg/Int32", "hash")
            .qos(QoSProfile::default().keep_last(1))
            .rx_buffer::<1024>()
            .build(|_data: &[u8]| {})
            .expect("generator-shape subscription builds");
    }

    #[test]
    fn nodectx_publisher_and_bridge_shape() {
        // NodeCtx publisher symmetry + the bridge two-ctx borrow pattern:
        // build the dest publisher on one NodeCtx (dropped), then register
        // the source subscription on another — the owned publisher outlives.
        let mut exec: Executor = Executor::from_session(MockSession::new());
        let id = exec.node_builder("n").build().expect("node");

        // convenient + builder publisher on NodeCtx
        let _p = exec
            .node_mut(id)
            .create_publisher::<TestMsg>("/p")
            .expect("ctx convenient publisher");
        let dest_pub = exec
            .node_mut(id)
            .publisher("/fwd")
            .generic("std_msgs/msg/Int32", "hash")
            .build()
            .expect("ctx generic publisher builds"); // NodeCtx dropped here

        // re-borrow exec for the source sub; closure owns dest_pub
        let _s = exec
            .node_mut(id)
            .subscription("/src")
            .generic("std_msgs/msg/Int32", "hash")
            .message_info()
            .build(move |payload: &[u8], _info: &nros_core::RawMessageInfo| {
                let _ = dest_pub.publish_raw(payload);
            })
            .expect("bridge-shape source subscription builds");
    }
}
