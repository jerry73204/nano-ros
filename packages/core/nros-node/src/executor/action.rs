//! Action server and client registration on the executor and handle types.

use core::marker::PhantomData;

use nros_core::RosAction;
use nros_rmw::{ActionInfo, QoSProfile, ServiceInfo, Session, TopicInfo};

#[allow(unused_imports)]
use crate::rmw_type_registry::{MessageForRmw, register_type};

use super::{
    action_core::{ActionClientCore, ActionServerCore, RawActiveGoal},
    arena::{
        ActionClientCallbackEntry, ActionClientRawArenaEntry, ActionServerArenaEntry,
        ActionServerRawArenaEntry, BufferStrategy, CallbackMeta, EntryKind, TraceName,
        action_client_callback_try_process, action_client_raw_try_process,
        action_server_raw_try_process, action_server_try_process, always_ready,
        as_active_goal_count, as_complete_goal, as_for_each_active_goal, as_publish_feedback,
        as_raw_active_goal_count, as_raw_complete_goal, as_raw_for_each_active_goal,
        as_raw_publish_feedback, as_raw_set_goal_status, as_set_goal_status, buffered_region_size,
        drop_entry, no_pre_sample,
    },
    handles::{ActionServer, ActiveGoal},
    spin::Executor,
    spsc_ring::SpscRing,
    triple_buffer::TripleBuffer,
    types::{
        HandleId, InvocationMode, NodeError, RawAcceptedCallback, RawCancelCallback,
        RawFeedbackCallback, RawGoalCallback, RawGoalResponseCallback, RawResultCallback,
    },
};

/// phase-392 W5.b2 — how many zenoh queryables ONE action server costs.
///
/// An action is three services on the wire (`send_goal`, `cancel_goal`,
/// `get_result`; the feedback and status channels are topics, not queryables),
/// so a launch file declaring one action server is declaring THREE entries in
/// the backend's queryable table. A consumer sizing that table from a model
/// must multiply, and this is the multiplier.
///
/// Defined here rather than in the consumer for the reason issue 0827 measured:
/// a count restated where it cannot be derived drifts from the code that
/// decides it. `check-infra-queryable-counts` ties this to the distinct
/// `create_service` calls below, so adding a fourth action channel fails the
/// gate instead of silently under-sizing every image that declares an action.
pub const ACTION_SERVER_QUERYABLES: usize = 3;

/// Publishers an action SERVER creates, per action, beyond anything the author
/// declares: the feedback topic and the status topic (`create_publisher` calls
/// below). A consumer sizing the backend's publisher table from a declaration
/// must add these, because an `ENTITIES action_server:...` spec declares ONE
/// entity and costs two publisher slots.
///
/// Same rule as [`ACTION_SERVER_QUERYABLES`], and defined for the same reason:
/// the number lives next to the calls that decide it, so it cannot drift from
/// them without failing the gate that ties the two together.
pub const ACTION_SERVER_PUBLISHERS: usize = 2;

/// Subscriptions an action CLIENT creates, per action: the feedback topic.
/// Status is polled through the result client rather than subscribed, so this
/// is one and not two -- check the `create_subscription` calls below before
/// changing it, not this comment.
///
/// The same under-sizing hazard as the two above: `ENTITIES action_client:...`
/// is one declared entity and one subscriber slot that nothing else accounts
/// for.
pub const ACTION_CLIENT_SUBSCRIPTIONS: usize = 1;

// ============================================================================
// Raw action registration specs
// ============================================================================

/// Inputs for raw (untyped) action-server registration.
///
/// Collapses the runtime arguments shared by the
/// `register_action_server_raw*` family. Buffer sizes / max-goals stay
/// as const-generic turbofish parameters on the registration methods.
pub struct RawActionServerSpec<'a> {
    /// `None` registers on the executor's own node; `Some(id)` routes
    /// the server's 5 underlying handles (send_goal / cancel_goal /
    /// get_result servers + feedback / status publishers) through the
    /// named Node's session.
    pub node_id: Option<super::node_record::NodeId>,
    pub action_name: &'a str,
    pub type_name: &'a str,
    pub type_hash: &'a str,
    /// QoS for the action's three underlying service servers (send_goal
    /// / cancel_goal / get_result; Phase 193.4b). The feedback + status
    /// publishers keep their own profiles. Use
    /// [`QoSProfile::services_default`] for the rclc-compatible default.
    pub qos: QoSProfile,
    pub goal_callback: RawGoalCallback,
    pub cancel_callback: RawCancelCallback,
    pub accepted_callback: Option<RawAcceptedCallback>,
    pub context: *mut core::ffi::c_void,
}

/// Inputs for raw (untyped) action-client registration.
///
/// Collapses the runtime arguments shared by the
/// `register_action_client_raw*` family. Buffer sizes stay as
/// const-generic turbofish parameters on the registration methods.
pub struct RawActionClientSpec<'a> {
    /// `None` registers on the executor's own node; `Some(id)` routes
    /// the client's 4 underlying handles (send_goal / cancel_goal /
    /// get_result service clients + feedback subscriber) through the
    /// named Node's session.
    pub node_id: Option<super::node_record::NodeId>,
    pub action_name: &'a str,
    pub type_name: &'a str,
    pub type_hash: &'a str,
    pub goal_response_callback: Option<RawGoalResponseCallback>,
    pub feedback_callback: Option<RawFeedbackCallback>,
    pub result_callback: Option<RawResultCallback>,
    pub context: *mut core::ffi::c_void,
}

// ============================================================================
// Action server registration
// ============================================================================

impl<'s> Executor<'s> {
    /// Register an action server with goal/cancel callbacks.
    ///
    /// The executor automatically dispatches:
    /// - Goal acceptance via `goal_callback`
    /// - Cancel requests via `cancel_callback`
    /// - Result serving for completed goals
    ///
    /// Use the returned [`ActionServerHandle`] to publish feedback and complete goals.
    ///
    /// Uses default buffer sizes and max 4 concurrent goals.
    pub fn register_action_server<A, GoalF, CancelF>(
        &mut self,
        action_name: &str,
        goal_callback: GoalF,
        cancel_callback: CancelF,
    ) -> Result<ActionServerHandle<A>, NodeError>
    where
        A: RosAction + 'static,
        A::Goal: Clone + MessageForRmw,
        A::Result: Clone + Default + MessageForRmw,
        A::Feedback: MessageForRmw,
        A::SendGoalRequest: MessageForRmw,
        A::SendGoalResponse: MessageForRmw,
        A::GetResultRequest: MessageForRmw,
        A::GetResultResponse: MessageForRmw,
        A::FeedbackMessage: MessageForRmw,
        GoalF: FnMut(&nros_core::GoalId, &A::Goal) -> nros_core::GoalResponse + 'static,
        CancelF:
            FnMut(&nros_core::GoalId, nros_core::GoalStatus) -> nros_core::CancelResponse + 'static,
    {
        self.register_action_server_sized::<A, GoalF, CancelF, { crate::config::DEFAULT_RX_BUF_SIZE }, { crate::config::DEFAULT_RX_BUF_SIZE }, { crate::config::DEFAULT_RX_BUF_SIZE }, 4>(
            action_name,
            goal_callback,
            cancel_callback,
        )
    }

    /// Register an action server with custom buffer sizes.
    pub fn register_action_server_sized<
        A,
        GoalF,
        CancelF,
        const GOAL_BUF: usize,
        const RESULT_BUF: usize,
        const FEEDBACK_BUF: usize,
        const MAX_GOALS: usize,
    >(
        &mut self,
        action_name: &str,
        goal_callback: GoalF,
        cancel_callback: CancelF,
    ) -> Result<ActionServerHandle<A>, NodeError>
    where
        A: RosAction + 'static,
        A::Goal: Clone + MessageForRmw,
        A::Result: Clone + Default + MessageForRmw,
        A::Feedback: MessageForRmw,
        A::SendGoalRequest: MessageForRmw,
        A::SendGoalResponse: MessageForRmw,
        A::GetResultRequest: MessageForRmw,
        A::GetResultResponse: MessageForRmw,
        A::FeedbackMessage: MessageForRmw,
        GoalF: FnMut(&nros_core::GoalId, &A::Goal) -> nros_core::GoalResponse + 'static,
        CancelF:
            FnMut(&nros_core::GoalId, nros_core::GoalStatus) -> nros_core::CancelResponse + 'static,
    {
        // Phase 212.K.7.6.b + K.7.7.c — under `rmw-cyclonedds`, register
        // the user-facing message types AND the five action-protocol
        // envelope types with the cyclonedds runtime registry before
        // creating the underlying service / topic entities. No-op for
        // other RMWs. See `Node::create_action_server_sized` for the
        // detailed rationale.
        register_type::<A::Goal>()?;
        register_type::<A::Result>()?;
        register_type::<A::Feedback>()?;
        register_type::<A::SendGoalRequest>()?;
        register_type::<A::SendGoalResponse>()?;
        register_type::<A::GetResultRequest>()?;
        register_type::<A::GetResultResponse>()?;
        register_type::<A::FeedbackMessage>()?;
        // Phase 244 E3 (RFC-0044) — register the fixed `action_msgs` protocol
        // types (CancelGoal_{Request,Response}, GoalStatusArray) the cancel /
        // status plumbing serializes. The generated `impl RosAction` overrides
        // this (default = no-op); previously every example hand-registered these
        // three under `#[cfg(feature = "rmw-cyclonedds")]`.
        A::register_protocol_types().map_err(|()| NodeError::ActionCreationFailed)?;
        type Entry<
            A,
            GoalF,
            CancelF,
            const GB: usize,
            const RB: usize,
            const FB: usize,
            const MG: usize,
        > = ActionServerArenaEntry<A, GoalF, CancelF, GB, RB, FB, MG>;

        let slot = self.next_entry_slot()?;

        // Create the action server entities (same logic as Node::create_action_server_sized)
        let action_info = ActionInfo::new(action_name, A::ACTION_NAME, A::ACTION_HASH);

        // ROS 2 matches the action's send_goal / get_result services by their
        // per-channel service types (`<Action>_SendGoal` / `<Action>_GetResult`)
        // and the feedback topic by `<Action>_FeedbackMessage` — not the bare
        // action type. Pass those so a real `rcl_action` peer discovers us.
        let send_goal_type = super::action_core::action_service_base_type(
            <A::SendGoalRequest as nros_core::RosMessage>::TYPE_NAME,
            A::ACTION_NAME,
        );
        let get_result_type = super::action_core::action_service_base_type(
            <A::GetResultRequest as nros_core::RosMessage>::TYPE_NAME,
            A::ACTION_NAME,
        );
        let feedback_type = <A::FeedbackMessage as nros_core::RosMessage>::TYPE_NAME;

        let send_goal_keyexpr: heapless::String<256> = action_info.send_goal_key();
        let send_goal_info = ServiceInfo::new(
            &send_goal_keyexpr,
            send_goal_type,
            A::SEND_GOAL_SERVICE_HASH,
        )
        .with_domain(self.domain_id);
        let send_goal_server = self
            .session
            .create_service(&send_goal_info, QoSProfile::services_default())
            .map_err(NodeError::Transport)?;

        let cancel_goal_keyexpr: heapless::String<256> = action_info.cancel_goal_key();
        let cancel_goal_info = ServiceInfo::new(
            &cancel_goal_keyexpr,
            "action_msgs::srv::dds_::CancelGoal_",
            A::ACTION_HASH,
        )
        .with_domain(self.domain_id);
        let cancel_goal_server = self
            .session
            .create_service(&cancel_goal_info, QoSProfile::services_default())
            .map_err(NodeError::Transport)?;

        let get_result_keyexpr: heapless::String<256> = action_info.get_result_key();
        let get_result_info = ServiceInfo::new(
            &get_result_keyexpr,
            get_result_type,
            A::GET_RESULT_SERVICE_HASH,
        )
        .with_domain(self.domain_id);
        let get_result_server = self
            .session
            .create_service(&get_result_info, QoSProfile::services_default())
            .map_err(NodeError::Transport)?;

        let feedback_keyexpr: heapless::String<256> = action_info.feedback_key();
        let feedback_topic = TopicInfo::new(
            &feedback_keyexpr,
            feedback_type,
            <A::FeedbackMessage as nros_core::RosMessage>::TYPE_HASH,
        )
        .with_domain(self.domain_id);
        let feedback_publisher = self
            .session
            .create_publisher(&feedback_topic, QoSProfile::QOS_PROFILE_DEFAULT)
            .map_err(NodeError::Transport)?;

        let status_keyexpr: heapless::String<256> = action_info.status_key();
        let status_topic = TopicInfo::new(
            &status_keyexpr,
            "action_msgs::msg::dds_::GoalStatusArray_",
            A::ACTION_HASH,
        )
        .with_domain(self.domain_id);
        let status_publisher = self
            .session
            .create_publisher(&status_topic, QoSProfile::QOS_PROFILE_ACTION_STATUS_DEFAULT)
            .map_err(NodeError::Transport)?;

        let server = ActionServer {
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
        };

        let offset = self
            .arena_alloc::<Entry<A, GoalF, CancelF, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>>(
            )?;

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset)
                as *mut Entry<A, GoalF, CancelF, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>;
            core::ptr::write(
                entry_ptr,
                Entry {
                    server,
                    goal_callback,
                    cancel_callback,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::ActionServer,
            has_data: always_ready,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::Always,
            try_process: action_server_try_process::<
                A,
                GoalF,
                CancelF,
                GOAL_BUF,
                RESULT_BUF,
                FEEDBACK_BUF,
                MAX_GOALS,
            >,
            drop_fn: drop_entry::<
                Entry<A, GoalF, CancelF, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>,
            >,
        };
        self.emplace_entry(slot, meta, TraceName::Text(action_name));

        Ok(ActionServerHandle {
            entry_index: slot,
            publish_feedback_fn: as_publish_feedback::<
                A,
                GoalF,
                CancelF,
                GOAL_BUF,
                RESULT_BUF,
                FEEDBACK_BUF,
                MAX_GOALS,
            >,
            complete_goal_fn: as_complete_goal::<
                A,
                GoalF,
                CancelF,
                GOAL_BUF,
                RESULT_BUF,
                FEEDBACK_BUF,
                MAX_GOALS,
            >,
            set_goal_status_fn: as_set_goal_status::<
                A,
                GoalF,
                CancelF,
                GOAL_BUF,
                RESULT_BUF,
                FEEDBACK_BUF,
                MAX_GOALS,
            >,
            active_goal_count_fn: as_active_goal_count::<
                A,
                GoalF,
                CancelF,
                GOAL_BUF,
                RESULT_BUF,
                FEEDBACK_BUF,
                MAX_GOALS,
            >,
            for_each_active_goal_fn: as_for_each_active_goal::<
                A,
                GoalF,
                CancelF,
                GOAL_BUF,
                RESULT_BUF,
                FEEDBACK_BUF,
                MAX_GOALS,
            >,
            _phantom: PhantomData,
        })
    }
}

// ============================================================================
// Handle types for arena-registered action server
// ============================================================================

/// Handle to an action server registered in the executor's arena.
///
/// Returned by [`Executor::register_action_server()`]. Provides methods
/// to interact with the server (publish feedback, complete goals) while the
/// executor automatically handles goal acceptance, cancel requests, and
/// result serving during [`spin_once()`](Executor::spin_once).
#[allow(clippy::type_complexity)]
pub struct ActionServerHandle<A: RosAction> {
    pub(crate) entry_index: usize,
    publish_feedback_fn:
        unsafe fn(*mut u8, &nros_core::GoalId, &A::Feedback) -> Result<(), NodeError>,
    complete_goal_fn: unsafe fn(
        *mut u8,
        &nros_core::GoalId,
        nros_core::GoalStatus,
        A::Result,
    ) -> Result<(), NodeError>,
    set_goal_status_fn: unsafe fn(*mut u8, &nros_core::GoalId, nros_core::GoalStatus),
    active_goal_count_fn: unsafe fn(*const u8) -> usize,
    for_each_active_goal_fn: unsafe fn(*const u8, &mut dyn FnMut(&ActiveGoal<A>)),
    _phantom: PhantomData<A>,
}

impl<A: RosAction> Clone for ActionServerHandle<A> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<A: RosAction> Copy for ActionServerHandle<A> {}

impl<A: RosAction> ActionServerHandle<A> {
    /// Get the [`HandleId`] for this action server.
    ///
    /// Used with `Trigger::One` or `HandleSet` for trigger configuration.
    pub fn handle_id(&self) -> HandleId {
        HandleId(self.entry_index)
    }

    /// Publish feedback for an active goal.
    ///
    /// Serialises the feedback message and sends it to all clients
    /// monitoring this goal. Returns an error if the handle slot has
    /// been removed from the executor.
    pub fn publish_feedback(
        &self,
        executor: &mut Executor,
        goal_id: &nros_core::GoalId,
        feedback: &A::Feedback,
    ) -> Result<(), NodeError> {
        let meta = executor.entries[self.entry_index]
            .as_ref()
            .ok_or(NodeError::BufferTooSmall)?;
        let arena_ptr = executor.arena.as_mut_ptr() as *mut u8;
        unsafe {
            let data_ptr = arena_ptr.add(meta.offset);
            (self.publish_feedback_fn)(data_ptr, goal_id, feedback)
        }
    }

    /// Complete a goal with a terminal status and result payload.
    ///
    /// The goal is moved from the active set to the completed-results
    /// slab. Clients waiting on a result will receive the response.
    /// `status` should be one of `Succeeded`, `Aborted`, or `Canceled`.
    ///
    /// # Errors
    ///
    /// `NodeError::BufferTooSmall` when the handle slot has been removed from
    /// the executor, or when the serialized result exceeds the server's
    /// `RESULT_BUF` and so cannot be retained for a later `get_result` (issue
    /// 0796 — this used to return `()` and swallow both).
    /// Terminate `goal_id` as SUCCEEDED — phase-379 W5.
    ///
    /// The three terminal verbs (`succeed` / `abort` / `canceled`) name the
    /// ACTION SPEC's terminal states, which is also what rclcpp_action's
    /// `ServerGoalHandle` spells them: `succeed(result)`, `abort(result)`,
    /// `canceled(result)` — and, since phase-417 W4.b, what C
    /// (`nros_action_canceled`) and C++ (`ActionServer<A>::canceled`) spell
    /// them too. They take the goal ID rather than hanging off a
    /// handle because a goal here lives in a fixed-capacity arena and is named
    /// by its UUID — see the `divergence` row for why neither client library's
    /// ownership model is available without an allocator.
    ///
    /// `complete_goal` remains the general form for a status computed at
    /// runtime.
    pub fn succeed(
        &self,
        executor: &mut Executor,
        goal_id: &nros_core::GoalId,
        result: A::Result,
    ) -> Result<(), NodeError> {
        self.complete_goal(executor, goal_id, nros_core::GoalStatus::Succeeded, result)
    }

    /// Terminate `goal_id` as ABORTED. See [`Self::succeed`].
    pub fn abort(
        &self,
        executor: &mut Executor,
        goal_id: &nros_core::GoalId,
        result: A::Result,
    ) -> Result<(), NodeError> {
        self.complete_goal(executor, goal_id, nros_core::GoalStatus::Aborted, result)
    }

    /// Terminate `goal_id` as CANCELED. See [`Self::succeed`].
    ///
    /// phase-417 W4.b — this was `cancel`, and the argument for that spelling
    /// is in [`Self::cancel`]. It loses to the drop-in claim: C spells it
    /// `nros_action_canceled`, rclcpp_action spells it
    /// `ServerGoalHandle::canceled`, and Rust was the only one of the three
    /// disagreeing — over a naming preference with no platform reason behind
    /// it (RFC-0089: a rename is cheap, a surface our own languages disagree
    /// about is not).
    pub fn canceled(
        &self,
        executor: &mut Executor,
        goal_id: &nros_core::GoalId,
        result: A::Result,
    ) -> Result<(), NodeError> {
        self.complete_goal(executor, goal_id, nros_core::GoalStatus::Canceled, result)
    }

    /// Deprecated spelling of [`Self::canceled`] — phase-417 W4.b.
    ///
    /// The case for `cancel` was internal consistency: the other two verbs are
    /// imperatives (`succeed`, `abort`), and mixing an imperative with a past
    /// participle inside one family reads as an accident. Real, and outweighed
    /// by C, C++ and rclcpp_action all saying `canceled`.
    ///
    /// A forwarder rather than a hard removal because these are INHERENT
    /// methods: a caller updates one call site and nothing else observes the
    /// change. (A trait-method rename gets the opposite treatment — it breaks
    /// every implementor, so there a hard error is the honest signal.)
    #[deprecated(
        since = "0.1.0",
        note = "renamed to `canceled` to match C's `nros_action_canceled` and rclcpp_action's \
                `ServerGoalHandle::canceled` (phase-417 W4.b)"
    )]
    pub fn cancel(
        &self,
        executor: &mut Executor,
        goal_id: &nros_core::GoalId,
        result: A::Result,
    ) -> Result<(), NodeError> {
        self.canceled(executor, goal_id, result)
    }

    pub fn complete_goal(
        &self,
        executor: &mut Executor,
        goal_id: &nros_core::GoalId,
        status: nros_core::GoalStatus,
        result: A::Result,
    ) -> Result<(), NodeError> {
        let meta = executor.entries[self.entry_index]
            .as_ref()
            .ok_or(NodeError::BufferTooSmall)?;
        let arena_ptr = executor.arena.as_mut_ptr() as *mut u8;
        unsafe {
            let data_ptr = arena_ptr.add(meta.offset);
            (self.complete_goal_fn)(data_ptr, goal_id, status, result)
        }
    }

    /// Update a goal's status without completing it.
    ///
    /// Use this to transition a goal to `Executing` or `Canceling`
    /// while it is still active. To finish a goal, use [`complete_goal`](Self::complete_goal).
    pub fn set_goal_status(
        &self,
        executor: &mut Executor,
        goal_id: &nros_core::GoalId,
        status: nros_core::GoalStatus,
    ) {
        if let Some(meta) = executor.entries[self.entry_index].as_ref() {
            let arena_ptr = executor.arena.as_mut_ptr() as *mut u8;
            unsafe {
                let data_ptr = arena_ptr.add(meta.offset);
                (self.set_goal_status_fn)(data_ptr, goal_id, status);
            }
        }
    }

    /// Get the number of currently active goals.
    ///
    /// Returns 0 if the action server handle has been removed from the executor.
    pub fn active_goal_count(&self, executor: &Executor) -> usize {
        match executor.entries[self.entry_index].as_ref() {
            Some(meta) => {
                let arena_ptr = executor.arena.as_ptr() as *const u8;
                unsafe {
                    let data_ptr = arena_ptr.add(meta.offset);
                    (self.active_goal_count_fn)(data_ptr)
                }
            }
            None => 0,
        }
    }

    /// Iterate over all currently active goals.
    ///
    /// Calls `f` for each goal that has been accepted but not yet
    /// completed. Useful for monitoring progress or canceling stale goals.
    pub fn for_each_active_goal(&self, executor: &Executor, mut f: impl FnMut(&ActiveGoal<A>)) {
        if let Some(meta) = executor.entries[self.entry_index].as_ref() {
            let arena_ptr = executor.arena.as_ptr() as *const u8;
            unsafe {
                let data_ptr = arena_ptr.add(meta.offset);
                (self.for_each_active_goal_fn)(data_ptr, &mut f);
            }
        }
    }
}

// ============================================================================
// Raw (untyped) action server registration
// ============================================================================

impl<'s> Executor<'s> {
    /// Register a raw action server with raw-bytes callbacks.
    ///
    /// Unlike [`register_action_server()`](Executor::register_action_server), this does
    /// not require `RosAction` — the goal/cancel callbacks receive raw CDR
    /// bytes. This is used by the C API thin wrapper.
    ///
    /// `type_name` and `type_hash` identify the action type for key expression
    /// construction and liveliness tokens.
    #[allow(clippy::too_many_arguments)]
    pub fn register_action_server_raw(
        &mut self,
        spec: RawActionServerSpec<'_>,
    ) -> Result<ActionServerRawHandle, NodeError> {
        self.register_action_server_raw_sized::<{ crate::config::DEFAULT_RX_BUF_SIZE }, { crate::config::DEFAULT_RX_BUF_SIZE }, { crate::config::DEFAULT_RX_BUF_SIZE }, 4>(
            spec,
        )
    }

    /// Register a raw action server with custom buffer sizes.
    ///
    /// `spec.node_id` selects the target: `None` registers on the
    /// executor's own node, `Some(id)` routes the server's 5 underlying
    /// handles through the named Node's session (Phase 104.C.3.3.a).
    /// `spec.qos` applies to the action's three underlying service
    /// servers (send_goal / cancel_goal / get_result; Phase 193.4b); the
    /// feedback + status publishers keep their own profiles.
    pub fn register_action_server_raw_sized<
        const GOAL_BUF: usize,
        const RESULT_BUF: usize,
        const FEEDBACK_BUF: usize,
        const MAX_GOALS: usize,
    >(
        &mut self,
        spec: RawActionServerSpec<'_>,
    ) -> Result<ActionServerRawHandle, NodeError> {
        let RawActionServerSpec {
            node_id,
            action_name,
            type_name,
            type_hash,
            qos,
            goal_callback,
            cancel_callback,
            accepted_callback,
            context,
        } = spec;

        type Entry<const GB: usize, const RB: usize, const FB: usize, const MG: usize> =
            ActionServerRawArenaEntry<GB, RB, FB, MG>;

        let slot = self.next_entry_slot()?;

        let action_info = ActionInfo::new(action_name, type_name, type_hash);
        // Issue 0656 — capture the domain BEFORE the session borrow below, for
        // the same reason `node_name`/`ns` are cloned here: the `&mut session`
        // in the create scope would otherwise conflict with `&self`.
        let domain_id = self.domain_id;
        let (node_name, ns, session_idx) = match node_id {
            Some(id) => {
                let r = self
                    .nodes
                    .get(id.index())
                    .ok_or(NodeError::InvalidSchedContextBinding)?;
                (r.name.clone(), r.namespace.clone(), r.session_idx)
            }
            None => (self.node_name.clone(), self.namespace.clone(), 0u8),
        };

        // Thread node identity through each underlying ServiceInfo /
        // TopicInfo so the Zenoh shim declares a liveliness token for
        // each entity. Without `with_node_name`,
        // `declare_entity_liveliness` short-circuits and
        // `wait_for_action_server` has nothing to find — same fix as
        // `Node::create_action_server_sized` (commit ea5e80b4).
        // All 5 session-create calls grouped into one scope so the
        // mutable session borrow drops before arena alloc below.
        let (
            send_goal_server,
            cancel_goal_server,
            get_result_server,
            feedback_publisher,
            status_publisher,
        ) = {
            // phase-338 W3 — ROS 2 matches these by their PER-CHANNEL types, not
            // the bare action type. The typed path derives them from
            // `A::SendGoalRequest::TYPE_NAME`; the raw path has only the bare
            // type, and advertising it here left send_goal / get_result /
            // feedback undiscoverable, so every goal timed out.
            let send_goal_type: heapless::String<256> =
                super::action_core::action_channel_type(type_name, "SendGoal");
            let get_result_type: heapless::String<256> =
                super::action_core::action_channel_type(type_name, "GetResult");
            let feedback_type: heapless::String<256> =
                super::action_core::action_channel_type(type_name, "FeedbackMessage");

            let send_goal_keyexpr: heapless::String<256> = action_info.send_goal_key();
            let mut send_goal_info =
                ServiceInfo::new(&send_goal_keyexpr, &send_goal_type, type_hash)
                    .with_namespace(&ns)
                    .with_domain(domain_id);
            if !node_name.is_empty() {
                send_goal_info = send_goal_info.with_node_name(&node_name);
            }

            let cancel_goal_keyexpr: heapless::String<256> = action_info.cancel_goal_key();
            let mut cancel_goal_info = ServiceInfo::new(
                &cancel_goal_keyexpr,
                "action_msgs::srv::dds_::CancelGoal_",
                type_hash,
            )
            .with_namespace(&ns)
            .with_domain(domain_id);
            if !node_name.is_empty() {
                cancel_goal_info = cancel_goal_info.with_node_name(&node_name);
            }

            let get_result_keyexpr: heapless::String<256> = action_info.get_result_key();
            let mut get_result_info =
                ServiceInfo::new(&get_result_keyexpr, &get_result_type, type_hash)
                    .with_namespace(&ns)
                    .with_domain(domain_id);
            if !node_name.is_empty() {
                get_result_info = get_result_info.with_node_name(&node_name);
            }

            let feedback_keyexpr: heapless::String<256> = action_info.feedback_key();
            let mut feedback_topic = TopicInfo::new(&feedback_keyexpr, &feedback_type, type_hash)
                .with_namespace(&ns)
                .with_domain(domain_id);
            if !node_name.is_empty() {
                feedback_topic = feedback_topic.with_node_name(&node_name);
            }

            let status_keyexpr: heapless::String<256> = action_info.status_key();
            let mut status_topic = TopicInfo::new(
                &status_keyexpr,
                "action_msgs::msg::dds_::GoalStatusArray_",
                type_hash,
            )
            .with_namespace(&ns)
            .with_domain(domain_id);
            if !node_name.is_empty() {
                status_topic = status_topic.with_node_name(&node_name);
            }

            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            (
                session
                    .create_service(&send_goal_info, qos)
                    .map_err(NodeError::Transport)?,
                session
                    .create_service(&cancel_goal_info, qos)
                    .map_err(NodeError::Transport)?,
                session
                    .create_service(&get_result_info, qos)
                    .map_err(NodeError::Transport)?,
                session
                    .create_publisher(&feedback_topic, QoSProfile::QOS_PROFILE_DEFAULT)
                    .map_err(NodeError::Transport)?,
                session
                    .create_publisher(&status_topic, QoSProfile::QOS_PROFILE_ACTION_STATUS_DEFAULT)
                    .map_err(NodeError::Transport)?,
            )
        };

        let core = ActionServerCore {
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
        };

        let offset = self.arena_alloc::<Entry<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>>()?;

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr =
                arena_ptr.add(offset) as *mut Entry<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>;
            core::ptr::write(
                entry_ptr,
                Entry {
                    core,
                    goal_callback,
                    cancel_callback,
                    accepted_callback,
                    context,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::ActionServer,
            has_data: always_ready,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::Always,
            try_process: action_server_raw_try_process::<
                GOAL_BUF,
                RESULT_BUF,
                FEEDBACK_BUF,
                MAX_GOALS,
            >,
            drop_fn: drop_entry::<Entry<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(spec.action_name));
        self.apply_node_default_sched(slot, node_id, None);

        Ok(ActionServerRawHandle {
            entry_index: slot,
            publish_feedback_fn: as_raw_publish_feedback::<
                GOAL_BUF,
                RESULT_BUF,
                FEEDBACK_BUF,
                MAX_GOALS,
            >,
            complete_goal_fn: as_raw_complete_goal::<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>,
            set_goal_status_fn: as_raw_set_goal_status::<
                GOAL_BUF,
                RESULT_BUF,
                FEEDBACK_BUF,
                MAX_GOALS,
            >,
            active_goal_count_fn: as_raw_active_goal_count::<
                GOAL_BUF,
                RESULT_BUF,
                FEEDBACK_BUF,
                MAX_GOALS,
            >,
            for_each_active_goal_fn: as_raw_for_each_active_goal::<
                GOAL_BUF,
                RESULT_BUF,
                FEEDBACK_BUF,
                MAX_GOALS,
            >,
        })
    }
}

// ============================================================================
// Raw action server handle
// ============================================================================

/// Handle to a raw (untyped) action server registered in the executor's arena.
///
/// Returned by [`Executor::register_action_server_raw()`]. Provides methods
/// to interact with the server using raw CDR bytes.
#[repr(C)]
#[allow(clippy::type_complexity)]
pub struct ActionServerRawHandle {
    pub(crate) entry_index: usize,
    publish_feedback_fn:
        unsafe fn(*mut u8, &nros_core::GoalId, *const u8, usize) -> Result<(), NodeError>,
    complete_goal_fn: unsafe fn(
        *mut u8,
        &nros_core::GoalId,
        nros_core::GoalStatus,
        *const u8,
        usize,
    ) -> Result<(), NodeError>,
    set_goal_status_fn: unsafe fn(*mut u8, &nros_core::GoalId, nros_core::GoalStatus),
    active_goal_count_fn: unsafe fn(*const u8) -> usize,
    for_each_active_goal_fn: unsafe fn(*const u8, &mut dyn FnMut(&RawActiveGoal)),
}

impl Clone for ActionServerRawHandle {
    fn clone(&self) -> Self {
        *self
    }
}

impl Copy for ActionServerRawHandle {}

/// Sentinel value indicating an `ActionServerRawHandle` is not bound to an
/// arena entry yet. Used by Phase 87.5 to replace `Option<...>` with a
/// `#[repr(C)]`-compatible inline field.
///
/// Function pointers are populated with `unreachable_*` stubs that panic
/// if anyone is reckless enough to dispatch through an unbound handle —
/// callers must check `entry_index == INVALID_ENTRY_INDEX` first.
pub const INVALID_ENTRY_INDEX: usize = usize::MAX;

impl ActionServerRawHandle {
    /// Construct a sentinel handle representing "not registered yet".
    ///
    /// All function pointers are unreachable stubs; only valid use is
    /// to populate `#[repr(C)]` storage that is later overwritten by a
    /// real handle (or queried via `is_invalid()` to skip operations).
    pub const fn invalid() -> Self {
        unsafe fn unreachable_publish_feedback(
            _: *mut u8,
            _: &nros_core::GoalId,
            _: *const u8,
            _: usize,
        ) -> Result<(), NodeError> {
            unreachable!("ActionServerRawHandle::publish_feedback called on invalid handle")
        }
        unsafe fn unreachable_complete_goal(
            _: *mut u8,
            _: &nros_core::GoalId,
            _: nros_core::GoalStatus,
            _: *const u8,
            _: usize,
        ) -> Result<(), NodeError> {
            unreachable!("ActionServerRawHandle::complete_goal called on invalid handle")
        }
        unsafe fn unreachable_set_goal_status(
            _: *mut u8,
            _: &nros_core::GoalId,
            _: nros_core::GoalStatus,
        ) {
            unreachable!("ActionServerRawHandle::set_goal_status called on invalid handle")
        }
        unsafe fn unreachable_active_goal_count(_: *const u8) -> usize {
            unreachable!("ActionServerRawHandle::active_goal_count called on invalid handle")
        }
        unsafe fn unreachable_for_each_active_goal(
            _: *const u8,
            _: &mut dyn FnMut(&RawActiveGoal),
        ) {
            unreachable!("ActionServerRawHandle::for_each_active_goal called on invalid handle")
        }
        Self {
            entry_index: INVALID_ENTRY_INDEX,
            publish_feedback_fn: unreachable_publish_feedback,
            complete_goal_fn: unreachable_complete_goal,
            set_goal_status_fn: unreachable_set_goal_status,
            active_goal_count_fn: unreachable_active_goal_count,
            for_each_active_goal_fn: unreachable_for_each_active_goal,
        }
    }

    /// `true` if this handle is the sentinel returned by `Self::invalid()`.
    pub const fn is_invalid(&self) -> bool {
        self.entry_index == INVALID_ENTRY_INDEX
    }
}

impl Default for ActionServerRawHandle {
    fn default() -> Self {
        Self::invalid()
    }
}

impl ActionServerRawHandle {
    /// Get the [`HandleId`] for this action server.
    pub fn handle_id(&self) -> HandleId {
        HandleId(self.entry_index)
    }

    /// Publish feedback with raw CDR bytes (untyped variant).
    ///
    /// Used by the C API when feedback is already serialised.
    pub fn publish_feedback_raw(
        &self,
        executor: &mut Executor,
        goal_id: &nros_core::GoalId,
        feedback_data: &[u8],
    ) -> Result<(), NodeError> {
        let meta = executor.entries[self.entry_index]
            .as_ref()
            .ok_or(NodeError::BufferTooSmall)?;
        let arena_ptr = executor.arena.as_mut_ptr() as *mut u8;
        unsafe {
            let data_ptr = arena_ptr.add(meta.offset);
            (self.publish_feedback_fn)(
                data_ptr,
                goal_id,
                feedback_data.as_ptr(),
                feedback_data.len(),
            )
        }
    }

    /// Complete a goal with raw CDR result bytes (untyped variant).
    ///
    /// Moves the goal from the active set to the completed-results slab.
    ///
    /// # Errors
    ///
    /// `NodeError::BufferTooSmall` when the handle slot has been removed from
    /// the executor, or when `result_data` exceeds the server's `RESULT_BUF`
    /// and so cannot be retained for a later `get_result` (issue 0796 — this
    /// used to return `()` and swallow both).
    pub fn complete_goal_raw(
        &self,
        executor: &mut Executor,
        goal_id: &nros_core::GoalId,
        status: nros_core::GoalStatus,
        result_data: &[u8],
    ) -> Result<(), NodeError> {
        let meta = executor.entries[self.entry_index]
            .as_ref()
            .ok_or(NodeError::BufferTooSmall)?;
        let arena_ptr = executor.arena.as_mut_ptr() as *mut u8;
        unsafe {
            let data_ptr = arena_ptr.add(meta.offset);
            (self.complete_goal_fn)(
                data_ptr,
                goal_id,
                status,
                result_data.as_ptr(),
                result_data.len(),
            )
        }
    }

    /// Update a goal's status without completing it.
    ///
    /// Use this to transition a goal to `Executing` or `Canceling`
    /// while it is still active. To finish a goal, use [`complete_goal_raw`](Self::complete_goal_raw).
    pub fn set_goal_status(
        &self,
        executor: &mut Executor,
        goal_id: &nros_core::GoalId,
        status: nros_core::GoalStatus,
    ) {
        if let Some(meta) = executor.entries[self.entry_index].as_ref() {
            let arena_ptr = executor.arena.as_mut_ptr() as *mut u8;
            unsafe {
                let data_ptr = arena_ptr.add(meta.offset);
                (self.set_goal_status_fn)(data_ptr, goal_id, status);
            }
        }
    }

    /// Get the number of currently active goals.
    ///
    /// Returns 0 if the action server handle has been removed from the executor.
    pub fn active_goal_count(&self, executor: &Executor) -> usize {
        match executor.entries[self.entry_index].as_ref() {
            Some(meta) => {
                let arena_ptr = executor.arena.as_ptr() as *const u8;
                unsafe {
                    let data_ptr = arena_ptr.add(meta.offset);
                    (self.active_goal_count_fn)(data_ptr)
                }
            }
            None => 0,
        }
    }

    /// Iterate over all currently active goals (raw/untyped variant).
    ///
    /// Calls `f` for each goal that has been accepted but not yet completed.
    pub fn for_each_active_goal(&self, executor: &Executor, mut f: impl FnMut(&RawActiveGoal)) {
        if let Some(meta) = executor.entries[self.entry_index].as_ref() {
            let arena_ptr = executor.arena.as_ptr() as *const u8;
            unsafe {
                let data_ptr = arena_ptr.add(meta.offset);
                (self.for_each_active_goal_fn)(data_ptr, &mut f);
            }
        }
    }

    /// Look up the status of a single active goal by UUID.
    ///
    /// Returns `Some(status)` while the goal is still in the arena's
    /// `active_goals` vector. Returns `None` once the goal has been
    /// retired (completed + result delivered, or cancelled + acknowledged).
    ///
    /// This is the authoritative source of goal status — the C/C++ FFI
    /// layers call this from `nros_action_get_goal_status` rather than
    /// reading a cached field on their own handle structs.
    pub fn goal_status(
        &self,
        executor: &Executor,
        goal_id: &nros_core::GoalId,
    ) -> Option<nros_core::GoalStatus> {
        let mut found = None;
        self.for_each_active_goal(executor, |g| {
            if g.goal_id.uuid == goal_id.uuid && found.is_none() {
                found = Some(g.status);
            }
        });
        found
    }
}

// ============================================================================
// Action client registration
// ============================================================================

impl<'s> Executor<'s> {
    /// Register a raw action client with the executor.
    ///
    /// Creates service clients for send_goal, cancel_goal, get_result, and a
    /// feedback subscriber. The executor polls these during `spin_once` and
    /// invokes the provided callbacks when responses/feedback arrive.
    ///
    /// # Arguments
    /// * `action_name` — action name (e.g., "/fibonacci")
    /// * `type_name` — action type (e.g., "example_interfaces::action::dds_::Fibonacci_")
    /// * `type_hash` — type hash (e.g., "TypeHashNotSupported")
    /// * `goal_response_callback` — called when goal is accepted/rejected
    /// * `feedback_callback` — called when feedback is received
    /// * `result_callback` — called when result is received
    /// * `context` — opaque pointer passed to all callbacks
    #[allow(clippy::too_many_arguments)]
    pub fn register_action_client_raw(
        &mut self,
        spec: RawActionClientSpec<'_>,
    ) -> Result<ActionClientRawHandle, NodeError> {
        self.register_action_client_raw_sized::<
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
            { crate::config::DEFAULT_RX_BUF_SIZE },
        >(spec)
    }

    /// Register a raw action client with explicit buffer sizes.
    ///
    /// `spec.node_id` selects the target: `None` registers on the
    /// executor's own node, `Some(id)` routes the client's 4 underlying
    /// handles through the named Node's session (Phase 104.C.3.3.a).
    pub fn register_action_client_raw_sized<
        const GOAL_BUF: usize,
        const RESULT_BUF: usize,
        const FEEDBACK_BUF: usize,
    >(
        &mut self,
        spec: RawActionClientSpec<'_>,
    ) -> Result<ActionClientRawHandle, NodeError> {
        let RawActionClientSpec {
            node_id,
            action_name,
            type_name,
            type_hash,
            goal_response_callback,
            feedback_callback,
            result_callback,
            context,
        } = spec;

        type Entry<const GB: usize, const RB: usize, const FB: usize> =
            ActionClientRawArenaEntry<GB, RB, FB>;

        let slot = self.next_entry_slot()?;

        let action_info = ActionInfo::new(action_name, type_name, type_hash);
        // Issue 0656 — capture the domain BEFORE the session borrow below, for
        // the same reason `node_name`/`ns` are cloned here: the `&mut session`
        // in the create scope would otherwise conflict with `&self`.
        let domain_id = self.domain_id;
        let (node_name, ns, session_idx) = match node_id {
            Some(id) => {
                let r = self
                    .nodes
                    .get(id.index())
                    .ok_or(NodeError::InvalidSchedContextBinding)?;
                (r.name.clone(), r.namespace.clone(), r.session_idx)
            }
            None => (self.node_name.clone(), self.namespace.clone(), 0u8),
        };

        let (send_goal_client, cancel_goal_client, get_result_client, feedback_sub) = {
            // phase-338 W3 — per-channel types, matching the raw SERVER path.
            // Both raw sides used the bare action type, so nano-ros talked to
            // itself but was invisible to any `rcl_action` peer; fixing only one
            // side would have broken the self-consistent pairs instead.
            let send_goal_type: heapless::String<256> =
                super::action_core::action_channel_type(type_name, "SendGoal");
            let get_result_type: heapless::String<256> =
                super::action_core::action_channel_type(type_name, "GetResult");
            let feedback_type: heapless::String<256> =
                super::action_core::action_channel_type(type_name, "FeedbackMessage");

            let send_goal_keyexpr: heapless::String<256> = action_info.send_goal_key();
            let mut send_goal_info =
                ServiceInfo::new(&send_goal_keyexpr, &send_goal_type, type_hash)
                    .with_namespace(&ns)
                    .with_domain(domain_id);
            if !node_name.is_empty() {
                send_goal_info = send_goal_info.with_node_name(&node_name);
            }

            let cancel_goal_keyexpr: heapless::String<256> = action_info.cancel_goal_key();
            let mut cancel_goal_info = ServiceInfo::new(
                &cancel_goal_keyexpr,
                "action_msgs::srv::dds_::CancelGoal_",
                type_hash,
            )
            .with_namespace(&ns)
            .with_domain(domain_id);
            if !node_name.is_empty() {
                cancel_goal_info = cancel_goal_info.with_node_name(&node_name);
            }

            let get_result_keyexpr: heapless::String<256> = action_info.get_result_key();
            let mut get_result_info =
                ServiceInfo::new(&get_result_keyexpr, &get_result_type, type_hash)
                    .with_namespace(&ns)
                    .with_domain(domain_id);
            if !node_name.is_empty() {
                get_result_info = get_result_info.with_node_name(&node_name);
            }

            let feedback_keyexpr: heapless::String<256> = action_info.feedback_key();
            let mut feedback_topic = TopicInfo::new(&feedback_keyexpr, &feedback_type, type_hash)
                .with_namespace(&ns)
                .with_domain(domain_id);
            if !node_name.is_empty() {
                feedback_topic = feedback_topic.with_node_name(&node_name);
            }

            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            // issue 0870 — these were `map_err(|_| NodeError::ActionCreationFailed)`,
            // discarding a `TransportError` the caller could have used. An action
            // client declares four entities back to back, so "one of them failed"
            // is the least useful thing this seam can say. `NodeError` already
            // carries `Transport(TransportError)` — nothing needed adding, the
            // error simply was not passed on. Swept across all 17 session
            // `create_*` sites here. The two `register_protocol_types` sites keep
            // `ActionCreationFailed`: their `map_err(|()| …)` has no payload, so
            // there the variant IS the whole truth.
            (
                session
                    .create_client(&send_goal_info, QoSProfile::services_default())
                    .map_err(|e| {
                        nros_log::nros_error!(
                            nros_log::get_logger("nros_node"),
                            "action client: send_goal client failed: {:?}",
                            e
                        );
                        NodeError::Transport(e)
                    })?,
                session
                    .create_client(&cancel_goal_info, QoSProfile::services_default())
                    .map_err(|e| {
                        nros_log::nros_error!(
                            nros_log::get_logger("nros_node"),
                            "action client: cancel_goal client failed: {:?}",
                            e
                        );
                        NodeError::Transport(e)
                    })?,
                session
                    .create_client(&get_result_info, QoSProfile::services_default())
                    .map_err(|e| {
                        nros_log::nros_error!(
                            nros_log::get_logger("nros_node"),
                            "action client: get_result client failed: {:?}",
                            e
                        );
                        NodeError::Transport(e)
                    })?,
                session
                    .create_subscription(&feedback_topic, QoSProfile::BEST_EFFORT)
                    .map_err(|e| {
                        nros_log::nros_error!(
                            nros_log::get_logger("nros_node"),
                            "action client: feedback subscription failed: {:?}",
                            e
                        );
                        NodeError::Transport(e)
                    })?,
            )
        };

        let core = ActionClientCore::new(
            send_goal_client,
            cancel_goal_client,
            get_result_client,
            feedback_sub,
        );

        let offset = self.arena_alloc::<Entry<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>>()?;

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut Entry<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>;
            core::ptr::write(
                entry_ptr,
                Entry {
                    core,
                    goal_response_callback,
                    feedback_callback,
                    result_callback,
                    context,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::ActionClient,
            has_data: always_ready,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::Always,
            try_process: action_client_raw_try_process::<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>,
            drop_fn: drop_entry::<Entry<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(spec.action_name));
        self.apply_node_default_sched(slot, node_id, None);

        Ok(ActionClientRawHandle { entry_index: slot })
    }

    /// RFC-0041 / Phase 239.2 — register a **typed callback** action client.
    /// Goal-response / feedback / result are eager-drained at `spin_once` and
    /// dispatched as deserialized `A::Feedback` / `A::Result` to the typed
    /// closures. Returns the scheduling [`HandleId`] and a `*mut` to the arena
    /// entry's core (used to build the typed
    /// [`ActionClientCallback`](super::handles::ActionClientCallback)).
    #[allow(clippy::too_many_arguments, clippy::type_complexity)]
    pub(crate) fn register_action_client_callback<
        A,
        GRespF,
        FbF,
        ResF,
        const GOAL_BUF: usize,
        const RESULT_BUF: usize,
        const FEEDBACK_BUF: usize,
    >(
        &mut self,
        node_id: Option<super::node_record::NodeId>,
        action_name: &str,
        type_name: &str,
        type_hash: &str,
        feedback_depth: u16,
        on_goal_response: GRespF,
        on_feedback: FbF,
        on_result: ResF,
    ) -> Result<
        (
            HandleId,
            *mut ActionClientCore<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>,
        ),
        NodeError,
    >
    where
        A: nros_core::RosAction + 'static,
        GRespF: FnMut(&nros_core::GoalId, bool) + 'static,
        FbF: FnMut(&nros_core::GoalId, &A::Feedback) + 'static,
        ResF: FnMut(&nros_core::GoalId, nros_core::GoalStatus, &A::Result) + 'static,
    {
        type Entry<A, G, Fb, R, const GB: usize, const RB: usize, const FB: usize> =
            ActionClientCallbackEntry<A, G, Fb, R, GB, RB, FB>;

        let slot = self.next_entry_slot()?;
        let action_info = ActionInfo::new(action_name, type_name, type_hash);
        // Phase 244 E3 (RFC-0044) — register the `action_msgs` protocol types
        // (CancelGoal_{Request,Response}, GoalStatusArray) the client's cancel
        // service + status subscription serialize, before creating those
        // entities. Generated `impl RosAction` overrides this (default no-op);
        // replaces the example's hand-rolled `#[cfg(rmw-cyclonedds)]` block.
        A::register_protocol_types().map_err(|()| NodeError::ActionCreationFailed)?;
        // Issue 0656 — capture the domain BEFORE the session borrow below, for
        // the same reason `node_name`/`ns` are cloned here: the `&mut session`
        // in the create scope would otherwise conflict with `&self`.
        let domain_id = self.domain_id;
        let (node_name, ns, session_idx) = match node_id {
            Some(id) => {
                let r = self
                    .nodes
                    .get(id.index())
                    .ok_or(NodeError::InvalidSchedContextBinding)?;
                (r.name.clone(), r.namespace.clone(), r.session_idx)
            }
            None => (self.node_name.clone(), self.namespace.clone(), 0u8),
        };

        let (send_goal_client, cancel_goal_client, get_result_client, feedback_sub) = {
            // phase-338 W3 — per-channel types, matching the raw SERVER path.
            // Both raw sides used the bare action type, so nano-ros talked to
            // itself but was invisible to any `rcl_action` peer; fixing only one
            // side would have broken the self-consistent pairs instead.
            let send_goal_type: heapless::String<256> =
                super::action_core::action_channel_type(type_name, "SendGoal");
            let get_result_type: heapless::String<256> =
                super::action_core::action_channel_type(type_name, "GetResult");
            let feedback_type: heapless::String<256> =
                super::action_core::action_channel_type(type_name, "FeedbackMessage");

            let send_goal_keyexpr: heapless::String<256> = action_info.send_goal_key();
            let mut send_goal_info =
                ServiceInfo::new(&send_goal_keyexpr, &send_goal_type, type_hash)
                    .with_namespace(&ns)
                    .with_domain(domain_id);
            if !node_name.is_empty() {
                send_goal_info = send_goal_info.with_node_name(&node_name);
            }
            let cancel_goal_keyexpr: heapless::String<256> = action_info.cancel_goal_key();
            let mut cancel_goal_info = ServiceInfo::new(
                &cancel_goal_keyexpr,
                "action_msgs::srv::dds_::CancelGoal_",
                type_hash,
            )
            .with_namespace(&ns)
            .with_domain(domain_id);
            if !node_name.is_empty() {
                cancel_goal_info = cancel_goal_info.with_node_name(&node_name);
            }
            let get_result_keyexpr: heapless::String<256> = action_info.get_result_key();
            let mut get_result_info =
                ServiceInfo::new(&get_result_keyexpr, &get_result_type, type_hash)
                    .with_namespace(&ns)
                    .with_domain(domain_id);
            if !node_name.is_empty() {
                get_result_info = get_result_info.with_node_name(&node_name);
            }
            let feedback_keyexpr: heapless::String<256> = action_info.feedback_key();
            let mut feedback_topic = TopicInfo::new(&feedback_keyexpr, &feedback_type, type_hash)
                .with_namespace(&ns)
                .with_domain(domain_id);
            if !node_name.is_empty() {
                feedback_topic = feedback_topic.with_node_name(&node_name);
            }
            let session = self
                .session_at_mut(session_idx)
                .ok_or(NodeError::BackendMismatch)?;
            (
                session
                    .create_client(&send_goal_info, QoSProfile::services_default())
                    .map_err(NodeError::Transport)?,
                session
                    .create_client(&cancel_goal_info, QoSProfile::services_default())
                    .map_err(NodeError::Transport)?,
                session
                    .create_client(&get_result_info, QoSProfile::services_default())
                    .map_err(NodeError::Transport)?,
                session
                    .create_subscription(&feedback_topic, QoSProfile::BEST_EFFORT)
                    .map_err(NodeError::Transport)?,
            )
        };

        let core = ActionClientCore::new(
            send_goal_client,
            cancel_goal_client,
            get_result_client,
            feedback_sub,
        );

        // Phase 239.5 — trailing-allocate the feedback QoS-depth buffer alongside
        // the entry (ring for depth > 1, triple for depth ≤ 1), then drain
        // `core.feedback_subscriber` into it in the dispatcher.
        let (_slot_count, trailing_bytes) =
            buffered_region_size(feedback_depth as u32, FEEDBACK_BUF);
        let (offset, trailing_offset) = self.arena_alloc_with_trailing::<Entry<
            A,
            GRespF,
            FbF,
            ResF,
            GOAL_BUF,
            RESULT_BUF,
            FEEDBACK_BUF,
        >>(trailing_bytes)?;
        let buf_ptr = unsafe { (self.arena.as_mut_ptr() as *mut u8).add(trailing_offset) };
        let feedback_buffer = if feedback_depth <= 1 {
            BufferStrategy::Triple(unsafe { TripleBuffer::init(buf_ptr, FEEDBACK_BUF) })
        } else {
            BufferStrategy::Ring(unsafe {
                SpscRing::init(buf_ptr, FEEDBACK_BUF, feedback_depth as usize)
            })
        };
        let core_ptr = unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset)
                as *mut Entry<A, GRespF, FbF, ResF, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>;
            core::ptr::write(
                entry_ptr,
                ActionClientCallbackEntry {
                    core,
                    feedback_buffer,
                    on_goal_response,
                    on_feedback,
                    on_result,
                    _phantom: core::marker::PhantomData,
                },
            );
            &mut (*entry_ptr).core as *mut ActionClientCore<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>
        };

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::ActionClient,
            has_data: always_ready,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::Always,
            try_process: action_client_callback_try_process::<
                A,
                GRespF,
                FbF,
                ResF,
                GOAL_BUF,
                RESULT_BUF,
                FEEDBACK_BUF,
            >,
            drop_fn: drop_entry::<Entry<A, GRespF, FbF, ResF, GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>>,
        };
        self.emplace_entry(slot, meta, TraceName::Text(action_name));
        self.apply_node_default_sched(slot, node_id, None);
        Ok((HandleId(slot), core_ptr))
    }
}

impl<'s> Executor<'s> {
    /// Register an existing `ActionClientCore` with the executor for async polling.
    ///
    /// Unlike `register_action_client_raw` (which creates new transport handles),
    /// this takes ownership of an existing core. Use this when the core was
    /// already created by the C/C++ action client init.
    pub fn register_action_client_core<
        const GOAL_BUF: usize,
        const RESULT_BUF: usize,
        const FEEDBACK_BUF: usize,
    >(
        &mut self,
        core: ActionClientCore<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>,
        goal_response_callback: Option<RawGoalResponseCallback>,
        feedback_callback: Option<RawFeedbackCallback>,
        result_callback: Option<RawResultCallback>,
        context: *mut core::ffi::c_void,
    ) -> Result<ActionClientRawHandle, NodeError> {
        type Entry<const GB: usize, const RB: usize, const FB: usize> =
            ActionClientRawArenaEntry<GB, RB, FB>;

        let slot = self.next_entry_slot()?;
        let offset = self.arena_alloc::<Entry<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>>()?;

        unsafe {
            let arena_ptr = self.arena.as_mut_ptr() as *mut u8;
            let entry_ptr = arena_ptr.add(offset) as *mut Entry<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>;
            core::ptr::write(
                entry_ptr,
                Entry {
                    core,
                    goal_response_callback,
                    feedback_callback,
                    result_callback,
                    context,
                },
            );
        }

        let meta = CallbackMeta {
            offset,
            kind: EntryKind::ActionClient,
            has_data: always_ready,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::Always,
            try_process: action_client_raw_try_process::<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>,
            drop_fn: drop_entry::<Entry<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>>,
        };
        self.emplace_entry(slot, meta, TraceName::Slot("action_client", slot));

        Ok(ActionClientRawHandle { entry_index: slot })
    }
}

/// Handle returned by [`Executor::register_action_client_raw()`].
///
/// Provides methods to send goals, request results, and cancel goals
/// via the executor's non-blocking path.
pub struct ActionClientRawHandle {
    entry_index: usize,
}

impl ActionClientRawHandle {
    /// Get the entry index for this action client.
    pub fn entry_index(&self) -> usize {
        self.entry_index
    }
}

// ============================================================================
// phase-417 W4.b — the three terminal verbs
// ============================================================================

// Gated exactly like `executor::tests`: MockSession is the `ConcreteSession`
// only when no real backend is linked, and `from_session` needs `alloc`.
#[cfg(all(test, feature = "alloc", not(feature = "rmw-cffi")))]
mod terminal_verb_tests {
    use super::*;
    use core::sync::atomic::{AtomicI8, Ordering};
    use nros_core::{CdrReader, CdrWriter, DeserError, Deserialize, SerError, Serialize};
    use nros_rmw::TransportError;

    use crate::mock::MockSession;

    /// The status the last `complete_goal_fn` call was handed. `-1` is "no
    /// call yet", which is what makes "the verb never reached `complete_goal`"
    /// distinguishable from "it reached it with the wrong status".
    static LAST_STATUS: AtomicI8 = AtomicI8::new(-1);

    #[derive(Default)]
    struct Unit;

    impl Serialize for Unit {
        fn serialize(&self, _w: &mut CdrWriter) -> Result<(), SerError> {
            Ok(())
        }
    }
    impl Deserialize for Unit {
        fn deserialize(_r: &mut CdrReader) -> Result<Self, DeserError> {
            Ok(Self)
        }
    }
    impl nros_core::RosMessage for Unit {
        const TYPE_NAME: &'static str = "test/action/Verbs_Unit";
        const TYPE_HASH: &'static str = "test_hash";
    }

    struct VerbAction;

    impl RosAction for VerbAction {
        type Goal = Unit;
        type Result = Unit;
        type Feedback = Unit;
        type SendGoalRequest = Unit;
        type SendGoalResponse = Unit;
        type GetResultRequest = Unit;
        type GetResultResponse = Unit;
        type FeedbackMessage = Unit;
        const ACTION_NAME: &'static str = "test/action/dds_/Verbs_";
        const ACTION_HASH: &'static str = "test_hash";
    }

    unsafe fn record_status(
        _data: *mut u8,
        _goal_id: &nros_core::GoalId,
        status: nros_core::GoalStatus,
        _result: Unit,
    ) -> Result<(), NodeError> {
        LAST_STATUS.store(status as i8, Ordering::SeqCst);
        Ok(())
    }

    unsafe fn unused_feedback(
        _data: *mut u8,
        _goal_id: &nros_core::GoalId,
        _fb: &Unit,
    ) -> Result<(), NodeError> {
        unreachable!("the terminal-verb test never publishes feedback")
    }

    unsafe fn unused_set_status(
        _data: *mut u8,
        _goal_id: &nros_core::GoalId,
        _status: nros_core::GoalStatus,
    ) {
        unreachable!("the terminal-verb test never sets a non-terminal status")
    }

    unsafe fn unused_count(_data: *const u8) -> usize {
        unreachable!("the terminal-verb test never counts active goals")
    }

    unsafe fn unused_for_each(_data: *const u8, _f: &mut dyn FnMut(&ActiveGoal<VerbAction>)) {
        unreachable!("the terminal-verb test never iterates active goals")
    }

    unsafe fn never_processes(
        _data: *mut u8,
        _delta_us: u64,
        _slot: u8,
    ) -> Result<bool, TransportError> {
        Ok(false)
    }

    unsafe fn drops_nothing(_data: *mut u8) {}

    fn handle() -> ActionServerHandle<VerbAction> {
        ActionServerHandle {
            entry_index: 0,
            publish_feedback_fn: unused_feedback,
            complete_goal_fn: record_status,
            set_goal_status_fn: unused_set_status,
            active_goal_count_fn: unused_count,
            for_each_active_goal_fn: unused_for_each,
            _phantom: PhantomData,
        }
    }

    fn meta() -> CallbackMeta {
        CallbackMeta {
            offset: 0,
            kind: EntryKind::ActionServer,
            try_process: never_processes,
            has_data: always_ready,
            pre_sample: no_pre_sample,
            invocation: InvocationMode::OnNewData,
            drop_fn: drops_nothing,
        }
    }

    /// Each verb must reach `complete_goal` with the SPEC state it is named
    /// for, and `cancel` must forward to `canceled` rather than be a second
    /// code path (RFC-0019: the wrapper spelling may double, the behaviour may
    /// not).
    ///
    /// The observation point is the handle's own `complete_goal_fn`, so a verb
    /// that quietly did nothing fails on `-1` and a verb wired to the wrong
    /// status fails on the discriminant — the two ways a forwarder goes wrong.
    #[test]
    fn terminal_verbs_carry_their_spec_state_and_cancel_forwards_to_canceled() {
        let mut executor = Executor::from_session(MockSession::new());
        executor.entries[0] = Some(meta());

        let h = handle();
        let goal = nros_core::GoalId::zero();

        LAST_STATUS.store(-1, Ordering::SeqCst);
        h.succeed(&mut executor, &goal, Unit).unwrap();
        assert_eq!(
            LAST_STATUS.load(Ordering::SeqCst),
            nros_core::GoalStatus::Succeeded as i8,
            "succeed() must terminate the goal as SUCCEEDED"
        );

        LAST_STATUS.store(-1, Ordering::SeqCst);
        h.abort(&mut executor, &goal, Unit).unwrap();
        assert_eq!(
            LAST_STATUS.load(Ordering::SeqCst),
            nros_core::GoalStatus::Aborted as i8,
            "abort() must terminate the goal as ABORTED"
        );

        // The renamed verb. `canceled` is the spelling C
        // (`nros_action_canceled`), C++ (`ActionServer<A>::canceled`) and
        // rclcpp_action (`ServerGoalHandle::canceled`) all use.
        LAST_STATUS.store(-1, Ordering::SeqCst);
        h.canceled(&mut executor, &goal, Unit).unwrap();
        assert_eq!(
            LAST_STATUS.load(Ordering::SeqCst),
            nros_core::GoalStatus::Canceled as i8,
            "canceled() must terminate the goal as CANCELED"
        );

        // The deprecated alias still COMPILES (this call is the proof) and
        // lands in the same place. `-D warnings` implies `-D deprecated` in
        // this workspace, so the allow is what keeps the alias testable at all.
        LAST_STATUS.store(-1, Ordering::SeqCst);
        #[allow(deprecated)]
        h.cancel(&mut executor, &goal, Unit).unwrap();
        assert_eq!(
            LAST_STATUS.load(Ordering::SeqCst),
            nros_core::GoalStatus::Canceled as i8,
            "the deprecated `cancel` must forward to `canceled`, not be a second path"
        );

        // The entry was hand-built, not registered; hand it back before Drop
        // walks the arena.
        executor.entries[0] = None;
    }
}
