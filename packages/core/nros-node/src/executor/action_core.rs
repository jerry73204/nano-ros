//! Type-agnostic action protocol core types.
//!
//! [`ActionServerCore`] and [`ActionClientCore`] handle the raw-bytes
//! action protocol (GoalId framing, status publishing, result slab)
//! without requiring `RosAction` type parameters. The typed
//! [`ActionServer`](super::handles::ActionServer) and
//! [`ActionClient`](super::handles::ActionClient) wrap these cores
//! and add serialization/deserialization at the boundary.

use nros_core::{CdrReader, CdrWriter, GoalId, GoalInfo, GoalStatus, GoalStatusStamped, Serialize};
use nros_rmw::{ClientTrait, Publisher, ServiceTrait, Subscription, TransportError};

use super::types::NodeError;
use crate::session;

/// Scratch buffer for a decoded CancelGoal request. Cancel payloads are
/// tiny (a `GoalId` + a `builtin_interfaces/Time` stamp), so a fixed
/// 256-byte buffer covers them without a const-generic parameter like the
/// goal/result/feedback slabs.
pub(crate) const CANCEL_BUF: usize = 256;

/// DDS type name of an action's `send_goal` / `get_result` service.
///
/// ROS 2 names the action's two services by their per-channel service types —
/// `<Action>_SendGoal` / `<Action>_GetResult` — **not** the bare action type
/// (`<Action>`). A real `rcl_action` peer matches on those, so advertising the
/// bare action type leaves our send_goal/get_result services undiscovered and
/// every goal times out.
///
/// The codegen emits each channel's *request* envelope type name in DDS form,
/// e.g. `example_interfaces::action::dds_::Fibonacci_SendGoal_Request_`. The
/// service layer (`xrce_dds_request_type` / the Zenoh shim) extends a base type
/// ending in `_` with `Request_` / `Response_`, so the base we must pass is the
/// request type name with its trailing `Request_` stripped:
/// `…Fibonacci_SendGoal_Request_` → `…Fibonacci_SendGoal_`. Falls back to the
/// bare action type if the request name has an unexpected shape.
pub(crate) fn action_service_base_type<'a>(
    request_type_name: &'a str,
    fallback_action_type: &'a str,
) -> &'a str {
    request_type_name
        .strip_suffix("Request_")
        .unwrap_or(fallback_action_type)
}

/// The per-channel DDS type name for a RAW-registered action server.
///
/// The typed path derives these from `A::SendGoalRequest::TYPE_NAME` via
/// [`action_service_base_type`]. The raw path (`register_action_server_raw*`)
/// has only the BARE action type, so it must construct them — and until
/// phase-338 W3 it did not, advertising `…Fibonacci_` on send_goal / get_result
/// / feedback where ROS 2 expects `…Fibonacci_SendGoal_`,
/// `…Fibonacci_GetResult_` and `…Fibonacci_FeedbackMessage_`. The type name is
/// baked into the keyexpr, so a client's query never matched the server's
/// queryable and every goal timed out with `Transport(Timeout)` — exactly the
/// failure [`action_service_base_type`]'s own doc warns about, on the other
/// registration path.
///
/// `action_type` is DDS-form and ends in `_` (`…::dds_::Fibonacci_`); the
/// result replaces that suffix with `_<Channel>_`. Returns the bare type
/// unchanged if it has an unexpected shape, matching the typed path's
/// fallback.
pub fn action_channel_type<const N: usize>(
    action_type: &str,
    channel: &str,
) -> heapless::String<N> {
    let mut out: heapless::String<N> = heapless::String::new();
    let base = action_type.strip_suffix('_').unwrap_or(action_type);
    if out.push_str(base).is_err()
        || out.push('_').is_err()
        || out.push_str(channel).is_err()
        || out.push('_').is_err()
    {
        let mut fallback: heapless::String<N> = heapless::String::new();
        let _ = fallback.push_str(action_type);
        return fallback;
    }
    out
}

/// Scratch buffer for serializing a `GoalStatusArray` before publishing it
/// on the status topic. 512 bytes holds the CDR header plus a status entry
/// (`GoalInfo` + status enum) for every concurrently-tracked goal.
const STATUS_ARRAY_BUF: usize = 512;

// ============================================================================
// Supporting types
// ============================================================================

/// Goal tracked by the core — only GoalId + status, no typed data.
#[derive(Clone, Copy)]
pub struct RawActiveGoal {
    /// Goal ID.
    pub goal_id: GoalId,
    /// Current status.
    pub status: GoalStatus,
}

/// A `get_result` request held until its goal terminates (Phase 237).
///
/// `sequence_number` is the service-backend reply-correlation token; the backend
/// must be able to `send_response(sequence_number, …)` after the handler returned
/// (Cyclone native; XRCE/Zenoh via the Phase 237 seq-keyed reply tables).
#[derive(Clone, Copy)]
pub struct PendingGetResult {
    /// Goal whose terminal result the requester is waiting for.
    pub goal_id: GoalId,
    /// Backend reply-correlation token for the deferred `send_response`.
    pub sequence_number: i64,
}

/// Completed goal result metadata — indexes into the result slab.
///
/// Entries are kept in **completion order**, which is also **increasing
/// `offset` order**: results are appended at `result_slab_used` and the slab is
/// compacted in place whenever an entry is reclaimed. Every reclamation path
/// preserves that ordering (`heapless::Vec::remove` / `retain`, never
/// `swap_remove`) because [`ActionServerCore::compact_result_slab`] moves
/// survivors *down* and would clobber a not-yet-moved entry otherwise.
#[derive(Clone, Copy)]
pub struct CompletedResultEntry {
    /// Unique identifier for the completed goal.
    pub goal_id: GoalId,
    /// Terminal status of the goal.
    pub status: GoalStatus,
    /// Byte offset into the result slab.
    pub offset: usize,
    /// Length of the serialised result in bytes.
    pub len: usize,
    /// `true` once a `get_result` reply carrying this result has been sent —
    /// either the deferred flush in
    /// [`ActionServerCore::complete_goal_raw`] or the immediate reply in
    /// [`ActionServerCore::try_handle_get_result_raw`].
    ///
    /// Issue 0796: this is the reclamation priority. A delivered result has
    /// served its purpose (rcl would let its `result_timeout` retire it), so it
    /// is evicted before any result nobody has fetched yet.
    pub delivered: bool,
}

/// Phase 122.3.c.6.d — information about a peeked cancel-goal
/// request. Returned by
/// [`ActionServerCore::try_recv_cancel_request`].
pub struct PendingCancelRequest {
    /// The goal_id named in the cancel request.
    pub goal_id: GoalId,
    /// Service sequence number — pass back to
    /// [`ActionServerCore::send_cancel_reply`].
    pub sequence_number: i64,
    /// Snapshot of the goal's current status at peek time
    /// (`GoalStatus::Unknown` if no matching active goal).
    pub current_status: GoalStatus,
}

/// Information about a received goal request.
pub struct RawGoalRequest {
    /// The parsed goal ID.
    pub goal_id: GoalId,
    /// Sequence number for the service reply.
    pub sequence_number: i64,
    /// Offset into the goal buffer where the CDR payload begins.
    /// Backends may prepend a sequence-number header (DDS) or place
    /// the payload at offset 0 (zenoh).
    pub data_offset: usize,
    /// Length of valid CDR data starting at `data_offset`.
    pub data_len: usize,
}

// ============================================================================
// GoalId CDR helpers
// ============================================================================

/// Read a GoalId from a CDR reader as a fixed `uint8[16]` array.
///
/// ROS 2 actions carry the goal id as `unique_identifier_msgs/UUID`, whose
/// single field is a **fixed-size** `uint8[16]` array — CDR fixed arrays have
/// **no** length prefix. We must read exactly 16 bytes with no leading count,
/// matching `unique_identifier_msgs::msg::UUID::deserialize`. (The pre-233.6
/// framing wrote a `u32(16)` sequence prefix, which self-matched nano-ros peers
/// but added 4 bytes a real `rcl_action` peer rejects.)
fn read_goal_id(reader: &mut CdrReader<'_>) -> Result<GoalId, NodeError> {
    let mut goal_id = GoalId::default();
    for byte in &mut goal_id.uuid {
        *byte = reader
            .read_u8()
            .map_err(|_| NodeError::Transport(TransportError::DeserializationError))?;
    }
    Ok(goal_id)
}

/// Write a GoalId into a CDR writer as a fixed `uint8[16]` array (no length
/// prefix) — see [`read_goal_id`] for why the prefix must be absent.
fn write_goal_id(writer: &mut CdrWriter<'_>, goal_id: &GoalId) -> Result<(), NodeError> {
    for b in &goal_id.uuid {
        writer.write_u8(*b).map_err(|_| NodeError::Serialization)?;
    }
    Ok(())
}

// ============================================================================
// ActionServerCore
// ============================================================================

/// Type-agnostic action server core handling the raw-bytes protocol.
///
/// Manages active goal tracking (GoalId + status), completed result storage
/// in a fixed-size slab, and all CDR framing for the action protocol.
///
/// The typed [`ActionServer`](super::handles::ActionServer) wraps this
/// and adds `A::Goal` / `A::Feedback` / `A::Result` (de)serialization.
pub struct ActionServerCore<
    const GOAL_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const RESULT_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const FEEDBACK_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const MAX_GOALS: usize = 4,
> {
    pub(crate) send_goal_server: session::RmwServiceServer,
    pub(crate) cancel_goal_server: session::RmwServiceServer,
    pub(crate) get_result_server: session::RmwServiceServer,
    pub(crate) feedback_publisher: session::RmwPublisher,
    pub(crate) status_publisher: session::RmwPublisher,
    pub(crate) active_goals: heapless::Vec<RawActiveGoal, MAX_GOALS>,
    pub(crate) completed_results: heapless::Vec<CompletedResultEntry, MAX_GOALS>,
    /// `get_result` requests that arrived while their goal was still active
    /// (Phase 237). `rclcpp_action` sends `get_result` immediately after
    /// acceptance and expects the reply only once the goal terminates, so we
    /// hold the request's correlation token (`sequence_number`) here and flush
    /// it in [`Self::complete_goal_raw`]. Deferral relies on the service
    /// backend honoring `send_response(seq)` after the handler returns — the
    /// seq-keyed reply contract (Cyclone native; XRCE/Zenoh per Phase 237).
    pub(crate) pending_get_results: heapless::Vec<PendingGetResult, MAX_GOALS>,
    /// Slab storage for completed result CDR bytes.
    ///
    /// A bump region whose survivors are compacted on reclamation — see
    /// [`CompletedResultEntry`] and
    /// [`reserve_result_space`](Self::reserve_result_space).
    pub(crate) result_slab: [u8; RESULT_BUF],
    /// Bytes of [`result_slab`](Self::result_slab) currently held by the
    /// entries in `completed_results`. Compaction keeps the live bytes in
    /// `[0, result_slab_used)` with no holes, so this is exactly the sum of the
    /// entries' `len`.
    pub(crate) result_slab_used: usize,
    pub(crate) goal_buffer: [u8; GOAL_BUF],
    pub(crate) feedback_buffer: [u8; FEEDBACK_BUF],
    pub(crate) cancel_buffer: [u8; CANCEL_BUF],
}

impl<
    const GOAL_BUF: usize,
    const RESULT_BUF: usize,
    const FEEDBACK_BUF: usize,
    const MAX_GOALS: usize,
> ActionServerCore<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF, MAX_GOALS>
{
    /// Phase 122.3.c.6.b — construct an `ActionServerCore` from the
    /// 5 already-built transport channels. Caller (typically the C
    /// API's `nros_action_server_init_polling`) owns wiring the
    /// channels via the session's `create_*` methods.
    pub fn from_channels(
        send_goal_server: session::RmwServiceServer,
        cancel_goal_server: session::RmwServiceServer,
        get_result_server: session::RmwServiceServer,
        feedback_publisher: session::RmwPublisher,
        status_publisher: session::RmwPublisher,
    ) -> Self {
        Self {
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
            cancel_buffer: [0u8; CANCEL_BUF],
        }
    }

    /// Try to receive a goal request from the send_goal service.
    ///
    /// Returns the parsed GoalId, sequence number, and data length.
    /// The full CDR data (including GoalId) remains in `goal_buffer`.
    pub fn try_recv_goal_request(&mut self) -> Result<Option<RawGoalRequest>, NodeError> {
        // Capture buf base ptr before borrowing through `take_request`
        // so we can recover the data offset after the borrow ends.
        // DDS-style backends place a sequence-number prefix before the
        // CDR payload; reading the buffer from offset 0 unconditionally
        // would feed the prefix bytes to the deserializer.
        let buf_start = self.goal_buffer.as_ptr() as usize;
        // Phase 120: NoData (no pending request) is the steady-state
        // expected condition — collapse it to `Ok(None)` instead of
        // surfacing as ServiceRequestFailed. Any other transport
        // error remains ServiceRequestFailed.
        let request = match self.send_goal_server.take_request(&mut self.goal_buffer) {
            Ok(opt) => opt,
            Err(TransportError::NoData) => return Ok(None),
            Err(_) => return Err(NodeError::Transport(TransportError::ServiceRequestFailed)),
        };

        let request = match request {
            Some(r) => r,
            None => return Ok(None),
        };

        let data_offset = (request.data.as_ptr() as usize).saturating_sub(buf_start);
        let data_len = request.data.len();
        let sequence_number = request.sequence_number;
        #[allow(clippy::drop_non_drop)]
        drop(request);

        let mut reader =
            CdrReader::new_with_header(&self.goal_buffer[data_offset..data_offset + data_len])
                .map_err(|_| NodeError::Transport(TransportError::DeserializationError))?;

        let goal_id = read_goal_id(&mut reader)?;

        Ok(Some(RawGoalRequest {
            goal_id,
            sequence_number,
            data_offset,
            data_len,
        }))
    }

    /// Get a reference to the goal buffer (valid after `try_recv_goal_request`).
    pub fn goal_buffer(&self) -> &[u8] {
        &self.goal_buffer
    }

    /// Accept a goal: records it, sends the acceptance reply, publishes status.
    ///
    /// # Ordering
    ///
    /// The goal is recorded in `active_goals` **before** the reply goes out,
    /// and the recording is rolled back if the reply fails. Issue 0322: this
    /// used to reply first and then `let _ = self.active_goals.push(...)`, so
    /// once `MAX_GOALS` (default 4) were active, a 5th `send_goal` was
    /// answered `accepted=true` and then dropped — no execution, no feedback,
    /// no result, no terminal status. An rclcpp/rclpy client that saw
    /// `accepted=true` waited on its result future forever.
    ///
    /// A full table is now answered truthfully with `accepted=false` via
    /// [`Self::reject_goal`], which is a contract the client already handles.
    pub fn accept_goal(&mut self, goal_id: GoalId, seq: i64) -> Result<(), NodeError> {
        // Serialize first: a serialization failure here must leave no trace,
        // and nothing below depends on the table.
        let mut writer =
            crate::tx_writer(&mut self.cancel_buffer).map_err(|_| NodeError::BufferTooSmall)?;
        writer.write_u8(1).map_err(|_| NodeError::Serialization)?;
        writer.write_i32(0).map_err(|_| NodeError::Serialization)?;
        writer.write_u32(0).map_err(|_| NodeError::Serialization)?;
        let reply_len = writer.position();

        // Capacity is decided BEFORE anything reaches the wire, so a full
        // table becomes an honest rejection rather than a lie.
        if self
            .active_goals
            .push(RawActiveGoal {
                goal_id,
                status: GoalStatus::Accepted,
            })
            .is_err()
        {
            return self.reject_goal(seq);
        }

        if self
            .send_goal_server
            .send_response(seq, &self.cancel_buffer[..reply_len])
            .is_err()
        {
            // The client never learned it was accepted, so un-record it —
            // otherwise the slot leaks and lowers the effective capacity for
            // every later goal. `pop` removes the entry pushed just above:
            // `&mut self` means nothing else can have touched the table.
            self.active_goals.pop();
            return Err(NodeError::ServiceReplyFailed);
        }

        // Past this point the acceptance is on the wire and irreversible.
        //
        // The status-array publish is therefore NOT propagated: both C and C++
        // callers collapse `Err` to a generic error code
        // (`nros-c/src/action/server.rs`, `nros-cpp/src/action.rs`), so
        // returning one here would report "accept failed" for a goal that IS
        // accepted and running — inviting the caller to reject or retry it.
        // The client already holds `accepted=true` and will still receive the
        // result; a missed status sample is degraded, not broken. Issue 0322
        // proposed propagating this; see that issue for why it is deliberately
        // not done.
        let _status = self.publish_status_array();
        Ok(())
    }

    /// Reject a goal: sends the rejection reply.
    pub fn reject_goal(&mut self, seq: i64) -> Result<(), NodeError> {
        // Serialize response: accepted=false + stamp
        let mut writer =
            crate::tx_writer(&mut self.cancel_buffer).map_err(|_| NodeError::BufferTooSmall)?;
        writer.write_u8(0).map_err(|_| NodeError::Serialization)?;
        writer.write_i32(0).map_err(|_| NodeError::Serialization)?;
        writer.write_u32(0).map_err(|_| NodeError::Serialization)?;
        let reply_len = writer.position();

        self.send_goal_server
            .send_response(seq, &self.cancel_buffer[..reply_len])
            .map_err(|_| NodeError::ServiceReplyFailed)
    }

    /// Publish feedback with raw CDR bytes.
    ///
    /// Writes GoalId framing + raw feedback bytes into the feedback buffer
    /// and publishes.
    pub fn publish_feedback_raw(
        &mut self,
        goal_id: &GoalId,
        feedback_cdr: &[u8],
    ) -> Result<(), NodeError> {
        // GoalId framing (4 + 16 = 20 bytes) + feedback_cdr must fit in FEEDBACK_BUF
        let needed = 4 + 20 + feedback_cdr.len(); // CDR header + GoalId + feedback
        if needed > FEEDBACK_BUF {
            return Err(NodeError::BufferTooSmall);
        }

        let mut writer =
            crate::tx_writer(&mut self.feedback_buffer).map_err(|_| NodeError::BufferTooSmall)?;

        write_goal_id(&mut writer, goal_id)?;

        // Copy raw feedback bytes directly after GoalId
        let pos = writer.position();
        if pos + feedback_cdr.len() > FEEDBACK_BUF {
            return Err(NodeError::BufferTooSmall);
        }
        self.feedback_buffer[pos..pos + feedback_cdr.len()].copy_from_slice(feedback_cdr);
        let len = pos + feedback_cdr.len();

        self.feedback_publisher
            .publish_raw(&self.feedback_buffer[..len])
            .map_err(|_| NodeError::Transport(TransportError::PublishFailed))
    }

    /// Set a goal's status and publish the updated GoalStatusArray.
    pub fn set_goal_status(&mut self, goal_id: &GoalId, status: GoalStatus) {
        for goal in &mut self.active_goals {
            if goal.goal_id.uuid == goal_id.uuid {
                goal.status = status;
                break;
            }
        }
        let _ = self.publish_status_array();
    }

    /// Compact the result slab: walk the retained entries in offset order,
    /// move each survivor down to the first free byte, and rewrite its offset.
    ///
    /// Issue 0796. `completed_results` is ordered by `offset` (see
    /// [`CompletedResultEntry`]), so `write <= entry.offset` at every step and
    /// the `copy_within` can never clobber an entry that has not moved yet.
    /// Bounded work: at most `MAX_GOALS` (default 4) moves totalling at most
    /// `RESULT_BUF` bytes, and only on a reclamation.
    fn compact_result_slab(&mut self) {
        let mut write = 0usize;
        for entry in self.completed_results.iter_mut() {
            debug_assert!(
                write <= entry.offset,
                "completed_results must stay in increasing-offset order"
            );
            if entry.offset != write {
                self.result_slab
                    .copy_within(entry.offset..entry.offset + entry.len, write);
                entry.offset = write;
            }
            write += entry.len;
        }
        self.result_slab_used = write;
    }

    /// Drop the retained result for `goal_id`, if any, and compact.
    /// Returns `true` when an entry was dropped.
    fn drop_completed_result(&mut self, goal_id: &GoalId) -> bool {
        match self
            .completed_results
            .iter()
            .position(|e| e.goal_id.uuid == goal_id.uuid)
        {
            Some(idx) => {
                self.completed_results.remove(idx);
                self.compact_result_slab();
                true
            }
            None => false,
        }
    }

    /// Reclaim exactly one completed result. Returns `false` when there is
    /// nothing left to reclaim.
    ///
    /// **Eviction policy** (issue 0796). rcl keeps a terminated goal's result
    /// until it has been collected *and* a per-goal `result_timeout` elapses,
    /// then `rcl_action_expire_goals()` reclaims it. We have no clock in the
    /// core — every RTOS port spells one differently and the core is `no_std`
    /// with no time source threaded through it — so reclamation is **on demand
    /// and priority-ordered** instead of timed:
    ///
    /// 1. the oldest **delivered** result (a client already has these bytes;
    ///    this is the case rcl's timeout is really for), else
    /// 2. the oldest result overall.
    ///
    /// Rule 1 means a fetched result never pins storage. Rule 2 means a client
    /// that asks for nothing cannot wedge the server: its stale result is
    /// displaced by newer ones rather than blocking every later goal. A goal
    /// evicted under rule 2 whose client asks *later* is answered
    /// `GoalStatus::Unknown` with the default result by
    /// [`try_handle_get_result_raw`](Self::try_handle_get_result_raw) — degraded,
    /// but an answer, where the pre-0796 code left the requester hanging
    /// forever.
    fn evict_one_completed_result(&mut self) -> bool {
        if self.completed_results.is_empty() {
            return false;
        }
        let idx = self
            .completed_results
            .iter()
            .position(|e| e.delivered)
            .unwrap_or(0);
        self.completed_results.remove(idx);
        self.compact_result_slab();
        true
    }

    /// Make room for one more entry of `needed` bytes, reclaiming completed
    /// results as required.
    ///
    /// `Err(NodeError::BufferTooSmall)` means the result can never be retained
    /// because it exceeds `RESULT_BUF` outright — an empty slab would not hold
    /// it either, so the caller must raise the action server's `RESULT_BUF`.
    /// That is the ONLY failure: a full slab or a full entry table is
    /// reclaimed, not refused.
    fn reserve_result_space(&mut self, needed: usize) -> Result<(), NodeError> {
        if needed > RESULT_BUF {
            return Err(NodeError::BufferTooSmall);
        }
        loop {
            if self.completed_results.len() < MAX_GOALS
                && RESULT_BUF - self.result_slab_used >= needed
            {
                return Ok(());
            }
            if !self.evict_one_completed_result() {
                // Unreachable while `needed <= RESULT_BUF` and `MAX_GOALS >= 1`,
                // but the loop must not spin on a degenerate instantiation.
                return Err(NodeError::BufferTooSmall);
            }
        }
    }

    /// Mark `goal_id`'s retained result as fetched, making it the first
    /// candidate for reclamation.
    fn mark_result_delivered(&mut self, goal_id: &GoalId) {
        for entry in self.completed_results.iter_mut() {
            if entry.goal_id.uuid == goal_id.uuid {
                entry.delivered = true;
                break;
            }
        }
    }

    /// Reclaim every completed result whose `get_result` reply has already been
    /// sent — nano-ros's analogue of `rcl_action_expire_goals()`. Returns the
    /// number of entries reclaimed.
    ///
    /// Calling this is **optional**: [`complete_goal_raw`](Self::complete_goal_raw)
    /// reclaims on demand, so a server that never calls it still runs forever.
    /// It exists for a server that would rather return the memory eagerly (e.g.
    /// before a long idle period) than at the next completion.
    pub fn expire_completed_results(&mut self) -> usize {
        let before = self.completed_results.len();
        self.completed_results.retain(|e| !e.delivered);
        let removed = before - self.completed_results.len();
        if removed > 0 {
            self.compact_result_slab();
        }
        removed
    }

    /// Number of completed results currently retained.
    pub fn completed_result_count(&self) -> usize {
        self.completed_results.len()
    }

    /// Bytes of the result slab currently held by retained results.
    pub fn result_slab_used(&self) -> usize {
        self.result_slab_used
    }

    /// `true` while `goal_id`'s completed result is still retained (i.e. a
    /// `get_result` for it would be answered from the slab rather than as
    /// `Unknown`).
    pub fn has_completed_result(&self, goal_id: &GoalId) -> bool {
        self.completed_results
            .iter()
            .any(|e| e.goal_id.uuid == goal_id.uuid)
    }

    /// Complete a goal: remove from active, store raw result CDR in slab,
    /// publish status.
    ///
    /// # Errors
    ///
    /// `NodeError::BufferTooSmall` when `result_cdr` is larger than
    /// `RESULT_BUF` and therefore cannot be retained for a later `get_result`.
    /// Any client already waiting on `~/_action/get_result` is still answered
    /// (straight from `result_cdr`), and the terminal status is still
    /// published — but a *later* `get_result` for this goal gets
    /// `GoalStatus::Unknown`. Raise the server's `RESULT_BUF`.
    ///
    /// Issue 0796: this returned `()`, so the pre-fix slab exhaustion — the
    /// server silently dropping every result once the bump allocator hit
    /// `RESULT_BUF`, and silently stranding every waiting requester with it —
    /// was invisible to the caller. A full slab is no longer a failure at all
    /// (it is reclaimed), and the one remaining failure is reported.
    pub fn complete_goal_raw(
        &mut self,
        goal_id: &GoalId,
        status: GoalStatus,
        result_cdr: &[u8],
    ) -> Result<(), NodeError> {
        // Remove from active goals
        if let Some(pos) = self
            .active_goals
            .iter()
            .position(|g| g.goal_id.uuid == goal_id.uuid)
        {
            self.active_goals.swap_remove(pos);
        }

        // Re-completing a goal replaces its retained result rather than
        // stacking a second copy of it in the slab.
        self.drop_completed_result(goal_id);

        // Store result CDR in the slab, reclaiming older results if needed.
        let stored = match self.reserve_result_space(result_cdr.len()) {
            Ok(()) => {
                let offset = self.result_slab_used;
                let end = offset + result_cdr.len();
                self.result_slab[offset..end].copy_from_slice(result_cdr);
                self.result_slab_used = end;
                // `reserve_result_space` guaranteed a free entry slot.
                let pushed = self
                    .completed_results
                    .push(CompletedResultEntry {
                        goal_id: *goal_id,
                        status,
                        offset,
                        len: result_cdr.len(),
                        delivered: false,
                    })
                    .is_ok();
                debug_assert!(pushed, "reserve_result_space must leave an entry slot");
                Some((offset, result_cdr.len()))
            }
            Err(_) => None,
        };

        // Phase 237 — flush any get_result requests that arrived while this goal
        // was still active. Reply to each held requester via its retained
        // `sequence_number`.
        //
        // Issue 0796: this used to be skipped entirely when the result could not
        // be stored, so an oversized (or, pre-fix, merely unlucky) result left
        // an `rclcpp_action` client waiting on its result future forever. The
        // waiter is now answered from `result_cdr` directly when the slab could
        // not take it — the bytes are right here; only the *retention* failed.
        let mut delivered_any = false;
        let mut i = 0;
        while i < self.pending_get_results.len() {
            if self.pending_get_results[i].goal_id.uuid == goal_id.uuid {
                let seq = self.pending_get_results[i].sequence_number;
                // swap_remove moves the last entry into slot `i`; re-check `i`.
                let _ = self.pending_get_results.swap_remove(i);
                let sent = match stored {
                    Some((offset, len)) => {
                        self.reply_get_result_from_slab(seq, status, offset, len)
                    }
                    None => self.reply_get_result_bytes(seq, status, result_cdr),
                };
                delivered_any |= sent.is_ok();
            } else {
                i += 1;
            }
        }
        if delivered_any {
            self.mark_result_delivered(goal_id);
        }

        let _ = self.publish_status_array();

        if stored.is_some() {
            Ok(())
        } else {
            Err(NodeError::BufferTooSmall)
        }
    }

    /// Phase 122.3.c.6.e — register a `Waker` that fires when a new
    /// send_goal request arrives. Event-driven action servers
    /// register here in place of polling `try_recv_goal_request` on
    /// a timer.
    pub fn register_goal_waker(&self, waker: &core::task::Waker) {
        use nros_rmw::ServiceTrait;
        self.send_goal_server.register_waker(waker);
    }

    /// Phase 122.3.c.6.e — register a `Waker` that fires when a
    /// cancel-goal request arrives.
    pub fn register_cancel_waker(&self, waker: &core::task::Waker) {
        use nros_rmw::ServiceTrait;
        self.cancel_goal_server.register_waker(waker);
    }

    /// Phase 122.3.c.6.e — register a `Waker` that fires when a
    /// get_result query arrives.
    pub fn register_get_result_waker(&self, waker: &core::task::Waker) {
        use nros_rmw::ServiceTrait;
        self.get_result_server.register_waker(waker);
    }

    /// Phase 122.3.c.6.d — peek a pending cancel-goal request without
    /// generating a reply. Returns the goal_id named in the request,
    /// the matching service sequence number (use it with
    /// [`send_cancel_reply`](Self::send_cancel_reply)), and the
    /// goal's current status (`GoalStatus::Unknown` if no such
    /// active goal). Returns `Ok(None)` when no cancel request is
    /// pending.
    ///
    /// Used by L1 polling-mode action servers (nros-c / nros-cpp C
    /// FFI) that want to drive cancel-decision policy without
    /// passing a Rust closure across the C ABI. See the matching
    /// [`send_cancel_reply`](Self::send_cancel_reply) for the reply
    /// side. The high-level closure-based
    /// [`try_handle_cancel`](Self::try_handle_cancel) keeps working
    /// and now delegates to this pair.
    pub fn try_recv_cancel_request(&mut self) -> Result<Option<PendingCancelRequest>, NodeError> {
        let buf_start = self.cancel_buffer.as_ptr() as usize;
        let request = match self
            .cancel_goal_server
            .take_request(&mut self.cancel_buffer)
        {
            Ok(Some(r)) => r,
            Ok(None) | Err(TransportError::NoData) => return Ok(None),
            Err(_) => return Err(NodeError::Transport(TransportError::ServiceRequestFailed)),
        };

        let data_offset = (request.data.as_ptr() as usize).saturating_sub(buf_start);
        let data_len = request.data.len();
        let sequence_number = request.sequence_number;
        #[allow(clippy::drop_non_drop)]
        drop(request);

        let mut reader =
            CdrReader::new_with_header(&self.cancel_buffer[data_offset..data_offset + data_len])
                .map_err(|_| NodeError::Transport(TransportError::DeserializationError))?;

        let goal_id = read_goal_id(&mut reader)?;
        let current_status = self.find_goal_status(&goal_id);

        Ok(Some(PendingCancelRequest {
            goal_id,
            sequence_number,
            current_status,
        }))
    }

    /// Phase 122.3.c.6.d — send the reply to a previously-peeked
    /// cancel-goal request. `sequence_number` must match the value
    /// returned by [`try_recv_cancel_request`](Self::try_recv_cancel_request).
    ///
    /// `return_code` is the overall RPC status (`CancelReturnCode::Ok`
    /// = at least one cancel honoured; other variants = whole-request
    /// failure). `accepted` lists the goals that transition to
    /// `Canceling`; this function flips their stored status before
    /// publishing the status array.
    ///
    /// Issue 0796 — the parameter is a [`nros_core::CancelReturnCode`], not
    /// the per-goal [`nros_core::CancelResponse`] a cancel callback returns.
    /// The two were one type and their discriminants overlap with opposite
    /// meanings (`Reject`/`Ok` are both 0), so this signature is what stops a
    /// per-goal answer being written into the RPC field.
    pub fn send_cancel_reply(
        &mut self,
        sequence_number: i64,
        return_code: nros_core::CancelReturnCode,
        accepted: &[GoalId],
    ) -> Result<(), NodeError> {
        for id in accepted {
            self.set_goal_status(id, GoalStatus::Canceling);
        }

        let mut writer =
            crate::tx_writer(&mut self.goal_buffer).map_err(|_| NodeError::BufferTooSmall)?;
        writer
            .write_i8(return_code as i8)
            .map_err(|_| NodeError::Serialization)?;
        let count = u32::try_from(accepted.len()).unwrap_or(u32::MAX);
        writer
            .write_u32(count)
            .map_err(|_| NodeError::Serialization)?;
        for id in accepted {
            write_goal_id(&mut writer, id)?;
            // GoalInfo.stamp — zero timestamp.
            writer.write_i32(0).map_err(|_| NodeError::Serialization)?;
            writer.write_u32(0).map_err(|_| NodeError::Serialization)?;
        }
        let reply_len = writer.position();

        self.cancel_goal_server
            .send_response(sequence_number, &self.goal_buffer[..reply_len])
            .map_err(|_| NodeError::ServiceReplyFailed)?;

        if !accepted.is_empty() {
            let _ = self.publish_status_array();
        }

        Ok(())
    }

    /// Try to handle a cancel_goal request (type-agnostic).
    pub fn try_handle_cancel(
        &mut self,
        cancel_handler: impl FnOnce(&GoalId, GoalStatus) -> nros_core::CancelResponse,
    ) -> Result<Option<(GoalId, nros_core::CancelResponse)>, NodeError> {
        // (issue 0796) the handler answers about ONE goal; the reply below
        // carries the RPC-level return code. They are different types with
        // overlapping discriminants, so the translation is explicit.
        let buf_start = self.cancel_buffer.as_ptr() as usize;
        // Phase 120: NoData == steady-state idle; map to Ok(None).
        let request = match self
            .cancel_goal_server
            .take_request(&mut self.cancel_buffer)
        {
            Ok(Some(r)) => r,
            Ok(None) | Err(TransportError::NoData) => return Ok(None),
            Err(_) => return Err(NodeError::Transport(TransportError::ServiceRequestFailed)),
        };

        let data_offset = (request.data.as_ptr() as usize).saturating_sub(buf_start);
        let data_len = request.data.len();
        let sequence_number = request.sequence_number;
        #[allow(clippy::drop_non_drop)]
        drop(request);

        let mut reader =
            CdrReader::new_with_header(&self.cancel_buffer[data_offset..data_offset + data_len])
                .map_err(|_| NodeError::Transport(TransportError::DeserializationError))?;

        let goal_id = read_goal_id(&mut reader)?;

        let current_status = self.find_goal_status(&goal_id);
        let response = cancel_handler(&goal_id, current_status);

        let accepted = response == nros_core::CancelResponse::Accept;
        if accepted {
            self.set_goal_status(&goal_id, GoalStatus::Canceling);
        }

        // Serialize response: return_code (i8) + goals_canceling (sequence of GoalInfo)
        let return_code = if accepted {
            nros_core::CancelReturnCode::Ok
        } else {
            nros_core::CancelReturnCode::Rejected
        };
        let mut writer =
            crate::tx_writer(&mut self.goal_buffer).map_err(|_| NodeError::BufferTooSmall)?;
        writer
            .write_i8(return_code as i8)
            .map_err(|_| NodeError::Serialization)?;

        let num_canceling = if accepted { 1u32 } else { 0u32 };
        writer
            .write_u32(num_canceling)
            .map_err(|_| NodeError::Serialization)?;
        if accepted {
            write_goal_id(&mut writer, &goal_id)?;
            writer.write_i32(0).map_err(|_| NodeError::Serialization)?;
            writer.write_u32(0).map_err(|_| NodeError::Serialization)?;
        }
        let reply_len = writer.position();

        self.cancel_goal_server
            .send_response(sequence_number, &self.goal_buffer[..reply_len])
            .map_err(|_| NodeError::ServiceReplyFailed)?;

        Ok(Some((goal_id, response)))
    }

    /// Try to handle a get_result request using raw bytes.
    ///
    /// For completed goals, sends the stored raw result CDR from the slab.
    /// For active/unknown goals, sends the provided `default_result_cdr` bytes.
    ///
    /// `default_result_cdr` should contain serialized result data (without CDR
    /// header or status byte) — typically `A::Result::default()` serialized.
    pub fn try_handle_get_result_raw(
        &mut self,
        default_result_cdr: &[u8],
    ) -> Result<Option<GoalId>, NodeError> {
        let buf_start = self.goal_buffer.as_ptr() as usize;
        // Phase 120: NoData == steady-state idle; map to Ok(None).
        let request = match self.get_result_server.take_request(&mut self.goal_buffer) {
            Ok(Some(r)) => r,
            Ok(None) | Err(TransportError::NoData) => return Ok(None),
            Err(_) => return Err(NodeError::Transport(TransportError::ServiceRequestFailed)),
        };

        let data_offset = (request.data.as_ptr() as usize).saturating_sub(buf_start);
        let data_len = request.data.len();
        let sequence_number = request.sequence_number;
        #[allow(clippy::drop_non_drop)]
        drop(request);

        let mut reader =
            CdrReader::new_with_header(&self.goal_buffer[data_offset..data_offset + data_len])
                .map_err(|_| NodeError::Transport(TransportError::DeserializationError))?;

        let goal_id = read_goal_id(&mut reader)?;

        // Look up in completed results
        let completed = self
            .completed_results
            .iter()
            .find(|c| c.goal_id.uuid == goal_id.uuid);

        if let Some(entry) = completed {
            // Completed: send status + stored result CDR from the slab.
            let (off, len, status) = (entry.offset, entry.len, entry.status);
            self.reply_get_result_from_slab(sequence_number, status, off, len)?;
            // Issue 0796 — the client now holds these bytes, so this entry
            // becomes the first candidate for reclamation.
            self.mark_result_delivered(&goal_id);
        } else if self
            .active_goals
            .iter()
            .any(|g| g.goal_id.uuid == goal_id.uuid)
        {
            // Active goal → DEFER (Phase 237). `rclcpp_action` sends get_result
            // right after acceptance and expects the reply only once the goal
            // terminates; replying now with a non-terminal status makes the
            // client treat an unfinished goal as done. Hold the request's
            // correlation token; `complete_goal_raw` flushes it. The backend
            // retains the reply token keyed by `sequence_number`.
            if self
                .pending_get_results
                .push(PendingGetResult {
                    goal_id,
                    sequence_number,
                })
                .is_err()
            {
                // Table full — fail loud rather than silently strand the
                // requester (caller surfaces it; the request is not re-queued).
                return Err(NodeError::BufferTooSmall);
            }
        } else {
            // Unknown goal → reply immediately with UNKNOWN + default result.
            // Issue 0796: a goal whose result was reclaimed (evicted by a newer
            // completion) lands here too. That is deliberate — an honest
            // `Unknown` beats an unanswered query.
            self.reply_get_result_bytes(sequence_number, GoalStatus::Unknown, default_result_cdr)?;
        }

        Ok(Some(goal_id))
    }

    /// Write the `get_result` reply prologue — `[CDR header][status i8][pad to
    /// 4]` — into `goal_buffer` and return the offset the result bytes go at.
    ///
    /// The align-to-4 matters: the result CDR starts with a `u32` sequence
    /// length the reader will `align(4)` to.
    fn write_get_result_header(&mut self, status: GoalStatus) -> Result<usize, NodeError> {
        let mut writer =
            crate::tx_writer(&mut self.goal_buffer).map_err(|_| NodeError::BufferTooSmall)?;
        writer
            .write_i8(status as i8)
            .map_err(|_| NodeError::Serialization)?;
        writer.align(4).map_err(|_| NodeError::Serialization)?;
        Ok(writer.position())
    }

    /// Build + send a `get_result` reply — `[status i8][align(4)][result CDR]`
    /// — copying the result bytes from the slab. Shared by the immediate
    /// completed-goal path and the deferred flush in `complete_goal_raw`
    /// (Phase 237). `goal_buffer` and `result_slab` are disjoint fields, so the
    /// header build (into `goal_buffer`) and the slab copy don't alias.
    fn reply_get_result_from_slab(
        &mut self,
        sequence_number: i64,
        status: GoalStatus,
        slab_offset: usize,
        slab_len: usize,
    ) -> Result<(), NodeError> {
        let pos = self.write_get_result_header(status)?;
        if pos + slab_len > GOAL_BUF {
            return Err(NodeError::BufferTooSmall);
        }
        self.goal_buffer[pos..pos + slab_len]
            .copy_from_slice(&self.result_slab[slab_offset..slab_offset + slab_len]);
        let reply_len = pos + slab_len;
        self.get_result_server
            .send_response(sequence_number, &self.goal_buffer[..reply_len])
            .map_err(|_| NodeError::ServiceReplyFailed)
    }

    /// Same reply, from a caller-owned slice instead of the slab.
    ///
    /// Used for the `Unknown` reply and — issue 0796 — for a waiter whose
    /// result was too large to retain: the bytes exist in the caller's buffer
    /// even when the slab could not take a copy. `bytes` must NOT alias
    /// `self` (the slab path is [`reply_get_result_from_slab`]).
    fn reply_get_result_bytes(
        &mut self,
        sequence_number: i64,
        status: GoalStatus,
        bytes: &[u8],
    ) -> Result<(), NodeError> {
        let pos = self.write_get_result_header(status)?;
        if pos + bytes.len() > GOAL_BUF {
            return Err(NodeError::BufferTooSmall);
        }
        self.goal_buffer[pos..pos + bytes.len()].copy_from_slice(bytes);
        let reply_len = pos + bytes.len();
        self.get_result_server
            .send_response(sequence_number, &self.goal_buffer[..reply_len])
            .map_err(|_| NodeError::ServiceReplyFailed)
    }

    /// Get the number of active goals.
    pub fn active_goal_count(&self) -> usize {
        self.active_goals.len()
    }

    /// Get a reference to all active goals.
    pub fn active_goals(&self) -> &[RawActiveGoal] {
        &self.active_goals
    }

    /// Find the status of a goal (active or unknown).
    pub fn find_goal_status(&self, goal_id: &GoalId) -> GoalStatus {
        self.active_goals
            .iter()
            .find(|g| g.goal_id.uuid == goal_id.uuid)
            .map(|g| g.status)
            .unwrap_or(GoalStatus::Unknown)
    }

    /// Publish the current GoalStatusArray on the status topic.
    pub fn publish_status_array(&self) -> Result<(), NodeError> {
        let mut buf = [0u8; STATUS_ARRAY_BUF];
        let mut writer = crate::tx_writer(&mut buf).map_err(|_| NodeError::BufferTooSmall)?;

        writer
            .write_u32(self.active_goals.len() as u32)
            .map_err(|_| NodeError::Serialization)?;

        for goal in &self.active_goals {
            let stamped = GoalStatusStamped::new(GoalInfo::with_id(goal.goal_id), goal.status);
            stamped
                .serialize(&mut writer)
                .map_err(|_| NodeError::Serialization)?;
        }

        let len = writer.position();
        self.status_publisher
            .publish_raw(&buf[..len])
            .map_err(|_| NodeError::Transport(TransportError::PublishFailed))
    }
}

// ============================================================================
// ActionClientCore
// ============================================================================

/// Type-agnostic action client core handling the raw-bytes protocol.
///
/// The typed [`ActionClient`](super::handles::ActionClient) wraps this
/// and adds serialization/deserialization at the boundary.
pub struct ActionClientCore<
    const GOAL_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const RESULT_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
    const FEEDBACK_BUF: usize = { crate::config::DEFAULT_RX_BUF_SIZE },
> {
    pub(crate) send_goal_client: session::RmwServiceClient,
    pub(crate) cancel_goal_client: session::RmwServiceClient,
    pub(crate) get_result_client: session::RmwServiceClient,
    pub(crate) feedback_subscriber: session::RmwSubscriber,
    pub(crate) goal_buffer: [u8; GOAL_BUF],
    pub(crate) result_buffer: [u8; RESULT_BUF],
    pub(crate) feedback_buffer: [u8; FEEDBACK_BUF],
    pub(crate) goal_counter: u64,
    /// Phase 84.D3: per-sub-client in-flight flags. Each of the three
    /// sub-clients (send_goal / cancel / get_result) is an independent
    /// request/reply channel and tracks its own "unconsumed reply"
    /// state. Cleared by `Promise::take` on success.
    pub(crate) in_flight_send_goal: bool,
    pub(crate) in_flight_cancel: bool,
    pub(crate) in_flight_get_result: bool,
}

impl<const GOAL_BUF: usize, const RESULT_BUF: usize, const FEEDBACK_BUF: usize>
    ActionClientCore<GOAL_BUF, RESULT_BUF, FEEDBACK_BUF>
{
    /// Whether the action server's `send_goal` service is CURRENTLY visible.
    /// See `ActionClient::action_server_is_ready` for the semantic; the C and
    /// C++ `wait_for_action_server` bindings spin on this (phase-428 W13 — the
    /// asynchronous discovery pair they used to drive is gone).
    ///
    /// phase-379 W6 / issue 1008 — `Err` (the backend cannot answer) reports
    /// NOT ready. The deleted `is_server_ready` defaulted to `true`, so an
    /// image whose backend has no discovery claimed the server was up.
    /// Answering `false` makes a caller wait; answering `true` makes it send
    /// into the void.
    pub fn is_server_ready(&self) -> bool {
        use nros_rmw::ClientTrait;
        matches!(self.send_goal_client.service_is_ready(), Ok(true))
    }

    /// Create a new action client core from the raw transport handles.
    pub fn new(
        send_goal_client: session::RmwServiceClient,
        cancel_goal_client: session::RmwServiceClient,
        get_result_client: session::RmwServiceClient,
        feedback_subscriber: session::RmwSubscriber,
    ) -> Self {
        Self {
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
        }
    }

    /// Send a goal with raw CDR bytes. Returns the generated GoalId.
    ///
    /// The `goal_cdr` bytes are the serialized goal data (without GoalId framing).
    /// This writes GoalId + goal_cdr into the goal buffer and sends the request.
    ///
    /// After calling, use `send_goal_client` and `result_buffer` to construct
    /// a Promise for the acceptance reply.
    pub fn send_goal_raw(&mut self, goal_cdr: &[u8]) -> Result<GoalId, NodeError> {
        self.goal_counter += 1;
        let mut goal_id = GoalId::default();
        let counter_bytes = self.goal_counter.to_le_bytes();
        goal_id.uuid[..8].copy_from_slice(&counter_bytes);

        let mut writer =
            crate::tx_writer(&mut self.goal_buffer).map_err(|_| NodeError::BufferTooSmall)?;

        write_goal_id(&mut writer, &goal_id)?;

        // Copy raw goal CDR bytes after GoalId
        let pos = writer.position();
        if pos + goal_cdr.len() > GOAL_BUF {
            return Err(NodeError::BufferTooSmall);
        }
        self.goal_buffer[pos..pos + goal_cdr.len()].copy_from_slice(goal_cdr);
        let req_len = pos + goal_cdr.len();

        self.send_goal_client
            .send_request_raw(&self.goal_buffer[..req_len])
            .map(|_seq| ())
            .map_err(|_| NodeError::ServiceRequestFailed)?;

        Ok(goal_id)
    }

    /// Try to receive feedback (non-blocking, raw bytes).
    ///
    /// Returns the GoalId and total data length. The full CDR data
    /// (including GoalId) is in `feedback_buffer`.
    pub fn try_recv_feedback_raw(&mut self) -> Result<Option<(GoalId, usize)>, NodeError> {
        let data = self
            .feedback_subscriber
            .take_serialized(&mut self.feedback_buffer)
            .map_err(NodeError::Transport)?;

        let len = match data {
            Some(len) => len,
            None => return Ok(None),
        };

        let mut reader = CdrReader::new_with_header(&self.feedback_buffer[..len])
            .map_err(|_| NodeError::Transport(TransportError::DeserializationError))?;

        let goal_id = read_goal_id(&mut reader)?;

        Ok(Some((goal_id, len)))
    }

    /// Cancel a goal (non-blocking). Sends the cancel request.
    ///
    /// After calling, use `cancel_goal_client` and `result_buffer` to construct
    /// a Promise for the cancel response.
    pub fn send_cancel_request(&mut self, goal_id: &GoalId) -> Result<(), NodeError> {
        let mut writer =
            crate::tx_writer(&mut self.goal_buffer).map_err(|_| NodeError::BufferTooSmall)?;

        write_goal_id(&mut writer, goal_id)?;
        writer.write_i32(0).map_err(|_| NodeError::Serialization)?;
        writer.write_u32(0).map_err(|_| NodeError::Serialization)?;

        let req_len = writer.position();

        self.cancel_goal_client
            .send_request_raw(&self.goal_buffer[..req_len])
            .map(|_seq| ())
            .map_err(|_| NodeError::ServiceRequestFailed)
    }

    /// Send a get_result request.
    ///
    /// After calling, use `get_result_client` and `result_buffer` to construct
    /// a Promise for the result response.
    pub fn send_get_result_request(&mut self, goal_id: &GoalId) -> Result<(), NodeError> {
        let mut writer =
            crate::tx_writer(&mut self.goal_buffer).map_err(|_| NodeError::BufferTooSmall)?;

        write_goal_id(&mut writer, goal_id)?;

        let req_len = writer.position();

        self.get_result_client
            .send_request_raw(&self.goal_buffer[..req_len])
            .map(|_seq| ())
            .map_err(|_| NodeError::ServiceRequestFailed)
    }

    /// Phase 122.3.c.6.e — register a `Waker` that fires when the
    /// send_goal RPC reply lands.
    pub fn register_goal_response_waker(&self, waker: &core::task::Waker) {
        use nros_rmw::ClientTrait;
        self.send_goal_client.register_waker(waker);
    }

    /// Phase 122.3.c.6.e — register a `Waker` for cancel-RPC replies.
    pub fn register_cancel_response_waker(&self, waker: &core::task::Waker) {
        use nros_rmw::ClientTrait;
        self.cancel_goal_client.register_waker(waker);
    }

    /// Phase 122.3.c.6.e — register a `Waker` for get_result replies.
    pub fn register_result_waker(&self, waker: &core::task::Waker) {
        use nros_rmw::ClientTrait;
        self.get_result_client.register_waker(waker);
    }

    /// Phase 122.3.c.6.e — register a `Waker` for feedback messages.
    pub fn register_feedback_waker(&self, waker: &core::task::Waker) {
        use nros_rmw::Subscription;
        self.feedback_subscriber.register_waker(waker);
    }

    /// Phase 122.3.c.6.c — poll for a cancel reply (non-blocking,
    /// raw bytes). Returns `Ok(Some(len))` when a reply landed; the
    /// CDR payload is in `result_buffer_ref()[..len]`. Reply layout
    /// is action_msgs/srv/CancelGoal_Response wire CDR.
    pub fn try_recv_cancel_reply(&mut self) -> Result<Option<usize>, NodeError> {
        match self
            .cancel_goal_client
            .take_response_raw(&mut self.result_buffer)
        {
            // Issue 0778 — the sequence id is discarded HERE, not missing:
            // an action's cancel / get_result client keeps one call in flight
            // at a time, so the id adds nothing the caller can use yet. It is
            // available the moment that changes.
            Ok(opt) => Ok(opt.map(|(len, _seq)| len)),
            Err(TransportError::NoData) => Ok(None),
            Err(_) => Err(NodeError::Transport(TransportError::DeserializationError)),
        }
    }

    /// Poll for a get_result reply (non-blocking, raw bytes).
    ///
    /// Returns `Ok(Some(total_len))` if a reply arrived (data in result buffer),
    /// `Ok(None)` if no reply yet.
    ///
    /// After receiving, use [`result_buffer_ref()`](Self::result_buffer_ref)
    /// to access the raw CDR data. The layout is: CDR header (4) + status
    /// byte (1) + result data.
    pub fn try_recv_get_result_reply(&mut self) -> Result<Option<usize>, NodeError> {
        // Phase 120: NoData == steady-state polling; map to Ok(None).
        match self
            .get_result_client
            .take_response_raw(&mut self.result_buffer)
        {
            // Issue 0778 — the sequence id is discarded HERE, not missing:
            // an action's cancel / get_result client keeps one call in flight
            // at a time, so the id adds nothing the caller can use yet. It is
            // available the moment that changes.
            Ok(opt) => Ok(opt.map(|(len, _seq)| len)),
            Err(TransportError::NoData) => Ok(None),
            Err(_) => Err(NodeError::Transport(TransportError::DeserializationError)),
        }
    }

    /// Poll for the send_goal acceptance reply (non-blocking, raw bytes).
    ///
    /// Returns `Ok(Some(total_len))` if a reply arrived (data in result buffer),
    /// `Ok(None)` if no reply yet.
    ///
    /// The reply CDR contains: header (4) + accepted (u8) + stamp (i32 + u32).
    pub fn try_recv_send_goal_reply(&mut self) -> Result<Option<usize>, NodeError> {
        // Phase 120: NoData == steady-state polling; map to Ok(None).
        match self
            .send_goal_client
            .take_response_raw(&mut self.result_buffer)
        {
            // Issue 0778 — the sequence id is discarded HERE, not missing:
            // an action's cancel / get_result client keeps one call in flight
            // at a time, so the id adds nothing the caller can use yet. It is
            // available the moment that changes.
            Ok(opt) => Ok(opt.map(|(len, _seq)| len)),
            Err(TransportError::NoData) => Ok(None),
            Err(_) => Err(NodeError::Transport(TransportError::DeserializationError)),
        }
    }

    /// Read-only access to the result buffer (after polling a reply).
    pub fn result_buffer_ref(&self) -> &[u8] {
        &self.result_buffer
    }

    /// Read-only access to the feedback buffer (after receiving feedback).
    pub fn feedback_buffer_ref(&self) -> &[u8] {
        &self.feedback_buffer
    }

    /// Get the current goal counter (used to reconstruct the last goal ID).
    pub fn goal_counter(&self) -> u64 {
        self.goal_counter
    }
}
