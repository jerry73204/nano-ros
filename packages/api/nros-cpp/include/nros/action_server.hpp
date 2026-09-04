// nros-cpp: Action server class
// Freestanding C++ — no exceptions, no STL required

/**
 * @file action_server.hpp
 * @ingroup grp_action
 * @brief `nros::ActionServer<A>` — typed action server.
 */

#ifndef NROS_CPP_ACTION_SERVER_HPP
#define NROS_CPP_ACTION_SERVER_HPP

#include <cstdint>
#include <cstddef>
#include <string.h>

#include "nros/config.hpp"
#include "nros/result.hpp"
// issue 0964 — the feedback/result TX buffers below size through
// `detail::buffer_bounds`, so this header needs it directly rather than by
// luck through a transitive include.
#include "nros/size_bound.hpp"

// Phase 118.D — most action_server FFI symbols come from
// `nros_cpp_ffi.h`; cbindgen renders Rust `*const [u8; 16]` as
// pointer-to-array (`const uint8_t (*goal_id)[16]`), so callsites
// below `reinterpret_cast` from the array-decay shape used in C++
// member-function signatures.
//
// `nros_cpp_action_server_set_callbacks` is excluded from cbindgen
// output (its Rust signature uses `Option<extern "C" fn(...)>`,
// which cbindgen renders as opaque structs). Declare locally below
// with plain function-pointer typedefs that match the actual ABI.
#include "nros_cpp_ffi.h"

extern "C" {
typedef int32_t (*nros_cpp_goal_callback_t)(const uint8_t goal_id[16], const uint8_t* data,
                                            size_t len, void* ctx);
typedef int32_t (*nros_cpp_cancel_callback_t)(const uint8_t goal_id[16], void* ctx);
/* Issue 0796 — post-accept hook. Excluded from cbindgen for the same reason as
 * the two above (`Option<extern "C" fn>` renders as an opaque struct), so this
 * declaration is MIRRORED in `nros/component.h` as well; `just check c`'s
 * cross-include TU is the gate that catches a half-updated mirror. */
typedef void (*nros_cpp_accepted_callback_t)(const uint8_t goal_id[16], void* ctx);

nros_cpp_ret_t nros_cpp_action_server_set_callbacks(void* handle, nros_cpp_goal_callback_t goal_cb,
                                                    nros_cpp_cancel_callback_t cancel_cb,
                                                    void* ctx);
nros_cpp_ret_t
nros_cpp_action_server_set_accepted_callback(void* handle, nros_cpp_accepted_callback_t accepted_cb,
                                             void* ctx);
} // extern "C"

namespace nros {

/// Value type for a goal identifier — the 16 bytes ROS 2 carries as
/// `unique_identifier_msgs/msg/UUID`.
///
/// Phase-417 W4.b. Every action signature here spelled the goal id
/// `const uint8_t goal_id[16]`, which DECAYS to a pointer: it cannot be
/// compared with `==`, cannot be returned by value, and cannot be stored in a
/// container. `rclcpp_action::GoalUUID` is a `std::array<uint8_t, 16>` and is
/// a map key in most real action servers, so the array-decay spelling is a
/// drop-in BLOCKER rather than a style difference.
///
/// C has had `nros_goal_uuid_t` (`nros/nros_generated.h`) and Rust
/// `nros_core::GoalId` all along; C++ was the odd one out. This is a value
/// type over the SAME 16 bytes, not a second identity scheme: standard
/// layout, trivially copyable, `sizeof == 16`, no allocator, no call into
/// anything — so it reaches every target the freestanding headers do.
///
/// Generation is deliberately NOT here. C's `nros_goal_uuid_generate` draws
/// randomness, which is behaviour, and RFC-0019 keeps behaviour Rust-side; the
/// goal id a server sees is the one the FFI hands it.
///
/// The raw `const uint8_t[16]` overloads are kept beside every `GoalUUID` one:
/// the FFI trampolines hand callbacks a `const uint8_t[16]`, and in-tree
/// examples spell it that way in their own callback signatures. The
/// converting constructor is `explicit` so a raw-array call site still selects
/// the raw-array overload rather than silently converting.
struct GoalUUID {
    /// Byte count — `unique_identifier_msgs/UUID` is a fixed `uint8[16]`.
    static const size_t SIZE = 16;

    /// UUID bytes, in wire order.
    uint8_t uuid[16];

    /// The zero ("null") goal id — mirrors `nros_core::GoalId::zero()`.
    GoalUUID() : uuid{} {}

    /// Copy the 16 bytes at `src`, which must point to at least that many.
    explicit GoalUUID(const uint8_t* src) : uuid{} {
        for (size_t i = 0; i < SIZE; ++i) {
            uuid[i] = src[i];
        }
    }

    /// Pointer to the 16 bytes — what the raw-array overloads and the FFI take.
    uint8_t* data() { return uuid; }
    const uint8_t* data() const { return uuid; }

    /// True when every byte is zero — mirrors `nros_core::GoalId::is_zero()`.
    bool is_zero() const {
        for (size_t i = 0; i < SIZE; ++i) {
            if (uuid[i] != 0) return false;
        }
        return true;
    }

    bool operator==(const GoalUUID& other) const {
        for (size_t i = 0; i < SIZE; ++i) {
            if (uuid[i] != other.uuid[i]) return false;
        }
        return true;
    }

    bool operator!=(const GoalUUID& other) const { return !(*this == other); }

    /// Lexicographic order over the 16 bytes, so `GoalUUID` is usable as a
    /// `std::map` key — the shape `rclcpp_action::GoalUUID` gets for free by
    /// being a `std::array`. The ordering carries no meaning beyond that.
    bool operator<(const GoalUUID& other) const {
        for (size_t i = 0; i < SIZE; ++i) {
            if (uuid[i] != other.uuid[i]) return uuid[i] < other.uuid[i];
        }
        return false;
    }
};

static_assert(sizeof(GoalUUID) == 16,
              "nros::GoalUUID must be exactly the 16 bytes the FFI passes as uint8_t[16]");
static_assert(alignof(GoalUUID) == alignof(uint8_t),
              "nros::GoalUUID must not add alignment over its uint8_t[16]");

/// Goal acceptance response returned from the user's goal callback.
enum class GoalResponse : int32_t {
    Reject = 0,
    AcceptAndExecute = 1,
    AcceptAndDefer = 2,
};

/// Per-goal cancel decision returned from the user's cancel callback.
///
/// Issue 0796 — this is NOT the `action_msgs/srv/CancelGoal` RPC status; that
/// is `CancelReturnCode` below. The two carry different meanings on
/// overlapping values (`Reject` and `Ok` are both 0), so never cast between
/// them. C names them apart too (`nros_cancel_response_t` vs
/// `nros_cancel_return_code_t`), and Rust now does (`nros_core::CancelResponse`
/// vs `nros_core::CancelReturnCode`).
enum class CancelResponse : int32_t {
    Reject = 0,
    Accept = 1,
};

/// `action_msgs/srv/CancelGoal` RPC return code — the WHOLE-REQUEST outcome
/// (issue 0796).
///
/// Written by a polling-tier server into `send_cancel_reply`, and read by a
/// client from `ActionClient::try_recv_cancel_response`. Distinct from the
/// per-goal `CancelResponse` above; these four discriminants are the wire
/// contract, fixed by `action_msgs`.
enum class CancelReturnCode : int8_t {
    Ok = 0,
    Rejected = 1,
    UnknownGoal = 2,
    GoalTerminated = 3,
};

/// Mirror of `action_msgs/msg/GoalStatus` — lifecycle state reported by
/// `for_each_active_goal`.
enum class GoalStatus : int8_t {
    Unknown = 0,
    Accepted = 1,
    Executing = 2,
    Canceling = 3,
    Succeeded = 4,
    Canceled = 5,
    Aborted = 6,
};

/// Typed action server for a ROS 2 action.
///
/// Mirrors `rclcpp_action::Server<A>` with a callback-based API. The
/// action type `A` must provide nested `Goal`, `Result`, and `Feedback`
/// types with `TYPE_NAME`, `TYPE_HASH`, `SERIALIZED_SIZE_MAX`,
/// `ffi_serialize()`, and `ffi_deserialize()`.
///
/// Usage:
/// ```cpp
/// using Fib = example_interfaces::action::Fibonacci;
/// nros::ActionServer<Fib> srv;
/// NROS_TRY(node.create_action_server(srv, "/fibonacci"));
///
/// srv.set_goal_callback(
///     [](const uint8_t[16], const Fib::Goal& g) {
///         if (g.order > 46) return nros::GoalResponse::Reject;
///         return nros::GoalResponse::AcceptAndExecute;
///     });
/// ```
///
/// Callbacks must be stateless (empty-capture lambdas or plain function
/// pointers). This is a freestanding C++14 library without `std::function`,
/// so per-instance closure storage is not available.
template <typename A> class ActionServer {
  public:
    using GoalType = typename A::Goal;
    using ResultType = typename A::Result;
    using FeedbackType = typename A::Feedback;

    /// User-facing typed goal callback signature.
    using TypedGoalFn = GoalResponse (*)(const uint8_t uuid[16], const GoalType& goal);
    /// User-facing typed goal callback signature with user context (Phase 84.G9).
    using TypedGoalFnWithCtx = GoalResponse (*)(const uint8_t uuid[16], const GoalType& goal,
                                                void* ctx);
    /// User-facing typed cancel callback signature.
    using TypedCancelFn = CancelResponse (*)(const uint8_t uuid[16]);
    /// User-facing typed cancel callback signature with user context (Phase 84.G9).
    using TypedCancelFnWithCtx = CancelResponse (*)(const uint8_t uuid[16], void* ctx);
    /// User-facing typed accepted-goal callback signature (issue 0796).
    using TypedAcceptedFn = void (*)(const uint8_t uuid[16]);
    /// User-facing typed accepted-goal callback signature with user context.
    using TypedAcceptedFnWithCtx = void (*)(const uint8_t uuid[16], void* ctx);
    /// User-facing visitor signature for `for_each_active_goal`.
    using TypedVisitorFn = void (*)(const uint8_t uuid[16], GoalStatus status);

    /// Register a typed goal callback.
    ///
    /// `F` must be a stateless callable that decays to `TypedGoalFn`
    /// (empty-capture lambda or plain function pointer).
    /// F must be a stateless callable convertible to TypedGoalFn
    /// (empty-capture lambda or plain function pointer).
    template <typename F> Result set_goal_callback(F f) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        user_goal_fn_ = TypedGoalFn(f); // compile error if F is not convertible
        user_goal_fn_ctx_ = nullptr;    // mutually exclusive with _with_ctx
        user_goal_ctx_ = nullptr;
        return install_callbacks();
    }

    /// Register a typed goal callback with a user context pointer.
    ///
    /// The bare function pointer is stored alongside a `void*` that is
    /// forwarded to every invocation — lets callers reach stateful
    /// objects without capturing lambdas or file-scope globals. Overrides
    /// and is overridden by `set_goal_callback()` (the two modes are
    /// mutually exclusive).
    Result set_goal_callback_with_ctx(TypedGoalFnWithCtx f, void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        user_goal_fn_ctx_ = f;
        user_goal_ctx_ = ctx;
        user_goal_fn_ = nullptr;
        return install_callbacks();
    }

    /// Register a cancel callback.
    ///
    /// F must be a stateless callable convertible to TypedCancelFn.
    template <typename F> Result set_cancel_callback(F f) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        user_cancel_fn_ = TypedCancelFn(f); // compile error if F is not convertible
        user_cancel_fn_ctx_ = nullptr;      // mutually exclusive with _with_ctx
        user_cancel_ctx_ = nullptr;
        return install_callbacks();
    }

    /// Register a cancel callback with a user context pointer.
    ///
    /// Mirrors `set_goal_callback_with_ctx` — the bare function pointer
    /// receives a `void*` alongside each UUID so stateful cancel policies
    /// don't need captured lambdas or global state. Mutually exclusive
    /// with `set_cancel_callback()`.
    Result set_cancel_callback_with_ctx(TypedCancelFnWithCtx f, void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        user_cancel_fn_ctx_ = f;
        user_cancel_ctx_ = ctx;
        user_cancel_fn_ = nullptr;
        return install_callbacks();
    }

    /// Register a callback invoked once per ACCEPTED goal (issue 0796).
    ///
    /// Fires after the accept reply has reached the client, for
    /// `GoalResponse::AcceptAndExecute` and `GoalResponse::AcceptAndDefer`
    /// alike. This is where a deferred goal starts executing: the goal
    /// callback answers accept/reject and must return promptly, so it is the
    /// wrong place to begin work.
    ///
    /// Mirrors C's `nros_accepted_callback_t` (passed at
    /// `nros_action_server_init`) and Rust's third argument to
    /// `create_action_server_with_callbacks(goal, cancel, accepted)`. Without
    /// it a C++ user who returned ACCEPT_AND_DEFER was never told the goal had
    /// been accepted.
    ///
    /// F must be a stateless callable convertible to TypedAcceptedFn.
    template <typename F> Result set_accepted_callback(F f) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        user_accepted_fn_ = TypedAcceptedFn(f); // compile error if F is not convertible
        user_accepted_fn_ctx_ = nullptr;        // mutually exclusive with _with_ctx
        user_accepted_ctx_ = nullptr;
        return install_callbacks();
    }

    /// Register an accepted-goal callback with a user context pointer.
    ///
    /// Mirrors `set_goal_callback_with_ctx`. Mutually exclusive with
    /// `set_accepted_callback()`.
    Result set_accepted_callback_with_ctx(TypedAcceptedFnWithCtx f, void* ctx) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        user_accepted_fn_ctx_ = f;
        user_accepted_ctx_ = ctx;
        user_accepted_fn_ = nullptr;
        return install_callbacks();
    }

    /// Publish feedback for an active goal.
    ///
    /// @param goal_id  16-byte goal UUID from the goal callback.
    /// @param feedback Feedback to publish.
    /// @return Result indicating success or failure.
    Result publish_feedback(const uint8_t goal_id[16], const FeedbackType& feedback) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);

        uint8_t buf[::nros::detail::buffer_bounds<FeedbackType>::tx];
        size_t len = 0;
        if (FeedbackType::ffi_serialize(&feedback, buf, sizeof(buf), &len) != 0) {
            return Result(ErrorCode::Error);
        }
        return Result(nros_cpp_action_server_publish_feedback(
            storage_, executor_, reinterpret_cast<const uint8_t(*)[16]>(goal_id), buf, len));
    }

    /// Terminate a goal with a result and the status it terminated in.
    ///
    /// Issue 0796 — `status` used to be hardcoded `Succeeded` inside the shim,
    /// so a server that aborted a goal reported success to the client. The
    /// parameter order matches `PollingActionServer::complete_goal` and the C
    /// API's `nros_action_server_complete_goal_raw`, so the three surfaces
    /// agree; `Succeeded` is defaulted because it is the common case and
    /// because that keeps existing two-argument calls compiling.
    Result complete_goal(const uint8_t goal_id[16], GoalStatus status, const ResultType& result) {
        if (!initialized_) return Result(ErrorCode::NotInitialized);

        uint8_t buf[::nros::detail::buffer_bounds<ResultType>::tx];
        size_t len = 0;
        if (ResultType::ffi_serialize(&result, buf, sizeof(buf), &len) != 0) {
            return Result(ErrorCode::Error);
        }
        return Result(nros_cpp_action_server_complete_goal(
            storage_, executor_, reinterpret_cast<const uint8_t(*)[16]>(goal_id),
            static_cast<int32_t>(status), buf, len));
    }

    /// Terminate a goal as SUCCEEDED. Equivalent to
    /// `complete_goal(goal_id, GoalStatus::Succeeded, result)`.
    Result complete_goal(const uint8_t goal_id[16], const ResultType& result) {
        return complete_goal(goal_id, GoalStatus::Succeeded, result);
    }

    /// @ref publish_feedback taking a `GoalUUID` value (phase-417 W4.b).
    Result publish_feedback(const GoalUUID& goal_id, const FeedbackType& feedback) {
        return publish_feedback(goal_id.data(), feedback);
    }

    /// @ref complete_goal taking a `GoalUUID` value (phase-417 W4.b).
    Result complete_goal(const GoalUUID& goal_id, GoalStatus status, const ResultType& result) {
        return complete_goal(goal_id.data(), status, result);
    }

    /// @ref complete_goal taking a `GoalUUID` value (phase-417 W4.b).
    Result complete_goal(const GoalUUID& goal_id, const ResultType& result) {
        return complete_goal(goal_id.data(), GoalStatus::Succeeded, result);
    }

    /// Terminate `goal_id` as SUCCEEDED — phase-417 W4.b.
    ///
    /// C has `nros_action_succeed` / `_abort` / `_canceled`, Rust has
    /// `ActionServerHandle::succeed` / `abort` / `canceled`, and every
    /// rclcpp_action server ends with `goal_handle->succeed(result)`. C++ had
    /// only `complete_goal`, so the terminal line of a ported server was one
    /// nobody could keep. These three are FORWARDERS onto `complete_goal`:
    /// the goal state machine stays Rust-side (RFC-0019), and no state is
    /// added here.
    ///
    /// `complete_goal` remains the general form for a status computed at
    /// runtime.
    Result succeed(const uint8_t goal_id[16], const ResultType& result) {
        return complete_goal(goal_id, GoalStatus::Succeeded, result);
    }

    /// Terminate `goal_id` as ABORTED. See @ref succeed.
    Result abort(const uint8_t goal_id[16], const ResultType& result) {
        return complete_goal(goal_id, GoalStatus::Aborted, result);
    }

    /// Terminate `goal_id` as CANCELED. See @ref succeed.
    ///
    /// Spelled `canceled` — rclcpp_action's and C's spelling — not `cancel`.
    /// Rust used to disagree here and was renamed to match in the same work
    /// item.
    Result canceled(const uint8_t goal_id[16], const ResultType& result) {
        return complete_goal(goal_id, GoalStatus::Canceled, result);
    }

    /// @ref succeed taking a `GoalUUID` value.
    Result succeed(const GoalUUID& goal_id, const ResultType& result) {
        return complete_goal(goal_id.data(), GoalStatus::Succeeded, result);
    }

    /// @ref abort taking a `GoalUUID` value.
    Result abort(const GoalUUID& goal_id, const ResultType& result) {
        return complete_goal(goal_id.data(), GoalStatus::Aborted, result);
    }

    /// @ref canceled taking a `GoalUUID` value.
    Result canceled(const GoalUUID& goal_id, const ResultType& result) {
        return complete_goal(goal_id.data(), GoalStatus::Canceled, result);
    }

    /// Iterate over every currently live goal and invoke `f(uuid, status)`.
    ///
    /// `F` must be a stateless callable convertible to
    /// `void (*)(const uint8_t uuid[16], GoalStatus status)`. The arena
    /// never stores the original goal CDR payload, so only identity +
    /// status are forwarded — if you need the goal bytes, stash them in
    /// a `{uuid → state}` table from inside `set_goal_callback`.
    /// F must be a stateless callable convertible to void(*)(const uint8_t[16], GoalStatus).
    template <typename F> Result for_each_active_goal(F f) {
        using Fn = void (*)(const uint8_t[16], GoalStatus);
        if (!initialized_) return Result(ErrorCode::NotInitialized);
        user_visitor_fn_ = Fn(f); // compile error if F is not convertible

        auto trampoline = [](const uint8_t goal_id[16], int8_t status, void* ctx) {
            auto* self = static_cast<ActionServer*>(ctx);
            if (!self || self->user_visitor_fn_ == nullptr) return;
            self->user_visitor_fn_(goal_id, static_cast<GoalStatus>(status));
        };
        Result ret(nros_cpp_action_server_for_each_active_goal(
            storage_, executor_,
            reinterpret_cast<void (*)(const uint8_t(*)[16], int8_t, void*)>(+trampoline), this));
        user_visitor_fn_ = nullptr; // one-shot — don't leak the function pointer between calls
        return ret;
    }

    /// Check if the action server is initialized and valid.
    bool is_valid() const { return initialized_; }

    /// Destructor — releases action server resources.
    ~ActionServer() {
        if (initialized_) {
            nros_cpp_action_server_destroy(storage_);
            initialized_ = false;
        }
    }

    // Move semantics (non-copyable). Relocation goes through the
    // `nros_cpp_action_server_relocate` runtime call (Phase 84.C1) and
    // then `install_callbacks()` re-registers the goal/cancel trampolines
    // with the new `this` as the arena callback context — this is the one
    // type in nros-cpp that registers its storage address externally.
    ActionServer(ActionServer&& other)
        : executor_(other.executor_), user_goal_fn_(other.user_goal_fn_),
          user_goal_fn_ctx_(other.user_goal_fn_ctx_), user_goal_ctx_(other.user_goal_ctx_),
          user_cancel_fn_(other.user_cancel_fn_), user_cancel_fn_ctx_(other.user_cancel_fn_ctx_),
          user_cancel_ctx_(other.user_cancel_ctx_), user_accepted_fn_(other.user_accepted_fn_),
          user_accepted_fn_ctx_(other.user_accepted_fn_ctx_),
          user_accepted_ctx_(other.user_accepted_ctx_), user_visitor_fn_(other.user_visitor_fn_),
          initialized_(other.initialized_) {
        if (other.initialized_) {
            nros_cpp_action_server_relocate(other.storage_, storage_);
            other.initialized_ = false;
            install_callbacks();
        }
    }

    ActionServer& operator=(ActionServer&& other) {
        if (this != &other) {
            if (initialized_) {
                nros_cpp_action_server_destroy(storage_);
            }
            executor_ = other.executor_;
            user_goal_fn_ = other.user_goal_fn_;
            user_goal_fn_ctx_ = other.user_goal_fn_ctx_;
            user_goal_ctx_ = other.user_goal_ctx_;
            user_cancel_fn_ = other.user_cancel_fn_;
            user_cancel_fn_ctx_ = other.user_cancel_fn_ctx_;
            user_cancel_ctx_ = other.user_cancel_ctx_;
            user_accepted_fn_ = other.user_accepted_fn_;
            user_accepted_fn_ctx_ = other.user_accepted_fn_ctx_;
            user_accepted_ctx_ = other.user_accepted_ctx_;
            user_visitor_fn_ = other.user_visitor_fn_;
            initialized_ = other.initialized_;
            if (other.initialized_) {
                nros_cpp_action_server_relocate(other.storage_, storage_);
                other.initialized_ = false;
                install_callbacks();
            }
        }
        return *this;
    }

    /// Default constructor — creates an uninitialized action server.
    /// Use `Node::create_action_server()` to initialize.
    ActionServer()
        : executor_(nullptr), user_goal_fn_(nullptr), user_goal_fn_ctx_(nullptr),
          user_goal_ctx_(nullptr), user_cancel_fn_(nullptr), user_cancel_fn_ctx_(nullptr),
          user_cancel_ctx_(nullptr), user_accepted_fn_(nullptr), user_accepted_fn_ctx_(nullptr),
          user_accepted_ctx_(nullptr), user_visitor_fn_(nullptr), initialized_(false) {}

  private:
    ActionServer(const ActionServer&) = delete;
    ActionServer& operator=(const ActionServer&) = delete;

    friend class Node;

    // ── C trampolines ───────────────────────────────────────────────
    //
    // `ctx` is a pointer to this `ActionServer<A>` instance, so the
    // trampoline reads the user's stored function pointer via the
    // instance's own fields — no shared mutable statics.

    static int32_t goal_trampoline(const uint8_t goal_id[16], const uint8_t* data, size_t len,
                                   void* ctx) {
        auto* self = static_cast<ActionServer*>(ctx);
        if (!self) return static_cast<int32_t>(GoalResponse::Reject);
        GoalType g;
        if (GoalType::ffi_deserialize(data, len, &g) != 0) {
            return static_cast<int32_t>(GoalResponse::Reject);
        }
        if (self->user_goal_fn_ctx_ != nullptr) {
            return static_cast<int32_t>(self->user_goal_fn_ctx_(goal_id, g, self->user_goal_ctx_));
        }
        if (self->user_goal_fn_ != nullptr) {
            return static_cast<int32_t>(self->user_goal_fn_(goal_id, g));
        }
        return static_cast<int32_t>(GoalResponse::Reject);
    }

    static int32_t cancel_trampoline(const uint8_t goal_id[16], void* ctx) {
        auto* self = static_cast<ActionServer*>(ctx);
        if (!self) return static_cast<int32_t>(CancelResponse::Accept);
        if (self->user_cancel_fn_ctx_ != nullptr) {
            return static_cast<int32_t>(self->user_cancel_fn_ctx_(goal_id, self->user_cancel_ctx_));
        }
        if (self->user_cancel_fn_ != nullptr) {
            return static_cast<int32_t>(self->user_cancel_fn_(goal_id));
        }
        return static_cast<int32_t>(CancelResponse::Accept);
    }

    static void accepted_trampoline(const uint8_t goal_id[16], void* ctx) {
        auto* self = static_cast<ActionServer*>(ctx);
        if (!self) return;
        if (self->user_accepted_fn_ctx_ != nullptr) {
            self->user_accepted_fn_ctx_(goal_id, self->user_accepted_ctx_);
            return;
        }
        if (self->user_accepted_fn_ != nullptr) {
            self->user_accepted_fn_(goal_id);
        }
    }

    Result install_callbacks() {
        bool goal_set = (user_goal_fn_ != nullptr) || (user_goal_fn_ctx_ != nullptr);
        bool cancel_set = (user_cancel_fn_ != nullptr) || (user_cancel_fn_ctx_ != nullptr);
        bool accepted_set = (user_accepted_fn_ != nullptr) || (user_accepted_fn_ctx_ != nullptr);
        nros_cpp_goal_callback_t gcb = goal_set ? &goal_trampoline : nullptr;
        nros_cpp_cancel_callback_t ccb = cancel_set ? &cancel_trampoline : nullptr;
        nros_cpp_ret_t ret = nros_cpp_action_server_set_callbacks(storage_, gcb, ccb, this);
        if (ret != 0) return Result(ret);
        // Issue 0796 — a SECOND install call rather than a fourth parameter on
        // the first: `nros_cpp_action_server_set_callbacks` is declared in
        // three places and called directly by C components, so growing it
        // would have broken every one of them. Both calls pass the same
        // `this`, which is the shared context slot the runtime keeps.
        nros_cpp_accepted_callback_t acb = accepted_set ? &accepted_trampoline : nullptr;
        return Result(nros_cpp_action_server_set_accepted_callback(storage_, acb, this));
    }

    alignas(8) uint8_t storage_[NROS_CPP_ACTION_SERVER_STORAGE_SIZE];
    void* executor_; // Executor context needed for feedback/result operations
    TypedGoalFn user_goal_fn_;
    TypedGoalFnWithCtx user_goal_fn_ctx_;
    void* user_goal_ctx_;
    TypedCancelFn user_cancel_fn_;
    TypedCancelFnWithCtx user_cancel_fn_ctx_;
    void* user_cancel_ctx_;
    TypedAcceptedFn user_accepted_fn_;
    TypedAcceptedFnWithCtx user_accepted_fn_ctx_;
    void* user_accepted_ctx_;
    TypedVisitorFn user_visitor_fn_;
    bool initialized_;
    // Phase 87.6 put a `char action_name_[256]` here for a `get_action_name()`
    // that was never written; phase-417 W4.b deleted it. It was populated at
    // construction and read by nothing, and the name it copied is already held
    // Rust-side in the runtime struct (`nros_action_server_t::action_name`), so
    // it was 256 bytes of duplicated state per server on targets whose whole
    // malloc arena defaults to 16 KB. If an accessor is wanted, it belongs on
    // an FFI getter over the name the runtime already owns (RFC-0019), not on a
    // second copy here.
};

} // namespace nros

// Phase 84.G8: out-of-line definition of Node::create_action_server<A>().
#include "nros/node.hpp"

namespace nros {

template <typename A>
Result Node::create_action_server(ActionServer<A>& out, const char* action_name, const QoS& qos,
                                  const ActionServerOptions& options) {
    if (!initialized_) return Result(ErrorCode::NotInitialized);
    nros_cpp_qos_t ffi_qos = detail::qos_to_ffi(qos);
    nros_cpp_ret_t ret = nros_cpp_action_server_create(&handle_, action_name, A::TYPE_NAME,
                                                       A::Goal::TYPE_HASH, ffi_qos, out.storage_);
    if (ret != 0) return Result(ret);
    // Register with executor — creates transport handles (3 queryables + 2 publishers).
    // Deferred from create to avoid FreeRTOS QEMU deadlocks. Phase 87.6:
    // names are passed at register-time (buffers live on the C++
    // `nros::ActionServer<A>` class, not in the runtime struct).
    // Phase 189.M3.3.c — `sched_context` binds the action's (arena-registered)
    // goal-service handle to a scheduling context. UNSET ⇒ 0 (inherit, no-op).
    uint8_t sched = (options.sched_context == SCHED_CONTEXT_UNSET)
                        ? 0u
                        : static_cast<uint8_t>(options.sched_context);
    ret = nros_cpp_action_server_register(out.storage_, executor_handle_, action_name, A::TYPE_NAME,
                                          A::Goal::TYPE_HASH, sched);
    if (ret == 0) {
        out.executor_ = executor_handle_;
        out.initialized_ = true;
    }
    return Result(ret);
}

} // namespace nros

// ============================================================================
// rclcpp:: — the ROS 2 spelling (RFC-0089 stage 6, step A)
// ============================================================================
//
// Moved here from `nros/rclcpp_compat.hpp`, which no longer carries a surface
// of its own: RFC-0089 §"Naming: replace, with alias as the migration step"
// makes the ROS 2 spelling a first-class name declared by the API header that
// owns the concept, at which point a shim has nothing left to bridge.

// `rclcpp_action::Server<A>` — a type alias only. The call shapes
// (`send_goal_async`, the goal-response callbacks) are their own item; what a
// ported file gets today is the type name and everything `nros::ActionServer`
// already offers under it.
namespace rclcpp_action {
template <typename A> using Server = ::nros::ActionServer<A>;
} // namespace rclcpp_action

#endif // NROS_CPP_ACTION_SERVER_HPP
