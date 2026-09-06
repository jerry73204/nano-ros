//! Proc macros for nros message type generation
//!
//! Provides `#[derive(RosMessage)]` and `#[derive(RosService)]` macros.

use proc_macro::TokenStream;
use quote::quote;
use syn::{DeriveInput, Fields, LitByteStr, LitStr, Path, parse_macro_input};

// Phase 212.N.9 — `nros::main!()` proc-macro family. Replaces today's
// Entry-pkg `build.rs + include!(concat!(env!("OUT_DIR"), "/run_plan.rs"))`
// shape with a one-line `main.rs`. See `main_macro.rs` for the impl.
/// phase-432 W2.4 — the Rust-entry parity gate: the shared corpus rendered by
/// THIS crate's `quote!` and compared against `nros codegen entry`'s committed
/// rendering of the same facts. Test-only; it reads files from the repo.
#[cfg(test)]
mod entry_parity;
mod main_macro;
mod source_metadata_sidecars;

/// One-line `fn main()` for Entry pkgs. Four forms:
///
/// ```ignore
/// nros::main!();                                          // single-node self-bringup
/// nros::main!(board = LinuxBoard);                       // single-node, explicit board
/// nros::main!(launch = "demo_bringup");                   // multi-node, default launch
/// nros::main!(launch = "demo_bringup:sim.launch.xml");    // multi-node, explicit file
/// nros::main!(
///     board  = LinuxBoard,
///     launch = "demo_bringup:sim.launch.xml",
///     args   = [("use_sim", "true")],
/// );
/// ```
///
/// Reads `[package.metadata.nros.entry] deploy = "<board>"` from the
/// Entry pkg's own `Cargo.toml` when `board = …` is absent; consults
/// the workspace pkg-index (Phase 212.N.10) to resolve the bringup
/// pkg's launch file (Phase 212.N.11 parser).
///
/// Emits `fn main()` that delegates to
/// `<Board as ::nros::__macro_support::nros_platform::BoardEntry>::run(...)`,
/// dispatching one `<pkg>::register(runtime)?;` call per
/// launch-XML `<node>` entry. See `docs/design/0024-multi-node-workspace-layout.md`
/// §11.6 for the design lock.
#[proc_macro]
pub fn main(input: TokenStream) -> TokenStream {
    main_macro::expand(input)
}

/// Sanitise a cargo package name into a C-identifier-safe symbol component.
///
/// Cargo allows `-` in package names; C identifiers don't. Each non
/// `[A-Za-z0-9_]` byte is replaced with `_` so the result is a valid
/// suffix for the per-pkg register symbol emitted by [`node!`].
///
/// Crate-private (proc-macro crates can't export non-macro items); the
/// `sanitize_tests` module exercises it directly.
fn sanitize_pkg_name_for_symbol(pkg: &str) -> String {
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

/// Derive macro for ROS message types
///
/// Generates `Serialize`, `Deserialize`, and `RosMessage` implementations.
///
/// # Attributes
///
/// - `#[ros(type_name = "...")]` - Full ROS type name (required)
/// - `#[ros(hash = "...")]` - RIHS type hash (required)
///
/// # Example
///
/// ```ignore
/// use nros_macros::RosMessage;
///
/// #[derive(RosMessage)]
/// #[ros(type_name = "std_msgs::msg::dds_::String_")]
/// #[ros(hash = "abc123...")]
/// pub struct StringMsg {
///     pub data: heapless::String<256>,
/// }
/// ```
#[proc_macro_derive(RosMessage, attributes(ros))]
pub fn derive_ros_message(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;
    let generics = &input.generics;
    let (impl_generics, ty_generics, where_clause) = generics.split_for_impl();

    // Extract attributes
    let mut type_name: Option<String> = None;
    let mut type_hash: Option<String> = None;

    for attr in &input.attrs {
        if attr.path().is_ident("ros") {
            let _ = attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("type_name") {
                    let value: LitStr = meta.value()?.parse()?;
                    type_name = Some(value.value());
                } else if meta.path.is_ident("hash") {
                    let value: LitStr = meta.value()?.parse()?;
                    type_hash = Some(value.value());
                }
                Ok(())
            });
        }
    }

    let type_name = type_name.unwrap_or_else(|| format!("{}::msg::dds_::{}_", "unknown", name));
    let type_hash = type_hash.unwrap_or_else(|| "0".repeat(64));

    // Get fields
    let fields = match &input.data {
        syn::Data::Struct(data) => match &data.fields {
            Fields::Named(fields) => &fields.named,
            Fields::Unit => {
                // Unit struct (no fields)
                return generate_unit_struct_impl(
                    name,
                    &impl_generics,
                    &ty_generics,
                    where_clause,
                    &type_name,
                    &type_hash,
                );
            }
            _ => {
                return syn::Error::new_spanned(&input, "RosMessage only supports named fields")
                    .to_compile_error()
                    .into();
            }
        },
        _ => {
            return syn::Error::new_spanned(&input, "RosMessage can only be derived for structs")
                .to_compile_error()
                .into();
        }
    };

    // Generate serialize calls for each field
    let serialize_fields = fields.iter().map(|f| {
        let field_name = &f.ident;
        quote! {
            self.#field_name.serialize(writer)?;
        }
    });

    // Generate deserialize calls for each field
    let deserialize_fields = fields.iter().map(|f| {
        let field_name = &f.ident;
        quote! {
            #field_name: Deserialize::deserialize(reader)?,
        }
    });

    let expanded = quote! {
        impl #impl_generics nros_serdes::Serialize for #name #ty_generics #where_clause {
            fn serialize(&self, writer: &mut nros_serdes::CdrWriter) -> Result<(), nros_serdes::SerError> {
                use nros_serdes::Serialize;
                #(#serialize_fields)*
                Ok(())
            }
        }

        impl #impl_generics nros_serdes::Deserialize for #name #ty_generics #where_clause {
            fn deserialize(reader: &mut nros_serdes::CdrReader) -> Result<Self, nros_serdes::DeserError> {
                use nros_serdes::Deserialize;
                Ok(Self {
                    #(#deserialize_fields)*
                })
            }
        }

        impl #impl_generics nros_core::RosMessage for #name #ty_generics #where_clause {
            const TYPE_NAME: &'static str = #type_name;
            const TYPE_HASH: &'static str = #type_hash;
        }
    };

    TokenStream::from(expanded)
}

/// Export a Rust type implementing `nros::Node` as the package node.
///
/// Phase 212.N.12 — `nros::node!()` is the canonical name (matches the
/// rclcpp_components / ROS 2 launch.xml `<node pkg=…>` convention). The
/// legacy `nros::node!()` macro and `Component*` trait family were
/// retired in the same phase.
///
/// # Emitted items
///
/// Per invocation the macro currently emits:
///
/// 1. `pub fn register(runtime: &mut RuntimeCtx<'_>) -> Result<(), RuntimeError>`
///    — the Entry-pkg-callable wrapper that registers the four typed
///    `register` / `init` / `dispatch` / `tick` trampolines with the
///    runtime by stable pkg name (Phase 212.N.7 step-3.4).
/// 2. `#[unsafe(no_mangle)] pub extern "C" fn
///    __nros_node_<pkg>_dispatch_strategy() -> u8` — Phase 216.A.5
///    ABI export of the Node's [`DispatchStrategy`] discriminant
///    (`<Type as Node>::DISPATCH as u8`). Read out-of-tree by
///    `nros check` (216.D.1) and consumed from a separate compilation
///    unit by the RTIC (216.B.3) / Embassy (216.C.3) dispatch tasks.
///    `<pkg>` is the value of `CARGO_PKG_NAME` after
///    `sanitize_pkg_name_for_symbol` (hyphens → underscores).
///
/// [`DispatchStrategy`]: ../nros/enum.DispatchStrategy.html
///
/// # Example
///
/// ```ignore
/// struct Talker;
///
/// impl nros::Node for Talker {
///     const NAME: &'static str = "talker";
///
///     fn register(ctx: &mut nros::NodeContext<'_>) -> nros::NodeResult<()> {
///         let mut node = ctx.create_node(
///             nros::NodeId::new("node"),
///             nros::NodeOptions::new("talker"),
///         )?;
///         let _pub = node.create_publisher::<std_msgs::msg::String>(
///             nros::EntityId::new("pub_chatter"),
///             "chatter",
///         )?;
///         Ok(())
///     }
/// }
///
/// nros::node!(Talker);
/// ```
#[proc_macro]
pub fn node(input: TokenStream) -> TokenStream {
    node_impl(input)
}

fn node_impl(input: TokenStream) -> TokenStream {
    let node_ty = parse_macro_input!(input as Path);

    // Phase 212.N.7 step-3.4 — the package-name string handed to
    // `register_dispatch_slot_dyn` (diagnostics) +
    // `RuntimeError::ComponentRegister` is the sanitised pkg-name used by
    // the codegen-emitted `run_plan` to reference each Node pkg, so
    // the two strings round-trip 1:1.
    //
    // `proc_macro::tracked_env::var` is still unstable, so we use plain
    // `std::env::var`. Cargo sets `CARGO_PKG_NAME` for every compilation
    // (proc-macro crates inherit the parent crate's env at expansion).
    // The fallback "unknown" only triggers in toolchains that don't set
    // it (none today); it keeps the macro robust against future hosts.
    let pkg_raw = std::env::var("CARGO_PKG_NAME").unwrap_or_else(|_| "unknown".to_string());
    let pkg_sym = sanitize_pkg_name_for_symbol(&pkg_raw);
    let pkg_name_lit = pkg_sym.clone();

    // Phase 216.A.5 — per-Node-pkg `DispatchStrategy` ABI export. The
    // identifier is `__nros_node_<pkg>_dispatch_strategy`, sharing the
    // same sanitised `<pkg>` substring as the diagnostics string above
    // so out-of-tree tools (`nros check`, RTIC/Embassy dispatch tasks)
    // can resolve the symbol from `CARGO_PKG_NAME` directly.
    let dispatch_fn_name = quote::format_ident!("__nros_node_{}_dispatch_strategy", pkg_sym);
    // Phase 257 (W0-B / D6) — the uniform cross-language install seam.
    let component_install_fn_name = quote::format_ident!("__nros_component_{}_install", pkg_sym);
    // phase-391 W5.3b — per-class slot storage. Emitted HERE, in the macro,
    // because `#node_ty` is concrete at this site: the storage is sized
    // `size_of::<TypedSlot<C>>()` exactly, and a `static` inside the generic
    // install fn would be SHARED across every component class (one static per
    // generic fn, not per monomorphisation — probed, not assumed).
    let component_store_name = quote::format_ident!("__NROS_COMPONENT_{}_SLOT_STORE", pkg_sym);
    let component_present_name = quote::format_ident!("__NROS_NODE_PKG_{}_EXPORT_PRESENT", pkg_sym);
    let component_class_name = quote::format_ident!("__nros_component_{}_class_name", pkg_sym);
    let node_class_leaf = node_ty
        .segments
        .last()
        .map(|segment| segment.ident.to_string())
        .unwrap_or_else(|| "Node".to_string());
    let component_class = format!("{}::{}\0", pkg_sym, node_class_leaf);
    let component_class_len = component_class.len();
    let component_class_bytes =
        LitByteStr::new(component_class.as_bytes(), proc_macro2::Span::call_site());

    // Phase 216.A.5 follow-up — the on_callback trampoline the B.3 RTIC /
    // C.3 Embassy dispatch tasks invoke after dequeuing a
    // `SignaledCallback<'static>` from the SPSC ring. The chosen signature
    // is `extern "C"` with `(state, cb_id_ptr, cb_id_len, ctx)` because:
    //
    //   1. `SignaledCallback.cb_id` is `&'a str` (Phase 216.A.2 layout,
    //      `nros-platform/src/board/runtime.rs:47`) — a Rust fat pointer.
    //      The dispatch task already has `(ptr, len)` in hand from the
    //      `&str` it pulls off the ring, so passing them as separate
    //      `*const u8 + usize` args costs zero packing/unpacking.
    //   2. Sibling export `__nros_node_<pkg>_dispatch_strategy` uses
    //      `extern "C"` — keeping the same ABI across the two
    //      macro-emitted symbols means out-of-tree tools (`nros check`,
    //      the RTIC/Embassy dispatch tasks themselves) resolve both via
    //      the same `dlsym` / objdump path with no cfg-driven ABI
    //      branching.
    //   3. `extern "C"` with raw pointers is `no_std`-clean and matches
    //      the spec block in Phase 216.A.5 verbatim.
    let on_callback_fn_name = quote::format_ident!("__nros_node_{}_on_callback", pkg_sym);

    // Phase 212.N.7 step-6 — the legacy `#[unsafe(no_mangle)] extern
    // "Rust" fn __nros_component_<pkg>_{register,init,dispatch,tick}`
    // symbols and the `__NROS_COMPONENT_<PKG>_EXPORT_PRESENT` `#[used]`
    // marker are gone. They existed for the Phase 212.M.5.a BSP baker
    // (`freertos-qemu-mps2-an385-bsp` — retired in step-4), which
    // walked the mangled names from a generated `system_main.rs`. The
    // Phase 212.N Entry pkg path calls `<pkg>::register(runtime)`
    // directly through the path API, so the four typed fns now live
    // as local items inside the `register(runtime)` wrapper. The
    // macro emits ONE public item: the wrapper itself.
    let expanded = quote! {
        // Phase 212.N.7 step-3.4 — Entry-pkg-callable `register(runtime)`
        // wrapper. The codegen-emitted `run_plan(runtime)` body
        // (`nros-build::generate_run_plan`) dispatches one
        // `<pkg>::register(runtime)?` call per launch-XML `<node>` entry,
        // so every Node pkg whose `lib.rs` invokes `nros::node!()`
        // gets a stable per-pkg API here.
        //
        // SAFETY (transmutes below): typed `fn(args...) -> ret` /
        // `unsafe fn(args...) -> ret` and the zero-arg
        // `extern "Rust" fn()` aliases share the same ABI representation
        // (one pointer); the transmute is purely a type-level
        // reinterpretation. The impl-side transmute on the other side
        // (`nros::node_runtime`) recovers the same typed signature
        // before invoking — the round-trip is type-preserving so long
        // as both sides agree on the typed signature, which they do
        // (both live in `nros`).
        //
        // Phase 212.M-F.13 path (b): emit references go through
        // `::nros::__macro_support::nros_platform::*` rather than the
        // bare `::nros_platform::*` path so Node pkgs only need
        // a single `nros` dep in their `Cargo.toml`. The
        // `__macro_support` module is a `#[doc(hidden)]` re-export
        // alias maintained by `packages/api/nros/src/lib.rs`.

        // Phase 257 (W0-B / D6) — the uniform cross-language component-install seam.
        // A typed Entry of ANY language calls
        // `__nros_component_<pkg>_install(node, executor, self)` to install this Node
        // onto the shared `Executor` it hands in (`executor` = the entry's
        // `nros::global_handle()` / a node's `executor_handle()` = `*mut Executor`).
        // For a Rust node `_node`/`_self` are unused — it self-creates its node (its
        // `Node::NAME`) on the shared executor (phase-257 D7 Option C). Returns 0 on
        // success, nonzero on a registration error (or -1 without the cffi runtime).
        // `not_unsafe_ptr_arg_deref`: this is a C-ABI export — the raw-ptr
        // params are mandatory (the foreign typed entry hands in the executor
        // handle) and the body is `unsafe`. The deref-safety contract lives on
        // `install_node_typed`'s caller (the entry), not this thin trampoline.
        // phase-391 W5.3b — this class's slot storage: heap-free on the FFI
        // install path, exactly sized because the type is concrete here.
        // Shared by the trampoline below and `register(runtime)`; capacity is
        // the per-class instance cap (NROS_RUNTIME_MAX_CLASS_INSTANCES).
        // phase-391 W5-endgame step 2c (issue 0857) — the store's registry
        // capacities come from the class's own `ENTITY_BOUNDS` (knob caps by
        // default, exact when the class declares them), so a static cell
        // pays for what the class declares, not the worst case.
        #[doc(hidden)]
        static #component_store_name: ::nros::ComponentSlotStorage<
            #node_ty,
            { ::nros::config::MAX_CLASS_INSTANCES },
            { <#node_ty as ::nros::Node>::ENTITY_BOUNDS.publishers },
            { <#node_ty as ::nros::Node>::ENTITY_BOUNDS.service_clients },
            { <#node_ty as ::nros::Node>::ENTITY_BOUNDS.action_clients },
            { <#node_ty as ::nros::Node>::ENTITY_BOUNDS.action_servers },
            { <#node_ty as ::nros::Node>::ENTITY_BOUNDS.service_servers },
        > = ::nros::ComponentSlotStorage::new();

        #[allow(clippy::not_unsafe_ptr_arg_deref)]
        #[unsafe(no_mangle)]
        pub extern "C" fn #component_install_fn_name(
            _node: *const ::core::ffi::c_void,
            executor: *mut ::core::ffi::c_void,
            _self: *mut ::core::ffi::c_void,
        ) -> i32 {
            unsafe { ::nros::install_node_typed_in(executor, &#component_store_name) }
        }

        #[unsafe(no_mangle)]
        pub static #component_present_name: u8 = 1;

        #[unsafe(no_mangle)]
        pub static #component_class_name: [u8; #component_class_len] = *#component_class_bytes;

        #[unsafe(no_mangle)]
        pub extern "C" fn #dispatch_fn_name() -> u8 {
            <#node_ty as ::nros::Node>::DISPATCH as u8
        }

        // Phase 216.A.5 follow-up — extern "C" trampoline the B.3 RTIC
        // and C.3 Embassy dispatch tasks call after dequeuing a
        // `SignaledCallback<'static>` from the SPSC ring + resolving the
        // owning Node-pkg by `CallbackId` (the registry is a separate
        // follow-up). The call sequence on the dispatch-task side is:
        //
        //   let cb = ring.pop()?;                       // SignaledCallback<'static>
        //   let pkg = lookup(cb.cb_id)?;                // Node-by-cb_id registry (TBD)
        //   let f = dlsym(pkg, "__nros_node_<pkg>_on_callback")?;
        //   let state = node_state_table[pkg];          // *mut State, from i()
        //   f(state, cb.cb_id.as_ptr(), cb.cb_id.len(),
        //     cb.ctx_ptr);
        //
        // The trampoline reconstitutes the `&'static str` from
        // `(cb_id_ptr, cb_id_len)` — both ends agree on UTF-8 (every
        // `CallbackId` is built from a Rust `&str`), so the
        // `from_utf8_unchecked` is sound by construction. The `state`
        // and `ctx` re-cast mirrors the existing `unsafe fn d()` inside
        // `register()` below — the only delta is the extern "C" surface
        // + the (ptr, len) split for `cb_id`.
        //
        // SAFETY at the call site (documented for future
        // dispatch-task authors — the trampoline does NOT recheck):
        //   * `state` must point to a live `<NodeTy as ExecutableNode>::State`
        //     produced by this pkg's `i()` (= same as the existing
        //     `unsafe fn d()` contract).
        //   * `cb_id_ptr` must point to `cb_id_len` valid UTF-8 bytes
        //     with `'static` lifetime — produced by the codegen-emitted
        //     `CallbackId(&'static str)` literals.
        //   * `ctx` must point to a live `CallbackCtx<'static>` the
        //     dispatch task owns for the duration of the call.
        //   * Concurrent calls against the same `state` are forbidden
        //     (same dispatch-slot non-reentrancy as `unsafe fn d()`).
        #[unsafe(no_mangle)]
        pub unsafe extern "C" fn #on_callback_fn_name(
            state: *mut ::core::ffi::c_void,
            cb_id_ptr: *const u8,
            cb_id_len: usize,
            ctx: *mut ::core::ffi::c_void,
        ) {
            // SAFETY: caller upholds the four bullets above.
            let cb_id = unsafe {
                let bytes = ::core::slice::from_raw_parts(cb_id_ptr, cb_id_len);
                let s: &'static str = ::core::str::from_utf8_unchecked(bytes);
                ::nros::CallbackId(s)
            };
            let state_mut = unsafe {
                &mut *(state as *mut <#node_ty as ::nros::ExecutableNode>::State)
            };
            let ctx_mut = unsafe {
                &mut *(ctx as *mut ::nros::CallbackCtx<'static>)
            };
            <#node_ty as ::nros::ExecutableNode>::on_callback(
                state_mut,
                ::nros::Callback::__from_id(cb_id),
                ctx_mut,
            );
        }

        /// Phase 216 final wave — framework-side per-Node registration.
        ///
        /// The `nros::main!()` proc-macro emits one
        /// `<pkg>::register_dispatch(&mut executor)?;` call per
        /// declared Node pkg in the RTIC / Embassy `#[init]` body
        /// (see `packages/core/nros-macros/src/main_macro.rs`
        /// `Framework::Rtic` / `Framework::Embassy` emit branches).
        /// The call:
        ///
        ///   1. Constructs the Node's per-pkg `State` blob via
        ///      `<NodeTy as ExecutableNode>::init()`.
        ///   2. Leaks the state into a raw `*mut c_void` pointer via
        ///      `__private_node_state_into_raw` (the same shape the
        ///      `register(runtime)` wrapper's local `i()` fn uses).
        ///   3. Pushes `(state_ptr, __nros_node_<pkg>_on_callback)`
        ///      onto the `Executor`'s dispatch-slot registry, where
        ///      the framework dispatch task's
        ///      `executor.dispatch_callback(cb_id, ctx)` calls scan
        ///      and invoke it.
        ///
        /// Returns `Err(())` if the Executor's slot table is full
        /// (`MAX_NODES` entries — raise via
        /// `NROS_EXECUTOR_MAX_NODES` at build time). The Entry pkg
        /// surfaces the failure with `expect("register dispatch")`
        /// so a too-many-Nodes misconfig fails loud at boot rather
        /// than dropping dispatch silently.
        ///
        /// References `::nros::Executor<'static>` which is gated on `rmw-cffi`
        /// inside the `nros` crate. The Node pkg must depend on `nros`
        /// with the `rmw-cffi` feature enabled (every shipped 216
        /// example pkg does). A consumer w/o `rmw-cffi` enabled gets a
        /// hard `cannot find type Executor` error at expansion site —
        /// surfaces the misconfig loud rather than silently producing
        /// a Node pkg that can't be deployed to a framework target.
        pub fn register_dispatch(
            executor: &mut ::nros::Executor<'static>,
        ) -> ::core::result::Result<(), ()> {
            let state: <#node_ty as ::nros::ExecutableNode>::State =
                <#node_ty as ::nros::ExecutableNode>::init();
            let state_ptr =
                ::nros::__private_node_state_into_raw::<#node_ty>(state)
                    as *mut ::core::ffi::c_void;
            executor.register_dispatch_slot(state_ptr, #on_callback_fn_name)
        }

        /// Phase 258 (Track 2, 2a) — owned-spin entry registration. The
        /// codegen-emitted `run_plan(runtime)` body + `nros::main!`
        /// owned-spin loop call `<pkg>::register(runtime)?` once per Node.
        ///
        /// This now installs through the **uniform `install` seam** — the
        /// same `nros::install_node_typed` path the C / C++ typed entries
        /// use — via a raw `*mut Executor` the runtime sink exposes
        /// (`executor_handle()`), replacing the legacy opaque four-fn-ptr
        /// `register_dispatch_slot_dyn` bridge. The installed component
        /// enrolls in the executor's tick registry, so service-client /
        /// action poll nodes tick (phase-257 D2 closed for owned-spin too).
        ///
        /// References `::nros::install_node_typed` → `::nros::Executor<'static>`,
        /// gated on `rmw-cffi` inside `nros`; the Node pkg must depend on
        /// `nros` with `rmw-cffi` (every owned-spin example pkg does). A
        /// null handle (a sink with no live executor) maps to
        /// `RuntimeError::NodeRegister`.
        pub fn register(
            runtime: &mut ::nros::__macro_support::nros_platform::RuntimeCtx<'_>,
        ) -> ::core::result::Result<(), ::nros::__macro_support::nros_platform::RuntimeError> {
            let executor = runtime.runtime.executor_handle();
            // SAFETY: `executor` is the runtime sink's live `*mut Executor`
            // (or null, which `install_node_typed_with_launch` rejects as an error).
            // W4a — `runtime.params` carries the launch-baked `<param>` initials.
            // Phase 268 W1 — `runtime.node_identity` carries the launch `<node name= namespace=>`
            // baked by `nros::main!` so `ExecutorSink::create_node` uses the injected identity
            // instead of the `NodeOptions` default (RFC-0046). `None` → backward-compatible.
            // Phase 305 W3 (issue 0255) — `runtime.remaps` carries the launch `<remap>` rules;
            // `ExecutorSink::create_entity` applies them (plus `~`/relative expansion).
            // Issue #52 — `runtime.qos_overrides` carries the plan's per-topic QoS
            // overrides; `ExecutorSink::create_node` installs them on the node so
            // entity creation folds the matching ones in.
            match unsafe {
                ::nros::install_node_typed_with_launch_in(
                    executor,
                    &#component_store_name,
                    runtime.params,
                    runtime.node_identity,
                    runtime.remaps,
                    runtime.qos_overrides,
                )
            } {
                0 => ::core::result::Result::Ok(()),
                // Issue 0095 — `-2` is executor callback-table exhaustion; surface
                // the actionable `NROS_EXECUTOR_MAX_CBS` knob, not opaque NodeRegister.
                -2 => ::core::result::Result::Err(
                    ::nros::__macro_support::nros_platform::RuntimeError::ExecutorFull(#pkg_name_lit),
                ),
                _ => ::core::result::Result::Err(
                    ::nros::__macro_support::nros_platform::RuntimeError::NodeRegister(#pkg_name_lit),
                ),
            }
        }
    };

    TokenStream::from(expanded)
}

fn generate_unit_struct_impl(
    name: &syn::Ident,
    impl_generics: &syn::ImplGenerics,
    ty_generics: &syn::TypeGenerics,
    where_clause: Option<&syn::WhereClause>,
    type_name: &str,
    type_hash: &str,
) -> TokenStream {
    let expanded = quote! {
        impl #impl_generics nros_serdes::Serialize for #name #ty_generics #where_clause {
            fn serialize(&self, _writer: &mut nros_serdes::CdrWriter) -> Result<(), nros_serdes::SerError> {
                Ok(())
            }
        }

        impl #impl_generics nros_serdes::Deserialize for #name #ty_generics #where_clause {
            fn deserialize(_reader: &mut nros_serdes::CdrReader) -> Result<Self, nros_serdes::DeserError> {
                Ok(Self {})
            }
        }

        impl #impl_generics nros_core::RosMessage for #name #ty_generics #where_clause {
            const TYPE_NAME: &'static str = #type_name;
            const TYPE_HASH: &'static str = #type_hash;
        }
    };
    TokenStream::from(expanded)
}

/// Derive macro for ROS service types
///
/// # Attributes
///
/// - `#[ros(service_name = "...")]` - Full ROS service name (required)
/// - `#[ros(hash = "...")]` - RIHS service hash (required)
/// - `#[ros(request = "RequestType")]` - Request type name (required)
/// - `#[ros(reply = "ReplyType")]` - Reply type name (required)
///
/// # Example
///
/// ```ignore
/// use nros_macros::RosService;
///
/// #[derive(RosService)]
/// #[ros(service_name = "std_srvs::srv::dds_::Empty_")]
/// #[ros(hash = "abc123...")]
/// #[ros(request = "EmptyRequest")]
/// #[ros(reply = "EmptyReply")]
/// pub struct Empty;
/// ```
#[proc_macro_derive(RosService, attributes(ros))]
pub fn derive_ros_service(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;

    // Extract attributes
    let mut service_name: Option<String> = None;
    let mut service_hash: Option<String> = None;
    let mut request_type: Option<syn::Ident> = None;
    let mut reply_type: Option<syn::Ident> = None;

    for attr in &input.attrs {
        if attr.path().is_ident("ros") {
            let _ = attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("service_name") {
                    let value: LitStr = meta.value()?.parse()?;
                    service_name = Some(value.value());
                } else if meta.path.is_ident("hash") {
                    let value: LitStr = meta.value()?.parse()?;
                    service_hash = Some(value.value());
                } else if meta.path.is_ident("request") {
                    let value: LitStr = meta.value()?.parse()?;
                    request_type = Some(syn::Ident::new(
                        &value.value(),
                        proc_macro2::Span::call_site(),
                    ));
                } else if meta.path.is_ident("reply") {
                    let value: LitStr = meta.value()?.parse()?;
                    reply_type = Some(syn::Ident::new(
                        &value.value(),
                        proc_macro2::Span::call_site(),
                    ));
                }
                Ok(())
            });
        }
    }

    let service_name =
        service_name.unwrap_or_else(|| format!("{}::srv::dds_::{}_", "unknown", name));
    let service_hash = service_hash.unwrap_or_else(|| "0".repeat(64));

    let request_type = match request_type {
        Some(t) => t,
        None => {
            return syn::Error::new_spanned(
                &input,
                "RosService requires #[ros(request = \"...\")]",
            )
            .to_compile_error()
            .into();
        }
    };

    let reply_type = match reply_type {
        Some(t) => t,
        None => {
            return syn::Error::new_spanned(&input, "RosService requires #[ros(reply = \"...\")]")
                .to_compile_error()
                .into();
        }
    };

    let expanded = quote! {
        impl nros_core::RosService for #name {
            type Request = #request_type;
            type Reply = #reply_type;

            const SERVICE_NAME: &'static str = #service_name;
            const SERVICE_HASH: &'static str = #service_hash;
        }
    };

    TokenStream::from(expanded)
}

#[cfg(test)]
mod sanitize_tests {
    use super::sanitize_pkg_name_for_symbol;

    #[test]
    fn plain_pkg_name_is_passthrough() {
        assert_eq!(sanitize_pkg_name_for_symbol("talker_pkg"), "talker_pkg");
    }

    #[test]
    fn hyphens_become_underscores() {
        assert_eq!(sanitize_pkg_name_for_symbol("my-cool-pkg"), "my_cool_pkg");
    }

    #[test]
    fn mixed_specials_become_underscores() {
        assert_eq!(sanitize_pkg_name_for_symbol("a.b+c-d"), "a_b_c_d");
    }

    #[test]
    fn empty_is_empty() {
        assert_eq!(sanitize_pkg_name_for_symbol(""), "");
    }
}
