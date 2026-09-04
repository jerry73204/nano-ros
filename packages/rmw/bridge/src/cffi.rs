//! C FFI surface for the bridge (Phase 128.F.5).
//!
//! Mirrors the public types/headers in `packages/api/nros-c/include/nros/bridge.h`
//! and `packages/api/nros-cpp/include/nros/bridge.hpp`. The C symbols
//! are emitted with `#[unsafe(no_mangle)] pub extern "C"` so a binary
//! that links `libnros_bridge.a` picks them up at static-link time.
//!
//! The Rust side of the bridge already lives in [`crate`]; this
//! module is purely an ABI adapter.

extern crate alloc;

use alloc::{boxed::Box, string::String, vec::Vec};
use core::ffi::{CStr, c_char};

use nros_node::executor::{Executor, SessionSpec};
use nros_rmw_cffi::{
    NROS_RMW_RET_ERROR, NROS_RMW_RET_INVALID_ARGUMENT, NROS_RMW_RET_NO_BACKEND, NROS_RMW_RET_OK,
    NrosRmwRet,
};

use crate::PubSubBridge;

/// C-side spec mirroring `nros_node::executor::SessionSpec`. Fields
/// must stay in lockstep with `nros_session_spec_t` in
/// `<nros/bridge.h>`.
#[repr(C)]
pub struct NrosSessionSpec {
    pub rmw: *const c_char,
    pub locator: *const c_char,
    pub domain_id: u32,
    pub node_name: *const c_char,
    pub namespace_: *const c_char,
}

/// Owned handle returned by [`nros_init_multi`]. Trait-object façade
/// wraps the executor + any bridges built against it; the binary
/// hands the raw pointer back to [`nros_fini_multi`] for cleanup.
struct ExecutorBox {
    executor: Executor<'static>,
    /// Persistent storage for the C-string copies the executor still
    /// references (`SessionSpec` holds `&'cfg str` borrows). Kept
    /// alive for the executor's lifetime; freed by `nros_fini_multi`.
    _spec_strings: Vec<String>,
}

#[inline]
unsafe fn cstr_or_empty(p: *const c_char) -> &'static str {
    if p.is_null() {
        ""
    } else {
        // SAFETY: caller is responsible for keeping the pointer live
        // for the lifetime of the returned slice. We store every
        // string in `_spec_strings` before invoking `open_multi`, so
        // the 'static lifetime here is a controlled lie that the
        // surrounding ExecutorBox upholds.
        match unsafe { CStr::from_ptr(p) }.to_str() {
            Ok(s) => unsafe { core::mem::transmute::<&str, &'static str>(s) },
            Err(_) => "",
        }
    }
}

/// Open an executor against the given specs. See `<nros/bridge.h>`
/// for the documented contract.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_init_multi(
    specs: *const NrosSessionSpec,
    specs_len: usize,
    out: *mut *mut core::ffi::c_void,
) -> NrosRmwRet {
    if out.is_null() {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    unsafe { *out = core::ptr::null_mut() };
    if specs.is_null() || specs_len == 0 {
        return NROS_RMW_RET_NO_BACKEND;
    }

    // Issue 0436 — register the linked backends, as `nros_cpp_init` does. This
    // entry point did NOT, so a caller that used the multi path INSTEAD of
    // `nros_*_init` (which is the whole point of it) met an empty registry and
    // every spec's lookup missed. `nros_app_register_backends` is the generated
    // strong def (weak no-op in nros-c when a board supplies none) and is
    // idempotent, so calling it here costs nothing and removes the asymmetry.
    unsafe extern "C" {
        fn nros_app_register_backends();
    }
    unsafe {
        nros_app_register_backends();
    }

    // Snapshot every C string into owned `String`s so the executor's
    // borrowed views stay valid past this call.
    let mut owned: Vec<String> = Vec::with_capacity(specs_len * 4);
    let mut rust_specs: Vec<SessionSpec<'static>> = Vec::with_capacity(specs_len);

    for i in 0..specs_len {
        let spec = unsafe { &*specs.add(i) };
        let rmw = unsafe { cstr_or_empty(spec.rmw) };
        let locator = unsafe { cstr_or_empty(spec.locator) };
        let node_name = unsafe { cstr_or_empty(spec.node_name) };
        let namespace_ = unsafe { cstr_or_empty(spec.namespace_) };

        // Heap-stash each str so SessionSpec's `'static` borrows
        // remain valid for the executor's lifetime (we hand the
        // owned Vec to ExecutorBox below).
        let r = String::from(rmw);
        let l = String::from(locator);
        let n = String::from(node_name);
        let ns = String::from(namespace_);
        owned.push(r);
        owned.push(l);
        owned.push(n);
        owned.push(ns);

        // SAFETY: the slices live in `owned` which moves into
        // ExecutorBox right after `open_multi` returns. Pointers
        // captured here are valid for the whole ExecutorBox lifetime.
        let len = owned.len();
        let r_ref: &'static str =
            unsafe { core::mem::transmute::<&str, &'static str>(owned[len - 4].as_str()) };
        let l_ref: &'static str =
            unsafe { core::mem::transmute::<&str, &'static str>(owned[len - 3].as_str()) };
        let n_ref: &'static str =
            unsafe { core::mem::transmute::<&str, &'static str>(owned[len - 2].as_str()) };
        let ns_ref: &'static str =
            unsafe { core::mem::transmute::<&str, &'static str>(owned[len - 1].as_str()) };

        rust_specs.push(
            SessionSpec::new(r_ref, l_ref)
                .domain_id(spec.domain_id)
                .node_name(n_ref)
                .namespace(if ns_ref.is_empty() { "/" } else { ns_ref }),
        );
    }

    match Executor::open_multi(&rust_specs) {
        Ok(executor) => {
            let boxed = Box::new(ExecutorBox {
                executor,
                _spec_strings: owned,
            });
            unsafe { *out = Box::into_raw(boxed) as *mut core::ffi::c_void };
            NROS_RMW_RET_OK
        }
        // Issue 0436 — this discarded the only description of WHY the open failed,
        // so a C/C++ caller saw a bare -1 for "backend not registered", "that RMW
        // name is not linked in", "the transport refused" and everything else.
        // Name it on the error path. Through `nros_log`, not `std::eprintln!`:
        // issue 0589 — std stdio SIGSEGVs a Zephyr native_sim image, and it
        // reaches `no_std` targets, which the old `cfg(feature = "std")` arm
        // never did.
        Err(e) => {
            nros_log::log_error!(
                nros_log::get_logger("nros_rmw_bridge"),
                "nros_init_multi failed — {e:?}"
            );
            NROS_RMW_RET_ERROR
        }
    }
}

/// Tear down a `nros_init_multi`-opened executor. Idempotent on NULL.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_fini_multi(exec: *mut core::ffi::c_void) {
    if exec.is_null() {
        return;
    }
    // SAFETY: pointer originated from `Box::into_raw` in `init_multi`.
    let _ = unsafe { Box::from_raw(exec as *mut ExecutorBox) };
}

/// Wraps a `PubSubBridge<RX, TX>` plus owned C-string copies in a
/// trait object so the C handle survives the move into Box.
struct BridgeBox {
    bridge: Box<dyn PumpableBridge>,
}

trait PumpableBridge {
    fn pump(&mut self) -> usize;
    fn pump_with_stats(&mut self) -> crate::PumpStats;
}

impl<const RX: usize, const TX: usize, const CV: usize> PumpableBridge
    for PubSubBridge<RX, TX, CV>
{
    fn pump(&mut self) -> usize {
        PubSubBridge::pump(self).unwrap_or(0)
    }
    fn pump_with_stats(&mut self) -> crate::PumpStats {
        PubSubBridge::pump_with_stats(self).unwrap_or_default()
    }
}

#[repr(C)]
pub struct NrosPumpStats {
    pub forwarded: usize,
    pub dropped_echo: usize,
}

/// One side (source or destination) of a pubsub bridge: the Node to
/// open, the RMW backend to route through, and the topic. See
/// `<nros/bridge.h>` for the documented contract.
#[repr(C)]
pub struct NrosBridgeEndpoint {
    pub node: *const c_char,
    pub rmw: *const c_char,
    pub topic: *const c_char,
}

/// Create a raw pubsub bridge. See `<nros/bridge.h>` for the
/// documented contract.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_pubsub_bridge_create(
    exec: *mut core::ffi::c_void,
    src: *const NrosBridgeEndpoint,
    dst: *const NrosBridgeEndpoint,
    type_name: *const c_char,
    type_hash: *const c_char,
    origin: *const c_char,
    out: *mut *mut core::ffi::c_void,
) -> NrosRmwRet {
    if exec.is_null() || src.is_null() || dst.is_null() || out.is_null() {
        return NROS_RMW_RET_INVALID_ARGUMENT;
    }
    unsafe { *out = core::ptr::null_mut() };

    let eb = unsafe { &mut *(exec as *mut ExecutorBox) };
    let src = unsafe { &*src };
    let dst = unsafe { &*dst };
    let src_node = unsafe { cstr_or_empty(src.node) };
    let src_rmw = unsafe { cstr_or_empty(src.rmw) };
    let src_topic = unsafe { cstr_or_empty(src.topic) };
    let dst_node = unsafe { cstr_or_empty(dst.node) };
    let dst_rmw = unsafe { cstr_or_empty(dst.rmw) };
    let dst_topic = unsafe { cstr_or_empty(dst.topic) };
    let type_name = unsafe { cstr_or_empty(type_name) };
    let type_hash = unsafe { cstr_or_empty(type_hash) };
    let origin = unsafe { cstr_or_empty(origin) };

    // Source side.
    // phase-412 W2 -- NAME the knob. A bridge creates TWO nodes whose names are
    // RUNTIME strings, so they are the one node demand no declaration carries
    // and the one case NROS_EXECUTOR_MAX_NODES can be derived short for. Every
    // other consumer fails with `NodeError::NodeTableFull`, which names the
    // knob; this path used to map it to a bare NROS_RMW_RET_ERROR and lose it.
    // Deriving the knob is only safe because the shortfall is legible here.
    let mut src = match eb.executor.create_node_on(src_node, src_rmw) {
        Ok(n) => n,
        Err(nros_node::executor::NodeError::NodeTableFull) => {
            nros_log::log_error!(
                nros_log::get_logger("nros_rmw_bridge"),
                "nros_pubsub_bridge_create: node table FULL creating the source \
                 node -- raise NROS_EXECUTOR_MAX_NODES. A bridge creates TWO \
                 nodes and declares neither, so a DERIVED value does not count \
                 them (phase-412 W2)."
            );
            return NROS_RMW_RET_ERROR;
        }
        Err(_) => return NROS_RMW_RET_ERROR,
    };
    let sub = match src.create_subscription_raw(src_topic, type_name, type_hash) {
        Ok(s) => s,
        Err(_) => return NROS_RMW_RET_ERROR,
    };
    drop(src);

    // Destination side.
    let mut dst = match eb.executor.create_node_on(dst_node, dst_rmw) {
        Ok(n) => n,
        Err(nros_node::executor::NodeError::NodeTableFull) => {
            nros_log::log_error!(
                nros_log::get_logger("nros_rmw_bridge"),
                "nros_pubsub_bridge_create: node table FULL creating the \
                 destination node -- raise NROS_EXECUTOR_MAX_NODES (phase-412 W2)."
            );
            return NROS_RMW_RET_ERROR;
        }
        Err(_) => return NROS_RMW_RET_ERROR,
    };
    let pubr = match dst.create_publisher_raw(dst_topic, type_name, type_hash) {
        Ok(p) => p,
        Err(_) => return NROS_RMW_RET_ERROR,
    };
    drop(dst);

    let origin_static: &'static str = if origin.is_empty() {
        ""
    } else {
        // Leak — bridges are one-shot per process; the leak is
        // O(bridges) and bounded by application config.
        Box::leak(String::from(origin).into_boxed_str())
    };

    // RFC-0088 D3 / phase-421 W3 — a two-backend image is the one place where
    // the two sides can disagree about what the forwarded bytes mean. Refuse
    // here, naming both formats: the C caller only gets a return code, so the
    // log line is the whole diagnosis.
    let inner = match PubSubBridge::new(sub, pubr, origin_static) {
        Ok(b) => b,
        Err(e) => {
            nros_log::log_error!(
                nros_log::get_logger("nros_rmw_bridge"),
                "nros_pubsub_bridge_create: {}",
                e
            );
            return NROS_RMW_RET_ERROR;
        }
    };
    let bridge = Box::new(BridgeBox {
        bridge: Box::new(inner),
    });
    unsafe { *out = Box::into_raw(bridge) as *mut core::ffi::c_void };
    NROS_RMW_RET_OK
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_pubsub_bridge_pump(bridge: *mut core::ffi::c_void) -> usize {
    if bridge.is_null() {
        return 0;
    }
    let bb = unsafe { &mut *(bridge as *mut BridgeBox) };
    bb.bridge.pump()
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_pubsub_bridge_pump_with_stats(
    bridge: *mut core::ffi::c_void,
) -> NrosPumpStats {
    if bridge.is_null() {
        return NrosPumpStats {
            forwarded: 0,
            dropped_echo: 0,
        };
    }
    let bb = unsafe { &mut *(bridge as *mut BridgeBox) };
    let s = bb.bridge.pump_with_stats();
    NrosPumpStats {
        forwarded: s.forwarded,
        dropped_echo: s.dropped_echo,
    }
}

#[unsafe(no_mangle)]
pub unsafe extern "C" fn nros_pubsub_bridge_destroy(bridge: *mut core::ffi::c_void) {
    if bridge.is_null() {
        return;
    }
    let _ = unsafe { Box::from_raw(bridge as *mut BridgeBox) };
}
