//! Platform initialization and `run()` entry point for ESP32-C3 QEMU.
//!
//! Uses `nros-smoltcp` for socket management and `openeth-smoltcp` for
//! Ethernet when the `ethernet` feature is enabled, or zenoh-pico's
//! built-in serial when the `serial` feature is enabled.

#[cfg(not(any(feature = "ethernet", feature = "serial")))]
compile_error!("Enable at least one transport: `ethernet` or `serial`");

// Phase 214.E.2 — at-most-one-transport guard.
#[cfg(all(feature = "ethernet", feature = "serial"))]
compile_error!("Pick exactly one transport: `ethernet` and `serial` are mutually exclusive");

use esp_hal::rng::Rng;

use nros_platform_esp32_qemu::random;

use crate::config::Config;

// NOTE: We intentionally do NOT import `type Result<T>` in this module.
// The `esp_println::println!` macro uses `?` internally which expands to
// `Result<(), core::fmt::Error>`. A `type Result<T>` alias here would shadow
// `core::result::Result` and cause "expected 1 generic argument but 2 supplied" errors.

fn network_identity_seed(config: &Config) -> u32 {
    let mut seed = 0x9e37_79b9u32;
    for byte in config.mac_addr.iter().chain(config.ip.iter()) {
        seed ^= u32::from(*byte);
        seed = seed.rotate_left(5).wrapping_mul(0x85eb_ca6b);
    }
    seed
}

// ---- Ethernet imports and static storage ----

#[cfg(feature = "ethernet")]
use core::mem::MaybeUninit;

#[cfg(feature = "ethernet")]
use nros_smoltcp::SmoltcpBridge;
#[cfg(feature = "ethernet")]
use openeth_smoltcp::OpenEth;
#[cfg(feature = "ethernet")]
use smoltcp::iface::{Interface, SocketSet};
#[cfg(feature = "ethernet")]
use smoltcp::wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address};

// Static storage for network objects (initialized by init_hardware, must
// outlive the function call so set_network_state pointers remain valid).
#[cfg(feature = "ethernet")]
static mut ETH_DEVICE: MaybeUninit<OpenEth> = MaybeUninit::uninit();
#[cfg(feature = "ethernet")]
static mut NET_IFACE: MaybeUninit<Interface> = MaybeUninit::uninit();
#[cfg(feature = "ethernet")]
static mut NET_SOCKETS: MaybeUninit<SocketSet<'static>> = MaybeUninit::uninit();

/// Helper to create a socket set with pre-allocated storage
///
/// # Safety
///
/// Must be called at most once during board init. `nros_smoltcp::get_socket_storage`
/// hands out an aliasable `&'static mut [SocketStorage<'static>]`; calling this
/// twice would produce two mutable references to the same backing storage.
#[cfg(feature = "ethernet")]
unsafe fn create_socket_set() -> SocketSet<'static> {
    let storage = unsafe { nros_smoltcp::get_socket_storage() };
    SocketSet::new(&mut storage[..])
}

// ---- Ethernet init ----

/// Initialize Ethernet transport via OpenETH + smoltcp.
#[cfg(feature = "ethernet")]
#[allow(static_mut_refs)]
fn init_ethernet(config: &Config) {
    // Initialize OpenETH driver
    esp_println::println!("Initializing OpenETH...");
    let openeth_config = openeth_smoltcp::Config {
        base_addr: openeth_smoltcp::ESP32C3_BASE,
        mac_addr: config.mac_addr,
    };
    // Construct the driver directly in static storage. OpenEth::init() writes
    // the addresses of its internal DMA buffers (tx_buf/rx_buf, which live
    // inside the struct) into hardware TX/RX descriptors — so it MUST be
    // called after the struct has reached its final address. Calling init()
    // before the move left the descriptors pointing at stale stack memory,
    // causing QEMU to transmit all-zero frames.
    //
    // Issue #64 — use `new_in_place` rather than `OpenEth::new(...)` +
    // `ETH_DEVICE.write(...)`: the by-value `new` materialises an ~11 KB
    // `OpenEth` (tx_buf + rx_bufs[4] + rx_frame) on the stack, which overflows
    // the esp32-c3 ~18 KB stack into `.bss` and silently corrupts whatever lives
    // there — it was wiping the esp-alloc heap metadata (Size 98304 → 0, then
    // `memory allocation of N bytes failed`) and clobbering the zenoh connect
    // locator (the 0xffffffff Load-access-fault). Constructing in place removes
    // the temporary entirely.
    let eth = unsafe {
        OpenEth::new_in_place(ETH_DEVICE.as_mut_ptr(), openeth_config);
        ETH_DEVICE.assume_init_mut()
    };
    eth.init();

    let mac = eth.mac_address();
    esp_println::println!(
        "  MAC: {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}",
        mac[0],
        mac[1],
        mac[2],
        mac[3],
        mac[4],
        mac[5]
    );

    // Create smoltcp interface
    esp_println::println!("");
    esp_println::println!("Creating network interface...");

    let mac_addr = EthernetAddress::from_bytes(&mac);
    let iface_config = smoltcp::iface::Config::new(mac_addr.into());
    let iface = Interface::new(
        iface_config,
        eth,
        smoltcp::time::Instant::from_millis(nros_platform_esp32_qemu::clock::clock_ms() as i64),
    );
    unsafe { NET_IFACE.write(iface) };
    let sockets = unsafe { create_socket_set() };
    unsafe { NET_SOCKETS.write(sockets) };

    // Configure static IP (no DHCP in QEMU)
    let iface = unsafe { NET_IFACE.assume_init_mut() };
    let ip_addr = Ipv4Address::new(config.ip[0], config.ip[1], config.ip[2], config.ip[3]);
    iface.update_ip_addrs(|addrs| {
        addrs
            .push(IpCidr::new(IpAddress::Ipv4(ip_addr), config.prefix))
            .ok();
    });

    let gw = Ipv4Address::new(
        config.gateway[0],
        config.gateway[1],
        config.gateway[2],
        config.gateway[3],
    );
    let _ = iface.routes_mut().add_default_ipv4_route(gw);

    esp_println::println!(
        "  IP: {}.{}.{}.{}/{}",
        config.ip[0],
        config.ip[1],
        config.ip[2],
        config.ip[3],
        config.prefix
    );
    esp_println::println!(
        "  Gateway: {}.{}.{}.{}",
        config.gateway[0],
        config.gateway[1],
        config.gateway[2],
        config.gateway[3]
    );

    // Initialize transport bridge
    SmoltcpBridge::init().expect("SmoltcpBridge::init double-call");

    // Create and register TCP + UDP sockets via transport crate
    let sockets = unsafe { NET_SOCKETS.assume_init_mut() };
    unsafe {
        nros_smoltcp::create_and_register_sockets(sockets);
        nros_smoltcp::create_and_register_udp_sockets(sockets);
    }

    // Store global state for poll callback
    let eth = unsafe { ETH_DEVICE.assume_init_mut() };
    unsafe {
        crate::network::set_network_state(
            iface as *mut Interface,
            sockets as *mut SocketSet<'static>,
            eth as *mut OpenEth as *mut (),
        );

        nros_smoltcp::set_poll_callback(crate::network::smoltcp_network_poll);

        // Register the network poll as the sleep callback so busy-wait
        // sleep polls the network stack to avoid missing packets during
        // zenoh-pico's connect handshake.
        nros_platform_esp32_qemu::sleep::set_poll_callback(crate::network::smoltcp_network_poll);
    }

    esp_println::println!(
        "  smoltcp poll callback registered: {}",
        nros_smoltcp::has_poll_callback()
    );
    esp_println::println!("Ethernet ready.");
}

// ---- Serial init ----

/// Initialize serial transport.
///
/// ESP32-C3 QEMU uses zenoh-pico's built-in serial support — no additional
/// driver crates are needed. The zenoh locator string (e.g.,
/// `serial/UART_0#baudrate=115200`) tells zenoh-pico which UART to use.
#[cfg(feature = "serial")]
fn init_serial(config: &Config) {
    esp_println::println!("Initializing serial transport...");
    esp_println::println!("  Baud: {}", config.baudrate);
    esp_println::println!("  Locator: {}", config.zenoh_locator);
    esp_println::println!("Serial ready.");
}

// ---- Main init + run ----

/// Initialize all ESP32-C3 QEMU hardware and the transport stack.
///
/// Sets up ESP32 peripherals, heap allocator, RNG, and the selected
/// transport (Ethernet and/or serial depending on enabled features).
/// After calling this, you can create an `Executor` and start using nano-ros.
///
/// This is automatically called by [`run()`]. Call it directly only when
/// using an alternative execution model (e.g., RTIC) that needs hardware
/// initialized before returning control to the framework.
///
/// # Panics
///
/// Panics if hardware initialization fails. Must be called exactly once
/// before any nros operations.
pub fn init_hardware(config: &Config) {
    esp_println::println!("");
    esp_println::println!("========================================");
    esp_println::println!("  nros ESP32-C3 QEMU Platform");
    esp_println::println!("========================================");
    esp_println::println!("");

    // Step 1: Initialize ESP32 peripherals
    esp_println::println!("Initializing ESP32-C3...");
    let _peripherals = esp_hal::init(esp_hal::Config::default());

    // Step 2: Set up heap allocator. For zenoh / non-DDS builds, esp-alloc
    // carves the Rust global heap out of DRAM at runtime. Keep this SMALL:
    // `link.x`'s `.stack` region fills DRAM from end-of-`.bss` up to the top,
    // so every KB given to this `.bss` heap array is a KB stolen from the
    // stack. zenoh-pico runs its own 32 KB `FreeListHeap` for `z_malloc`, so
    // the Rust heap barely sees use (≈20 B live after `Executor::open`); the
    // nros + zenoh-pico spin/poll path, by contrast, drives a deep call stack.
    // #190 — 48 KB, and the number is a two-sided constraint, not a floor:
    //
    // * TOO SMALL (the #64-era 16 KB): the phase-271 executor backing is a
    //   large heap allocation — 16 KB dies at open ("memory allocation of
    //   17032 bytes failed" right after "Ethernet ready."), the #184 class.
    // * TOO BIG (96/128 KB): `.stack` is the LINKER LEFTOVER after `.bss`
    //   (link.x fills DRAM up to 0x3fcce400), so every KB given to this
    //   array is a KB taken from the stack. At 96 KB the stack shrank to
    //   ~18 KB while the zenoh-pico handshake + nested smoltcp poll path
    //   needs far more (#64 measured ≈98 KB deep) — the overflow wrote
    //   frames straight down into `.bss` (`__stack_chk_guard` and the log
    //   statics sit first), producing the #190 phantom corruptions: InitAck
    //   cookies full of DRAM pointers + `Z_TRANSPORT_LEASE`, wild jumps
    //   (mepc=0x9ae65930), the pre-#190 0xffffffff config-pointer fault.
    //   None of them were allocator or zenoh-pico bugs.
    //
    // 48 KB fits the executor arena AND leaves a ~67 KB stack; both esp32
    // pair-delivery directions run green on it. Check `.stack` in
    // `readelf -S` after changing ANY large static — there is no runtime
    // stack-overflow guard on this target.
    // For DDS builds the example crate enables `nros-platform/global-allocator`,
    // which registers a 256 KB static `FreeListHeap` instead — calling
    // `esp_alloc::heap_allocator!` on top of that produces the
    // "the `#[global_allocator]` in nros_platform conflicts with
    // global allocator in: esp_alloc" link error (Phase 101.7).
    #[cfg(not(feature = "dds-heap"))]
    esp_alloc::heap_allocator!(size: 48 * 1024);

    // Step 3: Register the monotonic clock with the shared busy-wait sleep
    // loop in `nros-baremetal-common`. Without this, `sleep_ms` silently
    // no-ops and zenoh-pico's connect handshake polls the network zero
    // times → Transport(ConnectionFailed).
    nros_platform_esp32_qemu::sleep::init_clock();

    // Step 4: Initialize hardware RNG (for zenoh-pico session ID)
    let rng = Rng::new();
    let rng_seed = rng.random() ^ network_identity_seed(config);
    random::seed(rng_seed);
    #[cfg(feature = "ethernet")]
    nros_smoltcp::seed_ephemeral_port((rng_seed as u16) ^ u16::from(config.mac_addr[5]));

    // Step 4: Initialize selected transport(s)
    #[cfg(feature = "ethernet")]
    init_ethernet(config);

    #[cfg(feature = "serial")]
    init_serial(config);

    esp_println::println!("");
}

/// Phase 313 W-direct-exec (#0243) — lightweight NO-SESSION direct-exec entry
/// for logging / init-only fixtures. Runs the Config-driven ESP32-C3 bringup
/// (`init_hardware` + esp-println log writer) then a NULLARY closure WITHOUT
/// opening an `Executor` session, then spins forever (ESP32 has no host exit;
/// the harness kills QEMU once it sees the output). The new-family replacement
/// for the retired legacy `run(Config, closure)` (which routed through
/// `nros_board_common::run` + the `nros_board_common::board_init` traits).
///
/// This is a free fn — NOT a method on the `rmw-zenoh`-gated
/// [`crate::Esp32QemuEntry`] — so the no-RMW logging smoke can reach it.
/// Firmware that opens a session uses `nros::main!` → `Esp32QemuEntry` instead.
pub fn run_bare<F, E: core::fmt::Debug>(config: Config, setup: F) -> !
where
    F: FnOnce() -> core::result::Result<(), E>,
{
    // issue 0708 — publish the nros_log sink list at the boot funnel. A record
    // raised inside a LIBRARY (the zenoh session-pool diagnostic of issue 0589,
    // for one) is dropped until a sink list exists, and its author cannot know
    // whether the board published one.
    //
    // issue 1048 — the publication moved INTO `register_log_writer`, one line
    // after the fn-pointer writer it feeds, and this funnel no longer publishes
    // ahead of it. Order is load-bearing and it used to be backwards here:
    // `init` DRAINS the early ring through the sinks it installs, so installing
    // `PlatformSink` before `nros_platform_esp32_qemu::register_log_writer` sent
    // every record held during `init_hardware` to a null writer — the one place
    // the early ring exists to serve. Both funnels now reach the pair together.
    init_hardware(&config);
    register_log_writer();
    match setup() {
        Ok(()) => esp_println::println!("nros: application complete"),
        Err(e) => esp_println::println!("nros: application error: {e:?}"),
    }
    // ESP32 has no process exit — spin forever (the harness kills QEMU once it
    // sees the completion / severity lines).
    #[allow(clippy::empty_loop)]
    loop {
        core::hint::spin_loop();
    }
}

/// Phase 88.15.f — register an `esp_println`-backed writer with
/// `nros-platform-esp32-qemu`'s log fn-ptr slot. Called once from
/// [`run()`] right after `init_hardware`. Mirrors the wifi board's
/// shape from Phase 88.16.E.
///
/// `pub(crate)` so the Phase 225.O `BoardEntry::run` shim in
/// `board_entry.rs` can route nros log records to the esp-println
/// console (node-registration / executor diagnostics).
pub(crate) fn register_log_writer() {
    fn writer(severity: u8, name: &[u8], message: &[u8]) {
        let label = match severity {
            0 => "TRACE",
            1 => "DEBUG",
            2 => "INFO",
            3 => "WARN",
            4 => "ERROR",
            5 => "FATAL",
            _ => "?",
        };
        let name_str = core::str::from_utf8(name).unwrap_or("");
        let msg_str = core::str::from_utf8(message).unwrap_or("");
        if !name_str.is_empty() {
            esp_println::println!("[{}] {}: {}", label, name_str, msg_str);
        } else {
            esp_println::println!("[{}] {}", label, msg_str);
        }
    }
    nros_platform_esp32_qemu::register_log_writer(Some(writer));

    // Issue #64 — bridge the `log` crate facade too, so a node body written
    // against `log::info!` (and nros's own framework `::log::info!`) reaches the
    // same console. esp-println's logger writes straight to it.
    //
    // Issue 1048 — and the CFG IS THE POINT, not decoration. `log::set_logger`
    // and `log::set_max_level` are both `#[cfg(target_has_atomic = "ptr")]` in
    // `log` 0.4, and this board's target is `riscv32imc-unknown-none-elf` — RV32
    // I M C, no `A` extension, so `rustc --print cfg` lists no
    // `target_has_atomic="ptr"` and NEITHER FUNCTION EXISTS. `init_logger` was
    // therefore a call that compiled, ran, and installed nothing: no logger can
    // be installed on this board, by anyone, and every `log::*` record was
    // dropped. Writing the condition out means the dead call is dead in the
    // SOURCE rather than only in the binary, and any future board on a
    // non-atomic target (thumbv6m is the other one rustc reports here) gets the
    // same statement instead of the same silence.
    #[cfg(target_has_atomic = "ptr")]
    esp_println::logger::init_logger(log::LevelFilter::Info);

    // Issue 1048 — publish the `nros_log` sink list, which is what actually
    // delivers on this board, and publish it at EVERY funnel: `run_bare` did
    // this and `BoardEntry::run` did not, so a `nros::main!` image reached the
    // application with no sinks while a fixture image printed fine.
    //
    // Without it `nros_log` is no better off than `log` was — `dispatch_to_sinks`
    // finds a null sink list and HOLDS each record in the early ring for a board
    // that never speaks. `init_default` installs `PlatformSink`, which routes to
    // `nros_platform_log_write`, the fn-pointer writer registered immediately
    // above: no atomics, and the same console. Same spelling every other board
    // uses (`nros-board-{linux,mps2-an385,freertos,threadx,nuttx,zephyr}`) —
    // naming the sink by hand here was a second spelling of one idea.
    // Idempotent: re-calling swaps the list pointer for the same default.
    ::nros_platform_cffi::log::init_default();
}
