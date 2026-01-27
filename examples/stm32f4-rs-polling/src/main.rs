//! Bare-metal Polling Example for nano-ros on STM32F4 with zenoh-pico-shim
//!
//! This example demonstrates how to use nano-ros without any async runtime
//! (no RTIC, no Embassy). It uses a simple polling loop that's suitable for:
//!
//! - Very simple systems with minimal overhead
//! - Systems where you need full control over timing
//! - Learning/prototyping before adopting an executor
//!
//! # Architecture
//!
//! The main loop manually:
//! 1. Polls smoltcp for network activity
//! 2. Bridges smoltcp sockets to zenoh-pico platform buffers
//! 3. Polls zenoh for incoming messages
//! 4. Publishes data at a fixed rate
//!
//! # Hardware
//!
//! - Board: NUCLEO-F429ZI (or similar STM32F4 with Ethernet)
//! - Connect Ethernet cable to the board's RJ45 port
//!
//! # Network Configuration
//!
//! Default (static IP):
//! - Device IP: 192.168.1.10
//! - Gateway: 192.168.1.1
//! - Zenoh Router: 192.168.1.1:7447
//!
//! # Building
//!
//! ```bash
//! cargo build --release
//! # Flash with probe-rs
//! cargo run --release
//! ```
//!
//! # Note
//!
//! This example requires the zenoh-pico C library to be cross-compiled for
//! ARM Cortex-M. The current implementation shows the correct integration
//! pattern but zenoh operations are stubbed until cross-compilation is set up.

#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt::{debug, error, info, warn};
use defmt_rtt as _;
use panic_probe as _;

use stm32f4xx_hal::{gpio::GpioExt, prelude::*, rcc::RccExt};

use stm32_eth::{
    dma::{RxRingEntry, TxRingEntry},
    EthPins, Parts, PartsIn,
};

use smoltcp::{
    iface::{Config, Interface, SocketSet},
    socket::tcp::{Socket as TcpSocket, SocketBuffer as TcpSocketBuffer},
    time::Instant,
    wire::{EthernetAddress, IpAddress, IpCidr, Ipv4Address},
};

// Import zenoh-pico-shim (with smoltcp platform)
use zenoh_pico_shim::platform_smoltcp;

// ============================================================================
// Network Configuration
// ============================================================================

/// Device MAC address (locally administered)
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

/// Device IP address (static)
const IP_ADDRESS: [u8; 4] = [192, 168, 1, 10];

/// Network gateway
const GATEWAY: Ipv4Address = Ipv4Address::new(192, 168, 1, 1);

/// Zenoh router address
#[allow(dead_code)] // Used when zenoh-pico is available
const ZENOH_ROUTER: &[u8] = b"tcp/192.168.1.1:7447\0";

// ============================================================================
// Static Buffer Allocation
// ============================================================================

/// Number of RX DMA descriptors
const RX_DESC_COUNT: usize = 4;

/// Number of TX DMA descriptors
const TX_DESC_COUNT: usize = 4;

/// TCP socket RX buffer size
const TCP_RX_BUFFER_SIZE: usize = 2048;

/// TCP socket TX buffer size
const TCP_TX_BUFFER_SIZE: usize = 2048;

/// Maximum number of sockets
const MAX_SOCKETS: usize = 2;

/// Poll interval in milliseconds
const POLL_INTERVAL_MS: u32 = 10;

/// Publish interval in milliseconds
const PUBLISH_INTERVAL_MS: u32 = 1000;

// Ethernet DMA descriptors - must be in normal RAM (not CCM)
#[link_section = ".ethram"]
static mut RX_RING: [RxRingEntry; RX_DESC_COUNT] = [RxRingEntry::INIT; RX_DESC_COUNT];
#[link_section = ".ethram"]
static mut TX_RING: [TxRingEntry; TX_DESC_COUNT] = [TxRingEntry::INIT; TX_DESC_COUNT];

// TCP socket buffers for zenoh
static mut TCP_RX_BUFFER: [u8; TCP_RX_BUFFER_SIZE] = [0u8; TCP_RX_BUFFER_SIZE];
static mut TCP_TX_BUFFER: [u8; TCP_TX_BUFFER_SIZE] = [0u8; TCP_TX_BUFFER_SIZE];

// Socket storage
static mut SOCKET_STORAGE: [smoltcp::iface::SocketStorage<'static>; MAX_SOCKETS] =
    [smoltcp::iface::SocketStorage::EMPTY; MAX_SOCKETS];

// ============================================================================
// Platform Callback for smoltcp Integration
// ============================================================================

/// Network poll callback (no-op - polling is done in main loop)
unsafe extern "C" fn network_poll_callback() {
    // Polling is done in the main loop where we have access to all state
}

// ============================================================================
// Simple Timer
// ============================================================================

/// Simple timing tracker using DWT cycle counter
struct SimpleTimer {
    last_tick: u32,
    ticks_per_ms: u32,
}

impl SimpleTimer {
    fn new(sysclk_hz: u32) -> Self {
        Self {
            last_tick: cortex_m::peripheral::DWT::cycle_count(),
            ticks_per_ms: sysclk_hz / 1000,
        }
    }

    /// Get current tick count (wraps around)
    fn now(&self) -> u32 {
        cortex_m::peripheral::DWT::cycle_count()
    }

    /// Check if the given number of milliseconds have elapsed since last check
    fn elapsed_ms(&mut self, ms: u32) -> bool {
        let now = self.now();
        let elapsed_ticks = now.wrapping_sub(self.last_tick);
        let required_ticks = ms * self.ticks_per_ms;

        if elapsed_ticks >= required_ticks {
            self.last_tick = now;
            true
        } else {
            false
        }
    }

    /// Get elapsed milliseconds since start
    fn elapsed_ms_total(&self) -> u64 {
        let now = self.now();
        (now as u64 * 1000) / (self.ticks_per_ms as u64)
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

#[entry]
fn main() -> ! {
    info!("nano-ros polling + zenoh-pico-shim example starting...");

    // Get access to device peripherals
    let dp = stm32f4xx_hal::pac::Peripherals::take().unwrap();
    let cp = cortex_m::Peripherals::take().unwrap();

    // Configure clocks using stm32f4xx-hal 0.21 builder API
    let rcc = dp.RCC.constrain();
    let clocks = rcc
        .cfgr
        .use_hse(8.MHz()) // NUCLEO-F429ZI has 8 MHz HSE
        .sysclk(168.MHz())
        .hclk(168.MHz())
        .pclk1(42.MHz())
        .pclk2(84.MHz())
        .freeze();

    let sysclk_hz = clocks.sysclk().to_Hz();
    info!("Clocks configured: sysclk = {} Hz", sysclk_hz);

    // Enable DWT cycle counter for timing
    let mut dcb = cp.DCB;
    let mut dwt = cp.DWT;
    dcb.enable_trace();
    dwt.enable_cycle_counter();

    // ═══════════════════════════════════════════════════════════════════════
    // Initialize Ethernet
    // ═══════════════════════════════════════════════════════════════════════

    // Configure GPIO for Ethernet (RMII mode)
    let gpioa = dp.GPIOA.split();
    let gpiob = dp.GPIOB.split();
    let gpioc = dp.GPIOC.split();
    let gpiog = dp.GPIOG.split();

    // RMII pins for NUCLEO-F429ZI
    let ref_clk = gpioa.pa1.into_floating_input();
    let crs = gpioa.pa7.into_floating_input();
    let tx_en = gpiog.pg11.into_floating_input();
    let tx_d0 = gpiog.pg13.into_floating_input();
    let tx_d1 = gpiob.pb13.into_floating_input();
    let rx_d0 = gpioc.pc4.into_floating_input();
    let rx_d1 = gpioc.pc5.into_floating_input();

    let eth_pins = EthPins {
        ref_clk,
        crs,
        tx_en,
        tx_d0,
        tx_d1,
        rx_d0,
        rx_d1,
    };

    // MDC/MDIO for PHY management
    let mdio = gpioa.pa2.into_alternate();
    let mdc = gpioc.pc1.into_alternate();

    // LED for status indication
    let mut led = gpiob.pb7.into_push_pull_output();

    info!("Initializing Ethernet...");

    let eth_parts_in = PartsIn {
        dma: dp.ETHERNET_DMA,
        mac: dp.ETHERNET_MAC,
        mmc: dp.ETHERNET_MMC,
        ptp: dp.ETHERNET_PTP,
    };

    // Safety: These static mutable references are only created once during init
    // and the DMA hardware requires them to remain valid for the lifetime of the program.
    #[allow(static_mut_refs)]
    let Parts { mut dma, .. } = unsafe {
        stm32_eth::new_with_mii(
            eth_parts_in,
            &mut RX_RING,
            &mut TX_RING,
            clocks,
            eth_pins,
            mdio,
            mdc,
        )
        .expect("Failed to initialize Ethernet")
    };

    // ═══════════════════════════════════════════════════════════════════════
    // Initialize smoltcp
    // ═══════════════════════════════════════════════════════════════════════

    info!("Creating smoltcp interface...");

    let mac_addr = EthernetAddress::from_bytes(&MAC_ADDRESS);
    let config = Config::new(mac_addr.into());

    let mut dma_ref = &mut dma;
    let mut iface = Interface::new(config, &mut dma_ref, Instant::from_millis(0));

    // Set IP address
    iface.update_ip_addrs(|addrs| {
        addrs
            .push(IpCidr::new(
                IpAddress::v4(IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]),
                24,
            ))
            .ok();
    });

    // Set default gateway
    iface
        .routes_mut()
        .add_default_ipv4_route(GATEWAY)
        .expect("Failed to add default route");

    info!(
        "IP address: {}.{}.{}.{}",
        IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]
    );

    // Create socket set
    let mut sockets = unsafe { SocketSet::new(&mut SOCKET_STORAGE[..]) };

    // Create TCP socket for zenoh connection
    let tcp_rx_buffer = unsafe { TcpSocketBuffer::new(&mut TCP_RX_BUFFER[..]) };
    let tcp_tx_buffer = unsafe { TcpSocketBuffer::new(&mut TCP_TX_BUFFER[..]) };
    let tcp_socket = TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);

    let tcp_handle = sockets.add(tcp_socket);

    // ═══════════════════════════════════════════════════════════════════════
    // Initialize zenoh-pico-shim platform
    // ═══════════════════════════════════════════════════════════════════════

    info!("Initializing zenoh-pico platform...");

    // Set the platform clock (will be updated in main loop)
    platform_smoltcp::smoltcp_set_clock_ms(0);

    // Set the network poll callback
    platform_smoltcp::smoltcp_set_poll_callback(Some(network_poll_callback));

    // Initialize the platform
    let ret = platform_smoltcp::smoltcp_init();
    if ret < 0 {
        error!("Failed to initialize smoltcp platform: {}", ret);
    } else {
        info!("smoltcp platform initialized");
    }

    // When zenoh-pico is available:
    // use zenoh_pico_shim::ShimContext;
    // let ctx = ShimContext::new(ZENOH_ROUTER).expect("Failed to connect");
    // let publisher = ctx.declare_publisher(b"nano_ros/polling/counter\0").unwrap();

    // ═══════════════════════════════════════════════════════════════════════
    // Main Polling Loop
    // ═══════════════════════════════════════════════════════════════════════

    info!("Entering main polling loop...");

    let mut poll_timer = SimpleTimer::new(sysclk_hz);
    let mut publish_timer = SimpleTimer::new(sysclk_hz);
    let clock_start = SimpleTimer::new(sysclk_hz);
    let mut counter: u32 = 0;

    loop {
        // Poll at 10ms intervals
        if poll_timer.elapsed_ms(POLL_INTERVAL_MS) {
            // Update platform clock for zenoh-pico
            let clock_ms = clock_start.elapsed_ms_total();
            platform_smoltcp::smoltcp_set_clock_ms(clock_ms);

            // Get smoltcp timestamp
            let timestamp = Instant::from_millis(clock_ms as i64);

            // Poll smoltcp
            let mut dma_ref = &mut dma;
            let _activity = iface.poll(timestamp, &mut dma_ref, &mut sockets);

            // Bridge smoltcp socket to zenoh-pico platform buffers
            let socket = sockets.get_mut::<TcpSocket>(tcp_handle);

            if socket.is_active() {
                // Read from smoltcp socket → push to shim RX buffer
                if socket.can_recv() {
                    let mut buf = [0u8; 256];
                    match socket.recv_slice(&mut buf) {
                        Ok(n) if n > 0 => {
                            debug!("Received {} bytes from TCP", n);
                            let ret = platform_smoltcp::smoltcp_socket_push_rx(0, buf.as_ptr(), n);
                            if ret < 0 {
                                warn!("Failed to push RX data to shim");
                            }
                        }
                        Ok(_) => {}
                        Err(_) => warn!("TCP recv error"),
                    }
                }

                // Pop from shim TX buffer → send through smoltcp socket
                if socket.can_send() {
                    let mut buf = [0u8; 256];
                    let n = platform_smoltcp::smoltcp_socket_pop_tx(0, buf.as_mut_ptr(), buf.len());
                    if n > 0 {
                        debug!("Sending {} bytes via TCP", n);
                        if socket.send_slice(&buf[..n as usize]).is_err() {
                            warn!("TCP send error");
                        }
                    }
                }
            }

            // Poll zenoh-pico platform
            let ret = platform_smoltcp::smoltcp_poll();
            if ret < 0 {
                // Expected for stub - poll callback not fully implemented
            }
        }

        // Publish at 1Hz
        if publish_timer.elapsed_ms(PUBLISH_INTERVAL_MS) {
            counter = counter.wrapping_add(1);

            // Toggle LED to show activity
            led.toggle();

            // When zenoh-pico is available:
            // publisher.publish(&counter.to_le_bytes()).ok();

            info!("Counter = {} (zenoh publish stubbed)", counter);
        }
    }
}
