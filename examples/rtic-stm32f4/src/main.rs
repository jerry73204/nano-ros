//! RTIC Example for nano-ros on STM32F4 with zenoh-pico-shim
//!
//! This example demonstrates nano-ros with zenoh-pico-shim on an STM32F4
//! microcontroller using smoltcp for TCP/IP networking.
//!
//! # Architecture
//!
//! ```text
//! RTIC Tasks
//! ├── poll_network (priority 2)
//! │   └── Polls smoltcp, bridges to zenoh-pico platform buffers
//! ├── zenoh_poll (priority 2)
//! │   └── Calls ShimContext::spin_once() for zenoh processing
//! └── publisher_task (priority 1)
//!     └── Periodic publishing of sensor data
//! ```
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

use defmt::{debug, error, info, warn};
use defmt_rtt as _;
use panic_probe as _;

use rtic::app;
use rtic_monotonics::systick::prelude::*;

use stm32f4xx_hal::{gpio::GpioExt, prelude::*, rcc::RccExt};

use stm32_eth::{
    dma::{EthernetDMA, RxRingEntry, TxRingEntry},
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

/// Network poll callback
///
/// This is called by zenoh-pico's platform layer when it needs to poll
/// the network. We bridge smoltcp's socket operations to the shim's buffers.
unsafe extern "C" fn network_poll_callback() {
    // This callback is invoked from zenoh-pico's network layer.
    // It should poll smoltcp and transfer data between smoltcp sockets
    // and the shim's internal buffers.
    //
    // The actual implementation would:
    // 1. Poll the smoltcp interface
    // 2. For each active zenoh socket:
    //    - Read from smoltcp socket → push to shim RX buffer
    //    - Pop from shim TX buffer → send through smoltcp socket
    //
    // This is handled in the poll_network RTIC task instead, as it has
    // access to the shared resources.
}

// ============================================================================
// RTIC Application
// ============================================================================

systick_monotonic!(Mono, 1000); // 1 kHz = 1ms resolution

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART1, USART2, USART3])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        eth_dma: EthernetDMA<'static, 'static>,
        iface: Interface,
        sockets: SocketSet<'static>,
        counter: u32,
    }

    #[local]
    struct Local {
        tcp_handle: smoltcp::iface::SocketHandle,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("nano-ros RTIC + zenoh-pico-shim example starting...");

        let dp = cx.device;

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

        // Initialize monotonic timer
        Mono::start(cx.core.SYST, clocks.sysclk().to_Hz());

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

        // Initialize Ethernet peripheral
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

        // Enable Ethernet interrupts
        dma.enable_interrupt();

        // Create smoltcp interface
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
        let sockets = unsafe { SocketSet::new(&mut SOCKET_STORAGE[..]) };

        // Create TCP socket for zenoh connection
        let tcp_rx_buffer = unsafe { TcpSocketBuffer::new(&mut TCP_RX_BUFFER[..]) };
        let tcp_tx_buffer = unsafe { TcpSocketBuffer::new(&mut TCP_TX_BUFFER[..]) };
        let tcp_socket = TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);

        let mut sockets = sockets;
        let tcp_handle = sockets.add(tcp_socket);

        // ═══════════════════════════════════════════════════════════════════════
        // Initialize zenoh-pico-shim platform
        // ═══════════════════════════════════════════════════════════════════════
        info!("Initializing zenoh-pico platform...");

        // Set the platform clock (will be updated in poll task)
        platform_smoltcp::smoltcp_set_clock_ms(0);

        // Set the network poll callback
        // Note: The actual polling is done in poll_network task
        platform_smoltcp::smoltcp_set_poll_callback(Some(network_poll_callback));

        // Initialize the platform
        let ret = platform_smoltcp::smoltcp_init();
        if ret < 0 {
            error!("Failed to initialize smoltcp platform: {}", ret);
        } else {
            info!("smoltcp platform initialized");
        }

        // ═══════════════════════════════════════════════════════════════════════
        // Initialize zenoh-pico shim session
        // ═══════════════════════════════════════════════════════════════════════
        //
        // Note: The following code requires zenoh-pico to be cross-compiled for
        // ARM Cortex-M. When available, uncomment this section:
        //
        // use zenoh_pico_shim::ShimContext;
        //
        // info!("Connecting to zenoh router...");
        // match ShimContext::new(ZENOH_ROUTER) {
        //     Ok(ctx) => {
        //         info!("Connected to zenoh router!");
        //
        //         // Declare publisher
        //         match ctx.declare_publisher(b"nano_ros/rtic/counter\0") {
        //             Ok(publisher) => {
        //                 info!("Publisher declared for nano_ros/rtic/counter");
        //                 // Store publisher in shared resources
        //             }
        //             Err(e) => error!("Failed to declare publisher: {}", e),
        //         }
        //     }
        //     Err(e) => error!("Failed to connect to zenoh: {}", e),
        // }
        //
        // For now, we demonstrate the network stack integration without zenoh.

        info!("Starting periodic tasks...");

        // Start polling tasks
        poll_network::spawn().ok();
        zenoh_poll::spawn().ok();
        publisher_task::spawn().ok();

        (
            Shared {
                eth_dma: dma,
                iface,
                sockets,
                counter: 0,
            },
            Local { tcp_handle },
        )
    }

    /// Periodic network polling task
    ///
    /// Polls the smoltcp interface and bridges data between smoltcp sockets
    /// and the zenoh-pico platform's socket buffers.
    #[task(shared = [eth_dma, iface, sockets], local = [tcp_handle], priority = 2)]
    async fn poll_network(mut cx: poll_network::Context) {
        let tcp_handle = *cx.local.tcp_handle;

        loop {
            // Get current timestamp
            let now = Mono::now();
            let timestamp = Instant::from_millis(now.ticks() as i64);

            // Update platform clock for zenoh-pico
            platform_smoltcp::smoltcp_set_clock_ms(now.ticks() as u64);

            // Poll the network interface
            (
                &mut cx.shared.eth_dma,
                &mut cx.shared.iface,
                &mut cx.shared.sockets,
            )
                .lock(|mut eth_dma, iface, sockets| {
                    // Poll smoltcp
                    let _activity = iface.poll(timestamp, &mut eth_dma, sockets);

                    // Bridge smoltcp socket to zenoh-pico platform buffers
                    let socket = sockets.get_mut::<TcpSocket>(tcp_handle);

                    if socket.is_active() {
                        // Read from smoltcp socket → push to shim RX buffer
                        if socket.can_recv() {
                            let mut buf = [0u8; 256];
                            match socket.recv_slice(&mut buf) {
                                Ok(n) if n > 0 => {
                                    debug!("Received {} bytes from TCP", n);
                                    // Push to zenoh-pico platform buffer
                                    // Note: Handle 0 is the first zenoh socket
                                    let ret = platform_smoltcp::smoltcp_socket_push_rx(
                                        0,
                                        buf.as_ptr(),
                                        n,
                                    );
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
                            let n = platform_smoltcp::smoltcp_socket_pop_tx(
                                0,
                                buf.as_mut_ptr(),
                                buf.len(),
                            );
                            if n > 0 {
                                debug!("Sending {} bytes via TCP", n);
                                if socket.send_slice(&buf[..n as usize]).is_err() {
                                    warn!("TCP send error");
                                }
                            }
                        }
                    }
                });

            Mono::delay(POLL_INTERVAL_MS.millis()).await;
        }
    }

    /// Zenoh polling task
    ///
    /// Calls zenoh-pico's spin_once to process incoming messages and
    /// invoke subscriber callbacks.
    #[task(priority = 2)]
    async fn zenoh_poll(_cx: zenoh_poll::Context) {
        loop {
            // When zenoh-pico is available, this would be:
            // if let Some(ctx) = &ZENOH_CONTEXT {
            //     match ctx.spin_once(0) {
            //         Ok(events) if events > 0 => {
            //             debug!("Processed {} zenoh events", events);
            //         }
            //         Ok(_) => {}
            //         Err(e) => warn!("Zenoh spin error: {}", e),
            //     }
            // }

            // For now, just poll the platform layer
            let ret = platform_smoltcp::smoltcp_poll();
            if ret < 0 {
                // Poll callback not set or error - expected for stub
            }

            Mono::delay(POLL_INTERVAL_MS.millis()).await;
        }
    }

    /// Publish messages periodically
    ///
    /// Demonstrates publishing ROS 2 messages at a fixed rate.
    #[task(shared = [counter], priority = 1)]
    async fn publisher_task(mut cx: publisher_task::Context) {
        loop {
            // Increment counter
            let count = cx.shared.counter.lock(|c| {
                *c = c.wrapping_add(1);
                *c
            });

            // When zenoh-pico is available, this would publish:
            // if let Some(publisher) = &PUBLISHER {
            //     // Create Int32 message
            //     use nano_ros_core::Serialize;
            //     use nano_ros_serdes::CdrWriter;
            //     use std_msgs::msg::Int32;
            //
            //     let msg = Int32 { data: count as i32 };
            //     let mut buffer = [0u8; 64];
            //     let mut writer = CdrWriter::new(&mut buffer);
            //     if msg.serialize(&mut writer).is_ok() {
            //         let len = writer.position();
            //         if publisher.publish(&buffer[..len]).is_ok() {
            //             info!("Published: counter = {}", count);
            //         }
            //     }
            // }

            info!("Counter = {} (zenoh publish stubbed)", count);

            Mono::delay(PUBLISH_INTERVAL_MS.millis()).await;
        }
    }

    /// Ethernet interrupt handler
    #[task(binds = ETH, shared = [eth_dma], priority = 3)]
    fn eth_interrupt(mut cx: eth_interrupt::Context) {
        cx.shared.eth_dma.lock(|_eth_dma| {
            EthernetDMA::interrupt_handler();
        });
    }
}
