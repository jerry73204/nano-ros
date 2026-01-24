//! smoltcp TCP Echo Server for NUCLEO-F429ZI
//!
//! This example validates that smoltcp + stm32-eth works correctly on the
//! STM32F429ZI before integrating with zenoh-pico.
//!
//! # Hardware Setup
//!
//! - Board: NUCLEO-F429ZI (or similar STM32F4 with Ethernet)
//! - Connect Ethernet cable to the board's RJ45 port
//! - Configure host PC with static IP 192.168.1.1/24 or use DHCP
//!
//! # Network Configuration
//!
//! Default (static IP):
//! - Device IP: 192.168.1.10
//! - Gateway: 192.168.1.1
//! - Netmask: 255.255.255.0
//! - TCP port: 7 (echo)
//!
//! # Testing
//!
//! From host PC:
//! ```bash
//! # Test TCP echo
//! nc 192.168.1.10 7
//! # Type text and see it echoed back
//! ```
//!
//! # Building
//!
//! ```bash
//! cargo build --release
//! # Flash with probe-rs
//! cargo run --release
//! ```

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

// ============================================================================
// Network Configuration
// ============================================================================

/// Device MAC address (locally administered)
const MAC_ADDRESS: [u8; 6] = [0x02, 0x00, 0x00, 0x00, 0x00, 0x01];

/// Device IP address (static)
const IP_ADDRESS: [u8; 4] = [192, 168, 1, 10];

/// Network gateway
const GATEWAY: Ipv4Address = Ipv4Address::new(192, 168, 1, 1);

/// TCP echo server port
const ECHO_PORT: u16 = 7;

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

// Ethernet DMA descriptors - must be in normal RAM (not CCM)
#[link_section = ".ethram"]
static mut RX_RING: [RxRingEntry; RX_DESC_COUNT] = [RxRingEntry::INIT; RX_DESC_COUNT];
#[link_section = ".ethram"]
static mut TX_RING: [TxRingEntry; TX_DESC_COUNT] = [TxRingEntry::INIT; TX_DESC_COUNT];

// TCP socket buffers
static mut TCP_RX_BUFFER: [u8; TCP_RX_BUFFER_SIZE] = [0u8; TCP_RX_BUFFER_SIZE];
static mut TCP_TX_BUFFER: [u8; TCP_TX_BUFFER_SIZE] = [0u8; TCP_TX_BUFFER_SIZE];

// Socket storage
static mut SOCKET_STORAGE: [smoltcp::iface::SocketStorage<'static>; MAX_SOCKETS] =
    [smoltcp::iface::SocketStorage::EMPTY; MAX_SOCKETS];

// ============================================================================
// RTIC Application
// ============================================================================

systick_monotonic!(Mono, 1000); // 1 kHz = 1ms resolution

#[app(device = stm32f4xx_hal::pac, peripherals = true, dispatchers = [USART1, USART2])]
mod app {
    use super::*;

    #[shared]
    struct Shared {
        eth_dma: EthernetDMA<'static, 'static>,
        iface: Interface,
        sockets: SocketSet<'static>,
    }

    #[local]
    struct Local {
        tcp_handle: smoltcp::iface::SocketHandle,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        info!("smoltcp TCP Echo Server starting...");

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

        // RMII pins for NUCLEO-F429ZI - use floating input mode
        // The ethernet driver will configure them as alternate function internally
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

        // MDC/MDIO for PHY management (Alternate function 11)
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

        // SAFETY: RX_RING and TX_RING are only accessed once during initialization
        // and the stm32_eth crate requires mutable references for the DMA ring buffers.
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

        // Wait for link to come up
        info!("Waiting for Ethernet link...");
        // Note: In a real application, you'd poll the PHY status
        // For now, we just proceed and let smoltcp handle it

        // Enable Ethernet interrupts
        dma.enable_interrupt();

        // Create smoltcp interface
        info!("Creating smoltcp interface...");

        let mac_addr = EthernetAddress::from_bytes(&MAC_ADDRESS);
        let config = Config::new(mac_addr.into());

        // Device trait is implemented for &mut EthernetDMA, so Interface::new
        // takes &mut (&mut EthernetDMA) = &mut &mut EthernetDMA
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
        info!("Gateway: 192.168.1.1");

        // Create socket set
        let sockets = unsafe { SocketSet::new(&mut SOCKET_STORAGE[..]) };

        // Create TCP socket for echo server
        let tcp_rx_buffer = unsafe { TcpSocketBuffer::new(&mut TCP_RX_BUFFER[..]) };
        let tcp_tx_buffer = unsafe { TcpSocketBuffer::new(&mut TCP_TX_BUFFER[..]) };
        let tcp_socket = TcpSocket::new(tcp_rx_buffer, tcp_tx_buffer);

        let mut sockets = sockets;
        let tcp_handle = sockets.add(tcp_socket);

        // Start listening on echo port
        {
            let socket = sockets.get_mut::<TcpSocket>(tcp_handle);
            socket.listen(ECHO_PORT).expect("Failed to listen");
            info!("TCP echo server listening on port {}", ECHO_PORT);
        }

        // Start polling task
        poll_network::spawn().ok();

        (
            Shared {
                eth_dma: dma,
                iface,
                sockets,
            },
            Local { tcp_handle },
        )
    }

    /// Periodic network polling task
    #[task(shared = [eth_dma, iface, sockets], local = [tcp_handle], priority = 1)]
    async fn poll_network(mut cx: poll_network::Context) {
        let tcp_handle = *cx.local.tcp_handle;

        loop {
            // Get current timestamp
            let now = Mono::now();
            let timestamp = Instant::from_millis(now.ticks() as i64);

            // Poll the network interface
            (
                &mut cx.shared.eth_dma,
                &mut cx.shared.iface,
                &mut cx.shared.sockets,
            )
                .lock(|mut eth_dma, iface, sockets| {
                    // Poll smoltcp - Device trait is impl for &mut EthernetDMA
                    // so we need &mut (&mut EthernetDMA) for the poll call
                    let _activity = iface.poll(timestamp, &mut eth_dma, sockets);

                    // Handle TCP echo
                    let socket = sockets.get_mut::<TcpSocket>(tcp_handle);

                    if socket.is_active() {
                        // Echo received data back
                        if socket.can_recv() && socket.can_send() {
                            let mut buf = [0u8; 256];
                            match socket.recv_slice(&mut buf) {
                                Ok(n) if n > 0 => {
                                    debug!("Received {} bytes", n);
                                    if socket.send_slice(&buf[..n]).is_err() {
                                        warn!("Failed to send echo response");
                                    }
                                }
                                Ok(_) => {}
                                Err(_) => {
                                    warn!("Recv error");
                                }
                            }
                        }
                    } else if !socket.is_listening() {
                        // Socket closed, restart listening
                        info!("Client disconnected, restarting listener");
                        socket.abort();
                        if socket.listen(ECHO_PORT).is_err() {
                            error!("Failed to listen");
                        }
                    }
                });

            // Poll every 10ms
            Mono::delay(10.millis()).await;
        }
    }

    /// Ethernet interrupt handler
    #[task(binds = ETH, shared = [eth_dma], priority = 2)]
    fn eth_interrupt(mut cx: eth_interrupt::Context) {
        cx.shared.eth_dma.lock(|_eth_dma| {
            // Clear interrupt flags - this is a static function in stm32-eth 0.8
            EthernetDMA::interrupt_handler();
        });
    }
}
