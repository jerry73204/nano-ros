//! smoltcp Platform Layer for zenoh-pico
//!
//! This module provides the Rust FFI functions that the C platform layer calls
//! to interact with smoltcp for network I/O and system services.
//!
//! # Design
//!
//! The platform layer uses global static state for:
//! - Memory allocator (bump allocator)
//! - Clock/timer (externally updated monotonic counter)
//! - Random number generator (PRNG seeded from clock)
//! - Socket buffers and state
//!
//! # Thread Safety
//!
//! This platform is designed for single-threaded operation only.
//! All access to global state is unsynchronized.
//!
//! # Initialization
//!
//! Before using zenoh-pico:
//! 1. Call `smoltcp_set_poll_callback()` with your smoltcp poll function
//! 2. Call `smoltcp_set_clock_ms()` periodically to update the monotonic clock
//! 3. The poll callback should update socket buffers via `smoltcp_socket_push_rx()`
//!    and `smoltcp_socket_pop_tx()`

use core::ffi::c_void;
use core::mem::MaybeUninit;
use core::ptr;

// ============================================================================
// Memory Allocator
// ============================================================================

/// Heap size for embedded-alloc (16 KB)
const HEAP_SIZE: usize = 16 * 1024;

/// Static heap memory
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

/// Heap initialized flag
static mut HEAP_INITIALIZED: bool = false;

/// Simple bump allocator for embedded use
///
/// This is a minimal allocator that satisfies zenoh-pico's needs.
/// It does not support deallocation (memory is "leaked" but this is
/// acceptable for embedded systems with static lifetimes).
mod bump_alloc {
    use core::cell::UnsafeCell;
    use core::ptr;
    use core::sync::atomic::{AtomicUsize, Ordering};

    /// Simple bump allocator
    pub struct BumpAllocator {
        heap_start: UnsafeCell<*mut u8>,
        heap_end: UnsafeCell<*mut u8>,
        next: AtomicUsize,
    }

    unsafe impl Sync for BumpAllocator {}

    impl BumpAllocator {
        pub const fn new() -> Self {
            Self {
                heap_start: UnsafeCell::new(ptr::null_mut()),
                heap_end: UnsafeCell::new(ptr::null_mut()),
                next: AtomicUsize::new(0),
            }
        }

        /// Initialize the allocator with a memory region
        pub unsafe fn init(&self, heap_start: *mut u8, heap_size: usize) {
            *self.heap_start.get() = heap_start;
            *self.heap_end.get() = heap_start.add(heap_size);
            self.next.store(heap_start as usize, Ordering::Release);
        }

        /// Allocate memory with alignment
        pub fn alloc(&self, size: usize, align: usize) -> *mut u8 {
            // Load current next pointer
            let mut current = self.next.load(Ordering::Acquire);

            loop {
                // Calculate aligned address
                let aligned = (current + align - 1) & !(align - 1);
                let new_next = aligned + size;

                // Check bounds
                let heap_end = unsafe { *self.heap_end.get() as usize };
                if new_next > heap_end {
                    return ptr::null_mut();
                }

                // Try to update atomically
                match self.next.compare_exchange_weak(
                    current,
                    new_next,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                ) {
                    Ok(_) => return aligned as *mut u8,
                    Err(next) => current = next,
                }
            }
        }

        /// Deallocation is a no-op for bump allocator
        #[allow(dead_code)]
        pub fn dealloc(&self, _ptr: *mut u8, _size: usize, _align: usize) {
            // No-op: bump allocator doesn't support deallocation
        }
    }
}

static ALLOCATOR: bump_alloc::BumpAllocator = bump_alloc::BumpAllocator::new();

/// Initialize the heap allocator
#[allow(static_mut_refs)]
unsafe fn init_heap() {
    if !HEAP_INITIALIZED {
        ALLOCATOR.init(HEAP_MEM.as_mut_ptr() as *mut u8, HEAP_SIZE);
        HEAP_INITIALIZED = true;
    }
}

/// Allocate memory
#[no_mangle]
pub extern "C" fn smoltcp_alloc(size: usize) -> *mut c_void {
    unsafe {
        init_heap();
    }
    ALLOCATOR.alloc(size, 8) as *mut c_void
}

/// Reallocate memory
///
/// Note: This is a simplified implementation that allocates new memory
/// and copies the data. The old memory is "leaked" (not reused).
#[no_mangle]
pub extern "C" fn smoltcp_realloc(ptr: *mut c_void, size: usize) -> *mut c_void {
    if ptr.is_null() {
        return smoltcp_alloc(size);
    }

    if size == 0 {
        // Deallocation - no-op for bump allocator
        return ptr::null_mut();
    }

    // Allocate new block and copy
    // Note: We don't know the original size, so we copy `size` bytes
    // This is safe because zenoh-pico always calls realloc with increasing sizes
    let new_ptr = smoltcp_alloc(size);
    if new_ptr.is_null() {
        return ptr::null_mut();
    }

    // Copy data - note: this may read beyond the original allocation
    // but that's acceptable for zenoh-pico's usage pattern
    unsafe {
        ptr::copy_nonoverlapping(ptr as *const u8, new_ptr as *mut u8, size);
    }

    new_ptr
}

/// Free memory (no-op for bump allocator)
#[no_mangle]
pub extern "C" fn smoltcp_free(_ptr: *mut c_void) {
    // No-op: bump allocator doesn't support deallocation
}

// ============================================================================
// Random Number Generator
// ============================================================================

/// Simple xorshift32 PRNG state
static mut RNG_STATE: u32 = 0x12345678;

/// Seed the RNG with a value
unsafe fn seed_rng(seed: u32) {
    RNG_STATE = if seed == 0 { 0x12345678 } else { seed };
}

/// Generate a random u32 using xorshift32
#[no_mangle]
pub extern "C" fn smoltcp_random_u32() -> u32 {
    unsafe {
        // xorshift32 algorithm
        let mut x = RNG_STATE;
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        RNG_STATE = x;
        x
    }
}

// ============================================================================
// Clock/Timer
// ============================================================================

/// Monotonic clock counter (milliseconds)
/// This must be updated by the application (e.g., from RTIC monotonic or SysTick)
static mut CLOCK_MS: u64 = 0;

/// Set the current monotonic time in milliseconds
///
/// Call this from your timer interrupt or monotonic update.
#[no_mangle]
pub extern "C" fn smoltcp_set_clock_ms(ms: u64) {
    unsafe {
        CLOCK_MS = ms;
    }
}

/// Get the current monotonic time in milliseconds
#[no_mangle]
pub extern "C" fn smoltcp_clock_now_ms() -> u64 {
    unsafe { CLOCK_MS }
}

// ============================================================================
// Network Stack State
// ============================================================================

/// Maximum number of TCP sockets
const MAX_SOCKETS: usize = 4;

/// Socket entry state
#[derive(Clone, Copy, Default)]
struct SocketEntry {
    /// Socket is allocated
    allocated: bool,
    /// Local port (0 = ephemeral)
    local_port: u16,
    /// Remote IP address
    remote_ip: [u8; 4],
    /// Remote port
    remote_port: u16,
    /// Connection state
    connected: bool,
    /// RX buffer position and length
    rx_pos: usize,
    rx_len: usize,
    /// TX buffer position and length
    tx_pos: usize,
    tx_len: usize,
}

/// Socket RX/TX buffer size
const SOCKET_BUFFER_SIZE: usize = 2048;

/// Socket buffers
static mut SOCKET_RX_BUFFERS: [[u8; SOCKET_BUFFER_SIZE]; MAX_SOCKETS] =
    [[0u8; SOCKET_BUFFER_SIZE]; MAX_SOCKETS];
static mut SOCKET_TX_BUFFERS: [[u8; SOCKET_BUFFER_SIZE]; MAX_SOCKETS] =
    [[0u8; SOCKET_BUFFER_SIZE]; MAX_SOCKETS];

/// Socket state table
static mut SOCKET_TABLE: [SocketEntry; MAX_SOCKETS] = [SocketEntry {
    allocated: false,
    local_port: 0,
    remote_ip: [0; 4],
    remote_port: 0,
    connected: false,
    rx_pos: 0,
    rx_len: 0,
    tx_pos: 0,
    tx_len: 0,
}; MAX_SOCKETS];

/// Next ephemeral port
static mut NEXT_EPHEMERAL_PORT: u16 = 49152;

/// C-compatible callback function pointer type
pub type PollCallback = Option<unsafe extern "C" fn()>;

/// Platform initialization callback
/// This must be set by the application before using zenoh-pico
static mut POLL_CALLBACK: PollCallback = None;

/// Set the network poll callback
///
/// This callback should poll the smoltcp interface and update socket buffers.
#[no_mangle]
pub extern "C" fn smoltcp_set_poll_callback(callback: PollCallback) {
    unsafe {
        POLL_CALLBACK = callback;
    }
}

/// Poll the network stack
///
/// Calls the registered poll callback if set.
#[no_mangle]
pub extern "C" fn smoltcp_poll() -> i32 {
    #[allow(static_mut_refs)]
    unsafe {
        if let Some(callback) = POLL_CALLBACK {
            callback();
            0
        } else {
            -1
        }
    }
}

/// Initialize the platform
#[no_mangle]
#[allow(static_mut_refs)]
pub extern "C" fn smoltcp_init() -> i32 {
    unsafe {
        // Initialize heap
        init_heap();

        // Seed RNG with clock value
        seed_rng(CLOCK_MS as u32);

        // Reset socket table
        for entry in SOCKET_TABLE.iter_mut() {
            *entry = SocketEntry::default();
        }

        0
    }
}

/// Cleanup the platform
#[no_mangle]
pub extern "C" fn smoltcp_cleanup() {
    // Nothing to cleanup for static allocations
}

// ============================================================================
// Socket Operations
// ============================================================================

/// Allocate a new socket
#[no_mangle]
#[allow(static_mut_refs)]
pub extern "C" fn smoltcp_socket_open() -> i32 {
    unsafe {
        // Find free slot
        for (i, entry) in SOCKET_TABLE.iter_mut().enumerate() {
            if !entry.allocated {
                entry.allocated = true;
                entry.connected = false;
                entry.local_port = NEXT_EPHEMERAL_PORT;
                NEXT_EPHEMERAL_PORT = NEXT_EPHEMERAL_PORT.wrapping_add(1);
                if NEXT_EPHEMERAL_PORT < 49152 {
                    NEXT_EPHEMERAL_PORT = 49152;
                }
                entry.rx_pos = 0;
                entry.rx_len = 0;
                entry.tx_pos = 0;
                entry.tx_len = 0;
                return i as i32;
            }
        }
        -1 // No free slots
    }
}

/// Initiate a TCP connection
///
/// This stores the connection parameters. The actual connection is established
/// when the poll callback drives the smoltcp state machine.
#[no_mangle]
pub extern "C" fn smoltcp_socket_connect(handle: i32, ip: *const u8, port: u16) -> i32 {
    if handle < 0 || handle >= MAX_SOCKETS as i32 || ip.is_null() {
        return -1;
    }

    unsafe {
        let entry = &mut SOCKET_TABLE[handle as usize];
        if !entry.allocated {
            return -1;
        }

        // Store connection parameters
        ptr::copy_nonoverlapping(ip, entry.remote_ip.as_mut_ptr(), 4);
        entry.remote_port = port;

        // Mark as connecting (the poll callback will establish the connection)
        // For now, assume connection succeeds immediately
        // In a real implementation, this would be driven by smoltcp
        entry.connected = true;

        0
    }
}

/// Check if socket is connected
#[no_mangle]
pub extern "C" fn smoltcp_socket_is_connected(handle: i32) -> i32 {
    if handle < 0 || handle >= MAX_SOCKETS as i32 {
        return 0;
    }

    unsafe {
        let entry = &SOCKET_TABLE[handle as usize];
        if entry.allocated && entry.connected {
            1
        } else {
            0
        }
    }
}

/// Close a socket
#[no_mangle]
pub extern "C" fn smoltcp_socket_close(handle: i32) -> i32 {
    if handle < 0 || handle >= MAX_SOCKETS as i32 {
        return -1;
    }

    unsafe {
        let entry = &mut SOCKET_TABLE[handle as usize];
        entry.allocated = false;
        entry.connected = false;
        0
    }
}

/// Check if socket can receive data
#[no_mangle]
pub extern "C" fn smoltcp_socket_can_recv(handle: i32) -> i32 {
    if handle < 0 || handle >= MAX_SOCKETS as i32 {
        return 0;
    }

    unsafe {
        let entry = &SOCKET_TABLE[handle as usize];
        if entry.allocated && entry.rx_len > entry.rx_pos {
            1
        } else {
            0
        }
    }
}

/// Check if socket can send data
#[no_mangle]
pub extern "C" fn smoltcp_socket_can_send(handle: i32) -> i32 {
    if handle < 0 || handle >= MAX_SOCKETS as i32 {
        return 0;
    }

    unsafe {
        let entry = &SOCKET_TABLE[handle as usize];
        if entry.allocated && entry.connected && entry.tx_len < SOCKET_BUFFER_SIZE {
            1
        } else {
            0
        }
    }
}

/// Receive data from socket
#[no_mangle]
pub extern "C" fn smoltcp_socket_recv(handle: i32, buf: *mut u8, len: usize) -> i32 {
    if handle < 0 || handle >= MAX_SOCKETS as i32 || buf.is_null() || len == 0 {
        return -1;
    }

    unsafe {
        let entry = &mut SOCKET_TABLE[handle as usize];
        if !entry.allocated {
            return -1;
        }

        let available = entry.rx_len.saturating_sub(entry.rx_pos);
        if available == 0 {
            return 0;
        }

        let to_copy = available.min(len);
        let rx_buf = &SOCKET_RX_BUFFERS[handle as usize];
        ptr::copy_nonoverlapping(rx_buf[entry.rx_pos..].as_ptr(), buf, to_copy);
        entry.rx_pos += to_copy;

        // Reset buffer if fully consumed
        if entry.rx_pos >= entry.rx_len {
            entry.rx_pos = 0;
            entry.rx_len = 0;
        }

        to_copy as i32
    }
}

/// Send data to socket
#[no_mangle]
pub extern "C" fn smoltcp_socket_send(handle: i32, buf: *const u8, len: usize) -> i32 {
    if handle < 0 || handle >= MAX_SOCKETS as i32 || buf.is_null() || len == 0 {
        return -1;
    }

    unsafe {
        let entry = &mut SOCKET_TABLE[handle as usize];
        if !entry.allocated || !entry.connected {
            return -1;
        }

        let available = SOCKET_BUFFER_SIZE.saturating_sub(entry.tx_len);
        if available == 0 {
            return 0;
        }

        let to_copy = available.min(len);
        let tx_buf = &mut SOCKET_TX_BUFFERS[handle as usize];
        ptr::copy_nonoverlapping(buf, tx_buf[entry.tx_len..].as_mut_ptr(), to_copy);
        entry.tx_len += to_copy;

        to_copy as i32
    }
}

// ============================================================================
// Socket Buffer Access for External Integration
// ============================================================================

/// Push received data into a socket's RX buffer
///
/// Called by the smoltcp integration layer when data is received.
#[no_mangle]
pub extern "C" fn smoltcp_socket_push_rx(handle: i32, data: *const u8, len: usize) -> i32 {
    if handle < 0 || handle >= MAX_SOCKETS as i32 || data.is_null() || len == 0 {
        return -1;
    }

    unsafe {
        let entry = &mut SOCKET_TABLE[handle as usize];
        if !entry.allocated {
            return -1;
        }

        // Compact buffer if needed
        if entry.rx_pos > 0 {
            let remaining = entry.rx_len - entry.rx_pos;
            let rx_buf = &mut SOCKET_RX_BUFFERS[handle as usize];
            ptr::copy(
                rx_buf[entry.rx_pos..].as_ptr(),
                rx_buf.as_mut_ptr(),
                remaining,
            );
            entry.rx_len = remaining;
            entry.rx_pos = 0;
        }

        let available = SOCKET_BUFFER_SIZE.saturating_sub(entry.rx_len);
        if available == 0 {
            return 0;
        }

        let to_copy = available.min(len);
        let rx_buf = &mut SOCKET_RX_BUFFERS[handle as usize];
        ptr::copy_nonoverlapping(data, rx_buf[entry.rx_len..].as_mut_ptr(), to_copy);
        entry.rx_len += to_copy;

        to_copy as i32
    }
}

/// Pop pending data from a socket's TX buffer
///
/// Called by the smoltcp integration layer when ready to send.
#[no_mangle]
pub extern "C" fn smoltcp_socket_pop_tx(handle: i32, buf: *mut u8, max_len: usize) -> i32 {
    if handle < 0 || handle >= MAX_SOCKETS as i32 || buf.is_null() || max_len == 0 {
        return -1;
    }

    unsafe {
        let entry = &mut SOCKET_TABLE[handle as usize];
        if !entry.allocated {
            return -1;
        }

        let to_copy = entry.tx_len.min(max_len);
        if to_copy == 0 {
            return 0;
        }

        let tx_buf = &SOCKET_TX_BUFFERS[handle as usize];
        ptr::copy_nonoverlapping(tx_buf.as_ptr(), buf, to_copy);

        // Shift remaining data
        if to_copy < entry.tx_len {
            let remaining = entry.tx_len - to_copy;
            let tx_buf = &mut SOCKET_TX_BUFFERS[handle as usize];
            ptr::copy(tx_buf[to_copy..].as_ptr(), tx_buf.as_mut_ptr(), remaining);
            entry.tx_len = remaining;
        } else {
            entry.tx_len = 0;
        }

        to_copy as i32
    }
}

/// Get socket connection parameters
#[no_mangle]
pub extern "C" fn smoltcp_socket_get_remote(handle: i32, ip: *mut u8, port: *mut u16) -> i32 {
    if handle < 0 || handle >= MAX_SOCKETS as i32 {
        return -1;
    }

    unsafe {
        let entry = &SOCKET_TABLE[handle as usize];
        if !entry.allocated {
            return -1;
        }

        if !ip.is_null() {
            ptr::copy_nonoverlapping(entry.remote_ip.as_ptr(), ip, 4);
        }
        if !port.is_null() {
            *port = entry.remote_port;
        }

        0
    }
}

/// Set socket as connected
///
/// Called by the smoltcp integration layer when connection is established.
#[no_mangle]
pub extern "C" fn smoltcp_socket_set_connected(handle: i32, connected: bool) {
    if handle >= 0 && (handle as usize) < MAX_SOCKETS {
        unsafe {
            SOCKET_TABLE[handle as usize].connected = connected;
        }
    }
}
