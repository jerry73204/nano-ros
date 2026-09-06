//! Configuration for QEMU FreeRTOS nodes
//!
//! phase-337 W5 — the `{mac, ip, netmask, gateway, locator, domain_id}` core is
//! [`nros_board_common::BaseConfig`], shared with every other networked board,
//! and the scheduling defaults are [`nros_board_common::FreertosScheduling`],
//! shared with the `build.rs` that emits the C-side `NROS_APP_CONFIG` mirror
//! (they had drifted by 128 KiB of app stack before that). What stays here is
//! what is genuinely FreeRTOS: the normalized-priority map and the `nros.toml`
//! parser.

use nros_board_common::{
    BaseConfig, FreertosScheduling, base_config::netmask_from_prefix, freertos_config,
};

/// Network and node configuration for a FreeRTOS + lwIP board.
///
/// # Default (Talker)
///
/// - IP: 192.0.3.10/24, Gateway: 192.0.3.1
/// - Zenoh: `tcp/192.0.3.1:7447`
#[derive(Clone)]
pub struct Config {
    /// The shared `{mac, ip, netmask, gateway, zenoh_locator, domain_id}` core.
    pub base: BaseConfig,

    // ── Scheduling ─────────────────────────────────────────────────────
    // RAW FreeRTOS priorities (0..configMAX_PRIORITIES-1, higher = more
    // urgent) — the same units a `[tiers.<name>.freertos] priority` is written
    // in, so the two can be compared without knowing a mapping (issue 0623).
    // A `[node.rt]` section still accepts the legacy normalized 0-31 scale and
    // converts on read; `[node.rt.freertos]` is raw.
    /// Application task priority (raw FreeRTOS, default 3).
    pub app_priority: u8,
    /// Application task stack size in bytes (default 131072 — MEASURED, issue
    /// 1146; see `nros_board_common::freertos_config::app_stack_bytes`).
    pub app_stack_bytes: u32,
    /// Zenoh-pico read task priority (raw FreeRTOS, default 4).
    pub zenoh_read_priority: u8,
    /// Zenoh-pico read task stack size in bytes (default 5120).
    pub zenoh_read_stack_bytes: u32,
    /// Zenoh-pico lease task priority (raw FreeRTOS, default 4).
    pub zenoh_lease_priority: u8,
    /// Zenoh-pico lease task stack size in bytes (default 5120).
    pub zenoh_lease_stack_bytes: u32,
    /// Network poll task priority (raw FreeRTOS, default 4).
    pub poll_priority: u8,
    /// Network poll interval in milliseconds (default 5).
    pub poll_interval_ms: u32,
}

impl Default for Config {
    fn default() -> Self {
        // issue-0274 follow-up (sentinel 14.5): the app stack is overridable at
        // build time — a 10-node macro entry's register pass overflows the
        // default. One parser, shared with the `build.rs` emitter, so the two
        // cannot disagree about what the override means.
        let sched = FreertosScheduling {
            app_stack_bytes: freertos_config::app_stack_bytes(option_env!(
                "NROS_FREERTOS_APP_STACK_KB"
            )),
            ..FreertosScheduling::default()
        };
        Self {
            base: BaseConfig::default(),
            app_priority: sched.app_priority,
            app_stack_bytes: sched.app_stack_bytes,
            zenoh_read_priority: sched.zenoh_read_priority,
            zenoh_read_stack_bytes: sched.zenoh_read_stack_bytes,
            zenoh_lease_priority: sched.zenoh_lease_priority,
            zenoh_lease_stack_bytes: sched.zenoh_lease_stack_bytes,
            poll_priority: sched.poll_priority,
            poll_interval_ms: sched.poll_interval_ms,
        }
    }
}

impl Config {
    /// Preset for a listener/subscriber node.
    pub fn listener() -> Self {
        Self {
            base: BaseConfig {
                mac: [0x02, 0x00, 0x00, 0x00, 0x00, 0x01],
                ip: [192, 0, 3, 11],
                ..BaseConfig::default()
            },
            ..Self::default()
        }
    }

    /// Alias for `Config::default()`.
    pub fn talker() -> Self {
        Self::default()
    }

    /// Builder: set MAC address.
    pub fn with_mac(mut self, mac: [u8; 6]) -> Self {
        self.base.mac = mac;
        self
    }

    /// Builder: set IP address.
    pub fn with_ip(mut self, ip: [u8; 4]) -> Self {
        self.base.ip = ip;
        self
    }

    /// Builder: set network mask.
    pub fn with_netmask(mut self, netmask: [u8; 4]) -> Self {
        self.base.netmask = netmask;
        self
    }

    /// Builder: set gateway.
    pub fn with_gateway(mut self, gateway: [u8; 4]) -> Self {
        self.base.gateway = gateway;
        self
    }

    /// Builder: set the transport locator the RMW backend connects
    /// through (e.g. `"tcp/192.0.3.1:7447"`,
    /// `"serial/UART_0#baudrate=115200"`).
    pub fn with_locator(mut self, locator: &'static str) -> Self {
        self.base.zenoh_locator = locator;
        self
    }

    /// Deprecated alias for [`with_locator`](Self::with_locator).
    #[deprecated(
        since = "0.6.0",
        note = "renamed to `with_locator()` — the config API must not name a backend (issue 0330)"
    )]
    pub fn with_zenoh_locator(self, locator: &'static str) -> Self {
        self.with_locator(locator)
    }

    /// Builder: set ROS 2 domain ID.
    pub fn with_domain_id(mut self, domain_id: u32) -> Self {
        self.base.domain_id = domain_id;
        self
    }

    /// Builder: set application task priority (normalized 0–31).
    pub fn with_app_priority(mut self, p: u8) -> Self {
        self.app_priority = p;
        self
    }

    /// Builder: set application task stack size in bytes.
    pub fn with_app_stack_bytes(mut self, s: u32) -> Self {
        self.app_stack_bytes = s;
        self
    }

    /// Builder: set network poll interval in milliseconds.
    pub fn with_poll_interval_ms(mut self, ms: u32) -> Self {
        self.poll_interval_ms = ms;
        self
    }

    /// Map a normalized 0–31 priority to FreeRTOS priority.
    ///
    /// FreeRTOS uses 0 (idle) to `configMAX_PRIORITIES - 1` (highest).
    /// The shared `FreeRTOSConfig.h` sets `configMAX_PRIORITIES = 8`,
    /// so the output range is 0–7.
    ///
    /// Mapping: `freertos_pri = round(normalized * 7 / 31)` (linear,
    /// round-to-nearest). The rounding matters: `FreertosScheduling`'s
    /// defaults were normalized FROM the old raw constants expecting
    /// this map to round-trip (16 ≈ 3.6 → 4, the historical
    /// POLL_TASK_PRIORITY; 12 ≈ 2.7 → 3, the historical
    /// APP_TASK_PRIORITY). The previous flooring map sent 16 → 3,
    /// which dropped the zenoh read/lease/poll tasks BELOW
    /// `TCPIP_THREAD_PRIO` (4) and below typical application tiers,
    /// so under inbound load the RX drain starved, the LAN9118 path
    /// dropped frames, and every publisher stalled on lwIP's
    /// retransmission timeout (observed as 1-3 s island-wide stalls).
    ///
    /// Issue 0623 — the body moved to `nros_board_common::freertos_config`, so
    /// the build-script emitter and this runtime path share ONE conversion.
    /// They did not before, and the C entry's `clamp_prio` disagreed with this
    /// one on every default.
    pub fn to_freertos_priority(normalized: u8) -> u32 {
        nros_board_common::freertos_config::to_freertos_priority(normalized)
    }

    /// Parse configuration from a TOML string.
    ///
    /// Missing fields use board-specific defaults. This is designed to work
    /// with `include_str!("../config.toml")` for compile-time embedding.
    ///
    /// # Supported fields
    ///
    /// ```toml
    /// [[transport]]
    /// ip = "192.0.3.10/24"
    /// mac = "02:00:00:00:00:00"
    /// gateway = "192.0.3.1"
    /// locator = "tcp/192.0.3.1:7447"
    ///
    /// [node]
    /// domain_id = 0
    /// ```
    pub fn from_toml(toml: &'static str) -> Self {
        let mut config = Self::default();
        let mut section = "";

        for line in toml.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            // Phase 172.K — `[[transport]]` array-of-tables + dotted `[node.rt]`.
            if line.starts_with('[') {
                if line.starts_with("[[") {
                    if let Some(end) = line.find("]]") {
                        section = line[2..end].trim();
                    }
                } else if let Some(end) = line.find(']') {
                    section = line[1..end].trim();
                }
                continue;
            }
            if let Some(eq_pos) = line.find('=') {
                let key = line[..eq_pos].trim();
                let value = line[eq_pos + 1..].trim();
                let value = if (value.starts_with('"') && value.ends_with('"'))
                    || (value.starts_with('\'') && value.ends_with('\''))
                {
                    &value[1..value.len() - 1]
                } else {
                    value
                };

                match (section, key) {
                    // Phase 172.K — direct-mode nros.toml only; legacy
                    // `[network]`/`[zenoh]`/`[scheduling]` arms dropped in K.6.

                    // direct-mode nros.toml: `[[transport]]`
                    // (ip CIDR → ip + netmask) + `[node]` + `[node.rt]`.
                    ("transport", "ip") => {
                        let (addr, pfx) = value.split_once('/').unwrap_or((value, ""));
                        if let Some(ip) = parse_ipv4(addr) {
                            config.base.ip = ip;
                        }
                        if let Some(p) = parse_u32(pfx) {
                            config.base.netmask = netmask_from_prefix(p as u8);
                        }
                    }
                    ("transport", "mac") => {
                        if let Some(mac) = parse_mac(value) {
                            config.base.mac = mac;
                        }
                    }
                    ("transport", "gateway") => {
                        if let Some(gw) = parse_ipv4(value) {
                            config.base.gateway = gw;
                        }
                    }
                    ("transport", "locator") => {
                        config.base.zenoh_locator = value;
                    }
                    ("node", "domain_id") => {
                        if let Some(d) = parse_u32(value) {
                            config.base.domain_id = d;
                        }
                    }
                    // Issue 0623 — `[node.rt]` is the LEGACY normalized 0-31
                    // spelling and is converted on read, so configs written
                    // before the units were unified keep their schedule.
                    ("node.rt", "app_priority") => {
                        if let Some(v) = parse_u32(value) {
                            config.app_priority = Self::to_freertos_priority(v.min(31) as u8) as u8;
                        }
                    }
                    // ...and `[node.rt.freertos]` is RAW, mirroring
                    // `[tiers.<name>.freertos] priority`. One vocabulary for
                    // the two numbers that meet in one scheduler.
                    ("node.rt.freertos", "app_priority") => {
                        if let Some(v) = parse_u32(value) {
                            config.app_priority = v as u8;
                        }
                    }
                    ("node.rt", "app_stack_bytes") => {
                        if let Some(v) = parse_u32(value) {
                            config.app_stack_bytes = v;
                        }
                    }
                    // Issue 0623 — `[node.rt]` is the LEGACY normalized 0-31
                    // spelling and is converted on read, so configs written
                    // before the units were unified keep their schedule.
                    ("node.rt", "zenoh_read_priority") => {
                        if let Some(v) = parse_u32(value) {
                            config.zenoh_read_priority =
                                Self::to_freertos_priority(v.min(31) as u8) as u8;
                        }
                    }
                    // ...and `[node.rt.freertos]` is RAW, mirroring
                    // `[tiers.<name>.freertos] priority`. One vocabulary for
                    // the two numbers that meet in one scheduler.
                    ("node.rt.freertos", "zenoh_read_priority") => {
                        if let Some(v) = parse_u32(value) {
                            config.zenoh_read_priority = v as u8;
                        }
                    }
                    ("node.rt", "zenoh_read_stack_bytes") => {
                        if let Some(v) = parse_u32(value) {
                            config.zenoh_read_stack_bytes = v;
                        }
                    }
                    // Issue 0623 — `[node.rt]` is the LEGACY normalized 0-31
                    // spelling and is converted on read, so configs written
                    // before the units were unified keep their schedule.
                    ("node.rt", "zenoh_lease_priority") => {
                        if let Some(v) = parse_u32(value) {
                            config.zenoh_lease_priority =
                                Self::to_freertos_priority(v.min(31) as u8) as u8;
                        }
                    }
                    // ...and `[node.rt.freertos]` is RAW, mirroring
                    // `[tiers.<name>.freertos] priority`. One vocabulary for
                    // the two numbers that meet in one scheduler.
                    ("node.rt.freertos", "zenoh_lease_priority") => {
                        if let Some(v) = parse_u32(value) {
                            config.zenoh_lease_priority = v as u8;
                        }
                    }
                    ("node.rt", "zenoh_lease_stack_bytes") => {
                        if let Some(v) = parse_u32(value) {
                            config.zenoh_lease_stack_bytes = v;
                        }
                    }
                    // Issue 0623 — `[node.rt]` is the LEGACY normalized 0-31
                    // spelling and is converted on read, so configs written
                    // before the units were unified keep their schedule.
                    ("node.rt", "poll_priority") => {
                        if let Some(v) = parse_u32(value) {
                            config.poll_priority =
                                Self::to_freertos_priority(v.min(31) as u8) as u8;
                        }
                    }
                    // ...and `[node.rt.freertos]` is RAW, mirroring
                    // `[tiers.<name>.freertos] priority`. One vocabulary for
                    // the two numbers that meet in one scheduler.
                    ("node.rt.freertos", "poll_priority") => {
                        if let Some(v) = parse_u32(value) {
                            config.poll_priority = v as u8;
                        }
                    }
                    ("node.rt", "poll_interval_ms") => {
                        if let Some(v) = parse_u32(value) {
                            config.poll_interval_ms = v;
                        }
                    }
                    _ => {}
                }
            }
        }

        // Phase 177.38 — build-time ROS-domain override for per-fixture
        // isolation. `NROS_DOMAIN_ID` set at build time bakes a distinct domain
        // into this fixture without editing config.toml (the example default
        // stays clean). Cyclone derives RTPS ports from the domain, so
        // build-fixtures gives each communicating role-set its own domain and
        // concurrent fixtures don't collide. Empty/unset keeps the config value.
        if let Some(d) = option_env!("NROS_DOMAIN_ID").and_then(parse_u32) {
            config.base.domain_id = d;
        }

        config
    }
}

// ── Minimal no_std parsers ──────────────────────────────────────────────

/// Parse an IPv4 address string ("192.0.3.10") into [u8; 4].
fn parse_ipv4(s: &str) -> Option<[u8; 4]> {
    let mut result = [0u8; 4];
    let mut octet_idx = 0;
    let mut current: u16 = 0;
    let mut has_digit = false;

    for b in s.as_bytes() {
        match b {
            b'0'..=b'9' => {
                current = current * 10 + (*b - b'0') as u16;
                if current > 255 {
                    return None;
                }
                has_digit = true;
            }
            b'.' => {
                if !has_digit || octet_idx >= 3 {
                    return None;
                }
                result[octet_idx] = current as u8;
                octet_idx += 1;
                current = 0;
                has_digit = false;
            }
            _ => return None,
        }
    }

    if has_digit && octet_idx == 3 {
        result[3] = current as u8;
        Some(result)
    } else {
        None
    }
}

/// Parse a MAC address string ("02:00:00:00:00:00") into [u8; 6].
fn parse_mac(s: &str) -> Option<[u8; 6]> {
    let mut result = [0u8; 6];
    let mut byte_idx = 0;

    for part in s.split(':') {
        if byte_idx >= 6 || part.len() != 2 {
            return None;
        }
        result[byte_idx] = parse_hex_byte(part)?;
        byte_idx += 1;
    }

    if byte_idx == 6 { Some(result) } else { None }
}

/// Parse a two-character hex string ("0a") into a u8.
fn parse_hex_byte(s: &str) -> Option<u8> {
    let bytes = s.as_bytes();
    if bytes.len() != 2 {
        return None;
    }
    let hi = hex_digit(bytes[0])?;
    let lo = hex_digit(bytes[1])?;
    Some(hi * 16 + lo)
}

fn hex_digit(b: u8) -> Option<u8> {
    match b {
        b'0'..=b'9' => Some(b - b'0'),
        b'a'..=b'f' => Some(b - b'a' + 10),
        b'A'..=b'F' => Some(b - b'A' + 10),
        _ => None,
    }
}

/// Parse a decimal integer string.
fn parse_u32(s: &str) -> Option<u32> {
    let mut result: u32 = 0;
    let mut has_digit = false;
    for b in s.as_bytes() {
        match b {
            b'0'..=b'9' => {
                result = result.checked_mul(10)?.checked_add((*b - b'0') as u32)?;
                has_digit = true;
            }
            _ => return None,
        }
    }
    if has_digit { Some(result) } else { None }
}

// Phase 173.5 — nros.toml `[[transport]]` IP into the board `Config`
// (NanoRosOwned: this board owns the bundled lwIP stack). lwIP stores a
// dotted netmask, so the prefix is expanded; no UART field ⇒ baudrate
// keeps the trait's no-op default.
