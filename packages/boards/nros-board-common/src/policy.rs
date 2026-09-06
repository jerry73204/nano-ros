//! Link-feature env reader + per-platform policy mask.
//!
//! Phase 134.2 introduced the `LinkFeatures` env reader + the
//! `LinkPolicy` mask that overrides per-platform invariants
//! (Orin SPE has no Ethernet → `Force(false)` masks TCP / UDP /
//! MC / SERIAL / TLS, etc.). Phase 136.2 extracted it from
//! `zpico-sys/build.rs` into `zpico-sys/build/policy.rs`. Phase
//! 152.5 lifted it into the `nros-board-common` library so the
//! per-kernel generic board crates can share one canonical
//! implementation alongside the manifest parser.
//!
//! Use from `build.rs`:
//! ```ignore
//! use nros_board_common::policy::{LinkFeatures, LinkPolicy};
//! let link = LinkFeatures::from_env().apply(&LinkPolicy::posix());
//! ```

use std::{collections::BTreeMap, env};

/// Protocol link features read from Cargo feature flags.
///
/// Each field corresponds to a `link-*` Cargo feature that controls
/// the matching `Z_FEATURE_LINK_*` flag passed to zenoh-pico at compile time.
pub struct LinkFeatures {
    pub tcp: bool,
    pub udp_unicast: bool,
    pub udp_multicast: bool,
    pub serial: bool,
    pub raweth: bool,
    pub tls: bool,
    // Phase 100.4 — NVIDIA Tegra IVC link transport.
    pub ivc: bool,
    // Phase 115.B — runtime-pluggable user transport.
    pub custom: bool,
    // RFC-0080 — CAN / CAN FD link transport.
    pub can: bool,
    // RFC-0083 — CAN unicast over ISO-TP. Separate from `can` because they are
    // different links, not two modes of one: `can` is multicast and carries
    // pushed data only, `isotp` is a unicast face and also carries queries and
    // liveliness -- ROS services, actions, parameters and graph introspection.
    pub isotp: bool,
}

impl LinkFeatures {
    /// Read link features from Cargo environment variables.
    ///
    /// Phase 128.E.1 made `tcp`, `udp_unicast`, `udp_multicast`, `serial`
    /// always-on (one binary serves any locator, picked at session-open). Phase
    /// 204.7 re-gates the **IP** trio (`tcp`/`udp_unicast`/`udp_multicast`) on
    /// `zpico-sys`'s `link-ip` feature — **default-ON**, so the 128.E.1 behaviour
    /// is unchanged for every existing build; a serial-only board drops `link-ip`
    /// to shed the IP-link C entirely (size). `serial` stays unconditionally on
    /// (no build-host dep, trivial framing).
    ///
    /// `raweth`, `tls`, `ivc`, `custom` stay explicit because each
    /// carries a real build-host requirement (raw-socket capability,
    /// mbedTLS / OpenSSL provider, NVIDIA IVC headers, the
    /// `zpico-platform-custom` crate).
    pub fn from_env() -> Self {
        // Phase 204.7 — IP link (TCP/UDP) gated on `zpico-sys`'s `link-ip`
        // feature (**default-ON** → unchanged for every existing build; 128.E.1's
        // always-on default preserved), with a per-build env override
        // `NROS_LINK_IP=0` for a serial-only node to shed it. When off, the vendor
        // IP-link C isn't compiled (`Z_FEATURE_LINK_{TCP,UDP_*}=0`) and
        // `--gc-sections` (204.8) strips the now-unreferenced smoltcp platform
        // impl — no cargo `default-features=false` dance (which would also drop the
        // staticlib `linkme-register` opt-out). Matches the `NROS_SMOLTCP_*`
        // per-example env pattern.
        let ip = env::var("CARGO_FEATURE_LINK_IP").is_ok()
            && !matches!(
                env::var("NROS_LINK_IP").ok().as_deref(),
                Some("0") | Some("false") | Some("off")
            );
        Self {
            tcp: ip,
            udp_unicast: ip,
            udp_multicast: ip,
            serial: true,
            raweth: env::var("CARGO_FEATURE_LINK_RAWETH").is_ok(),
            tls: env::var("CARGO_FEATURE_LINK_TLS").is_ok(),
            ivc: env::var("CARGO_FEATURE_LINK_IVC").is_ok(),
            custom: env::var("CARGO_FEATURE_LINK_CUSTOM").is_ok(),
            can: env::var("CARGO_FEATURE_LINK_CAN").is_ok(),
            isotp: env::var("CARGO_FEATURE_LINK_ISOTP").is_ok(),
        }
    }

    /// Phase 134.2 — apply a platform-invariant policy mask. Each
    /// `PolicyChoice` value either forces the field to a literal (SPE
    /// has no Ethernet → `Force(false)` masks TCP/UDP/MC/SERIAL/TLS)
    /// or lets the upstream `LinkFeatures::from_env()` value through
    /// (`Follow`). Constructor matches what the previous per-build-fn
    /// `build.define("Z_FEATURE_LINK_*", "0")` literals encoded, just
    /// in declarative form.
    pub fn apply(mut self, policy: &LinkPolicy) -> Self {
        self.tcp = policy.tcp.resolve(self.tcp);
        self.udp_unicast = policy.udp_unicast.resolve(self.udp_unicast);
        self.udp_multicast = policy.udp_multicast.resolve(self.udp_multicast);
        self.serial = policy.serial.resolve(self.serial);
        self.raweth = policy.raweth.resolve(self.raweth);
        self.tls = policy.tls.resolve(self.tls);
        self.ivc = policy.ivc.resolve(self.ivc);
        self.custom = policy.custom.resolve(self.custom);
        self.can = policy.can.resolve(self.can);
        self.isotp = policy.isotp.resolve(self.isotp);
        self
    }

    pub fn tcp_flag(&self) -> u8 {
        self.tcp as u8
    }
    pub fn udp_unicast_flag(&self) -> u8 {
        self.udp_unicast as u8
    }
    pub fn udp_multicast_flag(&self) -> u8 {
        self.udp_multicast as u8
    }
    pub fn serial_flag(&self) -> u8 {
        self.serial as u8
    }
    pub fn raweth_flag(&self) -> u8 {
        self.raweth as u8
    }
    pub fn tls_flag(&self) -> u8 {
        self.tls as u8
    }
    pub fn ivc_flag(&self) -> u8 {
        self.ivc as u8
    }
    pub fn custom_flag(&self) -> u8 {
        self.custom as u8
    }
    pub fn can_flag(&self) -> u8 {
        self.can as u8
    }
    pub fn isotp_flag(&self) -> u8 {
        self.isotp as u8
    }
}

/// Phase 134.2 — per-platform link-feature policy mask.
///
/// Layered on top of `LinkFeatures::from_env()`. Each field is a
/// `PolicyChoice`: `Force(bool)` overrides the env-derived value;
/// `Follow` lets it through. Replaces the eight functions' worth of
/// scattered `build.define("Z_FEATURE_LINK_*", "0")` literals with
/// one declarative table per platform.
#[derive(Copy, Clone)]
pub enum PolicyChoice {
    Force(bool),
    Follow,
}

impl PolicyChoice {
    pub fn resolve(self, env_value: bool) -> bool {
        match self {
            PolicyChoice::Force(v) => v,
            PolicyChoice::Follow => env_value,
        }
    }
}

#[derive(Copy, Clone)]
pub struct LinkPolicy {
    pub tcp: PolicyChoice,
    pub udp_unicast: PolicyChoice,
    pub udp_multicast: PolicyChoice,
    pub serial: PolicyChoice,
    pub raweth: PolicyChoice,
    pub tls: PolicyChoice,
    pub ivc: PolicyChoice,
    pub custom: PolicyChoice,
    // RFC-0080 — CAN / CAN FD.
    pub can: PolicyChoice,
    // RFC-0083 — CAN unicast over ISO-TP.
    pub isotp: PolicyChoice,
}

impl LinkPolicy {
    /// All-`Follow` baseline: every flag tracks Cargo env exactly.
    /// Used by every platform whose network stack supports the full
    /// set of transports (FreeRTOS+lwIP, NuttX, ThreadX/NetX,
    /// bare-metal/smoltcp).
    pub const fn passthrough() -> Self {
        Self {
            tcp: PolicyChoice::Follow,
            udp_unicast: PolicyChoice::Follow,
            udp_multicast: PolicyChoice::Follow,
            serial: PolicyChoice::Follow,
            raweth: PolicyChoice::Follow,
            tls: PolicyChoice::Follow,
            ivc: PolicyChoice::Follow,
            custom: PolicyChoice::Follow,
            can: PolicyChoice::Follow,
            isotp: PolicyChoice::Follow,
        }
    }

    /// POSIX policy — same as passthrough today. Phase 134.7 adds
    /// the missing multicast aliases in `platform_aliases.c`; until
    /// that lands the linker still fails on `_z_read_udp_multicast`
    /// if multicast is on, but `LinkFeatures::from_env()` already
    /// hardcodes `udp_multicast=true` so the policy can't paper
    /// over the gap.
    pub const fn posix() -> Self {
        Self::passthrough()
    }

    /// IVC-only — no Ethernet, no serial, no TLS; the link layer is an
    /// inter-processor mailbox and only IVC + custom transports are valid.
    ///
    /// phase-337 W7.b — was `orin_spe()`, named for the one board that used it.
    /// The board is gone; the POLICY is not board-specific (any AMP mailbox
    /// target has this shape), so it keeps the capability name instead.
    ///
    /// issue 1143 — selected by [`LinkPolicy::for_board`] from the board's
    /// `capabilities.ip_stack = false`, which is what the old comment here
    /// ("`nros-zpico-build` picks a policy per PLATFORM, and IVC-only has no
    /// in-tree platform") was waiting for. The rung is per-BOARD because the
    /// fact is: a mailbox-only board sits in the same FreeRTOS family as an
    /// lwIP one.
    pub const fn ivc_only() -> Self {
        Self {
            tcp: PolicyChoice::Force(false),
            udp_unicast: PolicyChoice::Force(false),
            udp_multicast: PolicyChoice::Force(false),
            serial: PolicyChoice::Force(false),
            raweth: PolicyChoice::Force(false),
            tls: PolicyChoice::Force(false),
            ivc: PolicyChoice::Follow,
            custom: PolicyChoice::Follow,
            can: PolicyChoice::Follow,
            isotp: PolicyChoice::Follow,
        }
    }

    /// Phase 146.2 — FreeRTOS + lwIP. zenoh-pico's
    /// `src/system/freertos/lwip/network.c` ships an explicit
    /// `#error "Serial not supported yet on FreeRTOS + LWIP port"`
    /// behind `Z_FEATURE_LINK_SERIAL == 1`, and no FreeRTOS user
    /// has wired a serial backend through `zpico-serial`. Force
    /// serial off so neither the upstream `#error` fires nor
    /// `src/system/common/serial.c` builds and emits unresolved
    /// `_z_*_serial_internal` calls. Same shape forces raweth and
    /// TLS off because no backend exists for them on FreeRTOS.
    pub const fn freertos_lwip() -> Self {
        Self {
            tcp: PolicyChoice::Follow,
            udp_unicast: PolicyChoice::Follow,
            // Phase 154 — vendor `system/freertos/lwip/network.c`
            // line 780 has a typo (`sockrecv->socket` for
            // `_z_close_udp_multicast`; field is `_socket`).
            // nano-ros doesn't use UDP multicast over zenoh-pico
            // on FreeRTOS+lwIP (router is TCP-only), so force
            // the feature off to stop the vendor typo from
            // firing once we start compiling
            // `system/freertos/lwip` (Phase 154 added it to the
            // manifest's `include`).
            udp_multicast: PolicyChoice::Force(false),
            serial: PolicyChoice::Force(false),
            raweth: PolicyChoice::Force(false),
            tls: PolicyChoice::Force(false),
            ivc: PolicyChoice::Follow,
            custom: PolicyChoice::Follow,
            can: PolicyChoice::Follow,
            isotp: PolicyChoice::Follow,
        }
    }

    /// Phase 146.2 — NuttX. Same shape as `freertos_lwip()`: NuttX
    /// ships no serial backend, no raweth, no TLS provider; forcing
    /// these features on under `LinkFeatures::from_env`'s Phase
    /// 128.E.1 "always-on" defaults causes `_z_*_serial_internal`
    /// to surface as link-time undefined symbols on every NuttX
    /// Rust example build.
    pub const fn nuttx() -> Self {
        Self {
            tcp: PolicyChoice::Follow,
            udp_unicast: PolicyChoice::Follow,
            udp_multicast: PolicyChoice::Follow,
            serial: PolicyChoice::Force(false),
            raweth: PolicyChoice::Force(false),
            tls: PolicyChoice::Force(false),
            ivc: PolicyChoice::Follow,
            custom: PolicyChoice::Follow,
            can: PolicyChoice::Follow,
            isotp: PolicyChoice::Follow,
        }
    }

    /// Phase 146.2 — ThreadX (both Linux sim and RV64 board).
    /// `c/platform/threadx/network.c` ships TCP/UDP/serial code over
    /// NetX Duo BSD but is NOT listed under `[platform.threadx]
    /// extra_sources` — threadx uses `platform_aliases.c` for
    /// network ops instead, and the alias TU has no serial wrapper.
    /// Force serial off to match: every ThreadX example uses
    /// TCP/UDP over NetX Duo BSD, none use serial.
    pub const fn threadx() -> Self {
        Self {
            tcp: PolicyChoice::Follow,
            udp_unicast: PolicyChoice::Follow,
            udp_multicast: PolicyChoice::Follow,
            serial: PolicyChoice::Force(false),
            raweth: PolicyChoice::Force(false),
            tls: PolicyChoice::Force(false),
            ivc: PolicyChoice::Follow,
            custom: PolicyChoice::Follow,
            can: PolicyChoice::Follow,
            isotp: PolicyChoice::Follow,
        }
    }

    /// issue 1143 — the BOARD rung of policy selection, applied on top of the
    /// per-PLATFORM baseline the caller already chose.
    ///
    /// The fact is `capabilities.ip_stack`, RFC-0086's vocabulary, resolved
    /// through `PlatformsTree::capabilities_with_board` so the board's own
    /// `[capabilities]` outranks its platform's. `false` means this image has
    /// no IP stack at all — no MAC, no PHY, no socket layer — and the only
    /// links that can exist are the mailbox ones, which is exactly
    /// [`LinkPolicy::ivc_only`].
    ///
    /// **Not `supported_netstacks = []`, and this is the whole finding of
    /// 1143.** That list says WHICH in-image stack a board can be built with,
    /// and four in-tree boards declare it empty for three different reasons:
    /// `linux` and `freertos-posix` because the HOST kernel owns the stack
    /// (sockets exist, they are just not ours to select), the two `nuttx-qemu`
    /// boards and `zephyr` because the RTOS owns it (likewise), and a
    /// mailbox-only board would because there is genuinely nothing. Reading
    /// `[]` as "no links" would have forced TCP off on every one of those and
    /// broken every native and NuttX zenoh build — a working-board re-policy
    /// dressed as a fix. `ip_stack` separates "we do not choose the stack"
    /// from "there is no stack".
    ///
    /// **Absent is not false.** A capability neither the platform nor the
    /// board declares leaves the platform baseline alone, so a build with no
    /// board rung at all (a plain `cargo build -p zpico-sys`, an out-of-tree
    /// consumer) resolves byte-identically to what it did before this existed.
    /// `nros-platform-freertos` and `-posix` / `-nuttx` / `-zephyr` all
    /// declare `ip_stack = true`, so every in-tree board inherits an explicit
    /// `true` unless it says otherwise, and no in-tree board says otherwise.
    pub fn for_board(platform_default: Self, caps: &BTreeMap<String, bool>) -> Self {
        match caps.get(IP_STACK) {
            Some(false) => Self::ivc_only(),
            // `Some(true)` and absent alike: the platform's own policy stands.
            _ => platform_default,
        }
    }
}

/// The RFC-0086 capability that decides whether a socket-based link can exist
/// in this image. One spelling, shared by the policy selector above and by the
/// `[build.zenoh] defines_conditional` / `extra_sources` rows that pick the
/// matching zenoh-pico `system/` TU set — the two halves of issue 1143 have to
/// agree about the fact or they pick a socket type and a link policy that
/// disagree, which is the 0135 class.
pub const IP_STACK: &str = "ip_stack";

#[cfg(test)]
mod for_board_tests {
    use super::*;

    fn caps(pairs: &[(&str, bool)]) -> BTreeMap<String, bool> {
        pairs.iter().map(|(k, v)| (k.to_string(), *v)).collect()
    }

    fn shape(p: &LinkPolicy) -> [bool; 4] {
        [
            p.tcp.resolve(true),
            p.udp_unicast.resolve(true),
            p.serial.resolve(true),
            p.ivc.resolve(true),
        ]
    }

    /// A board declaring `ip_stack = false` is the ONLY thing that reaches
    /// `ivc_only()` — the state that had no selector at all before 1143.
    #[test]
    fn a_board_with_no_ip_stack_selects_ivc_only() {
        let got = LinkPolicy::for_board(LinkPolicy::freertos_lwip(), &caps(&[("ip_stack", false)]));
        assert_eq!(shape(&got), shape(&LinkPolicy::ivc_only()));
        assert!(!got.tcp.resolve(true), "TCP must be forced off");
        assert!(got.ivc.resolve(true), "IVC must still follow the feature");
    }

    /// The regression this change most has to avoid: `mps2-an385-freertos`
    /// inherits the freertos platform's `ip_stack = true`, and must come out
    /// of the selector holding the same policy it held before.
    #[test]
    fn a_board_with_an_ip_stack_keeps_its_platform_policy() {
        let got = LinkPolicy::for_board(LinkPolicy::freertos_lwip(), &caps(&[("ip_stack", true)]));
        assert_eq!(shape(&got), shape(&LinkPolicy::freertos_lwip()));
        assert!(got.tcp.resolve(true));
        assert!(
            !got.serial.resolve(true),
            "freertos still forces serial off"
        );
    }

    /// Absent is not false — a plain `cargo build` with no board rung, and an
    /// out-of-tree platform nobody has described, both keep the baseline.
    #[test]
    fn an_undeclared_capability_changes_nothing() {
        for baseline in [
            LinkPolicy::posix(),
            LinkPolicy::freertos_lwip(),
            LinkPolicy::nuttx(),
            LinkPolicy::threadx(),
        ] {
            let got = LinkPolicy::for_board(baseline, &BTreeMap::new());
            assert_eq!(shape(&got), shape(&baseline));
            let other = LinkPolicy::for_board(baseline, &caps(&[("heap", false)]));
            assert_eq!(shape(&other), shape(&baseline));
        }
    }

    /// `supported_netstacks = []` is NOT the predicate: were it, the `linux`,
    /// `freertos-posix`, `nuttx-qemu-*` and `zephyr` boards — every one of
    /// which declares the empty list and every one of which has working
    /// sockets — would lose TCP. Pinned here because the plausible-but-wrong
    /// rule is the one a reader reaches for first.
    /// The ladder end to end: a real `nros-board.toml` on disk, resolved
    /// against a platform declaring `ip_stack = true`, arriving at a
    /// `LinkPolicy`. The unit cases above pin the selector; this pins the
    /// path a build actually walks — `NROS_BOARD_TOML` -> `BoardKnobsFile` ->
    /// `capabilities_with_board` -> `for_board` — because every rung of it is
    /// somewhere the fact could be dropped instead of read.
    #[test]
    fn the_board_rung_resolves_from_a_board_toml_on_disk() {
        use crate::platform_config::{BoardKnobsFile, PlatformsTree};

        let dir = tempfile::tempdir().expect("tempdir");
        // The freertos platform's own claim, mirrored rather than read: the
        // real manifest is checked by `check-capability-conditionals` rule 5,
        // and a test that reads a file another change is editing is a flake.
        std::fs::create_dir_all(dir.path().join("freertos")).unwrap();
        std::fs::write(
            dir.path().join("freertos/nros-platform.toml"),
            "[capabilities]\nip_stack = true\n",
        )
        .unwrap();
        let tree = PlatformsTree::load_search_path(&[dir.path().to_path_buf()]).expect("tree");

        let resolve = |board: Option<&BoardKnobsFile>, name: Option<&str>| {
            let caps = tree
                .capabilities_with_board("freertos", board, name)
                .expect("capabilities");
            (
                caps.get(IP_STACK).copied(),
                LinkPolicy::for_board(LinkPolicy::freertos_lwip(), &caps)
                    .tcp
                    .resolve(true),
            )
        };

        // No board rung at all — a plain `cargo build -p zpico-sys`.
        assert_eq!(resolve(None, None), (Some(true), true));

        // An lwIP board: says nothing about `ip_stack`, inherits the
        // platform's `true`, keeps TCP. This is `mps2-an385-freertos`'s shape,
        // and the regression to avoid.
        let boards = dir.path().join("boards");
        std::fs::create_dir_all(&boards).unwrap();
        let lwip = boards.join("lwip-board.toml");
        std::fs::write(
            &lwip,
            "[[board]]\nnames = [\"lwip-board\"]\nplatform = \"freertos\"\n\
             supported_netstacks = [\"lwip\"]\n\n[board.capabilities]\nheap = true\n",
        )
        .unwrap();
        let lwip = BoardKnobsFile::load(&lwip).expect("load lwip board");
        assert_eq!(resolve(Some(&lwip), Some("lwip-board")), (Some(true), true));

        // A host-owned-stack board: `supported_netstacks = []`, and STILL has
        // sockets. `freertos-posix` and `linux` are both this shape; reading
        // the empty list as "no links" would have taken TCP off them.
        let hosted = boards.join("hosted-board.toml");
        std::fs::write(
            &hosted,
            "[[board]]\nnames = [\"hosted-board\"]\nplatform = \"freertos\"\n\
             supported_netstacks = []\n",
        )
        .unwrap();
        let hosted = BoardKnobsFile::load(&hosted).expect("load hosted board");
        assert_eq!(
            resolve(Some(&hosted), Some("hosted-board")),
            (Some(true), true),
            "an empty `supported_netstacks` must not move the policy"
        );

        // A mailbox-only board: declares the fact, and only this reaches
        // `ivc_only()`.
        let mailbox = boards.join("mailbox-board.toml");
        std::fs::write(
            &mailbox,
            "[[board]]\nnames = [\"mailbox-board\"]\nplatform = \"freertos\"\n\
             supported_netstacks = []\n\n\
             [board.capabilities]\nheap = true\nip_stack = false\n",
        )
        .unwrap();
        let mailbox = BoardKnobsFile::load(&mailbox).expect("load mailbox board");
        assert_eq!(
            resolve(Some(&mailbox), Some("mailbox-board")),
            (Some(false), false)
        );

        // And the fact travels in the file's OWN board block, so a build that
        // did not export `NROS_BOARD` still finds it when the file declares
        // exactly one board — the common case, and the one an out-of-tree
        // consumer is in.
        assert_eq!(resolve(Some(&mailbox), None), (Some(false), false));
    }

    #[test]
    fn an_empty_netstack_list_is_not_the_predicate() {
        // What such a board's capability map actually looks like: the stack is
        // the host's or the RTOS's, so `ip_stack` is true and no netstack is
        // ours to select.
        let host_owned = caps(&[("ip_stack", true)]);
        let got = LinkPolicy::for_board(LinkPolicy::posix(), &host_owned);
        assert!(
            got.tcp.resolve(true),
            "a host-owned stack must keep TCP; reading `supported_netstacks = []` \
             as `no links` is what this asserts against"
        );
    }
}
