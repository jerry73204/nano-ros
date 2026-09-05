//! Zenoh key expression generation for RMW types
//!
//! Extension traits that add zenoh-specific key expression methods
//! to the middleware-agnostic `TopicInfo`, `ServiceInfo`, and `QoSProfile` types.

use nros_rmw::{
    QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy, ServiceInfo, TopicInfo,
};

/// Extension trait for generating zenoh key expressions from `TopicInfo`
pub trait TopicKeyExpr {
    /// Generate the full topic key in rmw_zenoh format
    ///
    /// - Humble (default): `<domain_id>/<topic_name>/<type_name>/TypeHashNotSupported`
    /// - Iron/Jazzy/Rolling (`ros-{iron,jazzy,rolling}`): `<domain_id>/<topic_name>/<type_name>/<type_hash>`
    fn to_key<const N: usize>(&self) -> heapless::String<N>;

    /// Generate a wildcard topic key for subscribing
    /// Format: `<domain_id>/<topic_name>/<type_name>/*`
    fn to_key_wildcard<const N: usize>(&self) -> heapless::String<N>;
}

impl TopicKeyExpr for TopicInfo<'_> {
    fn to_key<const N: usize>(&self) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let topic_stripped = self.name.trim_matches('/');
        #[cfg(not(any(feature = "ros-iron", feature = "ros-jazzy")))]
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/{}/TypeHashNotSupported",
                self.domain_id,
                topic_stripped,
                DdsTypeName(self.type_name)
            ),
        );
        #[cfg(any(feature = "ros-iron", feature = "ros-jazzy"))]
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/{}/{}",
                self.domain_id,
                topic_stripped,
                DdsTypeName(self.type_name),
                self.type_hash
            ),
        );
        key
    }

    fn to_key_wildcard<const N: usize>(&self) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let topic_stripped = self.name.trim_matches('/');
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/{}/*",
                self.domain_id,
                topic_stripped,
                DdsTypeName(self.type_name)
            ),
        );
        key
    }
}

/// Render a service type name in the DDS form rmw_zenoh puts on the wire.
///
/// nano-ros' raw service API takes the ROS-style `pkg/srv/Type` (every call
/// site in this tree passes that form, and it is what `ros2 service call`
/// prints), but rmw_zenoh keys carry the mangled `pkg::srv::dds_::Type_`.
/// Leaving it unmangled does not merely look wrong: `/` is a zenoh keyexpr
/// SEPARATOR, so `example_interfaces/srv/AddTwoInts` splits the key into three
/// extra segments and nothing lines up with the fields a peer expects. That is
/// why a service registered from this stack was invisible to `ros2 service
/// list` even with the domain correct (issue 0824).
///
/// Verified against a native `demo_nodes_cpp add_two_ints_server` on
/// rmw_zenoh, which declares:
///   `…/SS/%/%/add_two_ints_server/%add_two_ints/example_interfaces::srv::dds_::AddTwoInts_/…`
///
/// A name that already contains `::` is passed through untouched, so callers
/// that supply the mangled form (generated code does) are unaffected.
/// Anything that is not exactly `a/b/c` is passed through as well rather than
/// mangled into nonsense.
pub(crate) struct DdsTypeName<'a>(pub &'a str);

impl core::fmt::Display for DdsTypeName<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let s = self.0;
        if s.contains("::") {
            return f.write_str(s);
        }
        let mut parts = s.split('/');
        match (parts.next(), parts.next(), parts.next(), parts.next()) {
            (Some(pkg), Some(kind), Some(name), None)
                if !pkg.is_empty() && !kind.is_empty() && !name.is_empty() =>
            {
                // The action path derives sub-types by appending to the base
                // ("Fibonacci" -> "Fibonacci_SendGoal_"), so the name can
                // ALREADY end in the trailing underscore. Appending a second
                // one put "Fibonacci_SendGoal__" on the wire against a native
                // peer's "Fibonacci_SendGoal_".
                if name.ends_with('_') {
                    write!(f, "{pkg}::{kind}::dds_::{name}")
                } else {
                    write!(f, "{pkg}::{kind}::dds_::{name}_")
                }
            }
            _ => f.write_str(s),
        }
    }
}

/// Extension trait for generating zenoh key expressions from `ServiceInfo`
pub trait ServiceKeyExpr {
    /// Generate the service key in rmw_zenoh format
    ///
    /// - Humble (default): `<domain_id>/<service_name>/<type_name>/TypeHashNotSupported`
    /// - Iron/Jazzy/Rolling (`ros-{iron,jazzy,rolling}`): `<domain_id>/<service_name>/<type_name>/<type_hash>`
    fn to_key<const N: usize>(&self) -> heapless::String<N>;

    /// Generate a wildcard service key for client queries
    /// Format: `<domain_id>/<service_name>/<type_name>/*`
    fn to_key_wildcard<const N: usize>(&self) -> heapless::String<N>;
}

impl ServiceKeyExpr for ServiceInfo<'_> {
    fn to_key<const N: usize>(&self) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let service_stripped = self.name.trim_matches('/');
        #[cfg(not(any(feature = "ros-iron", feature = "ros-jazzy")))]
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/{}/TypeHashNotSupported",
                self.domain_id,
                service_stripped,
                DdsTypeName(self.type_name)
            ),
        );
        #[cfg(any(feature = "ros-iron", feature = "ros-jazzy"))]
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/{}/{}",
                self.domain_id,
                service_stripped,
                DdsTypeName(self.type_name),
                self.type_hash
            ),
        );
        key
    }

    fn to_key_wildcard<const N: usize>(&self) -> heapless::String<N> {
        let mut key = heapless::String::new();
        let service_stripped = self.name.trim_matches('/');
        let _ = core::fmt::write(
            &mut key,
            format_args!(
                "{}/{}/{}/*",
                self.domain_id,
                service_stripped,
                DdsTypeName(self.type_name)
            ),
        );
        key
    }
}

/// Extension trait for generating zenoh QoS liveliness strings from `QoSProfile`
pub trait QosKeyExpr {
    /// Convert QoS settings to rmw_zenoh liveliness token format string.
    ///
    /// Format: `reliability:durability:history,depth:deadline:lifespan:liveliness,lease:avoid_ros_namespace_conventions`
    ///
    /// rmw_zenoh encoding:
    /// - reliability: SYSTEM_DEFAULT=0, RELIABLE=1, BEST_EFFORT=2
    /// - durability: SYSTEM_DEFAULT=0, TRANSIENT_LOCAL=1, VOLATILE=2
    /// - history: SYSTEM_DEFAULT=0, KEEP_LAST=1, KEEP_ALL=2
    fn to_qos_string<const N: usize>(&self) -> heapless::String<N>;
}

impl QosKeyExpr for QoSProfile {
    fn to_qos_string<const N: usize>(&self) -> heapless::String<N> {
        let mut s = heapless::String::new();

        // issue 0829 — the sentinel gets upstream's own digit, 0, rather than
        // being folded onto a concrete policy. It should never arrive here:
        // `ZenohSession::create_{publisher,subscription}` resolve before
        // deriving anything from the profile, precisely so a ROS peer parsing
        // this token out of the graph never reads `0:0:0,0` as a policy. Emit
        // the honest digit anyway, so a leak reads as "unstated" to that peer
        // instead of as a reliability we never chose.
        let reliability = match self.reliability {
            QoSReliabilityPolicy::SystemDefault => 0,
            QoSReliabilityPolicy::Reliable => 1,
            QoSReliabilityPolicy::BestEffort => 2,
        };

        let durability = match self.durability {
            QoSDurabilityPolicy::SystemDefault => 0,
            QoSDurabilityPolicy::TransientLocal => 1,
            QoSDurabilityPolicy::Volatile => 2,
        };

        let history = match self.history {
            QoSHistoryPolicy::SystemDefault => 0,
            QoSHistoryPolicy::KeepLast => 1,
            QoSHistoryPolicy::KeepAll => 2,
        };

        let _ = core::fmt::write(
            &mut s,
            format_args!(
                "{}:{}:{},{}:,:,:,,",
                reliability, durability, history, self.depth
            ),
        );

        s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    #[cfg(not(any(feature = "ros-iron", feature = "ros-jazzy")))]
    fn test_topic_key_generation() {
        let topic =
            TopicInfo::new("/chatter", "std_msgs::msg::dds_::String_", "abc123").with_domain(42);
        let key: heapless::String<128> = topic.to_key();
        assert!(key.contains("42"));
        assert!(key.contains("chatter"));
        assert!(key.contains("TypeHashNotSupported"));
    }

    #[test]
    #[cfg(any(feature = "ros-iron", feature = "ros-jazzy"))]
    fn test_topic_key_generation_iron() {
        let topic = TopicInfo::new("/chatter", "std_msgs::msg::dds_::String_", "RIHS01_abc123")
            .with_domain(42);
        let key: heapless::String<128> = topic.to_key();
        assert!(key.contains("42"));
        assert!(key.contains("chatter"));
        assert!(key.contains("RIHS01_abc123"));
        assert!(!key.contains("TypeHashNotSupported"));
    }

    #[test]
    #[cfg(not(any(feature = "ros-iron", feature = "ros-jazzy")))]
    fn test_topic_info_to_key_humble() {
        let topic = TopicInfo::new(
            "/chatter",
            "std_msgs::msg::dds_::Int32_",
            "TypeHashNotSupported",
        );
        let key: heapless::String<128> = topic.to_key();
        assert_eq!(
            key.as_str(),
            "0/chatter/std_msgs::msg::dds_::Int32_/TypeHashNotSupported"
        );
    }

    #[test]
    #[cfg(any(feature = "ros-iron", feature = "ros-jazzy"))]
    fn test_topic_info_to_key_iron() {
        let topic = TopicInfo::new("/chatter", "std_msgs::msg::dds_::Int32_", "RIHS01_deadbeef");
        let key: heapless::String<128> = topic.to_key();
        assert_eq!(
            key.as_str(),
            "0/chatter/std_msgs::msg::dds_::Int32_/RIHS01_deadbeef"
        );
    }

    #[test]
    fn test_topic_info_to_key_wildcard() {
        let topic = TopicInfo::new(
            "/chatter",
            "std_msgs::msg::dds_::Int32_",
            "TypeHashNotSupported",
        );
        let key: heapless::String<128> = topic.to_key_wildcard();
        assert_eq!(key.as_str(), "0/chatter/std_msgs::msg::dds_::Int32_/*");
    }

    #[test]
    #[cfg(not(any(feature = "ros-iron", feature = "ros-jazzy")))]
    fn test_service_key() {
        let service = ServiceInfo::new(
            "/add_two_ints",
            "example_interfaces::srv::dds_::AddTwoInts",
            "TypeHashNotSupported",
        );
        let key: heapless::String<128> = service.to_key();
        assert!(key.contains("add_two_ints"));
        assert!(key.contains("TypeHashNotSupported"));
    }

    #[test]
    #[cfg(any(feature = "ros-iron", feature = "ros-jazzy"))]
    fn test_service_key_iron() {
        let service = ServiceInfo::new(
            "/add_two_ints",
            "example_interfaces::srv::dds_::AddTwoInts",
            "RIHS01_cafebabe",
        );
        let key: heapless::String<128> = service.to_key();
        assert!(key.contains("add_two_ints"));
        assert!(key.contains("RIHS01_cafebabe"));
        assert!(!key.contains("TypeHashNotSupported"));
    }

    #[test]
    fn test_qos_string_sensor_data() {
        let qos = QoSProfile::QOS_PROFILE_SENSOR_DATA;
        let s: heapless::String<32> = qos.to_qos_string();
        assert_eq!(s.as_str(), "2:2:1,5:,:,:,,");
    }

    #[test]
    fn test_qos_string_default() {
        let qos = QoSProfile::QOS_PROFILE_DEFAULT;
        let s: heapless::String<32> = qos.to_qos_string();
        assert_eq!(s.as_str(), "1:2:1,10:,:,:,,");
    }

    /// issue 0793 — this asserted `1:1:1` (TransientLocal) while testing
    /// `QOS_PROFILE_PARAMETERS`, so it pinned the very defect 0793 fixed: that
    /// profile disagreed with upstream's `rmw_qos_profile_parameters`, which is
    /// VOLATILE. Two tests pinned it, in two crates; correcting the constant and
    /// only the `nros-rmw` test left this one red in tier 2.
    #[test]
    fn test_qos_string_parameters_is_volatile() {
        let qos = QoSProfile::QOS_PROFILE_PARAMETERS;
        let s: heapless::String<32> = qos.to_qos_string();
        assert_eq!(s.as_str(), "1:2:1,1000:,:,:,,");
    }

    /// Keeps the TransientLocal ENCODING covered, which the rename above would
    /// otherwise drop: durability 1 vs 2 is the only thing separating these two
    /// strings, and nothing else asserts the 1.
    #[test]
    fn test_qos_string_transient_local() {
        let qos = QoSProfile::QOS_PROFILE_ACTION_STATUS_DEFAULT;
        let s: heapless::String<32> = qos.to_qos_string();
        assert_eq!(s.as_str(), "1:1:1,1:,:,:,,");
    }

    /// The KEEP_ALL encoding (history digit 2), built rather than borrowed.
    ///
    /// phase-428 W10 — this read `QOS_PROFILE_PARAMETER_EVENTS`, which was
    /// KEEP_ALL only because that preset was WRONG: upstream
    /// `rmw_qos_profile_parameter_events` is KEEP_LAST(1000). So this test both
    /// depended on the defect and helped hide it, and correcting the preset
    /// turned it red — the identical two-crate shape issue 0793 hit four
    /// entries up, which its own comment already describes ("Two tests pinned
    /// it, in two crates").
    ///
    /// The fix is not a new preset to borrow: no NAMED preset is KEEP_ALL now,
    /// and a test about an ENCODING should state the value it encodes.
    #[test]
    fn test_qos_string_keep_all() {
        let qos = QoSProfile::new().keep_all().depth(0);
        let s: heapless::String<32> = qos.to_qos_string();
        assert_eq!(s.as_str(), "1:2:2,0:,:,:,,");
    }

    #[test]
    fn test_qos_string_custom() {
        let qos = QoSProfile::new()
            .best_effort()
            .transient_local()
            .keep_all()
            .depth(42);
        let s: heapless::String<32> = qos.to_qos_string();
        assert_eq!(s.as_str(), "2:1:2,42:,:,:,,");
    }
}
