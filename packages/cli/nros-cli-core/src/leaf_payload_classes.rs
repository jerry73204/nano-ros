//! Issue 1125 — the zenoh payload classes for a CARGO LEAF, derived from what
//! it subscribes to.
//!
//! # What was missing
//!
//! Issue 0827 gave a cargo leaf a derived pool budget, and
//! [`crate::leaf_entity_env`] renders it into a gitignored `[env]` sidecar. That
//! budget is computed from the ENTITY inventory alone, which counts
//! subscriptions and knows nothing about their SIZE — so it can answer
//! `ZPICO_MAX_SUBSCRIBERS` and cannot answer `ZPICO_MAX_LARGE_SUBSCRIBERS`.
//!
//! "Large" is not a property of the declaration. It is a property of the TYPE's
//! `rx` bound relative to the class split
//! ([`rosidl_codegen::bounds::DEFAULT_SMALL_CLASS_CEILING`]), which is the
//! MESSAGE-BOUND inventory's question. The CMake lane joins the two
//! (`_nros_bounds_join_subscribed` in `cmake/NanoRosMessageBounds.cmake`); this
//! module is the same join for a leaf that has no CMake at all.
//!
//! Measured cost of not having it: `examples/native/rust/talker`, a node with
//! ONE publisher and NO subscription, links
//! `nros_rmw_zenoh::shim::subscriber::LARGE_PAYLOADS` at **131,072 B** — the
//! crate default `MAX_LARGE_SUBSCRIBERS` (2) × `SUBSCRIBER_RING_DEPTH` (4) ×
//! `SUBSCRIBER_LARGE_SIZE` (16,384) — for a class it can never route a single
//! subscription into.
//!
//! # Where the bounds come from
//!
//! `nros sync` runs codegen BEFORE it renders the sidecar, and codegen writes
//! [`rosidl_codegen::INVENTORY_JSON_NAME`] into every generated package. So the
//! join key and the join table are both on disk beside the leaf by the time
//! this runs. Nothing is compiled to read them.
//!
//! # Why all three knobs move together
//!
//! The runtime routes on `min(ZPICO_SUBSCRIBER_SIZE_THRESHOLD,
//! NROS_SUBSCRIBER_BUFFER_SIZE)` (`SMALL_CLASS_CEILING` in
//! `shim/subscriber.rs`, issue 0841), NOT on the threshold alone. Publishing
//! the large COUNT while leaving the small BLOCK at its 1024-byte default would
//! classify a 1,500-byte subscribed type "small" here and route it LARGE at
//! runtime — into a class this derivation just declared empty, which
//! `alloc_payload_block` then refuses at `create_subscription`. Deriving the
//! small block from the same join is what makes the classification and the
//! routing the same predicate. That is exactly why
//! `_nros_bounds_publish_payload_classes` publishes the three together, and it
//! is the reason this module does not publish a subset.
//!
//! # Refusing
//!
//! Every failure is a REFUSAL that leaves the crate defaults standing, because
//! the defaults are LARGE. A pool short of what the image receives is not a
//! smaller pool — it is a `SubscriberCreationFailed` at registration.

use std::path::Path;

use rosidl_codegen::bounds::{BoundState, DEFAULT_SMALL_CLASS_CEILING, bounds_from_json};

use crate::entity_inventory::{EntityInventory, ReceivedTypes};

/// The directory `nros sync` writes generated message crates into.
///
/// The same name `cmd::ws::is_generated_path` keys on, and the default of
/// `nros sync --build-dir`.
const GENERATED_DIR: &str = "generated";

/// The zenoh payload classes for one leaf.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PayloadClasses {
    Derived(DerivedPayloadClasses),
    /// Nothing may be sized. The reason is reported once, at sync.
    Refused {
        reason: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct DerivedPayloadClasses {
    /// Subscribing ENTITIES whose type's `rx` bound is over the ceiling, so
    /// each needs its own `large` block. Entities, not distinct types: two
    /// subscriptions on one large type need two blocks, and a type count would
    /// under-reserve by exactly the duplicates.
    ///
    /// ZERO IS AN ANSWER, not an abstention — phase-403 W4 removed this knob's
    /// floor on the measured ground that `alloc_payload_block` bounds-checks
    /// the class index BEFORE subscripting `LARGE_PAYLOADS`, so a zero-length
    /// pool returns `None` and is never indexed. It is emitted unfloored, and
    /// [`crate::entity_inventory::c_array_pool_floor`] is deliberately NOT
    /// applied: that floor belongs to the fixed C arrays in `zpico.c`, and this
    /// is a Rust static that is legal at length zero (issues 1015 / 1033).
    pub large_count: usize,
    /// The largest `rx` among those, or 0 when there are none.
    pub large_max: usize,
    /// The largest `rx` AT OR UNDER the ceiling among the subscribed set, or 0
    /// when nothing subscribed fits under it (including "nothing subscribes").
    pub small_max: usize,
    /// How many subscribing entities the join saw. Reported, not sized from.
    pub subscribed: usize,
}

impl PayloadClasses {
    pub fn tag(&self) -> &'static str {
        match self {
            PayloadClasses::Derived(_) => "derived",
            PayloadClasses::Refused { .. } => "refused",
        }
    }
}

/// Every `pkg/msg/Name` -> bound row this leaf's `generated/` tree prices.
///
/// One level deep: `nros sync --build-dir generated` writes one directory per
/// ament package with the artifact at its root.
fn leaf_bound_inventory(leaf: &Path) -> Result<Vec<(String, BoundState)>, String> {
    let dir = leaf.join(GENERATED_DIR);
    let Ok(rd) = std::fs::read_dir(&dir) else {
        // No generated tree at all. Not an error here: a leaf with no message
        // dependency has none, and it also has no typed subscription to price.
        // The caller refuses on the LOOKUP MISS instead, which names the type.
        return Ok(Vec::new());
    };
    let mut entries: Vec<_> = rd.filter_map(|e| e.ok()).map(|e| e.path()).collect();
    entries.sort();
    let mut out = Vec::new();
    for pkg in entries {
        let path = pkg.join(rosidl_codegen::INVENTORY_JSON_NAME);
        if !path.is_file() {
            continue;
        }
        let text = std::fs::read_to_string(&path)
            .map_err(|e| format!("reading {}: {e}", path.display()))?;
        // A malformed or wrong-schema artifact REFUSES the whole leaf rather
        // than contributing a subset: a partial table turns a priced type into
        // an unpriced one, and the two refusals say different things.
        let rows = bounds_from_json(&text).map_err(|e| format!("{}: {e}", path.display()))?;
        out.extend(rows.into_iter().map(|r| (r.type_name, r.bound)));
    }
    Ok(out)
}

/// The join: this leaf's subscribed types, classified against the split.
///
/// `ceiling` is the small/large split — [`DEFAULT_SMALL_CLASS_CEILING`] unless
/// a caller states another, the same shape as
/// `nros_derive_message_bound_knobs(SMALL_CLASS_CEILING ...)`.
pub fn payload_classes_for_leaf(leaf: &Path, inv: &EntityInventory) -> PayloadClasses {
    join(inv, DEFAULT_SMALL_CLASS_CEILING, || {
        leaf_bound_inventory(leaf)
    })
}

/// The join proper, with its two inputs supplied — so a test can state a bound
/// table without a directory, and the production caller reads one from disk.
pub fn join(
    inv: &EntityInventory,
    ceiling: usize,
    bounds: impl FnOnce() -> Result<Vec<(String, BoundState)>, String>,
) -> PayloadClasses {
    let subscribed = match inv.subscribed_types() {
        ReceivedTypes::Resolved(v) => v,
        ReceivedTypes::Refused { reason } => {
            return PayloadClasses::Refused {
                reason: format!(
                    "the subscribed-type set did not resolve, so the payload classes are \
                     derived from it or not at all:\n  {reason}"
                ),
            };
        }
    };

    // NOTHING SUBSCRIBED is the answer `large_count = 0`, and it needs no bound
    // table at all — which is the case this issue is about, and the case the
    // CMake lane cannot reach when the linked closure holds an unbounded type
    // it merely publishes. Reading the table anyway would let a malformed
    // artifact refuse an image that receives nothing.
    if subscribed.is_empty() {
        return PayloadClasses::Derived(DerivedPayloadClasses::default());
    }

    let table = match bounds() {
        Ok(t) => t,
        Err(reason) => return PayloadClasses::Refused { reason },
    };

    let mut out = DerivedPayloadClasses::default();
    let mut open: Vec<String> = Vec::new();
    let mut unpriced: Vec<String> = Vec::new();
    for (type_name, count) in subscribed {
        out.subscribed += count;
        let Some((_, bound)) = table.iter().find(|(t, _)| t == &type_name) else {
            unpriced.push(type_name.clone());
            continue;
        };
        let BoundState::Bounded { rx, .. } = bound else {
            open.push(format!("{type_name} ({})", bound.tag()));
            continue;
        };
        if *rx > ceiling {
            out.large_count += count;
            out.large_max = out.large_max.max(*rx);
        } else {
            out.small_max = out.small_max.max(*rx);
        }
    }

    // Checked before `unpriced`, the same order and for the same reason as the
    // CMake join: a type this image RECEIVES and this tree cannot bound is a
    // different problem from one the inventory has never heard of, and saying
    // "not in the inventory" about a type that IS in it sends the reader
    // looking for a typo.
    if !open.is_empty() {
        return PayloadClasses::Refused {
            reason: format!(
                "{} type(s) this leaf RECEIVES carry no derived bound, so their payload \
                 class cannot be sized:\n    {}\n  Bound the member in its `.msg` \
                 (`string<=64`) or cap it `inline` in the package's `nros-codegen.toml` \
                 (RFC-0033).",
                open.len(),
                open.join("\n    ")
            ),
        };
    }
    if !unpriced.is_empty() {
        return PayloadClasses::Refused {
            reason: format!(
                "{} type(s) this leaf receives are not in its `generated/` bound \
                 inventory, so their payload class cannot be derived:\n    {}\n  Either \
                 the declaration names a type this leaf does not generate, or it names a \
                 service/action type -- the bound inventory prices MESSAGES \
                 (`pkg/msg/Name`) and nothing else.",
                unpriced.len(),
                unpriced.join("\n    ")
            ),
        };
    }
    PayloadClasses::Derived(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::entity_inventory::{ComponentEntities, Declaration, EntityDecl};

    fn inv_of(specs: &[&str]) -> EntityInventory {
        let mut inv = EntityInventory::new("t");
        let mut decls = Vec::new();
        for s in specs {
            decls.extend(EntityDecl::parse(s).expect("spec parses"));
        }
        inv.insert(ComponentEntities {
            pkg: "p".into(),
            component: "c".into(),
            class: "c".into(),
            declaration: if decls.is_empty() {
                Declaration::None
            } else {
                Declaration::Stated(decls)
            },
        });
        inv
    }

    fn table(rows: &[(&str, BoundState)]) -> Vec<(String, BoundState)> {
        rows.iter()
            .map(|(t, b)| ((*t).to_string(), b.clone()))
            .collect()
    }

    fn bounded(rx: usize) -> BoundState {
        BoundState::Bounded { tx: rx, rx }
    }

    /// The whole point of issue 1125: a publisher-only leaf reserves no large
    /// block. ZERO IS THE ANSWER, and it is reached without consulting the
    /// bound table at all.
    #[test]
    fn a_leaf_that_subscribes_to_nothing_needs_no_large_block() {
        let inv = inv_of(&["publisher:std_msgs/msg/String:/chatter", "timer"]);
        let got = join(&inv, 2048, || {
            panic!("the bound table must not be read when nothing is subscribed")
        });
        let PayloadClasses::Derived(k) = got else {
            panic!("{got:?}")
        };
        assert_eq!(k.large_count, 0);
        assert_eq!(k.subscribed, 0);
    }

    /// Entities, not distinct types — two subscriptions on one large type need
    /// two blocks.
    #[test]
    fn the_large_count_counts_entities_not_types() {
        let inv = inv_of(&[
            "sub:sensor_msgs/msg/Image:/a",
            "sub:sensor_msgs/msg/Image:/b",
        ]);
        let got = join(&inv, 2048, || {
            Ok(table(&[("sensor_msgs/msg/Image", bounded(40_000))]))
        });
        let PayloadClasses::Derived(k) = got else {
            panic!("{got:?}")
        };
        assert_eq!(k.large_count, 2);
        assert_eq!(k.large_max, 40_000);
        assert_eq!(k.small_max, 0);
    }

    /// A subscribed type at or under the split sizes the SMALL block and adds
    /// no large one.
    #[test]
    fn a_small_type_sizes_the_small_block_only() {
        let inv = inv_of(&["sub:std_msgs/msg/Bool:/t"]);
        let got = join(&inv, 2048, || {
            Ok(table(&[("std_msgs/msg/Bool", bounded(12))]))
        });
        let PayloadClasses::Derived(k) = got else {
            panic!("{got:?}")
        };
        assert_eq!((k.large_count, k.large_max, k.small_max), (0, 0, 12));
    }

    /// The band issue 0841 measured: a type between the small block's default
    /// (1024) and the split (2048) is SMALL here, which is only sound because
    /// the small block is derived to hold it in the same sidecar.
    #[test]
    fn a_type_in_the_0841_band_is_small_and_sizes_the_block_to_hold_it() {
        let inv = inv_of(&["sub:pkg/msg/Mid:/t"]);
        let got = join(&inv, 2048, || Ok(table(&[("pkg/msg/Mid", bounded(1500))])));
        let PayloadClasses::Derived(k) = got else {
            panic!("{got:?}")
        };
        assert_eq!(k.large_count, 0);
        assert_eq!(
            k.small_max, 1500,
            "the small block must be sized to hold what was classified into it"
        );
    }

    /// An unbounded SUBSCRIBED type refuses: nothing may be sized from it.
    #[test]
    fn an_unbounded_subscribed_type_refuses() {
        let inv = inv_of(&["sub:std_msgs/msg/String:/chatter"]);
        let got = join(&inv, 2048, || {
            Ok(table(&[(
                "std_msgs/msg/String",
                BoundState::Unbounded {
                    reason: "unbounded member: data (string)".into(),
                },
            )]))
        });
        let PayloadClasses::Refused { reason } = got else {
            panic!("{got:?}")
        };
        assert!(reason.contains("std_msgs/msg/String"), "{reason}");
    }

    /// A subscribed type no artifact prices refuses, and names it.
    #[test]
    fn a_subscribed_type_the_inventory_does_not_price_refuses() {
        let inv = inv_of(&["sub:pkg/msg/Missing:/t"]);
        let got = join(&inv, 2048, || Ok(Vec::new()));
        let PayloadClasses::Refused { reason } = got else {
            panic!("{got:?}")
        };
        assert!(reason.contains("pkg/msg/Missing"), "{reason}");
    }

    /// A malformed artifact refuses the leaf rather than pricing a subset.
    #[test]
    fn a_bound_table_that_cannot_be_read_refuses() {
        let inv = inv_of(&["sub:pkg/msg/A:/t"]);
        let got = join(&inv, 2048, || Err("schema_version 99".into()));
        assert!(matches!(got, PayloadClasses::Refused { .. }), "{got:?}");
    }
}
