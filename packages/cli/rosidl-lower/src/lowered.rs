//! The target-concrete, language-neutral IR (RFC-0068 Stage 2 output).
//!
//! `lower()` takes a [`ResolvedMessage`], the capacity [`CapacityResolver`]
//! (CodegenConfig), and computes the per-field facts a
//! renderer must not re-derive: storage decision, plainness, alignment, the CDR
//! op, and the field order. Language spelling is NOT here — a template maps the
//! neutral facts to `u32`/`uint32_t`/`write_u32`/… (RFC-0068 Stage 3).

use std::borrow::Cow;

use rosidl_parser::ast::{ConstantValue, FieldType, PrimitiveType};
use rosidl_resolve::ResolvedMessage;

use crate::config::{CapacityResolver, FieldKind, FieldStorage, StorageMode, with_element_bound};

/// Stand-in alignment for a nested struct field.
///
/// phase-432 W1.1 — this used to be `TargetProfile::ptr_width`, which is why
/// that profile read as target-critical. It is not: `align` is consumed only
/// to decide `plain`, and a nested field is never plain, so no value here can
/// change an outcome. Written down as a constant so the next reader does not
/// have to re-derive that.
const NESTED_ALIGN_STANDIN: usize = 8;

/// Neutral CDR read/write op for a scalar. A renderer maps this to its own
/// method name (`write_u32` / `z_serialize_uint32` / …) — the op itself is
/// language-agnostic.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CdrOp {
    Bool,
    U8,
    I8,
    U16,
    I16,
    U32,
    I32,
    U64,
    I64,
    F32,
    F64,
    /// A string member (length-prefixed).
    String,
    /// A nested message member (delegates to its own (de)serialize).
    Nested,
}

impl CdrOp {
    fn from_primitive(p: PrimitiveType) -> Self {
        match p {
            PrimitiveType::Bool => CdrOp::Bool,
            PrimitiveType::Byte | PrimitiveType::UInt8 => CdrOp::U8,
            PrimitiveType::Char => CdrOp::U8,
            PrimitiveType::Int8 => CdrOp::I8,
            PrimitiveType::UInt16 => CdrOp::U16,
            PrimitiveType::Int16 => CdrOp::I16,
            PrimitiveType::UInt32 => CdrOp::U32,
            PrimitiveType::Int32 => CdrOp::I32,
            PrimitiveType::UInt64 => CdrOp::U64,
            PrimitiveType::Int64 => CdrOp::I64,
            PrimitiveType::Float32 => CdrOp::F32,
            PrimitiveType::Float64 => CdrOp::F64,
        }
    }

    /// CDR wire size (bytes) of the scalar, which equals its natural alignment.
    fn cdr_size(self) -> usize {
        match self {
            CdrOp::Bool | CdrOp::U8 | CdrOp::I8 => 1,
            CdrOp::U16 | CdrOp::I16 => 2,
            CdrOp::U32 | CdrOp::I32 | CdrOp::F32 => 4,
            CdrOp::U64 | CdrOp::I64 | CdrOp::F64 => 8,
            // String / Nested are not fixed-size scalars.
            CdrOp::String | CdrOp::Nested => 0,
        }
    }

    /// Whether the scalar can participate in a POD blit fast path — the numeric
    /// integer/float ops only. `bool` is excluded (CDR bool is a constrained
    /// `u8`, not an arbitrary byte), as are the non-scalar `String`/`Nested`.
    fn is_plain_scalar(self) -> bool {
        matches!(
            self,
            CdrOp::U8
                | CdrOp::I8
                | CdrOp::U16
                | CdrOp::I16
                | CdrOp::U32
                | CdrOp::I32
                | CdrOp::U64
                | CdrOp::I64
                | CdrOp::F32
                | CdrOp::F64
        )
    }
}

/// How a field's payload is stored in the generated struct.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LoweredStorage {
    /// Value stored inline (scalars, fixed arrays, single nested struct).
    Inline,
    /// Fixed-capacity buffer of `cap` (bounded string, or an unbounded string
    /// the config pins to inline).
    Fixed { cap: usize },
    /// Bounded sequence of `cap` (`type[<=N]`, or an unbounded sequence the
    /// config pins to inline).
    Bounded { cap: usize },
    /// Heap-backed (`alloc::Vec` / `String` — the config's `heap` mode).
    Heap,
    /// Zero-copy borrow of `cap` into the CDR receive buffer (the config's
    /// `borrowed` mode — RFC-0033 / issue 0007).
    Borrowed { cap: usize },
}

impl LoweredStorage {
    /// Recover the `(mode, cap)` a `CapacityResolver::resolve` would have
    /// produced for a configurable (unbounded string / sequence) field. Lets a
    /// codegen builder read the storage decision from the IR instead of calling
    /// the resolver a second time (phase-335 W1.c). `Inline` is not a
    /// configurable-storage field and maps to `(Owned, 0)` defensively.
    pub fn as_field_storage(&self) -> FieldStorage {
        match *self {
            LoweredStorage::Inline => FieldStorage {
                cap: 0,
                mode: StorageMode::Inline,
            },
            LoweredStorage::Fixed { cap } | LoweredStorage::Bounded { cap } => FieldStorage {
                cap,
                mode: StorageMode::Inline,
            },
            LoweredStorage::Heap => FieldStorage {
                cap: 0,
                mode: StorageMode::Heap,
            },
            LoweredStorage::Borrowed { cap } => FieldStorage {
                cap,
                mode: StorageMode::View,
            },
        }
    }
}

/// The shape class of a field — what the renderer branches on.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FieldShape {
    Scalar,
    /// Length-prefixed string (single).
    Str,
    /// Fixed-size array `type[N]`.
    Array {
        len: usize,
    },
    /// Sequence `type[]` / `type[<=N]`.
    Sequence,
    /// Single nested message.
    Nested,
}

/// One field lowered to concrete, target-specific, language-neutral facts.
///
/// phase-432 W2.5a — this IS the render context. Every message surface (`rmw`,
/// `rust`, `nros`, `c`, `cpp`) projects its view struct from here; none of them
/// re-matches on `rosidl_parser` to answer a question this struct already
/// answers. The accessors below are that contract: they are the ONLY sanctioned
/// spelling of "is this a sequence", "what is the element's CDR op", "how long
/// is the array".
#[derive(Debug, Clone)]
pub struct LoweredField {
    pub name: String,
    /// The type AS PARSED — what the `.msg` says, hence what CDR puts on the
    /// wire and what a ROS-ABI mirror (the `rmw` surface) must spell.
    ///
    /// Deliberately NOT the element-capped shape: see [`Self::element_cap`] and
    /// [`Self::storage_type`], which answer a different question.
    pub field_type: FieldType,
    /// phase-403 W7 — the codegen config's `element_cap` for this field, when
    /// one applies (an unbounded-string element inside a container whose mode
    /// bounds the wire). `None` otherwise.
    ///
    /// This is a fact about nano-ros STORAGE, not about the wire: it narrows
    /// what we keep in RAM, and CDR is unchanged either way. So the storage
    /// surfaces (`c`, `nros`, `cpp`) render [`Self::storage_type`] while the
    /// ROS-ABI mirror (`rmw`) renders [`Self::field_type`]. Two questions, two
    /// answers — not two spellings of one answer.
    pub element_cap: Option<usize>,
    /// Whether this field's storage came from the [`CapacityResolver`] — true
    /// exactly for the two configurable shapes (an unbounded string and an
    /// unbounded sequence). A `.msg`-bounded string/sequence, an array, a
    /// scalar and a nested struct are NOT configurable, whatever the config
    /// says about them.
    pub configurable: bool,
    pub shape: FieldShape,
    pub storage: LoweredStorage,
    /// CDR op of the field's scalar (or its element, for arrays/sequences);
    /// `None` for a single nested struct. Read it through [`Self::scalar_op`] /
    /// [`Self::element_op`], which say which of the two a caller means.
    pub cdr_op: Option<CdrOp>,
    /// Alignment of the field's payload, bytes.
    pub align: usize,
    /// Whether this field is POD-blit eligible.
    pub plain: bool,
    /// The `.msg` default, as parsed. A language spells it (`constant_value_to_rust`
    /// and friends); the value itself is neutral.
    pub default_value: Option<ConstantValue>,
}

impl LoweredField {
    /// The shape a nano-ros STORAGE surface renders: [`Self::field_type`] with
    /// any [`Self::element_cap`] folded into the element. Identical to
    /// `field_type` whenever no element cap applies, which is the common case.
    pub fn storage_type(&self) -> Cow<'_, FieldType> {
        with_element_bound(&self.field_type, self.element_cap)
    }

    /// A single scalar (`bool`, `int32`, `float64`, …).
    pub fn is_primitive(&self) -> bool {
        matches!(self.shape, FieldShape::Scalar)
    }

    /// A single string member — bounded or not, narrow or wide.
    pub fn is_string(&self) -> bool {
        matches!(self.shape, FieldShape::Str)
    }

    /// A fixed-size array `type[N]`.
    pub fn is_array(&self) -> bool {
        matches!(self.shape, FieldShape::Array { .. })
    }

    /// A sequence — `type[]` or `type[<=N]`.
    pub fn is_sequence(&self) -> bool {
        matches!(self.shape, FieldShape::Sequence)
    }

    /// A single nested message.
    pub fn is_nested(&self) -> bool {
        matches!(self.shape, FieldShape::Nested)
    }

    /// `N` for `type[N]`, else 0.
    pub fn array_len(&self) -> usize {
        match self.shape {
            FieldShape::Array { len } => len,
            _ => 0,
        }
    }

    /// The CDR op of the field ITSELF — `Some` only for a scalar. A string's op
    /// is [`CdrOp::String`] and is not a scalar op; ask [`Self::is_string`].
    pub fn scalar_op(&self) -> Option<CdrOp> {
        self.is_primitive().then_some(self.cdr_op).flatten()
    }

    /// The CDR op of the field's ELEMENT — `Some` only for an array or a
    /// sequence. [`CdrOp::String`] here means a container OF strings.
    pub fn element_op(&self) -> Option<CdrOp> {
        (self.is_array() || self.is_sequence())
            .then_some(self.cdr_op)
            .flatten()
    }

    /// The element of an array/sequence is a scalar.
    pub fn element_is_primitive(&self) -> bool {
        matches!(self.element_op(), Some(op) if op != CdrOp::String)
    }

    /// The element of an array/sequence is a string.
    pub fn element_is_string(&self) -> bool {
        self.element_op() == Some(CdrOp::String)
    }

    /// Heap-backed storage (RFC-0033 `mode = "heap"`).
    pub fn is_heap(&self) -> bool {
        matches!(self.storage, LoweredStorage::Heap)
    }

    /// Zero-copy borrow into the receive buffer (RFC-0033 `mode = "view"`).
    pub fn is_borrowed(&self) -> bool {
        matches!(self.storage, LoweredStorage::Borrowed { .. })
    }

    /// The resolved capacity a CONFIGURABLE field's storage carries, else 0.
    ///
    /// Zero for a non-configurable field even when its storage has a capacity:
    /// a `string<=8`'s 8 comes from the `.msg`, not from the resolver, and the
    /// surfaces spell it off the type.
    pub fn configured_cap(&self) -> usize {
        if !self.configurable {
            return 0;
        }
        match self.storage {
            LoweredStorage::Fixed { cap }
            | LoweredStorage::Bounded { cap }
            | LoweredStorage::Borrowed { cap } => cap,
            LoweredStorage::Inline | LoweredStorage::Heap => 0,
        }
    }
}

/// A message lowered to the target-concrete IR.
#[derive(Debug, Clone)]
pub struct LoweredType {
    pub type_name: String,
    pub type_hash: String,
    /// Fields in `repr(C)` order. ROS/CDR is declaration-order-positional, so
    /// this is the parsed order — carried explicitly so the fact is a fact, not
    /// an assumption a renderer re-derives.
    pub fields: Vec<LoweredField>,
    /// Struct alignment = max field alignment (min 1).
    pub align: usize,
    /// POD-blit eligible: every field plain AND all fields share one alignment
    /// (else `repr(C)` inserts inter-field padding and the blit is unsound).
    pub plain: bool,
}

/// Lower a resolved message under the capacity `config`.
///
/// Target-agnostic: nothing here depends on pointer width or enum width, and
/// phase-432 W1.1 removed the `TargetProfile` that suggested otherwise. Layout
/// is the target compiler's to decide; where two languages must AGREE on a
/// representation, the generated source pins it (RFC-0091 §5).
pub fn lower(resolved: &ResolvedMessage, config: &CapacityResolver) -> LoweredType {
    // `pkg/msg/Name` → (pkg, Name) for the config lookup keys.
    let (package, message) = split_type_name(&resolved.type_name);
    let fields = lower_fields(package, message, &resolved.parsed.fields, config);

    let align = fields.iter().map(|f| f.align).max().unwrap_or(1).max(1);
    // Plain iff every field is plain AND all fields share one alignment (uniform
    // alignment ⇒ no inter-field or trailing padding under repr(C)).
    let uniform_align = fields
        .iter()
        .map(|f| f.align)
        .collect::<std::collections::BTreeSet<_>>();
    let plain = !fields.is_empty() && fields.iter().all(|f| f.plain) && uniform_align.len() == 1;

    LoweredType {
        type_name: resolved.type_name.clone(),
        type_hash: resolved.type_hash.clone(),
        fields,
        align,
        plain,
    }
}

/// Lower every field of `msg` (named `package` / `message` for the config
/// lookup keys) to its concrete facts. Exposed so a codegen builder can read
/// per-field storage from the IR rather than re-resolving it (phase-335 W1.c).
pub fn lower_fields(
    package: &str,
    message: &str,
    fields: &[rosidl_parser::ast::Field],
    config: &CapacityResolver,
) -> Vec<LoweredField> {
    fields
        .iter()
        .map(|f| lower_field(f, package, message, config))
        .collect()
}

fn lower_field(
    field: &rosidl_parser::ast::Field,
    package: &str,
    message: &str,
    config: &CapacityResolver,
) -> LoweredField {
    let name = field.name.as_str();
    let ft = &field.field_type;
    let (shape, storage, cdr_op, align, plain) = match ft {
        FieldType::Primitive(p) => {
            let op = CdrOp::from_primitive(*p);
            (
                FieldShape::Scalar,
                LoweredStorage::Inline,
                Some(op),
                op.cdr_size().max(1),
                op.is_plain_scalar(),
            )
        }
        FieldType::String | FieldType::WString => {
            let s = config.resolve(package, message, name, FieldKind::String);
            let storage = match s.mode {
                StorageMode::Inline => LoweredStorage::Fixed { cap: s.cap },
                StorageMode::Heap => LoweredStorage::Heap,
                StorageMode::View => LoweredStorage::Borrowed { cap: s.cap },
            };
            (FieldShape::Str, storage, Some(CdrOp::String), 4, false)
        }
        FieldType::BoundedString(n) | FieldType::BoundedWString(n) => (
            FieldShape::Str,
            LoweredStorage::Fixed { cap: *n },
            Some(CdrOp::String),
            4,
            false,
        ),
        FieldType::Array { element_type, size } => {
            let (op, elem_align, elem_plain) = element_facts(element_type);
            (
                FieldShape::Array { len: *size },
                LoweredStorage::Inline,
                op,
                elem_align,
                // A fixed array of a plain element is itself plain.
                elem_plain,
            )
        }
        FieldType::Sequence { element_type } => {
            let (op, elem_align, _) = element_facts(element_type);
            let s = config.resolve(package, message, name, FieldKind::Sequence);
            let storage = match s.mode {
                StorageMode::Inline => LoweredStorage::Bounded { cap: s.cap },
                StorageMode::Heap => LoweredStorage::Heap,
                StorageMode::View => LoweredStorage::Borrowed { cap: s.cap },
            };
            (FieldShape::Sequence, storage, op, elem_align.max(4), false)
        }
        FieldType::BoundedSequence {
            element_type,
            max_size,
        } => {
            let (op, elem_align, _) = element_facts(element_type);
            (
                FieldShape::Sequence,
                LoweredStorage::Bounded { cap: *max_size },
                op,
                elem_align.max(4),
                false,
            )
        }
        FieldType::NamespacedType { .. } => (
            FieldShape::Nested,
            LoweredStorage::Inline,
            None,
            // A nested struct's alignment is unknown without its own
            // lowering, so this is a conservative stand-in — and its VALUE
            // cannot matter: `plain` is false for every nested field (the
            // `false` below), and `align` is only ever read to decide
            // plainness. It was `TargetProfile::ptr_width` until phase-432
            // W1.1, which is the whole reason that profile looked load-bearing.
            NESTED_ALIGN_STANDIN,
            false,
        ),
    };

    // The two configurable shapes, named once. `configurable` is the same
    // predicate the arms above branch on to call `config.resolve` — stated as a
    // fact so a surface stops re-deriving it (phase-432 W2.5a).
    let configurable = matches!(
        ft,
        FieldType::String | FieldType::WString | FieldType::Sequence { .. }
    );

    LoweredField {
        name: name.to_string(),
        field_type: ft.clone(),
        element_cap: config.declared_element_bound(package, message, name, ft),
        configurable,
        shape,
        storage,
        cdr_op,
        align,
        plain,
        default_value: field.default_value.clone(),
    }
}

/// Facts about an array/sequence element: its CDR op (None if nested), its
/// alignment, and whether it is plain.
fn element_facts(elem: &FieldType) -> (Option<CdrOp>, usize, bool) {
    match elem {
        FieldType::Primitive(p) => {
            let op = CdrOp::from_primitive(*p);
            (Some(op), op.cdr_size().max(1), op.is_plain_scalar())
        }
        FieldType::String
        | FieldType::WString
        | FieldType::BoundedString(_)
        | FieldType::BoundedWString(_) => (Some(CdrOp::String), 4, false),
        FieldType::NamespacedType { .. } => (None, NESTED_ALIGN_STANDIN, false),
        // Nested arrays/sequences of arrays are not a ROS .msg shape.
        FieldType::Array { element_type, .. }
        | FieldType::Sequence { element_type }
        | FieldType::BoundedSequence { element_type, .. } => element_facts(element_type),
    }
}

fn split_type_name(type_name: &str) -> (&str, &str) {
    // `pkg/msg/Name` → ("pkg", "Name"); tolerate `pkg/Name`.
    let mut it = type_name.split('/');
    let pkg = it.next().unwrap_or("");
    let name = type_name.rsplit('/').next().unwrap_or(type_name);
    (pkg, name)
}

#[cfg(test)]
mod tests {
    use super::*;
    use rosidl_parser::parse_message;

    fn no_deps(_: &str) -> Option<rosidl_parser::Message> {
        None
    }

    fn lower_shapes() -> LoweredType {
        // A subset of the fingerprint corpus's Shapes.msg — every field shape.
        let src = "\
bool     flag
uint32   u32_v
float64  f64_v
string   text
int64[]      seq_prim
float64[3]   arr_fixed
int32[<=4]   seq_bounded
string<=8    str_bounded
";
        let msg = parse_message(src).unwrap();
        let r = ResolvedMessage::resolve("shapes_msgs/msg/Shapes", &msg, no_deps).unwrap();
        lower(&r, &CapacityResolver::empty())
    }

    fn field<'a>(t: &'a LoweredType, name: &str) -> &'a LoweredField {
        t.fields.iter().find(|f| f.name == name).unwrap()
    }

    #[test]
    fn scalar_facts() {
        let t = lower_shapes();
        let u = field(&t, "u32_v");
        assert_eq!(u.shape, FieldShape::Scalar);
        assert_eq!(u.storage, LoweredStorage::Inline);
        assert_eq!(u.cdr_op, Some(CdrOp::U32));
        assert_eq!(u.align, 4);
        assert!(u.plain);
        // bool is not plain (constrained CDR u8).
        assert!(!field(&t, "flag").plain);
        assert_eq!(field(&t, "f64_v").align, 8);
    }

    #[test]
    fn string_and_bounded_facts() {
        let t = lower_shapes();
        // unbounded string, empty config → the `owned` default: a fixed-capacity
        // buffer sized to the nros string default (256).
        assert_eq!(
            field(&t, "text").storage,
            LoweredStorage::Fixed { cap: 256 }
        );
        assert_eq!(field(&t, "text").shape, FieldShape::Str);
        // bounded string carries its .msg cap.
        assert_eq!(
            field(&t, "str_bounded").storage,
            LoweredStorage::Fixed { cap: 8 }
        );
    }

    #[test]
    fn array_and_sequence_facts() {
        let t = lower_shapes();
        let arr = field(&t, "arr_fixed");
        assert_eq!(arr.shape, FieldShape::Array { len: 3 });
        assert_eq!(arr.storage, LoweredStorage::Inline);
        assert_eq!(arr.cdr_op, Some(CdrOp::F64));
        // unbounded sequence → owned default (bounded to the nros seq default 64);
        // an explicitly bounded sequence carries its .msg cap.
        assert_eq!(
            field(&t, "seq_prim").storage,
            LoweredStorage::Bounded { cap: 64 }
        );
        assert_eq!(
            field(&t, "seq_bounded").storage,
            LoweredStorage::Bounded { cap: 4 }
        );
    }

    /// phase-432 W1.1 — the negative control for deleting `TargetProfile`.
    ///
    /// This replaces `same_resolved_message_lowers_differently_per_target`,
    /// which asserted that a nested field's `align` was 8 on host and 4 on
    /// arm-eabi. That was true, and it was the reason the profile READ as
    /// target-critical — but the same test also asserted `!plain` for both,
    /// which is the fact that made the difference unobservable: `align` is
    /// consumed only to decide plainness, and a nested field is never plain.
    ///
    /// So the invariant worth holding is not "align tracks the target" but
    /// "align cannot change an outcome here". If a future change makes a
    /// nested field plain, or makes `align` reachable by anything else, this
    /// fails and the deletion has to be revisited.
    #[test]
    fn a_nested_field_is_never_plain_so_its_align_cannot_matter() {
        let inner = parse_message("int32 a\n").unwrap();
        let outer = parse_message("test_msgs/Inner child\nint32 tag\n").unwrap();
        let resolve = |fqn: &str| -> Option<rosidl_parser::Message> {
            if fqn.ends_with("/Inner") || fqn == "test_msgs/Inner" {
                Some(inner.clone())
            } else {
                None
            }
        };
        let r = ResolvedMessage::resolve("test_msgs/msg/Outer", &outer, resolve).unwrap();
        let lowered = lower(&r, &CapacityResolver::empty());

        // The hash is a Resolve fact and never a lowering one.
        assert!(!lowered.type_hash.is_empty());
        // A nested field is not plain, and neither is the struct holding it.
        assert!(!field(&lowered, "child").plain);
        assert!(!lowered.plain);
        // Its align is the stand-in, and nothing downstream reads it.
        assert_eq!(field(&lowered, "child").align, NESTED_ALIGN_STANDIN);
    }

    /// phase-432 W2.5a — the accessors the five message surfaces now project
    /// through, checked against the shapes they used to `match` on themselves.
    ///
    /// Each assertion below replaced a `matches!(field.field_type, …)` in a view
    /// builder; if one of them stops agreeing, a surface's private answer and
    /// the IR's answer have diverged, which is exactly the defect this wave
    /// removed.
    #[test]
    fn the_shape_accessors_answer_what_the_surfaces_used_to_re_derive() {
        let t = lower_shapes();

        let scalar = field(&t, "u32_v");
        assert!(scalar.is_primitive() && !scalar.is_string());
        assert_eq!(scalar.scalar_op(), Some(CdrOp::U32));
        // A scalar has no element, so no surface may read one off it.
        assert_eq!(scalar.element_op(), None);
        assert!(!scalar.element_is_primitive() && !scalar.element_is_string());
        assert_eq!(scalar.array_len(), 0);

        let text = field(&t, "text");
        assert!(text.is_string() && !text.is_primitive());
        // `String`'s CDR op is `String` — but it is NOT a scalar op, so a
        // surface asking for `primitive_method` gets nothing.
        assert_eq!(text.scalar_op(), None);
        assert!(text.configurable, "an unbounded string is config-resolved");
        assert_eq!(text.configured_cap(), 256);

        let bounded = field(&t, "str_bounded");
        assert!(bounded.is_string());
        assert!(
            !bounded.configurable,
            "a `.msg`-bounded string states its own cap; the resolver never sees it"
        );
        assert_eq!(
            bounded.configured_cap(),
            0,
            "its 8 comes from the type, not the config"
        );

        let arr = field(&t, "arr_fixed");
        assert!(arr.is_array() && !arr.is_sequence());
        assert_eq!(arr.array_len(), 3);
        assert_eq!(arr.element_op(), Some(CdrOp::F64));
        assert!(arr.element_is_primitive() && !arr.element_is_string());
        assert_eq!(arr.scalar_op(), None, "an array is not itself a scalar");

        let seq = field(&t, "seq_prim");
        assert!(seq.is_sequence() && !seq.is_array());
        assert!(seq.configurable);
        assert_eq!(seq.configured_cap(), 64);
        assert_eq!(seq.element_op(), Some(CdrOp::I64));

        let seq_bounded = field(&t, "seq_bounded");
        assert!(seq_bounded.is_sequence());
        assert!(
            !seq_bounded.configurable,
            "`int32[<=4]` states its own bound"
        );

        // No field here is heap or borrowed under the empty resolver.
        assert!(t.fields.iter().all(|f| !f.is_heap() && !f.is_borrowed()));
    }

    /// A sequence OF strings: the element op is `String`, so
    /// `element_is_string` is true and `element_is_primitive` is false. Both
    /// surfaces (C and nros) branch on exactly this pair.
    #[test]
    fn a_string_element_is_not_a_primitive_element() {
        let msg = parse_message("string[] names\nstring[3] fixed_names\n").unwrap();
        let r = ResolvedMessage::resolve("m/msg/Names", &msg, no_deps).unwrap();
        let t = lower(&r, &CapacityResolver::empty());
        for name in ["names", "fixed_names"] {
            let f = field(&t, name);
            assert!(f.element_is_string(), "{name}");
            assert!(!f.element_is_primitive(), "{name}");
        }
    }

    /// A nested field has NO cdr op at all, so neither element predicate fires
    /// — the arm every view builder spelled as a `_ => (false, false, …)`.
    #[test]
    fn a_nested_element_has_no_cdr_op() {
        let inner = parse_message("int32 a\n").unwrap();
        let outer = parse_message("test_msgs/Inner one\ntest_msgs/Inner[] many\n").unwrap();
        let resolve = |fqn: &str| -> Option<rosidl_parser::Message> {
            fqn.ends_with("Inner").then(|| inner.clone())
        };
        let r = ResolvedMessage::resolve("test_msgs/msg/Outer", &outer, resolve).unwrap();
        let t = lower(&r, &CapacityResolver::empty());

        let one = field(&t, "one");
        assert!(one.is_nested());
        assert_eq!(one.cdr_op, None);
        assert_eq!(one.scalar_op(), None);

        let many = field(&t, "many");
        assert!(many.is_sequence());
        assert_eq!(many.element_op(), None);
        assert!(!many.element_is_primitive() && !many.element_is_string());
    }

    /// `element_cap` narrows STORAGE, never the wire — so `field_type` keeps
    /// what the `.msg` said and `storage_type()` carries the fold. The `rmw`
    /// surface renders the first, the `c`/`nros`/`cpp` surfaces the second.
    #[test]
    fn an_element_cap_moves_storage_type_and_leaves_field_type_alone() {
        let msg = parse_message("string[] tags\n").unwrap();
        let r = ResolvedMessage::resolve("p/msg/M", &msg, no_deps).unwrap();

        let plain = lower(&r, &CapacityResolver::empty());
        let f = field(&plain, "tags");
        assert_eq!(f.element_cap, None);
        assert_eq!(&*f.storage_type(), &f.field_type);

        let cfg = CapacityResolver::from_toml_str(
            "[fields]\n\"p/M.tags\" = { cap = 4, element_cap = 8, mode = \"inline\" }\n",
        )
        .unwrap();
        let capped = lower(&r, &cfg);
        let f = field(&capped, "tags");
        assert_eq!(f.element_cap, Some(8));
        assert_eq!(
            f.field_type,
            rosidl_parser::ast::FieldType::Sequence {
                element_type: Box::new(rosidl_parser::ast::FieldType::String),
            },
            "the wire type is what the .msg says, cap or no cap"
        );
        assert_eq!(
            &*f.storage_type(),
            &rosidl_parser::ast::FieldType::Sequence {
                element_type: Box::new(rosidl_parser::ast::FieldType::BoundedString(8)),
            },
        );
    }

    #[test]
    fn struct_not_plain_when_mixed_alignment_or_strings() {
        // Shapes has strings/sequences → not plain.
        assert!(!lower_shapes().plain);

        // A uniform-alignment all-scalar struct IS plain.
        let msg = parse_message("uint32 a\nint32 b\nfloat32 c\n").unwrap();
        let r = ResolvedMessage::resolve("m/msg/AllU32", &msg, no_deps).unwrap();
        let t = lower(&r, &CapacityResolver::empty());
        assert!(t.plain, "all-4-byte-scalar struct should be plain");
        assert_eq!(t.align, 4);

        // Mixed alignment (u8 + u32) → padding → not plain.
        let msg2 = parse_message("uint8 a\nuint32 b\n").unwrap();
        let r2 = ResolvedMessage::resolve("m/msg/Mixed", &msg2, no_deps).unwrap();
        let t2 = lower(&r2, &CapacityResolver::empty());
        assert!(!t2.plain, "mixed-alignment struct must not be plain");
    }
}
