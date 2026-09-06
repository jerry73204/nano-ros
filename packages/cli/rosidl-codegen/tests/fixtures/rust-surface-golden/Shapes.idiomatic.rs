// emit:ok
// Idiomatic Rust layer - user-friendly types
// Package: fingerprint-corpus
// Message: Shapes

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Shapes {
    
    
    pub flag: bool,
    
    
    
    pub i8_v: i8,
    
    
    
    pub u8_v: u8,
    
    
    
    pub i16_v: i16,
    
    
    
    pub u16_v: u16,
    
    
    
    pub i32_v: i32,
    
    
    
    pub u32_v: u32,
    
    
    
    pub i64_v: i64,
    
    
    
    pub u64_v: u64,
    
    
    
    pub f32_v: f32,
    
    
    
    pub f64_v: f64,
    
    
    
    pub text: std::string::String,
    
    
    
    pub seq_prim: std::vec::Vec<i64>,
    
    
    
    pub seq_string: std::vec::Vec<std::string::String>,
    
    
    
    pub arr_fixed: [f64; 3],
    
    
    
    pub seq_bounded: std::vec::Vec<i32>,
    
    
    
    pub str_bounded: std::string::String,
    
    
}

impl Shapes {
    
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for Shapes {
    fn default() -> Self {
        // Leverage RMW message's C init function to get correct default values
        <Self as crate::rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Shapes::default())
    }
}

// Reference-based conversions (idiomatic ↔ RMW)
impl From<&Shapes> for crate::msg::rmw::Shapes {
    #[allow(unused_variables)]
    fn from(idiomatic: &Shapes) -> Self {
        Self {
            
            
            // Primitives are Copy, just copy the value
            flag: idiomatic.flag,
            
            
            
            // Primitives are Copy, just copy the value
            i8_v: idiomatic.i8_v,
            
            
            
            // Primitives are Copy, just copy the value
            u8_v: idiomatic.u8_v,
            
            
            
            // Primitives are Copy, just copy the value
            i16_v: idiomatic.i16_v,
            
            
            
            // Primitives are Copy, just copy the value
            u16_v: idiomatic.u16_v,
            
            
            
            // Primitives are Copy, just copy the value
            i32_v: idiomatic.i32_v,
            
            
            
            // Primitives are Copy, just copy the value
            u32_v: idiomatic.u32_v,
            
            
            
            // Primitives are Copy, just copy the value
            i64_v: idiomatic.i64_v,
            
            
            
            // Primitives are Copy, just copy the value
            u64_v: idiomatic.u64_v,
            
            
            
            // Primitives are Copy, just copy the value
            f32_v: idiomatic.f32_v,
            
            
            
            // Primitives are Copy, just copy the value
            f64_v: idiomatic.f64_v,
            
            
            
            // String → rosidl_runtime_rs::String (use rosidl_runtime_rs 0.5 API)
            text: crate::rosidl_runtime_rs::String::from(idiomatic.text.as_str()),
            
            
            
            // Vec<primitive> → Sequence<primitive> (use rosidl_runtime_rs 0.5 API)
            seq_prim: crate::rosidl_runtime_rs::Sequence::from(idiomatic.seq_prim.clone()),
            
            
            
            // Vec<String> → Sequence<rosidl_runtime_rs::String>
            seq_string: crate::rosidl_runtime_rs::Sequence::from(
                idiomatic.seq_string.iter().map(|item| crate::rosidl_runtime_rs::String::from(item.as_str())).collect::<Vec<_>>()
            ),
            
            
            
            // [primitive; N] arrays can be cloned directly
            arr_fixed: idiomatic.arr_fixed.clone(),
            
            
            
            // Vec<primitive> → BoundedSequence<primitive> (use try_from)
            seq_bounded: crate::rosidl_runtime_rs::BoundedSequence::try_from(idiomatic.seq_bounded.clone()).unwrap(),
            
            
            
            // String → rosidl_runtime_rs::BoundedString<N> (use try_from)
            str_bounded: crate::rosidl_runtime_rs::BoundedString::try_from(idiomatic.str_bounded.as_str()).unwrap(),
            
            
        }
    }
}

impl From<&crate::msg::rmw::Shapes> for Shapes {
    #[allow(unused_variables)]
    fn from(rmw: &crate::msg::rmw::Shapes) -> Self {
        Self {
            
            
            // Primitives are Copy, just copy the value
            flag: rmw.flag,
            
            
            
            // Primitives are Copy, just copy the value
            i8_v: rmw.i8_v,
            
            
            
            // Primitives are Copy, just copy the value
            u8_v: rmw.u8_v,
            
            
            
            // Primitives are Copy, just copy the value
            i16_v: rmw.i16_v,
            
            
            
            // Primitives are Copy, just copy the value
            u16_v: rmw.u16_v,
            
            
            
            // Primitives are Copy, just copy the value
            i32_v: rmw.i32_v,
            
            
            
            // Primitives are Copy, just copy the value
            u32_v: rmw.u32_v,
            
            
            
            // Primitives are Copy, just copy the value
            i64_v: rmw.i64_v,
            
            
            
            // Primitives are Copy, just copy the value
            u64_v: rmw.u64_v,
            
            
            
            // Primitives are Copy, just copy the value
            f32_v: rmw.f32_v,
            
            
            
            // Primitives are Copy, just copy the value
            f64_v: rmw.f64_v,
            
            
            
            // rosidl_runtime_rs::String → String (use rosidl_runtime_rs 0.5 API)
            text: rmw.text.to_string(),
            
            
            
            // Sequence<primitive> → Vec<primitive> (use rosidl_runtime_rs 0.5 API)
            seq_prim: rmw.seq_prim.as_slice().to_vec(),
            
            
            
            // Sequence<rosidl_runtime_rs::String> → Vec<String>
            seq_string: rmw.seq_string.as_slice().iter().map(|item| item.to_string()).collect(),
            
            
            
            // [primitive; N] arrays can be cloned directly
            arr_fixed: rmw.arr_fixed.clone(),
            
            
            
            // BoundedSequence<primitive> → Vec<primitive> (use rosidl_runtime_rs 0.5 API)
            seq_bounded: rmw.seq_bounded.as_slice().to_vec(),
            
            
            
            // rosidl_runtime_rs::BoundedString<N> → String
            str_bounded: rmw.str_bounded.to_string(),
            
            
        }
    }
}

// Owned conversions delegate to reference-based ones
impl From<crate::msg::rmw::Shapes> for Shapes {
    fn from(rmw: crate::msg::rmw::Shapes) -> Self {
        Self::from(&rmw)
    }
}

impl From<Shapes> for crate::msg::rmw::Shapes {
    fn from(idiomatic: Shapes) -> Self {
        Self::from(&idiomatic)
    }
}

// Message trait implementation for rosidl_runtime_rs
impl crate::rosidl_runtime_rs::Message for Shapes {
    type RmwMsg = crate::msg::rmw::Shapes;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        // Convert from idiomatic to RMW format
        std::borrow::Cow::Owned(msg_cow.into_owned().into())
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        // Convert from RMW to idiomatic format
        msg.into()
    }
}