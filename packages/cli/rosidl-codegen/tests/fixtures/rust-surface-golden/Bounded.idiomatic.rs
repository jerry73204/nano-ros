// emit:ok
// Idiomatic Rust layer - user-friendly types
// Package: fingerprint-corpus
// Message: Bounded

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Bounded {
    
    
    pub flag: bool,
    
    
    
    pub wide: i64,
    
    
    
    pub narrow: u8,
    
    
    
    pub d: f64,
    
    
    
    pub label: std::string::String,
    
    
    
    pub fixed: [i32; 4],
    
    
    
    pub labels: std::vec::Vec<std::string::String>,
    
    
}

impl Bounded {
    
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for Bounded {
    fn default() -> Self {
        // Leverage RMW message's C init function to get correct default values
        <Self as crate::rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Bounded::default())
    }
}

// Reference-based conversions (idiomatic ↔ RMW)
impl From<&Bounded> for crate::msg::rmw::Bounded {
    #[allow(unused_variables)]
    fn from(idiomatic: &Bounded) -> Self {
        Self {
            
            
            // Primitives are Copy, just copy the value
            flag: idiomatic.flag,
            
            
            
            // Primitives are Copy, just copy the value
            wide: idiomatic.wide,
            
            
            
            // Primitives are Copy, just copy the value
            narrow: idiomatic.narrow,
            
            
            
            // Primitives are Copy, just copy the value
            d: idiomatic.d,
            
            
            
            // String → rosidl_runtime_rs::BoundedString<N> (use try_from)
            label: crate::rosidl_runtime_rs::BoundedString::try_from(idiomatic.label.as_str()).unwrap(),
            
            
            
            // [primitive; N] arrays can be cloned directly
            fixed: idiomatic.fixed.clone(),
            
            
            
            // Vec<String> → BoundedSequence<rosidl_runtime_rs::BoundedString<N>> (use try_from)
            labels: crate::rosidl_runtime_rs::BoundedSequence::try_from(
                idiomatic.labels.iter().map(|item| crate::rosidl_runtime_rs::BoundedString::try_from(item.as_str()).unwrap()).collect::<Vec<_>>()
            ).unwrap(),
            
            
        }
    }
}

impl From<&crate::msg::rmw::Bounded> for Bounded {
    #[allow(unused_variables)]
    fn from(rmw: &crate::msg::rmw::Bounded) -> Self {
        Self {
            
            
            // Primitives are Copy, just copy the value
            flag: rmw.flag,
            
            
            
            // Primitives are Copy, just copy the value
            wide: rmw.wide,
            
            
            
            // Primitives are Copy, just copy the value
            narrow: rmw.narrow,
            
            
            
            // Primitives are Copy, just copy the value
            d: rmw.d,
            
            
            
            // rosidl_runtime_rs::BoundedString<N> → String
            label: rmw.label.to_string(),
            
            
            
            // [primitive; N] arrays can be cloned directly
            fixed: rmw.fixed.clone(),
            
            
            
            // BoundedSequence<rosidl_runtime_rs::BoundedString<N>> → Vec<String>
            labels: rmw.labels.as_slice().iter().map(|item| item.to_string()).collect(),
            
            
        }
    }
}

// Owned conversions delegate to reference-based ones
impl From<crate::msg::rmw::Bounded> for Bounded {
    fn from(rmw: crate::msg::rmw::Bounded) -> Self {
        Self::from(&rmw)
    }
}

impl From<Bounded> for crate::msg::rmw::Bounded {
    fn from(idiomatic: Bounded) -> Self {
        Self::from(&idiomatic)
    }
}

// Message trait implementation for rosidl_runtime_rs
impl crate::rosidl_runtime_rs::Message for Bounded {
    type RmwMsg = crate::msg::rmw::Bounded;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        // Convert from idiomatic to RMW format
        std::borrow::Cow::Owned(msg_cow.into_owned().into())
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        // Convert from RMW to idiomatic format
        msg.into()
    }
}