// emit:ok
// Idiomatic Rust layer - user-friendly types
// Package: fingerprint-corpus
// Message: Capped

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Capped {
    
    
    pub label: std::string::String,
    
    
    
    pub samples: std::vec::Vec<i64>,
    
    
    
    pub tags: std::vec::Vec<std::string::String>,
    
    
}

impl Capped {
    
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for Capped {
    fn default() -> Self {
        // Leverage RMW message's C init function to get correct default values
        <Self as crate::rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Capped::default())
    }
}

// Reference-based conversions (idiomatic ↔ RMW)
impl From<&Capped> for crate::msg::rmw::Capped {
    #[allow(unused_variables)]
    fn from(idiomatic: &Capped) -> Self {
        Self {
            
            
            // String → rosidl_runtime_rs::String (use rosidl_runtime_rs 0.5 API)
            label: crate::rosidl_runtime_rs::String::from(idiomatic.label.as_str()),
            
            
            
            // Vec<primitive> → Sequence<primitive> (use rosidl_runtime_rs 0.5 API)
            samples: crate::rosidl_runtime_rs::Sequence::from(idiomatic.samples.clone()),
            
            
            
            // Vec<String> → Sequence<rosidl_runtime_rs::String>
            tags: crate::rosidl_runtime_rs::Sequence::from(
                idiomatic.tags.iter().map(|item| crate::rosidl_runtime_rs::String::from(item.as_str())).collect::<Vec<_>>()
            ),
            
            
        }
    }
}

impl From<&crate::msg::rmw::Capped> for Capped {
    #[allow(unused_variables)]
    fn from(rmw: &crate::msg::rmw::Capped) -> Self {
        Self {
            
            
            // rosidl_runtime_rs::String → String (use rosidl_runtime_rs 0.5 API)
            label: rmw.label.to_string(),
            
            
            
            // Sequence<primitive> → Vec<primitive> (use rosidl_runtime_rs 0.5 API)
            samples: rmw.samples.as_slice().to_vec(),
            
            
            
            // Sequence<rosidl_runtime_rs::String> → Vec<String>
            tags: rmw.tags.as_slice().iter().map(|item| item.to_string()).collect(),
            
            
        }
    }
}

// Owned conversions delegate to reference-based ones
impl From<crate::msg::rmw::Capped> for Capped {
    fn from(rmw: crate::msg::rmw::Capped) -> Self {
        Self::from(&rmw)
    }
}

impl From<Capped> for crate::msg::rmw::Capped {
    fn from(idiomatic: Capped) -> Self {
        Self::from(&idiomatic)
    }
}

// Message trait implementation for rosidl_runtime_rs
impl crate::rosidl_runtime_rs::Message for Capped {
    type RmwMsg = crate::msg::rmw::Capped;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        // Convert from idiomatic to RMW format
        std::borrow::Cow::Owned(msg_cow.into_owned().into())
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        // Convert from RMW to idiomatic format
        msg.into()
    }
}