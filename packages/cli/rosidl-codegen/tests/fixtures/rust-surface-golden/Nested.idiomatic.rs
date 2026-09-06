// emit:ok
// Idiomatic Rust layer - user-friendly types
// Package: fingerprint-corpus
// Message: Nested

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Nested {
    
    
    pub one: crate::msg::Shapes,
    
    
    
    pub many: std::vec::Vec<crate::msg::Shapes>,
    
    
}

impl Nested {
    
    pub fn new() -> Self {
        Self::default()
    }
}

impl Default for Nested {
    fn default() -> Self {
        // Leverage RMW message's C init function to get correct default values
        <Self as crate::rosidl_runtime_rs::Message>::from_rmw_message(crate::msg::rmw::Nested::default())
    }
}

// Reference-based conversions (idiomatic ↔ RMW)
impl From<&Nested> for crate::msg::rmw::Nested {
    #[allow(unused_variables)]
    fn from(idiomatic: &Nested) -> Self {
        Self {
            
            
            // Nested messages need reference conversion
            one: (&idiomatic.one).into(),
            
            
            
            // Vec → Sequence conversion with element conversion for nested messages
            many: crate::rosidl_runtime_rs::Sequence::from(
                idiomatic.many.iter().map(|item| item.into()).collect::<Vec<_>>()
            ),
            
            
        }
    }
}

impl From<&crate::msg::rmw::Nested> for Nested {
    #[allow(unused_variables)]
    fn from(rmw: &crate::msg::rmw::Nested) -> Self {
        Self {
            
            
            // Nested messages need reference conversion
            one: (&rmw.one).into(),
            
            
            
            // Sequence → Vec conversion with element conversion for nested messages
            many: rmw.many.as_slice().iter().map(|item| item.into()).collect(),
            
            
        }
    }
}

// Owned conversions delegate to reference-based ones
impl From<crate::msg::rmw::Nested> for Nested {
    fn from(rmw: crate::msg::rmw::Nested) -> Self {
        Self::from(&rmw)
    }
}

impl From<Nested> for crate::msg::rmw::Nested {
    fn from(idiomatic: Nested) -> Self {
        Self::from(&idiomatic)
    }
}

// Message trait implementation for rosidl_runtime_rs
impl crate::rosidl_runtime_rs::Message for Nested {
    type RmwMsg = crate::msg::rmw::Nested;

    fn into_rmw_message(msg_cow: std::borrow::Cow<'_, Self>) -> std::borrow::Cow<'_, Self::RmwMsg> {
        // Convert from idiomatic to RMW format
        std::borrow::Cow::Owned(msg_cow.into_owned().into())
    }

    fn from_rmw_message(msg: Self::RmwMsg) -> Self {
        // Convert from RMW to idiomatic format
        msg.into()
    }
}