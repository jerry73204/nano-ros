//! Transport abstraction traits

/// Topic information
pub struct TopicInfo<'a> {
    pub name: &'a str,
    pub type_name: &'a str,
    pub rihs_hash: &'a str,
}

/// Transport abstraction trait
pub trait Transport {
    type Error;

    /// Create a new transport connection
    fn connect(&mut self) -> Result<(), Self::Error>;

    /// Disconnect
    fn disconnect(&mut self) -> Result<(), Self::Error>;
}
