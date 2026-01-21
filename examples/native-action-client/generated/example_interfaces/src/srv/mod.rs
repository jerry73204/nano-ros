//! Service types for this package

mod add_two_ints;
pub use add_two_ints::{AddTwoInts, AddTwoIntsRequest, AddTwoIntsResponse};

mod set_bool;
pub use set_bool::{SetBool, SetBoolRequest, SetBoolResponse};

mod trigger;
pub use trigger::{Trigger, TriggerRequest, TriggerResponse};

