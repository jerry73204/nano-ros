//! Subscriber handle

use core::marker::PhantomData;

/// Handle to a subscriber
///
/// This is a lightweight handle that references a subscriber registered
/// with a Node. The type parameter `M` ensures type safety when receiving.
#[derive(Debug)]
pub struct SubscriberHandle<M> {
    index: usize,
    _marker: PhantomData<M>,
}

impl<M> SubscriberHandle<M> {
    /// Create a new subscriber handle
    pub(crate) fn new(index: usize) -> Self {
        Self {
            index,
            _marker: PhantomData,
        }
    }

    /// Get the internal index
    pub(crate) fn index(&self) -> usize {
        self.index
    }

    /// Convert to an untyped handle (for internal use)
    #[allow(dead_code)] // Will be used for topic info lookup
    pub(crate) fn untyped(&self) -> SubscriberHandle<()> {
        SubscriberHandle {
            index: self.index,
            _marker: PhantomData,
        }
    }
}

impl<M> Clone for SubscriberHandle<M> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<M> Copy for SubscriberHandle<M> {}

impl<M> PartialEq for SubscriberHandle<M> {
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index
    }
}

impl<M> Eq for SubscriberHandle<M> {}
