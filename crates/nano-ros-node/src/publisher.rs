//! Publisher handle

use core::marker::PhantomData;

/// Handle to a publisher
///
/// This is a lightweight handle that references a publisher registered
/// with a Node. The type parameter `M` ensures type safety when publishing.
#[derive(Debug)]
pub struct PublisherHandle<M> {
    index: usize,
    _marker: PhantomData<M>,
}

impl<M> PublisherHandle<M> {
    /// Create a new publisher handle
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
    pub(crate) fn untyped(&self) -> PublisherHandle<()> {
        PublisherHandle {
            index: self.index,
            _marker: PhantomData,
        }
    }
}

impl<M> Clone for PublisherHandle<M> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<M> Copy for PublisherHandle<M> {}

impl<M> PartialEq for PublisherHandle<M> {
    fn eq(&self, other: &Self) -> bool {
        self.index == other.index
    }
}

impl<M> Eq for PublisherHandle<M> {}
