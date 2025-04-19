/// Unique identifier for tracing purposes
#[derive(Clone, Copy, Debug)]
pub struct TracingId<T> {
    /// Pointer to the object used as a unique ID.
    /// Safety: Do NOT dereference the pointer.
    #[cfg(feature = "tracing")]
    id: *const T,

    /// Marker for the type. Needed when `tracing` feature is disabled.
    #[cfg(not(feature = "tracing"))]
    _marker: std::marker::PhantomData<T>,
}

impl<T> TracingId<T> {
    /// Creates new `TracingId` from the pointer.
    ///
    /// # Safety
    /// The pointer is used as a unique ID so users must ensure that they never create `TracingId`
    /// with same address for different objects.
    ///
    /// The pointer does not need to point to valid memory.
    pub const unsafe fn new(_id: *const T) -> Self {
        Self {
            #[cfg(feature = "tracing")]
            id: _id,
            #[cfg(not(feature = "tracing"))]
            _marker: std::marker::PhantomData,
        }
    }

    /// Obtain the address representing the ID.
    ///
    /// # Safety
    /// Do NOT dereference the pointer.
    #[cfg(feature = "tracing")]
    pub(crate) const unsafe fn c_void(self) -> *const std::ffi::c_void {
        self.id.cast::<std::ffi::c_void>()
    }
}

/// # Safety
///
/// The address is never dereferenced and is used only as a unique ID so it is safe to send to another thread.
unsafe impl<T> Send for TracingId<T> {}

/// # Safety
///
/// The `TracingId` does not allow interior mutability because the pointer is never dereferenced.
/// It is safe to use across multiple threads.
unsafe impl<T> Sync for TracingId<T> {}
