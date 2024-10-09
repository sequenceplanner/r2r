use crate::{
    trace_callback_register, trace_service_callback_added, trace_subscription_callback_added,
    trace_timer_callback_added, TracingId,
};
use r2r_rcl::{rcl_service_t, rcl_timer_t};
use std::{
    any::type_name,
    marker::PhantomData,
    sync::atomic::{AtomicUsize, Ordering::Relaxed},
};

/// Tracing wrapper for callback
pub struct Callback<F, M>
where
    F: FnMut(M),
{
    func: F,
    id: usize,
    msg_type: PhantomData<M>,
}

impl<F, M> Callback<F, M>
where
    F: FnMut(M),
{
    /// Generates unique ID for the callback
    fn gen_id() -> usize {
        static COUNTER: AtomicUsize = AtomicUsize::new(1);
        COUNTER.fetch_add(1, Relaxed)
    }

    fn new(callback: F, id: usize) -> Self {
        trace_callback_register(id, type_name::<F>());

        Self {
            func: callback,
            id,
            msg_type: PhantomData,
        }
    }

    /// Emits trace event associating this `callback` with the `service`.
    ///
    /// Wraps the callback to allow tracing the callback calls.
    pub fn new_service(service: *const rcl_service_t, callback: F) -> Self {
        let id = Self::gen_id();
        trace_service_callback_added(service, id);

        Self::new(callback, id)
    }

    /// Emits trace event associating this `callback` with the `timer`.
    ///
    /// Wraps the callback to allow tracing the callback calls.
    pub fn new_timer(timer: TracingId<rcl_timer_t>, callback: F) -> Self {
        let id = Self::gen_id();
        trace_timer_callback_added(timer, id);

        Self::new(callback, id)
    }

    /// Emits trace event associating this `callback` with the `subscription`.
    ///
    /// Wraps the callback to allow tracing the callback calls.
    pub fn new_subscription<S>(subscriber: &S, callback: F) -> Self {
        let id = Self::gen_id();
        trace_subscription_callback_added(subscriber, id);

        Self::new(callback, id)
    }

    /// Call the `callback`.
    /// This emits `ros2:callback_start` and `ros2:callback_end` events at
    /// the beginning and end respectively.
    pub fn call(&mut self, msg: M) {
        crate::trace_callback_start(self.id, false);
        (self.func)(msg);
        crate::trace_callback_end(self.id);
    }
}
