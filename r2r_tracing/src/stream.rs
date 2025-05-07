use std::{
    ffi::c_void,
    future::{self, Future},
};

use futures::{channel::mpsc::Receiver, stream::FusedStream, Stream, StreamExt as _};
use r2r_rcl::rcl_service_t;

#[cfg(feature = "tracing")]
use crate::Callback;
use crate::TracingId;

/// A stream wrapper containing tracing data.
///
/// When the `tracing` feature is enabled, you can use the [`Self::traced_callback`] method
/// to trace the execution of the callback. Stream polling is not traced.
#[derive(Debug)]
pub struct StreamWithTracingData<T> {
    stream: Receiver<T>,

    #[cfg(feature = "tracing")]
    tracing_id: TracingIdWithType,
}

#[cfg(feature = "tracing")]
#[derive(Debug, Clone, Copy)]
enum TracingIdWithType {
    // The type of subscription is `c_void` because the actual type would
    // be a generic R2R subscriber and not `rcl_subscription_t`.
    Subscription(TracingId<c_void>),
    Service(TracingId<rcl_service_t>),
}

impl<T> StreamWithTracingData<T> {
    /// Converts the stream into a future that calls the provided callback for each message.
    ///
    /// If `tracing` feature is enabled:
    /// - Each time the callback is called, its start and end time will be traced by the
    ///   ROS 2 tracing framework.
    /// - You should not poll the stream before calling this method because
    ///   otherwise it might confuse software that analyzes the trace.
    pub fn traced_callback<C>(self, callback: C) -> impl Future<Output = ()> + Unpin
    where
        C: FnMut(T),
    {
        #[cfg(feature = "tracing")]
        {
            let mut callback_wrapper = match self.tracing_id {
                TracingIdWithType::Subscription(id) => Callback::new_subscription(id, callback),
                TracingIdWithType::Service(id) => Callback::new_service(id, callback),
            };

            self.stream.for_each(move |msg| {
                callback_wrapper.call(msg);
                future::ready(())
            })
        }
        #[cfg(not(feature = "tracing"))]
        {
            let mut callback = callback;
            self.stream.for_each(move |msg| {
                callback(msg);
                future::ready(())
            })
        }
    }
}

impl<T> Stream for StreamWithTracingData<T> {
    type Item = T;

    fn poll_next(
        self: std::pin::Pin<&mut Self>, cx: &mut std::task::Context<'_>,
    ) -> std::task::Poll<Option<Self::Item>> {
        let this = self.get_mut();
        this.stream.poll_next_unpin(cx)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.stream.size_hint()
    }
}

impl<T> FusedStream for StreamWithTracingData<T> {
    fn is_terminated(&self) -> bool {
        self.stream.is_terminated()
    }
}

/// A builder for `StreamWithTracingData`.
///
/// This struct exists to allow creation of `StreamWithTracingData` without polluting its public API.
/// It is used internally by r2r and should not be reexported.
pub struct StreamWithTracingDataBuilder;

impl StreamWithTracingDataBuilder {
    #[must_use]
    #[allow(
        clippy::not_unsafe_ptr_arg_deref,
        reason = "The pointer is not dereferenced"
    )]
    #[cfg_attr(
        not(feature = "tracing"),
        expect(
            unused_variables,
            reason = "service_id is not saved if tracing is disabled"
        )
    )]
    pub fn build_service<T>(
        stream: Receiver<T>, service_id: TracingId<rcl_service_t>,
    ) -> StreamWithTracingData<T> {
        StreamWithTracingData {
            stream,
            #[cfg(feature = "tracing")]
            tracing_id: TracingIdWithType::Service(service_id),
        }
    }

    #[must_use]
    #[cfg_attr(
        not(feature = "tracing"),
        expect(
            unused_variables,
            reason = "subscription_id is not saved if tracing is disabled"
        )
    )]
    pub fn build_subscription<T>(
        stream: Receiver<T>, subscription_id: TracingId<c_void>,
    ) -> StreamWithTracingData<T> {
        StreamWithTracingData {
            stream,
            #[cfg(feature = "tracing")]
            tracing_id: TracingIdWithType::Subscription(subscription_id),
        }
    }
}
