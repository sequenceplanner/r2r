use futures::{channel::oneshot, Future, TryFutureExt};
use std::{
    ffi::{c_void, CString},
    fmt::Debug,
    marker::PhantomData,
    sync::{Arc, Mutex, Once, Weak},
};

use crate::{error::*, msg_types::*, qos::QosProfile};
use r2r_rcl::*;

// The publish function is thread safe. ROS2 docs state:
// =============
//
// This function is thread safe so long as access to both the
// publisher and the" `ros_message` is synchronized."  That means that
// calling rcl_publish() from multiple threads is allowed, but"
// calling rcl_publish() at the same time as non-thread safe
// publisher" functions is not, e.g. calling rcl_publish() and
// rcl_publisher_fini()" concurrently is not allowed."  Before calling
// rcl_publish() the message can change and after calling"
// rcl_publish() the message can change, but it cannot be changed
// during the" publish call."  The same `ros_message`, however, can be
// passed to multiple calls of" rcl_publish() simultaneously, even if
// the publishers differ."  The `ros_message` is unmodified by
// rcl_publish()."
//
// TODO: I guess there is a potential error source in destructuring
// while calling publish. I don't think its worth to protect with a
// mutex/rwlock for this though...
//
// Methods that mutate need to called from the thread owning the Node.
// I don't think we can count on Node being generally thread-safe.
// So keep pub/sub management and polling contained to one thread
// and send out publishers.

unsafe impl<T> Send for Publisher<T> where T: WrappedTypesupport {}

pub(crate) struct Publisher_ {
    handle: rcl_publisher_t,

    // TODO use a mpsc to avoid the mutex?
    poll_inter_process_subscriber_channels: Mutex<Vec<oneshot::Sender<()>>>,
}

impl Publisher_ {
    fn get_inter_process_subscription_count(&self) -> Result<usize> {
        // See https://github.com/ros2/rclcpp/issues/623

        let mut inter_process_subscription_count = 0;

        let result = unsafe {
            rcl_publisher_get_subscription_count(
                &self.handle as *const rcl_publisher_t,
                &mut inter_process_subscription_count as *mut usize,
            )
        };

        if result == RCL_RET_OK as i32 {
            Ok(inter_process_subscription_count)
        } else {
            Err(Error::from_rcl_error(result))
        }
    }

    pub(crate) fn poll_has_inter_process_subscribers(&self) {
        let mut poll_inter_process_subscriber_channels =
            self.poll_inter_process_subscriber_channels.lock().unwrap();

        if poll_inter_process_subscriber_channels.is_empty() {
            return;
        }
        let inter_process_subscription_count = self.get_inter_process_subscription_count();
        match inter_process_subscription_count {
            Ok(0) => {
                // not available...
            }
            Ok(_) => {
                // send ok and close channels
                while let Some(sender) = poll_inter_process_subscriber_channels.pop() {
                    let _res = sender.send(()); // we ignore if receiver dropped.
                }
            }
            Err(_) => {
                // error, close all channels
                poll_inter_process_subscriber_channels.clear();
            }
        }
    }

    pub(crate) fn destroy(mut self, node: &mut rcl_node_t) {
        let _ret = unsafe { rcl_publisher_fini(&mut self.handle as *mut _, node) };

        // TODO: check ret
    }
}

/// A ROS (typed) publisher.
///
/// This contains a `Weak Arc` to a typed publisher. As such it is safe to
/// move between threads.
#[derive(Debug, Clone)]
pub struct Publisher<T>
where
    T: WrappedTypesupport,
{
    pub(crate) handle: Weak<Publisher_>,
    type_: PhantomData<T>,
}

unsafe impl Send for PublisherUntyped {}

/// A ROS (untyped) publisher.
///
/// This contains a `Weak Arc` to an "untyped" publisher. As such it is safe to
/// move between threads.
#[derive(Debug, Clone)]
pub struct PublisherUntyped {
    pub(crate) handle: Weak<Publisher_>,
    type_: String,
}

pub fn make_publisher<T>(handle: Weak<Publisher_>) -> Publisher<T>
where
    T: WrappedTypesupport,
{
    Publisher {
        handle,
        type_: PhantomData,
    }
}

pub fn make_publisher_untyped(handle: Weak<Publisher_>, type_: String) -> PublisherUntyped {
    PublisherUntyped { handle, type_ }
}

pub fn create_publisher_helper(
    node: &mut rcl_node_t, topic: &str, typesupport: *const rosidl_message_type_support_t,
    qos_profile: QosProfile,
) -> Result<Arc<Publisher_>> {
    let topic_c_string = CString::new(topic).map_err(|_| Error::RCL_RET_INVALID_ARGUMENT)?;

    // Allocate the memory now so that the location of the rcl handle
    // does not change after call to rcl_publisher_init.
    // This is important because tracing in rcl expects the handle to be at a fixed location.
    let mut publisher_arc = Arc::new(Publisher_ {
        handle: unsafe { rcl_get_zero_initialized_publisher() },
        poll_inter_process_subscriber_channels: Mutex::new(Vec::new()),
    });
    let publisher_mut = Arc::get_mut(&mut publisher_arc)
        .expect("No other Arc should exist. The Arc was just created.");

    let result = unsafe {
        let mut publisher_options = rcl_publisher_get_default_options();
        publisher_options.qos = qos_profile.into();
        rcl_publisher_init(
            &mut publisher_mut.handle,
            node,
            typesupport,
            topic_c_string.as_ptr(),
            &publisher_options,
        )
    };
    if result == RCL_RET_OK as i32 {
        Ok(publisher_arc)
    } else {
        Err(Error::from_rcl_error(result))
    }
}

impl PublisherUntyped {
    /// Publish an "untyped" ROS message represented by a `serde_json::Value`.
    ///
    /// It is up to the user to make sure the fields are correct.
    pub fn publish(&self, msg: serde_json::Value) -> Result<()> {
        // upgrade to actual ref. if still alive
        let publisher = self
            .handle
            .upgrade()
            .ok_or(Error::RCL_RET_PUBLISHER_INVALID)?;

        let native_msg = WrappedNativeMsgUntyped::new_from(&self.type_)?;
        native_msg.from_json(msg)?;

        r2r_tracing::trace_publish(native_msg.void_ptr());

        let result = unsafe {
            rcl_publish(
                &publisher.handle as *const rcl_publisher_t,
                native_msg.void_ptr(),
                std::ptr::null_mut(),
            )
        };

        if result == RCL_RET_OK as i32 {
            Ok(())
        } else {
            log::error!("could not publish {}", result);
            Err(Error::from_rcl_error(result))
        }
    }

    /// Publish an pre-serialized ROS message represented by a `&[u8]`.
    ///
    /// It is up to the user to make sure data is a valid ROS serialized message.
    pub fn publish_raw(&self, data: &[u8]) -> Result<()> {
        // TODO should this be an unsafe function? I'm not sure what happens if the data is malformed ..

        // upgrade to actual ref. if still alive
        let publisher = self
            .handle
            .upgrade()
            .ok_or(Error::RCL_RET_PUBLISHER_INVALID)?;

        // Safety: Not retained beyond this function
        let msg_buf = rcl_serialized_message_t {
            buffer: data.as_ptr() as *mut u8,
            buffer_length: data.len(),
            buffer_capacity: data.len(),

            // Since its read only, this should never be used ..
            allocator: unsafe { rcutils_get_default_allocator() },
        };

        r2r_tracing::trace_publish((&msg_buf as *const rcl_serialized_message_t).cast::<c_void>());

        let result = unsafe {
            rcl_publish_serialized_message(
                &publisher.handle,
                &msg_buf as *const rcl_serialized_message_t,
                std::ptr::null_mut(),
            )
        };

        if result == RCL_RET_OK as i32 {
            Ok(())
        } else {
            log::error!("could not publish {}", result);
            Err(Error::from_rcl_error(result))
        }
    }

    /// Gets the number of external subscribers (i.e. it doesn't
    /// count subscribers from the same process).
    pub fn get_inter_process_subscription_count(&self) -> Result<usize> {
        self.handle
            .upgrade()
            .ok_or(Error::RCL_RET_PUBLISHER_INVALID)?
            .get_inter_process_subscription_count()
    }

    /// Waits for at least one external subscriber to begin subscribing to the
    /// topic. It doesn't count subscribers from the same process.
    pub fn wait_for_inter_process_subscribers(&self) -> Result<impl Future<Output = Result<()>>> {
        let (sender, receiver) = oneshot::channel();

        self.handle
            .upgrade()
            .ok_or(Error::RCL_RET_PUBLISHER_INVALID)?
            .poll_inter_process_subscriber_channels
            .lock()
            .unwrap()
            .push(sender);

        Ok(receiver.map_err(|_| Error::RCL_RET_CLIENT_INVALID))
    }
}

impl<T: 'static> Publisher<T>
where
    T: WrappedTypesupport,
{
    /// Publish a ROS message.
    pub fn publish(&self, msg: &T) -> Result<()>
    where
        T: WrappedTypesupport,
    {
        // upgrade to actual ref. if still alive
        let publisher = self
            .handle
            .upgrade()
            .ok_or(Error::RCL_RET_PUBLISHER_INVALID)?;
        let native_msg: WrappedNativeMsg<T> = WrappedNativeMsg::<T>::from(msg);

        r2r_tracing::trace_publish(native_msg.void_ptr());

        let result = unsafe {
            rcl_publish(
                &publisher.handle as *const rcl_publisher_t,
                native_msg.void_ptr(),
                std::ptr::null_mut(),
            )
        };

        if result == RCL_RET_OK as i32 {
            Ok(())
        } else {
            log::error!("could not publish {}", result);
            Err(Error::from_rcl_error(result))
        }
    }

    pub fn borrow_loaned_message(&self) -> Result<WrappedNativeMsg<T>>
    where
        T: WrappedTypesupport,
    {
        // upgrade to actual ref. if still alive
        let publisher = self
            .handle
            .upgrade()
            .ok_or(Error::RCL_RET_PUBLISHER_INVALID)?;

        if unsafe { rcl_publisher_can_loan_messages(&publisher.handle as *const rcl_publisher_t) } {
            let mut loaned_msg: *mut c_void = std::ptr::null_mut();
            let ret = unsafe {
                rcl_borrow_loaned_message(
                    &publisher.handle as *const rcl_publisher_t,
                    T::get_ts(),
                    &mut loaned_msg,
                )
            };
            if ret != RCL_RET_OK as i32 {
                log::error!("Failed getting loaned message");
                return Err(Error::from_rcl_error(ret));
            }

            let handle_box = Box::new(publisher.handle);
            let msg = WrappedNativeMsg::<T>::from_loaned(
                loaned_msg as *mut T::CStruct,
                Box::new(|msg: *mut T::CStruct| {
                    let ret = unsafe {
                        let handle_ptr = Box::into_raw(handle_box);
                        let ret = rcl_return_loaned_message_from_publisher(
                            handle_ptr,
                            msg as *mut c_void,
                        );
                        drop(Box::from_raw(handle_ptr));
                        ret
                    };

                    if ret != RCL_RET_OK as i32 {
                        panic!("rcl_deallocate_loaned_message failed");
                    }
                }),
            );
            Ok(msg)
        } else {
            static LOG_LOANED_ERROR: Once = Once::new();
            LOG_LOANED_ERROR.call_once(|| {
                log::error!(
                    "Currently used middleware can't loan messages. Local allocator will be used."
                );
            });

            Ok(WrappedNativeMsg::<T>::new())
        }
    }

    /// Publish a "native" ROS message.
    ///
    /// This function is useful if you want to bypass the generated
    /// rust types as it lets you work with the raw C struct.
    pub fn publish_native(&self, msg: &mut WrappedNativeMsg<T>) -> Result<()>
    where
        T: WrappedTypesupport,
    {
        // upgrade to actual ref. if still alive
        let publisher = self
            .handle
            .upgrade()
            .ok_or(Error::RCL_RET_PUBLISHER_INVALID)?;

        r2r_tracing::trace_publish(msg.void_ptr());

        let result = if msg.is_loaned {
            unsafe {
                // signal that we are relinquishing responsibility of the memory
                msg.release();

                // publish and return loaned message to middleware
                rcl_publish_loaned_message(
                    &publisher.handle as *const rcl_publisher_t,
                    msg.void_ptr_mut(),
                    std::ptr::null_mut(),
                )
            }
        } else {
            unsafe {
                rcl_publish(
                    &publisher.handle as *const rcl_publisher_t,
                    msg.void_ptr(),
                    std::ptr::null_mut(),
                )
            }
        };

        if result == RCL_RET_OK as i32 {
            Ok(())
        } else {
            log::error!("could not publish native {}", result);
            Err(Error::from_rcl_error(result))
        }
    }

    /// Gets the number of external subscribers (i.e. it doesn't
    /// count subscribers from the same process).
    pub fn get_inter_process_subscription_count(&self) -> Result<usize> {
        self.handle
            .upgrade()
            .ok_or(Error::RCL_RET_PUBLISHER_INVALID)?
            .get_inter_process_subscription_count()
    }

    /// Waits for at least one external subscriber to begin subscribing to the
    /// topic. It doesn't count subscribers from the same process.
    pub fn wait_for_inter_process_subscribers(&self) -> Result<impl Future<Output = Result<()>>> {
        let (sender, receiver) = oneshot::channel();

        self.handle
            .upgrade()
            .ok_or(Error::RCL_RET_PUBLISHER_INVALID)?
            .poll_inter_process_subscriber_channels
            .lock()
            .unwrap()
            .push(sender);

        Ok(receiver.map_err(|_| Error::RCL_RET_CLIENT_INVALID))
    }
}
