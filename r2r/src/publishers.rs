use std::ffi::c_void;
use std::ffi::CString;
use std::fmt::Debug;
use std::marker::PhantomData;
use std::sync::Once;
use std::sync::Weak;

use crate::error::*;
use crate::msg_types::*;
use crate::qos::QosProfile;
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

/// A ROS (typed) publisher.
///
/// This contains a `Weak Arc` to a typed publisher. As such it is safe to
/// move between threads.
#[derive(Debug, Clone)]
pub struct Publisher<T>
where
    T: WrappedTypesupport,
{
    handle: Weak<rcl_publisher_t>,
    type_: PhantomData<T>,
}

unsafe impl Send for PublisherUntyped {}

/// A ROS (untyped) publisher.
///
/// This contains a `Weak Arc` to an "untyped" publisher. As such it is safe to
/// move between threads.
#[derive(Debug, Clone)]
pub struct PublisherUntyped {
    handle: Weak<rcl_publisher_t>,
    type_: String,
}

pub fn make_publisher<T>(handle: Weak<rcl_publisher_t>) -> Publisher<T>
where
    T: WrappedTypesupport,
{
    Publisher {
        handle,
        type_: PhantomData,
    }
}

pub fn make_publisher_untyped(handle: Weak<rcl_publisher_t>, type_: String) -> PublisherUntyped {
    PublisherUntyped { handle, type_ }
}

pub fn create_publisher_helper(
    node: &mut rcl_node_t, topic: &str, typesupport: *const rosidl_message_type_support_t,
    qos_profile: QosProfile,
) -> Result<rcl_publisher_t> {
    let mut publisher_handle = unsafe { rcl_get_zero_initialized_publisher() };
    let topic_c_string = CString::new(topic).map_err(|_| Error::RCL_RET_INVALID_ARGUMENT)?;

    let result = unsafe {
        let mut publisher_options = rcl_publisher_get_default_options();
        publisher_options.qos = qos_profile.into();
        rcl_publisher_init(
            &mut publisher_handle,
            node,
            typesupport,
            topic_c_string.as_ptr(),
            &publisher_options,
        )
    };
    if result == RCL_RET_OK as i32 {
        Ok(publisher_handle)
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

        let result =
            unsafe { rcl_publish(publisher.as_ref(), native_msg.void_ptr(), std::ptr::null_mut()) };

        if result == RCL_RET_OK as i32 {
            Ok(())
        } else {
            log::error!("could not publish {}", result);
            Err(Error::from_rcl_error(result))
        }
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
        let result =
            unsafe { rcl_publish(publisher.as_ref(), native_msg.void_ptr(), std::ptr::null_mut()) };

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

        if unsafe { rcl_publisher_can_loan_messages(publisher.as_ref()) } {
            let mut loaned_msg: *mut c_void = std::ptr::null_mut();
            let ret = unsafe {
                rcl_borrow_loaned_message(publisher.as_ref(), T::get_ts(), &mut loaned_msg)
            };
            if ret != RCL_RET_OK as i32 {
                log::error!("Failed getting loaned message");
                return Err(Error::from_rcl_error(ret));
            }

            let handle_box = Box::new(*publisher.as_ref());
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

        let result = if msg.is_loaned {
            unsafe {
                // signal that we are relinquishing responsibility of the memory
                msg.release();

                // publish and return loaned message to middleware
                rcl_publish_loaned_message(
                    publisher.as_ref(),
                    msg.void_ptr_mut(),
                    std::ptr::null_mut(),
                )
            }
        } else {
            unsafe { rcl_publish(publisher.as_ref(), msg.void_ptr(), std::ptr::null_mut()) }
        };

        if result == RCL_RET_OK as i32 {
            Ok(())
        } else {
            log::error!("could not publish native {}", result);
            Err(Error::from_rcl_error(result))
        }
    }
}
