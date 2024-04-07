use std::{fmt::Debug, mem::MaybeUninit, time::Duration};

use crate::{error::*, msg_types::generated_msgs::builtin_interfaces};
use r2r_rcl::*;

/// Different ROS clock types.
#[derive(Debug, Copy, Clone)]
pub enum ClockType {
    RosTime,
    SystemTime,
    SteadyTime,
}

unsafe impl Send for Clock {}

/// A ROS clock.
pub struct Clock {
    pub(crate) clock_handle: Box<rcl_clock_t>,
    clock_type: ClockType,
}

pub fn clock_type_to_rcl(ct: &ClockType) -> rcl_clock_type_t {
    match ct {
        ClockType::RosTime => rcl_clock_type_t::RCL_ROS_TIME,
        ClockType::SystemTime => rcl_clock_type_t::RCL_SYSTEM_TIME,
        ClockType::SteadyTime => rcl_clock_type_t::RCL_STEADY_TIME,
    }
}

impl Clock {
    /// Create a new clock with the specified type.
    pub fn create(ct: ClockType) -> Result<Clock> {
        let mut clock_handle = MaybeUninit::<rcl_clock_t>::uninit();

        let rcl_ct = clock_type_to_rcl(&ct);
        let ret = unsafe {
            rcl_clock_init(rcl_ct, clock_handle.as_mut_ptr(), &mut rcutils_get_default_allocator())
        };
        if ret != RCL_RET_OK as i32 {
            log::error!("could not create {:?} clock: {}", ct, ret);
            return Err(Error::from_rcl_error(ret));
        }

        let clock_handle = Box::new(unsafe { clock_handle.assume_init() });
        Ok(Clock {
            clock_handle,
            clock_type: ct,
        })
    }

    pub fn get_now(&mut self) -> Result<Duration> {
        let valid = unsafe { rcl_clock_valid(&mut *self.clock_handle) };
        if !valid {
            return Err(Error::from_rcl_error(RCL_RET_INVALID_ARGUMENT as i32));
        }
        let mut tp: rcutils_time_point_value_t = 0;
        let ret = unsafe { rcl_clock_get_now(&mut *self.clock_handle, &mut tp) };

        if ret != RCL_RET_OK as i32 {
            log::error!("could not create steady clock: {}", ret);
            return Err(Error::from_rcl_error(ret));
        }

        let dur = Duration::from_nanos(tp as u64);

        Ok(dur)
    }

    pub fn get_clock_type(&self) -> ClockType {
        self.clock_type
    }

    /// TODO: move to builtin helper methods module.
    pub fn to_builtin_time(d: &Duration) -> builtin_interfaces::msg::Time {
        let sec = d.as_secs() as i32;
        let nanosec = d.subsec_nanos();
        builtin_interfaces::msg::Time { sec, nanosec }
    }

    /// Enables alternative source of time for this clock
    ///
    /// The clock must be [`ClockType::RosTime`].
    ///
    /// Wrapper for `rcl_enable_ros_time_override`
    #[cfg(r2r__rosgraph_msgs__msg__Clock)]
    pub(crate) fn enable_ros_time_override(
        &mut self, initial_time: rcl_time_point_value_t,
    ) -> Result<()> {
        let valid = unsafe { rcl_clock_valid(&mut *self.clock_handle) };
        if !valid {
            return Err(Error::from_rcl_error(RCL_RET_INVALID_ARGUMENT as i32));
        }

        let ret = unsafe { rcl_enable_ros_time_override(&mut *self.clock_handle) };
        if ret != RCL_RET_OK as i32 {
            log::error!("could not enable ros time override: {}", ret);
            return Err(Error::from_rcl_error(ret));
        }

        self.set_ros_time_override(initial_time)?;

        Ok(())
    }

    /// Disables alternative source of time for this clock
    ///
    /// The clock must be [`ClockType::RosTime`].
    ///
    /// Wrapper for `rcl_disable_ros_time_override`
    #[cfg(r2r__rosgraph_msgs__msg__Clock)]
    pub(crate) fn disable_ros_time_override(&mut self) -> Result<()> {
        let valid = unsafe { rcl_clock_valid(&mut *self.clock_handle) };
        if !valid {
            return Err(Error::from_rcl_error(RCL_RET_INVALID_ARGUMENT as i32));
        }

        let ret = unsafe { rcl_disable_ros_time_override(&mut *self.clock_handle) };
        if ret != RCL_RET_OK as i32 {
            log::error!("could not disable ros time override: {}", ret);
            return Err(Error::from_rcl_error(ret));
        }

        Ok(())
    }

    /// Sets new time value if the clock has enabled alternative time source
    ///
    /// If the clock does not have alternative time source enabled this function will not change the time.
    ///
    /// The clock must be [`ClockType::RosTime`].
    ///
    /// Wrapper for `rcl_set_ros_time_override`
    #[cfg(r2r__rosgraph_msgs__msg__Clock)]
    pub(crate) fn set_ros_time_override(&mut self, time: rcl_time_point_value_t) -> Result<()> {
        let valid = unsafe { rcl_clock_valid(&mut *self.clock_handle) };
        if !valid {
            return Err(Error::from_rcl_error(RCL_RET_INVALID_ARGUMENT as i32));
        }

        let ret = unsafe { rcl_set_ros_time_override(&mut *self.clock_handle, time) };
        if ret != RCL_RET_OK as i32 {
            log::error!("could not set ros time override: {}", ret);
            return Err(Error::from_rcl_error(ret));
        }

        Ok(())
    }
}

impl From<builtin_interfaces::msg::Time> for rcutils_time_point_value_t {
    fn from(msg: builtin_interfaces::msg::Time) -> Self {
        (msg.sec as rcl_time_point_value_t) * 1_000_000_000
            + (msg.nanosec as rcl_time_point_value_t)
    }
}

impl Drop for Clock {
    fn drop(&mut self) {
        unsafe {
            rcl_clock_fini(&mut *self.clock_handle);
        }
    }
}
