use std::fmt::Debug;
use std::mem::MaybeUninit;
use std::time::Duration;

use crate::error::*;
use crate::msg_types::generated_msgs::builtin_interfaces;
use r2r_rcl::*;

/// Different ROS clock types.
#[derive(Debug)]
pub enum ClockType {
    RosTime,
    SystemTime,
    SteadyTime,
}

unsafe impl Send for Clock {}

/// A ROS clock.
pub struct Clock {
    pub(crate) clock_handle: Box<rcl_clock_t>,
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
            rcl_clock_init(
                rcl_ct,
                clock_handle.as_mut_ptr(),
                &mut rcutils_get_default_allocator(),
            )
        };
        if ret != RCL_RET_OK as i32 {
            eprintln!("could not create {:?} clock: {}", ct, ret);
            return Err(Error::from_rcl_error(ret));
        }

        let clock_handle = Box::new(unsafe { clock_handle.assume_init() });
        Ok(Clock { clock_handle })
    }

    pub fn get_now(&mut self) -> Result<Duration> {
        let valid = unsafe { rcl_clock_valid(&mut *self.clock_handle) };
        if !valid {
            return Err(Error::from_rcl_error(RCL_RET_INVALID_ARGUMENT as i32));
        }
        let mut tp: rcutils_time_point_value_t = 0;
        let ret = unsafe { rcl_clock_get_now(&mut *self.clock_handle, &mut tp) };

        if ret != RCL_RET_OK as i32 {
            eprintln!("could not create steady clock: {}", ret);
            return Err(Error::from_rcl_error(ret));
        }

        let dur = Duration::from_nanos(tp as u64);

        Ok(dur)
    }

    /// TODO: move to builtin helper methods module.
    pub fn to_builtin_time(d: &Duration) -> builtin_interfaces::msg::Time {
        let sec = d.as_secs() as i32;
        let nanosec = d.subsec_nanos();
        builtin_interfaces::msg::Time { sec, nanosec }
    }
}

impl Drop for Clock {
    fn drop(&mut self) {
        unsafe {
            rcl_clock_fini(&mut *self.clock_handle);
        }
    }
}
