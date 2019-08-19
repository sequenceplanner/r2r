#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]
#![allow(dead_code)]
pub mod rcl_bindings;
pub use rcl_bindings::*;

use std::ffi::CStr;
use std::ffi::CString;


// special treatment to convert to/from rust strings.
// ros strings are owned by ros, assignment is a copy
impl rosidl_generator_c__String {
    pub fn to_str(&self) -> &str {
        let s = unsafe { CStr::from_ptr(self.data as *mut i8) };
        s.to_str().unwrap_or("")
    }

    pub fn assign(&mut self, other: &str) -> () {
        let q = CString::new(other).unwrap();
        let to_send_ptr = q.as_ptr() as *const i8;
        unsafe {
            rosidl_generator_c__String__assign(self as *mut _, to_send_ptr);
        }
    }
}

impl Default for rmw_message_info_t {
    fn default() -> Self {
        rmw_message_info_t {
            publisher_gid: rmw_gid_t {
                implementation_identifier: std::ptr::null(),
                data: [0; 24],
            },
            from_intra_process: false,
        }
    }
}

impl Default for rmw_qos_profile_t {
    fn default() -> Self {
        rmw_qos_profile_t {
            history: rmw_qos_history_policy_t::RMW_QOS_POLICY_HISTORY_SYSTEM_DEFAULT,
            depth: 10,
            reliability: rmw_qos_reliability_policy_t::RMW_QOS_POLICY_RELIABILITY_SYSTEM_DEFAULT,
            durability: rmw_qos_durability_policy_t::RMW_QOS_POLICY_DURABILITY_SYSTEM_DEFAULT,
            avoid_ros_namespace_conventions: false,
            deadline: rmw_time_t { sec: 0, nsec: 0 },
            lifespan: rmw_time_t { sec: 0, nsec: 0 },
            liveliness: rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
            liveliness_lease_duration: rmw_time_t { sec: 0, nsec: 0 },
        }
    }
}

