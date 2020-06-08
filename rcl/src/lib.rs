#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]
#![allow(dead_code)]
include!(concat!(env!("OUT_DIR"), "/rcl_bindings.rs"));

use std::ffi::CStr;
use std::ffi::CString;

impl Default for rmw_message_info_t {
    fn default() -> Self {
        rmw_message_info_t {
            source_timestamp: 0,
            received_timestamp: 0,
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


// special treatment to convert to/from rust strings.
// ros strings are owned by ros, assignment is a copy
impl rosidl_runtime_c__String {
    pub fn to_str(&self) -> &str {
        let s = unsafe { CStr::from_ptr(self.data as *mut i8) };
        s.to_str().unwrap_or("")
    }

    pub fn assign(&mut self, other: &str) -> () {
        let q = CString::new(other).unwrap();
        let to_send_ptr = q.as_ptr() as *const i8;
        unsafe {
            rosidl_runtime_c__String__assign(self as *mut _, to_send_ptr);
        }
    }
}

use widestring::U16String;
impl rosidl_runtime_c__U16String {
    pub fn to_str(&self) -> String {
        let s = unsafe { U16String::from_ptr(self.data, self.size) };
        // U16Str = U16String::from_ptr(buffer, strlen as usize);
        // let s = unsafe { CStr::from_ptr(self.data as *mut i8) };
        //s.to_str().unwrap_or("")
        s.to_string_lossy()
    }

    pub fn assign(&mut self, other: &str) -> () {
        let wstr = U16String::from_str(other);
        let to_send_ptr = wstr.as_ptr() as *const uint_least16_t;
        unsafe {
            rosidl_runtime_c__U16String__assignn(self as *mut _, to_send_ptr, wstr.len());
        }
    }
}

impl rosidl_runtime_c__U16String__Sequence {
    pub fn update(&mut self, values: &[String]) {
        unsafe { rosidl_runtime_c__U16String__Sequence__fini(self as *mut _); }
        unsafe { rosidl_runtime_c__U16String__Sequence__init(self as *mut _, values.len()); }
        let strs = unsafe { std::slice::from_raw_parts_mut(self.data, values.len()) };
        for (target, source) in strs.iter_mut().zip(values) {
            target.assign(&source);
        }
    }

    pub fn to_vec(&self) -> Vec<String> {
        let mut target = Vec::with_capacity(self.size);
        let strs = unsafe { std::slice::from_raw_parts(self.data, self.size) };
        for s in strs {
            target.push(s.to_str().to_owned());
        }
        target
    }
}

impl rosidl_runtime_c__String__Sequence {
    pub fn update(&mut self, values: &[String]) {
        unsafe { rosidl_runtime_c__String__Sequence__fini(self as *mut _); }
        unsafe { rosidl_runtime_c__String__Sequence__init(self as *mut _, values.len()); }
        let strs = unsafe { std::slice::from_raw_parts_mut(self.data, values.len()) };
        for (target, source) in strs.iter_mut().zip(values) {
            target.assign(&source);
        }
    }

    pub fn to_vec(&self) -> Vec<String> {
        let mut target = Vec::with_capacity(self.size);
        let strs = unsafe { std::slice::from_raw_parts(self.data, self.size) };
        for s in strs {
            target.push(s.to_str().to_owned());
        }
        target
    }
}

// conversions from/to vectors of built in types

macro_rules! primitive_sequence {
    ($ctype:ident, $element_type:ident) => {
        paste::item! {
            impl [<$ctype __Sequence>] {
                pub fn update(&mut self, values: &[$element_type]) {
                    unsafe { [<$ctype __Sequence__fini>] (self as *mut _); }
                    unsafe { [<$ctype __Sequence__init>] (self as *mut _, values.len()); }
                    unsafe { std::ptr::copy(values.as_ptr(), self.data, values.len()); }
                }

                pub fn to_vec(&self) -> Vec<$element_type> {
                    let mut target = Vec::with_capacity(self.size);
                    unsafe { target.set_len(self.size); }
                    unsafe { std::ptr::copy(self.data, target.as_mut_ptr(), self.size); }
                    target
                }
            }
        }
    }
}

primitive_sequence!(rosidl_runtime_c__float32, f32);
primitive_sequence!(rosidl_runtime_c__float64, f64);
primitive_sequence!(rosidl_runtime_c__long_double, u128);
primitive_sequence!(rosidl_runtime_c__char, i8);
primitive_sequence!(rosidl_runtime_c__wchar, u16);
primitive_sequence!(rosidl_runtime_c__boolean, bool);
primitive_sequence!(rosidl_runtime_c__octet, u8);
primitive_sequence!(rosidl_runtime_c__uint8, u8);
primitive_sequence!(rosidl_runtime_c__int8, i8);
primitive_sequence!(rosidl_runtime_c__uint16, u16);
primitive_sequence!(rosidl_runtime_c__int16, i16);
primitive_sequence!(rosidl_runtime_c__uint32, u32);
primitive_sequence!(rosidl_runtime_c__int32, i32);
primitive_sequence!(rosidl_runtime_c__uint64, u64);
primitive_sequence!(rosidl_runtime_c__int64, i64);
