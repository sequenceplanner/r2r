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
        unsafe { rmw_get_zero_initialized_message_info() }
    }
}

// special treatment to convert to/from rust strings.
// ros strings are owned by ros, assignment is a copy
impl rosidl_runtime_c__String {
    pub fn to_str(&self) -> &str {
        let s = unsafe { CStr::from_ptr(self.data) };
        s.to_str().unwrap_or("")
    }

    pub fn assign(&mut self, other: &str) {
        let q = CString::new(other).unwrap();
        unsafe {
            rosidl_runtime_c__String__assign(self as *mut _, q.as_ptr());
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

    pub fn assign(&mut self, other: &str) {
        let wstr = U16String::from_str(other);
        let to_send_ptr = wstr.as_ptr() as *const uint_least16_t;
        unsafe {
            rosidl_runtime_c__U16String__assignn(self as *mut _, to_send_ptr, wstr.len());
        }
    }
}

impl rosidl_runtime_c__U16String__Sequence {
    pub fn update(&mut self, values: &[String]) {
        unsafe {
            rosidl_runtime_c__U16String__Sequence__fini(self as *mut _);
        }
        unsafe {
            rosidl_runtime_c__U16String__Sequence__init(self as *mut _, values.len());
        }
        let strs = unsafe { std::slice::from_raw_parts_mut(self.data, values.len()) };
        for (target, source) in strs.iter_mut().zip(values) {
            target.assign(source);
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
        unsafe {
            rosidl_runtime_c__String__Sequence__fini(self as *mut _);
        }
        unsafe {
            rosidl_runtime_c__String__Sequence__init(self as *mut _, values.len());
        }
        let strs = unsafe { std::slice::from_raw_parts_mut(self.data, values.len()) };
        for (target, source) in strs.iter_mut().zip(values) {
            target.assign(source);
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
    };
}

primitive_sequence!(rosidl_runtime_c__float32, f32);
primitive_sequence!(rosidl_runtime_c__float64, f64);

#[cfg(all(target_os = "macos", target_arch = "aarch64"))]
primitive_sequence!(rosidl_runtime_c__long_double, f64);

#[cfg(not(all(target_os = "macos", target_arch = "aarch64")))]
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
