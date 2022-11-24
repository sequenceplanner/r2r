use std::ffi::CStr;

use crate::msg_types::generated_msgs::rcl_interfaces;
use r2r_rcl::*;

/// ROS parameter value.
#[derive(Debug, PartialEq, Clone)]
pub enum ParameterValue {
    NotSet,
    Bool(bool),
    Integer(i64),
    Double(f64),
    String(String),
    BoolArray(Vec<bool>),
    ByteArray(Vec<u8>),
    IntegerArray(Vec<i64>),
    DoubleArray(Vec<f64>),
    StringArray(Vec<String>),
}

impl ParameterValue {
    pub(crate) fn from_rcl(v: &rcl_variant_t) -> Self {
        if !v.bool_value.is_null() {
            ParameterValue::Bool(unsafe { *v.bool_value })
        } else if !v.integer_value.is_null() {
            ParameterValue::Integer(unsafe { *v.integer_value })
        } else if !v.double_value.is_null() {
            ParameterValue::Double(unsafe { *v.double_value })
        } else if !v.string_value.is_null() {
            let s = unsafe { CStr::from_ptr(v.string_value) };
            let string = s.to_str().unwrap_or("").to_owned();
            ParameterValue::String(string)
        } else if !v.byte_array_value.is_null() {
            let vals = unsafe {
                std::slice::from_raw_parts((*v.byte_array_value).values, (*v.byte_array_value).size)
            };
            ParameterValue::ByteArray(vals.to_vec())
        } else if !v.bool_array_value.is_null() {
            let vals = unsafe {
                std::slice::from_raw_parts((*v.bool_array_value).values, (*v.bool_array_value).size)
            };
            ParameterValue::BoolArray(vals.to_vec())
        } else if !v.integer_array_value.is_null() {
            let vals = unsafe {
                std::slice::from_raw_parts(
                    (*v.integer_array_value).values,
                    (*v.integer_array_value).size,
                )
            };
            ParameterValue::IntegerArray(vals.to_vec())
        } else if !v.double_array_value.is_null() {
            let vals = unsafe {
                std::slice::from_raw_parts(
                    (*v.double_array_value).values,
                    (*v.double_array_value).size,
                )
            };
            ParameterValue::DoubleArray(vals.to_vec())
        } else if !v.string_array_value.is_null() {
            let vals = unsafe {
                std::slice::from_raw_parts(
                    (*v.string_array_value).data,
                    (*v.string_array_value).size,
                )
            };
            let s = vals
                .iter()
                .map(|cs| {
                    let s = unsafe { CStr::from_ptr(*cs) };
                    s.to_str().unwrap_or("").to_owned()
                })
                .collect();
            ParameterValue::StringArray(s)
        } else {
            ParameterValue::NotSet
        }
    }

    pub(crate) fn from_parameter_value_msg(msg: rcl_interfaces::msg::ParameterValue) -> Self {
        // todo: use constants from ParameterType message
        match msg.type_ {
            0 => ParameterValue::NotSet,
            1 => ParameterValue::Bool(msg.bool_value),
            2 => ParameterValue::Integer(msg.integer_value),
            3 => ParameterValue::Double(msg.double_value),
            4 => ParameterValue::String(msg.string_value),
            5 => ParameterValue::ByteArray(msg.byte_array_value),
            6 => ParameterValue::BoolArray(msg.bool_array_value),
            7 => ParameterValue::IntegerArray(msg.integer_array_value),
            8 => ParameterValue::DoubleArray(msg.double_array_value),
            9 => ParameterValue::StringArray(msg.string_array_value),
            _ => {
                println!("warning: malformed parametervalue message");
                ParameterValue::NotSet
            }
        }
    }

    pub(crate) fn into_parameter_value_msg(self) -> rcl_interfaces::msg::ParameterValue {
        let mut ret = rcl_interfaces::msg::ParameterValue::default();

        match self {
            ParameterValue::NotSet => {
                ret.type_ = 0; // uint8 PARAMETER_NOT_SET=0
            }
            ParameterValue::Bool(b) => {
                ret.type_ = 1; // uint8 PARAMETER_BOOL=1
                ret.bool_value = b;
            }
            ParameterValue::Integer(i) => {
                ret.type_ = 2; // uint8 PARAMETER_INTEGER=2
                ret.integer_value = i;
            }
            ParameterValue::Double(d) => {
                ret.type_ = 3; // uint8 PARAMETER_DOUBLE=3
                ret.double_value = d;
            }
            ParameterValue::String(s) => {
                ret.type_ = 4; // uint8 PARAMETER_STRING=4
                ret.string_value = s;
            }
            ParameterValue::ByteArray(ba) => {
                ret.type_ = 5; // uint8 PARAMETER_BYTE_ARRAY=5
                ret.byte_array_value = ba;
            }
            ParameterValue::BoolArray(ba) => {
                ret.type_ = 6; // uint8 PARAMETER_BOOL_ARRAY=6
                ret.bool_array_value = ba;
            }
            ParameterValue::IntegerArray(ia) => {
                ret.type_ = 7; // uint8 PARAMETER_INTEGER_ARRAY=7
                ret.integer_array_value = ia;
            }
            ParameterValue::DoubleArray(da) => {
                ret.type_ = 8; // uint8 PARAMETER_DOUBLE_ARRAY=8
                ret.double_array_value = da;
            }
            ParameterValue::StringArray(sa) => {
                ret.type_ = 9; // int PARAMETER_STRING_ARRAY=9
                ret.string_array_value = sa;
            }
        }
        ret
    }
}
