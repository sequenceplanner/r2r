use crate::{Error, Result};
use std::ffi::CStr;

use crate::msg_types::generated_msgs::rcl_interfaces;
use indexmap::IndexMap;
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
                if (*v.byte_array_value).values == std::ptr::null_mut() {
                    &[]
                } else {
                    std::slice::from_raw_parts(
                        (*v.byte_array_value).values,
                        (*v.byte_array_value).size,
                    )
                }
            };
            ParameterValue::ByteArray(vals.to_vec())
        } else if !v.bool_array_value.is_null() {
            let vals = unsafe {
                if (*v.bool_array_value).values == std::ptr::null_mut() {
                    &[]
                } else {
                    std::slice::from_raw_parts(
                        (*v.bool_array_value).values,
                        (*v.bool_array_value).size,
                    )
                }
            };
            ParameterValue::BoolArray(vals.to_vec())
        } else if !v.integer_array_value.is_null() {
            let vals = unsafe {
                if (*v.integer_array_value).values == std::ptr::null_mut() {
                    &[]
                } else {
                    std::slice::from_raw_parts(
                        (*v.integer_array_value).values,
                        (*v.integer_array_value).size,
                    )
                }
            };
            ParameterValue::IntegerArray(vals.to_vec())
        } else if !v.double_array_value.is_null() {
            let vals = unsafe {
                if (*v.double_array_value).values == std::ptr::null_mut() {
                    &[]
                } else {
                    std::slice::from_raw_parts(
                        (*v.double_array_value).values,
                        (*v.double_array_value).size,
                    )
                }
            };
            ParameterValue::DoubleArray(vals.to_vec())
        } else if !v.string_array_value.is_null() {
            let vals = unsafe {
                if (*v.string_array_value).data == std::ptr::null_mut() {
                    &[]
                } else {
                    std::slice::from_raw_parts(
                        (*v.string_array_value).data,
                        (*v.string_array_value).size,
                    )
                }
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
                log::debug!("warning: malformed parametervalue message");
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

    pub(crate) fn into_parameter_type(&self) -> u8 {
        match self {
            ParameterValue::NotSet => 0,          // uint8 PARAMETER_NOT_SET=0
            ParameterValue::Bool(_) => 1,         // uint8 PARAMETER_BOOL=1
            ParameterValue::Integer(_) => 2,      // uint8 PARAMETER_INTEGER=2
            ParameterValue::Double(_) => 3,       // uint8 PARAMETER_DOUBLE=3
            ParameterValue::String(_) => 4,       // uint8 PARAMETER_STRING=4
            ParameterValue::ByteArray(_) => 5,    // uint8 PARAMETER_BYTE_ARRAY=5
            ParameterValue::BoolArray(_) => 6,    // uint8 PARAMETER_BOOL_ARRAY=6
            ParameterValue::IntegerArray(_) => 7, // uint8 PARAMETER_INTEGER_ARRAY=7
            ParameterValue::DoubleArray(_) => 8,  // uint8 PARAMETER_DOUBLE_ARRAY=8
            ParameterValue::StringArray(_) => 9,  // int PARAMETER_STRING_ARRAY=9
        }
    }
}

/// ROS parameter.
pub struct Parameter {
    pub value: ParameterValue,
    pub description: &'static str,
    // TODO: Add other fields like min, max, step. Use field
    // attributes for defining them.
}

impl Parameter {
    pub fn new(value: ParameterValue) -> Self {
        Self {
            value,
            description: "",
        }
    }
    pub fn empty() -> Self {
        Self {
            value: ParameterValue::NotSet,
            description: "",
        }
    }
}

/// Trait for use it with
/// [`Node::make_derived_parameter_handler()`](crate::Node::make_derived_parameter_handler()).
///
/// The trait is usually derived with `r2r_macros::RosParams`. See
/// `parameters_derive.rs` example.
pub trait RosParams {
    fn register_parameters(
        &mut self, prefix: &str, param: Option<Parameter>, params: &mut IndexMap<String, Parameter>,
    ) -> Result<()>;
    fn get_parameter(&mut self, param_name: &str) -> Result<ParameterValue>;
    fn set_parameter(&mut self, param_name: &str, param_val: &ParameterValue) -> Result<()>;
}

// Implementation of RosParams for primitive types, i.e. leaf parameters
macro_rules! impl_ros_params {
    ($type:path, $param_value_type:path, $to_param_conv:path, $from_param_conv:path) => {
        impl RosParams for $type {
            fn register_parameters(
                &mut self, prefix: &str, param: Option<Parameter>,
                params: &mut IndexMap<String, Parameter>,
            ) -> Result<()> {
                if let Some(cli_param) = params.get(prefix) {
                    // Apply parameter value if set from command line or launch file
                    self.set_parameter("", &cli_param.value)
                        .map_err(|e| e.update_param_name(prefix))?;
                    // Remove the parameter (will be re-inserted below with deterministic order)
                    params.shift_remove(prefix);
                }
                // Insert the parameter with filled-in description etc.
                let mut param = param.unwrap();
                param.value = $param_value_type($to_param_conv(self)?);
                params.insert(prefix.to_owned(), param);
                Ok(())
            }

            fn get_parameter(&mut self, param_name: &str) -> Result<ParameterValue> {
                match param_name {
                    "" => Ok($param_value_type($to_param_conv(self)?)),
                    _ => Err(Error::InvalidParameterName {
                        name: param_name.to_owned(),
                    }),
                }
            }

            fn set_parameter(
                &mut self, param_name: &str, param_val: &ParameterValue,
            ) -> Result<()> {
                if param_name != "" {
                    return Err(Error::InvalidParameterName {
                        name: param_name.to_owned(),
                    });
                }
                match param_val {
                    $param_value_type(val) => {
                        *self = $from_param_conv(val)?;
                        Ok(())
                    }
                    _ => Err(Error::InvalidParameterType {
                        name: "".to_string(), // will be completed by callers who know the name
                        ty: std::stringify!($param_value_type),
                    }),
                }
            }
        }
    };
}

impl_ros_params!(bool, ParameterValue::Bool, noop, noop);
impl_ros_params!(i8, ParameterValue::Integer, try_conv, try_conv);
impl_ros_params!(i16, ParameterValue::Integer, try_conv, try_conv);
impl_ros_params!(i32, ParameterValue::Integer, try_conv, try_conv);
impl_ros_params!(i64, ParameterValue::Integer, noop, noop);
impl_ros_params!(u8, ParameterValue::Integer, try_conv, try_conv);
impl_ros_params!(u16, ParameterValue::Integer, try_conv, try_conv);
impl_ros_params!(u32, ParameterValue::Integer, try_conv, try_conv);
impl_ros_params!(f64, ParameterValue::Double, noop, noop);
impl_ros_params!(f32, ParameterValue::Double, to_f64, to_f32);
impl_ros_params!(String, ParameterValue::String, to_string, to_string);
// TODO: Implement array parameters

// Helper conversion functions
fn noop<T: Copy>(x: &T) -> Result<T> {
    Ok(*x)
}

fn to_f32(x: &f64) -> Result<f32> {
    Ok(*x as f32)
}
fn to_f64(x: &f32) -> Result<f64> {
    Ok(*x as f64)
}

fn try_conv<T, U>(x: &T) -> Result<U>
where
    T: Copy,
    U: TryFrom<T>,
    <U as TryFrom<T>>::Error: std::error::Error,
{
    U::try_from(*x).map_err(|e| Error::ParameterValueConv {
        name: "".into(),
        msg: e.to_string(),
    })
}

fn to_string(x: &str) -> Result<String> {
    Ok(x.to_string())
}
