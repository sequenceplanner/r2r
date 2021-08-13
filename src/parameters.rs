use super::*;

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

pub fn parameter_value_from_rcl(v: &rcl_variant_t) -> ParameterValue {
    if v.bool_value != std::ptr::null_mut() {
        ParameterValue::Bool(unsafe { *v.bool_value })
    } else if v.integer_value != std::ptr::null_mut() {
        ParameterValue::Integer(unsafe { *v.integer_value })
    } else if v.double_value != std::ptr::null_mut() {
        ParameterValue::Double(unsafe { *v.double_value })
    } else if v.string_value != std::ptr::null_mut() {
        let s = unsafe { CStr::from_ptr(v.string_value) };
        let string = s.to_str().unwrap_or("").to_owned();
        ParameterValue::String(string)
    } else if v.byte_array_value != std::ptr::null_mut() {
        let vals = unsafe {
            std::slice::from_raw_parts((*v.byte_array_value).values, (*v.byte_array_value).size)
        };
        ParameterValue::ByteArray(vals.iter().cloned().collect())
    } else if v.bool_array_value != std::ptr::null_mut() {
        let vals = unsafe {
            std::slice::from_raw_parts((*v.bool_array_value).values, (*v.bool_array_value).size)
        };
        ParameterValue::BoolArray(vals.iter().cloned().collect())
    } else if v.integer_array_value != std::ptr::null_mut() {
        let vals = unsafe {
            std::slice::from_raw_parts(
                (*v.integer_array_value).values,
                (*v.integer_array_value).size,
            )
        };
        ParameterValue::IntegerArray(vals.iter().cloned().collect())
    } else if v.double_array_value != std::ptr::null_mut() {
        let vals = unsafe {
            std::slice::from_raw_parts(
                (*v.double_array_value).values,
                (*v.double_array_value).size,
            )
        };
        ParameterValue::DoubleArray(vals.iter().cloned().collect())
    } else if v.string_array_value != std::ptr::null_mut() {
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
