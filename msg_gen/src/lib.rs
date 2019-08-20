#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]
#![allow(dead_code)]
include!("./msg_bindings.rs");
include!("./introspection_functions.rs");

#[macro_use]
extern crate lazy_static;

use std::collections::HashMap;
use rcl::*;

use std::ffi::CStr;

fn field_type(t: u8) -> String {

    // rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT = 1,
    // rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE = 2,
    // rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE = 3,
    // rosidl_typesupport_introspection_c__ROS_TYPE_CHAR = 4,
    // rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR = 5,
    // rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN = 6,
    // rosidl_typesupport_introspection_c__ROS_TYPE_OCTET = 7,
    // rosidl_typesupport_introspection_c__ROS_TYPE_UINT8 = 8,
    // rosidl_typesupport_introspection_c__ROS_TYPE_INT8 = 9,
    // rosidl_typesupport_introspection_c__ROS_TYPE_UINT16 = 10,
    // rosidl_typesupport_introspection_c__ROS_TYPE_INT16 = 11,
    // rosidl_typesupport_introspection_c__ROS_TYPE_UINT32 = 12,
    // rosidl_typesupport_introspection_c__ROS_TYPE_INT32 = 13,
    // rosidl_typesupport_introspection_c__ROS_TYPE_UINT64 = 14,
    // rosidl_typesupport_introspection_c__ROS_TYPE_INT64 = 15,
    // rosidl_typesupport_introspection_c__ROS_TYPE_STRING = 16,
    // rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING = 17,
    // rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE = 18,

    // todo: add these as needed...
    if t == (rosidl_typesupport_introspection_c__ROS_TYPE_STRING as u8) { "std::string::String".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN as u8) { "bool".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_UINT32 as u8) { "u32".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_INT32 as u8) { "i32".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT as u8) { "f32".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE as u8) { "f64".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE as u8) { "message".to_owned() }

    else { panic!("ros native type not implemented: {}", t); }
}

// TODO: this is a terrible hack :)
pub fn generate_rust_msg(module: &str, prefix: &str, name: &str) -> String {
    let key = format!("{}__{}__{}", module, prefix, name);
    let ptr = INTROSPECTION_FNS.get(key.as_str()).expect(&format!("code generation error: {}", name));
    let ptr = *ptr as *const i32 as *const rosidl_message_type_support_t;
    unsafe {
        let members = (*ptr).data as *const rosidl_typesupport_introspection_c__MessageMembers;
        let namespace = CStr::from_ptr((*members).message_namespace_).to_str().unwrap();
        let name = CStr::from_ptr((*members).message_name_).to_str().unwrap();
        let nn: Vec<&str> = namespace.split("__").into_iter().take(2).collect();
        let (module, prefix) = ( nn[0], nn[1] );

        let c_struct = format!("{module}__{prefix}__{msgname}", module = module, prefix=prefix, msgname = name);

        let memberslice = std::slice::from_raw_parts((*members).members_, (*members).member_count_ as usize);

        let mut fields = String::new();

        for member in memberslice {
            let field_name = CStr::from_ptr((*member).name_).to_str().unwrap();
            let type_id = (*member).type_id_;
            let is_array = (*member).is_array_; // TODO: use
            let rust_field_type = field_type(type_id);
            let rust_field_type = if rust_field_type == "message" {
                // perform a hack!
                let ts = (*member).members_;
                let members = (*ts).data as *const rosidl_typesupport_introspection_c__MessageMembers;
                let namespace = CStr::from_ptr((*members).message_namespace_).to_str().unwrap();
                let name = CStr::from_ptr((*members).message_name_).to_str().unwrap();
                let nn: Vec<&str> = namespace.split("__").into_iter().take(2).collect();
                let (module, prefix) = ( nn[0], nn[1] );
                format!("{module}::{prefix}::{msgname}", module = module, prefix=prefix, msgname = name)
            } else { rust_field_type };
            let s = if is_array {
                format!("pub {}: Vec<{}>,\n",field_name, rust_field_type)
            } else {
                format!("pub {}: {},\n",field_name, rust_field_type)
            };
            fields.push_str(&s);
        }


        let mut from_native = String::new();
        from_native.push_str(&format!("fn from_native(msg: &Self::CStruct) -> {} {{\n", name));
        from_native.push_str(&format!("  {} {{\n", name));

        for member in memberslice {
            let field_name = CStr::from_ptr((*member).name_).to_str().unwrap();
            let type_id = (*member).type_id_;
            let is_array = (*member).is_array_; // TODO: use

            let rust_field_type = field_type(type_id);

            if is_array {
                from_native.push_str(&format!("{field_name}: msg.{field_name}.to_vec(),\n", field_name = field_name));
            } else if rust_field_type == "std::string::String" {
                from_native.push_str(&format!("{field_name}: msg.{field_name}.to_str().to_owned(),\n", field_name = field_name));
            } else if rust_field_type == "message" {
                // perform a hack!
                let ts = (*member).members_;
                let members = (*ts).data as *const rosidl_typesupport_introspection_c__MessageMembers;
                let namespace = CStr::from_ptr((*members).message_namespace_).to_str().unwrap();
                let name = CStr::from_ptr((*members).message_name_).to_str().unwrap();
                let nn: Vec<&str> = namespace.split("__").into_iter().take(2).collect();
                let (module, prefix) = ( nn[0], nn[1] );
                from_native.push_str(&format!("{field_name}: {module}::{prefix}::{msgname}::from_native(&msg.{field_name}),", field_name = field_name, module = module, prefix=prefix, msgname = name));                
            } else {
                from_native.push_str(&format!("{field_name}: msg.{field_name},\n", field_name = field_name));
            }
        }
        from_native.push_str("      }\n    }\n");

        let mut copy_to_native = String::new();
        copy_to_native.push_str("fn copy_to_native(&self, msg: &mut Self::CStruct) {");

        for member in memberslice {
            let field_name = CStr::from_ptr((*member).name_).to_str().unwrap();
            let type_id = (*member).type_id_;
            let is_array = (*member).is_array_; // TODO: use

            let rust_field_type = field_type(type_id);

            // handle other special cases...
            if is_array {
                copy_to_native.push_str(&format!("msg.{field_name}.update(&self.{field_name});\n", field_name = field_name));
            } else if rust_field_type == "std::string::String" {
                copy_to_native.push_str(&format!("msg.{field_name}.assign(&self.{field_name});\n", field_name = field_name));
            } else if rust_field_type == "message" {
                copy_to_native.push_str(&format!("self.{field_name}.copy_to_native(&mut msg.{field_name});", field_name = field_name));
            } else {
                copy_to_native.push_str(&format!("msg.{field_name} = self.{field_name};\n", field_name = field_name));
            }
        }
        copy_to_native.push_str("}\n");

        let typesupport = format!("impl WrappedTypesupport for {msgname} {{ \n
            type CStruct = {c_struct}; \n\n
            fn get_ts() -> &'static rosidl_message_type_support_t {{ \n
                unsafe {{ &*rosidl_typesupport_c__get_message_type_support_handle__{c_struct}() }}
            }}\n
            fn create_msg() -> *mut {c_struct} {{\n
                unsafe {{ {c_struct}__create() }}\n
            }}\n
            fn destroy_msg(msg: *mut {c_struct}) -> () {{\n
                unsafe {{ {c_struct}__destroy(msg) }};\n
            }}\n
            {from_native}\n\n
            {copy_to_native}\n\n
        }}\n", msgname = name, c_struct = &c_struct, from_native=from_native, copy_to_native=copy_to_native);

        let module_str = format!("
                          #[derive(Clone,Debug,Default,PartialEq,Serialize,Deserialize)]
                          pub struct {msgname} {{\n
                              {fields}
                          }}\n
                          {typesupport}\n\n
                    ", msgname = name, fields = fields, typesupport = typesupport);

        module_str
    }

}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fn_ptrs() -> () {

        assert_eq!(generate_rust_msg("std_msgs", "msg", "string"), "hej");
    }
}

