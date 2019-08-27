#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]
#![allow(dead_code)]
include!(concat!(env!("OUT_DIR"), "/msg_bindings.rs"));
include!(concat!(env!("OUT_DIR"), "/introspection_functions.rs"));

#[macro_use]
extern crate lazy_static;

use std::collections::HashMap;
use rcl::*;

use std::ffi::CStr;

fn field_type(t: u8) -> String {
    // lovely...
    // move to common
    if t == (rosidl_typesupport_introspection_c__ROS_TYPE_STRING as u8) { "std::string::String".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN as u8) { "bool".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_CHAR as u8) { "i8".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR as u8) { "u16".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_OCTET as u8) { "u8".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_UINT8 as u8) { "u8".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_INT8 as u8) { "i8".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_UINT16 as u8) { "u16".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_INT16 as u8) { "i16".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_UINT32 as u8) { "u32".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_INT32 as u8) { "i32".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_UINT64 as u8) { "u64".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_INT64 as u8) { "i64".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT as u8) { "f32".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE as u8) { "f64".to_owned() }
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE as u8) { "u128".to_owned() } // f128 does not exist in rust
    else if t == (rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE as u8) { "message".to_owned() }

    else { panic!("ros native type not implemented: {}", t); }
}

unsafe fn introspection<'a>(ptr: *const rosidl_message_type_support_t) -> (String, String, String, String, &'a [rosidl_typesupport_introspection_c__MessageMember]) {
    let members = (*ptr).data as *const rosidl_typesupport_introspection_c__MessageMembers;
    let namespace = CStr::from_ptr((*members).message_namespace_).to_str().unwrap();
    let name = CStr::from_ptr((*members).message_name_).to_str().unwrap();
    let nn: Vec<&str> = namespace.split("__").into_iter().take(2).collect();
    let (module, prefix) = ( nn[0], nn[1] );
    let c_struct = format!("{module}__{prefix}__{msgname}", module = module, prefix=prefix, msgname = name);
    let memberslice = std::slice::from_raw_parts((*members).members_, (*members).member_count_ as usize);
    (module.to_owned(), prefix.to_owned(), name.to_owned(), c_struct, memberslice)
}

fn field_name(field_name: &str) -> String {
    // check for reserved words
    if field_name == "type" { "type_".into() } else { field_name.to_owned() }
}

// TODO: this is a terrible hack :)
pub fn generate_rust_msg(module_: &str, prefix_: &str, name_: &str) -> String {
    let key = format!("{}__{}__{}", module_, prefix_, name_);
    let ptr = INTROSPECTION_FNS.get(key.as_str()).expect(&format!("code generation error: {}", name_));
    let ptr = *ptr as *const i32 as *const rosidl_message_type_support_t;
    unsafe {
        let (module, prefix, name, c_struct, members) = introspection(ptr);
        assert_eq!(module, module_);
        assert_eq!(prefix, prefix_);
        assert_eq!(name, name_);

        let mut fields = String::new();

        for member in members {
            let field_name = field_name(CStr::from_ptr(member.name_).to_str().unwrap());
            let rust_field_type = field_type(member.type_id_);
            let rust_field_type = if rust_field_type == "message" {
                let (module, prefix, name, _, _) = introspection(member.members_);
                format!("{module}::{prefix}::{msgname}", module = module, prefix=prefix, msgname = name)
            } else { rust_field_type };
            let s = if member.is_array_ {
                // if member.array_size_ > 0 {
                    // fixed size array
                    // format!("pub {}: [{};{}usize],\n",field_name, rust_field_type, array_size)
                    // actually lets use a vector anyway because its more convenient with traits. assert on the fixed size instead!
                //} else {
                    // vector type
                format!("pub {}: Vec<{}>,\n",field_name, rust_field_type)
                //}
            } else {
                format!("pub {}: {},\n",field_name, rust_field_type)
            };
            fields.push_str(&s);
        }

        let mut from_native = String::new();
        from_native.push_str(&format!("fn from_native(msg: &Self::CStruct) -> {} {{\n", name));
        from_native.push_str(&format!("  {} {{\n", name));

        for member in members {
            let field_name = field_name(CStr::from_ptr(member.name_).to_str().unwrap());
            let rust_field_type = field_type(member.type_id_);

            if member.is_array_ {
                if rust_field_type == "message" {
                    let (module, prefix, name, _, _) = introspection(member.members_);
                    from_native.push_str(&format!("{field_name} : {{\n", field_name = field_name));
                    from_native.push_str(&format!("let mut temp = Vec::with_capacity(msg.{field_name}.size);\n",field_name = field_name));
                    from_native.push_str(&format!("let slice = unsafe {{ std::slice::from_raw_parts(msg.{field_name}.data, msg.{field_name}.size)}};\n",field_name = field_name));
                    from_native.push_str(&format!("for s in slice {{ temp.push({module}::{prefix}::{msgname}::from_native(s)); }}\n", module = module, prefix=prefix, msgname = name));
                    from_native.push_str("temp },\n");
                } else {
                    if member.array_size_ > 0 {
                        // fixed size array, copy elements (happens to be the same now that we are using vectors...)
                        from_native.push_str(&format!("{field_name}: msg.{field_name}.to_vec(),\n", field_name = field_name));
                    } else {
                        from_native.push_str(&format!("{field_name}: msg.{field_name}.to_vec(),\n", field_name = field_name));
                    }
                }
            } else if rust_field_type == "std::string::String" {
                from_native.push_str(&format!("{field_name}: msg.{field_name}.to_str().to_owned(),\n", field_name = field_name));
            } else if rust_field_type == "message" {
                let (module, prefix, name, _, _) = introspection(member.members_);
                from_native.push_str(&format!("{field_name}: {module}::{prefix}::{msgname}::from_native(&msg.{field_name}),\n", field_name = field_name, module = module, prefix=prefix, msgname = name));
            } else {
                from_native.push_str(&format!("{field_name}: msg.{field_name},\n", field_name = field_name));
            }
        }
        from_native.push_str("      }\n    }\n");

        let mut copy_to_native = String::new();
        copy_to_native.push_str("fn copy_to_native(&self, msg: &mut Self::CStruct) {");

        for member in members {
            let field_name = field_name(CStr::from_ptr((*member).name_).to_str().unwrap());
            let rust_field_type = field_type(member.type_id_);

            if member.is_array_ {
                if rust_field_type == "message" {
                    let (_, _, _, c_struct, _) = introspection(member.members_);
                    copy_to_native.push_str(&format!("unsafe {{ {c_struct}__Sequence__fini(&mut msg.{field_name}) }};\n", c_struct = c_struct, field_name = field_name));
                    copy_to_native.push_str(&format!("unsafe {{ {c_struct}__Sequence__init(&mut msg.{field_name}, self.{field_name}.len()) }};\n", c_struct = c_struct, field_name = field_name));
                    copy_to_native.push_str(&format!("let slice = unsafe {{ std::slice::from_raw_parts_mut(msg.{field_name}.data, msg.{field_name}.size)}};\n",field_name = field_name));
                    copy_to_native.push_str(&format!("for (t,s) in slice.iter_mut().zip(&self.{field_name}) {{ s.copy_to_native(t);}}\n", field_name=field_name));
                } else {
                    if member.array_size_ > 0 && !member.is_upper_bound_ {
                        // fixed size array, just copy but first check the size!
                        copy_to_native.push_str(&format!("assert_eq!(self.{field_name}.len(), {array_size}, \"Field {{}} is fixed size of {{}}!\", \"{field_name}\", {array_size});\n", field_name = field_name, array_size = member.array_size_));
                        copy_to_native.push_str(&format!("msg.{field_name}.copy_from_slice(&self.{field_name}[..{array_size}]);\n", field_name = field_name, array_size = member.array_size_));
                    } else {
                        if member.is_upper_bound_ {
                            // extra assertion
                            copy_to_native.push_str(&format!("assert!(self.{field_name}.len() <= {array_size}, \"Field {{}} is upper bounded by {{}}!\", \"{field_name}\", {array_size});\n", field_name = field_name, array_size = member.array_size_));
                        }
                        copy_to_native.push_str(&format!("msg.{field_name}.update(&self.{field_name});\n", field_name = field_name));
                    }
                }
            } else if rust_field_type == "std::string::String" {
                copy_to_native.push_str(&format!("msg.{field_name}.assign(&self.{field_name});\n", field_name = field_name));
            } else if rust_field_type == "message" {
                copy_to_native.push_str(&format!("self.{field_name}.copy_to_native(&mut msg.{field_name});\n", field_name = field_name));
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


// this is even worse, it was added as an afterthought when I wanted to implement rostopic echo
pub fn generate_untyped_helpers(msgs: &Vec<common::RosMsg>) -> String {
    let mut ts_helper = format!("fn untyped_ts_helper(typename: &str) -> Result<&'static rosidl_message_type_support_t, ()> {{");
    for msg in msgs {
        ts_helper.push_str(&generate_untyped_ts_helper(&msg.module, &msg.prefix, &msg.name));
    }
    ts_helper.push_str(&format!("return Err(())\n}}"));

    let mut ds_helper = format!("fn untyped_deserialize_helper(typename: &str) -> Result<fn(native: *const std::os::raw::c_void) -> serde_json::Value, ()> {{");
    for msg in msgs {
        ds_helper.push_str(&generate_untyped_deserialize_helper(&msg.module, &msg.prefix, &msg.name));
    }
    ds_helper.push_str(&format!("return Err(())\n}}"));

    let mut se_helper = format!("
fn untyped_serialize_helper(typename: &str) -> Result<fn(json: serde_json::Value) -> Result<*mut std::os::raw::c_void, ()>, ()> {{");
    for msg in msgs {
        se_helper.push_str(&generate_untyped_serialize_helper(&msg.module, &msg.prefix, &msg.name));
    }
    se_helper.push_str(&format!("return Err(())\n}}"));

    let mut dealloc_helper = format!("fn untyped_dealloc_helper(typename: &str) -> Result<fn(*mut std::os::raw::c_void), ()> {{");
    for msg in msgs {
        dealloc_helper.push_str(&generate_untyped_dealloc_helper(&msg.module, &msg.prefix, &msg.name));
    }
    dealloc_helper.push_str(&format!("return Err(())\n}}"));

    format!("{} \n\n {} \n\n {} \n\n {} \n\n", ts_helper, ds_helper, se_helper, dealloc_helper)
}


pub fn generate_untyped_ts_helper(module_: &str, prefix_: &str, name_: &str) -> String {
    let typename = format!("{}/{}/{}", module_, prefix_, name_);
    let rustname = format!("{}::{}::{}", module_, prefix_, name_);

    format!("
    if typename == \"{typename}\" {{
        return Ok({rustname}::get_ts());
    }}
", typename = typename, rustname = rustname)
}

pub fn generate_untyped_deserialize_helper(module_: &str, prefix_: &str, name_: &str) -> String {
    let typename = format!("{}/{}/{}", module_, prefix_, name_);
    let rustname = format!("{}::{}::{}", module_, prefix_, name_);

    format!("
    if typename == \"{typename}\" {{
        let x = | native: *const std::os::raw::c_void | {{
            let ptr = native as *const <{rustname} as WrappedTypesupport>::CStruct;
            let msg = unsafe {{ {rustname}::from_native(&*ptr) }};
            serde_json::to_value(&msg).unwrap() // should never crash, we serialize from a known struct
        }};
        return Ok(x);
    }}", typename = typename, rustname = rustname)
}

pub fn generate_untyped_serialize_helper(module_: &str, prefix_: &str, name_: &str) -> String {
    let typename = format!("{}/{}/{}", module_, prefix_, name_);
    let rustname = format!("{}::{}::{}", module_, prefix_, name_);

    format!("
    if typename == \"{typename}\" {{
        let x = | json: serde_json::Value | {{
            let msg: Result<{rustname}, _> = serde_json::from_value(json);
            match msg {{
                Ok(msg) => {{
                    let native = {rustname}::create_msg();
                    unsafe {{ msg.copy_to_native(&mut *native); }}
                    Ok(native as *mut std::os::raw::c_void)
                }},
                Err(_) => Err(())
            }}
        }};
        return Ok(x);
    }}", typename = typename, rustname = rustname)
}

pub fn generate_untyped_dealloc_helper(module_: &str, prefix_: &str, name_: &str) -> String {
    let typename = format!("{}/{}/{}", module_, prefix_, name_);
    let rustname = format!("{}::{}::{}", module_, prefix_, name_);

    format!("
    if typename == \"{typename}\" {{
        let y = | native: *mut std::os::raw::c_void | {{
            let native_msg = native as *mut <{rustname} as WrappedTypesupport>::CStruct;
            {rustname}::destroy_msg(native_msg);
        }};
        return Ok(y);
    }}", typename = typename, rustname = rustname)
}
