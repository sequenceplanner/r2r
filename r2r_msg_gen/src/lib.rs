#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]
#![allow(dead_code)]
#![allow(clippy::all)]
include!(concat!(env!("OUT_DIR"), "/msg_bindings.rs"));
include!(concat!(env!("OUT_DIR"), "/introspection_functions.rs"));

#[macro_use]
extern crate lazy_static;

use r2r_rcl::*;
use std::collections::HashMap;

use std::ffi::CStr;

// because the c enum has been named between galactic and the next release,
// we cannot know its name. therefor we use the constants as is and hope we notice
// when they change.
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
fn field_type(t: u8) -> String {
    // lovely...
    // move to common
    if t == 16 {
        "std::string::String".to_owned()
    } else if t == 17 {
        "std::string::String".to_owned()
    } else if t == 6 {
        "bool".to_owned()
    } else if t == 4 {
        "i8".to_owned()
    } else if t == 5 {
        "i16".to_owned()
    } else if t == 7 {
        "u8".to_owned()
    } else if t == 8 {
        "u8".to_owned()
    } else if t == 9 {
        "i8".to_owned()
    } else if t == 10 {
        "u16".to_owned()
    } else if t == 11 {
        "i16".to_owned()
    } else if t == 12 {
        "u32".to_owned()
    } else if t == 13 {
        "i32".to_owned()
    } else if t == 14 {
        "u64".to_owned()
    } else if t == 15 {
        "i64".to_owned()
    } else if t == 1 {
        "f32".to_owned()
    } else if t == 2 {
        "f64".to_owned()
    } else if t == 3 {
        // f128 does not exist in rust
        "u128".to_owned()
    } else if t == 18 {
        "message".to_owned()
    } else {
        panic!("ros native type not implemented: {}", t);
    }
}

unsafe fn introspection<'a>(
    ptr: *const rosidl_message_type_support_t,
) -> (
    String,
    String,
    String,
    String,
    &'a [rosidl_typesupport_introspection_c__MessageMember],
) {
    let members = (*ptr).data as *const rosidl_typesupport_introspection_c__MessageMembers;
    let namespace = CStr::from_ptr((*members).message_namespace_)
        .to_str()
        .unwrap();
    let name = CStr::from_ptr((*members).message_name_).to_str().unwrap();
    let nn: Vec<&str> = namespace.split("__").into_iter().take(2).collect();
    let (module, prefix) = (nn[0], nn[1]);
    let c_struct = format!(
        "{module}__{prefix}__{msgname}",
        module = module,
        prefix = prefix,
        msgname = name
    );
    let memberslice =
        std::slice::from_raw_parts((*members).members_, (*members).member_count_ as usize);
    (
        module.to_owned(),
        prefix.to_owned(),
        name.to_owned(),
        c_struct,
        memberslice,
    )
}

fn field_name(field_name: &str) -> String {
    // check for reserved words
    if field_name == "type" {
        "type_".into()
    } else {
        field_name.to_owned()
    }
}

pub fn generate_rust_service(module_: &str, prefix_: &str, name_: &str) -> String {
    format!(
        "
        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {{
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {{
                unsafe {{
                    &*rosidl_typesupport_c__get_service_type_support_handle__{}__{}__{}()
                }}
            }}
        }}

            ",
        module_, prefix_, name_
    )
}

pub fn generate_rust_action(module_: &str, prefix_: &str, name_: &str) -> String {
    format!(
        "
        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Action();
        impl WrappedActionTypeSupport for Action {{
            type Goal = Goal;
            type Result = Result;
            type Feedback = Feedback;

            // internal structs
            type FeedbackMessage = FeedbackMessage;
            type SendGoal = SendGoal::Service;
            type GetResult = GetResult::Service;

            fn get_ts() -> &'static rosidl_action_type_support_t {{
                unsafe {{
                    &*rosidl_typesupport_c__get_action_type_support_handle__{module}__{prefix}__{name}()
                }}
            }}

            fn make_goal_request_msg(goal_id: unique_identifier_msgs::msg::UUID, goal: Goal) -> SendGoal::Request {{
                SendGoal::Request {{
                     goal_id,
                     goal
                }}
            }}

            fn make_goal_response_msg(accepted: bool, stamp: builtin_interfaces::msg::Time) -> SendGoal::Response {{
                SendGoal::Response {{
                     accepted,
                     stamp
                }}
            }}

            fn make_feedback_msg(goal_id: unique_identifier_msgs::msg::UUID, feedback: Feedback) -> FeedbackMessage {{
                FeedbackMessage {{
                     goal_id,
                     feedback
                }}
            }}

            fn make_result_request_msg(goal_id: unique_identifier_msgs::msg::UUID) -> GetResult::Request {{
                GetResult::Request {{
                     goal_id,
                }}
            }}

            fn make_result_response_msg(status: i8, result: Result) -> GetResult::Response {{
                GetResult::Response {{
                     status,
                     result,
                }}
            }}

            fn destructure_goal_request_msg(msg: SendGoal::Request) -> (unique_identifier_msgs::msg::UUID, Goal) {{
                (msg.goal_id, msg.goal)
            }}

            fn destructure_goal_response_msg(msg: SendGoal::Response) -> (bool, builtin_interfaces::msg::Time) {{
                (msg.accepted, msg.stamp)
            }}

            fn destructure_feedback_msg(msg: FeedbackMessage) -> (unique_identifier_msgs::msg::UUID, Feedback) {{
                (msg.goal_id, msg.feedback)
            }}

            fn destructure_result_response_msg(msg: GetResult::Response) -> (i8, Result) {{
                (msg.status, msg.result)
            }}

            fn destructure_result_request_msg(msg: GetResult::Request) -> unique_identifier_msgs::msg::UUID {{
                msg.goal_id
            }}
        }}

            ",
        module = module_, prefix = prefix_, name = name_
    )
}

// TODO: this is a terrible hack :)
pub fn generate_rust_msg(module_: &str, prefix_: &str, name_: &str) -> String {
    let key = format!("{}__{}__{}", module_, prefix_, name_);
    let ptr = INTROSPECTION_FNS
        .get(key.as_str())
        .unwrap_or_else(|| panic!("code generation error: {}", key));
    let ptr = *ptr as *const i32 as *const rosidl_message_type_support_t;
    unsafe {
        let (module, prefix, mut name, c_struct, members) = introspection(ptr);
        assert_eq!(module, module_);
        assert_eq!(prefix, prefix_);
        assert_eq!(name, name_);

        if prefix == "srv" || prefix == "action" {
            // for srv, the message name is both the service name and _Request or _Respone
            // we only want to keep the last part.
            // same for actions with _Goal, _Result, _Feedback
            // TODO: refactor...
            // handle special case of ActionName_ServiceName_Response
            let nn = name.splitn(3, '_').collect::<Vec<&str>>();
            if let [_mod_name, _srv_name, msg_name] = &nn[..] {
                name = msg_name.to_string();
            } else if let [_mod_name, msg_name] = &nn[..] {
                name = msg_name.to_string();
            } else {
                panic!("malformed service name {}", name);
            }
        }

        let mut fields = String::new();

        let is_empty_msg = members.len() == 1
            && field_name(CStr::from_ptr(members[0].name_).to_str().unwrap())
                == "structure_needs_at_least_one_member";

        for member in members {
            let field_name = field_name(CStr::from_ptr(member.name_).to_str().unwrap());
            if field_name == "structure_needs_at_least_one_member" {
                // Yay we can have empty structs in rust
                continue;
            }
            let rust_field_type = field_type(member.type_id_);
            let rust_field_type = if rust_field_type == "message" {
                let (module, prefix, name, _, _) = introspection(member.members_);
                // hack here to rustify nested action type names
                if prefix == "action" {
                    if let Some((n1, n2)) = name.rsplit_once("_") {
                        format!(
                            "{module}::{prefix}::{srvname}::{msgname}",
                            module = module,
                            prefix = prefix,
                            srvname = n1,
                            msgname = n2
                        )
                    } else {
                        format!(
                            "{module}::{prefix}::{msgname}",
                            module = module,
                            prefix = prefix,
                            msgname = name
                        )
                    }
                } else {
                    format!(
                        "{module}::{prefix}::{msgname}",
                        module = module,
                        prefix = prefix,
                        msgname = name
                    )
                }
            } else {
                rust_field_type
            };
            let s = if member.is_array_ {
                // if member.array_size_ > 0 {
                // fixed size array
                // format!("pub {}: [{};{}usize],\n",field_name, rust_field_type, array_size)
                // actually lets use a vector anyway because its more convenient with traits. assert on the fixed size instead!
                //} else {
                // vector type
                format!("pub {}: Vec<{}>,\n", field_name, rust_field_type)
            //}
            } else {
                format!("pub {}: {},\n", field_name, rust_field_type)
            };
            fields.push_str(&s);
        }

        let mut from_native = String::new();
        if is_empty_msg {
            from_native.push_str(&format!(
                "fn from_native(_msg: &Self::CStruct) -> {} {{\n",
                name
            ));
        } else {
            from_native.push_str(&format!(
                "fn from_native(msg: &Self::CStruct) -> {} {{\n",
                name
            ));
        }
        from_native.push_str(&format!("  {} {{\n", name));

        for member in members {
            let field_name = field_name(CStr::from_ptr(member.name_).to_str().unwrap());
            if field_name == "structure_needs_at_least_one_member" {
                // Yay we can have empty structs in rust
                continue;
            }
            let rust_field_type = field_type(member.type_id_);

            if member.is_array_ && member.array_size_ > 0 && !member.is_upper_bound_ {
                // these are plain arrays
                let ss = format!("// is_upper_bound_: {}\n", member.is_upper_bound_);
                from_native.push_str(&ss);
                let ss = format!("// member.array_size_ : {}\n", member.array_size_);
                from_native.push_str(&ss);
                if rust_field_type == "message" {
                    let (module, prefix, name, _, _) = introspection(member.members_);
                    from_native.push_str(&format!("{field_name} : {{\n", field_name = field_name));
                    from_native.push_str(&format!(
                        "let mut temp = Vec::with_capacity(msg.{field_name}.len());\n",
                        field_name = field_name
                    ));
                    from_native.push_str(&format!("for s in &msg.{field_name} {{ temp.push({module}::{prefix}::{msgname}::from_native(s)); }}\n", field_name = field_name, module = module, prefix=prefix, msgname = name));
                    from_native.push_str("temp },\n");
                } else if rust_field_type == "std::string::String" {
                    from_native.push_str(&format!(
                        "{field_name}: msg.{field_name}.iter().map(|s|s.to_str().to_owned()).collect(),\n",
                        field_name = field_name
                    ));
                } else {
                    from_native.push_str(&format!(
                        "{field_name}: msg.{field_name}.to_vec(),\n",
                        field_name = field_name
                    ));
                }
            } else if member.is_array_ && (member.array_size_ == 0 || member.is_upper_bound_) {
                // these are __Sequence:s
                let ss = format!("// is_upper_bound_: {}\n", member.is_upper_bound_);
                from_native.push_str(&ss);
                let ss = format!("// member.array_size_ : {}\n", member.array_size_);
                from_native.push_str(&ss);
                if rust_field_type == "message" {
                    let (module, prefix, name, _, _) = introspection(member.members_);
                    from_native.push_str(&format!("{field_name} : {{\n", field_name = field_name));
                    from_native.push_str(&format!(
                        "let mut temp = Vec::with_capacity(msg.{field_name}.size);\n",
                        field_name = field_name
                    ));
                    from_native.push_str(&format!("let slice = unsafe {{ std::slice::from_raw_parts(msg.{field_name}.data, msg.{field_name}.size)}};\n",field_name = field_name));
                    from_native.push_str(&format!("for s in slice {{ temp.push({module}::{prefix}::{msgname}::from_native(s)); }}\n", module = module, prefix=prefix, msgname = name));
                    from_native.push_str("temp },\n");
                } else {
                    from_native.push_str(&format!(
                        "{field_name}: msg.{field_name}.to_vec(),\n",
                        field_name = field_name
                    ));
                }
            } else if rust_field_type == "std::string::String" {
                from_native.push_str(&format!(
                    "{field_name}: msg.{field_name}.to_str().to_owned(),\n",
                    field_name = field_name
                ));
            } else if rust_field_type == "message" {
                let (module, prefix, name, _, _) = introspection(member.members_);
                // same hack as above to rustify message type names
                if prefix == "action" {
                    if let Some((n1, n2)) = name.rsplit_once("_") {
                        from_native.push_str(&format!("{field_name}: {module}::{prefix}::{srvname}::{msgname}::from_native(&msg.{field_name}),\n", field_name = field_name, module = module, prefix=prefix, srvname = n1, msgname = n2));
                    } else {
                        panic!("ooops at from_native");
                    }
                } else {
                    from_native.push_str(&format!("{field_name}: {module}::{prefix}::{msgname}::from_native(&msg.{field_name}),\n", field_name = field_name, module = module, prefix=prefix, msgname = name));
                }
            } else {
                from_native.push_str(&format!(
                    "{field_name}: msg.{field_name},\n",
                    field_name = field_name
                ));
            }
        }
        from_native.push_str("      }\n    }\n");

        let mut copy_to_native = String::new();
        if is_empty_msg {
            copy_to_native.push_str("fn copy_to_native(&self, _msg: &mut Self::CStruct) {");
        } else {
            copy_to_native.push_str("fn copy_to_native(&self, msg: &mut Self::CStruct) {");
        }

        for member in members {
            let field_name = field_name(CStr::from_ptr((*member).name_).to_str().unwrap());
            if field_name == "structure_needs_at_least_one_member" {
                // Yay we can have empty structs in rust
                continue;
            }
            let rust_field_type = field_type(member.type_id_);

            if member.is_array_ && member.array_size_ > 0 && !member.is_upper_bound_ {
                // these are plain arrays
                // fixed size array, just copy but first check the size!
                copy_to_native.push_str(&format!("assert_eq!(self.{field_name}.len(), {array_size}, \"Field {{}} is fixed size of {{}}!\", \"{field_name}\", {array_size});\n", field_name = field_name, array_size = member.array_size_));
                if rust_field_type == "message" {
                    copy_to_native.push_str(&format!("for (t,s) in msg.{field_name}.iter_mut().zip(&self.{field_name}) {{ s.copy_to_native(t);}}\n", field_name=field_name));
                } else if rust_field_type == "std::string::String" {
                    copy_to_native.push_str(&format!("for (t,s) in msg.{field_name}.iter_mut().zip(&self.{field_name}) {{ t.assign(&s);}}\n", field_name=field_name));
                } else {
                    copy_to_native.push_str(&format!(
                        "msg.{field_name}.copy_from_slice(&self.{field_name}[..{array_size}]);\n",
                        field_name = field_name,
                        array_size = member.array_size_
                    ));
                }
            } else if member.is_array_ && (member.array_size_ == 0 || member.is_upper_bound_) {
                // these are __Sequence:s
                if rust_field_type == "message" {
                    let (_, _, _, c_struct, _) = introspection(member.members_);
                    copy_to_native.push_str(&format!(
                        "unsafe {{ {c_struct}__Sequence__fini(&mut msg.{field_name}) }};\n",
                        c_struct = c_struct,
                        field_name = field_name
                    ));
                    copy_to_native.push_str(&format!("unsafe {{ {c_struct}__Sequence__init(&mut msg.{field_name}, self.{field_name}.len()) }};\n", c_struct = c_struct, field_name = field_name));
                    copy_to_native.push_str(&format!("let slice = unsafe {{ std::slice::from_raw_parts_mut(msg.{field_name}.data, msg.{field_name}.size)}};\n",field_name = field_name));
                    copy_to_native.push_str(&format!("for (t,s) in slice.iter_mut().zip(&self.{field_name}) {{ s.copy_to_native(t);}}\n", field_name=field_name));
                } else {
                    // extra assertion
                    if member.is_upper_bound_ {
                        copy_to_native.push_str(&format!("assert!(self.{field_name}.len() <= {array_size}, \"Field {{}} is upper bounded by {{}}!\", \"{field_name}\", {array_size});\n", field_name = field_name, array_size = member.array_size_));
                    }
                    copy_to_native.push_str(&format!(
                        "msg.{field_name}.update(&self.{field_name});\n",
                        field_name = field_name
                    ));
                }
            } else if rust_field_type == "std::string::String" {
                copy_to_native.push_str(&format!(
                    "msg.{field_name}.assign(&self.{field_name});\n",
                    field_name = field_name
                ));
            } else if rust_field_type == "message" {
                copy_to_native.push_str(&format!(
                    "self.{field_name}.copy_to_native(&mut msg.{field_name});\n",
                    field_name = field_name
                ));
            } else {
                copy_to_native.push_str(&format!(
                    "msg.{field_name} = self.{field_name};\n",
                    field_name = field_name
                ));
            }
        }
        copy_to_native.push_str("}\n");

        let typesupport = format!(
            "impl WrappedTypesupport for {msgname} {{ \n
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
        }}\n",
            msgname = name,
            c_struct = &c_struct,
            from_native = from_native,
            copy_to_native = copy_to_native
        );

        let impl_default = format!(
            "
                          impl Default for {msgname} {{
                              fn default() -> Self {{
                                  let msg_native = WrappedNativeMsg::<{msgname}>::new();
                                  {msgname}::from_native(&msg_native)
                              }}
                          }}
             ",
            msgname = name
        );

        let module_str = format!(
            "
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct {msgname} {{\n
                              {fields}
                          }}\n
                          {typesupport}\n
                          {default}\n\n
                    ",
            msgname = name,
            fields = fields,
            typesupport = typesupport,
            default = impl_default
        );

        module_str
    }
}

pub fn generate_untyped_helper(msgs: &Vec<r2r_common::RosMsg>) -> String {
    let open = String::from(
        "
impl WrappedNativeMsgUntyped {
    pub fn new_from(typename: &str) -> Result<Self> {
",
    );
    let close = String::from(
        "
        return Err(Error::InvalidMessageType{ msgtype: typename.into() })
    }
}
",
    );

    let mut lines = String::new();
    for msg in msgs {
        // for now don't generate untyped services or actions
        if msg.prefix == "srv" || msg.prefix == "action" {
            continue;
        }

        let typename = format!("{}/{}/{}", msg.module, msg.prefix, msg.name);
        let rustname = format!("{}::{}::{}", msg.module, msg.prefix, msg.name);

        lines.push_str(&format!(
            "
        if typename == \"{typename}\" {{
            return Ok(WrappedNativeMsgUntyped::new::<{rustname}>());
        }}
",
            typename = typename,
            rustname = rustname
        ));
    }

    format!("{}{}{}", open, lines, close)
}

pub fn generate_untyped_service_helper(msgs: &Vec<r2r_common::RosMsg>) -> String {
    let open = String::from(
        "
impl UntypedServiceSupport {
    pub fn new_from(typename: &str) -> Result<Self> {
",
    );
    let close = String::from(
        "
        return Err(Error::InvalidMessageType{ msgtype: typename.into() })
    }
}
",
    );

    let mut lines = String::new();
    for msg in msgs {
        if msg.prefix != "srv" {
            continue;
        }

        let typename = format!("{}/{}/{}", msg.module, msg.prefix, msg.name);
        let rustname = format!("{}::{}::{}::Service", msg.module, msg.prefix, msg.name);

        lines.push_str(&format!(
            "
        if typename == \"{typename}\" {{
            return Ok(UntypedServiceSupport::new::<{rustname}>());
        }}
",
            typename = typename,
            rustname = rustname
        ));
    }

    format!("{}{}{}", open, lines, close)
}

pub fn generate_untyped_action_helper(msgs: &Vec<r2r_common::RosMsg>) -> String {
    let open = String::from(
        "
impl UntypedActionSupport {
    pub fn new_from(typename: &str) -> Result<Self> {
",
    );
    let close = String::from(
        "
        return Err(Error::InvalidMessageType{ msgtype: typename.into() })
    }
}
",
    );

    let mut lines = String::new();
    for msg in msgs {
        if msg.prefix != "action" {
            continue;
        }

        let typename = format!("{}/{}/{}", msg.module, msg.prefix, msg.name);
        let rustname = format!("{}::{}::{}::Action", msg.module, msg.prefix, msg.name);

        lines.push_str(&format!(
            "
        if typename == \"{typename}\" {{
            return Ok(UntypedActionSupport::new::<{rustname}>());
        }}
",
            typename = typename,
            rustname = rustname
        ));
    }

    format!("{}{}{}", open, lines, close)
}
