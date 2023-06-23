#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]
#![allow(dead_code)]
#![allow(clippy::all)]

#[cfg(feature = "doc-only")]
include!(concat!(env!("OUT_DIR"), "/msg_bindings_doc_only.rs"));

#[cfg(not(feature = "doc-only"))]
include!(concat!(env!("OUT_DIR"), "/msg_bindings.rs"));

include!(concat!(env!("OUT_DIR"), "/introspection_functions.rs"));
include!(concat!(env!("OUT_DIR"), "/constants.rs"));

mod introspection;

use crate::introspection::MemberType;
use crate::introspection::MessageMember;
use quote::format_ident;
use quote::quote;
use r2r_common::RosMsg;
use r2r_rcl::*;
use rayon::prelude::*;
use std::borrow::Cow;
use std::ffi::CStr;
use std::mem;
use std::slice;

// Copied from bindgen.
// https://github.com/rust-lang/rust-bindgen/blob/e68b8c0e2b2ceeb42c35e74bd9344a1a99ec2e0c/src/ir/context.rs#L817
fn rust_mangle<'a>(name: &'a str) -> Cow<'a, str> {
    if name.contains('@')
        || name.contains('?')
        || name.contains('$')
        || matches!(
            name,
            "abstract"
                | "alignof"
                | "as"
                | "async"
                | "await"
                | "become"
                | "box"
                | "break"
                | "const"
                | "continue"
                | "crate"
                | "do"
                | "dyn"
                | "else"
                | "enum"
                | "extern"
                | "false"
                | "final"
                | "fn"
                | "for"
                | "if"
                | "impl"
                | "in"
                | "let"
                | "loop"
                | "macro"
                | "match"
                | "mod"
                | "move"
                | "mut"
                | "offsetof"
                | "override"
                | "priv"
                | "proc"
                | "pub"
                | "pure"
                | "ref"
                | "return"
                | "Self"
                | "self"
                | "sizeof"
                | "static"
                | "struct"
                | "super"
                | "trait"
                | "true"
                | "try"
                | "type"
                | "typeof"
                | "unsafe"
                | "unsized"
                | "use"
                | "virtual"
                | "where"
                | "while"
                | "yield"
                | "str"
                | "bool"
                | "f32"
                | "f64"
                | "usize"
                | "isize"
                | "u128"
                | "i128"
                | "u64"
                | "i64"
                | "u32"
                | "i32"
                | "u16"
                | "i16"
                | "u8"
                | "i8"
                | "_"
        )
    {
        let mut s = name.to_owned();
        s = s.replace('@', "_");
        s = s.replace('?', "_");
        s = s.replace('$', "_");
        s.push('_');
        return Cow::Owned(s);
    }
    Cow::Borrowed(name)
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
    let memberslice = slice::from_raw_parts((*members).members_, (*members).member_count_ as usize);
    (
        module.to_owned(),
        prefix.to_owned(),
        name.to_owned(),
        c_struct,
        memberslice,
    )
}

#[cfg(not(feature = "doc-only"))]
pub fn generate_rust_service(
    module_: &str,
    prefix_: &str,
    name_: &str,
) -> proc_macro2::TokenStream {
    let ident = format_ident!(
        "rosidl_typesupport_c__\
         get_service_type_support_handle__\
         {module_}__\
         {prefix_}__\
         {name_}"
    );

    quote!(
        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;

            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &* #ident ()
                }
            }
        }

    )
}

#[cfg(not(feature = "doc-only"))]
pub fn generate_rust_action(module_: &str, prefix_: &str, name_: &str) -> proc_macro2::TokenStream {
    let ident = format_ident!(
        "rosidl_typesupport_c__\
         get_action_type_support_handle__\
         {module_}__\
         {prefix_}__\
         {name_}"
    );

    quote! {
        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Action();
        impl WrappedActionTypeSupport for Action {
            type Goal = Goal;
            type Result = Result;
            type Feedback = Feedback;

            // internal structs
            type FeedbackMessage = FeedbackMessage;
            type SendGoal = SendGoal::Service;
            type GetResult = GetResult::Service;

            fn get_ts() -> &'static rosidl_action_type_support_t {
                unsafe {
                    &* #ident ()
                }
            }

            fn make_goal_request_msg(goal_id: unique_identifier_msgs::msg::UUID, goal: Goal) -> SendGoal::Request {
                SendGoal::Request {
                     goal_id,
                     goal
                }
            }

            fn make_goal_response_msg(accepted: bool, stamp: builtin_interfaces::msg::Time) -> SendGoal::Response {
                SendGoal::Response {
                     accepted,
                     stamp
                }
            }

            fn make_feedback_msg(goal_id: unique_identifier_msgs::msg::UUID, feedback: Feedback) -> FeedbackMessage {
                FeedbackMessage {
                     goal_id,
                     feedback
                }
            }

            fn make_result_request_msg(goal_id: unique_identifier_msgs::msg::UUID) -> GetResult::Request {
                GetResult::Request {
                     goal_id,
                }
            }

            fn make_result_response_msg(status: i8, result: Result) -> GetResult::Response {
                GetResult::Response {
                     status,
                     result,
                }
            }

            fn destructure_goal_request_msg(msg: SendGoal::Request) -> (unique_identifier_msgs::msg::UUID, Goal) {
                (msg.goal_id, msg.goal)
            }

            fn destructure_goal_response_msg(msg: SendGoal::Response) -> (bool, builtin_interfaces::msg::Time) {
                (msg.accepted, msg.stamp)
            }

            fn destructure_feedback_msg(msg: FeedbackMessage) -> (unique_identifier_msgs::msg::UUID, Feedback) {
                (msg.goal_id, msg.feedback)
            }

            fn destructure_result_response_msg(msg: GetResult::Response) -> (i8, Result) {
                (msg.status, msg.result)
            }

            fn destructure_result_request_msg(msg: GetResult::Request) -> unique_identifier_msgs::msg::UUID {
                msg.goal_id
            }
        }
    }
}

#[cfg(not(feature = "doc-only"))]
pub fn generate_rust_msg(module_: &str, prefix_: &str, name_: &str) -> proc_macro2::TokenStream {
    let key = format!("{}__{}__{}", module_, prefix_, name_);
    let func = *INTROSPECTION_FNS
        .get(key.as_str())
        .unwrap_or_else(|| panic!("code generation error: {}", key));
    let ptr = unsafe { func() };

    let (module, prefix, name, c_struct, c_members) = unsafe { introspection(ptr) };
    // let members: &[MessageMember] = unsafe { mem::transmute(c_members) };
    assert_eq!(module, module_);
    assert_eq!(prefix, prefix_);
    assert_eq!(name, name_);

    let name = if ["srv", "action"].contains(&prefix.as_str()) {
        // for srv, the message name is both the service name and _Request or _Respone
        // we only want to keep the last part.
        // same for actions with _Goal, _Result, _Feedback
        // TODO: refactor...
        // handle special case of ActionName_ServiceName_Response
        let nn: Vec<&str> = name.splitn(3, '_').collect();
        if let [_mod_name, _srv_name, msg_name] = &nn[..] {
            msg_name.to_string()
        } else if let [_mod_name, msg_name] = &nn[..] {
            msg_name.to_string()
        } else {
            panic!("malformed service name {}", name);
        }
    } else {
        name
    };

    // let module = format_ident!("{module}");
    // let prefix = format_ident!("{prefix}");
    let name = format_ident!("{name}");
    let c_struct = format_ident!("{c_struct}");

    let fields = {
        let fields: Vec<_> = c_members
            .into_iter()
            .filter_map(|c_member| {
                let member: &MessageMember = unsafe { mem::transmute(c_member) };

                let actual_field_name = member.name();
                let field_name = member.rust_name();
                let got_mangled = field_name != actual_field_name;
                if field_name == "structure_needs_at_least_one_member" {
                    // Yay we can have empty structs in rust
                    return None;
                }

                let field_name = format_ident!("{field_name}");

                let rust_field_type = member.type_id();
                let rust_field_type = if rust_field_type == MemberType::Message {
                    let (module, prefix, name, _, _) = unsafe { introspection(c_member.members_) };
                    let module = format_ident!("{module}");
                    let prefix = format_ident!("{prefix}");

                    // hack here to rustify nested action type names
                    if prefix == "action" {
                        if let Some((n1, n2)) = name.rsplit_once("_") {
                            let n1 = format_ident!("{n1}");
                            let n2 = format_ident!("{n2}");
                            quote! { #module :: #prefix :: #n1  :: #n2 }
                        } else {
                            let name = format_ident!("{name}");
                            quote! { #module :: #prefix :: #name }
                        }
                    } else {
                        let name = format_ident!("{name}");
                        quote! { #module :: #prefix :: #name }
                    }
                } else {
                    rust_field_type.to_rust_type()
                };

                let field = if c_member.is_array_ {
                    // if member.array_size_ > 0 {
                    // fixed size array
                    // format!("pub {}: [{};{}usize],\n",field_name, rust_field_type, array_size)
                    // actually lets use a vector anyway because its more convenient with traits. assert on the fixed size instead!
                    //} else {
                    // vector type
                    quote! { pub #field_name : Vec< #rust_field_type > }
                //}
                } else {
                    quote! { pub #field_name : #rust_field_type }
                };

                let attr = got_mangled.then(|| {
                    quote! { #[serde(rename = #actual_field_name )] }
                });

                Some(quote! {
                    #attr
                    #field
                })
            })
            .collect();
        quote! { #(#fields),* }
    };

    let from_native = {
        let fields = c_members.into_iter().filter_map(|c_member| {
            let member: &MessageMember = unsafe { mem::transmute(c_member) };
            let field_name = member.rust_name();
            if field_name == "structure_needs_at_least_one_member" {
                // Yay we can have empty structs in rust
                return None;
            }
            let field_name = format_ident!("{field_name}");
            let rust_field_type = member.type_id();
            let array_info = member.array_info();

            let fields = if let Some(array_info) = array_info {
                if array_info.size > 0 && !array_info.is_upper_bound {
                    // these are plain arrays
                    // let is_upper_bound = member.is_upper_bound();
                    // let array_size = array_info.size;
                    let fields1 = quote! {
                        // is_upper_bound_: #is_upper_bound,
                        // member.array_size_ : #array_size,
                    };

                    let field2 = match rust_field_type {
                        MemberType::Message => {
                            let (module, prefix, name, _, _) =
                                unsafe { introspection(c_member.members_) };
                            let module = format_ident!("{module}");
                            let prefix = format_ident!("{prefix}");
                            let name = format_ident!("{name}");

                            quote! {
                                #field_name: {
                                    let vec: Vec<_> = msg
                                        .#field_name
                                        .iter()
                                        .map(|s| #module :: #prefix :: #name :: from_native(s))
                                        .collect();
                                    vec
                                },
                            }
                        }
                        MemberType::String | MemberType::WString => {
                            quote! {
                                #field_name :
                                msg. #field_name .iter().map(|s|s.to_str().to_owned()).collect(),
                            }
                        }
                        _ => {
                            quote! {
                                #field_name : msg. #field_name .to_vec(),
                            }
                        }
                    };

                    quote! {
                        #fields1
                        #field2
                    }
                } else {
                    // these are __Sequence:s
                    // let is_upper_bound = member.is_upper_bound();
                    // let array_size = array_info.size;

                    let fields1 = quote! {
                        // is_upper_bound_: #is_upper_bound
                        // member.array_size_ : #array_size
                    };

                    let field2 = if rust_field_type == MemberType::Message {
                        let (module, prefix, name, _, _) =
                            unsafe { introspection(c_member.members_) };
                        let module = format_ident!("{module}");
                        let prefix = format_ident!("{prefix}");
                        let name = format_ident!("{name}");

                        quote! {
                            #field_name : {
                                let mut temp = Vec::with_capacity(msg. #field_name .size);
                                let slice = unsafe {
                                    std::slice::from_raw_parts(
                                        msg. #field_name .data,
                                        msg. #field_name .size
                                    )
                                };
                                for s in slice {
                                    temp.push( #module :: #prefix :: #name :: from_native(s));
                                }
                                temp
                            },
                        }
                    } else {
                        quote! {
                            #field_name: msg. #field_name .to_vec(),
                        }
                    };

                    quote! {
                        #fields1
                        #field2
                    }
                }
            } else {
                match rust_field_type {
                    MemberType::String | MemberType::WString => {
                        quote! {
                            #field_name: msg. #field_name .to_str().to_owned(),
                        }
                    }
                    MemberType::Message => {
                        let (module, prefix, name, _, _) =
                            unsafe { introspection(c_member.members_) };
                        let module = format_ident!("{module}");
                        let prefix = format_ident!("{prefix}");

                        // same hack as above to rustify message type names
                        if prefix == "action" {
                            let (srvname, msgname) =
                                name.rsplit_once("_").expect("ooops at from_native");
                            let srvname = format_ident!("{srvname}");
                            let msgname = format_ident!("{msgname}");

                            quote! {
                                #field_name:
                                #module ::
                                #prefix ::
                                #srvname ::
                                #msgname ::
                                from_native(&msg. #field_name ),
                            }
                        } else {
                            let name = format_ident!("{name}");

                            quote! {
                                #field_name :
                                #module :: #prefix :: #name ::from_native(&msg. #field_name),
                            }
                        }
                    }
                    _ => {
                        quote! {
                            #field_name : msg. #field_name,
                        }
                    }
                }
            };

            Some(fields)
        });
        // let name = name;
        quote! {
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> #name {
                #name {
                    #(#fields)*
                }
            }
        }
    };

    let copy_to_native = {
        let stmts = c_members.into_iter().filter_map(|c_member| {
            let member: &MessageMember = unsafe { mem::transmute(c_member) };
            let field_name_str = member.rust_name();
            if field_name_str == "structure_needs_at_least_one_member" {
                // Yay we can have empty structs in rust
                return None
            }
            let rust_field_type = member.type_id();
            let array_info = member.array_info();
            let field_name = format_ident!("{field_name_str}");

            let stmts = if let Some(array_info) = array_info {
                let array_size = array_info.size;

                if array_info.size > 0 && !array_info.is_upper_bound {
                    // these are plain arrays
                    // fixed size array, just copy but first check the size!

                    let content = match rust_field_type {
                        MemberType::Message => {
                            quote! {
                                for (t, s) in msg. #field_name .iter_mut().zip(&self. #field_name ) {
                                    s.copy_to_native(t);
                                }
                            }
                        }
                        MemberType::String | MemberType::WString => {
                            quote! {
                                for (t, s) in msg. #field_name .iter_mut().zip(&self. #field_name ) {
                                    t.assign(&s);
                                }
                            }
                        }
                        _ => {
                            quote! {
                                msg. #field_name .copy_from_slice(&self. #field_name [.. #array_size ]);
                            }
                        }
                    };

                    quote! {
                        assert_eq!(
                            self. #field_name .len(),
                            #array_size ,
                            "Field {} is fixed size of {}!", #field_name_str, #array_size
                        );
                        #content
                    }
                } else {
                    // these are __Sequence:s

                    if rust_field_type == MemberType::Message {
                        let (_, _, _, c_struct, _) = unsafe { introspection(c_member.members_) };
                        let init_func = format_ident!("{c_struct}__Sequence__init");
                        let fini_func = format_ident!("{c_struct}__Sequence__fini");
                        let field_name = format_ident!("{field_name}");

                        quote! {
                            unsafe {
                                #fini_func (&mut msg. #field_name);
                                #init_func (&mut msg. #field_name , self. #field_name .len());

                                let slice = std::slice::from_raw_parts_mut(msg. #field_name .data, msg. #field_name .size);
                                for (t, s) in slice.iter_mut().zip(&self. #field_name ) {
                                    s.copy_to_native(t);
                                }
                            }
                        }
                    } else {
                        // extra assertion
                        let assert_stmt = array_info.is_upper_bound.then(|| {
                            quote! {
                                assert!(
                                    self. #field_name .len() <= #array_size,
                                    "Field {} is upper bounded by {}!",
                                    #field_name_str, #array_size
                                );
                            }
                        });

                        quote! {
                            #assert_stmt
                            msg. #field_name .update(&self. #field_name );
                        }
                    }
                }
            } else {
                match rust_field_type {
                    MemberType::String | MemberType::WString => {
                        quote! {
                            msg. #field_name .assign(&self. #field_name );
                        }
                    }
                    MemberType::Message => {
                        quote! {
                            self. #field_name .copy_to_native(&mut msg. #field_name );
                        }
                    }
                    _ => {
                        quote! {
                            msg. #field_name = self. #field_name;
                        }
                    }
                }
            };

            Some(stmts)
        });

        quote! {
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                #(#stmts)*
            }
        }
    };

    let typesupport = {
        let type_support_handle =
            format_ident!("rosidl_typesupport_c__get_message_type_support_handle__{c_struct}");
        let create_func = format_ident!("{c_struct}__create");
        let destroy_func = format_ident!("{c_struct}__destroy");

        quote! {
            impl WrappedTypesupport for #name {
                type CStruct = #c_struct;

                fn get_ts() -> &'static rosidl_message_type_support_t {
                    unsafe {
                        &* #type_support_handle()
                    }
                }

                fn create_msg() -> *mut #c_struct {
                    unsafe {
                        #create_func ()
                    }
                }

                fn destroy_msg(msg: *mut #c_struct) -> () {
                    unsafe {
                        #destroy_func (msg)
                    };
                }

                #from_native
                #copy_to_native
            }
        }
    };

    let impl_default = quote! {
        impl Default for #name {
            fn default() -> Self {
                let msg_native = WrappedNativeMsg::< #name >::new();
                #name :: from_native(&msg_native)
            }
        }
    };

    let constant_items: Vec<_> = CONSTANTS_MAP
        .get(&key)
        .cloned()
        .into_iter()
        .flatten()
        .map(|(const_name, typ)| {
            let typ: Box<syn::Type> = syn::parse_str(typ).unwrap();
            let const_name = format_ident!("{const_name}");
            let value = format_ident!("{key}__{const_name}");
            quote! { pub const #const_name: #typ = #value; }
        })
        .collect();

    let impl_constants = if constant_items.is_empty() {
        quote! {}
    } else {
        quote! {
            #[allow(non_upper_case_globals)]
            impl #name {
                #(#constant_items)*
            }
        }
    };

    quote! {
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct #name {
            #fields
        }

        #typesupport
        #impl_default
        #impl_constants
    }
}

#[cfg(not(feature = "doc-only"))]
pub fn generate_untyped_helper(msgs: &[RosMsg]) -> proc_macro2::TokenStream {
    let (funcs, entries): (Vec<_>, Vec<_>) = msgs
        .into_par_iter()
        .filter(|msg| !["srv", "action"].contains(&msg.prefix.as_str()))
        .map(|msg| {
            let RosMsg {
                module,
                prefix,
                name,
            } = msg;
            let typename = type_name(&msg);
            let rustname = {
                let module = format_ident!("{module}");
                let prefix = format_ident!("{prefix}");
                let name = format_ident!("{name}");
                quote! { #module :: #prefix :: #name }
            };

            let func_ident =
                format_ident!("new_wrapped_native_msg_untyped_{module}_{prefix}_{name}");
            let func = quote! {
                #[allow(non_snake_case)]
                fn #func_ident() -> WrappedNativeMsgUntyped {
                    WrappedNativeMsgUntyped::new::< #rustname >()
                }
            };
            let entry = quote! { #typename => #func_ident };

            (unsafe { force_send(func) }, unsafe { force_send(entry) })
        })
        .unzip();
    let funcs = funcs.into_iter().map(|tokens| tokens.unwrap());
    let entries = entries.into_iter().map(|tokens| tokens.unwrap());

    quote! {
        impl WrappedNativeMsgUntyped {
            pub fn new_from(typename: &str) -> Result<Self> {
                #(#funcs)*

                static MAP: phf::Map<&'static str, fn() -> WrappedNativeMsgUntyped> = phf::phf_map! { #(#entries),* };

                let func = MAP.get(typename)
                    .ok_or_else(|| Error::InvalidMessageType{ msgtype: typename.into() })?;
                Ok(func())
            }
        }
    }
}

#[cfg(not(feature = "doc-only"))]
pub fn generate_untyped_service_helper(msgs: &[RosMsg]) -> proc_macro2::TokenStream {
    let (funcs, entries): (Vec<_>, Vec<_>) = msgs
        .into_par_iter()
        .filter(|msg| msg.prefix == "srv")
        .map(|msg| {
            let RosMsg {
                module,
                prefix,
                name,
            } = msg;
            let typename = type_name(&msg);
            let rustname = {
                let module = format_ident!("{module}");
                let prefix = format_ident!("{prefix}");
                let name = format_ident!("{name}");
                quote! { #module :: #prefix :: #name :: Service }
            };
            let func_ident = format_ident!("new_untyped_service_support_{module}_{prefix}_{name}");

            let func = quote! {
                #[allow(non_snake_case)]
                fn #func_ident() -> UntypedServiceSupport {
                    UntypedServiceSupport::new::< #rustname  >()
                }
            };
            let entry = quote! { #typename => #func_ident };

            unsafe { (force_send(func), force_send(entry)) }
        })
        .unzip();
    let funcs = funcs.into_iter().map(|tokens| tokens.unwrap());
    let entries = entries.into_iter().map(|tokens| tokens.unwrap());

    quote! {
        impl UntypedServiceSupport {
            pub fn new_from(typename: &str) -> Result<Self> {
                #(#funcs)*

                static MAP: phf::Map<&'static str, fn() -> UntypedServiceSupport> = phf::phf_map! { #(#entries),* };

                let func = MAP.get(typename)
                    .ok_or_else(|| Error::InvalidMessageType{ msgtype: typename.into() })?;
                Ok(func())
            }
        }
    }
}

#[cfg(not(feature = "doc-only"))]
pub fn generate_untyped_action_helper(msgs: &[RosMsg]) -> proc_macro2::TokenStream {
    let (funcs, entries): (Vec<_>, Vec<_>) = msgs
        .into_par_iter()
        .filter(|msg| msg.prefix == "action")
        .map(|msg| {
            let RosMsg {
                module,
                prefix,
                name,
            } = msg;
            let typename = type_name(&msg);
            let rustname = {
                let module = format_ident!("{module}");
                let prefix = format_ident!("{prefix}");
                let name = format_ident!("{name}");
                quote! { #module :: #prefix :: #name :: Action }
            };
            let func_ident = format_ident!("new_untyped_service_support_{module}_{prefix}_{name}");

            let func = quote! {
                #[allow(non_snake_case)]
                fn #func_ident() -> UntypedActionSupport {
                    UntypedActionSupport::new::< #rustname  >()
                }
            };
            let entry = quote! { #typename => #func_ident };

            unsafe { (force_send(func), force_send(entry)) }
        })
        .unzip();

    let funcs = funcs.into_iter().map(|tokens| tokens.unwrap());
    let entries = entries.into_iter().map(|tokens| tokens.unwrap());

    quote! {
        impl UntypedActionSupport {
            pub fn new_from(typename: &str) -> Result<Self> {
                #(#funcs)*

                static MAP: phf::Map<&'static str, fn() -> UntypedActionSupport> = phf::phf_map! { #(#entries),* };

                let func = MAP.get(typename)
                    .ok_or_else(|| Error::InvalidMessageType{ msgtype: typename.into() })?;
                Ok(func())
            }
        }
    }
}

fn type_name(msg: &RosMsg) -> String {
    let RosMsg {
        module,
        prefix,
        name,
    } = msg;
    format!("{module}/{prefix}/{name}")
}

fn rust_name(msg: &RosMsg) -> proc_macro2::TokenStream {
    let RosMsg {
        module,
        prefix,
        name,
    } = msg;
    let module = format_ident!("{module}");
    let prefix = format_ident!("{prefix}");
    let name = format_ident!("{name}");
    quote! { #module :: #prefix :: #name :: Action }
}

unsafe fn force_send<T>(value: T) -> force_send_sync::Send<T> {
    force_send_sync::Send::new(value)
}
