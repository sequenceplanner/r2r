#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]
#![allow(improper_ctypes)]
#![allow(dead_code)]
// #![warn(clippy::pedantic)]
include!(concat!(env!("OUT_DIR"), "/msg_bindings.rs"));
include!(concat!(env!("OUT_DIR"), "/introspection_functions.rs"));
include!(concat!(env!("OUT_DIR"), "/constants.rs"));

#[macro_use]
extern crate lazy_static;

use proc_macro2::{Ident, Span, TokenStream};
use quote::{format_ident, quote};
use r2r_rcl::{
    rosidl_action_type_support_t, rosidl_message_type_support_t,
    rosidl_runtime_c__String, rosidl_runtime_c__String__Sequence,
    rosidl_runtime_c__U16String, rosidl_runtime_c__U16String__Sequence,
    rosidl_runtime_c__boolean__Sequence, rosidl_runtime_c__double__Sequence,
    rosidl_runtime_c__float__Sequence, rosidl_runtime_c__int16__Sequence,
    rosidl_runtime_c__int32__Sequence, rosidl_runtime_c__int64__Sequence,
    rosidl_runtime_c__int8__Sequence, rosidl_runtime_c__octet__Sequence,
    rosidl_runtime_c__uint16__Sequence, rosidl_runtime_c__uint32__Sequence,
    rosidl_runtime_c__uint64__Sequence, rosidl_runtime_c__uint8__Sequence,
    rosidl_service_type_support_t, rosidl_typesupport_introspection_c__MessageMember,
    rosidl_typesupport_introspection_c__MessageMembers,
};
use std::borrow::Cow;
use std::collections::HashMap;
use std::ffi::CStr;

/// Convert the identifier name to a valid rust identifier.
///
/// Special characters ('@', '?', '$') are replaced with `_` and a trailing `_` is added if the name is a rust keyword.
///
/// Copied from bindgen.
/// <https://github.com/rust-lang/rust-bindgen/blob/e68b8c0e2b2ceeb42c35e74bd9344a1a99ec2e0c/src/ir/context.rs#L817>
fn rust_mangle(name: &str) -> Cow<str> {
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

/// Confirm the ROS_TYPE enum's value is still the same as what we hard coded in [`FieldType::new`].
#[cfg(any(r2r__ros__distro__galactic, r2r__ros__distro__foxy))]
pub fn assert_field_type_match_c_enum() {
    use r2r_rcl::{
        rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,
        rosidl_typesupport_introspection_c__ROS_TYPE_CHAR,
        rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,
        rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,
        rosidl_typesupport_introspection_c__ROS_TYPE_INT16,
        rosidl_typesupport_introspection_c__ROS_TYPE_INT32,
        rosidl_typesupport_introspection_c__ROS_TYPE_INT64,
        rosidl_typesupport_introspection_c__ROS_TYPE_INT8,
        rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE,
        rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,
        rosidl_typesupport_introspection_c__ROS_TYPE_OCTET,
        rosidl_typesupport_introspection_c__ROS_TYPE_STRING,
        rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,
        rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,
        rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,
        rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,
        rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR,
        rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING,
    };
    assert_eq!(rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT as u32, 1);
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE as u32,
        2
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE as u32,
        3
    );
    assert_eq!(rosidl_typesupport_introspection_c__ROS_TYPE_CHAR as u32, 4);
    assert_eq!(rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR as u32, 5);
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN as u32,
        6
    );
    assert_eq!(rosidl_typesupport_introspection_c__ROS_TYPE_OCTET as u32, 7);
    assert_eq!(rosidl_typesupport_introspection_c__ROS_TYPE_UINT8 as u32, 8);
    assert_eq!(rosidl_typesupport_introspection_c__ROS_TYPE_INT8 as u32, 9);
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_UINT16 as u32,
        10
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_INT16 as u32,
        11
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_UINT32 as u32,
        12
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_INT32 as u32,
        13
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_UINT64 as u32,
        14
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_INT64 as u32,
        15
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_STRING as u32,
        16
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING as u32,
        17
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE as u32,
        18
    );
}

/// Confirm the ROS_TYPE enum's value is still the same as what we hard coded in [`FieldType::new`].
#[cfg(any(r2r__ros__distro__humble, r2r__ros__distro__rolling))]
pub fn assert_field_type_match_c_enum() {
    use r2r_rcl::rosidl_typesupport_introspection_c_field_types::*;
    assert_eq!(rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT as u32, 1);
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE as u32,
        2
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE as u32,
        3
    );
    assert_eq!(rosidl_typesupport_introspection_c__ROS_TYPE_CHAR as u32, 4);
    assert_eq!(rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR as u32, 5);
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN as u32,
        6
    );
    assert_eq!(rosidl_typesupport_introspection_c__ROS_TYPE_OCTET as u32, 7);
    assert_eq!(rosidl_typesupport_introspection_c__ROS_TYPE_UINT8 as u32, 8);
    assert_eq!(rosidl_typesupport_introspection_c__ROS_TYPE_INT8 as u32, 9);
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_UINT16 as u32,
        10
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_INT16 as u32,
        11
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_UINT32 as u32,
        12
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_INT32 as u32,
        13
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_UINT64 as u32,
        14
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_INT64 as u32,
        15
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_STRING as u32,
        16
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING as u32,
        17
    );
    assert_eq!(
        rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE as u32,
        18
    );
}

enum FieldType {
    Bool,
    I8,
    U8,
    I16,
    U16,
    I32,
    U32,
    I64,
    U64,
    F32,
    F64,
    U128,
    String,
    Message(TypeDescription),
}

impl FieldType {
    /// Construct a field type from an integer representation of a `rosidl_typesupport_introspection_c__ROS_TYPE_` enum and a closure.
    ///
    /// The first argument is an integer representation of the enum that should be within [1, 18] range.
    ///
    /// The second argument is a closure that produces a TypeDescription. It is called when the first argument is `18` (which means custom message).
    ///
    /// The [`assert_field_type`] needs to be called at least once to ensure the C side enum's value still matches the integer value we hardcoded.
    fn new(t: u8, f: impl FnOnce() -> TypeDescription) -> Self {
        match t {
            1 => FieldType::F32,
            2 => FieldType::F64,
            3 => FieldType::U128,
            4 | 9 => FieldType::I8,
            5 | 11 => FieldType::I16,
            6 => FieldType::Bool,
            7 | 8 => FieldType::U8,
            10 => FieldType::U16,
            12 => FieldType::U32,
            13 => FieldType::I32,
            14 => FieldType::U64,
            15 => FieldType::I64,
            16 | 17 => FieldType::String,
            18 => FieldType::Message(f()),
            _ => unreachable!(
                "rosidl_typesupport_introspection_c__ROS_TYPE_xxx should be between 1 and 18"
            ),
        }
    }

    fn get_path(&self) -> TokenStream {
        match self {
            FieldType::Bool => quote!(bool),
            FieldType::I8 => quote!(i8),
            FieldType::U8 => quote!(u8),
            FieldType::I16 => quote!(i16),
            FieldType::U16 => quote!(u16),
            FieldType::I32 => quote!(i32),
            FieldType::U32 => quote!(u32),
            FieldType::I64 => quote!(i64),
            FieldType::U64 => quote!(u64),
            FieldType::F32 => quote!(f32),
            FieldType::F64 => quote!(f64),
            FieldType::U128 => quote!(u128),
            FieldType::String => quote!(std::string::String),
            FieldType::Message(ty) => ty.get_path(),
        }
    }
}

struct TypeDescription {
    rust_path: Vec<Ident>,
    c_name: Ident,
}

impl TypeDescription {
    fn new(namespace: &str, name: &str) -> Self {
        let mut namespace_iter = namespace.split("__");
        let pkg_name = namespace_iter.next().unwrap();
        let interface_type = namespace_iter.next().unwrap();
        let c_name = format_ident!("{pkg_name}__{interface_type}__{name}");

        let rust_path = match interface_type {
            "msg" => {
                vec![
                    Ident::new(pkg_name, Span::call_site()),
                    Ident::new(interface_type, Span::call_site()),
                    Ident::new(name, Span::call_site()),
                ]
            }
            "srv" => {
                let mut name_iter = name.splitn(2, '_');
                let srv_name = name_iter.next().unwrap();
                let Some(msg_name @ ("Request" | "Response")) = name_iter.next() else { panic!("Message name for a service should be \"Request\" or \"Response\"") };
                vec![
                    Ident::new(pkg_name, Span::call_site()),
                    Ident::new(interface_type, Span::call_site()),
                    Ident::new(srv_name, Span::call_site()),
                    Ident::new(msg_name, Span::call_site()),
                ]
            }
            "action" => {
                let mut name_iter = name.splitn(3, '_');
                let action_name = name_iter.next().unwrap();
                let srv_or_msg_name = name_iter.next().unwrap();
                let msg_name = name_iter.next();
                match (srv_or_msg_name, msg_name) {
                    (msg_name @ ("Goal" | "Result" | "Feedback" | "FeedbackMessage"), None) => {
                        vec![
                            Ident::new(pkg_name, Span::call_site()),
                            Ident::new(interface_type, Span::call_site()),
                            Ident::new(action_name, Span::call_site()),
                            Ident::new(msg_name, Span::call_site()),
                        ]
                    }
                    (
                        srv_name @ ("SendGoal" | "GetResult"),
                        Some(msg_name @ ("Request" | "Response")),
                    ) => {
                        vec![
                            Ident::new(pkg_name, Span::call_site()),
                            Ident::new(interface_type, Span::call_site()),
                            Ident::new(action_name, Span::call_site()),
                            Ident::new(srv_name, Span::call_site()),
                            Ident::new(msg_name, Span::call_site()),
                        ]
                    }
                    _ => {
                        panic!("Invalid message name {name} under {namespace}")
                    }
                }
            }
            interface_name => {
                panic!("Invalid interface name: {interface_name}")
            }
        };

        Self { rust_path, c_name }
    }

    /// Get the fully qualified path of the type in Rust. This should be a list of identifiers separated by `::`.
    fn get_path(&self) -> TokenStream {
        let components = &self.rust_path;
        quote!(#(#components)::*)
    }

    /// Get the name of the type in C. This should be a single identifier.
    fn get_c_name(&self) -> TokenStream {
        let c_name = &self.c_name;
        quote!(#c_name)
    }

    /// Get the name of the type in Rust. This should be a single identifier.
    fn get_rust_name(&self) -> TokenStream {
        let rust_name = self.rust_path.last().unwrap();
        quote!(#rust_name)
    }
}

enum FieldRepitition {
    Scalar,
    StaticArray(usize),
    BoundedArray(usize),
    UnboundedArray,
}

struct FieldDescription {
    field_name: Ident,
    c_field_name: Ident,
    is_mangled: bool,
    ty: FieldType,
    repitition: FieldRepitition,
}

impl FieldDescription {
    unsafe fn from_raw(member: &rosidl_typesupport_introspection_c__MessageMember) -> Option<Self> {
        let c_field_name = CStr::from_ptr(member.name_).to_str().unwrap();
        let field_name = rust_mangle(c_field_name);
        if field_name == "structure_needs_at_least_one_member" {
            return None;
        }
        let is_mangled = matches!(field_name, Cow::Owned(_));
        let ty = FieldType::new(member.type_id_, || {
            let members = *(*member.members_)
                .data
                .cast::<rosidl_typesupport_introspection_c__MessageMembers>();
            let namespace = CStr::from_ptr(members.message_namespace_).to_str().unwrap();
            let name = CStr::from_ptr(members.message_name_).to_str().unwrap();
            TypeDescription::new(namespace, name)
        });
        let repitition = match (member.is_array_, member.is_upper_bound_, member.array_size_) {
            (false, _, _) => FieldRepitition::Scalar,
            (true, true, n) => FieldRepitition::BoundedArray(n),
            (true, false, 0) => FieldRepitition::UnboundedArray,
            (true, false, n) => FieldRepitition::StaticArray(n),
        };
        Some(Self {
            field_name: Ident::new(&field_name, Span::call_site()),
            c_field_name: Ident::new(c_field_name, Span::call_site()),
            is_mangled,
            ty,
            repitition,
        })
    }

    fn get_struct_field_token(&self) -> TokenStream {
        let serde_rename = if self.is_mangled {
            let c_field_name_str = self.c_field_name.to_string();
            quote!(
                #[serde(rename = #c_field_name_str)]
            )
        } else {
            quote!()
        };

        let name = &self.field_name;
        let path = self.ty.get_path();

        match self.repitition {
            FieldRepitition::Scalar => quote!(
                #serde_rename
                pub #name: #path
            ),
            FieldRepitition::BoundedArray(n) => {
                let comment = format!("Max length: {n}");
                quote!(
                    #[doc = #comment]
                    #serde_rename
                    pub #name: Vec<#path>
                )
            }
            FieldRepitition::UnboundedArray => quote!(
                #serde_rename
                pub #name: Vec<#path>
            ),
            FieldRepitition::StaticArray(n) => {
                let comment = format!("Length: {n}");
                quote!(
                    #[doc = #comment]
                    #serde_rename
                    pub #name: Vec<#path>
                    // pub #name: [#path; #n]
                )
            }
        }
    }

    fn get_from_native_token(&self) -> TokenStream {
        use FieldRepitition::{BoundedArray, Scalar, StaticArray, UnboundedArray};
        use FieldType::{Message, String};

        let field_name = &self.field_name;
        let path = self.ty.get_path();

        let converter_expr = match &self.ty {
            Message(_) => quote!(#path::from_native(&native)),
            String => quote!(native.to_str().to_owned()),
            _ => quote!(),
        };

        match (&self.repitition, &self.ty) {
            (Scalar, Message(_)) => quote!(#field_name: #path::from_native(&msg.#field_name)),
            (Scalar, String) => quote!(#field_name: msg.#field_name.to_str().to_owned()),
            (Scalar, _) => quote!(#field_name: msg.#field_name),
            (StaticArray(_), Message(_) | String) => quote!(
                #field_name: {
                    // let mut data: [std::mem::MaybeUninit<#path>; #n] = unsafe {
                    //     std::mem::MaybeUninit::uninit().assume_init()
                    // };

                    // for (elem, native) in data[..].iter_mut().zip(msg.#field_name.iter()) {
                    //     elem.write(#converter_expr);
                    // }

                    // unsafe { std::mem::transmute::<_, [#path; #n]>(data) }
                    let mut data = Vec::with_capacity(msg.#field_name.len());
                    for native in &msg.#field_name {
                        data.push(#converter_expr);
                    }
                    data
                }
            ),
            (StaticArray(_), _) => quote!(#field_name: msg.#field_name.to_vec()),
            (BoundedArray(_) | UnboundedArray, Message(_) | String) => quote!(
                #field_name: {
                    let mut data = Vec::with_capacity(msg.#field_name.size);
                    let slice = unsafe { std::slice::from_raw_parts(msg.#field_name.data, msg.#field_name.size)};
                    for native in slice {
                        data.push(#converter_expr);
                    }
                    data
                }
            ),
            (BoundedArray(_) | UnboundedArray, _) => quote!(#field_name: msg.#field_name.to_vec()),
        }
    }

    fn get_copy_to_native_token(&self) -> TokenStream {
        use FieldRepitition::{BoundedArray, Scalar, StaticArray, UnboundedArray};
        use FieldType::{Message, String};

        let field_name = &self.field_name;

        let size_checking = match &self.repitition {
            BoundedArray(n) => {
                let err_msg = format!(
                    "Expected at most {{}} elements in `{field_name}` field, but found {{}}"
                );
                quote!(assert!(self.#field_name.len() <= #n, #err_msg, #n, self.#field_name.len());)
            }
            StaticArray(n) => {
                let err_msg =
                    format!("Expected {{}} elements in `{field_name}` field, but found {{}}");
                quote!(assert!(self.#field_name.len() == #n, #err_msg, #n, self.#field_name.len());)
            }
            _ => {
                quote!()
            }
        };

        match (&self.repitition, &self.ty) {
            (Scalar, Message(_)) => quote!(self.#field_name.copy_to_native(&mut msg.#field_name);),
            (Scalar, String) => quote!(msg.#field_name.assign(&self.#field_name);),
            (Scalar, _) => quote!(msg.#field_name = self.#field_name;),
            (StaticArray(_), Message(_)) => quote!(
                #size_checking
                for (t,s) in msg.#field_name.iter_mut().zip(&self.#field_name) {
                    s.copy_to_native(t);
                }
            ),
            (StaticArray(_), String) => quote!(
                #size_checking
                for (t,s) in msg.#field_name.iter_mut().zip(&self.#field_name) {
                    t.assign(s);
                }
            ),
            (StaticArray(n), _) => quote!(
                #size_checking
                msg.#field_name.copy_from_slice(&self.#field_name[..#n]);
            ),
            (BoundedArray(_) | UnboundedArray, Message(message_type_description)) => {
                let c_struct_ident = message_type_description.get_c_name();
                let c_struct_fini_ident = format_ident!("{c_struct_ident}__Sequence__fini");
                let c_struct_init_ident = format_ident!("{c_struct_ident}__Sequence__init");
                quote!(
                    #size_checking
                    let slice = unsafe {
                        #c_struct_fini_ident(&mut msg.#field_name);
                        #c_struct_init_ident(&mut msg.#field_name, self.#field_name.len());
                        std::slice::from_raw_parts_mut(msg.#field_name.data, msg.#field_name.size)
                    };
                    for (t,s) in slice.iter_mut().zip(&self.#field_name) {
                        s.copy_to_native(t);
                    }
                )
            }
            (BoundedArray(_) | UnboundedArray, _) => {
                quote!(
                    #size_checking
                    msg.#field_name.update(&self.#field_name);
                )
            }
        }
    }
}

struct StructDescription {
    ty: TypeDescription,
    fields: Vec<FieldDescription>,
    constants: Vec<(Ident, Ident, Ident)>,
}

impl StructDescription {
    unsafe fn from_raw(
        ptr: *const rosidl_message_type_support_t,
        constants: &[(String, String)],
    ) -> Self {
        let members = *(*ptr)
            .data
            .cast::<rosidl_typesupport_introspection_c__MessageMembers>();
        let namespace = CStr::from_ptr(members.message_namespace_).to_str().unwrap();
        let name = CStr::from_ptr(members.message_name_).to_str().unwrap();
        let ty = TypeDescription::new(namespace, name);
        let memberslice =
            std::slice::from_raw_parts(members.members_, members.member_count_ as usize);
        let constants = constants
            .iter()
            .map(|(const_name, type_name)| {
                let const_name = const_name.trim();
                let type_name = type_name.trim();
                (
                    Ident::new(const_name, Span::call_site()),
                    Ident::new(type_name, Span::call_site()),
                    Ident::new(
                        &format!("{}__{}", ty.get_c_name(), const_name),
                        Span::call_site(),
                    ),
                )
            })
            .collect();
        Self {
            ty,
            fields: memberslice
                .iter()
                .filter_map(|x| unsafe { FieldDescription::from_raw(x) })
                .collect(),
            constants,
        }
    }

    fn into_token_stream(self) -> TokenStream {
        let msgname = self.ty.get_rust_name();
        let field_items = self
            .fields
            .iter()
            .map(FieldDescription::get_struct_field_token);
        let arg_ident = if self.fields.is_empty() {
            Ident::new("_msg", Span::call_site())
        } else {
            Ident::new("msg", Span::call_site())
        };
        let fields_from_native = self
            .fields
            .iter()
            .map(FieldDescription::get_from_native_token);
        let fields_copy_to_native = self
            .fields
            .iter()
            .map(FieldDescription::get_copy_to_native_token);

        let c_struct_ident = self.ty.get_c_name();
        let c_struct_get_ts_ident = format_ident!(
            "rosidl_typesupport_c__get_message_type_support_handle__{c_struct_ident}"
        );
        let c_struct_create_ident = format_ident!("{c_struct_ident}__create");
        let c_struct_destroy_ident = format_ident!("{c_struct_ident}__destroy");

        let constants_block = if self.constants.is_empty() {
            quote!()
        } else {
            let consts_name = self.constants.iter().map(|(x, _, _)| x);
            let consts_type = self.constants.iter().map(|(_, y, _)| y);
            let consts_c_name = self.constants.iter().map(|(_, _, z)| z);

            quote!(
                impl #msgname {
                    #(pub const #consts_name: #consts_type = #consts_c_name;)*
                }
            )
        };

        quote! {
            #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
            #[serde(default)]
            pub struct #msgname {
                #(#field_items,)*
            }

            impl WrappedTypesupport for #msgname {
                type CStruct = #c_struct_ident;

                fn get_ts() -> &'static rosidl_message_type_support_t {
                    unsafe { &*#c_struct_get_ts_ident() }
                }

                fn create_msg() -> *mut Self::CStruct {
                    unsafe { #c_struct_create_ident() }
                }

                fn destroy_msg(msg: *mut Self::CStruct) -> () {
                    unsafe { #c_struct_destroy_ident(msg) };
                }

                fn from_native(#arg_ident: &Self::CStruct) -> #msgname {
                    #msgname{
                        #(#fields_from_native,)*
                    }
                }

                fn copy_to_native(&self, #arg_ident: &mut Self::CStruct) {
                    #(#fields_copy_to_native)*
                }
            }

            impl Default for #msgname {
                fn default() -> Self {
                    let msg_native = WrappedNativeMsg::<#msgname>::new();
                    #msgname::from_native(&msg_native)
                }
            }

            #constants_block
        }
    }
}

pub fn generate_rust_msg(module: &str, prefix: &str, name: &str) -> TokenStream {
    let key = format!("{module}__{prefix}__{name}");
    let ptr = INTROSPECTION_FNS
        .get(key.as_str())
        .unwrap_or_else(|| panic!("code generation error: {}", key));
    let ptr = *ptr as *const i32 as *const rosidl_message_type_support_t;
    let constants = CONSTANTS_MAP.get(key.as_str()).cloned().unwrap_or_default();

    let struct_description = unsafe { StructDescription::from_raw(ptr, &constants) };
    assert_eq!(struct_description.ty.rust_path[0], module);
    assert_eq!(struct_description.ty.rust_path[1], prefix);

    struct_description.into_token_stream()
}

pub fn generate_rust_service(module: &str, prefix: &str, name: &str) -> TokenStream {
    let type_support_ident = format_ident!(
        "rosidl_typesupport_c__get_service_type_support_handle__{module}__{prefix}__{name}"
    );
    quote!(
        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*#type_support_ident()
                }
            }
        }
    )
}

pub fn generate_rust_action(module: &str, prefix: &str, name: &str) -> TokenStream {
    let type_support_ident = format_ident!(
        "rosidl_typesupport_c__get_action_type_support_handle__{module}__{prefix}__{name}"
    );
    quote!(
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
                    &*#type_support_ident()
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
    )
}

pub fn generate_untyped_helper(msgs: &[r2r_common::RosMsg]) -> TokenStream {
    let msgs = msgs.iter().filter(|msg| msg.prefix == "msg");
    let msgs_typename = msgs
        .clone()
        .map(|msg| format!("{}/{}/{}", msg.module, msg.prefix, msg.name));
    let msgs_rustname = msgs.map(|msg| {
        let module = Ident::new(&msg.module, Span::call_site());
        let prefix = Ident::new(&msg.prefix, Span::call_site());
        let name = Ident::new(&msg.name, Span::call_site());
        quote!(#module::#prefix::#name)
    });
    quote! {
        impl WrappedNativeMsgUntyped {
            pub fn new_from(typename: &str) -> Result<Self> {
                match typename {
                    #(
                        #msgs_typename => Ok(WrappedNativeMsgUntyped::new::<#msgs_rustname>()),
                    )*
                    _ => Err(Error::InvalidMessageType{ msgtype: typename.into() })
                }
            }
        }
    }
}

pub fn generate_untyped_service_helper(msgs: &[r2r_common::RosMsg]) -> TokenStream {
    let msgs = msgs.iter().filter(|msg| msg.prefix == "srv");
    let msgs_typename = msgs
        .clone()
        .map(|msg| format!("{}/{}/{}", msg.module, msg.prefix, msg.name));
    let msgs_rustname = msgs.map(|msg| {
        let module = Ident::new(&msg.module, Span::call_site());
        let prefix = Ident::new(&msg.prefix, Span::call_site());
        let name = Ident::new(&msg.name, Span::call_site());
        quote!(#module::#prefix::#name::Service)
    });
    quote! {
        impl UntypedServiceSupport {
            pub fn new_from(typename: &str) -> Result<Self> {
                match typename {
                    #(
                        #msgs_typename => Ok(UntypedServiceSupport::new::<#msgs_rustname>()),
                    )*
                    _ => Err(Error::InvalidMessageType{ msgtype: typename.into() })
                }
            }
        }
    }
}

pub fn generate_untyped_action_helper(msgs: &[r2r_common::RosMsg]) -> TokenStream {
    let msgs = msgs.iter().filter(|msg| msg.prefix == "action");
    let msgs_typename = msgs
        .clone()
        .map(|msg| format!("{}/{}/{}", msg.module, msg.prefix, msg.name));
    let msgs_rustname = msgs.map(|msg| {
        let module = Ident::new(&msg.module, Span::call_site());
        let prefix = Ident::new(&msg.prefix, Span::call_site());
        let name = Ident::new(&msg.name, Span::call_site());
        quote!(#module::#prefix::#name::Action)
    });
    quote! {
        impl UntypedActionSupport {
            pub fn new_from(typename: &str) -> Result<Self> {
                match typename {
                    #(
                        #msgs_typename => Ok(UntypedActionSupport::new::<#msgs_rustname>()),
                    )*
                    _ => Err(Error::InvalidMessageType{ msgtype: typename.into() })
                }
            }
        }
    }
}
