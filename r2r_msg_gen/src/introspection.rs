use crate::rosidl_message_type_support_t as CTypeSupport;
use crate::rosidl_typesupport_introspection_c__MessageMember as CMessageMember;
use crate::rosidl_typesupport_introspection_c__MessageMembers as CMessageMembers;
use crate::rust_mangle;
use quote::quote;
use std::borrow::Cow;
use std::ffi::c_char;
use std::ffi::CStr;
use std::mem;
use std::slice;

pub struct Introspection<'a> {
    pub module: &'a str,
    pub prefix: &'a str,
    pub name: &'a str,
    pub members: &'a [MessageMember],
}

impl<'a> Introspection<'a> {
    pub fn c_struct_name(&self) -> String {
        let Self {
            module,
            prefix,
            name,
            ..
        } = *self;
        format!("{module}__{prefix}__{name}")
    }
}

#[repr(transparent)]
pub struct TypeSupport(CTypeSupport);

impl TypeSupport {
    pub unsafe fn introspection(&self) -> Introspection<'_> {
        let members = (self.0.data as *const MessageMembers).as_ref().unwrap();
        let namespace = members.message_namespace();
        let name = members.message_name();
        let members = members.members();

        let (module, remain) = namespace.split_once("__").unwrap();
        let (prefix, _) = remain.split_once("__").unwrap();

        Introspection {
            module,
            prefix,
            name,
            members,
        }
    }
}

#[repr(transparent)]
pub struct MessageMember(CMessageMember);

impl MessageMember {
    pub fn name(&self) -> &str {
        unsafe { ptr_to_str(self.0.name_) }
    }

    pub fn rust_name(&self) -> Cow<'_, str> {
        rust_mangle(self.name())
    }

    pub fn type_id(&self) -> MemberType {
        MemberType::from_type_id(self.0.type_id_).unwrap()
    }

    pub fn string_upper_bound(&self) -> Option<usize> {
        if self.type_id() == MemberType::String {
            Some(self.0.string_upper_bound_ as usize)
        } else {
            None
        }
    }

    pub fn members(&self) -> Option<&TypeSupport> {
        if self.type_id() != MemberType::Message {
            return None;
        }

        unsafe {
            let ptr = self.0.members_ as *const TypeSupport;
            let ref_ = ptr.as_ref().unwrap();
            Some(ref_)
        }
    }

    pub fn is_array(&self) -> bool {
        self.0.is_array_
    }

    pub fn array_size(&self) -> Option<usize> {
        if self.is_array() {
            Some(self.0.array_size_)
        } else {
            None
        }
    }

    pub fn is_upper_bound(&self) -> Option<bool> {
        if self.is_array() {
            Some(self.0.is_upper_bound_)
        } else {
            None
        }
    }

    pub fn array_info(&self) -> Option<ArrayInfo> {
        self.0.is_array_.then(|| ArrayInfo {
            size: self.0.array_size_,
            is_upper_bound: self.0.is_upper_bound_,
        })
    }

    pub fn offset(&self) -> usize {
        self.0.offset_ as usize
    }
}

#[repr(transparent)]
pub struct MessageMembers(CMessageMembers);

impl MessageMembers {
    pub fn message_namespace(&self) -> &str {
        unsafe { ptr_to_str(self.0.message_namespace_) }
    }

    pub fn message_name(&self) -> &str {
        unsafe { ptr_to_str(self.0.message_name_) }
    }

    pub fn member_count(&self) -> usize {
        self.0.member_count_ as usize
    }

    pub fn size_of(&self) -> usize {
        self.0.size_of_ as usize
    }

    pub fn members(&self) -> &[MessageMember] {
        unsafe {
            let members: &[CMessageMember] =
                slice::from_raw_parts(self.0.members_, self.member_count());
            mem::transmute(members)
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemberType {
    Bool,
    I8,
    I16,
    I32,
    I64,
    U8,
    U16,
    U32,
    U64,
    U128,
    F32,
    F64,
    Char,
    WChar,
    String,
    WString,
    Message,
}

impl MemberType {
    /// Get the enum variant for the type id.
    ///
    /// Because the c enum has been named between galactic and the
    /// next release, we cannot know its name. therefor we use the
    /// constants as is and hope we notice when they change.
    ///
    /// ```ignore
    /// rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT = 1,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE = 2,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_LONG_DOUBLE = 3,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_CHAR = 4,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_WCHAR = 5,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN = 6,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_OCTET = 7,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_UINT8 = 8,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_INT8 = 9,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_UINT16 = 10,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_INT16 = 11,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_UINT32 = 12,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_INT32 = 13,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_UINT64 = 14,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_INT64 = 15,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_STRING = 16,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_WSTRING = 17,
    /// rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE = 18,
    /// ```
    pub fn from_type_id(id: u8) -> Option<Self> {
        Some(match id {
            1 => Self::F32,
            2 => Self::F64,
            3 => Self::U128,
            4 => Self::Char,
            5 => Self::WChar,
            6 => Self::Bool,
            7 | 8 => Self::U8,
            9 => Self::I8,
            10 => Self::U16,
            11 => Self::I16,
            12 => Self::U32,
            13 => Self::I32,
            14 => Self::U64,
            15 => Self::I64,
            16 => Self::String,
            17 => Self::WString,
            18 => Self::Message,
            _ => return None,
        })
    }

    pub fn to_rust_type(&self) -> proc_macro2::TokenStream {
        match self {
            MemberType::Bool => quote! { bool },
            MemberType::I8 => quote! { i8 },
            MemberType::I16 => quote! { i16 },
            MemberType::I32 => quote! { i32 },
            MemberType::I64 => quote! { i64 },
            MemberType::U8 => quote! { u8 },
            MemberType::U16 => quote! { u16 },
            MemberType::U32 => quote! { u32 },
            MemberType::U64 => quote! { u64 },
            MemberType::U128 => quote! { u128 },
            MemberType::F32 => quote! { f32 },
            MemberType::F64 => quote! { f64 },
            MemberType::Char => quote! { std::ffi::c_char },
            MemberType::WChar => quote! { widestring::WideChar },
            MemberType::String => quote! { std::string::String },
            MemberType::WString => quote! { std::string::String },
            MemberType::Message => quote! { message },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ArrayInfo {
    pub size: usize,
    pub is_upper_bound: bool,
}

unsafe fn ptr_to_str(ptr: *const c_char) -> &'static str {
    CStr::from_ptr(ptr).to_str().unwrap()
}
