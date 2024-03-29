pub mod msg {
    use super::super::*;
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct UUID {
        pub uuid: Vec<u8>,
    }
    impl WrappedTypesupport for UUID {
        type CStruct = unique_identifier_msgs__msg__UUID;
        fn get_ts() -> &'static rosidl_message_type_support_t {
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__unique_identifier_msgs__msg__UUID()
            }
        }
        fn create_msg() -> *mut unique_identifier_msgs__msg__UUID {
            #[cfg(not(feature = "doc-only"))]
            unsafe { unique_identifier_msgs__msg__UUID__create() }
            #[cfg(feature = "doc-only")] unique_identifier_msgs__msg__UUID__create()
        }
        fn destroy_msg(msg: *mut unique_identifier_msgs__msg__UUID) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { unique_identifier_msgs__msg__UUID__destroy(msg) };
            #[cfg(feature = "doc-only")] unique_identifier_msgs__msg__UUID__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> UUID {
            UUID { uuid: msg.uuid.to_vec() }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            assert_eq!(
                self.uuid.len(), 16usize, "Field {} is fixed size of {}!", "uuid",
                16usize
            );
            msg.uuid.copy_from_slice(&self.uuid[..16usize]);
        }
    }
    impl Default for UUID {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<UUID>::new();
            UUID::from_native(&msg_native)
        }
    }
}
