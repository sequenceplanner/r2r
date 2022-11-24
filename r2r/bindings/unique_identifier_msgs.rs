  pub mod msg {
    use super::super::*;

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct UUID {

                              pub uuid: Vec<u8>,

                          }

                          impl WrappedTypesupport for UUID { 

            type CStruct = unique_identifier_msgs__msg__UUID; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__unique_identifier_msgs__msg__UUID() }
            }

            fn create_msg() -> *mut unique_identifier_msgs__msg__UUID {

                unsafe { unique_identifier_msgs__msg__UUID__create() }

            }

            fn destroy_msg(msg: *mut unique_identifier_msgs__msg__UUID) -> () {

                unsafe { unique_identifier_msgs__msg__UUID__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> UUID {
  UUID {
// is_upper_bound_: false
// member.array_size_ : 16
uuid: msg.uuid.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {assert_eq!(self.uuid.len(), 16, "Field {} is fixed size of {}!", "uuid", 16);
msg.uuid.copy_from_slice(&self.uuid[..16]);
}



        }


                          
                          impl Default for UUID {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<UUID>::new();
                                  UUID::from_native(&msg_native)
                              }
                          }
             


                      }
