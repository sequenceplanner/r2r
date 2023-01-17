  pub mod msg {
    use super::super::*;

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Constants {

                              
                          }

                          impl WrappedTypesupport for Constants { 

            type CStruct = bond__msg__Constants; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__bond__msg__Constants() }
            }

            fn create_msg() -> *mut bond__msg__Constants {

                unsafe { bond__msg__Constants__create() }

            }

            fn destroy_msg(msg: *mut bond__msg__Constants) -> () {

                unsafe { bond__msg__Constants__destroy(msg) };

            }

            fn from_native(_msg: &Self::CStruct) -> Constants {
  Constants {
      }
    }



            fn copy_to_native(&self, _msg: &mut Self::CStruct) {}



        }


                          
                          impl Default for Constants {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Constants>::new();
                                  Constants::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Status {

                              pub header: std_msgs::msg::Header,
pub id: std::string::String,
pub instance_id: std::string::String,
pub active: bool,
pub heartbeat_timeout: f32,
pub heartbeat_period: f32,

                          }

                          impl WrappedTypesupport for Status { 

            type CStruct = bond__msg__Status; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__bond__msg__Status() }
            }

            fn create_msg() -> *mut bond__msg__Status {

                unsafe { bond__msg__Status__create() }

            }

            fn destroy_msg(msg: *mut bond__msg__Status) -> () {

                unsafe { bond__msg__Status__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Status {
  Status {
header: std_msgs::msg::Header::from_native(&msg.header),
id: msg.id.to_str().to_owned(),
instance_id: msg.instance_id.to_str().to_owned(),
active: msg.active,
heartbeat_timeout: msg.heartbeat_timeout,
heartbeat_period: msg.heartbeat_period,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
msg.id.assign(&self.id);
msg.instance_id.assign(&self.instance_id);
msg.active = self.active;
msg.heartbeat_timeout = self.heartbeat_timeout;
msg.heartbeat_period = self.heartbeat_period;
}



        }


                          
                          impl Default for Status {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Status>::new();
                                  Status::from_native(&msg_native)
                              }
                          }
             


                      }
