  pub mod msg {
    use super::super::*;

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Octomap {

                              pub header: std_msgs::msg::Header,
pub binary: bool,
pub id: std::string::String,
pub resolution: f64,
pub data: Vec<i8>,

                          }

                          impl WrappedTypesupport for Octomap { 

            type CStruct = octomap_msgs__msg__Octomap; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__octomap_msgs__msg__Octomap() }
            }

            fn create_msg() -> *mut octomap_msgs__msg__Octomap {

                unsafe { octomap_msgs__msg__Octomap__create() }

            }

            fn destroy_msg(msg: *mut octomap_msgs__msg__Octomap) -> () {

                unsafe { octomap_msgs__msg__Octomap__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Octomap {
  Octomap {
header: std_msgs::msg::Header::from_native(&msg.header),
binary: msg.binary,
id: msg.id.to_str().to_owned(),
resolution: msg.resolution,
// is_upper_bound_: false
// member.array_size_ : 0
data: msg.data.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
msg.binary = self.binary;
msg.id.assign(&self.id);
msg.resolution = self.resolution;
msg.data.update(&self.data);
}



        }


                          
                          impl Default for Octomap {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Octomap>::new();
                                  Octomap::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct OctomapWithPose {

                              pub header: std_msgs::msg::Header,
pub origin: geometry_msgs::msg::Pose,
pub octomap: octomap_msgs::msg::Octomap,

                          }

                          impl WrappedTypesupport for OctomapWithPose { 

            type CStruct = octomap_msgs__msg__OctomapWithPose; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__octomap_msgs__msg__OctomapWithPose() }
            }

            fn create_msg() -> *mut octomap_msgs__msg__OctomapWithPose {

                unsafe { octomap_msgs__msg__OctomapWithPose__create() }

            }

            fn destroy_msg(msg: *mut octomap_msgs__msg__OctomapWithPose) -> () {

                unsafe { octomap_msgs__msg__OctomapWithPose__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> OctomapWithPose {
  OctomapWithPose {
header: std_msgs::msg::Header::from_native(&msg.header),
origin: geometry_msgs::msg::Pose::from_native(&msg.origin),
octomap: octomap_msgs::msg::Octomap::from_native(&msg.octomap),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
self.origin.copy_to_native(&mut msg.origin);
self.octomap.copy_to_native(&mut msg.octomap);
}



        }


                          
                          impl Default for OctomapWithPose {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<OctomapWithPose>::new();
                                  OctomapWithPose::from_native(&msg_native)
                              }
                          }
             


                      }
  pub mod srv {
#[allow(non_snake_case)]
    pub mod BoundingBoxQuery {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__octomap_msgs__srv__BoundingBoxQuery()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub min: geometry_msgs::msg::Point,
pub max: geometry_msgs::msg::Point,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = octomap_msgs__srv__BoundingBoxQuery_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__octomap_msgs__srv__BoundingBoxQuery_Request() }
            }

            fn create_msg() -> *mut octomap_msgs__srv__BoundingBoxQuery_Request {

                unsafe { octomap_msgs__srv__BoundingBoxQuery_Request__create() }

            }

            fn destroy_msg(msg: *mut octomap_msgs__srv__BoundingBoxQuery_Request) -> () {

                unsafe { octomap_msgs__srv__BoundingBoxQuery_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
min: geometry_msgs::msg::Point::from_native(&msg.min),
max: geometry_msgs::msg::Point::from_native(&msg.max),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.min.copy_to_native(&mut msg.min);
self.max.copy_to_native(&mut msg.max);
}



        }


                          
                          impl Default for Request {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Request>::new();
                                  Request::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Response {

                              
                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = octomap_msgs__srv__BoundingBoxQuery_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__octomap_msgs__srv__BoundingBoxQuery_Response() }
            }

            fn create_msg() -> *mut octomap_msgs__srv__BoundingBoxQuery_Response {

                unsafe { octomap_msgs__srv__BoundingBoxQuery_Response__create() }

            }

            fn destroy_msg(msg: *mut octomap_msgs__srv__BoundingBoxQuery_Response) -> () {

                unsafe { octomap_msgs__srv__BoundingBoxQuery_Response__destroy(msg) };

            }

            fn from_native(_msg: &Self::CStruct) -> Response {
  Response {
      }
    }



            fn copy_to_native(&self, _msg: &mut Self::CStruct) {}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }
#[allow(non_snake_case)]
    pub mod GetOctomap {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__octomap_msgs__srv__GetOctomap()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              
                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = octomap_msgs__srv__GetOctomap_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__octomap_msgs__srv__GetOctomap_Request() }
            }

            fn create_msg() -> *mut octomap_msgs__srv__GetOctomap_Request {

                unsafe { octomap_msgs__srv__GetOctomap_Request__create() }

            }

            fn destroy_msg(msg: *mut octomap_msgs__srv__GetOctomap_Request) -> () {

                unsafe { octomap_msgs__srv__GetOctomap_Request__destroy(msg) };

            }

            fn from_native(_msg: &Self::CStruct) -> Request {
  Request {
      }
    }



            fn copy_to_native(&self, _msg: &mut Self::CStruct) {}



        }


                          
                          impl Default for Request {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Request>::new();
                                  Request::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Response {

                              pub map: octomap_msgs::msg::Octomap,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = octomap_msgs__srv__GetOctomap_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__octomap_msgs__srv__GetOctomap_Response() }
            }

            fn create_msg() -> *mut octomap_msgs__srv__GetOctomap_Response {

                unsafe { octomap_msgs__srv__GetOctomap_Response__create() }

            }

            fn destroy_msg(msg: *mut octomap_msgs__srv__GetOctomap_Response) -> () {

                unsafe { octomap_msgs__srv__GetOctomap_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
map: octomap_msgs::msg::Octomap::from_native(&msg.map),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.map.copy_to_native(&mut msg.map);
}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }
  }
