  pub mod srv {
#[allow(non_snake_case)]
    pub mod GetInteractiveMarkers {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__visualization_msgs__srv__GetInteractiveMarkers()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              
                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = visualization_msgs__srv__GetInteractiveMarkers_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__srv__GetInteractiveMarkers_Request() }
            }

            fn create_msg() -> *mut visualization_msgs__srv__GetInteractiveMarkers_Request {

                unsafe { visualization_msgs__srv__GetInteractiveMarkers_Request__create() }

            }

            fn destroy_msg(msg: *mut visualization_msgs__srv__GetInteractiveMarkers_Request) -> () {

                unsafe { visualization_msgs__srv__GetInteractiveMarkers_Request__destroy(msg) };

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

                              pub sequence_number: u64,
pub markers: Vec<visualization_msgs::msg::InteractiveMarker>,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = visualization_msgs__srv__GetInteractiveMarkers_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__srv__GetInteractiveMarkers_Response() }
            }

            fn create_msg() -> *mut visualization_msgs__srv__GetInteractiveMarkers_Response {

                unsafe { visualization_msgs__srv__GetInteractiveMarkers_Response__create() }

            }

            fn destroy_msg(msg: *mut visualization_msgs__srv__GetInteractiveMarkers_Response) -> () {

                unsafe { visualization_msgs__srv__GetInteractiveMarkers_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
sequence_number: msg.sequence_number,
// is_upper_bound_: false
// member.array_size_ : 0
markers : {
let mut temp = Vec::with_capacity(msg.markers.size);
let slice = unsafe { std::slice::from_raw_parts(msg.markers.data, msg.markers.size)};
for s in slice { temp.push(visualization_msgs::msg::InteractiveMarker::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.sequence_number = self.sequence_number;
unsafe { visualization_msgs__msg__InteractiveMarker__Sequence__fini(&mut msg.markers) };
unsafe { visualization_msgs__msg__InteractiveMarker__Sequence__init(&mut msg.markers, self.markers.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.markers.data, msg.markers.size)};
for (t,s) in slice.iter_mut().zip(&self.markers) { s.copy_to_native(t);}
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
  pub mod msg {
    use super::super::*;

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct ImageMarker {

                              pub header: std_msgs::msg::Header,
pub ns: std::string::String,
pub id: i32,
pub type_: i32,
pub action: i32,
pub position: geometry_msgs::msg::Point,
pub scale: f32,
pub outline_color: std_msgs::msg::ColorRGBA,
pub filled: u8,
pub fill_color: std_msgs::msg::ColorRGBA,
pub lifetime: builtin_interfaces::msg::Duration,
pub points: Vec<geometry_msgs::msg::Point>,
pub outline_colors: Vec<std_msgs::msg::ColorRGBA>,

                          }

                          impl WrappedTypesupport for ImageMarker { 

            type CStruct = visualization_msgs__msg__ImageMarker; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__ImageMarker() }
            }

            fn create_msg() -> *mut visualization_msgs__msg__ImageMarker {

                unsafe { visualization_msgs__msg__ImageMarker__create() }

            }

            fn destroy_msg(msg: *mut visualization_msgs__msg__ImageMarker) -> () {

                unsafe { visualization_msgs__msg__ImageMarker__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> ImageMarker {
  ImageMarker {
header: std_msgs::msg::Header::from_native(&msg.header),
ns: msg.ns.to_str().to_owned(),
id: msg.id,
type_: msg.type_,
action: msg.action,
position: geometry_msgs::msg::Point::from_native(&msg.position),
scale: msg.scale,
outline_color: std_msgs::msg::ColorRGBA::from_native(&msg.outline_color),
filled: msg.filled,
fill_color: std_msgs::msg::ColorRGBA::from_native(&msg.fill_color),
lifetime: builtin_interfaces::msg::Duration::from_native(&msg.lifetime),
// is_upper_bound_: false
// member.array_size_ : 0
points : {
let mut temp = Vec::with_capacity(msg.points.size);
let slice = unsafe { std::slice::from_raw_parts(msg.points.data, msg.points.size)};
for s in slice { temp.push(geometry_msgs::msg::Point::from_native(s)); }
temp },
// is_upper_bound_: false
// member.array_size_ : 0
outline_colors : {
let mut temp = Vec::with_capacity(msg.outline_colors.size);
let slice = unsafe { std::slice::from_raw_parts(msg.outline_colors.data, msg.outline_colors.size)};
for s in slice { temp.push(std_msgs::msg::ColorRGBA::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
msg.ns.assign(&self.ns);
msg.id = self.id;
msg.type_ = self.type_;
msg.action = self.action;
self.position.copy_to_native(&mut msg.position);
msg.scale = self.scale;
self.outline_color.copy_to_native(&mut msg.outline_color);
msg.filled = self.filled;
self.fill_color.copy_to_native(&mut msg.fill_color);
self.lifetime.copy_to_native(&mut msg.lifetime);
unsafe { geometry_msgs__msg__Point__Sequence__fini(&mut msg.points) };
unsafe { geometry_msgs__msg__Point__Sequence__init(&mut msg.points, self.points.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.points.data, msg.points.size)};
for (t,s) in slice.iter_mut().zip(&self.points) { s.copy_to_native(t);}
unsafe { std_msgs__msg__ColorRGBA__Sequence__fini(&mut msg.outline_colors) };
unsafe { std_msgs__msg__ColorRGBA__Sequence__init(&mut msg.outline_colors, self.outline_colors.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.outline_colors.data, msg.outline_colors.size)};
for (t,s) in slice.iter_mut().zip(&self.outline_colors) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for ImageMarker {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<ImageMarker>::new();
                                  ImageMarker::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct InteractiveMarker {

                              pub header: std_msgs::msg::Header,
pub pose: geometry_msgs::msg::Pose,
pub name: std::string::String,
pub description: std::string::String,
pub scale: f32,
pub menu_entries: Vec<visualization_msgs::msg::MenuEntry>,
pub controls: Vec<visualization_msgs::msg::InteractiveMarkerControl>,

                          }

                          impl WrappedTypesupport for InteractiveMarker { 

            type CStruct = visualization_msgs__msg__InteractiveMarker; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__InteractiveMarker() }
            }

            fn create_msg() -> *mut visualization_msgs__msg__InteractiveMarker {

                unsafe { visualization_msgs__msg__InteractiveMarker__create() }

            }

            fn destroy_msg(msg: *mut visualization_msgs__msg__InteractiveMarker) -> () {

                unsafe { visualization_msgs__msg__InteractiveMarker__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> InteractiveMarker {
  InteractiveMarker {
header: std_msgs::msg::Header::from_native(&msg.header),
pose: geometry_msgs::msg::Pose::from_native(&msg.pose),
name: msg.name.to_str().to_owned(),
description: msg.description.to_str().to_owned(),
scale: msg.scale,
// is_upper_bound_: false
// member.array_size_ : 0
menu_entries : {
let mut temp = Vec::with_capacity(msg.menu_entries.size);
let slice = unsafe { std::slice::from_raw_parts(msg.menu_entries.data, msg.menu_entries.size)};
for s in slice { temp.push(visualization_msgs::msg::MenuEntry::from_native(s)); }
temp },
// is_upper_bound_: false
// member.array_size_ : 0
controls : {
let mut temp = Vec::with_capacity(msg.controls.size);
let slice = unsafe { std::slice::from_raw_parts(msg.controls.data, msg.controls.size)};
for s in slice { temp.push(visualization_msgs::msg::InteractiveMarkerControl::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
self.pose.copy_to_native(&mut msg.pose);
msg.name.assign(&self.name);
msg.description.assign(&self.description);
msg.scale = self.scale;
unsafe { visualization_msgs__msg__MenuEntry__Sequence__fini(&mut msg.menu_entries) };
unsafe { visualization_msgs__msg__MenuEntry__Sequence__init(&mut msg.menu_entries, self.menu_entries.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.menu_entries.data, msg.menu_entries.size)};
for (t,s) in slice.iter_mut().zip(&self.menu_entries) { s.copy_to_native(t);}
unsafe { visualization_msgs__msg__InteractiveMarkerControl__Sequence__fini(&mut msg.controls) };
unsafe { visualization_msgs__msg__InteractiveMarkerControl__Sequence__init(&mut msg.controls, self.controls.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.controls.data, msg.controls.size)};
for (t,s) in slice.iter_mut().zip(&self.controls) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for InteractiveMarker {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<InteractiveMarker>::new();
                                  InteractiveMarker::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct InteractiveMarkerControl {

                              pub name: std::string::String,
pub orientation: geometry_msgs::msg::Quaternion,
pub orientation_mode: u8,
pub interaction_mode: u8,
pub always_visible: bool,
pub markers: Vec<visualization_msgs::msg::Marker>,
pub independent_marker_orientation: bool,
pub description: std::string::String,

                          }

                          impl WrappedTypesupport for InteractiveMarkerControl { 

            type CStruct = visualization_msgs__msg__InteractiveMarkerControl; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__InteractiveMarkerControl() }
            }

            fn create_msg() -> *mut visualization_msgs__msg__InteractiveMarkerControl {

                unsafe { visualization_msgs__msg__InteractiveMarkerControl__create() }

            }

            fn destroy_msg(msg: *mut visualization_msgs__msg__InteractiveMarkerControl) -> () {

                unsafe { visualization_msgs__msg__InteractiveMarkerControl__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> InteractiveMarkerControl {
  InteractiveMarkerControl {
name: msg.name.to_str().to_owned(),
orientation: geometry_msgs::msg::Quaternion::from_native(&msg.orientation),
orientation_mode: msg.orientation_mode,
interaction_mode: msg.interaction_mode,
always_visible: msg.always_visible,
// is_upper_bound_: false
// member.array_size_ : 0
markers : {
let mut temp = Vec::with_capacity(msg.markers.size);
let slice = unsafe { std::slice::from_raw_parts(msg.markers.data, msg.markers.size)};
for s in slice { temp.push(visualization_msgs::msg::Marker::from_native(s)); }
temp },
independent_marker_orientation: msg.independent_marker_orientation,
description: msg.description.to_str().to_owned(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.name.assign(&self.name);
self.orientation.copy_to_native(&mut msg.orientation);
msg.orientation_mode = self.orientation_mode;
msg.interaction_mode = self.interaction_mode;
msg.always_visible = self.always_visible;
unsafe { visualization_msgs__msg__Marker__Sequence__fini(&mut msg.markers) };
unsafe { visualization_msgs__msg__Marker__Sequence__init(&mut msg.markers, self.markers.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.markers.data, msg.markers.size)};
for (t,s) in slice.iter_mut().zip(&self.markers) { s.copy_to_native(t);}
msg.independent_marker_orientation = self.independent_marker_orientation;
msg.description.assign(&self.description);
}



        }


                          
                          impl Default for InteractiveMarkerControl {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<InteractiveMarkerControl>::new();
                                  InteractiveMarkerControl::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct InteractiveMarkerFeedback {

                              pub header: std_msgs::msg::Header,
pub client_id: std::string::String,
pub marker_name: std::string::String,
pub control_name: std::string::String,
pub event_type: u8,
pub pose: geometry_msgs::msg::Pose,
pub menu_entry_id: u32,
pub mouse_point: geometry_msgs::msg::Point,
pub mouse_point_valid: bool,

                          }

                          impl WrappedTypesupport for InteractiveMarkerFeedback { 

            type CStruct = visualization_msgs__msg__InteractiveMarkerFeedback; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__InteractiveMarkerFeedback() }
            }

            fn create_msg() -> *mut visualization_msgs__msg__InteractiveMarkerFeedback {

                unsafe { visualization_msgs__msg__InteractiveMarkerFeedback__create() }

            }

            fn destroy_msg(msg: *mut visualization_msgs__msg__InteractiveMarkerFeedback) -> () {

                unsafe { visualization_msgs__msg__InteractiveMarkerFeedback__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> InteractiveMarkerFeedback {
  InteractiveMarkerFeedback {
header: std_msgs::msg::Header::from_native(&msg.header),
client_id: msg.client_id.to_str().to_owned(),
marker_name: msg.marker_name.to_str().to_owned(),
control_name: msg.control_name.to_str().to_owned(),
event_type: msg.event_type,
pose: geometry_msgs::msg::Pose::from_native(&msg.pose),
menu_entry_id: msg.menu_entry_id,
mouse_point: geometry_msgs::msg::Point::from_native(&msg.mouse_point),
mouse_point_valid: msg.mouse_point_valid,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
msg.client_id.assign(&self.client_id);
msg.marker_name.assign(&self.marker_name);
msg.control_name.assign(&self.control_name);
msg.event_type = self.event_type;
self.pose.copy_to_native(&mut msg.pose);
msg.menu_entry_id = self.menu_entry_id;
self.mouse_point.copy_to_native(&mut msg.mouse_point);
msg.mouse_point_valid = self.mouse_point_valid;
}



        }


                          
                          impl Default for InteractiveMarkerFeedback {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<InteractiveMarkerFeedback>::new();
                                  InteractiveMarkerFeedback::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct InteractiveMarkerInit {

                              pub server_id: std::string::String,
pub seq_num: u64,
pub markers: Vec<visualization_msgs::msg::InteractiveMarker>,

                          }

                          impl WrappedTypesupport for InteractiveMarkerInit { 

            type CStruct = visualization_msgs__msg__InteractiveMarkerInit; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__InteractiveMarkerInit() }
            }

            fn create_msg() -> *mut visualization_msgs__msg__InteractiveMarkerInit {

                unsafe { visualization_msgs__msg__InteractiveMarkerInit__create() }

            }

            fn destroy_msg(msg: *mut visualization_msgs__msg__InteractiveMarkerInit) -> () {

                unsafe { visualization_msgs__msg__InteractiveMarkerInit__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> InteractiveMarkerInit {
  InteractiveMarkerInit {
server_id: msg.server_id.to_str().to_owned(),
seq_num: msg.seq_num,
// is_upper_bound_: false
// member.array_size_ : 0
markers : {
let mut temp = Vec::with_capacity(msg.markers.size);
let slice = unsafe { std::slice::from_raw_parts(msg.markers.data, msg.markers.size)};
for s in slice { temp.push(visualization_msgs::msg::InteractiveMarker::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.server_id.assign(&self.server_id);
msg.seq_num = self.seq_num;
unsafe { visualization_msgs__msg__InteractiveMarker__Sequence__fini(&mut msg.markers) };
unsafe { visualization_msgs__msg__InteractiveMarker__Sequence__init(&mut msg.markers, self.markers.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.markers.data, msg.markers.size)};
for (t,s) in slice.iter_mut().zip(&self.markers) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for InteractiveMarkerInit {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<InteractiveMarkerInit>::new();
                                  InteractiveMarkerInit::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct InteractiveMarkerPose {

                              pub header: std_msgs::msg::Header,
pub pose: geometry_msgs::msg::Pose,
pub name: std::string::String,

                          }

                          impl WrappedTypesupport for InteractiveMarkerPose { 

            type CStruct = visualization_msgs__msg__InteractiveMarkerPose; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__InteractiveMarkerPose() }
            }

            fn create_msg() -> *mut visualization_msgs__msg__InteractiveMarkerPose {

                unsafe { visualization_msgs__msg__InteractiveMarkerPose__create() }

            }

            fn destroy_msg(msg: *mut visualization_msgs__msg__InteractiveMarkerPose) -> () {

                unsafe { visualization_msgs__msg__InteractiveMarkerPose__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> InteractiveMarkerPose {
  InteractiveMarkerPose {
header: std_msgs::msg::Header::from_native(&msg.header),
pose: geometry_msgs::msg::Pose::from_native(&msg.pose),
name: msg.name.to_str().to_owned(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
self.pose.copy_to_native(&mut msg.pose);
msg.name.assign(&self.name);
}



        }


                          
                          impl Default for InteractiveMarkerPose {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<InteractiveMarkerPose>::new();
                                  InteractiveMarkerPose::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct InteractiveMarkerUpdate {

                              pub server_id: std::string::String,
pub seq_num: u64,
pub type_: u8,
pub markers: Vec<visualization_msgs::msg::InteractiveMarker>,
pub poses: Vec<visualization_msgs::msg::InteractiveMarkerPose>,
pub erases: Vec<std::string::String>,

                          }

                          impl WrappedTypesupport for InteractiveMarkerUpdate { 

            type CStruct = visualization_msgs__msg__InteractiveMarkerUpdate; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__InteractiveMarkerUpdate() }
            }

            fn create_msg() -> *mut visualization_msgs__msg__InteractiveMarkerUpdate {

                unsafe { visualization_msgs__msg__InteractiveMarkerUpdate__create() }

            }

            fn destroy_msg(msg: *mut visualization_msgs__msg__InteractiveMarkerUpdate) -> () {

                unsafe { visualization_msgs__msg__InteractiveMarkerUpdate__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> InteractiveMarkerUpdate {
  InteractiveMarkerUpdate {
server_id: msg.server_id.to_str().to_owned(),
seq_num: msg.seq_num,
type_: msg.type_,
// is_upper_bound_: false
// member.array_size_ : 0
markers : {
let mut temp = Vec::with_capacity(msg.markers.size);
let slice = unsafe { std::slice::from_raw_parts(msg.markers.data, msg.markers.size)};
for s in slice { temp.push(visualization_msgs::msg::InteractiveMarker::from_native(s)); }
temp },
// is_upper_bound_: false
// member.array_size_ : 0
poses : {
let mut temp = Vec::with_capacity(msg.poses.size);
let slice = unsafe { std::slice::from_raw_parts(msg.poses.data, msg.poses.size)};
for s in slice { temp.push(visualization_msgs::msg::InteractiveMarkerPose::from_native(s)); }
temp },
// is_upper_bound_: false
// member.array_size_ : 0
erases: msg.erases.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.server_id.assign(&self.server_id);
msg.seq_num = self.seq_num;
msg.type_ = self.type_;
unsafe { visualization_msgs__msg__InteractiveMarker__Sequence__fini(&mut msg.markers) };
unsafe { visualization_msgs__msg__InteractiveMarker__Sequence__init(&mut msg.markers, self.markers.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.markers.data, msg.markers.size)};
for (t,s) in slice.iter_mut().zip(&self.markers) { s.copy_to_native(t);}
unsafe { visualization_msgs__msg__InteractiveMarkerPose__Sequence__fini(&mut msg.poses) };
unsafe { visualization_msgs__msg__InteractiveMarkerPose__Sequence__init(&mut msg.poses, self.poses.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.poses.data, msg.poses.size)};
for (t,s) in slice.iter_mut().zip(&self.poses) { s.copy_to_native(t);}
msg.erases.update(&self.erases);
}



        }


                          
                          impl Default for InteractiveMarkerUpdate {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<InteractiveMarkerUpdate>::new();
                                  InteractiveMarkerUpdate::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Marker {

                              pub header: std_msgs::msg::Header,
pub ns: std::string::String,
pub id: i32,
pub type_: i32,
pub action: i32,
pub pose: geometry_msgs::msg::Pose,
pub scale: geometry_msgs::msg::Vector3,
pub color: std_msgs::msg::ColorRGBA,
pub lifetime: builtin_interfaces::msg::Duration,
pub frame_locked: bool,
pub points: Vec<geometry_msgs::msg::Point>,
pub colors: Vec<std_msgs::msg::ColorRGBA>,
pub text: std::string::String,
pub mesh_resource: std::string::String,
pub mesh_use_embedded_materials: bool,

                          }

                          impl WrappedTypesupport for Marker { 

            type CStruct = visualization_msgs__msg__Marker; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__Marker() }
            }

            fn create_msg() -> *mut visualization_msgs__msg__Marker {

                unsafe { visualization_msgs__msg__Marker__create() }

            }

            fn destroy_msg(msg: *mut visualization_msgs__msg__Marker) -> () {

                unsafe { visualization_msgs__msg__Marker__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Marker {
  Marker {
header: std_msgs::msg::Header::from_native(&msg.header),
ns: msg.ns.to_str().to_owned(),
id: msg.id,
type_: msg.type_,
action: msg.action,
pose: geometry_msgs::msg::Pose::from_native(&msg.pose),
scale: geometry_msgs::msg::Vector3::from_native(&msg.scale),
color: std_msgs::msg::ColorRGBA::from_native(&msg.color),
lifetime: builtin_interfaces::msg::Duration::from_native(&msg.lifetime),
frame_locked: msg.frame_locked,
// is_upper_bound_: false
// member.array_size_ : 0
points : {
let mut temp = Vec::with_capacity(msg.points.size);
let slice = unsafe { std::slice::from_raw_parts(msg.points.data, msg.points.size)};
for s in slice { temp.push(geometry_msgs::msg::Point::from_native(s)); }
temp },
// is_upper_bound_: false
// member.array_size_ : 0
colors : {
let mut temp = Vec::with_capacity(msg.colors.size);
let slice = unsafe { std::slice::from_raw_parts(msg.colors.data, msg.colors.size)};
for s in slice { temp.push(std_msgs::msg::ColorRGBA::from_native(s)); }
temp },
text: msg.text.to_str().to_owned(),
mesh_resource: msg.mesh_resource.to_str().to_owned(),
mesh_use_embedded_materials: msg.mesh_use_embedded_materials,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
msg.ns.assign(&self.ns);
msg.id = self.id;
msg.type_ = self.type_;
msg.action = self.action;
self.pose.copy_to_native(&mut msg.pose);
self.scale.copy_to_native(&mut msg.scale);
self.color.copy_to_native(&mut msg.color);
self.lifetime.copy_to_native(&mut msg.lifetime);
msg.frame_locked = self.frame_locked;
unsafe { geometry_msgs__msg__Point__Sequence__fini(&mut msg.points) };
unsafe { geometry_msgs__msg__Point__Sequence__init(&mut msg.points, self.points.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.points.data, msg.points.size)};
for (t,s) in slice.iter_mut().zip(&self.points) { s.copy_to_native(t);}
unsafe { std_msgs__msg__ColorRGBA__Sequence__fini(&mut msg.colors) };
unsafe { std_msgs__msg__ColorRGBA__Sequence__init(&mut msg.colors, self.colors.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.colors.data, msg.colors.size)};
for (t,s) in slice.iter_mut().zip(&self.colors) { s.copy_to_native(t);}
msg.text.assign(&self.text);
msg.mesh_resource.assign(&self.mesh_resource);
msg.mesh_use_embedded_materials = self.mesh_use_embedded_materials;
}



        }


                          
                          impl Default for Marker {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Marker>::new();
                                  Marker::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct MarkerArray {

                              pub markers: Vec<visualization_msgs::msg::Marker>,

                          }

                          impl WrappedTypesupport for MarkerArray { 

            type CStruct = visualization_msgs__msg__MarkerArray; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__MarkerArray() }
            }

            fn create_msg() -> *mut visualization_msgs__msg__MarkerArray {

                unsafe { visualization_msgs__msg__MarkerArray__create() }

            }

            fn destroy_msg(msg: *mut visualization_msgs__msg__MarkerArray) -> () {

                unsafe { visualization_msgs__msg__MarkerArray__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> MarkerArray {
  MarkerArray {
// is_upper_bound_: false
// member.array_size_ : 0
markers : {
let mut temp = Vec::with_capacity(msg.markers.size);
let slice = unsafe { std::slice::from_raw_parts(msg.markers.data, msg.markers.size)};
for s in slice { temp.push(visualization_msgs::msg::Marker::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {unsafe { visualization_msgs__msg__Marker__Sequence__fini(&mut msg.markers) };
unsafe { visualization_msgs__msg__Marker__Sequence__init(&mut msg.markers, self.markers.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.markers.data, msg.markers.size)};
for (t,s) in slice.iter_mut().zip(&self.markers) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for MarkerArray {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<MarkerArray>::new();
                                  MarkerArray::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct MenuEntry {

                              pub id: u32,
pub parent_id: u32,
pub title: std::string::String,
pub command: std::string::String,
pub command_type: u8,

                          }

                          impl WrappedTypesupport for MenuEntry { 

            type CStruct = visualization_msgs__msg__MenuEntry; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__MenuEntry() }
            }

            fn create_msg() -> *mut visualization_msgs__msg__MenuEntry {

                unsafe { visualization_msgs__msg__MenuEntry__create() }

            }

            fn destroy_msg(msg: *mut visualization_msgs__msg__MenuEntry) -> () {

                unsafe { visualization_msgs__msg__MenuEntry__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> MenuEntry {
  MenuEntry {
id: msg.id,
parent_id: msg.parent_id,
title: msg.title.to_str().to_owned(),
command: msg.command.to_str().to_owned(),
command_type: msg.command_type,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.id = self.id;
msg.parent_id = self.parent_id;
msg.title.assign(&self.title);
msg.command.assign(&self.command);
msg.command_type = self.command_type;
}



        }


                          
                          impl Default for MenuEntry {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<MenuEntry>::new();
                                  MenuEntry::from_native(&msg_native)
                              }
                          }
             


                      }
