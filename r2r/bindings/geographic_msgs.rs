  pub mod srv {
#[allow(non_snake_case)]
    pub mod GetGeoPath {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetGeoPath()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub start: geographic_msgs::msg::GeoPoint,
pub goal: geographic_msgs::msg::GeoPoint,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = geographic_msgs__srv__GetGeoPath_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetGeoPath_Request() }
            }

            fn create_msg() -> *mut geographic_msgs__srv__GetGeoPath_Request {

                unsafe { geographic_msgs__srv__GetGeoPath_Request__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__srv__GetGeoPath_Request) -> () {

                unsafe { geographic_msgs__srv__GetGeoPath_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
start: geographic_msgs::msg::GeoPoint::from_native(&msg.start),
goal: geographic_msgs::msg::GeoPoint::from_native(&msg.goal),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.start.copy_to_native(&mut msg.start);
self.goal.copy_to_native(&mut msg.goal);
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

                              pub success: bool,
pub status: std::string::String,
pub plan: geographic_msgs::msg::GeoPath,
pub network: unique_identifier_msgs::msg::UUID,
pub start_seg: unique_identifier_msgs::msg::UUID,
pub goal_seg: unique_identifier_msgs::msg::UUID,
pub distance: f64,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = geographic_msgs__srv__GetGeoPath_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetGeoPath_Response() }
            }

            fn create_msg() -> *mut geographic_msgs__srv__GetGeoPath_Response {

                unsafe { geographic_msgs__srv__GetGeoPath_Response__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__srv__GetGeoPath_Response) -> () {

                unsafe { geographic_msgs__srv__GetGeoPath_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
success: msg.success,
status: msg.status.to_str().to_owned(),
plan: geographic_msgs::msg::GeoPath::from_native(&msg.plan),
network: unique_identifier_msgs::msg::UUID::from_native(&msg.network),
start_seg: unique_identifier_msgs::msg::UUID::from_native(&msg.start_seg),
goal_seg: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_seg),
distance: msg.distance,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.success = self.success;
msg.status.assign(&self.status);
self.plan.copy_to_native(&mut msg.plan);
self.network.copy_to_native(&mut msg.network);
self.start_seg.copy_to_native(&mut msg.start_seg);
self.goal_seg.copy_to_native(&mut msg.goal_seg);
msg.distance = self.distance;
}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }
#[allow(non_snake_case)]
    pub mod GetGeographicMap {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetGeographicMap()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub url: std::string::String,
pub bounds: geographic_msgs::msg::BoundingBox,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = geographic_msgs__srv__GetGeographicMap_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetGeographicMap_Request() }
            }

            fn create_msg() -> *mut geographic_msgs__srv__GetGeographicMap_Request {

                unsafe { geographic_msgs__srv__GetGeographicMap_Request__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__srv__GetGeographicMap_Request) -> () {

                unsafe { geographic_msgs__srv__GetGeographicMap_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
url: msg.url.to_str().to_owned(),
bounds: geographic_msgs::msg::BoundingBox::from_native(&msg.bounds),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.url.assign(&self.url);
self.bounds.copy_to_native(&mut msg.bounds);
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

                              pub success: bool,
pub status: std::string::String,
pub map: geographic_msgs::msg::GeographicMap,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = geographic_msgs__srv__GetGeographicMap_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetGeographicMap_Response() }
            }

            fn create_msg() -> *mut geographic_msgs__srv__GetGeographicMap_Response {

                unsafe { geographic_msgs__srv__GetGeographicMap_Response__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__srv__GetGeographicMap_Response) -> () {

                unsafe { geographic_msgs__srv__GetGeographicMap_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
success: msg.success,
status: msg.status.to_str().to_owned(),
map: geographic_msgs::msg::GeographicMap::from_native(&msg.map),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.success = self.success;
msg.status.assign(&self.status);
self.map.copy_to_native(&mut msg.map);
}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }
#[allow(non_snake_case)]
    pub mod GetRoutePlan {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__GetRoutePlan()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub network: unique_identifier_msgs::msg::UUID,
pub start: unique_identifier_msgs::msg::UUID,
pub goal: unique_identifier_msgs::msg::UUID,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = geographic_msgs__srv__GetRoutePlan_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetRoutePlan_Request() }
            }

            fn create_msg() -> *mut geographic_msgs__srv__GetRoutePlan_Request {

                unsafe { geographic_msgs__srv__GetRoutePlan_Request__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__srv__GetRoutePlan_Request) -> () {

                unsafe { geographic_msgs__srv__GetRoutePlan_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
network: unique_identifier_msgs::msg::UUID::from_native(&msg.network),
start: unique_identifier_msgs::msg::UUID::from_native(&msg.start),
goal: unique_identifier_msgs::msg::UUID::from_native(&msg.goal),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.network.copy_to_native(&mut msg.network);
self.start.copy_to_native(&mut msg.start);
self.goal.copy_to_native(&mut msg.goal);
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

                              pub success: bool,
pub status: std::string::String,
pub plan: geographic_msgs::msg::RoutePath,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = geographic_msgs__srv__GetRoutePlan_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__GetRoutePlan_Response() }
            }

            fn create_msg() -> *mut geographic_msgs__srv__GetRoutePlan_Response {

                unsafe { geographic_msgs__srv__GetRoutePlan_Response__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__srv__GetRoutePlan_Response) -> () {

                unsafe { geographic_msgs__srv__GetRoutePlan_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
success: msg.success,
status: msg.status.to_str().to_owned(),
plan: geographic_msgs::msg::RoutePath::from_native(&msg.plan),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.success = self.success;
msg.status.assign(&self.status);
self.plan.copy_to_native(&mut msg.plan);
}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }
#[allow(non_snake_case)]
    pub mod UpdateGeographicMap {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__geographic_msgs__srv__UpdateGeographicMap()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub updates: geographic_msgs::msg::GeographicMapChanges,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = geographic_msgs__srv__UpdateGeographicMap_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__UpdateGeographicMap_Request() }
            }

            fn create_msg() -> *mut geographic_msgs__srv__UpdateGeographicMap_Request {

                unsafe { geographic_msgs__srv__UpdateGeographicMap_Request__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__srv__UpdateGeographicMap_Request) -> () {

                unsafe { geographic_msgs__srv__UpdateGeographicMap_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
updates: geographic_msgs::msg::GeographicMapChanges::from_native(&msg.updates),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.updates.copy_to_native(&mut msg.updates);
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

                              pub success: bool,
pub status: std::string::String,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = geographic_msgs__srv__UpdateGeographicMap_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__srv__UpdateGeographicMap_Response() }
            }

            fn create_msg() -> *mut geographic_msgs__srv__UpdateGeographicMap_Response {

                unsafe { geographic_msgs__srv__UpdateGeographicMap_Response__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__srv__UpdateGeographicMap_Response) -> () {

                unsafe { geographic_msgs__srv__UpdateGeographicMap_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
success: msg.success,
status: msg.status.to_str().to_owned(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.success = self.success;
msg.status.assign(&self.status);
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
                          pub struct BoundingBox {

                              pub min_pt: geographic_msgs::msg::GeoPoint,
pub max_pt: geographic_msgs::msg::GeoPoint,

                          }

                          impl WrappedTypesupport for BoundingBox { 

            type CStruct = geographic_msgs__msg__BoundingBox; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__BoundingBox() }
            }

            fn create_msg() -> *mut geographic_msgs__msg__BoundingBox {

                unsafe { geographic_msgs__msg__BoundingBox__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__msg__BoundingBox) -> () {

                unsafe { geographic_msgs__msg__BoundingBox__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> BoundingBox {
  BoundingBox {
min_pt: geographic_msgs::msg::GeoPoint::from_native(&msg.min_pt),
max_pt: geographic_msgs::msg::GeoPoint::from_native(&msg.max_pt),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.min_pt.copy_to_native(&mut msg.min_pt);
self.max_pt.copy_to_native(&mut msg.max_pt);
}



        }


                          
                          impl Default for BoundingBox {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<BoundingBox>::new();
                                  BoundingBox::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct GeoPath {

                              pub header: std_msgs::msg::Header,
pub poses: Vec<geographic_msgs::msg::GeoPoseStamped>,

                          }

                          impl WrappedTypesupport for GeoPath { 

            type CStruct = geographic_msgs__msg__GeoPath; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPath() }
            }

            fn create_msg() -> *mut geographic_msgs__msg__GeoPath {

                unsafe { geographic_msgs__msg__GeoPath__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__msg__GeoPath) -> () {

                unsafe { geographic_msgs__msg__GeoPath__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> GeoPath {
  GeoPath {
header: std_msgs::msg::Header::from_native(&msg.header),
// is_upper_bound_: false
// member.array_size_ : 0
poses : {
let mut temp = Vec::with_capacity(msg.poses.size);
let slice = unsafe { std::slice::from_raw_parts(msg.poses.data, msg.poses.size)};
for s in slice { temp.push(geographic_msgs::msg::GeoPoseStamped::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
unsafe { geographic_msgs__msg__GeoPoseStamped__Sequence__fini(&mut msg.poses) };
unsafe { geographic_msgs__msg__GeoPoseStamped__Sequence__init(&mut msg.poses, self.poses.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.poses.data, msg.poses.size)};
for (t,s) in slice.iter_mut().zip(&self.poses) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for GeoPath {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<GeoPath>::new();
                                  GeoPath::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct GeoPoint {

                              pub latitude: f64,
pub longitude: f64,
pub altitude: f64,

                          }

                          impl WrappedTypesupport for GeoPoint { 

            type CStruct = geographic_msgs__msg__GeoPoint; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPoint() }
            }

            fn create_msg() -> *mut geographic_msgs__msg__GeoPoint {

                unsafe { geographic_msgs__msg__GeoPoint__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__msg__GeoPoint) -> () {

                unsafe { geographic_msgs__msg__GeoPoint__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> GeoPoint {
  GeoPoint {
latitude: msg.latitude,
longitude: msg.longitude,
altitude: msg.altitude,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.latitude = self.latitude;
msg.longitude = self.longitude;
msg.altitude = self.altitude;
}



        }


                          
                          impl Default for GeoPoint {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<GeoPoint>::new();
                                  GeoPoint::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct GeoPointStamped {

                              pub header: std_msgs::msg::Header,
pub position: geographic_msgs::msg::GeoPoint,

                          }

                          impl WrappedTypesupport for GeoPointStamped { 

            type CStruct = geographic_msgs__msg__GeoPointStamped; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPointStamped() }
            }

            fn create_msg() -> *mut geographic_msgs__msg__GeoPointStamped {

                unsafe { geographic_msgs__msg__GeoPointStamped__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__msg__GeoPointStamped) -> () {

                unsafe { geographic_msgs__msg__GeoPointStamped__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> GeoPointStamped {
  GeoPointStamped {
header: std_msgs::msg::Header::from_native(&msg.header),
position: geographic_msgs::msg::GeoPoint::from_native(&msg.position),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
self.position.copy_to_native(&mut msg.position);
}



        }


                          
                          impl Default for GeoPointStamped {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<GeoPointStamped>::new();
                                  GeoPointStamped::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct GeoPose {

                              pub position: geographic_msgs::msg::GeoPoint,
pub orientation: geometry_msgs::msg::Quaternion,

                          }

                          impl WrappedTypesupport for GeoPose { 

            type CStruct = geographic_msgs__msg__GeoPose; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPose() }
            }

            fn create_msg() -> *mut geographic_msgs__msg__GeoPose {

                unsafe { geographic_msgs__msg__GeoPose__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__msg__GeoPose) -> () {

                unsafe { geographic_msgs__msg__GeoPose__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> GeoPose {
  GeoPose {
position: geographic_msgs::msg::GeoPoint::from_native(&msg.position),
orientation: geometry_msgs::msg::Quaternion::from_native(&msg.orientation),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.position.copy_to_native(&mut msg.position);
self.orientation.copy_to_native(&mut msg.orientation);
}



        }


                          
                          impl Default for GeoPose {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<GeoPose>::new();
                                  GeoPose::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct GeoPoseStamped {

                              pub header: std_msgs::msg::Header,
pub pose: geographic_msgs::msg::GeoPose,

                          }

                          impl WrappedTypesupport for GeoPoseStamped { 

            type CStruct = geographic_msgs__msg__GeoPoseStamped; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeoPoseStamped() }
            }

            fn create_msg() -> *mut geographic_msgs__msg__GeoPoseStamped {

                unsafe { geographic_msgs__msg__GeoPoseStamped__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__msg__GeoPoseStamped) -> () {

                unsafe { geographic_msgs__msg__GeoPoseStamped__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> GeoPoseStamped {
  GeoPoseStamped {
header: std_msgs::msg::Header::from_native(&msg.header),
pose: geographic_msgs::msg::GeoPose::from_native(&msg.pose),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
self.pose.copy_to_native(&mut msg.pose);
}



        }


                          
                          impl Default for GeoPoseStamped {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<GeoPoseStamped>::new();
                                  GeoPoseStamped::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct GeographicMap {

                              pub header: std_msgs::msg::Header,
pub id: unique_identifier_msgs::msg::UUID,
pub bounds: geographic_msgs::msg::BoundingBox,
pub points: Vec<geographic_msgs::msg::WayPoint>,
pub features: Vec<geographic_msgs::msg::MapFeature>,
pub props: Vec<geographic_msgs::msg::KeyValue>,

                          }

                          impl WrappedTypesupport for GeographicMap { 

            type CStruct = geographic_msgs__msg__GeographicMap; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeographicMap() }
            }

            fn create_msg() -> *mut geographic_msgs__msg__GeographicMap {

                unsafe { geographic_msgs__msg__GeographicMap__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__msg__GeographicMap) -> () {

                unsafe { geographic_msgs__msg__GeographicMap__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> GeographicMap {
  GeographicMap {
header: std_msgs::msg::Header::from_native(&msg.header),
id: unique_identifier_msgs::msg::UUID::from_native(&msg.id),
bounds: geographic_msgs::msg::BoundingBox::from_native(&msg.bounds),
// is_upper_bound_: false
// member.array_size_ : 0
points : {
let mut temp = Vec::with_capacity(msg.points.size);
let slice = unsafe { std::slice::from_raw_parts(msg.points.data, msg.points.size)};
for s in slice { temp.push(geographic_msgs::msg::WayPoint::from_native(s)); }
temp },
// is_upper_bound_: false
// member.array_size_ : 0
features : {
let mut temp = Vec::with_capacity(msg.features.size);
let slice = unsafe { std::slice::from_raw_parts(msg.features.data, msg.features.size)};
for s in slice { temp.push(geographic_msgs::msg::MapFeature::from_native(s)); }
temp },
// is_upper_bound_: false
// member.array_size_ : 0
props : {
let mut temp = Vec::with_capacity(msg.props.size);
let slice = unsafe { std::slice::from_raw_parts(msg.props.data, msg.props.size)};
for s in slice { temp.push(geographic_msgs::msg::KeyValue::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
self.id.copy_to_native(&mut msg.id);
self.bounds.copy_to_native(&mut msg.bounds);
unsafe { geographic_msgs__msg__WayPoint__Sequence__fini(&mut msg.points) };
unsafe { geographic_msgs__msg__WayPoint__Sequence__init(&mut msg.points, self.points.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.points.data, msg.points.size)};
for (t,s) in slice.iter_mut().zip(&self.points) { s.copy_to_native(t);}
unsafe { geographic_msgs__msg__MapFeature__Sequence__fini(&mut msg.features) };
unsafe { geographic_msgs__msg__MapFeature__Sequence__init(&mut msg.features, self.features.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.features.data, msg.features.size)};
for (t,s) in slice.iter_mut().zip(&self.features) { s.copy_to_native(t);}
unsafe { geographic_msgs__msg__KeyValue__Sequence__fini(&mut msg.props) };
unsafe { geographic_msgs__msg__KeyValue__Sequence__init(&mut msg.props, self.props.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.props.data, msg.props.size)};
for (t,s) in slice.iter_mut().zip(&self.props) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for GeographicMap {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<GeographicMap>::new();
                                  GeographicMap::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct GeographicMapChanges {

                              pub header: std_msgs::msg::Header,
pub diffs: geographic_msgs::msg::GeographicMap,
pub deletes: Vec<unique_identifier_msgs::msg::UUID>,

                          }

                          impl WrappedTypesupport for GeographicMapChanges { 

            type CStruct = geographic_msgs__msg__GeographicMapChanges; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__GeographicMapChanges() }
            }

            fn create_msg() -> *mut geographic_msgs__msg__GeographicMapChanges {

                unsafe { geographic_msgs__msg__GeographicMapChanges__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__msg__GeographicMapChanges) -> () {

                unsafe { geographic_msgs__msg__GeographicMapChanges__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> GeographicMapChanges {
  GeographicMapChanges {
header: std_msgs::msg::Header::from_native(&msg.header),
diffs: geographic_msgs::msg::GeographicMap::from_native(&msg.diffs),
// is_upper_bound_: false
// member.array_size_ : 0
deletes : {
let mut temp = Vec::with_capacity(msg.deletes.size);
let slice = unsafe { std::slice::from_raw_parts(msg.deletes.data, msg.deletes.size)};
for s in slice { temp.push(unique_identifier_msgs::msg::UUID::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
self.diffs.copy_to_native(&mut msg.diffs);
unsafe { unique_identifier_msgs__msg__UUID__Sequence__fini(&mut msg.deletes) };
unsafe { unique_identifier_msgs__msg__UUID__Sequence__init(&mut msg.deletes, self.deletes.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.deletes.data, msg.deletes.size)};
for (t,s) in slice.iter_mut().zip(&self.deletes) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for GeographicMapChanges {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<GeographicMapChanges>::new();
                                  GeographicMapChanges::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct KeyValue {

                              pub key: std::string::String,
pub value: std::string::String,

                          }

                          impl WrappedTypesupport for KeyValue { 

            type CStruct = geographic_msgs__msg__KeyValue; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__KeyValue() }
            }

            fn create_msg() -> *mut geographic_msgs__msg__KeyValue {

                unsafe { geographic_msgs__msg__KeyValue__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__msg__KeyValue) -> () {

                unsafe { geographic_msgs__msg__KeyValue__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> KeyValue {
  KeyValue {
key: msg.key.to_str().to_owned(),
value: msg.value.to_str().to_owned(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.key.assign(&self.key);
msg.value.assign(&self.value);
}



        }


                          
                          impl Default for KeyValue {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<KeyValue>::new();
                                  KeyValue::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct MapFeature {

                              pub id: unique_identifier_msgs::msg::UUID,
pub components: Vec<unique_identifier_msgs::msg::UUID>,
pub props: Vec<geographic_msgs::msg::KeyValue>,

                          }

                          impl WrappedTypesupport for MapFeature { 

            type CStruct = geographic_msgs__msg__MapFeature; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__MapFeature() }
            }

            fn create_msg() -> *mut geographic_msgs__msg__MapFeature {

                unsafe { geographic_msgs__msg__MapFeature__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__msg__MapFeature) -> () {

                unsafe { geographic_msgs__msg__MapFeature__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> MapFeature {
  MapFeature {
id: unique_identifier_msgs::msg::UUID::from_native(&msg.id),
// is_upper_bound_: false
// member.array_size_ : 0
components : {
let mut temp = Vec::with_capacity(msg.components.size);
let slice = unsafe { std::slice::from_raw_parts(msg.components.data, msg.components.size)};
for s in slice { temp.push(unique_identifier_msgs::msg::UUID::from_native(s)); }
temp },
// is_upper_bound_: false
// member.array_size_ : 0
props : {
let mut temp = Vec::with_capacity(msg.props.size);
let slice = unsafe { std::slice::from_raw_parts(msg.props.data, msg.props.size)};
for s in slice { temp.push(geographic_msgs::msg::KeyValue::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.id.copy_to_native(&mut msg.id);
unsafe { unique_identifier_msgs__msg__UUID__Sequence__fini(&mut msg.components) };
unsafe { unique_identifier_msgs__msg__UUID__Sequence__init(&mut msg.components, self.components.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.components.data, msg.components.size)};
for (t,s) in slice.iter_mut().zip(&self.components) { s.copy_to_native(t);}
unsafe { geographic_msgs__msg__KeyValue__Sequence__fini(&mut msg.props) };
unsafe { geographic_msgs__msg__KeyValue__Sequence__init(&mut msg.props, self.props.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.props.data, msg.props.size)};
for (t,s) in slice.iter_mut().zip(&self.props) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for MapFeature {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<MapFeature>::new();
                                  MapFeature::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RouteNetwork {

                              pub header: std_msgs::msg::Header,
pub id: unique_identifier_msgs::msg::UUID,
pub bounds: geographic_msgs::msg::BoundingBox,
pub points: Vec<geographic_msgs::msg::WayPoint>,
pub segments: Vec<geographic_msgs::msg::RouteSegment>,
pub props: Vec<geographic_msgs::msg::KeyValue>,

                          }

                          impl WrappedTypesupport for RouteNetwork { 

            type CStruct = geographic_msgs__msg__RouteNetwork; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RouteNetwork() }
            }

            fn create_msg() -> *mut geographic_msgs__msg__RouteNetwork {

                unsafe { geographic_msgs__msg__RouteNetwork__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__msg__RouteNetwork) -> () {

                unsafe { geographic_msgs__msg__RouteNetwork__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RouteNetwork {
  RouteNetwork {
header: std_msgs::msg::Header::from_native(&msg.header),
id: unique_identifier_msgs::msg::UUID::from_native(&msg.id),
bounds: geographic_msgs::msg::BoundingBox::from_native(&msg.bounds),
// is_upper_bound_: false
// member.array_size_ : 0
points : {
let mut temp = Vec::with_capacity(msg.points.size);
let slice = unsafe { std::slice::from_raw_parts(msg.points.data, msg.points.size)};
for s in slice { temp.push(geographic_msgs::msg::WayPoint::from_native(s)); }
temp },
// is_upper_bound_: false
// member.array_size_ : 0
segments : {
let mut temp = Vec::with_capacity(msg.segments.size);
let slice = unsafe { std::slice::from_raw_parts(msg.segments.data, msg.segments.size)};
for s in slice { temp.push(geographic_msgs::msg::RouteSegment::from_native(s)); }
temp },
// is_upper_bound_: false
// member.array_size_ : 0
props : {
let mut temp = Vec::with_capacity(msg.props.size);
let slice = unsafe { std::slice::from_raw_parts(msg.props.data, msg.props.size)};
for s in slice { temp.push(geographic_msgs::msg::KeyValue::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
self.id.copy_to_native(&mut msg.id);
self.bounds.copy_to_native(&mut msg.bounds);
unsafe { geographic_msgs__msg__WayPoint__Sequence__fini(&mut msg.points) };
unsafe { geographic_msgs__msg__WayPoint__Sequence__init(&mut msg.points, self.points.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.points.data, msg.points.size)};
for (t,s) in slice.iter_mut().zip(&self.points) { s.copy_to_native(t);}
unsafe { geographic_msgs__msg__RouteSegment__Sequence__fini(&mut msg.segments) };
unsafe { geographic_msgs__msg__RouteSegment__Sequence__init(&mut msg.segments, self.segments.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.segments.data, msg.segments.size)};
for (t,s) in slice.iter_mut().zip(&self.segments) { s.copy_to_native(t);}
unsafe { geographic_msgs__msg__KeyValue__Sequence__fini(&mut msg.props) };
unsafe { geographic_msgs__msg__KeyValue__Sequence__init(&mut msg.props, self.props.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.props.data, msg.props.size)};
for (t,s) in slice.iter_mut().zip(&self.props) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for RouteNetwork {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RouteNetwork>::new();
                                  RouteNetwork::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RoutePath {

                              pub header: std_msgs::msg::Header,
pub network: unique_identifier_msgs::msg::UUID,
pub segments: Vec<unique_identifier_msgs::msg::UUID>,
pub props: Vec<geographic_msgs::msg::KeyValue>,

                          }

                          impl WrappedTypesupport for RoutePath { 

            type CStruct = geographic_msgs__msg__RoutePath; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RoutePath() }
            }

            fn create_msg() -> *mut geographic_msgs__msg__RoutePath {

                unsafe { geographic_msgs__msg__RoutePath__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__msg__RoutePath) -> () {

                unsafe { geographic_msgs__msg__RoutePath__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RoutePath {
  RoutePath {
header: std_msgs::msg::Header::from_native(&msg.header),
network: unique_identifier_msgs::msg::UUID::from_native(&msg.network),
// is_upper_bound_: false
// member.array_size_ : 0
segments : {
let mut temp = Vec::with_capacity(msg.segments.size);
let slice = unsafe { std::slice::from_raw_parts(msg.segments.data, msg.segments.size)};
for s in slice { temp.push(unique_identifier_msgs::msg::UUID::from_native(s)); }
temp },
// is_upper_bound_: false
// member.array_size_ : 0
props : {
let mut temp = Vec::with_capacity(msg.props.size);
let slice = unsafe { std::slice::from_raw_parts(msg.props.data, msg.props.size)};
for s in slice { temp.push(geographic_msgs::msg::KeyValue::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
self.network.copy_to_native(&mut msg.network);
unsafe { unique_identifier_msgs__msg__UUID__Sequence__fini(&mut msg.segments) };
unsafe { unique_identifier_msgs__msg__UUID__Sequence__init(&mut msg.segments, self.segments.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.segments.data, msg.segments.size)};
for (t,s) in slice.iter_mut().zip(&self.segments) { s.copy_to_native(t);}
unsafe { geographic_msgs__msg__KeyValue__Sequence__fini(&mut msg.props) };
unsafe { geographic_msgs__msg__KeyValue__Sequence__init(&mut msg.props, self.props.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.props.data, msg.props.size)};
for (t,s) in slice.iter_mut().zip(&self.props) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for RoutePath {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RoutePath>::new();
                                  RoutePath::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RouteSegment {

                              pub id: unique_identifier_msgs::msg::UUID,
pub start: unique_identifier_msgs::msg::UUID,
pub end: unique_identifier_msgs::msg::UUID,
pub props: Vec<geographic_msgs::msg::KeyValue>,

                          }

                          impl WrappedTypesupport for RouteSegment { 

            type CStruct = geographic_msgs__msg__RouteSegment; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__RouteSegment() }
            }

            fn create_msg() -> *mut geographic_msgs__msg__RouteSegment {

                unsafe { geographic_msgs__msg__RouteSegment__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__msg__RouteSegment) -> () {

                unsafe { geographic_msgs__msg__RouteSegment__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RouteSegment {
  RouteSegment {
id: unique_identifier_msgs::msg::UUID::from_native(&msg.id),
start: unique_identifier_msgs::msg::UUID::from_native(&msg.start),
end: unique_identifier_msgs::msg::UUID::from_native(&msg.end),
// is_upper_bound_: false
// member.array_size_ : 0
props : {
let mut temp = Vec::with_capacity(msg.props.size);
let slice = unsafe { std::slice::from_raw_parts(msg.props.data, msg.props.size)};
for s in slice { temp.push(geographic_msgs::msg::KeyValue::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.id.copy_to_native(&mut msg.id);
self.start.copy_to_native(&mut msg.start);
self.end.copy_to_native(&mut msg.end);
unsafe { geographic_msgs__msg__KeyValue__Sequence__fini(&mut msg.props) };
unsafe { geographic_msgs__msg__KeyValue__Sequence__init(&mut msg.props, self.props.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.props.data, msg.props.size)};
for (t,s) in slice.iter_mut().zip(&self.props) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for RouteSegment {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RouteSegment>::new();
                                  RouteSegment::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct WayPoint {

                              pub id: unique_identifier_msgs::msg::UUID,
pub position: geographic_msgs::msg::GeoPoint,
pub props: Vec<geographic_msgs::msg::KeyValue>,

                          }

                          impl WrappedTypesupport for WayPoint { 

            type CStruct = geographic_msgs__msg__WayPoint; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__geographic_msgs__msg__WayPoint() }
            }

            fn create_msg() -> *mut geographic_msgs__msg__WayPoint {

                unsafe { geographic_msgs__msg__WayPoint__create() }

            }

            fn destroy_msg(msg: *mut geographic_msgs__msg__WayPoint) -> () {

                unsafe { geographic_msgs__msg__WayPoint__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> WayPoint {
  WayPoint {
id: unique_identifier_msgs::msg::UUID::from_native(&msg.id),
position: geographic_msgs::msg::GeoPoint::from_native(&msg.position),
// is_upper_bound_: false
// member.array_size_ : 0
props : {
let mut temp = Vec::with_capacity(msg.props.size);
let slice = unsafe { std::slice::from_raw_parts(msg.props.data, msg.props.size)};
for s in slice { temp.push(geographic_msgs::msg::KeyValue::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.id.copy_to_native(&mut msg.id);
self.position.copy_to_native(&mut msg.position);
unsafe { geographic_msgs__msg__KeyValue__Sequence__fini(&mut msg.props) };
unsafe { geographic_msgs__msg__KeyValue__Sequence__init(&mut msg.props, self.props.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.props.data, msg.props.size)};
for (t,s) in slice.iter_mut().zip(&self.props) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for WayPoint {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<WayPoint>::new();
                                  WayPoint::from_native(&msg_native)
                              }
                          }
             


                      }
