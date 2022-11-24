  pub mod msg {
    use super::super::*;

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RadarReturn {

                              pub range: f32,
pub azimuth: f32,
pub elevation: f32,
pub doppler_velocity: f32,
pub amplitude: f32,

                          }

                          impl WrappedTypesupport for RadarReturn { 

            type CStruct = radar_msgs__msg__RadarReturn; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__radar_msgs__msg__RadarReturn() }
            }

            fn create_msg() -> *mut radar_msgs__msg__RadarReturn {

                unsafe { radar_msgs__msg__RadarReturn__create() }

            }

            fn destroy_msg(msg: *mut radar_msgs__msg__RadarReturn) -> () {

                unsafe { radar_msgs__msg__RadarReturn__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RadarReturn {
  RadarReturn {
range: msg.range,
azimuth: msg.azimuth,
elevation: msg.elevation,
doppler_velocity: msg.doppler_velocity,
amplitude: msg.amplitude,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.range = self.range;
msg.azimuth = self.azimuth;
msg.elevation = self.elevation;
msg.doppler_velocity = self.doppler_velocity;
msg.amplitude = self.amplitude;
}



        }


                          
                          impl Default for RadarReturn {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RadarReturn>::new();
                                  RadarReturn::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RadarScan {

                              pub header: std_msgs::msg::Header,
pub returns: Vec<radar_msgs::msg::RadarReturn>,

                          }

                          impl WrappedTypesupport for RadarScan { 

            type CStruct = radar_msgs__msg__RadarScan; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__radar_msgs__msg__RadarScan() }
            }

            fn create_msg() -> *mut radar_msgs__msg__RadarScan {

                unsafe { radar_msgs__msg__RadarScan__create() }

            }

            fn destroy_msg(msg: *mut radar_msgs__msg__RadarScan) -> () {

                unsafe { radar_msgs__msg__RadarScan__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RadarScan {
  RadarScan {
header: std_msgs::msg::Header::from_native(&msg.header),
// is_upper_bound_: false
// member.array_size_ : 0
returns : {
let mut temp = Vec::with_capacity(msg.returns.size);
let slice = unsafe { std::slice::from_raw_parts(msg.returns.data, msg.returns.size)};
for s in slice { temp.push(radar_msgs::msg::RadarReturn::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
unsafe { radar_msgs__msg__RadarReturn__Sequence__fini(&mut msg.returns) };
unsafe { radar_msgs__msg__RadarReturn__Sequence__init(&mut msg.returns, self.returns.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.returns.data, msg.returns.size)};
for (t,s) in slice.iter_mut().zip(&self.returns) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for RadarScan {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RadarScan>::new();
                                  RadarScan::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RadarTrack {

                              pub uuid: unique_identifier_msgs::msg::UUID,
pub position: geometry_msgs::msg::Point,
pub velocity: geometry_msgs::msg::Vector3,
pub acceleration: geometry_msgs::msg::Vector3,
pub size: geometry_msgs::msg::Vector3,
pub classification: u16,
pub position_covariance: Vec<f32>,
pub velocity_covariance: Vec<f32>,
pub acceleration_covariance: Vec<f32>,
pub size_covariance: Vec<f32>,

                          }

                          impl WrappedTypesupport for RadarTrack { 

            type CStruct = radar_msgs__msg__RadarTrack; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__radar_msgs__msg__RadarTrack() }
            }

            fn create_msg() -> *mut radar_msgs__msg__RadarTrack {

                unsafe { radar_msgs__msg__RadarTrack__create() }

            }

            fn destroy_msg(msg: *mut radar_msgs__msg__RadarTrack) -> () {

                unsafe { radar_msgs__msg__RadarTrack__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RadarTrack {
  RadarTrack {
uuid: unique_identifier_msgs::msg::UUID::from_native(&msg.uuid),
position: geometry_msgs::msg::Point::from_native(&msg.position),
velocity: geometry_msgs::msg::Vector3::from_native(&msg.velocity),
acceleration: geometry_msgs::msg::Vector3::from_native(&msg.acceleration),
size: geometry_msgs::msg::Vector3::from_native(&msg.size),
classification: msg.classification,
// is_upper_bound_: false
// member.array_size_ : 6
position_covariance: msg.position_covariance.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 6
velocity_covariance: msg.velocity_covariance.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 6
acceleration_covariance: msg.acceleration_covariance.to_vec(),
// is_upper_bound_: false
// member.array_size_ : 6
size_covariance: msg.size_covariance.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.uuid.copy_to_native(&mut msg.uuid);
self.position.copy_to_native(&mut msg.position);
self.velocity.copy_to_native(&mut msg.velocity);
self.acceleration.copy_to_native(&mut msg.acceleration);
self.size.copy_to_native(&mut msg.size);
msg.classification = self.classification;
assert_eq!(self.position_covariance.len(), 6, "Field {} is fixed size of {}!", "position_covariance", 6);
msg.position_covariance.copy_from_slice(&self.position_covariance[..6]);
assert_eq!(self.velocity_covariance.len(), 6, "Field {} is fixed size of {}!", "velocity_covariance", 6);
msg.velocity_covariance.copy_from_slice(&self.velocity_covariance[..6]);
assert_eq!(self.acceleration_covariance.len(), 6, "Field {} is fixed size of {}!", "acceleration_covariance", 6);
msg.acceleration_covariance.copy_from_slice(&self.acceleration_covariance[..6]);
assert_eq!(self.size_covariance.len(), 6, "Field {} is fixed size of {}!", "size_covariance", 6);
msg.size_covariance.copy_from_slice(&self.size_covariance[..6]);
}



        }


                          
                          impl Default for RadarTrack {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RadarTrack>::new();
                                  RadarTrack::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct RadarTracks {

                              pub header: std_msgs::msg::Header,
pub tracks: Vec<radar_msgs::msg::RadarTrack>,

                          }

                          impl WrappedTypesupport for RadarTracks { 

            type CStruct = radar_msgs__msg__RadarTracks; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__radar_msgs__msg__RadarTracks() }
            }

            fn create_msg() -> *mut radar_msgs__msg__RadarTracks {

                unsafe { radar_msgs__msg__RadarTracks__create() }

            }

            fn destroy_msg(msg: *mut radar_msgs__msg__RadarTracks) -> () {

                unsafe { radar_msgs__msg__RadarTracks__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> RadarTracks {
  RadarTracks {
header: std_msgs::msg::Header::from_native(&msg.header),
// is_upper_bound_: false
// member.array_size_ : 0
tracks : {
let mut temp = Vec::with_capacity(msg.tracks.size);
let slice = unsafe { std::slice::from_raw_parts(msg.tracks.data, msg.tracks.size)};
for s in slice { temp.push(radar_msgs::msg::RadarTrack::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
unsafe { radar_msgs__msg__RadarTrack__Sequence__fini(&mut msg.tracks) };
unsafe { radar_msgs__msg__RadarTrack__Sequence__init(&mut msg.tracks, self.tracks.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.tracks.data, msg.tracks.size)};
for (t,s) in slice.iter_mut().zip(&self.tracks) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for RadarTracks {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<RadarTracks>::new();
                                  RadarTracks::from_native(&msg_native)
                              }
                          }
             


                      }
