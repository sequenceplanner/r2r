  pub mod msg {
    use super::super::*;

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct BehaviorTreeLog {

                              pub timestamp: builtin_interfaces::msg::Time,
pub event_log: Vec<nav2_msgs::msg::BehaviorTreeStatusChange>,

                          }

                          impl WrappedTypesupport for BehaviorTreeLog { 

            type CStruct = nav2_msgs__msg__BehaviorTreeLog; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__BehaviorTreeLog() }
            }

            fn create_msg() -> *mut nav2_msgs__msg__BehaviorTreeLog {

                unsafe { nav2_msgs__msg__BehaviorTreeLog__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__msg__BehaviorTreeLog) -> () {

                unsafe { nav2_msgs__msg__BehaviorTreeLog__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> BehaviorTreeLog {
  BehaviorTreeLog {
timestamp: builtin_interfaces::msg::Time::from_native(&msg.timestamp),
// is_upper_bound_: false
// member.array_size_ : 0
event_log : {
let mut temp = Vec::with_capacity(msg.event_log.size);
let slice = unsafe { std::slice::from_raw_parts(msg.event_log.data, msg.event_log.size)};
for s in slice { temp.push(nav2_msgs::msg::BehaviorTreeStatusChange::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.timestamp.copy_to_native(&mut msg.timestamp);
unsafe { nav2_msgs__msg__BehaviorTreeStatusChange__Sequence__fini(&mut msg.event_log) };
unsafe { nav2_msgs__msg__BehaviorTreeStatusChange__Sequence__init(&mut msg.event_log, self.event_log.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.event_log.data, msg.event_log.size)};
for (t,s) in slice.iter_mut().zip(&self.event_log) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for BehaviorTreeLog {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<BehaviorTreeLog>::new();
                                  BehaviorTreeLog::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct BehaviorTreeStatusChange {

                              pub timestamp: builtin_interfaces::msg::Time,
pub node_name: std::string::String,
pub previous_status: std::string::String,
pub current_status: std::string::String,

                          }

                          impl WrappedTypesupport for BehaviorTreeStatusChange { 

            type CStruct = nav2_msgs__msg__BehaviorTreeStatusChange; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__BehaviorTreeStatusChange() }
            }

            fn create_msg() -> *mut nav2_msgs__msg__BehaviorTreeStatusChange {

                unsafe { nav2_msgs__msg__BehaviorTreeStatusChange__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__msg__BehaviorTreeStatusChange) -> () {

                unsafe { nav2_msgs__msg__BehaviorTreeStatusChange__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> BehaviorTreeStatusChange {
  BehaviorTreeStatusChange {
timestamp: builtin_interfaces::msg::Time::from_native(&msg.timestamp),
node_name: msg.node_name.to_str().to_owned(),
previous_status: msg.previous_status.to_str().to_owned(),
current_status: msg.current_status.to_str().to_owned(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.timestamp.copy_to_native(&mut msg.timestamp);
msg.node_name.assign(&self.node_name);
msg.previous_status.assign(&self.previous_status);
msg.current_status.assign(&self.current_status);
}



        }


                          
                          impl Default for BehaviorTreeStatusChange {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<BehaviorTreeStatusChange>::new();
                                  BehaviorTreeStatusChange::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Costmap {

                              pub header: std_msgs::msg::Header,
pub metadata: nav2_msgs::msg::CostmapMetaData,
pub data: Vec<u8>,

                          }

                          impl WrappedTypesupport for Costmap { 

            type CStruct = nav2_msgs__msg__Costmap; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__Costmap() }
            }

            fn create_msg() -> *mut nav2_msgs__msg__Costmap {

                unsafe { nav2_msgs__msg__Costmap__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__msg__Costmap) -> () {

                unsafe { nav2_msgs__msg__Costmap__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Costmap {
  Costmap {
header: std_msgs::msg::Header::from_native(&msg.header),
metadata: nav2_msgs::msg::CostmapMetaData::from_native(&msg.metadata),
// is_upper_bound_: false
// member.array_size_ : 0
data: msg.data.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
self.metadata.copy_to_native(&mut msg.metadata);
msg.data.update(&self.data);
}



        }


                          
                          impl Default for Costmap {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Costmap>::new();
                                  Costmap::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CostmapFilterInfo {

                              pub header: std_msgs::msg::Header,
pub type_: u8,
pub filter_mask_topic: std::string::String,
pub base: f32,
pub multiplier: f32,

                          }

                          impl WrappedTypesupport for CostmapFilterInfo { 

            type CStruct = nav2_msgs__msg__CostmapFilterInfo; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CostmapFilterInfo() }
            }

            fn create_msg() -> *mut nav2_msgs__msg__CostmapFilterInfo {

                unsafe { nav2_msgs__msg__CostmapFilterInfo__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__msg__CostmapFilterInfo) -> () {

                unsafe { nav2_msgs__msg__CostmapFilterInfo__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CostmapFilterInfo {
  CostmapFilterInfo {
header: std_msgs::msg::Header::from_native(&msg.header),
type_: msg.type_,
filter_mask_topic: msg.filter_mask_topic.to_str().to_owned(),
base: msg.base,
multiplier: msg.multiplier,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
msg.type_ = self.type_;
msg.filter_mask_topic.assign(&self.filter_mask_topic);
msg.base = self.base;
msg.multiplier = self.multiplier;
}



        }


                          
                          impl Default for CostmapFilterInfo {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CostmapFilterInfo>::new();
                                  CostmapFilterInfo::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct CostmapMetaData {

                              pub map_load_time: builtin_interfaces::msg::Time,
pub update_time: builtin_interfaces::msg::Time,
pub layer: std::string::String,
pub resolution: f32,
pub size_x: u32,
pub size_y: u32,
pub origin: geometry_msgs::msg::Pose,

                          }

                          impl WrappedTypesupport for CostmapMetaData { 

            type CStruct = nav2_msgs__msg__CostmapMetaData; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__CostmapMetaData() }
            }

            fn create_msg() -> *mut nav2_msgs__msg__CostmapMetaData {

                unsafe { nav2_msgs__msg__CostmapMetaData__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__msg__CostmapMetaData) -> () {

                unsafe { nav2_msgs__msg__CostmapMetaData__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> CostmapMetaData {
  CostmapMetaData {
map_load_time: builtin_interfaces::msg::Time::from_native(&msg.map_load_time),
update_time: builtin_interfaces::msg::Time::from_native(&msg.update_time),
layer: msg.layer.to_str().to_owned(),
resolution: msg.resolution,
size_x: msg.size_x,
size_y: msg.size_y,
origin: geometry_msgs::msg::Pose::from_native(&msg.origin),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.map_load_time.copy_to_native(&mut msg.map_load_time);
self.update_time.copy_to_native(&mut msg.update_time);
msg.layer.assign(&self.layer);
msg.resolution = self.resolution;
msg.size_x = self.size_x;
msg.size_y = self.size_y;
self.origin.copy_to_native(&mut msg.origin);
}



        }


                          
                          impl Default for CostmapMetaData {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<CostmapMetaData>::new();
                                  CostmapMetaData::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Particle {

                              pub pose: geometry_msgs::msg::Pose,
pub weight: f64,

                          }

                          impl WrappedTypesupport for Particle { 

            type CStruct = nav2_msgs__msg__Particle; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__Particle() }
            }

            fn create_msg() -> *mut nav2_msgs__msg__Particle {

                unsafe { nav2_msgs__msg__Particle__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__msg__Particle) -> () {

                unsafe { nav2_msgs__msg__Particle__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Particle {
  Particle {
pose: geometry_msgs::msg::Pose::from_native(&msg.pose),
weight: msg.weight,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.pose.copy_to_native(&mut msg.pose);
msg.weight = self.weight;
}



        }


                          
                          impl Default for Particle {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Particle>::new();
                                  Particle::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct ParticleCloud {

                              pub header: std_msgs::msg::Header,
pub particles: Vec<nav2_msgs::msg::Particle>,

                          }

                          impl WrappedTypesupport for ParticleCloud { 

            type CStruct = nav2_msgs__msg__ParticleCloud; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__ParticleCloud() }
            }

            fn create_msg() -> *mut nav2_msgs__msg__ParticleCloud {

                unsafe { nav2_msgs__msg__ParticleCloud__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__msg__ParticleCloud) -> () {

                unsafe { nav2_msgs__msg__ParticleCloud__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> ParticleCloud {
  ParticleCloud {
header: std_msgs::msg::Header::from_native(&msg.header),
// is_upper_bound_: false
// member.array_size_ : 0
particles : {
let mut temp = Vec::with_capacity(msg.particles.size);
let slice = unsafe { std::slice::from_raw_parts(msg.particles.data, msg.particles.size)};
for s in slice { temp.push(nav2_msgs::msg::Particle::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
unsafe { nav2_msgs__msg__Particle__Sequence__fini(&mut msg.particles) };
unsafe { nav2_msgs__msg__Particle__Sequence__init(&mut msg.particles, self.particles.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.particles.data, msg.particles.size)};
for (t,s) in slice.iter_mut().zip(&self.particles) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for ParticleCloud {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<ParticleCloud>::new();
                                  ParticleCloud::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct SpeedLimit {

                              pub header: std_msgs::msg::Header,
pub percentage: bool,
pub speed_limit: f64,

                          }

                          impl WrappedTypesupport for SpeedLimit { 

            type CStruct = nav2_msgs__msg__SpeedLimit; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__SpeedLimit() }
            }

            fn create_msg() -> *mut nav2_msgs__msg__SpeedLimit {

                unsafe { nav2_msgs__msg__SpeedLimit__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__msg__SpeedLimit) -> () {

                unsafe { nav2_msgs__msg__SpeedLimit__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> SpeedLimit {
  SpeedLimit {
header: std_msgs::msg::Header::from_native(&msg.header),
percentage: msg.percentage,
speed_limit: msg.speed_limit,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
msg.percentage = self.percentage;
msg.speed_limit = self.speed_limit;
}



        }


                          
                          impl Default for SpeedLimit {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<SpeedLimit>::new();
                                  SpeedLimit::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct VoxelGrid {

                              pub header: std_msgs::msg::Header,
pub data: Vec<u32>,
pub origin: geometry_msgs::msg::Point32,
pub resolutions: geometry_msgs::msg::Vector3,
pub size_x: u32,
pub size_y: u32,
pub size_z: u32,

                          }

                          impl WrappedTypesupport for VoxelGrid { 

            type CStruct = nav2_msgs__msg__VoxelGrid; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__msg__VoxelGrid() }
            }

            fn create_msg() -> *mut nav2_msgs__msg__VoxelGrid {

                unsafe { nav2_msgs__msg__VoxelGrid__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__msg__VoxelGrid) -> () {

                unsafe { nav2_msgs__msg__VoxelGrid__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> VoxelGrid {
  VoxelGrid {
header: std_msgs::msg::Header::from_native(&msg.header),
// is_upper_bound_: false
// member.array_size_ : 0
data: msg.data.to_vec(),
origin: geometry_msgs::msg::Point32::from_native(&msg.origin),
resolutions: geometry_msgs::msg::Vector3::from_native(&msg.resolutions),
size_x: msg.size_x,
size_y: msg.size_y,
size_z: msg.size_z,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.header.copy_to_native(&mut msg.header);
msg.data.update(&self.data);
self.origin.copy_to_native(&mut msg.origin);
self.resolutions.copy_to_native(&mut msg.resolutions);
msg.size_x = self.size_x;
msg.size_y = self.size_y;
msg.size_z = self.size_z;
}



        }


                          
                          impl Default for VoxelGrid {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<VoxelGrid>::new();
                                  VoxelGrid::from_native(&msg_native)
                              }
                          }
             


                      }
  pub mod action {
#[allow(non_snake_case)]
    pub mod BackUp {
    use super::super::super::*;

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
                    &*rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__BackUp()
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

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Goal {

                              pub target: geometry_msgs::msg::Point,
pub speed: f32,

                          }

                          impl WrappedTypesupport for Goal { 

            type CStruct = nav2_msgs__action__BackUp_Goal; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Goal() }
            }

            fn create_msg() -> *mut nav2_msgs__action__BackUp_Goal {

                unsafe { nav2_msgs__action__BackUp_Goal__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__BackUp_Goal) -> () {

                unsafe { nav2_msgs__action__BackUp_Goal__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Goal {
  Goal {
target: geometry_msgs::msg::Point::from_native(&msg.target),
speed: msg.speed,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.target.copy_to_native(&mut msg.target);
msg.speed = self.speed;
}



        }


                          
                          impl Default for Goal {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Goal>::new();
                                  Goal::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Result {

                              pub total_elapsed_time: builtin_interfaces::msg::Duration,

                          }

                          impl WrappedTypesupport for Result { 

            type CStruct = nav2_msgs__action__BackUp_Result; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Result() }
            }

            fn create_msg() -> *mut nav2_msgs__action__BackUp_Result {

                unsafe { nav2_msgs__action__BackUp_Result__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__BackUp_Result) -> () {

                unsafe { nav2_msgs__action__BackUp_Result__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Result {
  Result {
total_elapsed_time: builtin_interfaces::msg::Duration::from_native(&msg.total_elapsed_time),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.total_elapsed_time.copy_to_native(&mut msg.total_elapsed_time);
}



        }


                          
                          impl Default for Result {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Result>::new();
                                  Result::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Feedback {

                              pub distance_traveled: f32,

                          }

                          impl WrappedTypesupport for Feedback { 

            type CStruct = nav2_msgs__action__BackUp_Feedback; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_Feedback() }
            }

            fn create_msg() -> *mut nav2_msgs__action__BackUp_Feedback {

                unsafe { nav2_msgs__action__BackUp_Feedback__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__BackUp_Feedback) -> () {

                unsafe { nav2_msgs__action__BackUp_Feedback__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Feedback {
  Feedback {
distance_traveled: msg.distance_traveled,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.distance_traveled = self.distance_traveled;
}



        }


                          
                          impl Default for Feedback {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Feedback>::new();
                                  Feedback::from_native(&msg_native)
                              }
                          }
             


                    #[allow(non_snake_case)]
    pub mod SendGoal {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__BackUp_SendGoal()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub goal: nav2_msgs::action::BackUp::Goal,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__BackUp_SendGoal_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_SendGoal_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__BackUp_SendGoal_Request {

                unsafe { nav2_msgs__action__BackUp_SendGoal_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__BackUp_SendGoal_Request) -> () {

                unsafe { nav2_msgs__action__BackUp_SendGoal_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
goal: nav2_msgs::action::BackUp::Goal::from_native(&msg.goal),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub accepted: bool,
pub stamp: builtin_interfaces::msg::Time,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__BackUp_SendGoal_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_SendGoal_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__BackUp_SendGoal_Response {

                unsafe { nav2_msgs__action__BackUp_SendGoal_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__BackUp_SendGoal_Response) -> () {

                unsafe { nav2_msgs__action__BackUp_SendGoal_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
accepted: msg.accepted,
stamp: builtin_interfaces::msg::Time::from_native(&msg.stamp),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.accepted = self.accepted;
self.stamp.copy_to_native(&mut msg.stamp);
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
    pub mod GetResult {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__BackUp_GetResult()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__BackUp_GetResult_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_GetResult_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__BackUp_GetResult_Request {

                unsafe { nav2_msgs__action__BackUp_GetResult_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__BackUp_GetResult_Request) -> () {

                unsafe { nav2_msgs__action__BackUp_GetResult_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub status: i8,
pub result: nav2_msgs::action::BackUp::Result,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__BackUp_GetResult_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_GetResult_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__BackUp_GetResult_Response {

                unsafe { nav2_msgs__action__BackUp_GetResult_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__BackUp_GetResult_Response) -> () {

                unsafe { nav2_msgs__action__BackUp_GetResult_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
status: msg.status,
result: nav2_msgs::action::BackUp::Result::from_native(&msg.result),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.status = self.status;
self.result.copy_to_native(&mut msg.result);
}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct FeedbackMessage {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub feedback: nav2_msgs::action::BackUp::Feedback,

                          }

                          impl WrappedTypesupport for FeedbackMessage { 

            type CStruct = nav2_msgs__action__BackUp_FeedbackMessage; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__BackUp_FeedbackMessage() }
            }

            fn create_msg() -> *mut nav2_msgs__action__BackUp_FeedbackMessage {

                unsafe { nav2_msgs__action__BackUp_FeedbackMessage__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__BackUp_FeedbackMessage) -> () {

                unsafe { nav2_msgs__action__BackUp_FeedbackMessage__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> FeedbackMessage {
  FeedbackMessage {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
feedback: nav2_msgs::action::BackUp::Feedback::from_native(&msg.feedback),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
self.feedback.copy_to_native(&mut msg.feedback);
}



        }


                          
                          impl Default for FeedbackMessage {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<FeedbackMessage>::new();
                                  FeedbackMessage::from_native(&msg_native)
                              }
                          }
             


                        }
#[allow(non_snake_case)]
    pub mod ComputePathThroughPoses {
    use super::super::super::*;

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
                    &*rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__ComputePathThroughPoses()
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

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Goal {

                              pub goals: Vec<geometry_msgs::msg::PoseStamped>,
pub start: geometry_msgs::msg::PoseStamped,
pub planner_id: std::string::String,
pub use_start: bool,

                          }

                          impl WrappedTypesupport for Goal { 

            type CStruct = nav2_msgs__action__ComputePathThroughPoses_Goal; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Goal() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathThroughPoses_Goal {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_Goal__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathThroughPoses_Goal) -> () {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_Goal__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Goal {
  Goal {
// is_upper_bound_: false
// member.array_size_ : 0
goals : {
let mut temp = Vec::with_capacity(msg.goals.size);
let slice = unsafe { std::slice::from_raw_parts(msg.goals.data, msg.goals.size)};
for s in slice { temp.push(geometry_msgs::msg::PoseStamped::from_native(s)); }
temp },
start: geometry_msgs::msg::PoseStamped::from_native(&msg.start),
planner_id: msg.planner_id.to_str().to_owned(),
use_start: msg.use_start,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {unsafe { geometry_msgs__msg__PoseStamped__Sequence__fini(&mut msg.goals) };
unsafe { geometry_msgs__msg__PoseStamped__Sequence__init(&mut msg.goals, self.goals.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.goals.data, msg.goals.size)};
for (t,s) in slice.iter_mut().zip(&self.goals) { s.copy_to_native(t);}
self.start.copy_to_native(&mut msg.start);
msg.planner_id.assign(&self.planner_id);
msg.use_start = self.use_start;
}



        }


                          
                          impl Default for Goal {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Goal>::new();
                                  Goal::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Result {

                              pub path: nav_msgs::msg::Path,
pub planning_time: builtin_interfaces::msg::Duration,

                          }

                          impl WrappedTypesupport for Result { 

            type CStruct = nav2_msgs__action__ComputePathThroughPoses_Result; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Result() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathThroughPoses_Result {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_Result__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathThroughPoses_Result) -> () {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_Result__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Result {
  Result {
path: nav_msgs::msg::Path::from_native(&msg.path),
planning_time: builtin_interfaces::msg::Duration::from_native(&msg.planning_time),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.path.copy_to_native(&mut msg.path);
self.planning_time.copy_to_native(&mut msg.planning_time);
}



        }


                          
                          impl Default for Result {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Result>::new();
                                  Result::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Feedback {

                              
                          }

                          impl WrappedTypesupport for Feedback { 

            type CStruct = nav2_msgs__action__ComputePathThroughPoses_Feedback; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_Feedback() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathThroughPoses_Feedback {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_Feedback__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathThroughPoses_Feedback) -> () {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_Feedback__destroy(msg) };

            }

            fn from_native(_msg: &Self::CStruct) -> Feedback {
  Feedback {
      }
    }



            fn copy_to_native(&self, _msg: &mut Self::CStruct) {}



        }


                          
                          impl Default for Feedback {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Feedback>::new();
                                  Feedback::from_native(&msg_native)
                              }
                          }
             


                    #[allow(non_snake_case)]
    pub mod SendGoal {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub goal: nav2_msgs::action::ComputePathThroughPoses::Goal,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request) -> () {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
goal: nav2_msgs::action::ComputePathThroughPoses::Goal::from_native(&msg.goal),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub accepted: bool,
pub stamp: builtin_interfaces::msg::Time,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response) -> () {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_SendGoal_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
accepted: msg.accepted,
stamp: builtin_interfaces::msg::Time::from_native(&msg.stamp),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.accepted = self.accepted;
self.stamp.copy_to_native(&mut msg.stamp);
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
    pub mod GetResult {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__ComputePathThroughPoses_GetResult_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathThroughPoses_GetResult_Request {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathThroughPoses_GetResult_Request) -> () {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub status: i8,
pub result: nav2_msgs::action::ComputePathThroughPoses::Result,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__ComputePathThroughPoses_GetResult_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_GetResult_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathThroughPoses_GetResult_Response {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathThroughPoses_GetResult_Response) -> () {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_GetResult_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
status: msg.status,
result: nav2_msgs::action::ComputePathThroughPoses::Result::from_native(&msg.result),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.status = self.status;
self.result.copy_to_native(&mut msg.result);
}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct FeedbackMessage {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub feedback: nav2_msgs::action::ComputePathThroughPoses::Feedback,

                          }

                          impl WrappedTypesupport for FeedbackMessage { 

            type CStruct = nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage) -> () {

                unsafe { nav2_msgs__action__ComputePathThroughPoses_FeedbackMessage__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> FeedbackMessage {
  FeedbackMessage {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
feedback: nav2_msgs::action::ComputePathThroughPoses::Feedback::from_native(&msg.feedback),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
self.feedback.copy_to_native(&mut msg.feedback);
}



        }


                          
                          impl Default for FeedbackMessage {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<FeedbackMessage>::new();
                                  FeedbackMessage::from_native(&msg_native)
                              }
                          }
             


                        }
#[allow(non_snake_case)]
    pub mod ComputePathToPose {
    use super::super::super::*;

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
                    &*rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__ComputePathToPose()
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

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Goal {

                              pub goal: geometry_msgs::msg::PoseStamped,
pub start: geometry_msgs::msg::PoseStamped,
pub planner_id: std::string::String,
pub use_start: bool,

                          }

                          impl WrappedTypesupport for Goal { 

            type CStruct = nav2_msgs__action__ComputePathToPose_Goal; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Goal() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathToPose_Goal {

                unsafe { nav2_msgs__action__ComputePathToPose_Goal__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathToPose_Goal) -> () {

                unsafe { nav2_msgs__action__ComputePathToPose_Goal__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Goal {
  Goal {
goal: geometry_msgs::msg::PoseStamped::from_native(&msg.goal),
start: geometry_msgs::msg::PoseStamped::from_native(&msg.start),
planner_id: msg.planner_id.to_str().to_owned(),
use_start: msg.use_start,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal.copy_to_native(&mut msg.goal);
self.start.copy_to_native(&mut msg.start);
msg.planner_id.assign(&self.planner_id);
msg.use_start = self.use_start;
}



        }


                          
                          impl Default for Goal {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Goal>::new();
                                  Goal::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Result {

                              pub path: nav_msgs::msg::Path,
pub planning_time: builtin_interfaces::msg::Duration,

                          }

                          impl WrappedTypesupport for Result { 

            type CStruct = nav2_msgs__action__ComputePathToPose_Result; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Result() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathToPose_Result {

                unsafe { nav2_msgs__action__ComputePathToPose_Result__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathToPose_Result) -> () {

                unsafe { nav2_msgs__action__ComputePathToPose_Result__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Result {
  Result {
path: nav_msgs::msg::Path::from_native(&msg.path),
planning_time: builtin_interfaces::msg::Duration::from_native(&msg.planning_time),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.path.copy_to_native(&mut msg.path);
self.planning_time.copy_to_native(&mut msg.planning_time);
}



        }


                          
                          impl Default for Result {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Result>::new();
                                  Result::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Feedback {

                              
                          }

                          impl WrappedTypesupport for Feedback { 

            type CStruct = nav2_msgs__action__ComputePathToPose_Feedback; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_Feedback() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathToPose_Feedback {

                unsafe { nav2_msgs__action__ComputePathToPose_Feedback__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathToPose_Feedback) -> () {

                unsafe { nav2_msgs__action__ComputePathToPose_Feedback__destroy(msg) };

            }

            fn from_native(_msg: &Self::CStruct) -> Feedback {
  Feedback {
      }
    }



            fn copy_to_native(&self, _msg: &mut Self::CStruct) {}



        }


                          
                          impl Default for Feedback {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Feedback>::new();
                                  Feedback::from_native(&msg_native)
                              }
                          }
             


                    #[allow(non_snake_case)]
    pub mod SendGoal {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub goal: nav2_msgs::action::ComputePathToPose::Goal,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__ComputePathToPose_SendGoal_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathToPose_SendGoal_Request {

                unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathToPose_SendGoal_Request) -> () {

                unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
goal: nav2_msgs::action::ComputePathToPose::Goal::from_native(&msg.goal),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub accepted: bool,
pub stamp: builtin_interfaces::msg::Time,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__ComputePathToPose_SendGoal_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_SendGoal_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathToPose_SendGoal_Response {

                unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathToPose_SendGoal_Response) -> () {

                unsafe { nav2_msgs__action__ComputePathToPose_SendGoal_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
accepted: msg.accepted,
stamp: builtin_interfaces::msg::Time::from_native(&msg.stamp),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.accepted = self.accepted;
self.stamp.copy_to_native(&mut msg.stamp);
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
    pub mod GetResult {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__ComputePathToPose_GetResult_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathToPose_GetResult_Request {

                unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathToPose_GetResult_Request) -> () {

                unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub status: i8,
pub result: nav2_msgs::action::ComputePathToPose::Result,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__ComputePathToPose_GetResult_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_GetResult_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathToPose_GetResult_Response {

                unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathToPose_GetResult_Response) -> () {

                unsafe { nav2_msgs__action__ComputePathToPose_GetResult_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
status: msg.status,
result: nav2_msgs::action::ComputePathToPose::Result::from_native(&msg.result),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.status = self.status;
self.result.copy_to_native(&mut msg.result);
}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct FeedbackMessage {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub feedback: nav2_msgs::action::ComputePathToPose::Feedback,

                          }

                          impl WrappedTypesupport for FeedbackMessage { 

            type CStruct = nav2_msgs__action__ComputePathToPose_FeedbackMessage; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__ComputePathToPose_FeedbackMessage() }
            }

            fn create_msg() -> *mut nav2_msgs__action__ComputePathToPose_FeedbackMessage {

                unsafe { nav2_msgs__action__ComputePathToPose_FeedbackMessage__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__ComputePathToPose_FeedbackMessage) -> () {

                unsafe { nav2_msgs__action__ComputePathToPose_FeedbackMessage__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> FeedbackMessage {
  FeedbackMessage {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
feedback: nav2_msgs::action::ComputePathToPose::Feedback::from_native(&msg.feedback),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
self.feedback.copy_to_native(&mut msg.feedback);
}



        }


                          
                          impl Default for FeedbackMessage {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<FeedbackMessage>::new();
                                  FeedbackMessage::from_native(&msg_native)
                              }
                          }
             


                        }
#[allow(non_snake_case)]
    pub mod DummyRecovery {
    use super::super::super::*;

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
                    &*rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__DummyRecovery()
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

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Goal {

                              pub command: std_msgs::msg::String,

                          }

                          impl WrappedTypesupport for Goal { 

            type CStruct = nav2_msgs__action__DummyRecovery_Goal; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyRecovery_Goal() }
            }

            fn create_msg() -> *mut nav2_msgs__action__DummyRecovery_Goal {

                unsafe { nav2_msgs__action__DummyRecovery_Goal__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__DummyRecovery_Goal) -> () {

                unsafe { nav2_msgs__action__DummyRecovery_Goal__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Goal {
  Goal {
command: std_msgs::msg::String::from_native(&msg.command),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.command.copy_to_native(&mut msg.command);
}



        }


                          
                          impl Default for Goal {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Goal>::new();
                                  Goal::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Result {

                              pub total_elapsed_time: builtin_interfaces::msg::Duration,

                          }

                          impl WrappedTypesupport for Result { 

            type CStruct = nav2_msgs__action__DummyRecovery_Result; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyRecovery_Result() }
            }

            fn create_msg() -> *mut nav2_msgs__action__DummyRecovery_Result {

                unsafe { nav2_msgs__action__DummyRecovery_Result__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__DummyRecovery_Result) -> () {

                unsafe { nav2_msgs__action__DummyRecovery_Result__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Result {
  Result {
total_elapsed_time: builtin_interfaces::msg::Duration::from_native(&msg.total_elapsed_time),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.total_elapsed_time.copy_to_native(&mut msg.total_elapsed_time);
}



        }


                          
                          impl Default for Result {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Result>::new();
                                  Result::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Feedback {

                              
                          }

                          impl WrappedTypesupport for Feedback { 

            type CStruct = nav2_msgs__action__DummyRecovery_Feedback; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyRecovery_Feedback() }
            }

            fn create_msg() -> *mut nav2_msgs__action__DummyRecovery_Feedback {

                unsafe { nav2_msgs__action__DummyRecovery_Feedback__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__DummyRecovery_Feedback) -> () {

                unsafe { nav2_msgs__action__DummyRecovery_Feedback__destroy(msg) };

            }

            fn from_native(_msg: &Self::CStruct) -> Feedback {
  Feedback {
      }
    }



            fn copy_to_native(&self, _msg: &mut Self::CStruct) {}



        }


                          
                          impl Default for Feedback {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Feedback>::new();
                                  Feedback::from_native(&msg_native)
                              }
                          }
             


                    #[allow(non_snake_case)]
    pub mod SendGoal {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DummyRecovery_SendGoal()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub goal: nav2_msgs::action::DummyRecovery::Goal,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__DummyRecovery_SendGoal_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyRecovery_SendGoal_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__DummyRecovery_SendGoal_Request {

                unsafe { nav2_msgs__action__DummyRecovery_SendGoal_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__DummyRecovery_SendGoal_Request) -> () {

                unsafe { nav2_msgs__action__DummyRecovery_SendGoal_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
goal: nav2_msgs::action::DummyRecovery::Goal::from_native(&msg.goal),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub accepted: bool,
pub stamp: builtin_interfaces::msg::Time,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__DummyRecovery_SendGoal_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyRecovery_SendGoal_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__DummyRecovery_SendGoal_Response {

                unsafe { nav2_msgs__action__DummyRecovery_SendGoal_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__DummyRecovery_SendGoal_Response) -> () {

                unsafe { nav2_msgs__action__DummyRecovery_SendGoal_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
accepted: msg.accepted,
stamp: builtin_interfaces::msg::Time::from_native(&msg.stamp),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.accepted = self.accepted;
self.stamp.copy_to_native(&mut msg.stamp);
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
    pub mod GetResult {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__DummyRecovery_GetResult()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__DummyRecovery_GetResult_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyRecovery_GetResult_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__DummyRecovery_GetResult_Request {

                unsafe { nav2_msgs__action__DummyRecovery_GetResult_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__DummyRecovery_GetResult_Request) -> () {

                unsafe { nav2_msgs__action__DummyRecovery_GetResult_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub status: i8,
pub result: nav2_msgs::action::DummyRecovery::Result,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__DummyRecovery_GetResult_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyRecovery_GetResult_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__DummyRecovery_GetResult_Response {

                unsafe { nav2_msgs__action__DummyRecovery_GetResult_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__DummyRecovery_GetResult_Response) -> () {

                unsafe { nav2_msgs__action__DummyRecovery_GetResult_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
status: msg.status,
result: nav2_msgs::action::DummyRecovery::Result::from_native(&msg.result),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.status = self.status;
self.result.copy_to_native(&mut msg.result);
}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct FeedbackMessage {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub feedback: nav2_msgs::action::DummyRecovery::Feedback,

                          }

                          impl WrappedTypesupport for FeedbackMessage { 

            type CStruct = nav2_msgs__action__DummyRecovery_FeedbackMessage; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__DummyRecovery_FeedbackMessage() }
            }

            fn create_msg() -> *mut nav2_msgs__action__DummyRecovery_FeedbackMessage {

                unsafe { nav2_msgs__action__DummyRecovery_FeedbackMessage__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__DummyRecovery_FeedbackMessage) -> () {

                unsafe { nav2_msgs__action__DummyRecovery_FeedbackMessage__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> FeedbackMessage {
  FeedbackMessage {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
feedback: nav2_msgs::action::DummyRecovery::Feedback::from_native(&msg.feedback),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
self.feedback.copy_to_native(&mut msg.feedback);
}



        }


                          
                          impl Default for FeedbackMessage {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<FeedbackMessage>::new();
                                  FeedbackMessage::from_native(&msg_native)
                              }
                          }
             


                        }
#[allow(non_snake_case)]
    pub mod FollowPath {
    use super::super::super::*;

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
                    &*rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__FollowPath()
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

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Goal {

                              pub path: nav_msgs::msg::Path,
pub controller_id: std::string::String,
pub goal_checker_id: std::string::String,

                          }

                          impl WrappedTypesupport for Goal { 

            type CStruct = nav2_msgs__action__FollowPath_Goal; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Goal() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowPath_Goal {

                unsafe { nav2_msgs__action__FollowPath_Goal__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowPath_Goal) -> () {

                unsafe { nav2_msgs__action__FollowPath_Goal__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Goal {
  Goal {
path: nav_msgs::msg::Path::from_native(&msg.path),
controller_id: msg.controller_id.to_str().to_owned(),
goal_checker_id: msg.goal_checker_id.to_str().to_owned(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.path.copy_to_native(&mut msg.path);
msg.controller_id.assign(&self.controller_id);
msg.goal_checker_id.assign(&self.goal_checker_id);
}



        }


                          
                          impl Default for Goal {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Goal>::new();
                                  Goal::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Result {

                              pub result: std_msgs::msg::Empty,

                          }

                          impl WrappedTypesupport for Result { 

            type CStruct = nav2_msgs__action__FollowPath_Result; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Result() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowPath_Result {

                unsafe { nav2_msgs__action__FollowPath_Result__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowPath_Result) -> () {

                unsafe { nav2_msgs__action__FollowPath_Result__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Result {
  Result {
result: std_msgs::msg::Empty::from_native(&msg.result),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.result.copy_to_native(&mut msg.result);
}



        }


                          
                          impl Default for Result {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Result>::new();
                                  Result::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Feedback {

                              pub distance_to_goal: f32,
pub speed: f32,

                          }

                          impl WrappedTypesupport for Feedback { 

            type CStruct = nav2_msgs__action__FollowPath_Feedback; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_Feedback() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowPath_Feedback {

                unsafe { nav2_msgs__action__FollowPath_Feedback__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowPath_Feedback) -> () {

                unsafe { nav2_msgs__action__FollowPath_Feedback__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Feedback {
  Feedback {
distance_to_goal: msg.distance_to_goal,
speed: msg.speed,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.distance_to_goal = self.distance_to_goal;
msg.speed = self.speed;
}



        }


                          
                          impl Default for Feedback {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Feedback>::new();
                                  Feedback::from_native(&msg_native)
                              }
                          }
             


                    #[allow(non_snake_case)]
    pub mod SendGoal {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowPath_SendGoal()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub goal: nav2_msgs::action::FollowPath::Goal,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__FollowPath_SendGoal_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_SendGoal_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowPath_SendGoal_Request {

                unsafe { nav2_msgs__action__FollowPath_SendGoal_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowPath_SendGoal_Request) -> () {

                unsafe { nav2_msgs__action__FollowPath_SendGoal_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
goal: nav2_msgs::action::FollowPath::Goal::from_native(&msg.goal),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub accepted: bool,
pub stamp: builtin_interfaces::msg::Time,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__FollowPath_SendGoal_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_SendGoal_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowPath_SendGoal_Response {

                unsafe { nav2_msgs__action__FollowPath_SendGoal_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowPath_SendGoal_Response) -> () {

                unsafe { nav2_msgs__action__FollowPath_SendGoal_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
accepted: msg.accepted,
stamp: builtin_interfaces::msg::Time::from_native(&msg.stamp),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.accepted = self.accepted;
self.stamp.copy_to_native(&mut msg.stamp);
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
    pub mod GetResult {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowPath_GetResult()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__FollowPath_GetResult_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_GetResult_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowPath_GetResult_Request {

                unsafe { nav2_msgs__action__FollowPath_GetResult_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowPath_GetResult_Request) -> () {

                unsafe { nav2_msgs__action__FollowPath_GetResult_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub status: i8,
pub result: nav2_msgs::action::FollowPath::Result,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__FollowPath_GetResult_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_GetResult_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowPath_GetResult_Response {

                unsafe { nav2_msgs__action__FollowPath_GetResult_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowPath_GetResult_Response) -> () {

                unsafe { nav2_msgs__action__FollowPath_GetResult_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
status: msg.status,
result: nav2_msgs::action::FollowPath::Result::from_native(&msg.result),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.status = self.status;
self.result.copy_to_native(&mut msg.result);
}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct FeedbackMessage {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub feedback: nav2_msgs::action::FollowPath::Feedback,

                          }

                          impl WrappedTypesupport for FeedbackMessage { 

            type CStruct = nav2_msgs__action__FollowPath_FeedbackMessage; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowPath_FeedbackMessage() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowPath_FeedbackMessage {

                unsafe { nav2_msgs__action__FollowPath_FeedbackMessage__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowPath_FeedbackMessage) -> () {

                unsafe { nav2_msgs__action__FollowPath_FeedbackMessage__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> FeedbackMessage {
  FeedbackMessage {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
feedback: nav2_msgs::action::FollowPath::Feedback::from_native(&msg.feedback),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
self.feedback.copy_to_native(&mut msg.feedback);
}



        }


                          
                          impl Default for FeedbackMessage {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<FeedbackMessage>::new();
                                  FeedbackMessage::from_native(&msg_native)
                              }
                          }
             


                        }
#[allow(non_snake_case)]
    pub mod FollowWaypoints {
    use super::super::super::*;

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
                    &*rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__FollowWaypoints()
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

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Goal {

                              pub poses: Vec<geometry_msgs::msg::PoseStamped>,

                          }

                          impl WrappedTypesupport for Goal { 

            type CStruct = nav2_msgs__action__FollowWaypoints_Goal; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Goal() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowWaypoints_Goal {

                unsafe { nav2_msgs__action__FollowWaypoints_Goal__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowWaypoints_Goal) -> () {

                unsafe { nav2_msgs__action__FollowWaypoints_Goal__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Goal {
  Goal {
// is_upper_bound_: false
// member.array_size_ : 0
poses : {
let mut temp = Vec::with_capacity(msg.poses.size);
let slice = unsafe { std::slice::from_raw_parts(msg.poses.data, msg.poses.size)};
for s in slice { temp.push(geometry_msgs::msg::PoseStamped::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {unsafe { geometry_msgs__msg__PoseStamped__Sequence__fini(&mut msg.poses) };
unsafe { geometry_msgs__msg__PoseStamped__Sequence__init(&mut msg.poses, self.poses.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.poses.data, msg.poses.size)};
for (t,s) in slice.iter_mut().zip(&self.poses) { s.copy_to_native(t);}
}



        }


                          
                          impl Default for Goal {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Goal>::new();
                                  Goal::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Result {

                              pub missed_waypoints: Vec<i32>,

                          }

                          impl WrappedTypesupport for Result { 

            type CStruct = nav2_msgs__action__FollowWaypoints_Result; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Result() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowWaypoints_Result {

                unsafe { nav2_msgs__action__FollowWaypoints_Result__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowWaypoints_Result) -> () {

                unsafe { nav2_msgs__action__FollowWaypoints_Result__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Result {
  Result {
// is_upper_bound_: false
// member.array_size_ : 0
missed_waypoints: msg.missed_waypoints.to_vec(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.missed_waypoints.update(&self.missed_waypoints);
}



        }


                          
                          impl Default for Result {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Result>::new();
                                  Result::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Feedback {

                              pub current_waypoint: u32,

                          }

                          impl WrappedTypesupport for Feedback { 

            type CStruct = nav2_msgs__action__FollowWaypoints_Feedback; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_Feedback() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowWaypoints_Feedback {

                unsafe { nav2_msgs__action__FollowWaypoints_Feedback__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowWaypoints_Feedback) -> () {

                unsafe { nav2_msgs__action__FollowWaypoints_Feedback__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Feedback {
  Feedback {
current_waypoint: msg.current_waypoint,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.current_waypoint = self.current_waypoint;
}



        }


                          
                          impl Default for Feedback {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Feedback>::new();
                                  Feedback::from_native(&msg_native)
                              }
                          }
             


                    #[allow(non_snake_case)]
    pub mod SendGoal {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub goal: nav2_msgs::action::FollowWaypoints::Goal,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__FollowWaypoints_SendGoal_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowWaypoints_SendGoal_Request {

                unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowWaypoints_SendGoal_Request) -> () {

                unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
goal: nav2_msgs::action::FollowWaypoints::Goal::from_native(&msg.goal),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub accepted: bool,
pub stamp: builtin_interfaces::msg::Time,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__FollowWaypoints_SendGoal_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_SendGoal_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowWaypoints_SendGoal_Response {

                unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowWaypoints_SendGoal_Response) -> () {

                unsafe { nav2_msgs__action__FollowWaypoints_SendGoal_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
accepted: msg.accepted,
stamp: builtin_interfaces::msg::Time::from_native(&msg.stamp),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.accepted = self.accepted;
self.stamp.copy_to_native(&mut msg.stamp);
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
    pub mod GetResult {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__FollowWaypoints_GetResult_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowWaypoints_GetResult_Request {

                unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowWaypoints_GetResult_Request) -> () {

                unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub status: i8,
pub result: nav2_msgs::action::FollowWaypoints::Result,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__FollowWaypoints_GetResult_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_GetResult_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowWaypoints_GetResult_Response {

                unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowWaypoints_GetResult_Response) -> () {

                unsafe { nav2_msgs__action__FollowWaypoints_GetResult_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
status: msg.status,
result: nav2_msgs::action::FollowWaypoints::Result::from_native(&msg.result),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.status = self.status;
self.result.copy_to_native(&mut msg.result);
}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct FeedbackMessage {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub feedback: nav2_msgs::action::FollowWaypoints::Feedback,

                          }

                          impl WrappedTypesupport for FeedbackMessage { 

            type CStruct = nav2_msgs__action__FollowWaypoints_FeedbackMessage; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__FollowWaypoints_FeedbackMessage() }
            }

            fn create_msg() -> *mut nav2_msgs__action__FollowWaypoints_FeedbackMessage {

                unsafe { nav2_msgs__action__FollowWaypoints_FeedbackMessage__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__FollowWaypoints_FeedbackMessage) -> () {

                unsafe { nav2_msgs__action__FollowWaypoints_FeedbackMessage__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> FeedbackMessage {
  FeedbackMessage {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
feedback: nav2_msgs::action::FollowWaypoints::Feedback::from_native(&msg.feedback),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
self.feedback.copy_to_native(&mut msg.feedback);
}



        }


                          
                          impl Default for FeedbackMessage {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<FeedbackMessage>::new();
                                  FeedbackMessage::from_native(&msg_native)
                              }
                          }
             


                        }
#[allow(non_snake_case)]
    pub mod NavigateThroughPoses {
    use super::super::super::*;

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
                    &*rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__NavigateThroughPoses()
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

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Goal {

                              pub poses: Vec<geometry_msgs::msg::PoseStamped>,
pub behavior_tree: std::string::String,

                          }

                          impl WrappedTypesupport for Goal { 

            type CStruct = nav2_msgs__action__NavigateThroughPoses_Goal; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Goal() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateThroughPoses_Goal {

                unsafe { nav2_msgs__action__NavigateThroughPoses_Goal__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateThroughPoses_Goal) -> () {

                unsafe { nav2_msgs__action__NavigateThroughPoses_Goal__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Goal {
  Goal {
// is_upper_bound_: false
// member.array_size_ : 0
poses : {
let mut temp = Vec::with_capacity(msg.poses.size);
let slice = unsafe { std::slice::from_raw_parts(msg.poses.data, msg.poses.size)};
for s in slice { temp.push(geometry_msgs::msg::PoseStamped::from_native(s)); }
temp },
behavior_tree: msg.behavior_tree.to_str().to_owned(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {unsafe { geometry_msgs__msg__PoseStamped__Sequence__fini(&mut msg.poses) };
unsafe { geometry_msgs__msg__PoseStamped__Sequence__init(&mut msg.poses, self.poses.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.poses.data, msg.poses.size)};
for (t,s) in slice.iter_mut().zip(&self.poses) { s.copy_to_native(t);}
msg.behavior_tree.assign(&self.behavior_tree);
}



        }


                          
                          impl Default for Goal {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Goal>::new();
                                  Goal::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Result {

                              pub result: std_msgs::msg::Empty,

                          }

                          impl WrappedTypesupport for Result { 

            type CStruct = nav2_msgs__action__NavigateThroughPoses_Result; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Result() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateThroughPoses_Result {

                unsafe { nav2_msgs__action__NavigateThroughPoses_Result__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateThroughPoses_Result) -> () {

                unsafe { nav2_msgs__action__NavigateThroughPoses_Result__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Result {
  Result {
result: std_msgs::msg::Empty::from_native(&msg.result),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.result.copy_to_native(&mut msg.result);
}



        }


                          
                          impl Default for Result {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Result>::new();
                                  Result::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Feedback {

                              pub current_pose: geometry_msgs::msg::PoseStamped,
pub navigation_time: builtin_interfaces::msg::Duration,
pub estimated_time_remaining: builtin_interfaces::msg::Duration,
pub number_of_recoveries: i16,
pub distance_remaining: f32,
pub number_of_poses_remaining: i16,

                          }

                          impl WrappedTypesupport for Feedback { 

            type CStruct = nav2_msgs__action__NavigateThroughPoses_Feedback; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_Feedback() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateThroughPoses_Feedback {

                unsafe { nav2_msgs__action__NavigateThroughPoses_Feedback__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateThroughPoses_Feedback) -> () {

                unsafe { nav2_msgs__action__NavigateThroughPoses_Feedback__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Feedback {
  Feedback {
current_pose: geometry_msgs::msg::PoseStamped::from_native(&msg.current_pose),
navigation_time: builtin_interfaces::msg::Duration::from_native(&msg.navigation_time),
estimated_time_remaining: builtin_interfaces::msg::Duration::from_native(&msg.estimated_time_remaining),
number_of_recoveries: msg.number_of_recoveries,
distance_remaining: msg.distance_remaining,
number_of_poses_remaining: msg.number_of_poses_remaining,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.current_pose.copy_to_native(&mut msg.current_pose);
self.navigation_time.copy_to_native(&mut msg.navigation_time);
self.estimated_time_remaining.copy_to_native(&mut msg.estimated_time_remaining);
msg.number_of_recoveries = self.number_of_recoveries;
msg.distance_remaining = self.distance_remaining;
msg.number_of_poses_remaining = self.number_of_poses_remaining;
}



        }


                          
                          impl Default for Feedback {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Feedback>::new();
                                  Feedback::from_native(&msg_native)
                              }
                          }
             


                    #[allow(non_snake_case)]
    pub mod SendGoal {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub goal: nav2_msgs::action::NavigateThroughPoses::Goal,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__NavigateThroughPoses_SendGoal_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateThroughPoses_SendGoal_Request {

                unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateThroughPoses_SendGoal_Request) -> () {

                unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
goal: nav2_msgs::action::NavigateThroughPoses::Goal::from_native(&msg.goal),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub accepted: bool,
pub stamp: builtin_interfaces::msg::Time,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__NavigateThroughPoses_SendGoal_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_SendGoal_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateThroughPoses_SendGoal_Response {

                unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateThroughPoses_SendGoal_Response) -> () {

                unsafe { nav2_msgs__action__NavigateThroughPoses_SendGoal_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
accepted: msg.accepted,
stamp: builtin_interfaces::msg::Time::from_native(&msg.stamp),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.accepted = self.accepted;
self.stamp.copy_to_native(&mut msg.stamp);
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
    pub mod GetResult {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__NavigateThroughPoses_GetResult_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateThroughPoses_GetResult_Request {

                unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateThroughPoses_GetResult_Request) -> () {

                unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub status: i8,
pub result: nav2_msgs::action::NavigateThroughPoses::Result,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__NavigateThroughPoses_GetResult_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_GetResult_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateThroughPoses_GetResult_Response {

                unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateThroughPoses_GetResult_Response) -> () {

                unsafe { nav2_msgs__action__NavigateThroughPoses_GetResult_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
status: msg.status,
result: nav2_msgs::action::NavigateThroughPoses::Result::from_native(&msg.result),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.status = self.status;
self.result.copy_to_native(&mut msg.result);
}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct FeedbackMessage {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub feedback: nav2_msgs::action::NavigateThroughPoses::Feedback,

                          }

                          impl WrappedTypesupport for FeedbackMessage { 

            type CStruct = nav2_msgs__action__NavigateThroughPoses_FeedbackMessage; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateThroughPoses_FeedbackMessage() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateThroughPoses_FeedbackMessage {

                unsafe { nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateThroughPoses_FeedbackMessage) -> () {

                unsafe { nav2_msgs__action__NavigateThroughPoses_FeedbackMessage__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> FeedbackMessage {
  FeedbackMessage {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
feedback: nav2_msgs::action::NavigateThroughPoses::Feedback::from_native(&msg.feedback),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
self.feedback.copy_to_native(&mut msg.feedback);
}



        }


                          
                          impl Default for FeedbackMessage {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<FeedbackMessage>::new();
                                  FeedbackMessage::from_native(&msg_native)
                              }
                          }
             


                        }
#[allow(non_snake_case)]
    pub mod NavigateToPose {
    use super::super::super::*;

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
                    &*rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__NavigateToPose()
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

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Goal {

                              pub pose: geometry_msgs::msg::PoseStamped,
pub behavior_tree: std::string::String,

                          }

                          impl WrappedTypesupport for Goal { 

            type CStruct = nav2_msgs__action__NavigateToPose_Goal; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Goal() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateToPose_Goal {

                unsafe { nav2_msgs__action__NavigateToPose_Goal__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateToPose_Goal) -> () {

                unsafe { nav2_msgs__action__NavigateToPose_Goal__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Goal {
  Goal {
pose: geometry_msgs::msg::PoseStamped::from_native(&msg.pose),
behavior_tree: msg.behavior_tree.to_str().to_owned(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.pose.copy_to_native(&mut msg.pose);
msg.behavior_tree.assign(&self.behavior_tree);
}



        }


                          
                          impl Default for Goal {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Goal>::new();
                                  Goal::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Result {

                              pub result: std_msgs::msg::Empty,

                          }

                          impl WrappedTypesupport for Result { 

            type CStruct = nav2_msgs__action__NavigateToPose_Result; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Result() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateToPose_Result {

                unsafe { nav2_msgs__action__NavigateToPose_Result__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateToPose_Result) -> () {

                unsafe { nav2_msgs__action__NavigateToPose_Result__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Result {
  Result {
result: std_msgs::msg::Empty::from_native(&msg.result),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.result.copy_to_native(&mut msg.result);
}



        }


                          
                          impl Default for Result {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Result>::new();
                                  Result::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Feedback {

                              pub current_pose: geometry_msgs::msg::PoseStamped,
pub navigation_time: builtin_interfaces::msg::Duration,
pub estimated_time_remaining: builtin_interfaces::msg::Duration,
pub number_of_recoveries: i16,
pub distance_remaining: f32,

                          }

                          impl WrappedTypesupport for Feedback { 

            type CStruct = nav2_msgs__action__NavigateToPose_Feedback; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_Feedback() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateToPose_Feedback {

                unsafe { nav2_msgs__action__NavigateToPose_Feedback__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateToPose_Feedback) -> () {

                unsafe { nav2_msgs__action__NavigateToPose_Feedback__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Feedback {
  Feedback {
current_pose: geometry_msgs::msg::PoseStamped::from_native(&msg.current_pose),
navigation_time: builtin_interfaces::msg::Duration::from_native(&msg.navigation_time),
estimated_time_remaining: builtin_interfaces::msg::Duration::from_native(&msg.estimated_time_remaining),
number_of_recoveries: msg.number_of_recoveries,
distance_remaining: msg.distance_remaining,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.current_pose.copy_to_native(&mut msg.current_pose);
self.navigation_time.copy_to_native(&mut msg.navigation_time);
self.estimated_time_remaining.copy_to_native(&mut msg.estimated_time_remaining);
msg.number_of_recoveries = self.number_of_recoveries;
msg.distance_remaining = self.distance_remaining;
}



        }


                          
                          impl Default for Feedback {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Feedback>::new();
                                  Feedback::from_native(&msg_native)
                              }
                          }
             


                    #[allow(non_snake_case)]
    pub mod SendGoal {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub goal: nav2_msgs::action::NavigateToPose::Goal,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__NavigateToPose_SendGoal_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateToPose_SendGoal_Request {

                unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateToPose_SendGoal_Request) -> () {

                unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
goal: nav2_msgs::action::NavigateToPose::Goal::from_native(&msg.goal),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub accepted: bool,
pub stamp: builtin_interfaces::msg::Time,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__NavigateToPose_SendGoal_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_SendGoal_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateToPose_SendGoal_Response {

                unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateToPose_SendGoal_Response) -> () {

                unsafe { nav2_msgs__action__NavigateToPose_SendGoal_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
accepted: msg.accepted,
stamp: builtin_interfaces::msg::Time::from_native(&msg.stamp),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.accepted = self.accepted;
self.stamp.copy_to_native(&mut msg.stamp);
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
    pub mod GetResult {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__NavigateToPose_GetResult_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateToPose_GetResult_Request {

                unsafe { nav2_msgs__action__NavigateToPose_GetResult_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateToPose_GetResult_Request) -> () {

                unsafe { nav2_msgs__action__NavigateToPose_GetResult_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub status: i8,
pub result: nav2_msgs::action::NavigateToPose::Result,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__NavigateToPose_GetResult_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_GetResult_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateToPose_GetResult_Response {

                unsafe { nav2_msgs__action__NavigateToPose_GetResult_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateToPose_GetResult_Response) -> () {

                unsafe { nav2_msgs__action__NavigateToPose_GetResult_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
status: msg.status,
result: nav2_msgs::action::NavigateToPose::Result::from_native(&msg.result),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.status = self.status;
self.result.copy_to_native(&mut msg.result);
}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct FeedbackMessage {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub feedback: nav2_msgs::action::NavigateToPose::Feedback,

                          }

                          impl WrappedTypesupport for FeedbackMessage { 

            type CStruct = nav2_msgs__action__NavigateToPose_FeedbackMessage; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__NavigateToPose_FeedbackMessage() }
            }

            fn create_msg() -> *mut nav2_msgs__action__NavigateToPose_FeedbackMessage {

                unsafe { nav2_msgs__action__NavigateToPose_FeedbackMessage__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__NavigateToPose_FeedbackMessage) -> () {

                unsafe { nav2_msgs__action__NavigateToPose_FeedbackMessage__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> FeedbackMessage {
  FeedbackMessage {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
feedback: nav2_msgs::action::NavigateToPose::Feedback::from_native(&msg.feedback),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
self.feedback.copy_to_native(&mut msg.feedback);
}



        }


                          
                          impl Default for FeedbackMessage {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<FeedbackMessage>::new();
                                  FeedbackMessage::from_native(&msg_native)
                              }
                          }
             


                        }
#[allow(non_snake_case)]
    pub mod Spin {
    use super::super::super::*;

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
                    &*rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__Spin()
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

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Goal {

                              pub target_yaw: f32,

                          }

                          impl WrappedTypesupport for Goal { 

            type CStruct = nav2_msgs__action__Spin_Goal; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Goal() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Spin_Goal {

                unsafe { nav2_msgs__action__Spin_Goal__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Spin_Goal) -> () {

                unsafe { nav2_msgs__action__Spin_Goal__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Goal {
  Goal {
target_yaw: msg.target_yaw,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.target_yaw = self.target_yaw;
}



        }


                          
                          impl Default for Goal {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Goal>::new();
                                  Goal::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Result {

                              pub total_elapsed_time: builtin_interfaces::msg::Duration,

                          }

                          impl WrappedTypesupport for Result { 

            type CStruct = nav2_msgs__action__Spin_Result; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Result() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Spin_Result {

                unsafe { nav2_msgs__action__Spin_Result__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Spin_Result) -> () {

                unsafe { nav2_msgs__action__Spin_Result__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Result {
  Result {
total_elapsed_time: builtin_interfaces::msg::Duration::from_native(&msg.total_elapsed_time),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.total_elapsed_time.copy_to_native(&mut msg.total_elapsed_time);
}



        }


                          
                          impl Default for Result {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Result>::new();
                                  Result::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Feedback {

                              pub angular_distance_traveled: f32,

                          }

                          impl WrappedTypesupport for Feedback { 

            type CStruct = nav2_msgs__action__Spin_Feedback; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_Feedback() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Spin_Feedback {

                unsafe { nav2_msgs__action__Spin_Feedback__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Spin_Feedback) -> () {

                unsafe { nav2_msgs__action__Spin_Feedback__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Feedback {
  Feedback {
angular_distance_traveled: msg.angular_distance_traveled,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.angular_distance_traveled = self.angular_distance_traveled;
}



        }


                          
                          impl Default for Feedback {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Feedback>::new();
                                  Feedback::from_native(&msg_native)
                              }
                          }
             


                    #[allow(non_snake_case)]
    pub mod SendGoal {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Spin_SendGoal()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub goal: nav2_msgs::action::Spin::Goal,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__Spin_SendGoal_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_SendGoal_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Spin_SendGoal_Request {

                unsafe { nav2_msgs__action__Spin_SendGoal_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Spin_SendGoal_Request) -> () {

                unsafe { nav2_msgs__action__Spin_SendGoal_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
goal: nav2_msgs::action::Spin::Goal::from_native(&msg.goal),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub accepted: bool,
pub stamp: builtin_interfaces::msg::Time,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__Spin_SendGoal_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_SendGoal_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Spin_SendGoal_Response {

                unsafe { nav2_msgs__action__Spin_SendGoal_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Spin_SendGoal_Response) -> () {

                unsafe { nav2_msgs__action__Spin_SendGoal_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
accepted: msg.accepted,
stamp: builtin_interfaces::msg::Time::from_native(&msg.stamp),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.accepted = self.accepted;
self.stamp.copy_to_native(&mut msg.stamp);
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
    pub mod GetResult {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Spin_GetResult()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__Spin_GetResult_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_GetResult_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Spin_GetResult_Request {

                unsafe { nav2_msgs__action__Spin_GetResult_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Spin_GetResult_Request) -> () {

                unsafe { nav2_msgs__action__Spin_GetResult_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub status: i8,
pub result: nav2_msgs::action::Spin::Result,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__Spin_GetResult_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_GetResult_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Spin_GetResult_Response {

                unsafe { nav2_msgs__action__Spin_GetResult_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Spin_GetResult_Response) -> () {

                unsafe { nav2_msgs__action__Spin_GetResult_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
status: msg.status,
result: nav2_msgs::action::Spin::Result::from_native(&msg.result),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.status = self.status;
self.result.copy_to_native(&mut msg.result);
}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct FeedbackMessage {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub feedback: nav2_msgs::action::Spin::Feedback,

                          }

                          impl WrappedTypesupport for FeedbackMessage { 

            type CStruct = nav2_msgs__action__Spin_FeedbackMessage; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Spin_FeedbackMessage() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Spin_FeedbackMessage {

                unsafe { nav2_msgs__action__Spin_FeedbackMessage__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Spin_FeedbackMessage) -> () {

                unsafe { nav2_msgs__action__Spin_FeedbackMessage__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> FeedbackMessage {
  FeedbackMessage {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
feedback: nav2_msgs::action::Spin::Feedback::from_native(&msg.feedback),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
self.feedback.copy_to_native(&mut msg.feedback);
}



        }


                          
                          impl Default for FeedbackMessage {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<FeedbackMessage>::new();
                                  FeedbackMessage::from_native(&msg_native)
                              }
                          }
             


                        }
#[allow(non_snake_case)]
    pub mod Wait {
    use super::super::super::*;

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
                    &*rosidl_typesupport_c__get_action_type_support_handle__nav2_msgs__action__Wait()
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

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Goal {

                              pub time: builtin_interfaces::msg::Duration,

                          }

                          impl WrappedTypesupport for Goal { 

            type CStruct = nav2_msgs__action__Wait_Goal; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Goal() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Wait_Goal {

                unsafe { nav2_msgs__action__Wait_Goal__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Wait_Goal) -> () {

                unsafe { nav2_msgs__action__Wait_Goal__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Goal {
  Goal {
time: builtin_interfaces::msg::Duration::from_native(&msg.time),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.time.copy_to_native(&mut msg.time);
}



        }


                          
                          impl Default for Goal {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Goal>::new();
                                  Goal::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Result {

                              pub total_elapsed_time: builtin_interfaces::msg::Duration,

                          }

                          impl WrappedTypesupport for Result { 

            type CStruct = nav2_msgs__action__Wait_Result; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Result() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Wait_Result {

                unsafe { nav2_msgs__action__Wait_Result__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Wait_Result) -> () {

                unsafe { nav2_msgs__action__Wait_Result__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Result {
  Result {
total_elapsed_time: builtin_interfaces::msg::Duration::from_native(&msg.total_elapsed_time),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.total_elapsed_time.copy_to_native(&mut msg.total_elapsed_time);
}



        }


                          
                          impl Default for Result {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Result>::new();
                                  Result::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Feedback {

                              pub time_left: builtin_interfaces::msg::Duration,

                          }

                          impl WrappedTypesupport for Feedback { 

            type CStruct = nav2_msgs__action__Wait_Feedback; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_Feedback() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Wait_Feedback {

                unsafe { nav2_msgs__action__Wait_Feedback__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Wait_Feedback) -> () {

                unsafe { nav2_msgs__action__Wait_Feedback__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Feedback {
  Feedback {
time_left: builtin_interfaces::msg::Duration::from_native(&msg.time_left),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.time_left.copy_to_native(&mut msg.time_left);
}



        }


                          
                          impl Default for Feedback {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Feedback>::new();
                                  Feedback::from_native(&msg_native)
                              }
                          }
             


                    #[allow(non_snake_case)]
    pub mod SendGoal {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Wait_SendGoal()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub goal: nav2_msgs::action::Wait::Goal,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__Wait_SendGoal_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_SendGoal_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Wait_SendGoal_Request {

                unsafe { nav2_msgs__action__Wait_SendGoal_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Wait_SendGoal_Request) -> () {

                unsafe { nav2_msgs__action__Wait_SendGoal_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
goal: nav2_msgs::action::Wait::Goal::from_native(&msg.goal),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub accepted: bool,
pub stamp: builtin_interfaces::msg::Time,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__Wait_SendGoal_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_SendGoal_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Wait_SendGoal_Response {

                unsafe { nav2_msgs__action__Wait_SendGoal_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Wait_SendGoal_Response) -> () {

                unsafe { nav2_msgs__action__Wait_SendGoal_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
accepted: msg.accepted,
stamp: builtin_interfaces::msg::Time::from_native(&msg.stamp),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.accepted = self.accepted;
self.stamp.copy_to_native(&mut msg.stamp);
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
    pub mod GetResult {
    use super::super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__action__Wait_GetResult()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub goal_id: unique_identifier_msgs::msg::UUID,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__action__Wait_GetResult_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_GetResult_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Wait_GetResult_Request {

                unsafe { nav2_msgs__action__Wait_GetResult_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Wait_GetResult_Request) -> () {

                unsafe { nav2_msgs__action__Wait_GetResult_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
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

                              pub status: i8,
pub result: nav2_msgs::action::Wait::Result,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__action__Wait_GetResult_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_GetResult_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Wait_GetResult_Response {

                unsafe { nav2_msgs__action__Wait_GetResult_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Wait_GetResult_Response) -> () {

                unsafe { nav2_msgs__action__Wait_GetResult_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
status: msg.status,
result: nav2_msgs::action::Wait::Result::from_native(&msg.result),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.status = self.status;
self.result.copy_to_native(&mut msg.result);
}



        }


                          
                          impl Default for Response {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Response>::new();
                                  Response::from_native(&msg_native)
                              }
                          }
             


                        }

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct FeedbackMessage {

                              pub goal_id: unique_identifier_msgs::msg::UUID,
pub feedback: nav2_msgs::action::Wait::Feedback,

                          }

                          impl WrappedTypesupport for FeedbackMessage { 

            type CStruct = nav2_msgs__action__Wait_FeedbackMessage; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__action__Wait_FeedbackMessage() }
            }

            fn create_msg() -> *mut nav2_msgs__action__Wait_FeedbackMessage {

                unsafe { nav2_msgs__action__Wait_FeedbackMessage__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__action__Wait_FeedbackMessage) -> () {

                unsafe { nav2_msgs__action__Wait_FeedbackMessage__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> FeedbackMessage {
  FeedbackMessage {
goal_id: unique_identifier_msgs::msg::UUID::from_native(&msg.goal_id),
feedback: nav2_msgs::action::Wait::Feedback::from_native(&msg.feedback),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.goal_id.copy_to_native(&mut msg.goal_id);
self.feedback.copy_to_native(&mut msg.feedback);
}



        }


                          
                          impl Default for FeedbackMessage {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<FeedbackMessage>::new();
                                  FeedbackMessage::from_native(&msg_native)
                              }
                          }
             


                        }
  }
  pub mod srv {
#[allow(non_snake_case)]
    pub mod ClearCostmapAroundRobot {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub reset_distance: f32,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__srv__ClearCostmapAroundRobot_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__srv__ClearCostmapAroundRobot_Request {

                unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__srv__ClearCostmapAroundRobot_Request) -> () {

                unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
reset_distance: msg.reset_distance,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.reset_distance = self.reset_distance;
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

                              pub response: std_msgs::msg::Empty,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__srv__ClearCostmapAroundRobot_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapAroundRobot_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__srv__ClearCostmapAroundRobot_Response {

                unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__srv__ClearCostmapAroundRobot_Response) -> () {

                unsafe { nav2_msgs__srv__ClearCostmapAroundRobot_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
response: std_msgs::msg::Empty::from_native(&msg.response),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.response.copy_to_native(&mut msg.response);
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
    pub mod ClearCostmapExceptRegion {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub reset_distance: f32,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__srv__ClearCostmapExceptRegion_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__srv__ClearCostmapExceptRegion_Request {

                unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__srv__ClearCostmapExceptRegion_Request) -> () {

                unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
reset_distance: msg.reset_distance,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.reset_distance = self.reset_distance;
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

                              pub response: std_msgs::msg::Empty,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__srv__ClearCostmapExceptRegion_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearCostmapExceptRegion_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__srv__ClearCostmapExceptRegion_Response {

                unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__srv__ClearCostmapExceptRegion_Response) -> () {

                unsafe { nav2_msgs__srv__ClearCostmapExceptRegion_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
response: std_msgs::msg::Empty::from_native(&msg.response),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.response.copy_to_native(&mut msg.response);
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
    pub mod ClearEntireCostmap {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ClearEntireCostmap()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub request: std_msgs::msg::Empty,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__srv__ClearEntireCostmap_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearEntireCostmap_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__srv__ClearEntireCostmap_Request {

                unsafe { nav2_msgs__srv__ClearEntireCostmap_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__srv__ClearEntireCostmap_Request) -> () {

                unsafe { nav2_msgs__srv__ClearEntireCostmap_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
request: std_msgs::msg::Empty::from_native(&msg.request),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.request.copy_to_native(&mut msg.request);
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

                              pub response: std_msgs::msg::Empty,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__srv__ClearEntireCostmap_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ClearEntireCostmap_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__srv__ClearEntireCostmap_Response {

                unsafe { nav2_msgs__srv__ClearEntireCostmap_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__srv__ClearEntireCostmap_Response) -> () {

                unsafe { nav2_msgs__srv__ClearEntireCostmap_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
response: std_msgs::msg::Empty::from_native(&msg.response),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.response.copy_to_native(&mut msg.response);
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
    pub mod GetCostmap {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__GetCostmap()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub specs: nav2_msgs::msg::CostmapMetaData,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__srv__GetCostmap_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__GetCostmap_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__srv__GetCostmap_Request {

                unsafe { nav2_msgs__srv__GetCostmap_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__srv__GetCostmap_Request) -> () {

                unsafe { nav2_msgs__srv__GetCostmap_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
specs: nav2_msgs::msg::CostmapMetaData::from_native(&msg.specs),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.specs.copy_to_native(&mut msg.specs);
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

                              pub map: nav2_msgs::msg::Costmap,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__srv__GetCostmap_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__GetCostmap_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__srv__GetCostmap_Response {

                unsafe { nav2_msgs__srv__GetCostmap_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__srv__GetCostmap_Response) -> () {

                unsafe { nav2_msgs__srv__GetCostmap_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
map: nav2_msgs::msg::Costmap::from_native(&msg.map),
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
#[allow(non_snake_case)]
    pub mod LoadMap {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__LoadMap()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub map_url: std::string::String,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__srv__LoadMap_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__LoadMap_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__srv__LoadMap_Request {

                unsafe { nav2_msgs__srv__LoadMap_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__srv__LoadMap_Request) -> () {

                unsafe { nav2_msgs__srv__LoadMap_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
map_url: msg.map_url.to_str().to_owned(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.map_url.assign(&self.map_url);
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

                              pub map: nav_msgs::msg::OccupancyGrid,
pub result: u8,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__srv__LoadMap_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__LoadMap_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__srv__LoadMap_Response {

                unsafe { nav2_msgs__srv__LoadMap_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__srv__LoadMap_Response) -> () {

                unsafe { nav2_msgs__srv__LoadMap_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
map: nav_msgs::msg::OccupancyGrid::from_native(&msg.map),
result: msg.result,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.map.copy_to_native(&mut msg.map);
msg.result = self.result;
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
    pub mod ManageLifecycleNodes {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub command: u8,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__srv__ManageLifecycleNodes_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__srv__ManageLifecycleNodes_Request {

                unsafe { nav2_msgs__srv__ManageLifecycleNodes_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__srv__ManageLifecycleNodes_Request) -> () {

                unsafe { nav2_msgs__srv__ManageLifecycleNodes_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
command: msg.command,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.command = self.command;
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

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__srv__ManageLifecycleNodes_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__ManageLifecycleNodes_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__srv__ManageLifecycleNodes_Response {

                unsafe { nav2_msgs__srv__ManageLifecycleNodes_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__srv__ManageLifecycleNodes_Response) -> () {

                unsafe { nav2_msgs__srv__ManageLifecycleNodes_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
success: msg.success,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.success = self.success;
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
    pub mod SaveMap {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__nav2_msgs__srv__SaveMap()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub map_topic: std::string::String,
pub map_url: std::string::String,
pub image_format: std::string::String,
pub map_mode: std::string::String,
pub free_thresh: f32,
pub occupied_thresh: f32,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = nav2_msgs__srv__SaveMap_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SaveMap_Request() }
            }

            fn create_msg() -> *mut nav2_msgs__srv__SaveMap_Request {

                unsafe { nav2_msgs__srv__SaveMap_Request__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__srv__SaveMap_Request) -> () {

                unsafe { nav2_msgs__srv__SaveMap_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
map_topic: msg.map_topic.to_str().to_owned(),
map_url: msg.map_url.to_str().to_owned(),
image_format: msg.image_format.to_str().to_owned(),
map_mode: msg.map_mode.to_str().to_owned(),
free_thresh: msg.free_thresh,
occupied_thresh: msg.occupied_thresh,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.map_topic.assign(&self.map_topic);
msg.map_url.assign(&self.map_url);
msg.image_format.assign(&self.image_format);
msg.map_mode.assign(&self.map_mode);
msg.free_thresh = self.free_thresh;
msg.occupied_thresh = self.occupied_thresh;
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

                              pub result: bool,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = nav2_msgs__srv__SaveMap_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__nav2_msgs__srv__SaveMap_Response() }
            }

            fn create_msg() -> *mut nav2_msgs__srv__SaveMap_Response {

                unsafe { nav2_msgs__srv__SaveMap_Response__create() }

            }

            fn destroy_msg(msg: *mut nav2_msgs__srv__SaveMap_Response) -> () {

                unsafe { nav2_msgs__srv__SaveMap_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
result: msg.result,
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.result = self.result;
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
