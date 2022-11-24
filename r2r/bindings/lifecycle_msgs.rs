  pub mod msg {
    use super::super::*;

                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct State {

                              pub id: u8,
pub label: std::string::String,

                          }

                          impl WrappedTypesupport for State { 

            type CStruct = lifecycle_msgs__msg__State; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__lifecycle_msgs__msg__State() }
            }

            fn create_msg() -> *mut lifecycle_msgs__msg__State {

                unsafe { lifecycle_msgs__msg__State__create() }

            }

            fn destroy_msg(msg: *mut lifecycle_msgs__msg__State) -> () {

                unsafe { lifecycle_msgs__msg__State__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> State {
  State {
id: msg.id,
label: msg.label.to_str().to_owned(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.id = self.id;
msg.label.assign(&self.label);
}



        }


                          
                          impl Default for State {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<State>::new();
                                  State::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Transition {

                              pub id: u8,
pub label: std::string::String,

                          }

                          impl WrappedTypesupport for Transition { 

            type CStruct = lifecycle_msgs__msg__Transition; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__lifecycle_msgs__msg__Transition() }
            }

            fn create_msg() -> *mut lifecycle_msgs__msg__Transition {

                unsafe { lifecycle_msgs__msg__Transition__create() }

            }

            fn destroy_msg(msg: *mut lifecycle_msgs__msg__Transition) -> () {

                unsafe { lifecycle_msgs__msg__Transition__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Transition {
  Transition {
id: msg.id,
label: msg.label.to_str().to_owned(),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.id = self.id;
msg.label.assign(&self.label);
}



        }


                          
                          impl Default for Transition {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<Transition>::new();
                                  Transition::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct TransitionDescription {

                              pub transition: lifecycle_msgs::msg::Transition,
pub start_state: lifecycle_msgs::msg::State,
pub goal_state: lifecycle_msgs::msg::State,

                          }

                          impl WrappedTypesupport for TransitionDescription { 

            type CStruct = lifecycle_msgs__msg__TransitionDescription; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__lifecycle_msgs__msg__TransitionDescription() }
            }

            fn create_msg() -> *mut lifecycle_msgs__msg__TransitionDescription {

                unsafe { lifecycle_msgs__msg__TransitionDescription__create() }

            }

            fn destroy_msg(msg: *mut lifecycle_msgs__msg__TransitionDescription) -> () {

                unsafe { lifecycle_msgs__msg__TransitionDescription__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> TransitionDescription {
  TransitionDescription {
transition: lifecycle_msgs::msg::Transition::from_native(&msg.transition),
start_state: lifecycle_msgs::msg::State::from_native(&msg.start_state),
goal_state: lifecycle_msgs::msg::State::from_native(&msg.goal_state),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.transition.copy_to_native(&mut msg.transition);
self.start_state.copy_to_native(&mut msg.start_state);
self.goal_state.copy_to_native(&mut msg.goal_state);
}



        }


                          
                          impl Default for TransitionDescription {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<TransitionDescription>::new();
                                  TransitionDescription::from_native(&msg_native)
                              }
                          }
             


                    
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct TransitionEvent {

                              pub timestamp: u64,
pub transition: lifecycle_msgs::msg::Transition,
pub start_state: lifecycle_msgs::msg::State,
pub goal_state: lifecycle_msgs::msg::State,

                          }

                          impl WrappedTypesupport for TransitionEvent { 

            type CStruct = lifecycle_msgs__msg__TransitionEvent; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__lifecycle_msgs__msg__TransitionEvent() }
            }

            fn create_msg() -> *mut lifecycle_msgs__msg__TransitionEvent {

                unsafe { lifecycle_msgs__msg__TransitionEvent__create() }

            }

            fn destroy_msg(msg: *mut lifecycle_msgs__msg__TransitionEvent) -> () {

                unsafe { lifecycle_msgs__msg__TransitionEvent__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> TransitionEvent {
  TransitionEvent {
timestamp: msg.timestamp,
transition: lifecycle_msgs::msg::Transition::from_native(&msg.transition),
start_state: lifecycle_msgs::msg::State::from_native(&msg.start_state),
goal_state: lifecycle_msgs::msg::State::from_native(&msg.goal_state),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {msg.timestamp = self.timestamp;
self.transition.copy_to_native(&mut msg.transition);
self.start_state.copy_to_native(&mut msg.start_state);
self.goal_state.copy_to_native(&mut msg.goal_state);
}



        }


                          
                          impl Default for TransitionEvent {
                              fn default() -> Self {
                                  let msg_native = WrappedNativeMsg::<TransitionEvent>::new();
                                  TransitionEvent::from_native(&msg_native)
                              }
                          }
             


                      }
  pub mod srv {
#[allow(non_snake_case)]
    pub mod ChangeState {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__lifecycle_msgs__srv__ChangeState()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              pub transition: lifecycle_msgs::msg::Transition,

                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = lifecycle_msgs__srv__ChangeState_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__lifecycle_msgs__srv__ChangeState_Request() }
            }

            fn create_msg() -> *mut lifecycle_msgs__srv__ChangeState_Request {

                unsafe { lifecycle_msgs__srv__ChangeState_Request__create() }

            }

            fn destroy_msg(msg: *mut lifecycle_msgs__srv__ChangeState_Request) -> () {

                unsafe { lifecycle_msgs__srv__ChangeState_Request__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Request {
  Request {
transition: lifecycle_msgs::msg::Transition::from_native(&msg.transition),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.transition.copy_to_native(&mut msg.transition);
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

            type CStruct = lifecycle_msgs__srv__ChangeState_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__lifecycle_msgs__srv__ChangeState_Response() }
            }

            fn create_msg() -> *mut lifecycle_msgs__srv__ChangeState_Response {

                unsafe { lifecycle_msgs__srv__ChangeState_Response__create() }

            }

            fn destroy_msg(msg: *mut lifecycle_msgs__srv__ChangeState_Response) -> () {

                unsafe { lifecycle_msgs__srv__ChangeState_Response__destroy(msg) };

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
    pub mod GetAvailableStates {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__lifecycle_msgs__srv__GetAvailableStates()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              
                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = lifecycle_msgs__srv__GetAvailableStates_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__lifecycle_msgs__srv__GetAvailableStates_Request() }
            }

            fn create_msg() -> *mut lifecycle_msgs__srv__GetAvailableStates_Request {

                unsafe { lifecycle_msgs__srv__GetAvailableStates_Request__create() }

            }

            fn destroy_msg(msg: *mut lifecycle_msgs__srv__GetAvailableStates_Request) -> () {

                unsafe { lifecycle_msgs__srv__GetAvailableStates_Request__destroy(msg) };

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

                              pub available_states: Vec<lifecycle_msgs::msg::State>,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = lifecycle_msgs__srv__GetAvailableStates_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__lifecycle_msgs__srv__GetAvailableStates_Response() }
            }

            fn create_msg() -> *mut lifecycle_msgs__srv__GetAvailableStates_Response {

                unsafe { lifecycle_msgs__srv__GetAvailableStates_Response__create() }

            }

            fn destroy_msg(msg: *mut lifecycle_msgs__srv__GetAvailableStates_Response) -> () {

                unsafe { lifecycle_msgs__srv__GetAvailableStates_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
// is_upper_bound_: false
// member.array_size_ : 0
available_states : {
let mut temp = Vec::with_capacity(msg.available_states.size);
let slice = unsafe { std::slice::from_raw_parts(msg.available_states.data, msg.available_states.size)};
for s in slice { temp.push(lifecycle_msgs::msg::State::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {unsafe { lifecycle_msgs__msg__State__Sequence__fini(&mut msg.available_states) };
unsafe { lifecycle_msgs__msg__State__Sequence__init(&mut msg.available_states, self.available_states.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.available_states.data, msg.available_states.size)};
for (t,s) in slice.iter_mut().zip(&self.available_states) { s.copy_to_native(t);}
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
    pub mod GetAvailableTransitions {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__lifecycle_msgs__srv__GetAvailableTransitions()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              
                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = lifecycle_msgs__srv__GetAvailableTransitions_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__lifecycle_msgs__srv__GetAvailableTransitions_Request() }
            }

            fn create_msg() -> *mut lifecycle_msgs__srv__GetAvailableTransitions_Request {

                unsafe { lifecycle_msgs__srv__GetAvailableTransitions_Request__create() }

            }

            fn destroy_msg(msg: *mut lifecycle_msgs__srv__GetAvailableTransitions_Request) -> () {

                unsafe { lifecycle_msgs__srv__GetAvailableTransitions_Request__destroy(msg) };

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

                              pub available_transitions: Vec<lifecycle_msgs::msg::TransitionDescription>,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = lifecycle_msgs__srv__GetAvailableTransitions_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__lifecycle_msgs__srv__GetAvailableTransitions_Response() }
            }

            fn create_msg() -> *mut lifecycle_msgs__srv__GetAvailableTransitions_Response {

                unsafe { lifecycle_msgs__srv__GetAvailableTransitions_Response__create() }

            }

            fn destroy_msg(msg: *mut lifecycle_msgs__srv__GetAvailableTransitions_Response) -> () {

                unsafe { lifecycle_msgs__srv__GetAvailableTransitions_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
// is_upper_bound_: false
// member.array_size_ : 0
available_transitions : {
let mut temp = Vec::with_capacity(msg.available_transitions.size);
let slice = unsafe { std::slice::from_raw_parts(msg.available_transitions.data, msg.available_transitions.size)};
for s in slice { temp.push(lifecycle_msgs::msg::TransitionDescription::from_native(s)); }
temp },
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {unsafe { lifecycle_msgs__msg__TransitionDescription__Sequence__fini(&mut msg.available_transitions) };
unsafe { lifecycle_msgs__msg__TransitionDescription__Sequence__init(&mut msg.available_transitions, self.available_transitions.len()) };
let slice = unsafe { std::slice::from_raw_parts_mut(msg.available_transitions.data, msg.available_transitions.size)};
for (t,s) in slice.iter_mut().zip(&self.available_transitions) { s.copy_to_native(t);}
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
    pub mod GetState {
    use super::super::super::*;

        #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__lifecycle_msgs__srv__GetState()
                }
            }
        }

            
                          #[derive(Clone,Debug,PartialEq,Serialize,Deserialize)]
                          #[serde(default)]
                          pub struct Request {

                              
                          }

                          impl WrappedTypesupport for Request { 

            type CStruct = lifecycle_msgs__srv__GetState_Request; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__lifecycle_msgs__srv__GetState_Request() }
            }

            fn create_msg() -> *mut lifecycle_msgs__srv__GetState_Request {

                unsafe { lifecycle_msgs__srv__GetState_Request__create() }

            }

            fn destroy_msg(msg: *mut lifecycle_msgs__srv__GetState_Request) -> () {

                unsafe { lifecycle_msgs__srv__GetState_Request__destroy(msg) };

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

                              pub current_state: lifecycle_msgs::msg::State,

                          }

                          impl WrappedTypesupport for Response { 

            type CStruct = lifecycle_msgs__srv__GetState_Response; 


            fn get_ts() -> &'static rosidl_message_type_support_t { 

                unsafe { &*rosidl_typesupport_c__get_message_type_support_handle__lifecycle_msgs__srv__GetState_Response() }
            }

            fn create_msg() -> *mut lifecycle_msgs__srv__GetState_Response {

                unsafe { lifecycle_msgs__srv__GetState_Response__create() }

            }

            fn destroy_msg(msg: *mut lifecycle_msgs__srv__GetState_Response) -> () {

                unsafe { lifecycle_msgs__srv__GetState_Response__destroy(msg) };

            }

            fn from_native(msg: &Self::CStruct) -> Response {
  Response {
current_state: lifecycle_msgs::msg::State::from_native(&msg.current_state),
      }
    }



            fn copy_to_native(&self, msg: &mut Self::CStruct) {self.current_state.copy_to_native(&mut msg.current_state);
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
