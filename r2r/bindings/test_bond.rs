pub mod srv { # [allow (non_snake_case)] pub mod TestBond { use super :: super :: super :: * ; # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] pub struct Service () ; impl WrappedServiceTypeSupport for Service { type Request = Request ; type Response = Response ; fn get_ts () -> & 'static rosidl_service_type_support_t { unsafe { & * rosidl_typesupport_c__get_service_type_support_handle__test_bond__srv__TestBond () } } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct Request { pub topic : std :: string :: String , pub id : std :: string :: String , pub delay_connect : u32 , pub delay_death : u32 , pub inhibit_death : bool , pub inhibit_death_message : bool , pub a : i64 , pub b : i64 } impl WrappedTypesupport for Request { type CStruct = test_bond__srv__TestBond_Request ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__test_bond__srv__TestBond_Request () } } fn create_msg () -> * mut test_bond__srv__TestBond_Request { unsafe { test_bond__srv__TestBond_Request__create () } } fn destroy_msg (msg : * mut test_bond__srv__TestBond_Request) -> () { unsafe { test_bond__srv__TestBond_Request__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> Request { Request { topic : msg . topic . to_str () . to_owned () , id : msg . id . to_str () . to_owned () , delay_connect : msg . delay_connect , delay_death : msg . delay_death , inhibit_death : msg . inhibit_death , inhibit_death_message : msg . inhibit_death_message , a : msg . a , b : msg . b , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { msg . topic . assign (& self . topic) ; msg . id . assign (& self . id) ; msg . delay_connect = self . delay_connect ; msg . delay_death = self . delay_death ; msg . inhibit_death = self . inhibit_death ; msg . inhibit_death_message = self . inhibit_death_message ; msg . a = self . a ; msg . b = self . b ; } } impl Default for Request { fn default () -> Self { let msg_native = WrappedNativeMsg :: < Request > :: new () ; Request :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct Response { pub sum : i64 } impl WrappedTypesupport for Response { type CStruct = test_bond__srv__TestBond_Response ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__test_bond__srv__TestBond_Response () } } fn create_msg () -> * mut test_bond__srv__TestBond_Response { unsafe { test_bond__srv__TestBond_Response__create () } } fn destroy_msg (msg : * mut test_bond__srv__TestBond_Response) -> () { unsafe { test_bond__srv__TestBond_Response__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> Response { Response { sum : msg . sum , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { msg . sum = self . sum ; } } impl Default for Response { fn default () -> Self { let msg_native = WrappedNativeMsg :: < Response > :: new () ; Response :: from_native (& msg_native) } } } }