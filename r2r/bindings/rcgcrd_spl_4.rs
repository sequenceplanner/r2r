pub mod msg { use super :: super :: * ; # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct RCGCRD { pub player_num : u8 , pub team_num : u8 , pub fallen : u8 , pub pose : Vec < f32 > , pub ball_age : f32 , pub ball : Vec < f32 > } impl WrappedTypesupport for RCGCRD { type CStruct = rcgcrd_spl_4__msg__RCGCRD ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__rcgcrd_spl_4__msg__RCGCRD () } } fn create_msg () -> * mut rcgcrd_spl_4__msg__RCGCRD { unsafe { rcgcrd_spl_4__msg__RCGCRD__create () } } fn destroy_msg (msg : * mut rcgcrd_spl_4__msg__RCGCRD) -> () { unsafe { rcgcrd_spl_4__msg__RCGCRD__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> RCGCRD { RCGCRD { player_num : msg . player_num , team_num : msg . team_num , fallen : msg . fallen , pose : msg . pose . to_vec () , ball_age : msg . ball_age , ball : msg . ball . to_vec () , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { msg . player_num = self . player_num ; msg . team_num = self . team_num ; msg . fallen = self . fallen ; assert_eq ! (self . pose . len () , 3usize , "Field {} is fixed size of {}!" , "pose" , 3usize) ; msg . pose . copy_from_slice (& self . pose [.. 3usize]) ; msg . ball_age = self . ball_age ; assert_eq ! (self . ball . len () , 2usize , "Field {} is fixed size of {}!" , "ball" , 2usize) ; msg . ball . copy_from_slice (& self . ball [.. 2usize]) ; } } impl Default for RCGCRD { fn default () -> Self { let msg_native = WrappedNativeMsg :: < RCGCRD > :: new () ; RCGCRD :: from_native (& msg_native) } } }