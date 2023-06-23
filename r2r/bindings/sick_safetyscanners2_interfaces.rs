pub mod srv { # [allow (non_snake_case)] pub mod FieldData { use super :: super :: super :: * ; # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] pub struct Service () ; impl WrappedServiceTypeSupport for Service { type Request = Request ; type Response = Response ; fn get_ts () -> & 'static rosidl_service_type_support_t { unsafe { & * rosidl_typesupport_c__get_service_type_support_handle__sick_safetyscanners2_interfaces__srv__FieldData () } } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct Request { } impl WrappedTypesupport for Request { type CStruct = sick_safetyscanners2_interfaces__srv__FieldData_Request ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__srv__FieldData_Request () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__srv__FieldData_Request { unsafe { sick_safetyscanners2_interfaces__srv__FieldData_Request__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__srv__FieldData_Request) -> () { unsafe { sick_safetyscanners2_interfaces__srv__FieldData_Request__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> Request { Request { } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { } } impl Default for Request { fn default () -> Self { let msg_native = WrappedNativeMsg :: < Request > :: new () ; Request :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct Response { pub fields : Vec < sick_safetyscanners2_interfaces :: msg :: Field > , pub device_name : std :: string :: String , pub monitoring_cases : Vec < sick_safetyscanners2_interfaces :: msg :: MonitoringCase > } impl WrappedTypesupport for Response { type CStruct = sick_safetyscanners2_interfaces__srv__FieldData_Response ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__srv__FieldData_Response () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__srv__FieldData_Response { unsafe { sick_safetyscanners2_interfaces__srv__FieldData_Response__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__srv__FieldData_Response) -> () { unsafe { sick_safetyscanners2_interfaces__srv__FieldData_Response__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> Response { Response { fields : { let mut temp = Vec :: with_capacity (msg . fields . size) ; let slice = unsafe { std :: slice :: from_raw_parts (msg . fields . data , msg . fields . size) } ; for s in slice { temp . push (sick_safetyscanners2_interfaces :: msg :: Field :: from_native (s)) ; } temp } , device_name : msg . device_name . to_str () . to_owned () , monitoring_cases : { let mut temp = Vec :: with_capacity (msg . monitoring_cases . size) ; let slice = unsafe { std :: slice :: from_raw_parts (msg . monitoring_cases . data , msg . monitoring_cases . size) } ; for s in slice { temp . push (sick_safetyscanners2_interfaces :: msg :: MonitoringCase :: from_native (s)) ; } temp } , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { unsafe { sick_safetyscanners2_interfaces__msg__Field__Sequence__fini (& mut msg . fields) ; sick_safetyscanners2_interfaces__msg__Field__Sequence__init (& mut msg . fields , self . fields . len ()) ; let slice = std :: slice :: from_raw_parts_mut (msg . fields . data , msg . fields . size) ; for (t , s) in slice . iter_mut () . zip (& self . fields) { s . copy_to_native (t) ; } } msg . device_name . assign (& self . device_name) ; unsafe { sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence__fini (& mut msg . monitoring_cases) ; sick_safetyscanners2_interfaces__msg__MonitoringCase__Sequence__init (& mut msg . monitoring_cases , self . monitoring_cases . len ()) ; let slice = std :: slice :: from_raw_parts_mut (msg . monitoring_cases . data , msg . monitoring_cases . size) ; for (t , s) in slice . iter_mut () . zip (& self . monitoring_cases) { s . copy_to_native (t) ; } } } } impl Default for Response { fn default () -> Self { let msg_native = WrappedNativeMsg :: < Response > :: new () ; Response :: from_native (& msg_native) } } } } pub mod msg { use super :: super :: * ; # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct ApplicationData { pub inputs : sick_safetyscanners2_interfaces :: msg :: ApplicationInputs , pub outputs : sick_safetyscanners2_interfaces :: msg :: ApplicationOutputs } impl WrappedTypesupport for ApplicationData { type CStruct = sick_safetyscanners2_interfaces__msg__ApplicationData ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__ApplicationData () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__ApplicationData { unsafe { sick_safetyscanners2_interfaces__msg__ApplicationData__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__ApplicationData) -> () { unsafe { sick_safetyscanners2_interfaces__msg__ApplicationData__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> ApplicationData { ApplicationData { inputs : sick_safetyscanners2_interfaces :: msg :: ApplicationInputs :: from_native (& msg . inputs) , outputs : sick_safetyscanners2_interfaces :: msg :: ApplicationOutputs :: from_native (& msg . outputs) , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { self . inputs . copy_to_native (& mut msg . inputs) ; self . outputs . copy_to_native (& mut msg . outputs) ; } } impl Default for ApplicationData { fn default () -> Self { let msg_native = WrappedNativeMsg :: < ApplicationData > :: new () ; ApplicationData :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct ApplicationInputs { pub unsafe_inputs_input_sources : Vec < bool > , pub unsafe_inputs_flags : Vec < bool > , pub monitoring_case_number_inputs : Vec < u16 > , pub monitoring_case_number_inputs_flags : Vec < bool > , pub linear_velocity_inputs_velocity_0 : i16 , pub linear_velocity_inputs_velocity_0_valid : bool , pub linear_velocity_inputs_velocity_0_transmitted_safely : bool , pub linear_velocity_inputs_velocity_1 : i16 , pub linear_velocity_inputs_velocity_1_valid : bool , pub linear_velocity_inputs_velocity_1_transmitted_safely : bool , pub sleep_mode_input : u8 } impl WrappedTypesupport for ApplicationInputs { type CStruct = sick_safetyscanners2_interfaces__msg__ApplicationInputs ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__ApplicationInputs () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__ApplicationInputs { unsafe { sick_safetyscanners2_interfaces__msg__ApplicationInputs__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__ApplicationInputs) -> () { unsafe { sick_safetyscanners2_interfaces__msg__ApplicationInputs__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> ApplicationInputs { ApplicationInputs { unsafe_inputs_input_sources : msg . unsafe_inputs_input_sources . to_vec () , unsafe_inputs_flags : msg . unsafe_inputs_flags . to_vec () , monitoring_case_number_inputs : msg . monitoring_case_number_inputs . to_vec () , monitoring_case_number_inputs_flags : msg . monitoring_case_number_inputs_flags . to_vec () , linear_velocity_inputs_velocity_0 : msg . linear_velocity_inputs_velocity_0 , linear_velocity_inputs_velocity_0_valid : msg . linear_velocity_inputs_velocity_0_valid , linear_velocity_inputs_velocity_0_transmitted_safely : msg . linear_velocity_inputs_velocity_0_transmitted_safely , linear_velocity_inputs_velocity_1 : msg . linear_velocity_inputs_velocity_1 , linear_velocity_inputs_velocity_1_valid : msg . linear_velocity_inputs_velocity_1_valid , linear_velocity_inputs_velocity_1_transmitted_safely : msg . linear_velocity_inputs_velocity_1_transmitted_safely , sleep_mode_input : msg . sleep_mode_input , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { msg . unsafe_inputs_input_sources . update (& self . unsafe_inputs_input_sources) ; msg . unsafe_inputs_flags . update (& self . unsafe_inputs_flags) ; msg . monitoring_case_number_inputs . update (& self . monitoring_case_number_inputs) ; msg . monitoring_case_number_inputs_flags . update (& self . monitoring_case_number_inputs_flags) ; msg . linear_velocity_inputs_velocity_0 = self . linear_velocity_inputs_velocity_0 ; msg . linear_velocity_inputs_velocity_0_valid = self . linear_velocity_inputs_velocity_0_valid ; msg . linear_velocity_inputs_velocity_0_transmitted_safely = self . linear_velocity_inputs_velocity_0_transmitted_safely ; msg . linear_velocity_inputs_velocity_1 = self . linear_velocity_inputs_velocity_1 ; msg . linear_velocity_inputs_velocity_1_valid = self . linear_velocity_inputs_velocity_1_valid ; msg . linear_velocity_inputs_velocity_1_transmitted_safely = self . linear_velocity_inputs_velocity_1_transmitted_safely ; msg . sleep_mode_input = self . sleep_mode_input ; } } impl Default for ApplicationInputs { fn default () -> Self { let msg_native = WrappedNativeMsg :: < ApplicationInputs > :: new () ; ApplicationInputs :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct ApplicationOutputs { pub evaluation_path_outputs_eval_out : Vec < bool > , pub evaluation_path_outputs_is_safe : Vec < bool > , pub evaluation_path_outputs_is_valid : Vec < bool > , pub monitoring_case_number_outputs : Vec < u16 > , pub monitoring_case_number_outputs_flags : Vec < bool > , pub sleep_mode_output : u8 , pub sleep_mode_output_valid : bool , pub error_flag_contamination_warning : bool , pub error_flag_contamination_error : bool , pub error_flag_manipulation_error : bool , pub error_flag_glare : bool , pub error_flag_reference_contour_intruded : bool , pub error_flag_critical_error : bool , pub error_flags_are_valid : bool , pub linear_velocity_outputs_velocity_0 : i16 , pub linear_velocity_outputs_velocity_0_valid : bool , pub linear_velocity_outputs_velocity_0_transmitted_safely : bool , pub linear_velocity_outputs_velocity_1 : i16 , pub linear_velocity_outputs_velocity_1_valid : bool , pub linear_velocity_outputs_velocity_1_transmitted_safely : bool , pub resulting_velocity : Vec < i16 > , pub resulting_velocity_flags : Vec < bool > } impl WrappedTypesupport for ApplicationOutputs { type CStruct = sick_safetyscanners2_interfaces__msg__ApplicationOutputs ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__ApplicationOutputs () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__ApplicationOutputs { unsafe { sick_safetyscanners2_interfaces__msg__ApplicationOutputs__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__ApplicationOutputs) -> () { unsafe { sick_safetyscanners2_interfaces__msg__ApplicationOutputs__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> ApplicationOutputs { ApplicationOutputs { evaluation_path_outputs_eval_out : msg . evaluation_path_outputs_eval_out . to_vec () , evaluation_path_outputs_is_safe : msg . evaluation_path_outputs_is_safe . to_vec () , evaluation_path_outputs_is_valid : msg . evaluation_path_outputs_is_valid . to_vec () , monitoring_case_number_outputs : msg . monitoring_case_number_outputs . to_vec () , monitoring_case_number_outputs_flags : msg . monitoring_case_number_outputs_flags . to_vec () , sleep_mode_output : msg . sleep_mode_output , sleep_mode_output_valid : msg . sleep_mode_output_valid , error_flag_contamination_warning : msg . error_flag_contamination_warning , error_flag_contamination_error : msg . error_flag_contamination_error , error_flag_manipulation_error : msg . error_flag_manipulation_error , error_flag_glare : msg . error_flag_glare , error_flag_reference_contour_intruded : msg . error_flag_reference_contour_intruded , error_flag_critical_error : msg . error_flag_critical_error , error_flags_are_valid : msg . error_flags_are_valid , linear_velocity_outputs_velocity_0 : msg . linear_velocity_outputs_velocity_0 , linear_velocity_outputs_velocity_0_valid : msg . linear_velocity_outputs_velocity_0_valid , linear_velocity_outputs_velocity_0_transmitted_safely : msg . linear_velocity_outputs_velocity_0_transmitted_safely , linear_velocity_outputs_velocity_1 : msg . linear_velocity_outputs_velocity_1 , linear_velocity_outputs_velocity_1_valid : msg . linear_velocity_outputs_velocity_1_valid , linear_velocity_outputs_velocity_1_transmitted_safely : msg . linear_velocity_outputs_velocity_1_transmitted_safely , resulting_velocity : msg . resulting_velocity . to_vec () , resulting_velocity_flags : msg . resulting_velocity_flags . to_vec () , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { msg . evaluation_path_outputs_eval_out . update (& self . evaluation_path_outputs_eval_out) ; msg . evaluation_path_outputs_is_safe . update (& self . evaluation_path_outputs_is_safe) ; msg . evaluation_path_outputs_is_valid . update (& self . evaluation_path_outputs_is_valid) ; msg . monitoring_case_number_outputs . update (& self . monitoring_case_number_outputs) ; msg . monitoring_case_number_outputs_flags . update (& self . monitoring_case_number_outputs_flags) ; msg . sleep_mode_output = self . sleep_mode_output ; msg . sleep_mode_output_valid = self . sleep_mode_output_valid ; msg . error_flag_contamination_warning = self . error_flag_contamination_warning ; msg . error_flag_contamination_error = self . error_flag_contamination_error ; msg . error_flag_manipulation_error = self . error_flag_manipulation_error ; msg . error_flag_glare = self . error_flag_glare ; msg . error_flag_reference_contour_intruded = self . error_flag_reference_contour_intruded ; msg . error_flag_critical_error = self . error_flag_critical_error ; msg . error_flags_are_valid = self . error_flags_are_valid ; msg . linear_velocity_outputs_velocity_0 = self . linear_velocity_outputs_velocity_0 ; msg . linear_velocity_outputs_velocity_0_valid = self . linear_velocity_outputs_velocity_0_valid ; msg . linear_velocity_outputs_velocity_0_transmitted_safely = self . linear_velocity_outputs_velocity_0_transmitted_safely ; msg . linear_velocity_outputs_velocity_1 = self . linear_velocity_outputs_velocity_1 ; msg . linear_velocity_outputs_velocity_1_valid = self . linear_velocity_outputs_velocity_1_valid ; msg . linear_velocity_outputs_velocity_1_transmitted_safely = self . linear_velocity_outputs_velocity_1_transmitted_safely ; msg . resulting_velocity . update (& self . resulting_velocity) ; msg . resulting_velocity_flags . update (& self . resulting_velocity_flags) ; } } impl Default for ApplicationOutputs { fn default () -> Self { let msg_native = WrappedNativeMsg :: < ApplicationOutputs > :: new () ; ApplicationOutputs :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct DataHeader { pub version_version : u8 , pub version_major_version : u8 , pub version_minor_version : u8 , pub version_release : u8 , pub serial_number_of_device : u32 , pub serial_number_of_channel_plug : u32 , pub channel_number : u8 , pub sequence_number : u32 , pub scan_number : u32 , pub timestamp_date : u16 , pub timestamp_time : u32 } impl WrappedTypesupport for DataHeader { type CStruct = sick_safetyscanners2_interfaces__msg__DataHeader ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__DataHeader () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__DataHeader { unsafe { sick_safetyscanners2_interfaces__msg__DataHeader__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__DataHeader) -> () { unsafe { sick_safetyscanners2_interfaces__msg__DataHeader__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> DataHeader { DataHeader { version_version : msg . version_version , version_major_version : msg . version_major_version , version_minor_version : msg . version_minor_version , version_release : msg . version_release , serial_number_of_device : msg . serial_number_of_device , serial_number_of_channel_plug : msg . serial_number_of_channel_plug , channel_number : msg . channel_number , sequence_number : msg . sequence_number , scan_number : msg . scan_number , timestamp_date : msg . timestamp_date , timestamp_time : msg . timestamp_time , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { msg . version_version = self . version_version ; msg . version_major_version = self . version_major_version ; msg . version_minor_version = self . version_minor_version ; msg . version_release = self . version_release ; msg . serial_number_of_device = self . serial_number_of_device ; msg . serial_number_of_channel_plug = self . serial_number_of_channel_plug ; msg . channel_number = self . channel_number ; msg . sequence_number = self . sequence_number ; msg . scan_number = self . scan_number ; msg . timestamp_date = self . timestamp_date ; msg . timestamp_time = self . timestamp_time ; } } impl Default for DataHeader { fn default () -> Self { let msg_native = WrappedNativeMsg :: < DataHeader > :: new () ; DataHeader :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct DerivedValues { pub multiplication_factor : u16 , pub number_of_beams : u16 , pub scan_time : u16 , pub start_angle : f32 , pub angular_beam_resolution : f32 , pub interbeam_period : u32 } impl WrappedTypesupport for DerivedValues { type CStruct = sick_safetyscanners2_interfaces__msg__DerivedValues ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__DerivedValues () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__DerivedValues { unsafe { sick_safetyscanners2_interfaces__msg__DerivedValues__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__DerivedValues) -> () { unsafe { sick_safetyscanners2_interfaces__msg__DerivedValues__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> DerivedValues { DerivedValues { multiplication_factor : msg . multiplication_factor , number_of_beams : msg . number_of_beams , scan_time : msg . scan_time , start_angle : msg . start_angle , angular_beam_resolution : msg . angular_beam_resolution , interbeam_period : msg . interbeam_period , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { msg . multiplication_factor = self . multiplication_factor ; msg . number_of_beams = self . number_of_beams ; msg . scan_time = self . scan_time ; msg . start_angle = self . start_angle ; msg . angular_beam_resolution = self . angular_beam_resolution ; msg . interbeam_period = self . interbeam_period ; } } impl Default for DerivedValues { fn default () -> Self { let msg_native = WrappedNativeMsg :: < DerivedValues > :: new () ; DerivedValues :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct ExtendedLaserScan { pub laser_scan : sensor_msgs :: msg :: LaserScan , pub reflektor_status : Vec < bool > , pub reflektor_median : Vec < bool > , pub intrusion : Vec < bool > } impl WrappedTypesupport for ExtendedLaserScan { type CStruct = sick_safetyscanners2_interfaces__msg__ExtendedLaserScan ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__ExtendedLaserScan () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__ExtendedLaserScan { unsafe { sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__ExtendedLaserScan) -> () { unsafe { sick_safetyscanners2_interfaces__msg__ExtendedLaserScan__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> ExtendedLaserScan { ExtendedLaserScan { laser_scan : sensor_msgs :: msg :: LaserScan :: from_native (& msg . laser_scan) , reflektor_status : msg . reflektor_status . to_vec () , reflektor_median : msg . reflektor_median . to_vec () , intrusion : msg . intrusion . to_vec () , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { self . laser_scan . copy_to_native (& mut msg . laser_scan) ; msg . reflektor_status . update (& self . reflektor_status) ; msg . reflektor_median . update (& self . reflektor_median) ; msg . intrusion . update (& self . intrusion) ; } } impl Default for ExtendedLaserScan { fn default () -> Self { let msg_native = WrappedNativeMsg :: < ExtendedLaserScan > :: new () ; ExtendedLaserScan :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct Field { pub ranges : Vec < f32 > , pub start_angle : f32 , pub angular_resolution : f32 , pub protective_field : bool } impl WrappedTypesupport for Field { type CStruct = sick_safetyscanners2_interfaces__msg__Field ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__Field () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__Field { unsafe { sick_safetyscanners2_interfaces__msg__Field__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__Field) -> () { unsafe { sick_safetyscanners2_interfaces__msg__Field__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> Field { Field { ranges : msg . ranges . to_vec () , start_angle : msg . start_angle , angular_resolution : msg . angular_resolution , protective_field : msg . protective_field , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { msg . ranges . update (& self . ranges) ; msg . start_angle = self . start_angle ; msg . angular_resolution = self . angular_resolution ; msg . protective_field = self . protective_field ; } } impl Default for Field { fn default () -> Self { let msg_native = WrappedNativeMsg :: < Field > :: new () ; Field :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct GeneralSystemState { pub run_mode_active : bool , pub standby_mode_active : bool , pub contamination_warning : bool , pub contamination_error : bool , pub reference_contour_status : bool , pub manipulation_status : bool , pub safe_cut_off_path : Vec < bool > , pub non_safe_cut_off_path : Vec < bool > , pub reset_required_cut_off_path : Vec < bool > , pub current_monitoring_case_no_table_1 : u8 , pub current_monitoring_case_no_table_2 : u8 , pub current_monitoring_case_no_table_3 : u8 , pub current_monitoring_case_no_table_4 : u8 , pub application_error : bool , pub device_error : bool } impl WrappedTypesupport for GeneralSystemState { type CStruct = sick_safetyscanners2_interfaces__msg__GeneralSystemState ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__GeneralSystemState () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__GeneralSystemState { unsafe { sick_safetyscanners2_interfaces__msg__GeneralSystemState__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__GeneralSystemState) -> () { unsafe { sick_safetyscanners2_interfaces__msg__GeneralSystemState__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> GeneralSystemState { GeneralSystemState { run_mode_active : msg . run_mode_active , standby_mode_active : msg . standby_mode_active , contamination_warning : msg . contamination_warning , contamination_error : msg . contamination_error , reference_contour_status : msg . reference_contour_status , manipulation_status : msg . manipulation_status , safe_cut_off_path : msg . safe_cut_off_path . to_vec () , non_safe_cut_off_path : msg . non_safe_cut_off_path . to_vec () , reset_required_cut_off_path : msg . reset_required_cut_off_path . to_vec () , current_monitoring_case_no_table_1 : msg . current_monitoring_case_no_table_1 , current_monitoring_case_no_table_2 : msg . current_monitoring_case_no_table_2 , current_monitoring_case_no_table_3 : msg . current_monitoring_case_no_table_3 , current_monitoring_case_no_table_4 : msg . current_monitoring_case_no_table_4 , application_error : msg . application_error , device_error : msg . device_error , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { msg . run_mode_active = self . run_mode_active ; msg . standby_mode_active = self . standby_mode_active ; msg . contamination_warning = self . contamination_warning ; msg . contamination_error = self . contamination_error ; msg . reference_contour_status = self . reference_contour_status ; msg . manipulation_status = self . manipulation_status ; msg . safe_cut_off_path . update (& self . safe_cut_off_path) ; msg . non_safe_cut_off_path . update (& self . non_safe_cut_off_path) ; msg . reset_required_cut_off_path . update (& self . reset_required_cut_off_path) ; msg . current_monitoring_case_no_table_1 = self . current_monitoring_case_no_table_1 ; msg . current_monitoring_case_no_table_2 = self . current_monitoring_case_no_table_2 ; msg . current_monitoring_case_no_table_3 = self . current_monitoring_case_no_table_3 ; msg . current_monitoring_case_no_table_4 = self . current_monitoring_case_no_table_4 ; msg . application_error = self . application_error ; msg . device_error = self . device_error ; } } impl Default for GeneralSystemState { fn default () -> Self { let msg_native = WrappedNativeMsg :: < GeneralSystemState > :: new () ; GeneralSystemState :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct IntrusionData { pub data : Vec < sick_safetyscanners2_interfaces :: msg :: IntrusionDatum > } impl WrappedTypesupport for IntrusionData { type CStruct = sick_safetyscanners2_interfaces__msg__IntrusionData ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__IntrusionData () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__IntrusionData { unsafe { sick_safetyscanners2_interfaces__msg__IntrusionData__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__IntrusionData) -> () { unsafe { sick_safetyscanners2_interfaces__msg__IntrusionData__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> IntrusionData { IntrusionData { data : { let mut temp = Vec :: with_capacity (msg . data . size) ; let slice = unsafe { std :: slice :: from_raw_parts (msg . data . data , msg . data . size) } ; for s in slice { temp . push (sick_safetyscanners2_interfaces :: msg :: IntrusionDatum :: from_native (s)) ; } temp } , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { unsafe { sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence__fini (& mut msg . data) ; sick_safetyscanners2_interfaces__msg__IntrusionDatum__Sequence__init (& mut msg . data , self . data . len ()) ; let slice = std :: slice :: from_raw_parts_mut (msg . data . data , msg . data . size) ; for (t , s) in slice . iter_mut () . zip (& self . data) { s . copy_to_native (t) ; } } } } impl Default for IntrusionData { fn default () -> Self { let msg_native = WrappedNativeMsg :: < IntrusionData > :: new () ; IntrusionData :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct IntrusionDatum { pub size : u32 , pub flags : Vec < bool > } impl WrappedTypesupport for IntrusionDatum { type CStruct = sick_safetyscanners2_interfaces__msg__IntrusionDatum ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__IntrusionDatum () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__IntrusionDatum { unsafe { sick_safetyscanners2_interfaces__msg__IntrusionDatum__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__IntrusionDatum) -> () { unsafe { sick_safetyscanners2_interfaces__msg__IntrusionDatum__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> IntrusionDatum { IntrusionDatum { size : msg . size , flags : msg . flags . to_vec () , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { msg . size = self . size ; msg . flags . update (& self . flags) ; } } impl Default for IntrusionDatum { fn default () -> Self { let msg_native = WrappedNativeMsg :: < IntrusionDatum > :: new () ; IntrusionDatum :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct MeasurementData { pub number_of_beams : u32 , pub scan_points : Vec < sick_safetyscanners2_interfaces :: msg :: ScanPoint > } impl WrappedTypesupport for MeasurementData { type CStruct = sick_safetyscanners2_interfaces__msg__MeasurementData ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__MeasurementData () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__MeasurementData { unsafe { sick_safetyscanners2_interfaces__msg__MeasurementData__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__MeasurementData) -> () { unsafe { sick_safetyscanners2_interfaces__msg__MeasurementData__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> MeasurementData { MeasurementData { number_of_beams : msg . number_of_beams , scan_points : { let mut temp = Vec :: with_capacity (msg . scan_points . size) ; let slice = unsafe { std :: slice :: from_raw_parts (msg . scan_points . data , msg . scan_points . size) } ; for s in slice { temp . push (sick_safetyscanners2_interfaces :: msg :: ScanPoint :: from_native (s)) ; } temp } , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { msg . number_of_beams = self . number_of_beams ; unsafe { sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence__fini (& mut msg . scan_points) ; sick_safetyscanners2_interfaces__msg__ScanPoint__Sequence__init (& mut msg . scan_points , self . scan_points . len ()) ; let slice = std :: slice :: from_raw_parts_mut (msg . scan_points . data , msg . scan_points . size) ; for (t , s) in slice . iter_mut () . zip (& self . scan_points) { s . copy_to_native (t) ; } } } } impl Default for MeasurementData { fn default () -> Self { let msg_native = WrappedNativeMsg :: < MeasurementData > :: new () ; MeasurementData :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct MonitoringCase { pub monitoring_case_number : i32 , pub fields : Vec < i32 > , pub fields_valid : Vec < bool > } impl WrappedTypesupport for MonitoringCase { type CStruct = sick_safetyscanners2_interfaces__msg__MonitoringCase ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__MonitoringCase () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__MonitoringCase { unsafe { sick_safetyscanners2_interfaces__msg__MonitoringCase__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__MonitoringCase) -> () { unsafe { sick_safetyscanners2_interfaces__msg__MonitoringCase__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> MonitoringCase { MonitoringCase { monitoring_case_number : msg . monitoring_case_number , fields : msg . fields . to_vec () , fields_valid : msg . fields_valid . to_vec () , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { msg . monitoring_case_number = self . monitoring_case_number ; msg . fields . update (& self . fields) ; msg . fields_valid . update (& self . fields_valid) ; } } impl Default for MonitoringCase { fn default () -> Self { let msg_native = WrappedNativeMsg :: < MonitoringCase > :: new () ; MonitoringCase :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct OutputPaths { pub status : Vec < bool > , pub is_safe : Vec < bool > , pub is_valid : Vec < bool > , pub active_monitoring_case : i32 } impl WrappedTypesupport for OutputPaths { type CStruct = sick_safetyscanners2_interfaces__msg__OutputPaths ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__OutputPaths () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__OutputPaths { unsafe { sick_safetyscanners2_interfaces__msg__OutputPaths__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__OutputPaths) -> () { unsafe { sick_safetyscanners2_interfaces__msg__OutputPaths__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> OutputPaths { OutputPaths { status : msg . status . to_vec () , is_safe : msg . is_safe . to_vec () , is_valid : msg . is_valid . to_vec () , active_monitoring_case : msg . active_monitoring_case , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { msg . status . update (& self . status) ; msg . is_safe . update (& self . is_safe) ; msg . is_valid . update (& self . is_valid) ; msg . active_monitoring_case = self . active_monitoring_case ; } } impl Default for OutputPaths { fn default () -> Self { let msg_native = WrappedNativeMsg :: < OutputPaths > :: new () ; OutputPaths :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct RawMicroScanData { pub header : sick_safetyscanners2_interfaces :: msg :: DataHeader , pub derived_values : sick_safetyscanners2_interfaces :: msg :: DerivedValues , pub general_system_state : sick_safetyscanners2_interfaces :: msg :: GeneralSystemState , pub measurement_data : sick_safetyscanners2_interfaces :: msg :: MeasurementData , pub intrusion_data : sick_safetyscanners2_interfaces :: msg :: IntrusionData , pub application_data : sick_safetyscanners2_interfaces :: msg :: ApplicationData } impl WrappedTypesupport for RawMicroScanData { type CStruct = sick_safetyscanners2_interfaces__msg__RawMicroScanData ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__RawMicroScanData () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__RawMicroScanData { unsafe { sick_safetyscanners2_interfaces__msg__RawMicroScanData__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__RawMicroScanData) -> () { unsafe { sick_safetyscanners2_interfaces__msg__RawMicroScanData__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> RawMicroScanData { RawMicroScanData { header : sick_safetyscanners2_interfaces :: msg :: DataHeader :: from_native (& msg . header) , derived_values : sick_safetyscanners2_interfaces :: msg :: DerivedValues :: from_native (& msg . derived_values) , general_system_state : sick_safetyscanners2_interfaces :: msg :: GeneralSystemState :: from_native (& msg . general_system_state) , measurement_data : sick_safetyscanners2_interfaces :: msg :: MeasurementData :: from_native (& msg . measurement_data) , intrusion_data : sick_safetyscanners2_interfaces :: msg :: IntrusionData :: from_native (& msg . intrusion_data) , application_data : sick_safetyscanners2_interfaces :: msg :: ApplicationData :: from_native (& msg . application_data) , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { self . header . copy_to_native (& mut msg . header) ; self . derived_values . copy_to_native (& mut msg . derived_values) ; self . general_system_state . copy_to_native (& mut msg . general_system_state) ; self . measurement_data . copy_to_native (& mut msg . measurement_data) ; self . intrusion_data . copy_to_native (& mut msg . intrusion_data) ; self . application_data . copy_to_native (& mut msg . application_data) ; } } impl Default for RawMicroScanData { fn default () -> Self { let msg_native = WrappedNativeMsg :: < RawMicroScanData > :: new () ; RawMicroScanData :: from_native (& msg_native) } } # [derive (Clone , Debug , PartialEq , Serialize , Deserialize)] # [serde (default)] pub struct ScanPoint { pub angle : f32 , pub distance : u16 , pub reflectivity : u8 , pub valid : bool , pub infinite : bool , pub glare : bool , pub reflector : bool , pub contamination : bool , pub contamination_warning : bool } impl WrappedTypesupport for ScanPoint { type CStruct = sick_safetyscanners2_interfaces__msg__ScanPoint ; fn get_ts () -> & 'static rosidl_message_type_support_t { unsafe { & * rosidl_typesupport_c__get_message_type_support_handle__sick_safetyscanners2_interfaces__msg__ScanPoint () } } fn create_msg () -> * mut sick_safetyscanners2_interfaces__msg__ScanPoint { unsafe { sick_safetyscanners2_interfaces__msg__ScanPoint__create () } } fn destroy_msg (msg : * mut sick_safetyscanners2_interfaces__msg__ScanPoint) -> () { unsafe { sick_safetyscanners2_interfaces__msg__ScanPoint__destroy (msg) } ; } fn from_native (# [allow (unused)] msg : & Self :: CStruct) -> ScanPoint { ScanPoint { angle : msg . angle , distance : msg . distance , reflectivity : msg . reflectivity , valid : msg . valid , infinite : msg . infinite , glare : msg . glare , reflector : msg . reflector , contamination : msg . contamination , contamination_warning : msg . contamination_warning , } } fn copy_to_native (& self , # [allow (unused)] msg : & mut Self :: CStruct) { msg . angle = self . angle ; msg . distance = self . distance ; msg . reflectivity = self . reflectivity ; msg . valid = self . valid ; msg . infinite = self . infinite ; msg . glare = self . glare ; msg . reflector = self . reflector ; msg . contamination = self . contamination ; msg . contamination_warning = self . contamination_warning ; } } impl Default for ScanPoint { fn default () -> Self { let msg_native = WrappedNativeMsg :: < ScanPoint > :: new () ; ScanPoint :: from_native (& msg_native) } } }