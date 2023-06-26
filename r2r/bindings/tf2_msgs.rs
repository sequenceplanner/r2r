pub mod action {
    #[allow(non_snake_case)]
    pub mod LookupTransform {
        use super::super::super::*;
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        pub struct Action();
        impl WrappedActionTypeSupport for Action {
            type Goal = Goal;
            type Result = Result;
            type Feedback = Feedback;
            type FeedbackMessage = FeedbackMessage;
            type SendGoal = SendGoal::Service;
            type GetResult = GetResult::Service;
            fn get_ts() -> &'static rosidl_action_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_action_type_support_handle__tf2_msgs__action__LookupTransform()
                }
            }
            fn make_goal_request_msg(
                goal_id: unique_identifier_msgs::msg::UUID,
                goal: Goal,
            ) -> SendGoal::Request {
                SendGoal::Request { goal_id, goal }
            }
            fn make_goal_response_msg(
                accepted: bool,
                stamp: builtin_interfaces::msg::Time,
            ) -> SendGoal::Response {
                SendGoal::Response {
                    accepted,
                    stamp,
                }
            }
            fn make_feedback_msg(
                goal_id: unique_identifier_msgs::msg::UUID,
                feedback: Feedback,
            ) -> FeedbackMessage {
                FeedbackMessage {
                    goal_id,
                    feedback,
                }
            }
            fn make_result_request_msg(
                goal_id: unique_identifier_msgs::msg::UUID,
            ) -> GetResult::Request {
                GetResult::Request { goal_id }
            }
            fn make_result_response_msg(
                status: i8,
                result: Result,
            ) -> GetResult::Response {
                GetResult::Response {
                    status,
                    result,
                }
            }
            fn destructure_goal_request_msg(
                msg: SendGoal::Request,
            ) -> (unique_identifier_msgs::msg::UUID, Goal) {
                (msg.goal_id, msg.goal)
            }
            fn destructure_goal_response_msg(
                msg: SendGoal::Response,
            ) -> (bool, builtin_interfaces::msg::Time) {
                (msg.accepted, msg.stamp)
            }
            fn destructure_feedback_msg(
                msg: FeedbackMessage,
            ) -> (unique_identifier_msgs::msg::UUID, Feedback) {
                (msg.goal_id, msg.feedback)
            }
            fn destructure_result_response_msg(
                msg: GetResult::Response,
            ) -> (i8, Result) {
                (msg.status, msg.result)
            }
            fn destructure_result_request_msg(
                msg: GetResult::Request,
            ) -> unique_identifier_msgs::msg::UUID {
                msg.goal_id
            }
        }
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Goal {
            pub target_frame: std::string::String,
            pub source_frame: std::string::String,
            pub source_time: builtin_interfaces::msg::Time,
            pub timeout: builtin_interfaces::msg::Duration,
            pub target_time: builtin_interfaces::msg::Time,
            pub fixed_frame: std::string::String,
            pub advanced: bool,
        }
        impl WrappedTypesupport for Goal {
            type CStruct = tf2_msgs__action__LookupTransform_Goal;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__action__LookupTransform_Goal()
                }
            }
            fn create_msg() -> *mut tf2_msgs__action__LookupTransform_Goal {
                #[cfg(not(feature = "doc-only"))]
                unsafe { tf2_msgs__action__LookupTransform_Goal__create() }
                #[cfg(feature = "doc-only")]
                tf2_msgs__action__LookupTransform_Goal__create()
            }
            fn destroy_msg(msg: *mut tf2_msgs__action__LookupTransform_Goal) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { tf2_msgs__action__LookupTransform_Goal__destroy(msg) };
                #[cfg(feature = "doc-only")]
                tf2_msgs__action__LookupTransform_Goal__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Goal {
                Goal {
                    target_frame: msg.target_frame.to_str().to_owned(),
                    source_frame: msg.source_frame.to_str().to_owned(),
                    source_time: builtin_interfaces::msg::Time::from_native(
                        &msg.source_time,
                    ),
                    timeout: builtin_interfaces::msg::Duration::from_native(
                        &msg.timeout,
                    ),
                    target_time: builtin_interfaces::msg::Time::from_native(
                        &msg.target_time,
                    ),
                    fixed_frame: msg.fixed_frame.to_str().to_owned(),
                    advanced: msg.advanced,
                }
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                msg.target_frame.assign(&self.target_frame);
                msg.source_frame.assign(&self.source_frame);
                self.source_time.copy_to_native(&mut msg.source_time);
                self.timeout.copy_to_native(&mut msg.timeout);
                self.target_time.copy_to_native(&mut msg.target_time);
                msg.fixed_frame.assign(&self.fixed_frame);
                msg.advanced = self.advanced;
            }
        }
        impl Default for Goal {
            fn default() -> Self {
                let msg_native = WrappedNativeMsg::<Goal>::new();
                Goal::from_native(&msg_native)
            }
        }
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Result {
            pub transform: geometry_msgs::msg::TransformStamped,
            pub error: tf2_msgs::msg::TF2Error,
        }
        impl WrappedTypesupport for Result {
            type CStruct = tf2_msgs__action__LookupTransform_Result;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__action__LookupTransform_Result()
                }
            }
            fn create_msg() -> *mut tf2_msgs__action__LookupTransform_Result {
                #[cfg(not(feature = "doc-only"))]
                unsafe { tf2_msgs__action__LookupTransform_Result__create() }
                #[cfg(feature = "doc-only")]
                tf2_msgs__action__LookupTransform_Result__create()
            }
            fn destroy_msg(msg: *mut tf2_msgs__action__LookupTransform_Result) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { tf2_msgs__action__LookupTransform_Result__destroy(msg) };
                #[cfg(feature = "doc-only")]
                tf2_msgs__action__LookupTransform_Result__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Result {
                Result {
                    transform: geometry_msgs::msg::TransformStamped::from_native(
                        &msg.transform,
                    ),
                    error: tf2_msgs::msg::TF2Error::from_native(&msg.error),
                }
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                self.transform.copy_to_native(&mut msg.transform);
                self.error.copy_to_native(&mut msg.error);
            }
        }
        impl Default for Result {
            fn default() -> Self {
                let msg_native = WrappedNativeMsg::<Result>::new();
                Result::from_native(&msg_native)
            }
        }
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Feedback {}
        impl WrappedTypesupport for Feedback {
            type CStruct = tf2_msgs__action__LookupTransform_Feedback;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__action__LookupTransform_Feedback()
                }
            }
            fn create_msg() -> *mut tf2_msgs__action__LookupTransform_Feedback {
                #[cfg(not(feature = "doc-only"))]
                unsafe { tf2_msgs__action__LookupTransform_Feedback__create() }
                #[cfg(feature = "doc-only")]
                tf2_msgs__action__LookupTransform_Feedback__create()
            }
            fn destroy_msg(msg: *mut tf2_msgs__action__LookupTransform_Feedback) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { tf2_msgs__action__LookupTransform_Feedback__destroy(msg) };
                #[cfg(feature = "doc-only")]
                tf2_msgs__action__LookupTransform_Feedback__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Feedback {
                Feedback {}
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {}
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
            #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
            pub struct Service();
            impl WrappedServiceTypeSupport for Service {
                type Request = Request;
                type Response = Response;
                fn get_ts() -> &'static rosidl_service_type_support_t {
                    unsafe {
                        &*rosidl_typesupport_c__get_service_type_support_handle__tf2_msgs__action__LookupTransform_SendGoal()
                    }
                }
            }
            #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
            #[serde(default)]
            pub struct Request {
                pub goal_id: unique_identifier_msgs::msg::UUID,
                pub goal: tf2_msgs::action::LookupTransform::Goal,
            }
            impl WrappedTypesupport for Request {
                type CStruct = tf2_msgs__action__LookupTransform_SendGoal_Request;
                fn get_ts() -> &'static rosidl_message_type_support_t {
                    unsafe {
                        &*rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__action__LookupTransform_SendGoal_Request()
                    }
                }
                fn create_msg() -> *mut tf2_msgs__action__LookupTransform_SendGoal_Request {
                    #[cfg(not(feature = "doc-only"))]
                    unsafe {
                        tf2_msgs__action__LookupTransform_SendGoal_Request__create()
                    }
                    #[cfg(feature = "doc-only")]
                    tf2_msgs__action__LookupTransform_SendGoal_Request__create()
                }
                fn destroy_msg(
                    msg: *mut tf2_msgs__action__LookupTransform_SendGoal_Request,
                ) -> () {
                    #[cfg(not(feature = "doc-only"))]
                    unsafe {
                        tf2_msgs__action__LookupTransform_SendGoal_Request__destroy(msg)
                    };
                    #[cfg(feature = "doc-only")]
                    tf2_msgs__action__LookupTransform_SendGoal_Request__destroy(msg)
                }
                fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Request {
                    Request {
                        goal_id: unique_identifier_msgs::msg::UUID::from_native(
                            &msg.goal_id,
                        ),
                        goal: tf2_msgs::action::LookupTransform::Goal::from_native(
                            &msg.goal,
                        ),
                    }
                }
                fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                    self.goal_id.copy_to_native(&mut msg.goal_id);
                    self.goal.copy_to_native(&mut msg.goal);
                }
            }
            impl Default for Request {
                fn default() -> Self {
                    let msg_native = WrappedNativeMsg::<Request>::new();
                    Request::from_native(&msg_native)
                }
            }
            #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
            #[serde(default)]
            pub struct Response {
                pub accepted: bool,
                pub stamp: builtin_interfaces::msg::Time,
            }
            impl WrappedTypesupport for Response {
                type CStruct = tf2_msgs__action__LookupTransform_SendGoal_Response;
                fn get_ts() -> &'static rosidl_message_type_support_t {
                    unsafe {
                        &*rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__action__LookupTransform_SendGoal_Response()
                    }
                }
                fn create_msg() -> *mut tf2_msgs__action__LookupTransform_SendGoal_Response {
                    #[cfg(not(feature = "doc-only"))]
                    unsafe {
                        tf2_msgs__action__LookupTransform_SendGoal_Response__create()
                    }
                    #[cfg(feature = "doc-only")]
                    tf2_msgs__action__LookupTransform_SendGoal_Response__create()
                }
                fn destroy_msg(
                    msg: *mut tf2_msgs__action__LookupTransform_SendGoal_Response,
                ) -> () {
                    #[cfg(not(feature = "doc-only"))]
                    unsafe {
                        tf2_msgs__action__LookupTransform_SendGoal_Response__destroy(msg)
                    };
                    #[cfg(feature = "doc-only")]
                    tf2_msgs__action__LookupTransform_SendGoal_Response__destroy(msg)
                }
                fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Response {
                    Response {
                        accepted: msg.accepted,
                        stamp: builtin_interfaces::msg::Time::from_native(&msg.stamp),
                    }
                }
                fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                    msg.accepted = self.accepted;
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
            #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
            pub struct Service();
            impl WrappedServiceTypeSupport for Service {
                type Request = Request;
                type Response = Response;
                fn get_ts() -> &'static rosidl_service_type_support_t {
                    unsafe {
                        &*rosidl_typesupport_c__get_service_type_support_handle__tf2_msgs__action__LookupTransform_GetResult()
                    }
                }
            }
            #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
            #[serde(default)]
            pub struct Request {
                pub goal_id: unique_identifier_msgs::msg::UUID,
            }
            impl WrappedTypesupport for Request {
                type CStruct = tf2_msgs__action__LookupTransform_GetResult_Request;
                fn get_ts() -> &'static rosidl_message_type_support_t {
                    unsafe {
                        &*rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__action__LookupTransform_GetResult_Request()
                    }
                }
                fn create_msg() -> *mut tf2_msgs__action__LookupTransform_GetResult_Request {
                    #[cfg(not(feature = "doc-only"))]
                    unsafe {
                        tf2_msgs__action__LookupTransform_GetResult_Request__create()
                    }
                    #[cfg(feature = "doc-only")]
                    tf2_msgs__action__LookupTransform_GetResult_Request__create()
                }
                fn destroy_msg(
                    msg: *mut tf2_msgs__action__LookupTransform_GetResult_Request,
                ) -> () {
                    #[cfg(not(feature = "doc-only"))]
                    unsafe {
                        tf2_msgs__action__LookupTransform_GetResult_Request__destroy(msg)
                    };
                    #[cfg(feature = "doc-only")]
                    tf2_msgs__action__LookupTransform_GetResult_Request__destroy(msg)
                }
                fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Request {
                    Request {
                        goal_id: unique_identifier_msgs::msg::UUID::from_native(
                            &msg.goal_id,
                        ),
                    }
                }
                fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                    self.goal_id.copy_to_native(&mut msg.goal_id);
                }
            }
            impl Default for Request {
                fn default() -> Self {
                    let msg_native = WrappedNativeMsg::<Request>::new();
                    Request::from_native(&msg_native)
                }
            }
            #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
            #[serde(default)]
            pub struct Response {
                pub status: i8,
                pub result: tf2_msgs::action::LookupTransform::Result,
            }
            impl WrappedTypesupport for Response {
                type CStruct = tf2_msgs__action__LookupTransform_GetResult_Response;
                fn get_ts() -> &'static rosidl_message_type_support_t {
                    unsafe {
                        &*rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__action__LookupTransform_GetResult_Response()
                    }
                }
                fn create_msg() -> *mut tf2_msgs__action__LookupTransform_GetResult_Response {
                    #[cfg(not(feature = "doc-only"))]
                    unsafe {
                        tf2_msgs__action__LookupTransform_GetResult_Response__create()
                    }
                    #[cfg(feature = "doc-only")]
                    tf2_msgs__action__LookupTransform_GetResult_Response__create()
                }
                fn destroy_msg(
                    msg: *mut tf2_msgs__action__LookupTransform_GetResult_Response,
                ) -> () {
                    #[cfg(not(feature = "doc-only"))]
                    unsafe {
                        tf2_msgs__action__LookupTransform_GetResult_Response__destroy(
                            msg,
                        )
                    };
                    #[cfg(feature = "doc-only")]
                    tf2_msgs__action__LookupTransform_GetResult_Response__destroy(msg)
                }
                fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Response {
                    Response {
                        status: msg.status,
                        result: tf2_msgs::action::LookupTransform::Result::from_native(
                            &msg.result,
                        ),
                    }
                }
                fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                    msg.status = self.status;
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
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct FeedbackMessage {
            pub goal_id: unique_identifier_msgs::msg::UUID,
            pub feedback: tf2_msgs::action::LookupTransform::Feedback,
        }
        impl WrappedTypesupport for FeedbackMessage {
            type CStruct = tf2_msgs__action__LookupTransform_FeedbackMessage;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__action__LookupTransform_FeedbackMessage()
                }
            }
            fn create_msg() -> *mut tf2_msgs__action__LookupTransform_FeedbackMessage {
                #[cfg(not(feature = "doc-only"))]
                unsafe { tf2_msgs__action__LookupTransform_FeedbackMessage__create() }
                #[cfg(feature = "doc-only")]
                tf2_msgs__action__LookupTransform_FeedbackMessage__create()
            }
            fn destroy_msg(
                msg: *mut tf2_msgs__action__LookupTransform_FeedbackMessage,
            ) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe {
                    tf2_msgs__action__LookupTransform_FeedbackMessage__destroy(msg)
                };
                #[cfg(feature = "doc-only")]
                tf2_msgs__action__LookupTransform_FeedbackMessage__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> FeedbackMessage {
                FeedbackMessage {
                    goal_id: unique_identifier_msgs::msg::UUID::from_native(
                        &msg.goal_id,
                    ),
                    feedback: tf2_msgs::action::LookupTransform::Feedback::from_native(
                        &msg.feedback,
                    ),
                }
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                self.goal_id.copy_to_native(&mut msg.goal_id);
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
    pub mod FrameGraph {
        use super::super::super::*;
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__tf2_msgs__srv__FrameGraph()
                }
            }
        }
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Request {}
        impl WrappedTypesupport for Request {
            type CStruct = tf2_msgs__srv__FrameGraph_Request;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__srv__FrameGraph_Request()
                }
            }
            fn create_msg() -> *mut tf2_msgs__srv__FrameGraph_Request {
                #[cfg(not(feature = "doc-only"))]
                unsafe { tf2_msgs__srv__FrameGraph_Request__create() }
                #[cfg(feature = "doc-only")] tf2_msgs__srv__FrameGraph_Request__create()
            }
            fn destroy_msg(msg: *mut tf2_msgs__srv__FrameGraph_Request) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { tf2_msgs__srv__FrameGraph_Request__destroy(msg) };
                #[cfg(feature = "doc-only")]
                tf2_msgs__srv__FrameGraph_Request__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Request {
                Request {}
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {}
        }
        impl Default for Request {
            fn default() -> Self {
                let msg_native = WrappedNativeMsg::<Request>::new();
                Request::from_native(&msg_native)
            }
        }
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Response {
            pub frame_yaml: std::string::String,
        }
        impl WrappedTypesupport for Response {
            type CStruct = tf2_msgs__srv__FrameGraph_Response;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__srv__FrameGraph_Response()
                }
            }
            fn create_msg() -> *mut tf2_msgs__srv__FrameGraph_Response {
                #[cfg(not(feature = "doc-only"))]
                unsafe { tf2_msgs__srv__FrameGraph_Response__create() }
                #[cfg(feature = "doc-only")] tf2_msgs__srv__FrameGraph_Response__create()
            }
            fn destroy_msg(msg: *mut tf2_msgs__srv__FrameGraph_Response) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { tf2_msgs__srv__FrameGraph_Response__destroy(msg) };
                #[cfg(feature = "doc-only")]
                tf2_msgs__srv__FrameGraph_Response__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Response {
                Response {
                    frame_yaml: msg.frame_yaml.to_str().to_owned(),
                }
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                msg.frame_yaml.assign(&self.frame_yaml);
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
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct TF2Error {
        pub error: u8,
        pub error_string: std::string::String,
    }
    impl WrappedTypesupport for TF2Error {
        type CStruct = tf2_msgs__msg__TF2Error;
        fn get_ts() -> &'static rosidl_message_type_support_t {
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__msg__TF2Error()
            }
        }
        fn create_msg() -> *mut tf2_msgs__msg__TF2Error {
            #[cfg(not(feature = "doc-only"))]
            unsafe { tf2_msgs__msg__TF2Error__create() }
            #[cfg(feature = "doc-only")] tf2_msgs__msg__TF2Error__create()
        }
        fn destroy_msg(msg: *mut tf2_msgs__msg__TF2Error) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { tf2_msgs__msg__TF2Error__destroy(msg) };
            #[cfg(feature = "doc-only")] tf2_msgs__msg__TF2Error__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> TF2Error {
            TF2Error {
                error: msg.error,
                error_string: msg.error_string.to_str().to_owned(),
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            msg.error = self.error;
            msg.error_string.assign(&self.error_string);
        }
    }
    impl Default for TF2Error {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<TF2Error>::new();
            TF2Error::from_native(&msg_native)
        }
    }
    #[allow(non_upper_case_globals)]
    impl TF2Error {
        pub const CONNECTIVITY_ERROR: _bindgen_ty_229 = tf2_msgs__msg__TF2Error__CONNECTIVITY_ERROR;
        pub const EXTRAPOLATION_ERROR: _bindgen_ty_230 = tf2_msgs__msg__TF2Error__EXTRAPOLATION_ERROR;
        pub const INVALID_ARGUMENT_ERROR: _bindgen_ty_231 = tf2_msgs__msg__TF2Error__INVALID_ARGUMENT_ERROR;
        pub const LOOKUP_ERROR: _bindgen_ty_228 = tf2_msgs__msg__TF2Error__LOOKUP_ERROR;
        pub const NO_ERROR: _bindgen_ty_227 = tf2_msgs__msg__TF2Error__NO_ERROR;
        pub const TIMEOUT_ERROR: _bindgen_ty_232 = tf2_msgs__msg__TF2Error__TIMEOUT_ERROR;
        pub const TRANSFORM_ERROR: _bindgen_ty_233 = tf2_msgs__msg__TF2Error__TRANSFORM_ERROR;
    }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct TFMessage {
        pub transforms: Vec<geometry_msgs::msg::TransformStamped>,
    }
    impl WrappedTypesupport for TFMessage {
        type CStruct = tf2_msgs__msg__TFMessage;
        fn get_ts() -> &'static rosidl_message_type_support_t {
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__tf2_msgs__msg__TFMessage()
            }
        }
        fn create_msg() -> *mut tf2_msgs__msg__TFMessage {
            #[cfg(not(feature = "doc-only"))]
            unsafe { tf2_msgs__msg__TFMessage__create() }
            #[cfg(feature = "doc-only")] tf2_msgs__msg__TFMessage__create()
        }
        fn destroy_msg(msg: *mut tf2_msgs__msg__TFMessage) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { tf2_msgs__msg__TFMessage__destroy(msg) };
            #[cfg(feature = "doc-only")] tf2_msgs__msg__TFMessage__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> TFMessage {
            TFMessage {
                transforms: {
                    let mut temp = Vec::with_capacity(msg.transforms.size);
                    let slice = unsafe {
                        std::slice::from_raw_parts(
                            msg.transforms.data,
                            msg.transforms.size,
                        )
                    };
                    for s in slice {
                        temp.push(geometry_msgs::msg::TransformStamped::from_native(s));
                    }
                    temp
                },
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            unsafe {
                geometry_msgs__msg__TransformStamped__Sequence__fini(
                    &mut msg.transforms,
                );
                geometry_msgs__msg__TransformStamped__Sequence__init(
                    &mut msg.transforms,
                    self.transforms.len(),
                );
                let slice = std::slice::from_raw_parts_mut(
                    msg.transforms.data,
                    msg.transforms.size,
                );
                for (t, s) in slice.iter_mut().zip(&self.transforms) {
                    s.copy_to_native(t);
                }
            }
        }
    }
    impl Default for TFMessage {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<TFMessage>::new();
            TFMessage::from_native(&msg_native)
        }
    }
}
