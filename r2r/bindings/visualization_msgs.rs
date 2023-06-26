pub mod msg {
    use super::super::*;
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct ImageMarker {
        pub header: std_msgs::msg::Header,
        pub ns: std::string::String,
        pub id: i32,
        #[serde(rename = "type")]
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
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__ImageMarker()
            }
        }
        fn create_msg() -> *mut visualization_msgs__msg__ImageMarker {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__ImageMarker__create() }
            #[cfg(feature = "doc-only")] visualization_msgs__msg__ImageMarker__create()
        }
        fn destroy_msg(msg: *mut visualization_msgs__msg__ImageMarker) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__ImageMarker__destroy(msg) };
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__ImageMarker__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> ImageMarker {
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
                points: {
                    let mut temp = Vec::with_capacity(msg.points.size);
                    let slice = unsafe {
                        std::slice::from_raw_parts(msg.points.data, msg.points.size)
                    };
                    for s in slice {
                        temp.push(geometry_msgs::msg::Point::from_native(s));
                    }
                    temp
                },
                outline_colors: {
                    let mut temp = Vec::with_capacity(msg.outline_colors.size);
                    let slice = unsafe {
                        std::slice::from_raw_parts(
                            msg.outline_colors.data,
                            msg.outline_colors.size,
                        )
                    };
                    for s in slice {
                        temp.push(std_msgs::msg::ColorRGBA::from_native(s));
                    }
                    temp
                },
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            self.header.copy_to_native(&mut msg.header);
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
            unsafe {
                geometry_msgs__msg__Point__Sequence__fini(&mut msg.points);
                geometry_msgs__msg__Point__Sequence__init(
                    &mut msg.points,
                    self.points.len(),
                );
                let slice = std::slice::from_raw_parts_mut(
                    msg.points.data,
                    msg.points.size,
                );
                for (t, s) in slice.iter_mut().zip(&self.points) {
                    s.copy_to_native(t);
                }
            }
            unsafe {
                std_msgs__msg__ColorRGBA__Sequence__fini(&mut msg.outline_colors);
                std_msgs__msg__ColorRGBA__Sequence__init(
                    &mut msg.outline_colors,
                    self.outline_colors.len(),
                );
                let slice = std::slice::from_raw_parts_mut(
                    msg.outline_colors.data,
                    msg.outline_colors.size,
                );
                for (t, s) in slice.iter_mut().zip(&self.outline_colors) {
                    s.copy_to_native(t);
                }
            }
        }
    }
    impl Default for ImageMarker {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<ImageMarker>::new();
            ImageMarker::from_native(&msg_native)
        }
    }
    #[allow(non_upper_case_globals)]
    impl ImageMarker {
        pub const ADD: _bindgen_ty_239 = visualization_msgs__msg__ImageMarker__ADD;
        pub const CIRCLE: _bindgen_ty_234 = visualization_msgs__msg__ImageMarker__CIRCLE;
        pub const LINE_LIST: _bindgen_ty_236 = visualization_msgs__msg__ImageMarker__LINE_LIST;
        pub const LINE_STRIP: _bindgen_ty_235 = visualization_msgs__msg__ImageMarker__LINE_STRIP;
        pub const POINTS: _bindgen_ty_238 = visualization_msgs__msg__ImageMarker__POINTS;
        pub const POLYGON: _bindgen_ty_237 = visualization_msgs__msg__ImageMarker__POLYGON;
        pub const REMOVE: _bindgen_ty_240 = visualization_msgs__msg__ImageMarker__REMOVE;
    }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
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
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__InteractiveMarker()
            }
        }
        fn create_msg() -> *mut visualization_msgs__msg__InteractiveMarker {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__InteractiveMarker__create() }
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__InteractiveMarker__create()
        }
        fn destroy_msg(msg: *mut visualization_msgs__msg__InteractiveMarker) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__InteractiveMarker__destroy(msg) };
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__InteractiveMarker__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> InteractiveMarker {
            InteractiveMarker {
                header: std_msgs::msg::Header::from_native(&msg.header),
                pose: geometry_msgs::msg::Pose::from_native(&msg.pose),
                name: msg.name.to_str().to_owned(),
                description: msg.description.to_str().to_owned(),
                scale: msg.scale,
                menu_entries: {
                    let mut temp = Vec::with_capacity(msg.menu_entries.size);
                    let slice = unsafe {
                        std::slice::from_raw_parts(
                            msg.menu_entries.data,
                            msg.menu_entries.size,
                        )
                    };
                    for s in slice {
                        temp.push(visualization_msgs::msg::MenuEntry::from_native(s));
                    }
                    temp
                },
                controls: {
                    let mut temp = Vec::with_capacity(msg.controls.size);
                    let slice = unsafe {
                        std::slice::from_raw_parts(msg.controls.data, msg.controls.size)
                    };
                    for s in slice {
                        temp.push(
                            visualization_msgs::msg::InteractiveMarkerControl::from_native(
                                s,
                            ),
                        );
                    }
                    temp
                },
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            self.header.copy_to_native(&mut msg.header);
            self.pose.copy_to_native(&mut msg.pose);
            msg.name.assign(&self.name);
            msg.description.assign(&self.description);
            msg.scale = self.scale;
            unsafe {
                visualization_msgs__msg__MenuEntry__Sequence__fini(
                    &mut msg.menu_entries,
                );
                visualization_msgs__msg__MenuEntry__Sequence__init(
                    &mut msg.menu_entries,
                    self.menu_entries.len(),
                );
                let slice = std::slice::from_raw_parts_mut(
                    msg.menu_entries.data,
                    msg.menu_entries.size,
                );
                for (t, s) in slice.iter_mut().zip(&self.menu_entries) {
                    s.copy_to_native(t);
                }
            }
            unsafe {
                visualization_msgs__msg__InteractiveMarkerControl__Sequence__fini(
                    &mut msg.controls,
                );
                visualization_msgs__msg__InteractiveMarkerControl__Sequence__init(
                    &mut msg.controls,
                    self.controls.len(),
                );
                let slice = std::slice::from_raw_parts_mut(
                    msg.controls.data,
                    msg.controls.size,
                );
                for (t, s) in slice.iter_mut().zip(&self.controls) {
                    s.copy_to_native(t);
                }
            }
        }
    }
    impl Default for InteractiveMarker {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<InteractiveMarker>::new();
            InteractiveMarker::from_native(&msg_native)
        }
    }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
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
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__InteractiveMarkerControl()
            }
        }
        fn create_msg() -> *mut visualization_msgs__msg__InteractiveMarkerControl {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__InteractiveMarkerControl__create() }
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__InteractiveMarkerControl__create()
        }
        fn destroy_msg(
            msg: *mut visualization_msgs__msg__InteractiveMarkerControl,
        ) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__InteractiveMarkerControl__destroy(msg) };
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__InteractiveMarkerControl__destroy(msg)
        }
        fn from_native(
            #[allow(unused)]
            msg: &Self::CStruct,
        ) -> InteractiveMarkerControl {
            InteractiveMarkerControl {
                name: msg.name.to_str().to_owned(),
                orientation: geometry_msgs::msg::Quaternion::from_native(
                    &msg.orientation,
                ),
                orientation_mode: msg.orientation_mode,
                interaction_mode: msg.interaction_mode,
                always_visible: msg.always_visible,
                markers: {
                    let mut temp = Vec::with_capacity(msg.markers.size);
                    let slice = unsafe {
                        std::slice::from_raw_parts(msg.markers.data, msg.markers.size)
                    };
                    for s in slice {
                        temp.push(visualization_msgs::msg::Marker::from_native(s));
                    }
                    temp
                },
                independent_marker_orientation: msg.independent_marker_orientation,
                description: msg.description.to_str().to_owned(),
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            msg.name.assign(&self.name);
            self.orientation.copy_to_native(&mut msg.orientation);
            msg.orientation_mode = self.orientation_mode;
            msg.interaction_mode = self.interaction_mode;
            msg.always_visible = self.always_visible;
            unsafe {
                visualization_msgs__msg__Marker__Sequence__fini(&mut msg.markers);
                visualization_msgs__msg__Marker__Sequence__init(
                    &mut msg.markers,
                    self.markers.len(),
                );
                let slice = std::slice::from_raw_parts_mut(
                    msg.markers.data,
                    msg.markers.size,
                );
                for (t, s) in slice.iter_mut().zip(&self.markers) {
                    s.copy_to_native(t);
                }
            }
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
    #[allow(non_upper_case_globals)]
    impl InteractiveMarkerControl {
        pub const BUTTON: _bindgen_ty_249 = visualization_msgs__msg__InteractiveMarkerControl__BUTTON;
        pub const FIXED: _bindgen_ty_245 = visualization_msgs__msg__InteractiveMarkerControl__FIXED;
        pub const INHERIT: _bindgen_ty_244 = visualization_msgs__msg__InteractiveMarkerControl__INHERIT;
        pub const MENU: _bindgen_ty_248 = visualization_msgs__msg__InteractiveMarkerControl__MENU;
        pub const MOVE_3D: _bindgen_ty_254 = visualization_msgs__msg__InteractiveMarkerControl__MOVE_3D;
        pub const MOVE_AXIS: _bindgen_ty_250 = visualization_msgs__msg__InteractiveMarkerControl__MOVE_AXIS;
        pub const MOVE_PLANE: _bindgen_ty_251 = visualization_msgs__msg__InteractiveMarkerControl__MOVE_PLANE;
        pub const MOVE_ROTATE: _bindgen_ty_253 = visualization_msgs__msg__InteractiveMarkerControl__MOVE_ROTATE;
        pub const MOVE_ROTATE_3D: _bindgen_ty_256 = visualization_msgs__msg__InteractiveMarkerControl__MOVE_ROTATE_3D;
        pub const NONE: _bindgen_ty_247 = visualization_msgs__msg__InteractiveMarkerControl__NONE;
        pub const ROTATE_3D: _bindgen_ty_255 = visualization_msgs__msg__InteractiveMarkerControl__ROTATE_3D;
        pub const ROTATE_AXIS: _bindgen_ty_252 = visualization_msgs__msg__InteractiveMarkerControl__ROTATE_AXIS;
        pub const VIEW_FACING: _bindgen_ty_246 = visualization_msgs__msg__InteractiveMarkerControl__VIEW_FACING;
    }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
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
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__InteractiveMarkerFeedback()
            }
        }
        fn create_msg() -> *mut visualization_msgs__msg__InteractiveMarkerFeedback {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__InteractiveMarkerFeedback__create() }
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__InteractiveMarkerFeedback__create()
        }
        fn destroy_msg(
            msg: *mut visualization_msgs__msg__InteractiveMarkerFeedback,
        ) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__InteractiveMarkerFeedback__destroy(msg) };
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__InteractiveMarkerFeedback__destroy(msg)
        }
        fn from_native(
            #[allow(unused)]
            msg: &Self::CStruct,
        ) -> InteractiveMarkerFeedback {
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
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            self.header.copy_to_native(&mut msg.header);
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
    #[allow(non_upper_case_globals)]
    impl InteractiveMarkerFeedback {
        pub const BUTTON_CLICK: _bindgen_ty_276 = visualization_msgs__msg__InteractiveMarkerFeedback__BUTTON_CLICK;
        pub const KEEP_ALIVE: _bindgen_ty_273 = visualization_msgs__msg__InteractiveMarkerFeedback__KEEP_ALIVE;
        pub const MENU_SELECT: _bindgen_ty_275 = visualization_msgs__msg__InteractiveMarkerFeedback__MENU_SELECT;
        pub const MOUSE_DOWN: _bindgen_ty_277 = visualization_msgs__msg__InteractiveMarkerFeedback__MOUSE_DOWN;
        pub const MOUSE_UP: _bindgen_ty_278 = visualization_msgs__msg__InteractiveMarkerFeedback__MOUSE_UP;
        pub const POSE_UPDATE: _bindgen_ty_274 = visualization_msgs__msg__InteractiveMarkerFeedback__POSE_UPDATE;
    }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct InteractiveMarkerInit {
        pub server_id: std::string::String,
        pub seq_num: u64,
        pub markers: Vec<visualization_msgs::msg::InteractiveMarker>,
    }
    impl WrappedTypesupport for InteractiveMarkerInit {
        type CStruct = visualization_msgs__msg__InteractiveMarkerInit;
        fn get_ts() -> &'static rosidl_message_type_support_t {
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__InteractiveMarkerInit()
            }
        }
        fn create_msg() -> *mut visualization_msgs__msg__InteractiveMarkerInit {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__InteractiveMarkerInit__create() }
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__InteractiveMarkerInit__create()
        }
        fn destroy_msg(msg: *mut visualization_msgs__msg__InteractiveMarkerInit) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__InteractiveMarkerInit__destroy(msg) };
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__InteractiveMarkerInit__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> InteractiveMarkerInit {
            InteractiveMarkerInit {
                server_id: msg.server_id.to_str().to_owned(),
                seq_num: msg.seq_num,
                markers: {
                    let mut temp = Vec::with_capacity(msg.markers.size);
                    let slice = unsafe {
                        std::slice::from_raw_parts(msg.markers.data, msg.markers.size)
                    };
                    for s in slice {
                        temp.push(
                            visualization_msgs::msg::InteractiveMarker::from_native(s),
                        );
                    }
                    temp
                },
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            msg.server_id.assign(&self.server_id);
            msg.seq_num = self.seq_num;
            unsafe {
                visualization_msgs__msg__InteractiveMarker__Sequence__fini(
                    &mut msg.markers,
                );
                visualization_msgs__msg__InteractiveMarker__Sequence__init(
                    &mut msg.markers,
                    self.markers.len(),
                );
                let slice = std::slice::from_raw_parts_mut(
                    msg.markers.data,
                    msg.markers.size,
                );
                for (t, s) in slice.iter_mut().zip(&self.markers) {
                    s.copy_to_native(t);
                }
            }
        }
    }
    impl Default for InteractiveMarkerInit {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<InteractiveMarkerInit>::new();
            InteractiveMarkerInit::from_native(&msg_native)
        }
    }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct InteractiveMarkerPose {
        pub header: std_msgs::msg::Header,
        pub pose: geometry_msgs::msg::Pose,
        pub name: std::string::String,
    }
    impl WrappedTypesupport for InteractiveMarkerPose {
        type CStruct = visualization_msgs__msg__InteractiveMarkerPose;
        fn get_ts() -> &'static rosidl_message_type_support_t {
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__InteractiveMarkerPose()
            }
        }
        fn create_msg() -> *mut visualization_msgs__msg__InteractiveMarkerPose {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__InteractiveMarkerPose__create() }
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__InteractiveMarkerPose__create()
        }
        fn destroy_msg(msg: *mut visualization_msgs__msg__InteractiveMarkerPose) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__InteractiveMarkerPose__destroy(msg) };
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__InteractiveMarkerPose__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> InteractiveMarkerPose {
            InteractiveMarkerPose {
                header: std_msgs::msg::Header::from_native(&msg.header),
                pose: geometry_msgs::msg::Pose::from_native(&msg.pose),
                name: msg.name.to_str().to_owned(),
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            self.header.copy_to_native(&mut msg.header);
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
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct InteractiveMarkerUpdate {
        pub server_id: std::string::String,
        pub seq_num: u64,
        #[serde(rename = "type")]
        pub type_: u8,
        pub markers: Vec<visualization_msgs::msg::InteractiveMarker>,
        pub poses: Vec<visualization_msgs::msg::InteractiveMarkerPose>,
        pub erases: Vec<std::string::String>,
    }
    impl WrappedTypesupport for InteractiveMarkerUpdate {
        type CStruct = visualization_msgs__msg__InteractiveMarkerUpdate;
        fn get_ts() -> &'static rosidl_message_type_support_t {
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__InteractiveMarkerUpdate()
            }
        }
        fn create_msg() -> *mut visualization_msgs__msg__InteractiveMarkerUpdate {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__InteractiveMarkerUpdate__create() }
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__InteractiveMarkerUpdate__create()
        }
        fn destroy_msg(
            msg: *mut visualization_msgs__msg__InteractiveMarkerUpdate,
        ) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__InteractiveMarkerUpdate__destroy(msg) };
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__InteractiveMarkerUpdate__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> InteractiveMarkerUpdate {
            InteractiveMarkerUpdate {
                server_id: msg.server_id.to_str().to_owned(),
                seq_num: msg.seq_num,
                type_: msg.type_,
                markers: {
                    let mut temp = Vec::with_capacity(msg.markers.size);
                    let slice = unsafe {
                        std::slice::from_raw_parts(msg.markers.data, msg.markers.size)
                    };
                    for s in slice {
                        temp.push(
                            visualization_msgs::msg::InteractiveMarker::from_native(s),
                        );
                    }
                    temp
                },
                poses: {
                    let mut temp = Vec::with_capacity(msg.poses.size);
                    let slice = unsafe {
                        std::slice::from_raw_parts(msg.poses.data, msg.poses.size)
                    };
                    for s in slice {
                        temp.push(
                            visualization_msgs::msg::InteractiveMarkerPose::from_native(
                                s,
                            ),
                        );
                    }
                    temp
                },
                erases: msg.erases.to_vec(),
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            msg.server_id.assign(&self.server_id);
            msg.seq_num = self.seq_num;
            msg.type_ = self.type_;
            unsafe {
                visualization_msgs__msg__InteractiveMarker__Sequence__fini(
                    &mut msg.markers,
                );
                visualization_msgs__msg__InteractiveMarker__Sequence__init(
                    &mut msg.markers,
                    self.markers.len(),
                );
                let slice = std::slice::from_raw_parts_mut(
                    msg.markers.data,
                    msg.markers.size,
                );
                for (t, s) in slice.iter_mut().zip(&self.markers) {
                    s.copy_to_native(t);
                }
            }
            unsafe {
                visualization_msgs__msg__InteractiveMarkerPose__Sequence__fini(
                    &mut msg.poses,
                );
                visualization_msgs__msg__InteractiveMarkerPose__Sequence__init(
                    &mut msg.poses,
                    self.poses.len(),
                );
                let slice = std::slice::from_raw_parts_mut(
                    msg.poses.data,
                    msg.poses.size,
                );
                for (t, s) in slice.iter_mut().zip(&self.poses) {
                    s.copy_to_native(t);
                }
            }
            msg.erases.update(&self.erases);
        }
    }
    impl Default for InteractiveMarkerUpdate {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<InteractiveMarkerUpdate>::new();
            InteractiveMarkerUpdate::from_native(&msg_native)
        }
    }
    #[allow(non_upper_case_globals)]
    impl InteractiveMarkerUpdate {
        pub const KEEP_ALIVE: _bindgen_ty_279 = visualization_msgs__msg__InteractiveMarkerUpdate__KEEP_ALIVE;
        pub const UPDATE: _bindgen_ty_280 = visualization_msgs__msg__InteractiveMarkerUpdate__UPDATE;
    }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct Marker {
        pub header: std_msgs::msg::Header,
        pub ns: std::string::String,
        pub id: i32,
        #[serde(rename = "type")]
        pub type_: i32,
        pub action: i32,
        pub pose: geometry_msgs::msg::Pose,
        pub scale: geometry_msgs::msg::Vector3,
        pub color: std_msgs::msg::ColorRGBA,
        pub lifetime: builtin_interfaces::msg::Duration,
        pub frame_locked: bool,
        pub points: Vec<geometry_msgs::msg::Point>,
        pub colors: Vec<std_msgs::msg::ColorRGBA>,
        pub texture_resource: std::string::String,
        pub texture: sensor_msgs::msg::CompressedImage,
        pub uv_coordinates: Vec<visualization_msgs::msg::UVCoordinate>,
        pub text: std::string::String,
        pub mesh_resource: std::string::String,
        pub mesh_file: visualization_msgs::msg::MeshFile,
        pub mesh_use_embedded_materials: bool,
    }
    impl WrappedTypesupport for Marker {
        type CStruct = visualization_msgs__msg__Marker;
        fn get_ts() -> &'static rosidl_message_type_support_t {
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__Marker()
            }
        }
        fn create_msg() -> *mut visualization_msgs__msg__Marker {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__Marker__create() }
            #[cfg(feature = "doc-only")] visualization_msgs__msg__Marker__create()
        }
        fn destroy_msg(msg: *mut visualization_msgs__msg__Marker) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__Marker__destroy(msg) };
            #[cfg(feature = "doc-only")] visualization_msgs__msg__Marker__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Marker {
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
                points: {
                    let mut temp = Vec::with_capacity(msg.points.size);
                    let slice = unsafe {
                        std::slice::from_raw_parts(msg.points.data, msg.points.size)
                    };
                    for s in slice {
                        temp.push(geometry_msgs::msg::Point::from_native(s));
                    }
                    temp
                },
                colors: {
                    let mut temp = Vec::with_capacity(msg.colors.size);
                    let slice = unsafe {
                        std::slice::from_raw_parts(msg.colors.data, msg.colors.size)
                    };
                    for s in slice {
                        temp.push(std_msgs::msg::ColorRGBA::from_native(s));
                    }
                    temp
                },
                texture_resource: msg.texture_resource.to_str().to_owned(),
                texture: sensor_msgs::msg::CompressedImage::from_native(&msg.texture),
                uv_coordinates: {
                    let mut temp = Vec::with_capacity(msg.uv_coordinates.size);
                    let slice = unsafe {
                        std::slice::from_raw_parts(
                            msg.uv_coordinates.data,
                            msg.uv_coordinates.size,
                        )
                    };
                    for s in slice {
                        temp.push(visualization_msgs::msg::UVCoordinate::from_native(s));
                    }
                    temp
                },
                text: msg.text.to_str().to_owned(),
                mesh_resource: msg.mesh_resource.to_str().to_owned(),
                mesh_file: visualization_msgs::msg::MeshFile::from_native(
                    &msg.mesh_file,
                ),
                mesh_use_embedded_materials: msg.mesh_use_embedded_materials,
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            self.header.copy_to_native(&mut msg.header);
            msg.ns.assign(&self.ns);
            msg.id = self.id;
            msg.type_ = self.type_;
            msg.action = self.action;
            self.pose.copy_to_native(&mut msg.pose);
            self.scale.copy_to_native(&mut msg.scale);
            self.color.copy_to_native(&mut msg.color);
            self.lifetime.copy_to_native(&mut msg.lifetime);
            msg.frame_locked = self.frame_locked;
            unsafe {
                geometry_msgs__msg__Point__Sequence__fini(&mut msg.points);
                geometry_msgs__msg__Point__Sequence__init(
                    &mut msg.points,
                    self.points.len(),
                );
                let slice = std::slice::from_raw_parts_mut(
                    msg.points.data,
                    msg.points.size,
                );
                for (t, s) in slice.iter_mut().zip(&self.points) {
                    s.copy_to_native(t);
                }
            }
            unsafe {
                std_msgs__msg__ColorRGBA__Sequence__fini(&mut msg.colors);
                std_msgs__msg__ColorRGBA__Sequence__init(
                    &mut msg.colors,
                    self.colors.len(),
                );
                let slice = std::slice::from_raw_parts_mut(
                    msg.colors.data,
                    msg.colors.size,
                );
                for (t, s) in slice.iter_mut().zip(&self.colors) {
                    s.copy_to_native(t);
                }
            }
            msg.texture_resource.assign(&self.texture_resource);
            self.texture.copy_to_native(&mut msg.texture);
            unsafe {
                visualization_msgs__msg__UVCoordinate__Sequence__fini(
                    &mut msg.uv_coordinates,
                );
                visualization_msgs__msg__UVCoordinate__Sequence__init(
                    &mut msg.uv_coordinates,
                    self.uv_coordinates.len(),
                );
                let slice = std::slice::from_raw_parts_mut(
                    msg.uv_coordinates.data,
                    msg.uv_coordinates.size,
                );
                for (t, s) in slice.iter_mut().zip(&self.uv_coordinates) {
                    s.copy_to_native(t);
                }
            }
            msg.text.assign(&self.text);
            msg.mesh_resource.assign(&self.mesh_resource);
            self.mesh_file.copy_to_native(&mut msg.mesh_file);
            msg.mesh_use_embedded_materials = self.mesh_use_embedded_materials;
        }
    }
    impl Default for Marker {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<Marker>::new();
            Marker::from_native(&msg_native)
        }
    }
    #[allow(non_upper_case_globals)]
    impl Marker {
        pub const ADD: _bindgen_ty_269 = visualization_msgs__msg__Marker__ADD;
        pub const ARROW: _bindgen_ty_257 = visualization_msgs__msg__Marker__ARROW;
        pub const CUBE: _bindgen_ty_258 = visualization_msgs__msg__Marker__CUBE;
        pub const CUBE_LIST: _bindgen_ty_263 = visualization_msgs__msg__Marker__CUBE_LIST;
        pub const CYLINDER: _bindgen_ty_260 = visualization_msgs__msg__Marker__CYLINDER;
        pub const DELETE: _bindgen_ty_271 = visualization_msgs__msg__Marker__DELETE;
        pub const DELETEALL: _bindgen_ty_272 = visualization_msgs__msg__Marker__DELETEALL;
        pub const LINE_LIST: _bindgen_ty_262 = visualization_msgs__msg__Marker__LINE_LIST;
        pub const LINE_STRIP: _bindgen_ty_261 = visualization_msgs__msg__Marker__LINE_STRIP;
        pub const MESH_RESOURCE: _bindgen_ty_267 = visualization_msgs__msg__Marker__MESH_RESOURCE;
        pub const MODIFY: _bindgen_ty_270 = visualization_msgs__msg__Marker__MODIFY;
        pub const POINTS: _bindgen_ty_265 = visualization_msgs__msg__Marker__POINTS;
        pub const SPHERE: _bindgen_ty_259 = visualization_msgs__msg__Marker__SPHERE;
        pub const SPHERE_LIST: _bindgen_ty_264 = visualization_msgs__msg__Marker__SPHERE_LIST;
        pub const TEXT_VIEW_FACING: _bindgen_ty_266 = visualization_msgs__msg__Marker__TEXT_VIEW_FACING;
        pub const TRIANGLE_LIST: _bindgen_ty_268 = visualization_msgs__msg__Marker__TRIANGLE_LIST;
    }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct MarkerArray {
        pub markers: Vec<visualization_msgs::msg::Marker>,
    }
    impl WrappedTypesupport for MarkerArray {
        type CStruct = visualization_msgs__msg__MarkerArray;
        fn get_ts() -> &'static rosidl_message_type_support_t {
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__MarkerArray()
            }
        }
        fn create_msg() -> *mut visualization_msgs__msg__MarkerArray {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__MarkerArray__create() }
            #[cfg(feature = "doc-only")] visualization_msgs__msg__MarkerArray__create()
        }
        fn destroy_msg(msg: *mut visualization_msgs__msg__MarkerArray) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__MarkerArray__destroy(msg) };
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__MarkerArray__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> MarkerArray {
            MarkerArray {
                markers: {
                    let mut temp = Vec::with_capacity(msg.markers.size);
                    let slice = unsafe {
                        std::slice::from_raw_parts(msg.markers.data, msg.markers.size)
                    };
                    for s in slice {
                        temp.push(visualization_msgs::msg::Marker::from_native(s));
                    }
                    temp
                },
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            unsafe {
                visualization_msgs__msg__Marker__Sequence__fini(&mut msg.markers);
                visualization_msgs__msg__Marker__Sequence__init(
                    &mut msg.markers,
                    self.markers.len(),
                );
                let slice = std::slice::from_raw_parts_mut(
                    msg.markers.data,
                    msg.markers.size,
                );
                for (t, s) in slice.iter_mut().zip(&self.markers) {
                    s.copy_to_native(t);
                }
            }
        }
    }
    impl Default for MarkerArray {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<MarkerArray>::new();
            MarkerArray::from_native(&msg_native)
        }
    }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
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
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__MenuEntry()
            }
        }
        fn create_msg() -> *mut visualization_msgs__msg__MenuEntry {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__MenuEntry__create() }
            #[cfg(feature = "doc-only")] visualization_msgs__msg__MenuEntry__create()
        }
        fn destroy_msg(msg: *mut visualization_msgs__msg__MenuEntry) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__MenuEntry__destroy(msg) };
            #[cfg(feature = "doc-only")] visualization_msgs__msg__MenuEntry__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> MenuEntry {
            MenuEntry {
                id: msg.id,
                parent_id: msg.parent_id,
                title: msg.title.to_str().to_owned(),
                command: msg.command.to_str().to_owned(),
                command_type: msg.command_type,
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            msg.id = self.id;
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
    #[allow(non_upper_case_globals)]
    impl MenuEntry {
        pub const FEEDBACK: _bindgen_ty_241 = visualization_msgs__msg__MenuEntry__FEEDBACK;
        pub const ROSLAUNCH: _bindgen_ty_243 = visualization_msgs__msg__MenuEntry__ROSLAUNCH;
        pub const ROSRUN: _bindgen_ty_242 = visualization_msgs__msg__MenuEntry__ROSRUN;
    }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct MeshFile {
        pub filename: std::string::String,
        pub data: Vec<u8>,
    }
    impl WrappedTypesupport for MeshFile {
        type CStruct = visualization_msgs__msg__MeshFile;
        fn get_ts() -> &'static rosidl_message_type_support_t {
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__MeshFile()
            }
        }
        fn create_msg() -> *mut visualization_msgs__msg__MeshFile {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__MeshFile__create() }
            #[cfg(feature = "doc-only")] visualization_msgs__msg__MeshFile__create()
        }
        fn destroy_msg(msg: *mut visualization_msgs__msg__MeshFile) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__MeshFile__destroy(msg) };
            #[cfg(feature = "doc-only")] visualization_msgs__msg__MeshFile__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> MeshFile {
            MeshFile {
                filename: msg.filename.to_str().to_owned(),
                data: msg.data.to_vec(),
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            msg.filename.assign(&self.filename);
            msg.data.update(&self.data);
        }
    }
    impl Default for MeshFile {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<MeshFile>::new();
            MeshFile::from_native(&msg_native)
        }
    }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct UVCoordinate {
        pub u: f32,
        pub v: f32,
    }
    impl WrappedTypesupport for UVCoordinate {
        type CStruct = visualization_msgs__msg__UVCoordinate;
        fn get_ts() -> &'static rosidl_message_type_support_t {
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__msg__UVCoordinate()
            }
        }
        fn create_msg() -> *mut visualization_msgs__msg__UVCoordinate {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__UVCoordinate__create() }
            #[cfg(feature = "doc-only")] visualization_msgs__msg__UVCoordinate__create()
        }
        fn destroy_msg(msg: *mut visualization_msgs__msg__UVCoordinate) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { visualization_msgs__msg__UVCoordinate__destroy(msg) };
            #[cfg(feature = "doc-only")]
            visualization_msgs__msg__UVCoordinate__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> UVCoordinate {
            UVCoordinate { u: msg.u, v: msg.v }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            msg.u = self.u;
            msg.v = self.v;
        }
    }
    impl Default for UVCoordinate {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<UVCoordinate>::new();
            UVCoordinate::from_native(&msg_native)
        }
    }
}
pub mod srv {
    #[allow(non_snake_case)]
    pub mod GetInteractiveMarkers {
        use super::super::super::*;
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
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
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Request {}
        impl WrappedTypesupport for Request {
            type CStruct = visualization_msgs__srv__GetInteractiveMarkers_Request;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__srv__GetInteractiveMarkers_Request()
                }
            }
            fn create_msg() -> *mut visualization_msgs__srv__GetInteractiveMarkers_Request {
                #[cfg(not(feature = "doc-only"))]
                unsafe {
                    visualization_msgs__srv__GetInteractiveMarkers_Request__create()
                }
                #[cfg(feature = "doc-only")]
                visualization_msgs__srv__GetInteractiveMarkers_Request__create()
            }
            fn destroy_msg(
                msg: *mut visualization_msgs__srv__GetInteractiveMarkers_Request,
            ) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe {
                    visualization_msgs__srv__GetInteractiveMarkers_Request__destroy(msg)
                };
                #[cfg(feature = "doc-only")]
                visualization_msgs__srv__GetInteractiveMarkers_Request__destroy(msg)
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
            pub sequence_number: u64,
            pub markers: Vec<visualization_msgs::msg::InteractiveMarker>,
        }
        impl WrappedTypesupport for Response {
            type CStruct = visualization_msgs__srv__GetInteractiveMarkers_Response;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__visualization_msgs__srv__GetInteractiveMarkers_Response()
                }
            }
            fn create_msg() -> *mut visualization_msgs__srv__GetInteractiveMarkers_Response {
                #[cfg(not(feature = "doc-only"))]
                unsafe {
                    visualization_msgs__srv__GetInteractiveMarkers_Response__create()
                }
                #[cfg(feature = "doc-only")]
                visualization_msgs__srv__GetInteractiveMarkers_Response__create()
            }
            fn destroy_msg(
                msg: *mut visualization_msgs__srv__GetInteractiveMarkers_Response,
            ) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe {
                    visualization_msgs__srv__GetInteractiveMarkers_Response__destroy(msg)
                };
                #[cfg(feature = "doc-only")]
                visualization_msgs__srv__GetInteractiveMarkers_Response__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Response {
                Response {
                    sequence_number: msg.sequence_number,
                    markers: {
                        let mut temp = Vec::with_capacity(msg.markers.size);
                        let slice = unsafe {
                            std::slice::from_raw_parts(
                                msg.markers.data,
                                msg.markers.size,
                            )
                        };
                        for s in slice {
                            temp.push(
                                visualization_msgs::msg::InteractiveMarker::from_native(s),
                            );
                        }
                        temp
                    },
                }
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                msg.sequence_number = self.sequence_number;
                unsafe {
                    visualization_msgs__msg__InteractiveMarker__Sequence__fini(
                        &mut msg.markers,
                    );
                    visualization_msgs__msg__InteractiveMarker__Sequence__init(
                        &mut msg.markers,
                        self.markers.len(),
                    );
                    let slice = std::slice::from_raw_parts_mut(
                        msg.markers.data,
                        msg.markers.size,
                    );
                    for (t, s) in slice.iter_mut().zip(&self.markers) {
                        s.copy_to_native(t);
                    }
                }
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
