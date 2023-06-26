pub mod srv {
    #[allow(non_snake_case)]
    pub mod GetMapROI {
        use super::super::super::*;
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__map_msgs__srv__GetMapROI()
                }
            }
        }
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Request {
            pub x: f64,
            pub y: f64,
            pub l_x: f64,
            pub l_y: f64,
        }
        impl WrappedTypesupport for Request {
            type CStruct = map_msgs__srv__GetMapROI_Request;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__srv__GetMapROI_Request()
                }
            }
            fn create_msg() -> *mut map_msgs__srv__GetMapROI_Request {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__GetMapROI_Request__create() }
                #[cfg(feature = "doc-only")] map_msgs__srv__GetMapROI_Request__create()
            }
            fn destroy_msg(msg: *mut map_msgs__srv__GetMapROI_Request) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__GetMapROI_Request__destroy(msg) };
                #[cfg(feature = "doc-only")]
                map_msgs__srv__GetMapROI_Request__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Request {
                Request {
                    x: msg.x,
                    y: msg.y,
                    l_x: msg.l_x,
                    l_y: msg.l_y,
                }
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                msg.x = self.x;
                msg.y = self.y;
                msg.l_x = self.l_x;
                msg.l_y = self.l_y;
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
            pub sub_map: nav_msgs::msg::OccupancyGrid,
        }
        impl WrappedTypesupport for Response {
            type CStruct = map_msgs__srv__GetMapROI_Response;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__srv__GetMapROI_Response()
                }
            }
            fn create_msg() -> *mut map_msgs__srv__GetMapROI_Response {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__GetMapROI_Response__create() }
                #[cfg(feature = "doc-only")] map_msgs__srv__GetMapROI_Response__create()
            }
            fn destroy_msg(msg: *mut map_msgs__srv__GetMapROI_Response) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__GetMapROI_Response__destroy(msg) };
                #[cfg(feature = "doc-only")]
                map_msgs__srv__GetMapROI_Response__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Response {
                Response {
                    sub_map: nav_msgs::msg::OccupancyGrid::from_native(&msg.sub_map),
                }
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                self.sub_map.copy_to_native(&mut msg.sub_map);
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
    pub mod GetPointMap {
        use super::super::super::*;
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__map_msgs__srv__GetPointMap()
                }
            }
        }
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Request {}
        impl WrappedTypesupport for Request {
            type CStruct = map_msgs__srv__GetPointMap_Request;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__srv__GetPointMap_Request()
                }
            }
            fn create_msg() -> *mut map_msgs__srv__GetPointMap_Request {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__GetPointMap_Request__create() }
                #[cfg(feature = "doc-only")] map_msgs__srv__GetPointMap_Request__create()
            }
            fn destroy_msg(msg: *mut map_msgs__srv__GetPointMap_Request) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__GetPointMap_Request__destroy(msg) };
                #[cfg(feature = "doc-only")]
                map_msgs__srv__GetPointMap_Request__destroy(msg)
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
            pub map: sensor_msgs::msg::PointCloud2,
        }
        impl WrappedTypesupport for Response {
            type CStruct = map_msgs__srv__GetPointMap_Response;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__srv__GetPointMap_Response()
                }
            }
            fn create_msg() -> *mut map_msgs__srv__GetPointMap_Response {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__GetPointMap_Response__create() }
                #[cfg(feature = "doc-only")]
                map_msgs__srv__GetPointMap_Response__create()
            }
            fn destroy_msg(msg: *mut map_msgs__srv__GetPointMap_Response) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__GetPointMap_Response__destroy(msg) };
                #[cfg(feature = "doc-only")]
                map_msgs__srv__GetPointMap_Response__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Response {
                Response {
                    map: sensor_msgs::msg::PointCloud2::from_native(&msg.map),
                }
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
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
    pub mod GetPointMapROI {
        use super::super::super::*;
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__map_msgs__srv__GetPointMapROI()
                }
            }
        }
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Request {
            pub x: f64,
            pub y: f64,
            pub z: f64,
            pub r: f64,
            pub l_x: f64,
            pub l_y: f64,
            pub l_z: f64,
        }
        impl WrappedTypesupport for Request {
            type CStruct = map_msgs__srv__GetPointMapROI_Request;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__srv__GetPointMapROI_Request()
                }
            }
            fn create_msg() -> *mut map_msgs__srv__GetPointMapROI_Request {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__GetPointMapROI_Request__create() }
                #[cfg(feature = "doc-only")]
                map_msgs__srv__GetPointMapROI_Request__create()
            }
            fn destroy_msg(msg: *mut map_msgs__srv__GetPointMapROI_Request) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__GetPointMapROI_Request__destroy(msg) };
                #[cfg(feature = "doc-only")]
                map_msgs__srv__GetPointMapROI_Request__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Request {
                Request {
                    x: msg.x,
                    y: msg.y,
                    z: msg.z,
                    r: msg.r,
                    l_x: msg.l_x,
                    l_y: msg.l_y,
                    l_z: msg.l_z,
                }
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                msg.x = self.x;
                msg.y = self.y;
                msg.z = self.z;
                msg.r = self.r;
                msg.l_x = self.l_x;
                msg.l_y = self.l_y;
                msg.l_z = self.l_z;
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
            pub sub_map: sensor_msgs::msg::PointCloud2,
        }
        impl WrappedTypesupport for Response {
            type CStruct = map_msgs__srv__GetPointMapROI_Response;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__srv__GetPointMapROI_Response()
                }
            }
            fn create_msg() -> *mut map_msgs__srv__GetPointMapROI_Response {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__GetPointMapROI_Response__create() }
                #[cfg(feature = "doc-only")]
                map_msgs__srv__GetPointMapROI_Response__create()
            }
            fn destroy_msg(msg: *mut map_msgs__srv__GetPointMapROI_Response) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__GetPointMapROI_Response__destroy(msg) };
                #[cfg(feature = "doc-only")]
                map_msgs__srv__GetPointMapROI_Response__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Response {
                Response {
                    sub_map: sensor_msgs::msg::PointCloud2::from_native(&msg.sub_map),
                }
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                self.sub_map.copy_to_native(&mut msg.sub_map);
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
    pub mod ProjectedMapsInfo {
        use super::super::super::*;
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__map_msgs__srv__ProjectedMapsInfo()
                }
            }
        }
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Request {
            pub projected_maps_info: Vec<map_msgs::msg::ProjectedMapInfo>,
        }
        impl WrappedTypesupport for Request {
            type CStruct = map_msgs__srv__ProjectedMapsInfo_Request;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__srv__ProjectedMapsInfo_Request()
                }
            }
            fn create_msg() -> *mut map_msgs__srv__ProjectedMapsInfo_Request {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__ProjectedMapsInfo_Request__create() }
                #[cfg(feature = "doc-only")]
                map_msgs__srv__ProjectedMapsInfo_Request__create()
            }
            fn destroy_msg(msg: *mut map_msgs__srv__ProjectedMapsInfo_Request) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__ProjectedMapsInfo_Request__destroy(msg) };
                #[cfg(feature = "doc-only")]
                map_msgs__srv__ProjectedMapsInfo_Request__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Request {
                Request {
                    projected_maps_info: {
                        let mut temp = Vec::with_capacity(msg.projected_maps_info.size);
                        let slice = unsafe {
                            std::slice::from_raw_parts(
                                msg.projected_maps_info.data,
                                msg.projected_maps_info.size,
                            )
                        };
                        for s in slice {
                            temp.push(map_msgs::msg::ProjectedMapInfo::from_native(s));
                        }
                        temp
                    },
                }
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                unsafe {
                    map_msgs__msg__ProjectedMapInfo__Sequence__fini(
                        &mut msg.projected_maps_info,
                    );
                    map_msgs__msg__ProjectedMapInfo__Sequence__init(
                        &mut msg.projected_maps_info,
                        self.projected_maps_info.len(),
                    );
                    let slice = std::slice::from_raw_parts_mut(
                        msg.projected_maps_info.data,
                        msg.projected_maps_info.size,
                    );
                    for (t, s) in slice.iter_mut().zip(&self.projected_maps_info) {
                        s.copy_to_native(t);
                    }
                }
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
        pub struct Response {}
        impl WrappedTypesupport for Response {
            type CStruct = map_msgs__srv__ProjectedMapsInfo_Response;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__srv__ProjectedMapsInfo_Response()
                }
            }
            fn create_msg() -> *mut map_msgs__srv__ProjectedMapsInfo_Response {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__ProjectedMapsInfo_Response__create() }
                #[cfg(feature = "doc-only")]
                map_msgs__srv__ProjectedMapsInfo_Response__create()
            }
            fn destroy_msg(msg: *mut map_msgs__srv__ProjectedMapsInfo_Response) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__ProjectedMapsInfo_Response__destroy(msg) };
                #[cfg(feature = "doc-only")]
                map_msgs__srv__ProjectedMapsInfo_Response__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Response {
                Response {}
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {}
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
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__map_msgs__srv__SaveMap()
                }
            }
        }
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Request {
            pub filename: std_msgs::msg::String,
        }
        impl WrappedTypesupport for Request {
            type CStruct = map_msgs__srv__SaveMap_Request;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__srv__SaveMap_Request()
                }
            }
            fn create_msg() -> *mut map_msgs__srv__SaveMap_Request {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__SaveMap_Request__create() }
                #[cfg(feature = "doc-only")] map_msgs__srv__SaveMap_Request__create()
            }
            fn destroy_msg(msg: *mut map_msgs__srv__SaveMap_Request) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__SaveMap_Request__destroy(msg) };
                #[cfg(feature = "doc-only")] map_msgs__srv__SaveMap_Request__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Request {
                Request {
                    filename: std_msgs::msg::String::from_native(&msg.filename),
                }
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                self.filename.copy_to_native(&mut msg.filename);
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
        pub struct Response {}
        impl WrappedTypesupport for Response {
            type CStruct = map_msgs__srv__SaveMap_Response;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__srv__SaveMap_Response()
                }
            }
            fn create_msg() -> *mut map_msgs__srv__SaveMap_Response {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__SaveMap_Response__create() }
                #[cfg(feature = "doc-only")] map_msgs__srv__SaveMap_Response__create()
            }
            fn destroy_msg(msg: *mut map_msgs__srv__SaveMap_Response) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__SaveMap_Response__destroy(msg) };
                #[cfg(feature = "doc-only")]
                map_msgs__srv__SaveMap_Response__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Response {
                Response {}
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {}
        }
        impl Default for Response {
            fn default() -> Self {
                let msg_native = WrappedNativeMsg::<Response>::new();
                Response::from_native(&msg_native)
            }
        }
    }
    #[allow(non_snake_case)]
    pub mod SetMapProjections {
        use super::super::super::*;
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        pub struct Service();
        impl WrappedServiceTypeSupport for Service {
            type Request = Request;
            type Response = Response;
            fn get_ts() -> &'static rosidl_service_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_service_type_support_handle__map_msgs__srv__SetMapProjections()
                }
            }
        }
        #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
        #[serde(default)]
        pub struct Request {}
        impl WrappedTypesupport for Request {
            type CStruct = map_msgs__srv__SetMapProjections_Request;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__srv__SetMapProjections_Request()
                }
            }
            fn create_msg() -> *mut map_msgs__srv__SetMapProjections_Request {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__SetMapProjections_Request__create() }
                #[cfg(feature = "doc-only")]
                map_msgs__srv__SetMapProjections_Request__create()
            }
            fn destroy_msg(msg: *mut map_msgs__srv__SetMapProjections_Request) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__SetMapProjections_Request__destroy(msg) };
                #[cfg(feature = "doc-only")]
                map_msgs__srv__SetMapProjections_Request__destroy(msg)
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
            pub projected_maps_info: Vec<map_msgs::msg::ProjectedMapInfo>,
        }
        impl WrappedTypesupport for Response {
            type CStruct = map_msgs__srv__SetMapProjections_Response;
            fn get_ts() -> &'static rosidl_message_type_support_t {
                unsafe {
                    &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__srv__SetMapProjections_Response()
                }
            }
            fn create_msg() -> *mut map_msgs__srv__SetMapProjections_Response {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__SetMapProjections_Response__create() }
                #[cfg(feature = "doc-only")]
                map_msgs__srv__SetMapProjections_Response__create()
            }
            fn destroy_msg(msg: *mut map_msgs__srv__SetMapProjections_Response) -> () {
                #[cfg(not(feature = "doc-only"))]
                unsafe { map_msgs__srv__SetMapProjections_Response__destroy(msg) };
                #[cfg(feature = "doc-only")]
                map_msgs__srv__SetMapProjections_Response__destroy(msg)
            }
            fn from_native(#[allow(unused)] msg: &Self::CStruct) -> Response {
                Response {
                    projected_maps_info: {
                        let mut temp = Vec::with_capacity(msg.projected_maps_info.size);
                        let slice = unsafe {
                            std::slice::from_raw_parts(
                                msg.projected_maps_info.data,
                                msg.projected_maps_info.size,
                            )
                        };
                        for s in slice {
                            temp.push(map_msgs::msg::ProjectedMapInfo::from_native(s));
                        }
                        temp
                    },
                }
            }
            fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
                unsafe {
                    map_msgs__msg__ProjectedMapInfo__Sequence__fini(
                        &mut msg.projected_maps_info,
                    );
                    map_msgs__msg__ProjectedMapInfo__Sequence__init(
                        &mut msg.projected_maps_info,
                        self.projected_maps_info.len(),
                    );
                    let slice = std::slice::from_raw_parts_mut(
                        msg.projected_maps_info.data,
                        msg.projected_maps_info.size,
                    );
                    for (t, s) in slice.iter_mut().zip(&self.projected_maps_info) {
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
pub mod msg {
    use super::super::*;
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct OccupancyGridUpdate {
        pub header: std_msgs::msg::Header,
        pub x: i32,
        pub y: i32,
        pub width: u32,
        pub height: u32,
        pub data: Vec<i8>,
    }
    impl WrappedTypesupport for OccupancyGridUpdate {
        type CStruct = map_msgs__msg__OccupancyGridUpdate;
        fn get_ts() -> &'static rosidl_message_type_support_t {
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__msg__OccupancyGridUpdate()
            }
        }
        fn create_msg() -> *mut map_msgs__msg__OccupancyGridUpdate {
            #[cfg(not(feature = "doc-only"))]
            unsafe { map_msgs__msg__OccupancyGridUpdate__create() }
            #[cfg(feature = "doc-only")] map_msgs__msg__OccupancyGridUpdate__create()
        }
        fn destroy_msg(msg: *mut map_msgs__msg__OccupancyGridUpdate) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { map_msgs__msg__OccupancyGridUpdate__destroy(msg) };
            #[cfg(feature = "doc-only")] map_msgs__msg__OccupancyGridUpdate__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> OccupancyGridUpdate {
            OccupancyGridUpdate {
                header: std_msgs::msg::Header::from_native(&msg.header),
                x: msg.x,
                y: msg.y,
                width: msg.width,
                height: msg.height,
                data: msg.data.to_vec(),
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            self.header.copy_to_native(&mut msg.header);
            msg.x = self.x;
            msg.y = self.y;
            msg.width = self.width;
            msg.height = self.height;
            msg.data.update(&self.data);
        }
    }
    impl Default for OccupancyGridUpdate {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<OccupancyGridUpdate>::new();
            OccupancyGridUpdate::from_native(&msg_native)
        }
    }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct PointCloud2Update {
        pub header: std_msgs::msg::Header,
        #[serde(rename = "type")]
        pub type_: u32,
        pub points: sensor_msgs::msg::PointCloud2,
    }
    impl WrappedTypesupport for PointCloud2Update {
        type CStruct = map_msgs__msg__PointCloud2Update;
        fn get_ts() -> &'static rosidl_message_type_support_t {
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__msg__PointCloud2Update()
            }
        }
        fn create_msg() -> *mut map_msgs__msg__PointCloud2Update {
            #[cfg(not(feature = "doc-only"))]
            unsafe { map_msgs__msg__PointCloud2Update__create() }
            #[cfg(feature = "doc-only")] map_msgs__msg__PointCloud2Update__create()
        }
        fn destroy_msg(msg: *mut map_msgs__msg__PointCloud2Update) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { map_msgs__msg__PointCloud2Update__destroy(msg) };
            #[cfg(feature = "doc-only")] map_msgs__msg__PointCloud2Update__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> PointCloud2Update {
            PointCloud2Update {
                header: std_msgs::msg::Header::from_native(&msg.header),
                type_: msg.type_,
                points: sensor_msgs::msg::PointCloud2::from_native(&msg.points),
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            self.header.copy_to_native(&mut msg.header);
            msg.type_ = self.type_;
            self.points.copy_to_native(&mut msg.points);
        }
    }
    impl Default for PointCloud2Update {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<PointCloud2Update>::new();
            PointCloud2Update::from_native(&msg_native)
        }
    }
    #[allow(non_upper_case_globals)]
    impl PointCloud2Update {
        pub const ADD: _bindgen_ty_57 = map_msgs__msg__PointCloud2Update__ADD;
        pub const DELETE: _bindgen_ty_58 = map_msgs__msg__PointCloud2Update__DELETE;
    }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct ProjectedMap {
        pub map: nav_msgs::msg::OccupancyGrid,
        pub min_z: f64,
        pub max_z: f64,
    }
    impl WrappedTypesupport for ProjectedMap {
        type CStruct = map_msgs__msg__ProjectedMap;
        fn get_ts() -> &'static rosidl_message_type_support_t {
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__msg__ProjectedMap()
            }
        }
        fn create_msg() -> *mut map_msgs__msg__ProjectedMap {
            #[cfg(not(feature = "doc-only"))]
            unsafe { map_msgs__msg__ProjectedMap__create() }
            #[cfg(feature = "doc-only")] map_msgs__msg__ProjectedMap__create()
        }
        fn destroy_msg(msg: *mut map_msgs__msg__ProjectedMap) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { map_msgs__msg__ProjectedMap__destroy(msg) };
            #[cfg(feature = "doc-only")] map_msgs__msg__ProjectedMap__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> ProjectedMap {
            ProjectedMap {
                map: nav_msgs::msg::OccupancyGrid::from_native(&msg.map),
                min_z: msg.min_z,
                max_z: msg.max_z,
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            self.map.copy_to_native(&mut msg.map);
            msg.min_z = self.min_z;
            msg.max_z = self.max_z;
        }
    }
    impl Default for ProjectedMap {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<ProjectedMap>::new();
            ProjectedMap::from_native(&msg_native)
        }
    }
    #[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
    #[serde(default)]
    pub struct ProjectedMapInfo {
        pub frame_id: std::string::String,
        pub x: f64,
        pub y: f64,
        pub width: f64,
        pub height: f64,
        pub min_z: f64,
        pub max_z: f64,
    }
    impl WrappedTypesupport for ProjectedMapInfo {
        type CStruct = map_msgs__msg__ProjectedMapInfo;
        fn get_ts() -> &'static rosidl_message_type_support_t {
            unsafe {
                &*rosidl_typesupport_c__get_message_type_support_handle__map_msgs__msg__ProjectedMapInfo()
            }
        }
        fn create_msg() -> *mut map_msgs__msg__ProjectedMapInfo {
            #[cfg(not(feature = "doc-only"))]
            unsafe { map_msgs__msg__ProjectedMapInfo__create() }
            #[cfg(feature = "doc-only")] map_msgs__msg__ProjectedMapInfo__create()
        }
        fn destroy_msg(msg: *mut map_msgs__msg__ProjectedMapInfo) -> () {
            #[cfg(not(feature = "doc-only"))]
            unsafe { map_msgs__msg__ProjectedMapInfo__destroy(msg) };
            #[cfg(feature = "doc-only")] map_msgs__msg__ProjectedMapInfo__destroy(msg)
        }
        fn from_native(#[allow(unused)] msg: &Self::CStruct) -> ProjectedMapInfo {
            ProjectedMapInfo {
                frame_id: msg.frame_id.to_str().to_owned(),
                x: msg.x,
                y: msg.y,
                width: msg.width,
                height: msg.height,
                min_z: msg.min_z,
                max_z: msg.max_z,
            }
        }
        fn copy_to_native(&self, #[allow(unused)] msg: &mut Self::CStruct) {
            msg.frame_id.assign(&self.frame_id);
            msg.x = self.x;
            msg.y = self.y;
            msg.width = self.width;
            msg.height = self.height;
            msg.min_z = self.min_z;
            msg.max_z = self.max_z;
        }
    }
    impl Default for ProjectedMapInfo {
        fn default() -> Self {
            let msg_native = WrappedNativeMsg::<ProjectedMapInfo>::new();
            ProjectedMapInfo::from_native(&msg_native)
        }
    }
}
