impl UntypedServiceSupport {
    pub fn new_from(typename: &str) -> Result<Self> {
        #[allow(non_snake_case)]
        fn new_untyped_service_support_action_msgs_srv_CancelGoal() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<action_msgs::srv::CancelGoal::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_diagnostic_msgs_srv_AddDiagnostics() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<diagnostic_msgs::srv::AddDiagnostics::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_diagnostic_msgs_srv_SelfTest() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<diagnostic_msgs::srv::SelfTest::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_lifecycle_msgs_srv_ChangeState() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<lifecycle_msgs::srv::ChangeState::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_lifecycle_msgs_srv_GetAvailableStates() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<
                lifecycle_msgs::srv::GetAvailableStates::Service,
            >()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_lifecycle_msgs_srv_GetAvailableTransitions() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<
                lifecycle_msgs::srv::GetAvailableTransitions::Service,
            >()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_lifecycle_msgs_srv_GetState() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<lifecycle_msgs::srv::GetState::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_nav_msgs_srv_GetMap() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<nav_msgs::srv::GetMap::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_nav_msgs_srv_GetPlan() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<nav_msgs::srv::GetPlan::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_nav_msgs_srv_LoadMap() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<nav_msgs::srv::LoadMap::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_nav_msgs_srv_SetMap() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<nav_msgs::srv::SetMap::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_rcl_interfaces_srv_DescribeParameters() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<
                rcl_interfaces::srv::DescribeParameters::Service,
            >()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_rcl_interfaces_srv_GetParameterTypes() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<
                rcl_interfaces::srv::GetParameterTypes::Service,
            >()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_rcl_interfaces_srv_GetParameters() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<rcl_interfaces::srv::GetParameters::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_rcl_interfaces_srv_ListParameters() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<rcl_interfaces::srv::ListParameters::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_rcl_interfaces_srv_SetParameters() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<rcl_interfaces::srv::SetParameters::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_rcl_interfaces_srv_SetParametersAtomically() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<
                rcl_interfaces::srv::SetParametersAtomically::Service,
            >()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_sensor_msgs_srv_SetCameraInfo() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<sensor_msgs::srv::SetCameraInfo::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_test_msgs_srv_Arrays() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<test_msgs::srv::Arrays::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_test_msgs_srv_BasicTypes() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<test_msgs::srv::BasicTypes::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_test_msgs_srv_Empty() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<test_msgs::srv::Empty::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_tf2_msgs_srv_FrameGraph() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<tf2_msgs::srv::FrameGraph::Service>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_visualization_msgs_srv_GetInteractiveMarkers() -> UntypedServiceSupport {
            UntypedServiceSupport::new::<
                visualization_msgs::srv::GetInteractiveMarkers::Service,
            >()
        }
        static MAP: phf::Map<&'static str, fn() -> UntypedServiceSupport> = phf::phf_map! {
            "action_msgs/srv/CancelGoal" =>
            new_untyped_service_support_action_msgs_srv_CancelGoal,
            "diagnostic_msgs/srv/AddDiagnostics" =>
            new_untyped_service_support_diagnostic_msgs_srv_AddDiagnostics,
            "diagnostic_msgs/srv/SelfTest" =>
            new_untyped_service_support_diagnostic_msgs_srv_SelfTest,
            "lifecycle_msgs/srv/ChangeState" =>
            new_untyped_service_support_lifecycle_msgs_srv_ChangeState,
            "lifecycle_msgs/srv/GetAvailableStates" =>
            new_untyped_service_support_lifecycle_msgs_srv_GetAvailableStates,
            "lifecycle_msgs/srv/GetAvailableTransitions" =>
            new_untyped_service_support_lifecycle_msgs_srv_GetAvailableTransitions,
            "lifecycle_msgs/srv/GetState" =>
            new_untyped_service_support_lifecycle_msgs_srv_GetState,
            "nav_msgs/srv/GetMap" => new_untyped_service_support_nav_msgs_srv_GetMap,
            "nav_msgs/srv/GetPlan" => new_untyped_service_support_nav_msgs_srv_GetPlan,
            "nav_msgs/srv/LoadMap" => new_untyped_service_support_nav_msgs_srv_LoadMap,
            "nav_msgs/srv/SetMap" => new_untyped_service_support_nav_msgs_srv_SetMap,
            "rcl_interfaces/srv/DescribeParameters" =>
            new_untyped_service_support_rcl_interfaces_srv_DescribeParameters,
            "rcl_interfaces/srv/GetParameterTypes" =>
            new_untyped_service_support_rcl_interfaces_srv_GetParameterTypes,
            "rcl_interfaces/srv/GetParameters" =>
            new_untyped_service_support_rcl_interfaces_srv_GetParameters,
            "rcl_interfaces/srv/ListParameters" =>
            new_untyped_service_support_rcl_interfaces_srv_ListParameters,
            "rcl_interfaces/srv/SetParameters" =>
            new_untyped_service_support_rcl_interfaces_srv_SetParameters,
            "rcl_interfaces/srv/SetParametersAtomically" =>
            new_untyped_service_support_rcl_interfaces_srv_SetParametersAtomically,
            "sensor_msgs/srv/SetCameraInfo" =>
            new_untyped_service_support_sensor_msgs_srv_SetCameraInfo,
            "test_msgs/srv/Arrays" => new_untyped_service_support_test_msgs_srv_Arrays,
            "test_msgs/srv/BasicTypes" =>
            new_untyped_service_support_test_msgs_srv_BasicTypes, "test_msgs/srv/Empty"
            => new_untyped_service_support_test_msgs_srv_Empty, "tf2_msgs/srv/FrameGraph"
            => new_untyped_service_support_tf2_msgs_srv_FrameGraph,
            "visualization_msgs/srv/GetInteractiveMarkers" =>
            new_untyped_service_support_visualization_msgs_srv_GetInteractiveMarkers
        };
        let func = MAP
            .get(typename)
            .ok_or_else(|| Error::InvalidMessageType {
                msgtype: typename.into(),
            })?;
        Ok(func())
    }
}
