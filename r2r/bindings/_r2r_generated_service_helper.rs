
impl UntypedServiceSupport {
    pub fn new_from(typename: &str) -> Result<Self> {

        if typename == "action_msgs/srv/CancelGoal" {
            return Ok(UntypedServiceSupport::new::<action_msgs::srv::CancelGoal::Service>());
        }

        if typename == "composition_interfaces/srv/ListNodes" {
            return Ok(UntypedServiceSupport::new::<composition_interfaces::srv::ListNodes::Service>());
        }

        if typename == "composition_interfaces/srv/LoadNode" {
            return Ok(UntypedServiceSupport::new::<composition_interfaces::srv::LoadNode::Service>());
        }

        if typename == "composition_interfaces/srv/UnloadNode" {
            return Ok(UntypedServiceSupport::new::<composition_interfaces::srv::UnloadNode::Service>());
        }

        if typename == "diagnostic_msgs/srv/AddDiagnostics" {
            return Ok(UntypedServiceSupport::new::<diagnostic_msgs::srv::AddDiagnostics::Service>());
        }

        if typename == "diagnostic_msgs/srv/SelfTest" {
            return Ok(UntypedServiceSupport::new::<diagnostic_msgs::srv::SelfTest::Service>());
        }

        if typename == "lifecycle_msgs/srv/ChangeState" {
            return Ok(UntypedServiceSupport::new::<lifecycle_msgs::srv::ChangeState::Service>());
        }

        if typename == "lifecycle_msgs/srv/GetAvailableStates" {
            return Ok(UntypedServiceSupport::new::<lifecycle_msgs::srv::GetAvailableStates::Service>());
        }

        if typename == "lifecycle_msgs/srv/GetAvailableTransitions" {
            return Ok(UntypedServiceSupport::new::<lifecycle_msgs::srv::GetAvailableTransitions::Service>());
        }

        if typename == "lifecycle_msgs/srv/GetState" {
            return Ok(UntypedServiceSupport::new::<lifecycle_msgs::srv::GetState::Service>());
        }

        if typename == "nav_msgs/srv/GetMap" {
            return Ok(UntypedServiceSupport::new::<nav_msgs::srv::GetMap::Service>());
        }

        if typename == "nav_msgs/srv/GetPlan" {
            return Ok(UntypedServiceSupport::new::<nav_msgs::srv::GetPlan::Service>());
        }

        if typename == "nav_msgs/srv/LoadMap" {
            return Ok(UntypedServiceSupport::new::<nav_msgs::srv::LoadMap::Service>());
        }

        if typename == "nav_msgs/srv/SetMap" {
            return Ok(UntypedServiceSupport::new::<nav_msgs::srv::SetMap::Service>());
        }

        if typename == "rcl_interfaces/srv/DescribeParameters" {
            return Ok(UntypedServiceSupport::new::<rcl_interfaces::srv::DescribeParameters::Service>());
        }

        if typename == "rcl_interfaces/srv/GetParameterTypes" {
            return Ok(UntypedServiceSupport::new::<rcl_interfaces::srv::GetParameterTypes::Service>());
        }

        if typename == "rcl_interfaces/srv/GetParameters" {
            return Ok(UntypedServiceSupport::new::<rcl_interfaces::srv::GetParameters::Service>());
        }

        if typename == "rcl_interfaces/srv/ListParameters" {
            return Ok(UntypedServiceSupport::new::<rcl_interfaces::srv::ListParameters::Service>());
        }

        if typename == "rcl_interfaces/srv/SetParameters" {
            return Ok(UntypedServiceSupport::new::<rcl_interfaces::srv::SetParameters::Service>());
        }

        if typename == "rcl_interfaces/srv/SetParametersAtomically" {
            return Ok(UntypedServiceSupport::new::<rcl_interfaces::srv::SetParametersAtomically::Service>());
        }

        if typename == "rosbag2_interfaces/srv/Burst" {
            return Ok(UntypedServiceSupport::new::<rosbag2_interfaces::srv::Burst::Service>());
        }

        if typename == "rosbag2_interfaces/srv/GetRate" {
            return Ok(UntypedServiceSupport::new::<rosbag2_interfaces::srv::GetRate::Service>());
        }

        if typename == "rosbag2_interfaces/srv/IsPaused" {
            return Ok(UntypedServiceSupport::new::<rosbag2_interfaces::srv::IsPaused::Service>());
        }

        if typename == "rosbag2_interfaces/srv/Pause" {
            return Ok(UntypedServiceSupport::new::<rosbag2_interfaces::srv::Pause::Service>());
        }

        if typename == "rosbag2_interfaces/srv/PlayNext" {
            return Ok(UntypedServiceSupport::new::<rosbag2_interfaces::srv::PlayNext::Service>());
        }

        if typename == "rosbag2_interfaces/srv/Resume" {
            return Ok(UntypedServiceSupport::new::<rosbag2_interfaces::srv::Resume::Service>());
        }

        if typename == "rosbag2_interfaces/srv/Seek" {
            return Ok(UntypedServiceSupport::new::<rosbag2_interfaces::srv::Seek::Service>());
        }

        if typename == "rosbag2_interfaces/srv/SetRate" {
            return Ok(UntypedServiceSupport::new::<rosbag2_interfaces::srv::SetRate::Service>());
        }

        if typename == "rosbag2_interfaces/srv/Snapshot" {
            return Ok(UntypedServiceSupport::new::<rosbag2_interfaces::srv::Snapshot::Service>());
        }

        if typename == "rosbag2_interfaces/srv/TogglePaused" {
            return Ok(UntypedServiceSupport::new::<rosbag2_interfaces::srv::TogglePaused::Service>());
        }

        if typename == "sensor_msgs/srv/SetCameraInfo" {
            return Ok(UntypedServiceSupport::new::<sensor_msgs::srv::SetCameraInfo::Service>());
        }

        if typename == "std_srvs/srv/Empty" {
            return Ok(UntypedServiceSupport::new::<std_srvs::srv::Empty::Service>());
        }

        if typename == "std_srvs/srv/SetBool" {
            return Ok(UntypedServiceSupport::new::<std_srvs::srv::SetBool::Service>());
        }

        if typename == "std_srvs/srv/Trigger" {
            return Ok(UntypedServiceSupport::new::<std_srvs::srv::Trigger::Service>());
        }

        if typename == "test_msgs/srv/Arrays" {
            return Ok(UntypedServiceSupport::new::<test_msgs::srv::Arrays::Service>());
        }

        if typename == "test_msgs/srv/BasicTypes" {
            return Ok(UntypedServiceSupport::new::<test_msgs::srv::BasicTypes::Service>());
        }

        if typename == "test_msgs/srv/Empty" {
            return Ok(UntypedServiceSupport::new::<test_msgs::srv::Empty::Service>());
        }

        if typename == "tf2_msgs/srv/FrameGraph" {
            return Ok(UntypedServiceSupport::new::<tf2_msgs::srv::FrameGraph::Service>());
        }

        if typename == "visualization_msgs/srv/GetInteractiveMarkers" {
            return Ok(UntypedServiceSupport::new::<visualization_msgs::srv::GetInteractiveMarkers::Service>());
        }

        return Err(Error::InvalidMessageType{ msgtype: typename.into() })
    }
}
