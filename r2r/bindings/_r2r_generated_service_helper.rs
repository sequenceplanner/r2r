
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

        if typename == "example_interfaces/srv/AddTwoInts" {
            return Ok(UntypedServiceSupport::new::<example_interfaces::srv::AddTwoInts::Service>());
        }

        if typename == "example_interfaces/srv/SetBool" {
            return Ok(UntypedServiceSupport::new::<example_interfaces::srv::SetBool::Service>());
        }

        if typename == "example_interfaces/srv/Trigger" {
            return Ok(UntypedServiceSupport::new::<example_interfaces::srv::Trigger::Service>());
        }

        if typename == "geographic_msgs/srv/GetGeoPath" {
            return Ok(UntypedServiceSupport::new::<geographic_msgs::srv::GetGeoPath::Service>());
        }

        if typename == "geographic_msgs/srv/GetGeographicMap" {
            return Ok(UntypedServiceSupport::new::<geographic_msgs::srv::GetGeographicMap::Service>());
        }

        if typename == "geographic_msgs/srv/GetRoutePlan" {
            return Ok(UntypedServiceSupport::new::<geographic_msgs::srv::GetRoutePlan::Service>());
        }

        if typename == "geographic_msgs/srv/UpdateGeographicMap" {
            return Ok(UntypedServiceSupport::new::<geographic_msgs::srv::UpdateGeographicMap::Service>());
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

        if typename == "logging_demo/srv/ConfigLogger" {
            return Ok(UntypedServiceSupport::new::<logging_demo::srv::ConfigLogger::Service>());
        }

        if typename == "map_msgs/srv/GetMapROI" {
            return Ok(UntypedServiceSupport::new::<map_msgs::srv::GetMapROI::Service>());
        }

        if typename == "map_msgs/srv/GetPointMap" {
            return Ok(UntypedServiceSupport::new::<map_msgs::srv::GetPointMap::Service>());
        }

        if typename == "map_msgs/srv/GetPointMapROI" {
            return Ok(UntypedServiceSupport::new::<map_msgs::srv::GetPointMapROI::Service>());
        }

        if typename == "map_msgs/srv/ProjectedMapsInfo" {
            return Ok(UntypedServiceSupport::new::<map_msgs::srv::ProjectedMapsInfo::Service>());
        }

        if typename == "map_msgs/srv/SaveMap" {
            return Ok(UntypedServiceSupport::new::<map_msgs::srv::SaveMap::Service>());
        }

        if typename == "map_msgs/srv/SetMapProjections" {
            return Ok(UntypedServiceSupport::new::<map_msgs::srv::SetMapProjections::Service>());
        }

        if typename == "nav2_msgs/srv/ClearCostmapAroundRobot" {
            return Ok(UntypedServiceSupport::new::<nav2_msgs::srv::ClearCostmapAroundRobot::Service>());
        }

        if typename == "nav2_msgs/srv/ClearCostmapExceptRegion" {
            return Ok(UntypedServiceSupport::new::<nav2_msgs::srv::ClearCostmapExceptRegion::Service>());
        }

        if typename == "nav2_msgs/srv/ClearEntireCostmap" {
            return Ok(UntypedServiceSupport::new::<nav2_msgs::srv::ClearEntireCostmap::Service>());
        }

        if typename == "nav2_msgs/srv/GetCostmap" {
            return Ok(UntypedServiceSupport::new::<nav2_msgs::srv::GetCostmap::Service>());
        }

        if typename == "nav2_msgs/srv/LoadMap" {
            return Ok(UntypedServiceSupport::new::<nav2_msgs::srv::LoadMap::Service>());
        }

        if typename == "nav2_msgs/srv/ManageLifecycleNodes" {
            return Ok(UntypedServiceSupport::new::<nav2_msgs::srv::ManageLifecycleNodes::Service>());
        }

        if typename == "nav2_msgs/srv/SaveMap" {
            return Ok(UntypedServiceSupport::new::<nav2_msgs::srv::SaveMap::Service>());
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

        if typename == "octomap_msgs/srv/BoundingBoxQuery" {
            return Ok(UntypedServiceSupport::new::<octomap_msgs::srv::BoundingBoxQuery::Service>());
        }

        if typename == "octomap_msgs/srv/GetOctomap" {
            return Ok(UntypedServiceSupport::new::<octomap_msgs::srv::GetOctomap::Service>());
        }

        if typename == "pcl_msgs/srv/UpdateFilename" {
            return Ok(UntypedServiceSupport::new::<pcl_msgs::srv::UpdateFilename::Service>());
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

        if typename == "rosapi_msgs/srv/DeleteParam" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::DeleteParam::Service>());
        }

        if typename == "rosapi_msgs/srv/GetActionServers" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::GetActionServers::Service>());
        }

        if typename == "rosapi_msgs/srv/GetParam" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::GetParam::Service>());
        }

        if typename == "rosapi_msgs/srv/GetParamNames" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::GetParamNames::Service>());
        }

        if typename == "rosapi_msgs/srv/GetROSVersion" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::GetROSVersion::Service>());
        }

        if typename == "rosapi_msgs/srv/GetTime" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::GetTime::Service>());
        }

        if typename == "rosapi_msgs/srv/HasParam" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::HasParam::Service>());
        }

        if typename == "rosapi_msgs/srv/MessageDetails" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::MessageDetails::Service>());
        }

        if typename == "rosapi_msgs/srv/NodeDetails" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::NodeDetails::Service>());
        }

        if typename == "rosapi_msgs/srv/Nodes" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::Nodes::Service>());
        }

        if typename == "rosapi_msgs/srv/Publishers" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::Publishers::Service>());
        }

        if typename == "rosapi_msgs/srv/ServiceNode" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::ServiceNode::Service>());
        }

        if typename == "rosapi_msgs/srv/ServiceProviders" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::ServiceProviders::Service>());
        }

        if typename == "rosapi_msgs/srv/ServiceRequestDetails" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::ServiceRequestDetails::Service>());
        }

        if typename == "rosapi_msgs/srv/ServiceResponseDetails" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::ServiceResponseDetails::Service>());
        }

        if typename == "rosapi_msgs/srv/ServiceType" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::ServiceType::Service>());
        }

        if typename == "rosapi_msgs/srv/Services" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::Services::Service>());
        }

        if typename == "rosapi_msgs/srv/ServicesForType" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::ServicesForType::Service>());
        }

        if typename == "rosapi_msgs/srv/SetParam" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::SetParam::Service>());
        }

        if typename == "rosapi_msgs/srv/Subscribers" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::Subscribers::Service>());
        }

        if typename == "rosapi_msgs/srv/TopicType" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::TopicType::Service>());
        }

        if typename == "rosapi_msgs/srv/Topics" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::Topics::Service>());
        }

        if typename == "rosapi_msgs/srv/TopicsAndRawTypes" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::TopicsAndRawTypes::Service>());
        }

        if typename == "rosapi_msgs/srv/TopicsForType" {
            return Ok(UntypedServiceSupport::new::<rosapi_msgs::srv::TopicsForType::Service>());
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

        if typename == "tf2_msgs/srv/FrameGraph" {
            return Ok(UntypedServiceSupport::new::<tf2_msgs::srv::FrameGraph::Service>());
        }

        if typename == "topic_tools_interfaces/srv/MuxAdd" {
            return Ok(UntypedServiceSupport::new::<topic_tools_interfaces::srv::MuxAdd::Service>());
        }

        if typename == "topic_tools_interfaces/srv/MuxDelete" {
            return Ok(UntypedServiceSupport::new::<topic_tools_interfaces::srv::MuxDelete::Service>());
        }

        if typename == "topic_tools_interfaces/srv/MuxList" {
            return Ok(UntypedServiceSupport::new::<topic_tools_interfaces::srv::MuxList::Service>());
        }

        if typename == "topic_tools_interfaces/srv/MuxSelect" {
            return Ok(UntypedServiceSupport::new::<topic_tools_interfaces::srv::MuxSelect::Service>());
        }

        if typename == "turtlesim/srv/Kill" {
            return Ok(UntypedServiceSupport::new::<turtlesim::srv::Kill::Service>());
        }

        if typename == "turtlesim/srv/SetPen" {
            return Ok(UntypedServiceSupport::new::<turtlesim::srv::SetPen::Service>());
        }

        if typename == "turtlesim/srv/Spawn" {
            return Ok(UntypedServiceSupport::new::<turtlesim::srv::Spawn::Service>());
        }

        if typename == "turtlesim/srv/TeleportAbsolute" {
            return Ok(UntypedServiceSupport::new::<turtlesim::srv::TeleportAbsolute::Service>());
        }

        if typename == "turtlesim/srv/TeleportRelative" {
            return Ok(UntypedServiceSupport::new::<turtlesim::srv::TeleportRelative::Service>());
        }

        if typename == "visualization_msgs/srv/GetInteractiveMarkers" {
            return Ok(UntypedServiceSupport::new::<visualization_msgs::srv::GetInteractiveMarkers::Service>());
        }

        return Err(Error::InvalidMessageType{ msgtype: typename.into() })
    }
}
