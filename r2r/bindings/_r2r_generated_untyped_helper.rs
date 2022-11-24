
impl WrappedNativeMsgUntyped {
    pub fn new_from(typename: &str) -> Result<Self> {

        if typename == "action_msgs/msg/GoalInfo" {
            return Ok(WrappedNativeMsgUntyped::new::<action_msgs::msg::GoalInfo>());
        }

        if typename == "action_msgs/msg/GoalStatus" {
            return Ok(WrappedNativeMsgUntyped::new::<action_msgs::msg::GoalStatus>());
        }

        if typename == "action_msgs/msg/GoalStatusArray" {
            return Ok(WrappedNativeMsgUntyped::new::<action_msgs::msg::GoalStatusArray>());
        }

        if typename == "actionlib_msgs/msg/GoalID" {
            return Ok(WrappedNativeMsgUntyped::new::<actionlib_msgs::msg::GoalID>());
        }

        if typename == "actionlib_msgs/msg/GoalStatus" {
            return Ok(WrappedNativeMsgUntyped::new::<actionlib_msgs::msg::GoalStatus>());
        }

        if typename == "actionlib_msgs/msg/GoalStatusArray" {
            return Ok(WrappedNativeMsgUntyped::new::<actionlib_msgs::msg::GoalStatusArray>());
        }

        if typename == "bond/msg/Constants" {
            return Ok(WrappedNativeMsgUntyped::new::<bond::msg::Constants>());
        }

        if typename == "bond/msg/Status" {
            return Ok(WrappedNativeMsgUntyped::new::<bond::msg::Status>());
        }

        if typename == "builtin_interfaces/msg/Duration" {
            return Ok(WrappedNativeMsgUntyped::new::<builtin_interfaces::msg::Duration>());
        }

        if typename == "builtin_interfaces/msg/Time" {
            return Ok(WrappedNativeMsgUntyped::new::<builtin_interfaces::msg::Time>());
        }

        if typename == "can_msgs/msg/Frame" {
            return Ok(WrappedNativeMsgUntyped::new::<can_msgs::msg::Frame>());
        }

        if typename == "diagnostic_msgs/msg/DiagnosticArray" {
            return Ok(WrappedNativeMsgUntyped::new::<diagnostic_msgs::msg::DiagnosticArray>());
        }

        if typename == "diagnostic_msgs/msg/DiagnosticStatus" {
            return Ok(WrappedNativeMsgUntyped::new::<diagnostic_msgs::msg::DiagnosticStatus>());
        }

        if typename == "diagnostic_msgs/msg/KeyValue" {
            return Ok(WrappedNativeMsgUntyped::new::<diagnostic_msgs::msg::KeyValue>());
        }

        if typename == "example_interfaces/msg/Bool" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Bool>());
        }

        if typename == "example_interfaces/msg/Byte" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Byte>());
        }

        if typename == "example_interfaces/msg/ByteMultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::ByteMultiArray>());
        }

        if typename == "example_interfaces/msg/Char" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Char>());
        }

        if typename == "example_interfaces/msg/Empty" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Empty>());
        }

        if typename == "example_interfaces/msg/Float32" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Float32>());
        }

        if typename == "example_interfaces/msg/Float32MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Float32MultiArray>());
        }

        if typename == "example_interfaces/msg/Float64" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Float64>());
        }

        if typename == "example_interfaces/msg/Float64MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Float64MultiArray>());
        }

        if typename == "example_interfaces/msg/Int16" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Int16>());
        }

        if typename == "example_interfaces/msg/Int16MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Int16MultiArray>());
        }

        if typename == "example_interfaces/msg/Int32" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Int32>());
        }

        if typename == "example_interfaces/msg/Int32MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Int32MultiArray>());
        }

        if typename == "example_interfaces/msg/Int64" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Int64>());
        }

        if typename == "example_interfaces/msg/Int64MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Int64MultiArray>());
        }

        if typename == "example_interfaces/msg/Int8" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Int8>());
        }

        if typename == "example_interfaces/msg/Int8MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::Int8MultiArray>());
        }

        if typename == "example_interfaces/msg/MultiArrayDimension" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::MultiArrayDimension>());
        }

        if typename == "example_interfaces/msg/MultiArrayLayout" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::MultiArrayLayout>());
        }

        if typename == "example_interfaces/msg/String" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::String>());
        }

        if typename == "example_interfaces/msg/UInt16" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::UInt16>());
        }

        if typename == "example_interfaces/msg/UInt16MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::UInt16MultiArray>());
        }

        if typename == "example_interfaces/msg/UInt32" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::UInt32>());
        }

        if typename == "example_interfaces/msg/UInt32MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::UInt32MultiArray>());
        }

        if typename == "example_interfaces/msg/UInt64" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::UInt64>());
        }

        if typename == "example_interfaces/msg/UInt64MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::UInt64MultiArray>());
        }

        if typename == "example_interfaces/msg/UInt8" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::UInt8>());
        }

        if typename == "example_interfaces/msg/UInt8MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::UInt8MultiArray>());
        }

        if typename == "example_interfaces/msg/WString" {
            return Ok(WrappedNativeMsgUntyped::new::<example_interfaces::msg::WString>());
        }

        if typename == "geographic_msgs/msg/BoundingBox" {
            return Ok(WrappedNativeMsgUntyped::new::<geographic_msgs::msg::BoundingBox>());
        }

        if typename == "geographic_msgs/msg/GeoPath" {
            return Ok(WrappedNativeMsgUntyped::new::<geographic_msgs::msg::GeoPath>());
        }

        if typename == "geographic_msgs/msg/GeoPoint" {
            return Ok(WrappedNativeMsgUntyped::new::<geographic_msgs::msg::GeoPoint>());
        }

        if typename == "geographic_msgs/msg/GeoPointStamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geographic_msgs::msg::GeoPointStamped>());
        }

        if typename == "geographic_msgs/msg/GeoPose" {
            return Ok(WrappedNativeMsgUntyped::new::<geographic_msgs::msg::GeoPose>());
        }

        if typename == "geographic_msgs/msg/GeoPoseStamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geographic_msgs::msg::GeoPoseStamped>());
        }

        if typename == "geographic_msgs/msg/GeographicMap" {
            return Ok(WrappedNativeMsgUntyped::new::<geographic_msgs::msg::GeographicMap>());
        }

        if typename == "geographic_msgs/msg/GeographicMapChanges" {
            return Ok(WrappedNativeMsgUntyped::new::<geographic_msgs::msg::GeographicMapChanges>());
        }

        if typename == "geographic_msgs/msg/KeyValue" {
            return Ok(WrappedNativeMsgUntyped::new::<geographic_msgs::msg::KeyValue>());
        }

        if typename == "geographic_msgs/msg/MapFeature" {
            return Ok(WrappedNativeMsgUntyped::new::<geographic_msgs::msg::MapFeature>());
        }

        if typename == "geographic_msgs/msg/RouteNetwork" {
            return Ok(WrappedNativeMsgUntyped::new::<geographic_msgs::msg::RouteNetwork>());
        }

        if typename == "geographic_msgs/msg/RoutePath" {
            return Ok(WrappedNativeMsgUntyped::new::<geographic_msgs::msg::RoutePath>());
        }

        if typename == "geographic_msgs/msg/RouteSegment" {
            return Ok(WrappedNativeMsgUntyped::new::<geographic_msgs::msg::RouteSegment>());
        }

        if typename == "geographic_msgs/msg/WayPoint" {
            return Ok(WrappedNativeMsgUntyped::new::<geographic_msgs::msg::WayPoint>());
        }

        if typename == "geometry_msgs/msg/Accel" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::Accel>());
        }

        if typename == "geometry_msgs/msg/AccelStamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::AccelStamped>());
        }

        if typename == "geometry_msgs/msg/AccelWithCovariance" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::AccelWithCovariance>());
        }

        if typename == "geometry_msgs/msg/AccelWithCovarianceStamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::AccelWithCovarianceStamped>());
        }

        if typename == "geometry_msgs/msg/Inertia" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::Inertia>());
        }

        if typename == "geometry_msgs/msg/InertiaStamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::InertiaStamped>());
        }

        if typename == "geometry_msgs/msg/Point" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::Point>());
        }

        if typename == "geometry_msgs/msg/Point32" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::Point32>());
        }

        if typename == "geometry_msgs/msg/PointStamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::PointStamped>());
        }

        if typename == "geometry_msgs/msg/Polygon" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::Polygon>());
        }

        if typename == "geometry_msgs/msg/PolygonStamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::PolygonStamped>());
        }

        if typename == "geometry_msgs/msg/Pose" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::Pose>());
        }

        if typename == "geometry_msgs/msg/Pose2D" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::Pose2D>());
        }

        if typename == "geometry_msgs/msg/PoseArray" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::PoseArray>());
        }

        if typename == "geometry_msgs/msg/PoseStamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::PoseStamped>());
        }

        if typename == "geometry_msgs/msg/PoseWithCovariance" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::PoseWithCovariance>());
        }

        if typename == "geometry_msgs/msg/PoseWithCovarianceStamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::PoseWithCovarianceStamped>());
        }

        if typename == "geometry_msgs/msg/Quaternion" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::Quaternion>());
        }

        if typename == "geometry_msgs/msg/QuaternionStamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::QuaternionStamped>());
        }

        if typename == "geometry_msgs/msg/Transform" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::Transform>());
        }

        if typename == "geometry_msgs/msg/TransformStamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::TransformStamped>());
        }

        if typename == "geometry_msgs/msg/Twist" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::Twist>());
        }

        if typename == "geometry_msgs/msg/TwistStamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::TwistStamped>());
        }

        if typename == "geometry_msgs/msg/TwistWithCovariance" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::TwistWithCovariance>());
        }

        if typename == "geometry_msgs/msg/TwistWithCovarianceStamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::TwistWithCovarianceStamped>());
        }

        if typename == "geometry_msgs/msg/Vector3" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::Vector3>());
        }

        if typename == "geometry_msgs/msg/Vector3Stamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::Vector3Stamped>());
        }

        if typename == "geometry_msgs/msg/Wrench" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::Wrench>());
        }

        if typename == "geometry_msgs/msg/WrenchStamped" {
            return Ok(WrappedNativeMsgUntyped::new::<geometry_msgs::msg::WrenchStamped>());
        }

        if typename == "lifecycle_msgs/msg/State" {
            return Ok(WrappedNativeMsgUntyped::new::<lifecycle_msgs::msg::State>());
        }

        if typename == "lifecycle_msgs/msg/Transition" {
            return Ok(WrappedNativeMsgUntyped::new::<lifecycle_msgs::msg::Transition>());
        }

        if typename == "lifecycle_msgs/msg/TransitionDescription" {
            return Ok(WrappedNativeMsgUntyped::new::<lifecycle_msgs::msg::TransitionDescription>());
        }

        if typename == "lifecycle_msgs/msg/TransitionEvent" {
            return Ok(WrappedNativeMsgUntyped::new::<lifecycle_msgs::msg::TransitionEvent>());
        }

        if typename == "map_msgs/msg/OccupancyGridUpdate" {
            return Ok(WrappedNativeMsgUntyped::new::<map_msgs::msg::OccupancyGridUpdate>());
        }

        if typename == "map_msgs/msg/PointCloud2Update" {
            return Ok(WrappedNativeMsgUntyped::new::<map_msgs::msg::PointCloud2Update>());
        }

        if typename == "map_msgs/msg/ProjectedMap" {
            return Ok(WrappedNativeMsgUntyped::new::<map_msgs::msg::ProjectedMap>());
        }

        if typename == "map_msgs/msg/ProjectedMapInfo" {
            return Ok(WrappedNativeMsgUntyped::new::<map_msgs::msg::ProjectedMapInfo>());
        }

        if typename == "nav2_msgs/msg/BehaviorTreeLog" {
            return Ok(WrappedNativeMsgUntyped::new::<nav2_msgs::msg::BehaviorTreeLog>());
        }

        if typename == "nav2_msgs/msg/BehaviorTreeStatusChange" {
            return Ok(WrappedNativeMsgUntyped::new::<nav2_msgs::msg::BehaviorTreeStatusChange>());
        }

        if typename == "nav2_msgs/msg/Costmap" {
            return Ok(WrappedNativeMsgUntyped::new::<nav2_msgs::msg::Costmap>());
        }

        if typename == "nav2_msgs/msg/CostmapFilterInfo" {
            return Ok(WrappedNativeMsgUntyped::new::<nav2_msgs::msg::CostmapFilterInfo>());
        }

        if typename == "nav2_msgs/msg/CostmapMetaData" {
            return Ok(WrappedNativeMsgUntyped::new::<nav2_msgs::msg::CostmapMetaData>());
        }

        if typename == "nav2_msgs/msg/Particle" {
            return Ok(WrappedNativeMsgUntyped::new::<nav2_msgs::msg::Particle>());
        }

        if typename == "nav2_msgs/msg/ParticleCloud" {
            return Ok(WrappedNativeMsgUntyped::new::<nav2_msgs::msg::ParticleCloud>());
        }

        if typename == "nav2_msgs/msg/SpeedLimit" {
            return Ok(WrappedNativeMsgUntyped::new::<nav2_msgs::msg::SpeedLimit>());
        }

        if typename == "nav2_msgs/msg/VoxelGrid" {
            return Ok(WrappedNativeMsgUntyped::new::<nav2_msgs::msg::VoxelGrid>());
        }

        if typename == "nav_msgs/msg/GridCells" {
            return Ok(WrappedNativeMsgUntyped::new::<nav_msgs::msg::GridCells>());
        }

        if typename == "nav_msgs/msg/MapMetaData" {
            return Ok(WrappedNativeMsgUntyped::new::<nav_msgs::msg::MapMetaData>());
        }

        if typename == "nav_msgs/msg/OccupancyGrid" {
            return Ok(WrappedNativeMsgUntyped::new::<nav_msgs::msg::OccupancyGrid>());
        }

        if typename == "nav_msgs/msg/Odometry" {
            return Ok(WrappedNativeMsgUntyped::new::<nav_msgs::msg::Odometry>());
        }

        if typename == "nav_msgs/msg/Path" {
            return Ok(WrappedNativeMsgUntyped::new::<nav_msgs::msg::Path>());
        }

        if typename == "octomap_msgs/msg/Octomap" {
            return Ok(WrappedNativeMsgUntyped::new::<octomap_msgs::msg::Octomap>());
        }

        if typename == "octomap_msgs/msg/OctomapWithPose" {
            return Ok(WrappedNativeMsgUntyped::new::<octomap_msgs::msg::OctomapWithPose>());
        }

        if typename == "pacmod3_msgs/msg/AccelAuxRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::AccelAuxRpt>());
        }

        if typename == "pacmod3_msgs/msg/AllSystemStatuses" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::AllSystemStatuses>());
        }

        if typename == "pacmod3_msgs/msg/AngVelRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::AngVelRpt>());
        }

        if typename == "pacmod3_msgs/msg/BrakeAuxRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::BrakeAuxRpt>());
        }

        if typename == "pacmod3_msgs/msg/BrakeDecelAuxRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::BrakeDecelAuxRpt>());
        }

        if typename == "pacmod3_msgs/msg/BrakeDecelCmd" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::BrakeDecelCmd>());
        }

        if typename == "pacmod3_msgs/msg/CabinClimateCmd" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::CabinClimateCmd>());
        }

        if typename == "pacmod3_msgs/msg/CabinClimateRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::CabinClimateRpt>());
        }

        if typename == "pacmod3_msgs/msg/ComponentRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::ComponentRpt>());
        }

        if typename == "pacmod3_msgs/msg/DateTimeRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::DateTimeRpt>());
        }

        if typename == "pacmod3_msgs/msg/DetectedObjectRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::DetectedObjectRpt>());
        }

        if typename == "pacmod3_msgs/msg/DoorRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::DoorRpt>());
        }

        if typename == "pacmod3_msgs/msg/DriveTrainFeatureRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::DriveTrainFeatureRpt>());
        }

        if typename == "pacmod3_msgs/msg/EStopRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::EStopRpt>());
        }

        if typename == "pacmod3_msgs/msg/EngineRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::EngineRpt>());
        }

        if typename == "pacmod3_msgs/msg/GlobalCmd" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::GlobalCmd>());
        }

        if typename == "pacmod3_msgs/msg/GlobalRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::GlobalRpt>());
        }

        if typename == "pacmod3_msgs/msg/GlobalRpt2" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::GlobalRpt2>());
        }

        if typename == "pacmod3_msgs/msg/HeadlightAuxRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::HeadlightAuxRpt>());
        }

        if typename == "pacmod3_msgs/msg/InteriorLightsRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::InteriorLightsRpt>());
        }

        if typename == "pacmod3_msgs/msg/KeyValuePair" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::KeyValuePair>());
        }

        if typename == "pacmod3_msgs/msg/LatLonHeadingRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::LatLonHeadingRpt>());
        }

        if typename == "pacmod3_msgs/msg/LinearAccelRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::LinearAccelRpt>());
        }

        if typename == "pacmod3_msgs/msg/MotorRpt1" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::MotorRpt1>());
        }

        if typename == "pacmod3_msgs/msg/MotorRpt2" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::MotorRpt2>());
        }

        if typename == "pacmod3_msgs/msg/MotorRpt3" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::MotorRpt3>());
        }

        if typename == "pacmod3_msgs/msg/NotificationCmd" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::NotificationCmd>());
        }

        if typename == "pacmod3_msgs/msg/OccupancyRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::OccupancyRpt>());
        }

        if typename == "pacmod3_msgs/msg/ParkingBrakeAuxRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::ParkingBrakeAuxRpt>());
        }

        if typename == "pacmod3_msgs/msg/RearLightsRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::RearLightsRpt>());
        }

        if typename == "pacmod3_msgs/msg/SafetyBrakeCmd" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SafetyBrakeCmd>());
        }

        if typename == "pacmod3_msgs/msg/SafetyBrakeRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SafetyBrakeRpt>());
        }

        if typename == "pacmod3_msgs/msg/SafetyFuncCmd" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SafetyFuncCmd>());
        }

        if typename == "pacmod3_msgs/msg/SafetyFuncCriticalStopRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SafetyFuncCriticalStopRpt>());
        }

        if typename == "pacmod3_msgs/msg/SafetyFuncRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SafetyFuncRpt>());
        }

        if typename == "pacmod3_msgs/msg/ShiftAuxRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::ShiftAuxRpt>());
        }

        if typename == "pacmod3_msgs/msg/SoftwareVersionRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SoftwareVersionRpt>());
        }

        if typename == "pacmod3_msgs/msg/SteeringAuxRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SteeringAuxRpt>());
        }

        if typename == "pacmod3_msgs/msg/SteeringCmd" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SteeringCmd>());
        }

        if typename == "pacmod3_msgs/msg/SteeringCmdLimitRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SteeringCmdLimitRpt>());
        }

        if typename == "pacmod3_msgs/msg/SupervisoryCtrl" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SupervisoryCtrl>());
        }

        if typename == "pacmod3_msgs/msg/SystemCmdBool" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SystemCmdBool>());
        }

        if typename == "pacmod3_msgs/msg/SystemCmdFloat" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SystemCmdFloat>());
        }

        if typename == "pacmod3_msgs/msg/SystemCmdInt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SystemCmdInt>());
        }

        if typename == "pacmod3_msgs/msg/SystemCmdLimitRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SystemCmdLimitRpt>());
        }

        if typename == "pacmod3_msgs/msg/SystemRptBool" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SystemRptBool>());
        }

        if typename == "pacmod3_msgs/msg/SystemRptFloat" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SystemRptFloat>());
        }

        if typename == "pacmod3_msgs/msg/SystemRptInt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::SystemRptInt>());
        }

        if typename == "pacmod3_msgs/msg/TirePressureRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::TirePressureRpt>());
        }

        if typename == "pacmod3_msgs/msg/TurnAuxRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::TurnAuxRpt>());
        }

        if typename == "pacmod3_msgs/msg/VehicleDynamicsRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::VehicleDynamicsRpt>());
        }

        if typename == "pacmod3_msgs/msg/VehicleSpeedRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::VehicleSpeedRpt>());
        }

        if typename == "pacmod3_msgs/msg/VinRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::VinRpt>());
        }

        if typename == "pacmod3_msgs/msg/WatchdogRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::WatchdogRpt>());
        }

        if typename == "pacmod3_msgs/msg/WatchdogRpt2" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::WatchdogRpt2>());
        }

        if typename == "pacmod3_msgs/msg/WheelSpeedRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::WheelSpeedRpt>());
        }

        if typename == "pacmod3_msgs/msg/WiperAuxRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::WiperAuxRpt>());
        }

        if typename == "pacmod3_msgs/msg/YawRateRpt" {
            return Ok(WrappedNativeMsgUntyped::new::<pacmod3_msgs::msg::YawRateRpt>());
        }

        if typename == "pcl_msgs/msg/ModelCoefficients" {
            return Ok(WrappedNativeMsgUntyped::new::<pcl_msgs::msg::ModelCoefficients>());
        }

        if typename == "pcl_msgs/msg/PointIndices" {
            return Ok(WrappedNativeMsgUntyped::new::<pcl_msgs::msg::PointIndices>());
        }

        if typename == "pcl_msgs/msg/PolygonMesh" {
            return Ok(WrappedNativeMsgUntyped::new::<pcl_msgs::msg::PolygonMesh>());
        }

        if typename == "pcl_msgs/msg/Vertices" {
            return Ok(WrappedNativeMsgUntyped::new::<pcl_msgs::msg::Vertices>());
        }

        if typename == "pendulum_msgs/msg/JointCommand" {
            return Ok(WrappedNativeMsgUntyped::new::<pendulum_msgs::msg::JointCommand>());
        }

        if typename == "pendulum_msgs/msg/JointState" {
            return Ok(WrappedNativeMsgUntyped::new::<pendulum_msgs::msg::JointState>());
        }

        if typename == "pendulum_msgs/msg/RttestResults" {
            return Ok(WrappedNativeMsgUntyped::new::<pendulum_msgs::msg::RttestResults>());
        }

        if typename == "radar_msgs/msg/RadarReturn" {
            return Ok(WrappedNativeMsgUntyped::new::<radar_msgs::msg::RadarReturn>());
        }

        if typename == "radar_msgs/msg/RadarScan" {
            return Ok(WrappedNativeMsgUntyped::new::<radar_msgs::msg::RadarScan>());
        }

        if typename == "radar_msgs/msg/RadarTrack" {
            return Ok(WrappedNativeMsgUntyped::new::<radar_msgs::msg::RadarTrack>());
        }

        if typename == "radar_msgs/msg/RadarTracks" {
            return Ok(WrappedNativeMsgUntyped::new::<radar_msgs::msg::RadarTracks>());
        }

        if typename == "rcl_interfaces/msg/FloatingPointRange" {
            return Ok(WrappedNativeMsgUntyped::new::<rcl_interfaces::msg::FloatingPointRange>());
        }

        if typename == "rcl_interfaces/msg/IntegerRange" {
            return Ok(WrappedNativeMsgUntyped::new::<rcl_interfaces::msg::IntegerRange>());
        }

        if typename == "rcl_interfaces/msg/ListParametersResult" {
            return Ok(WrappedNativeMsgUntyped::new::<rcl_interfaces::msg::ListParametersResult>());
        }

        if typename == "rcl_interfaces/msg/Log" {
            return Ok(WrappedNativeMsgUntyped::new::<rcl_interfaces::msg::Log>());
        }

        if typename == "rcl_interfaces/msg/Parameter" {
            return Ok(WrappedNativeMsgUntyped::new::<rcl_interfaces::msg::Parameter>());
        }

        if typename == "rcl_interfaces/msg/ParameterDescriptor" {
            return Ok(WrappedNativeMsgUntyped::new::<rcl_interfaces::msg::ParameterDescriptor>());
        }

        if typename == "rcl_interfaces/msg/ParameterEvent" {
            return Ok(WrappedNativeMsgUntyped::new::<rcl_interfaces::msg::ParameterEvent>());
        }

        if typename == "rcl_interfaces/msg/ParameterEventDescriptors" {
            return Ok(WrappedNativeMsgUntyped::new::<rcl_interfaces::msg::ParameterEventDescriptors>());
        }

        if typename == "rcl_interfaces/msg/ParameterType" {
            return Ok(WrappedNativeMsgUntyped::new::<rcl_interfaces::msg::ParameterType>());
        }

        if typename == "rcl_interfaces/msg/ParameterValue" {
            return Ok(WrappedNativeMsgUntyped::new::<rcl_interfaces::msg::ParameterValue>());
        }

        if typename == "rcl_interfaces/msg/SetParametersResult" {
            return Ok(WrappedNativeMsgUntyped::new::<rcl_interfaces::msg::SetParametersResult>());
        }

        if typename == "rmw_dds_common/msg/Gid" {
            return Ok(WrappedNativeMsgUntyped::new::<rmw_dds_common::msg::Gid>());
        }

        if typename == "rmw_dds_common/msg/NodeEntitiesInfo" {
            return Ok(WrappedNativeMsgUntyped::new::<rmw_dds_common::msg::NodeEntitiesInfo>());
        }

        if typename == "rmw_dds_common/msg/ParticipantEntitiesInfo" {
            return Ok(WrappedNativeMsgUntyped::new::<rmw_dds_common::msg::ParticipantEntitiesInfo>());
        }

        if typename == "rosapi_msgs/msg/TypeDef" {
            return Ok(WrappedNativeMsgUntyped::new::<rosapi_msgs::msg::TypeDef>());
        }

        if typename == "rosbridge_msgs/msg/ConnectedClient" {
            return Ok(WrappedNativeMsgUntyped::new::<rosbridge_msgs::msg::ConnectedClient>());
        }

        if typename == "rosbridge_msgs/msg/ConnectedClients" {
            return Ok(WrappedNativeMsgUntyped::new::<rosbridge_msgs::msg::ConnectedClients>());
        }

        if typename == "rosgraph_msgs/msg/Clock" {
            return Ok(WrappedNativeMsgUntyped::new::<rosgraph_msgs::msg::Clock>());
        }

        if typename == "sensor_msgs/msg/BatteryState" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::BatteryState>());
        }

        if typename == "sensor_msgs/msg/CameraInfo" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::CameraInfo>());
        }

        if typename == "sensor_msgs/msg/ChannelFloat32" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::ChannelFloat32>());
        }

        if typename == "sensor_msgs/msg/CompressedImage" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::CompressedImage>());
        }

        if typename == "sensor_msgs/msg/FluidPressure" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::FluidPressure>());
        }

        if typename == "sensor_msgs/msg/Illuminance" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::Illuminance>());
        }

        if typename == "sensor_msgs/msg/Image" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::Image>());
        }

        if typename == "sensor_msgs/msg/Imu" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::Imu>());
        }

        if typename == "sensor_msgs/msg/JointState" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::JointState>());
        }

        if typename == "sensor_msgs/msg/Joy" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::Joy>());
        }

        if typename == "sensor_msgs/msg/JoyFeedback" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::JoyFeedback>());
        }

        if typename == "sensor_msgs/msg/JoyFeedbackArray" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::JoyFeedbackArray>());
        }

        if typename == "sensor_msgs/msg/LaserEcho" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::LaserEcho>());
        }

        if typename == "sensor_msgs/msg/LaserScan" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::LaserScan>());
        }

        if typename == "sensor_msgs/msg/MagneticField" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::MagneticField>());
        }

        if typename == "sensor_msgs/msg/MultiDOFJointState" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::MultiDOFJointState>());
        }

        if typename == "sensor_msgs/msg/MultiEchoLaserScan" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::MultiEchoLaserScan>());
        }

        if typename == "sensor_msgs/msg/NavSatFix" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::NavSatFix>());
        }

        if typename == "sensor_msgs/msg/NavSatStatus" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::NavSatStatus>());
        }

        if typename == "sensor_msgs/msg/PointCloud" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::PointCloud>());
        }

        if typename == "sensor_msgs/msg/PointCloud2" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::PointCloud2>());
        }

        if typename == "sensor_msgs/msg/PointField" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::PointField>());
        }

        if typename == "sensor_msgs/msg/Range" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::Range>());
        }

        if typename == "sensor_msgs/msg/RegionOfInterest" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::RegionOfInterest>());
        }

        if typename == "sensor_msgs/msg/RelativeHumidity" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::RelativeHumidity>());
        }

        if typename == "sensor_msgs/msg/Temperature" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::Temperature>());
        }

        if typename == "sensor_msgs/msg/TimeReference" {
            return Ok(WrappedNativeMsgUntyped::new::<sensor_msgs::msg::TimeReference>());
        }

        if typename == "shape_msgs/msg/Mesh" {
            return Ok(WrappedNativeMsgUntyped::new::<shape_msgs::msg::Mesh>());
        }

        if typename == "shape_msgs/msg/MeshTriangle" {
            return Ok(WrappedNativeMsgUntyped::new::<shape_msgs::msg::MeshTriangle>());
        }

        if typename == "shape_msgs/msg/Plane" {
            return Ok(WrappedNativeMsgUntyped::new::<shape_msgs::msg::Plane>());
        }

        if typename == "shape_msgs/msg/SolidPrimitive" {
            return Ok(WrappedNativeMsgUntyped::new::<shape_msgs::msg::SolidPrimitive>());
        }

        if typename == "statistics_msgs/msg/MetricsMessage" {
            return Ok(WrappedNativeMsgUntyped::new::<statistics_msgs::msg::MetricsMessage>());
        }

        if typename == "statistics_msgs/msg/StatisticDataPoint" {
            return Ok(WrappedNativeMsgUntyped::new::<statistics_msgs::msg::StatisticDataPoint>());
        }

        if typename == "statistics_msgs/msg/StatisticDataType" {
            return Ok(WrappedNativeMsgUntyped::new::<statistics_msgs::msg::StatisticDataType>());
        }

        if typename == "std_msgs/msg/Bool" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Bool>());
        }

        if typename == "std_msgs/msg/Byte" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Byte>());
        }

        if typename == "std_msgs/msg/ByteMultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::ByteMultiArray>());
        }

        if typename == "std_msgs/msg/Char" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Char>());
        }

        if typename == "std_msgs/msg/ColorRGBA" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::ColorRGBA>());
        }

        if typename == "std_msgs/msg/Empty" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Empty>());
        }

        if typename == "std_msgs/msg/Float32" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Float32>());
        }

        if typename == "std_msgs/msg/Float32MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Float32MultiArray>());
        }

        if typename == "std_msgs/msg/Float64" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Float64>());
        }

        if typename == "std_msgs/msg/Float64MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Float64MultiArray>());
        }

        if typename == "std_msgs/msg/Header" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Header>());
        }

        if typename == "std_msgs/msg/Int16" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Int16>());
        }

        if typename == "std_msgs/msg/Int16MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Int16MultiArray>());
        }

        if typename == "std_msgs/msg/Int32" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Int32>());
        }

        if typename == "std_msgs/msg/Int32MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Int32MultiArray>());
        }

        if typename == "std_msgs/msg/Int64" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Int64>());
        }

        if typename == "std_msgs/msg/Int64MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Int64MultiArray>());
        }

        if typename == "std_msgs/msg/Int8" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Int8>());
        }

        if typename == "std_msgs/msg/Int8MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::Int8MultiArray>());
        }

        if typename == "std_msgs/msg/MultiArrayDimension" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::MultiArrayDimension>());
        }

        if typename == "std_msgs/msg/MultiArrayLayout" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::MultiArrayLayout>());
        }

        if typename == "std_msgs/msg/String" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::String>());
        }

        if typename == "std_msgs/msg/UInt16" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::UInt16>());
        }

        if typename == "std_msgs/msg/UInt16MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::UInt16MultiArray>());
        }

        if typename == "std_msgs/msg/UInt32" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::UInt32>());
        }

        if typename == "std_msgs/msg/UInt32MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::UInt32MultiArray>());
        }

        if typename == "std_msgs/msg/UInt64" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::UInt64>());
        }

        if typename == "std_msgs/msg/UInt64MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::UInt64MultiArray>());
        }

        if typename == "std_msgs/msg/UInt8" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::UInt8>());
        }

        if typename == "std_msgs/msg/UInt8MultiArray" {
            return Ok(WrappedNativeMsgUntyped::new::<std_msgs::msg::UInt8MultiArray>());
        }

        if typename == "stereo_msgs/msg/DisparityImage" {
            return Ok(WrappedNativeMsgUntyped::new::<stereo_msgs::msg::DisparityImage>());
        }

        if typename == "tf2_msgs/msg/TF2Error" {
            return Ok(WrappedNativeMsgUntyped::new::<tf2_msgs::msg::TF2Error>());
        }

        if typename == "tf2_msgs/msg/TFMessage" {
            return Ok(WrappedNativeMsgUntyped::new::<tf2_msgs::msg::TFMessage>());
        }

        if typename == "theora_image_transport/msg/Packet" {
            return Ok(WrappedNativeMsgUntyped::new::<theora_image_transport::msg::Packet>());
        }

        if typename == "trajectory_msgs/msg/JointTrajectory" {
            return Ok(WrappedNativeMsgUntyped::new::<trajectory_msgs::msg::JointTrajectory>());
        }

        if typename == "trajectory_msgs/msg/JointTrajectoryPoint" {
            return Ok(WrappedNativeMsgUntyped::new::<trajectory_msgs::msg::JointTrajectoryPoint>());
        }

        if typename == "trajectory_msgs/msg/MultiDOFJointTrajectory" {
            return Ok(WrappedNativeMsgUntyped::new::<trajectory_msgs::msg::MultiDOFJointTrajectory>());
        }

        if typename == "trajectory_msgs/msg/MultiDOFJointTrajectoryPoint" {
            return Ok(WrappedNativeMsgUntyped::new::<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>());
        }

        if typename == "turtlesim/msg/Color" {
            return Ok(WrappedNativeMsgUntyped::new::<turtlesim::msg::Color>());
        }

        if typename == "turtlesim/msg/Pose" {
            return Ok(WrappedNativeMsgUntyped::new::<turtlesim::msg::Pose>());
        }

        if typename == "ublox_msgs/msg/Ack" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::Ack>());
        }

        if typename == "ublox_msgs/msg/AidALM" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::AidALM>());
        }

        if typename == "ublox_msgs/msg/AidEPH" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::AidEPH>());
        }

        if typename == "ublox_msgs/msg/AidHUI" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::AidHUI>());
        }

        if typename == "ublox_msgs/msg/CfgANT" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgANT>());
        }

        if typename == "ublox_msgs/msg/CfgCFG" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgCFG>());
        }

        if typename == "ublox_msgs/msg/CfgDAT" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgDAT>());
        }

        if typename == "ublox_msgs/msg/CfgDGNSS" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgDGNSS>());
        }

        if typename == "ublox_msgs/msg/CfgGNSS" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgGNSS>());
        }

        if typename == "ublox_msgs/msg/CfgGNSSBlock" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgGNSSBlock>());
        }

        if typename == "ublox_msgs/msg/CfgHNR" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgHNR>());
        }

        if typename == "ublox_msgs/msg/CfgINF" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgINF>());
        }

        if typename == "ublox_msgs/msg/CfgINFBlock" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgINFBlock>());
        }

        if typename == "ublox_msgs/msg/CfgMSG" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgMSG>());
        }

        if typename == "ublox_msgs/msg/CfgNAV5" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgNAV5>());
        }

        if typename == "ublox_msgs/msg/CfgNAVX5" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgNAVX5>());
        }

        if typename == "ublox_msgs/msg/CfgNMEA" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgNMEA>());
        }

        if typename == "ublox_msgs/msg/CfgNMEA6" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgNMEA6>());
        }

        if typename == "ublox_msgs/msg/CfgNMEA7" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgNMEA7>());
        }

        if typename == "ublox_msgs/msg/CfgPRT" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgPRT>());
        }

        if typename == "ublox_msgs/msg/CfgRATE" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgRATE>());
        }

        if typename == "ublox_msgs/msg/CfgRST" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgRST>());
        }

        if typename == "ublox_msgs/msg/CfgSBAS" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgSBAS>());
        }

        if typename == "ublox_msgs/msg/CfgTMODE3" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgTMODE3>());
        }

        if typename == "ublox_msgs/msg/CfgUSB" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::CfgUSB>());
        }

        if typename == "ublox_msgs/msg/EsfINS" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::EsfINS>());
        }

        if typename == "ublox_msgs/msg/EsfMEAS" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::EsfMEAS>());
        }

        if typename == "ublox_msgs/msg/EsfRAW" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::EsfRAW>());
        }

        if typename == "ublox_msgs/msg/EsfRAWBlock" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::EsfRAWBlock>());
        }

        if typename == "ublox_msgs/msg/EsfSTATUS" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::EsfSTATUS>());
        }

        if typename == "ublox_msgs/msg/EsfSTATUSSens" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::EsfSTATUSSens>());
        }

        if typename == "ublox_msgs/msg/HnrPVT" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::HnrPVT>());
        }

        if typename == "ublox_msgs/msg/Inf" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::Inf>());
        }

        if typename == "ublox_msgs/msg/MgaGAL" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::MgaGAL>());
        }

        if typename == "ublox_msgs/msg/MonGNSS" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::MonGNSS>());
        }

        if typename == "ublox_msgs/msg/MonHW" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::MonHW>());
        }

        if typename == "ublox_msgs/msg/MonHW6" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::MonHW6>());
        }

        if typename == "ublox_msgs/msg/MonVER" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::MonVER>());
        }

        if typename == "ublox_msgs/msg/MonVERExtension" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::MonVERExtension>());
        }

        if typename == "ublox_msgs/msg/NavATT" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavATT>());
        }

        if typename == "ublox_msgs/msg/NavCLOCK" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavCLOCK>());
        }

        if typename == "ublox_msgs/msg/NavDGPS" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavDGPS>());
        }

        if typename == "ublox_msgs/msg/NavDGPSSV" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavDGPSSV>());
        }

        if typename == "ublox_msgs/msg/NavDOP" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavDOP>());
        }

        if typename == "ublox_msgs/msg/NavPOSECEF" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavPOSECEF>());
        }

        if typename == "ublox_msgs/msg/NavPOSLLH" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavPOSLLH>());
        }

        if typename == "ublox_msgs/msg/NavPVT" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavPVT>());
        }

        if typename == "ublox_msgs/msg/NavPVT7" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavPVT7>());
        }

        if typename == "ublox_msgs/msg/NavRELPOSNED" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavRELPOSNED>());
        }

        if typename == "ublox_msgs/msg/NavRELPOSNED9" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavRELPOSNED9>());
        }

        if typename == "ublox_msgs/msg/NavSAT" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavSAT>());
        }

        if typename == "ublox_msgs/msg/NavSATSV" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavSATSV>());
        }

        if typename == "ublox_msgs/msg/NavSBAS" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavSBAS>());
        }

        if typename == "ublox_msgs/msg/NavSBASSV" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavSBASSV>());
        }

        if typename == "ublox_msgs/msg/NavSOL" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavSOL>());
        }

        if typename == "ublox_msgs/msg/NavSTATUS" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavSTATUS>());
        }

        if typename == "ublox_msgs/msg/NavSVIN" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavSVIN>());
        }

        if typename == "ublox_msgs/msg/NavSVINFO" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavSVINFO>());
        }

        if typename == "ublox_msgs/msg/NavSVINFOSV" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavSVINFOSV>());
        }

        if typename == "ublox_msgs/msg/NavTIMEGPS" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavTIMEGPS>());
        }

        if typename == "ublox_msgs/msg/NavTIMEUTC" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavTIMEUTC>());
        }

        if typename == "ublox_msgs/msg/NavVELECEF" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavVELECEF>());
        }

        if typename == "ublox_msgs/msg/NavVELNED" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::NavVELNED>());
        }

        if typename == "ublox_msgs/msg/RxmALM" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::RxmALM>());
        }

        if typename == "ublox_msgs/msg/RxmEPH" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::RxmEPH>());
        }

        if typename == "ublox_msgs/msg/RxmRAW" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::RxmRAW>());
        }

        if typename == "ublox_msgs/msg/RxmRAWSV" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::RxmRAWSV>());
        }

        if typename == "ublox_msgs/msg/RxmRAWX" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::RxmRAWX>());
        }

        if typename == "ublox_msgs/msg/RxmRAWXMeas" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::RxmRAWXMeas>());
        }

        if typename == "ublox_msgs/msg/RxmRTCM" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::RxmRTCM>());
        }

        if typename == "ublox_msgs/msg/RxmSFRB" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::RxmSFRB>());
        }

        if typename == "ublox_msgs/msg/RxmSFRBX" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::RxmSFRBX>());
        }

        if typename == "ublox_msgs/msg/RxmSVSI" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::RxmSVSI>());
        }

        if typename == "ublox_msgs/msg/RxmSVSISV" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::RxmSVSISV>());
        }

        if typename == "ublox_msgs/msg/TimTM2" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::TimTM2>());
        }

        if typename == "ublox_msgs/msg/UpdSOS" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::UpdSOS>());
        }

        if typename == "ublox_msgs/msg/UpdSOSAck" {
            return Ok(WrappedNativeMsgUntyped::new::<ublox_msgs::msg::UpdSOSAck>());
        }

        if typename == "unique_identifier_msgs/msg/UUID" {
            return Ok(WrappedNativeMsgUntyped::new::<unique_identifier_msgs::msg::UUID>());
        }

        if typename == "visualization_msgs/msg/ImageMarker" {
            return Ok(WrappedNativeMsgUntyped::new::<visualization_msgs::msg::ImageMarker>());
        }

        if typename == "visualization_msgs/msg/InteractiveMarker" {
            return Ok(WrappedNativeMsgUntyped::new::<visualization_msgs::msg::InteractiveMarker>());
        }

        if typename == "visualization_msgs/msg/InteractiveMarkerControl" {
            return Ok(WrappedNativeMsgUntyped::new::<visualization_msgs::msg::InteractiveMarkerControl>());
        }

        if typename == "visualization_msgs/msg/InteractiveMarkerFeedback" {
            return Ok(WrappedNativeMsgUntyped::new::<visualization_msgs::msg::InteractiveMarkerFeedback>());
        }

        if typename == "visualization_msgs/msg/InteractiveMarkerInit" {
            return Ok(WrappedNativeMsgUntyped::new::<visualization_msgs::msg::InteractiveMarkerInit>());
        }

        if typename == "visualization_msgs/msg/InteractiveMarkerPose" {
            return Ok(WrappedNativeMsgUntyped::new::<visualization_msgs::msg::InteractiveMarkerPose>());
        }

        if typename == "visualization_msgs/msg/InteractiveMarkerUpdate" {
            return Ok(WrappedNativeMsgUntyped::new::<visualization_msgs::msg::InteractiveMarkerUpdate>());
        }

        if typename == "visualization_msgs/msg/Marker" {
            return Ok(WrappedNativeMsgUntyped::new::<visualization_msgs::msg::Marker>());
        }

        if typename == "visualization_msgs/msg/MarkerArray" {
            return Ok(WrappedNativeMsgUntyped::new::<visualization_msgs::msg::MarkerArray>());
        }

        if typename == "visualization_msgs/msg/MenuEntry" {
            return Ok(WrappedNativeMsgUntyped::new::<visualization_msgs::msg::MenuEntry>());
        }

        return Err(Error::InvalidMessageType{ msgtype: typename.into() })
    }
}
