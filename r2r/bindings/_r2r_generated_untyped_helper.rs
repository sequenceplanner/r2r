
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

        if typename == "builtin_interfaces/msg/Duration" {
            return Ok(WrappedNativeMsgUntyped::new::<builtin_interfaces::msg::Duration>());
        }

        if typename == "builtin_interfaces/msg/Time" {
            return Ok(WrappedNativeMsgUntyped::new::<builtin_interfaces::msg::Time>());
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

        if typename == "rosbag2_interfaces/msg/ReadSplitEvent" {
            return Ok(WrappedNativeMsgUntyped::new::<rosbag2_interfaces::msg::ReadSplitEvent>());
        }

        if typename == "rosbag2_interfaces/msg/WriteSplitEvent" {
            return Ok(WrappedNativeMsgUntyped::new::<rosbag2_interfaces::msg::WriteSplitEvent>());
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

        if typename == "test_msgs/msg/Arrays" {
            return Ok(WrappedNativeMsgUntyped::new::<test_msgs::msg::Arrays>());
        }

        if typename == "test_msgs/msg/BasicTypes" {
            return Ok(WrappedNativeMsgUntyped::new::<test_msgs::msg::BasicTypes>());
        }

        if typename == "test_msgs/msg/BoundedPlainSequences" {
            return Ok(WrappedNativeMsgUntyped::new::<test_msgs::msg::BoundedPlainSequences>());
        }

        if typename == "test_msgs/msg/BoundedSequences" {
            return Ok(WrappedNativeMsgUntyped::new::<test_msgs::msg::BoundedSequences>());
        }

        if typename == "test_msgs/msg/Builtins" {
            return Ok(WrappedNativeMsgUntyped::new::<test_msgs::msg::Builtins>());
        }

        if typename == "test_msgs/msg/Constants" {
            return Ok(WrappedNativeMsgUntyped::new::<test_msgs::msg::Constants>());
        }

        if typename == "test_msgs/msg/Defaults" {
            return Ok(WrappedNativeMsgUntyped::new::<test_msgs::msg::Defaults>());
        }

        if typename == "test_msgs/msg/Empty" {
            return Ok(WrappedNativeMsgUntyped::new::<test_msgs::msg::Empty>());
        }

        if typename == "test_msgs/msg/MultiNested" {
            return Ok(WrappedNativeMsgUntyped::new::<test_msgs::msg::MultiNested>());
        }

        if typename == "test_msgs/msg/Nested" {
            return Ok(WrappedNativeMsgUntyped::new::<test_msgs::msg::Nested>());
        }

        if typename == "test_msgs/msg/Strings" {
            return Ok(WrappedNativeMsgUntyped::new::<test_msgs::msg::Strings>());
        }

        if typename == "test_msgs/msg/UnboundedSequences" {
            return Ok(WrappedNativeMsgUntyped::new::<test_msgs::msg::UnboundedSequences>());
        }

        if typename == "test_msgs/msg/WStrings" {
            return Ok(WrappedNativeMsgUntyped::new::<test_msgs::msg::WStrings>());
        }

        if typename == "tf2_msgs/msg/TF2Error" {
            return Ok(WrappedNativeMsgUntyped::new::<tf2_msgs::msg::TF2Error>());
        }

        if typename == "tf2_msgs/msg/TFMessage" {
            return Ok(WrappedNativeMsgUntyped::new::<tf2_msgs::msg::TFMessage>());
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

        if typename == "visualization_msgs/msg/MeshFile" {
            return Ok(WrappedNativeMsgUntyped::new::<visualization_msgs::msg::MeshFile>());
        }

        if typename == "visualization_msgs/msg/UVCoordinate" {
            return Ok(WrappedNativeMsgUntyped::new::<visualization_msgs::msg::UVCoordinate>());
        }

        return Err(Error::InvalidMessageType{ msgtype: typename.into() })
    }
}
