
impl UntypedActionSupport {
    pub fn new_from(typename: &str) -> Result<Self> {

        if typename == "action_tutorials_interfaces/action/Fibonacci" {
            return Ok(UntypedActionSupport::new::<action_tutorials_interfaces::action::Fibonacci::Action>());
        }

        if typename == "example_interfaces/action/Fibonacci" {
            return Ok(UntypedActionSupport::new::<example_interfaces::action::Fibonacci::Action>());
        }

        if typename == "nav2_msgs/action/BackUp" {
            return Ok(UntypedActionSupport::new::<nav2_msgs::action::BackUp::Action>());
        }

        if typename == "nav2_msgs/action/ComputePathThroughPoses" {
            return Ok(UntypedActionSupport::new::<nav2_msgs::action::ComputePathThroughPoses::Action>());
        }

        if typename == "nav2_msgs/action/ComputePathToPose" {
            return Ok(UntypedActionSupport::new::<nav2_msgs::action::ComputePathToPose::Action>());
        }

        if typename == "nav2_msgs/action/DummyRecovery" {
            return Ok(UntypedActionSupport::new::<nav2_msgs::action::DummyRecovery::Action>());
        }

        if typename == "nav2_msgs/action/FollowPath" {
            return Ok(UntypedActionSupport::new::<nav2_msgs::action::FollowPath::Action>());
        }

        if typename == "nav2_msgs/action/FollowWaypoints" {
            return Ok(UntypedActionSupport::new::<nav2_msgs::action::FollowWaypoints::Action>());
        }

        if typename == "nav2_msgs/action/NavigateThroughPoses" {
            return Ok(UntypedActionSupport::new::<nav2_msgs::action::NavigateThroughPoses::Action>());
        }

        if typename == "nav2_msgs/action/NavigateToPose" {
            return Ok(UntypedActionSupport::new::<nav2_msgs::action::NavigateToPose::Action>());
        }

        if typename == "nav2_msgs/action/Spin" {
            return Ok(UntypedActionSupport::new::<nav2_msgs::action::Spin::Action>());
        }

        if typename == "nav2_msgs/action/Wait" {
            return Ok(UntypedActionSupport::new::<nav2_msgs::action::Wait::Action>());
        }

        if typename == "tf2_msgs/action/LookupTransform" {
            return Ok(UntypedActionSupport::new::<tf2_msgs::action::LookupTransform::Action>());
        }

        if typename == "turtlesim/action/RotateAbsolute" {
            return Ok(UntypedActionSupport::new::<turtlesim::action::RotateAbsolute::Action>());
        }

        return Err(Error::InvalidMessageType{ msgtype: typename.into() })
    }
}
