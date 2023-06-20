
impl UntypedActionSupport {
    pub fn new_from(typename: &str) -> Result<Self> {

        if typename == "test_msgs/action/Fibonacci" {
            return Ok(UntypedActionSupport::new::<test_msgs::action::Fibonacci::Action>());
        }

        if typename == "test_msgs/action/NestedMessage" {
            return Ok(UntypedActionSupport::new::<test_msgs::action::NestedMessage::Action>());
        }

        if typename == "tf2_msgs/action/LookupTransform" {
            return Ok(UntypedActionSupport::new::<tf2_msgs::action::LookupTransform::Action>());
        }

        return Err(Error::InvalidMessageType{ msgtype: typename.into() })
    }
}
