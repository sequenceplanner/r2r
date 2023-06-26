impl UntypedActionSupport {
    pub fn new_from(typename: &str) -> Result<Self> {
        #[allow(non_snake_case)]
        fn new_untyped_service_support_test_msgs_action_Fibonacci() -> UntypedActionSupport {
            UntypedActionSupport::new::<test_msgs::action::Fibonacci::Action>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_test_msgs_action_NestedMessage() -> UntypedActionSupport {
            UntypedActionSupport::new::<test_msgs::action::NestedMessage::Action>()
        }
        #[allow(non_snake_case)]
        fn new_untyped_service_support_tf2_msgs_action_LookupTransform() -> UntypedActionSupport {
            UntypedActionSupport::new::<tf2_msgs::action::LookupTransform::Action>()
        }
        static MAP: phf::Map<&'static str, fn() -> UntypedActionSupport> = phf::phf_map! {
            "test_msgs/action/Fibonacci" =>
            new_untyped_service_support_test_msgs_action_Fibonacci,
            "test_msgs/action/NestedMessage" =>
            new_untyped_service_support_test_msgs_action_NestedMessage,
            "tf2_msgs/action/LookupTransform" =>
            new_untyped_service_support_tf2_msgs_action_LookupTransform
        };
        let func = MAP
            .get(typename)
            .ok_or_else(|| Error::InvalidMessageType {
                msgtype: typename.into(),
            })?;
        Ok(func())
    }
}
