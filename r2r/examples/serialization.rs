use r2r::{WrappedNativeMsgUntyped, WrappedTypesupport};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let msg = r2r::std_msgs::msg::String {
        data: "Hello, world".into(),
    };

    let bytes = msg.to_serialized_bytes()?;

    // bytes to "untyped" r2r msg
    let mut native = WrappedNativeMsgUntyped::new_from("std_msgs/msg/String")?;
    native.from_serialized_bytes(&bytes)?;

    // "untyped" msg to json
    let json = native.to_json()?;

    println!("as json\n----\n{}", serde_json::to_string_pretty(&json)?);

    // bytes to r2r msg.
    let msg2 = r2r::std_msgs::msg::String::from_serialized_bytes(&bytes)?;

    println!("as r2r msg\n----\n{:#?}", msg2);

    // json to r2r msg
    let msg3: r2r::std_msgs::msg::String = serde_json::from_value(json)?;

    assert_eq!(msg, msg2);
    assert_eq!(msg2, msg3);

    Ok(())
}
