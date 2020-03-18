use std::collections::HashMap;
use std::env;
use std::fs::{self, File};
use std::io::Read;
use std::path::PathBuf;

#[derive(Debug)]
pub struct RosMsg {
    pub module: String, // e.g. std_msgs
    pub prefix: String, // e.g. "msg" or "srv"
    pub name: String, // e.g. "String"
}

// TODO: actions and srv are similiar
pub fn get_all_ros_msgs() -> Vec<String> {
    let resource_index_subfolder = "share/ament_index/resource_index";
    let resource_type = "rosidl_interfaces";
    let ament_prefix_var_name = "AMENT_PREFIX_PATH";
    let ament_prefix_var = env::var(ament_prefix_var_name).expect("Source your ROS!");

    let mut msgs: Vec<String> = Vec::new();

    for ament_prefix_path in ament_prefix_var.split(":") {
        // println!("prefix: {}", ament_prefix_path);

        let path = PathBuf::from(ament_prefix_path);
        let path = path.join(resource_index_subfolder);
        let path = path.join(resource_type);

        if let Ok(paths) = fs::read_dir(path) {

            for path in paths {
                // println!("PATH Name: {}", path.unwrap().path().display());

                let path = path.unwrap().path();
                let path2 = path.clone();
                let file_name = path2.file_name().unwrap();

                // println!("Messages for: {:?}", file_name);
                if let Ok(mut file) = File::open(path) {
                    let mut s = String::new();
                    file.read_to_string(&mut s).unwrap();
                    let lines = s.lines();

                    lines.for_each(|l| {
                        if l.starts_with("msg/") && (l.ends_with(".idl") || l.ends_with(".msg")) {
                            if let Some(file_name_str) = file_name.to_str() {
                                let substr = &l[4..l.len()-4];
                                let msg_name = format!("{}/msg/{}", file_name_str, substr);
                                msgs.push(msg_name);
                            }
                        }
                    });
                }
            }
        }
    }

    msgs.sort();
    msgs.dedup();

    return msgs;
}

#[test]
fn test_msg_list() {

    let msgs = get_all_ros_msgs();
    for m in &msgs {
        println!("{}", m);
    }

    assert!(msgs.contains(&"std_msgs/msg/String".to_string()));
    assert!(msgs.contains(&"builtin_interfaces/msg/Time".to_string()));

}

pub fn parse_msgs(msgs: &Vec<String>) -> Vec<RosMsg> {
    let v: Vec<Vec<&str>> = msgs.iter().map(|l| l.split("/").into_iter().take(3).collect()).collect();
    v.iter().filter(|v|v.len() == 3).
        map(|v| RosMsg { module: v[0].into(), prefix: v[1].into(), name: v[2].into()}).collect()

}

pub fn as_map(included_msgs: &[RosMsg]) -> HashMap<&str, HashMap<&str, Vec<&str>>> {
    let mut msgs = HashMap::new();
    for msg in included_msgs {
        msgs.entry(msg.module.as_str()).or_insert(HashMap::new()).entry(msg.prefix.as_str()).
            or_insert(Vec::new()).push(msg.name.as_str());
    }
    msgs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_msgs() -> () {
        let msgs = "
std_msgs/msg/Bool
x/y
std_msgs/msg/String
";
        let msgs = msgs.lines().map(|l|l.to_string()).collect();
        let parsed = parse_msgs(&msgs);
        assert_eq!(parsed[0].module, "std_msgs");
        assert_eq!(parsed[0].prefix, "msg");
        assert_eq!(parsed[0].name, "Bool");
        assert_eq!(parsed[1].module, "std_msgs");
        assert_eq!(parsed[1].prefix, "msg");
        assert_eq!(parsed[1].name, "String");
    }

    #[test]
    fn test_as_map() -> () {
        let msgs = "
std_msgs/msg/Bool
x/y
std_msgs/msg/String
";
        let msgs: Vec<String> = msgs.lines().map(|l|l.to_string()).collect();
        let parsed = parse_msgs(&msgs);
        let map = as_map(&parsed);

        assert_eq!(map.get("std_msgs").unwrap().get("msg").unwrap()[0], "Bool");
        assert_eq!(map.get("std_msgs").unwrap().get("msg").unwrap()[1], "String");


    }


}
