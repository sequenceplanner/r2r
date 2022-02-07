use std::collections::HashMap;
use std::fs::{self, File};
use std::io::Read;
use std::path::{Path, PathBuf};

// Hack to build rolling after https://github.com/ros2/rcl/pull/959 was merged.
//
// The problem is that now we need to use CMAKE to properly find the
// include paths. But we don't want to do that so we hope that the ros
// developers use the same convention everytime they move the include
// files to a subdirectory.
//
// The convention is to put include files in include/${PROJECT_NAME}
//
// So we check if there is a double directory on the form
// include/${PROJECT_NAME}/${PROJECT_NAME}, and if so append it only once.
//
// Should work mostly, and shouldn't really change often, so manual
// intervention could be applied. But yes it is hacky.
pub fn guess_cmake_include_path(path: &Path) -> Option<PathBuf> {
    if let Some(leaf) = path.file_name() {
        let double_include_path = Path::new(path).join("include").join(leaf).join(leaf);
        if double_include_path.is_dir() {
            // double dir detected, append the package name
            return Some(path.to_owned().join("include").join(leaf));
        } else {
            // dont append
            return Some(path.to_owned().join("include"));
        }
    }
    return None;
}

pub fn print_cargo_watches() {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-env-changed=CMAKE_INCLUDE_DIRS");
    println!("cargo:rerun-if-env-changed=CMAKE_LIBRARIES");
    println!("cargo:rerun-if-env-changed=CMAKE_RECURSIVE_DEPENDENCIES");
}

#[derive(Debug)]
pub struct RosMsg {
    pub module: String, // e.g. std_msgs
    pub prefix: String, // e.g. "msg" or "srv"
    pub name: String,   // e.g. "String"
}

fn get_msgs_from_package(package: &Path) -> Vec<String> {
    let resource_index_subfolder = "share/ament_index/resource_index";
    let resource_type = "rosidl_interfaces";

    let path = package.to_owned();
    let path = path.join(resource_index_subfolder);
    let path = path.join(resource_type);

    let mut msgs = vec![];

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
                            let substr = &l[4..l.len() - 4];
                            let msg_name = format!("{}/msg/{}", file_name_str, substr);
                            msgs.push(msg_name);
                        }
                    }
                    if l.starts_with("srv/") && (l.ends_with(".idl") || l.ends_with(".srv")) {
                        if let Some(file_name_str) = file_name.to_str() {
                            let substr = &l[4..l.len() - 4];
                            let srv_name = format!("{}/srv/{}", file_name_str, substr);
                            msgs.push(srv_name);
                        }
                    }
                    if l.starts_with("action/") && (l.ends_with(".idl") || l.ends_with(".action")) {
                        if let Some(file_name_str) = file_name.to_str() {
                            let substr = if l.ends_with(".action") {
                                &l[7..l.len() - 7]
                            } else {
                                &l[7..l.len() - 4] // .idl
                            };
                            let action_name = format!("{}/action/{}", file_name_str, substr);
                            println!("found action: {}", action_name);
                            msgs.push(action_name);
                        }
                    }
                });
            }
        }
    }
    msgs.sort();
    msgs.dedup();
    msgs
}

pub fn get_ros_msgs(paths: &[&Path]) -> Vec<String> {
    let mut msgs: Vec<String> = Vec::new();

    for p in paths {
        println!("looking at prefix: {:?}", p);
        let package_msgs = get_msgs_from_package(p);
        println!("... found {:?}", package_msgs);
        msgs.extend(package_msgs)
    }
    msgs.sort();
    msgs.dedup();
    msgs
}

pub fn parse_msgs(msgs: &[String]) -> Vec<RosMsg> {
    let v: Vec<Vec<&str>> = msgs
        .iter()
        .map(|l| l.split('/').into_iter().take(3).collect())
        .collect();

    // hack because I don't have time to find out the root cause of this at the moment.
    // for some reason the library files generated to this are called
    // liblibstatistics_collector_test_msgs__..., but I don't know where test_msgs come from.
    // (this seems to be a useless package anyway)
    // also affects message generation below.
    v.iter()
        .filter(|v| v.len() == 3)
        .map(|v| RosMsg {
            module: v[0].into(),
            prefix: v[1].into(),
            name: v[2].into(),
        })
        .filter(|v| v.module != "libstatistics_collector")
        .collect()
}

pub fn as_map(included_msgs: &[RosMsg]) -> HashMap<&str, HashMap<&str, Vec<&str>>> {
    let mut msgs = HashMap::new();
    for msg in included_msgs {
        msgs.entry(msg.module.as_str())
            .or_insert_with(HashMap::new)
            .entry(msg.prefix.as_str())
            .or_insert_with(Vec::new)
            .push(msg.name.as_str());
    }
    msgs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_msgs() {
        let msgs = "
std_msgs/msg/Bool
x/y
std_msgs/msg/String
";
        let msgs = msgs.lines().map(|l| l.to_string()).collect::<Vec<_>>();
        let parsed = parse_msgs(&msgs);
        assert_eq!(parsed[0].module, "std_msgs");
        assert_eq!(parsed[0].prefix, "msg");
        assert_eq!(parsed[0].name, "Bool");
        assert_eq!(parsed[1].module, "std_msgs");
        assert_eq!(parsed[1].prefix, "msg");
        assert_eq!(parsed[1].name, "String");
    }

    #[test]
    fn test_as_map() {
        let msgs = "
std_msgs/msg/Bool
x/y
std_msgs/msg/String
";
        let msgs: Vec<String> = msgs.lines().map(|l| l.to_string()).collect();
        let parsed = parse_msgs(&msgs);
        let map = as_map(&parsed);

        assert_eq!(map.get("std_msgs").unwrap().get("msg").unwrap()[0], "Bool");
        assert_eq!(
            map.get("std_msgs").unwrap().get("msg").unwrap()[1],
            "String"
        );
    }
}
