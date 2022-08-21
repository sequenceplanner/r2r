use std::collections::HashMap;
use std::fs::{self, File};
use std::io::Read;
use std::path::Path;
use std::env;
use itertools::Itertools;

pub fn print_cargo_watches() {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");
    println!("cargo:rerun-if-env-changed=CMAKE_INCLUDE_DIRS");
    println!("cargo:rerun-if-env-changed=CMAKE_LIBRARIES");
    println!("cargo:rerun-if-env-changed=CMAKE_IDL_PACKAGES");
    println!("cargo:rerun-if-env-changed=IDL_PACKAGE_FILTER");
}

pub fn setup_bindgen_builder() -> bindgen::Builder {
    let mut builder = bindgen::Builder::default()
        .derive_copy(false)
        .size_t_is_usize(true)
        .default_enum_style(bindgen::EnumVariation::Rust {
            non_exhaustive: false,
        });

    if let Ok(cmake_includes) = env::var("CMAKE_INCLUDE_DIRS") {
        // we are running from cmake, do special thing.
        let mut includes = cmake_includes.split(':').collect::<Vec<_>>();
        includes.sort_unstable();
        includes.dedup();

        for x in &includes {
            let clang_arg = format!("-I{}", x);
            println!("adding clang arg: {}", clang_arg);
            builder = builder.clang_arg(clang_arg);
        }

        env::var("CMAKE_LIBRARIES")
            .unwrap_or_default()
            .split(':')
            .into_iter()
            .filter(|s| s.contains(".so") || s.contains(".dylib"))
            .flat_map(|l| Path::new(l).parent().and_then(|p| p.to_str()))
            .unique()
            .for_each(|pp| println!("cargo:rustc-link-search=native={}", pp));
    } else {
        let ament_prefix_var_name = "AMENT_PREFIX_PATH";
        let ament_prefix_var = env::var(ament_prefix_var_name).expect("Source your ROS!");

        for p in ament_prefix_var.split(':') {
            let path = Path::new(p).join("include");

            let entries = std::fs::read_dir(path.clone());
            if let Ok(e) = entries {
                let dirs = e.filter_map(|a| {
                    let path = a.unwrap().path();
                    if path.is_dir() {
                        Some(path)
                    } else {
                        None
                    }
                }).collect::<Vec<_>>();

                builder = dirs.iter().fold(builder, |builder, d| {
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
                    if let Some(leaf) = d.file_name() {
                        let double_include_path = Path::new(d).join(leaf);
                        if double_include_path.is_dir() {
                            let temp = d.to_str().unwrap();
                            builder.clang_arg(format!("-I{}", temp))
                        } else {
                            // pre humble case, where we did not have include/package/package
                            let temp = d.parent().unwrap().to_str().unwrap();
                            builder.clang_arg(format!("-I{}", temp))
                        }
                    } else { builder }
                });
            }

            let lib_path = Path::new(p).join("lib");
            lib_path.to_str().map(|s| {
                println!("cargo:rustc-link-search=native={}", s);
            });
        }
    }

    return builder;
}

pub fn get_wanted_messages() -> Vec<RosMsg> {
    let msgs = if let Ok(cmake_package_dirs) = env::var("CMAKE_IDL_PACKAGES") {
        // CMAKE_PACKAGE_DIRS should be a (cmake) list of "cmake" dirs
        // e.g. For each dir install/r2r_minimal_node_msgs/share/r2r_minimal_node_msgs/cmake
        // we can traverse back and then look for .msg files in msg/ srv/ action/
        let dirs = cmake_package_dirs.split(':')
            .flat_map(|i| Path::new(i).parent())
            .collect::<Vec<_>>();

        get_ros_msgs_files(&dirs)
    } else {
        // Else we look for all msgs we can find using the ament prefix path.
        let ament_prefix_var = env::var("AMENT_PREFIX_PATH").expect("Source your ROS!");
        let paths = ament_prefix_var
            .split(':')
            .map(Path::new)
            .collect::<Vec<_>>();

        get_ros_msgs(&paths)
    };

    let msgs = parse_msgs(&msgs);

    // When working on large workspaces without colcon, build times
    // can be a pain. This code adds a the possibility to define an
    // additional filter to make building a little bit quicker.
    //
    // The environment variable IDL_PACKAGE_FILTER should be a semicolon
    // separated list of package names (e.g. std_msgs;my_msgs), so it
    // is required to be correct for packages to be used. This means
    // dependencies need to be manually specified.
    //
    // Suitable to customize with .cargo/config.toml [env] from consumers
    // of the r2r package.
    let needed_msg_pkgs = &["rcl_interfaces", "builtin_interfaces",
                            "unique_identifier_msgs", "action_msgs"];
    if let Ok(idl_filter) = env::var("IDL_PACKAGE_FILTER") {
        let mut idl_packages = idl_filter.split(';').collect::<Vec<&str>>();
        for needed in needed_msg_pkgs {
            if !idl_packages.contains(&needed) {
                idl_packages.push(needed);
            }
        }
        msgs.into_iter().filter(|msg| {
            idl_packages.contains(&msg.module.as_str())
        }).collect()
    } else {
        msgs
    }
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
            let path = path.unwrap().path();
            let path2 = path.clone();
            let file_name = path2.file_name().unwrap();

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
        let package_msgs = get_msgs_from_package(p);
        msgs.extend(package_msgs)
    }
    msgs.sort();
    msgs.dedup();
    msgs
}


fn get_msgs_in_dir(base: &Path, subdir: &str, package: &str) -> Vec<String> {
    let path = base.to_owned();
    let path = path.join(subdir);

    let mut msgs = vec![];

    if let Ok(paths) = fs::read_dir(path) {
        for path in paths {
            let path = path.unwrap().path();
            let filename = path.file_name().unwrap().to_str().unwrap();

            // message name.idl or name.msg
            if !filename.ends_with(".idl") {
                continue;
            }

            let substr = &filename[0..filename.len() - 4];

            msgs.push(format!("{}/{}/{}",package, subdir, substr));
        }
    }
    msgs
}

pub fn get_ros_msgs_files(paths: &[&Path]) -> Vec<String> {
    let mut msgs: Vec<String> = Vec::new();

    for p in paths {
        if let Some(package_name) = p.file_name() {
            let package_name = package_name.to_str().unwrap();
            msgs.extend(get_msgs_in_dir(p, "msg", package_name));
            msgs.extend(get_msgs_in_dir(p, "srv", package_name));
            msgs.extend(get_msgs_in_dir(p, "action", package_name));
        }
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
