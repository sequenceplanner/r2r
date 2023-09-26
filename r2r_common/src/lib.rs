use itertools::Itertools;
use os_str_bytes::RawOsString;
use regex::*;
use sha2::{Digest, Sha256};
use std::collections::HashMap;
use std::env;
use std::fs::{self, File};
use std::io::Read;
use std::path::Path;

#[cfg(not(feature = "doc-only"))]
const SUPPORTED_ROS_DISTROS: &[&str] = &["foxy", "galactic", "humble", "rolling"];

const WATCHED_ENV_VARS: &[&str] = &[
    "AMENT_PREFIX_PATH",
    "CMAKE_PREFIX_PATH",
    "CMAKE_INCLUDE_DIRS",
    "CMAKE_LIBRARIES",
    "CMAKE_IDL_PACKAGES",
    "IDL_PACKAGE_FILTER",
    "ROS_DISTRO",
];

pub fn get_env_hash() -> String {
    let mut hasher = Sha256::new();
    for var in WATCHED_ENV_VARS {
        hasher.update(var.as_bytes());
        hasher.update("=");

        if let Ok(value) = env::var(var) {
            hasher.update(value);
        }

        hasher.update("\n");
    }
    let hash = hasher.finalize();
    format!("{:x}", hash)
}

pub fn print_cargo_watches() {
    for var in WATCHED_ENV_VARS {
        println!("cargo:rerun-if-env-changed={}", var);
    }
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
    } else if !cfg!(feature = "doc-only") {
        let ament_prefix_var_name = "AMENT_PREFIX_PATH";
        let ament_prefix_var = if !cfg!(target_os = "windows") {
            RawOsString::new(env::var_os(ament_prefix_var_name).expect("Source your ROS!"))
        } else {
            let mut ament_str = env::var_os(ament_prefix_var_name).expect("Source your ROS!");
            if let Some(cmake_prefix_var) = env::var_os("CMAKE_PREFIX_PATH") {
                ament_str.push(";");
                ament_str.push(cmake_prefix_var);
            }
            RawOsString::new(ament_str)
        };
        let split_char = if cfg!(target_os = "windows") {
            ';'
        } else {
            ':'
        };
        for p in ament_prefix_var.split(split_char) {
            let path = Path::new(&p.to_os_str()).join("include");

            let entries = std::fs::read_dir(path.clone());
            if let Ok(e) = entries {
                let dirs = e
                    .filter_map(|a| {
                        let path = a.unwrap().path();
                        if path.is_dir() {
                            Some(path)
                        } else {
                            None
                        }
                    })
                    .collect::<Vec<_>>();

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
                    } else {
                        builder
                    }
                });
            }
        }
    }

    builder
}

#[cfg(feature = "doc-only")]
pub fn print_cargo_ros_distro() {}

#[cfg(not(feature = "doc-only"))]
pub fn print_cargo_ros_distro() {
    if cfg!(feature = "doc-only") {
        return;
    }

    let ros_distro =
        env::var("ROS_DISTRO").unwrap_or_else(|_| panic!("ROS_DISTRO not set: Source your ROS!"));

    if SUPPORTED_ROS_DISTROS.contains(&ros_distro.as_str()) {
        println!("cargo:rustc-cfg=r2r__ros__distro__{ros_distro}");
    } else {
        panic!("ROS_DISTRO not supported: {ros_distro}");
    }
}

pub fn print_cargo_link_search() {
    if env::var_os("CMAKE_INCLUDE_DIRS").is_some() {
        if let Some(paths) = env::var_os("CMAKE_LIBRARIES") {
            let paths = RawOsString::new(paths);
            paths
                .split(':')
                .filter(|s| {
                    s.contains(".so")
                        || s.contains(".dylib")
                        || s.contains(".dll")
                        || s.contains(".lib")
                })
                .filter_map(|l| {
                    let is_dll = l.contains(".dll");
                    let l = l.to_os_str();
                    let parent = if is_dll {
                        // Hack to replace /bin with /lib on windows
                        // (may not work in all cases)
                        // Should really be fixed in cmake integration
                        // but annoying to replace that file in all
                        // repos that use it.
                        Path::new(&l).parent()?.parent()?.join("lib").to_str()?.to_string()
                    } else {
                        Path::new(&l).parent()?.to_str()?.to_string()
                    };
                    Some(parent)
                })
                .unique()
                .for_each(|pp| println!("cargo:rustc-link-search=native={}", pp));
        }
    } else {
        let ament_prefix_var_name = "AMENT_PREFIX_PATH";
        if let Some(paths) = env::var_os(ament_prefix_var_name) {
            let paths = if !cfg!(target_os = "windows") {
                RawOsString::new(paths)
            } else if let Some(cmake_prefix_var) = env::var_os("CMAKE_PREFIX_PATH") {
                let mut cmake_paths = paths;
                cmake_paths.push(";");
                cmake_paths.push(cmake_prefix_var);
                RawOsString::new(cmake_paths)
            } else {
                RawOsString::new(paths)
            };
            let split_char = if cfg!(target_os = "windows") {
                ';'
            } else {
                ':'
            };
            for path in paths.split(split_char) {
                if cfg!(target_os = "windows") {
                    let lib_path = Path::new(&path.to_os_str()).join("Lib");
                    if !lib_path.exists() {
                        continue;
                    }
                    if let Some(s) = lib_path.to_str() {
                        println!("cargo:rustc-link-search={}", s);
                    }
                } else {
                    let lib_path = Path::new(&path.to_os_str()).join("lib");
                    if let Some(s) = lib_path.to_str() {
                        println!("cargo:rustc-link-search=native={}", s)
                    }
                }
            }
        }
    }
}

pub fn get_wanted_messages() -> Vec<RosMsg> {
    let msgs = if let Ok(cmake_package_dirs) = env::var("CMAKE_IDL_PACKAGES") {
        // CMAKE_PACKAGE_DIRS should be a (cmake) list of "cmake" dirs
        // e.g. For each dir install/r2r_minimal_node_msgs/share/r2r_minimal_node_msgs/cmake
        // we can traverse back and then look for .msg files in msg/ srv/ action/
        let dirs = cmake_package_dirs
            .split(':')
            .flat_map(|i| Path::new(i).parent())
            .collect::<Vec<_>>();

        get_ros_msgs_files(&dirs)
    } else {
        // Else we look for all msgs we can find using the ament prefix path.
        let split_char = if cfg!(target_os = "windows") {
            ';'
        } else {
            ':'
        };
        if !cfg!(target_os = "windows") {
            if let Ok(ament_prefix_var) = env::var("AMENT_PREFIX_PATH") {
                let paths = ament_prefix_var
                    .split(split_char)
                    .map(Path::new)
                    .collect::<Vec<_>>();

                get_ros_msgs(&paths)
            } else {
                vec![]
            }
        } else {
            match (env::var("AMENT_PREFIX_PATH"), env::var("CMAKE_PREFIX_PATH")) {
                (Ok(ament_prefix_var), Ok(cmake_prefix_var)) => {
                    let mut paths = ament_prefix_var
                        .split(split_char)
                        .map(Path::new)
                        .collect::<Vec<_>>();
                    paths.extend(cmake_prefix_var.split(split_char).map(Path::new));
                    get_ros_msgs(&paths)
                }
                (Ok(ament_prefix_var), _) => {
                    let paths = ament_prefix_var
                        .split(split_char)
                        .map(Path::new)
                        .collect::<Vec<_>>();
                    get_ros_msgs(&paths)
                }
                (_, Ok(cmake_prefix_var)) => {
                    let paths = cmake_prefix_var
                        .split(split_char)
                        .map(Path::new)
                        .collect::<Vec<_>>();
                    get_ros_msgs(&paths)
                }
                _ => vec![],
            }
        }
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
    let needed_msg_pkgs = &[
        "rcl_interfaces",
        "builtin_interfaces",
        "unique_identifier_msgs",
        "action_msgs",
    ];
    if let Ok(idl_filter) = env::var("IDL_PACKAGE_FILTER") {
        let mut idl_packages = idl_filter.split(';').collect::<Vec<&str>>();
        for needed in needed_msg_pkgs {
            if !idl_packages.contains(needed) {
                idl_packages.push(needed);
            }
        }
        msgs.into_iter()
            .filter(|msg| idl_packages.contains(&msg.module.as_str()))
            .collect()
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

            msgs.push(format!("{}/{}/{}", package, subdir, substr));
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
        .map(|l| l.split('/').take(3).collect())
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

thread_local! {
    static UPPERCASE_BEFORE: Regex = Regex::new(r"(.)([A-Z][a-z]+)").unwrap();
    static UPPERCASE_AFTER: Regex = Regex::new(r"([a-z0-9])([A-Z])").unwrap();
}

/// camel case to to snake case adapted from from ros_idl_cmake. This
/// is not a general "to snake case" converter, it only handles the
/// specific case of CamelCase to snake_case that we need.
pub fn camel_to_snake(s: &str) -> String {
    let s = UPPERCASE_BEFORE.with(|ub| ub.replace_all(s, "${1}_${2}"));
    let s = UPPERCASE_AFTER.with(|ua| ua.replace_all(&s, "${1}_${2}"));
    s.to_lowercase()
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
        assert_eq!(map.get("std_msgs").unwrap().get("msg").unwrap()[1], "String");
    }

    #[test]
    fn test_camel_to_snake_case() {
        assert_eq!(camel_to_snake("AB01CD02"), "ab01_cd02");
        assert_eq!(camel_to_snake("UnboundedSequences"), "unbounded_sequences");
        assert_eq!(
            camel_to_snake("BoundedPlainUnboundedSequences"),
            "bounded_plain_unbounded_sequences"
        );
        assert_eq!(camel_to_snake("WStrings"), "w_strings");
    }
}
