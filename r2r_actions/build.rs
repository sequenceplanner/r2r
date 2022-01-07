use itertools::Itertools;
use std::env;
use std::path::{Path, PathBuf};

fn main() {
    let mut builder = bindgen::Builder::default()
        .header("src/action_wrapper.h")
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
            .for_each(|pp| {
                println!("cargo:rustc-link-search=native={}", pp)
                // we could potentially do the below instead of hardcoding which libs we rely on.
                // let filename = path.file_stem().and_then(|f| f.to_str()).unwrap();
                // let without_lib = filename.strip_prefix("lib").unwrap();
                // println!("cargo:rustc-link-lib=dylib={}", without_lib);
            });
    } else {
        let ament_prefix_var_name = "AMENT_PREFIX_PATH";
        let ament_prefix_var = env::var(ament_prefix_var_name).expect("Source your ROS!");

        for ament_prefix_path in ament_prefix_var.split(':') {
            builder = builder.clang_arg(format!("-I{}/include", ament_prefix_path));
            println!(
                "added include search dir: {}",
                format!("-I{}/include", ament_prefix_path)
            );
            println!("cargo:rustc-link-search=native={}/lib", ament_prefix_path);
        }
    }

    println!("cargo:rustc-link-lib=dylib=rcl_action");

    let bindings = builder
        .no_debug("_OSUnaligned.*")
        .derive_partialeq(true)
        .derive_copy(true)
        // allowlist a few specific things that we need.
        .allowlist_recursively(false)
        .allowlist_type("rcl_action_client_t")
        .opaque_type("rcl_action_client_t")
        .allowlist_type("rcl_action_server_t")
        .opaque_type("rcl_action_server_t")
        .allowlist_type("rcl_action_goal_info_t")
        .allowlist_type("rcl_action_goal_handle_t")
        .opaque_type("rcl_action_goal_handle_t")
        .allowlist_type("rcl_action_cancel_request_t")
        .allowlist_type("rcl_action_cancel_response_t")
        .allowlist_type("rcl_action_goal_event_t")
        .allowlist_type("rcl_action_goal_state_t")
        .opaque_type("rcl_action_goal_state_t")
        .allowlist_type("rcl_action_goal_status_array_t")
        .opaque_type("rcl_action_goal_status_array_t")
        .allowlist_function("rcl_action_.*")
        .allowlist_type("rcl_action_client_options_t")
        .allowlist_type("rcl_action_server_options_t")
        .allowlist_var("RCL_RET_ACTION_.*")
        .generate_comments(false)
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("action_bindings.rs"))
        .expect("Couldn't write bindings!");
}
