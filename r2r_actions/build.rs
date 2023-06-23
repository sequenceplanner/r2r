use std::fs::OpenOptions;
use std::path::{Path, PathBuf};
use std::{env, fs};

const BINDINGS_FILENAME: &str = "action_bindings.rs";

fn main() {
    r2r_common::print_cargo_watches();
    r2r_common::print_cargo_ros_distro();

    run_bindgen();
    run_dynlink();
}

fn run_bindgen() {
    let env_hash = r2r_common::get_env_hash();
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let bindgen_dir = out_dir.join(env_hash);
    let save_dir = manifest_dir.join("bindings");
    let cache_file = bindgen_dir.join(BINDINGS_FILENAME);
    let target_file = out_dir.join(BINDINGS_FILENAME);
    let mark_file = bindgen_dir.join("done");
    let saved_file = save_dir.join(BINDINGS_FILENAME);

    if cfg!(feature = "doc-only") {
        // If "doc-only" feature is present, copy from $crate/bindings/* to OUT_DIR
        eprintln!(
            "Copy from '{}' to '{}'",
            saved_file.display(),
            target_file.display()
        );
        fs::copy(&saved_file, &target_file).unwrap();
    } else {
        // If bindgen was done before, use cached files.
        if !mark_file.exists() {
            eprintln!("Generate bindings file '{}'", cache_file.display());
            fs::create_dir_all(&bindgen_dir).unwrap();
            generate_bindings(&cache_file);
            touch(&mark_file);
        } else {
            eprintln!("Used cached file  '{}'", cache_file.display());
        }

        fs::copy(&cache_file, &target_file).unwrap();

        #[cfg(feature = "save-bindgen")]
        {
            fs::create_dir_all(&save_dir).unwrap();
            fs::copy(&cache_file, &saved_file).unwrap();
        }
    }
}

fn run_dynlink() {
    r2r_common::print_cargo_link_search();
    println!("cargo:rustc-link-lib=dylib=rcl_action");
}

fn generate_bindings(out_file: &Path) {
    let bindings = r2r_common::setup_bindgen_builder()
        .header("src/action_wrapper.h")
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
        .allowlist_type("rcl_action_cancel_request_s")
        .opaque_type("rcl_action_cancel_request_s")
        .allowlist_type("rcl_action_cancel_response_t")
        .allowlist_type("rcl_action_cancel_response_s")
        .allowlist_type("rcl_action_goal_event_t")
        .allowlist_type("rcl_action_goal_event_e")
        .allowlist_type("rcl_action_goal_state_t")
        .opaque_type("rcl_action_goal_state_t")
        .allowlist_type("rcl_action_goal_status_array_t")
        .opaque_type("rcl_action_goal_status_array_t")
        .allowlist_function("rcl_action_.*")
        .allowlist_type("rcl_action_client_options_t")
        .opaque_type("rcl_action_client_options_t")
        .allowlist_type("rcl_action_server_options_t")
        .opaque_type("rcl_action_server_options_t")
        .allowlist_var("RCL_RET_ACTION_.*")
        .generate_comments(false)
        .generate()
        .expect("Unable to generate bindings");

    bindings
        .write_to_file(out_file)
        .expect("Couldn't write bindings!");
}

fn touch(path: &Path) {
    OpenOptions::new()
        .create(true)
        .write(true)
        .open(path)
        .unwrap_or_else(|_| panic!("Unable to create file '{}'", path.display()));
}
