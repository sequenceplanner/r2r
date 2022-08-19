use std::env;
use std::path::PathBuf;

fn main() {
    r2r_common::print_cargo_watches();

    let mut builder = r2r_common::setup_bindgen_builder();
    builder = builder.header("src/action_wrapper.h");

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

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("action_bindings.rs"))
        .expect("Couldn't write bindings!");
}
