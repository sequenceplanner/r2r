use std::env;
use std::path::PathBuf;

fn main() {
    r2r_common::print_cargo_watches();

    let mut builder = r2r_common::setup_bindgen_builder();
    builder = builder.header("src/rcl_wrapper.h");

    println!("cargo:rustc-link-lib=dylib=rcl");
    println!("cargo:rustc-link-lib=dylib=rcl_logging_spdlog");
    println!("cargo:rustc-link-lib=dylib=rcl_yaml_param_parser");
    println!("cargo:rustc-link-lib=dylib=rcutils");
    println!("cargo:rustc-link-lib=dylib=rmw");
    println!("cargo:rustc-link-lib=dylib=rosidl_typesupport_c");
    println!("cargo:rustc-link-lib=dylib=rosidl_runtime_c");

    let bindings = builder
        .allowlist_type("rcl_.*")
        .allowlist_type("rcutils_.*")
        .allowlist_type("rmw_.*")
        .allowlist_type("rosidl_.*")
        .allowlist_var("RCL_.*")
        .allowlist_var("RCUTILS_.*")
        .allowlist_var("RMW_.*")
        .allowlist_var("rosidl_.*")
        .allowlist_function("rcl_.*")
        .allowlist_function("rcutils_.*")
        .allowlist_function("rmw_.*")
        .allowlist_function("rosidl_.*")
        .allowlist_function(".*_typesupport_.*")
        .allowlist_function(".*_sequence_bound_.*")
        .no_debug("_OSUnaligned.*")
        .derive_partialeq(true)
        .derive_copy(true)
        .generate_comments(false)
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("rcl_bindings.rs"))
        .expect("Couldn't write bindings!");
}
