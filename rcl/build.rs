extern crate bindgen;

use std::env;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");

    let mut builder = bindgen::Builder::default()
        .header("src/rcl_wrapper.h")
        .derive_copy(false)
        .default_enum_style(bindgen::EnumVariation::Rust {
            non_exhaustive: false,
        });

    let ament_prefix_var_name = "AMENT_PREFIX_PATH";
    let ament_prefix_var = env::var(ament_prefix_var_name).expect("Source your ROS!");

    for ament_prefix_path in ament_prefix_var.split(":") {
        builder = builder.clang_arg(format!("-I{}/include", ament_prefix_path));
        println!("cargo:rustc-link-search=native={}/lib", ament_prefix_path);
    }

    println!("cargo:rustc-link-lib=dylib=rcl");
    // println!("cargo:rustc-link-lib=dylib=rcl_logging_noop");
    // default logging seem to be changed to spdlog
    println!("cargo:rustc-link-lib=dylib=rcl_logging_spdlog");
    println!("cargo:rustc-link-lib=dylib=rcutils");
    println!("cargo:rustc-link-lib=dylib=rmw");
    println!("cargo:rustc-link-lib=dylib=rmw_implementation");
    println!("cargo:rustc-link-lib=dylib=rosidl_typesupport_c");
    println!("cargo:rustc-link-lib=dylib=rosidl_generator_c");

    let bindings = builder.generate().expect("Unable to generate bindings");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("rcl_bindings.rs"))
        .expect("Couldn't write bindings!");
}
