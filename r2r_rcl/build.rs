use std::env;
use std::fs;
use std::fs::OpenOptions;
use std::path::Path;
use std::path::PathBuf;

fn main() {
    r2r_common::print_cargo_watches();
    r2r_common::print_cargo_ros_distro();
    run_bindgen();
    run_dynlink();
}

fn run_bindgen() {
    let env_hash = r2r_common::get_env_hash();
    let out_dir: PathBuf = env::var_os("OUT_DIR").unwrap().into();
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let bindgen_dir = out_dir.join(env_hash);
    let cached_file = bindgen_dir.join("rcl_bindings.rs");
    let mark_file = bindgen_dir.join("done");
    let target_file = out_dir.join("rcl_bindings.rs");
    let saved_file = manifest_dir.join("bindings").join("rcl_bindings.rs");

    // Use saved bindings file if "docs-only" feature is enabled.
    let src_file = if cfg!(feature = "doc-only") {
        &saved_file
    } else {
        // Generate bindings if bindings file does not
        // exist. Otherwise, use cached bindings file.
        if !mark_file.exists() {
            eprintln!("Generate bindings file '{}'", cached_file.display());
            gen_bindings(&cached_file);
            touch(&mark_file);
        } else {
            eprintln!("Use cached bindings file '{}'", cached_file.display());
        }

        #[cfg(feature = "save-bindgen")]
        {
            if let Some(dir) = saved_file.parent() {
                fs::create_dir_all(dir)
                    .unwrap_or_else(|_| panic!("Unable to create directory '{}'", dir.display()));
            }
            fs::copy(&cached_file, &saved_file).expect("File copy failed");
        }

        &cached_file
    };

    fs::copy(src_file, &target_file).unwrap_or_else(|_| {
        panic!(
            "Unable to copy from '{}' to '{}'",
            src_file.display(),
            target_file.display()
        )
    });
}

#[cfg(feature = "doc-only")]
fn run_dynlink() {}

#[cfg(not(feature = "doc-only"))]
fn run_dynlink() {
    r2r_common::print_cargo_link_search();
    println!("cargo:rustc-link-lib=dylib=rcl");
    println!("cargo:rustc-link-lib=dylib=rcl_logging_spdlog");
    println!("cargo:rustc-link-lib=dylib=rcl_yaml_param_parser");
    println!("cargo:rustc-link-lib=dylib=rcutils");
    println!("cargo:rustc-link-lib=dylib=rmw");
    println!("cargo:rustc-link-lib=dylib=rosidl_typesupport_c");
    println!("cargo:rustc-link-lib=dylib=rosidl_runtime_c");
}

fn gen_bindings(out_file: &Path) {
    if let Some(dir) = out_file.parent() {
        fs::create_dir_all(dir)
            .unwrap_or_else(|_| panic!("Unable to create directory '{}'", dir.display()));
    }

    let bindings = r2r_common::setup_bindgen_builder()
        .header("src/rcl_wrapper.h")
        .allowlist_type("rcl_.*")
        .allowlist_type("rcutils_.*")
        .allowlist_type("rmw_.*")
        .allowlist_type("rosidl_.*")
        .allowlist_type("RCUTILS_.*")
        .allowlist_var("RCL_.*")
        .allowlist_var("RCUTILS_.*")
        .allowlist_var("RMW_.*")
        .allowlist_var("rosidl_.*")
        .allowlist_var("g_rcutils_.*")
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
