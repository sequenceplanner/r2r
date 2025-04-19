#[cfg(not(feature = "tracing"))]
fn main() {}

#[cfg(feature = "tracing")]
fn main() {
    use tracing::{generate_r2r_tracepoints, generate_rclcpp_tracepoint_bindings};

    generate_rclcpp_tracepoint_bindings();
    generate_r2r_tracepoints();
}

#[cfg(feature = "tracing")]
mod tracing {
    use lttng_ust_generate::{CIntegerType, CTFType, Generator, Provider};
    use std::{env, path::PathBuf};

    macro_rules! create_tracepoint {
    ($provider:ident::$name:ident($($arg_name:ident: $arg_lttng_type:expr),* $(,)?)) => {
        $provider.create_class(concat!(stringify!($name), "_class"))
            $(
                .add_field(stringify!($arg_name), $arg_lttng_type)
            )*
        .instantiate(stringify!($name))
    };
}

    /// Generate bindings to the rclcpp tracepoints defined in the tracetools ros2 package.
    pub(crate) fn generate_rclcpp_tracepoint_bindings() {
        let bindings = r2r_common::setup_bindgen_builder()
            .header("src/tracetools_wrapper.h")
            .allowlist_function("ros_trace_rclcpp_.*")
            .allowlist_function("ros_trace_callback_.*")
            .generate_comments(true)
            .generate()
            .expect("Unable to generate bindings for tracetools");

        println!("cargo:rustc-link-lib=tracetools");

        // Write the bindings to the $OUT_DIR/tracetools_bindings.rs file.
        let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
        bindings
            .write_to_file(out_path.join("tracetools_bindings.rs"))
            .expect("Couldn't write tracetools bindings!");
    }

    pub(crate) fn generate_r2r_tracepoints() {
        let mut r2r = Provider::new("r2r");

        create_tracepoint!(r2r::spin_start(
            node_handle: CTFType::IntegerHex(CIntegerType::USize),
            timeout_s: CTFType::Integer(CIntegerType::U64),
            timeout_ns: CTFType::Integer(CIntegerType::U32),
        ));
        create_tracepoint!(r2r::spin_end(
            node_handle: CTFType::IntegerHex(CIntegerType::USize),
        ));
        create_tracepoint!(r2r::spin_wake(
            node_handle: CTFType::IntegerHex(CIntegerType::USize),
        ));
        create_tracepoint!(r2r::spin_timeout(
            node_handle: CTFType::IntegerHex(CIntegerType::USize),
        ));

        Generator::default()
            .generated_lib_name("r2r_tracepoints_r2r")
            .register_provider(r2r)
            .output_file_name(
                PathBuf::from(env::var("OUT_DIR").unwrap()).join("r2r_tracepoints.rs"),
            )
            .generate()
            .expect("Unable to generate tracepoint bindings for r2r");
    }
}
