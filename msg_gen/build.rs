use bindgen;
use common;
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::{Path, PathBuf};

fn main() {
    common::print_cargo_watches();

    let mut builder = bindgen::Builder::default();

    let msg_list = if let Some(cmake_includes) = env::var("CMAKE_INCLUDE_DIRS").ok() {
        let packages = cmake_includes
            .split(":")
            .flat_map(|i| Path::new(i).parent())
            .collect::<Vec<_>>();
        for p in cmake_includes.split(":") {
            builder = builder.clang_arg(format!("-I{}", p));
        }
        let deps = env::var("CMAKE_IDL_PACKAGES").unwrap_or(String::default());
        let deps = deps.split(":").collect::<Vec<_>>();
        let msgs = common::get_ros_msgs(&packages);
        common::parse_msgs(&msgs)
            .into_iter()
            .filter(|msg| deps.contains(&msg.module.as_str()))
            .collect::<Vec<_>>()
    } else {
        let ament_prefix_var = env::var("AMENT_PREFIX_PATH").expect("Source your ROS!");
        for p in ament_prefix_var.split(":") {
            builder = builder.clang_arg(format!("-I{}/include", p));
        }
        let paths = ament_prefix_var
            .split(":")
            .map(|i| Path::new(i))
            .collect::<Vec<_>>();

        let msgs = common::get_ros_msgs(&paths);
        common::parse_msgs(&msgs)
    };

    let msg_map = common::as_map(&msg_list);

    for module in msg_map.keys() {
        println!(
            "cargo:rustc-link-lib=dylib={}__rosidl_typesupport_c",
            module
        );
        println!(
            "cargo:rustc-link-lib=dylib={}__rosidl_typesupport_introspection_c",
            module
        );
        println!("cargo:rustc-link-lib=dylib={}__rosidl_generator_c", module);
    }

    let mut includes = String::new();
    let mut introspecion_map = String::from(
        "\
         lazy_static! { \n
           static ref INTROSPECTION_FNS: HashMap<&'static str, usize> = {\n
             let mut m = HashMap::new();\n",
    );

    for msg in msg_list {
        // filename is certainly CamelCase -> snake_case. convert
        use heck::SnakeCase;
        let include_filename = msg.name.to_snake_case();

        includes.push_str(&format!(
            "#include <{}/{}/{}.h>\n",
            &msg.module, &msg.prefix, &include_filename
        ));
        includes.push_str(&format!(
            "#include <{}/{}/detail/{}__rosidl_typesupport_introspection_c.h>\n",
            &msg.module, &msg.prefix, &include_filename
        ));

        if msg.prefix == "srv" {
            for s in &["Request", "Response"] {
                let key = &format!("{}__{}__{}_{}", &msg.module, &msg.prefix, &msg.name, s);
                let val = &format!("unsafe {{ rosidl_typesupport_introspection_c__get_message_type_support_handle__{}__{}__{}_{}() }} as *const i32 as usize", &msg.module, &msg.prefix, &msg.name, s);
                introspecion_map.push_str(&format!("m.insert(\"{}\", {});\n", key, val));
            }
        } else if msg.prefix == "action" {
            for s in &["Goal", "Result", "Feedback", "FeedbackMessage"] {
                let key = &format!("{}__{}__{}_{}", &msg.module, &msg.prefix, &msg.name, s);
                let val = &format!("unsafe {{ rosidl_typesupport_introspection_c__get_message_type_support_handle__{}__{}__{}_{}() }} as *const i32 as usize", &msg.module, &msg.prefix, &msg.name, s);
                introspecion_map.push_str(&format!("m.insert(\"{}\", {});\n", key, val));
            }
            // "internal" services
            for srv in &["SendGoal", "GetResult"] {
                // TODO: refactor this is copy paste from services...
                for s in &["Request", "Response"] {
                    let msgname = format!("{}_{}_{}", msg.name, srv, s);
                    let key = &format!("{}__{}__{}", &msg.module, &msg.prefix, msgname);
                    let val = &format!("unsafe {{ rosidl_typesupport_introspection_c__get_message_type_support_handle__{}__{}__{}() }} as *const i32 as usize", &msg.module, &msg.prefix, msgname);
                    introspecion_map.push_str(&format!("m.insert(\"{}\", {});\n", key, val));
                }
            }
        } else {
            let key = &format!("{}__{}__{}", &msg.module, &msg.prefix, &msg.name);
            let val = &format!("unsafe {{ rosidl_typesupport_introspection_c__get_message_type_support_handle__{}__{}__{}() }} as *const i32 as usize", &msg.module, &msg.prefix, &msg.name);
            introspecion_map.push_str(&format!("m.insert(\"{}\", {});\n", key, val));
        }
    }
    introspecion_map.push_str("m \n }; }\n\n");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    let msg_includes_fn = out_path.join("msg_includes.h");
    let introspection_fn = out_path.join("introspection_functions.rs");

    let mut f = File::create(msg_includes_fn.clone()).unwrap();
    write!(f, "{}", includes).unwrap();

    let mut f = File::create(introspection_fn).unwrap();
    write!(f, "{}", introspecion_map).unwrap();

    builder = builder
        .header(msg_includes_fn.to_str().unwrap())
        .derive_copy(false)
        // blacklist types that are handled by rcl bindings
        .blacklist_type("rosidl_message_type_support_t")
        .blacklist_type("rosidl_service_type_support_t")
        .blacklist_type("rosidl_runtime_c__String")
        .blacklist_type("rosidl_runtime_c__String__Sequence")
        .blacklist_type("rosidl_runtime_c__U16String")
        .blacklist_type("rosidl_runtime_c__U16String__Sequence")
        .blacklist_type("rosidl_runtime_c__float32__Sequence")
        .blacklist_type("rosidl_runtime_c__float__Sequence")
        .blacklist_type("rosidl_runtime_c__float64__Sequence")
        .blacklist_type("rosidl_runtime_c__double__Sequence")
        .blacklist_type("rosidl_runtime_c__long_double__Sequence")
        .blacklist_type("rosidl_runtime_c__char__Sequence")
        .blacklist_type("rosidl_runtime_c__wchar__Sequence")
        .blacklist_type("rosidl_runtime_c__boolean__Sequence")
        .blacklist_type("rosidl_runtime_c__octet__Sequence")
        .blacklist_type("rosidl_runtime_c__uint8__Sequence")
        .blacklist_type("rosidl_runtime_c__int8__Sequence")
        .blacklist_type("rosidl_runtime_c__uint16__Sequence")
        .blacklist_type("rosidl_runtime_c__int16__Sequence")
        .blacklist_type("rosidl_runtime_c__uint32__Sequence")
        .blacklist_type("rosidl_runtime_c__int32__Sequence")
        .blacklist_type("rosidl_runtime_c__uint64__Sequence")
        .blacklist_type("rosidl_runtime_c__int64__Sequence")
        .size_t_is_usize(true)
        .no_debug("_OSUnaligned.*")
        .default_enum_style(bindgen::EnumVariation::Rust {
            non_exhaustive: false,
        });

    let bindings = builder.generate().expect("Unable to generate bindings");

    bindings
        .write_to_file(out_path.join("msg_bindings.rs"))
        .expect("Couldn't write bindings!");
}
