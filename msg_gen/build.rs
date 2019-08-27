extern crate bindgen;

use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

use common::*;

fn main() {
    println!("cargo:rerun-if-changed=../msgs.txt");

    let msgs = read_file("../msgs.txt").expect("You need to create msgs.txt");
    let msg_list = parse_msgs(&msgs);
    let msg_map = as_map(&msg_list);

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
            "#include <{}/{}/{}__rosidl_typesupport_introspection_c.h>\n",
            &msg.module, &msg.prefix, &include_filename
        ));

        let key = &format!("{}__{}__{}", &msg.module, &msg.prefix, &msg.name);
        let val = &format!("unsafe {{ rosidl_typesupport_introspection_c__get_message_type_support_handle__{}__{}__{}() }} as *const i32 as usize", &msg.module, &msg.prefix, &msg.name);
        introspecion_map.push_str(&format!("m.insert(\"{}\", {});\n", key, val));
    }
    introspecion_map.push_str("m \n }; }\n\n");

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    let msg_includes_fn = out_path.join("msg_includes.h");
    let introspection_fn = out_path.join("introspection_functions.rs");

    let mut f = File::create(msg_includes_fn.clone()).unwrap();
    write!(f, "{}", includes).unwrap();

    let mut f = File::create(introspection_fn).unwrap();
    write!(f, "{}", introspecion_map).unwrap();

    let mut builder = bindgen::Builder::default()
        .header(msg_includes_fn.to_str().unwrap())
        .derive_copy(false)
        // blacklist types that are handled by rcl bindings
        .blacklist_type("rosidl_message_type_support_t")
        .blacklist_type("rosidl_generator_c__String")
        .blacklist_type("rosidl_generator_c__String__Sequence")
        .blacklist_type("rosidl_generator_c__float32__Sequence")
        .blacklist_type("rosidl_generator_c__float__Sequence")
        .blacklist_type("rosidl_generator_c__float64__Sequence")
        .blacklist_type("rosidl_generator_c__double__Sequence")
        .blacklist_type("rosidl_generator_c__long_double__Sequence")
        .blacklist_type("rosidl_generator_c__char__Sequence")
        .blacklist_type("rosidl_generator_c__wchar__Sequence")
        .blacklist_type("rosidl_generator_c__boolean__Sequence")
        .blacklist_type("rosidl_generator_c__octet__Sequence")
        .blacklist_type("rosidl_generator_c__uint8__Sequence")
        .blacklist_type("rosidl_generator_c__int8__Sequence")
        .blacklist_type("rosidl_generator_c__uint16__Sequence")
        .blacklist_type("rosidl_generator_c__int16__Sequence")
        .blacklist_type("rosidl_generator_c__uint32__Sequence")
        .blacklist_type("rosidl_generator_c__int32__Sequence")
        .blacklist_type("rosidl_generator_c__uint64__Sequence")
        .blacklist_type("rosidl_generator_c__int64__Sequence")
        .default_enum_style(bindgen::EnumVariation::Rust {
            non_exhaustive: false,
        });

    let ament_prefix_var_name = "AMENT_PREFIX_PATH";
    let ament_prefix_var = env::var(ament_prefix_var_name).expect("Source your ROS!");

    for ament_prefix_path in ament_prefix_var.split(":") {
        builder = builder.clang_arg(format!("-I{}/include", ament_prefix_path));
        println!("cargo:rustc-link-search=native={}/lib", ament_prefix_path);
    }

    let bindings = builder.generate().expect("Unable to generate bindings");

    bindings
        .write_to_file(out_path.join("msg_bindings.rs"))
        .expect("Couldn't write bindings!");
}
