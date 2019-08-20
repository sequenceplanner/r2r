extern crate bindgen;

use std::env;
use std::path::PathBuf;

use common::*;

fn main() {
    println!("cargo:rerun-if-changed=../");

    let msgs = read_file("../msgs.txt").unwrap();
    let msg_list = parse_msgs(&msgs);
    let msg_map = as_map(&msg_list);

    for module in msg_map.keys() {
        println!("cargo:rustc-link-lib=dylib={}__rosidl_typesupport_c", module);
        println!("cargo:rustc-link-lib=dylib={}__rosidl_typesupport_introspection_c", module);
        println!("cargo:rustc-link-lib=dylib={}__rosidl_generator_c", module);
    }

    let mut includes = String::new();
    let mut introspecion_map = String::from("\
         lazy_static! { \n
           static ref INTROSPECTION_FNS: HashMap<&'static str, usize> = {\n
             let mut m = HashMap::new();\n");

    for msg in msg_list {

        // I *think* these are always just lowercase
        let module = msg.module.to_lowercase();
        let prefix = msg.prefix.to_lowercase();
        let name = msg.name.to_lowercase();

        // filename is certainly CamelCase -> snake_case. convert
        use heck::SnakeCase;
        let include_filename = msg.name.to_snake_case();

        includes.push_str(&format!("#include <{}/{}/{}.h>\n", &module, &prefix, &include_filename));
        includes.push_str(&format!("#include <{}/{}/{}__rosidl_typesupport_introspection_c.h>\n", &module, &prefix, &include_filename));

        let key = &format!("{}__{}__{}", module, prefix, name);
        let val = &format!("unsafe {{ rosidl_typesupport_introspection_c__get_message_type_support_handle__{}__{}__{}() }} as *const i32 as usize", &msg.module, &msg.prefix, &msg.name);
        introspecion_map.push_str(&format!("m.insert(\"{}\", {});\n",key,val));
    }
    introspecion_map.push_str("m \n }; }\n\n");

    use std::io::Write;
    use std::fs::File;
    let mut f = File::create("src/msg_includes.h").unwrap();
    write!(f, "{}", includes).unwrap();

    let mut f = File::create("src/introspection_functions.rs").unwrap();
    write!(f, "{}", introspecion_map).unwrap();

    let headers_enabled = env::var_os("CARGO_FEATURE_HEADERS").is_some();
    let mut builder = bindgen::Builder::default()
        .header("src/msg_includes.h")
        .derive_copy(false)
        // blacklist types that are handled by rcl bindings
        .blacklist_type("rosidl_message_type_support_t")
        .blacklist_type("rosidl_generator_c__String")
        .blacklist_type("rosidl_generator_c__double__Sequence") // etc...
        .default_enum_style(bindgen::EnumVariation::Rust { non_exhaustive: false } );

    let ament_prefix_var_name = "AMENT_PREFIX_PATH";
    let ament_prefix_var = env::var(ament_prefix_var_name).expect("Source your ROS!");

    for ament_prefix_path in ament_prefix_var.split(":") {
        builder = builder.clang_arg(format!("-I{}/include", ament_prefix_path));
        println!("cargo:rustc-link-search=native={}/lib", ament_prefix_path);
    }

    // bindgen takes time so we dont want to do it always... must be a better way
    if headers_enabled {
        let bindings = builder.generate().expect("Unable to generate bindings");

        let out_path = PathBuf::from(".");
        bindings
            .write_to_file(out_path.join("src/msg_bindings.rs"))
            .expect("Couldn't write bindings!");
    }

//    assert!(false);
}
