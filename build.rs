use common::*;
use msg_gen::*;
use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    println!("cargo:rerun-if-env-changed=AMENT_PREFIX_PATH");

    let msgs = get_all_ros_msgs();
    let msgs_list = parse_msgs(&msgs);

    let msgs = as_map(&msgs_list);

    let mut modules = String::new();

    for (module, prefixes) in &msgs {
        println!(
            "cargo:rustc-link-lib=dylib={}__rosidl_typesupport_c",
            module
        );
        println!(
            "cargo:rustc-link-lib=dylib={}__rosidl_typesupport_introspection_c",
            module
        );
        println!("cargo:rustc-link-lib=dylib={}__rosidl_generator_c", module);

        modules.push_str(&format!(r#"pub mod {module}{{include!(concat!(env!("OUT_DIR"), "/{module}.rs"));}}{lf}"#, module=module, lf="\n"));

        let mut codegen = String::new();
        for (prefix, msgs) in prefixes {
            codegen.push_str(&format!("  pub mod {} {{\n", prefix));

            for msg in msgs {
                if prefix == &"srv" {
                    codegen.push_str("#[allow(non_snake_case)]\n");
                    codegen.push_str(&format!("    pub mod {} {{\n", msg));
                    for s in &["Request", "Response"] {
                        let msgname = format!("{}_{}", msg, s);
                        codegen.push_str("#[allow(unused_imports)]\n");
                        codegen.push_str("    use super::super::super::*;\n");
                        codegen.push_str(&generate_rust_msg(module, prefix, &msgname));
                        println!("cargo:rustc-cfg=r2r__{}__{}__{}", module, prefix, msg);
                    }
                    codegen.push_str("    }\n");
                } else {
                    // the need to allow unused seems to be a compiler bug...
                    codegen.push_str("#[allow(unused_imports)]\n");
                    codegen.push_str("    use super::super::*;\n");
                    codegen.push_str(&generate_rust_msg(module, prefix, msg));
                    println!("cargo:rustc-cfg=r2r__{}__{}__{}", module, prefix, msg);
                }
            }

            codegen.push_str("  }\n");
        }
        let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
        let mod_fn = out_path.join(&format!("{}.rs", module));
        let mut f = File::create(mod_fn).unwrap();
        write!(f, "{}", codegen).unwrap();
    }

    let untyped_helper = generate_untyped_helper(&msgs_list);

    let out_path = PathBuf::from(env::var("OUT_DIR").unwrap());
    let msgs_fn = out_path.join("_r2r_generated_msgs.rs");
    let untyped_fn = out_path.join("_r2r_generated_untyped_helper.rs");

    let mut f = File::create(msgs_fn).unwrap();
    write!(f, "{}", modules).unwrap();
    let mut f = File::create(untyped_fn).unwrap();
    write!(f, "{}", untyped_helper).unwrap();
}
