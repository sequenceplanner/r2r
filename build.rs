use msg_gen::*;
use common::*;

fn main() {
    let msgs = read_file("./msgs.txt").unwrap();
    let msgs = parse_msgs(&msgs);
    let msgs = as_map(&msgs);

    let mut codegen = String::new();

    for (module, prefixes) in &msgs {
        println!("cargo:rustc-link-lib=dylib={}__rosidl_typesupport_c", module);
        println!("cargo:rustc-link-lib=dylib={}__rosidl_typesupport_introspection_c", module);
        println!("cargo:rustc-link-lib=dylib={}__rosidl_generator_c", module);

        codegen.push_str(&format!("pub mod {} {{\n", module));

        for (prefix, msgs) in prefixes {
            codegen.push_str(&format!("  pub mod {} {{\n", prefix));
            codegen.push_str("    use super::super::*;\n");

            for msg in msgs {
                let module_lc = String::from(*module).to_lowercase();
                let prefix_lc = String::from(*prefix).to_lowercase();
                let name_lc = String::from(*msg).to_lowercase();
                codegen.push_str(&generate_rust_msg(&module_lc, &prefix_lc, &name_lc));
            }

            codegen.push_str("  }\n");

        }

        codegen.push_str("}\n");
    }

    use std::io::Write;
    use std::fs::File;
    let mut f = File::create("src/generated_msgs.rs").unwrap();
    write!(f, "{}", codegen).unwrap();
}
