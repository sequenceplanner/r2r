use std::fs::{File, OpenOptions};
use std::io::Write;
use std::path::{Path, PathBuf};
use std::{env, fs};

const LIST_FILENAME: &str = "files.txt";
const MSGS_FILENAME: &str = "_r2r_generated_msgs.rs";
const UNTYPED_FILENAME: &str = "_r2r_generated_untyped_helper.rs";
const UNTYPED_SERVICE_FILENAME: &str = "_r2r_generated_service_helper.rs";
const UNTYPED_ACTION_FILENAME: &str = "_r2r_generated_action_helper.rs";
const GENERATED_FILES: &[&str] = &[
    MSGS_FILENAME,
    UNTYPED_FILENAME,
    UNTYPED_SERVICE_FILENAME,
    UNTYPED_ACTION_FILENAME,
];

fn pretty_tokenstream(stream: proc_macro2::TokenStream) -> String {
    prettyplease::unparse(&syn::parse2::<syn::File>(stream).unwrap())
}

fn main() {
    r2r_common::print_cargo_watches();
    r2r_common::print_cargo_ros_distro();

    let env_hash = r2r_common::get_env_hash();
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let bindgen_dir = out_dir.join(env_hash);
    let save_dir = manifest_dir.join("bindings");
    let mark_file = bindgen_dir.join("done");

    if cfg!(feature = "doc-only") {
        // If "doc-only" feature is present, copy from $crate/bindings/* to OUT_DIR
        copy_files(&save_dir, &out_dir);
    } else {
        // If bindgen was done before, use cached files.
        if !mark_file.exists() {
            eprintln!("Generate bindings in '{}'", bindgen_dir.display());
            generate_bindings(&bindgen_dir);
            touch(&mark_file);
        } else {
            eprintln!("Used cached files in '{}'", bindgen_dir.display());
        }

        copy_files(&bindgen_dir, &out_dir);

        #[cfg(feature = "save-bindgen")]
        {
            fs::create_dir_all(&save_dir).unwrap();
            copy_files(&bindgen_dir, &save_dir);
        }
    }
}

fn generate_bindings(bindgen_dir: &Path) {
    r2r_msg_gen::assert_field_type_match_c_enum();
    fs::create_dir_all(&bindgen_dir).unwrap();

    let msg_list = r2r_common::get_wanted_messages();
    let msgs = r2r_common::as_map(&msg_list);

    let mut modules = String::new();
    let mod_files: Vec<_> = msgs
        .iter()
        .map(|(module, prefixes)| {
            let mod_text = format!(
                r#"pub mod {module}{{include!(concat!(env!("OUT_DIR"), "/{module}.rs"));}}{lf}"#,
                module = module,
                lf = "\n"
            );
            modules.push_str(&mod_text);

            let mut codegen = String::new();

            prefixes.into_iter().for_each(|(prefix, msgs)| {
                codegen.push_str(&format!("  pub mod {} {{\n", prefix));

                if prefix == &"action" {
                    for msg in msgs {
                        codegen.push_str("#[allow(non_snake_case)]\n");
                        codegen.push_str(&format!("    pub mod {} {{\n", msg));
                        codegen.push_str("    use super::super::super::*;\n");

                        codegen.push_str(&pretty_tokenstream(r2r_msg_gen::generate_rust_action(
                            module, prefix, msg,
                        )));

                        for s in &["Goal", "Result", "Feedback"] {
                            let msgname = format!("{}_{}", msg, s);
                            codegen.push_str(&pretty_tokenstream(r2r_msg_gen::generate_rust_msg(
                                module, prefix, &msgname,
                            )));
                            println!("cargo:rustc-cfg=r2r__{}__{}__{}", module, prefix, msg);
                        }

                        // "internal" services that implements the action type
                        for srv in &["SendGoal", "GetResult"] {
                            codegen.push_str("#[allow(non_snake_case)]\n");
                            codegen.push_str(&format!("    pub mod {} {{\n", srv));
                            codegen.push_str("    use super::super::super::super::*;\n");

                            let srvname = format!("{}_{}", msg, srv);
                            codegen.push_str(&pretty_tokenstream(
                                r2r_msg_gen::generate_rust_service(module, prefix, &srvname),
                            ));

                            for s in &["Request", "Response"] {
                                let msgname = format!("{}_{}_{}", msg, srv, s);
                                codegen.push_str(&pretty_tokenstream(
                                    r2r_msg_gen::generate_rust_msg(module, prefix, &msgname),
                                ));
                            }
                            codegen.push_str("    }\n");
                        }

                        // also "internal" feedback message type that wraps the feedback type with a uuid
                        let feedback_msgname = format!("{}_FeedbackMessage", msg);
                        codegen.push_str(&pretty_tokenstream(r2r_msg_gen::generate_rust_msg(
                            module,
                            prefix,
                            &feedback_msgname,
                        )));

                        codegen.push_str("    }\n");
                    }
                } else if prefix == &"srv" {
                    for msg in msgs {
                        codegen.push_str("#[allow(non_snake_case)]\n");
                        codegen.push_str(&format!("    pub mod {} {{\n", msg));
                        codegen.push_str("    use super::super::super::*;\n");

                        codegen.push_str(&pretty_tokenstream(r2r_msg_gen::generate_rust_service(
                            module, prefix, msg,
                        )));

                        for s in &["Request", "Response"] {
                            let msgname = format!("{}_{}", msg, s);
                            codegen.push_str(&pretty_tokenstream(r2r_msg_gen::generate_rust_msg(
                                module, prefix, &msgname,
                            )));
                            println!("cargo:rustc-cfg=r2r__{}__{}__{}", module, prefix, msg);
                        }
                        codegen.push_str("    }\n");
                    }
                } else if prefix == &"msg" {
                    codegen.push_str("    use super::super::*;\n");
                    for msg in msgs {
                        codegen.push_str(&pretty_tokenstream(r2r_msg_gen::generate_rust_msg(
                            module, prefix, msg,
                        )));
                        println!("cargo:rustc-cfg=r2r__{}__{}__{}", module, prefix, msg);
                    }
                } else {
                    panic!("unknown prefix type: {}", prefix);
                }

                codegen.push_str("  }\n");
            });

            let file_name = format!("{}.rs", module);
            let mod_file = bindgen_dir.join(&file_name);
            fs::write(&mod_file, codegen).unwrap();

            file_name
        })
        .collect();

    // Write helper files
    {
        let untyped_helper = pretty_tokenstream(r2r_msg_gen::generate_untyped_helper(&msg_list));
        let untyped_service_helper =
            pretty_tokenstream(r2r_msg_gen::generate_untyped_service_helper(&msg_list));
        let untyped_action_helper =
            pretty_tokenstream(r2r_msg_gen::generate_untyped_action_helper(&msg_list));

        let msgs_file = bindgen_dir.join(MSGS_FILENAME);
        let untyped_file = bindgen_dir.join(UNTYPED_FILENAME);
        let untyped_service_file = bindgen_dir.join(UNTYPED_SERVICE_FILENAME);
        let untyped_action_file = bindgen_dir.join(UNTYPED_ACTION_FILENAME);

        fs::write(&msgs_file, &modules).unwrap();
        fs::write(&untyped_file, &untyped_helper).unwrap();
        fs::write(&untyped_service_file, &untyped_service_helper).unwrap();
        fs::write(&untyped_action_file, &untyped_action_helper).unwrap();
    }

    // Save file list
    {
        let list_file = bindgen_dir.join(LIST_FILENAME);
        let mut writer = File::create(list_file).unwrap();

        for file_name in mod_files {
            writeln!(writer, "{}", file_name).unwrap();
        }

        for file_name in GENERATED_FILES {
            writeln!(writer, "{}", file_name).unwrap();
        }
    }
}

fn copy_files(src_dir: &Path, tgt_dir: &Path) {
    eprintln!(
        "Copy files from '{}' to '{}'",
        src_dir.display(),
        tgt_dir.display()
    );

    let src_list_file = src_dir.join(LIST_FILENAME);
    let tgt_list_file = tgt_dir.join(LIST_FILENAME);
    fs::read_to_string(&src_list_file)
        .unwrap()
        .lines()
        .for_each(|file_name| {
            let src_file = src_dir.join(file_name);
            let tgt_file = tgt_dir.join(file_name);
            fs::copy(&src_file, &tgt_file).unwrap();
        });

    fs::copy(&src_list_file, &tgt_list_file).unwrap();
}

fn touch(path: &Path) {
    OpenOptions::new()
        .create(true)
        .write(true)
        .open(path)
        .unwrap_or_else(|_| panic!("Unable to create file '{}'", path.display()));
}
