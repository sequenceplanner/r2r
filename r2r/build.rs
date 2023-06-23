use quote::format_ident;
use quote::quote;
use rayon::prelude::*;
use std::fs::{File, OpenOptions};
use std::io::{self, prelude::*, BufWriter};
use std::path::{Path, PathBuf};
use std::{env, fmt, fs};

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

fn main() {
    r2r_common::print_cargo_watches();
    r2r_common::print_cargo_ros_distro();

    let env_hash = r2r_common::get_env_hash();
    let out_dir = PathBuf::from(env::var_os("OUT_DIR").unwrap());
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let bindgen_dir = out_dir.join(env_hash);
    let save_dir = manifest_dir.join("bindings");
    let mark_file = bindgen_dir.join("done");

    #[cfg(feature = "doc-only")]
    {
        // If "doc-only" feature is present, copy from $crate/bindings/* to OUT_DIR
        copy_files(&save_dir, &out_dir);
    }

    #[cfg(not(feature = "doc-only"))]
    {
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

#[cfg(not(feature = "doc-only"))]
fn generate_bindings(bindgen_dir: &Path) {
    fs::create_dir_all(bindgen_dir).unwrap();

    let msg_list = r2r_common::get_wanted_messages();
    let msgs = r2r_common::as_map(&msg_list);

    {
        let modules = {
            let mut modules: Vec<_> = msgs
                .par_iter()
                .map(|(module, _)| {
                    let path_suffix = format!("/{module}.rs");
                    let module_ident = format_ident!("{module}");
                    let tokens = quote! {
                        pub mod #module_ident {
                            include!(concat!(env!("OUT_DIR"), #path_suffix));
                        }
                    };
                    (module, unsafe { force_send(tokens) })
                })
                .collect();
            modules.par_sort_unstable_by_key(|(module, _)| *module);
            let modules = modules.into_iter().map(|(_, tokens)| tokens.unwrap());

            quote! { #(#modules)* }
        };

        let msgs_file = bindgen_dir.join(MSGS_FILENAME);
        write_to_file(&msgs_file, &modules).unwrap();
    }

    let mod_files: Vec<_> = msgs
        .par_iter()
        .map(|(module, prefixes)| {
            let snipplets: Vec<_> = prefixes
                .into_par_iter()
                .map(|(prefix, msgs)| {
                    let prefix_content = match *prefix {
                        "msg" => {
                            let msg_snipplets = msgs.into_iter().map(|msg| {
                                println!("cargo:rustc-cfg=r2r__{}__{}__{}", module, prefix, msg);
                                r2r_msg_gen::generate_rust_msg(module, prefix, msg)
                            });

                            quote! {
                                use super::super::*;
                                #(#msg_snipplets)*
                            }
                        }
                        "srv" => {
                            let msg_snipplets = msgs.into_iter().map(|msg| {
                                let service_snipplet =
                                    r2r_msg_gen::generate_rust_service(module, prefix, msg);
                                let msg_snipplets = ["Request", "Response"].iter().map(|s| {
                                    let msgname = format!("{}_{}", msg, s);
                                    println!(
                                        "cargo:rustc-cfg=r2r__{}__{}__{}",
                                        module, prefix, msg
                                    );
                                    r2r_msg_gen::generate_rust_msg(module, prefix, &msgname)
                                });
                                let msg = format_ident!("{msg}");

                                quote! {
                                    #[allow(non_snake_case)]
                                    pub mod #msg {
                                        use super::super::super::*;

                                        #service_snipplet
                                        #(#msg_snipplets)*
                                    }
                                }
                            });
                            quote! {
                                #(#msg_snipplets)*
                            }
                        }
                        "action" => {
                            let msg_snipplets = msgs.into_iter().map(|msg| {
                                let action_snipplet =
                                    r2r_msg_gen::generate_rust_action(module, prefix, msg);

                                let msg_snipplets =
                                    ["Goal", "Result", "Feedback"].iter().map(|s| {
                                        let msgname = format!("{}_{}", msg, s);
                                        println!(
                                            "cargo:rustc-cfg=r2r__{}__{}__{}",
                                            module, prefix, msg
                                        );
                                        r2r_msg_gen::generate_rust_msg(module, prefix, &msgname)
                                    });

                                // "internal" services that implements the action type
                                let service_snipplets =
                                    ["SendGoal", "GetResult"].iter().map(|srv| {
                                        let srvname = format!("{}_{}", msg, srv);
                                        let service_snipplet = r2r_msg_gen::generate_rust_service(
                                            module, prefix, &srvname,
                                        );

                                        let msg_snipplets =
                                            ["Request", "Response"].iter().map(|s| {
                                                let msgname = format!("{}_{}_{}", msg, srv, s);
                                                r2r_msg_gen::generate_rust_msg(
                                                    module, prefix, &msgname,
                                                )
                                            });

                                        let srv = format_ident!("{srv}");
                                        quote! {
                                            #[allow(non_snake_case)]
                                            pub mod #srv {
                                                use super::super::super::super::*;

                                                #service_snipplet
                                                #(#msg_snipplets)*
                                            }
                                        }
                                    });

                                // also "internal" feedback message type that wraps the feedback type with a uuid
                                let feedback_msgname = format!("{}_FeedbackMessage", msg);
                                let feedback_msg_snipplet = r2r_msg_gen::generate_rust_msg(
                                    module,
                                    prefix,
                                    &feedback_msgname,
                                );

                                let msg = format_ident!("{msg}");
                                quote! {
                                    #[allow(non_snake_case)]
                                    pub mod #msg {
                                        use super::super::super::*;

                                        #action_snipplet
                                        #(#msg_snipplets)*
                                        #(#service_snipplets)*
                                        #feedback_msg_snipplet
                                    }
                                }
                            });

                            quote! {
                                #(#msg_snipplets)*
                            }
                        }
                        _ => {
                            panic!("unknown prefix type: {}", prefix);
                        }
                    };

                    let prefix = format_ident!("{prefix}");

                    let mod_content = quote! {
                        pub mod #prefix {
                            #prefix_content
                        }
                    };

                    unsafe { force_send(mod_content) }
                })
                .collect();

            let snipplets = snipplets.into_iter().map(|snipplet| snipplet.unwrap());
            let mod_content = quote! { #(#snipplets)* };
            let file_name = format!("{}.rs", module);
            let mod_file = bindgen_dir.join(&file_name);
            write_to_file(&mod_file, &mod_content).unwrap();

            file_name
        })
        .collect();

    // Write helper files
    {
        let untyped_helper = r2r_msg_gen::generate_untyped_helper(&msg_list);
        let untyped_file = bindgen_dir.join(UNTYPED_FILENAME);
        write_to_file(&untyped_file, &untyped_helper).unwrap();
    }

    {
        let untyped_service_helper = r2r_msg_gen::generate_untyped_service_helper(&msg_list);
        let untyped_service_file = bindgen_dir.join(UNTYPED_SERVICE_FILENAME);
        write_to_file(&untyped_service_file, &untyped_service_helper).unwrap();
    }

    {
        let untyped_action_helper = r2r_msg_gen::generate_untyped_action_helper(&msg_list);
        let untyped_action_file = bindgen_dir.join(UNTYPED_ACTION_FILENAME);
        write_to_file(&untyped_action_file, &untyped_action_helper).unwrap();
    }

    // Save file list
    {
        let list_file = bindgen_dir.join(LIST_FILENAME);
        let mut writer = BufWriter::new(File::create(list_file).unwrap());

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

fn write_to_file(path: &Path, content: impl fmt::Display) -> io::Result<()> {
    let mut writer = BufWriter::new(File::create(path)?);
    write!(writer, "{}", content)?;
    writer.flush()?;
    Ok(())
}

unsafe fn force_send<T>(value: T) -> force_send_sync::Send<T> {
    force_send_sync::Send::new(value)
}
