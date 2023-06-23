use bindgen::Bindings;
use heck::ToSnakeCase;
use itertools::chain;
use itertools::iproduct;
use quote::format_ident;
use quote::quote;
use r2r_common::RosMsg;
use rayon::prelude::*;
use std::fs::File;
use std::fs::OpenOptions;
use std::io::prelude::*;
use std::io::BufWriter;
use std::mem;
use std::path::{Path, PathBuf};
use std::{env, fs};

const MSG_INCLUDES_FILENAME: &str = "msg_includes.h";
const INTROSPECTION_FILENAME: &str = "introspection_functions.rs";
const CONSTANTS_FILENAME: &str = "constants.rs";
const BINDINGS_FILENAME: &str = "msg_bindings.rs";
const GENERATED_FILES: &[&str] = &[
    MSG_INCLUDES_FILENAME,
    INTROSPECTION_FILENAME,
    CONSTANTS_FILENAME,
    BINDINGS_FILENAME,
];
const SRV_SUFFICES: &[&str] = &["Request", "Response"];
const ACTION_SUFFICES: &[&str] = &["Goal", "Result", "Feedback", "FeedbackMessage"];

fn main() {
    r2r_common::print_cargo_watches();
    r2r_common::print_cargo_ros_distro();

    let msg_list = r2r_common::get_wanted_messages();
    run_bindgen(&msg_list);
    run_dynlink(&msg_list);
}

fn run_bindgen(msg_list: &[RosMsg]) {
    let env_hash = r2r_common::get_env_hash();
    let out_dir = PathBuf::from(env::var("OUT_DIR").unwrap());
    let manifest_dir = Path::new(env!("CARGO_MANIFEST_DIR"));
    let bindgen_dir = out_dir.join(env_hash);
    let mark_file = bindgen_dir.join("done");
    let save_dir = manifest_dir.join("bindings");

    if cfg!(feature = "doc-only") {
        // If "doc-only" feature is present, copy from $crate/bindings/* to OUT_DIR
        eprintln!(
            "Copy files from '{}' to '{}'",
            save_dir.display(),
            out_dir.display()
        );

        for filename in GENERATED_FILES {
            let src = save_dir.join(filename);
            let tgt = out_dir.join(filename);
            fs::copy(&src, &tgt).unwrap();
        }
    } else {
        // If bindgen was done before, use cached files.
        if !mark_file.exists() {
            eprintln!("Generate bindings in '{}'", bindgen_dir.display());
            fs::create_dir_all(&bindgen_dir).unwrap();
            generate_bindings(&bindgen_dir, msg_list);
            touch(&mark_file);
        } else {
            eprintln!("Used cached files in '{}'", bindgen_dir.display());
        }

        for filename in GENERATED_FILES {
            let src = bindgen_dir.join(filename);
            let tgt = out_dir.join(filename);
            fs::copy(&src, &tgt).unwrap();
        }

        #[cfg(feature = "save-bindgen")]
        {
            fs::create_dir_all(&save_dir).unwrap();

            for filename in GENERATED_FILES {
                let src = bindgen_dir.join(filename);
                let tgt = save_dir.join(filename);
                fs::copy(&src, &tgt).unwrap();
            }
        }
    }
}

fn generate_bindings(bindgen_dir: &Path, msg_list: &[RosMsg]) {
    // Run codegen in parallel.
    rayon::scope(|scope| {
        scope.spawn(|_| {
            let bindings = generate_bindings_file(bindgen_dir);
            generate_constants(bindgen_dir, msg_list, &bindings);
        });
        scope.spawn(|_| {
            generate_includes(bindgen_dir, msg_list);
        });
        scope.spawn(|_| {
            generate_introspecion_map(bindgen_dir, msg_list);
        });
    });
}

fn generate_includes(bindgen_dir: &Path, msg_list: &[RosMsg]) {
    let msg_includes_file = bindgen_dir.join(MSG_INCLUDES_FILENAME);

    // Generate a C include line for each message type.
    let mut include_lines: Vec<_> = msg_list
        .par_iter()
        .flat_map(|msg| {
            let RosMsg {
                name,
                module,
                prefix,
                ..
            } = msg;

            // filename is certainly CamelCase -> snake_case. convert
            let include_filename = name.to_snake_case();

            [
                format!("#include <{module}/{prefix}/{include_filename}.h>"),
                format!(
                    "#include <{module}/{prefix}/detail/\
                     {include_filename}__rosidl_typesupport_introspection_c.h>"
                ),
            ]
        })
        .collect();

    // Sort the lines.
    include_lines.par_sort();

    // Write the file content
    let mut writer = BufWriter::new(File::create(&msg_includes_file).unwrap());
    for line in include_lines {
        writeln!(writer, "{line}").unwrap();
    }
}

fn generate_introspecion_map(bindgen_dir: &Path, msg_list: &[RosMsg]) {
    let introspection_file = bindgen_dir.join(INTROSPECTION_FILENAME);

    let mut entries: Vec<_> = msg_list
        .par_iter()
        .flat_map(|msg| {
            let RosMsg {
                module,
                prefix,
                name,
            } = msg;

            match prefix.as_str() {
                "srv" => SRV_SUFFICES
                    .iter()
                    .map(|s| {
                        let key = format!("{module}__{prefix}__{name}_{s}");
                        let ident = format!(
                            "rosidl_typesupport_introspection_c__get_message_type_support_handle__\
                                 {module}__\
                                 {prefix}__\
                                 {name}_\
                                 {s}"
                        );
                        (key, ident)
                    })
                    .map(|(key, ident)| (key, ident))
                    .collect(),
                "action" => {
                    let iter1 = ACTION_SUFFICES.iter().map(|s| {
                        let key = format!("{module}__{prefix}__{name}_{s}");
                        let ident = format!(
                            "rosidl_typesupport_introspection_c__\
                             get_message_type_support_handle__\
                                 {module}__\
                                 {prefix}__\
                                 {name}_\
                                 {s}",
                        );
                        (key, ident)
                    });

                    // "internal" services
                    let iter2 =
                        iproduct!(["SendGoal", "GetResult"], SRV_SUFFICES).map(move |(srv, s)| {
                            // TODO: refactor this is copy paste from services...
                            let msgname = format!("{name}_{srv}_{s}");
                            let key = format!("{module}__{prefix}__{msgname}");
                            let ident = format!(
                                "rosidl_typesupport_introspection_c__\
                                 get_message_type_support_handle__\
                                 {module}__\
                                 {prefix}__\
                                 {msgname}"
                            );
                            (key, ident)
                        });

                    chain!(iter1, iter2)
                        .map(|(key, ident)| (key, ident))
                        .collect()
                }
                "msg" => {
                    let key = format!("{module}__{prefix}__{name}");
                    let ident = format!(
                        "rosidl_typesupport_introspection_c__\
                         get_message_type_support_handle__\
                         {module}__\
                         {prefix}__\
                         {name}"
                    );
                    vec![(key, ident)]
                }
                _ => unreachable!(),
            }
        })
        .map(|(key, func_str)| {
            // Generate a hashmap entry
            let func_ident = format_ident!("{func_str}");
            let tokens = quote! {
                #key =>
                  #func_ident
                  as unsafe extern "C" fn() -> *const rosidl_message_type_support_t
            };

            // force_send to workaround !Send
            (key, unsafe { force_send(tokens) })
        })
        .collect();

    // Sort the entries by key
    entries.par_sort_by_cached_key(|(key, _)| key.to_string());

    let entries = entries.into_iter().map(|(_, tokens)| tokens.unwrap());

    // Write the file content
    let introspecion_map = quote! {
        type IntrospectionFn = unsafe extern "C" fn() -> *const rosidl_message_type_support_t;

        static INTROSPECTION_FNS: phf::Map<&'static str, IntrospectionFn> = phf::phf_map! {
            #(#entries),*
        };
    };

    let mut writer = BufWriter::new(File::create(introspection_file).unwrap());
    writeln!(&mut writer, "{}", introspecion_map).unwrap();
}

fn generate_bindings_file(bindgen_dir: &Path) -> Bindings {
    let msg_includes_file = bindgen_dir.join(MSG_INCLUDES_FILENAME);
    let bindings_file = bindgen_dir.join(BINDINGS_FILENAME);

    let builder = r2r_common::setup_bindgen_builder()
        .header(msg_includes_file.to_str().unwrap())
        .derive_copy(false)
        .allowlist_function("rosidl_typesupport_c__.*")
        .allowlist_function("rosidl_typesupport_introspection_c__.*")
        .allowlist_function(r"[\w_]*__(msg|srv|action)__[\w_]*__(create|destroy)")
        .allowlist_function(r"[\w_]*__(msg|srv|action)__[\w_]*__Sequence__(init|fini)")
        .allowlist_var(r"[\w_]*__(msg|srv|action)__[\w_]*__[\w_]*")
        // blacklist types that are handled by rcl bindings
        .blocklist_type("rosidl_message_type_support_t")
        .blocklist_type("rosidl_service_type_support_t")
        .blocklist_type("rosidl_action_type_support_t")
        .blocklist_type("rosidl_runtime_c__String")
        .blocklist_type("rosidl_runtime_c__String__Sequence")
        .blocklist_type("rosidl_runtime_c__U16String")
        .blocklist_type("rosidl_runtime_c__U16String__Sequence")
        .blocklist_type("rosidl_runtime_c__float32__Sequence")
        .blocklist_type("rosidl_runtime_c__float__Sequence")
        .blocklist_type("rosidl_runtime_c__float64__Sequence")
        .blocklist_type("rosidl_runtime_c__double__Sequence")
        .blocklist_type("rosidl_runtime_c__long_double__Sequence")
        .blocklist_type("rosidl_runtime_c__char__Sequence")
        .blocklist_type("rosidl_runtime_c__wchar__Sequence")
        .blocklist_type("rosidl_runtime_c__boolean__Sequence")
        .blocklist_type("rosidl_runtime_c__octet__Sequence")
        .blocklist_type("rosidl_runtime_c__uint8__Sequence")
        .blocklist_type("rosidl_runtime_c__int8__Sequence")
        .blocklist_type("rosidl_runtime_c__uint16__Sequence")
        .blocklist_type("rosidl_runtime_c__int16__Sequence")
        .blocklist_type("rosidl_runtime_c__uint32__Sequence")
        .blocklist_type("rosidl_runtime_c__int32__Sequence")
        .blocklist_type("rosidl_runtime_c__uint64__Sequence")
        .blocklist_type("rosidl_runtime_c__int64__Sequence")
        .size_t_is_usize(true)
        .no_debug("_OSUnaligned.*")
        .generate_comments(false)
        .default_enum_style(bindgen::EnumVariation::Rust {
            non_exhaustive: false,
        });

    let bindings = builder.generate().expect("Unable to generate bindings");
    bindings
        .write_to_file(bindings_file)
        .expect("Couldn't write bindings!");

    bindings
}

fn generate_constants(bindgen_dir: &Path, msg_list: &[RosMsg], bindings: &Bindings) {
    let constants_file = bindgen_dir.join(CONSTANTS_FILENAME);

    // Turn the source string into tokens.
    let tokens: syn::File =
        syn::parse_str(&bindings.to_string()).expect("Unable to parse generated bindings");

    // Workaround !Send
    let items: &[force_send_sync::SendSync<syn::Item>] =
        unsafe { mem::transmute(tokens.items.as_slice()) };

    /// The key is used to index constant items.
    #[derive(Debug, PartialEq, Eq, PartialOrd, Ord)]
    struct Key {
        pub module: String,
        pub prefix: String,
        pub name: String,

        /// Suffix is None if prefix is "msg". Otherwise, it's not None.
        pub suffix: Option<String>,
    }

    // find all lines which look suspiciosly like a constant.
    let mut constants: Vec<_> = items
        .par_iter()
        .filter_map(|item| {
            // Filter out non-const items.
            let syn::Item::Const(item) = &**item else {
                return None;
            };

            // Filter out constants ending with "__MAX_SIZE" or "__MAX_STRING_SIZE".
            let ident = item.ident.to_string();
            if ident.ends_with("__MAX_SIZE") || ident.ends_with("__MAX_STRING_SIZE") {
                return None;
            }

            // Create the key for the constant.
            let (key, const_name) = {
                let (module, remain) = ident.split_once("__")?;
                let (prefix, remain) = remain.split_once("__")?;
                let (name_and_suffix, const_name) = remain.split_once("__")?;
                let (name, suffix) = match name_and_suffix.rsplit_once('_') {
                    Some((name, suffix)) => (name, Some(suffix.to_string())),
                    None => (name_and_suffix, None),
                };

                if let Some(suffix) = &suffix {
                    if !SRV_SUFFICES.contains(&suffix.as_str())
                        && !ACTION_SUFFICES.contains(&suffix.as_str())
                    {
                        return None;
                    }
                }

                let key = Key {
                    module: module.to_string(),
                    prefix: prefix.to_string(),
                    name: name.to_string(),
                    suffix,
                };

                (key, const_name)
            };

            // Generate the entry for the constant.
            let typ = &item.ty;
            let entry = (const_name.to_string(), quote! { #typ }.to_string());
            Some((key, entry))
        })
        .collect();

    // Sort the constants to enable later binary range search.
    constants.par_sort_unstable();

    let mut entries: Vec<_> = msg_list
        .par_iter()
        .flat_map(|msg| {
            // Generate a key for each message type.

            let RosMsg {
                module,
                prefix,
                name,
            } = msg;

            match prefix.as_str() {
                "msg" => vec![Key {
                    module: module.to_string(),
                    prefix: prefix.to_string(),
                    name: name.to_string(),
                    suffix: None,
                }],
                "srv" => SRV_SUFFICES
                    .iter()
                    .map(|suffix| Key {
                        module: module.to_string(),
                        prefix: prefix.to_string(),
                        name: name.to_string(),
                        suffix: Some(suffix.to_string()),
                    })
                    .collect(),
                "action" => ACTION_SUFFICES
                    .iter()
                    .map(|suffix| Key {
                        module: module.to_string(),
                        prefix: prefix.to_string(),
                        name: name.to_string(),
                        suffix: Some(suffix.to_string()),
                    })
                    .collect(),
                _ => unreachable!(),
            }
        })
        .filter_map(|key| {
            // Search for items with the same key using binary searches.
            let range = {
                let idx = constants.partition_point(|(other, _)| other < &key);
                let len = constants
                    .get(idx..)?
                    .partition_point(|(other, _)| other == &key);
                if len == 0 {
                    return None;
                }
                idx..(idx + len)
            };

            let Key {
                module,
                prefix,
                name,
                suffix,
            } = key;
            let msg = match suffix {
                Some(suffix) => format!("{module}__{prefix}__{name}_{suffix}"),
                None => format!("{module}__{prefix}__{name}"),
            };

            Some((msg, &constants[range]))
        })
        .map(|(msg, msg_constants)| {
            // Generate map entries.
            let consts = msg_constants
                .iter()
                .map(|(_, (const_name, typ))| quote! { (#const_name, #typ) });
            let entry = quote! { #msg => &[ #(#consts),* ] };

            // Workaround !Send
            (msg, unsafe { force_send(entry) })
        })
        .collect();

    // Sort entries by message name
    entries.par_sort_by_cached_key(|(msg, _)| msg.to_string());
    let entries = entries.into_iter().map(|(_, tokens)| tokens.unwrap());

    // Write the file content.
    let constants_map = quote! {
        static CONSTANTS_MAP: phf::Map<&'static str, &[(&str, &str)]> = phf::phf_map! {
            #(#entries),*
        };
    };

    let mut writer = BufWriter::new(File::create(constants_file).unwrap());
    writeln!(&mut writer, "{}", constants_map).unwrap();
}

fn run_dynlink(#[allow(unused_variables)] msg_list: &[RosMsg]) {
    r2r_common::print_cargo_link_search();

    let msg_map = r2r_common::as_map(msg_list);
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
}

fn touch(path: &Path) {
    OpenOptions::new()
        .create(true)
        .write(true)
        .open(path)
        .unwrap_or_else(|_| panic!("Unable to create file '{}'", path.display()));
}

unsafe fn force_send<T>(value: T) -> force_send_sync::Send<T> {
    force_send_sync::Send::new(value)
}
