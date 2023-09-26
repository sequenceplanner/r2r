use proc_macro2::TokenStream;
use quote::{quote, quote_spanned};
use syn::spanned::Spanned;
use syn::{parse_macro_input, Data, DeriveInput, Fields};

extern crate proc_macro;

/// Derives RosParams trait for a structure to use it with
/// `r2r::Node::make_derived_parameter_handler()`.
#[proc_macro_derive(RosParams)]
pub fn derive_r2r_params(input: proc_macro::TokenStream) -> proc_macro::TokenStream {
    // Parse the input tokens into a syntax tree.
    let input = parse_macro_input!(input as DeriveInput);

    // Used in the quasi-quotation below as `#name`.
    let name = input.ident;

    let register_calls = get_register_calls(&input.data);
    let get_param_matches = param_matches_for(quote!(get_parameter(suffix)), &input.data);
    let set_param_matches =
        param_matches_for(quote!(set_parameter(suffix, param_val)), &input.data);

    let expanded = quote! {
        // The generated impl.
        impl ::r2r::RosParams for #name {
            fn register_parameters(
                &mut self,
                prefix: &str,
                params: &mut ::std::collections::hash_map::HashMap<String, ::r2r::ParameterValue>,
            ) -> ::r2r::Result<()> {
                let prefix = if prefix.is_empty() {
                    String::from("")
                } else {
                    format!("{prefix}.")
                };
                #register_calls
                Ok(())
            }
            fn get_parameter(&mut self, param_name: &str) -> ::r2r::Result<::r2r::ParameterValue>
            {
                let (prefix, suffix) = match param_name.split_once('.') {
                    None => (param_name, ""),
                    Some((prefix, suffix)) => (prefix, suffix)
                };
                let result = match prefix {
                    #get_param_matches
                    _ => Err(::r2r::Error::InvalidParameterName {
                        name: "".into(),
                    }),
                };
                result.map_err(|e| e.update_param_name(&param_name))
            }
            fn set_parameter(&mut self, param_name: &str, param_val: &::r2r::ParameterValue) -> ::r2r::Result<()>
            {
                let (prefix, suffix) = match param_name.split_once('.') {
                    None => (param_name, ""),
                    Some((prefix, suffix)) => (prefix, suffix)
                };
                let result = match prefix {
                    #set_param_matches
                    _ => Err(::r2r::Error::InvalidParameterName {
                        name: "".into(),
                    }),
                };
                result.map_err(|e| e.update_param_name(&param_name))
            }
        }
    };

    // Hand the output tokens back to the compiler.
    proc_macro::TokenStream::from(expanded)
}

// Generate calls to register functions of struct fields
fn get_register_calls(data: &Data) -> TokenStream {
    match *data {
        Data::Struct(ref data) => match data.fields {
            Fields::Named(ref fields) => {
                let field_matches = fields.named.iter().map(|f| {
                    let name = &f.ident;
                    let format_str = format!("{{prefix}}{}", name.as_ref().unwrap());
                    quote_spanned! {
                        f.span() =>
                            self.#name.register_parameters(&format!(#format_str), params)?;
                    }
                });
                quote! {
                    #(#field_matches)*
                }
            }
            _ => unimplemented!(),
        },
        Data::Enum(_) | Data::Union(_) => unimplemented!(),
    }
}

// Generate match arms for RosParams::update_parameters()
fn param_matches_for(call: TokenStream, data: &Data) -> TokenStream {
    match *data {
        Data::Struct(ref data) => match data.fields {
            Fields::Named(ref fields) => {
                let field_matches = fields.named.iter().map(|f| {
                    let name = &f.ident;
                    let name_str = format!("{}", name.as_ref().unwrap());
                    quote_spanned! {
                        f.span() =>
                            #name_str => {
                                self.#name.#call
                            }
                    }
                });
                quote! {
                    #(#field_matches)*
                }
            }
            _ => unimplemented!(),
        },
        Data::Enum(_) | Data::Union(_) => unimplemented!(),
    }
}
