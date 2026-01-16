//! Proc macros for nano-ros message type generation
//!
//! Provides `#[derive(RosMessage)]` and `#[derive(RosService)]` macros.

use proc_macro::TokenStream;
use quote::quote;
use syn::{parse_macro_input, DeriveInput, Fields, LitStr};

/// Derive macro for ROS message types
///
/// Generates `Serialize`, `Deserialize`, and `RosMessage` implementations.
///
/// # Attributes
///
/// - `#[ros(type_name = "...")]` - Full ROS type name (required)
/// - `#[ros(hash = "...")]` - RIHS type hash (required)
///
/// # Example
///
/// ```ignore
/// use nano_ros_macros::RosMessage;
///
/// #[derive(RosMessage)]
/// #[ros(type_name = "std_msgs::msg::dds_::String_")]
/// #[ros(hash = "abc123...")]
/// pub struct StringMsg {
///     pub data: heapless::String<256>,
/// }
/// ```
#[proc_macro_derive(RosMessage, attributes(ros))]
pub fn derive_ros_message(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;
    let generics = &input.generics;
    let (impl_generics, ty_generics, where_clause) = generics.split_for_impl();

    // Extract attributes
    let mut type_name: Option<String> = None;
    let mut type_hash: Option<String> = None;

    for attr in &input.attrs {
        if attr.path().is_ident("ros") {
            let _ = attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("type_name") {
                    let value: LitStr = meta.value()?.parse()?;
                    type_name = Some(value.value());
                } else if meta.path.is_ident("hash") {
                    let value: LitStr = meta.value()?.parse()?;
                    type_hash = Some(value.value());
                }
                Ok(())
            });
        }
    }

    let type_name = type_name.unwrap_or_else(|| format!("{}::msg::dds_::{}_", "unknown", name));
    let type_hash = type_hash.unwrap_or_else(|| "0".repeat(64));

    // Get fields
    let fields = match &input.data {
        syn::Data::Struct(data) => match &data.fields {
            Fields::Named(fields) => &fields.named,
            Fields::Unit => {
                // Unit struct (no fields)
                return generate_unit_struct_impl(
                    name,
                    &impl_generics,
                    &ty_generics,
                    where_clause,
                    &type_name,
                    &type_hash,
                );
            }
            _ => {
                return syn::Error::new_spanned(&input, "RosMessage only supports named fields")
                    .to_compile_error()
                    .into()
            }
        },
        _ => {
            return syn::Error::new_spanned(&input, "RosMessage can only be derived for structs")
                .to_compile_error()
                .into()
        }
    };

    // Generate serialize calls for each field
    let serialize_fields = fields.iter().map(|f| {
        let field_name = &f.ident;
        quote! {
            self.#field_name.serialize(writer)?;
        }
    });

    // Generate deserialize calls for each field
    let deserialize_fields = fields.iter().map(|f| {
        let field_name = &f.ident;
        quote! {
            #field_name: Deserialize::deserialize(reader)?,
        }
    });

    let expanded = quote! {
        impl #impl_generics nano_ros_serdes::Serialize for #name #ty_generics #where_clause {
            fn serialize(&self, writer: &mut nano_ros_serdes::CdrWriter) -> Result<(), nano_ros_serdes::SerError> {
                use nano_ros_serdes::Serialize;
                #(#serialize_fields)*
                Ok(())
            }
        }

        impl #impl_generics nano_ros_serdes::Deserialize for #name #ty_generics #where_clause {
            fn deserialize(reader: &mut nano_ros_serdes::CdrReader) -> Result<Self, nano_ros_serdes::DeserError> {
                use nano_ros_serdes::Deserialize;
                Ok(Self {
                    #(#deserialize_fields)*
                })
            }
        }

        impl #impl_generics nano_ros_core::RosMessage for #name #ty_generics #where_clause {
            const TYPE_NAME: &'static str = #type_name;
            const TYPE_HASH: &'static str = #type_hash;
        }
    };

    TokenStream::from(expanded)
}

fn generate_unit_struct_impl(
    name: &syn::Ident,
    impl_generics: &syn::ImplGenerics,
    ty_generics: &syn::TypeGenerics,
    where_clause: Option<&syn::WhereClause>,
    type_name: &str,
    type_hash: &str,
) -> TokenStream {
    let expanded = quote! {
        impl #impl_generics nano_ros_serdes::Serialize for #name #ty_generics #where_clause {
            fn serialize(&self, _writer: &mut nano_ros_serdes::CdrWriter) -> Result<(), nano_ros_serdes::SerError> {
                Ok(())
            }
        }

        impl #impl_generics nano_ros_serdes::Deserialize for #name #ty_generics #where_clause {
            fn deserialize(_reader: &mut nano_ros_serdes::CdrReader) -> Result<Self, nano_ros_serdes::DeserError> {
                Ok(Self {})
            }
        }

        impl #impl_generics nano_ros_core::RosMessage for #name #ty_generics #where_clause {
            const TYPE_NAME: &'static str = #type_name;
            const TYPE_HASH: &'static str = #type_hash;
        }
    };
    TokenStream::from(expanded)
}

/// Derive macro for ROS service types
///
/// # Attributes
///
/// - `#[ros(service_name = "...")]` - Full ROS service name (required)
/// - `#[ros(hash = "...")]` - RIHS service hash (required)
/// - `#[ros(request = "RequestType")]` - Request type name (required)
/// - `#[ros(reply = "ReplyType")]` - Reply type name (required)
///
/// # Example
///
/// ```ignore
/// use nano_ros_macros::RosService;
///
/// #[derive(RosService)]
/// #[ros(service_name = "std_srvs::srv::dds_::Empty_")]
/// #[ros(hash = "abc123...")]
/// #[ros(request = "EmptyRequest")]
/// #[ros(reply = "EmptyReply")]
/// pub struct Empty;
/// ```
#[proc_macro_derive(RosService, attributes(ros))]
pub fn derive_ros_service(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let name = &input.ident;

    // Extract attributes
    let mut service_name: Option<String> = None;
    let mut service_hash: Option<String> = None;
    let mut request_type: Option<syn::Ident> = None;
    let mut reply_type: Option<syn::Ident> = None;

    for attr in &input.attrs {
        if attr.path().is_ident("ros") {
            let _ = attr.parse_nested_meta(|meta| {
                if meta.path.is_ident("service_name") {
                    let value: LitStr = meta.value()?.parse()?;
                    service_name = Some(value.value());
                } else if meta.path.is_ident("hash") {
                    let value: LitStr = meta.value()?.parse()?;
                    service_hash = Some(value.value());
                } else if meta.path.is_ident("request") {
                    let value: LitStr = meta.value()?.parse()?;
                    request_type = Some(syn::Ident::new(
                        &value.value(),
                        proc_macro2::Span::call_site(),
                    ));
                } else if meta.path.is_ident("reply") {
                    let value: LitStr = meta.value()?.parse()?;
                    reply_type = Some(syn::Ident::new(
                        &value.value(),
                        proc_macro2::Span::call_site(),
                    ));
                }
                Ok(())
            });
        }
    }

    let service_name =
        service_name.unwrap_or_else(|| format!("{}::srv::dds_::{}_", "unknown", name));
    let service_hash = service_hash.unwrap_or_else(|| "0".repeat(64));

    let request_type = match request_type {
        Some(t) => t,
        None => {
            return syn::Error::new_spanned(&input, "RosService requires #[ros(request = \"...\")]")
                .to_compile_error()
                .into()
        }
    };

    let reply_type = match reply_type {
        Some(t) => t,
        None => {
            return syn::Error::new_spanned(&input, "RosService requires #[ros(reply = \"...\")]")
                .to_compile_error()
                .into()
        }
    };

    let expanded = quote! {
        impl nano_ros_core::RosService for #name {
            type Request = #request_type;
            type Reply = #reply_type;

            const SERVICE_NAME: &'static str = #service_name;
            const SERVICE_HASH: &'static str = #service_hash;
        }
    };

    TokenStream::from(expanded)
}
