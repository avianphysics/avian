//! Provides derive implementations for [Avian Physics](https://github.com/avianphysics/avian).

use proc_macro::TokenStream;

use proc_macro_error2::{abort, emit_error, proc_macro_error};
use quote::quote;
use syn::{Data, DeriveInput, parse_macro_input, spanned::Spanned};

// Modified macro from the discontinued Heron
// https://github.com/jcornaz/heron/blob/main/macros/src/lib.rs
/// A derive macro for defining physics layers using an enum.
///
/// Each variant of the enum represents a layer. Each layer has a unique bit determined by
/// the order of the variants. The bit value can be retrieved using the `to_bits` method.
///
/// # Requirements
///
/// - The enum must have at most 32 variants.
/// - The enum variants must not have any fields.
/// - The enum must have a default variant with the `#[default]` attribute.
///   - The first bit `1 << 0` will *always* be reserved for the default layer.
///     The bit values of the other layers are determined by their order in the enum, starting from `1 << 1`.
///
/// # Example
///
/// ```ignore
/// #[derive(PhysicsLayer, Clone, Copy, Debug, Default)]
/// enum GameLayer {
///     #[default]
///     Default, // Layer 0 - the default layer that objects are assigned to
///     Player,  // Layer 1
///     Enemy,   // Layer 2
///     Ground,  // Layer 3
/// }
///
/// // The first bit is reserved for the default layer.
/// assert_eq!(GameLayer::default().to_bits(), 1 << 0);
///
/// // The `GameLayer::Ground` layer is the fourth layer, so its bit value is `1 << 3`.
/// assert_eq!(GameLayer::Ground.to_bits(), 1 << 3);
/// ```
#[proc_macro_error]
#[proc_macro_derive(PhysicsLayer)]
pub fn derive_physics_layer(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    let enum_ident = input.ident;

    fn non_enum_item_error(span: proc_macro2::Span) -> TokenStream {
        abort!(span, "only enums can automatically derive `PhysicsLayer`");
    }
    let variants = match &input.data {
        Data::Enum(data) => &data.variants,
        Data::Struct(data) => {
            return non_enum_item_error(data.struct_token.span);
        }
        Data::Union(data) => {
            return non_enum_item_error(data.union_token.span);
        }
    };

    if variants.len() > 32 {
        emit_error!(
            enum_ident,
            "`PhysicsLayer` only supports a maximum of 32 layers"
        );
    }

    let mut default_variant_index = None;

    for (i, variant) in variants.iter().enumerate() {
        for attr in variant.attrs.iter() {
            if attr.path().is_ident("default") {
                if default_variant_index.is_some() {
                    emit_error!(enum_ident, "multiple defaults");
                    break;
                }
                default_variant_index = Some(i);
            }
        }
    }

    let Some(default_variant_index) = default_variant_index else {
        abort!(
            enum_ident,
            "`PhysicsLayer` enums must derive `Default` and have a variant annotated with the `#[default]` attribute.";
            note = "Manually implementing `Default` using `impl Default for FooLayer` is not supported."
        );
    };

    // Move the default variant to the front (this probably isn't the best way to do this)
    let mut variants = variants.iter().collect::<Vec<_>>();
    let default_variant = variants.remove(default_variant_index);
    variants.insert(0, default_variant);

    let to_bits_result: Result<Vec<_>, _> = variants
        .iter()
        .enumerate()
        .map(|(index, variant)| {
            if !variant.fields.is_empty() {
                return Err(variant.fields.span());
            }
            let bits: u32 = 1 << index;
            let ident = &variant.ident;

            Ok(quote! { #enum_ident::#ident => #bits, })
        })
        .collect();

    let to_bits = match to_bits_result {
        Ok(tokens) => tokens,
        Err(span) => {
            abort!(
                span,
                "can only derive `PhysicsLayer` for enums without fields"
            );
        }
    };

    let all_bits: u32 = if variants.len() == 32 {
        0xffffffff
    } else {
        (1 << variants.len()) - 1
    };

    let expanded = quote! {
        impl PhysicsLayer for #enum_ident {
            fn all_bits() -> u32 {
                #all_bits
            }

            fn to_bits(&self) -> u32 {
                match self {
                    #(#to_bits)*
                }
            }
        }
    };

    TokenStream::from(expanded)
}
