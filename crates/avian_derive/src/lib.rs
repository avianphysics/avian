//! Provides derive implementations for [Avian Physics](https://github.com/avianphysics/avian).

use proc_macro::TokenStream;

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
#[proc_macro_derive(PhysicsLayer)]
pub fn derive_physics_layer(input: TokenStream) -> TokenStream {
    let input = parse_macro_input!(input as DeriveInput);
    match expand_physics_layer(input) {
        Ok(tokens) => tokens.into(),
        Err(err) => err.to_compile_error().into(),
    }
}

/// Accumulates a non-fatal diagnostic, combining it with any already collected so that
/// `to_compile_error()` later emits all of them together (the old `emit_error!` behavior).
fn push_err(acc: &mut Option<syn::Error>, err: syn::Error) {
    match acc {
        Some(existing) => existing.combine(err),
        none => *none = Some(err),
    }
}

fn expand_physics_layer(input: DeriveInput) -> syn::Result<proc_macro2::TokenStream> {
    let enum_ident = input.ident;

    // Wrong item kind: nothing else can be computed, so this is fatal (was `abort!`).
    let variants = match &input.data {
        Data::Enum(data) => &data.variants,
        Data::Struct(data) => {
            return Err(syn::Error::new(
                data.struct_token.span,
                "only enums can automatically derive `PhysicsLayer`",
            ));
        }
        Data::Union(data) => {
            return Err(syn::Error::new(
                data.union_token.span,
                "only enums can automatically derive `PhysicsLayer`",
            ));
        }
    };

    if variants.len() > 32 {
        return Err(syn::Error::new_spanned(
            &enum_ident,
            "`PhysicsLayer` only supports a maximum of 32 layers",
        ));
    }

    // Non-fatal diagnostics are collected here so several can be reported at once.
    let mut errors: Option<syn::Error> = None;

    let mut default_variant_index = None;
    for (i, variant) in variants.iter().enumerate() {
        for attr in variant.attrs.iter() {
            if attr.path().is_ident("default") {
                if default_variant_index.is_some() {
                    push_err(
                        &mut errors,
                        syn::Error::new_spanned(&enum_ident, "multiple defaults"),
                    );
                    break;
                }
                default_variant_index = Some(i);
            }
        }
    }

    // No default variant: required for the rest of the logic, so fatal (was `abort!` + note).
    let Some(default_variant_index) = default_variant_index else {
        let mut err = syn::Error::new_spanned(
            &enum_ident,
            "`PhysicsLayer` enums must derive `Default` and have a variant annotated with the \
             `#[default]` attribute.\n\nnote: Manually implementing `Default` using \
             `impl Default for FooLayer` is not supported.",
        );
        if let Some(existing) = errors {
            err.combine(existing);
        }
        return Err(err);
    };

    // Move the default variant to the front (this probably isn't the best way to do this)
    let mut variants = variants.iter().collect::<Vec<_>>();
    let default_variant = variants.remove(default_variant_index);
    variants.insert(0, default_variant);

    let mut to_bits = Vec::with_capacity(variants.len());
    for (index, variant) in variants.iter().enumerate() {
        if !variant.fields.is_empty() {
            push_err(
                &mut errors,
                syn::Error::new(
                    variant.fields.span(),
                    "can only derive `PhysicsLayer` for enums without fields",
                ),
            );
            continue;
        }
        let bits: u32 = 1 << index;
        let ident = &variant.ident;
        to_bits.push(quote! { #enum_ident::#ident => #bits, });
    }

    // If anything was collected along the way, fail now with all of it.
    if let Some(err) = errors {
        return Err(err);
    }

    let all_bits: u32 = if variants.len() == 32 {
        0xffffffff
    } else {
        (1 << variants.len()) - 1
    };

    Ok(quote! {
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
    })
}
