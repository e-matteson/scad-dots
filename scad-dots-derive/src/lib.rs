extern crate proc_macro;
#[macro_use]
extern crate quote;
extern crate syn;

use proc_macro::TokenStream;

#[proc_macro_derive(MapDots, attributes(map_dots))]
pub fn map_dots(input: TokenStream) -> TokenStream {
    // Construct a string representation of the type definition
    let s = input.to_string();

    // Parse the string representation
    let ast = syn::parse_derive_input(&s).unwrap();

    // Build the impl
    let gen = impl_map_dots(&ast);
    // println!("IMPL: {}", gen.as_str());

    // Return the generated impl
    gen.parse().unwrap()
}

fn impl_map_dots(ast: &syn::DeriveInput) -> quote::Tokens {
    const ATTR_NAME: &'static str = "map_dots";

    let name = &ast.ident;
    let all_fields: Vec<_> = match ast.body {
        syn::Body::Struct(syn::VariantData::Struct(ref body_fields)) => {
            body_fields.to_owned()
        }
        // TODO support tuple structs
        _ => panic!("Can only derive MapDots for non-tuple structs"),
    };

    let mapped = fields_to_initializer_lines(
        &all_fields,
        ".map(f)",
        &|field| !is_ignored(&field, ATTR_NAME),
    );

    let ignored = fields_to_initializer_lines(
        &all_fields,
        "",
        &|field| is_ignored(&field, ATTR_NAME),
    );

    quote! {
       impl MapDots for #name {
           fn map(&self, f: &Fn(&Dot) -> Dot) -> Self {
               #name {
                   #(#mapped)*
                   #(#ignored)*
               }
           }
       }
    }
}



#[proc_macro_derive(MinMaxCoord, attributes(min_max_coord))]
pub fn compare_coords(input: TokenStream) -> TokenStream {
    // Construct a string representation of the type definition
    let s = input.to_string();

    // Parse the string representation
    let ast = syn::parse_derive_input(&s).unwrap();

    // Build the impl
    let gen = impl_compare_coords(&ast);

    // Return the generated impl
    gen.parse().unwrap()
}

fn impl_compare_coords(ast: &syn::DeriveInput) -> quote::Tokens {
    const ATTR_NAME: &'static str = "min_max_coord";

    let name = &ast.ident;
    let fields = match ast.body {
        syn::Body::Struct(syn::VariantData::Struct(ref body_fields)) => {
            body_fields.to_owned()
        }
        // TODO support tuple structs
        _ => panic!("Can only derive MinMaxCoord for non-tuple structs"),
    };

    let used: Vec<_> = fields
        .iter()
        .filter(|field| !is_ignored(field, ATTR_NAME))
        .filter_map(|field| field.ident.as_ref())
        .collect();

    quote! {
        impl MinMaxCoord for #name {
            fn all_coords(&self, axis: Axis) -> Vec<f32> {
                let mut v = Vec::new();
                #(v.extend(self.#used.all_coords(axis));)*
                v
            }
        }
    }
}


fn get_attr_values(
    field: &&syn::Field,
    attr_name: &str,
) -> Option<Vec<String>> {
    if let Some(ref attr) = field.attrs.iter().find(|x| x.name() == attr_name) {
        if let syn::MetaItem::List(_, ref nested) = attr.value {
            return Some(
                nested
                    .iter()
                    .filter_map(|item| {
                        if let &syn::NestedMetaItem::MetaItem(
                            syn::MetaItem::Word(ref word),
                        ) = item
                        {
                            Some(word.as_ref().to_owned())
                        } else {
                            None
                        }
                    })
                    .collect(),
            );
        }
    }
    None
}

// TODO no double reference
fn is_ignored(field: &&syn::Field, attr_name: &str) -> bool {
    const IGNORE_NAME: &str = "ignore";
    if let Some(v) = get_attr_values(field, attr_name) {
        v.iter().any(|s| s.as_str() == IGNORE_NAME)
    } else {
        false
    }
}

fn fields_to_initializer_lines<T>(
    fields: &[syn::Field],
    method: T,
    filter_condition: &Fn(&syn::Field) -> bool,
) -> Vec<quote::Tokens>
where
    T: Into<syn::Ident>,
{
    let method: syn::Ident = method.into();
    fields
        .iter()
        .filter(|field| filter_condition(field))
        .filter_map(|field| field.ident.as_ref())
        .map(|ident| quote! { #ident: self.#ident#method, })
        .collect()
}
