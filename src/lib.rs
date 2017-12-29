#![feature(conservative_impl_trait)]
#![feature(inclusive_range_syntax)]
#![feature(slice_rotate)]
#![allow(unused_doc_comment)] // what does this warning even mean?

extern crate nalgebra;

#[macro_use]
extern crate nom;

#[macro_use]
extern crate failure;
// extern crate failure_derive;
// extern crate error_chain;

#[macro_use]
extern crate scad_generator;

#[macro_use]
extern crate approx;

#[macro_use]
extern crate scad_dots_derive;

pub use self::harness::{check_model, Action, MAX_RELATIVE};
pub use self::parse::scad_relative_eq;

#[macro_use]
pub mod core;

pub mod utils;
pub mod post;
pub mod rect;
pub mod cuboid;
pub mod triangle;

pub mod errors;
pub mod render;

pub mod harness;
pub mod parse;
