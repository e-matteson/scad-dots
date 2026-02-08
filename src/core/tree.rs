use core::utils::{ColorSpec, Plane, V3};
use core::{Cylinder, Dot, DotShape, Extrusion, DotAlign};

#[derive(Debug, Clone)]
pub enum Tree {
    Object(TreeObject),
    Operator(TreeOperator),
}

#[derive(Debug, Clone)]
pub enum TreeObject {
    /// A primitive object representing a dot with equal side lengths.
    Dot(Dot),
    /// A primitive object representing a cylinder with an arbitrary height and diameter.
    Cylinder(Cylinder),
    /// A primitive object representing a 2d polygon that it is extruded into the 3rd dimension.
    Extrusion(Extrusion),
}

#[derive(Debug, Clone)]
pub enum TreeOperator {
    /// An operator that takes the union of its children.
    Union(Vec<Tree>),
    /// An operator that gets the smallest convex shape that encloses all the elements
    Hull(Vec<Tree>),
    /// Subtract all following elements from the first
    Diff(Vec<Tree>),
    Intersect(Vec<Tree>),
    Rotate(f32, V3, Vec<Tree>),
    Color(ColorSpec, Box<Tree>),
    Mirror(V3, Box<Tree>), // Mirrors across plane with the given normal vec
}

#[macro_export]
macro_rules! union {
    ( $( $tree_like:expr),* $(,)* ) => {
        $crate::core::Tree::union(
            vec![ $($crate::core::Tree::from($tree_like),)* ]
        )
    }
}

#[macro_export]
macro_rules! hull {
    ( $( $tree_like:expr),* $(,)* ) => {
        $crate::core::Tree::hull(
            vec![ $($crate::core::Tree::from($tree_like),)* ]
        )
    }
}

#[macro_export]
macro_rules! diff {
    ( $( $tree_like:expr),* $(,)* ) => {
        $crate::core::Tree::diff(
            vec![ $($crate::core::Tree::from($tree_like),)* ]
        )
    }
}

#[macro_export]
macro_rules! intersect {
    ( $( $tree_like:expr),* $(,)* ) => {
        $crate::core::Tree::intersect(
            vec![ $($crate::core::Tree::from($tree_like),)* ]
        )
    }
}

#[macro_export]
macro_rules! mirror {
    ($normal:expr, $tree_like:expr $(,)* ) => {
        $crate::core::Tree::mirror($normal, $crate::core::Tree::from($tree_like))
    };
}

#[macro_export]
macro_rules! red {
    ($tree_like:expr $(,)* ) => {
        $crate::core::Tree::color($crate::utils::ColorSpec::Red, $crate::core::Tree::from($tree_like))
    };
}

impl Tree {
    pub fn union<T>(tree_like: Vec<T>) -> Self
    where
        T: Into<Self>,
    {
        Tree::Operator(TreeOperator::Union(
            tree_like.into_iter().map(|x| x.into()).collect(),
        ))
    }

    pub fn hull<T>(tree_like: Vec<T>) -> Self
    where
        T: Into<Self>,
    {
        Tree::Operator(TreeOperator::Hull(
            tree_like.into_iter().map(|x| x.into()).collect(),
        ))
    }

    pub fn diff<T>(tree_like: Vec<T>) -> Self
    where
        T: Into<Self>,
    {
        Tree::Operator(TreeOperator::Diff(
            tree_like.into_iter().map(|x| x.into()).collect(),
        ))
    }

    pub fn intersect<T>(tree_like: Vec<T>) -> Self
    where
        T: Into<Self>,
    {
        Tree::Operator(TreeOperator::Intersect(
            tree_like.into_iter().map(|x| x.into()).collect(),
        ))
    }

    pub fn rotate<T>(degrees: f32, axis: V3, tree_like: Vec<T>) -> Self
    where
        T: Into<Self>,
    {
        Tree::Operator(TreeOperator::Rotate(
            degrees,
            axis,
            tree_like.into_iter().map(|x| x.into()).collect(),
        ))
    }

    pub fn mirror<S, T>(normal: S, tree_like: T) -> Self
    where
        T: Into<Self>,
        S: Into<V3>,
    {
        Tree::Operator(TreeOperator::Mirror(
            normal.into(),
            Box::new(tree_like.into()),
        ))
    }

    pub fn color<T>(color: ColorSpec, tree_like: T) -> Self
    where
        T: Into<Self>,
    {
        Tree::Operator(TreeOperator::Color(color, Box::new(tree_like.into())))
    }
}

impl From<Dot> for Tree {
    fn from(dot: Dot) -> Self {
        Tree::Object(TreeObject::Dot(dot))
    }
}

impl<'a, T> From<&'a T> for Tree
where
    T: Into<Tree> + Clone,
{
    fn from(tree_like: &'a T) -> Tree {
        tree_like.to_owned().into()
    }
}

// TODO use intra-docs link when that works.

/// Call `.drop(bottom_z, shape)` on each of the given Dots. Return the hull of all the original dots and all the dropped dots.
pub fn drop_solid(dots: &[Dot], plane: Plane, align: DotAlign, shape: Option<DotShape>) -> Tree {
    let dropped_dots = dots.iter().map(|d| d.drop_to_plane(plane, align, shape));
    let all_dots: Vec<_> =
        dots.into_iter().cloned().chain(dropped_dots).collect();
    Tree::hull(all_dots)
}
