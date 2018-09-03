use core::utils::{P2, P3};

use core::{Dot, DotAlign, Tree, TreeObject};

/// Extrude the given perimeter into the z dimension. The bottom surface of the extrusion will be on the z=`bottom_z` plane, and have the given z `thickness`.
#[derive(Debug, Clone)]
pub struct Extrusion {
    pub perimeter: Vec<P2>,
    pub bottom_z: f32,
    pub thickness: f32,
}

impl Extrusion {
    pub fn from_dot_centers(
        perimeter: &[Dot],
        thickness: f32,
        bottom_z: f32,
    ) -> Self {
        let discard_z = |pos: P3| P2::new(pos.x, pos.y);
        let centers: Vec<_> = perimeter
            .iter()
            .map(|dot| discard_z(dot.pos(DotAlign::centroid())))
            .collect();

        Self {
            perimeter: centers,
            bottom_z,
            thickness,
        }
    }
}

impl From<Extrusion> for Tree {
    fn from(extrusion: Extrusion) -> Tree {
        Tree::Object(TreeObject::Extrusion(extrusion))
    }
}
