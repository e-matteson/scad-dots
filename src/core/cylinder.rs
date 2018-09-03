use core::{Tree, TreeObject};

use core::utils::{Axis, Corner1 as C1, P3, R3, V3};

// Cylinders have only basic support, without all the nice features of Dots.
// They should only be used for making discs that are shorter than their
// diameter.
// The default orientation is for the cylinder's axis (height) to be the z axis.
#[derive(Debug, Clone, Copy)]
pub struct Cylinder {
    pub center_bot_pos: P3,
    pub diameter: f32,
    pub height: f32,
    pub rot: R3,
}

#[derive(Debug, Clone, Copy)]
pub struct CylinderSpec {
    pub pos: P3,
    pub align: CylinderAlign,
    pub diameter: f32,
    pub height: f32,
    pub rot: R3,
}

/// Specify an alignment point on a Cylinder. This does not depend on a particular Cylinder's dimensions.
#[derive(Debug, Clone, Copy)]
pub enum CylinderAlign {
    /// The center of the circle at the bottom (C1::P0) or top (C1::P1) of the cylinder.
    EndCenter(C1),
    /// The centroid of the cylinder (the center of the circular cross-section, at half of the total height).
    /// TODO is this name accurate?
    Centroid,
}

impl Cylinder {
    /// Create a new cylinder.
    pub fn new(spec: CylinderSpec) -> Self {
        Self {
            center_bot_pos: spec.center_bot_pos(),
            diameter: spec.diameter,
            height: spec.height,
            rot: spec.rot,
        }
    }

    pub fn pos(&self, align: CylinderAlign) -> P3 {
        self.center_bot_pos + align.offset(self.diameter, self.height, self.rot)
    }

    pub fn unit_axis(&self) -> V3 {
        let z: V3 = Axis::Z.into();
        self.rot * z
    }

    pub fn axis(&self) -> V3 {
        self.height * self.unit_axis()
    }
}

impl From<Cylinder> for Tree {
    fn from(cylinder: Cylinder) -> Tree {
        Tree::Object(TreeObject::Cylinder(cylinder))
    }
}

impl CylinderSpec {
    fn center_bot_pos(&self) -> P3 {
        self.pos - self.align.offset(self.diameter, self.height, self.rot)
    }
}

impl CylinderAlign {
    /// Return a vector from a cylinder's canonical alignment point (at the center of the bottom circle) to this alignment point.
    fn offset(self, _diameter: f32, height: f32, rot: R3) -> V3 {
        match self {
            CylinderAlign::EndCenter(end) => match end {
                C1::P0 => V3::zeros(),
                C1::P1 => rot * V3::new(0., 0., height),
            },
            CylinderAlign::Centroid => {
                // Find the vector to halfway between the 2 end-centers.
                let to_top = CylinderAlign::EndCenter(C1::P1)
                    .offset(_diameter, height, rot);
                let to_bot = CylinderAlign::EndCenter(C1::P0)
                    .offset(_diameter, height, rot);
                (to_top + to_bot) / 2.
            }
        }
    }
}
