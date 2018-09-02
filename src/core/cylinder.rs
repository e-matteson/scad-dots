use core::{Tree, TreeObject};
use errors::ScadDotsError;

use core::utils::{
    radians_to_degrees, unwrap_rot_axis, Axis, Corner1 as C1, P3, R3, V3,
};

// Cylinders have only basic support, without all the nice features of Dots.
// They should only be used for making discs that are shorter than their
// diameter.
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

#[derive(Debug, Clone, Copy)]
pub enum CylinderAlign {
    EndCenter(C1),
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

    // pub fn pos(&self, align: CylinderAlign)

    pub fn rot_degs_for_rendering(&self) -> f32 {
        radians_to_degrees(self.rot.angle())
    }

    /// TODO what is this exactly? It's not the actual vector along the axis
    /// of the cylinder, apparently. It works for rendering, but it's
    /// misleading for users.
    pub fn rot_axis_for_rendering(&self) -> Result<V3, ScadDotsError> {
        unwrap_rot_axis(self.rot)
    }

    pub fn dim_unit_vec_axis(&self) -> V3 {
        let z: V3 = Axis::Z.into();
        self.rot * z
    }

    pub fn dim_vec_axis(&self) -> V3 {
        self.height * self.dim_unit_vec_axis()
    }

    pub fn center_solid(&self) -> P3 {
        self.center_bot_pos + self.dim_vec_axis() / 2.
    }

    pub fn center_top(&self) -> P3 {
        self.center_bot_pos + self.dim_vec_axis()
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
    fn offset(&self, _diameter: f32, height: f32, rot: R3) -> V3 {
        match self {
            CylinderAlign::EndCenter(end) => match end {
                C1::P0 => V3::zeros(),
                C1::P1 => rot * V3::new(0., 0., height),
            },
        }
    }
}
