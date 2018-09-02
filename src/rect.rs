use core::utils::{
    midpoint, Axis, Corner2 as C2, Corner3 as C3, CubeFace, P3, R3, V3,
};
use core::{
    chain_loop, drop_solid, mark, Dot, DotSpec, MapDots, MinMaxCoord, Shape,
    Tree,
};
use cuboid::{Cuboid, CuboidLink};

use errors::{ResultExt, ScadDotsError};

#[derive(Debug, Clone, Copy, MinMaxCoord, MapDots)]
pub struct Rect {
    pub p00: Dot,
    pub p01: Dot,
    pub p10: Dot,
    pub p11: Dot,
}

#[derive(Debug, Clone, Copy)]
pub struct RectSpec {
    pub pos: P3,
    pub align: RectAlign,
    pub x_length: f32,
    pub y_length: f32,
    pub size: f32,
    pub rot: R3,
    pub shapes: RectShapes,
}

#[derive(Debug, Clone, Copy)]
pub enum RectAlign {
    Corner {
        rect: C2,
        dot: C3,
    },
    Midpoint {
        rect_a: C2,
        dot_a: C3,
        rect_b: C2,
        dot_b: C3,
    },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RectShapes {
    Cube,
    Sphere,
    Cylinder,
    Custom {
        p00: Shape,
        p10: Shape,
        p11: Shape,
        p01: Shape,
    },
}

#[derive(Debug, Clone, Copy)]
pub enum RectLink {
    Solid,
    Frame,
    Dots,
    YPosts,
    Chamfer,
}

/// Any struct implementing this trait can be used to construct a Rect, by by
/// constructing each of the 4 dots that form the corners of the Rect.
pub trait RectSpecTrait: Copy {
    fn to_dot(&self, corner: C2) -> Result<Dot, ScadDotsError>;
}

////////////////////////////////////////////////////////////////////////////////

impl Rect {
    pub fn new<T>(spec: T) -> Result<Rect, ScadDotsError>
    where
        T: RectSpecTrait,
    {
        // TODO check dimensions, ensure positive
        Ok(Rect {
            p00: spec.to_dot(C2::P00)?,
            p01: spec.to_dot(C2::P01)?,
            p10: spec.to_dot(C2::P10)?,
            p11: spec.to_dot(C2::P11)?,
        })
    }

    pub fn size(&self) -> f32 {
        self.p00.size
    }

    pub fn rot(&self) -> R3 {
        self.p00.rot
    }

    pub fn pos(&self, align: RectAlign) -> P3 {
        match align {
            RectAlign::Corner { rect, dot } => self.pos_corner(rect, dot),
            RectAlign::Midpoint {
                rect_a,
                dot_a,
                rect_b,
                dot_b,
            } => midpoint(
                self.pos_corner(rect_a, dot_a),
                self.pos_corner(rect_b, dot_b),
            ),
        }
    }

    fn pos_corner(&self, rect: C2, dot: C3) -> P3 {
        self.dot(rect).pos(dot)
    }

    // Return a copy of 1 of the 4 corner dots.
    pub fn dot(&self, corner: C2) -> Dot {
        match corner {
            C2::P00 => self.p00,
            C2::P10 => self.p10,
            C2::P01 => self.p01,
            C2::P11 => self.p11,
        }
    }

    /// Return a vector describing the direction and length of 1 edge of the
    /// Rect (starting from the Rect's origin). The edge's axis is relative to
    /// the Rect's default orientation, not it's actual rotated orientation.
    pub fn edge(&self, axis: Axis) -> V3 {
        let origin = self.pos(RectAlign::origin());
        let corner_point = self.pos(RectAlign::outside(axis.into()));
        corner_point - origin
    }

    /// Return a unit vector describing the direction of 1 edge of the Rect. The
    /// edge's axis is relative to the Rect's default orientation, not it's
    /// actual rotated orientation.
    pub fn edge_unit_vec(&self, axis: Axis) -> V3 {
        self.edge(axis).normalize()
    }

    /// Return the length of 1 edge of the Rect. The edge's axis is relative to
    /// the Rect's default orientation, not it's actual rotated orientation.
    pub fn edge_length(&self, axis: Axis) -> f32 {
        self.edge(axis).norm()
    }

    pub fn drop_solid(&self, bottom_z: f32, shape: Option<Shape>) -> Tree {
        drop_solid(&self.dots(), bottom_z, shape)
    }

    /// For debugging. Return the union of 32 small spheres placed at each
    /// corner of each Dot of the Rect.
    pub fn mark_corners(&self) -> Tree {
        // for debugging
        let mut marks = Vec::new();
        for align in RectAlign::all_corners() {
            marks.push(mark(self.pos(align), 1.));
        }
        Tree::union(marks)
    }

    pub fn link(&self, style: RectLink) -> Result<Tree, ScadDotsError> {
        let dots = self.dots();
        Ok(match style {
            RectLink::Dots => Tree::union(dots),
            RectLink::Solid => Tree::hull(dots),
            RectLink::Frame => chain_loop(&[
                self.dot(C2::P00),
                self.dot(C2::P01),
                self.dot(C2::P11),
                self.dot(C2::P10),
            ])?,
            RectLink::YPosts => union![
                hull![self.dot(C2::P00), self.dot(C2::P01)],
                hull![self.dot(C2::P10), self.dot(C2::P11)],
            ],
            RectLink::Chamfer => self
                .chamfer()
                .context("failed to link Rect in Chamfer style")?,
        })
    }

    fn chamfer(&self) -> Result<Tree, ScadDotsError> {
        // This is probably a reasonable default size, but we might want to take it as an arg in RectLink::Chamfer
        let new_dot_size = self.p00.size / 100.;
        let new_dot_shape = Shape::Cube;

        let p00 = Cuboid::from_dot(self.p00, new_dot_size, new_dot_shape)?;
        let p01 = Cuboid::from_dot(self.p01, new_dot_size, new_dot_shape)?;
        let p10 = Cuboid::from_dot(self.p10, new_dot_size, new_dot_shape)?;
        let p11 = Cuboid::from_dot(self.p11, new_dot_size, new_dot_shape)?;
        Ok(hull![
            p00.link(CuboidLink::ZPost(C2::P11))?,
            p00.link(CuboidLink::ZPost(C2::P10))?,
            p00.link(CuboidLink::ZPost(C2::P01))?,
            p10.link(CuboidLink::ZPost(C2::P01))?,
            p10.link(CuboidLink::ZPost(C2::P00))?,
            p10.link(CuboidLink::ZPost(C2::P11))?,
            p01.link(CuboidLink::ZPost(C2::P10))?,
            p01.link(CuboidLink::ZPost(C2::P00))?,
            p01.link(CuboidLink::ZPost(C2::P11))?,
            p11.link(CuboidLink::ZPost(C2::P00))?,
            p11.link(CuboidLink::ZPost(C2::P10))?,
            p11.link(CuboidLink::ZPost(C2::P01))?,
        ])
    }

    fn dots(&self) -> Vec<Dot> {
        C2::all_clockwise()
            .into_iter()
            .map(|c| self.dot(c))
            .collect()
    }
}

impl RectSpec {
    /// The length of the Rect's inner edge along the given axis (relative to the default orientation).
    pub fn inner_length(&self, axis: Axis) -> f32 {
        match axis {
            Axis::X => self.x_length - 2. * self.size,
            Axis::Y => self.y_length - 2. * self.size,
            Axis::Z => self.size,
        }
    }

    /// Make a copy with a new position
    pub fn with_pos(self, new_pos: P3) -> Self {
        let mut new = self;
        new.pos = new_pos;
        new
    }

    /// Make a copy with a new alignment
    pub fn with_align(self, new_align: RectAlign) -> Self {
        let mut new = self;
        new.align = new_align;
        new
    }

    /// Make a copy with a new rotation
    pub fn with_rot(self, new_rot: R3) -> Self {
        let mut new = self;
        new.rot = new_rot;
        new
    }

    /// Make a copy with a new outer x edge length.
    pub fn with_x_length(self, new_x_length: f32) -> Self {
        let mut new = self;
        new.x_length = new_x_length;
        new
    }

    /// Make a copy with a new outer y edge length.
    pub fn with_y_length(self, new_y_length: f32) -> Self {
        let mut new = self;
        new.y_length = new_y_length;
        new
    }

    /// Make a copy with a new dot size.
    pub fn with_size(self, new_size: f32) -> Self {
        let mut new = self;
        new.size = new_size;
        new
    }

    /// Make a copy with new dot shapes
    pub fn with_shapes(self, new_shapes: RectShapes) -> Self {
        let mut new = self;
        new.shapes = new_shapes;
        new
    }
}

impl RectSpecTrait for RectSpec {
    fn to_dot(&self, corner: C2) -> Result<Dot, ScadDotsError> {
        let dot_dimensions = V3::new(self.size, self.size, self.size);
        let rect_dimensions =
            V3::new(self.x_length - self.size, self.y_length - self.size, 0.);
        let origin =
            self.pos
                - self.align.offset(dot_dimensions, rect_dimensions, self.rot);

        let pos = origin + corner.offset(rect_dimensions, self.rot);
        let spec = DotSpec {
            pos: pos,
            align: C3::P000.into(),
            rot: self.rot,
            size: self.size,
        };
        Ok(Dot::new(self.shapes.get(corner), spec))
    }
}

impl RectAlign {
    /// Align to the Rect's origin, the outside P000 corner.
    pub fn origin() -> RectAlign {
        RectAlign::outside(C3::P000)
    }

    // Align to the midpoint of the other 2 given alignment positions.
    pub fn midpoint(
        a: RectAlign,
        b: RectAlign,
    ) -> Result<RectAlign, ScadDotsError> {
        match (a, b) {
            (
                RectAlign::Corner {
                    rect: rect_a,
                    dot: dot_a,
                },
                RectAlign::Corner {
                    rect: rect_b,
                    dot: dot_b,
                },
            ) => Ok(RectAlign::Midpoint {
                rect_a: rect_a,
                dot_a: dot_a,
                rect_b: rect_b,
                dot_b: dot_b,
            }),
            _ => {
                return Err(ScadDotsError::Midpoint);
            }
        }
    }

    /// Align to the midpoint bteween the 2 outer corners of a `Rect`, a and b.
    pub fn outside_midpoint(a: C3, b: C3) -> RectAlign {
        RectAlign::midpoint(RectAlign::outside(a), RectAlign::outside(b))
            .expect("bug in outside_midpoint()")
    }

    /// Align to the midpoint between the two inner corners of a `Rect`, a and b. "Inner corner" means, imagine a hollow rectangle made of 4 cubes, linked together to form a thick border. The empty space within the border is a like a box with 8 corners, each called inner corners.
    pub fn inside_midpoint(a: C3, b: C3) -> RectAlign {
        RectAlign::midpoint(RectAlign::inside(a), RectAlign::inside(b))
            .expect("bug in inside_midpoint()")
    }

    pub fn outside(corner: C3) -> RectAlign {
        RectAlign::Corner {
            dot: corner,
            rect: corner.into(),
        }
    }

    /// Align to the given inner corner. "Inner corner" means, imagine a hollow rectangle made of 4 cube-shaped dots, linked together to form a thick border. The empty space within the border is a like a box with 8 corners, each called inner corners.
    pub fn inside(corner: C3) -> RectAlign {
        // TODO this is not quite analagous to CuboidAlign::inside(), because
        // it's on either the top of bottom surface. Does that matter? Should
        // there be a cuboid one for inside of top/bottom surface?
        RectAlign::Corner {
            dot: corner.copy_invert(Axis::X).copy_invert(Axis::Y),
            rect: corner.into(),
        }
    }

    /// Align to the center-of-mass.
    pub fn centroid() -> RectAlign {
        RectAlign::midpoint(
            RectAlign::outside(C3::P000),
            RectAlign::outside(C3::P111),
        ).expect("bad args to midpoint calculation")
    }

    /// Align to the center of the given face of the Rect.
    pub fn center_face(face: CubeFace) -> RectAlign {
        let (a, b) = face.corners();
        RectAlign::midpoint(RectAlign::outside(a), RectAlign::outside(b))
            .expect("got bad corners from CubeFace")
    }

    /// Return all the corner alignments.
    fn all_corners() -> Vec<RectAlign> {
        let mut v = Vec::new();
        for d in C3::all() {
            for r in C2::all_clockwise() {
                v.push(RectAlign::Corner { dot: d, rect: r });
            }
        }
        v
    }

    fn offset(&self, dot_dimensions: V3, rect_dimensions: V3, rot: R3) -> V3 {
        let helper = |dot: C3, rect: C2| {
            dot.offset(dot_dimensions, rot) + rect.offset(rect_dimensions, rot)
        };

        match *self {
            RectAlign::Corner { dot, rect } => helper(dot, rect),
            RectAlign::Midpoint {
                dot_a,
                rect_a,
                dot_b,
                rect_b,
            } => (helper(dot_a, rect_a) + helper(dot_b, rect_b)) / 2.,
        }
    }
}

impl RectShapes {
    pub fn get(&self, corner: C2) -> Shape {
        match *self {
            RectShapes::Custom { p00, p10, p11, p01 } => match corner {
                C2::P00 => p00,
                C2::P01 => p01,
                C2::P10 => p10,
                C2::P11 => p11,
            },
            RectShapes::Cube => Shape::Cube,
            RectShapes::Sphere => Shape::Sphere,
            RectShapes::Cylinder => Shape::Cylinder,
        }
    }
}

impl From<Shape> for RectShapes {
    fn from(shape: Shape) -> Self {
        match shape {
            Shape::Cube => RectShapes::Cube,
            Shape::Cylinder => RectShapes::Cylinder,
            Shape::Sphere => RectShapes::Sphere,
        }
    }
}
