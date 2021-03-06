use core::utils::{
    midpoint, Axis, Corner1 as C1, Corner2 as C2, Corner3 as C3, CubeFace,
    Fraction, P3, R3, V3,
};
use core::{mark, Dot, DotShape, MapDots, MinMaxCoord, Tree};
use errors::ScadDotsError;
use post::{Post, PostLink};
use rect::{Rect, RectAlign, RectLink, RectShapes, RectSpec};

#[derive(Debug, Clone, Copy, MapDots, MinMaxCoord)]
/// A cuboid (box) is made of 2 rects, one above the other
pub struct Cuboid {
    pub top: Rect,
    pub bot: Rect,
}

#[derive(Debug, Clone, Copy)]
pub struct CuboidSpec {
    pub pos: P3,
    pub align: CuboidAlign,
    pub x_length: f32,
    pub y_length: f32,
    pub z_length: f32,
    pub size: f32,
    pub rot: R3,
    pub shapes: CuboidShapes,
}

#[derive(Debug, Clone, Copy)]
pub struct CuboidSpecChamferZHole {
    pub pos: P3,
    pub align: CuboidAlign,
    pub x_length: f32,
    pub y_length: f32,
    pub z_length: f32,
    pub chamfer: Fraction,
    pub rot: R3,
    pub shapes: CuboidShapes,
}

#[derive(Debug, Clone, Copy)]
pub enum CuboidAlign {
    Corner {
        cuboid: C3,
        dot: C3,
    },
    Midpoint {
        cuboid_a: C3,
        dot_a: C3,
        cuboid_b: C3,
        dot_b: C3,
    },
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CuboidShapes {
    Cube,
    Sphere,
    Cylinder,
    Round,
    Custom {
        p000: DotShape,
        p100: DotShape,
        p110: DotShape,
        p010: DotShape,
        p001: DotShape,
        p101: DotShape,
        p111: DotShape,
        p011: DotShape,
    },
}

#[derive(Debug, Clone, Copy)]
pub enum CuboidLink {
    Solid,
    Frame,
    Dots,
    Sides,
    Face(CubeFace),
    OpenBot,
    ZPost(C2),
    ChamferZ,
}

/// Any struct implementing this trait can be used to construct a Cuboid, by
/// constructing the upper and lower Rects that together form a Cuboid.
pub trait CuboidSpecTrait: Copy {
    fn to_rect(&self, upper_or_lower: C1) -> Result<Rect, ScadDotsError>;
}

////////////////////////////////////////////////////////////////////////////////

impl CuboidAlign {
    pub fn origin() -> Self {
        Self::outside(C3::P000)
    }

    pub fn outside(corner: C3) -> Self {
        CuboidAlign::Corner {
            cuboid: corner,
            dot: corner,
        }
    }

    pub fn inside(corner: C3) -> Self {
        CuboidAlign::Corner {
            cuboid: corner,
            dot: corner.copy_invert_all_axes(),
        }
    }

    pub fn center_face(face: CubeFace) -> Self {
        let (a, b) = face.corners();
        Self::midpoint(Self::outside(a), Self::outside(b))
            .expect("got bad corners from CubeFace")
    }

    pub fn center_inside_face(face: CubeFace) -> Self {
        let (a, b) = face.corners();
        Self::midpoint(Self::inside(a), Self::inside(b))
            .expect("got bad corners from CubeFace")
    }

    pub fn centroid() -> Self {
        Self::midpoint(Self::outside(C3::P000), Self::outside(C3::P111))
            .expect("bad args to midpoint calculation")
    }

    pub fn outside_midpoint(a: C3, b: C3) -> Self {
        // Return the midpoint between the two outer corners a and b
        // TODO better name?
        Self::midpoint(Self::outside(a), Self::outside(b))
            .expect("bug in outside_midpoint()")
    }

    pub fn inside_midpoint(a: C3, b: C3) -> Self {
        // Return the midpoint between the two inner corners a and b
        Self::midpoint(Self::inside(a), Self::inside(b))
            .expect("bug in inside_midpoint()")
    }

    pub fn midpoint(a: Self, b: Self) -> Result<Self, ScadDotsError> {
        match (a, b) {
            (
                CuboidAlign::Corner {
                    cuboid: cuboid_a,
                    dot: dot_a,
                },
                CuboidAlign::Corner {
                    cuboid: cuboid_b,
                    dot: dot_b,
                },
            ) => Ok(CuboidAlign::Midpoint {
                cuboid_a,
                dot_a,
                cuboid_b,
                dot_b,
            }),
            _ => Err(ScadDotsError::Midpoint),
        }
    }

    /// Return a list of all possible alignment values
    pub fn all_corners() -> Vec<Self> {
        let mut v = Vec::new();
        for d in C3::all() {
            for c in C3::all() {
                v.push(CuboidAlign::Corner { cuboid: c, dot: d });
            }
        }
        v
    }

    fn offset(self, cuboid_dimensions: V3, dot_dimensions: V3, rot: R3) -> V3 {
        // TODO share code with RectAlign::offset()?
        let helper = |cuboid: C3, dot: C3| {
            dot.offset(dot_dimensions, rot)
                + cuboid.offset(cuboid_dimensions, rot)
        };

        match self {
            CuboidAlign::Corner { cuboid, dot } => helper(cuboid, dot),
            CuboidAlign::Midpoint {
                cuboid_a,
                dot_a,
                cuboid_b,
                dot_b,
            } => (helper(cuboid_a, dot_a) + helper(cuboid_b, dot_b)) / 2.,
        }
    }
}

impl From<CuboidAlign> for RectAlign {
    fn from(c: CuboidAlign) -> RectAlign {
        match c {
            CuboidAlign::Corner { cuboid, dot } => RectAlign::Corner {
                rect: cuboid.into(),
                dot,
            },
            CuboidAlign::Midpoint {
                cuboid_a,
                dot_a,
                cuboid_b,
                dot_b,
            } => RectAlign::Midpoint {
                rect_a: cuboid_a.into(),
                dot_a,
                rect_b: cuboid_b.into(),
                dot_b,
            },
        }
    }
}

impl Cuboid {
    pub fn new<T>(spec: T) -> Result<Self, ScadDotsError>
    where
        T: CuboidSpecTrait,
    {
        Ok(Self {
            bot: spec.to_rect(C1::P0)?,
            top: spec.to_rect(C1::P1)?,
        })
    }

    pub fn from_dot<T>(
        dot: Dot,
        new_size_of_dots: f32,
        shapes: T,
    ) -> Result<Self, ScadDotsError>
    where
        T: Into<CuboidShapes>,
    {
        if dot.shape != DotShape::Cube {
            return Err(ScadDotsError::Args
                .context("Cuboid can only be created from a cube-shaped dot"));
        }

        let spec = CuboidSpec {
            pos: dot.p000,
            align: CuboidAlign::origin(),
            x_length: dot.size,
            y_length: dot.size,
            z_length: dot.size,
            rot: dot.rot,
            size: new_size_of_dots,
            shapes: shapes.into(),
        };
        Self::new(spec)
    }

    /// Return a vector describing the direction and length of 1 edge of the
    /// Cuboid (starting from the Cuboid's origin). The edge's axis is relative to
    /// the Cuboid's default orientation, not it's actual rotated orientation.
    pub fn edge(&self, axis: Axis) -> V3 {
        match axis {
            Axis::X | Axis::Y => self.bot.edge(axis),
            Axis::Z => {
                let origin = self.pos(CuboidAlign::origin());
                let corner_point = self.pos(CuboidAlign::outside(C3::P001));
                corner_point - origin
            }
        }
    }

    pub fn edge_unit_vec(&self, axis: Axis) -> V3 {
        self.edge(axis).normalize()
    }

    pub fn edge_length(&self, axis: Axis) -> f32 {
        self.edge(axis).norm()
    }

    pub fn size(&self) -> f32 {
        self.top.size()
    }

    pub fn pos(&self, align: CuboidAlign) -> P3 {
        match align {
            CuboidAlign::Corner { cuboid, dot } => self.pos_corner(cuboid, dot),
            CuboidAlign::Midpoint {
                cuboid_a,
                dot_a,
                cuboid_b,
                dot_b,
            } => midpoint(
                self.pos_corner(cuboid_a, dot_a),
                self.pos_corner(cuboid_b, dot_b),
            ),
        }
    }

    fn pos_corner(&self, cuboid: C3, dot: C3) -> P3 {
        self.dot(cuboid).pos(dot)
    }

    pub fn rot(&self) -> R3 {
        // Assumes both the top and bottom rects have the same rotation
        // (otherwise self was constructed incorrectly)
        self.top.rot()
    }

    pub fn dot(&self, corner: C3) -> Dot {
        let rect_corner = C2::from(corner);
        if corner.is_high(Axis::Z) {
            self.top.dot(rect_corner)
        } else {
            self.bot.dot(rect_corner)
        }
    }

    /// Return a vertical post between the upper and lower Dots at the given xy corner.
    pub fn vertical_post(&self, corner: C2) -> Post {
        // TODO rename to get_vertical_post or something, really unclear
        Post {
            top: self.top.dot(corner),
            bot: self.bot.dot(corner),
        }
    }

    pub fn rect(&self, face: CubeFace) -> Rect {
        match face {
            CubeFace::Z0 => self.bot,
            CubeFace::Z1 => self.top,
            CubeFace::X0 => Rect {
                p00: self.bot.dot(C2::P00),
                p10: self.bot.dot(C2::P01),
                p01: self.top.dot(C2::P00),
                p11: self.top.dot(C2::P01),
            },
            CubeFace::X1 => Rect {
                p00: self.bot.dot(C2::P10),
                p10: self.bot.dot(C2::P11),
                p01: self.top.dot(C2::P10),
                p11: self.top.dot(C2::P11),
            },
            CubeFace::Y0 => Rect {
                p00: self.bot.dot(C2::P00),
                p10: self.bot.dot(C2::P10),
                p01: self.top.dot(C2::P00),
                p11: self.top.dot(C2::P10),
            },
            CubeFace::Y1 => Rect {
                p00: self.bot.dot(C2::P01),
                p10: self.bot.dot(C2::P11),
                p01: self.top.dot(C2::P01),
                p11: self.top.dot(C2::P11),
            },
        }
    }

    pub fn mark_corners(&self) -> Tree {
        // for debugging
        let mut marks = Vec::new();
        for align in CuboidAlign::all_corners() {
            marks.push(mark(self.pos(align), 1.));
        }
        Tree::union(marks)
    }

    pub fn link(&self, style: CuboidLink) -> Result<Tree, ScadDotsError> {
        Ok(match style {
            CuboidLink::Solid => hull![
                self.bot.link(RectLink::Solid)?,
                self.top.link(RectLink::Solid)?
            ],
            CuboidLink::Frame => union![
                self.bot.link(RectLink::Frame)?,
                self.top.link(RectLink::Frame)?,
                self.vertical_post(C2::P00).link(PostLink::Solid),
                self.vertical_post(C2::P10).link(PostLink::Solid),
                self.vertical_post(C2::P11).link(PostLink::Solid),
                self.vertical_post(C2::P01).link(PostLink::Solid),
            ],
            CuboidLink::Dots => union![
                self.top.link(RectLink::Dots)?,
                self.bot.link(RectLink::Dots)?,
            ],
            CuboidLink::Face(face) => self.rect(face).link(RectLink::Solid)?,
            CuboidLink::ZPost(corner) => {
                self.vertical_post(corner).link(PostLink::Solid)
            }
            CuboidLink::Sides => union![
                self.link(CuboidLink::Face(CubeFace::X0))?,
                self.link(CuboidLink::Face(CubeFace::X1))?,
                self.link(CuboidLink::Face(CubeFace::Y0))?,
                self.link(CuboidLink::Face(CubeFace::Y1))?,
            ],
            CuboidLink::OpenBot => union![
                self.link(CuboidLink::Sides)?,
                self.link(CuboidLink::Face(CubeFace::Z1))?,
            ],
            CuboidLink::ChamferZ => union![
                self.bot.link(RectLink::Chamfer)?,
                self.top.link(RectLink::Chamfer)?,
            ],
        })
    }
}

impl CuboidSpecTrait for CuboidSpec {
    fn to_rect(&self, upper_or_lower: C1) -> Result<Rect, ScadDotsError> {
        let dot_lengths = V3::new(self.size, self.size, self.size);
        let cuboid_lengths = V3::new(
            self.x_length - self.size,
            self.y_length - self.size,
            self.z_length - self.size,
        );
        let origin =
            self.pos - self.align.offset(cuboid_lengths, dot_lengths, self.rot);

        let height = upper_or_lower.offset(cuboid_lengths.z, self.rot);

        let spec = RectSpec {
            pos: origin + height,
            align: RectAlign::origin(),
            y_length: self.y_length,
            x_length: self.x_length,
            size: self.size,
            rot: self.rot,
            shapes: self.shapes.get(upper_or_lower),
        };
        Rect::new(spec)
    }
}

impl From<CuboidSpecChamferZHole> for CuboidSpec {
    fn from(spec: CuboidSpecChamferZHole) -> Self {
        CuboidSpec {
            size: spec.chamfer.unwrap() * spec.x_length.min(spec.y_length) / 2.,
            pos: spec.pos,
            align: spec.align,
            x_length: spec.x_length,
            y_length: spec.y_length,
            z_length: spec.z_length,
            rot: spec.rot,
            shapes: spec.shapes,
        }
    }
}

impl CuboidSpecTrait for CuboidSpecChamferZHole {
    fn to_rect(&self, upper_or_lower: C1) -> Result<Rect, ScadDotsError> {
        // TODO this is a bit inefficient because we'll re-convert self to
        // CuboidSpec every time this is called. But it makes the API a lot
        // simpler than the old system with multiple traits.
        let canonical_spec = CuboidSpec::from(*self);
        canonical_spec.to_rect(upper_or_lower)
    }
}

impl CuboidShapes {
    fn get(self, upper_or_lower: C1) -> RectShapes {
        match self {
            CuboidShapes::Round => match upper_or_lower {
                C1::P1 => RectShapes::Sphere,
                C1::P0 => RectShapes::Cylinder,
            },
            CuboidShapes::Custom {
                p000,
                p010,
                p100,
                p110,
                p001,
                p101,
                p011,
                p111,
            } => match upper_or_lower {
                C1::P1 => RectShapes::Custom {
                    p00: p001,
                    p01: p011,
                    p10: p101,
                    p11: p111,
                },
                C1::P0 => RectShapes::Custom {
                    p00: p000,
                    p01: p010,
                    p10: p100,
                    p11: p110,
                },
            },
            CuboidShapes::Cube => RectShapes::Cube,
            CuboidShapes::Sphere => RectShapes::Sphere,
            CuboidShapes::Cylinder => RectShapes::Cylinder,
        }
    }
}

impl From<DotShape> for CuboidShapes {
    fn from(shape: DotShape) -> CuboidShapes {
        match shape {
            DotShape::Cube => CuboidShapes::Cube,
            DotShape::Sphere => CuboidShapes::Sphere,
            DotShape::Cylinder => CuboidShapes::Cylinder,
        }
    }
}
