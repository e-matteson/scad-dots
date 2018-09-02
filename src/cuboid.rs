use core::utils::{
    midpoint, Axis, Corner1 as C1, Corner2 as C2, Corner3 as C3, CubeFace,
    Fraction, P3, R3, V3,
};
use core::{mark, Dot, MapDots, MinMaxCoord, Shape, Tree};
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

#[derive(Debug, Clone, Copy)]
pub enum CuboidShapes {
    Cube,
    Sphere,
    Cylinder,
    Round,
    Custom {
        p000: Shape,
        p100: Shape,
        p110: Shape,
        p010: Shape,
        p001: Shape,
        p101: Shape,
        p111: Shape,
        p011: Shape,
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
    pub fn origin() -> CuboidAlign {
        CuboidAlign::outside(C3::P000)
    }

    pub fn outside(corner: C3) -> CuboidAlign {
        CuboidAlign::Corner {
            cuboid: corner,
            dot: corner,
        }
    }

    pub fn inside(corner: C3) -> CuboidAlign {
        CuboidAlign::Corner {
            cuboid: corner,
            dot: corner.copy_invert_all_axes(),
        }
    }

    pub fn center_face(face: CubeFace) -> CuboidAlign {
        let (a, b) = face.corners();
        CuboidAlign::midpoint(CuboidAlign::outside(a), CuboidAlign::outside(b))
            .expect("got bad corners from CubeFace")
    }

    pub fn center_inside_face(face: CubeFace) -> CuboidAlign {
        let (a, b) = face.corners();
        CuboidAlign::midpoint(CuboidAlign::inside(a), CuboidAlign::inside(b))
            .expect("got bad corners from CubeFace")
    }

    pub fn centroid() -> CuboidAlign {
        CuboidAlign::midpoint(
            CuboidAlign::outside(C3::P000),
            CuboidAlign::outside(C3::P111),
        ).expect("bad args to midpoint calculation")
    }

    pub fn outside_midpoint(a: C3, b: C3) -> CuboidAlign {
        // Return the midpoint between the two outer corners a and b
        // TODO better name?
        CuboidAlign::midpoint(CuboidAlign::outside(a), CuboidAlign::outside(b))
            .expect("bug in outside_midpoint()")
    }

    pub fn inside_midpoint(a: C3, b: C3) -> CuboidAlign {
        // Return the midpoint between the two inner corners a and b
        CuboidAlign::midpoint(CuboidAlign::inside(a), CuboidAlign::inside(b))
            .expect("bug in inside_midpoint()")
    }

    pub fn midpoint(
        a: CuboidAlign,
        b: CuboidAlign,
    ) -> Result<CuboidAlign, ScadDotsError> {
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
                cuboid_a: cuboid_a,
                dot_a: dot_a,
                cuboid_b: cuboid_b,
                dot_b: dot_b,
            }),
            _ => return Err(ScadDotsError::Midpoint),
        }
    }

    /// Return a list of all possible alignment values
    pub fn all_corners() -> Vec<CuboidAlign> {
        let mut v = Vec::new();
        for d in C3::all() {
            for c in C3::all() {
                v.push(CuboidAlign::Corner { cuboid: c, dot: d });
            }
        }
        v
    }

    pub fn offset(&self, cuboid_dim_vec: V3, dot_dim_vec: V3, rot: R3) -> V3 {
        // TODO share code with RectAlign::offset()?
        let helper = |cuboid: C3, dot: C3| {
            dot.offset(dot_dim_vec, rot) + cuboid.offset(cuboid_dim_vec, rot)
        };

        match *self {
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
                dot: dot,
            },
            CuboidAlign::Midpoint {
                cuboid_a,
                dot_a,
                cuboid_b,
                dot_b,
            } => RectAlign::Midpoint {
                rect_a: cuboid_a.into(),
                dot_a: dot_a,
                rect_b: cuboid_b.into(),
                dot_b: dot_b,
            },
        }
    }
}

impl Cuboid {
    pub fn new<T>(spec: T) -> Result<Cuboid, ScadDotsError>
    where
        T: CuboidSpecTrait,
    {
        Ok(Cuboid {
            bot: spec.to_rect(C1::P0)?,
            top: spec.to_rect(C1::P1)?,
        })
    }

    pub fn from_dot<T>(
        dot: Dot,
        new_size_of_dots: f32,
        shapes: T,
    ) -> Result<Cuboid, ScadDotsError>
    where
        T: Into<CuboidShapes>,
    {
        if dot.shape != Shape::Cube {
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
        Cuboid::new(spec)
    }

    // pub fn dot_face_protrusions(
    //     dot: Dot,
    //     face: CubeFace,
    //     size: f32,
    // ) -> Result<Rect> {
    //     // Put a tiny sphere inside each corner of the dot's face, half
    //     // sticking out of the surface. Useful for hulling the face with some
    //     // other surface that's not parallel to it.
    //     let cuboid = Cuboid::from_dot(dot, size, Shape::Sphere)?;
    //     let rect = cuboid.get_rect(face);
    //     let sign = if face.is_high() { 1. } else { -1. };
    //     let offset_unit_vec = cuboid.dim_unit_vec(face.axis());
    //     let offset = size / 2. * sign * offset_unit_vec;
    //     Ok(rect.copy_translate(&offset))
    // }

    pub fn dim_vec(&self, axis: Axis) -> V3 {
        match axis {
            Axis::X | Axis::Y => self.bot.dim_vec(axis),
            Axis::Z => {
                // NOTE used to subtract size, that was wrong
                self.pos(CuboidAlign::outside(C3::P001))
                    - self.pos(CuboidAlign::origin())
            }
        }
    }

    pub fn dim_unit_vec(&self, axis: Axis) -> V3 {
        self.dim_vec(axis).normalize()
    }

    pub fn dim_len(&self, axis: Axis) -> f32 {
        self.dim_vec(axis).norm()
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
        self.get_dot(cuboid).pos(dot)
    }

    pub fn rot(&self) -> R3 {
        // Assumes both the top and bottom rects have the same rotation
        // (otherwise self was constructed incorrectly)
        self.top.rot()
    }

    pub fn get_dot(&self, corner: C3) -> Dot {
        // TODO rename to just dot
        let rect_corner = C2::from(corner);
        if corner.is_high(Axis::Z) {
            self.top.get_dot(rect_corner)
        } else {
            self.bot.get_dot(rect_corner)
        }
    }

    pub fn get_z_post(&self, corner: C2) -> Post {
        // TODO rename to get_vertical_post or something, really unclear
        Post {
            top: self.top.get_dot(corner),
            bot: self.bot.get_dot(corner),
        }
    }

    pub fn get_rect(&self, face: CubeFace) -> Rect {
        match face {
            CubeFace::Z0 => self.bot,
            CubeFace::Z1 => self.top,
            CubeFace::X0 => Rect {
                p00: self.bot.get_dot(C2::P00),
                p10: self.bot.get_dot(C2::P01),
                p01: self.top.get_dot(C2::P00),
                p11: self.top.get_dot(C2::P01),
            },
            CubeFace::X1 => Rect {
                p00: self.bot.get_dot(C2::P10),
                p10: self.bot.get_dot(C2::P11),
                p01: self.top.get_dot(C2::P10),
                p11: self.top.get_dot(C2::P11),
            },
            CubeFace::Y0 => Rect {
                p00: self.bot.get_dot(C2::P00),
                p10: self.bot.get_dot(C2::P10),
                p01: self.top.get_dot(C2::P00),
                p11: self.top.get_dot(C2::P10),
            },
            CubeFace::Y1 => Rect {
                p00: self.bot.get_dot(C2::P01),
                p10: self.bot.get_dot(C2::P11),
                p01: self.top.get_dot(C2::P01),
                p11: self.top.get_dot(C2::P11),
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
                self.get_z_post(C2::P00).link(PostLink::Solid),
                self.get_z_post(C2::P10).link(PostLink::Solid),
                self.get_z_post(C2::P11).link(PostLink::Solid),
                self.get_z_post(C2::P01).link(PostLink::Solid),
            ],
            CuboidLink::Dots => union![
                self.top.link(RectLink::Dots)?,
                self.bot.link(RectLink::Dots)?,
            ],
            CuboidLink::Face(face) => {
                self.get_rect(face).link(RectLink::Solid)?
            }
            CuboidLink::ZPost(corner) => {
                self.get_z_post(corner).link(PostLink::Solid)
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
    pub fn get(&self, upper_or_lower: C1) -> RectShapes {
        match *self {
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
            _ => self.common_rect(),
        }
    }

    fn common_rect(&self) -> RectShapes {
        match *self {
            CuboidShapes::Cube => RectShapes::Cube,
            CuboidShapes::Sphere => RectShapes::Sphere,
            CuboidShapes::Cylinder => RectShapes::Cylinder,
            _ => panic!("custom cuboid shape?"),
        }
    }
}

impl From<Shape> for CuboidShapes {
    fn from(shape: Shape) -> CuboidShapes {
        match shape {
            Shape::Cube => CuboidShapes::Cube,
            Shape::Sphere => CuboidShapes::Sphere,
            Shape::Cylinder => CuboidShapes::Cylinder,
        }
    }
}
