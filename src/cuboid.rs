use core::utils::{
    midpoint, Axis, Corner1 as C1, Corner2 as C2, Corner3 as C3, CubeFace,
    Fraction, P3, R3, V3,
};
use core::{mark, Dot, MapDots, MinMaxCoord, Shape, Tree};
use errors::ScadDotsError;
use post::{Post, PostLink};
use rect::{Rect, RectAlign, RectLink, RectShapes, RectSpec, RectSpecBasic};

#[derive(Debug, Clone, Copy, MapDots, MinMaxCoord)]
/// A cuboid (box) is made of 2 rects, one above the other
pub struct Cuboid {
    pub top: Rect,
    pub bot: Rect,
}

#[derive(Debug, Clone, Copy)]
pub struct CuboidSpecBasic {
    pub pos: P3,
    pub align: CuboidAlign,
    pub x_dim: f32,
    pub y_dim: f32,
    pub z_dim: f32,
    pub size: f32,
    pub rot: R3,
}

#[derive(Debug, Clone, Copy)]
pub struct CuboidSpecChamferZHole {
    pub pos: P3,
    pub align: CuboidAlign,
    pub x_dim: f32,
    pub y_dim: f32,
    pub z_dim: f32,
    pub chamfer: Fraction,
    pub rot: R3,
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

pub trait CuboidSpec: Copy + Sized {
    type C: CuboidSpecToRect;
    // TODO rename
    fn into_convertable(self) -> Result<Self::C, ScadDotsError>;
}

pub trait CuboidSpecToRect: Copy {
    type R: RectSpec;
    fn to_rect_spec(&self, z_val: C1) -> Result<Self::R, ScadDotsError>;
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

    pub fn center_solid() -> CuboidAlign {
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
    pub fn new<T>(
        shapes: CuboidShapes,
        spec: T,
    ) -> Result<Cuboid, ScadDotsError>
    where
        T: CuboidSpec,
    {
        // TODO check whether size is larger than any of the dimensions
        let spec = spec.into_convertable()?;
        let bot = Rect::new(shapes.get(C1::P0), spec.to_rect_spec(C1::P0)?)?;
        let top = Rect::new(shapes.get(C1::P1), spec.to_rect_spec(C1::P1)?)?;
        Ok(Cuboid { bot: bot, top: top })
    }

    pub fn from_dot(
        dot: Dot,
        size: f32,
        shape: Shape,
    ) -> Result<Cuboid, ScadDotsError> {
        if let Shape::Cube = dot.shape {
        } else {
            return Err(ScadDotsError::Args
                .context("Cuboid can only be created from a cube-shaped dot"));
        }
        let d = CuboidSpecBasic {
            pos: dot.p000,
            align: CuboidAlign::origin(),
            x_dim: dot.size,
            y_dim: dot.size,
            z_dim: dot.size,
            size: size,
            rot: dot.rot,
        };
        Cuboid::new(shape.into(), d)
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

impl CuboidSpecToRect for CuboidSpecBasic {
    type R = RectSpecBasic;

    fn to_rect_spec(&self, z_val: C1) -> Result<RectSpecBasic, ScadDotsError> {
        let dot_lengths = V3::new(self.size, self.size, self.size);
        let cuboid_lengths = V3::new(
            self.x_dim - self.size,
            self.y_dim - self.size,
            self.z_dim - self.size,
        );
        let origin =
            self.pos - self.align.offset(cuboid_lengths, dot_lengths, self.rot);
        let pos = origin + z_val.offset(cuboid_lengths.z, self.rot);
        Ok(RectSpecBasic {
            pos: pos,
            align: RectAlign::origin(),
            y_dim: self.y_dim,
            x_dim: self.x_dim,
            size: self.size,
            rot: self.rot,
        })
    }
}

impl CuboidSpec for CuboidSpecBasic {
    type C = CuboidSpecBasic;

    fn into_convertable(self) -> Result<CuboidSpecBasic, ScadDotsError> {
        Ok(self)
    }
}

impl CuboidSpec for CuboidSpecChamferZHole {
    type C = CuboidSpecBasic;

    fn into_convertable(self) -> Result<CuboidSpecBasic, ScadDotsError> {
        // TODO the dot size might be larger than the z dimension!
        // This is ok if you only ever use it as a hole punched through a wall with thickness z.
        // But bad if you try to do fancier things!
        // TODO write fancier chamfer feature that does this right!
        Ok(CuboidSpecBasic {
            pos: self.pos,
            align: self.align,
            x_dim: self.x_dim,
            y_dim: self.y_dim,
            z_dim: self.z_dim,
            size: self.chamfer.unwrap() * self.x_dim.min(self.y_dim) / 2.,
            rot: self.rot,
        })
    }
}

impl CuboidShapes {
    pub fn get(&self, z_val: C1) -> RectShapes {
        match *self {
            CuboidShapes::Round => match z_val {
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
            } => match z_val {
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
