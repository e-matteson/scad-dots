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
pub struct RectSpecBasic {
    pub pos: P3,
    pub align: RectAlign,
    pub x_dim: f32,
    pub y_dim: f32,
    pub size: f32,
    pub rot: R3,
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

#[derive(Debug, Clone, Copy)]
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

// When you create a new way of specifying dimensions, you must EITHER:
//  a) Impl RectSpecToDot (and have RectSpec just return itself)
//  b) Impl RectSpec, turning itself into a different RectSpec type which has
//      already implemented RectSpecToDot
pub trait RectSpecToDot: RectSpec {
    fn to_dot_spec(&self, corner: C2) -> Result<DotSpec, ScadDotsError>;
}

pub trait RectSpec: Copy + Sized {
    type T: RectSpecToDot;
    // TODO rename
    fn into_convertable(self) -> Result<Self::T, ScadDotsError>;
}

////////////////////////////////////////////////////////////////////////////////

impl Rect {
    pub fn new<T>(shapes: RectShapes, spec: T) -> Result<Rect, ScadDotsError>
    where
        T: RectSpec,
    {
        let spec = spec.into_convertable()?;
        // TODO check dimensions, ensure positive
        Ok(Rect {
            p00: Rect::make_dot(C2::P00, shapes, spec)?,
            p01: Rect::make_dot(C2::P01, shapes, spec)?,
            p10: Rect::make_dot(C2::P10, shapes, spec)?,
            p11: Rect::make_dot(C2::P11, shapes, spec)?,
        })
    }

    fn make_dot<T>(
        corner: C2,
        shapes: RectShapes,
        spec: T,
    ) -> Result<Dot, ScadDotsError>
    where
        T: RectSpecToDot,
    {
        Ok(Dot::new(shapes.get(corner), spec.to_dot_spec(corner)?))
    }

    // pub fn copy_translate(&self, offset: &V3) -> Rect {
    //     Rect {
    //         p00: self.p00.copy_translate(offset),
    //         p01: self.p01.copy_translate(offset),
    //         p11: self.p11.copy_translate(offset),
    //         p10: self.p10.copy_translate(offset),
    //     }
    // }

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
        self.get_dot(rect).pos(dot)
    }

    pub fn get_dot(&self, corner: C2) -> Dot {
        match corner {
            C2::P00 => self.p00,
            C2::P10 => self.p10,
            C2::P01 => self.p01,
            C2::P11 => self.p11,
        }
    }

    pub fn dim_vec(&self, axis: Axis) -> V3 {
        let point1 = self.pos(RectAlign::origin());
        let point2 = match axis {
            Axis::X => self.pos(RectAlign::outside(C3::P100)),
            Axis::Y => self.pos(RectAlign::outside(C3::P010)),
            Axis::Z => self.pos(RectAlign::outside(C3::P001)),
        };
        point2 - point1
    }

    pub fn dim_unit_vec(&self, axis: Axis) -> V3 {
        self.dim_vec(axis).normalize()
    }

    pub fn dim_len(&self, axis: Axis) -> f32 {
        self.dim_vec(axis).norm()
    }

    pub fn drop_solid(&self, bottom_z: f32, shape: Option<Shape>) -> Tree {
        drop_solid(&self.dots(), bottom_z, shape)
    }

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
                self.get_dot(C2::P00),
                self.get_dot(C2::P01),
                self.get_dot(C2::P11),
                self.get_dot(C2::P10),
            ])?,
            RectLink::YPosts => union![
                hull![self.get_dot(C2::P00), self.get_dot(C2::P01)],
                hull![self.get_dot(C2::P10), self.get_dot(C2::P11)],
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
            .map(|c| self.get_dot(c))
            .collect()
    }
}

impl RectSpecBasic {
    pub fn inner_y_dim(&self) -> f32 {
        self.y_dim - 2. * self.size
    }

    pub fn inner_x_dim(&self) -> f32 {
        self.x_dim - 2. * self.size
    }

    pub fn with_pos(self, new_pos: P3) -> Self {
        RectSpecBasic {
            pos: new_pos,
            align: self.align,
            x_dim: self.x_dim,
            y_dim: self.y_dim,
            size: self.size,
            rot: self.rot,
        }
    }
    pub fn with_align(self, new_align: RectAlign) -> Self {
        RectSpecBasic {
            pos: self.pos,
            align: new_align,
            x_dim: self.x_dim,
            y_dim: self.y_dim,
            size: self.size,
            rot: self.rot,
        }
    }
    pub fn with_rot(self, new_rot: R3) -> Self {
        RectSpecBasic {
            pos: self.pos,
            align: self.align,
            x_dim: self.x_dim,
            y_dim: self.y_dim,
            size: self.size,
            rot: new_rot,
        }
    }
}

impl RectSpecToDot for RectSpecBasic {
    fn to_dot_spec(&self, corner: C2) -> Result<DotSpec, ScadDotsError> {
        let dot_lengths = V3::new(self.size, self.size, self.size);
        let rect_lengths =
            V3::new(self.x_dim - self.size, self.y_dim - self.size, 0.);
        let origin =
            self.pos - self.align.offset(dot_lengths, rect_lengths, self.rot);

        let pos = origin + corner.offset(rect_lengths, self.rot);

        Ok(DotSpec {
            pos: pos,
            align: C3::P000.into(),
            rot: self.rot,
            size: self.size,
        })
    }
}

impl RectSpec for RectSpecBasic {
    type T = RectSpecBasic;
    fn into_convertable(self) -> Result<RectSpecBasic, ScadDotsError> {
        Ok(self)
    }
}

impl RectAlign {
    pub fn origin() -> RectAlign {
        RectAlign::outside(C3::P000)
    }

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

    pub fn outside_midpoint(a: C3, b: C3) -> RectAlign {
        RectAlign::midpoint(RectAlign::outside(a), RectAlign::outside(b))
            .expect("bug in outside_midpoint()")
    }

    pub fn inside_midpoint(a: C3, b: C3) -> RectAlign {
        // Return the midpoint between the two inner corners a and b
        RectAlign::midpoint(RectAlign::inside(a), RectAlign::inside(b))
            .expect("bug in inside_midpoint()")
    }

    pub fn outside(corner: C3) -> RectAlign {
        RectAlign::Corner {
            dot: corner,
            rect: corner.into(),
        }
    }

    pub fn inside(corner: C3) -> RectAlign {
        // TODO this is not quite analagous to CuboidAlign::inside(), because
        // it's on either the top of bottom surface. Does that matter? Should
        // there be a cuboid one for inside of top/bottom surface?
        RectAlign::Corner {
            dot: corner.copy_invert(Axis::X).copy_invert(Axis::Y),
            rect: corner.into(),
        }
    }

    pub fn center_solid() -> RectAlign {
        RectAlign::midpoint(
            RectAlign::outside(C3::P000),
            RectAlign::outside(C3::P111),
        ).expect("bad args to midpoint calculation")
    }

    pub fn center_face(face: CubeFace) -> RectAlign {
        let (a, b) = face.corners();
        RectAlign::midpoint(RectAlign::outside(a), RectAlign::outside(b))
            .expect("got bad corners from CubeFace")
    }

    pub fn all_corners() -> Vec<RectAlign> {
        let mut v = Vec::new();
        for d in C3::all() {
            for r in C2::all_clockwise() {
                v.push(RectAlign::Corner { dot: d, rect: r });
            }
        }
        v
    }

    pub fn offset(&self, dot_dim_vec: V3, rect_dim_vec: V3, rot: R3) -> V3 {
        let helper = |dot: C3, rect: C2| {
            dot.offset(dot_dim_vec, rot) + rect.offset(rect_dim_vec, rot)
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
            _ => self.common(),
        }
    }

    fn common(&self) -> Shape {
        match *self {
            RectShapes::Cube => Shape::Cube,
            RectShapes::Sphere => Shape::Sphere,
            RectShapes::Cylinder => Shape::Cylinder,
            _ => panic!("custom rect shape?"),
        }
    }
}
