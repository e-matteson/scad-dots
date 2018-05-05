use utils::{axis_degrees, rotate, rotation_between, sin_deg, Axis, CubeFace,
            P3, R3, V3};
use core::{mark, Dot, DotAlign, DotSpec, MapDots, MinMaxCoord, Shape, Tree};

use failure::Error;

#[derive(Debug, Clone, Copy, MapDots, MinMaxCoord)]
pub struct Triangle {
    pub a: Dot,
    pub b: Dot,
    pub c: Dot,
}

#[derive(Debug, Clone, Copy)]
pub struct TriangleSpec {
    pub deg_b: f32,
    pub len_bc: f32,
    pub deg_c: f32,
    pub size: f32,
    pub point_b: P3,
    pub rot: R3,
}

#[derive(Debug, Clone, Copy)]
pub enum TriCorner {
    A,
    B,
    C,
}

////////////////////////////////////////////////////////////////////////////////

impl Triangle {
    pub fn new(spec: TriangleSpec) -> Result<Triangle, Error> {
        let shape = Shape::Cylinder;
        let b_spec = DotSpec {
            pos: spec.center(TriCorner::B),
            align: DotAlign::center_face(CubeFace::Z0),
            size: spec.size,
            rot: spec.rot,
        };
        let b = Dot::new(shape, b_spec);
        let a_spec = DotSpec {
            pos: spec.center(TriCorner::A),
            align: DotAlign::center_face(CubeFace::Z0),
            size: spec.size,
            rot: spec.rot,
        };

        let a = Dot::new(shape, a_spec);

        let c_spec = DotSpec {
            pos: spec.center(TriCorner::C),
            align: DotAlign::center_face(CubeFace::Z0),
            size: spec.size,
            rot: spec.rot,
        };
        let c = Dot::new(shape, c_spec);

        // print!("ca: {}, ab: {}", spec.len(TriCorner::C), spec.len_ab());

        Ok(Triangle { a: a, b: b, c: c })
    }

    pub fn mark(&self, spec: TriangleSpec) -> Result<Tree, Error> {
        Ok(union![
            mark(spec.point(TriCorner::A), 1.),
            mark(spec.point(TriCorner::B), 1.),
            mark(spec.point(TriCorner::C), 1.),
        ])
    }

    pub fn link(&self) -> Result<Tree, Error> {
        Ok(hull![self.a, self.b, self.c])
    }
}

impl TriangleSpec {
    fn center(&self, v: TriCorner) -> P3 {
        self.point(v) + self.unit_to_center(v) * self.dist_to_center(v)
    }

    fn unit_to_center(&self, vertex: TriCorner) -> V3 {
        let (v1, v2) = match vertex {
            TriCorner::C => (TriCorner::C, TriCorner::A),
            TriCorner::A => (TriCorner::A, TriCorner::B),
            TriCorner::B => (TriCorner::B, TriCorner::C),
        };
        let half_angle = self.deg(vertex) / 2.;
        self.rot_z(half_angle, self.unit_side(v1, v2))
    }

    fn dist_to_center(&self, vertex: TriCorner) -> f32 {
        self.size / (2. * sin_deg(self.deg(vertex) / 2.))
    }

    pub fn point(&self, vertex: TriCorner) -> P3 {
        match vertex {
            TriCorner::B => self.point_b,
            TriCorner::A => {
                self.point_b + self.side(TriCorner::B, TriCorner::A)
            }
            TriCorner::C => {
                self.point_b + self.side(TriCorner::B, TriCorner::C)
            }
        }
    }

    pub fn deg(&self, vertex: TriCorner) -> f32 {
        match vertex {
            TriCorner::A => 180. - self.deg_c - self.deg_b,
            TriCorner::B => self.deg_b,
            TriCorner::C => self.deg_c,
        }
    }

    pub fn side(&self, v1: TriCorner, v2: TriCorner) -> V3 {
        self.unit_side(v1, v2) * self.len(v1, v2)
    }

    pub fn len(&self, v1: TriCorner, v2: TriCorner) -> f32 {
        self.len_opposite(opposite(v1, v2))
    }

    fn len_opposite(&self, vertex: TriCorner) -> f32 {
        self.len_bc / sin_deg(self.deg(TriCorner::A))
            * sin_deg(self.deg(vertex))
    }

    fn unit(&self, axis: Axis) -> V3 {
        rotate(&self.rot, &axis.into())
    }

    fn rot_z(&self, degrees: f32, vec: V3) -> V3 {
        rotate(&axis_degrees(self.unit(Axis::Z), degrees), &vec)
    }

    pub fn rot_from_x(
        &self,
        c1: TriCorner,
        c2: TriCorner,
    ) -> Result<R3, Error> {
        rotation_between(&self.unit(Axis::X), &self.unit_side(c1, c2))
    }

    pub fn unit_side(&self, v1: TriCorner, v2: TriCorner) -> V3 {
        let reverse = || -1. * self.unit_side(v2, v1);

        match (v1, v2) {
            (TriCorner::B, TriCorner::A) => {
                self.rot_z(self.deg(TriCorner::B), self.unit(Axis::X))
            }
            (TriCorner::C, TriCorner::A) => self.rot_z(
                -1. * self.deg(TriCorner::C),
                -1. * self.unit(Axis::X),
            ),
            (TriCorner::B, TriCorner::C) => self.unit(Axis::X),
            (TriCorner::A, TriCorner::B)
            | (TriCorner::A, TriCorner::C)
            | (TriCorner::C, TriCorner::B) => reverse(),
            _ => panic!("not a valid triangle side: identical vertices"),
        }
    }
}

fn opposite(v1: TriCorner, v2: TriCorner) -> TriCorner {
    match (v1, v2) {
        (TriCorner::A, TriCorner::B) | (TriCorner::B, TriCorner::A) => {
            TriCorner::C
        }
        (TriCorner::A, TriCorner::C) | (TriCorner::C, TriCorner::A) => {
            TriCorner::B
        }
        (TriCorner::B, TriCorner::C) | (TriCorner::C, TriCorner::B) => {
            TriCorner::A
        }
        _ => panic!("not a valid triangle side"),
    }
}
