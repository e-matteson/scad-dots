use std::f32::consts::PI;

use core::utils::{
    axis_radians, map_float, radial_offset, radians_to_degrees, rotate,
    translate_p3_along_until, unwrap_rot_axis, Axis, Corner3 as C3, CubeFace,
    P2, P3, R3, V3,
};

use core::{Snake, Tree};
use errors::ScadDotsError;

/// The smallest building block of the 3d model.
#[derive(Debug, Clone, Copy)]
pub struct Dot {
    pub shape: Shape,
    pub p000: P3,
    pub size: f32,
    pub rot: R3,
}

#[derive(Debug, Clone, Copy)]
pub struct DotSpec {
    pub pos: P3,
    pub align: DotAlign,
    pub size: f32,
    pub rot: R3,
}

#[derive(Debug, Copy, Clone)]
pub enum DotAlign {
    Corner(C3),
    Midpoint(C3, C3),
}

/// The possible shapes of a dot
#[derive(Debug, Clone, Copy)]
pub enum Shape {
    Cube,
    Sphere,
    Cylinder,
}

pub trait MapDots: Sized {
    // Can be derived using pipit-3d-derive crate
    fn map(&self, f: &Fn(&Dot) -> Dot) -> Self;

    fn map_translate(&self, offset: V3) -> Self {
        self.map(&|d: &Dot| d.translate(offset))
    }

    fn map_translate_z(&self, z_offset: f32) -> Self {
        self.map_translate(V3::new(0., 0., z_offset))
    }

    fn map_rotate(&self, rot: R3) -> Self {
        self.map(&|d: &Dot| d.rotate(rot))
    }
}
pub trait MinMaxCoord {
    // Can be derived using pipit-3d-derive crate
    fn all_coords(&self, axis: Axis) -> Vec<f32>;

    fn max_coord(&self, axis: Axis) -> f32 {
        Self::map_float(f32::max, self.all_coords(axis))
    }

    fn min_coord(&self, axis: Axis) -> f32 {
        Self::map_float(f32::min, self.all_coords(axis))
    }

    fn less_than<T>(&self, other: &T, axis: Axis) -> bool
    where
        T: MinMaxCoord,
    {
        self.min_coord(axis) < other.min_coord(axis)
    }

    fn greater_than<T>(&self, other: &T, axis: Axis) -> bool
    where
        T: MinMaxCoord,
    {
        self.max_coord(axis) > other.max_coord(axis)
    }

    fn bound_length(&self, axis: Axis) -> f32 {
        self.max_coord(axis) - self.min_coord(axis)
    }

    fn midpoint(&self, axis: Axis) -> f32 {
        0.5 * (self.max_coord(axis) + self.min_coord(axis))
    }

    fn midpoint2(&self) -> P2 {
        P2::new(self.midpoint(Axis::X), self.midpoint(Axis::Y))
    }

    fn midpoint3(&self) -> P3 {
        P3::new(
            self.midpoint(Axis::X),
            self.midpoint(Axis::Y),
            self.midpoint(Axis::Z),
        )
    }

    fn map_float(f: fn(f32, f32) -> f32, floats: Vec<f32>) -> f32 {
        // Use the version from the core
        map_float(f, floats)
    }
}

impl Dot {
    /// Create a new dot.
    pub fn new(shape: Shape, spec: DotSpec) -> Dot {
        Dot {
            shape: shape,
            p000: spec.origin(),
            size: spec.size,
            rot: spec.rot,
        }
    }

    pub fn dim_unit_vec(&self, axis: Axis) -> V3 {
        rotate(self.rot, axis)
    }

    /// Create a dot centered under the given dot, with its bottom surface at
    /// the given Z height. It will sit flat on the z-plane at the default
    /// rotation, regardless of the rotation of the original dot. It will have
    /// the same size as the original dot. If shape is None, it will have the
    /// same shape as the original.
    pub fn drop(&self, bottom_z: f32, shape: Option<Shape>) -> Dot {
        self.drop_along(Axis::Z.into(), bottom_z, shape)
    }

    /// Like `Dot::drop()`, but more general: the new dot will be dropped in the
    /// given direction, instead of straight down. The new dot's rotation will
    /// still be reset to sit flat on the z-plane, regardless of direction.
    pub fn drop_along(
        &self,
        direction: V3,
        bottom_z: f32,
        shape: Option<Shape>,
    ) -> Dot {
        // Get the position of the center of the dot
        let pos = translate_p3_along_until(
            self.pos(DotAlign::center_solid()),
            direction,
            Axis::Z,
            bottom_z,
        );

        // Create a Dot whose bottom face is centered on that position.
        // Reset its rotation.
        Dot::new(
            shape.unwrap_or(self.shape),
            DotSpec {
                pos: pos,
                align: DotAlign::center_face(CubeFace::Z0),
                size: self.size,
                rot: R3::identity(),
            },
        )
    }

    pub fn translate(&self, offset: V3) -> Dot {
        Dot {
            shape: self.shape,
            p000: self.p000 + offset,
            size: self.size,
            rot: self.rot,
        }
    }

    pub fn rotate(&self, rot: R3) -> Dot {
        Dot {
            shape: self.shape,
            p000: rot * self.p000,
            size: self.size,
            rot: rot * self.rot,
        }
    }

    pub fn rotate_to(&self, new_rot: R3) -> Dot {
        // TODO check
        let rot_difference = self.rot.rotation_to(&new_rot);
        self.rotate(rot_difference)
    }

    /// Make a copy of the dot at the new position.
    pub fn translate_to(&self, pos: P3, align: DotAlign) -> Dot {
        let spec = DotSpec {
            pos: pos,
            align: align,
            size: self.size,
            rot: self.rot,
        };
        Dot::new(self.shape, spec)
    }

    /// Translate the dot along the given direction vector, until the part of
    /// the dot specified by `align` has the given coordinate value.
    pub fn translate_along_until<T>(
        &self,
        direction: V3,
        axis: Axis,
        axis_value: f32,
        align: T,
    ) -> Dot
    where
        T: Clone,
        DotAlign: From<T>,
    {
        let start_pos = self.pos(align.clone());
        let spec = DotSpec {
            pos: translate_p3_along_until(
                start_pos, direction, axis, axis_value,
            ),
            align: align.into(),
            size: self.size,
            rot: self.rot,
        };
        Dot::new(self.shape, spec)
    }

    pub fn with_coord(&self, coordinate: f32, dimension: Axis) -> Dot {
        let mut new = *self;
        new.p000[dimension.index()] = coordinate;
        new
    }

    pub fn copy_to_other_dim(&self, other: Dot, dimension: Axis) -> Dot {
        let mut new = *self;
        new.p000[dimension.index()] = other.p000[dimension.index()];
        new
    }

    pub fn with_shape(&self, new_shape: Shape) -> Dot {
        let mut new = *self;
        new.shape = new_shape;
        new
    }

    /// Get the dot's axis of rotation.
    pub fn rot_axis(&self) -> Result<V3, ScadDotsError> {
        unwrap_rot_axis(self.rot)
    }

    /// Get the dot's angle of rotation in degrees.
    pub fn rot_degs(&self) -> f32 {
        radians_to_degrees(self.rot.angle())
    }

    pub fn pos<T>(&self, align: T) -> P3
    where
        DotAlign: From<T>,
    {
        self.p000 + DotAlign::from(align).offset(self.size, self.rot)
    }

    /// Get distance between the origins of the dots.
    /// Does NOT calculate actual minimum distance between their surfaces!
    pub fn dist(&self, other: Dot) -> f32 {
        (self.p000 - other.p000).norm()
    }

    pub fn less_than(&self, other: Dot, axis: Axis) -> bool {
        // self.p000[axis.index()] < other.p000[axis.index()]
        self.min_coord(axis) < other.min_coord(axis)
    }

    pub fn snake(
        &self,
        other: Dot,
        order: [Axis; 3],
    ) -> Result<[Dot; 4], ScadDotsError> {
        Ok(Snake::new(*self, other, order)?.dots)
    }

    pub fn explode_radially(
        &self,
        radius: f32,
        axis: Option<V3>,
        count: usize,
        adjust_dot_rotations: bool,
    ) -> Result<Vec<Dot>, ScadDotsError> {
        let axis = axis.unwrap_or(rotate(self.rot, Axis::Z));

        let mut dots = Vec::new();
        for i in 0..count {
            let radians = (i as f32) / (count as f32) * 2. * PI;
            let offset = radial_offset(radians, radius, axis)?;

            let rot = if adjust_dot_rotations {
                axis_radians(axis, radians) * self.rot
            } else {
                self.rot
            };

            let new = Dot::new(
                self.shape,
                DotSpec {
                    pos: self.pos(DotAlign::center_solid()) + offset,
                    align: DotAlign::center_solid(),
                    size: self.size,
                    rot: rot,
                },
            );

            dots.push(new)
        }
        Ok(dots)
    }
}

//  TODO impl default for dotspec and shape, derive for dot
impl Default for Dot {
    fn default() -> Dot {
        Dot::new(
            Shape::Cube,
            DotSpec {
                pos: P3::origin(),
                align: DotAlign::Corner(C3::P000),
                size: 1.,
                rot: R3::identity(),
            },
        )
    }
}

impl DotSpec {
    pub fn origin(&self) -> P3 {
        self.pos - self.align.offset(self.size, self.rot)
    }
}

impl DotAlign {
    pub fn origin() -> DotAlign {
        C3::P000.into()
    }

    pub fn center_solid() -> DotAlign {
        DotAlign::Midpoint(C3::P000, C3::P111)
    }

    pub fn center_face(face: CubeFace) -> DotAlign {
        let (a, b) = face.corners();
        DotAlign::Midpoint(a, b)
    }

    pub fn offset(&self, dot_size: f32, rot: R3) -> V3 {
        let dot_spec = dot_size * V3::new(1., 1., 1.);

        let helper = |dot: C3| dot.offset(dot_spec, rot);

        match *self {
            DotAlign::Corner(a) => helper(a),
            DotAlign::Midpoint(a, b) => (helper(a) + helper(b)) / 2.,
        }
    }
}

impl From<C3> for DotAlign {
    fn from(corner3: C3) -> Self {
        DotAlign::Corner(corner3)
    }
}

impl MapDots for Dot {
    fn map(&self, f: &Fn(&Dot) -> Dot) -> Dot {
        f(self)
    }
}

impl<T> MapDots for [T; 4]
where
    T: MapDots,
{
    fn map(&self, f: &Fn(&Dot) -> Dot) -> Self {
        [
            self[0].map(f),
            self[1].map(f),
            self[2].map(f),
            self[3].map(f),
        ]
    }
}

impl MinMaxCoord for Dot {
    fn all_coords(&self, axis: Axis) -> Vec<f32> {
        C3::all()
            .into_iter()
            .map(|corner| self.pos(corner)[axis.index()])
            .collect()
    }
}

impl MinMaxCoord for P2 {
    fn all_coords(&self, axis: Axis) -> Vec<f32> {
        vec![match axis {
            Axis::X => self.x,
            Axis::Y => self.y,
            Axis::Z => panic!("P2 has no z coordinate"),
        }]
    }
}

impl MinMaxCoord for P3 {
    fn all_coords(&self, axis: Axis) -> Vec<f32> {
        vec![match axis {
            Axis::X => self.x,
            Axis::Y => self.y,
            Axis::Z => self.z,
        }]
    }
}

impl<S, T> MinMaxCoord for (S, T)
where
    S: MinMaxCoord,
    T: MinMaxCoord,
{
    fn all_coords(&self, axis: Axis) -> Vec<f32> {
        let mut v = self.0.all_coords(axis);
        v.extend(self.1.all_coords(axis));
        v
    }
}

impl<T> MinMaxCoord for Vec<T>
where
    T: MinMaxCoord,
{
    fn all_coords(&self, axis: Axis) -> Vec<f32> {
        let mut v = Vec::new();
        for thing in self.iter() {
            v.extend(thing.all_coords(axis))
        }
        v
    }
}

impl<T> MinMaxCoord for [T; 3]
where
    T: MinMaxCoord,
{
    fn all_coords(&self, axis: Axis) -> Vec<f32> {
        let mut v = self[0].all_coords(axis);
        v.extend(self[1].all_coords(axis));
        v.extend(self[2].all_coords(axis));
        v
    }
}

impl<T> MinMaxCoord for [T; 4]
where
    T: MinMaxCoord,
{
    fn all_coords(&self, axis: Axis) -> Vec<f32> {
        let mut v = self[0].all_coords(axis);
        v.extend(self[1].all_coords(axis));
        v.extend(self[2].all_coords(axis));
        v.extend(self[3].all_coords(axis));
        v
    }
}

pub fn mark(pos: P3, size: f32) -> Tree {
    // Put a little sphere at the given position, for debugging
    // TODO make it red
    Dot::new(
        Shape::Sphere,
        DotSpec {
            pos: pos,
            align: DotAlign::center_solid(),
            size: size,
            rot: R3::identity(),
        },
    ).into()
}
