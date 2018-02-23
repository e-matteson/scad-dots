use std::f32;
use std::f32::consts::PI;
use nalgebra::{Point2, Point3, Unit, UnitQuaternion, Vector2, Vector3, Vector4};
pub use nalgebra::distance;

pub use errors::{ArgError, RatioError, RotationError};
use failure::{Error, Fail};

pub type P3 = Point3<f32>;
pub type P2 = Point2<f32>;
pub type V2 = Vector2<f32>;
pub type V3 = Vector3<f32>;
pub type V4 = Vector4<f32>;
pub type R3 = UnitQuaternion<f32>;

const MAX_REL: f32 = 0.0001;

#[derive(Debug, Clone, Copy, Ord, PartialOrd, Eq, PartialEq, Hash)]
pub enum Axis {
    X,
    Y,
    Z,
}

#[derive(Debug, Clone, Copy)]
pub enum Corner1 {
    P0,
    P1,
}

#[derive(Debug, Clone, Copy)]
pub enum Corner2 {
    P00,
    P01,
    P11,
    P10,
}

#[derive(Debug, Clone, Copy)]
pub enum Corner3 {
    P000,
    P010,
    P110,
    P100,
    P001,
    P011,
    P111,
    P101,
}

#[derive(Debug, Clone, Copy)]
pub enum RectEdge {
    X0,
    X1,
    Y0,
    Y1,
}

#[derive(Debug, Clone, Copy)]
pub enum CubeFace {
    X0,
    X1,
    Y0,
    Y1,
    Z0,
    Z1,
}

#[derive(Debug, Clone, Copy)]
pub struct Fraction(f32);

////////////////////////////////////////////////////////////////////////////////

impl Axis {
    pub fn index(&self) -> usize {
        match *self {
            Axis::X => 0,
            Axis::Y => 1,
            Axis::Z => 2,
        }
    }

    pub fn of_p3(&self, pos: P3) -> f32 {
        // TODO refactor stuff to use this
        pos[self.index()]
    }
}

// TODO why did the From<> versions break after the nalgebra 0.13 update?
//  Something to do with the Allocator type param?
//  Into<> works, but is a bit less convenient.

impl Into<V3> for Axis {
    fn into(self) -> V3 {
        match self {
            Axis::X => V3::x_axis().unwrap(),
            Axis::Y => V3::y_axis().unwrap(),
            Axis::Z => V3::z_axis().unwrap(),
        }
    }
}

impl Into<Unit<V3>> for Axis {
    fn into(self) -> Unit<V3> {
        match self {
            Axis::X => V3::x_axis(),
            Axis::Y => V3::y_axis(),
            Axis::Z => V3::z_axis(),
        }
    }
}

impl RectEdge {
    pub fn is_high(&self) -> bool {
        match *self {
            RectEdge::X0 | RectEdge::Y0 => false,
            RectEdge::X1 | RectEdge::Y1 => true,
        }
    }

    pub fn axis(&self) -> Axis {
        match *self {
            RectEdge::X0 | RectEdge::X1 => Axis::X,
            RectEdge::Y0 | RectEdge::Y1 => Axis::Y,
        }
    }

    pub fn is_x(&self) -> bool {
        match *self {
            RectEdge::X0 | RectEdge::X1 => true,
            RectEdge::Y0 | RectEdge::Y1 => false,
        }
    }

    pub fn sign(&self) -> f32 {
        if self.is_high() {
            1.
        } else {
            -1.
        }
    }
}

impl Corner1 {
    pub fn offset(&self, z_length: f32, rot: &R3) -> V3 {
        let v: V3 = (*self).into();
        rotate(rot, &(v * z_length))
    }

    pub fn is_high(&self) -> bool {
        match *self {
            Corner1::P0 => false,
            Corner1::P1 => true,
        }
    }
    pub fn sign(&self) -> f32 {
        if self.is_high() {
            1.
        } else {
            -1.
        }
    }
}

impl Into<V3> for Corner1 {
    fn into(self) -> V3 {
        // Assume the 1 dimension is the z axis
        let x = 0.;
        let y = 0.;
        match self {
            Corner1::P1 => V3::new(x, y, 1.),
            Corner1::P0 => V3::new(x, y, 0.),
        }
    }
}

impl Corner2 {
    pub fn offset(&self, dim_vec: &V3, rot: &R3) -> V3 {
        // TODO share code with Corner3.offset()?
        let v: V3 = self.to_owned().into();
        rotate(rot, &v.component_mul(dim_vec))
    }

    pub fn all_clockwise_from(corner: Corner2) -> Vec<Corner2> {
        let index = match corner {
            // must match order of all()
            Corner2::P00 => 0,
            Corner2::P01 => 1,
            Corner2::P11 => 2,
            Corner2::P10 => 3,
        };
        let mut v = Corner2::all_clockwise();
        v.rotate_left(index);
        v
    }

    pub fn all_clockwise() -> Vec<Corner2> {
        vec![Corner2::P00, Corner2::P01, Corner2::P11, Corner2::P10]
    }

    pub fn is_high(&self, axis: Axis) -> Result<bool, Error> {
        let bools = self.to_bools();
        Ok(match axis {
            Axis::X => bools.0,
            Axis::Y => bools.1,
            Axis::Z => {
                return Err(ArgError
                    .context("The Z value of a Corner2 is not defined")
                    .into())
            }
        })
    }

    pub fn to_c3(&self, z: Corner1) -> Corner3 {
        let c3: Corner3 = (*self).into();
        c3.copy_to(Axis::Z, z.is_high())
    }

    fn to_bools(&self) -> (bool, bool) {
        match *self {
            Corner2::P00 => (false, false),
            Corner2::P01 => (false, true),
            Corner2::P11 => (true, true),
            Corner2::P10 => (true, false),
        }
    }
}

impl Into<V3> for Corner2 {
    fn into(self) -> V3 {
        let z = 0.;
        match self {
            Corner2::P00 => V3::new(0., 0., z),
            Corner2::P01 => V3::new(0., 1., z),
            Corner2::P11 => V3::new(1., 1., z),
            Corner2::P10 => V3::new(1., 0., z),
        }
    }
}

impl Corner3 {
    // TODO come up with better approach than the bool tuples

    pub fn offset(&self, dim_vec: &V3, rot: &R3) -> V3 {
        let v: V3 = self.to_owned().into();
        rotate(rot, &v.component_mul(dim_vec))
    }

    pub fn is_high(&self, axis: Axis) -> bool {
        let bools = self.to_bools();
        match axis {
            Axis::X => bools.0,
            Axis::Y => bools.1,
            Axis::Z => bools.2,
        }
    }

    pub fn copy_to(self, axis: Axis, new_val: bool) -> Self {
        let mut bools = self.to_bools();
        match axis {
            Axis::X => {
                bools.0 = new_val;
            }
            Axis::Y => {
                bools.1 = new_val;
            }
            Axis::Z => {
                bools.2 = new_val;
            }
        }
        Corner3::from_bools(bools)
    }

    pub fn copy_invert(self, axis: Axis) -> Self {
        let current = self.is_high(axis);
        self.copy_to(axis, !current)
    }

    pub fn copy_invert_all_axes(self) -> Self {
        self.copy_invert(Axis::X)
            .copy_invert(Axis::Y)
            .copy_invert(Axis::Z)
    }

    pub fn all() -> Vec<Corner3> {
        vec![
            Corner3::P000,
            Corner3::P010,
            Corner3::P110,
            Corner3::P100,
            Corner3::P001,
            Corner3::P011,
            Corner3::P111,
            Corner3::P101,
        ]
    }

    fn to_bools(&self) -> (bool, bool, bool) {
        match *self {
            Corner3::P000 => (false, false, false),
            Corner3::P010 => (false, true, false),
            Corner3::P110 => (true, true, false),
            Corner3::P100 => (true, false, false),
            Corner3::P001 => (false, false, true),
            Corner3::P011 => (false, true, true),
            Corner3::P111 => (true, true, true),
            Corner3::P101 => (true, false, true),
        }
    }

    fn from_bools(bools: (bool, bool, bool)) -> Corner3 {
        match bools {
            (false, false, false) => Corner3::P000,
            (false, true, false) => Corner3::P010,
            (true, true, false) => Corner3::P110,
            (true, false, false) => Corner3::P100,
            (false, false, true) => Corner3::P001,
            (false, true, true) => Corner3::P011,
            (true, true, true) => Corner3::P111,
            (true, false, true) => Corner3::P101,
        }
    }
}

impl Into<V3> for Corner3 {
    fn into(self) -> V3 {
        match self {
            Corner3::P000 => V3::new(0., 0., 0.),
            Corner3::P010 => V3::new(0., 1., 0.),
            Corner3::P110 => V3::new(1., 1., 0.),
            Corner3::P100 => V3::new(1., 0., 0.),
            Corner3::P001 => V3::new(0., 0., 1.),
            Corner3::P011 => V3::new(0., 1., 1.),
            Corner3::P111 => V3::new(1., 1., 1.),
            Corner3::P101 => V3::new(1., 0., 1.),
        }
    }
}

impl From<Corner2> for Corner3 {
    fn from(corner2: Corner2) -> Self {
        /// Convert up to 3 dimensions, setting Z to zero.
        match corner2 {
            Corner2::P00 => Corner3::P000,
            Corner2::P01 => Corner3::P010,
            Corner2::P11 => Corner3::P110,
            Corner2::P10 => Corner3::P100,
        }
    }
}

impl From<Corner3> for Corner2 {
    fn from(corner3: Corner3) -> Self {
        /// Convert down to 2 dimensions, discarding the Z component
        match corner3 {
            Corner3::P000 | Corner3::P001 => Corner2::P00,
            Corner3::P010 | Corner3::P011 => Corner2::P01,
            Corner3::P110 | Corner3::P111 => Corner2::P11,
            Corner3::P100 | Corner3::P101 => Corner2::P10,
        }
    }
}

impl From<Corner3> for Corner1 {
    /// Convert down to 1 dimensions, discarding the x and y components
    fn from(corner3: Corner3) -> Self {
        if corner3.is_high(Axis::Z) {
            Corner1::P1
        } else {
            Corner1::P0
        }
    }
}

impl CubeFace {
    pub fn is_high(&self) -> bool {
        match *self {
            CubeFace::X0 | CubeFace::Y0 | CubeFace::Z0 => false,
            CubeFace::X1 | CubeFace::Y1 | CubeFace::Z1 => true,
        }
    }

    pub fn axis(&self) -> Axis {
        match *self {
            CubeFace::X0 | CubeFace::X1 => Axis::X,
            CubeFace::Y0 | CubeFace::Y1 => Axis::Y,
            CubeFace::Z0 | CubeFace::Z1 => Axis::Z,
        }
    }
    pub fn corners(&self) -> (Corner3, Corner3) {
        match *self {
            CubeFace::X0 => (Corner3::P000, Corner3::P011),
            CubeFace::X1 => (Corner3::P100, Corner3::P111),
            CubeFace::Y0 => (Corner3::P000, Corner3::P101),
            CubeFace::Y1 => (Corner3::P010, Corner3::P111),
            CubeFace::Z0 => (Corner3::P000, Corner3::P110),
            CubeFace::Z1 => (Corner3::P001, Corner3::P111),
        }
    }

    pub fn all() -> Vec<CubeFace> {
        vec![
            CubeFace::X0,
            CubeFace::Y0,
            CubeFace::Z0,
            CubeFace::X1,
            CubeFace::Y1,
            CubeFace::Z1,
        ]
    }
}

impl Fraction {
    pub fn new(value: f32) -> Result<Fraction, Error> {
        if value < 0. || value > 1. {
            return Err(RatioError(value).into());
            // bail!("invalid ratio: {}", value);
        }
        Ok(Fraction(value))
    }

    pub fn unwrap(&self) -> f32 {
        self.0
    }

    pub fn complement(&self) -> f32 {
        1. - self.unwrap()
    }
}

/// Apply a rotation to a vector. Why doesn't nalgebra give a method for this?
pub fn rotate(rot: &R3, v: &V3) -> V3 {
    rot * v
}

pub fn degrees_between(vector_a: V3, vector_b: V3) -> Result<f32, Error> {
    Ok(radians_to_degrees(
        rotation_between(&vector_a, &vector_b)?.angle(),
    ))
}

/// Convert angle from radians to degrees
pub fn radians_to_degrees(rad: f32) -> f32 {
    rad / PI * 180.0
}

pub fn degrees_to_radians(deg: f32) -> f32 {
    deg / 180.0 * PI
}

/// Create a rotation struct from an axis and an angle
pub fn axis_radians(axis: V3, radians: f32) -> R3 {
    R3::from_axis_angle(&Unit::new_normalize(axis), radians)
}

pub fn axis_degrees(axis: V3, degrees: f32) -> R3 {
    R3::from_axis_angle(&Unit::new_normalize(axis), degrees_to_radians(degrees))
}

pub fn rotation_between(a: &V3, b: &V3) -> Result<R3, Error> {
    R3::rotation_between(a, b).ok_or_else(|| {
        let err: Error = RotationError
            .context("failed to get rotation between vectors")
            .into();
        err
    })
}

pub fn midpoint(a: P3, b: P3) -> P3 {
    weighted_midpoint(Fraction::new(0.5).expect("bad fraction"), a, b)
}

pub fn weighted_midpoint(weight_a: Fraction, a: P3, b: P3) -> P3 {
    // TODO there must be a better way to convert P3 to V3, right?
    let vec_b = b - P3::origin();
    a * weight_a.unwrap() + vec_b * weight_a.complement()
}

pub fn weighted_average(weight_a: Fraction, a: f32, b: f32) -> f32 {
    a * weight_a.unwrap() + b * weight_a.complement()
}

pub fn copy_p3_to(pos: P3, coord: f32, axis: Axis) -> P3 {
    let mut new_pos = pos;
    new_pos[axis.index()] = coord;
    new_pos
}

pub fn copy_p3_to_other_dim(self_pos: P3, other_pos: P3, axis: Axis) -> P3 {
    let mut new_pos = self_pos;
    new_pos[axis.index()] = other_pos[axis.index()];
    new_pos
}

pub fn translate_p3(pos: P3, dist: f32, axis: Axis) -> P3 {
    let mut new_pos = pos;
    new_pos[axis.index()] += dist;
    new_pos
}

/// Translate the point along the given direction vector, until the given
/// axis has the given value.
pub fn translate_p3_along_until(
    pos: P3,
    direction: V3,
    axis: Axis,
    axis_value: f32,
) -> P3 {
    // TODO use consistent arg order with translate_p3
    let i = axis.index();
    let m = (axis_value - pos[i]) / direction[i];
    pos + m * direction
}

pub fn get_plane_normal(origin: P3, end1: P3, end2: P3) -> V3 {
    (end1 - origin).cross(&(end2 - origin))
}

pub fn map_float(f: fn(f32, f32) -> f32, floats: Vec<f32>) -> f32 {
    // TODO does this make sense for anything other than min and max?
    // floats.into_iter().fold(0. / 0., f)
    floats.into_iter().fold(f32::NAN, f)
}

pub fn min_v3_coord(v: V3) -> f32 {
    // TODO cleaner way to do this?
    map_float(f32::min, vec![v.x, v.y, v.z])
}

// fn translation(point1: P3, point2: P3, distance: f32) -> V3 {
//     let unit_vec = (point2 - point1).normalize();
//     unit_vec * distance
// }

pub fn sin_deg(degrees: f32) -> f32 {
    f32::sin(degrees_to_radians(degrees))
}

pub fn cos_deg(degrees: f32) -> f32 {
    f32::cos(degrees_to_radians(degrees))
}

pub fn relative_less_eq(a: f32, b: f32) -> bool {
    a < b || relative_eq!(a, b, max_relative = MAX_REL)
}
pub fn relative_less(a: f32, b: f32) -> bool {
    a < b && !relative_eq!(a, b, max_relative = MAX_REL)
}
