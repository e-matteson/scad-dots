use std::collections::HashSet;

use utils::{map_float, radians_to_degrees, rotate, Axis, Corner3 as C3,
            CubeFace, P2, P3, R3, V2, V3, V4, copy_p3_to};
use errors::{ChainError, RotationError, SnakeError};
use failure::Error;

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

// Cylinders have only basic support, without all the nice features of Dots.
// They should only be used for making discs that are shorter than they're
// diameter.
#[derive(Debug, Clone, Copy)]
pub struct Cylinder {
    pub center_bot_pos: P3,
    pub diameter: f32,
    pub height: f32,
    pub rot: R3,
}

/// Draw a taxicab path between two dots
#[derive(Debug)]
pub struct Snake {
    pub dots: [Dot; 4],
}

#[derive(Debug, Clone, Copy)]
pub enum SnakeLink {
    Chain,
}

#[derive(Debug, Clone)]
pub enum Tree {
    Dot(Dot),
    Cylinder(Cylinder),
    Union(Vec<Tree>),
    Hull(Vec<Tree>),
    Diff(Vec<Tree>),
    Intersect(Vec<Tree>),
    Color(ColorSpec, Vec<Tree>),
    Extrusion(f32, Vec<V2>),
}

#[derive(Debug, Clone, Copy)]
pub enum ColorSpec {
    Red,
    Green,
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

////////////////////////////////////////////////////////////////////////////////

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
        rotate(&self.rot, &axis.into())
    }

    pub fn drop_cylinder(&self, bottom_z: f32) -> Dot {
        // Get the position of the center of the lower dot
        let pos = self.pos(DotAlign::center_solid());
        // Drop its z coordinate.
        let pos = copy_p3_to(pos, bottom_z, Axis::Z);

        // Create a Dot whose bottom face is centered on that position.
        // Reset its rotation.
        Dot::new(
            Shape::Cylinder,
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
            rot: rot * self.rot, // TODO order?
        }
    }

    pub fn translate_to(&self, pos: P3, align: DotAlign) -> Dot {
        // Make a copy of the dot at a new position.
        let spec = DotSpec {
            pos: pos,
            align: align,
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
    pub fn rot_axis(&self) -> Result<V3, Error> {
        unwrap_rot_axis(&self.rot)
    }

    /// Get the dot's angle of rotation in degrees.
    pub fn rot_degs(&self) -> f32 {
        radians_to_degrees(self.rot.angle())
    }

    pub fn pos<T>(&self, align: T) -> P3
    where
        DotAlign: From<T>,
    {
        self.p000 + DotAlign::from(align).offset(self.size, &self.rot)
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
    ) -> Result<[Dot; 4], Error> {
        Ok(Snake::new(*self, other, order)?.dots)
    }
}


impl MapDots for Dot {
    fn map(&self, f: &Fn(&Dot) -> Dot) -> Dot {
        f(self)
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
        self.pos - self.align.offset(self.size, &self.rot)
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

    pub fn offset(&self, dot_size: f32, rot: &R3) -> V3 {
        let dot_spec = dot_size * V3::new(1., 1., 1.);

        let helper = |dot: C3| dot.offset(&dot_spec, rot);

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


impl Snake {
    pub fn new(start: Dot, end: Dot, order: [Axis; 3]) -> Result<Snake, Error> {
        if Snake::has_repeated_axes(&order) {
            return Err(SnakeError.into());
        }
        let mut dots = [Dot::default(); 4];
        dots[0] = start.to_owned();

        // Some dots in the snake may be redundant, and have the same positions.
        // That seems fine.
        for (index, axis) in order.iter().enumerate() {
            dots[index + 1] = dots[index].copy_to_other_dim(end, *axis);
        }
        Ok(Snake { dots: dots })
    }

    fn has_repeated_axes(order: &[Axis; 3]) -> bool {
        let set: HashSet<_> = order.iter().collect();
        set.len() != order.len()
    }

    pub fn link(&self, style: SnakeLink) -> Result<Tree, Error> {
        match style {
            SnakeLink::Chain => chain(&self.dots),
        }
    }
}

#[macro_export]
macro_rules! red {
    ( $( $tree:expr),* $(,)* ) => {
        Tree::Color(ColorSpec::Red, vec![
            $($tree.into(),)*
        ])
    }
}

#[macro_export]
macro_rules! union {
    ( $( $tree:expr),* $(,)* ) => {
        Tree::Union(vec![
            $($tree.into(),)*
        ])
    }
}

#[macro_export]
macro_rules! hull {
    ( $( $tree:expr),* $(,)* ) => {
        Tree::Hull(vec![
            $($tree.into(),)*
        ])
    }
}

#[macro_export]
macro_rules! diff {
    ( $( $tree:expr),* $(,)* ) => {
        Tree::Diff(vec![
            $($tree.into(),)*
        ])
    }
}

#[macro_export]
macro_rules! intersect {
    ( $( $tree:expr),* $(,)* ) => {
        Tree::Intersect(vec![
            $($tree.into(),)*
        ])
    }
}

#[macro_export]
macro_rules! dot {
    ( $tree:expr ) => {
        Tree::Dot($tree.into())
    }
}

impl From<Dot> for Tree {
    fn from(dot: Dot) -> Tree {
        dot![dot]
    }
}

impl From<Cylinder> for Tree {
    fn from(cylinder: Cylinder) -> Tree {
        Tree::Cylinder(cylinder)
    }
}

impl Cylinder {
    pub fn rot_degs(&self) -> f32 {
        radians_to_degrees(self.rot.angle())
    }

    pub fn rot_axis(&self) -> Result<V3, Error> {
        unwrap_rot_axis(&self.rot)
    }
}


// /// Store links between each subsequent pair of things
pub fn chain<T>(things: &[T]) -> Result<Tree, Error>
where
    T: Clone + Into<Tree>,
{
    let segments: Vec<_> = chain_helper(things)?
        .into_iter()
        .map(|(a, b)| hull![a.into(), b.into()])
        .collect();
    Ok(Tree::Union(segments))
}

pub fn chain_loop<T>(things: &[T]) -> Result<Tree, Error>
where
    T: Clone + Into<Tree>,
{
    let mut circular = things.to_owned();
    circular.push(things.get(0).expect("tried to loop empty slice").to_owned());
    chain(&circular)
}

pub fn chain_helper<T>(v: &[T]) -> Result<Vec<(T, T)>, Error>
where
    T: Clone,
{
    let mut i = v.iter().peekable();
    let mut pairs = Vec::new();
    loop {
        let current = match i.next() {
            Some(n) => n.to_owned(),
            None => return Err(ChainError.into()),
        };
        let next = match i.peek() {
            Some(&n) => n.to_owned(),
            None => break,
        };
        pairs.push((current.to_owned(), next.to_owned()));
    }
    Ok(pairs)
}

pub fn mark(pos: P3, size: f32) -> Tree {
    // Put a little sphere at the given position, for debugging
    let s = Dot::new(
        Shape::Sphere,
        DotSpec {
            pos: pos,
            align: DotAlign::center_solid(),
            size: size,
            rot: R3::identity(),
        },
    );
    dot![s]
}

fn unwrap_rot_axis(rot: &R3) -> Result<V3, Error> {
    match rot.axis() {
        Some(unit) => Ok(unit.unwrap()),
        None => {
            if rot.angle() != 0.0 {
                // TODO approx equal
                return Err(RotationError.into());
            }
            // Shouldn't matter what axis we use here, since the angle is 0
            Ok(Axis::Z.into())
        }
    }
}


impl ColorSpec {
    pub fn name(&self) -> String {
        match *self {
            ColorSpec::Red => "red",
            ColorSpec::Green => "green",
        }.to_owned()
    }
    pub fn rgb(&self) -> V3 {
        match *self {
            ColorSpec::Red => V3::new(1., 0., 0.),
            ColorSpec::Green => V3::new(0., 1., 0.),
        }.to_owned()
    }
    pub fn rgba(&self) -> V4 {
        let alpha = 0.5;
        match *self {
            ColorSpec::Red => V4::new(1., 0., 0., alpha),
            ColorSpec::Green => V4::new(0., 1., 0., alpha),
        }.to_owned()
    }
}

impl MinMaxCoord for P2 {
    fn all_coords(&self, axis: Axis) -> Vec<f32> {
        vec![
            match axis {
                Axis::X => self.x,
                Axis::Y => self.y,
                Axis::Z => panic!("P2 has no z coordinate"),
            },
        ]
    }
}

impl MinMaxCoord for P3 {
    fn all_coords(&self, axis: Axis) -> Vec<f32> {
        vec![
            match axis {
                Axis::X => self.x,
                Axis::Y => self.y,
                Axis::Z => self.z,
            },
        ]
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

pub fn extrude_z(height: f32, polygon: &[Dot]) -> Tree {
    let discard_z = |pos: P3| V2::new(pos.x, pos.y);
    let centers: Vec<_> = polygon
        .iter()
        .map(|dot| discard_z(dot.pos(DotAlign::center_solid())))
        .collect();
    Tree::Extrusion(height, centers)
}
