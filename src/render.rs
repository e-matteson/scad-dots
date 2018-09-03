use scad_generator::*;

use core::utils::{
    radians_to_degrees, rotate, unwrap_rot_axis, Corner3 as C3, P2, P3, V2, V3,
};
use core::{
    Cylinder, Dot, DotShape, Extrusion, Tree, TreeObject, TreeOperator,
};
use errors::{ResultExt, ScadDotsError};

pub trait Render {
    fn render(
        &self,
        options: RenderQuality,
    ) -> Result<ScadObject, ScadDotsError>;
}

#[derive(Debug, Clone, Copy)]
pub enum RenderQuality {
    // Default,
    Low,
    Medium,
    High,
}

impl RenderQuality {
    pub fn detail(&self) -> i32 {
        match *self {
            RenderQuality::Medium => 20,
            RenderQuality::High => 60,
            RenderQuality::Low => 5,
        }
    }
}

pub fn to_file<T>(
    thing: &T,
    path: String,
    options: RenderQuality,
) -> Result<(), ScadDotsError>
where
    T: Render,
{
    let scad_file = make_scad_file(thing, options)?;
    scad_file.write_to_file(path);
    Ok(())
}

pub fn to_code<T>(
    thing: &T,
    options: RenderQuality,
) -> Result<String, ScadDotsError>
where
    T: Render,
{
    let scad_file = make_scad_file(thing, options)?;
    Ok(scad_file.get_code())
}

fn make_scad_file<T>(
    thing: &T,
    options: RenderQuality,
) -> Result<ScadFile, ScadDotsError>
where
    T: Render,
{
    let mut scad_file = ScadFile::new();
    // detail controls resolution of curves
    scad_file.set_detail(options.detail());
    scad_file
        .add_object(thing.render(options).context("failed to render to scad")?);
    Ok(scad_file)
}

impl TreeOperator {
    fn operation(&self) -> ScadObject {
        match self {
            TreeOperator::Union(_) => scad!(Union),
            TreeOperator::Hull(_) => scad!(Hull),
            TreeOperator::Diff(_) => scad!(Difference),
            TreeOperator::Intersect(_) => scad!(Intersection),
            TreeOperator::Color(color, _) => scad!(Color(color.rgb())),
            TreeOperator::Mirror(normal, _) => scad!(Mirror(*normal)),
        }
    }

    fn children(&self) -> Vec<Tree> {
        // TODO return refs?
        match self {
            TreeOperator::Union(ref v)
            | TreeOperator::Hull(ref v)
            | TreeOperator::Diff(ref v)
            | TreeOperator::Intersect(ref v) => v.clone(),

            TreeOperator::Color(_, ref tree)
            | TreeOperator::Mirror(_, ref tree) => vec![*tree.to_owned()],
        }
    }
}

impl Render for TreeObject {
    fn render(
        &self,
        options: RenderQuality,
    ) -> Result<ScadObject, ScadDotsError> {
        match self {
            TreeObject::Dot(ref dot) => dot.render(options),
            TreeObject::Cylinder(ref cylinder) => cylinder.render(options),
            TreeObject::Extrusion(ref extrusion) => extrusion.render(options),
        }
    }
}

impl Render for TreeOperator {
    fn render(
        &self,
        options: RenderQuality,
    ) -> Result<ScadObject, ScadDotsError> {
        let mut operation = self.operation();
        for child in self.children() {
            operation.add_child(
                child
                    .render(options)
                    .context("failed to render child of operator")?,
            );
        }
        Ok(operation)
    }
}

impl Render for Tree {
    fn render(
        &self,
        options: RenderQuality,
    ) -> Result<ScadObject, ScadDotsError> {
        match self {
            Tree::Object(ref object) => object.render(options),
            Tree::Operator(ref operator) => operator.render(options),
        }
    }
}

impl Render for Cylinder {
    fn render(
        &self,
        _options: RenderQuality,
    ) -> Result<ScadObject, ScadDotsError> {
        let obj = scad!(
                Translate(self.scad_translation());{
                    scad!(
                        Rotate(
                            self.rot_degs(),
                            self.rot_axis()?
                        );{
                            // Make cylinder w/ bottom face centered on origin
                            scad!(
                                Cylinder(self.height, Diameter(self.diameter))
                            )
                        }
                    )
                }
        );
        Ok(obj)
    }
}

impl Cylinder {
    fn scad_translation(&self) -> V3 {
        self.center_bot_pos - P3::origin()
    }

    fn rot_degs(&self) -> f32 {
        radians_to_degrees(self.rot.angle())
    }

    fn rot_axis(&self) -> Result<V3, ScadDotsError> {
        unwrap_rot_axis(self.rot)
    }
}

impl Render for Dot {
    fn render(
        &self,
        _options: RenderQuality,
    ) -> Result<ScadObject, ScadDotsError> {
        let obj = scad!(
            Translate(self.scad_translation());{
                scad!(
                    Rotate(self.rot_degs(), self.rot_axis()?);{
                        self.render_shape()
                    }
                )
            }
        );
        Ok(obj)
    }
}

impl Dot {
    pub fn scad_translation(&self) -> V3 {
        self.pos(C3::P000) + self.scad_to_p000() - P3::origin()
    }

    fn scad_to_p000(&self) -> V3 {
        let half = self.size / 2.;
        let v = match self.shape {
            DotShape::Cube => V3::new(0., 0., 0.),
            DotShape::Sphere => V3::new(half, half, half),
            DotShape::Cylinder => V3::new(half, half, 0.),
        };
        rotate(self.rot, v)
    }

    pub fn render_shape(&self) -> ScadObject {
        match self.shape {
            DotShape::Cube =>
            // Make cube, with bottom face centered on the origin
            {
                scad!(Cube(V3::new(self.size, self.size, self.size)))
            }
            DotShape::Sphere =>
            // Make sphere, with bottom surface touching the origin
            {
                scad!(Sphere(Diameter(self.size)))
            }
            DotShape::Cylinder =>
            // Make cylinder, with bottom face centered on the origin
            {
                scad!(Cylinder(self.size, Diameter(self.size)))
            }
        }
    }
}

impl Extrusion {
    pub fn scad_translation(&self) -> V3 {
        V3::new(0., 0., self.bottom_z)
    }
}

impl Render for Extrusion {
    fn render(
        &self,
        _options: RenderQuality,
    ) -> Result<ScadObject, ScadDotsError> {
        let points: Vec<V2> =
            self.perimeter.iter().map(|p| p - P2::origin()).collect();
        let mut params = LinExtrudeParams::default();
        params.height = self.thickness;
        Ok(scad!(
            Translate(self.scad_translation());{
                scad!(LinearExtrude(params);{
                    scad!( Polygon(PolygonParameters::new(points)))
                })}))
    }
}
