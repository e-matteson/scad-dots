use scad_generator::*;

use core::{Cylinder, Dot, Extrusion, Shape, Tree};
use utils::{rotate, Corner3 as C3, P2, P3, V2, V3};

use failure::Error;

pub trait Render {
    fn render(&self, options: RenderQuality) -> Result<ScadObject, Error>;
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
) -> Result<(), Error>
where
    T: Render,
{
    let scad_file = make_scad_file(thing, options)?;
    scad_file.write_to_file(path);
    Ok(())
}

pub fn to_code<T>(thing: &T, options: RenderQuality) -> Result<String, Error>
where
    T: Render,
{
    let scad_file = make_scad_file(thing, options)?;
    Ok(scad_file.get_code())
}

fn make_scad_file<T>(
    thing: &T,
    options: RenderQuality,
) -> Result<ScadFile, Error>
where
    T: Render,
{
    let mut scad_file = ScadFile::new();
    // detail controls resolution of curves
    scad_file.set_detail(options.detail());
    scad_file.add_object(thing.render(options)?);
    Ok(scad_file)
}

impl Tree {
    // Some helpful methods for implementing render.
    pub fn get_operation(&self) -> ScadObject {
        match *self {
            Tree::Union(_) => scad!(Union),
            Tree::Hull(_) => scad!(Hull),
            Tree::Diff(_) => scad!(Difference),
            Tree::Intersect(_) => scad!(Intersection),
            Tree::Color(color, _) => scad!(Color(color.rgb())),
            Tree::Mirror(normal, _) => scad!(Mirror(normal)),
            Tree::Dot(..) | Tree::Cylinder(..) | Tree::Extrusion(..) => {
                Tree::panic_not_operator()
            }
        }
    }

    pub fn get_children(&self) -> Vec<Tree> {
        match *self {
            Tree::Union(ref v)
            | Tree::Hull(ref v)
            | Tree::Diff(ref v)
            | Tree::Color(_, ref v)
            | Tree::Mirror(_, ref v)
            | Tree::Intersect(ref v) => v.clone(),
            Tree::Dot(..) | Tree::Cylinder(..) | Tree::Extrusion(..) => {
                Tree::panic_not_operator()
            }
        }
    }

    fn panic_not_operator() -> ! {
        panic!("not an operator, you should have checked for Dot already!")
    }
}

impl Render for Tree {
    fn render(&self, options: RenderQuality) -> Result<ScadObject, Error> {
        match *self {
            Tree::Dot(ref dot) => dot.render(options),
            Tree::Cylinder(ref cylinder) => cylinder.render(options),
            Tree::Extrusion(ref extrusion) => extrusion.render(options),
            _ => {
                let mut operation = self.get_operation();
                for child in self.get_children() {
                    operation.add_child(child.render(options)?);
                }
                Ok(operation)
            }
        }
    }
}

impl Render for Cylinder {
    fn render(&self, _options: RenderQuality) -> Result<ScadObject, Error> {
        let obj = scad!(
                Translate(self.center_bot_pos_vec());{
                    scad!(
                        Rotate(
                            self.rot_degs_for_rendering(),
                            self.rot_axis_for_rendering()?
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
    fn center_bot_pos_vec(&self) -> V3 {
        self.center_bot_pos - P3::origin()
    }
}

impl Render for Dot {
    fn render(&self, _options: RenderQuality) -> Result<ScadObject, Error> {
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
            Shape::Cube => V3::new(0., 0., 0.),
            Shape::Sphere => V3::new(half, half, half),
            Shape::Cylinder => V3::new(half, half, 0.),
        };
        rotate(self.rot, v)
    }

    pub fn render_shape(&self) -> ScadObject {
        match self.shape {
            Shape::Cube =>
            // Make cube, with bottom face centered on the origin
            {
                scad!(Cube(V3::new(self.size, self.size, self.size)))
            }
            Shape::Sphere =>
            // Make sphere, with bottom surface touching the origin
            {
                scad!(Sphere(Diameter(self.size)))
            }
            Shape::Cylinder =>
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
    fn render(&self, _options: RenderQuality) -> Result<ScadObject, Error> {
        // TODO update, add translation
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
