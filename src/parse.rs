use std;

use approx::{AbsDiffEq, RelativeEq};
use errors::ScadDotsError;
use nom::{digit, float};

pub fn scad_relative_eq(
    a: &str,
    b: &str,
    max_rel: f32,
) -> Result<bool, ScadDotsError> {
    Ok(relative_eq!(
        parse_scad(a)?,
        parse_scad(b)?,
        max_relative = max_rel
    ))
}

fn parse_scad(scad: &str) -> Result<ScadThing, ScadDotsError> {
    let out = parser(scad.as_bytes());
    if out.is_done() {
        Ok(out.unwrap().1)
    } else {
        Err(ScadDotsError::Parse)
    }
}

type Double = (f32, f32);
type Triple = (f32, f32, f32);

#[derive(Debug, Clone, PartialEq)]
enum ScadThing {
    Difference(Vec<ScadThing>),
    Union(Vec<ScadThing>),
    Hull(Vec<ScadThing>),
    Translate(Triple, Vec<ScadThing>),
    Rotate(f32, Triple, Vec<ScadThing>),
    LinearExtrude {
        height: f32,
        center: bool,
        convecity: f32, // misspelled in scad library!
        twist: f32,
        slices: f32,
        children: Vec<ScadThing>, // can it actually have more than 1 child?
    },
    Polygon(Vec<Double>, f32), // assume 'paths' is always 'undef'
    // Color(Quad, Vec<ScadThing>),
    Color(Triple, Vec<ScadThing>),
    Mirror(Triple, Vec<ScadThing>),
    Cube(Triple),
    Cylinder(f32, f32),
    Sphere(f32),
}

#[derive(Debug, Clone, Copy)]
enum EqMethod {
    Rel { epsilon: f32, max: f32 },
    Abs { epsilon: f32 },
}

impl EqMethod {
    fn is_eq(&self, a: f32, b: f32) -> bool {
        match *self {
            EqMethod::Rel { epsilon, max } => a.relative_eq(&b, epsilon, max),
            EqMethod::Abs { epsilon } => a.abs_diff_eq(&b, epsilon),
        }
    }
}

impl ScadThing {
    fn map_eq(&self, other: &Self, method: EqMethod) -> bool {
        if !self.variant_eq(other) {
            println!("\nNOT EQUAL: variants\n");
            return false;
        }

        if self.bools() != other.bools() {
            return false;
        }

        if self.floats().len() != other.floats().len() {
            return false;
        }

        for (a, b) in self.floats().into_iter().zip(other.floats().into_iter())
        {
            if !method.is_eq(a, b) {
                println!("\nNOT EQUAL: {}, {}\n", a, b);
                return false;
            }
        }

        if self.children().len() != other.children().len() {
            return false;
        }

        for (c, d) in self
            .children()
            .into_iter()
            .zip(other.children().into_iter())
        {
            if !c.map_eq(&d, method) {
                return false;
            }
        }
        true
    }

    fn variant_eq(&self, other: &Self) -> bool {
        std::mem::discriminant(self) == std::mem::discriminant(other)
    }

    fn bools(&self) -> Vec<bool> {
        match *self {
            ScadThing::LinearExtrude { center, .. } => vec![center],

            ScadThing::Color(..)
            | ScadThing::Rotate(..)
            | ScadThing::Translate(..)
            | ScadThing::Union(..)
            | ScadThing::Hull(..)
            | ScadThing::Difference(..)
            | ScadThing::Mirror(..)
            | ScadThing::Cube(..)
            | ScadThing::Sphere(..)
            | ScadThing::Cylinder(..)
            | ScadThing::Polygon(..) => Vec::new(),
        }
    }

    fn floats(&self) -> Vec<f32> {
        match *self {
            ScadThing::Translate(v, _)
            | ScadThing::Cube(v)
            | ScadThing::Mirror(v, _) => vec![v.0, v.1, v.2],
            ScadThing::Rotate(f, v, _) => vec![f, v.0, v.1, v.2],
            ScadThing::Color(rgb, _) => vec![rgb.0, rgb.1, rgb.2],
            ScadThing::Cylinder(f1, f2) => vec![f1, f2],
            ScadThing::Sphere(f) => vec![f],
            ScadThing::LinearExtrude {
                height,
                convecity,
                twist,
                slices,
                ..
            } => vec![height, convecity, twist, slices],
            ScadThing::Polygon(ref points, convexity) => {
                let mut v = flatten(points);
                v.push(convexity);
                v
            }
            ScadThing::Difference(_)
            | ScadThing::Union(_)
            | ScadThing::Hull(_) => Vec::new(),
        }
    }

    fn children(&self) -> Vec<Self> {
        match *self {
            ScadThing::Translate(_, ref children)
            | ScadThing::Rotate(_, _, ref children)
            | ScadThing::Color(_, ref children)
            | ScadThing::Mirror(_, ref children)
            | ScadThing::Hull(ref children)
            | ScadThing::Difference(ref children)
            | ScadThing::LinearExtrude { ref children, .. }
            | ScadThing::Union(ref children) => children.to_owned(),
            ScadThing::Cube(..)
            | ScadThing::Sphere(..)
            | ScadThing::Cylinder(..)
            | ScadThing::Polygon(..) => Vec::new(),
        }
    }
}

impl AbsDiffEq for ScadThing {
    type Epsilon = f32;

    fn default_epsilon() -> Self::Epsilon {
        f32::default_epsilon()
    }

    fn abs_diff_eq(&self, other: &Self, epsilon: Self::Epsilon) -> bool {
        self.map_eq(other, EqMethod::Abs { epsilon })
    }
}

impl RelativeEq for ScadThing {
    fn default_max_relative() -> Self::Epsilon {
        f32::default_max_relative()
    }

    fn relative_eq(
        &self,
        other: &Self,
        epsilon: Self::Epsilon,
        max_relative: Self::Epsilon,
    ) -> bool {
        self.map_eq(
            other,
            EqMethod::Rel {
                epsilon,
                max: max_relative,
            },
        )
    }
}

fn flatten(points: &[Double]) -> Vec<f32> {
    let mut floats = Vec::new();
    for p in points {
        floats.extend(&[p.0, p.1]);
    }
    floats
}

named!(
    parser<ScadThing>,
    ws!(do_parse!(
        // ignore the curve detail level in the header
        _detail: opt!(detail) >> body: scad_thing >> (body)
    ))
);

named!(
    scad_thing<ScadThing>,
    ws!(alt!(
        cube | sphere
            | cylinder
            | union
            | difference
            | hull
            | translate
            | rotate
            | color
            | polygon
            | linear_extrude
            | mirror
    ))
);

named!(
    detail<f32>,
    ws!(do_parse!(
        // TODO return i32 instead?
        tag!("$fn=") >> detail: number >> tag!(";") >> (detail)
    ))
);

named!(
    union<ScadThing>,
    ws!(do_parse!(
        tag!("union")
            >> tag!("()")
            >> tag!("{")
            >> children: many1!(scad_thing)
            >> tag!("}")
            >> (ScadThing::Union(children))
    ))
);

named!(
    difference<ScadThing>,
    ws!(do_parse!(
        tag!("difference")
            >> tag!("()")
            >> tag!("{")
            >> children: many1!(scad_thing)
            >> tag!("}")
            >> (ScadThing::Difference(children))
    ))
);

named!(
    color<ScadThing>,
    ws!(do_parse!(
        tag!("color")
            >> tag!("(")
            >> rgb: rgb
            >> tag!(")")
            >> tag!("{")
            >> children: many1!(scad_thing)
            >> tag!("}")
            >> (ScadThing::Color(rgb, children))
    ))
);

named!(
    mirror<ScadThing>,
    ws!(do_parse!(
        tag!("mirror")
            >> tag!("(")
            >> vector: triple
            >> tag!(")")
            >> tag!("{")
            >> children: many1!(scad_thing)
            >> tag!("}")
            >> (ScadThing::Mirror(vector, children))
    ))
);

named!(
    translate<ScadThing>,
    ws!(do_parse!(
        tag!("translate")
            >> tag!("(")
            >> vector: triple
            >> tag!(")")
            >> tag!("{")
            >> children: many1!(scad_thing)
            >> tag!("}")
            >> (ScadThing::Translate(vector, children))
    ))
);

named!(
    polygon<ScadThing>,
    ws!(do_parse!(
        tag!("polygon")
            >> tag!("(")
            >> tag!("points")
            >> tag!("=")
            >> tag!("[")
            >> point_vec: many1!(double_trailing_comma)
            >> tag!("]")
            >> tag!(",")
            >> tag!("paths")
            >> tag!("=")
            >> tag!("undef")
            >> tag!(",")
            >> tag!("convexity")
            >> tag!("=")
            >> convexity: number
            >> tag!(")")
            >> tag!(";")
            >> (ScadThing::Polygon(point_vec, convexity))
    ))
);

named!(
    linear_extrude<ScadThing>,
    ws!(do_parse!(
        tag!("linear_extrude")
            >> tag!("(")
            >> tag!("height")
            >> tag!("=")
            >> height: number
            >> tag!(",")
            >> tag!("center")
            >> tag!("=")
            >> center: boolean
            >> tag!(",")
            >> tag!("convecity")
            >> tag!("=")
            >> convecity: number
            >> tag!(",")
            >> tag!("twist")
            >> tag!("=")
            >> twist: number
            >> tag!(",")
            >> tag!("slices")
            >> tag!("=")
            >> slices: number
            >> tag!(")")
            >> tag!("{")
            >> children: many1!(scad_thing)
            >> tag!("}")
            >> (ScadThing::LinearExtrude {
                height,
                center,
                convecity,
                twist,
                slices,
                children,
            })
    ))
);

named!(
    rotate<ScadThing>,
    ws!(do_parse!(
        tag!("rotate")
            >> tag!("(")
            >> angle: number
            >> tag!(",")
            >> axis: triple
            >> tag!(")")
            >> tag!("{")
            >> children: many1!(scad_thing)
            >> tag!("}")
            >> (ScadThing::Rotate(angle, axis, children))
    ))
);

named!(
    hull<ScadThing>,
    ws!(do_parse!(
        tag!("hull")
            >> tag!("()")
            >> tag!("{")
            >> children: many1!(scad_thing)
            >> tag!("}")
            >> (ScadThing::Hull(children))
    ))
);

named!(
    cube<ScadThing>,
    ws!(do_parse!(
        tag!("cube")
            >> tag!("(")
            >> dims: triple
            >> tag!(")")
            >> tag!(";")
            >> (ScadThing::Cube(dims))
    ))
);

named!(
    sphere<ScadThing>,
    ws!(do_parse!(
        tag!("sphere")
            >> tag!("(d=")
            >> diameter: number
            >> tag!(")")
            >> tag!(";")
            >> (ScadThing::Sphere(diameter))
    ))
);

named!(
    cylinder<ScadThing>,
    ws!(do_parse!(
        tag!("cylinder")
            >> tag!("(")
            >> tag!("h=")
            >> height: number
            >> tag!(",")
            >> tag!("d=")
            >> diameter: number
            >> tag!(")")
            >> tag!(";")
            >> (ScadThing::Cylinder(height, diameter))
    ))
);

named!(
    rgb<Triple>,
    ws!(do_parse!(
        tag!("[") >>
        r: number >>
        tag!(",") >>
        g: number >>
        tag!(",") >>
        b: number >>
    // tag!(",") >>
    // a: number >>
        tag!("]") >>
        // (r,g,b,a)
        (r,g,b)
    ))
);

named!(
    double_trailing_comma<Double>,
    ws!(do_parse!(p: double >> tag!(",") >> (p)))
);

named!(
    double<Double>,
    ws!(do_parse!(
        tag!("[") >> x: number >> tag!(",") >> y: number >> tag!("]") >> (x, y)
    ))
);

named!(
    triple<Triple>,
    ws!(do_parse!(
        tag!("[")
            >> x: number
            >> tag!(",")
            >> y: number
            >> tag!(",")
            >> z: number
            >> tag!("]")
            >> (x, y, z)
    ))
);

named!(boolean<bool>, alt!(true_string | false_string));
named!(true_string<bool>, ws!(do_parse!(tag!("true") >> (true))));
named!(false_string<bool>, ws!(do_parse!(tag!("false") >> (false))));

named!(number<f32>, alt!(float | integer));

named!(
    integer<f32>,
    do_parse!(
        sign: opt!(tag!("-"))
            >> num: map_res!(numeric_string, std::str::FromStr::from_str)
            >> (if sign.is_none() { num } else { num * -1. })
    )
);

named!(numeric_string<&str>, map_res!(digit, std::str::from_utf8));
