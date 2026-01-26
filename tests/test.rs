#[macro_use]
extern crate approx;
extern crate nalgebra;
extern crate nom;

#[macro_use(union, hull, mirror, red)]
extern crate scad_dots;

use scad_dots::harness::{check_model, Action, MAX_RELATIVE};
use scad_dots::parse::scad_relative_eq;

use scad_dots::core::*;
use scad_dots::core::{Corner1 as C1, Corner2 as C2, Corner3 as C3};
use scad_dots::cuboid::*;
use scad_dots::post::*;
use scad_dots::rect::*;
use scad_dots::triangle::*;

use std::f32::consts::PI;
#[test]
fn extrude1() {
    check_model("extrude1", Action::Test, || {
        let extrusion = Extrusion {
            perimeter: vec![
                P2::new(-5., -5.),
                P2::new(0., 10.),
                P2::new(20., 10.),
            ],
            thickness: 1.,
            bottom_z: -5.,
        };
        // change!
        Ok(extrusion.into())
    })
}

#[test]
fn cylinder_spec() {
    check_model("cylinder_spec", Action::Test, || {
        let spec = CylinderSpec {
            pos: P3::origin(),
            align: CylinderAlign::EndCenter(C1::P0),
            diameter: 10.,
            height: 3.,
            rot: R3::identity(),
        };
        Ok(Cylinder::new(spec).into())
    })
}

#[test]
fn cylinder_spec2() {
    check_model("cylinder_spec2", Action::Test, || {
        let spec = CylinderSpec {
            pos: P3::origin(),
            align: CylinderAlign::EndCenter(C1::P1),
            diameter: 10.,
            height: 3.,
            rot: R3::identity(),
        };
        Ok(Cylinder::new(spec).into())
    })
}

#[test]
fn cylinder_spec3() {
    check_model("cylinder_spec3", Action::Test, || {
        let spec = CylinderSpec {
            pos: P3::new(20., 0., 10.),
            align: CylinderAlign::EndCenter(C1::P1),
            diameter: 10.,
            height: 3.,
            rot: axis_degrees(Axis::X, 15.),
        };
        Ok(Cylinder::new(spec).into())
    })
}

#[test]
fn cylinder_spec4() {
    check_model("cylinder_spec4", Action::Test, || {
        let spec = CylinderSpec {
            pos: P3::new(0., 0., 20.),
            align: CylinderAlign::Centroid,
            diameter: 10.,
            height: 50.,
            rot: axis_degrees(Axis::Y, -30.),
        };
        let cyl = Cylinder::new(spec);
        let marker_size = 4.;
        Ok(union![
            cyl,
            red![union![
                mark(cyl.pos(CylinderAlign::EndCenter(C1::P0)), marker_size),
                mark(cyl.pos(CylinderAlign::Centroid), marker_size),
                mark(cyl.pos(CylinderAlign::EndCenter(C1::P1)), marker_size),
            ]]
        ])
    })
}

#[test]
fn explode_radially() {
    check_model("explode_radially", Action::Test, || {
        let x: V3 = Axis::X.into();
        let y: V3 = Axis::Y.into();
        // let z: V3 = Axis::Z.into();
        let rot = axis_degrees(y, 45.) * axis_degrees(x, 45.);
        let dot = Dot::new(DotSpec {
            pos: P3::new(20., 20., 20.),
            align: DotAlign::centroid(),
            size: 20.0,
            rot: rot,
            shape: DotShape::Cylinder,
        });

        Ok(union![
            Tree::union(dot.explode_radially(30., None, 5, true)?),
            // Tree::union(&dot.explode_radially(30., Some(axis), 5, true)?),
            dot,
        ])
    })
}

#[test]
fn explode_radially2() {
    check_model("explode_radially2", Action::Test, || {
        let x: V3 = Axis::X.into();
        let y: V3 = Axis::Y.into();
        let dot = Dot::new(DotSpec {
            pos: P3::new(20., 20., 20.),
            align: DotAlign::centroid(),
            size: 20.0,
            rot: axis_degrees(x, 45.),
            shape: DotShape::Cube,
        });
        let axis = y;
        Ok(union![
            Tree::union(dot.explode_radially(30., Some(axis), 5, false)?),
            dot,
        ])
    })
}

#[test]
fn mirror() {
    check_model("mirror", Action::Test, || {
        let r = Rect::new(RectSpec {
            pos: P3::origin(),
            align: RectAlign::origin(),
            y_length: 10.0,
            x_length: 5.0,
            size: 1.0,
            rot: axis_radians(V3::new(1.0, 0.0, -2.), PI / 2.0),
            shapes: RectShapes::Cube,
        })?;
        let original = chain(&[
            r.dot(C2::P01),
            r.dot(C2::P00),
            r.dot(C2::P11),
            r.dot(C2::P10),
        ])?;
        Ok(union![
            original.clone(),
            mirror![r.edge_unit_vec(Axis::X), original.clone()],
            Tree::mirror(Axis::X, original),
        ])
    })
}

#[test]
fn rect_cut_corners() {
    check_model("rect_cut_corners", Action::Test, || {
        let r = Rect::new(RectSpec {
            pos: P3::origin(),
            align: RectAlign::origin(),
            y_length: 2.0,
            x_length: 4.0,
            size: 1.0,
            rot: axis_degrees(Axis::X, 45.),
            shapes: RectShapes::Cube,
        })?;
        r.link(RectLink::Chamfer)
    })
}

#[test]
fn cuboid_chamfer_hole() {
    check_model("cuboid_chamfer_hole", Action::Test, || {
        // note that the z dimension is too thick!
        let hole_spec = CuboidSpecChamferZHole {
            pos: P3::origin(),
            align: CuboidAlign::center_face(CubeFace::Z0),
            x_length: 4.,
            y_length: 3.,
            z_length: 1.,
            chamfer: Fraction::new(1.)?,
            rot: R3::identity(),
            shapes: CuboidShapes::Cube,
        };
        let hole = Cuboid::new(hole_spec)?;
        hole.link(CuboidLink::ChamferZ)
    })
}

#[test]
fn triangle_simple() {
    check_model("triangle_simple", Action::Test, || {
        let spec = TriangleSpec {
            deg_b: 90.,
            len_bc: f32::sqrt(3.) * 10.,
            deg_c: 30.,
            size: 3.,
            point_b: P3::new(0., -9., 0.),
            rot: axis_degrees(Axis::Z, 30.),
        };
        let tri = Triangle::new(spec)?;
        Ok(union![tri.mark(spec)?, tri.link()?])
    })
}

#[test]
fn map_cuboid() {
    check_model("map_cuboid", Action::Test, || {
        let p1 = Cuboid::new(CuboidSpec {
            pos: P3::origin(),
            align: CuboidAlign::outside(C3::P111),
            x_length: 10.,
            y_length: 7.,
            z_length: 4.,
            size: 1.,
            rot: R3::identity(),
            shapes: DotShape::Cube.into(),
        })?;
        let p2 = p1.map_translate(V3::new(0., -10., 0.));
        let p3 = p1.map_rotate(axis_radians(Axis::Z, -PI / 8.));
        let p4 = p1.map_dots(&|d: &Dot| {
            d.translate(V3::new(20., 0., 0.))
                .rotate(axis_radians(Axis::Z, -PI / 8.))
        });
        Ok(union![
            p1.link(CuboidLink::Solid)?,
            p2.link(CuboidLink::Solid)?,
            p3.link(CuboidLink::Solid)?,
            p4.link(CuboidLink::Solid)?,
        ])
    })
}

#[test]
fn map_post() {
    check_model("map_post", Action::Test, || {
        let p1 = Post::new(PostSpec {
            pos: P3::new(4., 0., 10.),
            align: PostAlign::outside(C3::P111),
            len: 12.,
            size: 2.,
            rot: axis_radians(Axis::X, PI / 8.)
                * axis_radians(Axis::Y, PI / 2.),
            shapes: PostShapes::Cube,
        })?;
        let p2 = p1.map_translate(V3::new(0., 0., 4.));
        let axis =
            p1.pos(PostAlign::outside(C3::P001)) - p1.pos(PostAlign::origin());
        let p3 = p1.map_rotate(axis_radians(axis, PI / 8.));
        let p4 = p2.map_rotate(axis_radians(axis, PI / 8.));
        Ok(union![
            p1.link(PostLink::Dots),
            p2.link(PostLink::Dots),
            p3.link(PostLink::Dots),
            p4.link(PostLink::Dots),
        ])
    })
}

#[test]
fn map_rect() {
    check_model("map_rect", Action::Test, || {
        let r1 = Rect::new(RectSpec {
            pos: P3::origin(),
            align: RectAlign::origin(),
            x_length: 10.,
            y_length: 5.,
            size: 2.,
            rot: axis_radians(Axis::X, PI / 4.),
            shapes: DotShape::Cube.into(),
        })?;
        let r2 = r1.map_translate(V3::new(3., 0., 5.));
        let r3 = r1.map_rotate(axis_radians(V3::new(1., 1., 0.), PI / 2.));
        Ok(union![
            r1.link(RectLink::Dots)?,
            r2.link(RectLink::Dots)?,
            r3.link(RectLink::Dots)?,
        ])
    })
}

#[test]
fn scad_equality() {
    let scad1 = "
				union()
				{
								cylinder(h=1,d=2);
								}
";

    let scad2 = "
				union()
				{
                 cylinder(h=1,d=2);
				cylinder(h=3,d=4);
                 }
";

    assert!(!scad_relative_eq(scad1, scad2, 0.00001)
        .expect("failed to check equality"));
}

#[test]
fn cuboid_center_marks() {
    check_model("cuboid_center_marks", Action::Test, || {
        let cuboid = Cuboid::new(CuboidSpec {
            pos: P3::origin(),
            align: CuboidAlign::origin(),
            x_length: 20.,
            y_length: 20.,
            z_length: 20.,
            size: 3.,
            rot: R3::identity(),
            shapes: CuboidShapes::Cube,
        })?;
        Ok(union![
            cuboid.link(CuboidLink::Solid)?,
            mark(cuboid.pos(CuboidAlign::center_face(CubeFace::X0)), 1.),
            mark(cuboid.pos(CuboidAlign::center_face(CubeFace::X1)), 2.),
            mark(cuboid.pos(CuboidAlign::center_face(CubeFace::Y0)), 3.),
            mark(cuboid.pos(CuboidAlign::center_face(CubeFace::Y1)), 4.),
            mark(cuboid.pos(CuboidAlign::center_face(CubeFace::Z0)), 5.),
            mark(cuboid.pos(CuboidAlign::center_face(CubeFace::Z1)), 6.),
        ])
    })
}

#[test]
fn rect_center_marks() {
    check_model("rect_center_marks", Action::Test, || {
        let rect = Rect::new(RectSpec {
            pos: P3::origin(),
            align: RectAlign::origin(),
            x_length: 20.,
            y_length: 20.,
            size: 3.,
            rot: R3::identity(),
            shapes: RectShapes::Cube,
        })?;
        Ok(union![
            rect.link(RectLink::Solid)?,
            mark(rect.pos(RectAlign::center_face(CubeFace::X0)), 1.),
            mark(rect.pos(RectAlign::center_face(CubeFace::X1)), 2.),
            mark(rect.pos(RectAlign::center_face(CubeFace::Y0)), 3.),
            mark(rect.pos(RectAlign::center_face(CubeFace::Y1)), 4.),
            mark(rect.pos(RectAlign::center_face(CubeFace::Z0)), 5.),
            mark(rect.pos(RectAlign::center_face(CubeFace::Z1)), 6.),
        ])
    })
}

#[test]
fn dot_center_marks() {
    check_model("dot_center_marks", Action::Test, || {
        let dot = Dot::new(DotSpec {
            pos: P3::origin(),
            align: DotAlign::origin(),
            size: 20.0,
            rot: R3::identity(),
            shape: DotShape::Cube,
        });
        Ok(union![
            dot,
            mark(dot.pos(DotAlign::center_face(CubeFace::X0)), 1.),
            mark(dot.pos(DotAlign::center_face(CubeFace::X1)), 2.),
            mark(dot.pos(DotAlign::center_face(CubeFace::Y0)), 3.),
            mark(dot.pos(DotAlign::center_face(CubeFace::Y1)), 4.),
            mark(dot.pos(DotAlign::center_face(CubeFace::Z0)), 5.),
            mark(dot.pos(DotAlign::center_face(CubeFace::Z1)), 6.),
        ])
    })
}

#[test]
fn cuboid_from_dot() {
    check_model("cuboid_from_dot", Action::Test, || {
        let dot = Dot::new(DotSpec {
            pos: P3::new(1., 2., 3.),
            align: C3::P110.into(),
            size: 3.0,
            rot: axis_radians(V3::new(1., 2., 3.), PI / 3.),
            shape: DotShape::Cube,
        });

        let c = Cuboid::from_dot(dot, 0.5, DotShape::Cube)?;

        for corner in C3::all() {
            assert_relative_eq!(
                c.pos(CuboidAlign::outside(corner)),
                dot.pos(corner),
                max_relative = 0.0001,
            )
        }
        c.link(CuboidLink::Dots)
    })
}

#[test]
fn simple_dot() {
    check_model("simple_dot", Action::Test, || {
        let n = Dot::new(DotSpec {
            pos: P3::new(0., 0., 0.),
            align: C3::P000.into(),
            size: 2.0,
            rot: R3::identity(),
            shape: DotShape::Cube,
        });
        Ok(n.into())
    })
}

#[test]
fn dot_rot() {
    check_model("dot_rot", Action::Test, || {
        let n = Dot::new(DotSpec {
            pos: P3::new(0., 0., 0.),
            align: C3::P111.into(),
            size: 2.0,
            rot: axis_radians(V3::new(1., 1., 0.), PI / 2.),
            shape: DotShape::Cube,
        });
        Ok(n.into())
    })
}

#[test]
fn dot_rot_operator() {
    check_model("dot_rot_operator", Action::Test, || {
        let axis = V3::new(1., 1., 0.);
        let degrees = 30.;

        let _dot1 = Dot::new(DotSpec {
            pos: P3::new(0., 0., 0.),
            align: C3::P111.into(),
            size: 2.0,
            rot: axis_degrees(axis, degrees),
            shape: DotShape::Cube,
        });
        let dot2 = Dot::new(DotSpec {
            pos: P3::new(0., 0., 0.),
            align: C3::P111.into(),
            size: 2.0,
            rot: R3::identity(),
            shape: DotShape::Cube,
        });

        let _rendered1: Tree = _dot1.into();
        let rendered2 = Tree::rotate(degrees, axis, vec![dot2]);
        Ok(rendered2)
    })
}

#[test]
fn dot_cyl() {
    check_model("dot_cyl", Action::Test, || {
        let n = Dot::new(DotSpec {
            pos: P3::origin(),
            align: C3::P000.into(),
            size: 2.0,
            rot: axis_radians(V3::x_axis().into_inner(), PI / 4.),
            shape: DotShape::Cylinder,
        });
        Ok(n.into())
    })
}

// // TODO rewrite without SpecYAxis
// #[test]
// fn cuboid_center() {
//     check_model("cuboid_center", Action::Test, || {
//         let d = CuboidSpecYAxis {
//             pos: P3::origin(),
//             align: CuboidAlign::outside_midpoint(C3::P011, C3::P100),
//             x_length: 7.,
//             y_vec: V3::new(6., 10., -2.),
//             z_length: 5.,
//             size: 2.,
//         };
//         let c = Cuboid::new(CuboidShapes::Cube, d)?;
//         c.link(CuboidLink::Dots)
//     })
// }

#[test]
fn simple_rect() {
    check_model("simple_rect", Action::Test, || {
        let r = Rect::new(RectSpec {
            pos: P3::origin(),
            align: RectAlign::origin(),
            y_length: 5.0,
            x_length: 10.0,
            size: 2.0,
            rot: R3::identity(),
            shapes: RectShapes::Cube,
        })?;
        assert_eq!(10., r.max_coord(Axis::X));
        assert_eq!(5., r.max_coord(Axis::Y));
        assert_eq!(2., r.max_coord(Axis::Z));
        assert_eq!(0., r.min_coord(Axis::X));
        assert_eq!(0., r.min_coord(Axis::Y));
        assert_eq!(0., r.min_coord(Axis::Z));
        r.link(RectLink::Solid)
    })
}

#[test]
fn rect2() {
    // Not carefully checked
    check_model("rect2", Action::Test, || {
        let r = Rect::new(RectSpec {
            pos: P3::origin(),
            align: RectAlign::origin(),
            y_length: 10.0,
            x_length: 5.0,
            size: 2.0,
            rot: axis_radians(V3::new(1.0, 0.0, 0.0), PI / 4.0),
            shapes: RectShapes::Cylinder,
        })?;
        Ok(union![
            hull![r.dot(C2::P00), r.dot(C2::P01)],
            hull![r.dot(C2::P10), r.dot(C2::P11)],
            hull![r.dot(C2::P10), r.dot(C2::P00)],
        ])
    })
}

// #[test]
// fn post_ends() {
//     check_model("post_ends", Action::Preview, || {
//         let p = Post::new(
//             PostShapes::Cube,
//             PostSpecEnds {
//                 // bot: P3::origin(),
//                 // top: P3::new(10., 0., 0.),
//                 // align_bot_face: C2::P00,
//                 // size: 2.,
//                 // point_on_x_axis: P3::new(0., -2., 5.),

//                 // bot: P3::origin(),
//                 // top: P3::new(0., 0., 10.),
//                 // align_bot_face: C2::P00,
//                 // size: 2.,
//                 // x_axis_handle: P3::new(2., 1., 0.),

//                 bot: P3::origin(),
//                 top: P3::new(7., 0., 10.),
//                 align_bot_face: C2::P00,
//                 size: 2.,
//                 x_axis_handle: P3::new(2., 1., 0.),
//             },
//         )?;
//         Ok(p.link(PostLink::Solid))
//     })
// }

#[test]
fn simple_post() {
    check_model("simple_post", Action::Test, || {
        let p = Post::new(PostSpec {
            pos: P3::origin(),
            align: PostAlign::origin(),
            len: 10.,
            rot: R3::identity(),
            size: 3.0,
            shapes: PostShapes::Cube,
        })?;
        Ok(p.link(PostLink::Solid))
    })
}

#[test]
fn post_old_align() {
    check_model("post_old_align", Action::Test, || {
        let p = Post::new(PostSpec {
            pos: P3::origin(),
            align: PostAlign::Corner {
                post: C1::P0,
                dot: C3::P111,
            },
            len: 6.,
            rot: R3::identity(),
            // top: P3::new(0.0, 0.0, 6.0),
            size: 2.0,
            shapes: PostShapes::Round,
        })?;
        Ok(hull![p.bot, p.top])
    })
}

#[test]
fn simple_cuboid() {
    check_model("simple_cuboid", Action::Test, || {
        let p = Cuboid::new(CuboidSpec {
            pos: P3::origin(),
            align: CuboidAlign::origin(),
            y_length: 15.0,
            x_length: 10.0,
            z_length: 5.0,
            size: 0.5,
            rot: R3::identity(),
            shapes: CuboidShapes::Cube,
        })?;
        assert_relative_eq!(
            P3::origin(),
            p.pos(CuboidAlign::origin()),
            max_relative = MAX_RELATIVE
        );
        // assert_relative_eq!(
        //     P3::new(10., 15., 5.),
        //     p.top.dot(C2::P11).pos_corner(C3::P111),
        //     max_relative=MAX_RELATIVE
        // );
        // // assert_eq!(
        // //     P3::new(10., 15., 5.),
        // //     p.pos(CuboidAlign { dot: C3::P000, cuboid: C3::P111 })
        // // );
        // assert_relative_eq!(
        //     P3::origin(),
        //     p.pos(CuboidAlign { dot: C3::P000, cuboid: C3::P000 }),
        //     max_relative=MAX_RELATIVE
        // );
        p.link(CuboidLink::Solid)
    })
}

#[test]
fn cuboid_frame() {
    check_model("cuboid_frame", Action::Test, || {
        let p = Cuboid::new(CuboidSpec {
            pos: P3::origin(),
            align: CuboidAlign::origin(),
            y_length: 15.0,
            x_length: 10.0,
            z_length: 5.0,
            size: 0.5,
            rot: R3::identity(),
            shapes: CuboidShapes::Cube,
        })?;
        p.link(CuboidLink::Frame)
    })
}

#[test]
fn cuboid_sides() {
    check_model("cuboid_sides", Action::Test, || {
        let p = Cuboid::new(CuboidSpec {
            pos: P3::origin(),
            align: CuboidAlign::origin(),
            y_length: 15.0,
            x_length: 10.0,
            z_length: 5.0,
            size: 0.5,
            rot: R3::identity(),
            shapes: CuboidShapes::Cube,
        })?;
        p.link(CuboidLink::Sides)
    })
}

#[test]
fn cuboid_open_bot() {
    check_model("cuboid_open_bot", Action::Test, || {
        let p = Cuboid::new(CuboidSpec {
            pos: P3::origin(),
            align: CuboidAlign::origin(),
            y_length: 15.0,
            x_length: 10.0,
            z_length: 5.0,
            size: 0.5,
            rot: R3::identity(),
            shapes: CuboidShapes::Round,
        })?;
        p.link(CuboidLink::OpenBot)
    })
}

#[test]
fn spiral_cuboid() {
    // not carefully checked
    check_model("spiral_cuboid", Action::Test, || {
        let p = Cuboid::new(CuboidSpec {
            pos: P3::origin(),
            align: CuboidAlign::origin(),
            x_length: 10.0,
            y_length: 3.0,
            z_length: 4.0,
            size: 0.5,
            rot: axis_radians(V3::x_axis().into_inner(), PI / 8.),
            shapes: CuboidShapes::Cube,
        })?;
        Ok(union![
            hull![p.bot.dot(C2::P00), p.bot.dot(C2::P01)],
            hull![p.bot.dot(C2::P10), p.bot.dot(C2::P11)],
            hull![p.bot.dot(C2::P10), p.bot.dot(C2::P00)],
            hull![p.top.dot(C2::P00), p.top.dot(C2::P01)],
            hull![p.top.dot(C2::P10), p.top.dot(C2::P11)],
            hull![p.top.dot(C2::P10), p.top.dot(C2::P00)],
            hull![p.top.dot(C2::P01), p.bot.dot(C2::P11)],
        ])
    })
}

#[test]
fn snake() {
    check_model("snake", Action::Test, || {
        let start = Dot::new(DotSpec {
            pos: P3::origin(),
            align: C3::P111.into(),
            size: 2.0,
            rot: R3::identity(),
            shape: DotShape::Cube,
        });

        let end = Dot::new(DotSpec {
            pos: P3::new(-5., -10., 7.),
            align: C3::P111.into(),
            size: 2.0,
            rot: R3::identity(),
            shape: DotShape::Cube,
        });

        let snake = Snake::new(start, end, [Axis::X, Axis::Z, Axis::Y])?;

        Ok(snake.link(SnakeLink::Chain)?)
    })
}

#[test]
fn cuboid_corners() {
    check_model("cuboid_corners", Action::Test, || {
        let r = Cuboid::new(CuboidSpec {
            pos: P3::new(4., 6., 8.),
            align: CuboidAlign::Corner {
                dot: C3::P100,
                cuboid: C3::P010,
            },
            y_length: 15.0,
            x_length: 10.0,
            z_length: 5.0,
            size: 2.,
            rot: axis_radians(V3::x_axis().into_inner(), PI / 8.),
            shapes: CuboidShapes::Cube,
        })?;

        Ok(union![r.link(CuboidLink::Frame)?, r.mark_corners()])
    })
}

#[test]
fn fancy_translation() {
    assert_eq!(
        translate_p3_along_until(
            P3::new(1., 2., 3.),
            V3::new(10., 11., 12.),
            Axis::Z,
            15.
        ),
        P3::new(11., 13., 15.)
    );

    assert_eq!(
        translate_p3_along_until(
            P3::new(1., 2., 3.),
            V3::new(9., 12., 15.),
            Axis::X,
            4.,
        ),
        P3::new(4., 6., 8.)
    );

    assert_eq!(
        translate_p3_along_until(
            P3::new(1., -2., 3.),
            V3::new(9., 12., 15.),
            Axis::X,
            -2.,
        ),
        P3::new(-2., -6., -2.)
    );
}

#[test]
fn dot_fancy_translation() {
    check_model("dot_fancy_translation", Action::Test, || {
        let align = C3::P000;
        let a = Dot::new(DotSpec {
            pos: P3::new(0., 0., 0.),
            align: align.into(),
            size: 1.0,
            rot: rotation_between(Axis::X, V3::new(1., 1., 0.))?,
            shape: DotShape::Cube,
        });
        let b =
            a.translate_along_until(V3::new(9., 12., 15.), Axis::X, -3., align);
        Ok(union![a, b])
    })
}

#[test]
fn dot_fancy_translation2() {
    check_model("dot_fancy_translation2", Action::Test, || {
        let a = Dot::new(DotSpec {
            pos: P3::new(0., 0., 0.),
            align: DotAlign::centroid(),
            size: 1.0,
            rot: rotation_between(Axis::X, V3::new(1., 1., 0.))?,
            shape: DotShape::Cube,
        });
        let b = a.translate_along_until(
            V3::new(5., 0., 0.),
            Axis::X,
            -2.,
            C3::P100,
        );
        let c =
            a.translate_along_until(V3::new(5., 0., 0.), Axis::X, 2., C3::P000);
        Ok(union![a, b, c])
    })
}
