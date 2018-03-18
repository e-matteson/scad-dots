#[macro_use]
extern crate approx;
extern crate failure;
extern crate nalgebra;
extern crate nom;

#[macro_use(union, hull, dot)]
extern crate scad_dots;

use scad_dots::harness::{check_model, Action, MAX_RELATIVE};
use scad_dots::parse::scad_relative_eq;

use scad_dots::utils::{Corner1 as C1, Corner2 as C2, Corner3 as C3};
use scad_dots::utils::*;
use scad_dots::core::*;
use scad_dots::post::*;
use scad_dots::rect::*;
use scad_dots::cuboid::*;
use scad_dots::triangle::*;

use std::f32::consts::PI;

#[test]
fn rect_cut_corners() {
    check_model("rect_cut_corners", Action::Test, || {
        let r = Rect::new(
            RectShapes::Cube,
            RectSpecBasic {
                pos: P3::origin(),
                align: RectAlign::origin(),
                y_dim: 2.0,
                x_dim: 4.0,
                size: 1.0,
                rot: axis_degrees(Axis::X.into(), 45.),
            },
        )?;
        r.link(RectLink::Chamfer)
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
            rot: axis_degrees(Axis::Z.into(), 30.),
        };
        let tri = Triangle::new(spec)?;
        Ok(union![tri.mark(spec)?, tri.link()?])
    })
}

#[test]
fn map_cuboid() {
    check_model("map_cuboid", Action::Test, || {
        let p1 = Cuboid::new(
            CuboidShapes::Cube,
            CuboidSpecBasic {
                pos: P3::origin(),
                align: CuboidAlign::outside(C3::P111),
                x_dim: 10.,
                y_dim: 7.,
                z_dim: 4.,
                size: 1.,
                rot: R3::identity(),
            },
        )?;
        let p2 = p1.map_translate(V3::new(0., -10., 0.));
        let p3 = p1.map_rotate(axis_radians(Axis::Z.into(), -PI / 8.));
        let p4 = p1.map(&|d: &Dot| {
            d.translate(V3::new(20., 0., 0.))
                .rotate(axis_radians(Axis::Z.into(), -PI / 8.))
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
        let p1 = Post::new(
            PostShapes::Cube,
            PostSpecRot {
                pos: P3::new(4., 0., 10.),
                align: PostAlign::outside(C3::P111),
                len: 12.,
                size: 2.,
                rot: axis_radians(Axis::X.into(), PI / 8.)
                    * axis_radians(Axis::Y.into(), PI / 2.),
            },
        )?;
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
        let r1 = Rect::new(
            RectShapes::Cube,
            RectSpecBasic {
                pos: P3::origin(),
                align: RectAlign::origin(),
                x_dim: 10.,
                y_dim: 5.,
                size: 2.,
                rot: axis_radians(Axis::X.into(), PI / 4.),
            },
        )?;
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
				\
                 cylinder(h=3,d=4);
                 }
";

    assert!(!scad_relative_eq(scad1, scad2, 0.00001)
        .expect("failed to check equality"));
}

#[test]
fn cuboid_center_marks() {
    check_model("cuboid_center_marks", Action::Test, || {
        let cuboid = Cuboid::new(
            CuboidShapes::Cube,
            CuboidSpecBasic {
                pos: P3::origin(),
                align: CuboidAlign::origin(),
                x_dim: 20.,
                y_dim: 20.,
                z_dim: 20.,
                size: 3.,
                rot: R3::identity(),
            },
        )?;
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
        let rect = Rect::new(
            RectShapes::Cube,
            RectSpecBasic {
                pos: P3::origin(),
                align: RectAlign::origin(),
                x_dim: 20.,
                y_dim: 20.,
                size: 3.,
                rot: R3::identity(),
            },
        )?;
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
        let dot = Dot::new(
            Shape::Cube,
            DotSpec {
                pos: P3::origin(),
                align: DotAlign::origin(),
                size: 20.0,
                rot: R3::identity(),
            },
        );
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
        let dot = Dot::new(
            Shape::Cube,
            DotSpec {
                pos: P3::new(1., 2., 3.),
                align: C3::P110.into(),
                size: 3.0,
                rot: axis_radians(V3::new(1., 2., 3.), PI / 3.),
            },
        );

        let c = Cuboid::from_dot(dot, 0.5, Shape::Cube)?;

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
        let n = Dot::new(
            Shape::Cube,
            DotSpec {
                pos: P3::new(0., 0., 0.),
                align: C3::P000.into(),
                size: 2.0,
                rot: R3::identity(),
            },
        );
        Ok(dot![n])
    })
}

#[test]
fn dot_rot() {
    check_model("dot_rot", Action::Test, || {
        let n = Dot::new(
            Shape::Cube,
            DotSpec {
                pos: P3::new(0., 0., 0.),
                align: C3::P111.into(),
                size: 2.0,
                rot: axis_radians(V3::new(1., 1., 0.), PI / 2.),
            },
        );
        Ok(dot![n])
    })
}

#[test]
fn dot_cyl() {
    check_model("dot_cyl", Action::Test, || {
        let n = Dot::new(
            Shape::Cylinder,
            DotSpec {
                pos: P3::origin(),
                align: C3::P000.into(),
                size: 2.0,
                rot: axis_radians(V3::x_axis().unwrap(), PI / 4.),
            },
        );
        Ok(dot![n])
    })
}

// // TODO rewrite without SpecYAxis
// #[test]
// fn cuboid_center() {
//     check_model("cuboid_center", Action::Test, || {
//         let d = CuboidSpecYAxis {
//             pos: P3::origin(),
//             align: CuboidAlign::outside_midpoint(C3::P011, C3::P100),
//             x_dim: 7.,
//             y_vec: V3::new(6., 10., -2.),
//             z_dim: 5.,
//             size: 2.,
//         };
//         let c = Cuboid::new(CuboidShapes::Cube, d)?;
//         c.link(CuboidLink::Dots)
//     })
// }

#[test]
fn simple_rect() {
    check_model("simple_rect", Action::Test, || {
        let r = Rect::new(
            RectShapes::Cube,
            RectSpecBasic {
                pos: P3::origin(),
                align: RectAlign::origin(),
                y_dim: 5.0,
                x_dim: 10.0,
                size: 2.0,
                rot: R3::identity(),
            },
        )?;
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
        let r = Rect::new(
            RectShapes::Cylinder,
            RectSpecBasic {
                pos: P3::origin(),
                align: RectAlign::origin(),
                y_dim: 10.0,
                x_dim: 5.0,
                size: 2.0,
                rot: axis_radians(V3::new(1.0, 0.0, 0.0), PI / 4.0),
            },
        )?;
        Ok(union![
            hull![dot![r.get_dot(C2::P00)], dot![r.get_dot(C2::P01)]],
            hull![dot![r.get_dot(C2::P10)], dot![r.get_dot(C2::P11)]],
            hull![dot![r.get_dot(C2::P10)], dot![r.get_dot(C2::P00)]],
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
        let p = Post::new(
            PostShapes::Cube,
            PostSpecAxis {
                pos: P3::origin(),
                align: PostAlign::origin(),
                len: 10.,
                axis: Axis::Z.into(),
                size: 3.0,
                radians: 0.,
            },
        )?;
        Ok(p.link(PostLink::Solid))
    })
}

#[test]
fn post_old_align() {
    check_model("post_old_align", Action::Test, || {
        let p = Post::new(
            PostShapes::Round,
            PostSpecAxis {
                pos: P3::origin(),
                align: PostAlign::Corner {
                    post: C1::P0,
                    dot: C3::P111,
                },
                len: 6.,
                axis: Axis::Z.into(),
                // top: P3::new(0.0, 0.0, 6.0),
                size: 2.0,
                radians: 0.,
            },
        )?;
        Ok(hull![dot![p.bot], dot![p.top]])
    })
}

#[test]
fn simple_cuboid() {
    check_model("simple_cuboid", Action::Test, || {
        let p = Cuboid::new(
            CuboidShapes::Cube,
            CuboidSpecBasic {
                pos: P3::origin(),
                align: CuboidAlign::origin(),
                y_dim: 15.0,
                x_dim: 10.0,
                z_dim: 5.0,
                size: 0.5,
                rot: R3::identity(),
            },
        )?;
        assert_relative_eq!(
            P3::origin(),
            p.pos(CuboidAlign::origin()),
            max_relative = MAX_RELATIVE
        );
        // assert_relative_eq!(
        //     P3::new(10., 15., 5.),
        //     p.top.get_dot(C2::P11).pos_corner(C3::P111),
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
        let p = Cuboid::new(
            CuboidShapes::Cube,
            CuboidSpecBasic {
                pos: P3::origin(),
                align: CuboidAlign::origin(),
                y_dim: 15.0,
                x_dim: 10.0,
                z_dim: 5.0,
                size: 0.5,
                rot: R3::identity(),
            },
        )?;
        p.link(CuboidLink::Frame)
    })
}

#[test]
fn cuboid_sides() {
    check_model("cuboid_sides", Action::Test, || {
        let p = Cuboid::new(
            CuboidShapes::Cube,
            CuboidSpecBasic {
                pos: P3::origin(),
                align: CuboidAlign::origin(),
                y_dim: 15.0,
                x_dim: 10.0,
                z_dim: 5.0,
                size: 0.5,
                rot: R3::identity(),
            },
        )?;
        p.link(CuboidLink::Sides)
    })
}

#[test]
fn cuboid_open_bot() {
    check_model("cuboid_open_bot", Action::Test, || {
        let p = Cuboid::new(
            CuboidShapes::Round,
            CuboidSpecBasic {
                pos: P3::origin(),
                align: CuboidAlign::origin(),
                y_dim: 15.0,
                x_dim: 10.0,
                z_dim: 5.0,
                size: 0.5,
                rot: R3::identity(),
            },
        )?;
        p.link(CuboidLink::OpenBot)
    })
}

#[test]
fn spiral_cuboid() {
    // not carefully checked
    check_model("spiral_cuboid", Action::Test, || {
        let p = Cuboid::new(
            CuboidShapes::Cube,
            CuboidSpecBasic {
                pos: P3::origin(),
                align: CuboidAlign::origin(),
                x_dim: 10.0,
                y_dim: 3.0,
                z_dim: 4.0,
                size: 0.5,
                rot: axis_radians(V3::x_axis().unwrap(), PI / 8.),
            },
        )?;
        Ok(union![
            hull![dot![p.bot.get_dot(C2::P00)], dot![p.bot.get_dot(C2::P01)]],
            hull![dot![p.bot.get_dot(C2::P10)], dot![p.bot.get_dot(C2::P11)]],
            hull![dot![p.bot.get_dot(C2::P10)], dot![p.bot.get_dot(C2::P00)]],
            hull![dot![p.top.get_dot(C2::P00)], dot![p.top.get_dot(C2::P01)]],
            hull![dot![p.top.get_dot(C2::P10)], dot![p.top.get_dot(C2::P11)]],
            hull![dot![p.top.get_dot(C2::P10)], dot![p.top.get_dot(C2::P00)]],
            hull![dot![p.top.get_dot(C2::P01)], dot![p.bot.get_dot(C2::P11)]],
        ])
    })
}

#[test]
fn snake() {
    check_model("snake", Action::Test, || {
        let start = Dot::new(
            Shape::Cube,
            DotSpec {
                pos: P3::origin(),
                align: C3::P111.into(),
                size: 2.0,
                rot: R3::identity(),
            },
        );

        let end = Dot::new(
            Shape::Cube,
            DotSpec {
                pos: P3::new(-5., -10., 7.),
                align: C3::P111.into(),
                size: 2.0,
                rot: R3::identity(),
            },
        );

        let snake = Snake::new(start, end, [Axis::X, Axis::Z, Axis::Y])?;

        Ok(snake.link(SnakeLink::Chain)?)
    })
}

#[test]
fn cuboid_corners() {
    check_model("cuboid_corners", Action::Test, || {
        let r = Cuboid::new(
            CuboidShapes::Cube,
            CuboidSpecBasic {
                pos: P3::new(4., 6., 8.),
                align: CuboidAlign::Corner {
                    dot: C3::P100,
                    cuboid: C3::P010,
                },
                y_dim: 15.0,
                x_dim: 10.0,
                z_dim: 5.0,
                size: 2.,
                rot: axis_radians(V3::x_axis().unwrap(), PI / 8.),
            },
        )?;

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
        let a = Dot::new(
            Shape::Cube,
            DotSpec {
                pos: P3::new(0., 0., 0.),
                align: align.into(),
                size: 1.0,
                rot: rotation_between(&Axis::X.into(), &V3::new(1., 1., 0.))?,
            },
        );
        let b =
            a.translate_along_until(V3::new(9., 12., 15.), Axis::X, -3., align);
        Ok(union![a, b])
    })
}

#[test]
fn dot_fancy_translation2() {
    check_model("dot_fancy_translation2", Action::Test, || {
        let a = Dot::new(
            Shape::Cube,
            DotSpec {
                pos: P3::new(0., 0., 0.),
                align: DotAlign::center_solid(),
                size: 1.0,
                rot: rotation_between(&Axis::X.into(), &V3::new(1., 1., 0.))?,
            },
        );
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
