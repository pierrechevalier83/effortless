use crate::geometry;
use crate::params::{
    COL_ANGLES_Y, NUM_COLS, NUM_ROWS, ROW_ANGLES_X, SWITCH_FOOTPRINT_LEFT_WIRES, SWITCH_PLATE_XYZ,
    THUMB0_ROTATION, THUMB0_XYZ, VIRTUAL_INFINITY,
};
use crate::switch::{Hand, Switch};
use glam::{dvec2, dvec3, DVec2, DVec3};
use opencascade::angle::{rvec, Angle};
use opencascade::primitives::{Direction, Edge, Shape, Solid, Wire, WireBuilder};
use opencascade::workplane::Workplane;

pub struct Keyboard {
    switch_matrix: Vec<Vec<Switch>>,
    thumbs: Vec<Switch>,
    hand: Hand,
}

fn project_to_plane(workplane: &Workplane, world_pos: DVec3) -> DVec2 {
    let point_in_plane = workplane.to_local_pos(world_pos);
    // Drop the z dimension in the local coordinates of that plane, therefore projecting to the
    // plane
    dvec2(point_in_plane.x, point_in_plane.y)
}

// This is only created by projection, so we have the additional
// guarantee that the poinst are co-planar
// Also, we only focus on the points we use in our actual sketch
//
//       y     e-----f          z
//             |     |
//       c-----d 2,2 g-----h----i
//       |                      |
// a-----b 1,2         3,2  4,2  \
// |                              j
// | 0,2        2,1               |
// |       1,1         3,1  4,1   |
// |                              k
// | 0,1         2,0             /
// |       1,0 p-----o 3,0  4,0 |
// |           |     |          |
// | 0,0 r-----q     n-----m----l
// |     |
// t-----s                   u-----+-----v
//                           |           |
//                           x-----------w
struct XYMatrixSketch {
    workplane: Workplane,
    construction_points: Vec<DVec2>,
}

impl XYMatrixSketch {
    fn new(workplane: &Workplane, switch_matrix: &Vec<Vec<Switch>>, thumbs: &Vec<Switch>) -> Self {
        use Direction::*;
        // We pick NegZ for all of them, because due to the rotation of the rows, it is always
        // going to be the furthest away, and we wouldn't want to clip the geometry.
        let a = switch_matrix[0][2].coordinate(NegX, PosY, NegZ);
        let b = switch_matrix[0][2].coordinate(PosX, PosY, NegZ);
        let c = switch_matrix[1][2].coordinate(NegX, PosY, NegZ);
        let d = switch_matrix[1][2].coordinate(PosX, PosY, NegZ);
        let e = switch_matrix[2][2].coordinate(NegX, PosY, NegZ);
        let f = switch_matrix[2][2].coordinate(PosX, PosY, NegZ);
        let g = switch_matrix[3][2].coordinate(NegX, PosY, NegZ);
        let h = switch_matrix[3][2].coordinate(PosX, PosY, NegZ);
        let i = switch_matrix[4][2].coordinate(PosX, PosY, NegZ);
        let j = switch_matrix[4][1].coordinate(PosX, PosY, NegZ);
        let k = switch_matrix[4][1].coordinate(PosX, NegY, NegZ);
        let l = switch_matrix[4][0].coordinate(PosX, NegY, NegZ);
        let m = switch_matrix[3][0].coordinate(PosX, NegY, NegZ);
        let n = switch_matrix[3][0].coordinate(NegX, NegY, NegZ);
        let o = switch_matrix[2][0].coordinate(PosX, NegY, NegZ);
        let p = switch_matrix[2][0].coordinate(NegX, NegY, NegZ);
        let q = switch_matrix[1][0].coordinate(PosX, NegY, NegZ);
        let r = switch_matrix[1][0].coordinate(NegX, NegY, NegZ);
        let s = switch_matrix[0][0].coordinate(PosX, NegY, NegZ);
        let t = switch_matrix[0][0].coordinate(NegX, NegY, NegZ);
        let u = thumbs[0].coordinate(NegX, PosY, NegZ);
        let v = thumbs[1].coordinate(PosX, PosY, NegZ);
        let w = thumbs[1].coordinate(PosX, NegY, NegZ);
        let x = thumbs[0].coordinate(NegX, NegY, NegZ);
        let construction_points = [
            a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t, u, v, w, x,
        ]
        .into_iter()
        .map(|point| project_to_plane(&workplane, point))
        .collect();

        Self {
            workplane: workplane.clone(),
            construction_points,
        }
    }
    fn local_at(&self, reference: char) -> DVec2 {
        let index = (reference as u8 - 'a' as u8) as usize;
        self.construction_points[index]
    }
    fn world_at(&self, reference: char) -> DVec3 {
        let local = match reference {
            // TODO: parameterize these margins
            'j' => dvec2(self.local_at('j').x + 5., self.local_at('j').y),
            'k' => dvec2(self.local_at('k').x + 5., self.local_at('k').y),
            'l' => dvec2(self.local_at('l').x, self.local_at('l').y - 6.),
            'r' => dvec2(self.local_at('r').x, self.local_at('r').y - 6.),
            'v' => dvec2(self.local_at('v').x + 7., self.local_at('v').y),
            'w' => dvec2(self.local_at('w').x + 7., self.local_at('w').y),
            'y' => dvec2(self.local_at('c').x, self.local_at('e').y),
            'z' => dvec2(self.local_at('i').x, self.local_at('f').y),
            'A' => dvec2(VIRTUAL_INFINITY, self.local_at('l').y),
            'B' => dvec2(VIRTUAL_INFINITY, -VIRTUAL_INFINITY),
            'C' => dvec2(self.local_at('r').x, -VIRTUAL_INFINITY),
            // PCB
            // top-left
            'D' => dvec2(self.local_at('e').x - 1.1, self.local_at('e').y - 2.),
            // top-right
            'E' => dvec2(self.local_at('e').x - 1.1 + 51.0, self.local_at('e').y - 2.),
            // bottom-right
            'F' => dvec2(
                self.local_at('e').x + 51.0 - 1.1,
                self.local_at('e').y - 50.5 - 2.,
            ),
            // bottom-left
            'G' => dvec2(self.local_at('e').x - 1.1, self.local_at('e').y - 50.5 - 2.),
            // RP2040
            'H' => dvec2(
                self.local_at('e').x + 51.0 / 2. - 9.5 - 1.1,
                self.local_at('e').y - 2.,
            ),
            'I' => dvec2(
                self.local_at('e').x + 51.0 / 2. + 9.5 - 1.1,
                self.local_at('e').y - 2.,
            ),
            'J' => dvec2(
                self.local_at('e').x + 51.0 / 2. + 9.5 - 1.1,
                self.local_at('e').y - 27.0 - 2.,
            ),
            'K' => dvec2(
                self.local_at('e').x + 51.0 / 2. - 9.5 - 1.1,
                self.local_at('e').y - 27.0 - 2.,
            ),
            // Jack
            'L' => dvec2(
                self.local_at('e').x + 42.7 + 0.1 - 6.7 / 2. - 1.1,
                self.local_at('e').y - 2.,
            ),
            'M' => dvec2(
                self.local_at('e').x + 42.7 + 0.1 + 6.7 / 2. - 1.1,
                self.local_at('e').y - 2.,
            ),
            'N' => dvec2(
                self.local_at('e').x + 42.7 + 0.1 + 6.7 / 2. - 1.1,
                self.local_at('e').y - 16. - 2.,
            ),
            'O' => dvec2(
                self.local_at('e').x + 42.7 + 0.1 - 6.7 / 2. - 1.1,
                self.local_at('e').y - 16. - 2.,
            ),
            // Holes cutout
            // top-left
            'P' => dvec2(self.local_at('e').x - 1.1, self.local_at('e').y - 29. - 2.),
            // top-right
            'Q' => dvec2(
                self.local_at('e').x + 51.0 - 1.1,
                self.local_at('e').y - 29. - 2.,
            ),
            _ => self.local_at(reference),
        };
        self.workplane.to_world_pos(dvec3(local.x, local.y, 0.))
    }
    fn wire(&self) -> Wire {
        let a = self.world_at('a');
        let c = self.world_at('c');
        let e = self.world_at('e');
        let f = self.world_at('f');
        let j = self.world_at('j');
        let q = self.world_at('q');
        let s = self.world_at('s');
        let t = self.world_at('t');
        let v = self.world_at('v');
        let w = self.world_at('w');
        let x = self.world_at('x');
        let y = self.world_at('y');
        let z = self.world_at('z');

        let mut builder = WireBuilder::new();
        builder.add_edge(&Edge::segment(e, z));
        builder.add_edge(&Edge::spline_from_points(
            vec![z, j, v, w, x, q, s, t, a, c, e],
            Some(((z - f).normalize(), (e - y).normalize())),
        ));
        builder.build()
    }
    fn thumbs_cutout(&self) -> Wire {
        let l = self.world_at('l');
        let r = self.world_at('r');
        let aa = self.world_at('A');
        let bb = self.world_at('B');
        let cc = self.world_at('C');

        let mut builder = WireBuilder::new();
        builder.add_edge(&Edge::segment(r, l));
        builder.add_edge(&Edge::segment(l, aa));
        builder.add_edge(&Edge::segment(aa, bb));
        builder.add_edge(&Edge::segment(bb, cc));
        builder.add_edge(&Edge::segment(cc, r));
        builder.build().fillet(10.)
    }
    fn electronics_cutout(&self) -> Shape {
        let pcb_outline = ['D', 'E', 'F', 'G']
            .into_iter()
            .map(|point| self.world_at(point));
        let pcb: Shape = Wire::from_ordered_points(pcb_outline)
            .unwrap()
            .to_face()
            .extrude(self.workplane.normal() * (1.8 + 0.2))
            .into();
        let rp2040_outline = ['H', 'I', 'J', 'K']
            .into_iter()
            .map(|point| self.world_at(point));
        let rp2040: Shape = Wire::from_ordered_points(rp2040_outline)
            .unwrap()
            .to_face()
            .extrude(self.workplane.normal() * (13.5))
            .into();
        let jack_outline = ['L', 'M', 'N', 'O']
            .into_iter()
            .map(|point| self.world_at(point));
        let jack: Shape = Wire::from_ordered_points(jack_outline)
            .unwrap()
            .to_face()
            .extrude(self.workplane.normal() * (1.8 + 0.2 + 6.3 + 0.2))
            .into();
        let holes_cutout_outline = ['P', 'Q', 'F', 'G']
            .into_iter()
            .map(|point| self.world_at(point));
        let holes_cutout: Shape = Wire::from_ordered_points(holes_cutout_outline)
            .unwrap()
            .to_face()
            .extrude(self.workplane.normal() * (13.5))
            .into();
        Shape::union(&pcb, &rp2040)
            .union(&jack)
            .union(&holes_cutout)
            .into()
    }
}

// A simple sketch to trim a plane on the left side that we can print from
//      INF
// -INF C
//      |\
//      | \    0,0
//      |  \a
// (z)  |   b
// ^    |    \
// |    B-----A
// o--> (x)
struct XZMatrixSketch {
    workplane: Workplane,
    construction_points: Vec<DVec2>,
}

impl XZMatrixSketch {
    fn new(switch_matrix: &Vec<Vec<Switch>>, neg_y_translation: f64) -> Self {
        let workplane = Workplane::xz().translated(dvec3(0., 0., neg_y_translation));
        use Direction::*;
        // We pick PosZ for all of them, because we wouldn't want to trim our switch footprint,
        // now. Would we?
        let a = switch_matrix[0][0].coordinate(NegX, NegY, PosZ);
        let b = switch_matrix[0][0].coordinate(NegX, NegY, NegZ);
        let c = switch_matrix[1][2].coordinate(NegX, PosY, NegZ);
        let d = switch_matrix[1][2].coordinate(PosX, PosY, NegZ);
        let e = switch_matrix[2][2].coordinate(NegX, PosY, NegZ);
        let construction_points = [a, b, c, d, e]
            .iter()
            .map(|point| project_to_plane(&workplane, *point))
            .collect();
        Self {
            workplane,
            construction_points,
        }
    }
    fn local_at(&self, reference: char) -> DVec2 {
        let index = (reference as u8 - 'a' as u8) as usize;
        self.construction_points[index]
    }
    fn world_at(&self, reference: char) -> DVec3 {
        let local = match reference {
            'A' => dvec2(
                // TODO: parameterize this magic number. The idea is to have
                // at least 3 * COL_CURV angle so we can print from this side
                self.local_at('a').x + 10. * Angle::Degrees(15. + 18.).radians().cos(),
                0.,
            ),
            'B' => dvec2(-VIRTUAL_INFINITY, -VIRTUAL_INFINITY),
            'C' => dvec2(
                self.local_at('a').x - VIRTUAL_INFINITY * Angle::Degrees(15. + 18.).radians().cos(),
                VIRTUAL_INFINITY,
            ),
            _ => self.local_at(reference),
        };
        self.workplane.to_world_pos(dvec3(local.x, local.y, 0.))
    }
    fn wedge_cutout(&self) -> Shape {
        let wedge_outline = ['A', 'B', 'C']
            .into_iter()
            .map(|point| self.world_at(point));
        Wire::from_ordered_points(wedge_outline)
            .unwrap()
            .to_face()
            .extrude(self.workplane.normal() * VIRTUAL_INFINITY)
            .into()
    }
    fn jack_access(&self) -> Shape {
        let radius = 6.3 / 2.;
        let jack_outline = self.workplane.circle(
            self.local_at('e').x + 42.7 + 0.1 - 1.1,
            1.8 + 0.2 + radius,
            radius,
        );
        jack_outline
            .to_face()
            .extrude(self.workplane.normal() * -1. * VIRTUAL_INFINITY)
            .into()
    }
    fn usb_access(&self) -> Shape {
        let radius = 10.5 / 2.;
        let usb_outline =
            self.workplane
                .circle(self.local_at('e').x + 51.0 / 2. - 1.1, 7.0, radius);
        usb_outline
            .to_face()
            .extrude(self.workplane.normal() * -1. * VIRTUAL_INFINITY)
            .into()
    }
}

impl Keyboard {
    pub fn new(hand: Hand) -> Self {
        let col_x = geometry::col_x_positions();
        let col_y = geometry::col_y_positions();
        let col_z = geometry::col_z_positions();
        Self {
            switch_matrix: (0usize..NUM_COLS)
                .map(|col| {
                    // TODO (Pierre): it feels like some of the geometry expressed here should live in
                    // `geometry` instead and this should be  simple loop.
                    // That would make the number of rows easier to parameterize when the time
                    // comes.
                    let mid_switch_pos = dvec3(col_x[col], col_y[col], col_z[col]);
                    let col_rotation = rvec(
                        ROW_ANGLES_X[1],
                        Angle::Degrees(-COL_ANGLES_Y[col].degrees()),
                        Angle::Degrees(0.),
                    );
                    let mid_plane = Workplane::xy().transformed(mid_switch_pos, col_rotation);
                    let mid_switch = match (hand, col) {
                        (Hand::Right, 0) => Switch::with_extra_depth(mid_plane.clone(), hand, 0.5),
                        _ => Switch::new(mid_plane.clone(), hand),
                    };
                    let top_switch_pos = mid_switch.to_world_pos(dvec3(
                        0.,
                        geometry::row_delta_y(),
                        geometry::row_delta_z(),
                    ));
                    let top_switch_rotation = rvec(
                        Angle::Degrees(ROW_ANGLES_X[2].degrees()),
                        Angle::Degrees(
                            -COL_ANGLES_Y[col].degrees() * ROW_ANGLES_X[2].radians().cos(),
                        ),
                        Angle::Degrees(
                            COL_ANGLES_Y[col].degrees() * ROW_ANGLES_X[2].radians().sin(),
                        ),
                    );
                    let top_plane =
                        Workplane::xy().transformed(top_switch_pos, top_switch_rotation);
                    let top_switch = match (hand, col) {
                        (Hand::Right, 0) => Switch::with_extra_depth(top_plane.clone(), hand, 3.5),
                        _ => Switch::new(top_plane.clone(), hand),
                    };
                    let bottom_switch_pos = mid_switch.to_world_pos(dvec3(
                        0.,
                        -geometry::row_delta_y(),
                        geometry::row_delta_z(),
                    ));
                    let bottom_switch_rotation = rvec(
                        Angle::Degrees(ROW_ANGLES_X[0].degrees()),
                        Angle::Degrees(
                            -COL_ANGLES_Y[col].degrees() * ROW_ANGLES_X[0].radians().cos(),
                        ),
                        Angle::Degrees(
                            COL_ANGLES_Y[col].degrees() * ROW_ANGLES_X[0].radians().sin(),
                        ),
                    );
                    let bottom_plane =
                        Workplane::xy().transformed(bottom_switch_pos, bottom_switch_rotation);
                    let bottom_switch = match (hand, col) {
                        (Hand::Right, 0) => {
                            Switch::with_extra_depth(bottom_plane.clone(), hand, 1.5)
                        }
                        _ => Switch::new(bottom_plane.clone(), hand),
                    };
                    vec![bottom_switch, mid_switch, top_switch]
                })
                .collect(),
            thumbs: {
                let thumbs_rotation = THUMB0_ROTATION;
                let thumb0_position = THUMB0_XYZ;
                let workplane0 = Workplane::xy().transformed(thumb0_position, thumbs_rotation);
                let thumb0 = Switch::new(workplane0, hand);
                let thumb1_position = thumb0.to_world_pos(dvec3(SWITCH_PLATE_XYZ.x, 0., 0.));
                let workplane1 = Workplane::xy().transformed(thumb1_position, thumbs_rotation);
                let thumb1 = Switch::new(workplane1, hand);
                vec![thumb0, thumb1]
            },
            hand,
        }
    }
    /// This is the shape we want around the keyboard in the XY axix, projected to the plane that
    /// follows the bottom of the lowest switch
    fn top_xy_matrix_wire(&self) -> Wire {
        let center_switch = &self.switch_matrix[2][1];
        let mut workplane = center_switch.top_plane();
        let translation = workplane.to_world_pos(dvec3(0., 0., 100.)); // TODO: calculate rigorously to intersect the top of the topmost
                                                                       // switch
        workplane.translate_by(translation);
        XYMatrixSketch::new(&workplane, &self.switch_matrix, &self.thumbs).wire()
    }
    fn mid_xy_matrix_wire(&self) -> Wire {
        let workplane = self.switch_matrix[2][1].bottom_plane();
        XYMatrixSketch::new(&workplane, &self.switch_matrix, &self.thumbs).wire()
    }
    fn bottom_xy_matrix_wire(&self) -> Wire {
        let workplane = Workplane::xy();
        XYMatrixSketch::new(&workplane, &self.switch_matrix, &self.thumbs).wire()
    }
    fn thumbs_xy_cutout_bottom_wire(&self) -> Wire {
        let workplane = Workplane::xy().transformed(THUMB0_XYZ, THUMB0_ROTATION);
        XYMatrixSketch::new(&workplane, &self.switch_matrix, &self.thumbs).thumbs_cutout()
    }
    fn thumbs_xy_cutout_top_wire(&self) -> Wire {
        let bottom_right_switch = &self.switch_matrix[4][0];
        let /*mut*/ workplane = bottom_right_switch.top_plane();
        //let translation = workplane.to_world_pos(dvec3(0., 0., 100.)); // TODO: calculate rigorously to intersect the top of the topmost
        // switch
        //workplane.translate_by(translation);
        XYMatrixSketch::new(&workplane, &self.switch_matrix, &self.thumbs).thumbs_cutout()
    }
    fn electronics_cutout(&self) -> Shape {
        let workplane = Workplane::xy();
        XYMatrixSketch::new(&workplane, &self.switch_matrix, &self.thumbs).electronics_cutout()
    }
    fn loft_switches(
        btm: &Switch,
        top: &Switch,
        margin_left: Option<f64>,
        margin_right: Option<f64>,
    ) -> Shape {
        use Direction::*;
        // Describe a triangle, looking from the left that we can extrude to loft
        let btm_right = btm.coordinate(NegX, PosY, PosZ);
        let btm_left = top.coordinate(NegX, NegY, PosZ);
        let top_right = btm.to_world_pos(dvec3(
            -SWITCH_PLATE_XYZ.x / 2.,
            SWITCH_PLATE_XYZ.y / 2.,
            VIRTUAL_INFINITY,
        ));
        let top_left = top.to_world_pos(dvec3(
            -SWITCH_PLATE_XYZ.x / 2.,
            -SWITCH_PLATE_XYZ.y / 2.,
            VIRTUAL_INFINITY,
        ));
        let top_pt = (top_right + top_left) / 2.;
        let triangle = Wire::from_ordered_points([btm_left, btm_right, top_pt])
            .unwrap()
            .to_face();
        let shape: Shape = triangle
            .extrude((SWITCH_PLATE_XYZ.x + margin_right.unwrap_or(0.)) * btm.workplane.x_dir())
            .into();
        let shape: Shape = (&shape)
            .union(
                &triangle
                    .extrude(
                        (SWITCH_PLATE_XYZ.x + margin_right.unwrap_or(0.)) * top.workplane.x_dir(),
                    )
                    .into(),
            )
            .into();
        if let Some(margin_left) = margin_left {
            (&shape)
                .union(
                    &triangle
                        .extrude(-1. * margin_left * top.workplane.x_dir())
                        .into(),
                )
                .union(
                    &triangle
                        .extrude(-1. * margin_left * btm.workplane.x_dir())
                        .into(),
                )
                .into()
        } else {
            shape
        }
    }
    fn switch_base_pos(&self, col: usize, row: isize, is_left: bool) -> DVec3 {
        let coord_2_2 =
            self.switch_matrix[2][2].coordinate(Direction::NegX, Direction::PosY, Direction::NegZ);
        dvec3(coord_2_2.x - 1.1, coord_2_2.y, 1.8 + 0.2)
            + dvec3(
                0.1 + 2.5
                    + col as f64 * 10.
                    + match self.hand {
                        Hand::Left => {
                            if is_left {
                                0.
                            } else {
                                5.
                            }
                        }
                        Hand::Right => {
                            if is_left {
                                5.
                            } else {
                                0.
                            }
                        }
                    },
                -1. * (0.1 + 2. + 31. + (2. - row as f64) * 5.),
                0.,
            )
    }
    fn cylinder_connecting_two_points(a: DVec3, b: DVec3, radius: f64) -> Shape {
        let a_sphere = Shape::sphere(radius).at(a).build();
        let b_sphere = Shape::sphere(radius).at(b).build();
        let cylinder: Shape = Shape::cylinder_from_points(a, b, radius).into();
        a_sphere.union(&cylinder).union(&b_sphere).into()
    }
    // I know I made a horrible mess. Just trying to route the wires somewhat automatically so they
    // don't create issues.
    // Don't judge me, please. Or do. I deserve it. And I don't care :p
    fn wire_holes(&self) -> Shape {
        let wire_radius = SWITCH_FOOTPRINT_LEFT_WIRES[0].2;
        let mut shapes = (0..NUM_COLS)
            .flat_map(|col| {
                (0..NUM_ROWS)
                    .map(|row| {
                        let switch_top_left = self.switch_matrix[col][row].left_wire_coord();
                        let switch_bottom_left = self.switch_base_pos(col, row as isize, true);
                        let switch_top_right = self.switch_matrix[col][row].right_wire_coord();
                        let switch_bottom_right = self.switch_base_pos(col, row as isize, false);

                        let mut above_bottom_left = switch_bottom_left.clone();
                        above_bottom_left.z += if col == 0 {
                            match self.hand {
                                Hand::Left => 3.,
                                Hand::Right => 4.,
                            }
                        } else if col == 1 {
                            match self.hand {
                                Hand::Left => 6.,
                                Hand::Right => 7.,
                            }
                        } else if col == 2 {
                            match self.hand {
                                Hand::Left => 9.,
                                Hand::Right => 10.,
                            }
                        } else {
                            13.5
                        };
                        let mut above_bottom_right = switch_bottom_right.clone();
                        above_bottom_right.z += if col == 0 {
                            match self.hand {
                                Hand::Left => 4.,
                                Hand::Right => 3.,
                            }
                        } else if col == 1 {
                            match self.hand {
                                Hand::Left => 7.,
                                Hand::Right => 6.,
                            }
                        } else if col == 2 {
                            match self.hand {
                                Hand::Left => 10.,
                                Hand::Right => 9.,
                            }
                        } else {
                            13.5
                        };
                        Self::cylinder_connecting_two_points(
                            switch_bottom_left,
                            above_bottom_left,
                            wire_radius + 0.5,
                        )
                        .union(&Self::cylinder_connecting_two_points(
                            above_bottom_left,
                            switch_top_left,
                            wire_radius,
                        ))
                        .union(&Self::cylinder_connecting_two_points(
                            switch_bottom_right,
                            above_bottom_right,
                            wire_radius + 0.5,
                        ))
                        .union(&Self::cylinder_connecting_two_points(
                            above_bottom_right,
                            switch_top_right,
                            wire_radius,
                        ))
                        .into()
                    })
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<Shape>>();
        let mut union = shapes.pop().expect("We have more than zero switches");
        for shape in shapes {
            union = union.union(&shape).into();
        }
        let thumb0_base_left = self.switch_base_pos(3, -1, true);
        let mut thumb0_above_base_left = thumb0_base_left.clone();
        thumb0_above_base_left.z += 2.;
        let thumb0_top_left = self.thumbs[0].left_wire_coord();
        let thumb0_base_right = self.switch_base_pos(3, -1, false);
        let mut thumb0_above_base_right = thumb0_base_right.clone();
        thumb0_above_base_right.z += 2.;
        let thumb0_top_right = self.thumbs[0].right_wire_coord();
        let thumb1_base_left = self.switch_base_pos(4, -1, true);
        let mut thumb1_above_base_left = thumb1_base_left.clone();
        thumb1_above_base_left.z += 2.;
        let thumb1_top_left = self.thumbs[1].left_wire_coord();
        let thumb1_base_right = self.switch_base_pos(4, -1, false);
        let mut thumb1_above_base_right = thumb1_base_right.clone();
        thumb1_above_base_right.z += 2.;
        let thumb1_top_right = self.thumbs[1].right_wire_coord();
        union
            .union(&Self::cylinder_connecting_two_points(
                thumb0_base_left,
                thumb0_above_base_left,
                wire_radius,
            ))
            .union(&Self::cylinder_connecting_two_points(
                thumb0_above_base_left,
                thumb0_top_left,
                wire_radius,
            ))
            .union(&Self::cylinder_connecting_two_points(
                thumb0_base_right,
                thumb0_above_base_right,
                wire_radius,
            ))
            .union(&Self::cylinder_connecting_two_points(
                thumb0_above_base_right,
                thumb0_top_right,
                wire_radius,
            ))
            .union(&Self::cylinder_connecting_two_points(
                thumb1_base_left,
                thumb1_above_base_left,
                wire_radius,
            ))
            .union(&Self::cylinder_connecting_two_points(
                thumb1_above_base_left,
                thumb1_top_left,
                wire_radius,
            ))
            .union(&Self::cylinder_connecting_two_points(
                thumb1_base_right,
                thumb1_above_base_right,
                wire_radius,
            ))
            .union(&Self::cylinder_connecting_two_points(
                thumb1_above_base_right,
                thumb1_top_right,
                wire_radius,
            ))
            .into()
    }
    pub fn shape(&self) -> Shape {
        let mut shape: Shape = Solid::loft([
            self.bottom_xy_matrix_wire(),
            self.mid_xy_matrix_wire(),
            self.top_xy_matrix_wire(),
        ])
        .into();
        for (col_index, col) in self.switch_matrix.iter().enumerate() {
            let margin_left = match col_index {
                0 => Some(VIRTUAL_INFINITY),
                2 | 3 | 4 => Some(1.),
                _ => None,
            };
            let margin_right = match col_index {
                0 | 1 | 2 => Some(1.),
                4 => Some(VIRTUAL_INFINITY),
                _ => None,
            };
            for (row_index, switch) in col.iter().enumerate() {
                let margin_top = if row_index == NUM_ROWS - 1 {
                    Some(VIRTUAL_INFINITY)
                } else {
                    None
                };
                let margin_bottom = if row_index == 0 {
                    Some(VIRTUAL_INFINITY)
                } else {
                    None
                };
                shape = switch.punch(shape, margin_left, margin_right, margin_top, margin_bottom);
            }
            let loft = Self::loft_switches(
                &self.switch_matrix[col_index][0],
                &self.switch_matrix[col_index][1],
                margin_left,
                margin_right,
            );
            shape = shape.subtract(&loft).into();
            let loft = Self::loft_switches(
                &self.switch_matrix[col_index][1],
                &self.switch_matrix[col_index][2],
                margin_left,
                margin_right,
            );
            shape = shape.subtract(&loft).into();
        }
        for thumb in self.thumbs.iter() {
            shape = thumb.punch(shape, Some(3.), Some(5.), Some(3.), Some(VIRTUAL_INFINITY));
        }
        let thumbs_cutout: Shape = Solid::loft([
            self.thumbs_xy_cutout_bottom_wire(),
            self.thumbs_xy_cutout_top_wire(),
        ])
        .into();

        let coord_2_2 =
            self.switch_matrix[2][2].coordinate(Direction::NegX, Direction::PosY, Direction::NegZ);

        let y_for_cutouts = -1. * (coord_2_2.y - 3.);

        let wire_holes = self.wire_holes();

        shape = shape
            .subtract(
                &XZMatrixSketch::new(&self.switch_matrix, -VIRTUAL_INFINITY / 2.).wedge_cutout(),
            )
            .subtract(&XZMatrixSketch::new(&self.switch_matrix, y_for_cutouts).jack_access())
            .subtract(&XZMatrixSketch::new(&self.switch_matrix, y_for_cutouts).usb_access())
            .subtract(&thumbs_cutout)
            .subtract(&self.electronics_cutout())
            .subtract(&wire_holes)
            .into();
        shape
    }
}

#[cfg(test)]
mod test {
    use super::*;
    use opencascade::primitives::Shape;
    #[test]
    fn test_keyboard_shape() {
        let shape: Shape = Keyboard::new(Hand::Left).shape();
        shape.write_stl("test_keyboard_shape.stl").unwrap();
    }
    #[test]
    fn test_xy_matrix_sketch() {
        let keyboard = Keyboard::new(Hand::Left);
        let shape: Shape =
            XYMatrixSketch::new(&Workplane::xy(), &keyboard.switch_matrix, &keyboard.thumbs)
                .wire()
                .to_face()
                .into();
        shape.write_stl("test_xy_matrix_sketch.stl").unwrap();
    }
    #[test]
    fn test_xz_wedge_cutout() {
        let shape: Shape = XZMatrixSketch::new(
            &Keyboard::new(Hand::Left).switch_matrix,
            -VIRTUAL_INFINITY / 2.,
        )
        .wedge_cutout();
        shape.write_stl("test_xz_wedge_cutout.stl").unwrap();
    }
    #[test]
    fn test_xz_jack_access() {
        let shape: Shape =
            XZMatrixSketch::new(&Keyboard::new(Hand::Left).switch_matrix, -20.).jack_access();
        shape.write_stl("test_xz_jack_access.stl").unwrap();
    }
    #[test]
    fn test_wire_holes_left() {
        let shape: Shape = Keyboard::new(Hand::Left).wire_holes();
        shape.write_stl("test_wire_holes_left.stl").unwrap();
    }
    #[test]
    fn test_wire_holes_right() {
        let shape: Shape = Keyboard::new(Hand::Right).wire_holes();
        shape.write_stl("test_wire_holes_right.stl").unwrap();
    }
    #[test]
    fn test_thumbs_cutout() {
        let keyboard = Keyboard::new(Hand::Left);
        let shape: Shape = Solid::loft([
            keyboard.thumbs_xy_cutout_bottom_wire(),
            keyboard.thumbs_xy_cutout_top_wire(),
        ])
        .into();
        shape.write_stl("test_thumbs_cutout.stl").unwrap();
    }
}
