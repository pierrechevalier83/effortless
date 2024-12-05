use crate::geometry;
use crate::params::{
    COL_ANGLES_Y, NUM_COLS, ROW_ANGLES_X, SWITCH_PLATE_XYZ, THUMB0_ROTATION, THUMB0_XYZ,
    VIRTUAL_INFINITY,
};
use crate::switch::Switch;
use glam::{dvec2, dvec3, DVec2, DVec3};
use opencascade::angle::{rvec, Angle};
use opencascade::primitives::{Direction, Edge, Shape, Solid, Wire, WireBuilder};
use opencascade::workplane::Workplane;

pub struct Keyboard {
    switch_matrix: Vec<Vec<Switch>>,
    thumbs: Vec<Switch>,
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
//       u     e-----f          v
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
// t-----s
struct ProjectedXYSketch {
    workplane: Workplane,
    construction_points: Vec<DVec2>,
}

impl ProjectedXYSketch {
    fn new(workplane: &Workplane, switch_matrix: &Vec<Vec<Switch>>) -> Self {
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
        let construction_points = [a, b, c, d, e, f, g, h, i, j, k, l, m, n, o, p, q, r, s, t]
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
            // TODO: parameterize this right margin
            'j' => dvec2(self.local_at('j').x + 5., self.local_at('j').y),
            'k' => dvec2(self.local_at('k').x + 5., self.local_at('k').y),
            'u' => dvec2(self.local_at('c').x, self.local_at('e').y),
            'v' => dvec2(self.local_at('i').x, self.local_at('f').y),
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
        let k = self.world_at('k');
        let l = self.world_at('l');
        let m = self.world_at('m');
        let q = self.world_at('q');
        let r = self.world_at('r');
        let s = self.world_at('s');
        let t = self.world_at('t');
        let u = self.world_at('u');
        let v = self.world_at('v');

        let mut builder = WireBuilder::new();
        builder.add_edge(&Edge::segment(e, v));
        builder.add_edge(&Edge::spline_from_points(
            vec![v, j, k, l],
            Some(((v - f).normalize(), (m - l).normalize())),
        ));
        builder.add_edge(&Edge::segment(l, q));
        builder.add_edge(&Edge::spline_from_points(
            vec![q, s, t],
            Some(((r - q).normalize(), (q - t).normalize())),
        ));
        builder.add_edge(&Edge::segment(t, a));
        builder.add_edge(&Edge::spline_from_points(
            vec![a, c, e],
            Some(((c - a).normalize(), (e - u).normalize())),
        ));
        builder.build()
    }
}

// A simple sketch to trim the top of the XYSketches' loft
// -INF m--------------------------------------l INF
//      |                                      |
//      |                                      |
//      |     0,0                              |
//      |            1,0             3,0   4,0 |
//      |   b-----c        2,0                 |
// (z)  |   a     d-----e      h-----i----j----k INF
// ^    |    \          f-----g
// |    n-----o
// o--> (x)
struct XZSketch {
    workplane: Workplane,
    construction_points: Vec<DVec2>,
}

impl XZSketch {
    fn new(switch_matrix: &Vec<Vec<Switch>>) -> Self {
        let workplane = Workplane::xz().translated(dvec3(0., 0., -VIRTUAL_INFINITY / 2.));
        use Direction::*;
        // We pick PosZ for all of them, because we wouldn't want to trim our switch footprint,
        // now. Would we?
        let a = switch_matrix[0][0].coordinate(NegX, NegY, NegZ);
        let b = switch_matrix[0][0].coordinate(NegX, NegY, PosZ);
        let c = switch_matrix[0][0].coordinate(PosX, NegY, PosZ);
        let d = switch_matrix[1][0].coordinate(NegX, NegY, PosZ);
        let e = switch_matrix[1][0].coordinate(PosX, NegY, PosZ);
        let f = switch_matrix[2][0].coordinate(NegX, NegY, PosZ);
        let g = switch_matrix[2][0].coordinate(PosX, NegY, PosZ);
        let h = switch_matrix[3][0].coordinate(NegX, NegY, PosZ);
        let i = switch_matrix[3][0].coordinate(PosX, NegY, PosZ);
        let j = switch_matrix[4][0].coordinate(PosX, NegY, PosZ);
        let construction_points = [a, b, c, d, e, f, g, h, i, j]
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
            'k' => dvec2(VIRTUAL_INFINITY, self.local_at('j').y),
            'l' => dvec2(VIRTUAL_INFINITY, VIRTUAL_INFINITY),
            'm' => dvec2(-VIRTUAL_INFINITY, VIRTUAL_INFINITY),
            'n' => dvec2(-VIRTUAL_INFINITY, 0.),
            // TODO: parameterize this magic number. The idea is to have
            // at least 3 * COL_CURV angle so we can print from this side
            'o' => dvec2(
                self.local_at('a').x + 10. * Angle::Degrees(15. + 18.).radians().cos(),
                0.,
            ),
            _ => self.local_at(reference),
        };
        self.workplane.to_world_pos(dvec3(local.x, local.y, 0.))
    }
    fn shape(&self) -> Shape {
        let outline = [
            'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o',
        ]
        .into_iter()
        .map(|point| self.world_at(point));
        Wire::from_ordered_points(outline)
            .unwrap()
            .to_face()
            .extrude(self.workplane.normal() * VIRTUAL_INFINITY)
            .into()
    }
}

// A simple sketch to trim the leftofver on to the right of the right index column
//  INF                          INF
//  h----------------------------g
//  | 4,0                    4,2 |
//  a----b                 e-----f
//          \   4, 1    /
//            c----d
//  ^ (z)
//  |
//  o--> (y)
struct YZSketch {
    workplane: Workplane,
    construction_points: Vec<DVec2>,
}

impl YZSketch {
    fn new(switch_matrix: &Vec<Vec<Switch>>) -> Self {
        let workplane = Workplane::yz();
        use Direction::*;
        let a = switch_matrix[4][0].coordinate(PosX, NegY, PosZ);
        let b = switch_matrix[4][0].coordinate(PosX, PosY, PosZ);
        let c = switch_matrix[4][1].coordinate(PosX, NegY, PosZ);
        let d = switch_matrix[4][1].coordinate(PosX, PosY, PosZ);
        let e = switch_matrix[4][2].coordinate(PosX, NegY, PosZ);
        let f = switch_matrix[4][2].coordinate(PosX, PosY, PosZ);
        let construction_points = [a, b, c, d, e, f]
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
            'g' => dvec2(self.local_at('f').x, VIRTUAL_INFINITY),
            'h' => dvec2(self.local_at('a').x, VIRTUAL_INFINITY),
            _ => self.local_at(reference),
        };
        self.workplane.to_world_pos(dvec3(local.x, local.y, 0.))
    }
    fn shape(&self) -> Shape {
        let a = self.world_at('a');
        let d = self.world_at('d');
        let f = self.world_at('f');
        let g = self.world_at('g');
        let h = self.world_at('h');
        let mut builder = WireBuilder::new();
        builder.add_edge(&Edge::segment(a, h));
        builder.add_edge(&Edge::segment(h, g));
        builder.add_edge(&Edge::segment(g, f));
        builder.add_edge(&Edge::arc(a, d, f));
        builder
            .build()
            .to_face()
            .extrude(self.workplane.normal() * VIRTUAL_INFINITY)
            .into()
    }
}

impl Keyboard {
    pub fn new() -> Self {
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
                    let mid_switch = Switch::new(mid_plane.clone());
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
                    let top_switch = Switch::new(top_plane);
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
                    let bottom_switch = Switch::new(bottom_plane);
                    vec![bottom_switch, mid_switch, top_switch]
                })
                .collect(),
            thumbs: {
                let thumbs_rotation = THUMB0_ROTATION;
                let thumb0_position = THUMB0_XYZ;
                let workplane0 = Workplane::xy().transformed(thumb0_position, thumbs_rotation);
                let thumb0 = Switch::new(workplane0);
                let thumb1_position = thumb0.to_world_pos(dvec3(0., SWITCH_PLATE_XYZ.y, 0.));
                let workplane1 = Workplane::xy().transformed(thumb1_position, thumbs_rotation);
                let thumb1 = Switch::new(workplane1);
                vec![thumb0, thumb1]
            },
        }
    }
    /// This is the shape we want around the keyboard in the XY axix, projected to the plane that
    /// follows the bottom of the lowest switch
    fn top_xy_wire(&self) -> Wire {
        let center_switch = &self.switch_matrix[2][1];
        let mut workplane = center_switch.top_plane();
        let translation = workplane.to_world_pos(dvec3(0., 0., 100.)); // TODO: calculate rigorously to intersect the top of the topmost
                                                                       // switch
        workplane.translate_by(translation);
        ProjectedXYSketch::new(&workplane, &self.switch_matrix).wire()
    }
    fn mid_xy_wire(&self) -> Wire {
        let workplane = self.switch_matrix[2][1].bottom_plane();
        ProjectedXYSketch::new(&workplane, &self.switch_matrix).wire()
    }
    fn bottom_xy_wire(&self) -> Wire {
        // TODO: calculate for good angle for 3d printng on this edge
        let workplane = Workplane::xy(); //.translated(dvec3(50., 0., 0.));
        ProjectedXYSketch::new(&workplane, &self.switch_matrix).wire()
    }
    fn loft_switches(btm: &Switch, top: &Switch, margin_left: bool, margin_right: bool) -> Shape {
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
            .extrude(
                (SWITCH_PLATE_XYZ.x + if margin_right { 1. } else { 0. }) * btm.workplane.x_dir(),
            )
            .into();
        let shape: Shape = (&shape)
            .union(
                &triangle
                    .extrude(
                        (SWITCH_PLATE_XYZ.x + if margin_right { 1. } else { 0. })
                            * top.workplane.x_dir(),
                    )
                    .into(),
            )
            .into();
        if margin_left {
            (&shape)
                .union(&triangle.extrude(-1. * top.workplane.x_dir()).into())
                .union(&triangle.extrude(-1. * btm.workplane.x_dir()).into())
                .into()
        } else {
            shape
        }
    }
    pub fn shape(&self) -> Shape {
        let mut shape: Shape = Solid::loft([
            self.bottom_xy_wire(),
            self.mid_xy_wire(),
            self.top_xy_wire(),
        ])
        .into();
        for (col_index, col) in self.switch_matrix.iter().enumerate() {
            let margin_left = match col_index {
                0 | 3 => true,
                _ => false,
            };
            let margin_right = match col_index {
                0 | 1 => true,
                _ => false,
            };
            for switch in col.iter() {
                shape = switch.punch(shape, margin_left, margin_right);
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
        shape = shape
            .subtract(&XZSketch::new(&self.switch_matrix).shape())
            .subtract(&YZSketch::new(&self.switch_matrix).shape())
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
        let shape: Shape = Keyboard::new().shape();
        shape.write_stl("test_keyboard_shape.stl").unwrap();
    }
    #[test]
    fn test_xy_sketch() {
        let shape: Shape = ProjectedXYSketch::new(&Workplane::xy(), &Keyboard::new().switch_matrix)
            .wire()
            .to_face()
            .into();
        shape.write_stl("test_xy_sketch.stl").unwrap();
    }
    #[test]
    fn test_yz_sketch() {
        let shape: Shape = YZSketch::new(&Keyboard::new().switch_matrix).shape();
        shape.write_stl("test_yz_sketch.stl").unwrap();
    }
    #[test]
    fn test_xz_sketch() {
        let shape: Shape = XZSketch::new(&Keyboard::new().switch_matrix).shape();
        shape.write_stl("test_xz_sketch.stl").unwrap();
    }
}
