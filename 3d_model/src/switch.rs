use crate::params::{
    SWITCH_FOOTPRINT_HOLES, SWITCH_FOOTPRINT_LEFT_WIRES, SWITCH_FOOTPRINT_RIGHT_WIRES,
    SWITCH_HOLE_XYZ, SWITCH_PLATE_XYZ, VIRTUAL_INFINITY,
};
use glam::{dvec3, DVec3};
#[cfg(test)]
use opencascade::angle::RVec;
#[cfg(test)]
use opencascade::primitives::Face;
use opencascade::primitives::{Direction, Shape, Wire};
use opencascade::workplane::Workplane;
#[cfg(test)]
use opencascade::Error;

fn centered_rectangle(workplane: Workplane, x: f64, y: f64) -> Wire {
    centered_rectangle_with_optional_margins(workplane, x, y, None, None, None, None)
}

fn centered_rectangle_with_optional_margins(
    workplane: Workplane,
    x: f64,
    y: f64,
    margin_left: Option<f64>,
    margin_right: Option<f64>,
    margin_top: Option<f64>,
    margin_bottom: Option<f64>,
) -> Wire {
    workplane
        .sketch()
        .move_to(
            -x / 2. - margin_left.unwrap_or(0.),
            -y / 2. - margin_bottom.unwrap_or(0.),
        ) // Bottom Left
        .line_to(
            -x / 2. - margin_left.unwrap_or(0.),
            y / 2. + margin_top.unwrap_or(0.),
        ) // Top Left
        .line_to(
            x / 2. + margin_right.unwrap_or(0.),
            y / 2. + margin_top.unwrap_or(0.),
        ) // Top Right
        .line_to(
            x / 2. + margin_right.unwrap_or(0.),
            -y / 2. - margin_bottom.unwrap_or(0.),
        ) // Bottom Right
        .close()
}

#[derive(Clone, Copy)]
pub enum Hand {
    Left,
    Right,
}

pub struct Switch {
    pub workplane: Workplane,
    hand: Hand,
    extra_depth: Option<f64>,
}

impl Switch {
    // Create a switch at a certain position, with a certain rotation.
    // The position will be the center of the top face of the switch.
    pub fn new(workplane: Workplane, hand: Hand) -> Self {
        //position: DVec3, rotation: RVec) -> Self {
        Self {
            workplane,
            hand,
            extra_depth: None,
        }
    }
    pub fn with_extra_depth(workplane: Workplane, hand: Hand, extra_depth: f64) -> Self {
        //position: DVec3, rotation: RVec) -> Self {
        Self {
            workplane,
            hand,
            extra_depth: Some(extra_depth),
        }
    }
    #[cfg(test)]
    fn shape(&self) -> Shape {
        let plate = self
            .top_face()
            .extrude(-SWITCH_PLATE_XYZ.z * self.workplane.normal());
        self.punch(plate.into(), None, None, None, None)
    }
    fn all_space_above_the_switch(
        &self,
        margin_left: Option<f64>,
        margin_right: Option<f64>,
        margin_top: Option<f64>,
        margin_bottom: Option<f64>,
    ) -> Shape {
        let shape: Shape = centered_rectangle_with_optional_margins(
            self.workplane.clone(),
            SWITCH_PLATE_XYZ.x,
            SWITCH_PLATE_XYZ.y,
            margin_left,
            margin_right,
            margin_top,
            margin_bottom,
        )
        .to_face()
        .extrude(VIRTUAL_INFINITY * self.workplane.normal())
        .into();
        shape.clean()
    }
    pub fn punch(
        &self,
        shape: Shape,
        margin_left: Option<f64>,
        margin_right: Option<f64>,
        margin_top: Option<f64>,
        margin_bottom: Option<f64>,
    ) -> Shape {
        let all_space_above_the_switch =
            self.all_space_above_the_switch(margin_left, margin_right, margin_top, margin_bottom);
        let switch_body_cutout =
            centered_rectangle(self.workplane.clone(), SWITCH_HOLE_XYZ.x, SWITCH_HOLE_XYZ.y)
                .to_face()
                .extrude(-SWITCH_HOLE_XYZ.z * self.workplane.normal())
                .into();
        let holes = SWITCH_FOOTPRINT_HOLES
            .iter()
            .chain(
                match self.hand {
                    Hand::Left => SWITCH_FOOTPRINT_LEFT_WIRES,
                    Hand::Right => SWITCH_FOOTPRINT_RIGHT_WIRES,
                }
                .iter(),
            )
            .map(|(x, y, radius, depth)| {
                self.workplane
                    .clone()
                    .circle(*x, *y, *radius)
                    .to_face()
                    .extrude(-(depth + self.extra_depth.unwrap_or(0.)) * self.workplane.normal())
            })
            .collect::<Vec<_>>();

        let mut body = shape
            .subtract(&all_space_above_the_switch)
            .subtract(&switch_body_cutout);
        for hole in holes {
            body = body.subtract(&hole.into());
        }
        body.into()
    }
    #[cfg(test)]
    pub fn top_face(&self) -> Face {
        centered_rectangle(
            self.workplane.clone(),
            SWITCH_PLATE_XYZ.x,
            SWITCH_PLATE_XYZ.y,
        )
        .to_face()
    }
    pub fn coordinate(&self, x: Direction, y: Direction, z: Direction) -> DVec3 {
        let x = match x {
            Direction::PosX => SWITCH_PLATE_XYZ.x / 2.,
            Direction::NegX => -SWITCH_PLATE_XYZ.x / 2.,
            _ => panic!("Please, only use PosX or NegX for the x Direction"),
        };
        let y = match y {
            Direction::PosY => SWITCH_PLATE_XYZ.y / 2.,
            Direction::NegY => -SWITCH_PLATE_XYZ.y / 2.,
            _ => panic!("Please, only use PosY or NegY for the y Direction"),
        };
        let z = match z {
            Direction::PosZ => 0.,
            Direction::NegZ => -SWITCH_PLATE_XYZ.z,
            _ => panic!("Please, only use PosZ or NegZ for the z Direction"),
        };
        self.to_world_pos(dvec3(x, y, z))
    }
    pub fn left_wire_coord(&self) -> DVec3 {
        let (x, y, _r, depth) = match self.hand {
            Hand::Left => SWITCH_FOOTPRINT_LEFT_WIRES,
            Hand::Right => SWITCH_FOOTPRINT_RIGHT_WIRES,
        }[1];
        self.to_world_pos(dvec3(x, y, -depth))
    }
    pub fn right_wire_coord(&self) -> DVec3 {
        let (x, y, _r, depth) = match self.hand {
            Hand::Left => SWITCH_FOOTPRINT_LEFT_WIRES,
            Hand::Right => SWITCH_FOOTPRINT_RIGHT_WIRES,
        }[0];
        self.to_world_pos(dvec3(x, y, -depth))
    }
    pub fn to_world_pos(&self, point: DVec3) -> DVec3 {
        self.workplane.to_world_pos(point)
    }
    pub fn top_plane(&self) -> Workplane {
        self.workplane.clone()
    }
    pub fn bottom_plane(&self) -> Workplane {
        self.workplane
            .translated(self.to_world_pos(dvec3(0., 0., -SWITCH_PLATE_XYZ.z)))
    }
    #[cfg(test)]
    fn write_stl(&self, file: &str) -> Result<(), Error> {
        self.shape().write_stl(file)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use opencascade::angle::{rvec, Angle};
    #[test]
    fn test_one_centered_switch() {
        Switch::new(
            Workplane::xy().transformed(DVec3::ZERO, RVec::x(Angle::Degrees(0.))),
            Hand::Left,
        )
        .write_stl("test_one_centered_switch.stl")
        .unwrap();
    }
    #[test]
    fn test_one_uncentered_switch() {
        Switch::new(
            Workplane::xy().transformed(
                dvec3(0., 20., 30.),
                rvec(
                    Angle::Degrees(10.),
                    Angle::Degrees(-20.),
                    Angle::Degrees(0.),
                ),
            ),
            Hand::Left,
        )
        .write_stl("test_one_uncentered_switch.stl")
        .unwrap();
    }
}
