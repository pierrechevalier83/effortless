use glam::DVec3;
use opencascade::angle::{Angle, RVec};

// Choc switch dimensions (mm)
pub(crate) const SWITCH_PLATE_XYZ: DVec3 = DVec3::new(18., 17., 6.);
pub(crate) const SWITCH_HOLE_XYZ: DVec3 = DVec3::new(14., 14., 2.2);

// Keycap dimensions (mm) for calculating switch postitions
// and avoiding interference
pub(crate) const KEYCAP_XYZ: DVec3 = DVec3::new(17.5, 16.5, 8.);

// Number of switches
pub(crate) const NUM_COLS: usize = 5;
pub(crate) const NUM_ROWS: usize = 3;
#[allow(unused)]
pub(crate) const NUM_THUMBS: usize = 2;

// Angles
pub(crate) const COL_CURV_DEG: f64 = 6.;
pub(crate) const ROW_CURV_DEG: f64 = 17.;
pub(crate) const TENTING_DEG: f64 = 15.;
pub(crate) const COL_CURV: Angle = Angle::Degrees(COL_CURV_DEG);
pub(crate) const ROW_CURV: Angle = Angle::Degrees(ROW_CURV_DEG);
pub(crate) const TENTING: Angle = Angle::Degrees(TENTING_DEG);
pub(crate) const COL_ANGLES_Y: [Angle; NUM_COLS] = [
    TENTING,
    Angle::Degrees(TENTING_DEG + COL_CURV_DEG),
    Angle::Degrees(TENTING_DEG + 2. * COL_CURV_DEG),
    Angle::Degrees(TENTING_DEG + 3. * COL_CURV_DEG),
    Angle::Degrees(TENTING_DEG + 3. * COL_CURV_DEG), // This is not a typo: we prefer for both index columns to be flat
];
pub(crate) const ROW_ANGLES_X: [Angle; NUM_ROWS] = [
    Angle::Degrees(-ROW_CURV_DEG),
    Angle::Degrees(0.),
    Angle::Degrees(ROW_CURV_DEG),
];

// Column stagger (mm)
pub(crate) const COL_Y_STAGGER: [f64; NUM_COLS] = [-10., 0., 5., 0., 0.];
pub(crate) const COL_Z_STAGGER: [f64; NUM_COLS] = [6., -0.5, -3.5, 0., 0.];

// Thumb cluster
// Dimensions (mm)
#[allow(unused)]
pub(crate) const THUMB0_XYZ: DVec3 = DVec3::new(70., -50., 8.);
#[allow(unused)]
pub(crate) const THUMB0_ROTATION: RVec = RVec {
    x: Angle::Degrees(25.),
    y: Angle::Degrees(10.),
    z: Angle::Degrees(-15.),
};

// Choc v1 footprint (with tolerances)
// x, y, radius, depth
pub(crate) const SWITCH_FOOTPRINT_HOLES: [(f64, f64, f64, f64); 3] = [
    (0., 0., 3.4 / 2., 4.85),
    (-5.5, 0., 2.0 / 2., 4.85),
    (5.5, 0., 2.0 / 2., 4.85),
];
pub(crate) const SWITCH_FOOTPRINT_WIRES: [(f64, f64, f64, f64); 2] = [
    (0., -5.9, 2.2 / 2., SWITCH_PLATE_XYZ.z),
    (-5., -3.8, 2.2 / 2., SWITCH_PLATE_XYZ.z),
];

pub(crate) const VIRTUAL_INFINITY: f64 = 1_000.;

/*

// Used to turn 2d surfaces into 3d objects and to make holes just slightly bigger than what they're cutting
epsilon = 0.01;
// Pad dimensions
pad_xy = [
    12, // x (mm)
    8   // y (mm)
];

// Switch spacing
switch_spacing_xy = [
    0, // x (mm)
    0  // y (mm)
];

// Z-axis adjustments
z_adjustments = [
    -8, // bottommost point in z (mm)
    8   // height of the lip (mm)
];


*/
