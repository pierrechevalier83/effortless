use crate::params::{
    COL_ANGLES_Y, COL_CURV, COL_Y_STAGGER, COL_Z_STAGGER, KEYCAP_XYZ, NUM_COLS, ROW_CURV,
    SWITCH_PLATE_XYZ,
};

fn col_delta_y() -> [f64; NUM_COLS] {
    let col_y = col_y_positions();
    [
        0.,
        col_y[1] - col_y[0],
        col_y[2] - col_y[1],
        col_y[3] - col_y[2],
        col_y[4] - col_y[3],
    ]
}
fn col_delta_z() -> [f64; NUM_COLS] {
    let stag_z = COL_Z_STAGGER;
    [
        0.,
        stag_z[1] - stag_z[0],
        stag_z[2] - stag_z[1],
        stag_z[3] - stag_z[2],
        stag_z[4] - stag_z[3],
    ]
}
pub fn col_y_positions() -> [f64; NUM_COLS] {
    COL_Y_STAGGER
}
pub fn col_x_positions() -> [f64; NUM_COLS] {
    let col0_x = SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[0].radians().cos()
        - col_delta_z()[1] * COL_ANGLES_Y[0].radians().sin()
        + (KEYCAP_XYZ.z + col_delta_y()[0] * ROW_CURV.radians().tan() + col_delta_z()[0])
            * COL_CURV.radians().sin();
    let col1_x = col0_x
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[0].radians().cos()
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[1].radians().cos()
        - col_delta_z()[1] * COL_ANGLES_Y[1].radians().sin()
        + (KEYCAP_XYZ.z + col_delta_y()[1] * ROW_CURV.radians().tan() + col_delta_z()[1])
            * COL_CURV.radians().sin()
            * COL_ANGLES_Y[0].radians().cos();
    let col2_x = col1_x
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[1].radians().cos()
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[2].radians().cos()
        - col_delta_z()[2] * COL_ANGLES_Y[2].radians().sin()
        + (KEYCAP_XYZ.z + col_delta_y()[2] * ROW_CURV.radians().tan() + col_delta_z()[2])
            * COL_CURV.radians().sin()
            * COL_ANGLES_Y[1].radians().cos();
    let col3_x = col2_x
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[2].radians().cos()
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[3].radians().cos()
        - col_delta_z()[3] * COL_ANGLES_Y[3].radians().sin()
        + (KEYCAP_XYZ.z + col_delta_y()[3].abs() * ROW_CURV.radians().tan() - col_delta_z()[3])
            * COL_CURV.radians().sin()
            * COL_ANGLES_Y[2].radians().cos();
    let col4_x = col3_x
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[3].radians().cos()
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[4].radians().cos();
    [col0_x, col1_x, col2_x, col3_x, col4_x]
}
pub fn col_z_positions() -> [f64; NUM_COLS] {
    let col0_z = 10. /* TODO: Z0? */ + COL_Z_STAGGER[0] * COL_ANGLES_Y[1].radians().cos();
    let col1_z = col0_z
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[0].radians().sin()
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[1].radians().sin()
        + col_delta_z()[1] * COL_ANGLES_Y[0].radians().cos()
        + (KEYCAP_XYZ.z + col_delta_y()[1] * ROW_CURV.radians().tan() + col_delta_z()[1])
            * COL_CURV.radians().sin()
            * COL_ANGLES_Y[0].radians().sin();
    let col2_z = col1_z
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[1].radians().sin()
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[2].radians().sin()
        + col_delta_z()[2] * COL_ANGLES_Y[1].radians().cos()
        + (KEYCAP_XYZ.z + col_delta_y()[2] * ROW_CURV.radians().tan() + col_delta_z()[2])
            * COL_CURV.radians().sin()
            * COL_ANGLES_Y[1].radians().sin();
    let col3_z = col2_z
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[2].radians().sin()
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[3].radians().sin()
        + col_delta_z()[3] * COL_ANGLES_Y[2].radians().cos()
        + (KEYCAP_XYZ.z + col_delta_y()[3].abs() * ROW_CURV.radians().tan() - col_delta_z()[3])
            * COL_CURV.radians().sin()
            * COL_ANGLES_Y[3].radians().sin();
    let col4_z = col3_z
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[3].radians().sin()
        + SWITCH_PLATE_XYZ.x / 2. * COL_ANGLES_Y[4].radians().sin()
        + col_delta_z()[4] * COL_ANGLES_Y[3].radians().cos();
    [col0_z, col1_z, col2_z, col3_z, col4_z]
}
pub fn row_delta_y() -> f64 {
    (SWITCH_PLATE_XYZ.y / 2. + KEYCAP_XYZ.z * (ROW_CURV.radians() / 2.).tan())
        * (1. + ROW_CURV.radians().cos())
}
pub fn row_delta_z() -> f64 {
    (SWITCH_PLATE_XYZ.y / 2. + KEYCAP_XYZ.z * (ROW_CURV.radians() / 2.).tan())
        * ROW_CURV.radians().sin()
}
