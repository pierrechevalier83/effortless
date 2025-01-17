mod geometry;
mod keyboard;
mod params;
mod switch;
use crate::switch::Hand;
use keyboard::Keyboard;

fn main() -> Result<(), opencascade::Error> {
    Keyboard::new(Hand::Left)
        .shape()
        .write_stl("./keyboard_left.stl")?;
    Keyboard::new(Hand::Right)
        .shape()
        .write_stl("./keyboard_right.stl")
}
