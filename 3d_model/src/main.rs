mod geometry;
mod keyboard;
mod params;
mod switch;
use keyboard::Keyboard;

fn main() -> Result<(), opencascade::Error> {
    Keyboard::new().shape().write_stl("./keyboard.stl")
}
