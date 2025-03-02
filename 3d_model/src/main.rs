mod geometry;
mod keyboard;
mod params;
mod switch;
use keyboard::Keyboard;

fn main() -> Result<(), opencascade::Error> {
    // Now that we have a symmetrical design, we only need to generate a single keyboard
    Keyboard::new().shape().write_stl("./keyboard.stl")
}
