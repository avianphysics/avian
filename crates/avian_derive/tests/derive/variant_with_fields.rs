use avian_derive::PhysicsLayer;

trait PhysicsLayer {
    fn all_bits() -> u32;
    fn to_bits(&self) -> u32;
}

#[derive(PhysicsLayer, Default)]
enum WithFields {
    #[default]
    Good,
    Bad(u32),
}

fn main() {}
