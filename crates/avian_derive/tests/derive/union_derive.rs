use avian_derive::PhysicsLayer;

trait PhysicsLayer {
    fn all_bits() -> u32;
    fn to_bits(&self) -> u32;
}

#[derive(PhysicsLayer)]
union AlsoNotAnEnum {
    x: u32,
    y: f32,
}

fn main() {}
