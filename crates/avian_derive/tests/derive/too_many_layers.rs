use avian_derive::PhysicsLayer;

trait PhysicsLayer {
    fn all_bits() -> u32;
    fn to_bits(&self) -> u32;
}

#[derive(PhysicsLayer, Default)]
enum TooManyLayers {
    #[default]
    L00, L01, L02, L03, L04, L05, L06, L07,
    L08, L09, L10, L11, L12, L13, L14, L15,
    L16, L17, L18, L19, L20, L21, L22, L23,
    L24, L25, L26, L27, L28, L29, L30, L31, L32,
}

fn main() {}
