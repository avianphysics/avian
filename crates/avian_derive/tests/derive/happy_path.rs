use avian_derive::PhysicsLayer;

trait PhysicsLayer {
    fn all_bits() -> u32;
    fn to_bits(&self) -> u32;
}

#[derive(PhysicsLayer, Default)]
enum GameLayer {
    #[default]
    Default,
    Player,
    Enemy,
    Ground,
}

fn main() {
    assert_eq!(GameLayer::Default.to_bits(), 1 << 0);
    assert_eq!(GameLayer::Player.to_bits(), 1 << 1);
    assert_eq!(GameLayer::Enemy.to_bits(), 1 << 2);
    assert_eq!(GameLayer::Ground.to_bits(), 1 << 3);
    assert_eq!(GameLayer::all_bits(), 0b1111);
}
