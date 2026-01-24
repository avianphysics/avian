# ![Avian Physics](https://raw.githubusercontent.com/Jondolf/avian/avian/assets/branding/logo.svg)

[![MIT/Apache 2.0](https://img.shields.io/badge/license-MIT%2FApache-blue.svg)](https://github.com/avianphysics/avian#license)
[![ci](https://github.com/avianphysics/avian/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/avianphysics/avian/actions/workflows/ci.yml)
[![2D crates.io](https://img.shields.io/crates/v/avian2d?label=2D%20crates.io)](https://crates.io/crates/avian2d)
[![2D docs.rs](https://img.shields.io/docsrs/avian2d?label=2D%20docs.rs)](https://docs.rs/avian2d)
[![3D crates.io](https://img.shields.io/crates/v/avian3d?label=3D%20crates.io)](https://crates.io/crates/avian3d)
[![3D docs.rs](https://img.shields.io/docsrs/avian3d?label=3D%20docs.rs)](https://docs.rs/avian3d)

**Avian** is an ECS-driven 2D and 3D physics engine for the [Bevy game engine](https://bevyengine.org/).

---

## Design

Below are some of the core design principles used in Avian.

- **Made with Bevy, for Bevy.** No wrappers around existing engines.
- **Provide an ergonomic and familiar API.** Ergonomics is key for a good experience.
- **Utilize the ECS as much as possible.** The engine should feel like a part of Bevy, and it shouldn't
    need to maintain a separate physics world.
- **Use a highly modular plugin architecture.** Users should be able to replace parts of the engine
    with their own implementations.
- **Have good documentation.** A physics engine is pointless if you don't know how to use it.

## Features

Below are some of the current features of Avian.

- Dynamic, kinematic and static rigid bodies
  - Linear and angular velocity
  - External forces, torque and impulses
  - Gravity and gravity scale
  - Linear and angular damping
  - Locking translational and rotational axes
  - Rigid body dominance
  - Continuous Collision Detection (CCD)
  - Automatic deactivation with sleeping
- Collision detection powered by [Parry](https://parry.rs)
  - Colliders with configurable collision layers, density, material properties and more
  - Collider generation for meshes and entire scenes
  - Collision events
  - Access to colliding entities
  - Filtering and modifying collisions with collision hooks
  - Manual contact queries and intersection tests
- Constraints and joints
  - Several built-in joint types: fixed, distance, prismatic, revolute, spherical
  - Support for custom joints and other constraints using XPBD
- Spatial queries
  - Raycasting, shapecasting, point projection and intersection tests
  - Ergonomic component-based API for raycasts and shapecasts
  - Flexible `SpatialQuery` system parameter
  - Spatial query filters
- `Transform` interpolation and extrapolation for fixed timesteps
- Debug rendering for colliders, AABBs, contacts, joints, spatial queries, and more
- Configurable scheduling and high customizability
- Highly modular plugin architecture, freely extend and replace parts of the engine
- Support for custom collision backends
- `f32`/`f64` precision (`f32` by default)

You can find a more complete list along with documentation in the
[Table of Contents](https://docs.rs/avian3d/latest/avian3d/#table-of-contents)
on docs.rs.

## Documentation

- [2D documentation](https://docs.rs/avian2d)
- [3D documentation](https://docs.rs/avian3d)

## Usage Example

First, add `avian2d` or `avian3d` to your dependencies in `Cargo.toml`:

```toml
# For 2D applications:
[dependencies]
avian2d = "0.5"

# For 3D applications:
[dependencies]
avian3d = "0.5"

# If you want to use the most up-to-date version, you can follow the main branch:
[dependencies]
avian3d = { git = "https://github.com/avianphysics/avian", branch = "main" }
```

Below is a very simple example where a cube with initial angular velocity falls onto a circular platform.
This is a modified version of Bevy's [`3d_scene`](https://bevyengine.org/examples/3d-rendering/3d-scene/) example.

```rust
use avian3d::prelude::*;
use bevy::prelude::*;

fn main() {
    App::new()
        // Enable physics
        .add_plugins((DefaultPlugins, PhysicsPlugins::default()))
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    // Static physics object with a collision shape
    commands.spawn((
        RigidBody::Static,
        Collider::cylinder(4.0, 0.1),
        Mesh3d(meshes.add(Cylinder::new(4.0, 0.1))),
        MeshMaterial3d(materials.add(Color::WHITE)),
    ));

    // Dynamic physics object with a collision shape and initial angular velocity
    commands.spawn((
        RigidBody::Dynamic,
        Collider::cuboid(1.0, 1.0, 1.0),
        AngularVelocity(Vec3::new(2.5, 3.5, 1.5)),
        Mesh3d(meshes.add(Cuboid::from_length(1.0))),
        MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
        Transform::from_xyz(0.0, 4.0, 0.0),
    ));

    // Light
    commands.spawn((
        PointLight {
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-2.5, 4.5, 9.0).looking_at(Vec3::ZERO, Dir3::Y),
    ));
}
```

![A spinning cube falling onto a circular platform](https://github.com/user-attachments/assets/14d25e7e-9d46-467c-9fe6-dc408cd23398)

## More Examples

You can find lots of 2D and 3D examples in [/crates/avian2d/examples](/crates/avian2d/examples) and [/crates/avian3d/examples](/crates/avian3d/examples) respectively.

The examples support both `f32` and `f64` precisions, so the code contains some feature-dependent types like `Scalar` and `Vector`.
In actual usage these are not needed, so you can just use `f32` or `f64` types depending on the features you have chosen.

By default the examples use `f32`. To run the `f64` versions, you need to disable default features and manually choose the dimension
and precision:

```shell
# Manually specify dimension and precision. `parry-f64` enables collision detection using Parry.
cargo run --example cubes --no-default-features --features "3d f64 parry-f64"
```

## Version Table

| Bevy    | Avian     |
| ------- | --------- |
| 0.18    | 0.5, main |
| 0.17    | 0.4       |
| 0.16    | 0.3       |
| 0.15    | 0.2       |
| 0.14    | 0.1       |

Avian provides [migration guides](./migration-guides) for each version.

<details>
  <summary>Bevy XPBD versions (the predecessor of Avian)</summary>

  | Bevy | Bevy XPBD |
  | ---- | --------- |
  | 0.14 | 0.5       |
  | 0.13 | 0.4       |
  | 0.12 | 0.3       |
  | 0.11 | 0.2       |
  | 0.10 | 0.1       |

</details>

## Contributing

If you encounter any problems, feel free to open issues or create pull requests.
For larger changes and additions, it's better to open an issue or ask me for input
before making a pull request.

You can also ask for help or ask questions on the [Bevy Discord](https://discord.com/invite/gMUk5Ph)
server's Avian Physics topic in `#ecosystem-crates`. My username on the Discord is `Jondolf` (`@jondolfdev`).

## Acknowledgements

Huge thanks to the entire Bevy community for the incredible support!
All of your contributions, insight and requests are a massive help
in driving the state of physics in Bevy forward, and it's what
keeps me motivated to build the best engine I can.

I would also like to give a special thanks to [Johan Helsing][johan-helsing]
for inspiring this project and helping me significantly in the early stages.
His original [tutorial series][johan-xpbd-tutorial] is the reason `avian`
exists in the first place, and without his support and contributions,
the project wouldn't be anywhere near where it is today.

[johan-helsing]: https://github.com/johanhelsing
[johan-xpbd-tutorial]: https://johanhelsing.studio/posts/bevy-xpbd

## License

Avian is free and open source. All code in this repository is dual-licensed under either:

- MIT License ([LICENSE-MIT](/LICENSE-MIT) or <http://opensource.org/licenses/MIT>)
- Apache License, Version 2.0 ([LICENSE-APACHE](/LICENSE-APACHE) or <http://www.apache.org/licenses/LICENSE-2.0>)

at your option.
