//! An example demonstrating how to make a custom collider and use it for collision detection.

#![allow(clippy::unnecessary_cast)]

use avian2d::{collision::contact_types::PackedFeatureId, math::*, prelude::*};
use bevy::prelude::*;
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            // Add physics plugins and specify a units-per-meter scaling factor, 1 meter = 10 pixels.
            // The unit allows the engine to tune its parameters for the scale of the world, improving stability.
            PhysicsPlugins::default().with_length_unit(10.0),
            // Add collider backend for our custom collider.
            // This handles things like initializing and updating required components
            // and managing collider hierarchies.
            ColliderBackendPlugin::<CircleCollider>::default(),
            // Enable collision detection for our custom collider.
            NarrowPhasePlugin::<CircleCollider>::default(),
        ))
        .insert_resource(ClearColor(Color::srgb(0.01, 0.01, 0.025)))
        .insert_resource(Gravity::ZERO)
        .add_systems(Startup, setup)
        .add_systems(
            FixedUpdate,
            (center_gravity, rotate).in_set(PhysicsStepSet::First),
        )
        .run();
}

/// A basic collider with a circle shape. Only supports uniform scaling.
#[derive(Component)]
struct CircleCollider {
    /// The radius of the circle collider. This may be scaled by the `Transform` scale.
    radius: Scalar,
    /// The radius of the circle collider without `Transform` scale applied.
    unscaled_radius: Scalar,
    /// The scaling factor, determined by `Transform` scale.
    scale: Scalar,
}

impl CircleCollider {
    fn new(radius: Scalar) -> Self {
        Self {
            radius,
            unscaled_radius: radius,
            scale: 1.0,
        }
    }
}

impl AnyCollider for CircleCollider {
    // If your collider needs queries or resources to function, you can specify
    // a custom `SystemParam` here. In this case, we don't need any.
    type Context = ();

    fn aabb_with_context(
        &self,
        position: Vector,
        _: impl Into<Rotation>,
        _: AabbContext<Self::Context>,
    ) -> ColliderAabb {
        ColliderAabb::new(position, Vector::splat(self.radius))
    }

    // This is the actual collision detection part.
    // It computes all contacts between two colliders at the given positions.
    fn contact_manifolds_with_context(
        &self,
        other: &Self,
        position1: Vector,
        rotation1: impl Into<Rotation>,
        position2: Vector,
        rotation2: impl Into<Rotation>,
        prediction_distance: Scalar,
        manifolds: &mut Vec<ContactManifold>,
        _: ContactManifoldContext<Self::Context>,
    ) {
        // Clear the previous manifolds.
        manifolds.clear();

        let rotation1: Rotation = rotation1.into();
        let rotation2: Rotation = rotation2.into();

        let inv_rotation1 = rotation1.inverse();
        let delta_pos = inv_rotation1 * (position2 - position1);
        let delta_rot = inv_rotation1 * rotation2;

        let distance_squared = delta_pos.length_squared();
        let sum_radius = self.radius + other.radius;

        if distance_squared < (sum_radius + prediction_distance).powi(2) {
            let local_normal1 = if distance_squared != 0.0 {
                delta_pos.normalize_or_zero()
            } else {
                Vector::X
            };
            let local_normal2 = delta_rot.inverse() * (-local_normal1);
            let local_point1 = local_normal1 * self.radius;
            let local_point2 = local_normal2 * other.radius;

            let point = ContactPoint::new(
                local_point1,
                local_point2,
                sum_radius - distance_squared.sqrt(),
            )
            .with_feature_ids(PackedFeatureId::face(0), PackedFeatureId::face(0));

            manifolds.push(ContactManifold::new([point], rotation1 * local_normal1, 0));
        }
    }
}

// Implement mass computation for the collider shape.
// This is needed for physics to behave correctly.
impl ComputeMassProperties2d for CircleCollider {
    fn mass(&self, density: f32) -> f32 {
        // In 2D, the Z length is assumed to be `1.0`, so volume == area.
        let volume = core::f32::consts::PI * self.radius.powi(2) as f32;
        density * volume
    }

    fn unit_angular_inertia(&self) -> f32 {
        // Angular inertia for a circle, assuming a mass of `1.0`.
        self.radius.powi(2) as f32 / 2.0
    }

    fn center_of_mass(&self) -> Vec2 {
        Vec2::ZERO
    }
}

// Note: This circle collider only supports uniform scaling.
impl ScalableCollider for CircleCollider {
    fn scale(&self) -> Vector {
        Vector::splat(self.scale)
    }

    fn set_scale(&mut self, scale: Vector, _detail: u32) {
        // For non-unifprm scaling, this would need to be converted to an ellipse collider or a convex hull.
        self.scale = scale.max_element();
        self.radius = self.unscaled_radius * scale.max_element();
    }
}

/// A marker component for the rotating body at the center.
#[derive(Component)]
struct CenterBody;

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn(Camera2d);

    let center_radius = 200.0;
    let particle_radius = 5.0;

    let red = materials.add(Color::srgb(0.9, 0.3, 0.3));
    let blue = materials.add(Color::srgb(0.1, 0.6, 1.0));
    let particle_mesh = meshes.add(Circle::new(particle_radius));

    // Spawn rotating body at the center.
    commands
        .spawn((
            Mesh2d(meshes.add(Circle::new(center_radius))),
            MeshMaterial2d(materials.add(Color::srgb(0.7, 0.7, 0.8)).clone()),
            RigidBody::Kinematic,
            CircleCollider::new(center_radius.adjust_precision()),
            CenterBody,
        ))
        .with_children(|c| {
            // Spawn obstacles along the perimeter of the rotating body, like the teeth of a cog.
            let count = 8;
            let angle_step = core::f32::consts::TAU / count as f32;
            for i in 0..count {
                let pos = Quat::from_rotation_z(i as f32 * angle_step) * Vec3::Y * center_radius;
                c.spawn((
                    Mesh2d(particle_mesh.clone()),
                    MeshMaterial2d(red.clone()),
                    Transform::from_translation(pos).with_scale(Vec3::ONE * 5.0),
                    CircleCollider::new(particle_radius.adjust_precision()),
                ));
            }
        });

    let x_count = 10;
    let y_count = 30;

    // Spawm grid of particles. These will be pulled towards the rotating body.
    for x in -x_count / 2..x_count / 2 {
        for y in -y_count / 2..y_count / 2 {
            commands.spawn((
                Mesh2d(particle_mesh.clone()),
                MeshMaterial2d(blue.clone()),
                Transform::from_xyz(
                    x as f32 * 3.0 * particle_radius - 350.0,
                    y as f32 * 3.0 * particle_radius,
                    0.0,
                ),
                RigidBody::Dynamic,
                CircleCollider::new(particle_radius.adjust_precision()),
                LinearDamping(0.4),
            ));
        }
    }
}

/// Pulls all particles towards the center.
fn center_gravity(
    mut particles: Query<(&Transform, &mut LinearVelocity), Without<CenterBody>>,
    time: Res<Time>,
) {
    let delta_seconds = time.delta_secs_f64().adjust_precision();
    for (transform, mut lin_vel) in &mut particles {
        let pos_delta = transform.translation.truncate().adjust_precision();
        let dir = -pos_delta.normalize_or_zero();
        lin_vel.0 += 800.0 * delta_seconds * dir;
    }
}

/// Rotates the center body periodically clockwise and counterclockwise.
fn rotate(mut query: Query<&mut AngularVelocity, With<CenterBody>>, time: Res<Time>) {
    let sin = 3.0 * time.elapsed_secs_f64().adjust_precision().sin();
    for mut ang_vel in &mut query {
        ang_vel.0 = sin;
    }
}
