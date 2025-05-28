//! A 2D platformer example with one-way platforms to demonstrate
//! contact modification with `CollisionHooks`.
//!
//! Move with arrow keys, jump with Space and descend through
//! platforms by pressing Space while holding the down arrow.

#![allow(clippy::type_complexity)]

use avian2d::{math::*, prelude::*};
use bevy::{
    ecs::{
        entity::hash_set::EntityHashSet,
        system::{lifetimeless::Read, SystemParam},
    },
    prelude::*,
};
use examples_common_2d::ExampleCommonPlugin;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default()
                // Specify a units-per-meter scaling factor, 1 meter = 20 pixels.
                // The unit allows the engine to tune its parameters for the scale of the world, improving stability.
                .with_length_unit(20.0)
                // Add our custom collision hooks.
                .with_collision_hooks::<PlatformerCollisionHooks>(),
        ))
        .insert_resource(ClearColor(Color::srgb(0.05, 0.05, 0.1)))
        .insert_resource(Gravity(Vector::NEG_Y * 1000.0))
        .add_systems(Startup, setup)
        .add_systems(Update, (movement, pass_through_one_way_platform))
        .run();
}

#[derive(Component)]
struct Actor;

#[derive(Component)]
struct MovementSpeed(Scalar);

#[derive(Component)]
struct JumpImpulse(Scalar);

// Enable contact modification for one-way platforms with the `ActiveCollisionHooks` component.
// Here we use required components, but you could also add it manually.
#[derive(Clone, Eq, PartialEq, Debug, Default, Component)]
#[require(ActiveCollisionHooks::MODIFY_CONTACTS)]
pub struct OneWayPlatform(EntityHashSet);

/// A component to control how an actor interacts with a one-way platform.
#[derive(Copy, Clone, Eq, PartialEq, Debug, Default, Component, Reflect)]
pub enum PassThroughOneWayPlatform {
    #[default]
    /// Passes through a `OneWayPlatform` if the contact normal is in line with the platform's local-space up vector.
    ByNormal,
    /// Always passes through a `OneWayPlatform`, temporarily set this to allow an actor to jump down through a platform.
    Always,
    /// Never passes through a `OneWayPlatform`.
    Never,
}

fn setup(
    mut commands: Commands,
    mut materials: ResMut<Assets<ColorMaterial>>,
    mut meshes: ResMut<Assets<Mesh>>,
) {
    commands.spawn(Camera2d);

    // For borders
    let square_sprite = Sprite {
        color: Color::srgb(0.7, 0.7, 0.8),
        custom_size: Some(Vec2::splat(50.0)),
        ..default()
    };

    // Ceiling
    commands.spawn((
        square_sprite.clone(),
        Transform::from_xyz(0.0, 50.0 * 6.0, 0.0).with_scale(Vec3::new(20.0, 1.0, 1.0)),
        RigidBody::Static,
        Collider::rectangle(50.0, 50.0),
    ));
    // Floor
    commands.spawn((
        square_sprite.clone(),
        Transform::from_xyz(0.0, -50.0 * 6.0, 0.0).with_scale(Vec3::new(20.0, 1.0, 1.0)),
        RigidBody::Static,
        Collider::rectangle(50.0, 50.0),
    ));
    // Left wall
    commands.spawn((
        square_sprite.clone(),
        Transform::from_xyz(-50.0 * 9.5, 0.0, 0.0).with_scale(Vec3::new(1.0, 11.0, 1.0)),
        RigidBody::Static,
        Collider::rectangle(50.0, 50.0),
    ));
    // Right wall
    commands.spawn((
        square_sprite,
        Transform::from_xyz(50.0 * 9.5, 0.0, 0.0).with_scale(Vec3::new(1.0, 11.0, 1.0)),
        RigidBody::Static,
        Collider::rectangle(50.0, 50.0),
    ));

    // For one-way platforms
    let one_way_sprite = Sprite {
        color: Color::srgba(0.7, 0.7, 0.8, 0.25),
        custom_size: Some(Vec2::splat(50.0)),
        ..default()
    };

    // Spawn some one way platforms
    for y in -2..=2 {
        commands.spawn((
            one_way_sprite.clone(),
            Transform::from_xyz(0.0, y as f32 * 16.0 * 6.0, 0.0)
                .with_scale(Vec3::new(10.0, 0.5, 1.0)),
            RigidBody::Static,
            Collider::rectangle(50.0, 50.0),
            OneWayPlatform::default(),
        ));
    }

    // Spawn an actor for the user to control
    let actor_size = Vector::new(20.0, 20.0);
    let actor_mesh = meshes.add(Rectangle::from_size(actor_size.f32()));
    let actor_material = materials.add(Color::srgb(0.2, 0.7, 0.9));

    commands.spawn((
        Mesh2d(actor_mesh),
        MeshMaterial2d(actor_material),
        RigidBody::Dynamic,
        LockedAxes::ROTATION_LOCKED,
        Restitution::ZERO.with_combine_rule(CoefficientCombine::Min),
        Collider::rectangle(actor_size.x, actor_size.y),
        Actor,
        PassThroughOneWayPlatform::ByNormal,
        MovementSpeed(250.0),
        JumpImpulse(450.0),
    ));
}

fn movement(
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut actors: Query<(&mut LinearVelocity, &MovementSpeed, &JumpImpulse), With<Actor>>,
) {
    for (mut linear_velocity, movement_speed, jump_impulse) in &mut actors {
        let left = keyboard_input.any_pressed([KeyCode::KeyA, KeyCode::ArrowLeft]);
        let right = keyboard_input.any_pressed([KeyCode::KeyD, KeyCode::ArrowRight]);
        let horizontal = right as i8 - left as i8;

        // Move in input direction
        linear_velocity.x = horizontal as Scalar * movement_speed.0;

        // Assume "mostly stopped" to mean "grounded".
        // You should use raycasting, shapecasting or sensor colliders
        // for more robust ground detection.
        if linear_velocity.y.abs() < 0.1
            && !keyboard_input.pressed(KeyCode::ArrowDown)
            && keyboard_input.just_pressed(KeyCode::Space)
        {
            linear_velocity.y = jump_impulse.0;
        }
    }
}

fn pass_through_one_way_platform(
    mut commands: Commands,
    keyboard_input: Res<ButtonInput<KeyCode>>,
    mut actors: Query<(Entity, &mut PassThroughOneWayPlatform), With<Actor>>,
) {
    for (entity, mut pass_through_one_way_platform) in &mut actors {
        if keyboard_input.pressed(KeyCode::ArrowDown) && keyboard_input.pressed(KeyCode::Space) {
            *pass_through_one_way_platform = PassThroughOneWayPlatform::Always;

            // Wake up the body when it's allowed to drop down.
            // Otherwise it won't fall because gravity isn't simulated.
            commands.queue(WakeUpBody(entity));
        } else {
            *pass_through_one_way_platform = PassThroughOneWayPlatform::ByNormal;
        }
    }
}

// Define a custom `SystemParam` for our collision hooks.
// It can have read-only access to queries, resources, and other system parameters.
#[derive(SystemParam)]
struct PlatformerCollisionHooks<'w, 's> {
    one_way_platforms_query: Query<'w, 's, (Read<OneWayPlatform>, Read<GlobalTransform>)>,
    // NOTE: This precludes a `OneWayPlatform` passing through a `OneWayPlatform`.
    other_colliders_query: Query<
        'w,
        's,
        Option<Read<PassThroughOneWayPlatform>>,
        (With<Collider>, Without<OneWayPlatform>),
    >,
}

// Implement the `CollisionHooks` trait for our custom system parameter.
impl CollisionHooks for PlatformerCollisionHooks<'_, '_> {
    // Below is a description of the logic used for one-way platforms.

    /// Allows entities to pass through [`OneWayPlatform`] entities.
    ///
    /// Passing through is achieved by removing the collisions between the [`OneWayPlatform`]
    /// and the other entity if the entity should pass through.
    /// If a [`PassThroughOneWayPlatform`] is present on the non-platform entity,
    /// the value of the component dictates the pass-through behaviour.
    ///
    /// Entities known to be passing through each [`OneWayPlatform`] are stored in the
    /// [`OneWayPlatform`]. If an entity is known to be passing through a [`OneWayPlatform`],
    /// it is allowed to continue to do so, even if [`PassThroughOneWayPlatform`] has been
    /// set to disallow passing through.
    ///
    /// #### When an entity is known to already be passing through the [`OneWayPlatform`]
    ///
    /// When an entity begins passing through a [`OneWayPlatform`], it is added to the
    /// [`OneWayPlatform`]'s set of active penetrations, and will be allowed to continue
    /// to pass through until it is no longer penetrating the platform.
    ///
    /// #### When an entity is *not* known to be passing through the [`OneWayPlatform`]
    ///
    /// Depending on the setting of [`PassThroughOneWayPlatform`], the entity may be allowed to
    /// pass through.
    ///
    /// If no [`PassThroughOneWayPlatform`] is present, [`PassThroughOneWayPlatform::ByNormal`] is used.
    ///
    /// [`PassThroughOneWayPlatform`] may be in one of three states:
    /// 1. [`PassThroughOneWayPlatform::ByNormal`]
    ///     - This is the default state
    ///     - The entity may be allowed to pass through the [`OneWayPlatform`] depending on the contact normal
    ///         - If all contact normals are in line with the [`OneWayPlatform`]'s local-space up vector,
    ///           the entity is allowed to pass through
    /// 2. [`PassThroughOneWayPlatform::Always`]
    ///     - The entity will always pass through the [`OneWayPlatform`], regardless of contact normal
    ///     - This is useful for allowing an entity to jump down through a platform
    /// 3. [`PassThroughOneWayPlatform::Never`]
    ///     - The entity will never pass through the [`OneWayPlatform`], meaning the platform will act
    ///       as normal hard collision for this entity
    ///
    /// Even if an entity is changed to [`PassThroughOneWayPlatform::Never`], it will be allowed to pass
    /// through a [`OneWayPlatform`] if it is already penetrating the platform. Once it exits the platform,
    /// it will no longer be allowed to pass through.
    fn modify_contacts(&self, contacts: &mut ContactPair, commands: &mut Commands) -> bool {
        // This is the contact modification hook, called after collision detection,
        // but before constraints are created for the solver. Mutable access to the ECS
        // is not allowed, but we can queue commands to perform deferred changes.

        // Differentiate between which normal of the manifold we should use
        enum RelevantNormal {
            Normal1,
            Normal2,
        }

        // First, figure out which entity is the one-way platform, and which is the other.
        // Choose the appropriate normal for pass-through depending on which is which.
        let (platform_entity, one_way_platform, platform_transform, other_entity, relevant_normal) =
            if let Ok((one_way_platform, platform_transform)) =
                self.one_way_platforms_query.get(contacts.collider1)
            {
                (
                    contacts.collider1,
                    one_way_platform,
                    platform_transform,
                    contacts.collider2,
                    RelevantNormal::Normal1,
                )
            } else if let Ok((one_way_platform, platform_transform)) =
                self.one_way_platforms_query.get(contacts.collider2)
            {
                (
                    contacts.collider2,
                    one_way_platform,
                    platform_transform,
                    contacts.collider1,
                    RelevantNormal::Normal2,
                )
            } else {
                // Neither is a one-way-platform, so accept the collision:
                // we're done here.
                return true;
            };

        if one_way_platform.0.contains(&other_entity) {
            let any_penetrating = contacts.manifolds.iter().any(|manifold| {
                manifold
                    .points
                    .iter()
                    .any(|contact| contact.penetration > 0.0)
            });

            if any_penetrating {
                // If we were already allowing a collision for a particular entity,
                // and if it is penetrating us still, continue to allow it to do so.
                return false;
            } else {
                // If it's no longer penetrating us, forget it.
                commands.queue(OneWayPlatformCommand::Remove {
                    platform_entity,
                    entity: other_entity,
                });
            }
        }

        match self.other_colliders_query.get(other_entity) {
            // Pass-through is set to never, so accept the collision.
            Ok(Some(PassThroughOneWayPlatform::Never)) => true,
            // Pass-through is set to always, so always ignore this collision
            // and register it as an entity that's currently penetrating.
            Ok(Some(PassThroughOneWayPlatform::Always)) => {
                commands.queue(OneWayPlatformCommand::Add {
                    platform_entity,
                    entity: other_entity,
                });
                false
            }
            // Default behaviour is "by normal".
            Err(_) | Ok(None) | Ok(Some(PassThroughOneWayPlatform::ByNormal)) => {
                // If all contact normals are in line with the local up vector of this platform,
                // then this collision should occur: the entity is on top of the platform.
                let platform_up = platform_transform.up().truncate().adjust_precision();
                if contacts.manifolds.iter().all(|manifold| {
                    let normal = match relevant_normal {
                        RelevantNormal::Normal1 => manifold.normal,
                        RelevantNormal::Normal2 => -manifold.normal,
                    };

                    normal.length() > Scalar::EPSILON && normal.dot(platform_up) >= 0.5
                }) {
                    true
                } else {
                    // Otherwise, ignore the collision and register
                    // the other entity as one that's currently penetrating.
                    commands.queue(OneWayPlatformCommand::Add {
                        platform_entity,
                        entity: other_entity,
                    });
                    false
                }
            }
        }
    }
}

/// A command to add/remove entities to/from the set of entities
/// that are currently in contact with a one-way platform.
enum OneWayPlatformCommand {
    Add {
        platform_entity: Entity,
        entity: Entity,
    },
    Remove {
        platform_entity: Entity,
        entity: Entity,
    },
}

impl Command for OneWayPlatformCommand {
    fn apply(self, world: &mut World) {
        match self {
            OneWayPlatformCommand::Add {
                platform_entity,
                entity,
            } => {
                if let Some(mut platform) = world.get_mut::<OneWayPlatform>(platform_entity) {
                    platform.0.insert(entity);
                }
            }

            OneWayPlatformCommand::Remove {
                platform_entity,
                entity,
            } => {
                if let Some(mut platform) = world.get_mut::<OneWayPlatform>(platform_entity) {
                    platform.0.remove(&entity);
                }
            }
        }
    }
}
