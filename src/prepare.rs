//! Runs systems that prepare and initialize components used by physics.
//!
//! See [`PreparePlugin`].

#![allow(clippy::type_complexity)]

use crate::{prelude::*, sync::SyncConfig};
use bevy::{
    ecs::{intern::Interned, query::QueryFilter, schedule::ScheduleLabel},
    prelude::*,
};

/// Runs systems at the start of each physics frame. Initializes [rigid bodies](RigidBody)
/// and updates components.
///
/// - Adds missing rigid body components for entities with a [`RigidBody`] component
/// - Clamps restitution coefficients between 0 and 1
///
/// The [`Transform`] component will be initialized based on [`Position`] or [`Rotation`]
/// and vice versa. You can configure this synchronization using the [`PrepareConfig`] resource.
///
/// The plugin takes a collider type. This should be [`Collider`] for
/// the vast majority of applications, but for custom collisión backends
/// you may use any collider that implements the [`AnyCollider`] trait.
///
/// The systems run in [`PhysicsSet::Prepare`].
pub struct PreparePlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl PreparePlugin {
    /// Creates a [`PreparePlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `FixedPostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for PreparePlugin {
    fn default() -> Self {
        Self::new(FixedPostUpdate)
    }
}

/// Systems sets for initializing and syncing missing components.
/// You can use these to schedule your own initialization systems
/// without having to worry about implementation details.
///
/// 1. `First`: Runs at the start of the preparation step.
/// 2. `PropagateTransforms`: Responsible for propagating transforms.
/// 3. `InitTransforms`: Responsible for initializing [`Transform`] based on [`Position`] and [`Rotation`]
///    or vice versa.
/// 4. `Finalize`: Responsible for performing final updates after everything is initialized and updated.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PrepareSet {
    /// Runs at the start of the preparation step.
    First,
    /// Responsible for propagating transforms.
    PropagateTransforms,
    /// Responsible for initializing [`Transform`] based on [`Position`] and [`Rotation`]
    /// or vice versa. Parts of this system can be disabled with [`PrepareConfig`].
    /// Schedule your system with this to implement custom behavior for initializing transforms.
    InitTransforms,
    /// Responsible for performing final updates after everything is initialized.
    /// Updates mass properties and clamps collider density and restitution.
    Finalize,
}

impl Plugin for PreparePlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<SyncConfig>()
            .init_resource::<PrepareConfig>();

        app.register_type::<(SyncConfig, PrepareConfig)>();

        if app
            .world()
            .resource::<PrepareConfig>()
            .position_to_transform
        {
            app.register_required_components::<Position, Transform>();
            app.register_required_components::<Rotation, Transform>();
        }

        app.configure_sets(
            self.schedule,
            (
                PrepareSet::First,
                PrepareSet::PropagateTransforms,
                PrepareSet::InitTransforms,
                PrepareSet::Finalize,
            )
                .chain()
                .in_set(PhysicsSet::Prepare),
        );
    }
}

/// Configures what is initialized by the [`PreparePlugin`] and how.
#[derive(Resource, Reflect, Clone, Debug, PartialEq, Eq)]
#[reflect(Resource)]
pub struct PrepareConfig {
    /// Initializes [`Transform`] based on [`Position`] and [`Rotation`].
    /// Defaults to true.
    pub position_to_transform: bool,
    /// Initializes [`Position`] and [`Rotation`] based on [`Transform`].
    /// Defaults to true.
    pub transform_to_position: bool,
}

impl Default for PrepareConfig {
    fn default() -> Self {
        PrepareConfig {
            position_to_transform: true,
            transform_to_position: true,
        }
    }
}

/// A run condition that returns `true` if any entity matches the given query filter.
pub(crate) fn match_any<F: QueryFilter>(query: Query<(), F>) -> bool {
    !query.is_empty()
}

#[cfg(test)]
mod tests {
    #[cfg(feature = "3d")]
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn test_init_transforms_basics() {
        let mut app = App::new();

        // Automatically add `Transform` for every rigid body for this test.
        app.register_required_components::<RigidBody, Transform>();

        // Test all possible config permutations
        for (position_to_transform, transform_to_position) in
            [(true, true), (true, false), (false, true), (false, false)]
        {
            let config = PrepareConfig {
                position_to_transform,
                transform_to_position,
            };
            app.insert_resource(dbg!(config.clone()));

            // Spawn entities with `Position` and `Rotation`
            let (pos_0, rot_0) = {
                #[cfg(feature = "2d")]
                {
                    (Position::from_xy(1., 2.), Rotation::radians(0.5))
                }
                #[cfg(feature = "3d")]
                {
                    (
                        Position::from_xyz(1., 2., 3.),
                        Rotation(Quaternion::from_axis_angle(Vector::Y, 0.5)),
                    )
                }
            };
            let e_0_with_pos_and_rot = app
                .world_mut()
                .spawn((RigidBody::Dynamic, pos_0, rot_0))
                .id();

            let (pos_1, rot_1) = {
                #[cfg(feature = "2d")]
                {
                    (Position::from_xy(-1., 3.), Rotation::radians(0.1))
                }
                #[cfg(feature = "3d")]
                {
                    (
                        Position::from_xyz(-1., 3., -3.),
                        Rotation(Quaternion::from_axis_angle(Vector::X, 0.1)),
                    )
                }
            };
            let e_1_with_pos_and_rot = app
                .world_mut()
                .spawn((RigidBody::Dynamic, pos_1, rot_1))
                .id();

            // Spawn an entity with only `Position`
            let pos_2 = {
                #[cfg(feature = "2d")]
                {
                    Position::from_xy(10., 1.)
                }
                #[cfg(feature = "3d")]
                {
                    Position::from_xyz(10., 1., 5.)
                }
            };
            let e_2_with_pos = app.world_mut().spawn((RigidBody::Dynamic, pos_2)).id();

            // Spawn an entity with only `Rotation`
            let rot_3 = {
                #[cfg(feature = "2d")]
                {
                    Rotation::radians(0.4)
                }
                #[cfg(feature = "3d")]
                {
                    Rotation(Quaternion::from_axis_angle(Vector::Z, 0.4))
                }
            };
            let e_3_with_rot = app.world_mut().spawn((RigidBody::Dynamic, rot_3)).id();

            // Spawn entities with `Transform`
            let trans_4 = {
                Transform {
                    translation: Vec3::new(-1.1, 6., -7.),
                    rotation: Quat::from_axis_angle(Vec3::Y, 0.1),
                    scale: Vec3::ONE,
                }
            };
            let e_4_with_trans = app.world_mut().spawn((RigidBody::Dynamic, trans_4)).id();

            let trans_5 = {
                Transform {
                    translation: Vec3::new(8., -1., 0.),
                    rotation: Quat::from_axis_angle(Vec3::Y, -0.1),
                    scale: Vec3::ONE,
                }
            };
            let e_5_with_trans = app.world_mut().spawn((RigidBody::Dynamic, trans_5)).id();

            // Spawn entity without any transforms
            let e_6_without_trans = app.world_mut().spawn(RigidBody::Dynamic).id();

            // Spawn entity without a ridid body
            let e_7_without_rb = app.world_mut().spawn(()).id();

            // Run the system
            app.update();

            // Check the results are as expected
            if config.position_to_transform {
                assert!(app.world().get::<Transform>(e_0_with_pos_and_rot).is_some());
                let transform = app.world().get::<Transform>(e_0_with_pos_and_rot).unwrap();
                let expected: Vec3 = {
                    #[cfg(feature = "2d")]
                    {
                        pos_0.f32().extend(0.)
                    }
                    #[cfg(feature = "3d")]
                    {
                        pos_0.f32()
                    }
                };
                assert_eq!(transform.translation, expected);
                let expected = Quaternion::from(rot_0).f32();
                assert_eq!(transform.rotation, expected);

                assert!(app.world().get::<Transform>(e_1_with_pos_and_rot).is_some());
                let transform = app.world().get::<Transform>(e_1_with_pos_and_rot).unwrap();
                let expected: Vec3 = {
                    #[cfg(feature = "2d")]
                    {
                        pos_1.f32().extend(0.)
                    }
                    #[cfg(feature = "3d")]
                    {
                        pos_1.f32()
                    }
                };
                assert_eq!(transform.translation, expected);
                let expected = Quaternion::from(rot_1).f32();
                assert_eq!(transform.rotation, expected);

                assert!(app.world().get::<Transform>(e_2_with_pos).is_some());
                let transform = app.world().get::<Transform>(e_2_with_pos).unwrap();
                let expected: Vec3 = {
                    #[cfg(feature = "2d")]
                    {
                        pos_2.f32().extend(0.)
                    }
                    #[cfg(feature = "3d")]
                    {
                        pos_2.f32()
                    }
                };
                assert_eq!(transform.translation, expected);
                let expected = Quat::default();
                assert_eq!(transform.rotation, expected);

                assert!(app.world().get::<Transform>(e_3_with_rot).is_some());
                let transform = app.world().get::<Transform>(e_3_with_rot).unwrap();
                let expected: Vec3 = Vec3::default();
                assert_eq!(transform.translation, expected);
                let expected = Quaternion::from(rot_3).f32();
                assert_eq!(transform.rotation, expected);

                assert!(app.world().get::<Transform>(e_4_with_trans).is_some());
                let transform = app.world().get::<Transform>(e_4_with_trans).unwrap();
                assert_eq!(transform, &trans_4);

                assert!(app.world().get::<Transform>(e_5_with_trans).is_some());
                let transform = app.world().get::<Transform>(e_5_with_trans).unwrap();
                assert_eq!(transform, &trans_5);

                assert!(app.world().get::<Transform>(e_6_without_trans).is_some());
                let transform = app.world().get::<Transform>(e_6_without_trans).unwrap();
                assert_eq!(transform, &Transform::default());

                assert!(app.world().get::<Transform>(e_7_without_rb).is_none());
            }

            if config.transform_to_position {
                assert!(app.world().get::<Position>(e_0_with_pos_and_rot).is_some());
                let pos = app.world().get::<Position>(e_0_with_pos_and_rot).unwrap();
                assert_eq!(pos, &pos_0);
                assert!(app.world().get::<Rotation>(e_0_with_pos_and_rot).is_some());
                let rot = app.world().get::<Rotation>(e_0_with_pos_and_rot).unwrap();
                assert_eq!(rot, &rot_0);

                assert!(app.world().get::<Position>(e_1_with_pos_and_rot).is_some());
                let pos = app.world().get::<Position>(e_1_with_pos_and_rot).unwrap();
                assert_eq!(pos, &pos_1);
                assert!(app.world().get::<Rotation>(e_1_with_pos_and_rot).is_some());
                let rot = app.world().get::<Rotation>(e_1_with_pos_and_rot).unwrap();
                assert_eq!(rot, &rot_1);

                assert!(app.world().get::<Position>(e_2_with_pos).is_some());
                let pos = app.world().get::<Position>(e_2_with_pos).unwrap();
                assert_eq!(pos, &pos_2);
                assert!(app.world().get::<Rotation>(e_2_with_pos).is_some());
                let rot = app.world().get::<Rotation>(e_2_with_pos).unwrap();
                assert_eq!(rot, &Rotation::default());

                assert!(app.world().get::<Position>(e_3_with_rot).is_some());
                let pos = app.world().get::<Position>(e_3_with_rot).unwrap();
                assert_eq!(pos, &Position::default());
                assert!(app.world().get::<Rotation>(e_3_with_rot).is_some());
                let rot = app.world().get::<Rotation>(e_3_with_rot).unwrap();
                assert_eq!(rot, &rot_3);

                assert!(app.world().get::<Position>(e_4_with_trans).is_some());
                let pos = app.world().get::<Position>(e_4_with_trans).unwrap();
                let expected: Position = Position::new({
                    #[cfg(feature = "2d")]
                    {
                        trans_4.translation.truncate().adjust_precision()
                    }
                    #[cfg(feature = "3d")]
                    {
                        trans_4.translation.adjust_precision()
                    }
                });
                assert_eq!(pos, &expected);
                assert!(app.world().get::<Rotation>(e_4_with_trans).is_some());
                let rot = app.world().get::<Rotation>(e_4_with_trans).unwrap();
                #[cfg(feature = "2d")]
                assert_eq!(*rot, Rotation::from(trans_4.rotation));
                #[cfg(feature = "3d")]
                assert_relative_eq!(rot.f32(), trans_4.rotation);

                assert!(app.world().get::<Position>(e_5_with_trans).is_some());
                let pos = app.world().get::<Position>(e_5_with_trans).unwrap();
                let expected: Position = Position::new({
                    #[cfg(feature = "2d")]
                    {
                        trans_5.translation.truncate().adjust_precision()
                    }
                    #[cfg(feature = "3d")]
                    {
                        trans_5.translation.adjust_precision()
                    }
                });
                assert_eq!(pos, &expected);
                assert!(app.world().get::<Rotation>(e_5_with_trans).is_some());
                let rot = app.world().get::<Rotation>(e_5_with_trans).unwrap();
                #[cfg(feature = "2d")]
                assert_eq!(rot, &Rotation::from(trans_5.rotation));
                #[cfg(feature = "3d")]
                assert_relative_eq!(rot.f32(), trans_5.rotation);

                assert!(app.world().get::<Position>(e_6_without_trans).is_some());
                let pos = app.world().get::<Position>(e_6_without_trans).unwrap();
                assert_eq!(pos, &Position::default());
                assert!(app.world().get::<Rotation>(e_6_without_trans).is_some());
                let rot = app.world().get::<Rotation>(e_6_without_trans).unwrap();
                assert_eq!(rot, &Rotation::default());

                assert!(app.world().get::<Position>(e_7_without_rb).is_none());
                assert!(app.world().get::<Rotation>(e_7_without_rb).is_none());
            }
        }
    }
}
