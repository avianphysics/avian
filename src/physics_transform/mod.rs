//! Manages physics transforms and synchronizes them with [`Transform`].
//!
//! See [`PhysicsTransformPlugin`].

mod transform;
#[allow(unused_imports)]
pub(crate) use transform::init_physics_transform;
pub use transform::{PhysicsTransform, PreSolveDeltaPosition, PreSolveDeltaRotation};

mod helper;
pub use helper::PhysicsTransformHelper;

#[cfg(test)]
mod tests;

#[cfg(feature = "3d")]
use crate::math::QuatExt;
#[cfg(feature = "2d")]
use crate::math::Rot2Ext;
use crate::{
    prelude::*,
    schedule::{LastPhysicsTick, is_changed_after_tick},
};
use approx::AbsDiffEq;
use bevy::{
    ecs::{
        change_detection::Tick, intern::Interned, schedule::ScheduleLabel, system::SystemChangeTick,
    },
    prelude::*,
    transform::systems::{mark_dirty_trees, propagate_parent_transforms, sync_simple_transforms},
};

/// Manages physics transforms and synchronizes them with [`Transform`].
///
/// # Syncing Between [`PhysicsTransform`] and [`Transform`]
///
/// By default, each body's `Transform` will be updated when [`PhysicsTransform`]
/// changes, and vice versa. This means that you can use any of these components to move
/// or position bodies, and the changes be reflected in the other components.
///
/// You can configure what data is synchronized and how it is synchronized
/// using the [`PhysicsTransformConfig`] resource.
///
/// # `Transform` Hierarchies
///
/// When synchronizing changes in [`PhysicsTransform`] to `Transform`,
/// the engine treats nested [rigid bodies](RigidBody) as a flat structure. This means that
/// the bodies move independently of the parents, and moving the parent will not affect the child.
///
/// If you would like a child entity to be rigidly attached to its parent, you could use a [`FixedJoint`]
/// or write your own system to handle hierarchies differently.
pub struct PhysicsTransformPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl PhysicsTransformPlugin {
    /// Creates a [`PhysicsTransformPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `FixedPostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for PhysicsTransformPlugin {
    fn default() -> Self {
        Self::new(FixedPostUpdate)
    }
}

impl Plugin for PhysicsTransformPlugin {
    fn build(&self, app: &mut App) {
        app.init_resource::<PhysicsTransformConfig>();

        // In case `TransformPlugin` is not added
        app.init_resource::<StaticTransformOptimizations>();

        if app
            .world()
            .resource::<PhysicsTransformConfig>()
            .position_to_transform
        {
            app.register_required_components::<PhysicsTransform, Transform>();
        }

        // Run transform propagation and transform-to-position synchronization before physics.
        app.configure_sets(
            self.schedule,
            (
                PhysicsTransformSystems::Propagate,
                PhysicsTransformSystems::TransformToPosition,
            )
                .chain()
                .in_set(PhysicsSystems::Prepare),
        );
        app.add_systems(
            self.schedule,
            (
                mark_dirty_trees,
                propagate_parent_transforms,
                sync_simple_transforms,
            )
                .chain()
                .in_set(PhysicsTransformSystems::Propagate)
                .run_if(|config: Res<PhysicsTransformConfig>| config.propagate_before_physics),
        );
        app.add_systems(
            self.schedule,
            transform_to_position
                .in_set(PhysicsTransformSystems::TransformToPosition)
                .run_if(|config: Res<PhysicsTransformConfig>| config.transform_to_position),
        );

        // Run position-to-transform synchronization after physics.
        app.configure_sets(
            self.schedule,
            PhysicsTransformSystems::PositionToTransform.in_set(PhysicsSystems::Writeback),
        );
        app.add_systems(
            self.schedule,
            position_to_transform
                .in_set(PhysicsTransformSystems::PositionToTransform)
                .run_if(|config: Res<PhysicsTransformConfig>| config.position_to_transform),
        );
    }
}

/// Configures how physics transforms are managed and synchronized with [`Transform`].
#[derive(Resource, Reflect, Clone, Debug, PartialEq, Eq)]
#[reflect(Resource)]
pub struct PhysicsTransformConfig {
    /// If true, [`Transform`] is propagated before stepping physics to ensure that
    /// [`GlobalTransform`] is up-to-date.
    ///
    /// Default: `true`
    pub propagate_before_physics: bool,
    /// Updates [`PhysicsTransform`] based on [`Transform`] changes
    /// in [`PhysicsTransformSystems::TransformToPosition`],
    ///
    /// This allows using transforms for moving and positioning bodies,
    ///
    /// Default: `true`
    pub transform_to_position: bool,
    /// Updates [`Transform`] based on [`PhysicsTransform`] changes
    /// in [`PhysicsTransformSystems::PositionToTransform`],
    ///
    /// Default: `true`
    pub position_to_transform: bool,
    /// Updates [`Collider::scale()`] based on transform changes.
    ///
    /// This allows using transforms for scaling colliders.
    ///
    /// Default: `true`
    pub transform_to_collider_scale: bool,
}

impl Default for PhysicsTransformConfig {
    fn default() -> Self {
        PhysicsTransformConfig {
            propagate_before_physics: true,
            position_to_transform: true,
            transform_to_position: true,
            transform_to_collider_scale: true,
        }
    }
}

/// System sets for managing physics transforms.
#[derive(SystemSet, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum PhysicsTransformSystems {
    /// Propagates [`Transform`] before physics simulation.
    Propagate,
    /// Updates [`PhysicsTransform`] based on [`Transform`] changes before physics simulation.
    TransformToPosition,
    /// Updates [`Transform`] based on [`PhysicsTransform`] changes after physics simulation.
    PositionToTransform,
}

/// A deprecated alias for [`PhysicsTransformSystems`].
#[deprecated(since = "0.4.0", note = "Renamed to `PhysicsTransformSystems`")]
pub type PhysicsTransformSet = PhysicsTransformSystems;

/// Copies [`GlobalTransform`] changes to [`PhysicsTransform`].
/// This allows users to use transforms for moving and positioning bodies and colliders.
///
/// To account for hierarchies, transform propagation should be run before this system.
#[allow(clippy::type_complexity)]
pub fn transform_to_position(
    mut query: Query<(&GlobalTransform, &mut PhysicsTransform)>,
    length_unit: Res<PhysicsLengthUnit>,
    last_physics_tick: Res<LastPhysicsTick>,
    system_tick: SystemChangeTick,
) {
    // On the first tick, the last physics tick and system tick are both defaulted to 0,
    // but to handle change detection correctly, the system tick should always be larger.
    // So we use a minimum system tick of 1 here.
    let this_run = if last_physics_tick.0.get() == 0 {
        Tick::new(1)
    } else {
        system_tick.this_run()
    };

    // If the `GlobalTransform` translation and physics translation differ by less than 0.01 mm,
    // we ignore the change.
    let distance_tolerance = length_unit.real() * 1e-5;
    // If the `GlobalTransform` rotation and physics rotation differ by less than 0.1 degrees,
    // we ignore the change.
    let rotation_tolerance = 0.1f32.to_radians();

    for (global_transform, mut physics_transform) in &mut query {
        let physics_transform_changed = !physics_transform.is_added()
            && is_changed_after_tick(
                Ref::from(physics_transform.reborrow()),
                last_physics_tick.0,
                this_run,
            );
        if physics_transform_changed {
            continue;
        }

        let global_transform = global_transform.compute_transform();
        #[cfg(feature = "2d")]
        let transform_translation = global_transform.translation.truncate().real();
        #[cfg(feature = "3d")]
        let transform_translation = global_transform.translation.real();
        let transform_rotation = Rot::from_quat(global_transform.rotation);

        if physics_transform
            .translation
            .abs_diff_ne(&transform_translation, distance_tolerance)
        {
            physics_transform.translation = transform_translation;
        }

        if angle_between(physics_transform.rotation, transform_rotation).abs() > rotation_tolerance
        {
            physics_transform.rotation = transform_rotation;
        }
    }
}

/// Returns the angle in radians needed to make the two rotations coincide.
#[inline]
fn angle_between(a: Rot, b: Rot) -> f32 {
    #[cfg(feature = "2d")]
    {
        a.angle_to(b)
    }
    #[cfg(feature = "3d")]
    {
        a.angle_between(b)
    }
}

/// Marker component indicating that the `position_to_transform` system should be applied
/// to this entity.
///
/// By default, the `position_to_transform` system only runs for entities that have a
/// [`RigidBody`] component
#[derive(Component, Default)]
pub struct ApplyPosToTransform;

type PosToTransformComponents = (
    &'static mut Transform,
    &'static PhysicsTransform,
    Option<&'static ChildOf>,
);

type PosToTransformFilter = (
    Or<(With<RigidBody>, With<ApplyPosToTransform>)>,
    Changed<PhysicsTransform>,
);

type ParentComponents = (&'static GlobalTransform, Option<&'static PhysicsTransform>);

/// Copies [`PhysicsTransform`] changes to [`Transform`].
/// This allows users and the engine to use this component for moving and positioning bodies.
///
/// Nested rigid bodies move independently of each other, so the [`Transform`]s of child entities are updated
/// based on their own and their parent's [`PhysicsTransform`].
#[cfg(feature = "2d")]
pub fn position_to_transform(
    mut query: Query<PosToTransformComponents, PosToTransformFilter>,
    parents: Query<ParentComponents, With<Children>>,
) {
    for (mut transform, physics_transform, parent) in &mut query {
        if let Some(&ChildOf(parent)) = parent {
            if let Ok((parent_transform, parent_physics_transform)) = parents.get(parent) {
                // Compute the global transform of the parent using its `PhysicsTransform`.
                let parent_transform = parent_transform.compute_transform();
                let parent_pos =
                    parent_physics_transform.map_or(parent_transform.translation, |physics| {
                        physics
                            .translation
                            .f32()
                            .extend(parent_transform.translation.z)
                    });
                let parent_rot = parent_physics_transform
                    .map_or(parent_transform.rotation, |physics| {
                        physics.rotation.to_quat()
                    });
                let parent_scale = parent_transform.scale;
                let parent_transform = Transform::from_translation(parent_pos)
                    .with_rotation(parent_rot)
                    .with_scale(parent_scale);

                // The new local transform of the child body,
                // computed from the its global transform and its parents global transform
                let new_transform = GlobalTransform::from(
                    Transform::from_translation(
                        physics_transform
                            .translation
                            .f32()
                            .extend(parent_pos.z + transform.translation.z * parent_scale.z),
                    )
                    .with_rotation(physics_transform.rotation.to_quat()),
                )
                .reparented_to(&GlobalTransform::from(parent_transform));

                transform.translation = new_transform.translation;
                transform.rotation = new_transform.rotation;
            }
        } else {
            transform.translation = physics_transform
                .translation
                .f32()
                .extend(transform.translation.z);
            transform.rotation = physics_transform.rotation.to_quat();
        }
    }
}

/// Copies [`PhysicsTransform`] changes to [`Transform`].
/// This allows users and the engine to use this component for moving and positioning bodies.
///
/// Nested rigid bodies move independently of each other, so the [`Transform`]s of child entities are updated
/// based on their own and their parent's [`PhysicsTransform`].
#[cfg(feature = "3d")]
pub fn position_to_transform(
    mut query: Query<PosToTransformComponents, PosToTransformFilter>,
    parents: Query<ParentComponents, With<Children>>,
) {
    for (mut transform, physics_transform, parent) in &mut query {
        if let Some(&ChildOf(parent)) = parent {
            if let Ok((parent_transform, parent_physics_transform)) = parents.get(parent) {
                // Compute the global transform of the parent using its `PhysicsTransform`.
                let parent_transform = parent_transform.compute_transform();
                let parent_pos = parent_physics_transform
                    .map_or(parent_transform.translation, |physics| {
                        physics.translation.f32()
                    });
                let parent_rot = parent_physics_transform
                    .map_or(parent_transform.rotation, |physics| physics.rotation);
                let parent_scale = parent_transform.scale;
                let parent_transform = Transform::from_translation(parent_pos)
                    .with_rotation(parent_rot)
                    .with_scale(parent_scale);

                // The new local transform of the child body,
                // computed from the its global transform and its parents global transform
                let new_transform = GlobalTransform::from(
                    Transform::from_translation(physics_transform.translation.f32())
                        .with_rotation(physics_transform.rotation),
                )
                .reparented_to(&GlobalTransform::from(parent_transform));

                transform.translation = new_transform.translation;
                transform.rotation = new_transform.rotation;
            }
        } else {
            transform.translation = physics_transform.translation.f32();
            transform.rotation = physics_transform.rotation;
        }
    }
}
