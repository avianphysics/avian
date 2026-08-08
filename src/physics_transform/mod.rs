//! Manages physics transforms and synchronizes them with [`Transform`].
//!
//! See [`PhysicsTransformPlugin`].

mod transform;
#[allow(unused_imports)]
pub(crate) use transform::init_physics_transform;
pub use transform::{Position, PreSolveDeltaPosition, PreSolveDeltaRotation, Rotation};

mod helper;
pub use helper::PhysicsTransformHelper;

#[cfg(test)]
mod tests;

use crate::{
    prelude::*,
    schedule::{LastPhysicsTick, is_changed_after_tick},
    utils::{MIN_PAR_ITER_ENTITIES, ParallelQueryForEach},
};
use approx::AbsDiffEq;
use bevy::{
    ecs::{
        change_detection::Tick, intern::Interned, schedule::ScheduleLabel, system::SystemChangeTick,
    },
    math::Affine3A,
    prelude::*,
    transform::systems::{mark_dirty_trees, propagate_parent_transforms, sync_simple_transforms},
};

/// Manages physics transforms and synchronizes them with [`Transform`].
///
/// # Syncing Between [`Position`]/[`Rotation`] and [`Transform`]
///
/// By default, each body's `Transform` will be updated when [`Position`] or [`Rotation`]
/// change, and vice versa. This means that you can use any of these components to move
/// or position bodies, and the changes be reflected in the other components.
///
/// You can configure what data is synchronized and how it is synchronized
/// using the [`PhysicsTransformConfig`] resource.
///
/// # `Transform` Hierarchies
///
/// When synchronizing changes in [`Position`] or [`Rotation`] to `Transform`,
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
            app.register_required_components::<Position, Transform>();
            app.register_required_components::<Rotation, Transform>();
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
    /// Updates [`Position`] and [`Rotation`] based on [`Transform`] changes
    /// in [`PhysicsTransformSystems::TransformToPosition`],
    ///
    /// This allows using transforms for moving and positioning bodies,
    ///
    /// Default: `true`
    pub transform_to_position: bool,
    /// Updates [`Transform`] based on [`Position`] and [`Rotation`] changes
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
    /// Updates [`Position`] and [`Rotation`] based on [`Transform`] changes before physics simulation.
    TransformToPosition,
    /// Updates [`Transform`] based on [`Position`] and [`Rotation`] changes after physics simulation.
    PositionToTransform,
}

/// A deprecated alias for [`PhysicsTransformSystems`].
#[deprecated(since = "0.4.0", note = "Renamed to `PhysicsTransformSystems`")]
pub type PhysicsTransformSet = PhysicsTransformSystems;

/// Copies [`GlobalTransform`] changes to [`Position`] and [`Rotation`].
/// This allows users to use transforms for moving and positioning bodies and colliders.
///
/// To account for hierarchies, transform propagation should be run before this system.
#[allow(clippy::type_complexity)]
pub fn transform_to_position(
    mut query: Query<(&GlobalTransform, &mut Position, &mut Rotation)>,
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

    // If the `GlobalTransform` translation and `Position` differ by less than 0.01 mm, we ignore the change.
    let distance_tolerance = length_unit.real() * 1e-5;

    let last_physics_tick = last_physics_tick.0;

    query.par_for_each_mut(
        MIN_PAR_ITER_ENTITIES,
        |(global_transform, mut position, mut rotation)| {
            let affine = global_transform.affine();

            let position_changed = !position.is_added()
                && is_changed_after_tick(
                    Ref::from(position.reborrow()),
                    last_physics_tick,
                    this_run,
                );
            if !position_changed {
                #[cfg(feature = "2d")]
                let transform_translation = affine.translation.truncate().real();
                #[cfg(feature = "3d")]
                let transform_translation = Vec3::from(affine.translation).real();

                if position.abs_diff_ne(&transform_translation, distance_tolerance) {
                    position.0 = transform_translation;
                }
            }

            let rotation_changed = !rotation.is_added()
                && is_changed_after_tick(
                    Ref::from(rotation.reborrow()),
                    last_physics_tick,
                    this_run,
                );
            if !rotation_changed {
                let transform_rotation = rotation_from_affine(&affine);
                // The rotations differ by more than the tolerance if the cosine of the angle
                // between them is smaller than the cosine of the tolerance angle.
                if cos_angle_between(*rotation, transform_rotation) < ROTATION_COS_TOLERANCE {
                    *rotation = transform_rotation;
                }
            }
        },
    );
}

/// The cosine of the angle below which a difference between the `GlobalTransform` rotation
/// and [`Rotation`] is ignored. This corresponds to an angle of 0.1 degrees.
const ROTATION_COS_TOLERANCE: f32 = 0.999_998_5;

/// Returns the cosine of the angle between two rotations.
///
/// This is a cheaper alternative to `Rotation::angle_between`,
/// as it avoids inverse trigonometric functions.
#[inline]
fn cos_angle_between(a: Rotation, b: Rotation) -> f32 {
    #[cfg(feature = "2d")]
    {
        a.cos * b.cos + a.sin * b.sin
    }
    #[cfg(feature = "3d")]
    {
        // The angle between two unit quaternions is `2 * acos(|dot|)`,
        // and `cos(2 * acos(x)) == 2 * x^2 - 1`.
        let dot = a.dot(b.0);
        2.0 * dot * dot - 1.0
    }
}

/// Extracts the [`Rotation`] from the affine transform of a `GlobalTransform`.
///
/// This is equivalent to `Rotation::from(global_transform.compute_transform().rotation)`,
/// but avoids the full scale-rotation-translation decomposition, which is comparatively expensive.
#[inline]
fn rotation_from_affine(affine: &Affine3A) -> Rotation {
    let mat = affine.matrix3;

    let det_sign = mat.determinant().signum();

    #[cfg(feature = "2d")]
    {
        let x_axis = Vec2::new(mat.x_axis.x, mat.x_axis.y) * det_sign;
        let x_axis = x_axis.normalize_or(Vec2::X);
        Rotation {
            cos: x_axis.x,
            sin: x_axis.y,
        }
    }
    #[cfg(feature = "3d")]
    {
        let x_axis = (mat.x_axis * det_sign).normalize_or(Vec3A::X);
        let y_axis = mat.y_axis.normalize_or(Vec3A::Y);
        let z_axis = mat.z_axis.normalize_or(Vec3A::Z);
        Rotation(Quat::from_mat3a(&Mat3A::from_cols(x_axis, y_axis, z_axis)))
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
    &'static Position,
    &'static Rotation,
    Option<&'static ChildOf>,
);

type PosToTransformFilter = (
    Or<(With<RigidBody>, With<ApplyPosToTransform>)>,
    Or<(Changed<Position>, Changed<Rotation>)>,
);

type ParentComponents = (
    &'static GlobalTransform,
    Option<&'static Position>,
    Option<&'static Rotation>,
);

/// Copies [`Position`] and [`Rotation`] changes to [`Transform`].
/// This allows users and the engine to use these components for moving and positioning bodies.
///
/// Nested rigid bodies move independently of each other, so the [`Transform`]s of child entities are updated
/// based on their own and their parent's [`Position`] and [`Rotation`].
#[cfg(feature = "2d")]
pub fn position_to_transform(
    mut query: Query<PosToTransformComponents, PosToTransformFilter>,
    parents: Query<ParentComponents, With<Children>>,
) {
    for (mut transform, pos, rot, parent) in &mut query {
        if let Some(&ChildOf(parent)) = parent {
            if let Ok((parent_transform, parent_pos, parent_rot)) = parents.get(parent) {
                // Compute the global transform of the parent using its Position and Rotation
                let parent_transform = parent_transform.compute_transform();
                let parent_pos = parent_pos.map_or(parent_transform.translation, |pos| {
                    pos.f32().extend(parent_transform.translation.z)
                });
                let parent_rot =
                    parent_rot.map_or(parent_transform.rotation, |rot| Quat::from(*rot));
                let parent_scale = parent_transform.scale;
                let parent_transform = Transform::from_translation(parent_pos)
                    .with_rotation(parent_rot)
                    .with_scale(parent_scale);

                // The new local transform of the child body,
                // computed from the its global transform and its parents global transform
                let new_transform = GlobalTransform::from(
                    Transform::from_translation(
                        pos.f32()
                            .extend(parent_pos.z + transform.translation.z * parent_scale.z),
                    )
                    .with_rotation(Quat::from(*rot)),
                )
                .reparented_to(&GlobalTransform::from(parent_transform));

                transform.translation = new_transform.translation;
                transform.rotation = new_transform.rotation;
            }
        } else {
            transform.translation = pos.f32().extend(transform.translation.z);
            transform.rotation = Quat::from(*rot);
        }
    }
}

/// Copies [`Position`] and [`Rotation`] changes to [`Transform`].
/// This allows users and the engine to use these components for moving and positioning bodies.
///
/// Nested rigid bodies move independently of each other, so the [`Transform`]s of child entities are updated
/// based on their own and their parent's [`Position`] and [`Rotation`].
#[cfg(feature = "3d")]
pub fn position_to_transform(
    mut query: Query<PosToTransformComponents, PosToTransformFilter>,
    parents: Query<ParentComponents, With<Children>>,
) {
    for (mut transform, pos, rot, parent) in &mut query {
        if let Some(&ChildOf(parent)) = parent {
            if let Ok((parent_transform, parent_pos, parent_rot)) = parents.get(parent) {
                // Compute the global transform of the parent using its Position and Rotation
                let parent_transform = parent_transform.compute_transform();
                let parent_pos = parent_pos.map_or(parent_transform.translation, |pos| pos.f32());
                let parent_rot = parent_rot.map_or(parent_transform.rotation, |rot| rot.0);
                let parent_scale = parent_transform.scale;
                let parent_transform = Transform::from_translation(parent_pos)
                    .with_rotation(parent_rot)
                    .with_scale(parent_scale);

                // The new local transform of the child body,
                // computed from the its global transform and its parents global transform
                let new_transform = GlobalTransform::from(
                    Transform::from_translation(pos.f32()).with_rotation(rot.0),
                )
                .reparented_to(&GlobalTransform::from(parent_transform));

                transform.translation = new_transform.translation;
                transform.rotation = new_transform.rotation;
            }
        } else {
            transform.translation = pos.f32();
            transform.rotation = rot.0;
        }
    }
}
