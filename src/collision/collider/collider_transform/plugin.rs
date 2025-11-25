use crate::{
    ancestor_marker::{AncestorMarker, AncestorMarkerPlugin},
    physics_transform::PhysicsTransformSystems,
    prelude::*,
};
use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
};

/// A plugin for propagating and updating transforms for colliders.
///
/// - Propagates [`Transform`] to [`GlobalTransform`] and [`ColliderTransform`].
/// - Updates [`Position`] and [`Rotation`] for colliders.
///
/// This plugin requires that colliders have the [`ColliderMarker`] component,
/// which is added automatically for colliders if the [`ColliderBackendPlugin`] is enabled.
pub struct ColliderTransformPlugin {
    schedule: Interned<dyn ScheduleLabel>,
}

impl ColliderTransformPlugin {
    /// Creates a [`ColliderTransformPlugin`] with the schedule that is used for running the [`PhysicsSchedule`].
    ///
    /// The default schedule is `FixedPostUpdate`.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }
}

impl Default for ColliderTransformPlugin {
    fn default() -> Self {
        Self {
            schedule: FixedPostUpdate.intern(),
        }
    }
}

impl Plugin for ColliderTransformPlugin {
    fn build(&self, app: &mut App) {
        // Mark ancestors of colliders with `AncestorMarker<ColliderMarker>`.
        // This is used to speed up `ColliderTransform` propagation by skipping
        // trees that have no colliders.
        app.add_plugins(AncestorMarkerPlugin::<ColliderMarker>::default());

        // Propagate `ColliderTransform`s if there are new colliders.
        // Only traverses trees with `AncestorMarker<ColliderMarker>`.
        app.add_systems(
            self.schedule,
            propagate_collider_transforms.in_set(PhysicsTransformSystems::Propagate),
        );

        let physics_schedule = app
            .get_schedule_mut(PhysicsSchedule)
            .expect("add PhysicsSchedule first");

        // Update child collider positions before narrow phase collision detection.
        // Only traverses trees with `AncestorMarker<ColliderMarker>`.
        physics_schedule
            .add_systems(update_child_collider_position.in_set(PhysicsStepSystems::First));
    }
}

#[allow(clippy::type_complexity)]
pub(crate) fn update_child_collider_position(
    mut collider_query: Query<
        (
            &ColliderTransform,
            &mut Position,
            &mut Rotation,
            &ColliderOf,
        ),
        Without<RigidBody>,
    >,
    rb_query: Query<(&Position, &Rotation), (With<RigidBody>, With<Children>)>,
) {
    for (collider_transform, mut position, mut rotation, collider_of) in &mut collider_query {
        let Ok((rb_pos, rb_rot)) = rb_query.get(collider_of.body) else {
            continue;
        };

        position.0 = rb_pos.0 + rb_rot * collider_transform.translation;
        #[cfg(feature = "2d")]
        {
            *rotation = *rb_rot * collider_transform.rotation;
        }
        #[cfg(feature = "3d")]
        {
            *rotation = (rb_rot.0 * collider_transform.rotation.0)
                .normalize()
                .into();
        }
    }
}

// `ColliderTransform` propagation should only be continued if the child
// is a collider or is a `AncestorMarker<ColliderMarker>`.
type ShouldPropagate = Or<(With<AncestorMarker<ColliderMarker>>, With<ColliderMarker>)>;

/// Updates [`ColliderTransform`]s based on entity hierarchies. Each transform is computed by recursively
/// traversing the children of each rigid body and adding their transforms together to form
/// the total transform relative to the body.
///
/// This is largely a clone of `propagate_transforms` in `bevy_transform`.
#[allow(clippy::type_complexity)]
pub(crate) fn propagate_collider_transforms(
    mut root_query: Query<
        (Entity, Ref<Transform>, &Children),
        (Without<ChildOf>, With<AncestorMarker<ColliderMarker>>),
    >,
    collider_query: Query<
        (
            Ref<Transform>,
            Option<&mut ColliderTransform>,
            Option<&Children>,
        ),
        (With<ChildOf>, ShouldPropagate),
    >,
    parent_query: Query<(Entity, Ref<Transform>, Has<RigidBody>, Ref<ChildOf>), ShouldPropagate>,
) {
    root_query.par_iter_mut().for_each(
        |(entity, transform, children)| {
            for (child, child_transform, is_child_rb, child_of) in parent_query.iter_many(children) {
                assert_eq!(
                    child_of.parent(), entity,
                    "Malformed hierarchy. This probably means that your hierarchy has been improperly maintained, or contains a cycle"
                );
                let changed = transform.is_changed() || child_of.is_changed();
                let parent_transform = ColliderTransform::from(*transform);
                let child_transform = ColliderTransform::from(*child_transform);
                let scale = parent_transform.scale * child_transform.scale;

                // SAFETY:
                // - `child` must have consistent parentage, or the above assertion would panic.
                // Since `child` is parented to a root entity, the entire hierarchy leading to it is consistent.
                // - We may operate as if all descendants are consistent, since `propagate_collider_transform_recursive` will panic before
                //   continuing to propagate if it encounters an entity with inconsistent parentage.
                // - Since each root entity is unique and the hierarchy is consistent and forest-like,
                //   other root entities' `propagate_collider_transform_recursive` calls will not conflict with this one.
                // - Since this is the only place where `collider_query` gets used, there will be no conflicting fetches elsewhere.
                unsafe {
                    propagate_collider_transforms_recursive(
                        if is_child_rb {
                            ColliderTransform {
                                scale,
                                ..default()
                            }
                        } else {
                            ColliderTransform {
                                translation: parent_transform.scale * child_transform.translation,
                                rotation: child_transform.rotation,
                                scale,
                            }
                        },
                        &collider_query,
                        &parent_query,
                        child,
                        changed,
                    );
                }
            }
        },
    );
}

/// Recursively computes the [`ColliderTransform`] for `entity` and all of its descendants
/// by propagating transforms.
///
/// This is largely a clone of `propagate_recursive` in `bevy_transform`.
///
/// # Panics
///
/// If `entity`'s descendants have a malformed hierarchy, this function will panic occur before propagating
/// the transforms of any malformed entities and their descendants.
///
/// # Safety
///
/// - While this function is running, `collider_query` must not have any fetches for `entity`,
///   nor any of its descendants.
/// - The caller must ensure that the hierarchy leading to `entity`
///   is well-formed and must remain as a tree or a forest. Each entity must have at most one parent.
#[allow(clippy::type_complexity)]
unsafe fn propagate_collider_transforms_recursive(
    transform: ColliderTransform,
    collider_query: &Query<
        (
            Ref<Transform>,
            Option<&mut ColliderTransform>,
            Option<&Children>,
        ),
        (With<ChildOf>, ShouldPropagate),
    >,
    parent_query: &Query<(Entity, Ref<Transform>, Has<RigidBody>, Ref<ChildOf>), ShouldPropagate>,
    entity: Entity,
    mut changed: bool,
) {
    let children = {
        // SAFETY: This call cannot create aliased mutable references.
        //   - The top level iteration parallelizes on the roots of the hierarchy.
        //   - The caller ensures that each child has one and only one unique parent throughout the entire
        //     hierarchy.
        //
        // For example, consider the following malformed hierarchy:
        //
        //     A
        //   /   \
        //  B     C
        //   \   /
        //     D
        //
        // D has two parents, B and C. If the propagation passes through C, but the ChildOf component on D points to B,
        // the above check will panic as the origin parent does match the recorded parent.
        //
        // Also consider the following case, where A and B are roots:
        //
        //  A       B
        //   \     /
        //    C   D
        //     \ /
        //      E
        //
        // Even if these A and B start two separate tasks running in parallel, one of them will panic before attempting
        // to mutably access E.
        let Ok((transform_ref, collider_transform, children)) =
            (unsafe { collider_query.get_unchecked(entity) })
        else {
            return;
        };

        changed |=
            transform_ref.is_changed() || collider_transform.as_ref().is_some_and(|t| t.is_added());
        if changed
            && let Some(mut collider_transform) = collider_transform
            && *collider_transform != transform
        {
            *collider_transform = transform;
        }

        children
    };

    let Some(children) = children else { return };
    for (child, child_transform, is_rb, child_of) in parent_query.iter_many(children) {
        assert_eq!(
            child_of.parent(),
            entity,
            "Malformed hierarchy. This probably means that your hierarchy has been improperly maintained, or contains a cycle"
        );

        let child_transform = ColliderTransform::from(*child_transform);
        let scale = transform.scale * child_transform.scale;

        // SAFETY: The caller guarantees that `collider_query` will not be fetched
        // for any descendants of `entity`, so it is safe to call `propagate_collider_transforms_recursive` for each child.
        //
        // The above assertion ensures that each child has one and only one unique parent throughout the
        // entire hierarchy.
        unsafe {
            propagate_collider_transforms_recursive(
                if is_rb {
                    ColliderTransform { scale, ..default() }
                } else {
                    ColliderTransform {
                        translation: transform.transform_point(child_transform.translation),
                        #[cfg(feature = "2d")]
                        rotation: transform.rotation * child_transform.rotation,
                        #[cfg(feature = "3d")]
                        rotation: Rotation(transform.rotation.0 * child_transform.rotation.0),
                        scale,
                    }
                },
                collider_query,
                parent_query,
                child,
                changed || child_of.is_changed(),
            );
        }
    }
}

// TODO: Add thorough tests for propagation. It's pretty error-prone and changes are risky.
