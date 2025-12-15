use crate::{ancestor_marker::AncestorMarker, prelude::*};
use bevy::prelude::*;

/// A plugin for managing [`ColliderOf`] relationships based on the entity hierarchy.
///
/// By default, colliders are automatically attached to the nearest rigid body ancestor.
///
/// This plugin requires that colliders have the [`ColliderMarker`] component,
/// which is added automatically for colliders if the [`ColliderBackendPlugin`] is enabled.
pub struct ColliderHierarchyPlugin;

impl Plugin for ColliderHierarchyPlugin {
    fn build(&self, app: &mut App) {
        // Imsert `ColliderOf` for colliders that are added to a rigid body.
        app.add_observer(
            |trigger: On<Add, (RigidBody, ColliderMarker)>,
             mut commands: Commands,
             query: Query<(), With<ColliderMarker>>,
             rb_query: Query<Entity, With<RigidBody>>,
             parent_query: Query<&ChildOf>| {
                let entity = trigger.entity;

                // Make sure the entity is a collider.
                if !query.contains(entity) {
                    return;
                }

                // Find the closest rigid body ancestor.
                if let Some(body) = rb_query.get(entity).ok().map_or_else(
                    || {
                        parent_query
                            .iter_ancestors(trigger.entity)
                            .find(|&entity| rb_query.contains(entity))
                    },
                    Some,
                ) {
                    commands.entity(entity).insert(ColliderOf { body });
                }
            },
        );

        // Remove `ColliderOf` when the rigid body or collider is removed.
        app.add_observer(
            |trigger: On<Remove, (RigidBody, ColliderMarker)>,
             mut commands: Commands,
             query: Query<(), With<ColliderMarker>>| {
                let entity = trigger.entity;

                // Make sure the collider is on the same entity as the rigid body.
                if query.contains(entity) {
                    commands.entity(entity).try_remove::<ColliderOf>();
                }
            },
        );

        // Update `ColliderOf` components for colliders when their ancestors change.
        app.add_observer(on_collider_body_changed);

        // Remove `ColliderOf` from colliders when their rigid bodies are removed.
        app.add_observer(on_body_removed);
    }
}

/// Updates [`ColliderOf`] components for colliders when their ancestors change
/// or when a rigid body is added to the hierarchy.
fn on_collider_body_changed(
    trigger: On<Insert, (ChildOf, RigidBody, AncestorMarker<ColliderMarker>)>,
    mut commands: Commands,
    query: Query<
        Has<ColliderMarker>,
        Or<(With<ColliderMarker>, With<AncestorMarker<ColliderMarker>>)>,
    >,
    child_query: Query<&Children>,
    parent_query: Query<&ChildOf>,
    body_query: Query<Entity, With<RigidBody>>,
    collider_query: Query<(), With<ColliderMarker>>,
) {
    let entity = trigger.entity;

    // Skip if the entity is not a collider or an ancestor of a collider.
    let Ok(is_collider) = query.get(entity) else {
        return;
    };

    // Find the closest rigid body ancestor.
    let Some(body) = body_query.get(entity).ok().map_or_else(
        || {
            parent_query
                .iter_ancestors(trigger.entity)
                .find(|&entity| body_query.contains(entity))
        },
        Some,
    ) else {
        return;
    };

    // Insert `ColliderOf` for the new child entity.
    if is_collider {
        commands.entity(entity).insert(ColliderOf { body });
    }

    // Iterate over all descendants starting from the new child entity,
    // and update the `ColliderOf` component to point to the closest rigid body ancestor.
    for child in child_query.iter_descendants(entity) {
        // Stop traversal if we reach another rigid body.
        // FIXME: This might lead to siblings of the rigid body being skipped.
        if body_query.contains(child) {
            break;
        }

        // Update the `ColliderOf` component.
        if collider_query.contains(child) {
            commands.entity(child).insert(ColliderOf { body });
        }
    }
}

/// Removes [`ColliderOf`] from colliders when their rigid bodies are removed.
fn on_body_removed(
    trigger: On<Remove, RigidBody>,
    mut commands: Commands,
    body_collider_query: Query<&RigidBodyColliders>,
) {
    // TODO: Here we assume that rigid bodies are not nested, so `ColliderOf` is simply removed
    //       instead of being updated to point to a new rigid body in the hierarchy.

    let body = trigger.entity;

    // Remove `ColliderOf` from all colliders attached to the rigid body.
    if let Ok(colliders) = body_collider_query.get(body) {
        for collider in colliders.iter() {
            commands
                .entity(collider)
                .try_remove::<(ColliderOf, ColliderTransform)>();
        }
    }
}
