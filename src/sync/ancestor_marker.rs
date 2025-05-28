//! Functionality for marking ancestors of entities with marker components.

use core::marker::PhantomData;

use bevy::prelude::*;

/// A plugin that marks the ancestors of entities that have the given component `C`
/// with the [`AncestorMarker`] component.
///
/// One use case is speeding up transform propagation: we only need to propagate
/// down trees that have a certain type of entity, like a collider or a rigid body.
pub struct AncestorMarkerPlugin<C: Component>(PhantomData<C>);

impl<C: Component> Default for AncestorMarkerPlugin<C> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

impl<C: Component> Plugin for AncestorMarkerPlugin<C> {
    fn build(&self, app: &mut App) {
        // Add `AncestorMarker<C>` for the ancestors of colliders that are inserted as children,
        // until an ancestor that has other `AncestorMarker<C>` entities as children is encountered.
        app.add_observer(
            |trigger: Trigger<OnInsert, (ChildOf, C)>,
             mut commands: Commands,
             collider_query: Query<&C>,
             parent_query: Query<&ChildOf>,
             ancestor_query: Query<(), With<AncestorMarker<C>>>| {
                let entity = trigger.target();
                if collider_query.contains(entity) {
                    add_ancestor_markers(
                        entity,
                        &mut commands,
                        &parent_query,
                        &ancestor_query,
                        false,
                    );
                }
            },
        );

        // Remove `AncestorMarker<C>` from the ancestors of colliders that have been removed or moved to another parent,
        // until an ancestor that has other `AncestorMarker<C>` entities as children is encountered.
        #[allow(clippy::type_complexity)]
        app.add_observer(
            |trigger: Trigger<OnReplace, (ChildOf, C)>,
            mut commands: Commands,
            collider_query: Query<&C>,
            child_query: Query<&Children>,
            parent_query: Query<&ChildOf>,
            ancestor_query: Query<
                (Entity, Has<C>),
                Or<(With<AncestorMarker<C>>, With<C>)>
            >| {
                let entity = trigger.target();
                if collider_query.contains(entity) {
                    remove_ancestor_markers(entity, &mut commands, &parent_query, &child_query, &ancestor_query, false);
                }
            },
        );
    }
}

/// A marker component that marks an entity as an ancestor of an entity with the given component `C`.
///
/// This is added and removed automatically by the [`AncestorMarkerPlugin`] if it is enabled.
#[derive(Component, Copy, Reflect)]
#[reflect(Component, Default)]
pub struct AncestorMarker<C: Component> {
    #[reflect(ignore)]
    _phantom: PhantomData<C>,
}

impl<C: Component> Clone for AncestorMarker<C> {
    fn clone(&self) -> Self {
        Self::default()
    }
}

impl<C: Component> Default for AncestorMarker<C> {
    fn default() -> Self {
        Self {
            _phantom: PhantomData,
        }
    }
}

fn add_ancestor_markers<C: Component>(
    entity: Entity,
    commands: &mut Commands,
    parent_query: &Query<&ChildOf>,
    ancestor_query: &Query<(), With<AncestorMarker<C>>>,
    include_self: bool,
) {
    if include_self {
        commands
            .entity(entity)
            .insert(AncestorMarker::<C>::default());
    }

    // Traverse up the tree, marking entities with `AncestorMarker<C>`
    // until an entity that already has it is encountered.
    for parent_entity in parent_query.iter_ancestors(entity) {
        if ancestor_query.contains(parent_entity) {
            break;
        } else {
            commands
                .entity(parent_entity)
                .insert(AncestorMarker::<C>::default());
        }
    }
}

/// Remove the component from entity, unless it is already despawned.
fn remove_component<T: Bundle>(commands: &mut Commands, entity: Entity) {
    if let Ok(mut entity_commands) = commands.get_entity(entity) {
        entity_commands.try_remove::<T>();
    }
}

#[allow(clippy::type_complexity)]
fn remove_ancestor_markers<C: Component>(
    entity: Entity,
    commands: &mut Commands,
    parent_query: &Query<&ChildOf>,
    child_query: &Query<&Children>,
    ancestor_query: &Query<(Entity, Has<C>), Or<(With<AncestorMarker<C>>, With<C>)>>,
    include_self: bool,
) {
    if include_self {
        // Remove the marker from the `parent` unless a sibling of the `child`
        // is also a marked ancestor or has `C`.
        if let Ok(children) = child_query.get(entity) {
            let keep_marker = ancestor_query
                .iter_many(children)
                .any(|(parent_child, _has_c)| parent_child != entity);
            if keep_marker {
                return;
            } else {
                remove_component::<AncestorMarker<C>>(commands, entity);
            }
        } else {
            // The parent has no children, so it cannot be an ancestor.
            remove_component::<AncestorMarker<C>>(commands, entity);
        }
    }

    // Iterate over ancestors, removing `AncestorMarker<C>` markers until
    // an entity that has other `AncestorMarker<C>` children is encountered.
    let mut previous_parent = entity;
    for parent_entity in parent_query.iter_ancestors(entity) {
        if let Ok(children) = child_query.get(parent_entity) {
            // Keep the marker if `parent_entity` has a child that is a marked ancestor
            // or an entity that has `C`, but not the one that was removed.
            let keep_marker = ancestor_query
                .iter_many(children)
                .any(|(child, has_c)| child != previous_parent || (has_c && child != entity));

            if keep_marker {
                return;
            } else {
                remove_component::<AncestorMarker<C>>(commands, parent_entity);
            }
        } else {
            // The parent has no children, so it cannot be an ancestor.
            remove_component::<AncestorMarker<C>>(commands, entity);
        }

        previous_parent = parent_entity;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Component)]
    struct C;

    #[test]
    fn add_and_remove_component() {
        let mut app = App::new();

        app.add_plugins(AncestorMarkerPlugin::<C>::default());

        // Set up an entity tree like the following:
        //
        //     AN
        //    /  \
        //  BN    CY
        //       /  \
        //     DN    EN
        //    /  \
        //  FY    GY
        //
        // where Y means that the entity has `C`,
        // and N means that the entity does not have `C`.

        let an = app.world_mut().spawn_empty().id();

        let bn = app.world_mut().spawn(ChildOf(an)).id();
        let cy = app.world_mut().spawn((C, ChildOf(an))).id();

        let dn = app.world_mut().spawn(ChildOf(cy)).id();
        let en = app.world_mut().spawn(ChildOf(cy)).id();

        let fy = app.world_mut().spawn((C, ChildOf(dn))).id();
        let gy = app.world_mut().spawn((C, ChildOf(dn))).id();

        // Check that the correct entities have the `AncestorMarker<C>` component.
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(bn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(en).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(fy).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(gy).contains::<AncestorMarker<C>>());

        // Remove `C` from FY. DN, CY, and AN should all keep the `AncestorMarker<C>` marker.
        let mut entity_mut = app.world_mut().entity_mut(fy);
        entity_mut.remove::<C>();

        assert!(app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Remove `C` from GY. The `AncestorMarker<C>` marker should
        // now be removed from DN and CY, but it should remain on AN.
        let mut entity_mut = app.world_mut().entity_mut(gy);
        entity_mut.remove::<C>();

        assert!(!app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Remove `C` from CY. The `AncestorMarker<C>` marker should
        // now be removed from AN.
        let mut entity_mut = app.world_mut().entity_mut(cy);
        entity_mut.remove::<C>();

        assert!(!app.world().entity(an).contains::<AncestorMarker<C>>());
    }

    #[test]
    fn remove_children() {
        let mut app = App::new();

        app.add_plugins(AncestorMarkerPlugin::<C>::default());

        // Set up an entity tree like the following:
        //
        //     AN
        //    /  \
        //  BN    CY
        //       /  \
        //     DN    EN
        //    /  \
        //  FY    GY
        //
        // where Y means that the entity has `C`,
        // and N means that the entity does not have `C`.

        let an = app.world_mut().spawn_empty().id();

        let bn = app.world_mut().spawn(ChildOf(an)).id();
        let cy = app.world_mut().spawn((C, ChildOf(an))).id();

        let dn = app.world_mut().spawn(ChildOf(cy)).id();
        let en = app.world_mut().spawn(ChildOf(cy)).id();

        let fy = app.world_mut().spawn((C, ChildOf(dn))).id();
        let gy = app.world_mut().spawn((C, ChildOf(dn))).id();

        // Check that the correct entities have the `AncestorMarker<C>` component.
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(bn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(en).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(fy).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(gy).contains::<AncestorMarker<C>>());

        // Make FY an orphan.
        let mut entity_mut = app.world_mut().entity_mut(fy);
        entity_mut.remove::<ChildOf>();

        assert!(app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Make GY an orphan. The `AncestorMarker<C>` marker should
        // now be removed from DN and CY, but it should remain on AN.
        let mut entity_mut = app.world_mut().entity_mut(gy);
        entity_mut.remove::<ChildOf>();

        assert!(!app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Make CY an orphan. The `AncestorMarker<C>` marker should
        // now be removed from AN.
        let mut entity_mut = app.world_mut().entity_mut(cy);
        entity_mut.remove::<ChildOf>();

        assert!(!app.world().entity(an).contains::<AncestorMarker<C>>());

        // Make CY a child of AN again. The `AncestorMarker<C>` marker should
        // now be added to AN.
        let mut entity_mut = app.world_mut().entity_mut(an);
        entity_mut.add_child(cy);
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Make CY an orphan and delete AN. This must not crash.
        let mut entity_mut = app.world_mut().entity_mut(cy);
        entity_mut.remove::<ChildOf>();
        entity_mut.despawn();
    }

    #[test]
    fn move_children() {
        let mut app = App::new();

        app.add_plugins(AncestorMarkerPlugin::<C>::default());

        // Set up an entity tree like the following:
        //
        //     AN
        //    /  \
        //  BN    CY
        //       /  \
        //     DN    EN
        //    /  \
        //  FY    GY
        //
        // where Y means that the entity has `C`,
        // and N means that the entity does not have `C`.

        let an = app.world_mut().spawn_empty().id();

        let bn = app.world_mut().spawn(ChildOf(an)).id();
        let cy = app.world_mut().spawn((C, ChildOf(an))).id();

        let dn = app.world_mut().spawn(ChildOf(cy)).id();
        let en = app.world_mut().spawn(ChildOf(cy)).id();

        let fy = app.world_mut().spawn((C, ChildOf(dn))).id();
        let gy = app.world_mut().spawn((C, ChildOf(dn))).id();

        // Check that the correct entities have the `AncestorMarker<C>` component.
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(bn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(en).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(fy).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(gy).contains::<AncestorMarker<C>>());

        // Move FY to be a child of BN. BN should get the `AncestorMarker<C>` component.
        let mut entity_mut = app.world_mut().entity_mut(bn);
        entity_mut.add_child(fy);

        assert!(app.world().entity(bn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Move GY to be a child of BN. The `AncestorMarker<C>` marker should
        // now be removed from DN and CY.
        let mut entity_mut = app.world_mut().entity_mut(bn);
        entity_mut.add_child(gy);

        assert!(!app.world().entity(dn).contains::<AncestorMarker<C>>());
        assert!(!app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Move all children from BN to CY. The `AncestorMarker<C>` marker should
        // now be removed from BN, and CY should get it back.
        let mut entity_mut = app.world_mut().entity_mut(cy);
        entity_mut.add_children(&[fy, gy]);

        assert!(!app.world().entity(bn).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(cy).contains::<AncestorMarker<C>>());
        assert!(app.world().entity(an).contains::<AncestorMarker<C>>());

        // Move all children from CY to BN and remove CY. This must not crash.
        let mut entity_mut = app.world_mut().entity_mut(bn);
        entity_mut.add_children(&[dn, en, fy, gy]);
    }
}
