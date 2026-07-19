use core::marker::PhantomData;

use super::{JointGraph, JointGraphEdge, JointId};
use crate::{
    collision::contact_types::ContactId,
    data_structures::pair_key::PairKey,
    dynamics::joints::EntityConstraint,
    prelude::{
        ContactGraph, ContactStatusChange, ContactStatusChangeQueue, JointCollisionDisabled,
        JointDisabled, PhysicsSchedule, PhysicsStepSystems, RigidBodyColliders,
    },
};
use bevy::{
    ecs::{
        component::ComponentId, entity_disabling::Disabled, lifecycle::HookContext,
        query::QueryFilter, world::DeferredWorld,
    },
    prelude::*,
};

/// A plugin that manages the [`JointGraph`] for a specific [joint] type.
///
/// [joint]: crate::dynamics::joints
pub struct JointGraphPlugin<T: Component + EntityConstraint<2>>(PhantomData<T>);

impl<T: Component + EntityConstraint<2>> Default for JointGraphPlugin<T> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

#[derive(Resource, Default)]
struct JointGraphPluginInitialized;

impl<T: Component + EntityConstraint<2>> Plugin for JointGraphPlugin<T> {
    fn build(&self, app: &mut App) {
        let already_initialized = app
            .world()
            .is_resource_added::<JointGraphPluginInitialized>();

        app.init_resource::<JointGraph>();
        app.add_message::<JointGraphChange>();
        app.init_resource::<JointGraphPluginInitialized>();

        // Automatically add the `JointComponentId` component when the joint is added.
        app.register_required_components::<T, JointComponentId>();

        // Register hooks for adding and removing joints.
        app.world_mut()
            .register_component_hooks::<T>()
            .on_add(on_add_joint)
            .on_remove(on_remove_joint);

        // Add the joint to the joint graph when it is added and the joint is not disabled.
        app.add_observer(
            add_joint_to_graph::<T, Add, T, (With<JointComponentId>, Without<JointDisabled>)>,
        );

        // Remove the joint from the joint graph when it is removed.
        app.add_observer(remove_joint_from_graph::<Remove, T>);

        if !already_initialized {
            // Remove the joint from the joint graph when it is disabled.
            app.add_observer(remove_joint_from_graph::<Add, (Disabled, JointDisabled)>);

            // Remove contacts between bodies when the `JointCollisionDisabled` component is added.
            app.add_observer(on_disable_joint_collision);
        }

        // Add the joint back to the joint graph when `Disabled` is removed.
        app.add_observer(
            add_joint_to_graph::<
                T,
                Remove,
                Disabled,
                (
                    With<JointComponentId>,
                    Or<(With<Disabled>, Without<Disabled>)>,
                    Without<JointDisabled>,
                ),
            >,
        );

        // Add the joint back to the joint graph when `JointDisabled` is removed.
        app.add_observer(add_joint_to_graph::<T, Remove, JointDisabled, With<JointComponentId>>);

        app.add_systems(
            PhysicsSchedule,
            on_change_joint_entities::<T>
                .in_set(PhysicsStepSystems::First)
                .ambiguous_with(PhysicsStepSystems::First),
        );
    }
}

/// A [`Message`] recording a change to the [`JointGraph`].
///
/// This is used to communicate changes to the joint graph to the physics solver,
/// which may need to link and unlink joints from simulation islands or other structures.
#[derive(Message, Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointGraphChange {
    /// A joint was added to the [`JointGraph`].
    Added(JointId),
    /// A joint was removed from the [`JointGraph`].
    Removed(JointId),
}

/// A component that holds the [`ComponentId`] of the [joint] component on this entity, if any.
///
/// [joint]: crate::dynamics::joints
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
pub struct JointComponentId(Option<ComponentId>);

impl JointComponentId {
    /// Creates a new [`JointComponentId`] component with no active joint.
    pub fn new() -> Self {
        Self(None)
    }

    /// Returns the [`ComponentId`] of the active joint component, if any.
    pub fn id(&self) -> Option<ComponentId> {
        self.0
    }
}

fn add_joint_to_graph<
    T: Component + EntityConstraint<2>,
    E: EntityEvent,
    B: Bundle,
    F: QueryFilter,
>(
    trigger: On<E, B>,
    query: Query<(&T, Has<JointCollisionDisabled>), F>,
    mut joint_graph: ResMut<JointGraph>,
    mut joint_graph_changes: MessageWriter<JointGraphChange>,
) {
    let entity = trigger.event_target();

    let Ok((joint, collision_disabled)) = query.get(entity) else {
        return;
    };

    let [body1, body2] = joint.entities();

    // Add the joint to the joint graph.
    let joint_edge = JointGraphEdge::new(entity, body1, body2, collision_disabled);
    let joint_id = joint_graph.add_joint(joint_edge);

    // Record the change.
    joint_graph_changes.write(JointGraphChange::Added(joint_id));
}

fn remove_joint_from_graph<E: EntityEvent, B: Bundle>(
    trigger: On<E, B>,
    mut joint_graph: ResMut<JointGraph>,
    mut joint_graph_changes: MessageWriter<JointGraphChange>,
) {
    let entity = trigger.event_target();

    let Some(joint) = joint_graph.get(entity) else {
        return;
    };

    // Record the change.
    joint_graph_changes.write(JointGraphChange::Removed(joint.id));

    // Remove the joint from the joint graph.
    joint_graph.remove_joint(entity);
}

fn on_add_joint(mut world: DeferredWorld, ctx: HookContext) {
    let entity = ctx.entity;
    let component_id = ctx.component_id;

    let mut joint = world.get_mut::<JointComponentId>(entity).unwrap();
    let old_joint = joint.0;

    // Update the joint component with the new component ID.
    joint.0 = Some(component_id);

    if let Some(old_joint) = old_joint {
        // Joint already exists, remove the old one.
        world.commands().entity(entity).remove_by_id(old_joint);

        #[cfg(feature = "validate")]
        {
            use disqualified::ShortName;

            // Log a warning about the joint replacement in case it was not intentional.
            let components = world.components();
            let old_joint_shortname = components.get_info(old_joint).unwrap().name();
            let old_joint_name = ShortName(&old_joint_shortname);
            let new_joint_shortname = components.get_info(component_id).unwrap().name();
            let new_joint_name = ShortName(&new_joint_shortname);

            warn!(
                "{old_joint_name} was replaced with {new_joint_name} on entity {entity}. An entity can only hold one joint type at a time."
            );
        }
    }
}

fn on_remove_joint(mut world: DeferredWorld, ctx: HookContext) {
    let entity = ctx.entity;
    let component_id = ctx.component_id;

    // Remove the `JointComponentId` from the entity unless the component ID
    // was changed, implying that the joint is being replaced by another one.
    if let Some(mut joint) = world.get_mut::<JointComponentId>(entity)
        && joint.0 == Some(component_id)
    {
        joint.0 = None;

        // Remove the joint component.
        world
            .commands()
            .entity(entity)
            .try_remove::<JointComponentId>();
    }
}

fn on_disable_joint_collision(
    trigger: On<Add, JointCollisionDisabled>,
    query: Query<&RigidBodyColliders>,
    joint_graph: Res<JointGraph>,
    mut contact_graph: ResMut<ContactGraph>,
    mut contact_status_changes: ResMut<ContactStatusChangeQueue>,
) {
    let entity = trigger.entity;

    // Iterate through each collider of the body with fewer colliders,
    // find contacts with the other body, and remove them.
    let Some([body1, body2]) = joint_graph.bodies_of(entity) else {
        return;
    };
    let Ok([colliders1, colliders2]) = query.get_many([body1, body2]) else {
        return;
    };

    let (colliders, other_body) = if colliders1.len() < colliders2.len() {
        (colliders1, body2)
    } else {
        (colliders2, body1)
    };

    let contacts_to_remove: Vec<ContactId> = colliders
        .iter()
        .flat_map(|collider| {
            contact_graph
                .contact_edges_with(collider)
                .filter_map(|edge| {
                    if edge.body1 == Some(other_body) || edge.body2 == Some(other_body) {
                        Some(edge.id)
                    } else {
                        None
                    }
                })
        })
        .collect();

    for contact_id in contacts_to_remove {
        // Remove the contact from the contact graph.
        let pair_key = PairKey::new(body1.index_u32(), body2.index_u32());
        contact_graph.remove_edge_by_id(&pair_key, contact_id);

        // Record the contact removal.
        contact_status_changes.push(ContactStatusChange::StoppedGeneratingConstraints {
            contact_id,
            body1,
            body2,
        });
    }
}

/// Update the joint graph when the entities of a joint change.
fn on_change_joint_entities<T: Component + EntityConstraint<2>>(
    query: Query<(Entity, &T), Changed<T>>,
    mut joint_graph: ResMut<JointGraph>,
    mut joint_graph_changes: MessageWriter<JointGraphChange>,
) {
    for (entity, joint) in &query {
        let [body1, body2] = joint.entities();
        let Some(old_edge) = joint_graph.get(entity) else {
            continue;
        };

        if body1 != old_edge.body1 || body2 != old_edge.body2 {
            let old_id = old_edge.id;

            // Remove the old joint edge.
            if let Some(mut edge) = joint_graph.remove_joint(entity) {
                // Record the removal.
                joint_graph_changes.write(JointGraphChange::Removed(old_id));

                // Update the edge with the new bodies.
                edge.body1 = body1;
                edge.body2 = body2;

                // Add the joint edge back with the new bodies.
                let joint_id = joint_graph.add_joint(edge);

                // Record the addition.
                joint_graph_changes.write(JointGraphChange::Added(joint_id));
            }
        }
    }
}

// TODO: Tests
