use core::marker::PhantomData;

use crate::{collision::collider::EnlargedAabb, prelude::*};
use bevy::{ecs::system::StaticSystemParam, prelude::*};
use obvhs::{
    aabb::Aabb,
    bvh2::{
        insertion_removal::SiblingInsertionCandidate, leaf_collapser::collapse, node::Bvh2Node,
        reinsertion::ReinsertionOptimizer, Bvh2,
    },
    heapstack::HeapStack,
    ploc::{PlocSearchDistance, SortPrecision},
    BvhBuildParams,
};

pub struct ColliderTreePlugin<C: AnyCollider>(PhantomData<C>);

impl<C: AnyCollider> Default for ColliderTreePlugin<C> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

impl<C: AnyCollider> Plugin for ColliderTreePlugin<C> {
    fn build(&self, app: &mut App) {
        app.init_resource::<ColliderTrees>();

        // Allowing ambiguities is required so that it's possible
        // to have multiple collision backends at the same time.
        app.add_systems(
            PhysicsSchedule,
            update_aabb::<C>
                .in_set(PhysicsStepSet::BroadPhase)
                .after(BroadPhaseSet::First)
                .before(BroadPhaseSet::UpdateStructures)
                .ambiguous_with_all(),
        );

        app.add_systems(
            PhysicsSchedule,
            optimize_trees.in_set(BroadPhaseSet::UpdateStructures),
        );

        // Initialize `ColliderAabb` for colliders.
        app.add_observer(
            |trigger: Trigger<OnAdd, C>,
             mut query: Query<(
                &C,
                &Position,
                &Rotation,
                Option<&CollisionMargin>,
                &mut ColliderAabb,
            )>,
             narrow_phase_config: Res<NarrowPhaseConfig>,
             length_unit: Res<PhysicsLengthUnit>,
             collider_context: StaticSystemParam<C::Context>| {
                let contact_tolerance = length_unit.0 * narrow_phase_config.contact_tolerance;
                let aabb_context = AabbContext::new(trigger.target(), &*collider_context);

                if let Ok((collider, pos, rot, collision_margin, mut aabb)) =
                    query.get_mut(trigger.target())
                {
                    let collision_margin = collision_margin.map_or(0.0, |m| m.0);
                    *aabb = collider
                        .aabb_with_context(pos.0, *rot, aabb_context)
                        .grow(Vector::splat(contact_tolerance + collision_margin));
                }
            },
        );

        app.add_observer(
            |trigger: Trigger<OnAdd, EnlargedAabb>,
             mut collider_query: Query<(
                &RigidBody,
                &EnlargedAabb,
                &mut BroadPhaseProxyIndex,
                Option<&CollisionLayers>,
            )>,
             mut trees: ResMut<ColliderTrees>| {
                let entity = trigger.target();

                let Ok((rb, enlarged_aabb, mut proxy_index, layers)) =
                    collider_query.get_mut(entity)
                else {
                    return;
                };

                let proxy = BvhProxy {
                    entity,
                    layers: layers.copied().unwrap_or_default(),
                };

                match *rb {
                    RigidBody::Dynamic | RigidBody::Kinematic => {
                        proxy_index.0 = trees
                            .dynamic_tree
                            .add_proxy(Aabb::from(enlarged_aabb.get()), proxy);
                    }
                    RigidBody::Static => {
                        proxy_index.0 = trees
                            .static_tree
                            .add_proxy(Aabb::from(enlarged_aabb.get()), proxy);
                    }
                }
            },
        );

        // TODO: Remove proxies when colliders are removed or disabled.
        app.add_observer(
            |trigger: Trigger<OnRemove, EnlargedAabb>,
             collider_query: Query<(&BroadPhaseProxyIndex, &ColliderOf)>,
             body_query: Query<&RigidBody>,
             mut trees: ResMut<ColliderTrees>| {
                let entity = trigger.target();

                let Ok((proxy_index, &ColliderOf { body })) = collider_query.get(entity) else {
                    return;
                };

                let Ok(rb) = body_query.get(body) else {
                    return;
                };

                match *rb {
                    RigidBody::Dynamic | RigidBody::Kinematic => {
                        trees.dynamic_tree.remove_proxy(proxy_index.0);
                    }
                    RigidBody::Static => {
                        trees.static_tree.remove_proxy(proxy_index.0);
                    }
                }
            },
        );
    }
}

/// The index of a proxy in the broad phase.
#[derive(Component, Clone, Copy, Debug, Default, Reflect)]
pub struct BroadPhaseProxyIndex(pub u32);

/// Trees for accelerating queries on a set of colliders.
#[derive(Resource)]
pub struct ColliderTrees {
    pub dynamic_tree: ColliderTree,
    pub static_tree: ColliderTree,
}

impl Default for ColliderTrees {
    fn default() -> Self {
        Self {
            dynamic_tree: ColliderTree {
                bvh: Bvh2 {
                    nodes: vec![Bvh2Node::new(default(), 1, 0)],
                    primitive_indices: vec![0],
                    ..default()
                },
                proxies: vec![Some(BvhProxy {
                    entity: Entity::PLACEHOLDER,
                    layers: CollisionLayers::ALL,
                })],
                insertion_stack: HeapStack::new_with_capacity(2000),
            },
            static_tree: ColliderTree {
                bvh: Bvh2 {
                    nodes: vec![Bvh2Node::new(default(), 1, 0)],
                    primitive_indices: vec![0],
                    ..default()
                },
                proxies: vec![Some(BvhProxy {
                    entity: Entity::PLACEHOLDER,
                    layers: CollisionLayers::ALL,
                })],
                insertion_stack: HeapStack::new_with_capacity(2000),
            },
        }
    }
}

/// A Bounding Volume Hierarchy for accelerating queries on a set of colliders.
///
/// Add the [`BroadPhasePairs`] component to use this tree for broad phase collision detection.
#[derive(Default)]
pub struct ColliderTree {
    pub bvh: Bvh2,
    pub proxies: Vec<Option<BvhProxy>>,
    pub insertion_stack: HeapStack<SiblingInsertionCandidate>,
}

/// A component that stores pairs of entities with overlapping AABBs for a [`ColliderTree`].
#[derive(Component, Clone, Debug, Default)]
pub struct BroadPhasePairs(pub Vec<(Entity, Entity)>);

impl ColliderTree {
    pub fn add_proxy(&mut self, aabb: Aabb, proxy: BvhProxy) -> u32 {
        let id = self.proxies.len() as u32;
        self.bvh
            .insert_primitive(aabb, id, &mut self.insertion_stack);
        self.proxies.push(Some(proxy));
        id
    }

    pub fn remove_proxy(&mut self, proxy_index: u32) {
        // TODO: `SlotMao`
        self.proxies[proxy_index as usize] = None;
        self.bvh.remove_primitive(proxy_index);
    }

    /// Moves a proxy to a new position in the tree.
    ///
    /// Returns the new index of the proxy.
    pub fn move_proxy(&mut self, proxy_index: u32, aabb: Aabb) -> u32 {
        self.bvh.remove_primitive(proxy_index);
        self.bvh
            .insert_primitive(aabb, proxy_index, &mut self.insertion_stack) as u32
    }

    /// Fully rebuilds the tree from the given list of AABBs.
    pub fn rebuild_full(&mut self, aabbs: &[Aabb]) {
        self.bvh = PlocSearchDistance::Minimum.build(
            aabbs,
            (0..aabbs.len() as u32).collect(),
            SortPrecision::U64,
            0,
        );
    }

    /// Restructures the tree using parallel reinsertion, optimizing node locations based on SAH cost.
    pub fn optimize(&mut self) {
        let config = BvhBuildParams::fastest_build();
        ReinsertionOptimizer::run(&mut self.bvh, config.reinsertion_batch_ratio, None);
        collapse(
            &mut self.bvh,
            config.max_prims_per_leaf,
            config.collapse_traversal_cost,
        );
        ReinsertionOptimizer::run(
            &mut self.bvh,
            config.reinsertion_batch_ratio
                * config.post_collapse_reinsertion_batch_ratio_multiplier,
            None,
        );
    }
}

#[derive(Clone, Debug)]
pub struct BvhProxy {
    pub entity: Entity,
    pub layers: CollisionLayers,
}

impl From<ColliderAabb> for Aabb {
    fn from(value: ColliderAabb) -> Self {
        Self {
            #[cfg(feature = "3d")]
            min: value.min.into(),
            #[cfg(feature = "3d")]
            max: value.max.into(),
            #[cfg(feature = "2d")]
            min: value.min.extend(0.0).into(),
            #[cfg(feature = "2d")]
            max: value.max.extend(0.0).into(),
        }
    }
}

/// Updates the Axis-Aligned Bounding Boxes of all colliders.
#[allow(clippy::type_complexity)]
fn update_aabb<C: AnyCollider>(
    mut commands: Commands,
    mut colliders: Query<
        (
            Entity,
            &C,
            &mut ColliderAabb,
            &mut EnlargedAabb,
            &mut BroadPhaseProxyIndex,
            &Position,
            &Rotation,
            Option<&ColliderOf>,
            Option<&CollisionMargin>,
            Option<&SpeculativeMargin>,
            Has<SweptCcd>,
            Option<&LinearVelocity>,
            Option<&AngularVelocity>,
        ),
        Or<(
            Changed<Position>,
            Changed<Rotation>,
            Changed<LinearVelocity>,
            Changed<AngularVelocity>,
            Changed<C>,
        )>,
    >,
    rb_query: Query<(
        &RigidBody,
        &Position,
        &ComputedCenterOfMass,
        Option<&LinearVelocity>,
        Option<&AngularVelocity>,
    )>,
    narrow_phase_config: Res<NarrowPhaseConfig>,
    mut collider_trees: ResMut<ColliderTrees>,
    length_unit: Res<PhysicsLengthUnit>,
    time: Res<Time>,
    collider_context: StaticSystemParam<C::Context>,
) {
    let delta_secs = time.delta_seconds_adjusted();
    let default_speculative_margin = length_unit.0 * narrow_phase_config.default_speculative_margin;
    let contact_tolerance = length_unit.0 * narrow_phase_config.contact_tolerance;
    let margin = length_unit.0 * 0.05;

    let mut stack = HeapStack::new_with_capacity(2000);

    collider_trees.dynamic_tree.bvh.init_primitives_to_nodes();
    collider_trees.static_tree.bvh.init_primitives_to_nodes();

    for (
        entity,
        collider,
        mut aabb,
        mut enlarged_aabb,
        proxy_index,
        pos,
        rot,
        collider_of,
        collision_margin,
        speculative_margin,
        has_swept_ccd,
        lin_vel,
        ang_vel,
    ) in &mut colliders
    {
        let collision_margin = collision_margin.map_or(0.0, |margin| margin.0);
        let speculative_margin = if has_swept_ccd {
            Scalar::MAX
        } else {
            speculative_margin.map_or(default_speculative_margin, |margin| margin.0)
        };

        let context = AabbContext::new(entity, &*collider_context);

        if speculative_margin <= 0.0 {
            *aabb = collider
                .aabb_with_context(pos.0, *rot, context)
                .grow(Vector::splat(contact_tolerance + collision_margin));
            continue;
        }

        // TODO: Sensor tree
        let body_bundle = collider_of.and_then(|&ColliderOf { body }| rb_query.get(body).ok());
        let rb = body_bundle.map(|bundle| *bundle.0);

        // Expand the AABB based on the body's velocity and CCD speculative margin.
        let (lin_vel, ang_vel) = if let (Some(lin_vel), Some(ang_vel)) = (lin_vel, ang_vel) {
            (*lin_vel, *ang_vel)
        } else if let Some((_rb, parent_pos, center_of_mass, Some(lin_vel), Some(ang_vel))) =
            body_bundle
        {
            // If the rigid body is rotating, off-center colliders will orbit around it,
            // which affects their linear velocities. We need to compute the linear velocity
            // at the offset position.
            // TODO: This assumes that the colliders would continue moving in the same direction,
            //       but because they are orbiting, the direction will change. We should take
            //       into account the uniform circular motion.
            let offset = pos.0 - parent_pos.0 - center_of_mass.0;
            #[cfg(feature = "2d")]
            let vel_at_offset =
                lin_vel.0 + Vector::new(-ang_vel.0 * offset.y, ang_vel.0 * offset.x) * 1.0;
            #[cfg(feature = "3d")]
            let vel_at_offset = lin_vel.0 + ang_vel.cross(offset);
            (LinearVelocity(vel_at_offset), *ang_vel)
        } else {
            (LinearVelocity::ZERO, AngularVelocity::ZERO)
        };

        // Current position and predicted position for next feame
        let (start_pos, start_rot) = (*pos, *rot);
        let (end_pos, end_rot) = {
            #[cfg(feature = "2d")]
            {
                (
                    pos.0
                        + (lin_vel.0 * delta_secs)
                            .clamp_length_max(speculative_margin.max(contact_tolerance)),
                    *rot * Rotation::radians(ang_vel.0 * delta_secs),
                )
            }
            #[cfg(feature = "3d")]
            {
                let end_rot =
                    Rotation(Quaternion::from_scaled_axis(ang_vel.0 * delta_secs) * rot.0)
                        .fast_renormalize();
                (
                    pos.0
                        + (lin_vel.0 * delta_secs)
                            .clamp_length_max(speculative_margin.max(contact_tolerance)),
                    end_rot,
                )
            }
        };
        // Compute swept AABB, the space that the body would occupy if it was integrated for one frame
        // TODO: Should we expand the AABB in all directions for speculative contacts?
        *aabb = collider
            .swept_aabb_with_context(start_pos.0, start_rot, end_pos, end_rot, context)
            .grow(Vector::splat(collision_margin));

        let Some(rb) = rb else {
            continue;
        };

        let moved = enlarged_aabb.update(&aabb, margin);
        if moved {
            commands.entity(entity).insert(MovedProxy);

            // Update the collider tree.
            match rb {
                RigidBody::Dynamic | RigidBody::Kinematic => {
                    let node_id =
                        collider_trees.dynamic_tree.bvh.primitives_to_nodes[proxy_index.0 as usize];
                    collider_trees
                        .dynamic_tree
                        .bvh
                        .resize_node(node_id as usize, Aabb::from(enlarged_aabb.get()));
                    collider_trees
                        .dynamic_tree
                        .bvh
                        .reinsert_node(node_id as usize, &mut stack);
                }
                RigidBody::Static => {
                    let node_id =
                        collider_trees.static_tree.bvh.primitives_to_nodes[proxy_index.0 as usize];
                    collider_trees
                        .static_tree
                        .bvh
                        .resize_node(node_id as usize, Aabb::from(enlarged_aabb.get()));
                    collider_trees
                        .static_tree
                        .bvh
                        .reinsert_node(node_id as usize, &mut stack);
                }
            }
        }
    }
}

/// A marker component for a collider whose [`ColliderAabb`] has moved
/// outside of the previous [`EnlargedAabb`] and needs to be updated in the [`ColliderTree`].
#[derive(Component, Default)]
pub struct MovedProxy;

// TODO: Run this in parallel with the narrow phase.
fn optimize_trees(mut collider_trees: ResMut<ColliderTrees>) {
    collider_trees.dynamic_tree.optimize();
}
