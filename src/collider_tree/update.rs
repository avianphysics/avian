use core::cell::RefCell;
use core::marker::PhantomData;

use crate::{
    collider_tree::{
        ColliderTree, ColliderTreeDiagnostics, ColliderTreeProxy, ColliderTreeProxyKey,
        ColliderTreeSystems, ColliderTreeType, ColliderTrees, ProxyId,
        tree::ColliderTreeProxyFlags,
    },
    collision::collider::EnlargedAabb,
    data_structures::bit_vec::BitVec,
    dynamics::solver::solver_body::SolverBody,
    prelude::*,
    schedule::LastPhysicsTick,
};
use bevy::{
    ecs::{
        change_detection::Tick,
        entity_disabling::Disabled,
        query::QueryFilter,
        system::{StaticSystemParam, SystemChangeTick},
    },
    platform::collections::HashSet,
    prelude::*,
};
use obvhs::aabb::Aabb;
use thread_local::ThreadLocal;

/// A plugin for updating [`ColliderTree`]s for a collider type `C`.
///
/// [`ColliderTree`]: crate::collider_tree::ColliderTree
pub(super) struct ColliderTreeUpdatePlugin<C: AnyCollider>(PhantomData<C>);

impl<C: AnyCollider> Default for ColliderTreeUpdatePlugin<C> {
    fn default() -> Self {
        Self(PhantomData)
    }
}

impl<C: AnyCollider> Plugin for ColliderTreeUpdatePlugin<C> {
    fn build(&self, app: &mut App) {
        // Initialize resources.
        app.init_resource::<MovedProxies>()
            .init_resource::<EnlargedProxies>()
            .init_resource::<LastDynamicKinematicAabbUpdate>();

        // Add systems for updating collider AABBs before physics step.
        // This accounts for manually moved colliders.
        app.add_systems(
            PhysicsSchedule,
            update_moved_collider_aabbs::<C>
                .in_set(ColliderTreeSystems::UpdateAabbs)
                // Allowing ambiguities is required so that it's possible
                // to have multiple collision backends at the same time.
                .ambiguous_with_all(),
        );

        // Clear moved proxies and update dynamic and kinematic collider AABBs.
        app.add_systems(
            PhysicsSchedule,
            (clear_moved_proxies, update_solver_body_aabbs::<C>)
                .chain()
                .after(PhysicsStepSystems::Finalize)
                .before(PhysicsStepSystems::Last),
        );

        // Initialize `ColliderAabb` for colliders.
        app.add_observer(
            |trigger: On<Add, C>,
             mut query: Query<(
                &C,
                &Position,
                &Rotation,
                Option<&CollisionMargin>,
                &mut ColliderAabb,
                &mut EnlargedAabb,
            )>,
             narrow_phase_config: Res<NarrowPhaseConfig>,
             length_unit: Res<PhysicsLengthUnit>,
             collider_context: StaticSystemParam<C::Context>| {
                let contact_tolerance = length_unit.0 * narrow_phase_config.contact_tolerance;
                let aabb_context = AabbContext::new(trigger.entity, &*collider_context);

                if let Ok((collider, pos, rot, collision_margin, mut aabb, mut enlarged_aabb)) =
                    query.get_mut(trigger.entity)
                {
                    // TODO: Should we instead do this in `add_to_tree_on`?
                    let collision_margin = collision_margin.map_or(0.0, |m| m.0);
                    *aabb = collider
                        .aabb_with_context(pos.0, *rot, aabb_context)
                        .grow(Vector::splat(contact_tolerance + collision_margin));
                    enlarged_aabb.update(&aabb, 0.0);
                }
            },
        );

        // Aside from AABB updates, we need to handle the following cases:
        //
        // 1. On insert `C` or `ColliderOf`, add to new tree if not already present. Remove from old tree if present.
        // 2. On remove `C`, remove from tree.
        // 3. On remove `ColliderOf`, move to standalone tree if `C` still exists.
        // 4. On re-enable `C`, add to tree.
        // 5. On disable `C`, remove from tree.
        // 6. On replace `RigidBody`, move attached colliders to new tree.
        // 7. On add `Sensor`, set sensor proxy flag.
        // 8. On remove `Sensor`, unset sensor proxy flag.
        // 9. On replace `CollisionLayers`, update proxy layers.
        // 10. On replace `ActiveCollisionHooks`, set proxy flag.
        // 11. On replace `RigidBodyDisabled`, set/unset proxy flag.

        // Case 1
        app.add_observer(add_to_tree_on::<Insert, (C, ColliderOf), Without<ColliderDisabled>>);

        // Case 2
        // Note: We also include disabled entities here for the edge case where
        //       we despawn a disabled collider, which causes Case 4 to trigger first.
        //       Ideally Case 4 would not trigger for despawned entities.
        // TODO: Clean up the edge case described above.
        app.add_observer(remove_from_tree_on::<Remove, C, Allow<Disabled>>);

        // Case 3
        app.add_observer(
            |trigger: On<Remove, ColliderOf>,
             mut collider_query: Query<
                (
                    &ColliderTreeProxyKey,
                    &ColliderAabb,
                    &EnlargedAabb,
                    Option<&CollisionLayers>,
                    Has<Sensor>,
                    Has<CollisionEventsEnabled>,
                    Option<&ActiveCollisionHooks>,
                ),
                (With<C>, Without<ColliderDisabled>),
            >,
             mut trees: ResMut<ColliderTrees>,
             mut moved_proxies: ResMut<MovedProxies>| {
                let entity = trigger.entity;

                let Ok((
                    proxy_key,
                    collider_aabb,
                    enlarged_aabb,
                    layers,
                    is_sensor,
                    has_contact_events,
                    active_hooks,
                )) = collider_query.get_mut(entity)
                else {
                    return;
                };

                // Remove the proxy from its current tree.
                let tree = trees.tree_for_type_mut(proxy_key.tree_type());
                if tree.remove_proxy(proxy_key.id()).is_none() {
                    return;
                }
                moved_proxies.remove(proxy_key);

                // If the collider still exists, move it to the standalone tree.
                let aabb = Aabb::from(*collider_aabb);
                let enlarged_aabb = Aabb::from(enlarged_aabb.get());

                let proxy = ColliderTreeProxy {
                    collider: entity,
                    body: None,
                    layers: layers.copied().unwrap_or_default(),
                    aabb,
                    flags: ColliderTreeProxyFlags::new(
                        is_sensor,
                        false,
                        has_contact_events,
                        active_hooks.copied().unwrap_or_default(),
                    ),
                };

                let standalone_tree = &mut trees.standalone_tree;
                let proxy_id = standalone_tree.add_proxy(enlarged_aabb, proxy);
                let new_proxy_key =
                    ColliderTreeProxyKey::new(proxy_id, ColliderTreeType::Standalone);

                // Mark the proxy as moved.
                moved_proxies.insert(new_proxy_key);
            },
        );

        // Cases 4
        // Note: We use `Replace` here to run before Case 2.
        app.add_observer(
            add_to_tree_on::<Replace, Disabled, (Without<ColliderDisabled>, Allow<Disabled>)>,
        );
        app.add_observer(add_to_tree_on::<Replace, ColliderDisabled, ()>);

        // Case 5
        app.add_observer(
            remove_from_tree_on::<Add, Disabled, (Without<ColliderDisabled>, Allow<Disabled>)>,
        );
        app.add_observer(remove_from_tree_on::<Add, ColliderDisabled, ()>);

        // Case 6
        app.add_observer(
            |trigger: On<Insert, RigidBody>,
             body_query: Query<(&RigidBody, &RigidBodyColliders, Has<RigidBodyDisabled>)>,
             mut collider_query: Query<
                (
                    &ColliderAabb,
                    &EnlargedAabb,
                    &mut ColliderTreeProxyKey,
                    Option<&CollisionLayers>,
                    Has<Sensor>,
                    Has<CollisionEventsEnabled>,
                    Option<&ActiveCollisionHooks>,
                ),
                Without<ColliderDisabled>,
            >,
             mut trees: ResMut<ColliderTrees>,
             mut moved_proxies: ResMut<MovedProxies>| {
                let entity = trigger.entity;

                let Ok((new_rb, body_colliders, is_body_disabled)) = body_query.get(entity) else {
                    return;
                };

                for collider_entity in body_colliders.iter() {
                    let Ok((
                        collider_aabb,
                        enlarged_aabb,
                        mut proxy_key,
                        layers,
                        is_sensor,
                        has_contact_events,
                        active_hooks,
                    )) = collider_query.get_mut(collider_entity)
                    else {
                        continue;
                    };

                    let new_tree_type = ColliderTreeType::from_body(Some(*new_rb));

                    if new_tree_type == proxy_key.tree_type() {
                        // No tree change.
                        break;
                    }

                    // Remove the old proxy from its current tree.
                    let old_tree = trees.tree_for_type_mut(proxy_key.tree_type());
                    old_tree.remove_proxy(proxy_key.id());
                    moved_proxies.remove(&proxy_key);

                    // Insert the proxy into the new tree.
                    let aabb = Aabb::from(*collider_aabb);
                    let enlarged_aabb = Aabb::from(enlarged_aabb.get());

                    let proxy = ColliderTreeProxy {
                        collider: collider_entity,
                        body: Some(entity),
                        layers: layers.copied().unwrap_or_default(),
                        aabb,
                        flags: ColliderTreeProxyFlags::new(
                            is_sensor,
                            is_body_disabled,
                            has_contact_events,
                            active_hooks.copied().unwrap_or_default(),
                        ),
                    };

                    let new_tree = trees.tree_for_type_mut(new_tree_type);
                    let proxy_id = new_tree.add_proxy(enlarged_aabb, proxy);
                    let new_proxy_key = ColliderTreeProxyKey::new(proxy_id, new_tree_type);

                    // Store the new proxy key.
                    *proxy_key = new_proxy_key;

                    // Mark the proxy as moved.
                    moved_proxies.insert(new_proxy_key);
                }
            },
        );

        // Case 7
        app.add_observer(
            |trigger: On<Add, Sensor>,
             mut collider_query: Query<&ColliderTreeProxyKey, Without<ColliderDisabled>>,
             mut trees: ResMut<ColliderTrees>| {
                let entity = trigger.entity;

                let Ok(proxy_key) = collider_query.get_mut(entity) else {
                    return;
                };

                let tree = trees.tree_for_type_mut(proxy_key.tree_type());

                // Set sensor flag.
                if let Some(proxy) = tree.get_proxy_mut(proxy_key.id()) {
                    proxy.flags.insert(ColliderTreeProxyFlags::SENSOR);
                }
            },
        );

        // Case 8
        app.add_observer(
            |trigger: On<Remove, Sensor>,
             mut collider_query: Query<&ColliderTreeProxyKey, Without<ColliderDisabled>>,
             mut trees: ResMut<ColliderTrees>| {
                let entity = trigger.entity;

                let Ok(proxy_key) = collider_query.get_mut(entity) else {
                    return;
                };

                let tree = trees.tree_for_type_mut(proxy_key.tree_type());

                // Unset sensor flag.
                if let Some(proxy) = tree.get_proxy_mut(proxy_key.id()) {
                    proxy.flags.remove(ColliderTreeProxyFlags::SENSOR);
                }
            },
        );

        // Case 9
        app.add_observer(
            |trigger: On<Replace, CollisionLayers>,
             mut collider_query: Query<
                (&ColliderTreeProxyKey, Option<&CollisionLayers>),
                Without<ColliderDisabled>,
            >,
             mut trees: ResMut<ColliderTrees>| {
                let entity = trigger.entity;

                let Ok((proxy_key, layers)) = collider_query.get_mut(entity) else {
                    return;
                };

                let tree = trees.tree_for_type_mut(proxy_key.tree_type());

                // Update layers.
                if let Some(proxy) = tree.get_proxy_mut(proxy_key.id()) {
                    proxy.layers = layers.copied().unwrap_or_default();
                }
            },
        );

        // Case 10
        app.add_observer(
            |trigger: On<Replace, ActiveCollisionHooks>,
             mut collider_query: Query<
                (&ColliderTreeProxyKey, Option<&ActiveCollisionHooks>),
                Without<ColliderDisabled>,
            >,
             mut trees: ResMut<ColliderTrees>| {
                let entity = trigger.entity;

                let Ok((proxy_key, active_hooks)) = collider_query.get_mut(entity) else {
                    return;
                };

                let tree = trees.tree_for_type_mut(proxy_key.tree_type());

                // Update active hooks flags.
                if let Some(proxy) = tree.get_proxy_mut(proxy_key.id()) {
                    proxy.flags.set(
                        ColliderTreeProxyFlags::CUSTOM_FILTER,
                        active_hooks
                            .is_some_and(|h| h.contains(ActiveCollisionHooks::FILTER_PAIRS)),
                    );
                }
            },
        );

        // Case 11
        app.add_observer(
            |trigger: On<Replace, RigidBodyDisabled>,
             body_query: Query<(&RigidBodyColliders, Has<RigidBodyDisabled>)>,
             mut collider_query: Query<&ColliderTreeProxyKey, Without<ColliderDisabled>>,
             mut trees: ResMut<ColliderTrees>| {
                let entity = trigger.entity;

                let Ok((body_colliders, is_body_disabled)) = body_query.get(entity) else {
                    return;
                };

                for collider_entity in body_colliders.iter() {
                    let Ok(proxy_key) = collider_query.get_mut(collider_entity) else {
                        continue;
                    };

                    let tree = trees.tree_for_type_mut(proxy_key.tree_type());

                    // Update body disabled flag.
                    if let Some(proxy) = tree.get_proxy_mut(proxy_key.id()) {
                        proxy
                            .flags
                            .set(ColliderTreeProxyFlags::BODY_DISABLED, is_body_disabled);
                    }
                }
            },
        );
    }
}

/// Adds a collider to the appropriate collider tree when the event `E` is triggered.
fn add_to_tree_on<E: EntityEvent, B: Bundle, F: QueryFilter>(
    trigger: On<E, B>,
    body_query: Query<(&RigidBody, Has<RigidBodyDisabled>), Allow<Disabled>>,
    mut collider_query: Query<
        (
            Option<&ColliderOf>,
            &ColliderAabb,
            &EnlargedAabb,
            &mut ColliderTreeProxyKey,
            Option<&CollisionLayers>,
            Has<Sensor>,
            Has<CollisionEventsEnabled>,
            Option<&ActiveCollisionHooks>,
        ),
        F,
    >,
    mut trees: ResMut<ColliderTrees>,
    mut moved_proxies: ResMut<MovedProxies>,
) {
    let entity = trigger.event_target();

    let Ok((
        collider_of,
        collider_aabb,
        enlarged_aabb,
        mut proxy_key,
        layers,
        is_sensor,
        has_contact_events,
        active_hooks,
    )) = collider_query.get_mut(entity)
    else {
        return;
    };

    let (tree_type, is_body_disabled) =
        if let Some(Ok((rb, disabled))) = collider_of.map(|c| body_query.get(c.body)) {
            (ColliderTreeType::from_body(Some(*rb)), disabled)
        } else {
            (ColliderTreeType::Standalone, false)
        };

    let aabb = Aabb::from(*collider_aabb);
    let enlarged_aabb = Aabb::from(enlarged_aabb.get());

    let proxy = ColliderTreeProxy {
        collider: entity,
        body: collider_of.map(|c| c.body),
        layers: layers.copied().unwrap_or_default(),
        aabb,
        flags: ColliderTreeProxyFlags::new(
            is_sensor,
            is_body_disabled,
            has_contact_events,
            active_hooks.copied().unwrap_or_default(),
        ),
    };

    // Remove the old proxy if it exists.
    if *proxy_key != ColliderTreeProxyKey::PLACEHOLDER {
        let old_tree_type = proxy_key.tree_type();
        let old_tree = trees.tree_for_type_mut(old_tree_type);
        old_tree.remove_proxy(proxy_key.id());
        moved_proxies.remove(&proxy_key);
    }

    // Insert the proxy into the appropriate tree.
    let tree = trees.tree_for_type_mut(tree_type);
    let proxy_id = tree.add_proxy(enlarged_aabb, proxy);

    // Store the proxy key.
    *proxy_key = ColliderTreeProxyKey::new(proxy_id, tree_type);

    // Mark the proxy as moved.
    moved_proxies.insert(*proxy_key);
}

/// Removes a collider from its collider tree when the event `E` is triggered.
fn remove_from_tree_on<E: EntityEvent, B: Bundle, F: QueryFilter>(
    trigger: On<E, B>,
    mut collider_query: Query<&mut ColliderTreeProxyKey, F>,
    mut trees: ResMut<ColliderTrees>,
    mut moved_proxies: ResMut<MovedProxies>,
) {
    let entity = trigger.event_target();

    let Ok(mut proxy_key) = collider_query.get_mut(entity) else {
        return;
    };

    if *proxy_key == ColliderTreeProxyKey::PLACEHOLDER {
        return;
    }

    // Remove the proxy from its current tree.
    let tree = trees.tree_for_type_mut(proxy_key.tree_type());
    tree.remove_proxy(proxy_key.id());
    moved_proxies.remove(&proxy_key);

    // Invalidate the proxy key.
    *proxy_key = ColliderTreeProxyKey::PLACEHOLDER;
}

/// A resource for tracking the last system change tick
/// when dynamic or kinematic collider AABBs were updated.
#[derive(Resource, Default)]
struct LastDynamicKinematicAabbUpdate(Tick);

/// A resource for tracking moved proxies.
///
/// Moved proxies are those whose [`ColliderAabb`] has moved outside of their
/// previous [`EnlargedAabb`], or whose collider has been added to a [`ColliderTree`].
///
/// [`ColliderTree`]: crate::collider_tree::ColliderTree
#[derive(Resource, Default)]
pub struct MovedProxies {
    /// A vector of moved proxy keys.
    proxies: Vec<ColliderTreeProxyKey>,
    /// A set of moved proxy keys for quick lookup.
    set: HashSet<ColliderTreeProxyKey>,
}

impl MovedProxies {
    /// Returns the keys of the moved proxies.
    ///
    /// The order of the keys is the order in which they were inserted.
    #[inline]
    pub fn proxies(&self) -> &[ColliderTreeProxyKey] {
        &self.proxies
    }

    /// Returns `true` if the proxy with the given key has moved.
    #[inline]
    pub fn contains(&self, proxy_key: ColliderTreeProxyKey) -> bool {
        self.set.contains(&proxy_key)
    }

    /// Inserts a moved proxy key.
    ///
    /// Returns `true` if the proxy key was not already present.
    #[inline]
    pub fn insert(&mut self, proxy_key: ColliderTreeProxyKey) -> bool {
        if self.set.insert(proxy_key) {
            self.proxies.push(proxy_key);
            true
        } else {
            false
        }
    }

    /// Removes a moved proxy key. This uses a linear search,
    /// and may change the order of the remaining keys.
    ///
    /// If the proxy key is not present, nothing happens.
    #[inline]
    pub fn remove(&mut self, proxy_key: &ColliderTreeProxyKey) {
        if self.set.remove(proxy_key)
            && let Some(pos) = self.proxies.iter().position(|k| k == proxy_key)
        {
            self.proxies.swap_remove(pos);
        }
    }

    /// Clears the moved proxies.
    #[inline]
    pub fn clear(&mut self) {
        self.proxies.clear();
        self.set.clear();
    }
}

/// Bit vectors for tracking dynamic and kinematic proxies whose
/// [`ColliderAabb`] has moved outside of the previous [`EnlargedAabb`].
///
/// Set bits indicate [`ProxyId`]s of moved proxies.
#[derive(Resource, Default)]
pub struct EnlargedProxies {
    // Note: Box2D indexes by shape ID, so it only needs one bit vector.
    //       In our case, we would instead index by entity ID, but this would
    //       require a potentially huge and very sparse bit vector since not
    //       all entities are colliders. So we use separate bit vectors for
    //       different proxy types, and index by proxy ID instead.
    dynamic_proxies: EnlargedProxiesBitVec,
    kinematic_proxies: EnlargedProxiesBitVec,
    static_proxies: EnlargedProxiesBitVec,
    standalone_proxies: EnlargedProxiesBitVec,
}

impl EnlargedProxies {
    /// Returns the bit vector for the given [`ColliderTreeType`].
    #[inline]
    pub const fn bit_vec_for_type(&self, tree_type: ColliderTreeType) -> &EnlargedProxiesBitVec {
        match tree_type {
            ColliderTreeType::Dynamic => &self.dynamic_proxies,
            ColliderTreeType::Kinematic => &self.kinematic_proxies,
            ColliderTreeType::Static => &self.static_proxies,
            ColliderTreeType::Standalone => &self.standalone_proxies,
        }
    }

    /// Returns a mutable reference to the bit vector for the given [`ColliderTreeType`].
    #[inline]
    pub fn bit_vec_for_type_mut(
        &mut self,
        tree_type: ColliderTreeType,
    ) -> &mut EnlargedProxiesBitVec {
        match tree_type {
            ColliderTreeType::Dynamic => &mut self.dynamic_proxies,
            ColliderTreeType::Kinematic => &mut self.kinematic_proxies,
            ColliderTreeType::Static => &mut self.static_proxies,
            ColliderTreeType::Standalone => &mut self.standalone_proxies,
        }
    }
}

/// Bit vectors for tracking proxies whose [`ColliderAabb`] has moved outside of the previous [`EnlargedAabb`].
///
/// Set bits indicate [`ProxyId`]s of moved proxies.
///
/// [`ProxyId`]: crate::collider_tree::ProxyId
// TODO: We have a few of these now. We should maybe abstract this into a reusable structure.
#[derive(Default)]
pub struct EnlargedProxiesBitVec {
    global: BitVec,
    thread_local: ThreadLocal<RefCell<BitVec>>,
}

impl EnlargedProxiesBitVec {
    /// Clears the enlarged proxies and sets the capacity of the internal structures.
    #[inline]
    pub fn clear_and_set_capacity(&mut self, capacity: usize) {
        self.global.set_bit_count_and_clear(capacity);
        self.thread_local.iter_mut().for_each(|context| {
            let bit_vec_mut = &mut context.borrow_mut();
            bit_vec_mut.set_bit_count_and_clear(capacity);
        });
    }

    /// Combines the thread-local enlarged proxy bit vectors into the global one.
    #[inline]
    pub fn combine_thread_local(&mut self) {
        self.thread_local.iter_mut().for_each(|context| {
            let thread_local = context.borrow();
            self.global.or(&thread_local);
        });
    }
}

/// Updates the AABBs of the colliders of each [`SolverBody`] (awake dynamic and kinematic bodies)
/// after the physics step.
// TODO: Once dynamic an kinematic bodies have their own marker components,
//       we should use those instead of `SolverBody`. Solver bodies should
//       be an implementation detail of the solver.
// TODO: This approach with velocity-expanded AABBs is quite inefficient.
//       We could switch to Box2D-style CCD with fast bodies.
fn update_solver_body_aabbs<C: AnyCollider>(
    body_query: Query<
        (
            &Position,
            &ComputedCenterOfMass,
            &LinearVelocity,
            &AngularVelocity,
            &RigidBodyColliders,
            Has<SweptCcd>,
        ),
        With<SolverBody>,
    >,
    mut colliders: ParamSet<(
        Query<
            (
                Ref<C>,
                &mut ColliderAabb,
                &mut EnlargedAabb,
                &ColliderTreeProxyKey,
                &Position,
                &Rotation,
                Option<&CollisionMargin>,
                Option<&SpeculativeMargin>,
            ),
            Without<ColliderDisabled>,
        >,
        Query<(&ColliderAabb, &EnlargedAabb), Without<ColliderDisabled>>,
    )>,
    narrow_phase_config: Res<NarrowPhaseConfig>,
    length_unit: Res<PhysicsLengthUnit>,
    mut trees: ResMut<ColliderTrees>,
    mut moved_proxies: ResMut<MovedProxies>,
    mut enlarged_proxies: ResMut<EnlargedProxies>,
    time: Res<Time>,
    collider_context: StaticSystemParam<C::Context>,
    mut diagnostics: ResMut<ColliderTreeDiagnostics>,
    mut last_tick: ResMut<LastDynamicKinematicAabbUpdate>,
    system_tick: SystemChangeTick,
) {
    let start = crate::utils::Instant::now();

    let this_run = system_tick.this_run();

    // An upper bound on the number of proxies, for sizing the bit vectors.
    // TODO: Use a better way to track the number of proxies.
    let cap_dynamic = trees.dynamic_tree.proxies.capacity();
    let cap_kinematic = trees.kinematic_tree.proxies.capacity();

    // Clear and resize the enlarged proxy structures.
    let e = &mut enlarged_proxies;
    e.dynamic_proxies.clear_and_set_capacity(cap_dynamic);
    e.kinematic_proxies.clear_and_set_capacity(cap_kinematic);

    let delta_secs = time.delta_seconds_adjusted();
    let default_speculative_margin = length_unit.0 * narrow_phase_config.default_speculative_margin;
    let contact_tolerance = length_unit.0 * narrow_phase_config.contact_tolerance;
    let margin = length_unit.0 * 0.05;

    let collider_query = colliders.p0();

    body_query.par_iter().for_each(
        |(rb_pos, center_of_mass, lin_vel, ang_vel, body_colliders, has_swept_ccd)| {
            for collider_entity in body_colliders.iter() {
                let Ok((
                    collider,
                    mut aabb,
                    mut enlarged_aabb,
                    proxy_key,
                    pos,
                    rot,
                    collision_margin,
                    speculative_margin,
                )) = (unsafe { collider_query.get_unchecked(collider_entity) })
                else {
                    continue;
                };

                let collision_margin = collision_margin.map_or(0.0, |margin| margin.0);
                let speculative_margin = if has_swept_ccd {
                    Scalar::MAX
                } else {
                    speculative_margin.map_or(default_speculative_margin, |margin| margin.0)
                };

                let context = AabbContext::new(collider_entity, &*collider_context);

                if speculative_margin <= 0.0 {
                    *aabb = collider
                        .aabb_with_context(pos.0, *rot, context)
                        .grow(Vector::splat(contact_tolerance + collision_margin));
                } else {
                    // If the rigid body is rotating, off-center colliders will orbit around it,
                    // which affects their linear velocities. We need to compute the linear velocity
                    // at the offset position.
                    // TODO: This assumes that the colliders would continue moving in the same direction,
                    //       but because they are orbiting, the direction will change. We should take
                    //       into account the uniform circular motion.
                    let offset = pos.0 - rb_pos.0 - center_of_mass.0;
                    #[cfg(feature = "2d")]
                    let vel = lin_vel.0 + Vector::new(-ang_vel.0 * offset.y, ang_vel.0 * offset.x);
                    #[cfg(feature = "3d")]
                    let vel = lin_vel.0 + ang_vel.cross(offset);
                    let movement = (vel * delta_secs)
                        .clamp_length_max(speculative_margin.max(contact_tolerance));

                    // Current position and predicted position for next feame
                    #[cfg(feature = "2d")]
                    let (end_pos, end_rot) = (
                        pos.0 + movement,
                        *rot * Rotation::radians(ang_vel.0 * delta_secs),
                    );

                    #[cfg(feature = "3d")]
                    let (end_pos, end_rot) = (
                        pos.0 + movement,
                        Rotation(Quaternion::from_scaled_axis(ang_vel.0 * delta_secs) * rot.0)
                            .fast_renormalize(),
                    );

                    // Compute swept AABB, the space that the body would occupy if it was integrated for one frame
                    // TODO: Should we expand the AABB in all directions for speculative contacts?
                    *aabb = collider
                        .swept_aabb_with_context(pos.0, *rot, end_pos, end_rot, context)
                        .grow(Vector::splat(collision_margin));
                }

                let moved = enlarged_aabb.update(&aabb, margin);

                if moved {
                    let tree_type = proxy_key.tree_type();
                    let mut thread_local_bit_vec = enlarged_proxies
                        .bit_vec_for_type(tree_type)
                        .thread_local
                        .get_or(|| {
                            let capacity = match tree_type {
                                ColliderTreeType::Dynamic => cap_dynamic,
                                ColliderTreeType::Kinematic => cap_kinematic,
                                _ => unreachable!("Static or standalone proxy {proxy_key:?} moved in dynamic AABB update"),
                            };
                            let mut bit_vec = BitVec::new(capacity);
                            bit_vec.set_bit_count_and_clear(capacity);
                            RefCell::new(bit_vec)
                        })
                        .borrow_mut();

                    thread_local_bit_vec.set(proxy_key.id().index());
                }
            }
        },
    );

    // Update the AABBs of moved proxies in the dynamic and kinematic trees.
    let aabb_query = colliders.p1();
    for &tree_type in &[ColliderTreeType::Dynamic, ColliderTreeType::Kinematic] {
        let tree = trees.tree_for_type_mut(tree_type);
        let bit_vec = enlarged_proxies.bit_vec_for_type_mut(tree_type);

        tree.bvh.init_primitives_to_nodes_if_uninit();
        bit_vec.combine_thread_local();

        update_tree(
            tree_type,
            tree,
            &bit_vec.global,
            &aabb_query,
            &mut moved_proxies,
            |tree, proxy_id, enlarged_aabb| {
                tree.set_proxy_aabb(proxy_id, enlarged_aabb);
            },
        );

        // Refit the BVH after enlarging proxies.
        // TODO: For a smaller number of moved proxies, it can be faster
        //       to only refit upwards from the moved leaves.
        tree.refit_all();
    }

    // Update the last update tick.
    // TODO: Remove this
    last_tick.0 = this_run;

    diagnostics.update += start.elapsed();
}

/// Updates the AABBs of colliders that have been manually moved after the previous physics step.
pub fn update_moved_collider_aabbs<C: AnyCollider>(
    mut colliders: ParamSet<(
        Query<
            (
                Entity,
                Ref<Position>,
                Ref<Rotation>,
                &mut ColliderAabb,
                &mut EnlargedAabb,
                Ref<C>,
                Option<&CollisionMargin>,
                &ColliderTreeProxyKey,
            ),
            Without<ColliderDisabled>,
        >,
        Query<(&ColliderAabb, &EnlargedAabb), Without<ColliderDisabled>>,
    )>,
    narrow_phase_config: Res<NarrowPhaseConfig>,
    length_unit: Res<PhysicsLengthUnit>,
    mut trees: ResMut<ColliderTrees>,
    mut moved_proxies: ResMut<MovedProxies>,
    mut enlarged_proxies: ResMut<EnlargedProxies>,
    collider_context: StaticSystemParam<C::Context>,
    mut diagnostics: ResMut<ColliderTreeDiagnostics>,
    last_tick: Res<LastPhysicsTick>,
    system_tick: SystemChangeTick,
) {
    let start = crate::utils::Instant::now();

    let this_run = system_tick.this_run();

    // An upper bound on the number of proxies, for sizing the bit vectors.
    let cap_dynamic = trees.dynamic_tree.proxies.capacity();
    let cap_kinematic = trees.kinematic_tree.proxies.capacity();
    let cap_static = trees.static_tree.proxies.capacity();
    let cap_standalone = trees.standalone_tree.proxies.capacity();

    // Clear and resize the enlarged proxy structures.
    let e = &mut enlarged_proxies;
    e.dynamic_proxies.clear_and_set_capacity(cap_dynamic);
    e.kinematic_proxies.clear_and_set_capacity(cap_kinematic);
    e.static_proxies.clear_and_set_capacity(cap_static);
    e.standalone_proxies.clear_and_set_capacity(cap_standalone);

    let contact_tolerance = length_unit.0 * narrow_phase_config.contact_tolerance;

    // TODO: This doesn't do velocity-based enlargement like the dynamic/kinematic AABB update.
    //       We should overall rework CCD to not rely on velocity-based AABB enlargement for all bodies.
    // TODO: par-iter over all colliders, check if they have actually changed since the `LastPhysicsTick`
    let mut collider_query = colliders.p0();
    collider_query.par_iter_mut().for_each(
        |(entity, pos, rot, mut aabb, mut enlarged_aabb, collider, margin, proxy_key)| {
            // Skip if the collider's AABB can't have changed since the last physics tick.
            if !pos.last_changed().is_newer_than(last_tick.0, this_run)
                && !rot.last_changed().is_newer_than(last_tick.0, this_run)
                && !collider.last_changed().is_newer_than(last_tick.0, this_run)
            {
                return;
            }

            let margin = margin.map_or(0.0, |margin| margin.0);

            let context = AabbContext::new(entity, &*collider_context);

            // Compute the AABB of the collider.
            *aabb = collider
                .aabb_with_context(pos.0, *rot, context)
                .grow(Vector::splat(contact_tolerance + margin));
            let moved = enlarged_aabb.update(&aabb, 1e-4);

            if moved {
                let tree_type = proxy_key.tree_type();
                let mut thread_local_bit_vec = enlarged_proxies
                    .bit_vec_for_type(tree_type)
                    .thread_local
                    .get_or(|| {
                        let capacity = match tree_type {
                            ColliderTreeType::Dynamic => cap_dynamic,
                            ColliderTreeType::Kinematic => cap_kinematic,
                            ColliderTreeType::Static => cap_static,
                            ColliderTreeType::Standalone => cap_standalone,
                        };
                        let mut bit_vec = BitVec::new(capacity);
                        bit_vec.set_bit_count_and_clear(capacity);
                        RefCell::new(bit_vec)
                    })
                    .borrow_mut();

                thread_local_bit_vec.set(proxy_key.id().index());
            }
        },
    );

    // Reinsert moved proxies in each tree.
    let aabb_query = colliders.p1();
    for tree_type in ColliderTreeType::ALL {
        let tree = trees.tree_for_type_mut(tree_type);
        let bit_vec = enlarged_proxies.bit_vec_for_type_mut(tree_type);

        tree.bvh.init_primitives_to_nodes_if_uninit();
        bit_vec.combine_thread_local();

        let moved_count = bit_vec.global.count_ones();
        let moved_ratio = if tree.proxies.is_empty() {
            0.0
        } else {
            moved_count as f32 / tree.proxies.len() as f32
        };

        // For a small number of moved proxies, it's more efficient to refit up from just those leaves.
        // Otherwise, it's better to refit the entire tree once after updating all moved proxies.
        // TODO: Tune the threshold ratio.
        if moved_ratio < 0.1 {
            update_tree(
                tree_type,
                tree,
                &bit_vec.global,
                &aabb_query,
                &mut moved_proxies,
                |tree, proxy_id, enlarged_aabb| {
                    tree.resize_proxy_aabb(proxy_id, enlarged_aabb);
                },
            );
        } else {
            update_tree(
                tree_type,
                tree,
                &bit_vec.global,
                &aabb_query,
                &mut moved_proxies,
                |tree, proxy_id, enlarged_aabb| {
                    tree.set_proxy_aabb(proxy_id, enlarged_aabb);
                },
            );
            tree.refit_all();
        }
    }

    diagnostics.update += start.elapsed();
}

/// Updates the collider tree for the moved proxies indicated in the given bit vector.
fn update_tree(
    tree_type: ColliderTreeType,
    tree: &mut ColliderTree,
    bit_vec: &BitVec,
    aabbs: &Query<(&ColliderAabb, &EnlargedAabb), Without<ColliderDisabled>>,
    moved_proxies: &mut MovedProxies,
    update_proxy_fn: impl Fn(&mut ColliderTree, ProxyId, Aabb),
) {
    for (i, mut bits) in bit_vec.blocks().enumerate() {
        while bits != 0 {
            let trailing_zeros = bits.trailing_zeros();
            let proxy_id = ProxyId::new(i as u32 * 64 + trailing_zeros);
            let proxy = &mut tree.proxies[proxy_id.index()];
            let entity = proxy.collider;

            // TODO: Get rid of the query here. We should be able to mutably write the AABBs directly
            //       in the parallel loop above and avoid this extra query.
            let (aabb, enlarged_aabb) = aabbs.get(entity).unwrap_or_else(|_| {
                panic!(
                    "EnlargedAabb missing for moved collider entity {:?}",
                    entity
                )
            });

            let aabb = Aabb::from(*aabb);
            let enlarged_aabb = Aabb::from(enlarged_aabb.get());

            // Update the proxy's AABB.
            proxy.aabb = aabb;
            update_proxy_fn(tree, proxy_id, enlarged_aabb);

            // Record the moved proxy.
            let proxy_key = ColliderTreeProxyKey::new(proxy_id, tree_type);
            if moved_proxies.insert(proxy_key) {
                tree.moved_proxies.push(proxy_id);
            }

            // Clear the least significant set bit
            bits &= bits - 1;
        }
    }
}

fn clear_moved_proxies(mut moved_proxies: ResMut<MovedProxies>, mut trees: ResMut<ColliderTrees>) {
    moved_proxies.clear();
    trees.iter_trees_mut().for_each(|t| t.moved_proxies.clear());
}
