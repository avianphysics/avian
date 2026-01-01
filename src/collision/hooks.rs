//! Collision hooks for filtering and modifying contacts.
//!
//! See the [`CollisionHooks`] trait for more information.

use crate::prelude::*;
use bevy::{ecs::system::ReadOnlySystemParam, prelude::*};

/// A trait for user-defined hooks that can filter and modify contacts.
///
/// This can be useful for advanced contact scenarios, such as:
///
/// - One-way platforms
/// - Conveyor belts
/// - Non-uniform friction and restitution
///
/// Collision hooks are more flexible than built-in filtering options like [`CollisionLayers`],
/// but can be more complicated to define, and can have slightly more overhead.
/// It is recommended to use hooks only when existing options are not sufficient.
///
/// Only one set of collision hooks can be defined per broad phase and narrow phase.
///
/// # Defining Hooks
///
/// Collision hooks can be defined by implementing the [`CollisionHooks`] trait for a type
/// that implements [`SystemParam`]. The system parameter allows the hooks to do things like
/// access resources, query for components, and perform [spatial queries](crate::spatial_query).
///
/// Note that mutable access is not allowed for the system parameter, as hooks may be called
/// during parallel iteration. However, access to [`Commands`] is provided for deferred changes.
///
/// Below is an example of using collision hooks to implement interaction groups and one-way platforms:
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::{ecs::system::SystemParam, prelude::*};
///
/// /// A component that groups entities for interactions. Only entities in the same group can collide.
/// #[derive(Component)]
/// struct InteractionGroup(u32);
///
/// /// A component that marks an entity as a one-way platform.
/// #[derive(Component)]
/// struct OneWayPlatform;
///
/// // Define a `SystemParam` for the collision hooks.
/// #[derive(SystemParam)]
/// struct MyHooks<'w, 's> {
///     interaction_query: Query<'w, 's, &'static InteractionGroup>,
///     platform_query: Query<'w, 's, &'static Transform, With<OneWayPlatform>>,
/// }
///
/// // Implement the `CollisionHooks` trait.
/// impl CollisionHooks for MyHooks<'_, '_> {
///     fn filter_pairs(&self, collider1: Entity, collider2: Entity, _commands: &mut Commands) -> bool {
///         // Only allow collisions between entities in the same interaction group.
///         // This could be a basic solution for "multiple physics worlds" that don't interact.
///         let Ok([group1, group2]) = self.interaction_query.get_many([collider1, collider2]) else {
///            return true;
///         };
///         group1.0 == group2.0
///     }
///
///     fn modify_contacts(&self, contacts: &mut ContactPair, _commands: &mut Commands) -> bool {
///         // Allow entities to pass through the bottom and sides of one-way platforms.
///         // See the `one_way_platform_2d` example for a full implementation.
///         let (entity1, entity2) = (contacts.collider1, contacts.collider2);
///         !is_hitting_top_of_platform(entity1, entity2, &self.platform_query, &contacts)
///     }
/// }
/// #
/// # fn is_hitting_top_of_platform(
/// #     entity1: Entity,
/// #     entity2: Entity,
/// #     platform_query: &Query<&Transform, With<OneWayPlatform>>,
/// #     contacts: &ContactPair,
/// # ) -> bool {
/// #     todo!()
/// # }
/// ```
///
/// The hooks can then be added to the app using [`PhysicsPlugins::with_collision_hooks`]:
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::{ecs::system::SystemParam, prelude::*};
/// #
/// # #[derive(SystemParam)]
/// # struct MyHooks {}
/// #
/// # // No-op hooks for the example.
/// # impl CollisionHooks for MyHooks {}
/// #
/// fn main() {
///     App::new()
///         .add_plugins((
///             DefaultPlugins,
///             PhysicsPlugins::default().with_collision_hooks::<MyHooks>(),
///         ))
///         .run();
/// }
/// ```
///
/// This is equivalent to manually replacing the default [`BvhBroadPhasePlugin`] and [`NarrowPhasePlugin`]
/// with instances that have the desired hooks provided using generics.
///
/// [`SystemParam`]: bevy::ecs::system::SystemParam
///
/// # Activating Hooks
///
/// Hooks are *only* called for collisions where at least one entity has the [`ActiveCollisionHooks`] component
/// with the corresponding flags set. By default, no hooks are called.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Spawn a collider with filtering hooks enabled.
/// commands.spawn((Collider::capsule(0.5, 1.5), ActiveCollisionHooks::FILTER_PAIRS));
///
/// // Spawn a collider with both filtering and contact modification hooks enabled.
/// commands.spawn((
///     Collider::capsule(0.5, 1.5),
///     ActiveCollisionHooks::FILTER_PAIRS | ActiveCollisionHooks::MODIFY_CONTACTS
/// ));
///
/// // Alternatively, all hooks can be enabled with `ActiveCollisionHooks::all()`.
/// commands.spawn((Collider::capsule(0.5, 1.5), ActiveCollisionHooks::all()));
/// # }
/// ```
///
/// # Caveats
///
/// Collision hooks can access the ECS quite freely, but there are a few limitations:
///
/// - Only one set of collision hooks can be defined per broad phase and narrow phase.
/// - Only read-only ECS access is allowed for the hook system parameter. Use the provided [`Commands`] for deferred ECS operations.
///   - Note that command execution order is unspecified if the `parallel` feature is enabled.
/// - Access to the [`ContactGraph`] resource is not allowed inside [`CollisionHooks::filter_pairs`].
///   Trying to access it will result in a panic.
/// - Access to the [`ContactGraph`] resource is not allowed inside [`CollisionHooks::modify_contacts`].
///   Trying to access it will result in a panic.
#[expect(unused_variables)]
pub trait CollisionHooks: ReadOnlySystemParam + Send + Sync {
    /// A contact pair filtering hook that determines whether contacts should be computed
    /// between `collider1` and `collider2`. If `false` is returned, contacts will not be computed.
    ///
    /// This is called in the broad phase, before the [`ContactPair`] has been computed.
    ///
    /// The provided [`Commands`] can be used for deferred ECS operations that run after
    /// broad phase pairs have been found.
    ///
    /// # Notes
    ///
    /// - Only called if at least one entity in the contact pair has [`ActiveCollisionHooks::FILTER_PAIRS`] set.
    /// - Only called if at least one entity in the contact pair is not [`RigidBody::Static`] and not [`Sleeping`].
    /// - Access to the [`ContactGraph`] resource is not allowed in this method.
    ///   Trying to access it will result in a panic.
    fn filter_pairs(&self, collider1: Entity, collider2: Entity, commands: &mut Commands) -> bool {
        true
    }

    /// A contact modification hook that allows modifying the contacts for a given contact pair.
    /// If `false` is returned, the contact pair will be removed.
    ///
    /// This is called in the narrow phase, after the [`ContactPair`] has been computed for the pair,
    /// but before constraints have been generated for the contact solver.
    ///
    /// The provided [`Commands`] can be used for deferred ECS operations that run after
    /// the narrow phase has computed contact pairs and generated constraints.
    ///
    /// # Notes
    ///
    /// - Only called if at least one entity in the contact pair has [`ActiveCollisionHooks::MODIFY_CONTACTS`] set.
    /// - Only called if at least one entity in the contact pair is not [`RigidBody::Static`] and not [`Sleeping`].
    /// - Impulses stored in `contacts` are from the previous physics tick.
    /// - Command execution order is unspecified if the `parallel` feature is enabled.
    /// - Access to the [`ContactGraph`] resource is not allowed in this method.
    ///   Trying to access it will result in a panic.
    fn modify_contacts(&self, contacts: &mut ContactPair, commands: &mut Commands) -> bool {
        true
    }
}

// No-op implementation for `()` to allow default hooks for plugins.
impl CollisionHooks for () {}

/// A component with flags indicating which [`CollisionHooks`] should be called for collisions with an entity.
///
/// Hooks will only be called if either entity in a collision has the corresponding flags set.
///
/// Default: [`ActiveCollisionHooks::empty()`]
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// # fn setup(mut commands: Commands) {
/// // Spawn a collider with filtering hooks enabled.
/// commands.spawn((Collider::capsule(0.5, 1.5), ActiveCollisionHooks::FILTER_PAIRS));
///
/// // Spawn a collider with both filtering and contact modification hooks enabled.
/// commands.spawn((
///     Collider::capsule(0.5, 1.5),
///     ActiveCollisionHooks::FILTER_PAIRS | ActiveCollisionHooks::MODIFY_CONTACTS
/// ));
///
/// // Alternatively, all hooks can be enabled with `ActiveCollisionHooks::all()`.
/// commands.spawn((Collider::capsule(0.5, 1.5), ActiveCollisionHooks::all()));
/// # }
/// ```
#[repr(transparent)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[derive(Component, Hash, Clone, Copy, PartialEq, Eq, Debug, Reflect)]
#[reflect(opaque, Hash, PartialEq, Debug)]
pub struct ActiveCollisionHooks(u8);

bitflags::bitflags! {
    impl ActiveCollisionHooks: u8 {
        /// Set if [`CollisionHooks::filter_pairs`] should be called for collisions with this entity.
        const FILTER_PAIRS = 0b0000_0001;
        /// Set if [`CollisionHooks::modify_contacts`] should be called for collisions with this entity.
        const MODIFY_CONTACTS = 0b0000_0010;
    }
}
