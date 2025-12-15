//! Collision events for detecting when colliders start or stop touching.
//!
//! Avian provides two collision event types:
//!
//! - [`CollisionStart`]: Triggered when two colliders start touching.
//! - [`CollisionEnd`]: Triggered when two colliders stop touching.
//!
//! Depending on your use case, you may want to read them as [`Message`]s with a [`MessageReader`],
//! or observe them as [`Event`]s with an [observer](Observer). Avian supports both options.
//!
//! # Reading Collision Events
//!
//! To enable collision events for a collider entity, add the [`CollisionEventsEnabled`] component.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
//! use bevy::prelude::*;
//!
//! #[derive(Component)]
//! struct PressurePlate;
//!
//! fn setup_pressure_plates(mut commands: Commands) {
//!     commands.spawn((
//!         PressurePlate,
#![cfg_attr(feature = "2d", doc = "        Collider::rectangle(1.0, 1.0),")]
#![cfg_attr(feature = "3d", doc = "        Collider::cuboid(1.0, 0.1, 1.0),")]
//!         Sensor,
//!         // Enable collision events for this entity.
//!         CollisionEventsEnabled,
//!     ));
//! }
//! ```
//!
//! The [`CollisionStart`] and [`CollisionEnd`] events will now be both written as a [`Message`]
//! and triggered as an [`Event`] when the entity starts or stops touching another collider.
//! It is up to you to decide which event processing method is best suited for your use case.
//!
//! ## Collision `Message`
//!
//! Reading collision events as [`Message`]s with a [`MessageReader`] can be very efficient
//! for processing large numbers of collisions between pairs of entities, such as for detecting
//! bullet hits or playing impact sounds when two objects collide.
//!
//! The events are only written if one of the entities has the [`CollisionEventsEnabled`] component.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! fn print_started_collisions(mut collision_reader: MessageReader<CollisionStart>) {
//!     for event in collision_reader.read() {
//!         println!("{} and {} started colliding", event.collider1, event.collider2);
//!     }
//! }
//! ```
//!
//! ## Collision `Event`
//!
//! Observing collision events as [`Event`]s with an [observer](Observer) can be very useful
//! for entity-specific collision scenarios, such as for detecting when a player steps on
//! a pressure plate or enters a trigger volume.
//!
//! The events are only triggered if the target entity has the [`CollisionEventsEnabled`] component.
//!
//! ```
#![cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#![cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
//! # use bevy::prelude::*;
//! #
//! # #[derive(Component)]
//! # struct Player;
//! #
//! # #[derive(Component)]
//! # struct PressurePlate;
//! #
//! fn setup_pressure_plates(mut commands: Commands) {
//!     commands.spawn((
//!         PressurePlate,
#![cfg_attr(feature = "2d", doc = "        Collider::rectangle(1.0, 1.0),")]
#![cfg_attr(feature = "3d", doc = "        Collider::cuboid(1.0, 0.1, 1.0),")]
//!         Sensor,
//!         // Enable collision events for this entity.
//!         CollisionEventsEnabled,
//!     ))
//!     .observe(on_player_stepped_on_plate);
//! }
//!
//! fn on_player_stepped_on_plate(event: On<CollisionStart>, player_query: Query<&Player>) {
//!     // `colider1` and `body1` refer to the event target and its body.
//!     // `collider2` and `body2` refer to the other collider and its body.
//!     let pressure_plate = event.collider1;
//!     let other_entity = event.collider2;
//!
//!     if player_query.contains(other_entity) {
//!         println!("Player {other_entity} stepped on pressure plate {pressure_plate}");
//!     }
//! }
//! ```

use bevy::prelude::*;

/// A [collision event](self) that is triggered when two colliders start touching.
///
/// The event can be read using a [`MessageReader`] or observed using an [observer](Observer).
/// It is only triggered for entities with the [`CollisionEventsEnabled`] component.
///
/// # Example
///
/// Below is an example of observing the [`CollisionStart`] event using an [observer](Observer).
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// #[derive(Component)]
/// struct Player;
///
/// #[derive(Component)]
/// struct PressurePlate;
///
/// fn setup_pressure_plates(mut commands: Commands) {
///     commands.spawn((
///         PressurePlate,
#[cfg_attr(feature = "2d", doc = "        Collider::rectangle(1.0, 1.0),")]
#[cfg_attr(feature = "3d", doc = "        Collider::cuboid(1.0, 0.1, 1.0),")]
///         Sensor,
///         // Enable collision events for this entity.
///         CollisionEventsEnabled,
///     ))
///     .observe(on_player_stepped_on_plate);
/// }
///
/// fn on_player_stepped_on_plate(event: On<CollisionStart>, player_query: Query<&Player>) {
///     // `colider1` and `body1` refer to the event target and its body.
///     // `collider2` and `body2` refer to the other collider and its body.
///     let pressure_plate = event.collider1;
///     let other_entity = event.collider2;
///
///     if player_query.contains(other_entity) {
///         println!("Player {other_entity} stepped on pressure plate {pressure_plate}");
///     }
/// }
/// ```
///
/// The event can also be read as a [`Message`] using a [`MessageReader`].
/// This can be more efficient for processing large numbers of collisions.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// fn print_started_collisions(mut collision_reader: MessageReader<CollisionStart>) {
///     for event in collision_reader.read() {
///         println!("{} and {} started colliding", event.collider1, event.collider2);
///     }
/// }
/// ```
///
/// # Scheduling
///
/// The [`CollisionStart`] event is triggered after the physics step in the [`CollisionEventSystems`]
/// system set. At this point, the solver has already run and contact impulses have been updated.
///
/// [`CollisionEventSystems`]: super::narrow_phase::CollisionEventSystems
#[derive(EntityEvent, Message, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct CollisionStart {
    /// The entity of the collider that started colliding with [`collider2`](Self::collider2).
    ///
    /// For observers watching this event as an [`EntityEvent`], this is the target entity.
    #[event_target]
    pub collider1: Entity,
    /// The entity of the collider that started colliding with [`collider1`](Self::collider1).
    pub collider2: Entity,
    /// The rigid body that [`collider1`](Self::collider1) is attached to.
    ///
    /// If the collider is not attached to a rigid body, this will be `None`.
    pub body1: Option<Entity>,
    /// The rigid body that [`collider2`](Self::collider2) is attached to.
    ///
    /// If the collider is not attached to a rigid body, this will be `None`.
    pub body2: Option<Entity>,
    // TODO: Flags to expose the reason for the event, among other things.
    // flags: CollisionStartFlags,
}

/// A deprecated alias for [`CollisionStart`].
#[deprecated(since = "0.4.0", note = "Renamed to `CollisionStart`")]
pub type OnCollisionStart = CollisionStart;

/// A [collision event](self) that is triggered when two colliders stop touching.
///
/// The event can be read using a [`MessageReader`] or observed using an [observer](Observer).
/// It is only triggered for entities with the [`CollisionEventsEnabled`] component.
///
/// # Example
///
/// Below is an example of observing the [`CollisionEnd`] event using an [observer](Observer).
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// #[derive(Component)]
/// struct Player;
///
/// #[derive(Component)]
/// struct PressurePlate;
///
/// fn setup_pressure_plates(mut commands: Commands) {
///     commands.spawn((
///         PressurePlate,
#[cfg_attr(feature = "2d", doc = "        Collider::rectangle(1.0, 1.0),")]
#[cfg_attr(feature = "3d", doc = "        Collider::cuboid(1.0, 0.1, 1.0),")]
///         Sensor,
///         // Enable collision events for this entity.
///         CollisionEventsEnabled,
///     ))
///     .observe(on_player_stepped_off_plate);
/// }
///
/// fn on_player_stepped_off_plate(event: On<CollisionEnd>, player_query: Query<&Player>) {
///     // `colider1` and `body1` refer to the event target and its body.
///     // `collider2` and `body2` refer to the other collider and its body.
///     let pressure_plate = event.collider1;
///     let other_entity = event.collider2;
///
///     if player_query.contains(other_entity) {
///         println!("Player {other_entity} stepped off pressure plate {pressure_plate}");
///     }
/// }
/// ```
///
/// The event can also be read as a [`Message`] using a [`MessageReader`].
/// This can be more efficient for processing large numbers of collisions.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// fn print_ended_collisions(mut collision_reader: MessageReader<CollisionEnd>) {
///     for event in collision_reader.read() {
///         println!("{} and {} stopped colliding", event.collider1, event.collider2);
///     }
/// }
/// ```
///
/// # Scheduling
///
/// The [`CollisionEnd`] event is triggered after the physics step in the [`CollisionEventSystems`]
/// system set. At this point, the solver has already run and contact impulses have been updated.
///
/// Note that if one of the colliders was removed or the bounding boxes of the colliders stopped
/// overlapping, the [`ContactPair`] between the entities was also removed, and the contact data
/// will not be available through [`Collisions`].
///
/// [`CollisionEventSystems`]: super::narrow_phase::CollisionEventSystems
/// [`ContactPair`]: super::ContactPair
/// [`Collisions`]: super::Collisions
#[derive(EntityEvent, Message, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct CollisionEnd {
    /// The entity of the collider that stopped colliding with [`collider2`](Self::collider2).
    ///
    /// For observers watching this event as an [`EntityEvent`], this is the target entity.
    #[event_target]
    pub collider1: Entity,
    /// The entity of the collider that stopped colliding with [`collider1`](Self::collider1).
    pub collider2: Entity,
    /// The rigid body that [`collider1`](Self::collider1) is attached to.
    ///
    /// If the collider is not attached to a rigid body, this will be `None`.
    pub body1: Option<Entity>,
    /// The rigid body that [`collider2`](Self::collider2) is attached to.
    ///
    /// If the collider is not attached to a rigid body, this will be `None`.
    pub body2: Option<Entity>,
    // TODO: Flags to expose the reason for the event, among other things.
    // flags: CollisionEndFlags,
}

/// A deprecated alias for [`CollisionEnd`].
#[deprecated(since = "0.4.0", note = "Renamed to `CollisionEnd`")]
pub type OnCollisionEnd = CollisionEnd;

/// A marker component that enables [collision events](self) for an entity.
#[derive(Component, Clone, Copy, Debug, Default, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug)]
pub struct CollisionEventsEnabled;
