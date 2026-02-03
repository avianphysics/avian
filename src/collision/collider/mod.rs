//! Components, traits, and plugins related to collider functionality.

use crate::prelude::*;
use bevy::{
    ecs::{
        component::Mutable,
        entity::{EntityMapper, MapEntities, hash_set::EntityHashSet},
        system::{ReadOnlySystemParam, SystemParam, SystemParamItem},
    },
    prelude::*,
};
use derive_more::From;

mod backend;

pub use backend::{ColliderBackendPlugin, ColliderMarker};

#[cfg(all(feature = "collider-from-mesh", feature = "default-collider"))]
mod cache;
#[cfg(all(feature = "collider-from-mesh", feature = "default-collider"))]
pub use cache::ColliderCachePlugin;
pub mod collider_hierarchy;
pub mod collider_transform;
#[cfg(all(feature = "3d", any(feature = "parry-f32", feature = "parry-f64")))]
pub mod trimesh_builder;

mod layers;
pub use layers::*;

/// The default [`Collider`] that uses Parry.
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
mod parry;
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
pub use parry::*;

#[cfg(feature = "default-collider")]
mod constructor;
#[cfg(feature = "default-collider")]
pub use constructor::{
    ColliderConstructor, ColliderConstructorHierarchy, ColliderConstructorHierarchyConfig,
    ColliderConstructorHierarchyReady, ColliderConstructorReady,
};

/// A trait for creating colliders from other types.
pub trait IntoCollider<C: AnyCollider> {
    /// Creates a collider from `self`.
    fn collider(&self) -> C;
}

/// Context necessary to calculate [`ColliderAabb`]s for an [`AnyCollider`]
#[derive(Deref)]
pub struct AabbContext<'a, 'w, 's, T: ReadOnlySystemParam> {
    /// The entity for which the aabb is being calculated
    pub entity: Entity,
    #[deref]
    item: &'a SystemParamItem<'w, 's, T>,
}

impl<T: ReadOnlySystemParam> Clone for AabbContext<'_, '_, '_, T> {
    fn clone(&self) -> Self {
        Self {
            entity: self.entity,
            item: self.item,
        }
    }
}

impl<'a, 'w, 's, T: ReadOnlySystemParam> AabbContext<'a, 'w, 's, T> {
    /// Construct an [`AabbContext`]
    pub fn new(entity: Entity, item: &'a <T as SystemParam>::Item<'w, 's>) -> Self {
        Self { entity, item }
    }
}

impl AabbContext<'_, '_, '_, ()> {
    fn fake() -> Self {
        Self {
            entity: Entity::PLACEHOLDER,
            item: &(),
        }
    }
}

/// Context necessary to calculate [`ContactManifold`]s for a set of [`AnyCollider`]
#[derive(Deref)]
pub struct ContactManifoldContext<'a, 'w, 's, T: ReadOnlySystemParam> {
    /// The first collider entity involved in the contact.
    pub entity1: Entity,
    /// The second collider entity involved in the contact.
    pub entity2: Entity,
    #[deref]
    item: &'a SystemParamItem<'w, 's, T>,
}

impl<'a, 'w, 's, T: ReadOnlySystemParam> ContactManifoldContext<'a, 'w, 's, T> {
    /// Construct a [`ContactManifoldContext`]
    pub fn new(
        entity1: Entity,
        entity2: Entity,
        item: &'a <T as SystemParam>::Item<'w, 's>,
    ) -> Self {
        Self {
            entity1,
            entity2,
            item,
        }
    }
}

impl ContactManifoldContext<'_, '_, '_, ()> {
    fn fake() -> Self {
        Self {
            entity1: Entity::PLACEHOLDER,
            entity2: Entity::PLACEHOLDER,
            item: &(),
        }
    }
}

/// A trait that generalizes over colliders. Implementing this trait
/// allows colliders to be used with the physics engine.
pub trait AnyCollider: Component<Mutability = Mutable> + ComputeMassProperties {
    /// A type providing additional context for collider operations.
    ///
    /// `Context` allows you to access an arbitrary [`ReadOnlySystemParam`] on
    /// the world, for context-sensitive behavior in collider operations. You
    /// can use this to query components on the collider entity, or get any
    /// other necessary context from the world.
    ///
    /// # Example
    ///
    /// ```
    #[cfg_attr(
        feature = "2d",
        doc = "# use avian2d::{prelude::*, math::{Vector, Scalar}};"
    )]
    #[cfg_attr(
        feature = "3d",
        doc = "# use avian3d::{prelude::*, math::{Vector, Scalar}};"
    )]
    /// # use bevy::prelude::*;
    /// # use bevy::ecs::system::{SystemParam, lifetimeless::{SRes, SQuery}};
    /// #
    /// #[derive(Component)]
    /// pub struct VoxelCollider;
    ///
    /// #[derive(Component)]
    /// pub struct VoxelData {
    ///     // collider voxel data...
    /// }
    ///
    /// # impl ComputeMassProperties2d for VoxelCollider {
    /// #     fn mass(&self, density: f32) -> f32 {0.}
    /// #     fn unit_angular_inertia(&self) -> f32 { 0.}
    /// #     fn center_of_mass(&self) -> Vec2 { Vec2::ZERO }
    /// # }
    /// #
    /// # impl ComputeMassProperties3d for VoxelCollider {
    /// #     fn mass(&self, density: f32) -> f32 {0.}
    /// #     fn unit_principal_angular_inertia(&self) -> Vec3 { Vec3::ZERO }
    /// #     fn center_of_mass(&self) -> Vec3 { Vec3::ZERO }
    /// # }
    /// #
    /// impl AnyCollider for VoxelCollider {
    ///     type Context = (
    ///         // you can query extra components here
    ///         SQuery<&'static VoxelData>,
    ///         // or put any other read-only system param here
    ///         SRes<Time>,
    ///     );
    ///
    /// #   fn aabb_with_context(
    /// #       &self,
    /// #       _: Vector,
    /// #       _: impl Into<Rotation>,
    /// #       _: AabbContext<Self::Context>,
    /// #   ) -> ColliderAabb { unimplemented!() }
    /// #
    ///     fn contact_manifolds_with_context(
    ///         &self,
    ///         other: &Self,
    ///         position1: Vector,
    ///         rotation1: impl Into<Rotation>,
    ///         position2: Vector,
    ///         rotation2: impl Into<Rotation>,
    ///         prediction_distance: Scalar,
    ///         manifolds: &mut Vec<ContactManifold>,
    ///         context: ContactManifoldContext<Self::Context>,
    ///     ) {
    ///         let [voxels1, voxels2] = context.0.get_many([context.entity1, context.entity2])
    ///             .expect("our own `VoxelCollider` entities should have `VoxelData`");
    ///         let elapsed = context.1.elapsed();
    ///         // do some computation...
    /// #       unimplemented!()
    ///     }
    /// }
    /// ```
    type Context: for<'w, 's> ReadOnlySystemParam<Item<'w, 's>: Send + Sync>;

    /// Computes the [Axis-Aligned Bounding Box](ColliderAabb) of the collider
    /// with the given position and rotation.
    ///
    /// See [`SimpleCollider::aabb`] for collider types with empty [`AnyCollider::Context`]
    #[cfg_attr(
        feature = "2d",
        doc = "\n\nThe rotation is counterclockwise and in radians."
    )]
    fn aabb_with_context(
        &self,
        position: Vector,
        rotation: impl Into<Rotation>,
        context: AabbContext<Self::Context>,
    ) -> ColliderAabb;

    /// Computes the swept [Axis-Aligned Bounding Box](ColliderAabb) of the collider.
    /// This corresponds to the space the shape would occupy if it moved from the given
    /// start position to the given end position.
    ///
    /// See [`SimpleCollider::swept_aabb`] for collider types with empty [`AnyCollider::Context`]
    #[cfg_attr(
        feature = "2d",
        doc = "\n\nThe rotation is counterclockwise and in radians."
    )]
    fn swept_aabb_with_context(
        &self,
        start_position: Vector,
        start_rotation: impl Into<Rotation>,
        end_position: Vector,
        end_rotation: impl Into<Rotation>,
        context: AabbContext<Self::Context>,
    ) -> ColliderAabb {
        self.aabb_with_context(start_position, start_rotation, context.clone())
            .merged(self.aabb_with_context(end_position, end_rotation, context))
    }

    /// Computes all [`ContactManifold`]s between two colliders.
    ///
    /// Returns an empty vector if the colliders are separated by a distance greater than `prediction_distance`
    /// or if the given shapes are invalid.
    ///
    /// See [`SimpleCollider::contact_manifolds`] for collider types with empty [`AnyCollider::Context`]
    fn contact_manifolds_with_context(
        &self,
        other: &Self,
        position1: Vector,
        rotation1: impl Into<Rotation>,
        position2: Vector,
        rotation2: impl Into<Rotation>,
        prediction_distance: Scalar,
        manifolds: &mut Vec<ContactManifold>,
        context: ContactManifoldContext<Self::Context>,
    );
}

/// A simplified wrapper around [`AnyCollider`] that doesn't require passing in the context for
/// implementations that don't need context
pub trait SimpleCollider: AnyCollider<Context = ()> {
    /// Computes the [Axis-Aligned Bounding Box](ColliderAabb) of the collider
    /// with the given position and rotation.
    ///
    /// See [`AnyCollider::aabb_with_context`] for collider types with non-empty [`AnyCollider::Context`]
    fn aabb(&self, position: Vector, rotation: impl Into<Rotation>) -> ColliderAabb {
        self.aabb_with_context(position, rotation, AabbContext::fake())
    }

    /// Computes the swept [Axis-Aligned Bounding Box](ColliderAabb) of the collider.
    /// This corresponds to the space the shape would occupy if it moved from the given
    /// start position to the given end position.
    ///
    /// See [`AnyCollider::swept_aabb_with_context`] for collider types with non-empty [`AnyCollider::Context`]
    fn swept_aabb(
        &self,
        start_position: Vector,
        start_rotation: impl Into<Rotation>,
        end_position: Vector,
        end_rotation: impl Into<Rotation>,
    ) -> ColliderAabb {
        self.swept_aabb_with_context(
            start_position,
            start_rotation,
            end_position,
            end_rotation,
            AabbContext::fake(),
        )
    }

    /// Computes all [`ContactManifold`]s between two colliders, writing the results into `manifolds`.
    ///
    /// `manifolds` is cleared if the colliders are separated by a distance greater than `prediction_distance`
    /// or if the given shapes are invalid.
    ///
    /// See [`AnyCollider::contact_manifolds_with_context`] for collider types with non-empty [`AnyCollider::Context`]
    fn contact_manifolds(
        &self,
        other: &Self,
        position1: Vector,
        rotation1: impl Into<Rotation>,
        position2: Vector,
        rotation2: impl Into<Rotation>,
        prediction_distance: Scalar,
        manifolds: &mut Vec<ContactManifold>,
    ) {
        self.contact_manifolds_with_context(
            other,
            position1,
            rotation1,
            position2,
            rotation2,
            prediction_distance,
            manifolds,
            ContactManifoldContext::fake(),
        )
    }
}

impl<C: AnyCollider<Context = ()>> SimpleCollider for C {}

/// A trait for colliders that support scaling.
pub trait ScalableCollider: AnyCollider {
    /// Returns the global scaling factor of the collider.
    fn scale(&self) -> Vector;

    /// Sets the global scaling factor of the collider.
    ///
    /// If the scaling factor is not uniform and the resulting scaled shape
    /// can not be represented exactly, the given `detail` is used for an approximation.
    fn set_scale(&mut self, scale: Vector, detail: u32);

    /// Scales the collider by the given scaling factor.
    ///
    /// If the scaling factor is not uniform and the resulting scaled shape
    /// can not be represented exactly, the given `detail` is used for an approximation.
    fn scale_by(&mut self, factor: Vector, detail: u32) {
        self.set_scale(factor * self.scale(), detail)
    }
}

/// A marker component that indicates that a [collider](Collider) is disabled
/// and should not detect collisions or be included in spatial queries.
///
/// This is useful for temporarily disabling a collider without removing it from the world.
/// To re-enable the collider, simply remove this component.
///
/// Note that a disabled collider will still contribute to the mass properties of the rigid body
/// it is attached to. Set the [`Mass`] of the collider to zero to prevent this.
///
/// [`ColliderDisabled`] only applies to the entity it is attached to, not its children.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::*;
/// #
/// #[derive(Component)]
/// pub struct Character;
///
/// /// Disables colliders for all rigid body characters, for example during cutscenes.
/// fn disable_character_colliders(
///     mut commands: Commands,
///     query: Query<Entity, (With<RigidBody>, With<Character>)>,
/// ) {
///     for entity in &query {
///         commands.entity(entity).insert(ColliderDisabled);
///     }
/// }
///
/// /// Enables colliders for all rigid body characters.
/// fn enable_character_colliders(
///     mut commands: Commands,
///     query: Query<Entity, (With<RigidBody>, With<Character>)>,
/// ) {
///     for entity in &query {
///         commands.entity(entity).remove::<ColliderDisabled>();
///     }
/// }
/// ```
///
/// # Related Components
///
/// - [`RigidBodyDisabled`]: Disables a rigid body.
/// - [`JointDisabled`]: Disables a joint constraint.
#[derive(Reflect, Clone, Copy, Component, Debug, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default)]
pub struct ColliderDisabled;

/// A component that marks a [`Collider`] as a sensor, also known as a trigger.
///
/// Sensor colliders send [collision events](crate::collision#collision-events) and register intersections,
/// but allow other bodies to pass through them. This is often used to detect when something enters
/// or leaves an area or is intersecting some shape.
///
/// Sensor colliders do *not* contribute to the mass properties of rigid bodies.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     // Spawn a static body with a sensor collider.
///     // Other bodies will pass through, but it will still send collision events.
#[cfg_attr(
    feature = "2d",
    doc = "    commands.spawn((RigidBody::Static, Collider::circle(0.5), Sensor));"
)]
#[cfg_attr(
    feature = "3d",
    doc = "    commands.spawn((RigidBody::Static, Collider::sphere(0.5), Sensor));"
)]
/// }
/// ```
#[doc(alias = "Trigger")]
#[derive(Reflect, Clone, Component, Debug, Default, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct Sensor;

/// The Axis-Aligned Bounding Box of a [collider](Collider) in world space.
///
/// This is updated automatically.
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct ColliderAabb {
    /// The minimum point of the AABB.
    pub min: Vector,
    /// The maximum point of the AABB.
    pub max: Vector,
}

impl Default for ColliderAabb {
    fn default() -> Self {
        ColliderAabb::INVALID
    }
}

impl ColliderAabb {
    /// An invalid [`ColliderAabb`] that represents an empty AABB.
    pub const INVALID: Self = Self {
        min: Vector::INFINITY,
        max: Vector::NEG_INFINITY,
    };

    /// Creates a new [`ColliderAabb`] from the given `center` and `half_size`.
    pub fn new(center: Vector, half_size: Vector) -> Self {
        Self {
            min: center - half_size,
            max: center + half_size,
        }
    }

    /// Creates a new [`ColliderAabb`] from its minimum and maximum points.
    pub fn from_min_max(min: Vector, max: Vector) -> Self {
        Self { min, max }
    }

    /// Creates a new [`ColliderAabb`] from a given `SharedShape`.
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    pub fn from_shape(shape: &crate::parry::shape::SharedShape) -> Self {
        let aabb = shape.compute_local_aabb();
        Self {
            min: aabb.mins.into(),
            max: aabb.maxs.into(),
        }
    }

    /// Computes the center of the AABB,
    #[inline(always)]
    pub fn center(self) -> Vector {
        self.min.midpoint(self.max)
    }

    /// Computes the size of the AABB.
    #[inline(always)]
    pub fn size(self) -> Vector {
        self.max - self.min
    }

    /// Merges this AABB with another one.
    #[inline(always)]
    pub fn merged(self, other: Self) -> Self {
        ColliderAabb {
            min: self.min.min(other.min),
            max: self.max.max(other.max),
        }
    }

    /// Increases the size of the bounding volume in each direction by the given amount.
    #[inline(always)]
    pub fn grow(&self, amount: Vector) -> Self {
        let b = Self {
            min: self.min - amount,
            max: self.max + amount,
        };
        debug_assert!(b.min.cmple(b.max).all());
        b
    }

    /// Decreases the size of the bounding volume in each direction by the given amount.
    #[inline(always)]
    pub fn shrink(&self, amount: Vector) -> Self {
        let b = Self {
            min: self.min + amount,
            max: self.max - amount,
        };
        debug_assert!(b.min.cmple(b.max).all());
        b
    }

    /// Checks if `self` intersects with `other`.
    #[inline(always)]
    #[cfg(feature = "2d")]
    pub fn intersects(&self, other: &Self) -> bool {
        let x_overlaps = self.min.x <= other.max.x && self.max.x >= other.min.x;
        let y_overlaps = self.min.y <= other.max.y && self.max.y >= other.min.y;
        x_overlaps && y_overlaps
    }

    /// Checks if `self` intersects with `other`.
    #[inline(always)]
    #[cfg(feature = "3d")]
    pub fn intersects(&self, other: &Self) -> bool {
        let x_overlaps = self.min.x <= other.max.x && self.max.x >= other.min.x;
        let y_overlaps = self.min.y <= other.max.y && self.max.y >= other.min.y;
        let z_overlaps = self.min.z <= other.max.z && self.max.z >= other.min.z;
        x_overlaps && y_overlaps && z_overlaps
    }

    /// Checks if `self` contains `other`.
    #[inline(always)]
    pub fn contains(&self, other: &Self) -> bool {
        self.min.cmple(other.min).all() && self.max.cmpge(other.max).all()
    }
}

impl From<ColliderAabb> for obvhs::aabb::Aabb {
    fn from(value: ColliderAabb) -> Self {
        Self {
            #[cfg(feature = "3d")]
            min: value.min.f32().to_array().into(),
            #[cfg(feature = "3d")]
            max: value.max.f32().to_array().into(),
            #[cfg(feature = "2d")]
            min: value.min.f32().extend(0.0).to_array().into(),
            #[cfg(feature = "2d")]
            max: value.max.f32().extend(0.0).to_array().into(),
        }
    }
}

/// An Axis-Aligned Bounding Box that contains the [`ColliderAabb`] with an additional margin.
///
/// This is used to avoid updating the Bounding Volume Hierarchy acceleration structure
/// every time a collider moves only a small amount.
///
/// The enlarged AABB is updated automatically whenever the [`ColliderAabb`]
/// moves beyond the bounds of the current enlarged AABB.
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct EnlargedAabb(ColliderAabb);

impl EnlargedAabb {
    /// Creates a new [`EnlargedAabb`] from the given [`ColliderAabb`].
    pub fn new(aabb: ColliderAabb) -> Self {
        Self(aabb)
    }

    /// Updates the enlarged AABB with the given [`ColliderAabb`] and margin.
    ///
    /// If the AABB is already contained within the enlarged AABB, nothing happens.
    ///
    /// Returns `true` if the AABB was updated.
    pub fn update(&mut self, aabb: &ColliderAabb, margin: Scalar) -> bool {
        if self.contains(aabb) {
            return false;
        }

        let margin = Vector::splat(margin);
        self.0.min = aabb.min - margin;
        self.0.max = aabb.max + margin;

        true
    }

    /// Gets the [`ColliderAabb`] of the enlarged AABB.
    pub fn get(&self) -> ColliderAabb {
        self.0
    }
}

/// A component that adds an extra margin or "skin" around [`Collider`] shapes to help maintain
/// additional separation to other objects. This added thickness can help improve
/// stability and performance in some cases, especially for thin shapes such as trimeshes.
///
/// There are three primary reasons for collision margins:
///
/// 1. Collision detection is often more efficient when shapes are not overlapping
/// further than their collision margins. Deeply overlapping shapes require
/// more expensive collision algorithms.
///
/// 2. Some shapes such as triangles and planes are infinitely thin,
/// which can cause precision errors. A collision margin adds artificial
/// thickness to shapes, improving stability.
///
/// 3. Overall, collision margins give the physics engine more
/// room for error when resolving contacts. This can also help
/// prevent visible artifacts such as objects poking through the ground.
///
/// If a rigid body with a [`CollisionMargin`] has colliders as child entities,
/// and those colliders don't have their own [`CollisionMargin`] components,
/// the colliders will use the rigid body's [`CollisionMargin`].
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
#[cfg_attr(
    feature = "2d",
    doc = "    // Spawn a rigid body with a collider.
    // A margin of `0.1` is added around the shape.
    commands.spawn((
        RigidBody::Dynamic,
        Collider::capsule(2.0, 0.5),
        CollisionMargin(0.1),
    ));"
)]
#[cfg_attr(
    feature = "3d",
    doc = "    let mesh = Mesh::from(Torus::default());

    // Spawn a rigid body with a triangle mesh collider.
    // A margin of `0.1` is added around the shape.
    commands.spawn((
        RigidBody::Dynamic,
        Collider::trimesh_from_mesh(&mesh).unwrap(),
        CollisionMargin(0.1),
    ));"
)]
/// }
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, Default, Deref, DerefMut, PartialEq, From)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component)]
#[doc(alias = "ContactSkin")]
pub struct CollisionMargin(pub Scalar);

/// A component for reading which entities are colliding with a collider entity.
/// Must be added manually for desired colliders.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands) {
///     commands.spawn((
///         RigidBody::Dynamic,
///         Collider::capsule(0.5, 1.5),
///         // Add the `CollidingEntities` component to read entities colliding with this entity.
///         CollidingEntities::default(),
///     ));
/// }
///
/// fn my_system(query: Query<(Entity, &CollidingEntities)>) {
///     for (entity, colliding_entities) in &query {
///         println!(
///             "{} is colliding with the following entities: {:?}",
///             entity,
///             colliding_entities,
///         );
///     }
/// }
/// ```
#[derive(Reflect, Clone, Component, Debug, Default, Deref, DerefMut, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, Default, PartialEq)]
pub struct CollidingEntities(pub EntityHashSet);

impl MapEntities for CollidingEntities {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.0 = self
            .0
            .clone()
            .into_iter()
            .map(|e| entity_mapper.get_mapped(e))
            .collect()
    }
}
