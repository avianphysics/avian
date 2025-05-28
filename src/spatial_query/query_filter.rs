use bevy::{ecs::entity::hash_set::EntityHashSet, prelude::*};

use crate::prelude::*;

/// Rules that determine which colliders are taken into account in [spatial queries](crate::spatial_query).
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
    doc = "    let object = commands.spawn(Collider::circle(0.5)).id();"
)]
#[cfg_attr(
    feature = "3d",
    doc = "    let object = commands.spawn(Collider::sphere(0.5)).id();"
)]
///
///     // A query filter that has three collision layers and excludes the `object` entity
///     let query_filter = SpatialQueryFilter::from_mask(0b1011).with_excluded_entities([object]);
///
///     // Spawn a ray caster with the query filter
///     commands.spawn(RayCaster::default().with_query_filter(query_filter));
/// }
/// ```
#[derive(Clone, Debug, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct SpatialQueryFilter {
    /// Specifies which [collision layers](CollisionLayers) will be included in the [spatial query](crate::spatial_query).
    pub mask: LayerMask,
    /// Entities that will not be included in [spatial queries](crate::spatial_query).
    pub excluded_entities: EntityHashSet,
}

impl Default for SpatialQueryFilter {
    fn default() -> Self {
        Self::DEFAULT
    }
}

impl SpatialQueryFilter {
    /// The default [`SpatialQueryFilter`] configuration that includes all collision layers
    /// and has no excluded entities.
    pub const DEFAULT: Self = Self {
        mask: LayerMask::ALL,
        excluded_entities: EntityHashSet::new(),
    };

    /// Creates a new [`SpatialQueryFilter`] with the given [`LayerMask`] determining
    /// which [collision layers] will be included in the [spatial query].
    ///
    /// [collision layers]: CollisionLayers
    /// [spatial query]: crate::spatial_query
    pub fn from_mask(mask: impl Into<LayerMask>) -> Self {
        Self {
            mask: mask.into(),
            ..default()
        }
    }

    /// Creates a new [`SpatialQueryFilter`] with the given entities excluded from the [spatial query].
    ///
    /// [spatial query]: crate::spatial_query
    pub fn from_excluded_entities(entities: impl IntoIterator<Item = Entity>) -> Self {
        Self {
            excluded_entities: EntityHashSet::from_iter(entities),
            ..default()
        }
    }

    /// Sets the [`LayerMask`] of the filter configuration. Only colliders with the corresponding
    /// [collision layer memberships] will be included in the [spatial query].
    ///
    /// [collision layer memberships]: CollisionLayers
    /// [spatial query]: crate::spatial_query
    pub fn with_mask(mut self, masks: impl Into<LayerMask>) -> Self {
        self.mask = masks.into();
        self
    }

    /// Excludes the given entities from the [spatial query](crate::spatial_query).
    pub fn with_excluded_entities(mut self, entities: impl IntoIterator<Item = Entity>) -> Self {
        self.excluded_entities = EntityHashSet::from_iter(entities);
        self
    }

    /// Tests if an entity should be included in [spatial queries] based on the filter configuration.
    ///
    /// [spatial queries]: crate::spatial_query
    pub fn test(&self, entity: Entity, layers: CollisionLayers) -> bool {
        !self.excluded_entities.contains(&entity)
            && CollisionLayers::new(LayerMask::ALL, self.mask)
                .interacts_with(CollisionLayers::new(layers.memberships, LayerMask::ALL))
    }
}
