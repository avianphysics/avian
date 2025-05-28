use core::ops::{BitAnd, BitAndAssign, BitOr, BitOrAssign, BitXor, BitXorAssign, Not};

use bevy::prelude::*;

/// A layer used for determining which entities should interact with each other.
/// Physics layers are used heavily by [`CollisionLayers`].
///
/// This trait can be derived for enums with `#[derive(PhysicsLayer)]`.
pub trait PhysicsLayer: Sized + Default {
    /// Converts the layer to a bitmask.
    fn to_bits(&self) -> u32;
    /// Creates a layer bitmask with all bits set to 1.
    fn all_bits() -> u32;
}

impl<'a, L: PhysicsLayer> PhysicsLayer for &'a L
where
    &'a L: Default,
{
    fn to_bits(&self) -> u32 {
        L::to_bits(self)
    }

    fn all_bits() -> u32 {
        L::all_bits()
    }
}

/// A bitmask for layers.
///
/// A [`LayerMask`] can be constructed from bits directly, or from types implementing [`PhysicsLayer`].
/// The first bit `0b0001` is reserved for the default layer, which all entities belong to by default.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// #
/// #[derive(PhysicsLayer, Clone, Copy, Debug, Default)]
/// enum GameLayer {
///     #[default]
///     Default, // Layer 0 - the default layer that objects are assigned to
///     Player,  // Layer 1
///     Enemy,   // Layer 2
///     Ground,  // Layer 3
/// }
///
/// // The first bit is reserved for the default layer.
/// assert_eq!(GameLayer::default().to_bits(), 1 << 0);
///
/// // The `GameLayer::Ground` layer is the fourth layer, so its bit value is `1 << 3`.
/// assert_eq!(GameLayer::Ground.to_bits(), 1 << 3);
///
/// // Here, `GameLayer::Enemy` is automatically converted to a `LayerMask` for the comparison.
/// assert_eq!(LayerMask(0b00100), GameLayer::Enemy);
/// ```
///
/// Bitwise operations can be used to modify and combine masks:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// let mask1 = LayerMask(0b0001);
/// let mask2 = LayerMask(0b0010);
/// assert_eq!(mask1 | mask2, LayerMask(0b0011));
///
/// // You can also add layers from `u32` bitmasks and compare against them directly.
/// assert_eq!(mask1 | 0b0010, 0b0011);
/// ```
///
/// Another way to use [`LayerMask`] is to define layers as constants:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// // `1 << n` is bitshifting: the first layer shifted by `n` layers.
/// pub const FIRST_LAYER: LayerMask = LayerMask(1 << 0);
/// pub const LAST_LAYER: LayerMask = LayerMask(1 << 31);
///
/// // Bitwise operations for `LayerMask` unfortunately can't be const, so we need to access the `u32` values.
/// pub const COMBINED: LayerMask = LayerMask(FIRST_LAYER.0 | LAST_LAYER.0);
/// ```
#[derive(Reflect, Clone, Copy, Debug, Deref, DerefMut, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, PartialEq)]
pub struct LayerMask(pub u32);

impl From<u32> for LayerMask {
    fn from(layer: u32) -> Self {
        Self(layer)
    }
}

impl<L: PhysicsLayer> From<L> for LayerMask {
    fn from(layer: L) -> Self {
        LayerMask(layer.to_bits())
    }
}

impl<L: Into<LayerMask>, const N: usize> From<[L; N]> for LayerMask {
    fn from(value: [L; N]) -> Self {
        let mut bits = 0;

        for layer in value.into_iter().map(|l| {
            let layers: LayerMask = l.into();
            layers
        }) {
            bits |= layer.0;
        }

        LayerMask(bits)
    }
}

impl LayerMask {
    /// Contains all layers.
    pub const ALL: Self = Self(0xffff_ffff);
    /// Contains no layers.
    pub const NONE: Self = Self(0);
    /// Contains the default layer.
    pub const DEFAULT: Self = Self(1);

    /// Adds the given `layers` to `self`.
    ///
    /// # Example
    ///
    /// ```
    #[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
    #[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
    /// let mut layers = LayerMask(0b1010);
    ///
    /// // These are equivalent
    /// layers.add(0b0001);
    /// layers |= 0b0001;
    ///
    /// assert_eq!(layers, 0b1011);
    /// ```
    pub fn add(&mut self, layers: impl Into<Self>) {
        let layers: LayerMask = layers.into();
        *self |= layers;
    }

    /// Removes the given `layers` from `self`.
    ///
    /// # Example
    ///
    /// ```
    #[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
    #[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
    /// let mut layers = LayerMask(0b1010);
    ///
    /// // These are equivalent
    /// layers.remove(0b0010);
    /// layers &= !0b0010;
    ///
    /// assert_eq!(layers, 0b1000);
    /// ```
    pub fn remove(&mut self, layers: impl Into<Self>) {
        let layers: LayerMask = layers.into();
        *self &= !layers;
    }

    /// Returns `true` if `self` contains all of the given `layers`.
    ///
    /// # Example
    ///
    /// ```
    #[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
    #[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
    /// let mut layers = LayerMask(0b1010);
    ///
    /// // These are equivalent
    /// assert!(layers.has_all(0b1010));
    /// assert!((layers & 0b1010) != 0);
    ///
    /// assert!(!layers.has_all(0b0100));
    /// assert!((layers & 0b0100) == 0);
    /// ```
    #[doc(alias = "contains_all")]
    pub fn has_all(self, layers: impl Into<Self>) -> bool {
        let layers: LayerMask = layers.into();
        (self & layers) != 0
    }
}

impl<L: Into<LayerMask> + Copy> PartialEq<L> for LayerMask {
    fn eq(&self, other: &L) -> bool {
        let other: Self = (*other).into();
        self.0 == other.0
    }
}

impl<L: Into<LayerMask>> BitAnd<L> for LayerMask {
    type Output = Self;

    fn bitand(self, rhs: L) -> Self::Output {
        Self(self.0 & rhs.into().0)
    }
}

impl<L: Into<LayerMask>> BitAndAssign<L> for LayerMask {
    fn bitand_assign(&mut self, rhs: L) {
        self.0 = self.0 & rhs.into().0;
    }
}

impl<L: Into<LayerMask>> BitOr<L> for LayerMask {
    type Output = Self;

    fn bitor(self, rhs: L) -> Self::Output {
        Self(self.0 | rhs.into().0)
    }
}

impl<L: Into<LayerMask>> BitOrAssign<L> for LayerMask {
    fn bitor_assign(&mut self, rhs: L) {
        self.0 = self.0 | rhs.into().0;
    }
}

impl<L: Into<LayerMask>> BitXor<L> for LayerMask {
    type Output = Self;

    fn bitxor(self, rhs: L) -> Self::Output {
        Self(self.0 ^ rhs.into().0)
    }
}

impl<L: Into<LayerMask>> BitXorAssign<L> for LayerMask {
    fn bitxor_assign(&mut self, rhs: L) {
        self.0 = self.0 ^ rhs.into().0;
    }
}

impl Not for LayerMask {
    type Output = Self;

    fn not(self) -> Self::Output {
        Self(!self.0)
    }
}

/// Defines the collision layers of a collider using *memberships* and *filters*.
///
/// **Memberships** indicate what layers the collider is a part of.\
/// **Filters** indicate what layers the collider can interact with.
///
/// Two colliders `A` and `B` can interact if and only if:
///
/// - The memberships of `A` contain a layer that is also in the filters of `B`
/// - The memberships of `B` contain a layer that is also in the filters of `A`
///
/// The memberships and filters are stored as [`LayerMask`]s, which represent [bitmasks] for layers.
/// The first bit `0b0001` is reserved for the default layer, which all entities belong to by default.
///
/// By default, colliders have all filters and can interact with any layer.
///
/// [bitmasks]: https://en.wikipedia.org/wiki/Mask_(computing)
///
/// # Creation
///
/// Collision layers store memberships and filters using [`LayerMask`]s. A [`LayerMask`] can be created using
/// bitmasks, or by creating an enum that implements [`PhysicsLayer`].
///
/// Many [`CollisionLayers`] methods can take any type that implements `Into<LayerMask>`.
/// For example, you can use bitmasks with [`CollisionLayers::new`]:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// #
/// // Belongs to the second layer and interacts with colliders
/// // on the first, second, and third layer.
/// let layers = CollisionLayers::new(0b00010, 0b0111);
/// ```
///
/// You can also use an enum that implements [`PhysicsLayer`]:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// #
/// #[derive(PhysicsLayer, Default)]
/// enum GameLayer {
///     #[default]
///     Default, // Layer 0 - the default layer that objects are assigned to
///     Player,  // Layer 1
///     Enemy,   // Layer 2
///     Ground,  // Layer 3
/// }
///
/// // Player collides with enemies and the ground, but not with other players
/// let layers = CollisionLayers::new(
///     GameLayer::Player,
///     [GameLayer::Default, GameLayer::Enemy, GameLayer::Ground],
/// );
/// ```
///
/// You can also use [`LayerMask`] directly:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// #
/// // Belongs to the first layer and interacts with all layers.
/// let layers = CollisionLayers::new(LayerMask(0b0001), LayerMask::ALL);
/// ```
///
/// Layers can also be defined using constants and bitwise operations:
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// # use bevy::prelude::Commands;
/// #
/// // `1 << n` is bitshifting: the first layer shifted by `n` layers.
/// pub const FIRST_LAYER: u32 = 1 << 0; // Note: this is the default layer.
/// pub const SECOND_LAYER: u32 = 1 << 1;
/// pub const LAST_LAYER: u32 = 1 << 31;
///
/// fn spawn(mut commands: Commands) {
///     // This collider belongs to the first two layers and can interact with the last layer.
///     commands.spawn((
#[cfg_attr(feature = "2d", doc = "        Collider::circle(0.5),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(0.5),")]
///         CollisionLayers::from_bits(FIRST_LAYER | SECOND_LAYER, LAST_LAYER),
///     ));
/// }
/// ```
///
/// # Modifying Layers
///
/// Existing [`CollisionLayers`] can be modified by simply accessing the `memberships` and `filters`
/// and changing their [`LayerMask`]s.
///
/// ```
#[cfg_attr(feature = "2d", doc = "# use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "# use avian3d::prelude::*;")]
/// let mut layers = CollisionLayers::new(0b0010, 0b1011);
///
/// // Add memberships (these are equivalent)
/// layers.memberships.add(0b0001);
/// layers.memberships |= 0b0001;
///
/// assert_eq!(layers.memberships, 0b0011);
///
/// // Remove filters
/// layers.filters.remove(0b0001);
/// layers.filters &= !0b0001;
///
/// assert_eq!(layers.filters, 0b1010);
///
/// // Check if layers are contained
/// assert!(layers.memberships.has_all(0b0011));
/// assert!((layers.memberships & 0b0011) != 0);
/// ```
#[derive(Reflect, Clone, Copy, Component, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Component, PartialEq)]
pub struct CollisionLayers {
    /// The layers that an entity belongs to.
    #[doc(alias = "groups", alias = "layers")]
    pub memberships: LayerMask,
    /// The layers that an entity can interact with.
    #[doc(alias = "masks", alias = "layer_mask")]
    pub filters: LayerMask,
}

impl CollisionLayers {
    /// Contains the default layer and all filters.
    pub const DEFAULT: Self = Self {
        memberships: LayerMask::DEFAULT,
        filters: LayerMask::ALL,
    };

    /// Contains all memberships and filters.
    pub const ALL: Self = Self {
        memberships: LayerMask::ALL,
        filters: LayerMask::ALL,
    };

    /// Contains no memberships and no filters.
    pub const NONE: Self = Self {
        memberships: LayerMask::NONE,
        filters: LayerMask::NONE,
    };

    /// Contains all memberships but no filters.
    pub const ALL_MEMBERSHIPS: Self = Self {
        memberships: LayerMask::ALL,
        filters: LayerMask::NONE,
    };

    /// Contains all filters but no memberships.
    pub const ALL_FILTERS: Self = Self {
        memberships: LayerMask::NONE,
        filters: LayerMask::ALL,
    };

    /// Creates a new [`CollisionLayers`] configuration with the given collision memberships and filters.
    pub fn new(memberships: impl Into<LayerMask>, filters: impl Into<LayerMask>) -> Self {
        Self {
            memberships: memberships.into(),
            filters: filters.into(),
        }
    }

    /// Creates a new [`CollisionLayers`] configuration using bits.
    ///
    /// There is one bit per group and mask, so there are a total of 32 layers.
    /// For example, if an entity is a part of the layers `[0, 1, 3]` and can interact with the layers `[1, 2]`,
    /// the memberships in bits would be `0b01011` while the filters would be `0b00110`.
    pub const fn from_bits(memberships: u32, filters: u32) -> Self {
        Self {
            memberships: LayerMask(memberships),
            filters: LayerMask(filters),
        }
    }

    /// Returns true if an entity with this [`CollisionLayers`] configuration
    /// can interact with an entity with the `other` [`CollisionLayers`] configuration.
    pub fn interacts_with(self, other: Self) -> bool {
        (self.memberships & other.filters) != LayerMask::NONE
            && (other.memberships & self.filters) != LayerMask::NONE
    }
}

impl Default for CollisionLayers {
    fn default() -> Self {
        CollisionLayers::DEFAULT
    }
}

#[cfg(test)]
mod tests {
    use crate::prelude::*;

    #[derive(PhysicsLayer, Default)]
    enum GameLayer {
        #[default]
        Default,
        Player,
        Enemy,
        Ground,
    }

    #[test]
    fn creation() {
        let with_bitmask = CollisionLayers::new(0b00100, 0b01011);
        let with_enum = CollisionLayers::new(
            GameLayer::Enemy,
            [GameLayer::Default, GameLayer::Player, GameLayer::Ground],
        );
        let with_layers =
            CollisionLayers::new(LayerMask::from(GameLayer::Enemy), LayerMask(0b01011));

        assert_eq!(with_bitmask, with_enum);
        assert_eq!(with_bitmask, with_layers);

        assert!(with_bitmask.memberships.has_all(GameLayer::Enemy));
        assert!(!with_bitmask.memberships.has_all(GameLayer::Player));

        assert!(with_bitmask
            .filters
            .has_all([GameLayer::Player, GameLayer::Ground]));
        assert!(!with_bitmask.filters.has_all(GameLayer::Enemy));
    }
}
