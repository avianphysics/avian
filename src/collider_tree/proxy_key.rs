use core::hint::unreachable_unchecked;

use bevy::{ecs::component::Component, reflect::Reflect};

use crate::prelude::RigidBody;

/// A key for a proxy in a [`ColliderTree`], encoding both
/// the [`ProxyId`] and the [`ColliderTreeType`].
///
/// The tree type is stored in the lower 2 bits of the key,
/// leaving 30 bits for the [`ProxyId`].
///
/// [`ColliderTree`]: crate::collider_tree::ColliderTree
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Hash, Reflect)]
pub struct ColliderTreeProxyKey(u32);

impl ColliderTreeProxyKey {
    /// A placeholder proxy key used before the proxy is actually created.
    pub const PLACEHOLDER: Self = ColliderTreeProxyKey(u32::MAX);

    /// Creates a new [`ColliderTreeProxyKey`] from the given [`ProxyId`] and tree type.
    #[inline]
    pub const fn new(id: ProxyId, tree_type: ColliderTreeType) -> Self {
        // Encode the tree type in the lower 2 bits.
        ColliderTreeProxyKey((id.id() << 2) | (tree_type as u32))
    }

    /// Returns the [`ProxyId`] of the proxy.
    #[inline]
    pub const fn id(&self) -> ProxyId {
        ProxyId::new(self.0 >> 2)
    }

    /// Returns the [`ColliderTreeType`] of the proxy.
    #[inline]
    pub const fn tree_type(&self) -> ColliderTreeType {
        match self.0 & 0b11 {
            0 => ColliderTreeType::Dynamic,
            1 => ColliderTreeType::Kinematic,
            2 => ColliderTreeType::Static,
            3 => ColliderTreeType::Standalone,
            // Safety: Bitwise AND with 0b11 can only yield 0, 1, 2, or 3.
            _ => unsafe { unreachable_unchecked() },
        }
    }

    /// Returns the rigid body type associated with the proxy.
    ///
    /// If the proxy is a standalone collider with no body, returns `None`.
    #[inline]
    pub const fn body(&self) -> Option<RigidBody> {
        match self.0 & 0b11 {
            0 => Some(RigidBody::Dynamic),
            1 => Some(RigidBody::Kinematic),
            2 => Some(RigidBody::Static),
            3 => None,
            // Safety: Bitwise AND with 0b11 can only yield 0, 1, 2, or 3.
            _ => unsafe { unreachable_unchecked() },
        }
    }

    /// Returns `true` if the proxy belongs to a dynamic body.
    #[inline]
    pub const fn is_dynamic(&self) -> bool {
        if let Some(body) = self.body() {
            body as u32 == RigidBody::Dynamic as u32
        } else {
            false
        }
    }

    /// Returns `true` if the proxy belongs to a kinematic body.
    #[inline]
    pub const fn is_kinematic(&self) -> bool {
        if let Some(body) = self.body() {
            body as u32 == RigidBody::Kinematic as u32
        } else {
            false
        }
    }

    /// Returns `true` if the proxy belongs to a static body.
    #[inline]
    pub const fn is_static(&self) -> bool {
        if let Some(body) = self.body() {
            body as u32 == RigidBody::Static as u32
        } else {
            false
        }
    }

    /// Returns `true` if the proxy is a standalone collider with no body.
    #[inline]
    pub const fn is_standalone(&self) -> bool {
        self.body().is_none()
    }
}

/// A stable identifier for a proxy in a [`ColliderTree`].
///
/// [`ColliderTree`]: crate::collider_tree::ColliderTree
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Reflect)]
pub struct ProxyId(u32);

impl ProxyId {
    /// A placeholder proxy ID used before the proxy is actually created.
    pub const PLACEHOLDER: Self = ProxyId(u32::MAX >> 2);

    /// Creates a new [`ProxyId`] from the given `u32` identifier.
    ///
    /// Only the lower 30 bits should be used for the ID.
    ///
    /// # Panics
    ///
    /// Panics if either of the upper 2 bits are set and `debug_assertions` are enabled.
    #[inline]
    pub const fn new(id: u32) -> Self {
        debug_assert!(id < (1 << 30), "ProxyId can only use lower 30 bits");
        ProxyId(id)
    }

    /// Returns the proxy ID as a `u32`.
    #[inline]
    pub const fn id(&self) -> u32 {
        self.0
    }

    /// Returns the proxy ID as a `usize`.
    #[inline]
    pub const fn index(&self) -> usize {
        self.0 as usize
    }
}

impl From<u32> for ProxyId {
    #[inline]
    fn from(id: u32) -> Self {
        ProxyId::new(id)
    }
}

impl From<ProxyId> for u32 {
    #[inline]
    fn from(proxy_id: ProxyId) -> Self {
        proxy_id.id()
    }
}

impl PartialOrd for ProxyId {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for ProxyId {
    #[inline]
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        self.0.cmp(&other.0)
    }
}

/// The type of a collider tree, corresponding to the rigid body type.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Reflect)]
pub enum ColliderTreeType {
    /// A tree for dynamic bodies.
    Dynamic = 0,
    /// A tree for kinematic bodies.
    Kinematic = 1,
    /// A tree for static bodies.
    Static = 2,
    /// A tree for standalone colliders with no associated rigid body.
    Standalone = 3,
}

impl ColliderTreeType {
    /// An array of all possible [`ColliderTreeType`]s.
    pub const ALL: [ColliderTreeType; 4] = [
        ColliderTreeType::Dynamic,
        ColliderTreeType::Kinematic,
        ColliderTreeType::Static,
        ColliderTreeType::Standalone,
    ];

    /// Creates a new [`ColliderTreeType`] from the given optional rigid body type.
    ///
    /// `None` corresponds to standalone colliders with no body.
    #[inline]
    pub const fn from_body(body: Option<RigidBody>) -> Self {
        match body {
            Some(RigidBody::Dynamic) => ColliderTreeType::Dynamic,
            Some(RigidBody::Kinematic) => ColliderTreeType::Kinematic,
            Some(RigidBody::Static) => ColliderTreeType::Static,
            None => ColliderTreeType::Standalone,
        }
    }

    /// Returns `true` if the tree type is for dynamic bodies.
    #[inline]
    pub const fn is_dynamic(&self) -> bool {
        matches!(self, ColliderTreeType::Dynamic)
    }

    /// Returns `true` if the tree type is for kinematic bodies.
    #[inline]
    pub const fn is_kinematic(&self) -> bool {
        matches!(self, ColliderTreeType::Kinematic)
    }

    /// Returns `true` if the tree type is for static bodies.
    #[inline]
    pub const fn is_static(&self) -> bool {
        matches!(self, ColliderTreeType::Static)
    }

    /// Returns `true` if the tree type is for standalone colliders with no body.
    #[inline]
    pub const fn is_standalone(&self) -> bool {
        matches!(self, ColliderTreeType::Standalone)
    }
}

impl From<Option<RigidBody>> for ColliderTreeType {
    #[inline]
    fn from(body: Option<RigidBody>) -> Self {
        match body {
            Some(RigidBody::Dynamic) => ColliderTreeType::Dynamic,
            Some(RigidBody::Kinematic) => ColliderTreeType::Kinematic,
            Some(RigidBody::Static) => ColliderTreeType::Static,
            None => ColliderTreeType::Standalone,
        }
    }
}

impl From<ColliderTreeType> for Option<RigidBody> {
    #[inline]
    fn from(tree_type: ColliderTreeType) -> Self {
        match tree_type {
            ColliderTreeType::Dynamic => Some(RigidBody::Dynamic),
            ColliderTreeType::Kinematic => Some(RigidBody::Kinematic),
            ColliderTreeType::Static => Some(RigidBody::Static),
            ColliderTreeType::Standalone => None,
        }
    }
}

impl From<RigidBody> for ColliderTreeType {
    #[inline]
    fn from(body: RigidBody) -> Self {
        match body {
            RigidBody::Dynamic => ColliderTreeType::Dynamic,
            RigidBody::Kinematic => ColliderTreeType::Kinematic,
            RigidBody::Static => ColliderTreeType::Static,
        }
    }
}
