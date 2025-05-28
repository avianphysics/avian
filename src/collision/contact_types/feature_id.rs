use bevy::prelude::*;

/// A feature ID indicating the type of a geometric feature: a vertex, an edge, or (in 3D) a face.
///
/// This type packs the feature type into the same value as the feature index,
/// which indicates the specific vertex/edge/face that this ID belongs to.
#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Hash, PartialEq)]
pub struct PackedFeatureId(pub u32);

impl PackedFeatureId {
    /// Packed feature id identifying an unknown feature.
    pub const UNKNOWN: Self = Self(0);

    const CODE_MASK: u32 = 0x3fff_ffff;
    const HEADER_MASK: u32 = !Self::CODE_MASK;
    const HEADER_VERTEX: u32 = 0b01 << 30;
    #[cfg(feature = "3d")]
    const HEADER_EDGE: u32 = 0b10 << 30;
    const HEADER_FACE: u32 = 0b11 << 30;

    /// Converts a vertex feature id into a packed feature id.
    pub fn vertex(code: u32) -> Self {
        assert_eq!(code & Self::HEADER_MASK, 0);
        Self(Self::HEADER_VERTEX | code)
    }

    /// Converts a edge feature id into a packed feature id.
    #[cfg(feature = "3d")]
    pub fn edge(code: u32) -> Self {
        assert_eq!(code & Self::HEADER_MASK, 0);
        Self(Self::HEADER_EDGE | code)
    }

    /// Converts a face feature id into a packed feature id.
    pub fn face(code: u32) -> Self {
        assert_eq!(code & Self::HEADER_MASK, 0);
        Self(Self::HEADER_FACE | code)
    }

    /// Is the identified feature a face?
    pub fn is_face(self) -> bool {
        self.0 & Self::HEADER_MASK == Self::HEADER_FACE
    }

    /// Is the identified feature a vertex?
    pub fn is_vertex(self) -> bool {
        self.0 & Self::HEADER_MASK == Self::HEADER_VERTEX
    }

    /// Is the identified feature an edge?
    #[cfg(feature = "3d")]
    pub fn is_edge(self) -> bool {
        self.0 & Self::HEADER_MASK == Self::HEADER_EDGE
    }

    /// Is the identified feature unknown?
    pub fn is_unknown(self) -> bool {
        self == Self::UNKNOWN
    }
}

impl From<u32> for PackedFeatureId {
    fn from(code: u32) -> Self {
        Self(code)
    }
}

#[cfg(any(feature = "parry-f32", feature = "parry-f64"))]
impl From<crate::parry::shape::PackedFeatureId> for PackedFeatureId {
    fn from(id: crate::parry::shape::PackedFeatureId) -> Self {
        Self(id.0)
    }
}
