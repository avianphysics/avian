//! Types used for creating triangle meshes from [`Collider`]s.

use core::num::NonZeroU32;

use bevy::prelude::*;
use parry::shape::{SharedShape, TypedShape};
use thiserror::Error;

use crate::prelude::*;

/// An ergonomic builder for triangle meshes from [`Collider`]s.
///
/// The builder can configure different subdivision levels for different shapes.
/// If a shape was not explicitly configured, the builder will use [`Self::fallback_subdivisions`].
///
/// Shapes with rounded corners such as [`Collider::round_cuboid`] will be subdivided as if they were not rounded.
///
/// # Example
///
/// ```
/// # use avian3d::{prelude::*, math::Vector};
///
/// let collider = Collider::sphere(1.0);
///
/// // Using default settings
/// let trimesh = collider.trimesh_builder().build().unwrap();
///
/// // Using extra subdivisions
/// let trimesh = collider
///     .trimesh_builder()
///     .sphere_subdivisions(20, 20)
///     .build()
///     .unwrap();
///
/// // Setting different subdivisions for different shapes
/// let trimesh = collider
///     .trimesh_builder()
///     .sphere_subdivisions(20, 20)
///     .capsule_subdivisions(10, 5)
///     .fallback_subdivisions(15)
///     .build()
///     .unwrap();
///
/// // Generating the trimesh with a transformation
/// let trimesh = collider
///     .trimesh_builder()
///     .translated(Vector::new(1.0, 0.0, 0.0))
///     .build()
///     .unwrap();
/// ```
#[derive(Debug, Clone)]
pub struct TrimeshBuilder {
    /// The shape to be converted into a triangle mesh.
    pub shape: SharedShape,
    /// The position of the shape. The default is [0, 0, 0].
    pub position: Position,
    /// The rotation of the shape. The default is the identity rotation.
    pub rotation: Rotation,
    /// Whether a failure to trimesh a subshape in a compound shape should fail the entire build process.
    /// Default is true.
    pub fail_on_compound_error: bool,
    /// The number of subdivisions to use for shapes that do not have a specific subdivision count.
    /// Default is 16.
    pub fallback_subdivisions: NonZeroU32,
    /// The number of subdivisions for shapes that derive from a sphere.
    /// Default is None.
    pub sphere_subdivisions: Option<(NonZeroU32, NonZeroU32)>,
    /// The number of subdivisions for shapes that derive from a capsule.
    /// Default is None.
    pub capsule_subdivision: Option<(NonZeroU32, NonZeroU32)>,
    /// The number of subdivisions for shapes that derive from a cylinder.
    /// Default is None.
    pub cylinder_subdivisions: Option<NonZeroU32>,
    /// The number of subdivisions for shapes that derive from a cone.
    /// Default is None.
    pub cone_subdivisions: Option<NonZeroU32>,
}

/// A generic triangle mesh representation.
#[derive(Debug, Clone, PartialEq, Reflect, Default)]
pub struct Trimesh {
    /// The vertices in
    pub vertices: Vec<Vector>,
    /// The indices in counter-clockwise winding
    pub indices: Vec<[u32; 3]>,
}

impl Trimesh {
    /// Extends the trimesh with the vertices and indices of another trimesh.
    /// The indices of `other` will be offset by the number of vertices in `self`.
    pub fn extend(&mut self, other: Trimesh) {
        let next_vertex_index = self.vertices.len() as u32;
        self.vertices.extend(other.vertices);
        self.indices.extend(
            other
                .indices
                .iter()
                .map(|is| is.map(|i| i + next_vertex_index)),
        );
    }
}

/// An error that can occur when building a triangle mesh with [`TrimeshBuilder::build`].
#[derive(Debug, Error)]
pub enum TrimeshBuilderError {
    /// The shape is not supported by the builder.
    #[error("Unsupported shape type: {0}")]
    UnsupportedShape(String),
}

impl TrimeshBuilder {
    /// Creates a new [`TrimeshBuilder`] for the given shape. Usually you'll want to call [`Collider::trimesh_builder`] instead.
    pub fn new(shape: SharedShape) -> Self {
        TrimeshBuilder {
            shape,
            position: default(),
            rotation: default(),
            fail_on_compound_error: true,
            // arbitrary number
            fallback_subdivisions: 16_u32.try_into().unwrap(),
            sphere_subdivisions: None,
            capsule_subdivision: None,
            cylinder_subdivisions: None,
            cone_subdivisions: None,
        }
    }

    /// Translates the mesh. Subsequent calls to this method will add to the previous translation.
    pub fn translated(&mut self, position: impl Into<Position>) -> &mut Self {
        self.position.0 += position.into().0;
        self
    }

    /// Rotates the mesh. Subsequent calls to this method will add to the previous rotation.
    pub fn rotated(&mut self, rotation: impl Into<Rotation>) -> &mut Self {
        self.rotation = rotation.into() * self.rotation;
        self
    }

    /// Sets the fallback subdivision count for shapes that don't have a specific subdivision count.
    /// Default is 16.
    pub fn fallback_subdivisions(&mut self, subdivisions: impl TryInto<NonZeroU32>) -> &mut Self {
        self.fallback_subdivisions = subdivisions
            .try_into()
            .unwrap_or_else(|_| panic!("Fallback subdivision count must be non-zero"));
        self
    }

    /// Sets the subdivision count for sphere shapes. `theta` is the number of subdivisions along the
    /// latitude, and `phi` is the number of subdivisions along the longitude.
    pub fn sphere_subdivisions(
        &mut self,
        theta: impl TryInto<NonZeroU32>,
        phi: impl TryInto<NonZeroU32>,
    ) -> &mut Self {
        self.sphere_subdivisions = Some((
            theta
                .try_into()
                .unwrap_or_else(|_| panic!("Sphere theta subdivisions must be non-zero")),
            phi.try_into()
                .inspect(|phi| {
                    assert!(phi.get() >= 2, "Sphere phi subdivisions must be at least 2")
                })
                .unwrap_or_else(|_| panic!("Sphere phi subdivisions must be non-zero")),
        ));
        self
    }

    /// Sets the subdivision count for capsule shapes. `theta` is the number of subdivisions along the
    /// latitude, and `phi` is the number of subdivisions along the longitude.
    pub fn capsule_subdivisions(
        &mut self,
        theta: impl TryInto<NonZeroU32>,
        phi: impl TryInto<NonZeroU32>,
    ) -> &mut Self {
        self.capsule_subdivision = Some((
            theta
                .try_into()
                .unwrap_or_else(|_| panic!("Capsule theta subdivisions must be non-zero")),
            phi.try_into()
                .inspect(|phi| {
                    assert!(
                        phi.get() >= 2,
                        "Capsule phi subdivisions must be at least 2"
                    )
                })
                .unwrap_or_else(|_| panic!("Capsule phi subdivisions must be non-zero")),
        ));
        self
    }

    /// Sets the subdivision count for cylinder shapes.
    pub fn cylinder_subdivisions(&mut self, subdivisions: impl TryInto<NonZeroU32>) -> &mut Self {
        self.cylinder_subdivisions = Some(
            subdivisions
                .try_into()
                .unwrap_or_else(|_| panic!("Cylinder subdivisions must be non-zero")),
        );
        self
    }

    /// Sets the subdivision count for cone shapes.
    pub fn cone_subdivisions(&mut self, subdivisions: impl TryInto<NonZeroU32>) -> &mut Self {
        self.cone_subdivisions = Some(
            subdivisions
                .try_into()
                .unwrap_or_else(|_| panic!("Cone subdivisions must be non-zero")),
        );
        self
    }

    /// Whether a failure to trimesh a subshape in a compound shape should fail the entire build process.
    /// Default is true.
    pub fn fail_on_compound_error(&mut self, fail_on_compound_error: bool) -> &mut Self {
        self.fail_on_compound_error = fail_on_compound_error;
        self
    }

    fn subdivisions(&self, get: impl Fn(&Self) -> Option<NonZeroU32>) -> u32 {
        get(self).unwrap_or(self.fallback_subdivisions).into()
    }

    /// Builds the trimesh from the configured settings.
    ///
    /// Returns an error if the shape is not supported.
    /// If the shape is a compound, errors in a subshape will either be ignored or fail the entire build process
    /// depending on the value of [`Self::fail_on_compound_error`].
    pub fn build(&self) -> Result<Trimesh, TrimeshBuilderError> {
        let (vertices, indices) = match self.shape.as_typed_shape() {
            // Simple cases
            TypedShape::Cuboid(cuboid) => cuboid.to_trimesh(),
            TypedShape::Voxels(voxels) => voxels.to_trimesh(),
            TypedShape::ConvexPolyhedron(convex_polyhedron) => convex_polyhedron.to_trimesh(),
            TypedShape::HeightField(height_field) => height_field.to_trimesh(),
            // Triangles
            TypedShape::Triangle(triangle) => {
                (vec![triangle.a, triangle.b, triangle.c], vec![[0, 1, 2]])
            }
            TypedShape::TriMesh(tri_mesh) => {
                (tri_mesh.vertices().to_vec(), tri_mesh.indices().to_vec())
            }
            // Need subdivisions
            TypedShape::Ball(ball) => ball.to_trimesh(
                self.subdivisions(|t| t.sphere_subdivisions?.0.into()),
                self.subdivisions(|t| t.sphere_subdivisions?.1.into()),
            ),
            TypedShape::Capsule(capsule) => capsule.to_trimesh(
                self.subdivisions(|t| t.capsule_subdivision?.0.into()),
                self.subdivisions(|t| t.capsule_subdivision?.1.into()),
            ),
            TypedShape::Cylinder(cylinder) => {
                cylinder.to_trimesh(self.subdivisions(|t| t.cylinder_subdivisions))
            }
            TypedShape::Cone(cone) => cone.to_trimesh(self.subdivisions(|t| t.cone_subdivisions)),
            // Compounds need to be unpacked
            TypedShape::Compound(compound) => {
                let mut sub_builder = self.clone();
                return compound.shapes().iter().try_fold(
                    Trimesh::default(),
                    move |mut compound_trimesh, (sub_pos, shape)| {
                        sub_builder.shape = shape.clone();
                        sub_builder.position =
                            Position(self.position.0 + self.rotation * sub_pos.translation);
                        sub_builder.rotation =
                            self.rotation.mul_quat(sub_pos.rotation).normalize().into();
                        let trimesh = match sub_builder.build() {
                            Ok(trimesh) => trimesh,
                            Err(error) => {
                                return if self.fail_on_compound_error {
                                    Err(error)
                                } else {
                                    Ok(compound_trimesh)
                                };
                            }
                        };

                        compound_trimesh.extend(trimesh);

                        // No need to track recursive compounds because parry panics on nested compounds anyways lol
                        Ok(compound_trimesh)
                    },
                );
            }
            // Rounded shapes ignore the rounding and use the inner shape
            TypedShape::RoundCuboid(round_shape) => round_shape.inner_shape.to_trimesh(),
            TypedShape::RoundTriangle(round_shape) => (
                vec![
                    round_shape.inner_shape.a,
                    round_shape.inner_shape.b,
                    round_shape.inner_shape.c,
                ],
                vec![[0, 1, 2]],
            ),
            TypedShape::RoundConvexPolyhedron(round_shape) => round_shape.inner_shape.to_trimesh(),
            TypedShape::RoundCylinder(round_shape) => round_shape
                .inner_shape
                .to_trimesh(self.subdivisions(|t| t.cylinder_subdivisions)),
            TypedShape::RoundCone(round_shape) => round_shape
                .inner_shape
                .to_trimesh(self.subdivisions(|t| t.cone_subdivisions)),
            // Not supported
            TypedShape::Segment(segment) => {
                return Err(TrimeshBuilderError::UnsupportedShape(format!(
                    "{segment:?}",
                )));
            }
            TypedShape::Polyline(polyline) => {
                return Err(TrimeshBuilderError::UnsupportedShape(format!(
                    "{polyline:?}",
                )));
            }
            TypedShape::HalfSpace(half_space) => {
                return Err(TrimeshBuilderError::UnsupportedShape(format!(
                    "{half_space:?}",
                )));
            }
            TypedShape::Custom(_shape) => {
                return Err(TrimeshBuilderError::UnsupportedShape("Custom".to_string()));
            }
        };
        let pos = self.position;
        Ok(Trimesh {
            vertices: vertices
                .into_iter()
                .map(|v| pos.0 + self.rotation * v)
                .collect(),
            indices,
        })
    }
}

impl Collider {
    /// Create a [`TrimeshBuilder`] for building a [`Trimesh`].
    pub fn trimesh_builder(&self) -> TrimeshBuilder {
        TrimeshBuilder::new(self.shape_scaled().clone())
    }
}

#[cfg(feature = "collider-from-mesh")]
impl From<Trimesh> for Mesh {
    fn from(trimesh: Trimesh) -> Self {
        use bevy::asset::RenderAssetUsages;
        use bevy::mesh::{Indices, PrimitiveTopology, VertexAttributeValues, prelude::*};

        let mut mesh = Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
        mesh.insert_attribute(
            Mesh::ATTRIBUTE_POSITION,
            VertexAttributeValues::Float32x3(
                trimesh
                    .vertices
                    .into_iter()
                    .map(|v| v.f32().to_array())
                    .collect(),
            ),
        );
        mesh.insert_indices(Indices::U32(
            trimesh.indices.into_iter().flatten().collect(),
        ));
        mesh.compute_normals();
        if let Err(err) = mesh.generate_tangents() {
            warn!("Failed to generate tangents for mesh: {err}");
        }

        mesh
    }
}

#[cfg(test)]
mod tests {
    use bevy_math::DVec3;

    use super::*;

    #[test]
    fn rasterizes_cuboid() {
        let collider = Collider::cuboid(1.0, 2.0, 3.0);
        let trimesh = collider.trimesh_builder().build().unwrap();
        assert_eq!(trimesh.vertices.len(), 8);
        assert_eq!(trimesh.indices.len(), 12);
    }

    #[test]
    fn rasterizes_compound() {
        let a = Collider::cuboid(1.0, 2.0, 3.0);
        let b = Collider::sphere(0.4);
        let collider = Collider::compound(vec![
            (Vector::new(1.0, 2.0, 3.0), Quat::from_rotation_z(0.2), a),
            (
                Vector::new(-12.0, 4.0, -0.01),
                Quat::from_rotation_x(0.1),
                b,
            ),
        ]);
        let trimesh = collider
            .trimesh_builder()
            .fallback_subdivisions(2)
            .translated(Vector::new(3.0, -2.0, 0.0))
            .rotated(Quat::from_rotation_y(-3.0))
            .build()
            .unwrap();
        assert_eq!(
            trimesh.vertices,
            vec![
                DVec3::new(1.6634156046802073, -1.0794012104819941, -4.354963570435039)
                    .adjust_precision(),
                DVec3::new(2.086775603322501, -1.079401210481994, -1.3849860769934481)
                    .adjust_precision(),
                DVec3::new(1.1165170453015136, -0.8807318727109551, -1.2466790821715974)
                    .adjust_precision(),
                DVec3::new(0.6931570466592198, -0.8807318727109549, -4.2166565756131895)
                    .adjust_precision(),
                DVec3::new(2.0567779125581613, 0.8807319423722887, -4.411036004147714)
                    .adjust_precision(),
                DVec3::new(2.480137911200455, 0.8807319423722885, -1.4410585107061231)
                    .adjust_precision(),
                DVec3::new(1.5098793531794676, 1.0794012801433277, -1.3027515158842724)
                    .adjust_precision(),
                DVec3::new(1.0865193545371739, 1.0794012801433275, -4.2727290093258645)
                    .adjust_precision(),
                DVec3::new(14.886956777274035, 1.60199840348629, -1.6440063661356903)
                    .adjust_precision(),
                DVec3::new(14.485324381553463, 2.0000000696613336, -1.627092099091476)
                    .adjust_precision(),
                DVec3::new(15.277318379804553, 2.0000000696613336, -1.739988098729421)
                    .adjust_precision(),
                DVec3::new(14.875685984083981, 2.398001735836377, -1.7230738316852063)
                    .adjust_precision(),
            ]
        );
        assert_eq!(
            trimesh.indices,
            vec![
                [4, 5, 0],
                [5, 1, 0],
                [5, 6, 1],
                [6, 2, 1],
                [6, 7, 3],
                [2, 6, 3],
                [7, 4, 0],
                [3, 7, 0],
                [0, 1, 2],
                [3, 0, 2],
                [7, 6, 5],
                [4, 7, 5],
                [8, 9, 10],
                [8, 10, 9],
                [9, 11, 10],
                [10, 11, 9],
            ]
        );
    }
}
