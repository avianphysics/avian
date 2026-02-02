use core::iter::once;

use crate::prelude::*;
use bevy::{platform::collections::HashMap, prelude::*};
use itertools::Either;

/// A component that will automatically generate [`Collider`]s on its descendants at runtime.
/// The type of the generated collider can be specified using [`ColliderConstructor`].
/// This supports computing the shape dynamically from the mesh, in which case only the descendants
/// with a [`Mesh`] will have colliders generated.
///
/// In contrast to [`ColliderConstructor`], this component will *not* generate a collider on its own entity.
///
/// If this component is used on a scene, such as one spawned by a [`SceneRoot`], it will
/// wait until the scene is loaded before generating colliders. Note that this requires
/// the `bevy_scene` feature to be enabled.
///
/// The exact configuration for each descendant can be specified using the helper methods
/// such as [`with_constructor_for_name`](Self::with_constructor_for_name).
///
/// This component will only override a pre-existing [`Collider`] component on a descendant entity
/// when it has been explicitly mentioned in the `config`.
///
/// # See Also
///
/// For inserting colliders on the same entity, use [`ColliderConstructor`].
///
/// # Caveats
///
/// When a component has multiple ancestors with [`ColliderConstructorHierarchy`], the insertion order is undefined.
///
/// # Example
///
/// Below are some examples of using [`ColliderConstructorHierarchy`] to generate colliders
/// for a glTF scene at runtime. Note that this requires the `bevy_scene` feature to be enabled.
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands, mut assets: ResMut<AssetServer>) {
///     let scene = assets.load("my_model.gltf#Scene0");
///
#[cfg_attr(
    feature = "2d",
    doc = "    // Spawn the scene and automatically generate circle colliders"
)]
#[cfg_attr(
    feature = "3d",
    doc = "    // Spawn the scene and automatically generate triangle mesh colliders"
)]
///     commands.spawn((
///         SceneRoot(scene.clone()),
#[cfg_attr(
    feature = "2d",
    doc = "        ColliderConstructorHierarchy::new(ColliderConstructor::Circle { radius: 2.0 }),"
)]
#[cfg_attr(
    feature = "3d",
    doc = "        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMesh),"
)]
///     ));
///
///     // Specify configuration for specific meshes by name
///     commands.spawn((
///         SceneRoot(scene.clone()),
#[cfg_attr(
    feature = "2d",
    doc = "        ColliderConstructorHierarchy::new(ColliderConstructor::Circle { radius: 2.0 })
            .with_constructor_for_name(\"Tree\", ColliderConstructor::Rectangle { x_length: 1.0, y_length: 2.0 })"
)]
#[cfg_attr(
    feature = "3d",
    doc = "        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMesh)
            .with_constructor_for_name(\"Tree\", ColliderConstructor::ConvexHullFromMesh)"
)]
///             .with_layers_for_name("Tree", CollisionLayers::from_bits(0b0010, 0b1111))
///             .with_density_for_name("Tree", 2.5),
///     ));
///
///     // Only generate colliders for specific meshes by name
///     commands.spawn((
///         SceneRoot(scene.clone()),
///         ColliderConstructorHierarchy::new(None)
#[cfg_attr(
    feature = "2d",
    doc = "            .with_constructor_for_name(\"Tree\", ColliderConstructor::Circle { radius: 2.0 }),"
)]
#[cfg_attr(
    feature = "3d",
    doc = "            .with_constructor_for_name(\"Tree\", ColliderConstructor::ConvexHullFromMesh),"
)]
///     ));
///
///     // Generate colliders for everything except specific meshes by name
///     commands.spawn((
///         SceneRoot(scene),
#[cfg_attr(
    feature = "2d",
    doc = "        ColliderConstructorHierarchy::new(ColliderConstructor::Circle { radius: 2.0 })
            .without_constructor_for_name(\"Tree\"),"
)]
#[cfg_attr(
    feature = "3d",
    doc = "        ColliderConstructorHierarchy::new(ColliderConstructor::TrimeshFromMeshWithConfig(
             TrimeshFlags::MERGE_DUPLICATE_VERTICES
        ))
        .without_constructor_for_name(\"Tree\"),"
)]
///     ));
/// }
/// ```
#[derive(Component, Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, Debug, PartialEq, Default)]
pub struct ColliderConstructorHierarchy {
    /// The default collider type used for each entity that isn't included in [`config`](Self::config).
    /// If `None`, all entities except the ones in [`config`](Self::config) will be skipped.
    pub default_constructor: Option<ColliderConstructor>,
    /// The default [`CollisionLayers`] used for colliders in the hierarchy.
    ///
    /// [`CollisionLayers::default()`] by default, with the first layer and all filters.
    pub default_layers: CollisionLayers,
    /// The default [`ColliderDensity`] used for colliders in the hierarchy.
    ///
    /// `1.0` by default.
    pub default_density: ColliderDensity,
    /// Specifies data like the [`ColliderConstructor`] and [`CollisionLayers`] for entities
    /// in the hierarchy by `Name`. Entries with a `None` value will be skipped.
    ///
    /// For the entities not found in this `HashMap`, [`default_constructor`](Self::default_constructor),
    /// [`default_layers`](Self::default_layers), and [`default_density`](Self::default_density) will be used instead.
    pub config: HashMap<String, Option<ColliderConstructorHierarchyConfig>>,
}

/// Triggered when a [`ColliderConstructor`] successfully inserted a [`Collider`].
///
/// The event is not triggered when the [`ColliderConstructor`] failed to construct the [`Collider`]
/// or when there was already a [`Collider`] on the entity.
#[derive(EntityEvent, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ColliderConstructorReady {
    /// The entity that held the [`ColliderConstructor`].
    pub entity: Entity,
}

/// Triggered when a [`ColliderConstructorHierarchy`] finished inserting all its [`Collider`]s.
///
/// Note that the event will still be triggered when when the hierarchy had no colliders to insert
/// or failed to insert all of them, so this event is not a guarantee that there are actually
/// any colliders in the scene.
#[derive(EntityEvent, Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
pub struct ColliderConstructorHierarchyReady {
    /// The entity that held the [`ColliderConstructorHierarchy`].
    pub entity: Entity,
}

impl ColliderConstructorHierarchy {
    /// Creates a new [`ColliderConstructorHierarchy`] with the default [`ColliderConstructor`] used for
    /// generating colliders set to the given `default_constructor`.
    ///
    /// If the given constructor type is `None`, collider generation is skipped
    /// for all entities in the hierarchy except the ones in [`config`](Self::config).
    ///
    /// Collider constructors can be specified for individual entities using
    /// [`with_constructor_for_name`](Self::with_constructor_for_name).
    pub fn new(default_constructor: impl Into<Option<ColliderConstructor>>) -> Self {
        Self {
            default_constructor: default_constructor.into(),
            default_layers: CollisionLayers::default(),
            default_density: ColliderDensity(1.0),
            config: default(),
        }
    }

    /// Specifies the default [`CollisionLayers`] used for colliders not included in [`ColliderConstructorHierarchy::config`].
    pub fn with_default_layers(mut self, layers: CollisionLayers) -> Self {
        self.default_layers = layers;
        self
    }

    /// Specifies the default [`ColliderDensity`] used for colliders not included in [`ColliderConstructorHierarchy::config`].
    pub fn with_default_density(mut self, density: impl Into<ColliderDensity>) -> Self {
        self.default_density = density.into();
        self
    }

    /// Specifies the [`ColliderConstructor`] used for an entity with the given `name`.
    pub fn with_constructor_for_name(
        mut self,
        name: &str,
        constructor: ColliderConstructor,
    ) -> Self {
        if let Some(Some(data)) = self.config.get_mut(name) {
            data.constructor = Some(constructor);
        } else {
            self.config.insert(
                name.to_string(),
                Some(ColliderConstructorHierarchyConfig {
                    constructor: Some(constructor),
                    ..default()
                }),
            );
        }
        self
    }

    /// Specifies the [`CollisionLayers`] used for an entity with the given `name`.
    pub fn with_layers_for_name(self, name: &str, layers: CollisionLayers) -> Self {
        self.with_config_for_name(name, |config| config.layers = Some(layers))
    }

    /// Specifies the [`ColliderDensity`] used for an entity with the given `name`.
    pub fn with_density_for_name(self, name: &str, density: impl Into<ColliderDensity>) -> Self {
        let density = density.into();
        self.with_config_for_name(name, |config| config.density = Some(density))
    }

    /// Sets the [`ColliderConstructor`] for the entity associated with the given `name` to `None`,
    /// skipping collider generation for it.
    pub fn without_constructor_for_name(mut self, name: &str) -> Self {
        self.config.insert(name.to_string(), None);
        self
    }

    fn with_config_for_name(
        mut self,
        name: &str,
        mut mutate_config: impl FnMut(&mut ColliderConstructorHierarchyConfig),
    ) -> Self {
        if let Some(Some(config)) = self.config.get_mut(name) {
            mutate_config(config);
        } else {
            let mut config = ColliderConstructorHierarchyConfig::default();
            mutate_config(&mut config);
            self.config.insert(name.to_string(), Some(config));
        }
        self
    }
}

/// Configuration for a specific collider generated from a scene using [`ColliderConstructorHierarchy`].
#[derive(Clone, Debug, Default, PartialEq, Reflect)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Debug, Default, PartialEq)]
pub struct ColliderConstructorHierarchyConfig {
    /// The type of collider generated for the mesh.
    ///
    /// If `None`, [`ColliderConstructorHierarchy::default_constructor`] is used instead.
    pub constructor: Option<ColliderConstructor>,
    /// The [`CollisionLayers`] used for this collider.
    ///
    /// If `None`, [`ColliderConstructorHierarchy::default_layers`] is used instead.
    pub layers: Option<CollisionLayers>,
    /// The [`ColliderDensity`] used for this collider.
    ///
    /// If `None`, [`ColliderConstructorHierarchy::default_density`] is used instead.
    pub density: Option<ColliderDensity>,
}

/// A component that will automatically generate a [`Collider`] at runtime using [`Collider::try_from_constructor`].
/// Enabling the `collider-from-mesh` feature activates support for computing the shape dynamically from the mesh attached to the same entity.
///
/// Since [`Collider`] is not [`Reflect`], you can use this type to statically specify a collider's shape instead.
///
/// This component will never override a pre-existing [`Collider`] component on the same entity.
///
/// # See Also
///
/// For inserting colliders on an entity's descendants, use [`ColliderConstructorHierarchy`].
///
/// # Panics
///
/// The system handling the generation of colliders will panic if the specified [`ColliderConstructor`]
/// requires a mesh, but the entity does not have a `Handle<Mesh>` component.
///
/// # Example
///
/// ```
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn setup(mut commands: Commands, mut assets: ResMut<AssetServer>, mut meshes: Assets<Mesh>) {
#[cfg_attr(feature = "2d", doc = "     // Spawn a circle with radius 2")]
#[cfg_attr(
    feature = "3d",
    doc = "    // Spawn a cube with a convex hull collider generated from the mesh"
)]
///     commands.spawn((
#[cfg_attr(
    feature = "2d",
    doc = "        ColliderConstructor::Circle { radius: 2.0 },"
)]
#[cfg_attr(
    feature = "3d",
    doc = "        ColliderConstructor::ConvexHullFromMesh,"
)]
///         Mesh3d(meshes.add(Cuboid::default())),
///     ));
/// }
/// ```
#[derive(Clone, Debug, PartialEq, Reflect, Component)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[cfg_attr(feature = "collider-from-mesh", derive(Default))]
#[cfg_attr(feature = "collider-from-mesh", reflect(Default))]
#[reflect(Debug, Component, PartialEq)]
#[reflect(no_field_bounds)]
#[non_exhaustive]
#[allow(missing_docs)]
pub enum ColliderConstructor {
    /// Constructs a collider with [`Collider::circle`].
    #[cfg(feature = "2d")]
    Circle { radius: Scalar },
    /// Constructs a collider with [`Collider::sphere`].
    #[cfg(feature = "3d")]
    Sphere { radius: Scalar },
    /// Constructs a collider with [`Collider::ellipse`].
    #[cfg(feature = "2d")]
    Ellipse {
        half_width: Scalar,
        half_height: Scalar,
    },
    /// Constructs a collider with [`Collider::rectangle`].
    #[cfg(feature = "2d")]
    Rectangle { x_length: Scalar, y_length: Scalar },
    /// Constructs a collider with [`Collider::cuboid`].
    #[cfg(feature = "3d")]
    Cuboid {
        x_length: Scalar,
        y_length: Scalar,
        z_length: Scalar,
    },
    /// Constructs a collider with [`Collider::round_rectangle`].
    #[cfg(feature = "2d")]
    RoundRectangle {
        x_length: Scalar,
        y_length: Scalar,
        border_radius: Scalar,
    },
    /// Constructs a collider with [`Collider::round_cuboid`].
    #[cfg(feature = "3d")]
    RoundCuboid {
        x_length: Scalar,
        y_length: Scalar,
        z_length: Scalar,
        border_radius: Scalar,
    },
    /// Constructs a collider with [`Collider::cylinder`].
    #[cfg(feature = "3d")]
    Cylinder { radius: Scalar, height: Scalar },
    /// Constructs a collider with [`Collider::cone`].
    #[cfg(feature = "3d")]
    Cone { radius: Scalar, height: Scalar },
    /// Constructs a collider with [`Collider::capsule`].
    Capsule { radius: Scalar, height: Scalar },
    /// Constructs a collider with [`Collider::capsule_endpoints`].
    CapsuleEndpoints {
        radius: Scalar,
        a: Vector,
        b: Vector,
    },
    /// Constructs a collider with [`Collider::half_space`].
    HalfSpace { outward_normal: Vector },
    /// Constructs a collider with [`Collider::segment`].
    Segment { a: Vector, b: Vector },
    /// Constructs a collider with [`Collider::triangle`].
    Triangle { a: Vector, b: Vector, c: Vector },
    /// Constructs a collider with [`Collider::regular_polygon`].
    #[cfg(feature = "2d")]
    RegularPolygon { circumradius: f32, sides: u32 },
    /// Constructs a collider with [`Collider::polyline`].
    Polyline {
        vertices: Vec<Vector>,
        indices: Option<Vec<[u32; 2]>>,
    },
    /// Constructs a collider with [`Collider::trimesh`].
    Trimesh {
        vertices: Vec<Vector>,
        indices: Vec<[u32; 3]>,
    },
    /// Constructs a collider with [`Collider::trimesh_with_config`].
    TrimeshWithConfig {
        vertices: Vec<Vector>,
        indices: Vec<[u32; 3]>,
        flags: TrimeshFlags,
    },
    /// Constructs a collider with [`Collider::convex_decomposition`].
    #[cfg(feature = "2d")]
    ConvexDecomposition {
        vertices: Vec<Vector>,
        indices: Vec<[u32; 2]>,
    },
    /// Constructs a collider with [`Collider::convex_decomposition`].
    #[cfg(feature = "3d")]
    ConvexDecomposition {
        vertices: Vec<Vector>,
        indices: Vec<[u32; 3]>,
    },
    /// Constructs a collider with [`Collider::convex_decomposition_with_config`].
    #[cfg(feature = "2d")]
    ConvexDecompositionWithConfig {
        vertices: Vec<Vector>,
        indices: Vec<[u32; 2]>,
        params: VhacdParameters,
    },
    /// Constructs a collider with [`Collider::convex_decomposition_with_config`].
    #[cfg(feature = "3d")]
    ConvexDecompositionWithConfig {
        vertices: Vec<Vector>,
        indices: Vec<[u32; 3]>,
        params: VhacdParameters,
    },
    /// Constructs a collider with [`Collider::convex_hull`].
    #[cfg(feature = "2d")]
    ConvexHull { points: Vec<Vector> },
    /// Constructs a collider with [`Collider::convex_hull`].
    #[cfg(feature = "3d")]
    ConvexHull { points: Vec<Vector> },
    /// Constructs a collider with [`Collider::convex_polyline`].
    #[cfg(feature = "2d")]
    ConvexPolyline { points: Vec<Vector> },
    /// Constructs a collider with [`Collider::voxels`].
    Voxels {
        voxel_size: Vector,
        grid_coordinates: Vec<IVector>,
    },
    /// Constructs a collider with [`Collider::voxelized_polyline`].
    #[cfg(feature = "2d")]
    VoxelizedPolyline {
        vertices: Vec<Vector>,
        indices: Vec<[u32; 2]>,
        voxel_size: Scalar,
        fill_mode: FillMode,
    },
    /// Constructs a collider with [`Collider::voxelized_trimesh`].
    #[cfg(feature = "3d")]
    VoxelizedTrimesh {
        vertices: Vec<Vector>,
        indices: Vec<[u32; 3]>,
        voxel_size: Scalar,
        fill_mode: FillMode,
    },
    /// Constructs a collider with [`Collider::heightfield`].
    #[cfg(feature = "2d")]
    Heightfield { heights: Vec<Scalar>, scale: Vector },
    /// Constructs a collider with [`Collider::heightfield`].
    #[cfg(feature = "3d")]
    Heightfield {
        heights: Vec<Vec<Scalar>>,
        scale: Vector,
    },
    /// Constructs a collider with [`Collider::trimesh_from_mesh`].
    #[cfg(feature = "collider-from-mesh")]
    #[default]
    TrimeshFromMesh,
    /// Constructs a collider with [`Collider::trimesh_from_mesh_with_config`].
    #[cfg(all(
        feature = "3d",
        feature = "collider-from-mesh",
        feature = "default-collider"
    ))]
    TrimeshFromMeshWithConfig(TrimeshFlags),
    /// Constructs a collider with [`Collider::convex_decomposition_from_mesh`].
    #[cfg(feature = "collider-from-mesh")]
    ConvexDecompositionFromMesh,
    /// Constructs a collider with [`Collider::convex_decomposition_from_mesh_with_config`].
    #[cfg(all(
        feature = "3d",
        feature = "collider-from-mesh",
        feature = "default-collider"
    ))]
    ConvexDecompositionFromMeshWithConfig(VhacdParameters),
    /// Constructs a collider with [`Collider::convex_hull_from_mesh`].
    #[cfg(feature = "collider-from-mesh")]
    ConvexHullFromMesh,
    /// Constructs a collider with [`Collider::voxelized_trimesh_from_mesh`].
    #[cfg(feature = "collider-from-mesh")]
    VoxelizedTrimeshFromMesh {
        voxel_size: Scalar,
        fill_mode: FillMode,
    },
    /// Constructs a collider with [`Collider::compound`].
    Compound(Vec<(Position, Rotation, ColliderConstructor)>),
}

impl ColliderConstructor {
    /// Returns `true` if the collider type requires a mesh to be generated.
    #[cfg(feature = "collider-from-mesh")]
    pub fn requires_mesh(&self) -> bool {
        matches!(
            self,
            Self::TrimeshFromMesh
                | Self::TrimeshFromMeshWithConfig(_)
                | Self::ConvexDecompositionFromMesh
                | Self::ConvexDecompositionFromMeshWithConfig(_)
                | Self::ConvexHullFromMesh
                | Self::VoxelizedTrimeshFromMesh { .. }
        )
    }

    /// Construct a [`ColliderConstructor::Compound`] from arbitrary [`Position`] and [`Rotation`] representations.
    pub fn compound<P, R>(shapes: Vec<(P, R, ColliderConstructor)>) -> Self
    where
        P: Into<Position>,
        R: Into<Rotation>,
    {
        Self::Compound(
            shapes
                .into_iter()
                .map(|(pos, rot, constructor)| (pos.into(), rot.into(), constructor))
                .collect(),
        )
    }

    pub(crate) fn flatten_compound_constructors(
        constructors: Vec<(Position, Rotation, ColliderConstructor)>,
    ) -> Vec<(Position, Rotation, ColliderConstructor)> {
        constructors
            .into_iter()
            .flat_map(|(pos, rot, constructor)| match constructor {
                ColliderConstructor::Compound(nested) => {
                    Either::Left(Self::flatten_compound_constructors(nested).into_iter().map(
                        move |(nested_pos, nested_rot, nested_constructor)| {
                            (
                                Position(pos.0 + rot * nested_pos.0),
                                rot * nested_rot,
                                nested_constructor,
                            )
                        },
                    ))
                }
                other => Either::Right(once((pos, rot, other))),
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(feature = "bevy_scene")]
    use bevy::scene::ScenePlugin;
    use bevy::{ecs::query::QueryData, mesh::MeshPlugin};

    #[test]
    fn collider_constructor_requires_no_mesh_on_primitive() {
        let mut app = create_test_app();

        let entity = app.world_mut().spawn(PRIMITIVE_COLLIDER.clone()).id();

        app.update();

        assert!(app.query_ok::<&Collider>(entity));
        assert!(app.query_err::<&ColliderConstructor>(entity));
    }

    #[cfg(feature = "collider-from-mesh")]
    #[test]
    #[should_panic]
    fn collider_constructor_requires_mesh_on_computed() {
        let mut app = create_test_app();

        app.world_mut().spawn(COMPUTED_COLLIDER.clone());

        app.update();
    }

    #[cfg(feature = "collider-from-mesh")]
    #[test]
    fn collider_constructor_converts_mesh_on_computed() {
        let mut app = create_test_app();

        let mesh = app.add_mesh();
        let entity = app
            .world_mut()
            .spawn((COMPUTED_COLLIDER.clone(), Mesh3d(mesh)))
            .id();

        app.update();

        assert!(app.query_ok::<&Collider>(entity));
        assert!(app.query_ok::<&Mesh3d>(entity));
        assert!(app.query_err::<&ColliderConstructor>(entity));
    }

    #[test]
    fn collider_constructor_hierarchy_does_nothing_on_self_with_primitive() {
        let mut app = create_test_app();

        let entity = app
            .world_mut()
            .spawn(ColliderConstructorHierarchy::new(
                PRIMITIVE_COLLIDER.clone(),
            ))
            .id();

        app.update();

        assert!(app.query_err::<&ColliderConstructorHierarchy>(entity));
        assert!(app.query_err::<&Collider>(entity));
    }

    #[cfg(feature = "collider-from-mesh")]
    #[test]
    fn collider_constructor_hierarchy_does_nothing_on_self_with_computed() {
        let mut app = create_test_app();

        let mesh = app.add_mesh();
        let entity = app
            .world_mut()
            .spawn((
                ColliderConstructorHierarchy::new(COMPUTED_COLLIDER.clone()),
                Mesh3d(mesh),
            ))
            .id();

        app.update();

        assert!(app.query_ok::<&Mesh3d>(entity));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(entity));
        assert!(app.query_err::<&Collider>(entity));
    }

    #[cfg(feature = "collider-from-mesh")]
    #[test]
    fn collider_constructor_hierarchy_does_not_require_mesh_on_self_with_computed() {
        let mut app = create_test_app();

        let entity = app
            .world_mut()
            .spawn(ColliderConstructorHierarchy::new(COMPUTED_COLLIDER.clone()))
            .id();

        app.update();

        assert!(app.query_err::<&Collider>(entity));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(entity));
    }

    #[test]
    fn collider_constructor_hierarchy_inserts_primitive_colliders_on_all_descendants() {
        let mut app = create_test_app();

        // Hierarchy:
        // - parent
        //   - child1
        //   - child2
        //     - child3

        let parent = app
            .world_mut()
            .spawn(ColliderConstructorHierarchy::new(
                PRIMITIVE_COLLIDER.clone(),
            ))
            .id();
        let child1 = app.world_mut().spawn(()).id();
        let child2 = app.world_mut().spawn(()).id();
        let child3 = app.world_mut().spawn(()).id();

        app.world_mut()
            .entity_mut(parent)
            .add_children(&[child1, child2]);
        app.world_mut().entity_mut(child2).add_children(&[child3]);

        app.update();

        // No entities should have ColliderConstructorHierarchy
        assert!(app.query_err::<&ColliderConstructorHierarchy>(parent));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child1));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child2));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child3));

        assert!(app.query_err::<&Collider>(parent));
        assert!(app.query_ok::<&Collider>(child1));
        assert!(app.query_ok::<&Collider>(child2));
        assert!(app.query_ok::<&Collider>(child3));
    }

    #[cfg(feature = "collider-from-mesh")]
    #[test]
    fn collider_constructor_hierarchy_inserts_computed_colliders_only_on_descendants_with_mesh() {
        let mut app = create_test_app();
        let mesh = Mesh3d(app.add_mesh());

        // Hierarchy:
        // - parent
        //   - child1 (no mesh)
        //   - child2 (no mesh)
        //     - child3 (mesh)
        //   - child4 (mesh)
        //     - child5 (no mesh)
        //   - child6 (mesh)
        //   - child7 (mesh)
        //     - child8 (mesh)

        let parent = app
            .world_mut()
            .spawn(ColliderConstructorHierarchy::new(COMPUTED_COLLIDER.clone()))
            .id();
        let child1 = app.world_mut().spawn(()).id();
        let child2 = app.world_mut().spawn(()).id();
        let child3 = app.world_mut().spawn(mesh.clone()).id();
        let child4 = app.world_mut().spawn(mesh.clone()).id();
        let child5 = app.world_mut().spawn(()).id();
        let child6 = app.world_mut().spawn(mesh.clone()).id();
        let child7 = app.world_mut().spawn(mesh.clone()).id();
        let child8 = app.world_mut().spawn(mesh.clone()).id();

        app.world_mut()
            .entity_mut(parent)
            .add_children(&[child1, child2, child4, child6, child7]);
        app.world_mut().entity_mut(child2).add_child(child3);
        app.world_mut().entity_mut(child4).add_child(child5);
        app.world_mut().entity_mut(child7).add_child(child8);

        app.update();

        // No entities should have ColliderConstructorHierarchy
        assert!(app.query_err::<&ColliderConstructorHierarchy>(parent));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child1));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child2));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child3));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child4));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child5));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child6));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child7));
        assert!(app.query_err::<&ColliderConstructorHierarchy>(child8));

        assert!(app.query_err::<&Collider>(parent));
        assert!(app.query_err::<&Collider>(child1));
        assert!(app.query_err::<&Collider>(child2));
        assert!(app.query_ok::<&Collider>(child3));
        assert!(app.query_ok::<&Collider>(child4));
        assert!(app.query_err::<&Collider>(child5));
        assert!(app.query_ok::<&Collider>(child6));
        assert!(app.query_ok::<&Collider>(child7));
        assert!(app.query_ok::<&Collider>(child8));
    }

    #[cfg(all(feature = "collider-from-mesh", feature = "bevy_scene"))]
    #[test]
    #[cfg_attr(
        target_os = "linux",
        ignore = "The plugin setup requires access to the GPU, which is not available in the linux test environment"
    )]
    fn collider_constructor_hierarchy_inserts_correct_configs_on_scene() {
        use bevy::gltf::GltfMeshName;
        use parry::shape::ShapeType;

        #[derive(Resource)]
        struct SceneReady;

        let mut app = create_gltf_test_app();

        app.add_observer(
            |_trigger: On<bevy::scene::SceneInstanceReady>, mut commands: Commands| {
                commands.insert_resource(SceneReady);
            },
        );

        let scene_handle = app
            .world_mut()
            .resource_mut::<AssetServer>()
            .load("ferris.glb#Scene0");

        let hierarchy = app
            .world_mut()
            .spawn((
                SceneRoot(scene_handle),
                ColliderConstructorHierarchy::new(ColliderConstructor::ConvexDecompositionFromMesh)
                    // Use a primitive collider for the left arm.
                    .with_constructor_for_name("armL_mesh.ferris_material", PRIMITIVE_COLLIDER)
                    .with_density_for_name("armL_mesh.ferris_material", 2.0)
                    // Remove the right arm. Don't worry, crabs can regrow lost limbs!
                    .without_constructor_for_name("armR_mesh.ferris_material"),
                RigidBody::Dynamic,
            ))
            .id();

        let mut counter = 0;
        loop {
            if app.world().contains_resource::<SceneReady>() {
                break;
            }
            app.update();
            counter += 1;
            if counter > 1000 {
                panic!("SceneInstanceReady was never triggered");
            }
        }
        app.update();

        assert!(app.query_err::<&ColliderConstructorHierarchy>(hierarchy));
        assert!(app.query_err::<&Collider>(hierarchy));

        // Check densities
        let densities: HashMap<_, _> = app
            .world_mut()
            .query::<(&GltfMeshName, &ColliderDensity)>()
            .iter(app.world())
            .map(|(name, density)| (name.to_string(), density.0))
            .collect();

        assert_eq!(densities["eyes_mesh"], 1.0);
        assert_eq!(densities["armL_mesh"], 2.0);
        assert!(densities.get("armR_mesh").is_none());

        // Check collider shape types
        let colliders: HashMap<_, _> = app
            .world_mut()
            .query::<(&GltfMeshName, &Collider)>()
            .iter(app.world())
            .map(|(name, collider)| (name.to_string(), collider))
            .collect();

        assert_eq!(
            colliders["eyes_mesh"].shape().shape_type(),
            ShapeType::Compound
        );
        assert_eq!(
            colliders["armL_mesh"].shape().shape_type(),
            ShapeType::Capsule
        );
        assert!(colliders.get("armR_mesh").is_none());
    }

    const PRIMITIVE_COLLIDER: ColliderConstructor = ColliderConstructor::Capsule {
        height: 1.0,
        radius: 0.5,
    };

    #[cfg(feature = "collider-from-mesh")]
    const COMPUTED_COLLIDER: ColliderConstructor = ColliderConstructor::TrimeshFromMesh;

    fn create_test_app() -> App {
        let mut app = App::new();
        app.add_plugins((
            MinimalPlugins,
            AssetPlugin::default(),
            #[cfg(feature = "bevy_scene")]
            ScenePlugin,
            MeshPlugin,
            PhysicsPlugins::default(),
        ));

        app
    }

    #[cfg(all(feature = "collider-from-mesh", feature = "bevy_scene"))]
    fn create_gltf_test_app() -> App {
        use bevy::{diagnostic::DiagnosticsPlugin, winit::WinitPlugin};

        // Todo: it would be best to disable all rendering-related plugins,
        // but we have so far not succeeded in finding the right plugin combination
        // that still results in `SceneInstanceReady` being triggered.
        let mut app = App::new();
        app.add_plugins((
            DefaultPlugins
                .build()
                .disable::<WinitPlugin>()
                .disable::<DiagnosticsPlugin>(),
            PhysicsPlugins::default(),
        ));
        app.finish();
        app.cleanup();
        app
    }

    trait AppExt {
        fn query_ok<D: QueryData>(&mut self, entity: Entity) -> bool;
        fn query_err<D: QueryData>(&mut self, entity: Entity) -> bool {
            !self.query_ok::<D>(entity)
        }

        #[cfg(feature = "collider-from-mesh")]
        fn add_mesh(&mut self) -> Handle<Mesh>;
    }

    impl AppExt for App {
        fn query_ok<D: QueryData>(&mut self, entity: Entity) -> bool {
            let mut query = self.world_mut().query::<D>();
            let component = query.get(self.world(), entity);
            component.is_ok()
        }

        #[cfg(feature = "collider-from-mesh")]
        fn add_mesh(&mut self) -> Handle<Mesh> {
            self.world_mut()
                .get_resource_mut::<Assets<Mesh>>()
                .unwrap()
                .add(Mesh::from(Cuboid::default()))
        }
    }
}
