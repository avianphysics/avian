use crate::prelude::*;
use bevy::{color::palettes::css::*, prelude::*};

/// Gizmos used for debug rendering physics. See [`PhysicsDebugPlugin`]
///
/// You can configure what is rendered by inserting the [`PhysicsGizmos`] configuration group
/// or by getting it from the `GizmoConfigStore` resource and mutating the returned configuration.
///
/// To configure the debug rendering of specific entities, use the [`DebugRender`] component.
///
/// # Example
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn main() {
///     App::new()
///         .add_plugins((
///             DefaultPlugins,
///             PhysicsPlugins::default(),
///             // Enables debug rendering
///             PhysicsDebugPlugin,
///         ))
///         // Overwrite default debug rendering configuration (optional)
///         .insert_gizmo_config(
///             PhysicsGizmos {
///                 aabb_color: Some(Color::WHITE),
///                 ..default()
///             },
///             GizmoConfig::default(),
///         )
///         .run();
/// }
///
/// fn setup(mut commands: Commands) {
///     // This rigid body and its collider and AABB will get rendered
#[cfg_attr(
    feature = "2d",
    doc = "    commands.spawn((RigidBody::Dynamic, Collider::circle(0.5)));"
)]
#[cfg_attr(
    feature = "3d",
    doc = "    commands.spawn((RigidBody::Dynamic, Collider::sphere(0.5)));"
)]
/// }
/// ```
#[derive(Reflect, GizmoConfigGroup)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
pub struct PhysicsGizmos {
    /// The lengths of the axes drawn for an entity at the center of mass.
    pub axis_lengths: Option<Vector>,
    /// The color of the [AABBs](ColliderAabb). If `None`, the AABBs will not be rendered.
    pub aabb_color: Option<Color>,
    /// The color of the [collider](Collider) wireframes. If `None`, the colliders will not be rendered.
    pub collider_color: Option<Color>,
    /// The colors (in HSLA) for [sleeping](Sleeping) bodies will be multiplied by this array.
    /// If `None`, sleeping will have no effect on the colors.
    pub sleeping_color_multiplier: Option<[f32; 4]>,
    /// The color of the contact points. If `None`, the contact points will not be rendered.
    pub contact_point_color: Option<Color>,
    /// The color of the contact normals. If `None`, the contact normals will not be rendered.
    pub contact_normal_color: Option<Color>,
    /// The scale used for contact normals.
    pub contact_normal_scale: ContactGizmoScale,
    /// The color of the lines drawn from the centers of bodies to their joint anchors.
    pub joint_anchor_color: Option<Color>,
    /// The color of the lines drawn between joint anchors, indicating the separation.
    pub joint_separation_color: Option<Color>,
    /// The color used for the rays in [raycasts](spatial_query#raycasting).
    pub raycast_color: Option<Color>,
    /// The color used for the hit points in [raycasts](spatial_query#raycasting).
    pub raycast_point_color: Option<Color>,
    /// The color used for the hit normals in [raycasts](spatial_query#raycasting).
    pub raycast_normal_color: Option<Color>,
    /// The color used for the ray in [shapecasts](spatial_query#shapecasting).
    pub shapecast_color: Option<Color>,
    /// The color used for the shape in [shapecasts](spatial_query#shapecasting).
    pub shapecast_shape_color: Option<Color>,
    /// The color used for the hit points in [shapecasts](spatial_query#shapecasting).
    pub shapecast_point_color: Option<Color>,
    /// The color used for the hit normals in [shapecasts](spatial_query#shapecasting).
    pub shapecast_normal_color: Option<Color>,
    /// The color used for the bounds of [`PhysicsIsland`](dynamics::solver::islands::PhysicsIsland)s.
    pub island_color: Option<Color>,
    /// Determines if the visibility of entities with [colliders](Collider).
    pub mesh_visibility: MeshVisibility,
}

impl Default for PhysicsGizmos {
    fn default() -> Self {
        Self {
            axis_lengths: Some(Vector::splat(0.5)),
            aabb_color: None,
            collider_color: Some(ORANGE.into()),
            sleeping_color_multiplier: Some([1.0, 1.0, 0.4, 1.0]),
            contact_point_color: None,
            contact_normal_color: None,
            contact_normal_scale: ContactGizmoScale::default(),
            joint_anchor_color: Some(PINK.into()),
            joint_separation_color: Some(RED.into()),
            raycast_color: Some(RED.into()),
            raycast_point_color: Some(YELLOW.into()),
            raycast_normal_color: Some(PINK.into()),
            shapecast_color: Some(RED.into()),
            shapecast_shape_color: Some(Color::srgb(0.4, 0.6, 1.0)),
            shapecast_point_color: Some(YELLOW.into()),
            shapecast_normal_color: Some(PINK.into()),
            island_color: None,
            mesh_visibility: MeshVisibility::Ignore,
        }
    }
}

/// The scale used for contact normals rendered using gizmos.
#[derive(Reflect, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(PartialEq)]
pub enum ContactGizmoScale {
    /// The length of the rendered contact normal is constant.
    Constant(Scalar),
    /// The length of the rendered contact normal is scaled by the contact force and the given scaling factor.
    Scaled(Scalar),
}

impl Default for ContactGizmoScale {
    fn default() -> Self {
        Self::Scaled(0.025)
    }
}

/// Determines if the visibility of entities with [colliders](Collider) should
/// be overwritten. Setting this to `MeshVisibility::Overwrite(Visibility::Hidden)`,
/// will only show the debug renders.
#[derive(Reflect, Clone, Copy, PartialEq, Default)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(PartialEq)]
pub enum MeshVisibility {
    /// Do not change the visibility of the entity's mesh.
    #[default]
    Ignore,
    /// Always overwrite the visibility of the entity's mesh.
    Overwrite(Visibility),
}

impl MeshVisibility {
    /// Returns the first non-ignore visibility.
    pub fn or(self, other: Self) -> Self {
        if self == Self::Ignore { other } else { self }
    }
}

impl PhysicsGizmos {
    /// Creates a [`PhysicsGizmos`] configuration with all rendering options enabled.
    pub fn all() -> Self {
        Self {
            axis_lengths: Some(Vector::splat(0.5)),
            aabb_color: Some(Color::srgb(0.8, 0.8, 0.8)),
            collider_color: Some(ORANGE.into()),
            sleeping_color_multiplier: Some([1.0, 1.0, 0.4, 1.0]),
            contact_point_color: Some(LIGHT_CYAN.into()),
            contact_normal_color: Some(RED.into()),
            contact_normal_scale: ContactGizmoScale::default(),
            joint_anchor_color: Some(PINK.into()),
            joint_separation_color: Some(RED.into()),
            raycast_color: Some(RED.into()),
            raycast_point_color: Some(YELLOW.into()),
            raycast_normal_color: Some(PINK.into()),
            shapecast_color: Some(RED.into()),
            shapecast_shape_color: Some(Color::srgb(0.4, 0.6, 1.0)),
            shapecast_point_color: Some(YELLOW.into()),
            shapecast_normal_color: Some(PINK.into()),
            island_color: Some(RED.into()),
            mesh_visibility: MeshVisibility::Overwrite(Visibility::Hidden),
        }
    }

    /// Creates a [`PhysicsGizmos`] configuration with debug rendering enabled but all options turned off.
    ///
    /// Note: this doesn't affect entities with [`DebugRender`] component; their debug gizmos will be visible.
    pub fn none() -> Self {
        Self {
            axis_lengths: None,
            aabb_color: None,
            collider_color: None,
            sleeping_color_multiplier: None,
            contact_point_color: None,
            contact_normal_color: None,
            contact_normal_scale: ContactGizmoScale::default(),
            joint_anchor_color: None,
            joint_separation_color: None,
            raycast_color: None,
            raycast_point_color: None,
            raycast_normal_color: None,
            shapecast_color: None,
            shapecast_shape_color: None,
            shapecast_point_color: None,
            shapecast_normal_color: None,
            island_color: None,
            mesh_visibility: MeshVisibility::Ignore,
        }
    }

    /// Creates a [`PhysicsGizmos`] configuration with the given lengths for the axes
    /// that are drawn for the entity at the center of mass. Other debug rendering options will be disabled.
    pub fn axes(axis_lengths: Vector) -> Self {
        Self {
            axis_lengths: Some(axis_lengths),
            ..Self::none()
        }
    }

    /// Creates a [`PhysicsGizmos`] configuration with a given AABB color.
    /// Other debug rendering options will be disabled.
    pub fn aabbs(color: Color) -> Self {
        Self {
            aabb_color: Some(color),
            ..Self::none()
        }
    }

    /// Creates a [`PhysicsGizmos`] configuration with a given collider color.
    /// Other debug rendering options will be disabled.
    pub fn colliders(color: Color) -> Self {
        Self {
            collider_color: Some(color),
            ..Self::none()
        }
    }

    /// Creates a [`PhysicsGizmos`] configuration with a given contact point color.
    /// Other debug rendering options will be disabled.
    pub fn contact_points(color: Color) -> Self {
        Self {
            contact_point_color: Some(color),
            ..Self::none()
        }
    }

    /// Creates a [`PhysicsGizmos`] configuration with a given contact normal color.
    /// Other debug rendering options will be disabled.
    pub fn contact_normals(color: Color) -> Self {
        Self {
            contact_normal_color: Some(color),
            ..Self::none()
        }
    }

    /// Creates a [`PhysicsGizmos`] configuration with given colors for
    /// joint anchors and separation distances. Other debug rendering options will be disabled.
    pub fn joints(anchor_color: Option<Color>, separation_color: Option<Color>) -> Self {
        Self {
            joint_anchor_color: anchor_color,
            joint_separation_color: separation_color,
            ..Self::none()
        }
    }

    /// Sets the lengths of the axes drawn for the entity.
    pub fn with_axes(mut self, axis_lengths: Vector) -> Self {
        self.axis_lengths = Some(axis_lengths);
        self
    }

    /// Sets the AABB color.
    pub fn with_aabb_color(mut self, color: Color) -> Self {
        self.aabb_color = Some(color);
        self
    }

    /// Sets the collider color.
    pub fn with_collider_color(mut self, color: Color) -> Self {
        self.collider_color = Some(color);
        self
    }

    /// Sets the multiplier used for the colors (in HSLA) of [sleeping](Sleeping) bodies.
    pub fn with_sleeping_color_multiplier(mut self, color_multiplier: [f32; 4]) -> Self {
        self.sleeping_color_multiplier = Some(color_multiplier);
        self
    }

    /// Sets the contact point color.
    pub fn with_contact_point_color(mut self, color: Color) -> Self {
        self.contact_point_color = Some(color);
        self
    }

    /// Sets the contact normal color.
    pub fn with_contact_normal_color(mut self, color: Color) -> Self {
        self.contact_normal_color = Some(color);
        self
    }

    /// Sets the contact normal scale.
    pub fn with_contact_normal_scale(mut self, scale: ContactGizmoScale) -> Self {
        self.contact_normal_scale = scale;
        self
    }

    /// Sets the colors used for debug rendering joints.
    pub fn with_joint_colors(anchor_color: Option<Color>, separation_color: Option<Color>) -> Self {
        Self {
            joint_anchor_color: anchor_color,
            joint_separation_color: separation_color,
            ..Self::none()
        }
    }

    /// Sets the colors used for debug rendering raycasts.
    pub fn with_raycast_colors(
        mut self,
        ray: Option<Color>,
        hit_point: Option<Color>,
        hit_normal: Option<Color>,
    ) -> Self {
        self.raycast_color = ray;
        self.raycast_point_color = hit_point;
        self.raycast_normal_color = hit_normal;
        self
    }

    /// Sets the colors used for debug rendering shapecasts.
    pub fn with_shapecast_colors(
        mut self,
        ray: Option<Color>,
        shape: Option<Color>,
        hit_point: Option<Color>,
        hit_normal: Option<Color>,
    ) -> Self {
        self.shapecast_color = ray;
        self.shapecast_shape_color = shape;
        self.shapecast_point_color = hit_point;
        self.shapecast_normal_color = hit_normal;
        self
    }

    /// Sets the visibility of the entity's visual mesh.
    pub fn with_mesh_visibility(mut self, visibility: MeshVisibility) -> Self {
        self.mesh_visibility = visibility;
        self
    }

    /// Disables axis debug rendering.
    pub fn without_axes(mut self) -> Self {
        self.axis_lengths = None;
        self
    }

    /// Disables AABB debug rendering.
    pub fn without_aabbs(mut self) -> Self {
        self.aabb_color = None;
        self
    }

    /// Disables collider debug rendering.
    pub fn without_colliders(mut self) -> Self {
        self.collider_color = None;
        self
    }

    /// Disables contact point debug rendering.
    pub fn without_contact_points(mut self) -> Self {
        self.contact_point_color = None;
        self
    }

    /// Disables contact normal debug rendering.
    pub fn without_contact_normals(mut self) -> Self {
        self.contact_normal_color = None;
        self
    }

    /// Disables joint debug rendering.
    pub fn without_joints(mut self) -> Self {
        self.joint_anchor_color = None;
        self.joint_separation_color = None;
        self
    }

    /// Disables raycast debug rendering.
    pub fn without_raycasts(mut self) -> Self {
        self.raycast_color = None;
        self.raycast_point_color = None;
        self.raycast_normal_color = None;
        self
    }

    /// Disables shapecast debug rendering.
    pub fn without_shapecasts(mut self) -> Self {
        self.shapecast_color = None;
        self.shapecast_shape_color = None;
        self.shapecast_point_color = None;
        self.shapecast_normal_color = None;
        self
    }
}

/// A component for the debug render configuration of an entity. See [`PhysicsDebugPlugin`].
///
/// This overwrites the global [`PhysicsGizmos`] for this specific entity.
///
/// # Example
///
/// ```no_run
#[cfg_attr(feature = "2d", doc = "use avian2d::prelude::*;")]
#[cfg_attr(feature = "3d", doc = "use avian3d::prelude::*;")]
/// use bevy::prelude::*;
///
/// fn main() {
///     App::new()
///         .add_plugins((
///             DefaultPlugins,
///             PhysicsPlugins::default(),
///             // Enables debug rendering
///             PhysicsDebugPlugin,
///         ))
///         .run();
/// }
///
/// fn setup(mut commands: Commands) {
///     // This rigid body and its collider and AABB will get rendered
///     commands.spawn((
///         RigidBody::Dynamic,
#[cfg_attr(feature = "2d", doc = "        Collider::circle(0.5),")]
#[cfg_attr(feature = "3d", doc = "        Collider::sphere(0.5),")]
///         // Overwrite default collider color (optional)
///         DebugRender::default().with_collider_color(Color::srgb(1.0, 0.0, 0.0)),
///     ));
/// }
/// ```
#[derive(Component, Reflect, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serialize", derive(serde::Serialize, serde::Deserialize))]
#[cfg_attr(feature = "serialize", reflect(Serialize, Deserialize))]
#[reflect(Component, PartialEq)]
pub struct DebugRender {
    /// The lengths of the axes drawn for the entity at the center of mass.
    pub axis_lengths: Option<Vector>,
    /// The color of the [AABB](ColliderAabb). If `None`, the AABB will not be rendered.
    pub aabb_color: Option<Color>,
    /// The color of the [collider](Collider) wireframe. If `None`, the collider will not be rendered.
    pub collider_color: Option<Color>,
    /// If the entity is [sleeping](Sleeping), its colors (in HSLA) will be multiplied by this array.
    /// If `None`, sleeping will have no effect on the colors.
    pub sleeping_color_multiplier: Option<[f32; 4]>,
    /// Determines if the entity's visibility should be overwritten.
    pub mesh_visibility: MeshVisibility,
}

impl Default for DebugRender {
    fn default() -> Self {
        Self {
            #[cfg(feature = "2d")]
            axis_lengths: Some(Vector::new(5.0, 5.0)),
            #[cfg(feature = "3d")]
            axis_lengths: Some(Vector::new(0.5, 0.5, 0.5)),
            aabb_color: None,
            collider_color: Some(ORANGE.into()),
            sleeping_color_multiplier: Some([1.0, 1.0, 0.4, 1.0]),
            mesh_visibility: MeshVisibility::Ignore,
        }
    }
}

impl DebugRender {
    /// Creates a [`DebugRender`] configuration with all rendering options enabled.
    pub fn all() -> Self {
        Self {
            #[cfg(feature = "2d")]
            axis_lengths: Some(Vector::new(5.0, 5.0)),
            #[cfg(feature = "3d")]
            axis_lengths: Some(Vector::new(0.5, 0.5, 0.5)),
            aabb_color: Some(Color::srgb(0.8, 0.8, 0.8)),
            collider_color: Some(ORANGE.into()),
            sleeping_color_multiplier: Some([1.0, 1.0, 0.4, 1.0]),
            mesh_visibility: MeshVisibility::Overwrite(Visibility::Hidden),
        }
    }

    /// Disables all debug rendering for this entity.
    pub fn none() -> Self {
        Self {
            axis_lengths: None,
            aabb_color: None,
            collider_color: None,
            sleeping_color_multiplier: None,
            mesh_visibility: MeshVisibility::Ignore,
        }
    }

    /// Creates a [`DebugRender`] configuration with the given lengths for the axes
    /// that are drawn for the entity at the center of mass. Other debug rendering options will be disabled.
    pub fn axes(axis_lengths: Vector) -> Self {
        Self {
            axis_lengths: Some(axis_lengths),
            ..Self::none()
        }
    }

    /// Creates a [`DebugRender`] configuration with a given AABB color.
    /// Other debug rendering options will be disabled.
    pub fn aabb(color: Color) -> Self {
        Self {
            aabb_color: Some(color),
            ..Self::none()
        }
    }

    /// Creates a [`DebugRender`] configuration with a given collider color.
    /// Other debug rendering options will be disabled.
    pub fn collider(color: Color) -> Self {
        Self {
            collider_color: Some(color),
            ..Self::none()
        }
    }

    /// Sets the lengths of the axes drawn for the entity at the center of mass.
    pub fn with_axes(mut self, axis_lengths: Vector) -> Self {
        self.axis_lengths = Some(axis_lengths);
        self
    }

    /// Sets the AABB color.
    pub fn with_aabb_color(mut self, color: Color) -> Self {
        self.aabb_color = Some(color);
        self
    }

    /// Sets the collider color.
    pub fn with_collider_color(mut self, color: Color) -> Self {
        self.collider_color = Some(color);
        self
    }

    /// Sets the multiplier used for the colors (in HSLA) of [sleeping](Sleeping) bodies.
    pub fn with_sleeping_color_multiplier(mut self, color_multiplier: [f32; 4]) -> Self {
        self.sleeping_color_multiplier = Some(color_multiplier);
        self
    }

    /// Sets the visibility of the entity's visual mesh.
    pub fn with_mesh_visibility(mut self, visibility: MeshVisibility) -> Self {
        self.mesh_visibility = visibility;
        self
    }

    /// Disables axis debug rendering.
    pub fn without_axes(mut self) -> Self {
        self.axis_lengths = None;
        self
    }

    /// Disables AABB debug rendering.
    pub fn without_aabb(mut self) -> Self {
        self.aabb_color = None;
        self
    }

    /// Disables collider debug rendering.
    pub fn without_collider(mut self) -> Self {
        self.collider_color = None;
        self
    }
}
