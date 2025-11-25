use bevy::{diagnostic::DiagnosticPath, prelude::*};

use crate::{ColliderMarker, PhysicsSchedule, PhysicsStepSystems, RigidBody, dynamics::joints::*};

use super::{AppDiagnosticsExt, PhysicsDiagnostics, impl_diagnostic_paths};

/// A plugin that adds diagnostics for physics entity counts.
pub struct PhysicsEntityDiagnosticsPlugin;

impl Plugin for PhysicsEntityDiagnosticsPlugin {
    fn build(&self, app: &mut App) {
        // Register diagnostics for physics entity counts.
        // NOTE: This should be done after the `PhysicsDiagnosticsPlugin` is added.
        app.register_physics_diagnostics::<PhysicsEntityDiagnostics>();

        app.add_systems(
            PhysicsSchedule,
            diagnostic_entity_counts.in_set(PhysicsStepSystems::First),
        );
    }
}

/// Diagnostics for physics entity counts.
#[derive(Resource, Debug, Default, Reflect)]
#[reflect(Resource, Debug)]
pub struct PhysicsEntityDiagnostics {
    /// The number of dynamic bodies.
    pub dynamic_body_count: u32,
    /// The number of kinematic bodies.
    pub kinematic_body_count: u32,
    /// The number of static bodies.
    pub static_body_count: u32,
    /// The number of colliders.
    pub collider_count: u32,
    /// The number of joint.
    pub joint_count: u32,
}

impl PhysicsDiagnostics for PhysicsEntityDiagnostics {
    fn counter_paths(&self) -> Vec<(&'static DiagnosticPath, u32)> {
        vec![
            (Self::DYNAMIC_BODY_COUNT, self.dynamic_body_count),
            (Self::KINEMATIC_BODY_COUNT, self.kinematic_body_count),
            (Self::STATIC_BODY_COUNT, self.static_body_count),
            (Self::COLLIDER_COUNT, self.collider_count),
            (Self::JOINT_COUNT, self.joint_count),
        ]
    }
}

impl_diagnostic_paths! {
    impl PhysicsEntityDiagnostics {
        DYNAMIC_BODY_COUNT: "avian/entity_count/dynamic_bodies",
        KINEMATIC_BODY_COUNT: "avian/entity_count/kinematic_bodies",
        STATIC_BODY_COUNT: "avian/entity_count/static_bodies",
        COLLIDER_COUNT: "avian/entity_count/colliders",
        JOINT_COUNT: "avian/entity_count/joints",
    }
}

// TODO: This is pretty inefficient.
fn diagnostic_entity_counts(
    rigid_bodies_query: Query<&RigidBody>,
    colliders_query: Query<&ColliderMarker>,
    fixed_joint_query: Query<&FixedJoint>,
    prismatic_joint_query: Query<&PrismaticJoint>,
    distance_joint_query: Query<&DistanceJoint>,
    revolute_joint_query: Query<&RevoluteJoint>,
    #[cfg(feature = "3d")] spherical_joint_query: Query<&SphericalJoint>,
    mut diagnostics: ResMut<PhysicsEntityDiagnostics>,
) {
    diagnostics.dynamic_body_count = rigid_bodies_query
        .iter()
        .filter(|rb| rb.is_dynamic())
        .count() as u32;
    diagnostics.kinematic_body_count = rigid_bodies_query
        .iter()
        .filter(|rb| rb.is_kinematic())
        .count() as u32;
    diagnostics.static_body_count = rigid_bodies_query
        .iter()
        .filter(|rb| rb.is_static())
        .count() as u32;
    diagnostics.collider_count = colliders_query.iter().count() as u32;
    diagnostics.joint_count = fixed_joint_query.iter().count() as u32
        + prismatic_joint_query.iter().count() as u32
        + distance_joint_query.iter().count() as u32
        + revolute_joint_query.iter().count() as u32;
    #[cfg(feature = "3d")]
    {
        diagnostics.joint_count += spherical_joint_query.iter().count() as u32;
    }
}
