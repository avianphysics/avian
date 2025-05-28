use avian3d::{
    dynamics::solver::{
        schedule::SubstepSolverSet,
        solver_body::{SolverBody, SolverBodyInertia},
        xpbd::*,
    },
    math::*,
    prelude::*,
};
use bevy::{
    ecs::entity::{EntityMapper, MapEntities},
    prelude::*,
};
use examples_common_3d::ExampleCommonPlugin;

fn main() {
    let mut app = App::new();

    // Add plugins and startup system
    app.add_plugins((
        DefaultPlugins,
        ExampleCommonPlugin,
        PhysicsPlugins::default(),
    ))
    .add_systems(Startup, setup);

    app.add_systems(
        PhysicsSchedule,
        prepare_xpbd_joint::<CenterDistanceConstraint>
            .in_set(SolverSet::PreSubstep)
            .ambiguous_with_all(),
    );

    // Get physics substep schedule and add our custom distance constraint
    let substeps = app
        .get_schedule_mut(SubstepSchedule)
        .expect("add SubstepSchedule first");
    substeps.add_systems(
        solve_xpbd_joint::<CenterDistanceConstraint>.in_set(SubstepSolverSet::SolveUserConstraints),
    );

    // Run the app
    app.run();
}

/// A constraint that keeps the distance between the centers of two bodies at `rest_distance`.
#[derive(Component)]
struct CenterDistanceConstraint {
    entity1: Entity,
    entity2: Entity,
    rest_distance: Scalar,
    /// The relative dominance of the bodies.
    ///
    /// If the relative dominance is positive, the first body is dominant
    /// and is considered to have infinite mass.
    relative_dominance: i16,
    compliance: Scalar,
    /// Data stored from before the solver step.
    pre_step: PreStepData,
}

/// Pre-step data for the [`CenterDistanceConstraint`].
#[derive(Default)]
struct PreStepData {
    /// The world-space vector from the center of mass of body 1
    /// to the center of mass of body 2.
    center_difference: Vector,
}

impl CenterDistanceConstraint {
    /// Creates a new [`CenterDistanceConstraint`] with the given entities and rest distance.
    /// The compliance is set to `0.0` by default.
    pub fn new(entity1: Entity, entity2: Entity, rest_distance: Scalar) -> Self {
        Self {
            entity1,
            entity2,
            rest_distance,
            relative_dominance: 0,
            compliance: 0.0,
            pre_step: PreStepData::default(),
        }
    }
}

impl PositionConstraint for CenterDistanceConstraint {}
impl AngularConstraint for CenterDistanceConstraint {}

impl EntityConstraint<2> for CenterDistanceConstraint {
    fn entities(&self) -> [Entity; 2] {
        [self.entity1, self.entity2]
    }
}

impl XpbdConstraint<2> for CenterDistanceConstraint {
    fn prepare(&mut self, bodies: [&RigidBodyQueryReadOnlyItem; 2], _dt: Scalar) {
        let [body1, body2] = bodies;

        // Prepare the base center difference.
        // The solver will compute the updated version based on the position deltas of the bodies.
        self.pre_step.center_difference = body2.position.0 - body1.position.0;

        // Prepare the relative dominance.
        self.relative_dominance = body1.dominance() - body2.dominance();
    }

    // This method is called by the solver to actually solve the constraint.
    // The solver uses special `SolverBody` types that are optimized for performance and memory.
    // They only store the linear and angular velocities, position and rotation deltas, and some bitflags.
    // To compute up-to-date positional information, we apply the position and rotation deltas to the
    // pre-step data computed in the `prepare` method.
    fn solve(
        &mut self,
        bodies: [&mut SolverBody; 2],
        inertias: [&SolverBodyInertia; 2],
        dt: Scalar,
    ) {
        let [body1, body2] = bodies;
        let [inertia1, inertia2] = inertias;

        // Compute the effective inverse masses and angular inertias of the bodies.
        // These consider locked axes and dominance.
        let inv_mass1 = inertia1.effective_inv_mass();
        let inv_mass2 = inertia2.effective_inv_mass();
        let inv_angular_inertia1 = inertia1.effective_inv_angular_inertia();
        let inv_angular_inertia2 = inertia2.effective_inv_angular_inertia();

        // Compute the positional difference.
        let center_difference =
            body2.delta_position - body1.delta_position + self.pre_step.center_difference;

        // The current separation distance
        let distance = center_difference.length();

        // The value of the constraint function. When this is zero, the constraint is satisfied,
        // and the distance between the bodies is the rest length.
        let c = distance - self.rest_distance;

        // Avoid division by zero and unnecessary computation.
        if distance <= 0.0 || c == 0.0 {
            return;
        }

        // The opposite of the normalized center difference.
        // This is the gradient of the constraint function.
        let n = -center_difference / distance;

        // The world-space anchor points relative to the centers of mass of the bodies.
        // In this case, the offset from the center of mass is just zero.
        let r1 = Vector::ZERO;
        let r2 = Vector::ZERO;

        // Compute generalized inverse masses (method from PositionConstraint).
        // In this case, the offset from the center of mass is zero.
        let w1 = PositionConstraint::compute_generalized_inverse_mass(
            self,
            inv_mass1.max_element(),
            inv_angular_inertia1,
            r1,
            n,
        );
        let w2 = PositionConstraint::compute_generalized_inverse_mass(
            self,
            inv_mass2.max_element(),
            inv_angular_inertia2,
            r2,
            n,
        );

        // Compute the Lagrange multiplier update, essentially the signed magnitude of the correction.
        let lagrange = 0.0;
        let delta_lagrange =
            self.compute_lagrange_update(lagrange, c, &[w1, w2], self.compliance, dt);

        // Apply the positional correction (method from PositionConstraint).
        self.apply_positional_lagrange_update(
            body1,
            body2,
            inertia1,
            inertia2,
            delta_lagrange,
            n,
            r1,
            r2,
        );
    }
}

impl Joint for CenterDistanceConstraint {
    fn new(entity1: Entity, entity2: Entity) -> Self {
        Self::new(entity1, entity2, 0.0)
    }

    fn local_anchor_1(&self) -> Vector {
        Vector::ZERO
    }

    fn local_anchor_2(&self) -> Vector {
        Vector::ZERO
    }

    fn damping_linear(&self) -> Scalar {
        0.0
    }

    fn damping_angular(&self) -> Scalar {
        0.0
    }

    fn relative_dominance(&self) -> i16 {
        self.relative_dominance
    }
}

impl MapEntities for CenterDistanceConstraint {
    fn map_entities<M: EntityMapper>(&mut self, entity_mapper: &mut M) {
        self.entity1 = entity_mapper.get_mapped(self.entity1);
        self.entity2 = entity_mapper.get_mapped(self.entity2);
    }
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let cube_mesh = meshes.add(Cuboid::default());
    let cube_material = materials.add(Color::srgb(0.8, 0.7, 0.6));

    // Spawn a static cube and a dynamic cube that is outside of the rest length
    let static_cube = commands
        .spawn((
            Mesh3d(cube_mesh.clone()),
            MeshMaterial3d(cube_material.clone()),
            RigidBody::Static,
        ))
        .id();
    let dynamic_cube = commands
        .spawn((
            Mesh3d(cube_mesh),
            MeshMaterial3d(cube_material),
            Transform::from_xyz(3.0, 3.5, 0.0),
            RigidBody::Dynamic,
            MassPropertiesBundle::from_shape(&Cuboid::from_length(1.0), 1.0),
        ))
        .id();

    // Add a distance constraint to keep the cubes at a certain distance from each other.
    // The dynamic cube should swing around the static cube like a pendulum.
    commands.spawn(CenterDistanceConstraint::new(
        static_cube,
        dynamic_cube,
        2.5,
    ));

    // Light
    commands.spawn((
        PointLight {
            intensity: 2_000_000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));

    // Camera
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0.0, 0.0, 10.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
