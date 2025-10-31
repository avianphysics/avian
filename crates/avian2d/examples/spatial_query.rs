#![expect(clippy::unnecessary_cast)]

use avian2d::{
    collision::{
        collider::{PairContext, QueryCollider, QueryShapeCastHit, SingleContext},
        contact_types::PackedFeatureId,
    },
    math::*,
    prelude::*,
};
use bevy::{color::palettes::tailwind::GRAY_400, prelude::*, render::camera::ScalingMode};
use examples_common_2d::ExampleCommonPlugin;
use ops::FloatPow;
use rand::Rng;

fn main() {
    App::new()
        .add_plugins((
            DefaultPlugins,
            ExampleCommonPlugin,
            PhysicsPlugins::default()
                .with_length_unit(10.0)
                .build()
                .disable::<ColliderBackendPlugin<Collider>>()
                .disable::<NarrowPhasePlugin<Collider>>()
                .disable::<SpatialQueryPlugin<Collider>>(),
            PhysicsDebugPlugin::default(),
            ColliderBackendPlugin::<CircleCollider>::default(),
            NarrowPhasePlugin::<CircleCollider>::default(),
            SpatialQueryPlugin::<CircleCollider>::default(),
        ))
        .insert_gizmo_config(
            PhysicsGizmos::default().with_aabb_color(GRAY_400.into()),
            GizmoConfig {
                line: GizmoLineConfig {
                    width: 0.5,
                    ..default()
                },
                ..default()
            },
        )
        .add_systems(Startup, setup)
        .add_systems(FixedUpdate, move_random)
        .add_systems(Update, cast_ray)
        .run();
}

/// A simplified custom collider.
#[derive(Component, Clone, Copy, Debug, Reflect)]
struct CircleCollider {
    radius: Scalar,
    unscaled_radius: Scalar,
    scale: Scalar,
}

impl CircleCollider {
    fn new(radius: Scalar) -> Self {
        Self {
            radius,
            unscaled_radius: radius,
            scale: 1.0,
        }
    }
}

impl AnyCollider for CircleCollider {
    type Context = ();

    fn aabb_with_context(
        &self,
        position: Vector,
        _: impl Into<Rotation>,
        _: SingleContext<()>,
    ) -> ColliderAabb {
        ColliderAabb::new(position, Vector::splat(self.radius))
    }

    fn contact_manifolds_with_context(
        &self,
        other: &Self,
        position1: Vector,
        rotation1: impl Into<Rotation>,
        position2: Vector,
        _: impl Into<Rotation>,
        prediction_distance: Scalar,
        manifolds: &mut Vec<ContactManifold>,
        _: PairContext<()>,
    ) {
        // Clear the previous manifolds.
        manifolds.clear();

        let rotation1: Rotation = rotation1.into();

        let inv_rotation1 = rotation1.inverse();
        let delta_pos = inv_rotation1 * (position2 - position1);

        let distance_squared = delta_pos.length_squared();
        let sum_radius = self.radius + other.radius;

        if distance_squared < (sum_radius + prediction_distance).powi(2) {
            let local_normal1 = if distance_squared != 0.0 {
                delta_pos.normalize_or_zero()
            } else {
                Vector::X
            };
            let local_point1 = local_normal1 * self.radius;
            let normal = rotation1 * local_normal1;

            let separation = distance_squared.sqrt() - sum_radius;

            let point1 = rotation1 * local_point1;
            let anchor1 = point1 + normal * separation * 0.5;
            let anchor2 = anchor1 + (position1 - position2);
            let world_point = position1 + anchor1;

            let point = ContactPoint::new(anchor1, anchor2, world_point, -separation)
                .with_feature_ids(PackedFeatureId::face(0), PackedFeatureId::face(0));

            manifolds.push(ContactManifold::new([point], rotation1 * local_normal1));
        }
    }
}

impl QueryCollider for CircleCollider {
    type CastShape = Circle;
    type Shape = Circle;

    fn ray_hit(&self, ray: obvhs::ray::Ray, _: bool, _: SingleContext<()>) -> f32 {
        let offset = ray.origin;
        let projected = offset.dot(ray.direction);
        let closest_point = offset - projected * ray.direction;
        let distance_squared = self.radius.squared() - closest_point.length_squared();
        if distance_squared < 0.
            || ops::copysign(projected.squared(), -projected) < -distance_squared
        {
            f32::INFINITY
        } else {
            let toi = -projected - ops::sqrt(distance_squared);
            if toi > ray.tmax {
                f32::INFINITY
            } else {
                toi.max(0.)
            }
        }
    }

    fn ray_normal(&self, point: Vector, _: Dir2, _: bool, _: SingleContext<()>) -> Vec2 {
        point.normalize_or(Vec2::Y)
    }

    fn shape_cast(
        &self,
        shape: &Self::CastShape,
        _: Rotation,
        local_origin: Vec2,
        local_dir: Dir2,
        (tmin, tmax): (f32, f32),
        context: SingleContext<()>,
    ) -> Option<QueryShapeCastHit> {
        let mut c = self.clone();
        c.radius += shape.radius;
        let ray = obvhs::ray::Ray::new(
            local_origin.extend(0.).into(),
            local_dir.extend(0.).into(),
            tmin,
            tmax,
        );
        let hit = c.ray_hit(ray, true, context);
        if hit < f32::INFINITY {
            Some(QueryShapeCastHit {
                distance: hit,
                // TODO
                point: default(),
                normal: default(),
            })
        } else {
            None
        }
    }

    fn shape_intersection(
        &self,
        shape: &Self::CastShape,
        shape_rotation: Rotation,
        local_origin: Vec2,
        _: SingleContext<()>,
    ) -> bool {
        todo!()
    }

    fn closest_point(&self, point: Vec2, solid: bool, _: SingleContext<()>) -> Vec2 {
        todo!()
    }

    fn contains_point(&self, point: Vec2, _: SingleContext<()>) -> bool {
        todo!()
    }
}

// Implement mass computation for the collider shape.
// This is needed for physics to behave correctly.
impl ComputeMassProperties2d for CircleCollider {
    fn mass(&self, density: f32) -> f32 {
        // In 2D, the Z length is assumed to be `1.0`, so volume == area.
        let volume = core::f32::consts::PI * self.radius.powi(2) as f32;
        density * volume
    }

    fn unit_angular_inertia(&self) -> f32 {
        // Angular inertia for a circle, assuming a mass of `1.0`.
        self.radius.powi(2) as f32 / 2.0
    }

    fn center_of_mass(&self) -> Vec2 {
        Vec2::ZERO
    }
}

// Note: This circle collider only supports uniform scaling.
impl ScalableCollider for CircleCollider {
    fn scale(&self) -> Vector {
        Vector::splat(self.scale)
    }

    fn set_scale(&mut self, scale: Vector, _detail: u32) {
        // For non-unifprm scaling, this would need to be converted to an ellipse collider or a convex hull.
        self.scale = scale.max_element();
        self.radius = self.unscaled_radius * scale.max_element();
    }
}

const X_COUNT: i32 = 50;
const Y_COUNT: i32 = 30;
const PARTICLE_RADIUS: f32 = 5.;
const SPACING_FACTOR: f32 = 3.;

fn setup(mut commands: Commands) {
    commands.spawn((
        Camera2d,
        Projection::Orthographic(OrthographicProjection {
            scaling_mode: ScalingMode::FixedVertical {
                viewport_height: SPACING_FACTOR * PARTICLE_RADIUS * (Y_COUNT + 1) as f32,
            },
            ..OrthographicProjection::default_2d()
        }),
    ));

    for x in -X_COUNT / 2..=X_COUNT / 2 {
        for y in -Y_COUNT / 2..=Y_COUNT / 2 {
            commands.spawn((
                RigidBody::Kinematic,
                Transform::from_xyz(
                    x as f32 * SPACING_FACTOR * PARTICLE_RADIUS,
                    y as f32 * SPACING_FACTOR * PARTICLE_RADIUS,
                    0.0,
                ),
                CircleCollider::new(PARTICLE_RADIUS.adjust_precision()),
                CollisionLayers::new(LayerMask::DEFAULT, LayerMask::NONE),
            ));
        }
    }
}

fn move_random(window: Single<&Window>, mut query: Query<(&Position, &mut LinearVelocity)>) {
    let mut rng = rand::rng();
    for (pos, mut lin_vel) in query.iter_mut() {
        let max_y = (Y_COUNT + 1) as f32 * 0.5 * SPACING_FACTOR * PARTICLE_RADIUS;
        let aspect_ratio = window.resolution.width() / window.resolution.height();
        let out_of_x = pos.x.abs() > max_y * aspect_ratio - PARTICLE_RADIUS;
        let out_of_y = pos.y.abs() > max_y - PARTICLE_RADIUS;
        if out_of_x {
            lin_vel.x = -lin_vel.x.copysign(pos.x);
        }
        if out_of_y {
            lin_vel.y = -lin_vel.y.copysign(pos.y);
        }
        if rng.random::<f32>() < 0.15 {
            lin_vel.0 += Vec2::new(rng.random_range(-0.25..0.25), rng.random_range(-0.25..0.25));
        }
    }
}

fn cast_ray(mut gizmos: Gizmos, spatial_query: SpatialQuery<CircleCollider>, time: Res<Time>) {
    let t = time.elapsed_secs() * 0.1;
    let direction = Dir2::new_unchecked(Vec2::new(t.sin(), t.cos()));
    gizmos.circle_2d(Vec2::ZERO, 3., Color::WHITE);
    gizmos.line_2d(Vec2::ZERO, direction * 10000., Color::WHITE);
    spatial_query.shape_hits_callback(
        &Circle { radius: 7. },
        Vec2::ZERO,
        0.,
        direction,
        &ShapeCastConfig::from_max_distance(10000.),
        &SpatialQueryFilter::default(),
        &mut |hit| {
            let (pos, _, col) = spatial_query.colliders.get(hit.entity).unwrap();
            gizmos.circle_2d(
                Isometry2d::from_translation(**pos),
                col.radius,
                Color::srgb(1., 0.4, 0.4),
            );
            gizmos.circle_2d(direction * hit.distance, 7., Color::srgb(0.5, 1., 1.));
            true
        },
    );
}
