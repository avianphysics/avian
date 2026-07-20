use crate::{
    ToRealPrecision,
    math::{self, RVector, Real},
};

use super::{Collider, IntoCollider, ToF32Precision};
use bevy::prelude::{Deref, DerefMut};
use bevy_math::{bounding::Bounded2d, prelude::*};
use parry::{
    mass_properties::MassProperties,
    math::Pose,
    query::{
        PointQuery, RayCast, details::local_ray_intersection_with_support_map_with_params,
        gjk::VoronoiSimplex, point::local_point_projection_on_support_map,
    },
    shape::{
        FeatureId, PackedFeatureId, PolygonalFeature, PolygonalFeatureMap, Shape, SharedShape,
        SupportMap,
    },
};

const PI: Real = core::f64::consts::PI as Real;
const TAU: Real = core::f64::consts::TAU as Real;
const FRAC_PI_2: Real = core::f64::consts::FRAC_PI_2 as Real;

impl IntoCollider<Collider> for Circle {
    fn collider(&self) -> Collider {
        Collider::circle(self.radius)
    }
}

impl IntoCollider<Collider> for Ellipse {
    fn collider(&self) -> Collider {
        Collider::from(SharedShape::new(EllipseColliderShape(*self)))
    }
}

/// An ellipse shape that can be stored in a [`SharedShape`] for an ellipse [`Collider`].
///
/// This wrapper is required to allow implementing the necessary traits from [`parry`]
/// for Bevy's [`Ellipse`] type.
#[derive(Clone, Copy, Debug, Deref, DerefMut)]
pub struct EllipseColliderShape(pub Ellipse);

impl SupportMap for EllipseColliderShape {
    #[inline]
    fn local_support_point(&self, direction: RVector) -> RVector {
        let [a, b] = self.half_size.real().to_array();
        let denom = (direction.x.powi(2) * a * a + direction.y.powi(2) * b * b).sqrt();
        RVector::new(a * a * direction.x / denom, b * b * direction.y / denom)
    }
}

impl Shape for EllipseColliderShape {
    fn clone_dyn(&self) -> Box<dyn Shape> {
        Box::new(*self)
    }

    fn scale_dyn(
        &self,
        scale: RVector,
        _num_subdivisions: u32,
    ) -> Option<Box<dyn parry::shape::Shape>> {
        let half_size = scale.f32() * self.half_size;
        Some(Box::new(EllipseColliderShape(Ellipse::new(
            half_size.x,
            half_size.y,
        ))))
    }

    fn compute_local_aabb(&self) -> parry::bounding_volume::Aabb {
        let aabb = self.aabb_2d(Isometry2d::IDENTITY);
        parry::bounding_volume::Aabb::new(aabb.min.real(), aabb.max.real())
    }

    fn compute_aabb(&self, position: &Pose) -> parry::bounding_volume::Aabb {
        let isometry = math::pose_to_isometry(position);
        let aabb = self.aabb_2d(isometry);
        parry::bounding_volume::Aabb::new(aabb.min.real(), aabb.max.real())
    }

    fn compute_local_bounding_sphere(&self) -> parry::bounding_volume::BoundingSphere {
        let sphere = self.bounding_circle(Isometry2d::IDENTITY);
        parry::bounding_volume::BoundingSphere::new(sphere.center.real(), sphere.radius().real())
    }

    fn compute_bounding_sphere(&self, position: &Pose) -> parry::bounding_volume::BoundingSphere {
        let isometry = math::pose_to_isometry(position);
        let sphere = self.bounding_circle(isometry);
        parry::bounding_volume::BoundingSphere::new(sphere.center.real(), sphere.radius().real())
    }

    fn clone_box(&self) -> Box<dyn Shape> {
        Box::new(*self)
    }

    fn mass_properties(&self, density: Real) -> MassProperties {
        let volume = self.area().real();
        let mass = volume * density;
        let inertia = mass * self.half_size.length_squared().real() / 4.0;
        MassProperties::new(RVector::ZERO, mass, inertia)
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn shape_type(&self) -> parry::shape::ShapeType {
        parry::shape::ShapeType::Custom
    }

    fn as_typed_shape(&self) -> parry::shape::TypedShape<'_> {
        parry::shape::TypedShape::Custom(self)
    }

    fn ccd_thickness(&self) -> Real {
        self.half_size.max_element().real()
    }

    fn ccd_angular_thickness(&self) -> Real {
        core::f64::consts::PI.real()
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap> {
        Some(self as &dyn SupportMap)
    }
}

impl RayCast for EllipseColliderShape {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &parry::query::Ray,
        max_toi: Real,
        solid: bool,
    ) -> Option<parry::query::RayIntersection> {
        local_ray_intersection_with_support_map_with_params(
            self,
            &mut VoronoiSimplex::new(),
            ray,
            max_toi,
            solid,
        )
    }
}

impl PointQuery for EllipseColliderShape {
    fn project_local_point(&self, pt: RVector, solid: bool) -> parry::query::PointProjection {
        local_point_projection_on_support_map(self, &mut VoronoiSimplex::new(), pt, solid)
    }

    fn project_local_point_and_get_feature(
        &self,
        pt: RVector,
    ) -> (parry::query::PointProjection, parry::shape::FeatureId) {
        (self.project_local_point(pt, false), FeatureId::Unknown)
    }
}

impl IntoCollider<Collider> for Plane2d {
    fn collider(&self) -> Collider {
        let vec = self.normal.perp() * 100_000.0 / 2.0;
        Collider::segment(-vec, vec)
    }
}

impl IntoCollider<Collider> for Line2d {
    fn collider(&self) -> Collider {
        let vec = self.direction * 100_000.0 / 2.0;
        Collider::segment(-vec, vec)
    }
}

impl IntoCollider<Collider> for Segment2d {
    fn collider(&self) -> Collider {
        let (point1, point2) = (self.point1(), self.point2());
        Collider::segment(point1, point2)
    }
}

impl IntoCollider<Collider> for Triangle2d {
    fn collider(&self) -> Collider {
        Collider::triangle(self.vertices[0], self.vertices[1], self.vertices[2])
    }
}

impl IntoCollider<Collider> for Rectangle {
    fn collider(&self) -> Collider {
        Collider::from(SharedShape::cuboid(
            self.half_size.x.real(),
            self.half_size.y.real(),
        ))
    }
}

impl IntoCollider<Collider> for Polygon {
    fn collider(&self) -> Collider {
        let vertices = self.vertices.iter().map(|v| v.real()).collect();
        let indices = (0..self.vertices.len() as u32 - 1)
            .map(|i| [i, i + 1])
            .collect();
        Collider::convex_decomposition(vertices, indices)
    }
}

impl IntoCollider<Collider> for ConvexPolygon {
    fn collider(&self) -> Collider {
        let vertices = self.vertices().iter().map(|v| v.real()).collect();
        Collider::convex_polyline(vertices).unwrap()
    }
}

impl IntoCollider<Collider> for RegularPolygon {
    fn collider(&self) -> Collider {
        Collider::from(SharedShape::new(RegularPolygonColliderShape(*self)))
    }
}

/// A regular polygon shape that can be stored in a [`SharedShape`] for a regular polygon [`Collider`].
///
/// This wrapper is required to allow implementing the necessary traits from [`parry`]
/// for Bevy's [`RegularPolygon`] type.
#[derive(Clone, Copy, Debug, Deref, DerefMut)]
pub struct RegularPolygonColliderShape(pub RegularPolygon);

impl SupportMap for RegularPolygonColliderShape {
    #[inline]
    fn local_support_point(&self, direction: RVector) -> RVector {
        // TODO: For polygons with a small number of sides, maybe just iterating
        //       through the vertices and comparing dot products is faster?

        let external_angle = self.external_angle_radians().real();
        let circumradius = self.circumradius().real();

        // Counterclockwise
        let angle_from_top = if direction.x < 0.0 {
            -direction.angle_to(RVector::Y)
        } else {
            TAU - direction.angle_to(RVector::Y)
        };

        // How many rotations of `external_angle` correspond to the vertex closest to the support direction.
        let n = (angle_from_top / external_angle).round() % self.sides as Real;

        // Rotate by an additional 90 degrees so that the first vertex is always at the top.
        let target_angle = n * external_angle + FRAC_PI_2;

        // Compute the vertex corresponding to the target angle on the unit circle.
        circumradius * RVector::from_angle(target_angle)
    }
}

impl PolygonalFeatureMap for RegularPolygonColliderShape {
    #[inline]
    fn local_support_feature(&self, direction: RVector, out_feature: &mut PolygonalFeature) {
        let external_angle = self.external_angle_radians().real();
        let circumradius = self.circumradius().real();

        // Counterclockwise
        let angle_from_top = if direction.x < 0.0 {
            -direction.angle_to(RVector::Y)
        } else {
            TAU - direction.angle_to(RVector::Y)
        };

        // How many rotations of `external_angle` correspond to the vertices.
        let n_unnormalized = angle_from_top / external_angle;
        let n1 = n_unnormalized.floor() % self.sides as Real;
        let n2 = n_unnormalized.ceil() % self.sides as Real;

        // Rotate by an additional 90 degrees so that the first vertex is always at the top.
        let target_angle1 = n1 * external_angle + FRAC_PI_2;
        let target_angle2 = n2 * external_angle + FRAC_PI_2;

        // Compute the vertices corresponding to the target angle on the unit circle.
        let vertex1 = circumradius * RVector::from_angle(target_angle1);
        let vertex2 = circumradius * RVector::from_angle(target_angle2);

        *out_feature = PolygonalFeature {
            vertices: [vertex1, vertex2],
            vids: [
                PackedFeatureId::vertex(n1 as u32),
                PackedFeatureId::vertex(n2 as u32),
            ],
            fid: PackedFeatureId::face(n1 as u32),
            num_vertices: 2,
        };
    }
}

impl Shape for RegularPolygonColliderShape {
    fn clone_dyn(&self) -> Box<dyn Shape> {
        Box::new(*self)
    }

    fn scale_dyn(
        &self,
        scale: RVector,
        _num_subdivisions: u32,
    ) -> Option<Box<dyn parry::shape::Shape>> {
        let circumradius = scale.f32() * self.circumradius();
        Some(Box::new(RegularPolygonColliderShape(RegularPolygon::new(
            circumradius.length(),
            self.sides,
        ))))
    }

    fn compute_local_aabb(&self) -> parry::bounding_volume::Aabb {
        let aabb = self.aabb_2d(Isometry2d::IDENTITY);
        parry::bounding_volume::Aabb::new(aabb.min.real(), aabb.max.real())
    }

    fn compute_aabb(&self, position: &Pose) -> parry::bounding_volume::Aabb {
        let isometry = math::pose_to_isometry(position);
        let aabb = self.aabb_2d(isometry);
        parry::bounding_volume::Aabb::new(aabb.min.real(), aabb.max.real())
    }

    fn compute_local_bounding_sphere(&self) -> parry::bounding_volume::BoundingSphere {
        let sphere = self.bounding_circle(Isometry2d::IDENTITY);
        parry::bounding_volume::BoundingSphere::new(sphere.center.real(), sphere.radius().real())
    }

    fn compute_bounding_sphere(&self, position: &Pose) -> parry::bounding_volume::BoundingSphere {
        let isometry = math::pose_to_isometry(position);
        let sphere = self.bounding_circle(isometry);
        parry::bounding_volume::BoundingSphere::new(sphere.center.real(), sphere.radius().real())
    }

    fn clone_box(&self) -> Box<dyn Shape> {
        Box::new(*self)
    }

    fn mass_properties(&self, density: Real) -> MassProperties {
        let volume = self.area().real();
        let mass = volume * density;

        let half_external_angle = PI / self.sides as Real;
        let angular_inertia = mass * self.circumradius().real().powi(2) / 6.0
            * (1.0 + 2.0 * half_external_angle.cos().powi(2));

        MassProperties::new(RVector::ZERO, mass, angular_inertia)
    }

    fn is_convex(&self) -> bool {
        true
    }

    fn shape_type(&self) -> parry::shape::ShapeType {
        parry::shape::ShapeType::Custom
    }

    fn as_typed_shape(&self) -> parry::shape::TypedShape<'_> {
        parry::shape::TypedShape::Custom(self)
    }

    fn ccd_thickness(&self) -> Real {
        self.circumradius().real()
    }

    fn ccd_angular_thickness(&self) -> Real {
        core::f64::consts::PI.real() - self.internal_angle_radians().real()
    }

    fn as_support_map(&self) -> Option<&dyn SupportMap> {
        Some(self as &dyn SupportMap)
    }

    fn as_polygonal_feature_map(&self) -> Option<(&dyn PolygonalFeatureMap, Real)> {
        Some((self as &dyn PolygonalFeatureMap, 0.0))
    }

    fn feature_normal_at_point(&self, feature: FeatureId, _point: RVector) -> Option<RVector> {
        match feature {
            FeatureId::Face(id) => {
                let external_angle = self.external_angle_radians().real();
                let normal_angle = id as Real * external_angle - external_angle * 0.5 + FRAC_PI_2;
                Some(RVector::from_angle(normal_angle))
            }
            FeatureId::Vertex(id) => {
                let external_angle = self.external_angle_radians().real();
                let normal_angle = id as Real * external_angle + FRAC_PI_2;
                Some(RVector::from_angle(normal_angle))
            }
            _ => None,
        }
    }
}

impl RayCast for RegularPolygonColliderShape {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &parry::query::Ray,
        max_toi: Real,
        solid: bool,
    ) -> Option<parry::query::RayIntersection> {
        local_ray_intersection_with_support_map_with_params(
            self,
            &mut VoronoiSimplex::new(),
            ray,
            max_toi,
            solid,
        )
    }
}

impl PointQuery for RegularPolygonColliderShape {
    fn project_local_point(&self, pt: RVector, solid: bool) -> parry::query::PointProjection {
        local_point_projection_on_support_map(self, &mut VoronoiSimplex::new(), pt, solid)
    }

    fn project_local_point_and_get_feature(
        &self,
        pt: RVector,
    ) -> (parry::query::PointProjection, parry::shape::FeatureId) {
        (self.project_local_point(pt, false), FeatureId::Unknown)
    }
}

impl IntoCollider<Collider> for Capsule2d {
    fn collider(&self) -> Collider {
        Collider::capsule(self.radius, 2.0 * self.half_length)
    }
}
