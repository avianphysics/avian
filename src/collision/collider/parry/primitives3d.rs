use bevy_math::{
    Quat, Vec3,
    primitives::{
        Capsule3d, Cone, Cuboid, Cylinder, InfinitePlane3d, Line3d, Plane3d, Polyline3d, Segment3d,
        Sphere,
    },
};
use parry::shape::SharedShape;

use crate::{Collider, IntoCollider, RVector, ToRealPrecision};

impl IntoCollider<Collider> for Sphere {
    fn collider(&self) -> Collider {
        Collider::sphere(self.radius)
    }
}

impl IntoCollider<Collider> for InfinitePlane3d {
    fn collider(&self) -> Collider {
        let half_size = 10_000.0;
        let rotation = Quat::from_rotation_arc(Vec3::Y, *self.normal).real();
        let vertices = vec![
            rotation * RVector::new(half_size, 0.0, -half_size),
            rotation * RVector::new(-half_size, 0.0, -half_size),
            rotation * RVector::new(-half_size, 0.0, half_size),
            rotation * RVector::new(half_size, 0.0, half_size),
        ];

        Collider::trimesh(vertices, vec![[0, 1, 2], [1, 2, 0]])
    }
}

impl IntoCollider<Collider> for Plane3d {
    fn collider(&self) -> Collider {
        let half_size = self.half_size.real();
        let rotation = Quat::from_rotation_arc(Vec3::Y, *self.normal).real();
        let vertices = vec![
            rotation * RVector::new(half_size.x, 0.0, -half_size.y),
            rotation * RVector::new(-half_size.x, 0.0, -half_size.y),
            rotation * RVector::new(-half_size.x, 0.0, half_size.y),
            rotation * RVector::new(half_size.x, 0.0, half_size.y),
        ];

        Collider::trimesh(vertices, vec![[0, 1, 2], [1, 2, 0]])
    }
}

impl IntoCollider<Collider> for Line3d {
    fn collider(&self) -> Collider {
        let vec = self.direction * 10_000.0;
        Collider::segment(-vec, vec)
    }
}

impl IntoCollider<Collider> for Segment3d {
    fn collider(&self) -> Collider {
        let (point1, point2) = (self.point1(), self.point2());
        Collider::segment(point1, point2)
    }
}

impl IntoCollider<Collider> for Polyline3d {
    fn collider(&self) -> Collider {
        let vertices = self.vertices.iter().map(|v| v.real()).collect();
        Collider::polyline(vertices, None)
    }
}

impl IntoCollider<Collider> for Cuboid {
    fn collider(&self) -> Collider {
        let [hx, hy, hz] = self.half_size.real().to_array();
        Collider::from(SharedShape::cuboid(hx, hy, hz))
    }
}

impl IntoCollider<Collider> for Cylinder {
    fn collider(&self) -> Collider {
        Collider::from(SharedShape::cylinder(
            self.half_height.real(),
            self.radius.real(),
        ))
    }
}

impl IntoCollider<Collider> for Capsule3d {
    fn collider(&self) -> Collider {
        Collider::capsule(self.radius, 2.0 * self.half_length)
    }
}

impl IntoCollider<Collider> for Cone {
    fn collider(&self) -> Collider {
        Collider::cone(self.radius, self.height)
    }
}

// TODO: ConicalFrustum
// TODO: Torus
