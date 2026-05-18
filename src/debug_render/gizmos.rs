#![allow(clippy::unnecessary_cast)]

use crate::prelude::*;
use bevy::prelude::*;
#[cfg(all(
    feature = "default-collider",
    any(feature = "parry-f32", feature = "parry-f64")
))]
use parry::shape::{SharedShape, TypedShape};

/// An extension trait for `Gizmos<PhysicsGizmo>`.
pub trait PhysicsGizmoExt {
    /// Draws a line from `a` to `b`.
    fn draw_line(&mut self, a: Vector, b: Vector, color: Color);

    /// Draws lines between a list of points.
    fn draw_line_strip(
        &mut self,
        points: Vec<Vector>,
        position: impl Into<Position>,
        rotation: impl Into<Rotation>,
        closed: bool,
        color: Color,
    );

    /// Draws a polyline based on the given vertex and index buffers.
    fn draw_polyline(
        &mut self,
        vertices: &[Vector],
        indices: &[[u32; 2]],
        position: impl Into<Position>,
        rotation: impl Into<Rotation>,
        color: Color,
    );

    /// Draws an arrow from `a` to `b` with an arrowhead that has a length of `head_length`.
    fn draw_arrow(&mut self, a: Vector, b: Vector, head_length: Scalar, color: Color);

    /// Draws a [`Collider`] shape.
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    fn draw_collider(
        &mut self,
        collider: &Collider,
        position: impl Into<Position>,
        rotation: impl Into<Rotation>,
        color: Color,
    );

    /// Draws the results of a [raycast](SpatialQuery#raycasting).
    #[allow(clippy::too_many_arguments)]
    fn draw_raycast(
        &mut self,
        origin: Vector,
        direction: Dir,
        max_distance: Scalar,
        hits: &[RayHitData],
        ray_color: Color,
        point_color: Color,
        normal_color: Color,
        length_unit: Scalar,
    );

    /// Draws the results of a [shapecast](SpatialQuery#shapecasting).
    #[allow(clippy::too_many_arguments)]
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    fn draw_shapecast(
        &mut self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: impl Into<Rotation>,
        direction: Dir,
        max_distance: Scalar,
        hits: &[ShapeHitData],
        ray_color: Color,
        shape_color: Color,
        point_color: Color,
        normal_color: Color,
        length_unit: Scalar,
    );
}

impl PhysicsGizmoExt for Gizmos<'_, '_, PhysicsGizmos> {
    /// Draws a line from `a` to `b`.
    fn draw_line(&mut self, a: Vector, b: Vector, color: Color) {
        #[cfg(feature = "2d")]
        self.line_2d(a.f32(), b.f32(), color);
        #[cfg(feature = "3d")]
        self.line(a.f32(), b.f32(), color);
    }

    /// Draws lines between a list of points.
    fn draw_line_strip(
        &mut self,
        points: Vec<Vector>,
        position: impl Into<Position>,
        rotation: impl Into<Rotation>,
        closed: bool,
        color: Color,
    ) {
        let position: Position = position.into();
        let rotation: Rotation = rotation.into();

        let pos = position.f32();
        #[cfg(feature = "2d")]
        self.linestrip_2d(points.iter().map(|p| pos + (rotation * p).f32()), color);
        #[cfg(feature = "3d")]
        self.linestrip(points.iter().map(|p| pos + (rotation * p).f32()), color);

        if closed && points.len() > 2 {
            let a = position.0 + rotation * points[0];
            let b = position.0 + rotation * points.last().unwrap();
            self.draw_line(a, b, color);
        }
    }

    /// Draws a polyline based on the given vertex and index buffers.
    fn draw_polyline(
        &mut self,
        vertices: &[Vector],
        indices: &[[u32; 2]],
        position: impl Into<Position>,
        rotation: impl Into<Rotation>,
        color: Color,
    ) {
        let position: Position = position.into();
        let rotation: Rotation = rotation.into();

        for [i1, i2] in indices {
            let a = position.0 + rotation * vertices[*i1 as usize];
            let b = position.0 + rotation * vertices[*i2 as usize];
            self.draw_line(a, b, color);
        }
    }

    /// Draws an arrow from `a` to `b` with an arrowhead that has a length of `head_length`
    /// and a width of `head_width`.
    fn draw_arrow(&mut self, a: Vector, b: Vector, head_length: Scalar, color: Color) {
        #[cfg(feature = "2d")]
        {
            self.arrow_2d(a.f32(), b.f32(), color)
                .with_tip_length(head_length as f32);
        }

        #[cfg(feature = "3d")]
        {
            self.arrow(a.f32(), b.f32(), color)
                .with_tip_length(head_length as f32);
        }
    }

    /// Draws a collider shape with a given position and rotation.
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    fn draw_collider(
        &mut self,
        collider: &Collider,
        position: impl Into<Position>,
        rotation: impl Into<Rotation>,
        color: Color,
    ) {
        let position: Position = position.into();
        let rotation: Rotation = rotation.into();

        match collider.shape_scaled().as_typed_shape() {
            #[cfg(feature = "2d")]
            TypedShape::Ball(s) => {
                self.circle_2d(position.f32(), s.radius as f32, color);
            }
            #[cfg(feature = "3d")]
            TypedShape::Ball(s) => {
                self.sphere(
                    Isometry3d::new(position.f32(), rotation.f32()),
                    s.radius as f32,
                    color,
                );
            }
            #[cfg(feature = "2d")]
            TypedShape::Cuboid(s) => {
                self.rect_2d(
                    Isometry2d::new(
                        position.f32(),
                        Rot2::from_sin_cos(rotation.sin as f32, rotation.cos as f32),
                    ),
                    2.0 * s.half_extents.f32(),
                    color,
                );
            }
            #[cfg(feature = "3d")]
            TypedShape::Cuboid(s) => {
                use bevy_math::bounding::Aabb3d;

                self.aabb_3d(
                    Aabb3d::new(Vec3A::ZERO, s.half_extents.f32()),
                    Transform::from_translation(position.f32()).with_rotation(rotation.f32()),
                    color,
                );
            }
            #[cfg(feature = "2d")]
            TypedShape::Capsule(s) => {
                self.draw_line_strip(s.to_polyline(32), position, rotation, true, color);
            }
            #[cfg(feature = "3d")]
            TypedShape::Capsule(s) => {
                let (vertices, indices) = s.to_outline(32);
                self.draw_polyline(&vertices, &indices, position, rotation, color);
            }
            TypedShape::Segment(s) => {
                self.draw_line_strip(vec![s.a, s.b], position, rotation, false, color)
            }
            TypedShape::Triangle(s) => {
                self.draw_line_strip(vec![s.a, s.b, s.c], position, rotation, true, color)
            }
            TypedShape::TriMesh(s) => {
                for tri in s.triangles() {
                    self.draw_collider(
                        &Collider::from(SharedShape::new(tri)),
                        position,
                        rotation,
                        color,
                    );
                }
            }
            TypedShape::Polyline(s) => {
                self.draw_polyline(s.vertices(), s.indices(), position, rotation, color)
            }
            #[cfg(feature = "2d")]
            TypedShape::HalfSpace(s) => {
                let basis = Vector::new(-s.normal.y, s.normal.x);
                let a = basis * 10_000.0;
                let b = basis * -10_000.0;
                self.draw_line_strip(vec![a, b], position, rotation, false, color);
            }
            #[cfg(feature = "3d")]
            TypedShape::HalfSpace(s) => {
                let n = s.normal;
                let sign = n.z.signum();
                let a = -1.0 / (sign + n.z);
                let b = n.x * n.y * a;
                let basis1 = Vector::new(1.0 + sign * n.x * n.x * a, sign * b, -sign * n.x);
                let basis2 = Vector::new(b, sign + n.y * n.y * a, -n.y);
                let a = basis1 * 10_000.0;
                let b = basis1 * -10_000.0;
                let c = basis2 * 10_000.0;
                let d = basis2 * -10_000.0;
                self.draw_polyline(&[a, b, c, d], &[[0, 1], [2, 3]], position, rotation, color);
            }
            TypedShape::Voxels(v) => {
                #[cfg(feature = "2d")]
                let (vertices, indices) = v.to_polyline();
                #[cfg(feature = "3d")]
                let (vertices, indices) = v.to_outline();
                self.draw_polyline(&vertices, &indices, position, rotation, color);
            }
            TypedShape::HeightField(s) => {
                #[cfg(feature = "2d")]
                for segment in s.segments() {
                    self.draw_collider(
                        &Collider::from(SharedShape::new(segment)),
                        position,
                        rotation,
                        color,
                    );
                }
                #[cfg(feature = "3d")]
                for triangle in s.triangles() {
                    self.draw_collider(
                        &Collider::from(SharedShape::new(triangle)),
                        position,
                        rotation,
                        color,
                    );
                }
            }
            TypedShape::Compound(s) => {
                for (sub_pos, shape) in s.shapes() {
                    let pos = Position(position.0 + rotation * sub_pos.translation);
                    #[cfg(feature = "2d")]
                    let rot = rotation * Rotation::radians(sub_pos.rotation.angle());
                    #[cfg(feature = "3d")]
                    let rot = Rotation((rotation.mul_quat(sub_pos.rotation)).normalize());
                    self.draw_collider(&Collider::from(shape.to_owned()), pos, rot, color);
                }
            }
            #[cfg(feature = "2d")]
            TypedShape::ConvexPolygon(s) => {
                self.draw_line_strip(s.points().to_vec(), position, rotation, true, color);
            }
            #[cfg(feature = "3d")]
            TypedShape::ConvexPolyhedron(s) => {
                let indices = s
                    .edges()
                    .iter()
                    .map(|e| [e.vertices[0], e.vertices[1]])
                    .collect::<Vec<_>>();
                self.draw_polyline(s.points(), &indices, position, rotation, color);
            }
            #[cfg(feature = "3d")]
            TypedShape::Cylinder(s) => {
                let (vertices, indices) = s.to_outline(32);
                self.draw_polyline(&vertices, &indices, position, rotation, color);
            }
            #[cfg(feature = "3d")]
            TypedShape::Cone(s) => {
                let (vertices, indices) = s.to_outline(32);
                self.draw_polyline(&vertices, &indices, position, rotation, color);
            }
            // ------------
            // Round shapes
            // ------------
            #[cfg(feature = "2d")]
            TypedShape::RoundCuboid(s) => {
                self.draw_line_strip(s.to_polyline(32), position, rotation, true, color);
            }
            #[cfg(feature = "3d")]
            TypedShape::RoundCuboid(s) => {
                let (vertices, indices) = s.to_outline(32);
                self.draw_polyline(&vertices, &indices, position, rotation, color);
            }
            TypedShape::RoundTriangle(s) => {
                // Parry doesn't have a method for the rounded outline, so we have to just use a normal triangle
                // (or compute the outline manually based on the border radius)
                self.draw_collider(
                    &Collider::from(SharedShape::new(s.inner_shape)),
                    position,
                    rotation,
                    color,
                );
            }
            #[cfg(feature = "2d")]
            TypedShape::RoundConvexPolygon(s) => {
                self.draw_line_strip(s.to_polyline(32), position, rotation, true, color);
            }
            #[cfg(feature = "3d")]
            TypedShape::RoundConvexPolyhedron(s) => {
                let (vertices, indices) = s.to_outline(32);
                self.draw_polyline(&vertices, &indices, position, rotation, color);
            }
            #[cfg(feature = "3d")]
            TypedShape::RoundCylinder(s) => {
                let (vertices, indices) = s.to_outline(32, 32);
                self.draw_polyline(&vertices, &indices, position, rotation, color);
            }
            #[cfg(feature = "3d")]
            TypedShape::RoundCone(s) => {
                let (vertices, indices) = s.to_outline(32, 32);
                self.draw_polyline(&vertices, &indices, position, rotation, color);
            }
            TypedShape::Custom(_id) => {
                #[cfg(feature = "2d")]
                {
                    use crate::collision::collider::{
                        EllipseColliderShape, RegularPolygonColliderShape,
                    };

                    if let Some(ellipse) =
                        collider.shape_scaled().as_shape::<EllipseColliderShape>()
                    {
                        let isometry = Isometry2d::new(
                            position.f32(),
                            Rot2::from_sin_cos(rotation.sin as f32, rotation.cos as f32),
                        );
                        self.primitive_2d(&ellipse.0, isometry, color);
                    } else if let Some(polygon) = collider
                        .shape_scaled()
                        .as_shape::<RegularPolygonColliderShape>()
                    {
                        let isometry = Isometry2d::new(
                            position.f32(),
                            Rot2::from_sin_cos(rotation.sin as f32, rotation.cos as f32),
                        );
                        self.primitive_2d(&polygon.0, isometry, color);
                    }
                }
            }
        }
    }

    /// Draws the results of a [raycast](SpatialQuery#raycasting).
    #[allow(clippy::too_many_arguments)]
    fn draw_raycast(
        &mut self,
        origin: Vector,
        direction: Dir,
        max_distance: Scalar,
        hits: &[RayHitData],
        ray_color: Color,
        point_color: Color,
        normal_color: Color,
        length_unit: Scalar,
    ) {
        let max_distance = hits
            .iter()
            .max_by(|a, b| a.distance.total_cmp(&b.distance))
            .map_or(max_distance, |hit| hit.distance);

        // Draw ray as arrow
        self.draw_arrow(
            origin,
            origin + direction.adjust_precision() * max_distance,
            0.1 * length_unit,
            ray_color,
        );

        // Draw all hit points and normals
        for hit in hits {
            let point = origin + direction.adjust_precision() * hit.distance;

            // Draw hit point
            #[cfg(feature = "2d")]
            self.circle_2d(point.f32(), 0.1 * length_unit as f32, point_color);
            #[cfg(feature = "3d")]
            self.sphere(point.f32(), 0.1 * length_unit as f32, point_color);

            // Draw hit normal as arrow
            self.draw_arrow(
                point,
                point + hit.normal * 0.5 * length_unit,
                0.1 * length_unit,
                normal_color,
            );
        }
    }

    /// Draws the results of a [shapecast](SpatialQuery#shapecasting).
    #[cfg(all(
        feature = "default-collider",
        any(feature = "parry-f32", feature = "parry-f64")
    ))]
    #[allow(clippy::too_many_arguments)]
    fn draw_shapecast(
        &mut self,
        shape: &Collider,
        origin: Vector,
        shape_rotation: impl Into<Rotation>,
        direction: Dir,
        max_distance: Scalar,
        hits: &[ShapeHitData],
        ray_color: Color,
        shape_color: Color,
        point_color: Color,
        normal_color: Color,
        length_unit: Scalar,
    ) {
        let shape_rotation = shape_rotation.into();
        #[cfg(feature = "3d")]
        let shape_rotation = Rotation(shape_rotation.normalize());

        let max_distance = hits
            .iter()
            .max_by(|a, b| a.distance.total_cmp(&b.distance))
            .map_or(max_distance, |hit| hit.distance);

        // Draw collider at origin
        self.draw_collider(shape, origin, shape_rotation, shape_color);

        // Draw arrow from origin to position of shape at final hit
        // TODO: We could render the swept collider outline instead
        self.draw_arrow(
            origin,
            origin + max_distance * direction.adjust_precision(),
            0.1 * length_unit,
            ray_color,
        );

        // Draw all hit points, normals and the shape at the hit points
        for hit in hits {
            // Draw hit point
            #[cfg(feature = "2d")]
            self.circle_2d(hit.point1.f32(), 0.1 * length_unit as f32, point_color);
            #[cfg(feature = "3d")]
            self.sphere(hit.point1.f32(), 0.1 * length_unit as f32, point_color);

            // Draw hit normal as arrow
            self.draw_arrow(
                hit.point1,
                hit.point1 + hit.normal1 * 0.5 * length_unit,
                0.1 * length_unit,
                normal_color,
            );

            // Draw collider at hit point
            self.draw_collider(
                shape,
                origin + hit.distance * direction.adjust_precision(),
                shape_rotation,
                shape_color.with_alpha(0.3),
            );
        }
    }
}
