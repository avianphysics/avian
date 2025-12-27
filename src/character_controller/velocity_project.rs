use bevy::log::debug;

use crate::prelude::*;

/// Needed to improve stability when `n.dot(dir)` happens to be very close to zero.
const DOT_EPSILON: Scalar = 1e-6;

/// Projects input velocity `v` onto the planes defined by the given `normals`.
/// This ensures that `velocity` does not point into any of the planes, but along them.
///
/// This is often used after [`MoveAndSlide::cast_move`] to ensure a character moved that way
/// does not try to continue moving into colliding geometry.
#[must_use]
pub fn project_velocity_old(v: Vector, normals: &[Dir]) -> Vector {
    if normals.is_empty() {
        return v;
    }

    // The halfspaces defined by the contact normals form a polyhedral cone.
    // We want to find the closest point to v that lies inside this cone.
    //
    // There are three cases to consider:
    // 1. v is already inside the cone -> return v
    // 2. v is outside the cone
    //    a. Project v onto each plane and check if the projection is inside the cone
    //    b. Project v onto each edge (intersection of two planes) and check if the projection is inside the cone
    // 3. If no valid projection is found, return the apex of the cone (the origin)

    // Case 1: Check if v is inside the cone
    if normals
        .iter()
        .all(|normal| normal.adjust_precision().dot(v) >= -DOT_EPSILON)
    {
        return v;
    }

    // Best candidate so far
    let mut best_projection = Vector::ZERO;
    let mut best_distance_sq = Scalar::INFINITY;

    // Helper to test halfspace validity
    let is_valid = |projection: Vector| {
        normals
            .iter()
            .all(|n| projection.dot(n.adjust_precision()) >= -DOT_EPSILON)
    };

    // Case 2a: Face projections (single-plane active set)
    for n in normals {
        let n = n.adjust_precision();
        let n_dot_v = n.dot(v);
        if n_dot_v < -DOT_EPSILON {
            // Project v onto the plane defined by n:
            // projection = v - (v · n) n
            let projection = v - n_dot_v * n;

            // Check if better than previous best and valid
            let distance_sq = v.distance_squared(projection);
            if distance_sq < best_distance_sq && is_valid(projection) {
                best_distance_sq = distance_sq;
                best_projection = projection;
            }
        }
    }

    // Case 2b: Edge projections (two-plane active set)
    // TODO: Can we optimize this from O(n^3) to O(n^2)?
    #[cfg(feature = "3d")]
    {
        let n = normals.len();
        for i in 0..n {
            let ni = normals[i].adjust_precision();
            for nj in normals
                .iter()
                .take(n)
                .skip(i + 1)
                .map(|n| n.adjust_precision())
            {
                // Compute edge direction e = ni x nj
                let e = ni.cross(nj);
                let e_length_sq = e.length_squared();
                if e_length_sq < DOT_EPSILON {
                    // Nearly parallel edge
                    continue;
                }

                // Project v onto the line spanned by e:
                // projection = ((v · e) / |e|²) e
                let projection = e * (v.dot(e) / e_length_sq);

                // Check if better than previous best and valid
                let distance_sq = v.distance_squared(projection);
                if distance_sq < best_distance_sq && is_valid(projection) {
                    best_distance_sq = distance_sq;
                    best_projection = projection;
                }
            }
        }
    }

    // Case 3: If no candidate is found, the projection is at the apex (the origin)
    if best_distance_sq.is_infinite() {
        Vector::ZERO
    } else {
        best_projection
    }
}

/// Projects input velocity `v` onto the planes defined by the given `normals`.
/// This ensures that `velocity` does not point into any of the planes, but along them.
///
/// This is often used after [`MoveAndSlide::cast_move`] to ensure a character moved that way
/// does not try to continue moving into colliding geometry.
#[must_use]
pub fn project_velocity_new(v: Vector, normals: &[Dir]) -> Vector {
    -project_onto_conical_hull(-v, normals)
}

#[derive(Debug)]
enum SimplicialCone {
    Origin,
    Ray(#[cfg(feature = "3d")] Dir),
    #[cfg(feature = "3d")]
    Wedge(Dir, Dir),
}

/// Projects input velocity `v` onto the planes defined by the given `normals`.
/// This ensures that `velocity` does not point into any of the planes, but along them.
///
/// This is often used after [`MoveAndSlide::cast_move`] to ensure a character moved that way
/// does not try to continue moving into colliding geometry.
#[must_use]
fn project_onto_conical_hull(x0: Vector, normals: &[Dir]) -> Vector {
    let mut maybe_cone = Some(SimplicialCone::Origin);
    let mut search_direction = x0;
    let mut n_iters = 0;

    while let Some(cone) = maybe_cone {
        if search_direction.length_squared() < DOT_EPSILON * DOT_EPSILON {
            break;
        }

        let betas = normals
            .iter()
            .map(|n| n.adjust_precision().dot(search_direction));

        let Some((best_idx, best_beta)) = betas
            .enumerate()
            .max_by(|(_, beta1), (_, beta2)| Scalar::total_cmp(beta1, beta2))
        else {
            break;
        };

        if best_beta <= DOT_EPSILON {
            break;
        }

        let best_normal = normals[best_idx];

        (maybe_cone, search_direction) = cone.project_point(x0, best_normal);
        n_iters += 1;
        if n_iters >= 10 {
            break;
        }
    }
    if n_iters > 4 {
        debug!(
            "Slow to converge with {} normals, took {n_iters} iterations",
            normals.len()
        );
        debug!("Input -v = {x0}");
        debug!("Normals = {normals:?}");
    }
    search_direction
}

impl SimplicialCone {
    fn project_point(self, x0: Vector, new_direction: Dir) -> (Option<SimplicialCone>, Vector) {
        let new_direction_vec = new_direction.adjust_precision();
        match self {
            SimplicialCone::Origin => {
                let alpha = new_direction_vec.dot(x0);
                #[cfg(feature = "2d")]
                let ray = Self::Ray();
                #[cfg(feature = "3d")]
                let ray = Self::Ray(new_direction);
                (Some(ray), x0 - alpha * new_direction_vec)
            }
            #[cfg(feature = "2d")]
            SimplicialCone::Ray() => (None, Vector::ZERO),
            #[cfg(feature = "3d")]
            SimplicialCone::Ray(previous_direction) => {
                let cross = new_direction_vec.cross(previous_direction.adjust_precision());
                let alpha = x0.dot(cross);
                let new_search_vector = alpha * cross / cross.length_squared();
                let new_cone = if alpha > 0.0 {
                    Self::Wedge(new_direction, previous_direction)
                } else {
                    Self::Wedge(previous_direction, new_direction)
                };

                (Some(new_cone), new_search_vector)
            }
            #[cfg(feature = "3d")]
            SimplicialCone::Wedge(n1, n2) => {
                let cross1 = n1.adjust_precision().cross(new_direction_vec);
                let c1sq = cross1.length_squared();
                let d1 = x0.dot(cross1);
                let inside1 = d1 <= 0.0;

                let cross2 = new_direction_vec.cross(n2.adjust_precision());
                let c2sq = cross2.length_squared();
                let d2 = x0.dot(cross2);
                let inside2 = d2 <= 0.0;

                if inside1 & inside2 {
                    (None, Vector::ZERO)
                } else if d1 * d1.abs() * c2sq > d2 * d2.abs() * c1sq {
                    // the above is `if d1 / cross1.length() > d2 / cross2.length()`
                    // but avoiding that couple of square roots
                    (Some(Self::Wedge(n1, new_direction)), d1 * cross1 / c1sq)
                } else {
                    (Some(Self::Wedge(new_direction, n2)), d2 * cross2 / c2sq)
                }
            }
        }
    }
}

#[cfg(test)]
mod test {
    use crate::math::{Dir, PI, Scalar, Vector};

    #[test]
    fn check_agreement() {
        #[cfg(feature = "2d")]
        let normals = &[
            Dir::Y,
            Dir::from_xy(2.0, 1.0).unwrap(),
            Dir::from_xy(-2.0, 1.0).unwrap(),
            Dir::from_xy(1.0, 1.0).unwrap(),
            Dir::from_xy(-1.0, 1.0).unwrap(),
        ];

        #[cfg(feature = "3d")]
        let normals = &[
            Dir::Z,
            Dir::from_xyz(2.0, 0.0, 1.0).unwrap(),
            Dir::from_xyz(-2.0, 0.0, 1.0).unwrap(),
            Dir::from_xyz(0.0, 2.0, 1.0).unwrap(),
            Dir::from_xyz(0.0, -2.0, 1.0).unwrap(),
            Dir::from_xyz(1.5, 1.5, 1.0).unwrap(),
            Dir::from_xyz(1.5, -1.5, 1.0).unwrap(),
            Dir::from_xyz(-1.5, 1.5, 1.0).unwrap(),
            Dir::from_xyz(-1.5, -1.5, 1.0).unwrap(),
            Dir::from_xyz(1.0, 1.75, 1.0).unwrap(),
            Dir::from_xyz(1.0, -1.75, 1.0).unwrap(),
            Dir::from_xyz(-1.0, 1.75, 1.0).unwrap(),
            Dir::from_xyz(-1.0, -1.75, 1.0).unwrap(),
            Dir::from_xyz(1.75, 1.0, 1.0).unwrap(),
            Dir::from_xyz(1.75, -1.0, 1.0).unwrap(),
            Dir::from_xyz(-1.75, 1.0, 1.0).unwrap(),
            Dir::from_xyz(-1.75, -1.0, 1.0).unwrap(),
        ];

        for n in 1..=normals.len() {
            let velocities = QuasiRandomDirection::default();
            let mut worst_result = (0.0, Vector::ZERO, Vector::ZERO, Vector::ZERO);
            for vel in velocities.take(1000) {
                let old_result = super::project_velocity_old(vel, &normals[..n]);
                let new_result = super::project_velocity_new(vel, &normals[..n]);
                let badness = (old_result - new_result).length_squared();
                if badness >= worst_result.0 {
                    worst_result = (badness, vel, old_result, new_result);
                }
            }
            let (error, input, old_result, new_result) = worst_result;
            eprintln!(
                "For {n} normals, worst disagreement is {} between {} and {} from input {}",
                error.sqrt(),
                old_result,
                new_result,
                input
            );
        }
    }

    #[derive(Default)]
    struct QuasiRandomDirection {
        #[cfg(feature = "3d")]
        i: Scalar,
        j: Scalar,
    }

    #[cfg(feature = "3d")]
    impl QuasiRandomDirection {
        #[allow(clippy::excessive_precision)]
        const PLASTIC: Scalar = 1.32471795724475;
        const INV_PLASTIC: Scalar = 1.0 / Self::PLASTIC;
        const INV_PLASTIC_SQ: Scalar = Self::INV_PLASTIC * Self::INV_PLASTIC;
    }

    #[cfg(feature = "2d")]
    impl QuasiRandomDirection {
        #[allow(clippy::excessive_precision)]
        const GOLDEN: Scalar = 1.61803398875;
        const INV_GOLDEN: Scalar = 1.0 / Self::GOLDEN;
    }

    impl Iterator for QuasiRandomDirection {
        type Item = Vector;

        fn next(&mut self) -> Option<Self::Item> {
            let phi = 2.0 * PI * self.j;
            let x = phi.cos();
            let y = phi.sin();
            #[cfg(feature = "3d")]
            {
                let z = 2.0 * self.i - 1.0;
                let rho = (1.0 - z * z).sqrt();
                self.i = (self.i + Self::INV_PLASTIC) % 1.0;
                self.j = (self.j + Self::INV_PLASTIC_SQ) % 1.0;
                Some(Vector::new(rho * x, rho * y, z))
            }
            #[cfg(feature = "2d")]
            {
                self.j = (self.j + Self::INV_GOLDEN) % 1.0;
                Some(Vector::new(x, y))
            }
        }
    }
}
