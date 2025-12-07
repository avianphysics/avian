use bevy::log::info;

use crate::prelude::*;

/// Needed to improve stability when `n.dot(dir)` happens to be very close to zero.
const DOT_EPSILON: Scalar = 0.005;

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

#[must_use]
pub fn project_velocity_new(v: Vector, normals: &[Dir]) -> Vector {
    -project_onto_conical_hull(-v, normals)
}

enum SimplicialCone {
    Origin,
    Ray(Dir),
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

        let betas = normals.iter().map(|n| n.dot(search_direction));

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
        if n_iters >= 100 {
            break;
        }
    }
    if n_iters > 4 {
        info!(
            "Failed to converge with {} normals took {n_iters} iterations",
            normals.len()
        );
        info!("Input -v = {x0}");
        info!("Normals = {normals:?}");
    }
    search_direction
}

impl SimplicialCone {
    fn project_point(self, x0: Vector, new_direction: Dir) -> (Option<SimplicialCone>, Vector) {
        match self {
            SimplicialCone::Origin => {
                let alpha = new_direction.dot(x0);
                (Some(Self::Ray(new_direction)), x0 - alpha * new_direction)
            }
            #[cfg(feature = "2d")]
            SimplicialCone::Ray(_) => (None, Vector::ZERO),
            #[cfg(feature = "3d")]
            SimplicialCone::Ray(previous_direction) => {
                let cross = new_direction.cross(*previous_direction);
                let alpha = x0.dot(cross);
                let new_search_vector = alpha * cross / cross.length_squared();

                (
                    Some(Self::Wedge(previous_direction, new_direction)),
                    new_search_vector,
                )
            }
            #[cfg(feature = "3d")]
            SimplicialCone::Wedge(n1, n2) => {
                let cross1 = new_direction.cross(*n1);
                let alpha1 = x0.dot(cross1);
                let gamma1 = n2.dot(cross1);
                let d1 = -alpha1 * gamma1.signum();
                let inside1 = d1 <= DOT_EPSILON;

                let cross2 = new_direction.cross(*n2);
                let alpha2 = x0.dot(cross2);
                let gamma2 = n1.dot(cross2);
                let d2 = -alpha2 * gamma2.signum();
                let inside2 = d2 <= DOT_EPSILON;

                if inside1 & inside2 {
                    (None, Vector::ZERO)
                } else if d1 > d2 {
                    (
                        Some(Self::Wedge(n1, new_direction)),
                        alpha1 * cross1 / cross1.length_squared(),
                    )
                } else {
                    (
                        Some(Self::Wedge(n2, new_direction)),
                        alpha2 * cross2 / cross2.length_squared(),
                    )
                }
            }
        }
    }
}
