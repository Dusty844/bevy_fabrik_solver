use bevy::prelude::*;
use super::{RotationConstraint};


pub fn constrain_direction(
    main_direction: Vec3,
    parent_direction: Vec3,
    max_angle: f32,
    strength: f32,
) -> Vec3{
    let cos_theta = main_direction.dot(parent_direction);
    let cos_max = max_angle.cos();
    if cos_theta >= cos_max {
        return main_direction;
    }

    let ortho = (main_direction - parent_direction * cos_theta).try_normalize().unwrap_or_else(||{
        parent_direction.any_orthonormal_vector()
    });

    let constrained = (parent_direction * cos_max + ortho * max_angle.sin()).normalize();

    main_direction.lerp(constrained, strength).normalize()
}
