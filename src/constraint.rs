use bevy::prelude::*;
use crate::{RotationConstraint, utils::quat_abs};

pub fn constrain_direction_cone(
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

    main_direction.slerp(constrained, strength).normalize()
}

pub fn constrain_direction_ellipse(
    main_direction: Vec3,
    parent_direction: Vec3,
    max_x: f32,
    max_z: f32,
    strength: f32,
) -> Vec3{

    let swing_x = parent_direction.any_orthonormal_vector();
    let swing_z = parent_direction.cross(swing_x).normalize();

    let x = main_direction.dot(swing_x);
    let z = main_direction.dot(swing_z);

    let sx = max_x.sin();
    let sz = max_z.sin();
    
    let ellipse_value = (x * x) / (sx * sx) + (z * z) / (sz * sz);

    let (cx, cz) = if ellipse_value <= 1.0 {
        (x, z)
    } else {
        let scale = ellipse_value.sqrt();
        (x / scale, z / scale)
    };

    let cy = (1.0 - cx * cx - cz * cz).max(0.0).sqrt();

    let constrained = swing_x * cx + swing_z * cz + parent_direction * cy;

    main_direction.slerp(constrained.normalize(), strength).normalize()
}

