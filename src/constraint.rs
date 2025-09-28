use bevy::{math::Affine3A, prelude::*, render::render_resource::AsBindGroupShaderType};

use crate::utils::QuatExtra;

#[derive(Component, Copy, Clone, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
pub struct RotationConstraint{
    split_dir: Dir3,
    twist: RangeVec3,
    swing: RangeVec3,
}

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
pub struct RangeVec3{
    min: Vec3,
    max: Vec3,
}
impl Default for RangeVec3 {
    fn default() -> Self {
        Self{
            min: Vec3::MIN,
            max: Vec3::MAX,
        }
    }
}



pub fn constrain_forward(
    child_affine: Affine3A,
    main_affine: Affine3A,
    constraint: RotationConstraint,
) -> Quat{
    let parent = child_affine.to_scale_rotation_translation().1;
    let theoretical = parent.conjugate() * main_affine.to_scale_rotation_translation().1;

    let (mut twist, mut swing) = theoretical.twist_swing(constraint.split_dir.as_vec3());

    let mut scaled_twist = twist.to_scaled_axis();
    let mut scaled_swing = swing.to_scaled_axis();

    scaled_twist = scaled_twist.clamp(constraint.twist.min, constraint.twist.max);
    scaled_swing = scaled_swing.clamp(constraint.swing.min, constraint.swing.max);

        
    twist = Quat::from_scaled_axis(scaled_twist);
    swing = Quat::from_scaled_axis(scaled_swing);

    (twist * swing) * parent    

}
