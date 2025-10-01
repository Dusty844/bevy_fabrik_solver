use bevy::{math::Affine3A, prelude::*};

use crate::utils::QuatExtra;

#[derive(Component, Copy, Clone, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
pub struct RotationConstraint{
    pub identity: Quat,
    pub other_weight: f32,
    pub split_dir: Dir3,
    pub twist: RangeVec3,
    pub swing: RangeVec3,
}

impl Default for RotationConstraint {
    fn default() -> Self {
        let min = Vec3::splat(-0.75);
        let max = Vec3::splat(0.75);
        Self{
            identity: Quat::IDENTITY,
            other_weight: 0.5,
            split_dir: Dir3::Y,
            twist: RangeVec3::new(min, max),
            swing: RangeVec3::new(min, max),
        }
    }
}
//would be more effective to switch to single axes twist two axes swing but i havent figured out the maths for that yet.

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

impl RangeVec3 {
    fn new(min: Vec3, max:Vec3) -> RangeVec3 {
        RangeVec3 { min, max }
    }
}



pub fn constrain_forward(
    child_affine: Affine3A,
    main_affine: Affine3A,
    constraint: RotationConstraint,
) -> Quat{
    let parent = child_affine.to_scale_rotation_translation().1;
    let theoretical = constraint.identity.conjugate() * (parent.conjugate() * main_affine.to_scale_rotation_translation().1);

    let (mut twist, mut swing) = theoretical.twist_swing(constraint.split_dir.as_vec3());

    let mut scaled_twist = twist.to_scaled_axis();
    let mut scaled_swing = swing.to_scaled_axis();

    scaled_twist = scaled_twist.clamp(constraint.twist.min, constraint.twist.max);
    scaled_swing = scaled_swing.clamp(constraint.swing.min, constraint.swing.max);

        
    twist = Quat::from_scaled_axis(scaled_twist);
    swing = Quat::from_scaled_axis(scaled_swing);

    (twist * swing) * parent    

}
