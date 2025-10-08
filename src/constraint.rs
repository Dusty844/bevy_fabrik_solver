use bevy::{math::Affine3A, prelude::*};

use crate::utils::QuatExtra;

use super::RotationConstraint;


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
