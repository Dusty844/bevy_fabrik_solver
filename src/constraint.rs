use bevy::{math::Affine3A, prelude::*};

use crate::utils::QuatExtra;

use super::{RotationConstraint, JointTransform};


pub fn constrain_forward(
    child_rotation: Quat,
    main_rotation: Quat,
    constraint: RotationConstraint,
) -> Quat{
    let parent = child_rotation;
    let theoretical = (parent.conjugate() * main_rotation);

    let (mut twist, mut swing) = theoretical.twist_swing(constraint.split_dir.as_vec3());

    let mut scaled_twist = twist.to_scaled_axis();
    let mut scaled_swing = swing.to_scaled_axis();

    scaled_twist = scaled_twist.clamp(constraint.twist.min, constraint.twist.max);
    scaled_swing = scaled_swing.clamp(constraint.swing.min, constraint.swing.max);

        
    twist = Quat::from_scaled_axis(scaled_twist).normalize();
    swing = Quat::from_scaled_axis(scaled_swing).normalize();

    (twist * swing) * parent    

}

pub fn constrain_backward(
    main_rotation: Quat,
    parent_rotation: Quat,
    constraint: RotationConstraint,
) -> Quat{
    let parent = parent_rotation;
    let theoretical = parent.conjugate() * main_rotation;

    let (mut twist, mut swing) = theoretical.twist_swing(constraint.split_dir.as_vec3());

    let mut scaled_twist = twist.to_scaled_axis();
    let mut scaled_swing = swing.to_scaled_axis();

    scaled_twist = scaled_twist.clamp(constraint.twist.min, constraint.twist.max);
    scaled_swing = scaled_swing.clamp(constraint.swing.min, constraint.swing.max);

        
    twist = Quat::from_scaled_axis(scaled_twist).normalize();
    swing = Quat::from_scaled_axis(scaled_swing).normalize();

    (twist * swing) * parent
}
