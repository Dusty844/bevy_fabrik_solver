use bevy::prelude::*;

use crate::utils::QuatExtra;

use super::{RotationConstraint};


pub fn constrain_forward(
    child_rotation: Quat,
    main_rotation: Quat,
    constraint: RotationConstraint,
) -> Quat{
    let parent = child_rotation;
    let theoretical = parent.conjugate() * constraint.identity.conjugate() * main_rotation;

    let dir = parent.conjugate() * constraint.identity.conjugate() * constraint.split_dir.as_vec3();

    let (mut twist, mut swing) = theoretical.twist_swing(dir);

    let mut scaled_twist = twist.to_scaled_axis();
    let mut scaled_swing = swing.to_scaled_axis();

    scaled_twist = scaled_twist.clamp_length_max(constraint.twist);
    scaled_swing = scaled_swing.clamp_length_max(constraint.swing);

        
    twist = Quat::from_scaled_axis(scaled_twist).normalize();
    swing = Quat::from_scaled_axis(scaled_swing).normalize();

    twist * swing * constraint.identity * parent   

}

