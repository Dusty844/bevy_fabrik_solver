use bevy::{math::Affine3A, prelude::*};

use crate::utils::{AffineExtra, QuatExtra};

use super::{Joint, RotationConstraint};


pub fn constrain_forward(
    up_dir: &mut Vec3A,
    p_i: &mut Vec3A,
    p_i1: Vec3A,
    child_constraint: &RotationConstraint,
    parent_rot: Quat,
    parent_joint: &Joint,
){
    let local_z = Dir3::new_unchecked(parent_rot * Vec3::Z);
    let mut theoretical = Quat::IDENTITY;
    theoretical.align(Dir3::Y, Dir3A::new_unchecked(*up_dir), Dir3::Z, local_z);

    theoretical = parent_rot.conjugate() * theoretical;
    let (mut twist, mut swing) = theoretical.twist_swing(child_constraint.dir);

    //convert to scaled axes to be clamped.
    let mut scaled_twist = twist.to_scaled_axis();
    let mut scaled_swing = swing.to_scaled_axis();

    scaled_twist = scaled_twist.clamp(child_constraint.twist_min, child_constraint.twist_max);
    scaled_swing = scaled_swing.clamp(child_constraint.swing_min, child_constraint.swing_max);

    twist = Quat::from_scaled_axis(scaled_twist);
    swing = Quat::from_scaled_axis(scaled_swing);

    let out_rot = swing * twist;
    *up_dir = out_rot * Vec3A::Y;
    *p_i = if parent_joint.halfway {
        p_i1 - (*up_dir * parent_joint.length * 0.5)
    } else {
        p_i1 - (*up_dir * parent_joint.length)
    };
    
}

pub fn constrain_direction(
    up_dir: &mut Vec3A,
    child_constraint: &RotationConstraint,
    parent_rot: Quat,
){
    
    let local_z = Dir3::new_unchecked(parent_rot * Vec3::Z);
    let mut theoretical = Quat::IDENTITY;
    theoretical.align(Dir3::Y, Dir3A::new_unchecked(*up_dir), Dir3::Z, local_z);

    theoretical = parent_rot.conjugate() * theoretical;
    let (mut twist, mut swing) = theoretical.twist_swing(child_constraint.dir);

    //convert to scaled axes to be clamped.
    let mut scaled_twist = twist.to_scaled_axis();
    let mut scaled_swing = swing.to_scaled_axis();

    scaled_twist = scaled_twist.clamp(child_constraint.twist_min, child_constraint.twist_max);
    scaled_swing = scaled_swing.clamp(child_constraint.swing_min, child_constraint.swing_max);

    twist = Quat::from_scaled_axis(scaled_twist);
    swing = Quat::from_scaled_axis(scaled_swing);

    let out_rot = swing * twist;
    *up_dir = out_rot * Vec3A::Y;
    
    
}

fn constrain_rotation(
    rot: Quat,
    parent_rot: Quat,
    constraint: &RotationConstraint
) -> Quat{
    let next = parent_rot.conjugate() * rot;
    let (mut twist, mut swing) = next.twist_swing(constraint.dir);

    let mut scaled_twist = twist.to_scaled_axis();
    let mut scaled_swing = swing.to_scaled_axis();

    scaled_twist = scaled_twist.clamp(constraint.twist_min, constraint.twist_max);
    scaled_swing = scaled_swing.clamp(constraint.swing_min, constraint.swing_max);

    twist = Quat::from_scaled_axis(scaled_twist);
    swing = Quat::from_scaled_axis(scaled_swing);

    let final_rot = swing * twist;
    final_rot
}


