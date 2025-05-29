use bevy::{math::Affine3A, prelude::*};

use crate::utils::{AffineExtra, QuatExtra};

use super::{Joint, RotationConstraint};
// pub fn apply_constraint(
//     dir: Dir3A,
//     mut affine: Affine3A,
//     constraint: RotationConstraint,
// ) -> Affine3A{
//     let mut srt = affine.to_scale_rotation_translation();
    
//     // 1. Compute delta rotation FROM REST POSE
//     let rest_rot = constraint.relative_rotation;
//     let delta_rot = rest_rot.conjugate() * srt.1; // rest_rot^{-1} * current_rot
    
//     // 2. Decompose into TWIST (around dir) then SWING 
//     let (twist, swing) = delta_rot.twist_swing(dir.as_vec3a().into());
    
//     // 3. Apply constraints
//     let constrained_twist = Quat::constrain(twist, constraint.twist_constraint);
//     let constrained_swing = Quat::constrain(swing, constraint.swing_constraint);
    
//     // 4. RECOMPUTE rotation: rest_rot * (constrained_twist * constrained_swing)
//     srt.1 = rest_rot * (constrained_twist * constrained_swing);
        
//     affine = Affine3A::from_scale_rotation_translation(srt.0, srt.1, srt.2);
    
    
//     affine
// }


pub fn constrain_forward(
    up_dir: &mut Vec3A,
    p_i: &mut Vec3A,
    p_i1: Vec3A,
    child_constraint: &RotationConstraint,
    parent_rot: Quat,
    parent_joint: &Joint,
){
    //i think this needs to be changed to accomodate for multiple children, this
    //should really be run for every child joint.
    //constrain to the first child
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
