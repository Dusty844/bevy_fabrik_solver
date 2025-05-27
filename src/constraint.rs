use bevy::{math::Affine3A, prelude::*};

use crate::utils::{AffineExtra, QuatExtra};

use super::{Joint, RotationConstraint};
pub fn apply_constraint(
    dir: Dir3A,
    mut affine: Affine3A,
    constraint: RotationConstraint,
) -> Affine3A{
    let mut srt = affine.to_scale_rotation_translation();
    
    // 1. Compute delta rotation FROM REST POSE
    let rest_rot = constraint.relative_rotation;
    let delta_rot = rest_rot.conjugate() * srt.1; // rest_rot^{-1} * current_rot
    
    // 2. Decompose into TWIST (around dir) then SWING 
    let (twist, swing) = delta_rot.twist_swing(dir.as_vec3a().into());
    
    // 3. Apply constraints
    let constrained_twist = Quat::constrain(twist, constraint.twist_constraint);
    let constrained_swing = Quat::constrain(swing, constraint.swing_constraint);
    
    // 4. RECOMPUTE rotation: rest_rot * (constrained_twist * constrained_swing)
    srt.1 = rest_rot * (constrained_twist * constrained_swing);
        
    affine = Affine3A::from_scale_rotation_translation(srt.0, srt.1, srt.2);
    
    
    affine
}
