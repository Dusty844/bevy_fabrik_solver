use bevy::{color::palettes::css::{BLUE, GREEN, LIGHT_GRAY, RED, WHITE}, prelude::*};

use crate::JointParent;

use super::{Joint, JointTransform, RotationConstraint};

// We can create our own gizmo config group!
#[derive(Default, GizmoConfigGroup, Reflect)]
pub struct IkGizmos;

pub struct IkGizmosPlugin;

impl Plugin for IkGizmosPlugin{
    fn build(&self, app: &mut App) {

        app.add_systems(Startup, gizmo_config_setup);
        app.add_systems(PostUpdate, (joint_directional_gizmos, rotation_constraint_gizmos).after(TransformSystems::Propagate));
        app.init_gizmo_group::<IkGizmos>();
    }
}


pub fn gizmo_config_setup(
    mut config_store: ResMut<GizmoConfigStore>,
){
    let (config, _) = config_store.config_mut::<IkGizmos>();
    config.depth_bias = -1.0;
}

//simple directions
pub fn joint_directional_gizmos(
    j_q: Query<(&Joint, &JointTransform), With<Joint>>,
    mut gizmos: Gizmos<IkGizmos>,
){
    
    for (main_j, main_jt) in j_q.iter(){
        let translation = main_jt.translation - (main_jt.rotation * main_j.visual_offset);
        let main_up = main_jt.local_y();
        let main_forward = main_jt.local_z();
        let main_right = main_jt.local_x();
        gizmos.arrow(translation, translation + (main_up * main_j.length), BLUE);
        gizmos.arrow(translation - (main_forward * main_j.length * 0.2) + (main_up * main_j.length * 0.2), translation + (main_forward * main_j.length * 0.2) + (main_up * main_j.length * 0.2), RED);
        gizmos.arrow(translation - (main_right * main_j.length * 0.2) + (main_up * main_j.length * 0.2), translation + (main_right * main_j.length * 0.2) + (main_up * main_j.length * 0.2), GREEN);
    }
}

pub fn rotation_constraint_gizmos(
    constraint_q: Query<(Entity, &RotationConstraint, &JointParent)>,
    joint_q: Query<(&Joint, &JointTransform)>,
    mut gizmos: Gizmos<IkGizmos>,
){
    for (main, constraint, parent) in constraint_q.iter(){
        if let Ok((main_joint, main_t)) = joint_q.get(main) && let Ok((_, parent_t)) = joint_q.get(parent.0){
            //first swing constraint arrows in white
            

            let translation = main_t.translation - (main_t.rotation * main_joint.visual_offset);

            
            //identity (centre direction)    
            let identity_up_dir = constraint.identity * parent_t.local_y().as_vec3();
            gizmos.arrow(translation, translation + (identity_up_dir * main_joint.length * 0.333), WHITE);

            
            //x swing limits
            let limit_pos_x = Quat::from_rotation_x(constraint.x_max) * identity_up_dir;
            let limit_neg_x = Quat::from_rotation_x(-constraint.x_max) * identity_up_dir;

            gizmos.arrow(translation, translation + (limit_pos_x * main_joint.length * 0.25), LIGHT_GRAY);
            gizmos.arrow(translation, translation + (limit_neg_x * main_joint.length * 0.25), LIGHT_GRAY);


            
            //z swing limits
            let limit_pos_z = Quat::from_rotation_z(constraint.z_max) * identity_up_dir;
            let limit_neg_z = Quat::from_rotation_z(-constraint.z_max) * identity_up_dir;

            gizmos.arrow(translation, translation + (limit_pos_z * main_joint.length * 0.25), LIGHT_GRAY);
            gizmos.arrow(translation, translation + (limit_neg_z * main_joint.length * 0.25), LIGHT_GRAY);


            //twist limits
            let rot = Transform::IDENTITY.aligned_by(Vec3::Y, main_t.local_y(), Vec3::Z, constraint.identity * parent_t.local_z()).rotation;
            let iso = Isometry3d::new(translation + (main_t.local_y() * main_joint.length * 0.2), rot);

            gizmos.arc_3d(constraint.x_max * 2.0, main_joint.length * 0.15, iso, LIGHT_GRAY);

            
        }
    }   
}
