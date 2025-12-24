use bevy::{color::palettes::css::{BLUE_VIOLET, GREEN, RED}, prelude::*};
use super::{Joint, JointTransform, JointChildren};

pub fn joint_directional_gizmos(
    j_entity_q: Query<Entity, With<Joint>>,
    j_q: Query<&Joint>,
    jt_q: Query<&JointTransform>,
    children_q: Query<&JointChildren>,
    mut gizmos: Gizmos,
){
    
    for main_entity in j_entity_q.iter(){
        let main_jt = jt_q.get(main_entity).unwrap();
        let main_j = j_q.get(main_entity).unwrap();
        let translation = main_jt.translation - (main_jt.rotation * main_j.visual_offset);
        let main_up = main_jt.local_y();
        let main_forward = main_jt.local_z();
        gizmos.arrow(translation, translation + (main_up * main_j.length), BLUE_VIOLET);
        gizmos.arrow(translation, translation + (main_forward * main_j.length * 0.5), RED);

        // if let Ok(children) = children_q.get(main_entity) {
        //     for child in children.0.iter() {
        //         let child_j = j_q.get(*child).unwrap();
        //         let child_jt = jt_q.get(*child).unwrap();
        //         let child_up = child_jt.local_y();
        //         let child_forward = child_jt.local_z();
        //         let child_translation = child_jt.translation - (child_jt.rotation * child_j.visual_offset);
                
        //         let main_fwd_proj = (main_forward.as_vec3() - main_up * main_forward.dot(main_up.as_vec3())).normalize();
        //         let child_fwd_proj = (child_forward.as_vec3() - child_up * child_forward.dot(child_up.as_vec3())).normalize();

        //         let twist_angle = f32::atan2(main_up.as_vec3().dot(main_fwd_proj.cross(child_fwd_proj)), main_fwd_proj.dot(child_fwd_proj));

        //         gizmos.arrow(translation, translation + (main_fwd_proj * main_j.length), GREEN);
        //     }
        // }
    }
}
