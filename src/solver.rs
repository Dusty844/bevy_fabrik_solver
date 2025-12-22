use super::{
    BaseJoint, EEJoint, Joint, JointBookkeeping, JointChildren, JointParent,
    IkGlobalSettings, RotationConstraint
};

use crate::constraint::*;

use bevy::{ecs::entity::EntityHashSet, prelude::*};
use rayon::prelude::*;
use std::sync::{Arc, Mutex, RwLock};

use crate::utils::*;

pub fn solve(
    mut bk: ResMut<JointBookkeeping>,
    global_joint_settings: Res<IkGlobalSettings>,
    mut hierarchy_q: Query<(Entity, AnyOf<(&JointParent, &JointChildren)>)>,
    mut top_joints: Query<Entity, (With<EEJoint>, Without<JointChildren>)>,
    mut bottom_joints: Query<(Entity, &BaseJoint), (With<BaseJoint>, Without<JointParent>)>,
    mut effector_joints: Query<(Entity, &EEJoint)>,
    mut constraint_q: Query<&RotationConstraint, With<Joint>>,
) {
    for _ in 0..global_joint_settings.iterations {
        forward_reach(&mut bk, top_joints.reborrow(), hierarchy_q.reborrow(), effector_joints.reborrow(), constraint_q.reborrow());

        let new_dist = backward_reach(&mut bk, bottom_joints.reborrow(), hierarchy_q.reborrow(), effector_joints.reborrow(), constraint_q.reborrow());
        if bk.last_diff.distance(new_dist) < global_joint_settings.minimum_tolerance {
            bk.last_diff = new_dist;
            break;
        }
        bk.last_diff = new_dist;
    }
    
}

fn forward_reach(
    bk: &mut JointBookkeeping,
    top_joints: Query<Entity, (With<EEJoint>, Without<JointChildren>)>,
    hierarchy_q: Query<(Entity, AnyOf<(&JointParent, &JointChildren)>)>,
    effector_joints: Query<(Entity, &EEJoint)>, //includes effector joints that are not at the end of any chain
    constraint_q: Query<&RotationConstraint, With<Joint>>,
) {
    
    //is cloning really the best choice? ill have to benchmark at some point
    let mut current: Vec<Entity> = top_joints.into_iter().collect();
    let seen = Arc::new(RwLock::new(EntityHashSet::new()));

    loop {
        let next = Arc::new(Mutex::new(Vec::new()));

        current.par_iter().for_each(|main_entity| {
            seen.write().unwrap().insert(*main_entity);
            let mut main_transform = bk.joints.lock().unwrap().get(main_entity).unwrap().1;
            let main_joint = bk.joints.lock().unwrap().get(main_entity).unwrap().0;
            let mut ee_c: usize = 0;
            let mut children_c = 0;

            let initial_bottom_point = main_transform.translation - (main_transform.rotation * main_joint.offset);

            let mut p_i1 = Vec3::ZERO;
            let mut rots = vec![];
            let mut weights = vec![];
            
            if let Ok((_, (parent_maybe, child_maybe))) = hierarchy_q.get(*main_entity) {
                if let Some(children) = child_maybe {
                    children_c = children.0.len();

                    children.0.iter().for_each(|child| {
                        let child_transform = bk.joints.lock().unwrap().get(child).unwrap().1;
                        let child_joint = bk.joints.lock().unwrap().get(child).unwrap().0;

                        let child_bottom_point = child_transform.translation - (child_transform.rotation * child_joint.offset);
                        

                        let dir = (child_bottom_point - initial_bottom_point).normalize();
                        
                        let identity = if let Ok(constraint) = constraint_q.get(*main_entity) {
                            constraint.identity
                        }else{
                            Quat::IDENTITY
                        };
                        
                        let (rot, weight) = if let Ok(constraint) = constraint_q.get(*child) {
                            let bind_dir = (identity * Vec3::Y).normalize();
                            let dir_local = child_transform.rotation.inverse() * dir;

                            let constrained_local = constrain_direction(dir_local, bind_dir, constraint.x_max, 0.8);

                            let constrained_global = child_transform.rotation * constrained_local;
                            
                            let in_rot = Quat::from_rotation_arc(Vec3::Y, constrained_global);
                            
                            (in_rot, constraint.weight)
                        } else {
                            let in_rot = Quat::from_rotation_arc(Vec3::Y, dir);
                            (in_rot, 1.0)
                        };
                        

                        rots.push(rot);
                        weights.push(weight);
                        p_i1 += child_bottom_point;
                    });
                }
                if let Some(parent) = parent_maybe
                    && !seen.read().unwrap().contains(&parent.0)
                {
                    next.lock().unwrap().push(parent.0);
                }
            }            

            if let Ok((_, ee_joint)) = effector_joints.get(*main_entity) {
                ee_c = 1;
                let ee = bk.ends.read().unwrap().get(&ee_joint.0).unwrap().0;
                let ee_transform = bk.ends.read().unwrap().get(&ee_joint.0).unwrap().1;

                let (rot, possible_top_point) = if ee.joint_copy_rotation{
                    (ee_transform.rotation, ee_transform.translation)    
                }else{
                    let dir = (ee_transform.translation - initial_bottom_point).normalize();
                    let new = Quat::from_rotation_arc(Vec3::Y, dir);
                    (new, ee_transform.translation)        
                };
                
                rots.push(rot);
                weights.push(1.0); //weight for end effectors?
                p_i1 += possible_top_point;
            }
            p_i1 /= (ee_c + children_c) as f32;

            let new_top_point = p_i1;

            //new
            let final_rot = if (children_c + ee_c) <= 1 {
                rots[0]
            } else {
                rotation_averaging(
                    &rots,
                    &weights,
                    5,
                    main_transform.rotation,
                )
            };

            let new_bottom_point = new_top_point - (final_rot * Vec3::Y * main_joint.length);


            

            let final_translation = new_bottom_point + (final_rot * main_joint.offset);
            main_transform.translation = final_translation;
            main_transform.rotation = final_rot;

            bk.joints.lock().unwrap().get_mut(main_entity).unwrap().1 = main_transform;
            
        });
        {
            current.clear();
            let mut next_lock = next.lock().unwrap();
            current.append(&mut next_lock);
        }

        
        if current.is_empty() {
            break;
        }
    }
}

fn backward_reach(
    bk: &mut JointBookkeeping,
    bottom_joints: Query<(Entity, &BaseJoint), (With<BaseJoint>, Without<JointParent>)>,
    hierarchy_q: Query<(Entity, AnyOf<(&JointParent, &JointChildren)>)>,
    effector_joints: Query<(Entity, &EEJoint)>,
    constraint_q: Query<&RotationConstraint, With<Joint>>,
) -> Vec3 {
    let mut current: Vec<Entity> = vec![];
    let seen = Arc::new(RwLock::new(EntityHashSet::new()));
    let end_dists = Arc::new(Mutex::new(Vec3::ZERO));
    for (main_entity, base_joint) in bottom_joints.iter() {
        if let Ok((_, (_, child_maybe))) = hierarchy_q.get(main_entity) {
            if let Some(children) = child_maybe {
                let main_joint = bk.joints.lock().unwrap().get(&main_entity).unwrap().0;
                let mut main_transform = bk.joints.lock().unwrap().get(&main_entity).unwrap().1;
                let new_bottom_point = bk.bases.read().unwrap().get(&base_joint.0).unwrap().1.translation;
                main_transform.translation = new_bottom_point + (main_transform.rotation * main_joint.offset);

                for child in children.0.iter(){
                    current.push(*child);
                }

                if let Ok((_, ee_joint)) = effector_joints.get(main_entity) {
                    let (_, ee_transform) = *bk.ends.read().unwrap().get(&ee_joint.0).unwrap();
                    let dist = ee_transform.translation - main_transform.translation;
                    *end_dists.lock().unwrap() += dist;
                }

                bk.joints.lock().unwrap().get_mut(&main_entity).unwrap().1 = main_transform;
                seen.write().unwrap().insert(main_entity);
            }
        }
    }

    
    loop {
        let next: Arc<Mutex<Vec<Entity>>> = Arc::new(Mutex::new(Vec::new()));

        current.par_iter_mut().for_each(|main_entity| {
            let mut main_transform = bk.joints.lock().unwrap().get(main_entity).unwrap().1;
            let main_joint = bk.joints.lock().unwrap().get(main_entity).unwrap().0;

            if let Ok((_, (parent_maybe, child_maybe))) = hierarchy_q.get(*main_entity) {
                if let Some(parent) = parent_maybe {
                    let parent_transform = bk.joints.lock().unwrap().get(&parent.0).unwrap().1;
                    let parent_joint = bk.joints.lock().unwrap().get(&parent.0).unwrap().0;
                    let parent_bottom_point = parent_transform.translation - (parent_transform.rotation * parent_joint.offset);
                    let parent_top_joint = parent_bottom_point + (parent_transform.rotation * Vec3::Y * parent_joint.length);
                    main_transform.translation = parent_top_joint + (main_transform.rotation * main_joint.offset);


                    if let Ok((_, ee_joint)) = effector_joints.get(*main_entity) {
                        let (_, ee_transform) = *bk.ends.read().unwrap().get(&ee_joint.0).unwrap();
                        let dist = ee_transform.translation - main_transform.translation;
                        *end_dists.lock().unwrap() += dist;
                    }

                    bk.joints.lock().unwrap().get_mut(main_entity).unwrap().1 = main_transform;

                    
                    
                }
                if let Some(children) = child_maybe {
                    for child in children.0.iter() {
                        if !seen.read().unwrap().contains(child) {
                            next.lock().unwrap().push(*child);
                        }
                    }
                }
            }
        });
        {
            current.clear();
            let mut next_lock = next.lock().unwrap();
            current.append(&mut next_lock);
        }

        if current.is_empty() {
            break;
        }
    }
        
    *end_dists.lock().unwrap()
}

