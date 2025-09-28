use super::{
    Joint,
    JointBookkeeping,
    JointTransform,
    EndEffector,
    EEJoint,
    Base,
    BaseJoint,
    JointChildren,
    JointParent,
};

use crate::constraint::*;


use rayon::prelude::*;
use std::sync::{Arc, Mutex, RwLock};
use bevy::{ecs::entity::EntityHashSet, math::Affine3A, platform::collections::HashMap, prelude::*, reflect::List, tasks::ParallelSlice};

use crate::utils::*;


pub fn solve(
    mut bk: ResMut<JointBookkeeping>,
    mut hierarchy_q: Query<(Entity, AnyOf<(&JointParent, &JointChildren)>)>,
    end_joints: Query<Entity, (With<EEJoint>, Without<JointChildren>)>,
    mut effector_joints: Query<(Entity, &EEJoint)>,
    mut constraint_q: Query<&RotationConstraint, With<Joint>>,
){

    let mut end_set = HashMap::new();
    for entity in end_joints.iter(){
        end_set.insert(entity, *bk.joints.lock().unwrap().get(&entity).unwrap());
    }
    
    
    
    //dont like having to have a query for the hierarchy components either but oh well there seems to be no good way to do this afaik
    forward_reach(&mut bk, end_set, hierarchy_q.reborrow(), effector_joints.reborrow(), constraint_q.reborrow());
        
}


fn forward_reach(
    bk: &mut JointBookkeeping,
    end_set: HashMap<Entity, (Joint, JointTransform)>,
    hierarchy_q: Query<(Entity, AnyOf<(&JointParent, &JointChildren)>)>,
    effector_joints: Query<(Entity, &EEJoint)>,
    constraint_q: Query<&RotationConstraint, With<Joint>>,
) {
    //is cloning really the best choice? ill have to benchmark at some point
    let mut current: Vec<Entity> = end_set.clone().into_iter().map(|(entity, (_, _))| entity).collect();
    let seen = Arc::new(RwLock::new(EntityHashSet::new()));

    loop{
        let next = Arc::new(Mutex::new(Vec::new()));
        
        current.par_iter().for_each(|main_entity|{
            seen.write().unwrap().insert(*main_entity);
            let main_affine = bk.joints.lock().unwrap().get(main_entity).unwrap().1.affine;
            let main_joint = bk.joints.lock().unwrap().get(main_entity).unwrap().0;
            let mut ee_c: usize = 0;
            let mut children_c = 0;
            let p_i1 = Arc::new(Mutex::new(Vec3A::ZERO));
            let mut p_i = if main_joint.halfway {
                main_affine.translation - (main_affine.to_scale_rotation_translation().1 * Vec3A::Y * 0.5)
            } else {
                main_affine.translation
            };
            let rots: Arc<Mutex<Vec<Quat>>> = Arc::new(Mutex::new(Vec::new()));
        
            if let Ok((_, (parent_maybe, child_maybe))) = hierarchy_q.get(*main_entity){
                // if child(ren) is, then attatch to child(ren)
                if let Some(children) = child_maybe{
                    children_c = children.0.len();
                    //should i par iter? why not?
                
                
                    children.0.par_iter().for_each(|child|{
                        let child_affine = bk.joints.lock().unwrap().get(child).unwrap().1.affine;
                        let child_joint = bk.joints.lock().unwrap().get(child).unwrap().0;

                        //offset needs to be based off the child, not the parent in this case
                        let offset = Vec3A::from(child_affine.to_scale_rotation_translation().1 * child_joint.offset);

                        let bottom_point = if child_joint.halfway{
                            //need to reimpl the utils functions
                            child_affine.translation - (child_affine.local_y().as_vec3a() * child_joint.length * 0.5) + offset
                        } else {
                            child_affine.translation + offset
                        };
                        let rot = if let Ok(constraint) = constraint_q.get(*main_entity){
                            constrain_forward(child_affine, main_affine, *constraint)
                        } else {
                            child_affine.to_scale_rotation_translation().1
                        };

                        rots.lock().unwrap().push(rot);
                        *p_i1.lock().unwrap() += bottom_point;

                    });

                    
                }
                if let Some(parent) = parent_maybe && !seen.read().unwrap().contains(&parent.0){
                    next.lock().unwrap().push(parent.0);
                }
            }

            if let Ok((_, ee_joint)) = effector_joints.get(*main_entity) {
                ee_c = 1;
                let ee = bk.ends.read().unwrap().get(&ee_joint.0).unwrap().0;
                let ee_affine = bk.ends.read().unwrap().get(&ee_joint.0).unwrap().1.affine;
                let rot = if ee.joint_copy_rotation {
                    ee_affine.to_scale_rotation_translation().1
                }else {
                    main_affine.to_scale_rotation_translation().1
                };
                let bottom_point = if ee.joint_center {
                    ee_affine.translation + (rot * Vec3A::Y * main_joint.length * 0.5)
                }else {
                    ee_affine.translation
                };
            
                rots.lock().unwrap().push(rot);
                *p_i1.lock().unwrap() += bottom_point;
            
            }
            *p_i1.lock().unwrap() /= (ee_c + children_c) as f32;

            let final_rot = rotation_averaging(&rots.lock().unwrap(), 5, main_affine.to_scale_rotation_translation().1);

            p_i = if main_joint.halfway {
                *p_i1.lock().unwrap() - (final_rot * Vec3A::Y * main_joint.length * 0.5)
            } else {
                *p_i1.lock().unwrap() - (final_rot * Vec3A::Y * main_joint.length)
            };

            let r_i = (*p_i1.lock().unwrap() - p_i).length();

            let lambda = main_joint.length / r_i;

            p_i = (1.0 - lambda) * *p_i1.lock().unwrap() + lambda * p_i;

            let final_translation = if main_joint.halfway {
                p_i + (final_rot * Vec3A::Y * main_joint.length * 0.5)
            } else {
                p_i
            };

        
            let new_affine = Affine3A::from_rotation_translation(final_rot, final_translation.into());

            bk.joints.lock().unwrap().get_mut(main_entity).unwrap().1.affine = new_affine;
        
        });
        {
            let mut next_lock = next.lock().unwrap();
            current.append(&mut next_lock);
        }
             
        // current.append(other);
        if current.is_empty(){
            break;
        }
        
    }
    
}

fn backward_reach(
    
){
    
}

fn rotation_averaging(
    vec: &Vec<Quat>,
    x: usize,
    start: Quat,

) -> Quat{
    let mut a = [[0.0; 4]; 4];
    for quat in vec {
        for i in 0..4 {
            for j in 0..4 {
                a[i][j] += quat.to_array()[i] * quat.to_array()[j];
            }
        }
    }

    let mut avg_rot = start;
    avg_rot = avg_rot.normalize();

    for _ in 0..x {
        let mut av = [0.0; 4];
        for i in 0..4{
            for j in 0..4{
                av[i] += a[i][j] * avg_rot.to_array()[j]
            }
        }
        avg_rot = Quat::from_array(av).normalize();
        
    }
    if avg_rot.to_array()[0] < 0.0 {
        let mut temp = avg_rot.to_array();
        temp.iter_mut().for_each(|x| *x *= -1.0);
        avg_rot = Quat::from_array(temp);
    }
    avg_rot
    
}
