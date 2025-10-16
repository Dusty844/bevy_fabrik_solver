use super::{
    Base, BaseJoint, EEJoint, EndEffector, Joint, JointBookkeeping, JointChildren, JointParent,
    JointTransform, IkGlobalSettings, RotationConstraint
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
    for x in 0..global_joint_settings.iterations {
        forward_reach(&mut bk, top_joints.reborrow(), hierarchy_q.reborrow(), effector_joints.reborrow(), constraint_q.reborrow());

        let new_dist = backward_reach(&mut bk, bottom_joints.reborrow(), hierarchy_q.reborrow(), effector_joints.reborrow(), constraint_q.reborrow());
        // println!("{:?}",bk.last_diff.distance(new_dist));
        if bk.last_diff.distance(new_dist) < global_joint_settings.minimum_tolerance {
            bk.last_diff = new_dist;
            // println!("last iteration count: {:?}", x);
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
            let main_transform = bk.joints.lock().unwrap().get(main_entity).unwrap().1;
            let main_joint = bk.joints.lock().unwrap().get(main_entity).unwrap().0;
            let mut ee_c: usize = 0;
            let mut children_c = 0;
            let local_z = main_transform.local_z();

            let initial_bottom_point = if main_joint.halfway {
                main_transform.translation - (main_transform.local_y() * main_joint.length * 0.5)
            } else {
                main_transform.translation - (main_transform.local_y() * main_joint.length)
            };
            let p_i1 = Arc::new(Mutex::new(Vec3::ZERO));
            let rots: Arc<Mutex<Vec<Quat>>> = Arc::new(Mutex::new(Vec::new()));
            let weights: Arc<Mutex<Vec<f32>>> = Arc::new(Mutex::new(Vec::new()));
            
            if let Ok((_, (parent_maybe, child_maybe))) = hierarchy_q.get(*main_entity) {
                if let Some(children) = child_maybe {
                    children_c = children.0.len();

                    //should i par iter? why not?
                    children.0.par_iter().for_each(|child| {
                        let child_transform = bk.joints.lock().unwrap().get(child).unwrap().1;
                        let child_joint = bk.joints.lock().unwrap().get(child).unwrap().0;

                        

                        let top_point = if child_joint.halfway {
                            child_transform.translation - (child_transform.local_y() * child_joint.length * 0.5)
                        } else {
                            child_transform.translation 
                        };
                        let dir = (top_point - initial_bottom_point).normalize();
                        let in_rot = main_transform.aligned_by(Dir3::Y, dir, Dir3::Z, local_z).rotation;
                        let (rot, weight) = if let Ok(constraint) = constraint_q.get(*child) {
                            (constrain_forward(child_transform.rotation, in_rot, *constraint), constraint.weight)
                        } else {
                            (in_rot, 1.0)
                        };
                        

                        rots.lock().unwrap().push(rot);
                        weights.lock().unwrap().push(weight);
                        *p_i1.lock().unwrap() += top_point;
                    });
                }
                if let Some(parent) = parent_maybe
                    && !seen.read().unwrap().contains(&parent.0)
                {
                    next.lock().unwrap().push(parent.0);
                }
            }            

            if let Ok((_, ee_joint)) = effector_joints.get(*main_entity) {
                //whole thing needs a rework, i can totally do this with less of these ugly if variables
                ee_c = 1;
                let ee = bk.ends.read().unwrap().get(&ee_joint.0).unwrap().0;
                let ee_transform = bk.ends.read().unwrap().get(&ee_joint.0).unwrap().1;
                let copy_rot = if ee.joint_copy_rotation {
                    ee_transform.rotation
                } else {
                    main_transform.rotation
                };
                let top_point = if ee.joint_center {
                    ee_transform.translation + (copy_rot * Vec3::Y * main_joint.length * 0.5)
                } else {
                    ee_transform.translation
                };
                let ee_bottom_point = if main_joint.halfway {
                    ee_transform.translation - (copy_rot * Vec3::Y * main_joint.length * 0.5)
                } else {
                    ee_transform.translation - (copy_rot * Vec3::Y * main_joint.length)
                };
                let dir = if ee.joint_copy_rotation {
                    ee_transform.local_y().as_vec3()
                } else {
                    (top_point - ee_bottom_point).normalize()  
                };
                
                let rot = if ee.joint_copy_rotation {
                    ee_transform.rotation
                } else {
                    main_transform.aligned_by(Dir3::Y, dir, Dir3::Z, local_z).rotation
                };

                rots.lock().unwrap().push(rot);
                weights.lock().unwrap().push(1.0); //weight for end effectors?
                *p_i1.lock().unwrap() += top_point;
            }
            *p_i1.lock().unwrap() /= (ee_c + children_c) as f32;

            let top_point = *p_i1.lock().unwrap();

            //new
            let final_rot = if (children_c + ee_c) <= 1 {
                rots.lock().unwrap()[0]
            } else {
                rotation_averaging(
                    &rots.lock().unwrap(),
                    &weights.lock().unwrap(),
                    5,
                    main_transform.rotation,
                )
            };
            
            let bottom_point = if main_joint.halfway {
                top_point - (final_rot * Vec3::Y * main_joint.length * 0.5)
            } else {
                top_point - (final_rot * Vec3::Y * main_joint.length)
            };

            let dir = (top_point - bottom_point).normalize();
            
            let final_translation = if main_joint.halfway {
                top_point - (dir * main_joint.length * 0.5)
            } else {
                top_point - (dir * main_joint.length)
            };

            
            

            let mut new_transform = main_transform.aligned_by(Dir3::Y, dir, Dir3::Z, local_z);

            new_transform.translation = final_translation;

            bk.joints.lock().unwrap().get_mut(main_entity).unwrap().1 = new_transform;
            
                                
            
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
        if let Ok((_, (parent_maybe, child_maybe))) = hierarchy_q.get(main_entity) {
            if let Some(children) = child_maybe {
                let main_joint = bk.joints.lock().unwrap().get(&main_entity).unwrap().0;
                let main_transform = bk.joints.lock().unwrap().get(&main_entity).unwrap().1;
                let base_transform = bk.bases.read().unwrap().get(&base_joint.0).unwrap().1;
                let bottom_point = base_transform.translation;
                let mut top_point = Vec3::ZERO;
                for entity in children.0.iter() {
                    let child_joint = bk.joints.lock().unwrap().get(entity).unwrap().0;
                    let child_transform = bk.joints.lock().unwrap().get(entity).unwrap().1;
                    top_point += if child_joint.halfway{
                        child_transform.translation - (child_transform.local_y() * child_joint.length * 0.5)
                    } else {
                        child_transform.translation
                    };
                    if seen.read().unwrap().get(entity).is_none() {
                        current.push(*entity);
                    }
                }
                top_point /= children.0.len() as f32;

                let dir = (top_point - bottom_point).normalize();
                let new_translation = if main_joint.halfway {
                    bottom_point + (dir * main_joint.length * 0.5)  
                } else {
                    bottom_point + (dir * main_joint.length)
                };

                let local_z = main_transform.local_z();

                let mut new_transform = main_transform.aligned_by(Dir3::Y, dir, Dir3::Z, local_z);

                new_transform.translation = new_translation;

                if let Ok((_, ee_joint)) = effector_joints.get(main_entity) {
                    let (ee, ee_transform) = *bk.ends.read().unwrap().get(&ee_joint.0).unwrap();
                    let dist = ee_transform.translation - new_transform.translation;
                    *end_dists.lock().unwrap() += dist;
                }

                bk.joints.lock().unwrap().get_mut(&main_entity).unwrap().1 = new_transform;
                seen.write().unwrap().insert(main_entity);
            }
        }
    }

    loop {
        let next: Arc<Mutex<Vec<Entity>>> = Arc::new(Mutex::new(Vec::new()));

        current.par_iter_mut().for_each(|main_entity| {
            let main_transform = bk.joints.lock().unwrap().get(main_entity).unwrap().1;
            let main_joint = bk.joints.lock().unwrap().get(main_entity).unwrap().0;

            if let Ok((_, (parent_maybe, child_maybe))) = hierarchy_q.get(*main_entity) {
                if let Some(parent) = parent_maybe {
                    let parent_transform = bk.joints.lock().unwrap().get(&parent.0).unwrap().1;
                    let parent_joint = bk.joints.lock().unwrap().get(&parent.0).unwrap().0;

                    let initial_top_point = if main_joint.halfway {
                        main_transform.translation + (main_transform.local_y() * main_joint.length * 0.5)
                    } else {
                        main_transform.translation + (main_transform.local_y() * main_joint.length)
                    };

                    let bottom_point = if parent_joint.halfway {
                        parent_transform.translation + (parent_transform.local_y() * parent_joint.length * 0.5)
                    } else {
                        parent_transform.translation + (parent_transform.local_y() * parent_joint.length)
                    } + (parent_transform.rotation * main_joint.offset);

                    let local_z = main_transform.local_z();

                    
                    let initial_dir = (initial_top_point - bottom_point).normalize();
                    
                    // let mut rot = main_transform.aligned_by(Dir3::Y, initial_dir, Dir3::Z, local_z).rotation;
                    let mut rot = Quat::from_rotation_arc(Vec3::Y, initial_dir);
                    let in_rot = rot;

                    if let Ok(constraint) = constraint_q.get(*main_entity){
                        rot = constrain_backward(rot, parent_transform.rotation, *constraint);
                    }

                    rot = rot.slerp(in_rot, 0.2);

                    let top_point = bottom_point + (rot * Vec3::Y * main_joint.length);
                    

                    
                    let dir = (top_point - bottom_point).normalize();
                    
                    let new_translation = if main_joint.halfway {
                        bottom_point + (dir * main_joint.length * 0.5)  
                    } else {
                        bottom_point + (dir * main_joint.length)
                    };

                    


                    // let mut new_transform = main_transform.aligned_by(Dir3::Y, dir, Dir3::Z, local_z);
                    let mut new_transform = main_transform;
                    new_transform.rotation = rot;

                    new_transform.translation = new_translation;

                    

                    if let Ok((_, ee_joint)) = effector_joints.get(*main_entity) {
                        let (ee, ee_transform) = *bk.ends.read().unwrap().get(&ee_joint.0).unwrap();
                        let dist = ee_transform.translation - new_transform.translation;
                        *end_dists.lock().unwrap() += dist;
                    }
                    

                    bk.joints.lock().unwrap().get_mut(main_entity).unwrap().1 = new_transform;

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
