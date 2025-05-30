use bevy::ecs::relationship::RelationshipSourceCollection;
use bevy::math::Affine3A;
use bevy::platform::collections::HashSet;
use bevy::prelude::*;
use std::collections::VecDeque;



use crate::constraint::*;
use crate::RotationConstraint;
use crate::utils::*;
use super::{
    Joint,
    JointParent,
    JointChildren,
    Base,
    EndEffectorJoint,
    JointTransform,
    IkGlobalSettings
};



#[allow(clippy::type_complexity)]
#[allow(clippy::too_many_arguments)]
pub fn solve(
    base_q: Query<(Entity, &Base)>,
    joint_q: Query<(Entity, &Joint, Option<&RotationConstraint>)>,
    joint_parent_q: Query<&JointParent>,
    joint_children_q: Query<&JointChildren>,
    ee_joint_q: Query<&EndEffectorJoint>,
    mut joint_transforms: Query<(Entity, &mut JointTransform, Option<&ChildOf>)>,
    mut transforms: ParamSet<(
        Query<(&mut Transform, &GlobalTransform)>,
        TransformHelper,
    )>,
    settings: Res<IkGlobalSettings>,
){
    //SCALE IS NOT SUPPORTED
    for (entity, mut joint_t, _) in joint_transforms.iter_mut(){
        let gt = transforms.p1().compute_global_transform(entity).unwrap();
        let srt = gt.to_scale_rotation_translation();
        joint_t.affine = Affine3A::from_scale_rotation_translation(Vec3::ONE, srt.1, srt.2);
    }  


    
    for (base_entity, base) in base_q.iter(){
        if joint_parent_q.get(base.0).is_ok(){
            warn_once!("The Joint Entity {} has both a Base and a JointParent, and will be skipped in the solver, alleviate this by removing the BaseJoint component or the JointParent Component", base.0);
            continue;
        }
        if joint_q.get(base.0).is_err() {
            continue;
        }
        let start_entity = joint_q.get(base.0).unwrap();

        let base_translation = transforms.p1().compute_global_transform(base_entity).unwrap().to_scale_rotation_translation();
        

        

        //the sub chain [i] contains the start of a sub chain where endpoints [i] is the endpoint of said
        //subchain, the first one always extends from the root to the first endpoint, and is the
        //main chain. everything else extends from that.
        // at the moment, the subchain is the parent entity, although it may be correct to make it the child instead.
        let (end_points, sub_chains) = forward_reach_setup(start_entity, joint_children_q, joint_q, ee_joint_q);

        let sub_chain_length = sub_chains.len();
        
        //first entity is the endpoint, second is the sub chain, third is the end effector
        let ignore_chains = check_reachable(end_points.clone(), sub_chains, ee_joint_q, joint_transforms.reborrow());

        if ignore_chains.len() == sub_chain_length {
                        
            forward_reach(joint_transforms.reborrow(), joint_q, joint_children_q, joint_parent_q, ee_joint_q, end_points.clone());

            let _ = backward_reach(joint_transforms.reborrow(), joint_q, start_entity, joint_children_q, joint_parent_q, ee_joint_q, base_translation.2.into(), base_translation.1);
            
            
        } else{
            //summed dist of the end joints to end effectors. if this value and the summed
            // dist returned by the backward reach at the end of the iteration
            // are the same, then break out of the loop. otherwise set the
            // summed dist to returned dist at the end.
            let mut summed_dist = 0.0;
            
            for _ in 0..settings.iterations {
                forward_reach(joint_transforms.reborrow(), joint_q, joint_children_q, joint_parent_q, ee_joint_q, end_points.clone());

                let new_dist = backward_reach(joint_transforms.reborrow(), joint_q, start_entity, joint_children_q, joint_parent_q, ee_joint_q, base_translation.2.into(), base_translation.1);

                if new_dist == summed_dist || new_dist <= settings.minimum_tolerance{
                    break;
                }else{
                    summed_dist = new_dist;
                }
            }
        }

        let mut i = 0;
        //reset positions
        let mut current = vec![start_entity.0];
        while !current.is_empty() {
            
            while i < current.len(){
                let (_, current_jt, parent_maybe) = joint_transforms.get(current[i]).unwrap();
                if let Some(parent) = parent_maybe{
                    //current has a parent.
                    let new_gt = transforms.p1().compute_global_transform(parent.0).unwrap();
                    //normally i'd inverse, but should i for this setup? trying something new.
                    let new_affine = new_gt.affine().inverse();
                    let final_affine = new_affine * current_jt.affine;
                    let srt = final_affine.to_scale_rotation_translation();
                    
                    let new_transform = Transform::from_scale(srt.0).with_rotation(srt.1).with_translation(srt.2);
                    if new_transform.is_finite() {
                        *transforms.p0().get_mut(current[i]).unwrap().0 = new_transform;
                    }
                    
                    
                } else{

                    let srt = current_jt.affine.to_scale_rotation_translation();
                    //removing scale for now.
                    let new_transform = Transform::from_scale(Vec3::ONE).with_rotation(srt.1).with_translation(srt.2);
                    if new_transform.is_finite() {
                        *transforms.p0().get_mut(current[i]).unwrap().0 = new_transform;
                    }
                    
                }
                if let Ok(joint_children) = joint_children_q.get(current[i]){
                    current[i] = joint_children.0[0];
                    if joint_children.0.len() > 1 {
                        for c in 1..joint_children.len(){
                            current.add(joint_children.0[c]);
                        }
                    }
                }else{
                    current.remove(i);
                }
                
                
            }
            i += 1;
        }
        
        
    }
}


fn forward_reach_setup(
    start_joint: (Entity, &Joint, Option<&RotationConstraint>),
    joint_children_q: Query<&JointChildren>,
    joint_q: Query<(Entity, &Joint, Option<&RotationConstraint>)>,
    ee_joint_q: Query<&EndEffectorJoint>,
) -> (Vec<Entity>, Vec<(Entity, f32)>){
    let mut end_points = vec![start_joint.0];
    let mut start_length = joint_q.get(start_joint.0).unwrap().1.length;
    if start_joint.1.halfway {
        start_length *= 0.5;
    }
    let mut subchains = vec![(start_joint.0, start_length)];
    let mut active = vec![true];
    
    //this current count is the number of branches
    let mut current_count = 1;
    let mut i = 0;
    let mut current_exhausted = 0;

    let mut exhausted = false;
    //i can almost forsee this breaking.
    while !exhausted{
        while i < current_count {
            //check if the current joint has been exhausted or not
            if active[i] {
                //check for children of the current entity
                if let Ok(children) = joint_children_q.get(end_points[i]){
                    //if there is a children component, then check if the ammount is more than 1
                    if children.0.len() > 1 {
                        //if the ammount is more than 1, then first increase the count based on the extra amount.
                        current_count += children.0.len() - 1;
                        //temp was used previously to set the sub chain root as the parent, but now that isn't necessary.
                        //let temp = end_points[i];
                        //then replace the current entity with the first child
                        end_points[i] = children.0[0];

                        let new_length = joint_q.get(end_points[i]).unwrap().1.length;
                        subchains[i].1 += new_length;
                        //then loop through the rest and add them
                        //println!("multiple children detected!, adding these entities to the end_points: {:#?}", children.0);
                        
                        for c in 1..children.0.len(){
                            let new_length = joint_q.get(children.0[c]).unwrap().1.length;
                            //for each child, add a new subchain.
                            //should the subchain start at the parent or the child?
                            subchains.push((children.0[c], new_length));
                            end_points.push(children.0[c]);
                            active.push(true);
                        }
                    } else{
                        //if it's just 1 child, then replace the current and move on.
                        //println!("found a child, replacing entity number {} with {}", end_points[i], children.0[0]);
                        end_points[i] = children.0[0];
                        let new_length = joint_q.get(end_points[i]).unwrap().1.length;
                        subchains[i].1 += new_length;
                    }
                }else{
                    //if this joint didn't have any children, then it's been exhausted, and shouldn't be changed.
                    //and skip it in future runs\
                    //println!("found the end of branch {}, removing", i);
                    // if there isn't an end effector then remove it from the stack.
                    if ee_joint_q.get(end_points[i]).is_ok() {
                        active[i] = false; 
                        current_exhausted += 1;
                    } else {
                        active.remove(i);
                        subchains.remove(i);
                        end_points.remove(i);
                        current_count -= 1;
                    }
                    
                }
            }
            //advance the loop
            i += 1;
            
        }
        //if all branches have been exhaused, then exit the loop.
        if current_exhausted == current_count {
            //println!("exiting the loop");
            exhausted = true;
        }
        //if they haven't all been exhausted, then redo everything again. repeat the process until the loop exits.
        i = 0;
        
    }

    
    //println!("in the end, there was {} branch(es)", end_points.len());


    (end_points, subchains)
}


fn check_reachable(
    end_points: Vec<Entity>,
    sub_chains: Vec<(Entity, f32)>,
    ee_joint_q: Query<&EndEffectorJoint>,
    joint_transforms: Query<(Entity, &mut JointTransform, Option<&ChildOf>)>,
)-> Vec<(Entity, Entity, Entity)>{
    let mut ignore = vec![];
    for (i, entity) in end_points.iter().enumerate() {
        if let Ok(ee_joint) = ee_joint_q.get(entity){
            let ee_translation = joint_transforms.get(ee_joint.ee).unwrap().1.affine.translation;
            let sub_chain_translation = joint_transforms.get(sub_chains[i].0).unwrap().1.affine.translation;
            let distance = ee_translation.distance(sub_chain_translation);
            if distance > sub_chains[i].1 {
                ignore.push((end_points[i], sub_chains[i].0, ee_joint.ee))
            }   
                
        }
    }
    
    ignore
}

fn forward_reach(
    mut joint_transforms: Query<(Entity, &mut JointTransform, Option<&ChildOf>)>,
    joint_q: Query<(Entity, &Joint, Option<&RotationConstraint>)>,
    joint_children_q: Query<&JointChildren>,
    joint_parent_q: Query<&JointParent>,
    ee_joint_q: Query<&EndEffectorJoint>,
    end_points: Vec<Entity>,
    
){
    let mut current: Vec<Entity> = end_points.clone().iter().collect();
    //TRY ENTITYHASHSET APPARENTLY BETTER FOR THIS USE CASE WHICH IS REALLY FUNNY
    let mut visited = HashSet::new();
    let mut remove_all = false;
    

    while !current.is_empty() {
        
        let mut next = Vec::new();
        let mut remaining = Vec::new();

        for entity in current.drain(..){
            if visited.contains(&entity){
                
                continue;
            }

            let all_children_processed = joint_children_q.get(entity)
                .map(|children| children.0.iter().all(|c| visited.contains(&c)))
                .unwrap_or(true); // Default to true if no children

            if !all_children_processed {
                
                remaining.push(entity); // Defer processing until children are ready
                continue;
            }


            if let Ok(joint) = joint_q.get(entity){
                

                if let Ok(children) = joint_children_q.get(entity){
                    
                    let mut p_i1 = Vec3A::ZERO;
                    let mut d = 0.0;
                    let mut offset_avg = Vec3A::ZERO;
                    let parent_temp = joint_transforms.get(entity).unwrap().1.affine;
                    let rot = parent_temp.to_scale_rotation_translation().1;
                    let mut constraint_maybe: Option<&RotationConstraint> = None;

                    for c in 0..children.len() {
                        let child_affine = joint_transforms.get(children.0[c]).unwrap().1.affine;
                        let child_joint = joint_q.get(children.0[c]).unwrap();
                        if let Some(child_constraint) = child_joint.2{
                            //gets the first constrained child
                            if constraint_maybe.is_none() {
                                constraint_maybe = Some(child_constraint)
                            }
                        }
                        let offset = Vec3A::from(rot * child_joint.1.offset);
                        offset_avg += offset;
                        let bottom_point = if child_joint.1.halfway{
                            child_affine.translation - (child_affine.local_y() * child_joint.1.length * 0.5) + offset
                        } else {
                            child_affine.translation + offset
                        };
                        d += 1.0;
                        p_i1 += bottom_point;
                    }
                    p_i1 /= d;
                    offset_avg /= d;

                    let mut joint_t = joint_transforms.get_mut(entity).unwrap().1;

                    let mut p_i = if joint.1.halfway {
                        joint_t.affine.translation - (joint_t.affine.local_y() * joint.1.length * 0.5) - offset_avg
                    } else {
                        joint_t.affine.translation - offset_avg
                    };
                    
                    let mut dir = (p_i1 - p_i - offset_avg).normalize_or(joint_t.affine.local_y().as_vec3a());
                    if let Some(constraint) = constraint_maybe{
                        constrain_forward(&mut dir, &mut p_i, p_i1, constraint, joint_t.affine.to_scale_rotation_translation().1, joint.1);
                    }
                    
                    let r_i = (p_i1 - p_i - offset_avg).length();
                    
                    let lamda_i = joint.1.length / r_i;

                    p_i = (1.0 - lamda_i) * p_i1 + lamda_i * p_i;

                    joint_t.affine.translation = if joint.1.halfway {
                        p_i + (dir * joint.1.length * 0.5) 
                    } else {
                        p_i + offset_avg
                    };

                    let local_z = joint_t.affine.local_z();
                    joint_t.affine.align(Dir3::Y, Dir3A::new(dir).unwrap(), Dir3::Z, local_z);

                    // if let Some(rot_constraint) = joint.2 {
                    //     let dir2 = Dir3A::new_unchecked((parent_temp.translation - joint_t.affine.translation).normalize());
                    //     let affine = &joint_t.affine;
                    //     joint_t.affine = apply_constraint(dir2, *affine, *rot_constraint);
                    //     let local_up = joint_t.affine.local_y();
                    //     if joint.1.halfway {
                    //         joint_t.affine.translation = p_i1 - (local_up * joint.1.length * 0.5);
                    //     }
                    // }
                    

                } else if let Ok(ee_joint) = ee_joint_q.get(entity){
                    //the end effector variables are not supported for now...
                    let t_affine = joint_transforms.get(ee_joint.ee).unwrap().1.affine;
                    
                    let target_dir = t_affine.local_y();
                    let mut joint_t = joint_transforms.get_mut(entity).unwrap().1;
                    let mut p_i1 = t_affine.translation;
                    let mut p_i = if joint.1.halfway{
                        joint_t.affine.translation - (joint_t.affine.local_y() * joint.1.length * 0.5)  
                    } else {
                        joint_t.affine.translation
                    };
                    
                    if ee_joint.joint_center{
                        p_i1 += joint_t.affine.local_y() * joint.1.length * 0.5;
                    }
                    if ee_joint.joint_copy_rotation {
                        p_i = p_i1 - (target_dir * joint.1.length);
                    }
                    
                    let r_i = (p_i1 - p_i).length();
                    let dir = (p_i1 - p_i).normalize_or(joint_t.affine.local_y().as_vec3a());
                    let lambda = joint.1.length / r_i;
                        
                    p_i = (1.0 - lambda) * p_i1 + lambda * p_i;

                    joint_t.affine.translation = if joint.1.halfway {
                        p_i + (dir * joint.1.length * 0.5) 
                    } else {
                        p_i 
                    };

                    let local_z = joint_t.affine.local_z();
                    
                    joint_t.affine.align(Dir3::Y, Dir3A::new(dir).unwrap(), Dir3::Z, local_z);
                }
            }

            visited.insert(entity);

            if let Ok(joint_parent) = joint_parent_q.get(entity){
                let parent_entity = joint_parent.0;
                if !visited.contains(&parent_entity) {
                    next.push(parent_entity);
                } else {
                    remove_all = true; // Emergency exit
                    break;
                }
            }            
            
        }
        current = remaining;
        current.extend(next);
        if remove_all{
            current.clear();
        }
    }
    // println!("[FORWARD] COMPLETE");
 }

#[allow(clippy::too_many_arguments)]
fn backward_reach(
    mut joint_transforms: Query<(Entity, &mut JointTransform, Option<&ChildOf>)>,
    joint_q: Query<(Entity, &Joint, Option<&RotationConstraint>)>,
    start_entity: (Entity, &Joint, Option<&RotationConstraint>),
    joint_children_q: Query<&JointChildren>,
    joint_parent_q: Query<&JointParent>,
    ee_joint_q: Query<&EndEffectorJoint>,
    base_pos: Vec3A,
    base_rot: Quat,
) -> f32{
    let mut summed_dist = 0.0;

    let mut visited = HashSet::new();
    let mut queue = VecDeque::new();
    //include base rot
    let base_offset = Vec3A::from(base_rot * start_entity.1.offset);
    
    //set root to base
    let mut start_t = joint_transforms.get_mut(start_entity.0).unwrap().1;
    
    let mut top_point = if start_entity.1.halfway {
        start_t.affine.translation + (start_t.affine.local_y() * start_entity.1.length * 0.5)
    } else {
        start_t.affine.translation + (start_t.affine.local_y() * start_entity.1.length)
    };

    let r_i = (top_point - base_pos + base_offset).length();
    let dir = (top_point - base_pos + base_offset).normalize_or(start_t.affine.local_y().as_vec3a());
    let lamda_i = start_entity.1.length / r_i;

    top_point = (1.0 - lamda_i) * base_pos + lamda_i * top_point;

    start_t.affine.translation = if start_entity.1.halfway {
        top_point - (dir * start_entity.1.length * 0.5) + base_offset
    } else {
        top_point - (dir * start_entity.1.length) + base_offset
    };

    let local_z = start_t.affine.local_z();

    start_t.affine.align(Dir3::Y, Dir3A::new(dir).unwrap(), Dir3::Z, local_z);
    
    // Seed the queue with the start entity's children
    if let Ok(children) = joint_children_q.get(start_entity.0) {
        for child in &children.0 {
            queue.push_back(*child);
        }
    }

    while let Some(entity) = queue.pop_front(){
        if visited.contains(&entity){
            continue;
        }
        visited.insert(entity);
        let parent = joint_parent_q.get(entity).unwrap();
        let parent_affine = joint_transforms.get(parent.0).unwrap().1.affine;
        let parent_rot = parent_affine.to_scale_rotation_translation().1;
        
        let parent_joint = joint_q.get(parent.0).unwrap().1;
        let mut j_t = joint_transforms.get_mut(entity).unwrap().1;
        let joint = joint_q.get(entity).unwrap();
        let offset = Vec3A::from(parent_rot * joint.1.offset);
        let mut top_point = if joint.1.halfway {
            j_t.affine.translation + (j_t.affine.local_y() * joint.1.length * 0.5) + offset
        } else {
            j_t.affine.translation + (j_t.affine.local_y() * joint.1.length) + offset
        };
        let bottom_point = if parent_joint.halfway {
            parent_affine.translation + (parent_affine.local_y() * parent_joint.length * 0.5) + offset
        } else {
            parent_affine.translation + (parent_affine.local_y() * parent_joint.length) + offset
        };
        
        
        let mut dir = (top_point - bottom_point - offset).normalize_or(j_t.affine.local_y().as_vec3a());
        if let Some(constraint) = joint.2{
            constrain_backward(&mut dir, &mut top_point, bottom_point, constraint, parent_rot,j_t.affine.to_scale_rotation_translation().1, joint.1);
        }
        
        
        let r_i = (top_point - bottom_point - offset).length();
        
        let lamda_i = joint.1.length / r_i;

        top_point = (1.0 - lamda_i) * bottom_point + lamda_i * top_point;

        j_t.affine.translation = if joint.1.halfway {
            top_point - (dir * joint.1.length * 0.5) + offset
        } else {
            top_point - (dir * joint.1.length) + offset
        };

        let local_z = j_t.affine.local_z();

        j_t.affine.align(Dir3::Y, Dir3A::new(dir).unwrap(), Dir3::Z, local_z);
        // if let Some(rot_constraint) = joint.2 {
        //     let dir2 = Dir3A::new_unchecked((parent_affine.translation - j_t.affine.translation).normalize());
        //     let affine = &j_t.affine;
        //     j_t.affine = apply_constraint(dir2, *affine, *rot_constraint);
        //     let local_up = j_t.affine.local_y();
        //     if joint.1.halfway {
        //         j_t.affine.translation = bottom_point + (local_up * joint.1.length * 0.5);
        //     }
        // }
            

        if let Ok(children) = joint_children_q.get(entity) {
            for child in &children.0 {
                queue.push_back(*child);
            }
        } else {
            // Dead end: check if it's an end effector
            if let Ok(ee_joint) = ee_joint_q.get(entity) {
                let end_point = if joint.1.halfway{
                    j_t.affine.translation + (j_t.affine.local_y() * joint.1.length * 0.5)
                } else{
                    j_t.affine.translation + (j_t.affine.local_y() * joint.1.length)
                };
                let ee_t = joint_transforms.get(ee_joint.ee).unwrap().1;
                summed_dist += end_point.distance(ee_t.affine.translation);
            }
        }
    }
          
    summed_dist
}
