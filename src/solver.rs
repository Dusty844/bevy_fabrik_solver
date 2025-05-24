use bevy::ecs::relationship::RelationshipSourceCollection;
use bevy::prelude::*;

use std::sync::{Arc, Mutex};

use super::{
    Joint,
    JointParent,
    JointChildren,
    Base,
    BaseJoint,
    EndEffector,
    EndEffectorJoint,
    JointTransform,
    SubBase,
    IkGlobalSettings
};

use super::utils::*;




#[allow(clippy::type_complexity)]
#[allow(clippy::too_many_arguments)]
pub fn solve(
    base_q: Query<(Entity, &Base)>,
    joint_q: Query<(Entity, &Joint)>,
    joint_parent_q: Query<&JointParent>,
    joint_children_q: Query<&JointChildren>,
    ee_joint_q: Query<&EndEffectorJoint>,
    //mut sub_base_q: Query<&mut SubBase>,
    mut joint_transforms: Query<(Entity, &mut JointTransform, Option<&ChildOf>)>,
    mut transforms: ParamSet<(
        Query<(&mut Transform, &GlobalTransform)>,
        TransformHelper,
    )>,
    settings: Res<IkGlobalSettings>,
){
    for (entity, mut joint_t, _) in joint_transforms.iter_mut(){
        let gt = transforms.p1().compute_global_transform(entity).unwrap();
        joint_t.affine = gt.affine();
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

        let base_translation = transforms.p1().compute_global_transform(base_entity).unwrap().translation();

        

        //the sub chain [i] contains the start of a sub chain where endpoints [i] is the endpoint of said
        //subchain, the first one always extends from the root to the first endpoint, and is the
        //main chain. everything else extends from that.
        // at the moment, the subchain is the parent entity, although it may be correct to make it the child instead.
        let (end_points, sub_chains) = forward_reach_setup(start_entity, joint_children_q, joint_q, ee_joint_q);

        let sub_chain_length = sub_chains.len();
        
        //first entity is the endpoint, second is the sub chain, third is the end effector
        let ignore_chains = check_reachable(end_points, sub_chains, ee_joint_q, joint_transforms.reborrow());

        if ignore_chains.len() == sub_chain_length {
            //apply full optimization
            reach_out_full(joint_transforms.reborrow(), joint_q, start_entity, joint_children_q, base_translation, ignore_chains.clone());
            
        } else{
            //summed dist of the end joints to end effectors. if this value and the summed
            // dist returned by the backward reach at the end of the iteration
            // are the same, then break out of the loop. otherwise set the
            // summed dist to returned dist at the end.
            let mut summed_dist = 0.0;
            
            for x in 0..settings.iterations {
                forward_reach(joint_transforms.reborrow(), joint_q, start_entity, joint_children_q, joint_parent_q, ee_joint_q, ignore_chains.clone());
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
                    *transforms.p0().get_mut(current[i]).unwrap().0 = new_transform;
                    
                } else{

                    let srt = current_jt.affine.to_scale_rotation_translation();
                    let new_transform = Transform::from_scale(srt.0).with_rotation(srt.1).with_translation(srt.2);
                    *transforms.p0().get_mut(current[i]).unwrap().0 = new_transform;
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
    start_joint: (Entity, &Joint),
    joint_children_q: Query<&JointChildren>,
    joint_q: Query<(Entity, &Joint)>,
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

fn reach_out_full(
    mut joint_transforms: Query<(Entity, &mut JointTransform, Option<&ChildOf>)>,
    joint_q: Query<(Entity, &Joint)>,
    start_entity: (Entity, &Joint),
    joint_children_q: Query<&JointChildren>,
    base_translation: Vec3,
    ignore_chains: Vec<(Entity, Entity, Entity)>,
){
    //first, make the root joint be positioned to the base.
    let first_ee = ignore_chains[0].2;
    let first_ee_affine = joint_transforms.get(first_ee).unwrap().1.affine;
    let mut first_jt = joint_transforms.get_mut(start_entity.0).unwrap();
    first_jt.1.affine.translation = base_translation.into();
    //second, point the bone towards the end effector.
    let main_dir = (first_ee_affine.translation - first_jt.1.affine.translation).normalize();
    let second_dir = first_jt.1.affine.local_z();
    first_jt.1.affine.align(Dir3::Y, Vec3::from(main_dir), Dir3::Z, second_dir);
    let local_up = first_jt.1.affine.local_y();
    // possibly include joint offset?
    let first_length = if start_entity.1.halfway {
        first_jt.1.affine.translation += local_up * start_entity.1.length * 0.5;
        start_entity.1.length * 0.5
    }else{
        start_entity.1.length
    };
    
    let first_base = first_jt.1.affine.translation + (first_jt.1.affine.local_y() * first_length);

    
    //exit out early if root joint has no children for some reason, prevents panics from lack of children.
    if joint_children_q.get(first_jt.0).is_err(){
        return;
    }
    
    //set up the loop
    let mut current = joint_children_q.get(first_jt.0).unwrap().0.clone();
    let mut active = vec![true; current.len()];
     
    let mut bases = vec![first_base; current.len()];
    
    let mut current_count = current.len();
    let mut i = 0;
    let mut current_exhausted = 0;

    let mut exhausted = false;

    while !exhausted {
        while i < current_count{
            if active[i] {
                let ee_entity = ignore_chains[i].2;
                let ee_affine = joint_transforms.get(ee_entity).unwrap().1.affine;

            
                let (je, mut jt, _) = joint_transforms.get_mut(current[i]).unwrap();
            
                jt.affine.translation = bases[i];

                let main_dir = (ee_affine.translation - jt.affine.translation).normalize();            
                let second_dir = jt.affine.local_z();
                jt.affine.align(Dir3::Y, Vec3::from(main_dir), Dir3::Z, second_dir);
                let current_j = joint_q.get(current[i]).unwrap().1;
                let local_up = jt.affine.local_y();
                let new_base = if current_j.halfway {
                    jt.affine.translation += local_up * current_j.length * 0.5;
                    
                    jt.affine.translation + (jt.affine.local_y() * current_j.length * 0.5)
                } else {
                    jt.affine.translation + (jt.affine.local_y() * current_j.length)
                };
                

                
            
                if let Ok(children) = joint_children_q.get(current[i]){
                    current[i] = children.0[0];
                    bases[i] = new_base;
                    if children.0.len() > 1 {
                        for c in 1..children.0.len() {
                            if ignore_chains[i + 1].1 == children.0[c] {
                                current.push(children.0[c]);
                                bases.push(new_base);
                                active.push(true);
                                current_count += 1;
                            }
                        } 
                    }
                    
                }else{
                    current_exhausted += 1;
                    active[i] = false;
                }

            }
            i += 1;
        }
        if current_exhausted == current_count {
            //println!("exiting the loop");
            exhausted = true;
        }
        i = 0;
    }
}


fn forward_reach(
    mut joint_transforms: Query<(Entity, &mut JointTransform, Option<&ChildOf>)>,
    joint_q: Query<(Entity, &Joint)>,
    start_entity: (Entity, &Joint),
    joint_children_q: Query<&JointChildren>,
    joint_parent_q: Query<&JointParent>,
    ee_joint_q: Query<&EndEffectorJoint>,
    ignore_chains: Vec<(Entity, Entity, Entity)>,
    
){
    let mut current: Vec<Entity> = ignore_chains.clone().iter().map(|t| t.0).collect();

    let mut current_count = current.len();
    let mut i = 0;
    let mut exhausted = false;

    
    
    while !exhausted{
        while i < current_count {
            if let Ok(joint) = joint_q.get(current[i]) {

                

                if let Ok(ee_joint) = ee_joint_q.get(current[i]){
                    let ee_affine = joint_transforms.get(ee_joint.ee).unwrap().1.affine;
                    //this has to be here for borrow checks.
                    let (_, mut j_transform, _) = joint_transforms.get_mut(current[i]).unwrap();
                    let j_main_dir = Dir3A::new_unchecked((ee_affine.translation - j_transform.affine.translation).normalize());
                    let ee_main_dir = ee_affine.local_y();
                    let second_dir = j_transform.affine.local_z();

                    if ee_joint.joint_copy_rotation {
                        j_transform.affine.align(Dir3A::Y, ee_main_dir, Dir3A::Z, second_dir);
                    } else{
                        j_transform.affine.align(Dir3A::Y, j_main_dir, Dir3A::Z, second_dir);
                    }

                    let new_up = j_transform.affine.local_y();
                    
                    //if there is an end effector to follow
                    match (ee_joint.joint_center, joint.1.halfway) {
                        (true, true) => {
                            //is center, is half.
                            j_transform.affine.translation = ee_affine.translation;
                        }
                        (false, false) => {
                            //isn't center, isnt half
                            j_transform.affine.translation = ee_affine.translation - (new_up * joint.1.length);
                        }
                        (true, false) => {
                            //is center, isnt half
                            j_transform.affine.translation = ee_affine.translation - (new_up * joint.1.length * 0.5);
                        }
                        (false, true) => {
                            //isnt center is half
                            j_transform.affine.translation = ee_affine.translation - (new_up * joint.1.length * 0.5);
                        }
                    }
                
                }
                if let Ok(children) = joint_children_q.get(current[i]){
                    if children.len() <= 1 {
                        let mut child_affine = joint_transforms.get(children.0[0]).unwrap().1.affine;
                        let child_joint = joint_q.get(children.0[0]).unwrap().1;
                        if child_joint.halfway {
                           child_affine.translation -= child_affine.local_y() * child_joint.length * 0.5; 
                        }
                        let (_, mut j_transform, _) = joint_transforms.get_mut(current[i]).unwrap();
                        let main_dir = Dir3A::new_unchecked((child_affine.translation - j_transform.affine.translation).normalize());
                        let second_dir = j_transform.affine.local_z();

                        j_transform.affine.align(Dir3A::Y, main_dir, Dir3A::Z, second_dir);
                        
                        j_transform.affine.translation = child_affine.translation;
                        
                    }else{
                        let mut average = Vec3A::ZERO;
                        let mut d = 0.0;
                        for c in 0..children.len() {
                            let child_affine = joint_transforms.get(children.0[c]).unwrap().1.affine;
                            let child_joint = joint_q.get(children.0[c]).unwrap().1;
                            let next = if child_joint.halfway{
                                child_affine.translation - (child_affine.local_y() * child_joint.length * 0.5)
                            }else{
                                child_affine.translation - (child_affine.local_y() * child_joint.length)
                            };
                            average += next;
                            d += 1.0;
                        }
                        average /= d;
                        

                        let (_, mut j_transform, _) = joint_transforms.get_mut(current[i]).unwrap();
                        let main_dir = Dir3A::new_unchecked((average - j_transform.affine.translation).normalize());
                        let second_dir = j_transform.affine.local_z();

                        j_transform.affine.align(Dir3A::Y, main_dir, Dir3A::Z, second_dir);
                        
                        j_transform.affine.translation = average;
                    }
                }
                if let Ok(joint_parent) = joint_parent_q.get(current[i]) {
                    if let Ok(joint_children) = joint_children_q.get(joint_parent.0){
                        if current[i] == joint_children.0[0]{
                            current[i] = joint_parent.0;
                        }else{
                            current.remove(i);
                            i -= 1;
                            current_count -= 1;
                        }
                    }
                }

                if current[i] == start_entity.0 {
                    exhausted = true;
                }
            }
            
            i += 1;
        }
        i = 0;
    }
}

