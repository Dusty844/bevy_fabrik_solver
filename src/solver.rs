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




#[allow(clippy::type_complexity)]
#[allow(clippy::too_many_arguments)]
pub fn solve(
    base_q: Query<(&GlobalTransform, &Base)>,
    joint_q: Query<(Entity, &Joint)>,
    joint_parent_q: Query<&JointParent>,
    joint_children_q: Query<&JointChildren>,
    ee_joint_q: Query<&EndEffectorJoint>,
    //mut sub_base_q: Query<&mut SubBase>,
    mut transforms: ParamSet<(
        Query<(&mut Transform, Option<&mut JointTransform>, &GlobalTransform)>,
        TransformHelper,
    )>,
    settings: Res<IkGlobalSettings>,
){
    for (t,mut joint_t, global_t) in transforms.p0().iter_mut(){
        if let Some(mut aff) = joint_t{
            aff.affine = global_t.affine();
        }
        
    } 


    
    for (base_gt, base) in base_q.iter(){
        if joint_parent_q.get(base.0).is_ok(){
            warn_once!("The Joint Entity {} has both a Base and a JointParent, and will be skipped in the solver, alleviate this by removing the BaseJoint component or the JointParent Component", base.0);
            continue;
        }
        if joint_q.get(base.0).is_err() {
            continue;
        }
        let start_entity = joint_q.get(base.0).unwrap();

        //the sub chain [i] contains the start of a sub chain where endpoints [i] is the endpoint of said
        //subchain, the first one always extends from the root to the first endpoint, and is the
        //main chain. everything else extends from that.
        // at the moment, the subchain is the parent entity, although it may be correct to make it the child instead.
        let (end_points, sub_chains) = forward_reach_setup(start_entity.0, joint_children_q, joint_q);

        let sub_chain_length = sub_chains.len();
        
        //first entity is the endpoint, second is the sub chain, third is the end effector
        let ignore_chains = check_reachable(end_points, sub_chains, ee_joint_q, transforms.p0().as_readonly());

        if ignore_chains.len() == sub_chain_length {
            //apply full optimization

        }

        for x in 0..settings.iterations {
            
        }
        
        
    }
}



// this is temp
fn forward_reach_setup(
    start_joint: Entity,
    joint_children_q: Query<&JointChildren>,
    joint_q: Query<(Entity, &Joint)>,
) -> (Vec<Entity>, Vec<(Entity, f32)>){
    let mut end_points = vec![start_joint];
    let start_length = joint_q.get(start_joint).unwrap().1.length;
    let mut subchains = vec![(start_joint, start_length)];
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
                        let temp = end_points[i];
                        //then replace the current entity with the first child
                        end_points[i] = children.0[0];
                        //then loop through the rest and add them
                        //println!("multiple children detected!, adding these entities to the end_points: {:#?}", children.0);
                        
                        for c in 1..children.0.len(){
                            let new_length = joint_q.get(temp).unwrap().1.length;
                            //for each child, add a new subchain.
                            //should the subchain start at the parent or the child?
                            subchains.push((temp, new_length));
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
                    active[i] = false; 
                    current_exhausted += 1;
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
    transforms: Query<(&Transform, Option<&JointTransform>, &GlobalTransform)>,
)-> Vec<(Entity, Entity, Entity)>{
    let mut ignore = vec![];
    for (i, entity) in end_points.iter().enumerate() {
        if let Ok(ee_joint) = ee_joint_q.get(*entity){
            let ee_translation = transforms.get(ee_joint.ee).unwrap().2.translation();
            let sub_chain_translation = transforms.get(sub_chains[i].0).unwrap().2.translation();
            let distance = ee_translation.distance(sub_chain_translation);
            if distance > sub_chains[i].1 {
                ignore.push((end_points[i], sub_chains[i].0, ee_joint.ee))
            }   
                
        }
    }
    
    ignore
}

fn reach_out_full(
    
){
    
}
