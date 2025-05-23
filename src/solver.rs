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
    IkGlobalSettings
};





pub fn solve(
    base_q: Query<(&GlobalTransform, &Base)>,
    joint_q: Query<(Entity, &Joint)>,
    joint_parent_q: Query<&JointParent>,
    joint_children_q: Query<&JointChildren>,
    mut transforms: ParamSet<(
        Query<(&mut Transform, &mut JointTransform, &GlobalTransform)>,
        TransformHelper,
    )>,
    settings: Res<IkGlobalSettings>,
){
    for (t,mut joint_t, global_t) in transforms.p0().iter_mut(){
        joint_t.affine = global_t.affine();
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

        let end_points = forward_reach_setup(start_entity.0, joint_children_q);

        for x in 0..settings.iterations {
            
        }
        
        
    }
}



// this is temp
fn forward_reach_setup(
    start_joint: Entity,
    joint_children_q: Query<&JointChildren>,
) -> Vec<Entity>{
    let mut vec = vec![start_joint];
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
                if let Ok(children) = joint_children_q.get(vec[i]){
                    //if there is a children component, then check if the ammount is more than 1
                    if children.0.len() > 1 {
                        //if the ammount is more than 1, then first increase the count based on the extra amount.
                        current_count += children.0.len() - 1;
                        //then replace the current entity with the first child
                        vec[i] = children.0[0];
                        //then loop through the rest and add them
                        println!("multiple children detected!, adding these entities to the vec: {:#?}", children.0);
                        
                        for c in 1..children.0.len(){
                            
                            vec.push(children.0[c]);
                            active.push(true);
                        }
                    } else{
                        //if it's just 1 child, then replace the current and move on.
                        println!("found a child, replacing entity number {} with {}", vec[i], children.0[0]);
                        vec[i] = children.0[0];
                        
                    }
                }else{
                    //if this joint didn't have any children, then it's been exhausted, and shouldn't be changed.
                    //and skip it in future runs\
                    println!("found the end of branch {}, removing", i);
                    active[i] = false; 
                    current_exhausted += 1;
                }
            }
            //advance the loop
            i += 1;
            
        }
        //if all branches have been exhaused, then exit the loop.
        if current_exhausted == current_count {
            println!("exiting the loop");
            exhausted = true;
        }
        //if they haven't all been exhausted, then redo everything again. repeat the process until the loop exits.
        i = 0;
        
    }

    
    println!("in the end, there was {} branch(es)", vec.len());


    vec
}
