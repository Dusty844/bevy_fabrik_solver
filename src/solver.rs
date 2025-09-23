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



use rayon::prelude::*;
use std::sync::{Arc, Mutex, RwLock};
use bevy::{ecs::entity::EntityHashSet, platform::collections::HashMap, prelude::*, reflect::List, tasks::ParallelSlice};

pub fn solve(
    mut bk: ResMut<JointBookkeeping>,
    mut hierarchy_q: Query<(Entity, AnyOf<(&JointParent, &JointChildren)>)>,
    end_joints: Query<Entity, (With<EEJoint>, Without<JointChildren>)>,
    mut effector_joints: Query<(Entity, &EEJoint)>,
){

    let mut end_set = HashMap::new();
    for entity in end_joints.iter(){
        end_set.insert(entity, *bk.joints.lock().unwrap().get(&entity).unwrap());
    }
    
    
    
    //dont like having to have a query for the hierarchy components either but oh well there seems to be no good way to do this afaik
    forward_reach(&mut bk, end_set, hierarchy_q.reborrow(), effector_joints.reborrow());
        
}


fn forward_reach(
    bk: &mut JointBookkeeping,
    end_set: HashMap<Entity, (Joint, JointTransform)>,
    hierarchy_q: Query<(Entity, AnyOf<(&JointParent, &JointChildren)>)>,
    effector_joints: Query<(Entity, &EEJoint)>,
) {
    //is cloning really the best choice? ill have to benchmark at some point
    let mut current: Vec<Entity> = end_set.clone().into_iter().map(|(entity, (_, _))| entity).collect();
    let mut seen = Arc::new(RwLock::new(EntityHashSet::new()));

    loop{
        let next = Arc::new(Mutex::new(Vec::new()));
        
        current.par_iter().for_each(|main_entity|{
            seen.write().unwrap().insert(*main_entity);
            if let Ok((_, (parent_maybe, child_maybe))) = hierarchy_q.get(*main_entity){
                // if child(ren) is, then attatch to child(ren)
                if let Some(children) = child_maybe{
                    //should i par iter? why not?
                    let main_affine = bk.joints.lock().unwrap().get(main_entity).unwrap().1.affine;
                    children.0.par_iter().for_each(|child|{
                        let child_affine = bk.joints.lock().unwrap().get(child).unwrap().1.affine;
                        let child_joint = bk.joints.lock().unwrap().get(child).unwrap().0;

                        //offset needs to be based off the child, not the parent in this case
                        let offset = Vec3A::from(child_affine.to_scale_rotation_translation().1 * child_joint.offset);

                        let bottom_point = if child_joint.halfway{
                            //need to reimpl the utils functions
                            child_affine.translation + offset;
                        } else {
                            child_affine.translation + offset;
                        };
                        
                    });
                }
                if let Some(parent) = parent_maybe && !seen.read().unwrap().contains(&parent.0){
                    next.lock().unwrap().push(parent.0);
                }
            }

            if let Ok((_, ee_joint)) = effector_joints.get(*main_entity) {
                
                
            }
        });
             
    
        if current.is_empty(){
            break;
        }
        
    }
    
}
