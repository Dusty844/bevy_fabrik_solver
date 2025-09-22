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
    POOL,
};


use std::sync::{Arc, Mutex};
use bevy::{ecs::entity::EntityHashSet, platform::collections::HashMap, prelude::*, reflect::List};

pub fn solve(
    mut bk: ResMut<JointBookkeeping>,
    mut hierarchy_q: Query<(Entity, AnyOf<(&JointParent, &JointChildren)>)>,
    end_joints: Query<Entity, (With<EEJoint>, Without<JointChildren>)>,
){

    let mut end_set = HashMap::new();
    for entity in end_joints.iter(){
        end_set.insert(entity, *bk.joints.lock().unwrap().get(&entity).unwrap());
    }
    
    
    
    //dont like having to have a query for the hierarchy components either but oh well there seems to be no good way to do this afaik
    forward_reach(&mut bk, end_set, hierarchy_q.reborrow());
        
}


fn forward_reach(
    mut bk: &mut JointBookkeeping,
    end_set: HashMap<Entity, (Joint, JointTransform)>,
    hierarchy_q: Query<(Entity, AnyOf<(&JointParent, &JointChildren)>)>,
) {
    //is cloning really the best choice? ill have to benchmark at some point
    let mut current: Vec<(Entity, Joint, JointTransform)> = end_set.clone().into_iter().map(|(entity, (joint, jt))| (entity, joint, jt)).collect();
    let mut seen = EntityHashSet::new();

    loop{
        
        
             
    
        if current.is_empty(){
            break;
        }
        
    }
    
}
