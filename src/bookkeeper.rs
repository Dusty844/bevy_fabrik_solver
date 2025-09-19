use super::{
    Joint,
    JointBookkeeping,
    JointTransform,
    EndEffector,
    EEJoint,
    Base,
    BaseJoint,
};

use bevy::{ecs::component::HookContext, prelude::*};



pub fn joint_hooks(
    world: &mut World,
){

    //remove the joint from the book, what to do about parents and children?, no idea...
    world.register_component_hooks::<Joint>()
    .on_remove(
        |mut world, context|{
            world.resource_mut::<JointBookkeeping>().joints.lock().unwrap().remove(&context.entity);
        }
    );

    //removes end effector from book, and removes the ee joint on the other end
    world.register_component_hooks::<EndEffector>()
    .on_remove(
        |mut world, context|{
            let (ee, _) = world.resource_mut::<JointBookkeeping>().ends.remove(&context.entity).unwrap();
            world.commands().entity(ee.joint).try_remove::<EEJoint>();
        }
    );


    //same as above
    world.register_component_hooks::<Base>()
    .on_remove(
        |mut world, context|{
            let (base, _) = world.resource_mut::<JointBookkeeping>().bases.remove(&context.entity).unwrap();
            world.commands().entity(base.0).try_remove::<BaseJoint>();
        }
    );
    
}


pub fn collect_joint_transforms(
        mut transforms_q: Query<(&mut JointTransform, Entity), Changed<Transform>>,
        helper: TransformHelper,
){
    for (mut jt, entity) in transforms_q.iter_mut(){
        jt.affine = helper.compute_global_transform(entity).unwrap().affine();
    }
}



pub fn bookkeep_joints_start(
    mut joint_bookkeeper: ResMut<JointBookkeeping>,
    joints_q: Query<(&Joint, &JointTransform, Entity), Or<(Added<Joint>, Changed<Joint>, Changed<JointTransform>)>>,
    end_effectors_q: Query<(&EndEffector, &JointTransform, Entity), Or<(Added<EndEffector>, Changed<EndEffector>, Changed<JointTransform>)>>,
    bases_q: Query<(&Base, &JointTransform, Entity), Or<(Added<Base>, Changed<Base>, Changed<JointTransform>)>>,
){
    //Joints first, could make par iter
    for (joint, jt, entity) in joints_q.iter(){
        joint_bookkeeper.joints.lock().unwrap().insert(entity, (*joint, *jt));
    }

    for (end_effector, jt, entity) in end_effectors_q.iter(){
        joint_bookkeeper.ends.insert(entity, (*end_effector, *jt));
    }

    for (base, jt, entity) in bases_q.iter() {
        joint_bookkeeper.bases.insert(entity, (*base, *jt));
    }
    
    
    
}
