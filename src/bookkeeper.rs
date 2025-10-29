use std::sync::Arc;

use crate::{IkGlobalSettings, JointChildren, JointParent};

use super::{
    Joint,
    JointBookkeeping,
    JointTransform,
    EndEffector,
    EEJoint,
    Base,
    BaseJoint,
};

use bevy::{math::Affine3A, prelude::*};



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
        .on_add(|mut world, context|{
            let joint = world.get::<EndEffector>(context.entity).unwrap().joint;
            world.commands().entity(joint).insert(EEJoint(context.entity));
            //handles only insertion of basejoint
        
        })
        .on_remove(
            |mut world, context|{
                let (ee, _) = world.resource_mut::<JointBookkeeping>().ends.write().unwrap().remove(&context.entity).unwrap();
                world.commands().entity(ee.joint).try_remove::<EEJoint>();
            }
        );

    world.register_component_hooks::<EEJoint>()
        .on_add(|mut world, context|{
            let joint = world.get::<EEJoint>(context.entity).unwrap().0;
            world.commands().entity(joint).insert(
                EndEffector{
                    joint: context.entity,
                    joint_center: false,
                    joint_copy_rotation: false,
                });
            //handles only insertion of basejoint
        
        })
        .on_remove(
            |mut world, context|{
                
                let end = world.get::<EEJoint>(context.entity).unwrap().0;
                
                
                world.commands().entity(end).try_remove::<EndEffector>();
            }
        );


    //same as above
    world.register_component_hooks::<Base>()
    .on_add(|mut world, context|{
        let joint = world.get::<Base>(context.entity).unwrap().0;
        world.commands().entity(joint).insert(BaseJoint(context.entity));
        //handles only insertion of basejoint
        
    })
    .on_remove(
        |mut world, context|{
            let (base, _) = world.resource_mut::<JointBookkeeping>().bases.write().unwrap().remove(&context.entity).unwrap();
            world.commands().entity(base.0).try_remove::<BaseJoint>();
        }
    );

    world.register_component_hooks::<BaseJoint>()
        .on_add(|mut world, context|{
            let joint = world.get::<BaseJoint>(context.entity).unwrap().0;
            world.commands().entity(joint).insert(
                Base(context.entity));
            //handles only insertion of basejoint
        
        })
        .on_remove(
            |mut world, context|{
                
                let base = world.get::<BaseJoint>(context.entity).unwrap().0;
                
                
                world.commands().entity(base).try_remove::<Base>();
            }
        );

    
}


pub fn collect_joint_transforms(
        mut transforms_q: Query<(&mut JointTransform, Entity), Changed<Transform>>,
        helper: TransformHelper,
){
    for (mut jt, entity) in transforms_q.iter_mut(){
        let srt = helper.compute_global_transform(entity).unwrap().to_scale_rotation_translation();
        jt.scale = srt.0;
        jt.rotation = srt.1;
        jt.translation = srt.2;
    }
}


#[allow(clippy::type_complexity)]
pub fn bookkeep_joints_start(
    joint_bookkeeper: Res<JointBookkeeping>,
    joints_q: Query<(&Joint, &JointTransform, Entity)>,
    end_effectors_q: Query<(&EndEffector, &JointTransform, Entity)>,
    bases_q: Query<(&Base, &JointTransform, Entity)>,
    parent_setup: Query<(Entity, &ChildOf), Added<Joint>>,
    mut commands: Commands,
){
    //Joints first, could make par iter
    for (joint, jt, entity) in joints_q.iter(){
        joint_bookkeeper.joints.lock().unwrap().insert(entity, (*joint, *jt));
    }

    for (end_effector, jt, entity) in end_effectors_q.iter(){
        joint_bookkeeper.ends.write().unwrap().insert(entity, (*end_effector, *jt));
    }

    for (base, jt, entity) in bases_q.iter() {
        joint_bookkeeper.bases.write().unwrap().insert(entity, (*base, *jt));
    }
    for (entity, parent) in parent_setup.iter(){
        commands.entity(entity).try_insert_if_new(JointParent(parent.0));
    }

}

#[allow(clippy::type_complexity)]
pub fn sync_transforms(
    joint_bookkeeper: Res<JointBookkeeping>, 
    mut transforms_param_set: ParamSet<(
        Query<(&mut Transform, Option<&ChildOf>), With<JointTransform>>,
        TransformHelper,
    )>,
    mut joints_q: Query<&mut JointTransform>,
    parents_q: Query<&ChildOf, With<JointTransform>>,
) {

    


    //joints, dont really like having to lock the whole thing, but it doesnt seem possible to do something like [i], maybe it is ill have to find out
    
    for (entity, (_, jt)) in joint_bookkeeper.joints.lock().unwrap().iter() {
        let maybe_parent = parents_q.get(*entity);
        *joints_q.get_mut(*entity).unwrap() = *jt;
        
        if let Ok(parent) = maybe_parent{
            
            let new_gt = transforms_param_set.p1().compute_global_transform(parent.0).unwrap();
            let new_affine = new_gt.affine().inverse();
            let final_affine = new_affine * Affine3A::from_scale_rotation_translation(jt.scale, jt.rotation, jt.translation);
            let srt = final_affine.to_scale_rotation_translation();

            let new_transform = Transform::from_scale(srt.0).with_rotation(srt.1).with_translation(srt.2);

            if new_transform.is_finite(){
                *transforms_param_set.p0().get_mut(*entity).unwrap().0.bypass_change_detection() = new_transform;
            }
        }else{
            
            let new_transform = Transform::from_scale(jt.scale).with_rotation(jt.rotation).with_translation(jt.translation);
            
            if new_transform.is_finite(){
                *transforms_param_set.p0().get_mut(*entity).unwrap().0 = new_transform;
            }
        }
    }

    
    
        
    
    
}

pub fn force_gt(
    mut gt_joint_q: Query<(&mut GlobalTransform, &JointTransform)>,
    global_settings: Res<IkGlobalSettings>,
){
    if global_settings.force_global_transform {
        for (mut gt, joint) in gt_joint_q.iter_mut(){
            let transform = Transform::from_scale(joint.scale).with_rotation(joint.rotation).with_translation(joint.translation);
            *gt = GlobalTransform::from(transform);
        }
    }
    
}
