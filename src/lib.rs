use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
    math::Affine3A,
};
mod solver;

mod utils;

mod constraint;

pub struct IkSolverPlugin{
    schedule: Interned<dyn ScheduleLabel>,
}

impl Plugin for IkSolverPlugin{
    fn build(&self, app: &mut App) {
        app.add_systems(self.schedule, (prepare_joints, solver::solve).chain().before(TransformSystem::TransformPropagate))
        .register_type::<(Joint, JointParent, JointChildren, Base, BaseJoint, EndEffector, EndEffectorJoint, JointTransform, IkGlobalSettings, RotationConstraint)>()
        .insert_resource(IkGlobalSettings::default())

;
    }
}

impl Default for IkSolverPlugin{
    fn default() -> Self {
        Self{
            schedule: PostUpdate.intern(),
        }
    }
}

impl IkSolverPlugin {
    /// Creates a [`IkSolverPlugin`] with the schedule that is used for running the `solve` function.
    ///
    /// The default schedule is `Last`. You may want to set this to be somewhere in `FixedUpdate` and use transform interpolation.
    pub fn new(schedule: impl ScheduleLabel) -> Self {
        Self {
            schedule: schedule.intern(),
        }
    }

}


#[derive(Resource, Copy, Clone, Reflect, Debug)]
#[reflect(Resource, Debug)]
pub struct IkGlobalSettings{
    pub iterations: usize,
    pub minimum_tolerance: f32,
}

impl Default for IkGlobalSettings{
    fn default() -> Self {
        Self{
            iterations: 10,
            minimum_tolerance: 0.002,
        }
    }
}


///A `Joint` or bone in an IK chain.
#[derive(Component, Clone, Copy, Default, Reflect, Debug)]
#[reflect(Component, Debug)]
#[require(Transform, JointTransform)]
pub struct Joint{
    /// The Length of the `Joint`. Very important to the solver if you want a correct output.
    pub length: f32,

    /// The `Joint` Offset. This Field can stay at zero if your bone is connected to the parent.
    pub offset: Vec3,

    /// The halfway bool denotes whether a Zero local position is in the middle, or
    /// half way up. If true, then the joint position shows the middle of the bone, if
    /// false, then it shows the start of the bone.
    pub halfway: bool,
}

/// Internal `Transform` specifically for [`Joint`]s, Contains an `Affine3A`
/// which is used for:
///
/// - All the IK Math,
/// - Keeping Track of Global [`Joint`] `Transform` during the solve system.
/// - Easy Sync Back to Real `Transform`.
#[derive(Component, Clone, Copy, Default, Reflect, Debug)]
#[reflect(Component, Debug)]
#[require(Transform)]
pub struct JointTransform{
    affine: Affine3A,        
}

/// The Rotation Constraint of a [`Joint`]
#[derive(Component, Clone, Copy, Default, Reflect, Debug)]
#[reflect(Component, Debug)]
#[require(Transform, JointTransform)]
pub struct RotationConstraint{
    
    pub swing_min: Vec3,

    pub swing_max: Vec3,
    
    pub twist_min: Vec3,
    pub twist_max: Vec3,

    pub dir: Vec3,

    pub relative_rotation: Quat,
}




#[derive(Component, Debug, PartialEq, Eq, Reflect)]
#[relationship(relationship_target = JointChildren)]
pub struct JointParent(Entity);




#[derive(Component, Debug, Reflect, Default)]
#[relationship_target(relationship = JointParent)]
pub struct JointChildren(Vec<Entity>);




/// The End Effector is placed on the target that a chain of [`Joint`]s moves
/// towards.
///
/// In The Future, the `EndEffector` and [`EndEffectorJoint`] components will
/// be connected with a One-To-One RelationshipTarget.
#[derive(Component, Clone, Copy, Reflect, Debug)]
#[reflect(Component, Debug)]
#[require(Transform, JointTransform)]
pub struct EndEffector{
    /// The [`Joint`] Entity of a [`Joint`] in a chain whose position will be set
    /// to, or as close to the end effector.
    ///
    /// (Maybe?) The `EndEffector` can target a [`Joint`] in the middle of a
    /// chain, the [`Joint`]s above in the hierarchy will be moved towards the
    /// `EndEffector`. Meanwhile, [`Joint`]s lower in the hierarchy will be
    /// dragged backwards towards the `EndEffector`, pointing towards their
    /// previous directions.
    /// 
    /// This is unless there is another `EndEffector` on the final [`Joint`] at
    /// the bottom of the hierarchy, in which case the `EndEffector` will act
    /// as both an `EndEffector`, but also as a [`Base`].
    ///
    /// This specific behaviour does not apply to a [`Base`] in the same situation.
    pub joint: Entity,
    /// If set to true, then the target [`Joint`] will be positioned in the center of
    /// the `EndEffector`, relative to the length of the [`Joint`].
    pub joint_center: bool,
    /// If set to true, then the target [`Joint`] will try to copy the rotation of the
    /// `EndEffector`
    pub joint_copy_rotation: bool,
}






/// The `EndEffectorJoint` component is automatically placed on the target
/// [`Joint`] of an [`EndEffector`], this is for the algorithm only, and
/// shouldn't matter to the End User.
#[derive(Component, Clone, Copy, Reflect, Debug)]
#[reflect(Component, Debug)]
#[require(Transform)]
pub struct EndEffectorJoint{
    /// The targeted [`EndEffector`]
    pub ee: Entity,
    /// If set to true, then the target [`Joint`] will be positioned in the center of
    /// the [`EndEffector`], relative to the length of the [`Joint`].
    pub joint_center: bool,
    /// If set to true, then the target [`Joint`] will try to copy the rotation of the
    /// [`EndEffector`]
    pub joint_copy_rotation: bool,
    
    
}








/// The `Base` is the target that a root [`Joint`] (the [`Joint`] at the very
/// top of the hierarchy) is algorithmically constrained to.
///
/// The Entity field in the [`Base`] Component points to the root joint.
///
/// Depending on how you choose to spawn entities, you can place this component
/// on the root [`Joint`] and set the [`Base`].
/// 
/// Alternatively you can do the opposite and set the root [`Joint`] from the
/// [`Base`] component.
///
/// Either way, both sides should sync together prior to the solver.
/// 
/// A `Base` Component should not point to any [`Joint`] that has a
/// [`JointParent`]. or else, the entity field in the `Base` component will be
/// reset and the [`BaseJoint`] Component on said [`Joint`] will be removed.
///
/// In The Future, the `Base` and [`BaseJoint`] components will
/// be connected with a One-To-One RelationshipTarget.
#[derive(Component, Clone, Copy, Reflect, Debug)]
#[reflect(Component, Debug)]
#[require(Transform)]
pub struct Base(pub Entity);






/// The `BaseJoint` component is placed on the target [`Joint`] of a [`Base`].
/// 
/// Depending on how you choose to spawn entities, you can place this component
/// on the root [`Joint`] and set the [`Base`].
/// 
/// Alternatively you can do the opposite and set the root [`Joint`] from the
/// [`Base`] component.
///
/// Either way, both sides should sync together prior to the solver.
///
/// In The Future, the `Base` and [`BaseJoint`] components will
/// be connected with a One-To-One RelationshipTarget.
#[derive(Component, Clone, Copy, Reflect, Debug)]
#[reflect(Component, Debug)]
#[require(Transform)]
pub struct BaseJoint(pub Entity);


/// Puts JointParent on joints that have parent joint, maybe do this 
/// with component hooks instead? i'm not quite sure at the moment.
#[allow(clippy::type_complexity)]
fn prepare_joints(
    new_joints: Query<(Entity, &ChildOf), Added<Joint>>,
    joint_q: Query<&Joint>,
    base_j_q: Query<(Entity, &BaseJoint), Or<(Added<BaseJoint>, Changed<BaseJoint>)>>,
    base_q: Query<(Entity, &Base), Or<(Added<Base>, Changed<Base>)>>,
    ee_j_q: Query<(Entity, &EndEffectorJoint), Or<(Added<EndEffectorJoint>, Changed<EndEffectorJoint>)>>,
    ee_q: Query<(Entity, &EndEffector), Or<(Added<EndEffector>, Changed<EndEffector>)>>,
    mut commands: Commands,
){
    for (entity, parent) in new_joints.iter(){
        if joint_q.get(parent.0).is_ok(){
            commands.entity(entity).try_insert_if_new(JointParent(parent.0));
        }
    }

    //sync base from base joint end
    for (entity, base_j) in base_j_q.iter(){
        commands.entity(base_j.0).try_insert_if_new(Base(entity));
    }

    
    //sync base joint from base end
    for (entity, base) in base_q.iter(){
        commands.entity(base.0).try_insert_if_new(BaseJoint(entity));
    }

    

    
    //sync ee joint from base end
    for (entity, ee) in ee_q.iter(){
        commands.entity(ee.joint).try_insert(EndEffectorJoint{
            ee: entity,
            joint_center: ee.joint_center,
            joint_copy_rotation: ee.joint_copy_rotation
        });
    }
    
    //sync ee from ee joint end
    for (entity, ee_j) in ee_j_q.iter(){
        if let Ok(ee) = ee_q.get(ee_j.ee){
            if ee.1.joint != entity {
                //this is maybe not a good idea.

                warn!("tried setting more than one end effector joint to just one End Effector, creating a new End Effector.");
                commands.spawn((
                    EndEffector{
                        joint: entity,
                        joint_center: ee_j.joint_center,
                        joint_copy_rotation: ee_j.joint_copy_rotation
                    },
                    
                    //ChildOf(entity),
            ));
            }
        }else{
            commands.entity(ee_j.ee).try_insert_if_new(EndEffector{
                joint: entity,
                joint_center: ee_j.joint_center,
                joint_copy_rotation: ee_j.joint_copy_rotation
            });
        }
        
    }
    
}


