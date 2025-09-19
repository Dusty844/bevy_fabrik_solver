use bevy::{
    prelude::*,
    math::Affine3A,
    platform::collections::HashMap,
};
use std::sync::{Arc, Mutex};

mod solver;

mod bookkeeper;

pub struct IkSolverPlugin;

impl Plugin for IkSolverPlugin{
    fn build(&self, app: &mut App) {
        #[cfg(feature = "bevy_reflect")]
        app.register_type::<(Joint, JointParent, JointChildren, JointTransform, Base, BaseJoint, EndEffector, EEJoint, IkGlobalSettings)>();
        app.add_systems(PreStartup, bookkeeper::joint_hooks);
        app.add_systems(PostUpdate, (bookkeeper::collect_joint_transforms, bookkeeper::bookkeep_joints_start).chain().before(TransformSystem::TransformPropagate));
        app.insert_resource(IkGlobalSettings::default());
        app.insert_resource(JointBookkeeping::default());
    }
}

#[derive(Resource, Clone, Copy, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
pub struct IkGlobalSettings{
    pub iterations: usize,
    pub minimum_tolerance: f32,
    pub solver_mode: GlobalRotationMode
}

#[derive(Default, Debug, Clone, Copy, Reflect)]
pub enum GlobalRotationMode{
    #[default]
    DuringSolver,
    AfterSolver,
    Both,
}

impl Default for IkGlobalSettings{
    fn default() -> Self {
        Self{
            iterations: 10,
            minimum_tolerance: 0.002,
            solver_mode: GlobalRotationMode::DuringSolver,
        }
    }
}


#[derive(Component, Clone, Copy, Default, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
#[require(Transform, JointTransform)]
pub struct Joint{
    pub length: f32,
    pub halfway: bool,
}

#[derive(Component, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
#[relationship(relationship_target = JointChildren)]
pub struct JointParent(Entity);


#[derive(Component, Debug, Default)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
#[relationship_target(relationship = JointParent)]
pub struct JointChildren(Vec<Entity>);



#[derive(Component, Clone, Copy, Default, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
#[require(Transform)]
pub struct JointTransform{
    affine: Affine3A,        
}

#[derive(Component, Clone, Copy, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
#[require(Transform, JointTransform)]
pub struct EndEffector{
    pub joint: Entity,
    pub joint_center: bool,
    pub joint_copy_rotation: bool,
}

#[derive(Component, Clone, Copy, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
pub struct EEJoint{
    pub ee: Entity,
}

#[derive(Component, Clone, Copy, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
#[require(JointTransform)]
pub struct Base(pub Entity);

#[derive(Component, Clone, Copy, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
#[require(JointTransform)]
pub struct BaseJoint(pub Entity);


#[derive(Resource, Clone, Debug)]
pub struct JointBookkeeping{
    pub joints: Arc<Mutex<HashMap<Entity, (Joint, JointTransform)>>>,
    pub ends: HashMap<Entity, (EndEffector, JointTransform)>,
    pub bases: HashMap<Entity, (Base, JointTransform)>,
}

impl Default for JointBookkeeping{
    fn default() -> Self {
        Self{
            joints: Arc::new(Mutex::new(HashMap::new())),
            ends: HashMap::new(),
            bases: HashMap::new(),
        }
    }
}
