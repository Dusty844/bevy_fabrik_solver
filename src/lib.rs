use bevy::{
    prelude::*,
    platform::collections::HashMap,
};
use std::sync::{Arc, Mutex, RwLock};


mod solver;

mod constraint;

mod bookkeeper;

mod utils;


pub struct IkSolverPlugin;

impl Plugin for IkSolverPlugin{
    fn build(&self, app: &mut App) {
       
        app.add_systems(PreStartup, bookkeeper::joint_hooks);
        
        app.add_systems(PostUpdate, (
            bookkeeper::collect_joint_transforms,
            bookkeeper::bookkeep_joints_start,
            solver::solve,
            bookkeeper::sync_transforms,
        ).chain().before(TransformSystems::Propagate));
        app.add_systems(PostUpdate, bookkeeper::force_gt.after(TransformSystems::Propagate));
        
        app.insert_resource(IkGlobalSettings::default());
        app.insert_resource(JointBookkeeping::default());

        
    }
}

#[derive(Resource, Clone, Copy, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
#[cfg_attr(feature = "bevy_reflect", reflect(Resource))]
pub struct IkGlobalSettings{
    pub iterations: usize,
    pub minimum_tolerance: f32,
    pub force_global_transform: bool,
}


impl Default for IkGlobalSettings{
    fn default() -> Self {
        Self{
            iterations: 10,
            minimum_tolerance: 0.00001,
            force_global_transform: false,
        }
    }
}


#[derive(Component, Clone, Copy, Default, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
#[require(Transform, JointTransform)]
pub struct Joint{
    pub length: f32,
    pub visual_offset: Vec3,
    pub anchor_offset: Vec3,
}

#[derive(Component, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
#[relationship(relationship_target = JointChildren)]
#[require(Joint)]
pub struct JointParent(Entity);


#[derive(Component, Debug, Default)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
#[relationship_target(relationship = JointParent)]
#[require(Joint)]
pub struct JointChildren(Vec<Entity>);



#[derive(Component, Clone, Copy, Default, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
#[require(Transform)]
pub struct JointTransform{
    scale: Vec3,
    rotation: Quat,
    translation: Vec3,        
}

impl JointTransform {
    pub const IDENTITY: JointTransform = JointTransform{
        scale: Vec3::ONE,
        rotation: Quat::IDENTITY,
        translation: Vec3::ZERO,
    };
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
#[require(Joint)]
pub struct EEJoint(pub Entity);

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
    pub parents: Arc<RwLock<HashMap<Entity, JointParent>>>,
    pub children: Arc<RwLock<HashMap<Entity, JointChildren>>>,
    pub ends: Arc<RwLock<HashMap<Entity, (EndEffector, JointTransform)>>>,
    pub bases: Arc<RwLock<HashMap<Entity, (Base, JointTransform)>>>,
    pub last_diff: Vec3,
}

impl Default for JointBookkeeping{
    fn default() -> Self {
        Self{
            joints: Arc::new(Mutex::new(HashMap::new())),
            parents: Arc::new(RwLock::new(HashMap::new())),
            children: Arc::new(RwLock::new(HashMap::new())),
            ends: Arc::new(RwLock::new(HashMap::new())),
            bases: Arc::new(RwLock::new(HashMap::new())),
            last_diff: Vec3::ZERO,
        }
    }
}



#[derive(Component, Copy, Clone, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
pub struct RotationConstraint{
    pub identity: Quat,
    pub weight: f32,
    pub strength: f32,
    pub x_max: f32,
    pub z_max: f32,
    pub y: Vec2,
    
}

impl Default for RotationConstraint {
    fn default() -> Self {
        Self{
            identity: Quat::IDENTITY,
            weight: 1.0,
            strength: 0.75,
            y: vec2(-3.1415, 3.1415),
            x_max: 1.2566,
            z_max: 1.2566,
        }
    }
}
