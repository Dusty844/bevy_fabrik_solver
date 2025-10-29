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
impl IkGlobalSettings {
    fn force_set_gt(mut self) -> IkGlobalSettings {
        self.force_global_transform = true;
        self
        
    }
}


#[derive(Component, Clone, Copy, Default, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
#[require(Transform, JointTransform)]
pub struct Joint{
    pub length: f32,
    pub offset: Vec3,
    pub halfway: bool,
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
    pub split_dir: Dir3,
    pub twist: RangeVec3,
    pub swing: RangeVec3,
}

impl Default for RotationConstraint {
    fn default() -> Self {
        let min = Vec3::splat(-0.75);
        let max = Vec3::splat(0.75);
        Self{
            identity: Quat::IDENTITY,
            weight: 1.0,
            split_dir: Dir3::Y,
            twist: RangeVec3::new(min, max),
            swing: RangeVec3::new(min, max),
        }
    }
}
//would be more effective to switch to single axes twist two axes swing but i havent figured out the maths for that yet.

#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
pub struct RangeVec3{
    min: Vec3,
    max: Vec3,
}
impl Default for RangeVec3 {
    fn default() -> Self {
        Self{
            min: Vec3::MIN,
            max: Vec3::MAX,
        }
    }
}

impl RangeVec3 {
    pub fn new(min: Vec3, max:Vec3) -> RangeVec3 {
        RangeVec3 { min, max }
    }
}
