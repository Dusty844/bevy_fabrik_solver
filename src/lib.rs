use bevy::{
    ecs::{intern::Interned, schedule::ScheduleLabel},
    prelude::*,
};
mod solver;

pub struct IkSolverPlugin{
    iterations: usize,
    schedule: Interned<dyn ScheduleLabel>,
}

impl Plugin for IkSolverPlugin{
    fn build(&self, app: &mut App) {
        app.add_systems(PostUpdate, solver::solve);
    }
}

impl Default for IkSolverPlugin{
    fn default() -> Self {
        Self{
            iterations: 10,
            schedule: Last.intern(),
        }
    }
}

impl IkSolverPlugin {
    /// Creates a [`IkSolverPlugin`] with the schedule that is used for running the `solve` function.
    ///
    /// The default schedule is `Last`. You may want to set this to be somewhere in `FixedUpdate` and use transform interpolation.
    pub fn new(iters: usize, schedule: impl ScheduleLabel) -> Self {
        Self {
            iterations: iters,
            schedule: schedule.intern(),
        }
    }

    /// Sets the global number of forward backward iterations done in the solver.
    pub fn with_iterations(mut self, iters: usize) -> Self{
        self.iterations = iters;
        self
    }
}


///A `Joint` or bone in an IK chain.
#[derive(Component, Clone, Copy, Default, Reflect, Debug)]
#[reflect(Component, Debug)]
#[require(Transform)]
pub struct Joint{
    pub length: f32,
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
#[require(Transform)]
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
pub struct EndEffectorJoint(Entity);








/// The `Base` is the target that a root [`Joint`] (the [`Joint`] at the very
/// top of the hierarchy) is algorithmically constrained to.
///
/// The Entity field in the [`Base`] Component points to the root joint.
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
pub struct Base(Entity);






/// The `BaseJoint` component is automatically placed on the target
/// [`Joint`] of a [`Base`], this is for the algorithm only, and
/// shouldn't matter to the End User.
#[derive(Component, Clone, Copy, Reflect, Debug)]
#[reflect(Component, Debug)]
#[require(Transform)]
pub struct BaseJoint(Entity);



