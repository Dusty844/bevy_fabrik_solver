use bevy::prelude::*;

#[derive(Component, Copy, Clone, Debug)]
#[cfg_attr(feature = "bevy_reflect", derive(Reflect))]
pub struct Constraint{
    twist: RangeVec3,
    swing: RangeVec3,
}

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
