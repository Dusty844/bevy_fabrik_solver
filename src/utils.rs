use bevy::prelude::*;
use bevy::math::Affine3A;
use super::JointTransform;

impl JointTransform {
    pub fn local_x(self) -> Dir3 {
        Dir3::new(self.rotation * Vec3::X).unwrap()
    }
    pub fn local_y(self) -> Dir3 {
        Dir3::new(self.rotation * Vec3::Y).unwrap()
    }
    pub fn local_z(self) -> Dir3 {
        Dir3::new(self.rotation * Vec3::Z).unwrap()
    }
    pub fn align(&mut self, main_axis: impl TryInto<Dir3>, main_dir: impl TryInto<Dir3>, second_axis: impl TryInto<Dir3>, second_dir: impl TryInto<Dir3>) {
        let main_axis = main_axis.try_into().unwrap_or(Dir3::X);
        let main_direction = main_dir.try_into().unwrap_or(Dir3::X);
        let secondary_axis = second_axis.try_into().unwrap_or(Dir3::Y);
        let secondary_direction = second_dir.try_into().unwrap_or(Dir3::Y);

        // The solution quaternion will be constructed in two steps.
        // First, we start with a rotation that takes `main_axis` to `main_direction`.
        let first_rotation = Quat::from_rotation_arc(main_axis.into(), main_direction.into());

        // Let's follow by rotating about the `main_direction` axis so that the image of `secondary_axis`
        // is taken to something that lies in the plane of `main_direction` and `secondary_direction`. Since
        // `main_direction` is fixed by this rotation, the first criterion is still satisfied.
        let secondary_image = first_rotation * secondary_axis;
        let secondary_image_ortho = secondary_image
            .reject_from_normalized(main_direction.into())
            .try_normalize();
        let secondary_direction_ortho = secondary_direction
            .reject_from_normalized(main_direction.into())
            .try_normalize();

        // If one of the two weak vectors was parallel to `main_direction`, then we just do the first part
        self.rotation = match (secondary_image_ortho, secondary_direction_ortho) {
            (Some(secondary_img_ortho), Some(secondary_dir_ortho)) => {
                let second_rotation =
                    Quat::from_rotation_arc(secondary_img_ortho, secondary_dir_ortho);
                second_rotation * first_rotation
            }
            _ => first_rotation,
        };
    }
    pub fn aligned_by(mut self, main_axis: impl TryInto<Dir3>, main_dir: impl TryInto<Dir3>, second_axis: impl TryInto<Dir3>, second_dir: impl TryInto<Dir3>) -> JointTransform {
        self.align(main_axis, main_dir, second_axis, second_dir);

        self
    }
    
    
    
}

pub trait AffineExtra{
    fn align(&mut self, main_axis: impl TryInto<Dir3>, main_dir: impl TryInto<Dir3>, second_axis: impl TryInto<Dir3>, second_dir: impl TryInto<Dir3>);
    fn aligned_by(self, main_axis: impl TryInto<Dir3>, main_dir: impl TryInto<Dir3>, second_axis: impl TryInto<Dir3>, second_dir: impl TryInto<Dir3>) -> Affine3A;
    fn local_x(&self) -> Dir3A;
    fn local_y(&self) -> Dir3A;
    fn local_z(&self) -> Dir3A;
}

pub trait QuatExtra{
    fn twist_swing(&self, dir: Vec3) -> (Quat, Quat);
    fn align(&mut self, main_axis: impl TryInto<Dir3>, main_dir: impl TryInto<Dir3>, second_axis: impl TryInto<Dir3>, second_dir: impl TryInto<Dir3>);
}

impl AffineExtra for Affine3A{
    //shamelessly ripped straight from one of bevy's transform align and aligned_by functions. what can i say? it's a really useful function. 
    fn align(&mut self, main_axis: impl TryInto<Dir3>, main_dir: impl TryInto<Dir3>, second_axis: impl TryInto<Dir3>, second_dir: impl TryInto<Dir3>){
        let main_axis = main_axis.try_into().unwrap_or(Dir3::X);

        let main_direction = main_dir.try_into().unwrap_or(Dir3::X);

        let secondary_axis = second_axis.try_into().unwrap_or(Dir3::Y);

        let secondary_direction = second_dir.try_into().unwrap_or(Dir3::Y);
        
        let first_rotation = Quat::from_rotation_arc(main_axis.into(), main_direction.into());
        
        let secondary_image = first_rotation * secondary_axis;
        let secondary_image_ortho = secondary_image
            .reject_from_normalized(main_direction.into())
            .try_normalize();
        let secondary_direction_ortho = secondary_direction
            .reject_from_normalized(main_direction.into())
            .try_normalize();

                let mut srt = self.to_scale_rotation_translation();
        srt.1 = match (secondary_image_ortho, secondary_direction_ortho) {

            (Some(secondary_img_ortho), Some(secondary_dir_ortho)) => {

                let second_rotation =

                    Quat::from_rotation_arc(secondary_img_ortho, secondary_dir_ortho);

                second_rotation * first_rotation

            }

            _ => first_rotation,

        }.normalize();
        *self = Affine3A::from_scale_rotation_translation(srt.0, srt.1.normalize(), srt.2);
        
        
    }

    fn aligned_by(mut self, main_axis: impl TryInto<Dir3>, main_dir: impl TryInto<Dir3>, second_axis: impl TryInto<Dir3>, second_dir: impl TryInto<Dir3>) -> Affine3A {
        self.align(main_axis, main_dir, second_axis, second_dir);

        self
    }
    fn local_x(&self) -> Dir3A {
        Dir3A::new_unchecked(self.to_scale_rotation_translation().1 * Vec3A::X).fast_renormalize()
    }
    fn local_y(&self) -> Dir3A {
        Dir3A::new_unchecked(self.to_scale_rotation_translation().1 * Vec3A::Y).fast_renormalize()
    }
    fn local_z(&self) -> Dir3A {
        Dir3A::new_unchecked(self.to_scale_rotation_translation().1 * Vec3A::Z).fast_renormalize()
    }
}

impl QuatExtra for Quat{
    fn twist_swing(&self, dir: Vec3) -> (Quat, Quat) {
        let projected = self.xyz().project_onto(dir);
        let twist = Quat::from_xyzw(projected.x, projected.y, projected.z, self.w).normalize();
        let swing = *self * twist.inverse();

        (twist, swing)
    }
    

    fn align(&mut self, main_axis: impl TryInto<Dir3>, main_dir: impl TryInto<Dir3>, second_axis: impl TryInto<Dir3>, second_dir: impl TryInto<Dir3>){
        let main_axis = main_axis.try_into().unwrap_or(Dir3::X);

        let main_direction = main_dir.try_into().unwrap_or(Dir3::X);

        let secondary_axis = second_axis.try_into().unwrap_or(Dir3::Y);

        let secondary_direction = second_dir.try_into().unwrap_or(Dir3::Y);
        
        let first_rotation = Quat::from_rotation_arc(main_axis.into(), main_direction.into());
        
        let secondary_image = first_rotation * secondary_axis;
        let secondary_image_ortho = secondary_image
            .reject_from_normalized(main_direction.into())
            .try_normalize();
        let secondary_direction_ortho = secondary_direction
            .reject_from_normalized(main_direction.into())
            .try_normalize();

                
        *self = match (secondary_image_ortho, secondary_direction_ortho) {

            (Some(secondary_img_ortho), Some(secondary_dir_ortho)) => {

                let second_rotation =

                    Quat::from_rotation_arc(secondary_img_ortho, secondary_dir_ortho);

                second_rotation * first_rotation

            }

            _ => first_rotation,

        }.normalize();
        
        
        
    }
}

pub fn rotation_averaging(quats: &Vec<Quat>, weights: &Vec<f32>, quality_count: usize, start_quat: Quat) -> Quat {
    let mut accum = [[0.0; 4]; 4];
    for (x, quat) in quats.iter().enumerate() {
        for i in 0..4 {
            for j in 0..4 {
                accum[i][j] += weights[x] * (quat.to_array()[i] * quat.to_array()[j]);
            }
        }
    }

    let mut final_rot = start_quat.normalize();

    //higher the count, the better the approximation, when testing without a start quaternion
    for _ in 0..quality_count {
        let mut svd = [0.0; 4];
        for i in 0..4 {
            for j in 0..4 {
                svd[i] += accum[i][j] * final_rot.to_array()[j]
            }
        }
        final_rot = Quat::from_array(svd).normalize();
    }
    if final_rot.x < 0.0 {
        final_rot *= Quat::from_slice(&[-1.0; 4]);
    }
    final_rot
}
