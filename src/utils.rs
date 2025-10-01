use bevy::prelude::*;
use bevy::math::Affine3A;

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

// courtesy of Daniel Holden:
// https://theorangeduck.com/page/quaternion-weighted-average

pub fn accurate_quat_average(
    quats: &Vec<Quat>,
    weights: &Vec<f32>,
    quality_count: usize,
    start_quat: Quat,
)-> Quat {
    let mut accum = Mat4::ZERO;

    for (i, q) in quats.iter().enumerate() {
        // assumes that quats are xyzw instead of wxyz.. i hope
        accum += weights[i] * mat4(
            vec4(q.x * q.w, q.x * q.x, q.x * q.y, q.x * q.z),
            vec4(q.y * q.w, q.y * q.x, q.y * q.y, q.y * q.z),
            vec4(q.z * q.w, q.z * q.x, q.z * q.y, q.z * q.z),
            vec4(q.w * q.w, q.w * q.x, q.w * q.y, q.w * q.z)
        );  
    }
    let u = svd_dominant_eigen(accum, start_quat.into(), quality_count, 0.0001);
    let v = (accum * u).normalize();

    quat_abs(quat(v.x, v.y, v.z, v.w))
}

fn svd_dominant_eigen(
    a: Mat4,
    v0: Vec4,
    iterations: usize,
    epsilon: f32,
) -> Vec4{
    let mut v = v0;
    let mut ev = ((a * v) / v).x;

    for i in 0..iterations {

        
        let av = a * v;

        let v_new = av.normalize();
        let ev_new = ((a * v_new) / v_new).x;

        if f32::abs(ev - ev_new) < epsilon{
            break;
        }
        v = v_new;
        ev = ev_new;
    }

    v
}


fn quat_abs(a: Quat) -> Quat {
        if a.w.is_sign_negative() {
            -a
        }else {
            a
        }
    }
