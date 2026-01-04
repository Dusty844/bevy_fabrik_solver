use bevy::prelude::*;
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

pub fn rotation_averaging(quats: &Vec<Quat>, weights: &Vec<f32>, quality_count: usize, start_quat: Quat) -> Quat {
    let mut accum = Mat4::ZERO;
    for (i, quat) in quats.iter().enumerate() {
        let [x, y, z, w] = quat.to_array();
        let v = Vec4::new(x, y, z, w);
        accum += weights[i] * Mat4::from_cols(
        v * x,
        v * y,
        v * z,
        v * w,
    );
    }

    let mut final_rot = start_quat.normalize();

    for _ in 0..quality_count {
        let svd = accum.mul_vec4(final_rot.into());
        
        final_rot = Quat::from_vec4(svd).normalize();
    }
    quat_abs(final_rot)
}


pub fn quat_abs(
    x: Quat
) -> Quat {
    if x.w < 0.0 {
        -x
    } else {
        x
    }
}

