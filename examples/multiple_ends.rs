// This example shows both multiple joint chains and also multiple end
// effectors in action together, just note that the extra end effector is
// NOT like a pole target, at least with a normal weight. It freaks out
// a little if you move the end effector in question to a place the chain
// can't reach, or if the whole chain is tensioned.
use bevy::{input::mouse::AccumulatedMouseScroll, prelude::*};
use bevy_fabrik_solver::*;

fn main(){
    App::new()
        .add_plugins((DefaultPlugins, MeshPickingPlugin))
        .add_plugins(IkSolverPlugin)
        .add_systems(Startup, setup)
        .run();
}


// we don't need to turn on the global transform setting, since the
// standard bevy hierarchy is flat.
fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
){
    
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-0.5, 1.25, 3.0).looking_at(Vec3::ZERO, Dir3::Y),
    ));

    commands.spawn((
        DirectionalLight{
            illuminance: 18500.0,
            ..Default::default()
        },
        Transform::IDENTITY.looking_at(Vec3::new(-0.2, -8.0, 0.1), Dir3::Y),
    ));

    //the length of each joint
    let joint_length = 0.2;

    
    let joint = (Joint{
        length: joint_length,
        visual_offset: Vec3::Y * joint_length * 0.5,
        anchor_offset: Vec3::ZERO,
    }, RotationConstraint{
            identity: Quat::IDENTITY,
            weight: 1.0,
            strength: 1.0,
            ..Default::default()
        }
);

    let mesh = (
        Mesh3d(meshes.add(Cone::new(joint_length * 0.15, joint_length))),
        MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
    );

    let base_material = StandardMaterial{
        base_color: Color::srgb_u8(230, 144, 120),
        unlit: true,
        ..Default::default()
    };
    
    let base = commands.spawn((
        Name::new("Base"),
        Mesh3d(meshes.add(Sphere::new(joint_length * 0.2))),
        MeshMaterial3d(materials.add(base_material)),
        Transform::from_xyz(0.0, 0.0, 0.0),
    )).observe(translate_on_drag).observe(hover_scroll).id();

    let end_material = StandardMaterial{
        base_color: Color::srgb_u8(100, 144, 255),
        unlit: true,
        ..Default::default()
    };
    
    
    let end1 = commands.spawn((
        Name::new("End 1"),
        Mesh3d(meshes.add(Sphere::new(joint_length * 0.2))),
        MeshMaterial3d(materials.add(end_material.clone())),
        Transform::from_xyz(-0.25, 0.8, 0.0),
    )).observe(translate_on_drag).observe(hover_scroll).id();

    let end2 = commands.spawn((
        Name::new("End 2"),
        Mesh3d(meshes.add(Sphere::new(joint_length * 0.2))),
        MeshMaterial3d(materials.add(end_material.clone())),
        Transform::from_xyz(0.25, 0.9, 0.0),
    )).observe(translate_on_drag).observe(hover_scroll).id();

        
    //A third end effector with a lower weight, for good measure
    let end3 = commands.spawn((
        Name::new("End 3"),
        //slightly smaller sphere, to show it has less influence / is less important.
        Mesh3d(meshes.add(Sphere::new(joint_length * 0.18))),
        MeshMaterial3d(materials.add(end_material.clone())),
        Transform::from_xyz(0.2, 0.6, 0.0),
        EndEffector{
            // on an end effector like this, if the weight is too high,
            // it'll freak out. Too low and it'll be all floaty and weird.
            // Also, you can add an end effector like this if you want
            // to change the end effector settings.
            weight: 0.01, 
            ..Default::default()
        }
    )).observe(translate_on_drag).observe(hover_scroll).id();

        

// when doing it this way, the joints aren't acutally children of
// eachother as far as bevy is concerned, they're only connected in the
// eyes of the ik plugin

    commands.spawn((
        Name::new("BaseJoint"),
        BaseJoint(base),
        Joint{ // if the base joint has a Rotational Constraint component, it will be constrained to the base.
            length: joint_length,
            visual_offset: Vec3::Y * joint_length * 0.5,
            anchor_offset: Vec3::ZERO,
        },
        mesh.clone(),
        Transform::from_xyz(0.0, 0.0, 0.0),
        related!(JointChildren[(
            Name::new("Joint"),
            joint,
            mesh.clone(),
            Transform::from_xyz(0.0, joint_length, 0.0),
            related!(JointChildren[(
                Name::new("Joint"),
                joint,
                mesh.clone(),
                Transform::from_xyz(0.0, joint_length, 0.0),
                related!(JointChildren[(
                    Name::new("Joint"),
                    joint,
                    mesh.clone(),
                    Transform::from_xyz(0.0, joint_length, 0.0),
                    related!(JointChildren[(
                        Name::new("EndJoint"),
                        joint,
                        mesh.clone(),
                        Transform::from_xyz(0.0, joint_length, 0.0),
                        EEJoint(end1),
                    )])
                )])
            ), (
                Name::new("Joint"),
                joint,
                mesh.clone(),
                Transform::from_xyz(0.0, joint_length, 0.0),
                related!(JointChildren[(
                    Name::new("Joint"),
                    joint,
                    mesh.clone(),
                    Transform::from_xyz(0.0, joint_length, 0.0),
                    EEJoint(end3),
                    related!(JointChildren[(
                        Name::new("EndJoint"),
                        joint,
                        mesh.clone(),
                        Transform::from_xyz(0.0, joint_length, 0.0),
                        related!(JointChildren[(
                            Name::new("EndJoint"),
                            joint,
                            mesh.clone(),
                            Transform::from_xyz(0.0, joint_length, 0.0),
                            EEJoint(end2),
                        )])
                    )])
                )])
            )])
        )])
    ));

}



//functions for translating the effectors

fn translate_on_drag(
    drag: On<Pointer<Drag>>,
    mut transforms: Query<&mut Transform>,
    camera_s: Single<(&Camera, &GlobalTransform)>,
    scroll: Res<AccumulatedMouseScroll>,
){
    if drag.button == PointerButton::Primary {
        let Ok(mut transform) = transforms.get_mut(drag.entity) else {
            return;
        };

        let (camera, camera_transform) = *camera_s;

        let cursor_pos = drag.pointer_location.position;

        let Ok(ray) = camera.viewport_to_world(camera_transform, cursor_pos) else {
            return;
        };

        let depth = ray
            .direction
            .dot(transform.translation - ray.origin);

        if depth <= 0.0 {
            return;
        }

        let shifted_cursor = cursor_pos + drag.delta;

        let Ok(shifted_ray) =
            camera.viewport_to_world(camera_transform, shifted_cursor)
        else {
            return;
        };

        transform.translation = shifted_ray.get_point(depth);
        if scroll.delta.y.abs() > 0.0 {
            transform.translation += ray.direction * scroll.delta.y * 0.1;
        }
    } 
    
}

fn hover_scroll(
    scroll_e: On<Pointer<Scroll>>,
    mut transforms: Query<&mut Transform>,
    camera_s: Single<(&Camera, &GlobalTransform)>,
){
let Ok(mut transform) = transforms.get_mut(scroll_e.entity) else {
        return;
    };

    let (camera, camera_transform) = *camera_s;

    let cursor_pos = scroll_e.pointer_location.position;

    let Ok(ray) = camera.viewport_to_world(camera_transform, cursor_pos) else {
        return;
    };

    if scroll_e.y.abs() > 0.0 {
        transform.translation += ray.direction * scroll_e.y * 0.1;
    }
    
    
}
