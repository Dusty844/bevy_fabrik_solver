use bevy::{input::mouse::AccumulatedMouseScroll, prelude::*};
use bevy_fabrik_solver::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_egui::EguiPlugin;


fn main(){
    App::new()
        .add_plugins((DefaultPlugins, MeshPickingPlugin))
        .add_plugins(IkSolverPlugin)
        .add_systems(Startup, setup)
        .add_plugins(EguiPlugin::default())
        .add_plugins(WorldInspectorPlugin::new())
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut ik_settings: ResMut<IkGlobalSettings>,
    
){
    //since we have a  standard (bevy Children and ChildOf), set this to true
    ik_settings.force_global_transform = true;
    
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

    let base_material = StandardMaterial{
        base_color: Color::srgb_u8(230, 144, 120),
        unlit: true,
        ..Default::default()
    };
    
    //the Base, the entity that the root / base joint attatches itself to.
    let base = commands.spawn((
        Name::new("Base"),
        Mesh3d(meshes.add(Sphere::new(0.05))),
        MeshMaterial3d(materials.add(base_material)),
        Transform::from_xyz(0.0, 0.0, 0.0),
    )).observe(translate_on_drag).observe(hover_scroll).id();

    let end_material = StandardMaterial{
        base_color: Color::srgb_u8(100, 144, 255),
        unlit: true,
        ..Default::default()
    };

    let l_f_end = commands.spawn((
        Name::new("Left Foot End Effector"),
        Mesh3d(meshes.add(Sphere::new(0.05))),
        MeshMaterial3d(materials.add(end_material.clone())),
        Transform::from_xyz(-0.3, 0.0, 0.0),
    )).observe(translate_on_drag).observe(hover_scroll).id();

    let r_f_end = commands.spawn((
        Name::new("Right Foot End Effector"),
        Mesh3d(meshes.add(Sphere::new(0.05))),
        MeshMaterial3d(materials.add(end_material.clone())),
        Transform::from_xyz(0.3, 1.0, 0.0),
    )).observe(translate_on_drag).observe(hover_scroll).id();

    let l_h_end = commands.spawn((
        Name::new("Left Foot End Effector"),
        Mesh3d(meshes.add(Sphere::new(0.05))),
        MeshMaterial3d(materials.add(end_material.clone())),
        Transform::from_xyz(-0.5, 1.0, 0.0),
    )).observe(translate_on_drag).observe(hover_scroll).id();

    let r_h_end = commands.spawn((
        Name::new("Right Foot End Effector"),
        Mesh3d(meshes.add(Sphere::new(0.05))),
        MeshMaterial3d(materials.add(end_material.clone())),
        Transform::from_xyz(0.5, 1.0, 0.0),
    )).observe(translate_on_drag).observe(hover_scroll).id();

    let head_end = commands.spawn((
        Name::new("Head End Effector"),
        Mesh3d(meshes.add(Sphere::new(0.05))),
        MeshMaterial3d(materials.add(end_material.clone())),
        Transform::from_xyz(0.0, 2.0, 0.0),
    )).observe(translate_on_drag).observe(hover_scroll).id();

    commands.spawn((
        Name::new("Root"),
        Transform::from_xyz(0.0, 0.0, 1.0),
        Joint{
            length: 0.15,
            visual_offset: Vec3::Y * 0.15 * 0.5,
            ..Default::default()
        },
        Mesh3d(meshes.add(Cone::new(0.15 * 0.15, 0.15))),
        MeshMaterial3d(materials.add(Color::srgb_u8(60, 120, 255))),
        BaseJoint(base),
        children![(
            Name::new("UpperLeg_l"),
            Transform::from_xyz(0.0, 0.0, 1.0),
            Joint{
                length: 0.53,
                visual_offset: Vec3::Y * 0.53 * 0.5,
                ..Default::default()
            },
            RotationConstraint{
              identity: Quat::from_rotation_arc(Vec3::Y, Vec3::NEG_Y),
              y_max: 0.628,
              ..Default::default()  
            },
            Mesh3d(meshes.add(Cone::new(0.53 * 0.15, 0.53))),
            MeshMaterial3d(materials.add(Color::srgb_u8(60, 120, 255))),
            children![(
                Name::new("LowerLeg_l"),
                Transform::from_xyz(0.0, 0.0, 1.0),
                Joint{
                    length: 0.45,
                    visual_offset: Vec3::Y * 0.45 * 0.5,
                    ..Default::default()
                },
                RotationConstraint{
                  identity: Quat::from_rotation_arc(Vec3::Y, Vec3::NEG_Y),
                  y_max: 0.628,
                  ..Default::default()  
                },
                Mesh3d(meshes.add(Cone::new(0.45 * 0.15, 0.53))),
                MeshMaterial3d(materials.add(Color::srgb_u8(60, 120, 255))),
                children![(
                    Name::new("Foot_l"),
                    Transform::from_xyz(0.0, 0.0, 1.0),
                    Joint{
                        length: 0.45,
                        visual_offset: Vec3::Y * 0.45 * 0.5,
                        ..Default::default()
                    },
                    RotationConstraint{
                      identity: Quat::from_rotation_arc(Vec3::Y, Vec3::Z),
                      y_max: 0.392,
                      ..Default::default()  
                    },
                    Mesh3d(meshes.add(Cone::new(0.45 * 0.15, 0.53))),
                    MeshMaterial3d(materials.add(Color::srgb_u8(60, 120, 255))),
                    EEJoint(l_f_end)
                )]
            )]
        ), (
            Name::new("UpperLeg_r"),
            Transform::from_xyz(0.0, 0.0, 1.0),
            Joint{
                length: 0.53,
                visual_offset: Vec3::Y * 0.53 * 0.5,
                ..Default::default()
            },
            RotationConstraint{
              identity: Quat::from_rotation_arc(Vec3::Y, Vec3::NEG_Y),
              y_max: 0.628,
              ..Default::default()  
            },
            Mesh3d(meshes.add(Cone::new(0.53 * 0.15, 0.53))),
            MeshMaterial3d(materials.add(Color::srgb_u8(60, 120, 255))),
            children![(
                Name::new("LowerLeg_r"),
                Transform::from_xyz(0.0, 0.0, 1.0),
                Joint{
                    length: 0.45,
                    visual_offset: Vec3::Y * 0.45 * 0.5,
                    ..Default::default()
                },
                RotationConstraint{
                  identity: Quat::from_rotation_arc(Vec3::Y, Vec3::NEG_Y),
                  y_max: 0.628,
                  ..Default::default()  
                },
                Mesh3d(meshes.add(Cone::new(0.45 * 0.15, 0.53))),
                MeshMaterial3d(materials.add(Color::srgb_u8(60, 120, 255))),
                children![(
                    Name::new("Foot_r"),
                    Transform::from_xyz(0.0, 0.0, 1.0),
                    Joint{
                        length: 0.45,
                        visual_offset: Vec3::Y * 0.45 * 0.5,
                        ..Default::default()
                    },
                    RotationConstraint{
                      identity: Quat::from_rotation_arc(Vec3::Y, Vec3::Z),
                      y_max: 0.392,
                      ..Default::default()  
                    },
                    Mesh3d(meshes.add(Cone::new(0.45 * 0.15, 0.53))),
                    MeshMaterial3d(materials.add(Color::srgb_u8(60, 120, 255))),
                    EEJoint(r_f_end)
                )]
            )]
        )]
    ));
    


}



// observer to translate the end and base in this example.
fn translate_on_drag(
    drag: On<Pointer<Drag>>,
    mut transforms: Query<&mut Transform>,
    camera_s: Single<(&Camera, &GlobalTransform)>,
    scroll: Res<AccumulatedMouseScroll>,
){
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

//so the scroll part works when we aren't dragging
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
