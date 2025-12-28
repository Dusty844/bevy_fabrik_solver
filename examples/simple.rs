use bevy::{input::mouse::AccumulatedMouseScroll, prelude::*};
use bevy_fabrik_solver::*;

fn main(){
    App::new()
        .add_plugins((DefaultPlugins, MeshPickingPlugin))
        .add_plugins(IkSolverPlugin)
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut ik_settings: ResMut<IkGlobalSettings>,
    
){
    //since we have a standard (bevy Children and ChildOf), set this to true
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

    //the length of each joint
    let joint_length = 0.2;
    
    let joint = (
        Joint{
            length: joint_length,
            visual_offset: Vec3::Y * joint_length * 0.5,

            //the offset of the current base from the parent's end, obviously doesn't do anything to the bottom joint.
            anchor_offset: Vec3::ZERO,
        },
        RotationConstraint{
            identity: Quat::IDENTITY,
            weight: 1.0,
            strength: 0.95,
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
    
    //the Base, the entity that the root / base joint attatches itself to.
    let base = commands.spawn((
        Name::new("Base"),
        Base::default(),
        Mesh3d(meshes.add(Sphere::new(joint_length * 0.2))),
        MeshMaterial3d(materials.add(base_material)),
        Transform::from_xyz(0.0, 0.0, 0.0),

    )).observe(translate_on_drag).observe(hover_scroll).id();

    let end_material = StandardMaterial{
        base_color: Color::srgb_u8(100, 144, 255),
        unlit: true,
        ..Default::default()
    };
    
    
    //The End Effector, the entity that the chain points to.
    let end = commands.spawn((
        Name::new("End"),
        EndEffector::default(),
        Mesh3d(meshes.add(Sphere::new(joint_length * 0.2))),
        MeshMaterial3d(materials.add(end_material)),
        Transform::from_xyz(0.0, 1.0, 0.0),
    )).observe(translate_on_drag).observe(hover_scroll).id();

// A joint chain does not need to use bevy's default transform hierarchy
// of Children and ChildOf, since it uses it's own relationship being
// JointChildren and JointParent, we can use the default here because of
// a function that runs on startup which automatically places the required
// JointParent and JointChildren Components onto joints if they have
// parents or children that also have the Joint component on them. You
// don't have to spawn the joints as Children of eachother directly like
// we are here with the children! macro, but if you don't then be sure
// to add the relationships yourself, either using related! macro or
// some other way. 

    commands.spawn((
        Name::new("BaseJoint"),
        BaseJoint(base),
        joint,
        mesh.clone(),
        Transform::from_xyz(0.0, 0.0, 0.0),
        children![(
            Name::new("Joint"),
            joint,
            mesh.clone(),
            Transform::from_xyz(0.0, joint_length, 0.0),
            children![(
                Name::new("Joint"),
                joint,
                mesh.clone(),
                Transform::from_xyz(0.0, joint_length, 0.0),
                children![(
                    Name::new("Joint"),
                    joint,
                    mesh.clone(),
                    Transform::from_xyz(0.0, joint_length, 0.0),
                    children![(
                        Name::new("EndJoint"),
                        joint,
                        mesh.clone(),
                        Transform::from_xyz(0.0, joint_length, 0.0),
                        EEJoint(end),
                    )]
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
