use bevy::prelude::*;
use bevy_fabrik_solver::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_egui::EguiPlugin;

fn main(){
    App::new()
        .add_plugins((DefaultPlugins, IkSolverPlugin::default()))
        .add_systems(Startup, setup)
        .add_plugins(EguiPlugin { enable_multipass_for_primary_context: true })
        .add_plugins(WorldInspectorPlugin::new())
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
){
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-0.5, 0.1, 5.0).looking_at(Vec3::ZERO, Dir3::Y),
    ));

    commands.spawn((
        DirectionalLight{
            illuminance: 12500.0,
            ..Default::default()
        },
        Transform::IDENTITY.looking_at(Vec3::new(-0.2, -8.0, 0.1), Dir3::Y),
    ));

    

    let joint_length = 0.5;

    let base = commands.spawn((
        Transform::IDENTITY,
        Name::new("Base"),
    )).id();

    let end_effector1 = commands.spawn((
        Transform::from_xyz(0.15, 3.5, 0.2),
        Name::new("End Effector 1"),
        Mesh3d(meshes.add(Sphere::new(joint_length * 0.1))),
        MeshMaterial3d(materials.add(Color::srgb_u8(200, 20, 25))),
        
    )).id();

    let end_effector2 = commands.spawn((
        Transform::from_xyz(0.15, 3.5, 0.2),
        Name::new("End Effector 2"),
        Mesh3d(meshes.add(Sphere::new(joint_length * 0.1))),
        MeshMaterial3d(materials.add(Color::srgb_u8(200, 20, 25))),
    )).id();
    

    commands.spawn((
        Joint{
            length: joint_length,
            halfway: true,
            ..Default::default()
        },
        BaseJoint(base),
        Mesh3d(meshes.add(Cone::new(joint_length * 0.3, joint_length))),
        MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
        Transform::from_xyz(0.0, 0.0, 0.0),
        children![(
            Joint{
                length: joint_length,
                halfway: true,
                ..Default::default()
            },
            Mesh3d(meshes.add(Cone::new(joint_length * 0.3, joint_length))),
            MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
            Transform::from_xyz(0.0, joint_length, 0.0),
            children![(
                Joint{
                    length: joint_length,
                    halfway: true,
                    ..Default::default()
                },
                Mesh3d(meshes.add(Cone::new(joint_length * 0.3, joint_length))),
                MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
                Transform::from_xyz(0.0, joint_length, 0.0),
                children![(
                    Joint{
                        length: joint_length,
                        halfway: true,
                        ..Default::default()
                    },
                    EndEffectorJoint{
                        ee: end_effector1,
                        joint_center: true,
                        joint_copy_rotation: false,
                    },
                    Mesh3d(meshes.add(Cone::new(joint_length * 0.3, joint_length))),
                    MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
                    Transform::from_xyz(0.0, joint_length, 0.5 * joint_length),
            
                ),
                (
                    Joint{
                        length: joint_length,
                        halfway: true,
                        ..Default::default()
                    },
                    EndEffectorJoint{
                        ee: end_effector2,
                        joint_center: true,
                        joint_copy_rotation: false,
                    },
                    Mesh3d(meshes.add(Cone::new(joint_length * 0.3, joint_length))),
                    MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
                    Transform::from_xyz(0.0, joint_length, -0.5 * joint_length),
            
                )]
            )]
        )]
            
            
        
    ));

    
}
