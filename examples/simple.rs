use bevy::prelude::*;
use bevy_fabrik_solver::*;

fn main(){
    App::new()
        .add_plugins((DefaultPlugins, IkSolverPlugin::default()))
        .add_systems(Startup, setup)
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

    

    commands.spawn((
        Joint{
            length: joint_length,
        },
        Mesh3d(meshes.add(Cone::new(joint_length * 0.3, joint_length))),
        MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
        Transform::from_xyz(0.0, 0.0, 0.0),
        children![(
            Joint{
                length: joint_length,
            },
            Mesh3d(meshes.add(Cone::new(joint_length * 0.3, joint_length))),
            MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
            Transform::from_xyz(0.0, joint_length, 0.0),
            children![(
                Joint{
                    length: joint_length,
                },
                Mesh3d(meshes.add(Cone::new(joint_length * 0.3, joint_length))),
                MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
                Transform::from_xyz(0.0, joint_length, 0.0),
                children![(
                    Joint{
                        length: joint_length,
                    },
                    Mesh3d(meshes.add(Cone::new(joint_length * 0.3, joint_length))),
                    MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
                    Transform::from_xyz(0.0, joint_length, 0.0),
            
                )]
            )]
        )]
            
            
        
    ));

    
}
