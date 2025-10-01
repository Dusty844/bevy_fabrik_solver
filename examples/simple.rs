use bevy::prelude::*;
use bevy_fabrik_solver::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_egui::EguiPlugin;

fn main(){
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(IkSolverPlugin)
        .add_systems(Startup, setup)
        .add_systems(PostStartup, after.after(TransformSystems::Propagate))
        // .add_plugins(EguiPlugin { enable_multipass_for_primary_context: true })
        // .add_plugins(WorldInspectorPlugin::new())
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    
){
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-0.5, 1.25, 6.0).looking_at(Vec3::Y, Dir3::Y),
    ));

    commands.spawn((
        DirectionalLight{
            illuminance: 12500.0,
            ..Default::default()
        },
        Transform::IDENTITY.looking_at(Vec3::new(-0.2, -8.0, 0.1), Dir3::Y),
    ));
    let joint_length = 0.1;
    let joint = Joint{
        length: joint_length,
        offset: Vec3::ZERO,
        halfway: false,
    };

    let joint = commands.spawn((
        Name::new("BaseJoint"),
        joint,
        Mesh3d(meshes.add(Cone::new(joint_length * 0.3, joint_length))),
        MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
        Transform::from_xyz(0.0, 0.0, 0.0),
        children![(
            Name::new("Joint"),
            joint,
            Mesh3d(meshes.add(Cone::new(joint_length * 0.3, joint_length))),
            MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
            Transform::from_xyz(0.0, joint_length, 0.0),
        )]
    )).id();

    commands.spawn((
        Name::new("Base"),
        Base(joint),
        Mesh3d(meshes.add(Sphere::new(joint_length * 0.3))),
        MeshMaterial3d(materials.add(Color::srgb_u8(200, 144, 255))),
        Transform::from_xyz(0.0, 1.0, 0.0),
    ));

    

       
}

fn after(
    bk: Res<JointBookkeeping>,
){
    println!("{:#?}", bk.joints.lock().unwrap());
}
