use bevy::prelude::*;
use bevy_fabrik_solver::*;
use bevy_inspector_egui::quick::WorldInspectorPlugin;
use bevy_egui::EguiPlugin;

fn main(){
    App::new()
        .add_plugins(DefaultPlugins)
        //.add_plugins(IkSolverPlugin::default())
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
        Transform::from_xyz(-0.5, 1.25, 6.0).looking_at(Vec3::Y, Dir3::Y),
    ));

    commands.spawn((
        DirectionalLight{
            illuminance: 12500.0,
            ..Default::default()
        },
        Transform::IDENTITY.looking_at(Vec3::new(-0.2, -8.0, 0.1), Dir3::Y),
    ));

       
}
