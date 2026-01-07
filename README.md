# bevy_fabrik_solver
An Inverse Kinematics Solver based on the FABRIK algorithm. Aims to be deeply integrated with bevy's ECS.


## General Features

- Rotation constraints (somewhat limited at the moment, will revise at some point soon).

- Uses Bevy's One-to-Many Relationships, allowing for multiple children joints.

- Multiple end effectors on one chain: One end effector per joint (if you so wish).

- Automatic handling of joint relationships.

- Rotational and Translational weighting of joints and end effectors (useful when there is more then one thing to point at).

- Also is an IK solver



## Usage

First, Add `bevy_fabrik_solver` to your project, alongside `bevy`:
```toml

# Cargo.toml
[dependencies]
bevy = "0.17.0"
bevy_fabrik_solver = "0.1"

```

Then, a minimal example of an IK Joint Chain, with one End Effector, and one Base Joint:

```rust
use bevy::prelude::*;
use bevy_fabrik_solver::*;

fn main(){
    App::new()
        .add_plugins(DefaultPlugins)
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
    //since we have a standard hierarchy(bevy Children and ChildOf), set this to true
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
    ));

    
    
    let joint = Joint{
        length: joint_length,
        visual_offset: Vec3::Y * joint_length * 0.5,
        ..Default::default()
    };

    let mesh = (
        Mesh3d(meshes.add(Cone::new(joint_length * 0.15, joint_length))),
        MeshMaterial3d(materials.add(Color::srgb_u8(124, 144, 255))),
    );


    let base = commands.spawn((
        Base::default(), //the base, the joint chain originates from and sticks to this
        Transform::from_xyz(0.0, 0.0, 0.0),
    )).id();

    
    let end = commands.spawn((
        EndEffector::default() //the end effector, the joint chain will point towards this
        Transform::from_xyz(0.0, 1.0, 0.0),
    )).id();

    commands.spawn((
        BaseJoint(base), //the first joint, which will stick to the base
        joint,
        mesh.clone(),
        children![(
            joint,
            mesh.clone(),
            children![(
                joint,
                mesh.clone(),
                children![(
                    joint,
                    mesh.clone(),
                    children![(
                        joint,
                        mesh.clone(),
                        EEJoint(end), //the final joint that will point towards the end effector
                    )]
                )]
            )]
        )]
    ));

    
}


```
The Above Example Will result in something similar to this (see the [simple.rs](examples/simple.rs) example):

https://github.com/user-attachments/assets/93d7c481-cce7-4176-9866-988f1899b91a


## License

This project is dual-licensed under either

- MIT License: Available [online](http://opensource.org/licenses/MIT)
- Apache License, Version 2.0: Available [online](http://www.apache.org/licenses/LICENSE-2.0)

at your option.
