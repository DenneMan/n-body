use particle_simulation::{World, Octree, Boundary, Body};
use nalgebra::{Point3, Vector3};

use bevy::prelude::*;

use bevy_flycam::PlayerPlugin;

use rand::prelude::*;

fn point_in_sphere(rng: &mut ThreadRng) -> Point3<f32> {
    let v: Vector3<f32> = Vector3::new(rng.gen::<f32>() - 0.5, rng.gen::<f32>() - 0.5, rng.gen::<f32>() - 0.5) * 2.0;
    if v.magnitude_squared() == 0.0 {
        return Point3::origin() + v;
    }

    let r = rng.gen::<f32>().cbrt();

    Point3::origin() + v.normalize() * r
}

#[derive(Resource)]
struct BevyWorld(World);

#[derive(Component)]
struct BodyID {
    id: usize,
}

fn main() {
    App::new()
        .insert_resource(ClearColor(Color::rgb(0.0, 0.0, 0.0)))
        .add_plugins(DefaultPlugins)
        .add_plugin(PlayerPlugin)
        .add_startup_system(setup)
        .add_system(update_simulation)
        .run();

    let mut octree = Octree::new(Boundary::new(Point3::new(0.0, 0.0, 0.0), Point3::new(100.0, 100.0, 100.0)));

    octree.insert(Point3::new(25.0, 25.0, 25.0), 50.0);
    octree.insert(Point3::new(75.0, 25.0, 25.0), 500.0);

    println!("{}", octree.calculate_acceleration(Point3::new(50.0, 25.0, 25.0), 0.0));
}

const BODY_COUNT: usize = 100;

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>
) {
    let shared_mesh = meshes.add(Mesh::from(shape::Icosphere {
        subdivisions: 0,
        radius: 1.0,
    }));
    let shared_material = materials.add(StandardMaterial {
        base_color: Color::WHITE,
        unlit: true,
        ..Default::default()
    });

    let mut rng = thread_rng();

    let mut world = World::new();

    for id in 0..BODY_COUNT {
        let pos: Point3<f32> = point_in_sphere(&mut rng);
        let vel: Vector3<f32> = Vector3::new(pos.z, 0.0, -pos.x);
        let mesh = shared_mesh.clone();
        let material = shared_material.clone();

        world.insert(Body::new(pos, Vector3::zeros(), 1_000_000.0, 0.01));
        commands.spawn(PbrBundle {
            mesh,
            material,
            transform: Transform::from_xyz(pos.x, pos.y, pos.z).with_scale(Vec3::splat(0.005)),
            ..Default::default()
        }).insert(BodyID{id});
    }

    commands.insert_resource(BevyWorld(world));
}

fn update_simulation(
    mut bodies: Query<(&mut Transform, &mut BodyID)>,
    mut world: ResMut<BevyWorld>,
) {
    world.0.update_substeps(0.01, 1);

    for (mut transform, body) in bodies.iter_mut() {
        let pos: Point3<f32> = world.0.bodies[body.id].pos;
        transform.translation = Vec3::new(pos.x, pos.y, pos.z);
    }
}