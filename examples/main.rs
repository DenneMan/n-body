use particle_simulation::{Octree, Boundary};
use nalgebra::Point3;

fn main() {
    let mut octree = Octree::new(Boundary::new(Point3::new(0.0, 0.0, 0.0), Point3::new(100.0, 100.0, 100.0)));

    octree.insert(Point3::new(25.0, 25.0, 25.0), 1.0);
    octree.insert(Point3::new(75.0, 25.0, 25.0), 49.0);

    println!("{:?}", octree);
}