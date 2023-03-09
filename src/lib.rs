use nalgebra::{Point2, Vector2};

#[derive(Default)]
pub struct Body {
    pub pos: Point2<f32>,
    pub vel: Vector2<f32>,
    pub acc: Vector2<f32>,

    pub mass: f32,
    pub radius: f32,
}

impl Body {
    pub fn new(pos: Point2<f32>, vel: Vector2<f32>, mass: f32, radius: f32) -> Self {
        Self {
            pos,
            vel,
            acc: Vector2::zeros(),

            mass,
            radius,
        }
    }

    pub fn update(&mut self, dt: f32, acc: Vector2<f32>) {
        let new_pos: Point2<f32> = self.pos + self.vel*dt + self.acc*(dt*dt*0.5);
        let new_acc: Vector2<f32> = acc;
        let new_vel: Vector2<f32> = self.vel + (self.acc+new_acc)*(dt*0.5);
        self.pos = new_pos;
        self.vel = new_vel;
        self.acc = new_acc;
    }
}

#[derive(Debug)]
pub struct Boundary {
    pub min: Point2<f32>,
    pub max: Point2<f32>,
}

impl Boundary {
    pub fn new(min: Point2<f32>, max: Point2<f32>) -> Self {
        Self {
            min, 
            max,
        }
    }

    pub fn contains(&self, position: Point2<f32>) -> bool {
        position >= self.min && position <= self.max
    }

    pub fn intersects(&self, other: Self) -> bool {
        self.min <= other.max && self.max >= other.min
    }

    pub fn width(&self) -> f32 {
        self.max.x - self.min.x
    }

    pub fn height(&self) -> f32 {
        self.max.y - self.min.y
    }

    pub fn size(&self) -> Vector2<f32> {
        self.max - self.min
    }

    pub fn map_point(&self, point: Point2<f32>, to: &Self) -> Point2<f32> {
        to.min + (point - self.min).component_div(&self.size()).component_mul(&to.size())
    }

    pub fn map_distance(&self, distance: f32, to: &Self) -> f32 {
        distance / self.width() * to.width()
    }

    pub fn pad(mut self, pad: f32) -> Self {
        self.min.x -= pad;
        self.min.y -= pad;
        self.max.x += pad;
        self.max.y += pad;

        self
    }
}

/// Barnes-Hut octree implementation
#[derive(Debug)]
pub struct Quadtree {
    children: Vec<Quadtree>,
    center_of_mass: Point2<f32>,
    total_mass: f32,
    boundary: Boundary,
}

impl Quadtree {
    pub fn new(boundary: Boundary) -> Self {
        Self {
            children: Vec::new(),
            center_of_mass: Point2::origin(),
            total_mass: 0.0,
            boundary
        }
    }

    fn is_leaf(&self) -> bool {
        self.children.len() == 0
    }

    fn is_empty(&self) -> bool {
        self.total_mass == 0.0
    }

    fn subdivide(&mut self) {
        let min: Point2<f32> = self.boundary.min;
        let max: Point2<f32> = self.boundary.max;
        let ctr: Point2<f32> = nalgebra::center(&min, &max);
        self.children.push(Quadtree::new(Boundary::new(Point2::new(min.x, min.y), Point2::new(ctr.x, ctr.y))));
        self.children.push(Quadtree::new(Boundary::new(Point2::new(ctr.x, min.y), Point2::new(max.x, ctr.y))));
        self.children.push(Quadtree::new(Boundary::new(Point2::new(min.x, ctr.y), Point2::new(ctr.x, max.y))));
        self.children.push(Quadtree::new(Boundary::new(Point2::new(ctr.x, ctr.y), Point2::new(max.x, max.y))));
    }

    pub fn insert(&mut self, position: Point2<f32>, mass: f32) {

        if self.is_empty() {
            self.center_of_mass = position;
            self.total_mass = mass;

            return;
        }

        if self.is_leaf() && !self.is_empty() {
            if position == self.center_of_mass {
                panic!("Adding duplicate point results in infinite octree.")
            }

            self.subdivide();

            for child in self.children.iter_mut() {
                if child.boundary.contains(self.center_of_mass) {
                    child.insert(self.center_of_mass, self.total_mass);
                }
                if child.boundary.contains(position) {
                    child.insert(position, mass);
                }
            }
            
            self.center_of_mass = (self.center_of_mass * self.total_mass + Vector2::from(position.coords) * mass) / (self.total_mass + mass);
            self.total_mass += mass;

            return;
        }

        if !self.is_leaf() {
            self.children.iter_mut()
                .find(|child| {
                    child.boundary.contains(position)
                }).unwrap() // Error if somehow no child contains the position
                .insert(position, mass);

            self.center_of_mass = (self.center_of_mass * self.total_mass + Vector2::from(position.coords) * mass) / (self.total_mass + mass);
            self.total_mass += mass;

            return;
        }
    }

    pub fn calculate_acceleration(&self, position: Point2<f32>, theta: f32) -> Vector2<f32> {
        if (!self.is_empty() && self.is_leaf()) || self.boundary.size() < (position - self.center_of_mass) * theta {
            let m = self.total_mass;
            let d: Vector2<f32> = self.center_of_mass - position;
            let r = d.magnitude();

            if r < f32::EPSILON {
                return Vector2::zeros();
            }
            return (m / (r * r * r)) * d;
        }
        else if !self.is_leaf() {
            self.children.iter()
                .map(|child| {
                    child.calculate_acceleration(position, theta)
                }).sum()
        }
        else {
            return Vector2::zeros();
        }
    }
}

/// Space partitioning grid for collision detection
struct Grid {
    cells: Vec<Vec<usize>>,
    resolution: usize,
    boundary: Boundary,
}

impl Grid {
    fn new(min_cell_width: f32, boundary: Boundary) -> Self {
        let resolution = (boundary.width() / min_cell_width).floor() as usize;

        Self {
            cells: vec![Vec::new(); resolution * resolution],
            resolution,
            boundary,
        }
    }

    fn insert(&mut self, index: usize, position: Point2<f32>) {
        let pos = self.boundary.map_point(
            position, 
            &Boundary::new(
                Point2::new(0.0, 0.0), 
                Point2::new(self.resolution as f32, self.resolution as f32)
        ));
        let i = pos.x.floor() as usize;
        let j = pos.y.floor() as usize;

        self.cells[i + j * self.resolution].push(index);
    }

    fn get3x3(&self, x: usize, y: usize) -> Vec<usize> {
        let mut result = Vec::new();

        for i in -1..=1 {
            for j in -1..=1 {
                let x = (x as i32 + i) as usize;
                let y = (y as i32 + j) as usize;
                result.append(&mut (self.cells[x + y * self.resolution]).clone());
            } 
        }
        result
    }
}

pub struct World {
    pub bodies: Vec<Body>,
    largest_radius: f32,
}

impl World {
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            largest_radius: 0.0,
        }
    }

    pub fn insert(&mut self, body: Body) {
        self.largest_radius = self.largest_radius.max(body.radius); // TODO: Fix bug that appears if user edits radius of a body that is already in the world.
        self.bodies.push(body);
    }

    pub fn update_substeps(&mut self, dt: f32, substeps: u32) {
        for _ in 0..substeps {
            self.update(dt / substeps as f32);
        }
    }

    pub fn update(&mut self, dt: f32) {
        let boundary = self.calculate_boundary();
        let mut octree = Quadtree::new(boundary);

        self.bodies.iter().for_each(|body| {
            octree.insert(body.pos, body.mass);
        });

        self.bodies.iter_mut().for_each(|body| {
            let acc: Vector2<f32> = octree.calculate_acceleration(body.pos, 1.0);
            body.update(dt, acc);
        });

        let boundary = self.calculate_boundary().pad(self.largest_radius * 2.0);
        self.collide(boundary);
    }

    /// Always square
    pub fn calculate_boundary(&self) -> Boundary {
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;

        self.bodies.iter().for_each(|body| {
            min_x = min_x.min(body.pos.x);
            min_y = min_y.min(body.pos.y);
            max_x = max_x.max(body.pos.x);
            max_y = max_y.max(body.pos.y);
        });

        let size = (max_x - min_x).max(max_y - min_y);

        let x_error = (size - (max_x - min_x)) * 0.5;
        let y_error = (size - (max_y - min_y)) * 0.5;

        min_x -= x_error;
        min_y -= y_error;
        max_x += x_error;
        max_y += y_error;

        Boundary::new(Point2::new(min_x, min_y), Point2::new(max_x, max_y))
    }

    fn resolve(&mut self, i: usize, j: usize) {
        let p1 = &self.bodies[i];
        let p2 = &self.bodies[j];

        let difference: Vector2<f32> = p1.pos - p2.pos;
        let distance_sq = difference.norm_squared();

        let distance = distance_sq.sqrt();
        let normal: Vector2<f32> = difference / distance;

        let desired_distance = p1.radius + p2.radius;

        let v1: Vector2<f32> = p1.vel;
        let v2: Vector2<f32> = p2.vel;
        let m1 = p1.mass;
        let m2 = p2.mass;
        let t1 = 2.0 * m2 / (m1 + m2);
        let t2 = 2.0 * m1 / (m1 + m2);

        self.bodies[i].pos += normal * t1 * (desired_distance - distance) * 0.5;
        self.bodies[j].pos -= normal * t2 * (desired_distance - distance) * 0.5;
        self.bodies[i].vel -= t1 * ((v1 - v2).dot(&difference) / distance_sq) * difference;
        self.bodies[j].vel += t2 * ((v1 - v2).dot(&difference) / distance_sq) * difference;
    }

    fn collide(&mut self, boundary: Boundary) {
        let mut grid = Grid::new(self.largest_radius * 2.0, boundary);

        self.bodies.iter().enumerate().for_each(|(i, body)| {
            grid.insert(i, body.pos);
        });

        for x in 1..grid.resolution-1 {
            for y in 1..grid.resolution-1 {
                let indices = grid.get3x3(x, y);

                for i in 0..indices.len() {
                    let p0: Point2<f32> = self.bodies[indices[i]].pos;
                    let r0 = self.bodies[indices[i]].radius;

                    for j in 0..i  {
                        let p1: Point2<f32> = self.bodies[indices[j]].pos;
                        let r1 = self.bodies[indices[j]].radius;
                        if (p1 - p0).norm_squared() < (r0 + r1) * (r0 + r1) {
                            self.resolve(indices[i], indices[j]);
                        }
                    }
                }
            }
        }
    }

    /// Extremely inefficient (but accurate) calculation of the total energy in the system.
    pub fn total_energy(&self) -> f32 {
        let mut potential_energy = 0.0;
        let mut kinetic_energy = 0.0;
        for i in 0..self.bodies.len() {
            let mut potential = 0.0;
            for j in 0..i {
                potential -= self.bodies[j].mass / (self.bodies[j].pos - self.bodies[i].pos).magnitude();
            }
            potential_energy += potential * self.bodies[i].mass; 
            kinetic_energy += self.bodies[i].mass * self.bodies[i].vel.magnitude_squared() * 0.5;
        }

        potential_energy + kinetic_energy
    }
}