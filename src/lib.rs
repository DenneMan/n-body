use nalgebra::{Point2, Vector2};

trait InvSqrt {
    fn inv_sqrt(self) -> Self;
}

impl InvSqrt for f32 {
    fn inv_sqrt(self) -> Self {
        use std::mem::transmute;

		const THREEHALFS: f32 = 1.5f32;
		let x2: f32 = self * 0.5f32;
        
        // evil floating point bit level hacking
		let mut i: u32 = unsafe { transmute(self) };

        // what the fuck?
		i = 0x5f375a86 - (i >> 1);
		let mut y: f32 = unsafe { transmute(i) };

        // 1st iteration
		y = y * ( THREEHALFS - ( x2 * y * y ) );

        // 2nd iteration, this can be removed
//      y = y * ( THREEHALFS - ( x2 * y * y ) );     

		return y;
    }
}

impl InvSqrt for f64 {

    fn inv_sqrt(self) -> Self {
        use std::mem::transmute;
        
		const THREEHALFS: f64 = 1.5;
		let x2 = self * 0.5;
        
        // evil floating point bit level hacking
        let mut i: u64 = unsafe { transmute(self) };

        // what the fuck?
		i = 0x5fe6ec85e7de30da - (i >> 1);
		let mut y: f64 = unsafe { transmute(i) };

        // 1st iteration
        y = y * ( THREEHALFS - ( x2 * y * y ) );

        // 2nd iteration, this can be removed
//      y = y * ( THREEHALFS - ( x2 * y * y ) );

        return y;
    }
}

#[derive(Default)]
pub struct Body {
    pub pos: Point2<f64>,
    pub vel: Vector2<f64>,
    pub acc: Vector2<f64>,

    pub mass: f64,
    pub radius: f64,
}

impl Body {
    pub fn new(pos: Point2<f64>, vel: Vector2<f64>, mass: f64, radius: f64) -> Self {
        Self {
            pos,
            vel,
            acc: Vector2::zeros(),

            mass,
            radius,
        }
    }

    pub fn update(&mut self, dt: f64, acc: Vector2<f64>) {
        let new_pos: Point2<f64> = self.pos + self.vel*dt + self.acc*(dt*dt*0.5);
        let new_acc: Vector2<f64> = acc;
        let new_vel: Vector2<f64> = self.vel + (self.acc+new_acc)*(dt*0.5);
        self.pos = new_pos;
        self.vel = new_vel;
        self.acc = new_acc;
    }
}

#[derive(Clone, Debug)]
pub struct Boundary {
    pub min: Point2<f64>,
    pub max: Point2<f64>,
}

impl Boundary {
    pub const fn new(min: Point2<f64>, max: Point2<f64>) -> Self {
        Self {
            min, 
            max,
        }
    }

    pub fn contains(&self, position: Point2<f64>) -> bool {
        position >= self.min && position <= self.max
    }

    pub fn intersects(&self, other: &Self) -> bool {
        self.min <= other.max && self.max >= other.min
    }

    pub fn width(&self) -> f64 {
        self.max.x - self.min.x
    }

    pub fn height(&self) -> f64 {
        self.max.y - self.min.y
    }

    pub fn size(&self) -> Vector2<f64> {
        self.max - self.min
    }

    pub fn map(&self, point: Point2<f64>, to: &Self) -> Point2<f64> {
        to.min + (point - self.min).component_div(&self.size()).component_mul(&to.size())
    }

    pub fn map_distance(&self, distance: f64, to: &Self) -> f64 {
        distance / self.width() * to.width()
    }

    pub fn pad(&self, pad: f64) -> Self {
        let mut result = self.clone();

        result.min.x -= pad;
        result.min.y -= pad;
        result.max.x += pad;
        result.max.y += pad;

        result
    }
}

/// Barnes-Hut quadtree implementation
#[derive(Debug)]
pub struct Quadtree {
    pub children: Vec<Quadtree>,
    pub center_of_mass: Point2<f64>,
    pub total_mass: f64,
    pub boundary: Boundary,
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

    pub fn is_leaf(&self) -> bool {
        self.children.len() == 0
    }

    pub fn is_empty(&self) -> bool {
        self.total_mass == 0.0
    }

    fn subdivide(&mut self) {
        let min: Point2<f64> = self.boundary.min;
        let max: Point2<f64> = self.boundary.max;
        let ctr: Point2<f64> = nalgebra::center(&min, &max);
        self.children.push(Quadtree::new(Boundary::new(Point2::new(min.x, min.y), Point2::new(ctr.x, ctr.y))));
        self.children.push(Quadtree::new(Boundary::new(Point2::new(ctr.x, min.y), Point2::new(max.x, ctr.y))));
        self.children.push(Quadtree::new(Boundary::new(Point2::new(min.x, ctr.y), Point2::new(ctr.x, max.y))));
        self.children.push(Quadtree::new(Boundary::new(Point2::new(ctr.x, ctr.y), Point2::new(max.x, max.y))));
    }

    pub fn insert(&mut self, position: Point2<f64>, mass: f64) {

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

    pub fn calculate_acceleration_recursive(&self, position: Point2<f64>, theta: f64) -> Vector2<f64> {
        if self.is_empty() || position == self.center_of_mass {
            return Vector2::zeros();
        }

        if self.is_leaf() || self.boundary.size() < (position - self.center_of_mass).abs() * theta {
            let m = self.total_mass;
            let d: Vector2<f64> = self.center_of_mass - position;
            let r = d.magnitude_squared().inv_sqrt();

            return (m * r * r * r) * d;
        } else {
            return self.children.iter()
                .map(|child| {
                    child.calculate_acceleration_recursive(position, theta)
                }).sum();
        }
    }

    pub fn calculate_acceleration(&self, position: Point2<f64>, theta: f64) -> Vector2<f64> {
        let mut stack = vec![self];

        let mut sum: Vector2<f64> = Vector2::zeros();

        loop {
            if let Some(current) = stack.pop() {
                if current.is_leaf() || current.boundary.size() < (position - current.center_of_mass).abs() * theta {
                    let m = current.total_mass;
                    let d: Vector2<f64> = current.center_of_mass - position;
                    let r = d.magnitude().recip();
        
                    sum += (m * r * r * r) * d;
                } else {
                    for child in current.children.iter() {
                        if !child.is_empty() && position != child.center_of_mass {
                            stack.push(child);
                        }
                    }
                }
            }
            else {
                return sum;
            }
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
    fn new(min_cell_width: f64, boundary: Boundary) -> Self {
        let resolution = (boundary.width() / min_cell_width).floor() as usize;

        Self {
            cells: vec![Vec::new(); resolution * resolution],
            resolution,
            boundary,
        }
    }

    fn insert(&mut self, index: usize, position: Point2<f64>) {
        let pos: Point2<f64> = self.boundary.map(
            position, 
            &Boundary::new(
                Point2::new(0.0, 0.0), 
                Point2::new(self.resolution as f64, self.resolution as f64)
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
}

impl World {
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
        }
    }

    pub fn insert(&mut self, body: Body) {
        self.bodies.push(body);
    }

    pub fn update_substeps(&mut self, dt: f64, substeps: u32) {
        for _ in 0..substeps {
            self.update(dt / substeps as f64);
        }
    }

    pub fn update(&mut self, dt: f64) {
        if self.bodies.len() == 0 {
            return;
        }

        let boundary = self.calculate_boundary();
        let mut quadtree = Quadtree::new(boundary);

        self.bodies.iter().for_each(|body| {
            quadtree.insert(body.pos, body.mass);
        });

        self.bodies.iter_mut().for_each(|body| {
            let acc: Vector2<f64> = quadtree.calculate_acceleration(body.pos, 1.0);
            body.update(dt, acc);
        });

        self.collide();
    }

    /// Always square
    pub fn calculate_boundary(&self) -> Boundary {
        let mut min_x = f64::MAX;
        let mut min_y = f64::MAX;
        let mut max_x = f64::MIN;
        let mut max_y = f64::MIN;

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

    pub fn find_largest_radius(&self) -> Option<f64> {
        self.bodies.iter().map(|body| body.radius).max_by(|rhs, lhs| rhs.total_cmp(lhs))
    }

    fn resolve(&mut self, i: usize, j: usize) {
        let p1 = &self.bodies[i];
        let p2 = &self.bodies[j];

        let difference: Vector2<f64> = p1.pos - p2.pos;
        let distance_sq = difference.norm_squared();

        let distance = distance_sq.sqrt();
        let normal: Vector2<f64> = difference / distance;

        let desired_distance = p1.radius + p2.radius;

        let v1: Vector2<f64> = p1.vel;
        let v2: Vector2<f64> = p2.vel;
        let m1 = p1.mass;
        let m2 = p2.mass;
        let t1 = 2.0 * m2 / (m1 + m2);
        let t2 = 2.0 * m1 / (m1 + m2);

        self.bodies[i].pos += normal * t1 * (desired_distance - distance) * 0.5;
        self.bodies[j].pos -= normal * t2 * (desired_distance - distance) * 0.5;

        self.bodies[i].vel -= t1 * ((v1 - v2).dot(&difference) / distance_sq) * difference * 0.5; // 50% velocity loss
        self.bodies[j].vel += t2 * ((v1 - v2).dot(&difference) / distance_sq) * difference * 0.5; // 50% velocity loss
    }

    /// Will panic if there are no bodies.
    fn collide(&mut self) {
        let r = self.find_largest_radius().unwrap();
        let boundary = self.calculate_boundary().pad(r * 2.0);

        let mut grid = Grid::new(r * 4.0, boundary);

        self.bodies.iter().enumerate().for_each(|(i, body)| {
            grid.insert(i, body.pos);
        });

        for x in 1..grid.resolution-1 {
            for y in 1..grid.resolution-1 {
                let indices = grid.get3x3(x, y);

                for i in 0..indices.len() {
                    let p0: Point2<f64> = self.bodies[indices[i]].pos;
                    let r0 = self.bodies[indices[i]].radius;

                    for j in 0..i  {
                        let p1: Point2<f64> = self.bodies[indices[j]].pos;
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
    pub fn total_energy(&self) -> f64 {
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