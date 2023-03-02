use std::cmp::Ordering;
use std::f32::consts::PI;

use nalgebra::{Point3, Vector3};

const GRAVITATIONAL_CONSTANT: f32 = 6.6743E-11;

#[derive(Default)]
pub struct Body {
    pub pos: Point3<f32>,
    pub vel: Vector3<f32>,
    pub acc: Vector3<f32>,

    pub mass: f32,
    pub radius: f32,
}

impl Body {
    pub fn new(pos: Point3<f32>, vel: Vector3<f32>, mass: f32, radius: f32) -> Self {
        Self {
            pos,
            vel,
            acc: Vector3::zeros(),

            mass,
            radius,
        }
    }

    pub fn update(&mut self, dt: f32, acc: Vector3<f32>) {
        let new_pos: Point3<f32> = self.pos + self.vel*dt + self.acc*(dt*dt*0.5);
        let new_acc: Vector3<f32> = acc;
        let new_vel: Vector3<f32> = self.vel + (self.acc+new_acc)*(dt*0.5);
        self.pos = new_pos;
        self.vel = new_vel;
        self.acc = new_acc;
    }
}

#[derive(Debug)]
pub struct Boundary {
    pub min: Point3<f32>,
    pub max: Point3<f32>,
}

impl Boundary {
    pub fn new(min: Point3<f32>, max: Point3<f32>) -> Self {
        Self {
            min, 
            max,
        }
    }

    pub fn contains(&self, position: Point3<f32>) -> bool {
        position >= self.min && position <= self.max
    }

    pub fn intersects(&self, other: Self) -> bool {
        self.min <= other.max && self.max >= other.min
    }
}

/* struct QuadTree {
    children: Option<Box<[QuadTree; 4]>>,
    positions: Vec<Point3<f32>>,
    data: Vec<usize>,
    boundary: Boundary
}

impl QuadTree {
    fn new(boundary: Boundary) -> Self {
        QuadTree { 
            children: None, 
            positions: Vec::new(),
            data: Vec::new(), 
            boundary,
        }
    }

    fn insert(&mut self, position: Point2<f32>, data: usize) { // fix issue when position already in tree
        if self.data.len() == 8 {
            self.split();
            let children = self.children.as_mut().unwrap();
            
            let x_split = self.boundary.x + self.boundary.w * 0.5;
            let y_split = self.boundary.y + self.boundary.h * 0.5;
            match (position.x.partial_cmp(&x_split).unwrap(), position.y.partial_cmp(&y_split).unwrap()) {
                (Ordering::Less,                      Ordering::Less                     ) => children[0].insert(position, data),
                (Ordering::Greater | Ordering::Equal, Ordering::Less                     ) => children[1].insert(position, data),
                (Ordering::Less,                      Ordering::Greater | Ordering::Equal) => children[2].insert(position, data),
                (Ordering::Greater | Ordering::Equal, Ordering::Greater | Ordering::Equal) => children[3].insert(position, data),
            }
            for i in 0..self.data.len() {
                match (self.positions[i].x.partial_cmp(&x_split).unwrap(), self.positions[i].y.partial_cmp(&y_split).unwrap()) {
                    (Ordering::Less,                      Ordering::Less                     ) => children[0].insert(self.positions[i], self.data[i]),
                    (Ordering::Greater | Ordering::Equal, Ordering::Less                     ) => children[1].insert(self.positions[i], self.data[i]),
                    (Ordering::Less,                      Ordering::Greater | Ordering::Equal) => children[2].insert(self.positions[i], self.data[i]),
                    (Ordering::Greater | Ordering::Equal, Ordering::Greater | Ordering::Equal) => children[3].insert(self.positions[i], self.data[i]),
                }
            }
            self.data.clear();
            self.positions.clear();
        } else if let Some(children) = self.children.as_mut() {
            let x_split = self.boundary.x + self.boundary.w * 0.5;
            let y_split = self.boundary.y + self.boundary.h * 0.5;
            match (position.x.partial_cmp(&x_split).unwrap(), position.y.partial_cmp(&y_split).unwrap()) {
                (Ordering::Less,                      Ordering::Less                     ) => children[0].insert(position, data),
                (Ordering::Greater | Ordering::Equal, Ordering::Less                     ) => children[1].insert(position, data),
                (Ordering::Less,                      Ordering::Greater | Ordering::Equal) => children[2].insert(position, data),
                (Ordering::Greater | Ordering::Equal, Ordering::Greater | Ordering::Equal) => children[3].insert(position, data),
            }
        } else {
            self.data.push(data);
            self.positions.push(position);
        }
    }

    fn split(&mut self) {
        let w = self.boundary.w * 0.5;
        let h = self.boundary.h * 0.5;
        self.children = Some(Box::new([
            QuadTree::new(Boundary::new(self.boundary.x, self.boundary.y, w, h)),
            QuadTree::new(Boundary::new(self.boundary.x + w, self.boundary.y, w, h)),
            QuadTree::new(Boundary::new(self.boundary.x, self.boundary.y + h, w, h)),
            QuadTree::new(Boundary::new(self.boundary.x + w, self.boundary.y + h, w, h)),
        ]))
    }

    fn query(&mut self, boundary: Boundary, others: &mut Vec<usize>) {
        if let Some(children) = self.children.as_mut() {
            for i in 0..4 {
                if children[i].boundary.intersects(boundary) {
                    children[i].query(boundary, others);
                }
            }
        } else {
            for i in 0..self.data.len() {
                if boundary.contains(self.positions[i]) {
                    others.push(self.data[i]);
                }
            }
        }
    }

    fn render(&self, helper: &mut WindowHelper, graphics: &mut Graphics2D, flip: f32) {
        graphics.draw_rectangle(
            Rectangle::new(
                (self.boundary.x, flip - (self.boundary.y + self.boundary.h)).into(), 
                (self.boundary.x + self.boundary.w, flip - (self.boundary.y)).into()
            ), 
            Color::from_gray(1.0)
        );
        graphics.draw_rectangle(
            Rectangle::new(
                (self.boundary.x + 1.0, flip - (self.boundary.y + self.boundary.h - 1.0)).into(), 
                (self.boundary.x + self.boundary.w - 1.0, flip - (self.boundary.y + 1.0)).into()
            ), 
            Color::from_gray(0.0)
        );
        
        if let Some(children) = self.children.as_ref() {
            for i in 0..4 {
                children[i].render(helper, graphics, flip);
            }
        }
    }
} */

// Barnes-Hut octree implementation
#[derive(Debug)]
pub struct Octree {
    children: Vec<Octree>,
    center_of_mass: Point3<f32>,
    total_mass: f32,
    boundary: Boundary,
}

impl Octree {
    pub fn new(boundary: Boundary) -> Self {
        Self {
            children: Vec::new(),
            center_of_mass: Point3::origin(),
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
        let min: Point3<f32> = self.boundary.min;
        let max: Point3<f32> = self.boundary.max;
        let ctr: Point3<f32> = nalgebra::center(&min, &max);
        self.children.push(Octree::new(Boundary::new(Point3::new(min.x, min.y, min.z), Point3::new(ctr.x, ctr.y, ctr.z))));
        self.children.push(Octree::new(Boundary::new(Point3::new(ctr.x, min.y, min.z), Point3::new(max.x, ctr.y, ctr.z))));
        self.children.push(Octree::new(Boundary::new(Point3::new(min.x, ctr.y, min.z), Point3::new(ctr.x, max.y, ctr.z))));
        self.children.push(Octree::new(Boundary::new(Point3::new(ctr.x, ctr.y, min.z), Point3::new(max.x, max.y, ctr.z))));
        self.children.push(Octree::new(Boundary::new(Point3::new(min.x, min.y, ctr.z), Point3::new(ctr.x, ctr.y, max.z))));
        self.children.push(Octree::new(Boundary::new(Point3::new(ctr.x, min.y, ctr.z), Point3::new(max.x, ctr.y, max.z))));
        self.children.push(Octree::new(Boundary::new(Point3::new(min.x, ctr.y, ctr.z), Point3::new(ctr.x, max.y, max.z))));
        self.children.push(Octree::new(Boundary::new(Point3::new(ctr.x, ctr.y, ctr.z), Point3::new(max.x, max.y, max.z))));
    }

    pub fn insert(&mut self, position: Point3<f32>, mass: f32) {

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
            
            self.center_of_mass = (self.center_of_mass * self.total_mass + Vector3::from(position.coords) * mass) / (self.total_mass + mass);
            self.total_mass += mass;

            return;
        }

        if !self.is_leaf() {
            self.children.iter_mut()
                .find(|child| {
                    child.boundary.contains(position)
                }).unwrap() // Error if somehow no child contains the position
                .insert(position, mass);

            self.center_of_mass = (self.center_of_mass * self.total_mass + Vector3::from(position.coords) * mass) / (self.total_mass + mass);
            self.total_mass += mass;

            return;
        }
    }

    pub fn calculate_acceleration(&self, position: Point3<f32>, theta: f32) -> Vector3<f32> {
        if (!self.is_empty() && self.is_leaf()) || (self.boundary.max - self.boundary.min) < (position - self.center_of_mass) * theta {
            let m = self.total_mass;
            let d: Vector3<f32> = self.center_of_mass - position;
            let r = d.magnitude();

            if r < 0.01 {
                return Vector3::zeros();
            }
            return (GRAVITATIONAL_CONSTANT * m / (r * r * r)) * d;
        }
        else if !self.is_leaf() {
            self.children.iter()
                .map(|child| {
                    child.calculate_acceleration(position, theta)
                }).sum()
        }
        else {
            return Vector3::zeros();
        }
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

    pub fn update_substeps(&mut self, dt: f32, substeps: u32) {
        let dt = dt / substeps as f32;
        for _ in 0..substeps {
            self.update(dt);
        }
    }

    pub fn update(&mut self, dt: f32) {
        let boundary = self.calculate_cubic_boundary();
        let mut octree = Octree::new(boundary);

        self.bodies.iter().for_each(|body| {
            octree.insert(body.pos, body.mass);
        });

        self.bodies.iter_mut().for_each(|body| {
            let acc: Vector3<f32> = octree.calculate_acceleration(body.pos, 1.0);
            body.update(dt, acc);
        });
    }

    pub fn calculate_cubic_boundary(&self) -> Boundary {
        let mut min_x = f32::MAX;
        let mut min_y = f32::MAX;
        let mut min_z = f32::MAX;
        let mut max_x = f32::MIN;
        let mut max_y = f32::MIN;
        let mut max_z = f32::MIN;

        self.bodies.iter().for_each(|body| {
            min_x = min_x.min(body.pos.x);
            min_y = min_y.min(body.pos.y);
            min_z = min_z.min(body.pos.z);
            max_x = max_x.max(body.pos.x);
            max_y = max_y.max(body.pos.y);
            max_z = max_z.max(body.pos.z);
        });

        let size = (max_x - min_x).max(max_y - min_y).max(max_z - min_z);

        let x_error = (size - (max_x - min_x)) * 0.5;
        let y_error = (size - (max_y - min_y)) * 0.5;
        let z_error = (size - (max_z - min_z)) * 0.5;

        min_x -= x_error;
        min_y -= y_error;
        min_z -= z_error;
        max_x += x_error;
        max_y += y_error;
        max_z += z_error;

        Boundary::new(Point3::new(min_x, min_y, min_z), Point3::new(max_x, max_y, max_z))
    }

    /* Not used due to comment bellow
    fn resolve(&mut self, i: usize, j: usize) {
        let p1 = &self.bodies[i];
        let p2 = &self.bodies[j];

        let difference: Vector3<f32> = p1.pos - p2.pos;
        let distance_sq = difference.norm_squared();

        let distance_min = p1.radius + p2.radius;

        let distance = distance_sq.sqrt();
        let normal: Vector3<f32> = difference / distance;

        let v1: Vector3<f32> = p1.vel;
        let v2: Vector3<f32> = p2.vel;
        let m1 = p1.mass;
        let m2 = p2.mass;
        let t1 = m2 / (m1 + m2);
        let t2 = m1 / (m1 + m2);

        self.bodies[i].pos += normal * t1 * (distance_min - distance);
        self.bodies[j].pos -= normal * t2 * (distance_min - distance);
        self.bodies[i].vel -= (2.0 * t1 * ((v1 - v2).dot(&difference) / distance_sq) * difference) * 0.5;
        self.bodies[j].vel += (2.0 * t2 * ((v1 - v2).dot(&difference) / distance_sq) * difference) * 0.5;
    } */

    /* Does not work due to query requiering both radii
    fn collide(&mut self) {
        let mut quad_tree = QuadTree::new(self.boundary);
        self.bodies.iter().enumerate().for_each(|(i, Body)| {
            quad_tree.insert(Body.pos, i);
        });

        for i in 0..self.bodies.len() {
            let pos: Point3<f32> = self.bodies[i].pos;
            let radius = self.bodies[i].radius;

            let mut others = Vec::new();
            let query = Boundary::new(pos - Vector3::<f32>::new(radius * 2.0))
            quad_tree.query(Boundary::new(pos.x - RADIUS * 2.0, pos.y - RADIUS * 2.0, RADIUS * 4.0, RADIUS * 4.0), &mut others);


            for j in others {
                if i == j { continue; }
                if self.bodies[j].pos == pos { continue; } 

                let diff: Vector3<f32> = self.bodies[j].pos - pos;
                if diff.norm_squared() < (RADIUS + RADIUS) * (RADIUS + RADIUS) {
                    self.resolve(i, j);
                }
            }
        }
    } */
}

/* Old render system

struct MyWindow {
    sim: BodySimulation,
    boundary: Boundary,

    last_frame_end: Instant,
    time: f32,
}

impl MyWindow {
    fn new(x: f32, y: f32, w: f32, h: f32) -> Self {
        Self {
            sim: BodySimulation::new(),
            boundary: Boundary::new(x, y, w, h),

            last_frame_end: Instant::now(),
            time: 0.0,
        }
    }
}

impl WindowHandler for MyWindow {
    fn on_draw(&mut self, helper: &mut WindowHelper, graphics: &mut Graphics2D) {
        let begin_frame = Instant::now();

        let dt = 0.01;

        self.time += dt;

        if self.time > 0.0 {
            self.time = 0.0;
            self.sim.bodies.push(Body {
                pos: Point3::new(25.0, self.boundary.h - 25.0, 0.0),
                vel: Vector3::new(75.0, 0.0, 0.0),
                acc: Vector3::zeros(),
                mass: PI * RADIUS * RADIUS,
                drag: 0.0,
                radius: RADIUS,
            });
        }

        let now = Instant::now();
        self.sim.update(dt, 8);
        let physics_time = now.elapsed();
        println!("Render: {:?}ms, Update: {:?}ms, n: {}", (begin_frame - self.last_frame_end).as_millis(), physics_time.as_millis(), self.sim.bodies.len());

        { // Render: 
            graphics.clear_screen(Color::from_rgb(0.0, 0.0, 0.0));
            //let mut quad_tree = QuadTree::new(self.boundary);
            //self.bodies.iter().for_each(|Body| {
            //    quad_tree.insert(Body.pos, 0);
            //});
            //quad_tree.render(helper, graphics, self.boundary.h);

            self.sim.bodies.iter().for_each(|Body| {
                graphics.draw_circle((Body.pos.x, self.boundary.h - Body.pos.y), Body.radius, Color::WHITE);
            });

            helper.request_redraw();
        }

        let mut energy = 0.0;
        self.sim.bodies.iter().for_each(|Body| {
            energy += Body.mass * 9.81 * Body.pos.y;
            energy += Body.mass * Body.vel.norm_squared() * 0.5
        });

        self.last_frame_end = Instant::now();
    }
} */