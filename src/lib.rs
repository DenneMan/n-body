use std::cmp::Ordering;
use std::f32::consts::PI;

use nalgebra::{Point3, Vector3};

struct Particle {
    pos: Point3<f32>,
    vel: Vector3<f32>,
    acc: Vector3<f32>,

    mass: f32,
    radius: f32,
}

impl Particle {
    fn new(pos: Point3<f32>, vel: Vector3<f32>, mass: f32, radius: f32) -> Self {
        Particle {
            pos,
            vel,
            acc: Vector3::new(0.0, 0.0, 0.0),

            mass,
            radius,
        }
    }

    fn update(&mut self, dt: f32) {
        let new_pos: Point3<f32> = self.pos + self.vel*dt + self.acc*(dt*dt*0.5);
        let new_acc: Vector3<f32> = self.apply_forces(); // only needed if acceleration is not constant
        let new_vel: Vector3<f32> = self.vel + (self.acc+new_acc)*(dt*0.5);
        self.pos = new_pos;
        self.vel = new_vel;
        self.acc = new_acc;
    }

    fn apply_forces(&mut self) -> Vector3<f32> {
        let grav_acc = Vector3::new(0.0, -9.81, 0.0); // 9.81 m/s² down in the z-axis
        return grav_acc;
    }
}

#[derive(Debug, Clone, Copy)]
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
        self.children.push(Octree::new(Boundary::new(Point3::new(ctr.x, min.y, ctr.z), Point3::new(max.x, ctr.y, max.z))));
        self.children.push(Octree::new(Boundary::new(Point3::new(min.x, ctr.y, ctr.z), Point3::new(ctr.x, max.y, max.z))));
        self.children.push(Octree::new(Boundary::new(Point3::new(ctr.x, ctr.y, ctr.z), Point3::new(max.x, max.y, max.z))));
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
}


struct ParticleSimulation {
    particles: Vec<Particle>,
    boundary: Boundary,
}

impl ParticleSimulation {
    fn new() -> Self {
        ParticleSimulation {
            particles: Vec::new(),
            boundary: Boundary::new(Point3::origin(), Point3::origin()),
        }
    }

    fn update(&mut self, dt: f32, substeps: u32) {
        let dt = dt / substeps as f32;
        for _ in 0..substeps {

            self.particles.iter_mut().for_each(|particle| {
                particle.update(dt);

                self.boundary.min.x = self.boundary.min.x.min(particle.pos.x);
                self.boundary.min.y = self.boundary.min.y.min(particle.pos.y);
                self.boundary.min.z = self.boundary.min.z.min(particle.pos.z);
                self.boundary.max.x = self.boundary.max.x.max(particle.pos.x);
                self.boundary.max.y = self.boundary.max.y.max(particle.pos.y);
                self.boundary.max.z = self.boundary.max.z.max(particle.pos.z);
            });

            //self.collide();
        }
    }

    fn resolve(&mut self, i: usize, j: usize) {
        let p1 = &self.particles[i];
        let p2 = &self.particles[j];

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

        self.particles[i].pos += normal * t1 * (distance_min - distance);
        self.particles[j].pos -= normal * t2 * (distance_min - distance);
        self.particles[i].vel -= (2.0 * t1 * ((v1 - v2).dot(&difference) / distance_sq) * difference) * 0.5;
        self.particles[j].vel += (2.0 * t2 * ((v1 - v2).dot(&difference) / distance_sq) * difference) * 0.5;
    }

    /* Does not work due to query requiering both radii
    fn collide(&mut self) {
        let mut quad_tree = QuadTree::new(self.boundary);
        self.particles.iter().enumerate().for_each(|(i, particle)| {
            quad_tree.insert(particle.pos, i);
        });

        for i in 0..self.particles.len() {
            let pos: Point3<f32> = self.particles[i].pos;
            let radius = self.particles[i].radius;

            let mut others = Vec::new();
            let query = Boundary::new(pos - Vector3::<f32>::new(radius * 2.0))
            quad_tree.query(Boundary::new(pos.x - RADIUS * 2.0, pos.y - RADIUS * 2.0, RADIUS * 4.0, RADIUS * 4.0), &mut others);


            for j in others {
                if i == j { continue; }
                if self.particles[j].pos == pos { continue; } 

                let diff: Vector3<f32> = self.particles[j].pos - pos;
                if diff.norm_squared() < (RADIUS + RADIUS) * (RADIUS + RADIUS) {
                    self.resolve(i, j);
                }
            }
        }
    } */
}

/* Old render system

struct MyWindow {
    sim: ParticleSimulation,
    boundary: Boundary,

    last_frame_end: Instant,
    time: f32,
}

impl MyWindow {
    fn new(x: f32, y: f32, w: f32, h: f32) -> Self {
        Self {
            sim: ParticleSimulation::new(),
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
            self.sim.particles.push(Particle {
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
        println!("Render: {:?}ms, Update: {:?}ms, n: {}", (begin_frame - self.last_frame_end).as_millis(), physics_time.as_millis(), self.sim.particles.len());

        { // Render: 
            graphics.clear_screen(Color::from_rgb(0.0, 0.0, 0.0));
            //let mut quad_tree = QuadTree::new(self.boundary);
            //self.particles.iter().for_each(|particle| {
            //    quad_tree.insert(particle.pos, 0);
            //});
            //quad_tree.render(helper, graphics, self.boundary.h);

            self.sim.particles.iter().for_each(|particle| {
                graphics.draw_circle((particle.pos.x, self.boundary.h - particle.pos.y), particle.radius, Color::WHITE);
            });

            helper.request_redraw();
        }

        let mut energy = 0.0;
        self.sim.particles.iter().for_each(|particle| {
            energy += particle.mass * 9.81 * particle.pos.y;
            energy += particle.mass * particle.vel.norm_squared() * 0.5
        });

        self.last_frame_end = Instant::now();
    }
} */