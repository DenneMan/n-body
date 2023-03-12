use nalgebra::{Point2, Vector2};

use crate::physics::{
    boundary::Boundary,
    quadtree::Quadtree,
    body::Body,
    grid::Grid,
};

pub struct World {
    pub bodies: Vec<Body>,
    pub theta: f32,
}

impl World {
    /// Theta is used for the barnes-hut algorithm. The larger theta is, the more accuracy is traded for performance.
    pub fn new(theta: f32) -> Self {
        Self {
            bodies: Vec::new(),
            theta,
        }
    }

    pub fn insert(&mut self, body: Body) {
        self.bodies.push(body);
    }

    pub fn update(&mut self, dt: f32, substeps: u32) {
        puffin::profile_function!();
        for _ in 0..substeps {
            self.substep(dt / substeps as f32);
        }
    }

    pub fn substep(&mut self, dt: f32) {
        puffin::profile_function!();
        
        if self.bodies.len() == 0 {
            return;
        }
        self.integrate(dt);

        self.collide();
    }

    pub fn integrate(&mut self, dt: f32) {
        puffin::profile_function!();

        let quadtree = self.calculate_quadtree();

        self.bodies.iter_mut().for_each(|body| {
            let acc: Vector2<f32> = quadtree.calculate_acceleration(body.pos, self.theta);
            body.update(dt, acc);
        });
    }

    pub fn calculate_quadtree(&self) -> Quadtree {
        puffin::profile_function!();

        let boundary = self.calculate_boundary();
        let mut quadtree = Quadtree::new(boundary);

        self.bodies.iter().for_each(|body| {
            quadtree.insert(body.pos, body.mass);
        });

        quadtree
    }

    /// Always square
    pub fn calculate_boundary(&self) -> Boundary {
        puffin::profile_function!();

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

    pub fn find_largest_radius(&self) -> Option<f32> {
        puffin::profile_function!();

        self.bodies.iter().map(|body| body.radius).max_by(|rhs, lhs| rhs.total_cmp(lhs))
    }

    fn resolve(&mut self, i: usize, j: usize) {
        puffin::profile_function!();

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

        self.bodies[i].vel -= t1 * ((v1 - v2).dot(&difference) / distance_sq) * difference * 0.5; // 50% velocity loss
        self.bodies[j].vel += t2 * ((v1 - v2).dot(&difference) / distance_sq) * difference * 0.5; // 50% velocity loss
    }

    /// Will panic if there are no bodies.
    fn collide(&mut self) {
        puffin::profile_function!();

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