use std::cmp::Ordering;
use std::f32::consts::PI;
use std::mem::MaybeUninit;

use rand::rngs::ThreadRng;
use speedy2d::color::Color;
use speedy2d::shape::Rectangle;
use speedy2d::{Graphics2D, Window};
use speedy2d::window::{WindowHandler, WindowHelper, WindowCreationOptions, WindowSize, WindowPosition};

use rand::{self, Rng};

use nalgebra::{Point2, Vector2};

use std::time::{Instant, Duration};

fn main() {
    const WIDTH: u32 = 1024;
    const HEIGHT: u32 = 1024;

    let window = Window::new_with_options(
        "Wow", 
        WindowCreationOptions::new_windowed(
            WindowSize::PhysicalPixels((WIDTH, HEIGHT).into()), 
            Some(WindowPosition::Center)
        ).with_resizable(false)).unwrap();
    window.run_loop(ParticleSimulation::new(0.0, 0.0, WIDTH as f32, HEIGHT as f32));
}

struct Particle {
    pos: Point2<f32>,
    vel: Vector2<f32>,
    acc: Vector2<f32>,

    mass: f32,
    drag: f32,

    radius: f32,
}

impl Particle {
    fn new(x: f32, y: f32) -> Self {
        Particle {
            pos: Point2::new(x, y),
            vel: Vector2::new(0.0, 0.0),
            acc: Vector2::new(0.0, 0.0),

            mass: 1.0,
            drag: 0.1,

            radius: 50.0,
        }
    }

    fn update(&mut self, dt: f32) {
        let new_pos: Point2<f32> = self.pos + self.vel*dt + self.acc*(dt*dt*0.5);
        let new_acc: Vector2<f32> = self.apply_forces(); // only needed if acceleration is not constant
        let new_vel: Vector2<f32> = self.vel + (self.acc+new_acc)*(dt*0.5);
        self.pos = new_pos;
        self.vel = new_vel;
        self.acc = new_acc;
    }

    fn apply_forces(&mut self) -> Vector2<f32> {
        let grav_acc = Vector2::new(0.0, -9.81); // 9.81 m/s² down in the z-axis
        let drag_force: Vector2<f32> = 0.5 * self.drag * self.vel.component_mul(&self.vel); // D = 0.5 * (rho * C * Area * vel^2)
        let drag_acc: Vector2<f32> = drag_force / self.mass; // a = F/m
        return grav_acc - drag_acc;
    }
}

#[derive(Clone, Copy)]
struct Boundary {
    x: f32,
    y: f32,
    w: f32,
    h: f32,
}

impl Boundary {
    fn new(x: f32, y: f32, w: f32, h: f32) -> Self {
        Self {
            x,
            y,
            w,
            h,
        }
    }

    fn contains(&self, position: Point2<f32>) -> bool {
        position.x >= self.x && position.y >= self.y && position.x < self.x + self.w && position.y < self.y + self.h
    }

    fn intersects(&self, other: Self) -> bool {
        !(self.x >= other.x + other.w || self.x + self.w < other.x || self.y >= other.y + other.h || self.y + self.h < other.y)
    }
}

struct QuadTree<T>
    where T: Copy {
    children: Option<Box<[QuadTree<T>; 4]>>,
    positions: Vec<Point2<f32>>,
    data: Vec<T>,
    boundary: Boundary
}

impl<T> QuadTree<T>
where T: Copy {
    fn new(boundary: Boundary) -> Self {
        QuadTree { 
            children: None, 
            positions: Vec::new(),
            data: Vec::new(), 
            boundary,
        }
    }

    fn insert(&mut self, position: Point2<f32>, data: T) { // fix issue when position already in tree
        if self.data.len() == 16 {
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

    fn query(&mut self, boundary: Boundary, others: &mut Vec<T>) {
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
}


struct ParticleSimulation {
    particles: Vec<Particle>,
    boundary: Boundary,
    rng: ThreadRng,
}

impl ParticleSimulation {
    fn new(x: f32, y: f32, w: f32, h: f32) -> Self {
        ParticleSimulation {
            particles: Vec::new(),
            boundary: Boundary::new(x, y, w, h),
            rng: rand::thread_rng(),
        }
    }

    fn update(&mut self, dt: f32, substeps: u32) {
        let dt = dt / substeps as f32;
        for _ in 0..substeps {
            self.particles.iter_mut().for_each(|particle| {
                particle.update(dt);
            });
    
            self.bound();

            let mut quad_tree = QuadTree::new(self.boundary);
            self.particles.iter().enumerate().for_each(|(i, particle)| {
                quad_tree.insert(particle.pos, i);
            });

            for i in 0..self.particles.len() {
                let pos = self.particles[i].pos;
                let radius = self.particles[i].radius;

                let mut others = Vec::new();
                quad_tree.query(Boundary::new(pos.x - radius * 2.0, pos.y - radius * 2.0, radius * 4.0, radius * 4.0), &mut others);


                for j in others {
                    if i == j { continue; }

                    let diff: Vector2<f32> = self.particles[j].pos - pos;
                    if diff.norm_squared() < (radius + radius) * (radius + radius) {
                        self.resolve(i, j);
                    } 
                }
            }
        }
    }

    fn resolve(&mut self, i: usize, j: usize) {
        let p1 = &self.particles[i];
        let p2 = &self.particles[j];

        let difference: Vector2<f32> = self.particles[i].pos - self.particles[j].pos;
        let distance_sq = difference.norm_squared();

        let distance_min = self.particles[j].radius + self.particles[i].radius;

        let distance = distance_sq.sqrt();
        let normal: Vector2<f32> = difference / distance;

        let v1: Vector2<f32> = p1.vel;
        let v2: Vector2<f32> = p2.vel;
        let m1 = p1.mass;
        let m2 = p2.mass;
        let t1 = m2 / (m1 + m2);
        let t2 = m1 / (m1 + m2);

        self.particles[i].pos += normal * t1 * (distance_min - distance);
        self.particles[j].pos -= normal * t2 * (distance_min - distance);
        self.particles[i].vel -= (2.0 * t1 * ((v1 - v2).dot(&difference) / distance_sq) * difference) * 0.5;
        self.particles[j].vel += (2.0 * t2 * ((v1 - v2).dot(&difference) / distance_sq) * difference) * 0.5;
    }

    fn render(&mut self, helper: &mut WindowHelper, graphics: &mut Graphics2D) {
        graphics.clear_screen(Color::from_rgb(0.0, 0.0, 0.0));
        let mut quad_tree = QuadTree::new(self.boundary);
        self.particles.iter().for_each(|particle| {
            quad_tree.insert(particle.pos, 0);
        });
        quad_tree.render(helper, graphics, self.boundary.h);

        self.particles.iter().for_each(|particle| {
            graphics.draw_circle((particle.pos.x, self.boundary.h - particle.pos.y), particle.radius, Color::WHITE);
        });

        helper.request_redraw();
    }

    fn bound(&mut self) {
        self.particles.iter_mut().for_each(|particle| {
            if particle.pos.x < self.boundary.x + particle.radius {
                particle.pos.x = self.boundary.x + particle.radius;
                particle.vel.x *= -0.5;
            }
            if particle.pos.x > self.boundary.x + self.boundary.w - particle.radius {
                particle.pos.x = self.boundary.x + self.boundary.w - particle.radius;
                particle.vel.x *= -0.5;
            }
            if particle.pos.y < self.boundary.y + particle.radius {
                particle.pos.y = self.boundary.y + particle.radius;
                particle.vel.y *= -0.5;
            }
            if particle.pos.y > self.boundary.y + self.boundary.h - particle.radius {
                particle.pos.y = self.boundary.y + self.boundary.h - particle.radius;
                particle.vel.y *= -0.5;
            }
        })
    }

    fn collide(&mut self) {
        for i in 0..self.particles.len() {
            for j in 0..i {
                let difference: Vector2<f32> = self.particles[i].pos - self.particles[j].pos;
                let distance_sq = difference.norm_squared();
                let distance_min = self.particles[j].radius + self.particles[i].radius;
                if distance_sq < distance_min * distance_min {
                    self.resolve(i, j);
                }
            }
        }
    }
}

impl WindowHandler for ParticleSimulation {
    fn on_draw(&mut self, helper: &mut WindowHelper, graphics: &mut Graphics2D) {

        let dt = 0.01;
        let radius = 2.0;
        self.particles.push(Particle {
            pos: Point2::new(25.0, self.boundary.h - 25.0),
            vel: Vector2::new(75.0, 0.0),
            acc: Vector2::zeros(),
            mass: PI * radius * radius,
            drag: 0.0,
            radius,
        });

        let now = Instant::now();
        self.update(dt, 8);
        let physics_time = now.elapsed();
        println!("Physics time is {:?} at {} particles", physics_time, self.particles.len());

        self.render(helper, graphics);

        let mut energy = 0.0;
        self.particles.iter().for_each(|particle| {
            energy += particle.mass * 9.81 * particle.pos.y;
            energy += particle.mass * particle.vel.norm_squared() * 0.5
        });
    }
}