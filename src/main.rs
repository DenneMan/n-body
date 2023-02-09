use speedy2d::color::Color;
use speedy2d::{Graphics2D, Window};
use speedy2d::window::{WindowHandler, WindowHelper};

use rand::{self, Rng};

use nalgebra::{Point2, Vector2, distance_squared};

fn main() {
    let window = Window::new_fullscreen_borderless("Wow").unwrap();
    window.run_loop(ParticleSimulation::new());
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

            radius: 5.0,
        }
    }

    fn update(&mut self, dt: f32) {
        let new_pos = self.pos + self.vel*dt + self.acc*(dt*dt*0.5);
        let new_acc = self.apply_forces(); // only needed if acceleration is not constant
        let new_vel = self.vel + (self.acc+new_acc)*(dt*0.5);
        self.pos = new_pos;
        self.vel = new_vel;
        self.acc = new_acc;
    }

    fn apply_forces(&mut self) -> Vector2<f32> {
        let grav_acc = Vector2::new(0.0, -9.81); // 9.81 m/s² down in the z-axis
        //let drag_force: Vec2 = 0.5 * self.drag * Vec2::dot(self.vel, self.vel); // D = 0.5 * (rho * C * Area * vel^2)
        //let drag_acc: Vec2 = drag_force / self.mass; // a = F/m
        return grav_acc;
    }
}

struct ParticleSimulation {
    yoff: f32,
    particles: Vec<Particle>,
}

impl ParticleSimulation {
    fn new() -> Self {
        ParticleSimulation {
            yoff: 0.0,
            particles: Vec::new(),
        }
    }

    fn bound(&mut self) {
        self.particles.iter_mut().for_each(|particle| {
            if particle.pos.y - particle.radius < 0.0 {
                particle.pos.y = particle.radius;
            }
        })
    }

    fn collide(&mut self, dt: f32) {
        for i in 0..self.particles.len() {
            for j in i..self.particles.len() {
                let p1 = &self.particles[i];
                let p2 = &self.particles[j];

                let difference = p1.pos - p2.pos;
                let distance_sq = distance_squared(&p1.pos, &p2.pos);
                let min_distance
                if  < (p1.radius + p2.radius) * (p1.radius + p2.radius) {

                }
            }
        }
    }
}

impl WindowHandler for ParticleSimulation {
    fn on_start(
            &mut self,
            helper: &mut WindowHelper<()>,
            info: speedy2d::window::WindowStartupInfo
        ) {
        self.yoff = info.viewport_size_pixels().y as f32;

        let mut rng = rand::thread_rng();

        for _ in 0..100 {
            let x = rng.gen::<f32>() * info.viewport_size_pixels().x as f32;
            let y = rng.gen::<f32>() * info.viewport_size_pixels().y as f32;
            self.particles.push(Particle::new(x, y));
        }
    }

    fn on_draw(&mut self, helper: &mut WindowHelper, graphics: &mut Graphics2D) {
        graphics.clear_screen(Color::from_rgb(0.0, 0.0, 0.0));

        self.particles.iter_mut().for_each(|particle| {
            particle.update(0.1);
        });

        self.bound();

        self.particles.iter().for_each(|particle| {
            graphics.draw_circle((particle.pos.x, self.yoff - particle.pos.y), 5.0, Color::WHITE);
        });
        
        helper.request_redraw();
    }
}