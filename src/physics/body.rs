use nalgebra::{Point2, Vector2};

/// A velocity-verlet body with a mass and a radius.
#[derive(Default)]
pub struct Body {
    pub pos: Vector2<f32>,
    pub vel: Vector2<f32>,
    pub acc: Vector2<f32>,

    pub mass: f32,
    pub radius: f32,
}

impl Body {
    pub fn new(pos: Vector2<f32>, vel: Vector2<f32>, mass: f32, radius: f32) -> Self {
        Self {
            pos,
            vel,
            acc: Vector2::zeros(),

            mass,
            radius,
        }
    }

    pub fn update(&mut self, dt: f32, acc: Vector2<f32>) {
        let new_pos: Vector2<f32> = self.pos + self.vel*dt + self.acc*(dt*dt*0.5);
        let new_acc: Vector2<f32> = acc;
        let new_vel: Vector2<f32> = self.vel + (self.acc+new_acc)*(dt*0.5);
        self.pos = new_pos;
        self.vel = new_vel;
        self.acc = new_acc;
    }
}