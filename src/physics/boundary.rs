use nalgebra::{Point2, Vector2};

#[derive(Clone, Copy, Debug)]
pub struct Boundary {
    pub min: Point2<f32>,
    pub max: Point2<f32>,
}

impl Boundary {
    pub const fn new(min: Point2<f32>, max: Point2<f32>) -> Self {
        Self {
            min, 
            max,
        }
    }

    pub fn contains(&self, position: Point2<f32>) -> bool {
        position >= self.min && position <= self.max
    }

    pub fn intersects(&self, other: &Self) -> bool {
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

    pub fn area(&self) -> f32 {
        self.width() * self.height()
    }

    pub fn map(&self, point: Point2<f32>, to: &Self) -> Point2<f32> {
        to.min + (point - self.min).component_div(&self.size()).component_mul(&to.size())
    }

    pub fn pad(&self, pad: f32) -> Self {
        let mut result = self.clone();

        result.min.x -= pad;
        result.min.y -= pad;
        result.max.x += pad;
        result.max.y += pad;

        result
    }
}