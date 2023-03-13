use nalgebra::Vector2;

use crate::physics::{
    inv_sqrt::InvSqrt
};

use super::body::Body;

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum Quadrant {
    NW = 0, // North-east
    NE = 1, // North-west
    SW = 2, // South-west
    SE = 3, // South-east
}

impl From<u8> for Quadrant {
    fn from(value: u8) -> Self {
        unsafe { std::mem::transmute(value) }
    }
}

impl From<usize> for Quadrant {
    fn from(value: usize) -> Self {
        unsafe { std::mem::transmute(value as u8) }
    }
}

#[derive(Clone)]
pub struct QuadBoundary {
    pub center: Vector2<f32>,
    pub width: f32,
}

impl QuadBoundary {
    pub fn new(center: Vector2<f32>, width: f32) -> Self {
        Self {
            center,
            width, // Distance from center to edge
        }
    }

    pub fn contains(&self, point: Vector2<f32>) -> bool {
        !(point.x < self.center.x - self.width || point.x > self.center.x + self.width || point.y < self.center.y - self.width || point.y > self.center.y + self.width)
    }

    pub fn quadrant(&self, point: Vector2<f32>) -> Quadrant {
        let bits = (((point.y < self.center.y) as u8) << 1) | (point.x >= self.center.x) as u8;
        bits.into()
    }

    pub fn become_quadrant(mut self, quadrant: Quadrant) -> Self {
        let bits = quadrant as u8;
        self.width *= 0.5;

        self.center.x += -self.width + self.width * ((bits & 0b1) << 1) as f32;
        self.center.y +=  self.width - self.width * (   bits & 0b10   ) as f32;

        self
    }
}

#[derive(Clone)]
pub struct BarnesHutData {
    /// The center of mass for a branch
    pub pos: Vector2<f32>,
    /// The total mass for a branch
    pub mass: f32,
}

impl BarnesHutData {
    pub fn new(pos: Vector2<f32>, mass: f32) -> Self {
        Self {
            pos,
            mass,
        }
    }
}

#[derive(Clone)]
pub struct Quadnode { // TODO: Change to u16:s and bitshift by 2 to save 2 bits.
    /// Index to the first child, the other three are contiguous
    pub child: usize, // MAX if no children
    pub data: usize, // MAX if no data
}

impl Default for Quadnode {
    fn default() -> Self {
        Self {
            child: usize::MAX,
            data: usize::MAX,
        }
    }
}

impl Quadnode {
    pub fn get_child(&self) -> Option<usize> {
        if self.is_leaf() {
            return None;
        }
        Some(self.child)
    }

    pub fn get_data(&self) -> Option<usize> {
        if self.is_empty() {
            return None;
        }
        Some(self.data)
    }

    pub fn is_leaf(&self) -> bool {
        self.child == usize::MAX
    }

    pub fn is_empty(&self) -> bool {
        self.data == usize::MAX
    }
}

/// Barnes-Hut quadtree implementation
pub struct Quadtree {
    pub nodes: Vec<Quadnode>,
    pub data: Vec<BarnesHutData>,
    pub boundary: QuadBoundary,
}

impl Quadtree {
    pub fn new(boundary: QuadBoundary) -> Self {
        Self {
            nodes: vec![Quadnode::default(); 4],
            data: Vec::new(),
            boundary,
        }
    }

    fn subdivide(&mut self, node: usize) {
        self.nodes[node].child = self.nodes.len();
        self.nodes.push(Quadnode::default());
        self.nodes.push(Quadnode::default());
        self.nodes.push(Quadnode::default());
        self.nodes.push(Quadnode::default());
    }
    
    pub fn insert(&mut self, bodies: &[Body]) { // TODO: try not dividing by mass and then do a final pass over all data in the end. 
        puffin::profile_function!();

        for body in bodies {

            let mut boundary = self.boundary.clone();

            let quadrant = boundary.quadrant(body.pos);
            let mut node = quadrant as usize;
            boundary = boundary.become_quadrant(quadrant);

            loop {
                // Node is a branch
                if let Some(child) = self.nodes[node].get_child() {
                    // Update current node with body
                    let data = &mut self.data[self.nodes[node].data];
                    data.pos = (data.pos * data.mass + body.pos * body.mass) / (data.mass + body.mass);
                    data.mass += body.mass;

                    let quadrant = boundary.quadrant(body.pos);
                    boundary = boundary.become_quadrant(quadrant);
                    node = child + quadrant as usize;
                }
                // Node is a leaf with data
                else if let Some(data) = self.nodes[node].get_data() {
                    // Subdivide
                    let child = self.nodes.len();
                    self.subdivide(node);

                    // Clone data to containing child node
                    let child_data = self.data.len();
                    self.data.push(self.data[data].clone());
                    let quadrant = boundary.quadrant(self.data[data].pos);
                    self.nodes[child + quadrant as usize].data = child_data;

                    // Update current node with body
                    let data = &mut self.data[data];
                    data.pos = (data.pos * data.mass + body.pos * body.mass) / (data.mass + body.mass);
                    data.mass += body.mass;

                    // Insert current body to containing node
                    let quadrant = boundary.quadrant(body.pos);
                    boundary = boundary.become_quadrant(quadrant);
                    node = child + quadrant as usize;
                }
                // Node is an empty leaf
                else {
                    self.nodes[node].data = self.data.len();
                    self.data.push(BarnesHutData::new(body.pos, body.mass));
                    break;
                }
            }
        }
    }

    pub fn calculate_acceleration(&self, position: Vector2<f32>, theta: f32) -> Vector2<f32> {
        puffin::profile_function!();

        if self.nodes.len() == 0 {
            return Vector2::zeros();
        }

        let mut stack = Vec::new();
        let mut width_stack = Vec::new();

        for i in 0..4 {
            if !self.nodes[i].is_empty() && position != self.data[self.nodes[i].data].pos {
                stack.push(i);
                width_stack.push(self.boundary.width);
            }
        }

        let mut sum: Vector2<f32> = Vector2::zeros();

        while let Some(node) = stack.pop() {
            let width = width_stack.pop().unwrap();
            let node = &self.nodes[node];
            let data = &self.data[node.data];

            if node.is_leaf() || width * width < (position - data.pos).magnitude_squared() * theta * theta {
                let m = data.mass;
                let d: Vector2<f32> = data.pos - position;
                let r = d.magnitude_squared().inv_sqrt();
    
                sum += (m * r * r * r) * d;
            } else {
                for i in 0..4 {
                    let child = node.child + i;
                    if !self.nodes[child].is_empty() && position != self.data[self.nodes[child].data].pos {
                        stack.push(child);
                        width_stack.push(width * 0.5);
                    }
                }
            }
        }

        sum
    }
}