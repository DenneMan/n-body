use nalgebra::Vector2;

use crate::physics::{
    inv_sqrt::InvSqrt
};

use super::body::Body;

#[derive(Clone, Copy)]
pub enum Quadrant {
    I   = 0, // North-east
    II  = 1, // North-west
    III = 2, // South-west
    IV  = 3, // South-east
}

impl From<usize> for Quadrant {
    fn from(value: usize) -> Self {
        match value {
            0 => Quadrant::I,
            1 => Quadrant::II,
            2 => Quadrant::III,
            3 => Quadrant::IV,
            _ => unreachable!(),
        }
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
        if point.y > self.center.y {
            if point.x > self.center.x {
                return Quadrant::I;
            }
            else {
                return Quadrant::II;
            }
        }
        else {
            if point.x > self.center.x {
                return Quadrant::IV;
            }
            else {
                return Quadrant::III;
            }
        }
    }

    pub fn become_quadrant(mut self, quadrant: Quadrant) -> Self {
        match quadrant {
            Quadrant::I => {
                self.width *= 0.5;
                self.center.x += self.width;
                self.center.y += self.width;
            },
            Quadrant::II => {
                self.width *= 0.5;
                self.center.x -= self.width;
                self.center.y += self.width;
            },
            Quadrant::III => {
                self.width *= 0.5;
                self.center.x -= self.width;
                self.center.y -= self.width;
            },
            Quadrant::IV => {
                self.width *= 0.5;
                self.center.x += self.width;
                self.center.y -= self.width;
            },
        }

        self
    }
}

#[derive(Clone)]
pub struct BarnesHutData {
    pub center_of_mass: Vector2<f32>,
    pub mass: f32,
}

impl BarnesHutData {
    pub fn new(center_of_mass: Vector2<f32>, mass: f32) -> Self {
        Self {
            center_of_mass,
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
        self.nodes.push(Quadnode::default()); // 2, 1
        self.nodes.push(Quadnode::default()); // 3, 4
        self.nodes.push(Quadnode::default());
    }
    
    pub fn insert(&mut self, bodies: &[Body]) { // TODO: try not dividing by mass and then do a final pass over all quads in the end. 
        puffin::profile_function!();

        for body in bodies {

            let mut boundary = self.boundary.clone();

            let quadrant = boundary.quadrant(body.pos);
            let mut node = quadrant as usize;
            boundary = boundary.become_quadrant(quadrant);

            loop {
                // Node is a branch
                if let Some(child) = self.nodes[node].get_child() {
                    let data = &mut self.data[self.nodes[node].data];
                    data.center_of_mass = (data.center_of_mass * data.mass + body.pos * body.mass) / (data.mass + body.mass);
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
                    let quadrant = boundary.quadrant(self.data[data].center_of_mass);
                    self.nodes[child + quadrant as usize].data = child_data;

                    // Update current node with body
                    let data = &mut self.data[data];
                    data.center_of_mass = (data.center_of_mass * data.mass + body.pos * body.mass) / (data.mass + body.mass);
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

    /// An "unrecursed" version of the naive calculate_acceleration_recursive.
    pub fn calculate_acceleration(&self, position: Vector2<f32>, theta: f32) -> Vector2<f32> {
        puffin::profile_function!();

        if self.nodes.len() == 0 {
            return Vector2::zeros();
        }

        let mut stack = vec![0, 1, 2, 3];

        let mut boundary_stack = vec![
            self.boundary.clone().become_quadrant(Quadrant::I),
            self.boundary.clone().become_quadrant(Quadrant::II),
            self.boundary.clone().become_quadrant(Quadrant::III),
            self.boundary.clone().become_quadrant(Quadrant::IV),
        ];

        let mut sum: Vector2<f32> = Vector2::zeros();

        while let Some(node) = stack.pop() {
            let boundary = boundary_stack.pop().unwrap();
            let data = &self.data[self.nodes[node].data];

            if self.nodes[node].is_leaf() || boundary.width * boundary.width * 4.0 < (position - data.center_of_mass).norm_squared() * theta * theta {
                let m = data.mass;
                let d: Vector2<f32> = data.center_of_mass - position;
                let r = d.magnitude_squared().inv_sqrt();
    
                sum += (m * r * r * r) * d;
            } else {
                for i in 0..4 {
                    let child = self.nodes[node].child + i;
                    if !self.nodes[child].is_empty() && position != self.data[self.nodes[child].data].center_of_mass {
                        stack.push(child);
                        boundary_stack.push(boundary.clone().become_quadrant(i.into()));
                    }
                }
            }
        }

        sum
    }
}