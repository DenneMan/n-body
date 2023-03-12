use nalgebra::{Point2, Vector2};

use crate::physics::{
    boundary::Boundary,
    inv_sqrt::InvSqrt
};

/// Barnes-Hut quadtree implementation
pub struct Quadtree {
    pub children: Vec<Quadtree>, // Convert from nested to flattened
    pub center_of_mass: Point2<f32>,
    pub total_mass: f32,
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
        let min: Point2<f32> = self.boundary.min;
        let max: Point2<f32> = self.boundary.max;
        let ctr: Point2<f32> = nalgebra::center(&min, &max);
        self.children.push(Quadtree::new(Boundary::new(Point2::new(min.x, min.y), Point2::new(ctr.x, ctr.y))));
        self.children.push(Quadtree::new(Boundary::new(Point2::new(ctr.x, min.y), Point2::new(max.x, ctr.y))));
        self.children.push(Quadtree::new(Boundary::new(Point2::new(min.x, ctr.y), Point2::new(ctr.x, max.y))));
        self.children.push(Quadtree::new(Boundary::new(Point2::new(ctr.x, ctr.y), Point2::new(max.x, max.y))));
    }
    
    /// An "unrecursed" version of the naive insert_recursive.
    pub fn insert(&mut self, position: Point2<f32>, mass: f32) {
        puffin::profile_function!();

        let mut stack = vec![self];
        let mut positions = vec![position];
        let mut masses = vec![mass];

        loop {
            if let Some(current) = stack.pop() {
                let position = positions.pop().unwrap();
                let mass = masses.pop().unwrap();

                if current.is_empty() {
                    puffin::profile_scope!("is_empty");
                    current.center_of_mass = position;
                    current.total_mass = mass;
                }
                else if current.is_leaf() && !current.is_empty() {
                    puffin::profile_scope!("is_leaf");
        
                    if position == current.center_of_mass {
                        panic!("Adding duplicate point results in infinite octree.")
                    }
        
                    current.subdivide();

                    for child in current.children.iter_mut() {
                        if child.boundary.contains(current.center_of_mass) {
                            child.center_of_mass = current.center_of_mass;
                            child.total_mass = current.total_mass;
                            break;
                        }
                    }
        
                    for child in current.children.iter_mut() {
                        if child.boundary.contains(position) {
                            stack.push(child);
                            positions.push(position);
                            masses.push(mass);
                            break;
                        }
                    }
                    
                    current.center_of_mass = (current.center_of_mass * current.total_mass + Vector2::from(position.coords) * mass) / (current.total_mass + mass);
                    current.total_mass += mass;
                }
                else if !current.is_leaf() {
                    puffin::profile_scope!("is_branch");
        
                    for child in current.children.iter_mut() {
                        if child.boundary.contains(position) {
                            stack.push(child);
                            positions.push(position);
                            masses.push(mass);
                            break;
                        }
                    }
        
                    current.center_of_mass = (current.center_of_mass * current.total_mass + Vector2::from(position.coords) * mass) / (current.total_mass + mass);
                    current.total_mass += mass;
                }
            }
            else {
                return;
            }
        }
    }

    pub fn insert_recursive(&mut self, position: Point2<f32>, mass: f32) {
        puffin::profile_function!();

        if self.is_empty() {
            puffin::profile_scope!("is_empty");
            self.center_of_mass = position;
            self.total_mass = mass;
        }
        else if self.is_leaf() && !self.is_empty() {
            puffin::profile_scope!("is_leaf");

            if position == self.center_of_mass {
                panic!("Adding duplicate point results in infinite octree.")
            }

            self.subdivide();

            for child in self.children.iter_mut() {
                if child.boundary.contains(self.center_of_mass) {
                    child.insert_recursive(self.center_of_mass, self.total_mass);
                }
                if child.boundary.contains(position) {
                    child.insert_recursive(position, mass);
                }
            }
            
            self.center_of_mass = (self.center_of_mass * self.total_mass + Vector2::from(position.coords) * mass) / (self.total_mass + mass);
            self.total_mass += mass;
        }
        else if !self.is_leaf() {
            puffin::profile_scope!("is_branch");

            self.children.iter_mut()
                .find(|child| {
                    child.boundary.contains(position)
                }).unwrap() // Error if somehow no child contains the position
                .insert_recursive(position, mass);

            self.center_of_mass = (self.center_of_mass * self.total_mass + Vector2::from(position.coords) * mass) / (self.total_mass + mass);
            self.total_mass += mass;
        }
    }

    /// An "unrecursed" version of the naive calculate_acceleration_recursive.
    pub fn calculate_acceleration(&self, position: Point2<f32>, theta: f32) -> Vector2<f32> {
        puffin::profile_function!();

        let mut stack = vec![self];

        let mut sum: Vector2<f32> = Vector2::zeros();

        loop {
            if let Some(current) = stack.pop() {
                if current.is_leaf() || current.boundary.area() < (position - current.center_of_mass).norm_squared() * theta * theta {
                    let m = current.total_mass;
                    let d: Vector2<f32> = current.center_of_mass - position;
                    let r = d.magnitude_squared().inv_sqrt();
        
                    sum += (m * r * r * r) * d;
                } else {
                    current.children.iter()
                        .filter(|child| !child.is_empty() && position != child.center_of_mass)
                        .for_each(|child| stack.push(child));
                }
            }
            else {
                return sum;
            }
        }
    }

    pub fn calculate_acceleration_recursive(&self, position: Point2<f32>, theta: f32) -> Vector2<f32> {
        if self.is_empty() || position == self.center_of_mass {
            return Vector2::zeros();
        }

        if self.is_leaf() || self.boundary.area() < (position - self.center_of_mass).norm_squared() * theta * theta {
            let m = self.total_mass;
            let d: Vector2<f32> = self.center_of_mass - position;
            let r = d.magnitude_squared().inv_sqrt();

            return (m * r * r * r) * d;
        } else {
            return self.children.iter()
                .map(|child| {
                    child.calculate_acceleration_recursive(position, theta)
                }).sum();
        }
    }
}