use nalgebra::Point2;


use crate::physics::boundary::Boundary;

/// Space partitioning grid for collision detection
pub struct Grid {
    cells: Vec<Vec<usize>>,
    pub resolution: usize,
    boundary: Boundary,
}

impl Grid {
    pub fn new(min_cell_width: f32, boundary: Boundary) -> Self {
        let resolution = (boundary.width() / min_cell_width).floor() as usize;

        Self {
            cells: vec![Vec::new(); resolution * resolution],
            resolution,
            boundary,
        }
    }

    pub fn insert(&mut self, index: usize, position: Point2<f32>) {
        let pos: Point2<f32> = self.boundary.map(
            position, 
            &Boundary::new(
                Point2::new(0.0, 0.0), 
                Point2::new(self.resolution as f32, self.resolution as f32)
        ));
        let i = pos.x.floor() as usize;
        let j = pos.y.floor() as usize;

        self.cells[i + j * self.resolution].push(index);
    }

    pub fn get3x3(&self, x: usize, y: usize) -> Vec<usize> {
        let mut result = Vec::new();

        for i in -1..=1 {
            for j in -1..=1 {
                let x = (x as i32 + i) as usize;
                let y = (y as i32 + j) as usize;
                result.append(&mut (self.cells[x + y * self.resolution]).clone());
            } 
        }
        result
    }
}