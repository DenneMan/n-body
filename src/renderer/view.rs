use nalgebra::{Vector2, Point2};

use winit::event::VirtualKeyCode;
use winit_input_helper::WinitInputHelper;

use crate::physics::{
    boundary::Boundary,
    world::World,
};

pub enum ViewState {
    Free,
    Maximized,
    Follow(usize, Vector2<f32>),
}

pub struct View {
    pub state: ViewState,
    pub boundary: Boundary,
    pub window_width: u32,
    pub window_height: u32,
}

impl View {
    pub fn new(window_width: u32, window_height: u32) -> Self {
        Self {
            window_width,
            window_height,
            state: ViewState::Maximized,
            boundary: Boundary::new(Point2::origin(), Point2::origin()),
        }
    }

    pub fn resize(&mut self, width: u32, height: u32) {
        let center = Point2::from((self.boundary.min.coords + self.boundary.max.coords) * 0.5);

        let new_width  = self.boundary.width()  / self.window_width  as f32 * width  as f32;
        let new_height = self.boundary.height() / self.window_height as f32 * height as f32;

        let p = Vector2::new(new_width, new_height) * 0.5;

        self.boundary.min = center - p;
        self.boundary.max = center + p;

        self.window_width = width;
        self.window_height = height;
    }

    pub fn update(&mut self, input: &WinitInputHelper, world: &World) {
        self.handle_state_change(input, world);

        match self.state {
            ViewState::Free => {
                if input.mouse_held(2) {
                    let w = self.boundary.width();
                    let h = self.boundary.height();

                    let movement = input.mouse_diff();
                    self.boundary.min.x -= movement.0 / self.window_width  as f32 * w;
                    self.boundary.min.y += movement.1 / self.window_height as f32 * h;
                    self.boundary.max.x -= movement.0 / self.window_width  as f32 * w;
                    self.boundary.max.y += movement.1 / self.window_height as f32 * h;
                }

                if let Some(mouse) = input.mouse() {
                    let x = mouse.0 / self.window_width  as f32;
                    let y = mouse.1 / self.window_height as f32;

                    let w = -input.scroll_diff() * self.boundary.width() * 0.1;
                    let h = -input.scroll_diff() * self.boundary.height() * 0.1;

                    self.boundary.min.x -= w * x;
                    self.boundary.min.y -= h * (1.0 - y);
                    self.boundary.max.x += w * (1.0 - x);
                    self.boundary.max.y += h * y;
                }
            },
            ViewState::Maximized => {
                let boundary = world.calculate_boundary().pad(world.find_largest_radius().unwrap()); // TODO: solve panic

                let p: Vector2<f32> = if self.window_width > self.window_height {
                    Vector2::new(boundary.width() / self.window_height as f32 * self.window_width as f32, boundary.height()) * 0.5
                }
                else {
                    Vector2::new(boundary.width(), boundary.height() / self.window_width as f32 * self.window_height as f32) * 0.5
                };

                let center = Point2::from((boundary.min.coords + boundary.max.coords) * 0.5);

                self.boundary.min = center - p;
                self.boundary.max = center + p;
            },
            ViewState::Follow(body, mut offset) => {

                if input.mouse_held(2) {
                    let w = self.boundary.width();
                    let h = self.boundary.width();

                    let movement = input.mouse_diff();
                    offset.x -= movement.0 / self.window_width  as f32 * w;
                    offset.y += movement.1 / self.window_height as f32 * h;
                }

                let w = self.boundary.width() / 2.0;
                let h = self.boundary.height() / 2.0;
                let poi: Point2<f32> = world.bodies[body].pos + offset;

                self.boundary.min.x = poi.x - w;
                self.boundary.min.y = poi.y - h;
                self.boundary.max.x = poi.x + w;
                self.boundary.max.y = poi.y + h;

                if input.mouse().is_some() {
                    let w = -input.scroll_diff() * self.boundary.width() * 0.1;
                    let h = -input.scroll_diff() * self.boundary.width() * 0.1;

                    self.boundary.min.x -= w;
                    self.boundary.min.y -= h;
                    self.boundary.max.x += w;
                    self.boundary.max.y += h;
                }

                self.state = ViewState::Follow(body, offset)
            },
        }
    }

    fn handle_state_change(&mut self, input: &WinitInputHelper, world: &World) {
        let window_boundary = Boundary::new(Point2::origin(), Point2::new(self.window_width as f32, self.window_height as f32));

        match self.state {
            ViewState::Free => {
                if input.key_pressed(VirtualKeyCode::M) {
                    self.state = ViewState::Maximized;
                }
                else if input.mouse_pressed(0) {
                    if let Some(mouse) = input.mouse() {
                        let x = mouse.0; let y = self.window_height as f32 - mouse.1;
                        let world_pos: Point2<f32> = window_boundary.map(Point2::new(x, y), &self.boundary);

                        world.bodies.iter().enumerate().for_each(|(i, body)| if (world_pos - body.pos).magnitude_squared() < body.radius * body.radius {
                            self.state = ViewState::Follow(i, Vector2::zeros());
                        })
                    }
                }
            },
            ViewState::Maximized => {
                if input.key_pressed(VirtualKeyCode::M)
                || input.mouse_pressed(2) {
                    self.state = ViewState::Free;
                }
                else if input.mouse_pressed(0) {
                    if let Some(mouse) = input.mouse() {
                        let x = mouse.0; let y = self.window_height as f32 - mouse.1;
                        let world_pos: Point2<f32> = window_boundary.map(Point2::new(x, y), &self.boundary);
    
                        world.bodies.iter().enumerate().for_each(|(i, body)| if (world_pos - body.pos).magnitude_squared() < body.radius * body.radius {
                            self.state = ViewState::Follow(i, Vector2::zeros());
                        })
                    }
                }
            },
            ViewState::Follow(_, _) => {
                if input.key_pressed(VirtualKeyCode::M) {
                    self.state = ViewState::Maximized;
                }
                else if input.mouse_pressed(0) {
                    if let Some(mouse) = input.mouse() {
                        let x = mouse.0; let y = self.window_height as f32 - mouse.1;
                        let world_pos: Point2<f32> = window_boundary.map(Point2::new(x, y), &self.boundary);

                        if let Some((i, _)) =
                            world.bodies.iter()
                                .enumerate()
                                .find(|(_, body)| 
                                    (world_pos - body.pos).magnitude_squared() < body.radius * body.radius) {
                            self.state = ViewState::Follow(i, Vector2::zeros());
                        }
                        else {
                            self.state = ViewState::Free;
                        }
                    }
                }
            },
        }
    }
}