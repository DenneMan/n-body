use nalgebra::{Point2, Vector2};
use pixels::{Pixels, SurfaceTexture};
use winit::{window::Window, event_loop::EventLoop};
use winit_input_helper::WinitInputHelper;

use crate::physics::{
    world::World, 
    boundary::Boundary, 
    quadtree::Quadtree
};

pub mod view;
pub mod gui;

use gui::Framework;

use self::view::View;

pub struct Renderer {
    pub window: Window,
    pub pixels: Pixels,
    pub framework: Framework,
    pub view: View,
    pub width: u32,
    pub height: u32,
}

impl Renderer {
    pub fn new(window: Window, event_loop: &EventLoop<()>) -> Self {
        let (pixels, framework, width, height) = {
            let window_size = window.inner_size();
            let scale_factor = window.scale_factor() as f32;
            let surface_texture = SurfaceTexture::new(window_size.width, window_size.height, &window);
            let pixels = Pixels::new(window_size.width, window_size.height, surface_texture).unwrap();
            let framework = Framework::new(
                &event_loop,
                window_size.width,
                window_size.height,
                scale_factor,
                &pixels,
            );
    
            (pixels, framework, window_size.width, window_size.height)
        };

        Self {
            window,
            pixels,
            framework,
            view: View::new(width, height),
            width,
            height,
        }
    }

    pub fn start_frame(&mut self, input: &WinitInputHelper, world: &World) {
        if let Some(size) = input.window_resized() {
            if size.height > self.height {
                self.pixels.resize_surface(size.width, size.height).unwrap();
                self.pixels.resize_buffer(size.width, size.height).unwrap();
            }
            else {
                self.pixels.resize_buffer(size.width, size.height).unwrap();
                self.pixels.resize_surface(size.width, size.height).unwrap();
            }

            self.framework.resize(size.width, size.height);
            self.view.resize(size.width, size.height);

            self.width = size.width as u32;
            self.height = size.height as u32;
        }

        self.view.update(&input, &world);

        self.pixels.get_frame_mut().fill(0x00);
    }

    pub fn end_frame(&mut self) {
        self.framework.prepare(&self.window);

        // Render everything together
        self.pixels.render_with(|encoder, render_target, context| {
            // Render the world texture
            context.scaling_renderer.render(encoder, render_target);

            // Render egui
            self.framework.render(encoder, render_target, context);

            Ok(())
        }).unwrap();
    }

    pub fn set_pixel(&mut self, x: i32, y: i32, color: u32) {
        let x = x as u32;
        let y = y as u32;

        if x >= self.width || y >= self.height {
            return;
        }

        let frame = self.pixels.get_frame_mut();

        let i = (x + (self.height - 1 - y) * self.width) as usize;
        frame[i * 4 + 0] = ((color >> 24) & 0xff) as u8;
        frame[i * 4 + 1] = ((color >> 16) & 0xff) as u8;
        frame[i * 4 + 2] = ((color >> 8)  & 0xff) as u8;
        frame[i * 4 + 3] = ((color >> 0)  & 0xff) as u8;
    }
    
    fn draw_horizontal_line(&mut self, x0: i32, x1: i32, y: i32, color: u32) {
        for x in x0.max(0)..x1.min(self.width as i32) {
            self.set_pixel(x, y, color);
        }
    }
    
    fn draw_vertical_line(&mut self, x: i32, y0: i32, y1: i32, color: u32) {
        for y in y0.max(0)..y1.min(self.height as i32) {
            self.set_pixel(x, y, color);
        }
    }

    fn outlined_circle_helper(&mut self, x: i32, y: i32, i: i32, j: i32, color: u32) {
        self.set_pixel(x + i, y + j, color);
        self.set_pixel(x + i, y - j, color);
        self.set_pixel(x - i, y + j, color);
        self.set_pixel(x - i, y - j, color);
        self.set_pixel(x + j, y + i, color);
        self.set_pixel(x + j, y - i, color);
        self.set_pixel(x - j, y + i, color);
        self.set_pixel(x - j, y - i, color);
    }

    fn filled_circle_helper(&mut self, x: i32, y: i32, i: i32, j: i32, color: u32) {
        self.draw_horizontal_line(x - i, x + i, y + j, color);
        self.draw_horizontal_line(x - i, x + i, y - j, color);
        self.draw_horizontal_line(x - j, x + j, y + i, color);
        self.draw_horizontal_line(x - j, x + j, y - i, color);
    }

    pub fn draw_rectangle(&mut self, x0: i32, x1: i32, y0: i32, y1: i32, color: u32) {
        self.draw_horizontal_line(x0, x1, y0, color);
        self.draw_horizontal_line(x0, x1, y1, color);
        self.draw_vertical_line(x0, y0, y1, color);
        self.draw_vertical_line(x1, y0, y1, color);
    }

    pub fn draw_circle(&mut self, x: i32, y: i32, r: i32, color: u32, fill: bool) {
        let mut i = 0;
        let mut j = r;
        let mut d = 3 - (2 * r);
    
        if fill {
            self.filled_circle_helper(x, y, i, j, color);
        } else {
            self.outlined_circle_helper(x, y, i, j, color);
        }
    
        while i <= j {
            if d <= 0 {
                d += (4 * i) + 6;
            }  
            else  
            {
                d += (4 * i) - (4 * j) + 10;
                j -= 1;
            }
            i += 1;
    
            if fill {
                self.filled_circle_helper(x, y, i, j, color);
            } else {
                self.outlined_circle_helper(x, y, i, j, color);
            }
        }
    }

    fn draw_line(&mut self, mut x0: i32, mut y0: i32, x1: i32, y1: i32, color: u32) {
        let dx = (x1 as f32 - x0 as f32).abs();
        let dy = -(y1 as f32 - y0 as f32).abs();
        let mut error = dx + dy;
    
        loop {
            self.set_pixel(x0, y0, color);
            if x0 == x1 && y0 == y1 { break; }
            let e2 = 2.0 * error;
            if e2 >= dy {
                if x0 == x1 { break; }
                error += dy;
                if x0 < x1 {
                    x0 += 1;
                }
                else {
                    x0 -= 1;
                }
            }
            if e2 <= dx {
                if y0 == y1 { break; }
                error += dx;
                if y0 < y1 {
                    y0 += 1;
                }
                else {
                    y0 -= 1;
                }
            }
        }
    }

    pub fn draw_world(&mut self, world: &World) {
        let screen_boundary = Boundary::new(Point2::new(0.0, 0.0), Point2::new(self.width as f32, self.height as f32));

        for body in world.bodies.iter() {
            if self.view.boundary.pad(body.radius).contains(body.pos) {
                let pos = self.world_to_screen(body.pos);
                self.draw_circle(
                    pos.x as i32, 
                    pos.y as i32, 
                    (body.radius / self.view.boundary.width() * screen_boundary.width()) as i32, 
                    0xffffffff, 
                    true
                );
            }
        }
    }

    pub fn world_to_screen(&self, point: Point2<f32>) -> Point2<f32> {
        Point2::origin() + (point - self.view.boundary.min).component_div(&self.view.boundary.size()).component_mul(&Vector2::new(self.width as f32, self.height as f32))
    }

    pub fn screen_to_world(&self, point: Point2<f32>) -> Point2<f32> {
        self.view.boundary.min + (point - Point2::origin()).component_div(&Vector2::new(self.width as f32, self.height as f32)).component_mul(&self.view.boundary.size())
    }

    pub fn draw_relative_quadtree(&mut self, quadtree: &Quadtree, position: Point2<f32>, theta: f32) {
        let mut stack = vec![quadtree];
        loop {
            if let Some(current) = stack.pop() {
                if current.is_leaf() || current.boundary.area() < (position - current.center_of_mass).magnitude_squared() * theta * theta {
                    let p0: Point2<f32> = self.world_to_screen(position);
                    let p1: Point2<f32> = self.world_to_screen(current.center_of_mass);
                    let x0 = p0.x as i32; let x1 = p1.x as i32; let y0 = p0.y as i32; let y1 = p1.y as i32;

                    self.draw_circle(p1.x as i32, p1.y as i32, 2, 0xff0000ff, true);

                    self.draw_line(x0, y0, x1, y1, 0x444444ff);

                    let p0: Point2<f32> = self.world_to_screen(current.boundary.min);
                    let p1: Point2<f32> = self.world_to_screen(current.boundary.max);
                    let x0 = p0.x as i32; let x1 = p1.x as i32; let y0 = p0.y as i32; let y1 = p1.y as i32;
    
                    self.draw_rectangle(x0, x1, y0, y1, 0xffffffff);
                } else {
                    for child in current.children.iter() {
                        if !child.is_empty() && position != child.center_of_mass {
                            stack.push(child);
                        }
                    }
                }
            }
            else {
                return;
            }
        }

    }

    pub fn draw_quadtree(&mut self, quadtree: &Quadtree) {
        let mut stack = vec![quadtree];
    
        loop {
            if let Some(current) = stack.pop() {
                if current.boundary.intersects(&self.view.boundary) {
                    let p0: Point2<f32> = self.world_to_screen(current.boundary.min);
                    let p1: Point2<f32> = self.world_to_screen(current.boundary.max);
                    let x0 = p0.x as i32; let x1 = p1.x as i32; let y0 = p0.y as i32; let y1 = p1.y as i32;
    
                    self.draw_rectangle(x0, x1, y0, y1, 0xffffffff);
                }
                if !current.is_leaf() {
                    for child in current.children.iter() {
                        if !child.is_empty() {
                            stack.push(child);
                        }
                    }
                }
            }
            else {
                return;
            }
        }
    }
}