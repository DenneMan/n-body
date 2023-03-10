use std::{f64::consts::{TAU, PI}, time::Instant};

use n_body::{World, Boundary, Body, Quadtree};
use nalgebra::{Point2, Vector2};

use rand::prelude::*;

use log::error;

use pixels::{Pixels, SurfaceTexture};

use winit::{
    dpi::PhysicalSize,
    event::VirtualKeyCode,
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};

use winit_input_helper::WinitInputHelper;

const WIDTH: usize = 800;
const HEIGHT: usize = 800;

const WINDOW_BOUNDARY: Boundary = Boundary::new(Point2::new(0.0, 0.0), Point2::new(WIDTH as f64, HEIGHT as f64));

enum ViewState {
    Free,
    Maximized,
    Follow(usize, Vector2<f64>),
}

struct View {
    state: ViewState,
    view: Boundary,
}

impl View {
    fn new(world: &World) -> Self {
        Self {
            state: ViewState::Free,
            view: world.calculate_boundary(),
        }
    }

    fn update(&mut self, input: &WinitInputHelper, world: &World) {
        self.handle_state_change(input, world);

        match self.state {
            ViewState::Free => {
                if input.mouse_held(2) {
                    let w = self.view.width();
                    let h = self.view.width();

                    let movement = input.mouse_diff();
                    self.view.min.x -= movement.0 as f64 / WIDTH  as f64 * w;
                    self.view.min.y += movement.1 as f64 / HEIGHT as f64 * h;
                    self.view.max.x -= movement.0 as f64 / WIDTH  as f64 * w;
                    self.view.max.y += movement.1 as f64 / HEIGHT as f64 * h;
                }

                if let Some(mouse) = input.mouse() {
                    let x = mouse.0 as f64 / WIDTH  as f64;
                    let y = mouse.1 as f64 / HEIGHT as f64;

                    let w = -input.scroll_diff() as f64 * self.view.width() * 0.1;
                    let h = -input.scroll_diff() as f64 * self.view.width() * 0.1;

                    self.view.min.x -= w * x;
                    self.view.min.y -= h * (1.0 - y);
                    self.view.max.x += w * (1.0 - x);
                    self.view.max.y += h * y;
                }
            },
            ViewState::Maximized => {
                self.view = world.calculate_boundary().pad(world.find_largest_radius().unwrap()); // TODO: solve panic
            },
            ViewState::Follow(body, mut offset) => {

                if input.mouse_held(2) {
                    let w = self.view.width();
                    let h = self.view.width();

                    let movement = input.mouse_diff();
                    offset.x -= movement.0 as f64 / WIDTH  as f64 * w;
                    offset.y += movement.1 as f64 / HEIGHT as f64 * h;
                }

                let w = self.view.width() / 2.0;
                let h = self.view.height() / 2.0;
                let poi: Point2<f64> = world.bodies[body].pos + offset;

                self.view.min.x = poi.x - w;
                self.view.min.y = poi.y - h;
                self.view.max.x = poi.x + w;
                self.view.max.y = poi.y + h;

                if input.mouse().is_some() {
                    let w = -input.scroll_diff() as f64 * self.view.width() * 0.1;
                    let h = -input.scroll_diff() as f64 * self.view.width() * 0.1;

                    self.view.min.x -= w;
                    self.view.min.y -= h;
                    self.view.max.x += w;
                    self.view.max.y += h;
                }

                self.state = ViewState::Follow(body, offset)
            },
        }
    }

    fn handle_state_change(&mut self, input: &WinitInputHelper, world: &World) {
        match self.state {
            ViewState::Free => {
                if input.key_pressed(VirtualKeyCode::M) {
                    self.state = ViewState::Maximized;
                }
                else if input.mouse_pressed(0) {
                    let mouse: Point2<f64> = {
                        let tmp = input.mouse().unwrap(); // left click should only be if mouse is in window
                        Point2::new(tmp.0 as f64, HEIGHT as f64 - tmp.1 as f64)
                    };
                    let world_pos: Point2<f64> = WINDOW_BOUNDARY.map(mouse, &self.view);

                    world.bodies.iter().enumerate().for_each(|(i, body)| if (world_pos - body.pos).magnitude_squared() < body.radius * body.radius {
                        self.state = ViewState::Follow(i, Vector2::zeros());
                    })
                }
            },
            ViewState::Maximized => {
                if input.key_pressed(VirtualKeyCode::M)
                || input.mouse_pressed(2) {
                    self.state = ViewState::Free;
                }
                else if input.mouse_pressed(0) {
                    let mouse: Point2<f64> = {
                        let tmp = input.mouse().unwrap(); // left click should only be if mouse is in window
                        Point2::new(tmp.0 as f64, HEIGHT as f64 - tmp.1 as f64)
                    };
                    let world_pos: Point2<f64> = WINDOW_BOUNDARY.map(mouse, &self.view);

                    world.bodies.iter().enumerate().for_each(|(i, body)| if (world_pos - body.pos).magnitude_squared() < body.radius * body.radius {
                        self.state = ViewState::Follow(i, Vector2::zeros());
                    })
                }
            },
            ViewState::Follow(_, _) => {
                if input.key_pressed(VirtualKeyCode::M) {
                    self.state = ViewState::Maximized;
                }
                else if input.mouse_pressed(0) {
                    let mouse: Point2<f64> = {
                        let tmp = input.mouse().unwrap(); // left click should only be if mouse is in window
                        Point2::new(tmp.0 as f64, HEIGHT as f64 - tmp.1 as f64)
                    };
                    let world_pos: Point2<f64> = WINDOW_BOUNDARY.map(mouse, &self.view);

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
            },
        }
    }
}

fn main() {
    let size = PhysicalSize::new(WIDTH as u32, HEIGHT as u32);

    let event_loop = EventLoop::new();
    let mut input = WinitInputHelper::new();
    
    let window = WindowBuilder::new()
        .with_resizable(false)
        .with_title("Particles >:)")
        .with_inner_size(size)
        .with_min_inner_size(size)
        .build(&event_loop)
        .unwrap();

    let mut pixels = Pixels::new(
        WIDTH as u32, 
        HEIGHT as u32, 
        SurfaceTexture::new(size.width, size.height, &window)
    ).unwrap();

    let mut world = World::new();

    world.insert(Body::new(
        Point2::origin(),
        Vector2::zeros(),
        10000000.0,
        50.0,
    ));

    const N: u32 = 100;
    let mut rng = rand::thread_rng();
    for i in 0..N {
        let i = i as f64;
        let n = N as f64;

        let distance = rng.gen_range(0.25..1.0);
        let mut radius: f64 = rng.gen_range(0.0..1.0);
        radius = radius.powi(3) * 10.0 + 10.0;
        world.insert(Body::new(
            Point2::new((TAU / n * i).sin(), (TAU / n * i).cos()) * 1000.0 * distance,
            Vector2::new((TAU / n * i + PI * 0.5).sin(), (TAU / n * i + PI * 0.5).cos()) * 100.0 / distance.cbrt(),
            radius * radius * radius * 10.0,
            radius,
        ));
    }

    let time = Instant::now();
    let mut frames = 0;

    let mut paused = false;
    let mut quadtree = false;

    let mut view = View::new(&world);

    event_loop.run(move |event, _, control_flow| {
        if input.update(&event) {
            if input.key_pressed(VirtualKeyCode::Escape) || input.close_requested() || input.destroyed() {
                println!("Average fps: {}", frames as f32 / time.elapsed().as_secs_f32());
                *control_flow = ControlFlow::Exit;
                return;
            }

            if input.key_pressed(VirtualKeyCode::Space) {
                paused = !paused;
            }

            if input.key_pressed(VirtualKeyCode::Q) {
                quadtree = !quadtree;
            }

            if input.key_pressed(VirtualKeyCode::E) {
                println!("{}", world.total_energy());
            }

            // update view
            view.update(&input, &world);


            if !paused {
                world.update_substeps(0.01, 16);
            }

            let frame = pixels.get_frame_mut();

            frame.fill(0x00);

            if quadtree {
                draw_quadtree(&world, frame, &view)
            }

            //let boundary = world.calculate_boundary().pad(5.0);
            draw_particles(&world, frame, &view.view);

            if let Err(err) = pixels.render() {
                error!("pixels.render() failed: {}", err);
                *control_flow = ControlFlow::Exit;
                return;
            }
            frames += 1;
        }
    });
}

fn draw_quadtree(world: &World, frame: &mut [u8], view: &View) {
    let boundary = &view.view;

    let world_boundary = world.calculate_boundary();
    let mut quadtree = Quadtree::new(world_boundary);

    world.bodies.iter().for_each(|body| {
        quadtree.insert(body.pos, body.mass);
    });

    let mut stack = vec![&quadtree];

    loop {
        if let Some(current) = stack.pop() {
            if let ViewState::Follow(body, _) = view.state {
                let position = world.bodies[body].pos;

                if current.is_leaf() || current.boundary.size() < (position - current.center_of_mass).abs() * 1.0 {
                    let p0: Point2<f64> = boundary.map(position, &WINDOW_BOUNDARY);
                    let p1: Point2<f64> = boundary.map(current.center_of_mass, &WINDOW_BOUNDARY);
                    let x0 = p0.x as isize; let x1 = p1.x as isize; let y0 = p0.y as isize; let y1 = p1.y as isize;

                    draw_circle(p1.x, p1.y, 2.0, 0xff0000ff, true, frame);

                    draw_line(x0, y0, x1, y1, 0x444444ff, frame);

                    let p0: Point2<f64> = boundary.map(current.boundary.min, &WINDOW_BOUNDARY);
                    let p1: Point2<f64> = boundary.map(current.boundary.max, &WINDOW_BOUNDARY);
                    let x0 = p0.x as isize; let x1 = p1.x as isize; let y0 = p0.y as isize; let y1 = p1.y as isize;
    
                    draw_square(x0, x1, y0, y1, 0xffffffff, frame);
                } else {
                    for child in current.children.iter() {
                        if !child.is_empty() && position != child.center_of_mass {
                            stack.push(child);
                        }
                    }
                }
            }
            else {
                if current.boundary.intersects(boundary) {
                    let p0: Point2<f64> = boundary.map(current.boundary.min, &WINDOW_BOUNDARY);
                    let p1: Point2<f64> = boundary.map(current.boundary.max, &WINDOW_BOUNDARY);
                    let x0 = p0.x as isize; let x1 = p1.x as isize; let y0 = p0.y as isize; let y1 = p1.y as isize;
    
                    draw_square(x0, x1, y0, y1, 0xffffffff, frame);
                }
                if !current.is_leaf() {
                    for child in current.children.iter() {
                        if !child.is_empty() {
                            stack.push(child);
                        }
                    }
                }
            }
        }
        else {
            return;
        }
    }
}

fn draw_particles(world: &World, frame: &mut [u8], boundary: &Boundary) {
    let screen_boundary = Boundary::new(Point2::new(0.0, 0.0), Point2::new(WIDTH as f64, HEIGHT as f64));

    world.bodies.iter().filter(|&body| {
        boundary.pad(body.radius).contains(body.pos)
    }).map(|body| {
        (boundary.map(body.pos, &screen_boundary), boundary.map_distance(body.radius, &screen_boundary))
    }).for_each(|(pos, radius)| {
        draw_circle(pos.x, pos.y, radius, 0xffffffff, true, frame);
    });
}

fn draw_circle(x: f64, y: f64, r: f64, color: u32, fill: bool, frame: &mut [u8]) {
    fn not_filled(x: usize, y: usize, i: usize, j: usize, color: u32, frame: &mut [u8]) {
        set_pixel(x + i, y + j, color, frame);
        set_pixel(x + i, y - j, color, frame);
        set_pixel(x - i, y + j, color, frame);
        set_pixel(x - i, y - j, color, frame);
        set_pixel(x + j, y + i, color, frame);
        set_pixel(x + j, y - i, color, frame);
        set_pixel(x - j, y + i, color, frame);
        set_pixel(x - j, y - i, color, frame);
    }

    fn filled(x: isize, y: isize, i: isize, j: isize, color: u32, frame: &mut [u8]) {
        draw_horizontal_line(x - i, x + i, y + j, color, frame);
        draw_horizontal_line(x - i, x + i, y - j, color, frame);
        draw_horizontal_line(x - j, x + j, y + i, color, frame);
        draw_horizontal_line(x - j, x + j, y - i, color, frame);
    }

    let mut i = 0.0;
    let mut j = r;
    let mut d = 3.0 - (2.0 * r);

    if fill {
        filled(x as isize, y as isize, i as isize, j as isize, color, frame);
    } else {
        not_filled(x as usize, y as usize, i as usize, j as usize, color, frame);
    }

    while i <= j {
        if d <= 0.0 {
            d += (4.0 * i) + 6.0;
        }  
        else  
        {
            d += (4.0 * i) - (4.0 * j) + 10.0;
            j -= 1.0;
        }
        i += 1.0;

        if fill {
            filled(x as isize, y as isize, i as isize, j as isize, color, frame);
        } else {
            not_filled(x as usize, y as usize, i as usize, j as usize, color, frame);
        }
    }
}

fn draw_horizontal_line(x0: isize, x1: isize, y: isize, color: u32, frame: &mut [u8]) {
    for x in x0..x1 {
        set_pixel(x as usize, y as usize, color, frame);
    }
}

fn draw_vertical_line(x: isize, y0: isize, y1: isize, color: u32, frame: &mut [u8]) {
    for y in y0..y1 {
        set_pixel(x as usize, y as usize, color, frame);
    }
}

fn draw_square(x0: isize, x1: isize, y0: isize, y1: isize, color: u32, frame: &mut [u8]) {
    draw_horizontal_line(x0, x1, y0, 0xffffffff, frame);
    draw_horizontal_line(x0, x1, y1, 0xffffffff, frame);
    draw_vertical_line(x0, y0, y1, 0xffffffff, frame);
    draw_vertical_line(x1, y0, y1, 0xffffffff, frame);
}

fn set_pixel(x: usize, y: usize, color: u32, frame: &mut [u8]) {
    if x >= WIDTH || y >= HEIGHT {
        return;
    }

    let i = x + (HEIGHT - 1 - y) * WIDTH;
    frame[i * 4 + 0] = ((color >> 24) & 0xff) as u8;
    frame[i * 4 + 1] = ((color >> 16) & 0xff) as u8;
    frame[i * 4 + 2] = ((color >> 8)  & 0xff) as u8;
    frame[i * 4 + 3] = ((color >> 0)  & 0xff) as u8;
}

fn draw_line(mut x0: isize, mut y0: isize, x1: isize, y1: isize, color: u32, frame: &mut [u8]) {
    let dx = (x1 as f32 - x0 as f32).abs();
    let dy = -(y1 as f32 - y0 as f32).abs();
    let mut error = dx + dy;

    loop {
        set_pixel(x0 as usize, y0 as usize, color, frame);
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