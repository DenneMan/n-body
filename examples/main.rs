use std::{f64::consts::{TAU, PI}, time::Instant};

use n_body::{World, Boundary, Body};
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

    let radius = 5.0;
    world.insert(Body::new(
        Point2::origin(),
        Vector2::zeros(),
        radius * radius * radius * 100.0,
        radius,
    ));

    const N: u32 = 300;
    let mut rng = rand::thread_rng();
    for i in 0..N {
        let i = i as f64;
        let n = N as f64;

        let mut radius = rng.gen_range(0.5..1.0);
        radius *= radius;
        world.insert(Body::new(
            Point2::new((TAU / n * i).sin(), (TAU / n * i).cos()) * 75.0,
            Vector2::new((TAU / n * i + PI / 2.0).sin(), (TAU / n * i + PI / 2.0).cos()) * 10.0,
            radius * radius * radius,
            radius,
        ));
    }

    event_loop.run(move |event, _, control_flow| {
        if input.update(&event) {
            let full = Instant::now();
            if input.key_pressed(VirtualKeyCode::Escape) || input.close_requested() || input.destroyed() {
                *control_flow = ControlFlow::Exit;
                return;
            }

            if input.key_pressed(VirtualKeyCode::Space) {
                println!("{}", world.total_energy());
            }

            let now = Instant::now();
            world.update_substeps(0.1, 8);
            println!("Update: {:?}", now.elapsed());

            let frame = pixels.get_frame_mut();

            let now = Instant::now();
            frame.fill(0x00);
            println!("Clear: {:?}", now.elapsed());

            let now = Instant::now();
            let boundary = world.calculate_boundary().pad(5.0);
            draw_particles(&world, frame, boundary);
            println!("Draw: {:?}", now.elapsed());

            if let Err(err) = pixels.render() {
                error!("pixels.render() failed: {}", err);
                *control_flow = ControlFlow::Exit;
                return;
            }

            println!("Full: {:?}", full.elapsed());
        }
    });
}

fn draw_particles(world: &World, frame: &mut [u8], boundary: Boundary) {
    let screen_boundary = Boundary::new(Point2::new(0.0, 0.0), Point2::new(WIDTH as f64, HEIGHT as f64));

    world.bodies.iter().filter(|&body| {
        boundary.contains(body.pos)
    }).map(|body| {
        (boundary.map_point(body.pos, &screen_boundary), boundary.map_distance(body.radius, &screen_boundary))
    }).for_each(|(pos, radius)| {
        draw_circle(pos.x as usize, pos.y as usize, radius, 0xffffffff, false, frame);
    });
}

fn draw_circle(x: usize, y: usize, r: f64, color: u32, fill: bool, frame: &mut [u8]) {
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

    fn filled(x: usize, y: usize, i: usize, j: usize, color: u32, frame: &mut [u8]) {
        horizontal_line(x - i, x + i, y + j, color, frame);
        horizontal_line(x - i, x + i, y - j, color, frame);
        horizontal_line(x - j, x + j, y + i, color, frame);
        horizontal_line(x - j, x + j, y - i, color, frame);
    }

    let mut i = 0.0;
    let mut j = r;
    let mut d = 3.0 - (2.0 * r);

    if fill {
        filled(x, y, i as usize, j as usize, color, frame);
    } else {
        not_filled(x, y, i as usize, j as usize, color, frame);
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
            filled(x, y, i as usize, j as usize, color, frame);
        } else {
            not_filled(x, y, i as usize, j as usize, color, frame);
        }
    }
}

fn horizontal_line(x0: usize, x1: usize, y: usize, color: u32, frame: &mut [u8]) {
    for x in x0..x1 {
        set_pixel(x, y, color, frame);
    }
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
