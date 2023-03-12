mod physics;
mod renderer;

use crate::{
    physics::{
        body::Body, 
        world::World, 
        boundary::Boundary, quadtree::{Quadtree, QuadBoundary},
    }, 
    renderer::{
        Renderer,
    },
};

use std::{f32::consts::{TAU, PI}, time::Instant};

use nalgebra::Vector2;

use rand::prelude::*;

use renderer::view::ViewState;
use winit::{
    dpi::PhysicalSize,
    event::{VirtualKeyCode, Event},
    event_loop::{ControlFlow, EventLoop},
    window::WindowBuilder,
};

use winit_input_helper::WinitInputHelper;

const WIDTH: u32 = 800;
const HEIGHT: u32 = 800;

const WINDOW_BOUNDARY: Boundary = Boundary::new(Vector2::new(0.0, 0.0), Vector2::new(WIDTH as f32, HEIGHT as f32));

fn main() {
    let boundary = QuadBoundary::new(Vector2::zeros(), 1.0);
    let mut quadtree = Quadtree::new(boundary);

    let bodies = vec![
        Body::new(Vector2::new(0.5, 0.5), Vector2::zeros(), 10.0, 0.1),
    ];

    quadtree.insert(&bodies);

    puffin::set_scopes_on(true); // Enable puffin (profiler)

    let event_loop = EventLoop::new();
    let mut input = WinitInputHelper::new();
    
    let window = WindowBuilder::new()
        .with_title("Particles >:)")
        .with_inner_size(PhysicalSize::new(WIDTH as u32, HEIGHT as u32))
        .build(&event_loop)
        .unwrap();

    let mut world = World::new(1.0);
    
    let mut renderer = Renderer::new(window, &event_loop);

    world.insert(Body::new(
        Vector2::zeros(),
        Vector2::zeros(),
        10000000.0,
        50.0,
    ));

    const N: u32 = 100;
    let mut rng = rand::thread_rng();
    for i in 0..N {
        let i = i as f32;
        let n = N as f32;

        let distance = rng.gen_range(0.25..1.0);
        let mut radius: f32 = rng.gen_range(0.0..1.0);
        radius = radius.powi(3) * 10.0 + 10.0;
        world.insert(Body::new(
            Vector2::new((TAU / n * i).sin(), (TAU / n * i).cos()) * 1000.0 * distance,
            Vector2::new((TAU / n * i + PI * 0.5).sin(), (TAU / n * i + PI * 0.5).cos()) * 100.0 / distance.cbrt(),
            radius * radius * radius * 10.0,
            radius,
        ));
    }

    let time = Instant::now();
    let mut frames = 0;

    let mut paused = false;
    let mut quadtree = false;

    event_loop.run(move |event, _, control_flow| {
        if input.update(&event) {
            puffin::GlobalProfiler::lock().new_frame(); // call once per frame!

            if input.key_pressed(VirtualKeyCode::Escape) || input.quit() {
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

            if !paused {
                world.update(0.01, 64);
            }

            renderer.window.request_redraw();

            frames += 1;
        }
    
        match event {
            Event::WindowEvent { event, .. } => {
                // Update egui inputs
                renderer.framework.handle_event(&event);
            }
            Event::RedrawRequested(_) => {
                puffin::profile_scope!("render");
                renderer.start_frame(&input, &world);

                renderer.draw_world(&world);
        
                if quadtree {
                    let quadtree = world.calculate_quadtree();
        
                    match renderer.view.state {
                        ViewState::Free | ViewState::Maximized => {
                            renderer.draw_quadtree(&quadtree);
                        },
                        ViewState::Follow(body, _) => {
                            let position: Vector2<f32> = world.bodies[body].pos;
                            renderer.draw_relative_quadtree(&quadtree, position, world.theta);
                        },
                    }
                }

                renderer.end_frame();
            }
            _ => (),
        }
    });
}