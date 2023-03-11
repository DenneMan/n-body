# n-body

The aim of this project is to be able to simulate as many 2d bodies (i.e. circles) as possible. Therefore I am not to concerned with accuracy as long as it looks plausible.

### Steps to achieve this goal:

- [x] Barnes-hut algorithm for calculation of acceleration.

- [x] Grid space partitioning for collision detection.

- [x] Fast inverse square root from Quake III Arena.

- [ ] Multithread.

- [ ] GPU compute.

### Notes:
The gravitational constant is 1. Otherwise masses (or timestep) would have to be insanely high to see anything happen. By using a "fake" gravitational constant we can reduce the mass required.

### Controls in the main example
- Press Q to toggle rendering of quadtree.
- Press M to toggle a view where all bodies are garantied to be visible.
- Press Space to toggle pausing of the simulation.
- Left click on a body to focus/track said body.
- Middle click and drag to move around.
- Scroll to zoom to where the cursor is places.
