# Simulation

Simulated 7 m × 7 m room with obstacles and people (MuJoCo / Isaac Sim).

## Robot model with both cameras

`xml 2 cameras set.xml` is the robot exported from the Onshape CAD, with the front Intel RealSense D415
(depth 65° × 40°, color 69° × 42°) and the rear Logitech C270 (~48.8° × 28.6°). Meshes are in `assets/`.

```bash
pip install mujoco
python -m mujoco.viewer --mjcf="simulation/xml 2 cameras set.xml"
```

- `]` / `[` switch to the camera views (`d415_prof`, `d415_color`, `c270`).
- Key `3` shows the 1.5 m view frustums.
- Wheel velocity motors `motor_izq`, `motor_der` (rad/s); forward = `motor_izq` negative, `motor_der` positive.
