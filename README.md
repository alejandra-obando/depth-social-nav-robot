# depth-social-nav-robot

Social navigation robot with a depth camera, built for the final exam (third term) of Neurocontrol 2.

## Challenge

The robot shares a 7 m × 7 m room with people and obstacles and travels along its diagonal:

- **Phase 1 – Follow:** the robot follows a person from the start corner to the opposite corner, staying within 1.5 m of them.
- **Phase 2 – Guide:** the robot guides the person back to the start corner, checking that they keep following it.

The robot keeps the link with its person through an image or QR sign that the person wears. It must avoid static and moving obstacles, handle people crossing its path, and recover the link if the person is briefly hidden.

## Status

Work in progress.

## Repository structure

| Folder | Contents |
|--------|----------|
| `hardware/` | CAD, electronics and bill of materials |
| `software/` | ROS 2 workspace that runs on the robot |
| `neural/` | Hand-written neural models and their experiments |
| `simulation/` | Simulated room for testing |
| `docs/` | Rules, diagrams and presentation |
