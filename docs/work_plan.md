# Work plan

Exam: **Monday 30 November 2026**. Each stage is tested on its own before it is combined with the others.

## Who does what

| Who | Works on |
|-----|----------|
| **Maleja** | Simulation of the room and robot, and the neural architecture tested in simulation. |
| **Alejandra and the real-robot team** | Hardware, conventional control on the real robot, then the neural architecture on the real robot. |

## Stage 0 – Hardware (now)

- [ ] **Buy the USB-C to USB-A USB 3 cable for the D415** ([guide](../hardware/electronics/usb_cable_d415.md)). Nothing with the depth camera can be tested without it.
- [ ] Mount the D415 at the front and the C270 at the rear; write down their height and tilt in `hardware/`.
- [ ] Print the person's signs (front and back) and choose their size and height so both cameras read them between 0.5 m and 1.5 m.
- [ ] Check the Pi 4 is not under-voltage with both cameras connected (`vcgencmd get_throttled` must return `0x0`).

## Stage 1 – Conventional control on the real robot (real-robot team)

Classic control first, so there is a working robot on exam day no matter what.

1. **Perception:** detect the sign in the D415 image (front) and in the C270 image (rear); get the person's bearing and distance.
2. **Follow (phase 1):** proportional control of linear speed on distance (target ~1.0 m, never above 1.5 m) and of angular speed on bearing.
3. **Obstacle avoidance:** stop or turn using the D415 depth image (low obstacles) and the LiDAR (legs, walls, briefcases).
4. **Corner recognition** and **visual signal** at the end of phase 1.
5. **Guide (phase 2):** drive back to the stored start corner (odometry + LiDAR) while the C270 checks the person is behind; wait if they fall back.
6. **Link loss:** if the sign disappears, stop and search before moving on.

## Stage 2 – Neural architecture in simulation (Maleja)

Build the simulated 7 m × 7 m room (obstacles, walking people, a person wearing the sign) in `simulation/`
and test the neural models in `neural/` there, one at a time. Candidates:

- **Winner-take-all** to choose which detected person/sign to follow and ignore people crossing.
- **Leaky integrator** to keep the link for a few seconds when the person is hidden.
- **Motion detection** to notice moving people and briefcases.

Models are hand-written, without machine learning libraries, and each one has its own experiment in `neural/experiments/`.

## Stage 3 – Neural architecture on the real robot (real-robot team)

Replace pieces of the conventional control with the neural models that worked in simulation, **one piece at a time**,
and keep the conventional version as a fallback if a piece does not work on the real robot.

## Stage 4 – Rehearsal and presentation

- [ ] Full run of both phases in a room similar to the exam (stacks of 4 tablets, people walking).
- [ ] Time it: it must fit in 20 minutes.
- [ ] 10-minute slides: robot, architecture, which parts use neural structures, results.
