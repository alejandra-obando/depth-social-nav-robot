# depth-social-nav-robot

Social navigation robot with a depth camera, built for the final exam (third term) of Neurocontrol 2.
**Exam day: Monday 30 November 2026.**

## Challenge

The robot shares a 7 m × 7 m room with people and obstacles and travels along its diagonal:

- **Phase 1 – Follow:** the robot follows a person from the start corner to the opposite corner, staying within 1.5 m of them.
- **Phase 2 – Guide:** the robot guides the person back to the start corner, checking that they keep following it.

The robot keeps the link with its person through an image or QR sign that the person wears on the front **and** the back.
It must avoid static and moving obstacles, handle people crossing its path, and recover the link if the person is briefly hidden.
Full rules, faults and grading: [`docs/exam_rules.md`](docs/exam_rules.md).

## The robot

We reuse the differential robot from the second term (Raspberry Pi 4, ROS 2 Jazzy, two Dynamixel XM430 wheels, LD19 LiDAR, BNO055 IMU)
and add two cameras:

| Camera | Where | What it is for |
|--------|-------|----------------|
| **Intel RealSense D415** (depth) | Front | Phase 1: sees the person's front sign and measures the distance to them. Both phases: sees low obstacles (10 cm tablet stacks) that the LiDAR misses because it is mounted at ~16 cm. |
| **Logitech C270** (webcam) | Rear | Phase 2: the person walks **behind** the robot, so this camera checks that their sign is still there and how far they are. |

Parts list: [`hardware/bill_of_materials.md`](hardware/bill_of_materials.md).

> ⚠️ **Urgent purchase:** the D415 came with a USB-C to USB-C cable, but the Raspberry Pi 4 only has USB-A ports.
> We need a USB-C to USB-A **USB 3** cable. What to look for: [`hardware/electronics/usb_cable_d415.md`](hardware/electronics/usb_cable_d415.md).

## Team and work split

| Who | Works on |
|-----|----------|
| **Maleja** | Simulation (`simulation/`) of the room, the robot and the **neural architecture** (`neural/`). |
| **Alejandra and the real-robot team** | First the **conventional control** on the real robot (`software/`), then the **implementation of the neural architecture** on the real robot. |

Work stages and deadlines: [`docs/work_plan.md`](docs/work_plan.md).

## Status

- [x] Repository structure
- [ ] USB-C to USB-A cable for the D415 (buy now)
- [ ] Camera mounts (D415 front, C270 rear)
- [ ] Conventional control on the real robot
- [ ] Neural architecture in simulation
- [ ] Neural architecture on the real robot
- [ ] Full rehearsal of both phases and presentation

## Repository structure

| Folder | Contents |
|--------|----------|
| `hardware/` | CAD, electronics and bill of materials |
| `software/` | ROS 2 workspace that runs on the robot |
| `neural/` | Hand-written neural models and their experiments |
| `simulation/` | Simulated room for testing |
| `docs/` | Rules, work plan, diagrams and presentation |
