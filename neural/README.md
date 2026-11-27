# Neural

Hand-written neural models, without machine learning libraries (only NumPy). Each model is tested on its
own before it is used by the robot software.

- `models/` – the neural structures used by the robot, and `social_nav_net.py`, which connects them.
- `experiments/` – standalone tests and plots of each model.

Architecture diagram and the reasoning behind each block: [`../docs/neural_architecture.md`](../docs/neural_architecture.md).
References with links: [`../docs/references.md`](../docs/references.md).

## Using the architecture

`SocialNavNet` does not depend on MuJoCo or ROS. Every control step (~20 Hz) it receives plain numbers and
returns the wheel command, so the same class runs in the simulation and, later, inside a ROS 2 node:

```python
from neural.models.social_nav_net import SensorFrame, SocialNavNet

net = SocialNavNet()
cmd = net.step(SensorFrame(
    scan_angles=..., scan_ranges=...,        # LiDAR, robot frame (x forward, y left), rad / m
    depth_angles=..., depth_ranges=...,      # D415 obstacle points projected on the floor
    front_signs=[(bearing, distance, score)],  # D415 colour: the person's back sign
    rear_signs=[(bearing, distance, score)],   # C270: the person's front sign
    odom=(x, y, yaw),                        # odometry, origin at the start corner
), dt=0.05)
cmd.v, cmd.omega, cmd.signal, cmd.behaviour, cmd.phase
```

## Running the experiments

```bash
.venv/bin/python -m neural.experiments.run_all          # all of them, PASS/FAIL per check
.venv/bin/python -m neural.experiments.exp_neural_field # one of them
```

Plots are written to `experiments/results/`. Current status: 6 of 7 experiments pass all their checks;
`exp_motion_detector` fails the ego-motion check (see below).

## Open points to review

1. **Reichardt detector (motion):** it detects people crossing and their direction, but the compensation of
   the robot's own motion does not work yet (object edges are sampled at different spots in consecutive
   scans), and in the simulated scenarios it does not improve the results. Options: keep it, replace it with
   a looming (LGMD-type) neuron, or drop it.
2. **Gap closing and obstacle clusters** in `attractor_dynamics.py` are engineering steps needed to get the
   behaviour of Schöner et al. with a fine (5°) sensor; they are explained in the module docstring.
3. **Corner detector** is an engineered template, not a model from the literature.
4. All parameters were tuned in simulation; on the real robot at least `d_ref`, `v_max`, the link time
   constants and the sign heights must be checked.
