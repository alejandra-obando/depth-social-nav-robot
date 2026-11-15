# Software

ROS 2 workspace with the code that runs on the robot. Packages live in `src/`.

- `perception/` – depth camera, sign/QR detection and person tracking.
- `navigation/` – following (phase 1), guiding (phase 2) and obstacle avoidance.
- `corners/` – corner recognition with the LiDAR or a visual marker.
- `bringup/` – launch files, parameters and teleoperation.

The neural models in `../neural` are imported by these packages.
