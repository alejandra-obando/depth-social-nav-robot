# Bringup

ROS 2 side of the conventional controller.

| File | Contents |
|------|----------|
| `bringup/social_nav_node.py` | Converts camera images, `scan` and `odometry/filtered` into the inputs of `navigation.Navigator` and publishes `cmd_vel` (through the second-term LiDAR brake), `social_nav/state` and `social_nav/alert`; pauses on `parada`. |
| `bringup/frames.py` | ROS-free conversions of the node: LiDAR scan to robot frame (same as `freno_lidar`), start frame that survives an EKF restart. |
| `bringup/speaker_node.py` | Says the alerts with `aplay` (recorded phrase, or a beep if missing); same phrase at most every 6 s; repeats in LOST and WAIT_PERSON. |
| `bringup/sensor_check.py`, `checks.py` | Prints what one sensor gives the controller (front / rear tags, depth obstacles, LiDAR and corners), for the step-by-step tests. |
| `bringup/sounds.py` | The eight phrases (Spanish), their fallback beeps, and `make_voice` (espeak-ng). |
| `launch/social_nav.launch.py` | D415 and C270 drivers, both nodes; optionally the second-term base robot (`base:=true`). |
| `config/social_nav.yaml` | Every parameter: topics, camera and LiDAR poses, tag IDs and size, controller values. |
| `test/` | Sounds and frame conversions (no ROS needed). |

How to build, check the drivers and run: [`../../README.md`](../../README.md).
