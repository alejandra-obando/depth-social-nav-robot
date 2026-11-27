# Bringup

ROS 2 side of the conventional controller.

| File | Contents |
|------|----------|
| `bringup/social_nav_node.py` | Converts camera images, `scan` and `odometry/filtered` into the inputs of `navigation.Navigator` and publishes `cmd_vel` (through the second-term LiDAR brake), `social_nav/state` and `social_nav/alert`; pauses on `parada`. |
| `bringup/frames.py` | ROS-free conversions of the node: LiDAR scan to robot frame (same as `freno_lidar`), start frame that survives an EKF restart. |
| `bringup/speaker_node.py` | Plays the alerts on the USB speaker with `aplay`; repeats the beep while in LOST or WAIT_PERSON. |
| `bringup/sounds.py` | The four alert sounds, generated as WAV files. |
| `launch/social_nav.launch.py` | D415 and C270 drivers, both nodes; optionally the second-term base robot (`base:=true`). |
| `config/social_nav.yaml` | Every parameter: topics, camera and LiDAR poses, tag IDs and size, controller values. |
| `test/` | Sounds and frame conversions (no ROS needed). |

How to build, check the drivers and run: [`../../README.md`](../../README.md).
