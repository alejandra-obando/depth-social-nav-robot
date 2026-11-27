# Software

ROS 2 Jazzy workspace with the **conventional controller** that runs on the robot. Design, states and the
paper behind each block: [`../docs/conventional_control.md`](../docs/conventional_control.md).

| Package | Contents | Depends on ROS? |
|---------|----------|-----------------|
| [`perception/`](src/perception) | AprilTag detection (tag36h11), person tracking with a Kalman filter, obstacle points from the LiDAR and the depth camera | no |
| [`corners/`](src/corners) | Wall segments (split-and-merge) and room-corner recognition from the LiDAR | no |
| [`navigation/`](src/navigation) | State machine, follow and go-to-goal controllers, VFH obstacle avoidance, differential-drive kinematics, and `Navigator`, which joins everything | no |
| [`bringup/`](src/bringup) | ROS 2 nodes (`social_nav_node`, `speaker_node`), launch file and parameters | yes |

Also here: `scripts/` (deploy from the PC, install on the Pi), `systemd/` (service `social-nav`, sudoers rule),
`config/` (udev rule for the rear C270).

The first three are plain Python, so the MuJoCo simulation (`../simulation/run_conventional.py`) runs exactly
the code the robot runs, and they can be tested on any computer.

## Tests (any computer)

```bash
pip install numpy pupil-apriltags pytest
cd software/src && python -m pytest
```

## On the robot

The controller runs on the Pi of the second term, **on top of its base robot** (repository
`wallens-fruit-picker-robot`, systemd service `wallens`, `launch/robot.launch.py`). We reuse it as it is, and
add only what that robot did not have:

```
                       social-nav.service (this repository, started by hand for each run)
 D415 (realsense2_camera) ──┐
 C270 /dev/cam_rear (v4l2) ─┼─> social_nav_node ──> cmd_vel ──┐          speaker_node <── social_nav/alert
                            │        ▲   ▲                    │
 ───────────────────────────│────────│───│────────────────────│───────────────────────────────────────────
                            │  scan  │   │ odometry/filtered  ▼   wallens.service (second term, at boot)
                 LD19 ──────┴────────┘   EKF <── /odom + /bno055/imu     freno_lidar ──> cmd_vel_safe ──> wallens_driver
```

- **`cmd_vel` always goes through the LiDAR brake** (`freno_lidar` → `cmd_vel_safe` → driver). The driver also
  caps the speed (0.19 m/s, 1 rad/s) and stops 0.25 s after the last command, or while `/parada` is true.
- **Pose:** `odometry/filtered` (EKF: wheels + BNO055 gyro), not the raw `/odom`. If the EKF is respawned it
  restarts at 0; the node notices the jump and keeps the pose continuous.
- **LiDAR:** the LD19 node of `robot.launch.py` (frame `base_laser`, 89 mm in front of the axle, yaw −90°). The
  node uses the same conversion as `freno_lidar` (`lidar.*` in the YAML must match `wallens_safety.yaml`).
- **`/parada`** stops everything, as in the second term; the controller also pauses its timers while it is set.
- **Network:** the same `config/ros-red.env` of the wallens repository (`ROS_DOMAIN_ID=37`, discovery only on
  the Pi plus `ROS_STATIC_PEERS`).
- Code style taken from the second term: ROS-free logic plus a node that only translates messages
  (like `mision_logica.py` / `mision_frutas.py`), relative topic names, respawn for the drivers, no respawn for
  the node that moves the robot, Ctrl+C publishes a zero `cmd_vel` before exiting.

### 1. Install (once, on the Pi)

The Pi already has the wallens installation (its `scripts/00`…`05`, the `wallens` service active). From the PC:

```bash
WALLENS_IP=<pi ip> software/scripts/deploy.sh --no-tests    # copies software/ to ~/social_nav (first build fails: no deps yet)
ssh wallens@<pi ip>
sudo ~/social_nav/scripts/install-pi.sh deps     # realsense2_camera, v4l2_camera, aplay, pupil-apriltags, udev /dev/cam_rear,
                                                 # systemd unit social-nav, sudoers rule
~/social_nav/scripts/install-pi.sh               # links the 4 packages into ~/ros_ws/src and builds them
```

If `wallens_safety`'s footprint still includes the gripper and it was taken off, update `footprint` in
`wallens_safety.yaml` (and `nav2.yaml`, its test compares them), or the brake will stop for things that are not
there.

### 2. Deploy changes (PC)

```bash
software/scripts/deploy.sh          # tests -> rsync -> colcon build -> restarts social-nav if it is running
```

### 3. Check each part on its own (Pi, `source ~/wallens/scripts/ros-env.sh`)

```bash
systemctl is-active wallens                                   # base robot
ros2 topic hz /scan; ros2 topic hz /odometry/filtered         # >= 5 and >= 20 Hz (the wallens start-up check too)
ros2 launch bringup social_nav.launch.py --show-args        # arguments: cameras, base, params, rear_device
ros2 run realsense2_camera realsense2_camera_node --ros-args -r __node:=d415   # D415 alone
ros2 run v4l2_camera v4l2_camera_node --ros-args -r __ns:=/c270 -p video_device:=/dev/cam_rear
ros2 topic hz /d415/color/image_raw; ros2 topic hz /c270/image_raw
aplay -L                                                       # name of the USB speaker -> speaker_node.device
```

### 4. Calibrate and measure (before the exam)

- **Calibrate both cameras** (`ros2 run camera_calibration cameracalibrator ...`). Without calibration the
  node falls back to the nominal field of view and the tag distances are less accurate.
- **Measure the camera poses** from `base_link` (middle of the wheel axle; heights in the YAML are from the
  floor) and write them in the YAML (`front_camera`, `rear_camera`, `depth_camera`).
- **LiDAR:** keep `lidar.*` equal to the TF `base_link -> base_laser` of `robot.launch.py`.
- **Print the tags:** tag36h11, **ID 0 on the back, ID 1 on the chest**, black square **16 cm** (`tag_size`)
  with a white margin around it. Back tag low (~0.30 m, the D415 looks almost horizontally), chest tag at
  ~1.10 m (the C270 looks ~47° up). See `../simulation/README.md` for why.
- Measure `start_to_corner`: distance from the robot's start position to the start corner.

### 5. Run

Place the robot at the start corner facing the opposite corner (the first odometry pose is the start, with the
far corner straight ahead). On the Pi, or from the PC with `ssh`:

```bash
sudo systemctl start social-nav        # cameras + controller + speaker; never starts at boot
journalctl -u social-nav -f
ros2 topic echo /social_nav/state      # from any shell with ros-env.sh
sudo systemctl stop social-nav         # zero cmd_vel and exit
```

Or by hand: `ros2 launch bringup social_nav.launch.py` (Ctrl+C stops the robot). `base:=true` also starts
`robot.launch.py`, only with the `wallens` service stopped (two drivers on one bus).

| Sound | When |
|-------|------|
| two low beeps, every 2 s | LOST: the robot no longer sees the person (phase 1) |
| one beep, every 2 s | WAIT_PERSON: the person fell behind or is not seen (phase 2) |
| three rising notes | far corner recognised, end of phase 1 |
| long high tone | start corner reached, end of phase 2 |

The neural models in `../neural` can later replace pieces of this controller one at a time; the conventional
version stays as the fallback.
