# Perception

Plain Python (no ROS imports), used by `navigation` and by the MuJoCo simulation.

| File | Contents |
|------|----------|
| `camera.py` | Camera intrinsics and the camera's pose on the robot (optical frame → robot frame). |
| `apriltag_detector.py` | `TagDetector`: AprilTag 3 (`pupil-apriltags`, family tag36h11) → ID, position, distance and bearing of each tag in the robot frame. |
| `person_tracker.py` | `PersonTracker`: constant-velocity Kalman filter of the person in the odometry frame, with gating of wrong detections. |
| `depth_obstacles.py` | `DepthObstacles`: D415 depth image → obstacle points between 3 and 45 cm above the floor. |
| `obstacle_map.py` | `ObstacleMap`: current LiDAR scan + depth points remembered for 20 s (replaced when the camera sees the place again), without the tracked person. |

Tests: `test/test_perception.py` (`cd software/src && python -m pytest`).
