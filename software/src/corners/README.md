# Corners

Recognition of the room corners with the LiDAR. Plain Python (no ROS imports).

| File | Contents |
|------|----------|
| `line_extraction.py` | `extract_segments`: wall segments from a scan with split-and-merge (Nguyen et al. 2007). |
| `corner_detector.py` | `find_corners`: two long, nearly perpendicular walls meeting near their ends, with the robot inside the angle (rejects briefcases and other convex boxes). `CornerConfirmer`: the corner must be seen in several consecutive scans, close to the robot and near where the odometry expects it. |

The far corner ends phase 1; the start corner corrects the odometry at the end of phase 2.

Tests: `test/test_corners.py`.
