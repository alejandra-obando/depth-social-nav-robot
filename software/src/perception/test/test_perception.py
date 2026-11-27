import numpy as np

from perception.camera import Intrinsics, Mount
from perception.depth_obstacles import DepthObstacles
from perception.obstacle_map import ObstacleMap
from perception.person_tracker import PersonTracker
from perception.apriltag_detector import TagObservation


def test_front_camera_looks_forward():
    m = Mount.from_angles(0.16, 0.0, 0.1)
    # A point 1 m in front of the lens (optical z) is 1 m ahead of the camera in the robot frame.
    assert np.allclose(m.to_robot([0, 0, 1.0]), [1.16, 0.0, 0.1])
    # Optical x (right in the image) is the robot's -y.
    assert np.allclose(m.to_robot([1.0, 0, 0]) - m.position, [0, -1, 0])


def test_rear_camera_tilted_up():
    m = Mount.from_angles(-0.15, 0.0, 0.15, yaw=np.pi, pitch=np.radians(45))
    p = m.to_robot([0, 0, 1.0]) - m.position
    assert np.allclose(p, [-np.cos(np.pi / 4), 0, np.sin(np.pi / 4)], atol=1e-9)


def test_intrinsics_from_fov():
    k = Intrinsics.from_fov(640, 360, 42.0)
    assert np.isclose(k.fy, 180 / np.tan(np.radians(21)))
    assert (k.cx, k.cy) == (320, 180)


def _obs(x, y):
    return TagObservation(0, "front", x, y, 0.3, 100.0)


def test_tracker_follows_and_predicts():
    tr = PersonTracker()
    pose = (0.0, 0.0, 0.0)
    for k in range(40):                      # person walking at 0.3 m/s along x
        tr.predict(0.1)
        tr.update(pose, _obs(1.0 + 0.03 * k, 0.0))
    for _ in range(5):                       # hidden for 0.5 s
        tr.predict(0.1)
    p = tr.position()
    assert abs(p[0] - (1.0 + 0.03 * 44)) < 0.05
    assert abs(tr.radial_speed(pose) - 0.3) < 0.05


def test_tracker_rejects_outlier():
    tr = PersonTracker()
    for _ in range(10):
        tr.predict(0.1)
        tr.update((0, 0, 0), _obs(1.0, 0.0))
    tr.predict(0.1)
    assert not tr.update((0, 0, 0), _obs(1.0, 1.5))     # a jump of 1.5 m in 0.1 s
    assert tr.rejected == 1


def test_tracker_uses_robot_pose():
    tr = PersonTracker()
    tr.predict(0.1)
    tr.update((1.0, 2.0, np.pi / 2), _obs(1.0, 0.0))   # robot facing +y sees the person 1 m ahead
    assert np.allclose(tr.position(), [1.0, 3.0])
    d, b = tr.relative((1.0, 2.0, np.pi / 2))
    assert np.isclose(d, 1.0) and abs(b) < 1e-9


def test_depth_sees_box_not_floor():
    k = Intrinsics.from_fov(80, 48, 40.0)
    mount = Mount.from_angles(0.16, 0.0, 0.07)
    det = DepthObstacles(k, mount, stride=1)
    v = np.arange(48)[:, None].repeat(80, 1)
    # Flat floor seen from 7 cm: depth along each ray down to z = 0.
    ray_down = (v - k.cy) / k.fy
    floor = np.where(ray_down > 0, 0.07 / np.maximum(ray_down, 1e-9), 0.0)
    a, r = det.points(floor)
    assert len(a) == 0
    # A 10 cm high box face 0.6 m in front of the camera.
    box = floor.copy()
    rows = (0.07 - ray_down * 0.6 > 0.0) & (0.07 - ray_down * 0.6 < 0.10)
    box[rows] = np.where(box[rows] > 0.6, 0.6, box[rows])
    box[rows & (box == 0)] = 0.6
    a, r = det.points(box)
    assert len(a) > 0 and np.all(np.abs(r - 0.76) < 0.1)


def test_obstacle_memory_keeps_points_out_of_view():
    m = ObstacleMap(depth_memory=4.0)        # short memory to test forgetting
    m.add_depth((0, 0, 0), np.array([0.0]), np.array([1.0]))      # stack 1 m ahead
    # The robot turns 90 deg left: the stack leaves the camera's view but is remembered.
    m.step(0.5)
    m.add_depth((0, 0, np.pi / 2), np.array([]), np.array([]))
    pts = m.points((0, 0, np.pi / 2))
    assert len(pts) == 1 and np.allclose(pts[0], [0.0, -1.0], atol=0.03)
    # Back facing it, the camera sees nothing there any more: the point is dropped.
    m.add_depth((0, 0, 0), np.array([]), np.array([]))
    assert len(m.points((0, 0, 0))) == 0
    # And everything is forgotten after depth_memory seconds.
    m.add_depth((0, 0, 0), np.array([1.5]), np.array([0.5]))
    m.step(4.1)
    assert len(m.points((0, 0, 0))) == 0


def test_obstacle_map_ignores_the_person():
    m = ObstacleMap(person_radius=0.4)
    m.add_scan(np.array([0.0, np.pi / 2]), np.array([1.0, 1.0]))
    pts = m.points((0, 0, 0), person_xy=(1.1, 0.0))
    assert len(pts) == 1 and np.allclose(pts[0], [0, 1.0], atol=1e-9)


def test_detector_measures_a_synthetic_tag_and_survives_recreation():
    """A tag36h11 (ID 1, 16 cm) drawn 1 m in front of a pinhole camera.

    Creating and destroying detectors used to crash pupil-apriltags (segfault); the
    detectors are now shared, so this loop must finish.
    """
    import gc

    import pytest
    cv2 = pytest.importorskip("cv2")
    pytest.importorskip("pupil_apriltags")
    from perception.apriltag_detector import TagDetector

    k = Intrinsics.from_fov(640, 360, 42.0)
    side_px = int(round(k.fx * 0.16 / 1.0))                  # 16 cm seen from 1 m
    tag = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
                                        1, side_px)
    img = np.full((360, 640), 255, np.uint8)
    y0, x0 = 180 - side_px // 2, 320 - side_px // 2
    img[y0:y0 + side_px, x0:x0 + side_px] = tag
    mount = Mount.from_angles(0.0, 0.0, 0.0)
    for _ in range(50):
        a, b = TagDetector("front", k, mount), TagDetector("rear", k, mount)
        obs = a.detect(img) + b.detect(img)
        del a, b
        gc.collect()
    assert len(obs) == 2 and obs[0].tag_id == 1
    assert abs(obs[0].x - 1.0) < 0.02 and abs(obs[0].y) < 0.02


def test_obstacle_memory_skips_what_the_lidar_sees():
    m = ObstacleMap()
    m.add_scan(np.array([0.0]), np.array([1.0]))             # a person's legs 1 m ahead
    # Depth sees the person's feet (1 m ahead) and a low stack the LiDAR misses (0.8 m, 30 deg left).
    m.add_depth((0, 0, 0), np.array([0.0, np.radians(30)]), np.array([1.0, 0.8]))
    m.add_scan(np.array([]), np.array([]))                    # the person walks away
    pts = m.points((0, 0, 0))
    assert len(pts) == 1 and np.allclose(pts[0], [0.8 * np.cos(np.radians(30)), 0.4], atol=0.03)
