"""ROS 2 node that runs the conventional controller on the robot.

It only converts ROS messages to the plain inputs of ``navigation.navigator.Navigator``
and its output back to ``cmd_vel``; all the logic is in the ROS-free packages, the
same code that the MuJoCo simulation tests.  This is the pattern of the second-term
mission (``wallens_mision``: ROS-free state machine + a node that only translates topics).

It runs on top of the base robot of the second term (systemd service ``wallens``,
``launch/robot.launch.py`` of the wallens repository), which already provides:

    cmd_vel -> freno_lidar -> cmd_vel_safe -> wallens_driver -> wheels     (LiDAR brake always in between)
    /odom + /bno055/imu -> EKF -> odometry/filtered
    /scan (LD19, frame base_laser: x 0.089 m, yaw -90 deg: an object ahead is read at 90 deg)
    parada (std_msgs/Bool, transient_local): the driver stops while it is true

Subscribes (names are parameters, see config/social_nav.yaml)
    front image + camera_info   D415 colour  -> back tag (ID 0)
    rear image + camera_info    C270         -> chest tag (ID 1)
    depth image + camera_info   D415 depth   -> low obstacles
    scan                 sensor_msgs/LaserScan
    odometry/filtered    nav_msgs/Odometry    EKF of wheels + IMU
    parada               std_msgs/Bool        the controller pauses while it is true

Publishes
    cmd_vel             geometry_msgs/Twist   goes through freno_lidar before the wheels
    social_nav/state    std_msgs/String       current state, every control period (transient_local)
    social_nav/alert    std_msgs/String       sound to play when a state is entered (speaker_node)
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Bool, String

from navigation.navigator import Navigator, Observation, Params
from perception.apriltag_detector import TagDetector
from perception.camera import Intrinsics, Mount
from perception.depth_obstacles import DepthObstacles

from .frames import StartFrame, scan_points

# Mount (x, y, z, yaw, pitch up) and nominal vertical FOV of each camera.  x, y from the
# middle of the wheel axle (base_link), z from the FLOOR (base_link is 36.25 mm above it).
# Taken from the MuJoCo model: measure them on the real robot and set them in the YAML.
CAMERAS = {"front_camera": (0.163, 0.0, 0.098, 0.0, 0.084, 42.0),
           "rear_camera": (-0.147, 0.0, 0.151, np.pi, 0.82, 28.6),
           "depth_camera": (0.163, 0.0, 0.072, 0.0, 0.084, 40.0)}
TOPICS = {"front_image": "d415/color/image_raw", "front_info": "d415/color/camera_info",
          "rear_image": "c270/image_raw", "rear_info": "c270/camera_info",
          "depth_image": "d415/depth/image_rect_raw", "depth_info": "d415/depth/camera_info",
          "scan": "scan", "odom": "odometry/filtered", "cmd_vel": "cmd_vel", "parada": "parada"}

def image_to_gray(msg: Image):
    """sensor_msgs/Image -> H x W uint8 grey image (without cv_bridge)."""
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    enc = msg.encoding.lower()
    if enc in ("mono8", "8uc1"):
        return buf[:, :msg.width]
    if enc in ("rgb8", "bgr8", "rgba8", "bgra8"):
        ch = 4 if enc.endswith("a8") else 3
        px = buf[:, :msg.width * ch].reshape(msg.height, msg.width, ch)[:, :, :3].astype(np.float32)
        w = np.array([0.299, 0.587, 0.114] if enc.startswith("rgb") else [0.114, 0.587, 0.299], np.float32)
        return (px @ w).astype(np.uint8)
    if enc in ("yuyv", "yuv422_yuy2"):
        return buf[:, 0:msg.width * 2:2]       # the Y (luma) bytes
    if enc in ("uyvy", "yuv422"):
        return buf[:, 1:msg.width * 2:2]
    raise ValueError(f"unsupported image encoding {msg.encoding}")


def depth_to_metres(msg: Image):
    enc = msg.encoding.lower()
    if enc in ("16uc1", "mono16"):
        d = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)[:, :msg.width]
        return d.astype(np.float32) / 1000.0
    if enc == "32fc1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
    raise ValueError(f"unsupported depth encoding {msg.encoding}")


def yaw_of(q):
    return float(np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))


class SocialNavNode(Node):
    def __init__(self):
        super().__init__("social_nav_node")
        P = Params()
        # Every field of navigation.navigator.Params is a ROS parameter with the same name.
        for name, default in vars(P).items():
            self.declare_parameter(name, default)
        params = Params(**{n: self.get_parameter(n).value for n in vars(P)})
        self.declare_parameter("control_rate", 20.0)
        self.declare_parameter("camera_rate", 10.0)
        self.declare_parameter("depth_rate", 6.0)        # plan: 5-8 Hz
        self.declare_parameter("use_depth", True)        # false if the Pi cannot keep up (LiDAR only)
        self.declare_parameter("tag_size", 0.16)
        self.declare_parameter("tag_decimate", 2.0)
        # LD19 pose on the second-term robot (robot.launch.py, TF base_link -> base_laser).
        self.declare_parameter("lidar.x", 0.089)
        self.declare_parameter("lidar.y", 0.0)
        self.declare_parameter("lidar.yaw", -1.5708)
        self.declare_parameter("lidar.reversed", False)
        for k, v in TOPICS.items():
            self.declare_parameter(f"topics.{k}", v)
        for cam, values in CAMERAS.items():
            for k, v in zip(("x", "y", "z", "yaw", "pitch", "fovy_deg"), values):
                self.declare_parameter(f"{cam}.{k}", float(v))
        T = {k: self.get_parameter(f"topics.{k}").value for k in TOPICS}

        self.nav = Navigator(params)
        self.mounts = {c: Mount.from_angles(*[self.get_parameter(f"{c}.{k}").value
                                              for k in ("x", "y", "z", "yaw", "pitch")]) for c in CAMERAS}
        self.detectors = {}          # "front"/"rear" -> TagDetector, created when camera_info arrives
        self.depth = None
        self.tags, self.scan, self.depth_pts = [], None, None
        self.start = StartFrame()
        self.pose = None
        self.parada = False
        self.last_image = {"front": 0.0, "rear": 0.0, "depth": 0.0}
        self.last_tick = None

        q = qos_profile_sensor_data
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(CameraInfo, T["front_info"], lambda m: self.on_info("front", m), q)
        self.create_subscription(CameraInfo, T["rear_info"], lambda m: self.on_info("rear", m), q)
        self.create_subscription(CameraInfo, T["depth_info"], lambda m: self.on_info("depth", m), q)
        self.create_subscription(Image, T["front_image"], lambda m: self.on_image("front", m), q)
        self.create_subscription(Image, T["rear_image"], lambda m: self.on_image("rear", m), q)
        self.create_subscription(Image, T["depth_image"], self.on_depth, q)
        self.create_subscription(LaserScan, T["scan"], self.on_scan, q)
        self.create_subscription(Odometry, T["odom"], self.on_odom, 10)
        # /parada is published transient_local (kept for late subscribers) or volatile: listen to both.
        self.create_subscription(Bool, T["parada"], self.on_parada, latched)
        self.create_subscription(Bool, T["parada"], self.on_parada, 10)
        self.cmd_pub = self.create_publisher(Twist, T["cmd_vel"], 10)
        self.state_pub = self.create_publisher(String, "social_nav/state", latched)
        self.alert_pub = self.create_publisher(String, "social_nav/alert", 10)
        self.create_timer(1.0 / self.get_parameter("control_rate").value, self.tick)
        self.get_logger().info("social_nav_node ready: waiting for odometry/filtered and camera_info")

    # --- inputs ------------------------------------------------------------------------
    def _intrinsics(self, msg: CameraInfo, cam):
        K = np.array(msg.k, dtype=float)
        if K[0] > 0:
            return Intrinsics(K[0], K[4], K[2], K[5], msg.width, msg.height)
        # Uncalibrated camera: use the nominal field of view (calibrate before the exam!).
        fovy = self.get_parameter(f"{cam}_camera.fovy_deg").value
        self.get_logger().warn(f"{cam} camera has no calibration, using fovy = {fovy} deg")
        return Intrinsics.from_fov(msg.width, msg.height, fovy)

    def on_info(self, cam, msg):
        if cam == "depth":
            if self.depth is None:
                self.depth = DepthObstacles(self._intrinsics(msg, cam), self.mounts["depth_camera"])
            return
        if cam not in self.detectors:
            self.detectors[cam] = TagDetector(cam, self._intrinsics(msg, cam), self.mounts[f"{cam}_camera"],
                                              tag_size=self.get_parameter("tag_size").value,
                                              ids=(self.nav.p.back_tag_id, self.nav.p.chest_tag_id),
                                              decimate=self.get_parameter("tag_decimate").value)
            self.get_logger().info(f"{cam} tag detector ready")

    def _due(self, cam):
        now = time.monotonic()
        rate = self.get_parameter("depth_rate" if cam == "depth" else "camera_rate").value
        if now - self.last_image[cam] < 1.0 / rate:
            return False
        self.last_image[cam] = now
        return True

    def on_image(self, cam, msg):
        # One camera per phase (CPU): the front one in phase 1, the rear one from TURN_AROUND on.
        if cam != ("front" if self.nav.state.phase == 1 else "rear"):
            return
        if cam in self.detectors and self._due(cam):
            self.tags += self.detectors[cam].detect(image_to_gray(msg))

    def on_depth(self, msg):
        if self.depth is not None and self.get_parameter("use_depth").value and self._due("depth"):
            self.depth_pts = self.depth.points(depth_to_metres(msg))

    def on_scan(self, msg: LaserScan):
        g = self.get_parameter
        self.scan = scan_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
                                g("lidar.x").value, g("lidar.y").value, g("lidar.yaw").value,
                                g("lidar.reversed").value)

    def on_odom(self, msg: Odometry):
        p = msg.pose.pose
        first = self.start.origin is None
        resets = self.start.resets
        self.pose = self.start.update((p.position.x, p.position.y, yaw_of(p.orientation)))
        if first:
            self.get_logger().info("start pose stored: the far corner is straight ahead")
        elif self.start.resets != resets:
            self.get_logger().warn("odometry jumped (EKF restarted?): start frame re-anchored")

    def on_parada(self, msg: Bool):
        if msg.data != self.parada:
            self.get_logger().warn("parada: controller paused" if msg.data else "parada released")
        self.parada = msg.data

    # --- control loop --------------------------------------------------------------------
    def tick(self):
        now = time.monotonic()
        dt = 0.05 if self.last_tick is None else float(np.clip(now - self.last_tick, 0.01, 0.2))
        self.last_tick = now
        if self.pose is None or self.parada:
            # No odometry yet, or /parada: stand still and freeze the state machine and its timers.
            self.tags, self.scan, self.depth_pts = [], None, None
            self.cmd_pub.publish(Twist())
            return
        obs = Observation(dt=dt, pose=self.pose, tags=self.tags, scan=self.scan, depth=self.depth_pts)
        self.tags, self.scan, self.depth_pts = [], None, None
        old = self.nav.state
        cmd = self.nav.step(obs)
        if cmd.state != old.value:
            self.get_logger().info(f"{old.value} -> {cmd.state}: {self.nav.fsm.history[-1][3]}")
        tw = Twist()
        tw.linear.x, tw.angular.z = cmd.v, cmd.omega
        self.cmd_pub.publish(tw)
        self.state_pub.publish(String(data=cmd.state))
        if cmd.alert:
            self.alert_pub.publish(String(data=cmd.alert))


def main():
    # Without rclpy's signal handler Ctrl+C arrives as KeyboardInterrupt with the context
    # still alive, so the final zero cmd_vel can be published (as in mision_frutas.py).
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = SocialNavNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())        # stop the robot
            time.sleep(0.2)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
