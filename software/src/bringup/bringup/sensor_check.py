"""Checks one sensor at a time on the real robot, without moving it.

    ros2 run bringup sensor_check --ros-args -p check:=front --params-file <social_nav.yaml> -r __node:=social_nav_node
    check: front | rear | depth | lidar

Prints twice a second what the controller would get from that sensor, with the same
code and the same parameters (topics, camera poses, LiDAR pose) as social_nav_node:

    front / rear : every tag seen: ID, distance from the robot centre and from the robot's
                   front or rear edge, bearing (+ = left), height, detector margin
    depth        : obstacle points from the D415 depth image between 3 and 45 cm high:
                   how many, the nearest one and its bearing
    lidar        : the nearest LiDAR point in front (+-30 deg) and behind, and the room
                   corners found by the corner detector

``-r __node:=social_nav_node`` makes it read the social_nav_node section of the YAML.
"""

import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan

from perception.apriltag_detector import TagDetector
from perception.camera import Intrinsics, Mount
from perception.depth_obstacles import DepthObstacles

from .checks import describe_lidar, describe_points, describe_tags
from .frames import scan_points
from .social_nav_node import CAMERAS, TOPICS, depth_to_metres, image_to_gray

class SensorCheck(Node):
    def __init__(self):
        super().__init__("sensor_check")
        self.check = self.declare_parameter("check", "front").value
        self.declare_parameter("tag_size", 0.16)
        self.declare_parameter("back_tag_id", 0)
        self.declare_parameter("chest_tag_id", 1)
        for k, v in TOPICS.items():
            self.declare_parameter(f"topics.{k}", v)
        for cam, values in CAMERAS.items():
            for k, v in zip(("x", "y", "z", "yaw", "pitch", "fovy_deg"), values):
                self.declare_parameter(f"{cam}.{k}", float(v))
        for k, v in (("x", 0.089), ("y", 0.0), ("yaw", -1.5708), ("reversed", False)):
            self.declare_parameter(f"lidar.{k}", v)
        g = self.get_parameter
        T = {k: g(f"topics.{k}").value for k in TOPICS}
        self.line, self.handler, self.last = "waiting for data...", None, 0.0
        q = qos_profile_sensor_data
        if self.check in ("front", "rear"):
            self.create_subscription(CameraInfo, T[f"{self.check}_info"], self.on_info, q)
            self.create_subscription(Image, T[f"{self.check}_image"], self.on_image, q)
        elif self.check == "depth":
            self.create_subscription(CameraInfo, T["depth_info"], self.on_info, q)
            self.create_subscription(Image, T["depth_image"], self.on_depth, q)
        elif self.check == "lidar":
            self.create_subscription(LaserScan, T["scan"], self.on_scan, q)
        else:
            raise ValueError("check must be front, rear, depth or lidar")
        self.create_timer(0.5, lambda: self.get_logger().info(self.line))
        self.get_logger().info(f"checking {self.check}")

    def _mount(self, cam):
        return Mount.from_angles(*[self.get_parameter(f"{cam}.{k}").value for k in ("x", "y", "z", "yaw", "pitch")])

    def on_info(self, msg):
        if self.handler is not None:
            return
        cam = "depth_camera" if self.check == "depth" else f"{self.check}_camera"
        K = np.array(msg.k, dtype=float)
        if K[0] > 0:
            intr = Intrinsics(K[0], K[4], K[2], K[5], msg.width, msg.height)
            self.get_logger().info(f"calibrated: fx {K[0]:.0f} fy {K[4]:.0f} cx {K[2]:.0f} cy {K[5]:.0f}, "
                                   f"{msg.width}x{msg.height}")
        else:
            fovy = self.get_parameter(f"{cam}.fovy_deg").value
            intr = Intrinsics.from_fov(msg.width, msg.height, fovy)
            self.get_logger().warn(f"NOT calibrated: using fovy {fovy} deg ({msg.width}x{msg.height})")
        if self.check == "depth":
            self.handler = DepthObstacles(intr, self._mount(cam))
        else:
            self.handler = TagDetector(self.check, intr, self._mount(cam),
                                       tag_size=self.get_parameter("tag_size").value,
                                       ids=(self.get_parameter("back_tag_id").value,
                                            self.get_parameter("chest_tag_id").value), decimate=2.0)

    def _due(self):
        now = time.monotonic()
        if now - self.last < 0.2:
            return False
        self.last = now
        return True

    def on_image(self, msg):
        if self.handler is not None and self._due():
            self.line = describe_tags(self.handler.detect(image_to_gray(msg)), self.check)

    def on_depth(self, msg):
        if self.handler is not None and self._due():
            self.line = describe_points(*self.handler.points(depth_to_metres(msg)), "low obstacle points")

    def on_scan(self, msg):
        g = self.get_parameter
        a, r = scan_points(msg.ranges, msg.angle_min, msg.angle_increment, msg.range_min, msg.range_max,
                           g("lidar.x").value, g("lidar.y").value, g("lidar.yaw").value, g("lidar.reversed").value)
        self.line = describe_lidar(a, r)


def main():
    rclpy.init()
    node = SensorCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
