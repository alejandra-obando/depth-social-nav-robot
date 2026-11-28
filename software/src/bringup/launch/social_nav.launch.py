"""Starts the conventional controller on top of the base robot of the second term.

    ros2 launch bringup social_nav.launch.py                     # cameras + controller + speaker
    ros2 launch bringup social_nav.launch.py cameras:=false      # camera drivers started by hand
    ros2 launch bringup social_nav.launch.py base:=true          # also the base robot (service wallens stopped)
    ros2 launch bringup social_nav.launch.py controller:=false   # only the camera drivers (to test them)
    ros2 launch bringup social_nav.launch.py dry_run:=true       # everything, but cmd_vel goes to cmd_vel_dry:
                                                                 # the wheels never move
    ros2 launch bringup social_nav.launch.py params:=/path/to/my_params.yaml

The base robot (wheels driver, LiDAR brake, IMU, LD19, EKF, battery) is the launch file
of the wallens repository, ``launch/robot.launch.py``.  On the Pi it normally runs as
the systemd service ``wallens``, started at boot, so ``base`` is false by default:
launching it twice puts two drivers on the same Dynamixel bus.

Here only what the second term did not have: the D415 (front, colour + depth), the C270
(rear, ``/dev/cam_rear``), the controller and the speaker.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("bringup"))
    params = LaunchConfiguration("params")
    cameras = IfCondition(LaunchConfiguration("cameras"))
    controller = IfCondition(LaunchConfiguration("controller"))
    # Nodes that can fail without danger are restarted, as in robot.launch.py.
    alive = {"output": "screen", "respawn": True, "respawn_delay": 2.0}
    return LaunchDescription([
        DeclareLaunchArgument("params", default_value=str(share / "config" / "social_nav.yaml"),
                              description="parameter file of the controller and the speaker"),
        DeclareLaunchArgument("cameras", default_value="true", choices=["true", "false"],
                              description="start the D415 and C270 drivers"),
        DeclareLaunchArgument("base", default_value="false", choices=["true", "false"],
                              description="also start launch/robot.launch.py (only with the wallens service stopped)"),
        DeclareLaunchArgument("wallens_dir",
                              default_value=PathJoinSubstitution([EnvironmentVariable("HOME"), "wallens"]),
                              description="checkout of the wallens repository on the Pi"),
        DeclareLaunchArgument("controller", default_value="true", choices=["true", "false"],
                              description="start social_nav_node and speaker_node"),
        DeclareLaunchArgument("dry_run", default_value="false", choices=["true", "false"],
                              description="publish the velocity on cmd_vel_dry instead of cmd_vel (wheels never move)"),
        DeclareLaunchArgument("rear_device", default_value="/dev/cam_rear",
                              description="C270 device (udev rule in software/config/99-social-nav.rules)"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([LaunchConfiguration("wallens_dir"),
                                                                "launch", "robot.launch.py"])),
            launch_arguments={"brazo": "false"}.items(),
            condition=IfCondition(LaunchConfiguration("base"))),

        # D415: colour for the back tag, depth for the low stacks.  Topics /d415/color/... and
        # /d415/depth/... (realsense-ros 4: /<namespace>/<name>/...).  Low resolutions for the Pi.
        Node(package="realsense2_camera", executable="realsense2_camera_node", namespace="", name="d415",
             parameters=[{"enable_color": True, "enable_depth": True, "enable_infra1": False,
                          "enable_infra2": False, "enable_gyro": False, "enable_accel": False,
                          "rgb_camera.color_profile": "640x480x15",
                          "depth_module.depth_profile": "424x240x6",
                          "align_depth.enable": False, "pointcloud.enable": False}],
             condition=cameras, **alive),
        # C270 (rear): chest tag.  v4l2_camera, as in docs/camaras.md of the second term.
        # Its calibration goes in camera_info_url (file:///home/wallens/.config/social_nav/c270.yaml).
        Node(package="v4l2_camera", executable="v4l2_camera_node", namespace="c270", name="c270",
             parameters=[{"video_device": LaunchConfiguration("rear_device"), "image_size": [640, 480],
                          "pixel_format": "YUYV", "output_encoding": "rgb8",
                          "camera_frame_id": "c270_link"}],
             condition=cameras, **alive),

        # The controller is not respawned: if it dies the robot must stay still (the driver
        # stops 0.25 s after the last cmd_vel_safe) and the run is started again by hand.
        Node(package="bringup", executable="social_nav_node", name="social_nav_node",
             parameters=[params, {"topics.cmd_vel": PythonExpression(
                 ["'cmd_vel_dry' if '", LaunchConfiguration("dry_run"), "' == 'true' else 'cmd_vel'"])}],
             output="screen", condition=controller),
        Node(package="bringup", executable="speaker_node", name="speaker_node", parameters=[params],
             condition=controller, **alive),
    ])
