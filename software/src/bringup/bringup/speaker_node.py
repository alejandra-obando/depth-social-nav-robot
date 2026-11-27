"""ROS 2 node that plays the alerts on the USB speaker.

Subscribes
    /social_nav/alert  (std_msgs/String)  sound to play once, published by social_nav_node
    /social_nav/state  (std_msgs/String)  current state; LOST and WAIT_PERSON repeat their beep

Sounds are played with ``aplay`` (ALSA, installed on Raspberry Pi OS / Ubuntu).
Choose the USB speaker with the ``device`` parameter, e.g. ``plughw:CARD=Device`` (list
the cards with ``aplay -L``).  An empty device uses the system default.
"""

import shutil
import subprocess
import tempfile

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .sounds import write_all

REPEAT = {"LOST": "lost", "WAIT_PERSON": "wait"}


class SpeakerNode(Node):
    def __init__(self):
        super().__init__("speaker_node")
        self.declare_parameter("device", "")
        self.declare_parameter("repeat_period", 2.0)
        self.device = self.get_parameter("device").value
        self.files = write_all(tempfile.mkdtemp(prefix="social_nav_sounds_"))
        self.player = shutil.which("aplay")
        if self.player is None:
            self.get_logger().error("aplay not found: install alsa-utils to hear the alerts")
        self.state = None
        self.proc = None
        self.create_subscription(String, "social_nav/alert", self.on_alert, 10)
        self.create_subscription(String, "social_nav/state", self.on_state, 10)
        self.create_timer(self.get_parameter("repeat_period").value, self.on_timer)

    def play(self, name):
        if self.player is None or name not in self.files:
            return
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()               # a new alert replaces the one still playing
        cmd = [self.player, "-q"] + (["-D", self.device] if self.device else []) + [str(self.files[name])]
        self.proc = subprocess.Popen(cmd)
        self.get_logger().info(f"alert: {name}")

    def on_alert(self, msg):
        self.play(msg.data)

    def on_state(self, msg):
        self.state = msg.data

    def on_timer(self):
        if self.state in REPEAT:
            self.play(REPEAT[self.state])


def main():
    rclpy.init()
    node = SpeakerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
