"""ROS 2 node that speaks the alerts on the robot's speaker.

Subscribes
    social_nav/alert  (std_msgs/String)  alert to say once, published by social_nav_node
    social_nav/state  (std_msgs/String)  current state; LOST and WAIT_PERSON repeat their phrase

Each alert is a phrase from ``voice_dir/<alert>.wav`` (see sounds.py); without that
file a beep is played.  The same alert is not repeated within ``min_interval`` seconds
(the plan asks for 5-8 s), except the end-of-phase signals, which always sound.
Played with ``aplay``; ``device`` chooses the card (``aplay -L``), "" = default.
"""

import shutil
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .sounds import PHRASES, write_all

REPEAT = {"LOST": "lost", "WAIT_PERSON": "wait"}
ALWAYS = {"corner", "done"}


def choose_files(voice_dir, beeps):
    """alert -> WAV: the recorded phrase if it exists, else the beep."""
    voice = Path(voice_dir).expanduser()
    return {name: (voice / f"{name}.wav" if (voice / f"{name}.wav").is_file() else beeps[name]) for name in PHRASES}


class SpeakerNode(Node):
    def __init__(self):
        super().__init__("speaker_node")
        self.device = self.declare_parameter("device", "").value
        voice_dir = self.declare_parameter("voice_dir", "~/.config/social_nav/voice").value
        self.min_interval = self.declare_parameter("min_interval", 6.0).value
        repeat = self.declare_parameter("repeat_period", 6.0).value
        self.files = choose_files(voice_dir, write_all(tempfile.mkdtemp(prefix="social_nav_beeps_")))
        beeps = sorted(n for n, f in self.files.items() if not str(f).startswith(str(Path(voice_dir).expanduser())))
        if beeps:
            self.get_logger().warn(f"no recorded phrase for {beeps} in {voice_dir}: beeps instead "
                                   "(ros2 run bringup make_voice)")
        self.player = shutil.which("aplay")
        if self.player is None:
            self.get_logger().error("aplay not found: install alsa-utils to hear the alerts")
        self.state = None
        self.proc = None
        self.last = {}
        self.create_subscription(String, "social_nav/alert", lambda m: self.play(m.data), 10)
        self.create_subscription(String, "social_nav/state", self.on_state, 10)
        self.create_timer(repeat, self.on_timer)

    def play(self, name):
        now = time.monotonic()
        if self.player is None or name not in self.files:
            return
        if name not in ALWAYS and now - self.last.get(name, -1e9) < self.min_interval:
            return
        self.last[name] = now
        if self.proc is not None and self.proc.poll() is None:
            self.proc.terminate()               # a new alert replaces the one still playing
        cmd = [self.player, "-q"] + (["-D", self.device] if self.device else []) + [str(self.files[name])]
        self.proc = subprocess.Popen(cmd)
        self.get_logger().info(f"alert: {name} ({PHRASES[name]})")

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
