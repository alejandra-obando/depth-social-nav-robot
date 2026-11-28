"""Make the ROS-free packages in ``software/src`` importable without building the ROS workspace."""

import sys
from pathlib import Path

SRC = Path(__file__).resolve().parent.parent / "software" / "src"
PACKAGES = ("perception", "corners", "navigation", "bringup")


def add_software_to_path():
    for p in PACKAGES:
        path = str(SRC / p)
        if path not in sys.path:
            sys.path.insert(0, path)
