"""Lets pytest import the ROS-free packages straight from the source tree:

    cd software/src && python -m pytest
"""

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
for pkg in ("perception", "corners", "navigation", "bringup"):
    sys.path.insert(0, str(HERE / pkg))
