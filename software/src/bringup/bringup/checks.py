"""Text that sensor_check prints for each sensor (ROS-free, unit tested)."""

import numpy as np

from corners.corner_detector import find_corners
from corners.line_extraction import extract_segments

# Robot outline along x (front and rear edge), from software/config/footprint_no_gripper.yaml.
FRONT_EDGE, REAR_EDGE = 0.163, 0.164


def describe_tags(tags, camera):
    """One line per TagObservation; distance also from the robot edge that faces the camera."""
    if not tags:
        return "no tag"
    edge = FRONT_EDGE if camera == "front" else REAR_EDGE
    return " | ".join(
        f"ID {t.tag_id}: {t.distance:.2f} m from centre, {t.distance - edge:.2f} m from the {camera} edge, "
        f"bearing {np.degrees(t.bearing):+.0f} deg, height {t.z:.2f} m, margin {t.margin:.0f}" for t in tags)


def describe_points(angles, ranges, what="points"):
    if len(ranges) == 0:
        return f"no {what}"
    k = int(np.argmin(ranges))
    return f"{len(ranges)} {what}, nearest {ranges[k]:.2f} m at {np.degrees(angles[k]):+.0f} deg"


def describe_lidar(angles, ranges):
    a = np.arctan2(np.sin(angles), np.cos(angles))
    front, back = np.abs(a) < np.radians(30), np.abs(a) > np.radians(150)
    corners = find_corners(extract_segments(angles, ranges))
    c = ", ".join(f"{k.distance:.2f} m at {np.degrees(k.bearing):+.0f} deg ({np.degrees(k.angle):.0f} deg walls)"
                  for k in sorted(corners, key=lambda k: k.distance)[:3]) or "none"
    return (f"front: {describe_points(a[front], ranges[front], 'points')} | "
            f"behind: {describe_points(a[back], ranges[back], 'points')} | corners: {c}")
