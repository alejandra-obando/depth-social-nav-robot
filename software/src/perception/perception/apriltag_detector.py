"""AprilTag detection of the signs worn by the person (family tag36h11).

The person wears one tag on the back (seen by the front D415 while following)
and one on the chest (seen by the rear C270 while guiding).  The detector is
AprilTag 3 through ``pupil-apriltags`` (Wang & Olson 2016; Krogius, Haggenmiller
& Olson 2019); the pose of each tag comes from its four corners and the known
tag size.
"""

from dataclasses import dataclass

import numpy as np

from .camera import Intrinsics, Mount

# pupil-apriltags can crash (segfault in quick_decode_uninit) when a Detector is destroyed
# while another one uses the same tag family.  Each configuration is therefore created
# once per process and shared; it is never destroyed before the process ends.  A shared
# detector must be used from one thread at a time (the ROS node's default executor does that).
_DETECTORS = {}


def _shared_detector(family, threads, decimate):
    key = (family, threads, decimate)
    if key not in _DETECTORS:
        from pupil_apriltags import Detector     # imported here so the rest of the package works without it
        _DETECTORS[key] = Detector(families=family, nthreads=threads, quad_decimate=decimate)
    return _DETECTORS[key]


@dataclass
class TagObservation:
    tag_id: int
    camera: str            # name of the camera that saw it ("front" or "rear")
    x: float               # tag centre in the robot frame [m]
    y: float
    z: float
    margin: float          # decision margin from the detector: higher is more reliable

    @property
    def distance(self):
        """Horizontal distance from the robot centre [m]."""
        return float(np.hypot(self.x, self.y))

    @property
    def bearing(self):
        """Angle from the robot's forward axis, positive to the left [rad]."""
        return float(np.arctan2(self.y, self.x))


class TagDetector:
    """Detects tags in a grey image and returns their position in the robot frame.

    Parameters
    ----------
    camera     : name copied into every observation.
    intrinsics : camera intrinsics (calibrate the real cameras; the simulation uses its own).
    mount      : pose of the camera on the robot.
    tag_size   : side of the black square of the tag [m] (not the white margin).
    ids        : tag IDs to keep; detections of other IDs are dropped.
    min_margin : detections below this decision margin are dropped as unreliable.
    """

    def __init__(self, camera, intrinsics: Intrinsics, mount: Mount, tag_size=0.16, ids=(0, 1),
                 min_margin=20.0, family="tag36h11", decimate=1.0, threads=2):
        self.camera = camera
        self.intrinsics = intrinsics
        self.mount = mount
        self.tag_size = tag_size
        self.ids = set(ids)
        self.min_margin = min_margin
        self._detector = _shared_detector(family, threads, decimate)

    def detect(self, gray):
        """``gray``: H x W uint8 image.  Returns a list of TagObservation."""
        gray = np.ascontiguousarray(gray, dtype=np.uint8)
        found = self._detector.detect(gray, estimate_tag_pose=True,
                                      camera_params=self.intrinsics.params, tag_size=self.tag_size)
        out = []
        for d in found:
            if d.tag_id not in self.ids or d.decision_margin < self.min_margin:
                continue
            p = self.mount.to_robot(np.asarray(d.pose_t).ravel())
            out.append(TagObservation(int(d.tag_id), self.camera, float(p[0]), float(p[1]), float(p[2]),
                                      float(d.decision_margin)))
        return out
