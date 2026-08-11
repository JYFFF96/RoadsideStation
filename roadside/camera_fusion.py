from __future__ import print_function

import math
import numpy as np


class CameraProjector(object):
    """Project CARLA world points into the roadside RGB camera image.

    CARLA/UE coordinates are converted to the conventional camera frame:
    camera x -> image z, camera y -> image x, camera z -> -image y.
    """
    def __init__(self, width, height, fov_degrees, camera_transform):
        self.width = int(width)
        self.height = int(height)
        self.fov = float(fov_degrees)
        self.transform = camera_transform
        focal = self.width / (2.0 * math.tan(math.radians(self.fov) / 2.0))
        self.K = np.array([[focal, 0.0, self.width / 2.0],
                           [0.0, focal, self.height / 2.0],
                           [0.0, 0.0, 1.0]], dtype=np.float64)

    def _world_to_sensor(self, x, y, z):
        # CARLA Transform exposes inverse_matrix in 0.9.15 Python API.
        inv = np.asarray(self.transform.get_inverse_matrix(), dtype=np.float64)
        p = np.dot(inv, np.array([float(x), float(y), float(z), 1.0]))
        return p[0], p[1], p[2]

    def project(self, x, y, z):
        sx, sy, sz = self._world_to_sensor(x, y, z)
        # UE4 sensor frame: X forward, Y right, Z up.
        depth = sx
        if depth <= 0.1:
            return None
        cam = np.array([sy, -sz, depth], dtype=np.float64)
        pix = np.dot(self.K, cam)
        u, v = pix[0] / pix[2], pix[1] / pix[2]
        if u < 0 or u >= self.width or v < 0 or v >= self.height:
            return None
        return {"u": float(u), "v": float(v), "depth": float(depth)}

    def annotate_candidates(self, candidates):
        visible = []
        for item in candidates:
            p = self.project(item["x"], item["y"], item["z"])
            if p is not None:
                out = dict(item)
                out["camera_u"] = p["u"]
                out["camera_v"] = p["v"]
                out["camera_depth"] = p["depth"]
                visible.append(out)
        return visible
