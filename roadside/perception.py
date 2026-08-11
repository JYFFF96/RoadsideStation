from __future__ import print_function

from collections import defaultdict, deque

import numpy as np


def voxel_cluster_lidar(points, voxel_size=0.8, min_points=6,
                         min_z=-7.5, max_z=2.0, max_range=70.0):
    """Lightweight NumPy-only Euclidean-style clustering for Python 3.7.

    Points are filtered, voxelized, and neighboring occupied voxels are flood
    filled. The result is one centroid per cluster plus point count and bounds.
    This avoids a SciPy/sklearn dependency while giving V0.2 a real object
    proposal stage instead of treating every sensor return as an object.
    """
    if points is None or len(points) == 0:
        return []

    pts = np.asarray(points, dtype=np.float32)
    r2 = pts[:, 0] * pts[:, 0] + pts[:, 1] * pts[:, 1]
    mask = ((pts[:, 2] >= float(min_z)) &
            (pts[:, 2] <= float(max_z)) &
            (r2 <= float(max_range) * float(max_range)))
    pts = pts[mask]
    if len(pts) == 0:
        return []

    size = float(voxel_size)
    keys = np.floor(pts / size).astype(np.int32)
    buckets = defaultdict(list)
    for i, key in enumerate(keys):
        buckets[(int(key[0]), int(key[1]), int(key[2]))].append(i)

    occupied = set(buckets.keys())
    visited = set()
    clusters = []
    neighbor_offsets = [(dx, dy, dz)
                        for dx in (-1, 0, 1)
                        for dy in (-1, 0, 1)
                        for dz in (-1, 0, 1)]

    for start in occupied:
        if start in visited:
            continue
        visited.add(start)
        queue = deque([start])
        indices = []
        while queue:
            cell = queue.popleft()
            indices.extend(buckets[cell])
            cx, cy, cz = cell
            for dx, dy, dz in neighbor_offsets:
                nxt = (cx + dx, cy + dy, cz + dz)
                if nxt in occupied and nxt not in visited:
                    visited.add(nxt)
                    queue.append(nxt)

        if len(indices) < int(min_points):
            continue
        cpts = pts[indices]
        centroid = cpts.mean(axis=0)
        pmin = cpts.min(axis=0)
        pmax = cpts.max(axis=0)
        extent = pmax - pmin

        # Reject giant connected background structures and tiny speckles.
        if extent[0] > 15.0 or extent[1] > 15.0 or extent[2] > 8.0:
            continue
        if extent[0] < 0.15 and extent[1] < 0.15:
            continue

        clusters.append({
            "x": float(centroid[0]),
            "y": float(centroid[1]),
            "z": float(centroid[2]),
            "point_count": int(len(indices)),
            "extent": [float(extent[0]), float(extent[1]), float(extent[2])],
        })

    return clusters


def associate_radar(clusters, radar_detections, max_distance=3.0):
    """Attach the closest radar return to each LiDAR cluster."""
    if not clusters:
        return []
    radar = radar_detections or []
    max_d2 = float(max_distance) ** 2
    output = []
    for cluster in clusters:
        best = None
        best_d2 = max_d2
        for det in radar:
            dx = float(det["x"]) - cluster["x"]
            dy = float(det["y"]) - cluster["y"]
            dz = float(det["z"]) - cluster["z"]
            d2 = dx * dx + dy * dy + dz * dz
            if d2 < best_d2:
                best_d2 = d2
                best = det
        item = dict(cluster)
        item["radar"] = best
        output.append(item)
    return output
