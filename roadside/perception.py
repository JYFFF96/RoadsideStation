from __future__ import print_function

from collections import defaultdict, deque

import numpy as np


def voxel_cluster_lidar(points, voxel_size=0.8, min_points=6,
                         min_z=-7.5, max_z=2.0, max_range=70.0,
                         min_length=0.6, max_length=8.0,
                         min_width=0.4, max_width=4.0,
                         min_height=0.25, max_height=4.0,
                         max_objects=80):
    """NumPy-only LiDAR clustering with V0.2.1 geometry rejection.

    The result remains deliberately lightweight for Python 3.7, but now rejects
    many poles, walls, curbs and giant connected structures before tracking.
    Extents are axis-aligned in the LiDAR frame, so the limits are intentionally
    generous enough for rotated vehicles.
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
        ex = float(extent[0])
        ey = float(extent[1])
        ez = float(extent[2])

        horizontal_long = max(ex, ey)
        horizontal_short = min(ex, ey)
        if horizontal_long < float(min_length) or horizontal_long > float(max_length):
            continue
        if horizontal_short < float(min_width) or horizontal_short > float(max_width):
            continue
        if ez < float(min_height) or ez > float(max_height):
            continue

        clusters.append({
            "x": float(centroid[0]),
            "y": float(centroid[1]),
            "z": float(centroid[2]),
            "point_count": int(len(indices)),
            "extent": [ex, ey, ez],
        })

    # Larger clusters are usually more reliable than small isolated clutter.
    clusters.sort(key=lambda item: item["point_count"], reverse=True)
    return clusters[:int(max_objects)]


def associate_radar(clusters, radar_detections, max_distance=3.0):
    """Attach the closest unused radar return to each LiDAR cluster."""
    if not clusters:
        return []
    radar = radar_detections or []
    max_d2 = float(max_distance) ** 2
    used = set()
    output = []

    for cluster in clusters:
        best = None
        best_index = None
        best_d2 = max_d2
        for index, det in enumerate(radar):
            if index in used:
                continue
            dx = float(det["x"]) - cluster["x"]
            dy = float(det["y"]) - cluster["y"]
            dz = float(det["z"]) - cluster["z"]
            d2 = dx * dx + dy * dy + dz * dz
            if d2 < best_d2:
                best_d2 = d2
                best = det
                best_index = index
        if best_index is not None:
            used.add(best_index)
        item = dict(cluster)
        item["radar"] = best
        output.append(item)
    return output
