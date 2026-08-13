from __future__ import print_function

from collections import defaultdict, deque
import math


def _range_xy(p):
    return math.hypot(float(p[0]), float(p[1]))


def _near_existing(x, y, clusters, gate):
    g2 = float(gate) * float(gate)
    for c in clusters or []:
        dx = float(c.get("x", 0.0)) - float(x)
        dy = float(c.get("y", 0.0)) - float(y)
        if dx * dx + dy * dy <= g2:
            return True
    return False


def _candidate(points, mode):
    xs = [float(p[0]) for p in points]
    ys = [float(p[1]) for p in points]
    zs = [float(p[2]) for p in points]
    n = float(len(points))
    return {
        "x": sum(xs) / n,
        "y": sum(ys) / n,
        "z": sum(zs) / n,
        "point_count": len(points),
        "extent": [max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)],
        "cluster_mode": mode,
        "scale_votes": 1,
        "scale_modes": [mode],
        "sparse_discovered": True,
    }


def discover_far_sparse_candidates(points, existing_clusters, config=None):
    """Track-independent sparse candidate discovery for 30-80m LiDAR points.

    This branch intentionally uses only current-frame point evidence. It does
    not consume CARLA truth or tracker identity. Candidates still pass the
    normal road ROI and candidate-score gates in fusion before tracking.
    """
    c = config or {}
    if not c.get("far_sparse_discovery_enabled", False) or points is None:
        return []

    min_range = float(c.get("far_sparse_discovery_min_range", 30.0))
    max_range = float(c.get("far_sparse_discovery_max_range", 80.0))
    far_range = float(c.get("far_sparse_discovery_far_range", 50.0))
    mid_cell = float(c.get("far_sparse_discovery_mid_cell", 0.90))
    far_cell = float(c.get("far_sparse_discovery_far_cell", 1.20))
    mid_min_points = max(2, int(c.get("far_sparse_discovery_mid_min_points", 4)))
    far_min_points = max(2, int(c.get("far_sparse_discovery_far_min_points", 3)))
    dedupe = float(c.get("far_sparse_discovery_dedupe_distance", 2.0))
    max_candidates = max(1, int(c.get("far_sparse_discovery_max_candidates", 40)))

    min_length = float(c.get("far_sparse_discovery_min_length", 0.35))
    max_length = float(c.get("far_sparse_discovery_max_length", 7.5))
    min_width = float(c.get("far_sparse_discovery_min_width", 0.12))
    max_width = float(c.get("far_sparse_discovery_max_width", 3.5))
    min_height = float(c.get("far_sparse_discovery_min_height", 0.06))
    max_height = float(c.get("far_sparse_discovery_max_height", 3.2))

    pts = [p for p in points if min_range <= _range_xy(p) <= max_range]
    if not pts:
        return []

    # Split mid/far so the far field can use a coarser connectivity cell.
    groups = [
        ("far_discovery_mid", [p for p in pts if _range_xy(p) < far_range], mid_cell, mid_min_points),
        ("far_discovery_far", [p for p in pts if _range_xy(p) >= far_range], far_cell, far_min_points),
    ]

    out = []
    occupied_candidates = list(existing_clusters or [])
    for mode, band_points, cell, need in groups:
        if not band_points:
            continue
        buckets = defaultdict(list)
        for idx, p in enumerate(band_points):
            key = (int(math.floor(float(p[0]) / cell)), int(math.floor(float(p[1]) / cell)))
            buckets[key].append(idx)
        occupied = set(buckets)
        visited = set()
        offsets = [(a, b) for a in (-1, 0, 1) for b in (-1, 0, 1)]
        for start in occupied:
            if start in visited:
                continue
            visited.add(start)
            q = deque([start])
            indices = []
            while q:
                k = q.popleft()
                indices.extend(buckets[k])
                kx, ky = k
                for dx, dy in offsets:
                    nk = (kx + dx, ky + dy)
                    if nk in occupied and nk not in visited:
                        visited.add(nk)
                        q.append(nk)
            if len(indices) < need:
                continue
            support = [band_points[i] for i in indices]
            item = _candidate(support, mode)
            ex, ey, ez = [float(v) for v in item["extent"]]
            hl, hs = max(ex, ey), min(ex, ey)
            if hl < min_length or hl > max_length:
                continue
            if hs < min_width or hs > max_width:
                continue
            if ez < min_height or ez > max_height:
                continue
            if _near_existing(item["x"], item["y"], occupied_candidates, dedupe):
                continue
            out.append(item)
            occupied_candidates.append(item)
            if len(out) >= max_candidates:
                return out
    return out
