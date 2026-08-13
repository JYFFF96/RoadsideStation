from __future__ import print_function

import math
from collections import defaultdict, deque


def _range_xy(x, y):
    return math.hypot(float(x), float(y))


def _near_existing(x, y, items, gate):
    g2 = float(gate) * float(gate)
    for c in items or []:
        dx = float(c.get("x", 0.0)) - float(x)
        dy = float(c.get("y", 0.0)) - float(y)
        if dx * dx + dy * dy <= g2:
            return True
    return False


def _extent(points):
    xs = [float(p[0]) for p in points]
    ys = [float(p[1]) for p in points]
    zs = [float(p[2]) for p in points]
    return [max(xs) - min(xs), max(ys) - min(ys), max(zs) - min(zs)]


def build_far_geometry_candidates(points, existing_geometry, config=None):
    """Supplement 50-80m geometry from sparse current-frame LiDAR points.

    Tracker- and CARLA-truth-independent. Performs coarse BEV connected-component
    grouping, conservative vehicle-template filtering, and deduplication against
    existing local geometry. Output still passes normal ROI/Score/Tracker gates.
    """
    c = config or {}
    stats = {"input_points": 0, "components": 0, "template_pass": 0,
             "dedupe": 0, "built": 0}
    if not c.get("far_geometry_builder_enabled", True) or points is None:
        build_far_geometry_candidates.last_stats = stats
        return []

    min_range = float(c.get("far_geometry_builder_min_range", 50.0))
    max_range = float(c.get("far_geometry_builder_max_range", 80.0))
    cell = max(.2, float(c.get("far_geometry_builder_cell_size", 1.0)))
    neighbor = max(1, int(c.get("far_geometry_builder_neighbor_cells", 1)))
    min_points = max(1, int(c.get("far_geometry_builder_min_points", 2)))
    min_length = float(c.get("far_geometry_builder_min_length", 0.25))
    max_length = float(c.get("far_geometry_builder_max_length", 7.5))
    min_width = float(c.get("far_geometry_builder_min_width", 0.08))
    max_width = float(c.get("far_geometry_builder_max_width", 3.5))
    min_height = float(c.get("far_geometry_builder_min_height", 0.04))
    max_height = float(c.get("far_geometry_builder_max_height", 3.2))
    dedupe = float(c.get("far_geometry_builder_dedupe_distance", 1.8))
    max_candidates = max(0, int(c.get("far_geometry_builder_max_candidates", 30)))

    cells = defaultdict(list)
    for p in points:
        x, y = float(p[0]), float(p[1])
        rng = _range_xy(x, y)
        if rng < min_range or rng > max_range:
            continue
        stats["input_points"] += 1
        cells[(int(math.floor(x / cell)), int(math.floor(y / cell)))].append(p)

    remaining = set(cells)
    occupied = list(existing_geometry or [])
    out = []
    while remaining:
        seed = remaining.pop()
        q = deque([seed])
        keys = [seed]
        while q:
            cx, cy = q.popleft()
            for dx in range(-neighbor, neighbor + 1):
                for dy in range(-neighbor, neighbor + 1):
                    nk = (cx + dx, cy + dy)
                    if nk in remaining:
                        remaining.remove(nk)
                        q.append(nk)
                        keys.append(nk)
        support = []
        for k in keys:
            support.extend(cells[k])
        stats["components"] += 1
        if len(support) < min_points:
            continue
        e = _extent(support)
        hl = max(float(e[0]), float(e[1]))
        hs = min(float(e[0]), float(e[1]))
        h = float(e[2])
        if hl < min_length or hl > max_length or hs < min_width or hs > max_width or \
                h < min_height or h > max_height:
            continue
        stats["template_pass"] += 1
        n = float(len(support))
        x = sum(float(p[0]) for p in support) / n
        y = sum(float(p[1]) for p in support) / n
        z = sum(float(p[2]) for p in support) / n
        if _near_existing(x, y, occupied, dedupe):
            stats["dedupe"] += 1
            continue
        item = {"x": x, "y": y, "z": z,
                "point_count": len(support), "extent": e,
                "cluster_mode": "far_geometry_builder",
                "scale_votes": 1, "scale_modes": ["far_geometry_builder"],
                "far_geometry_built": True,
                "sparse_rescued": False, "sparse_discovered": False}
        out.append(item)
        occupied.append(item)
        stats["built"] += 1
        if max_candidates and len(out) >= max_candidates:
            break

    build_far_geometry_candidates.last_stats = stats
    return out


build_far_geometry_candidates.last_stats = {
    "input_points": 0, "components": 0, "template_pass": 0,
    "dedupe": 0, "built": 0
}
