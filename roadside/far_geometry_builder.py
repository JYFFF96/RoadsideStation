from __future__ import print_function

import math


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


def build_far_geometry_candidates(clusters, existing_geometry, config=None):
    """Build supplemental 50-80m candidates from sparse LiDAR clusters.

    Portable, CARLA-independent geometry-only helper. It never uses truth or
    tracker state. Candidates still pass the normal ROI, score and tracker path.
    """
    c = config or {}
    stats = {"input": 0, "sparse": 0, "template_pass": 0, "dedupe": 0, "built": 0}
    if not c.get("far_geometry_builder_enabled", False):
        build_far_geometry_candidates.last_stats = stats
        return []

    min_range = float(c.get("far_geometry_builder_min_range", 50.0))
    max_range = float(c.get("far_geometry_builder_max_range", 80.0))
    min_points = max(1, int(c.get("far_geometry_builder_min_points", 2)))
    min_length = float(c.get("far_geometry_builder_min_length", 0.30))
    max_length = float(c.get("far_geometry_builder_max_length", 7.5))
    min_width = float(c.get("far_geometry_builder_min_width", 0.10))
    max_width = float(c.get("far_geometry_builder_max_width", 3.5))
    min_height = float(c.get("far_geometry_builder_min_height", 0.05))
    max_height = float(c.get("far_geometry_builder_max_height", 3.2))
    dedupe = float(c.get("far_geometry_builder_dedupe_distance", 1.8))
    max_candidates = max(0, int(c.get("far_geometry_builder_max_candidates", 30)))

    out = []
    occupied = list(existing_geometry or [])
    for src in clusters or []:
        rng = _range_xy(src.get("x", 0.0), src.get("y", 0.0))
        if rng < min_range or rng > max_range:
            continue
        stats["input"] += 1
        points = int(src.get("point_count", 0))
        if points < min_points:
            continue
        stats["sparse"] += 1
        e = [float(v) for v in src.get("extent", [0, 0, 0])]
        hl = max(e[0], e[1]); hs = min(e[0], e[1]); h = e[2]
        if not (min_length <= hl <= max_length and
                min_width <= hs <= max_width and
                min_height <= h <= max_height):
            continue
        stats["template_pass"] += 1
        if _near_existing(src.get("x", 0.0), src.get("y", 0.0), occupied, dedupe):
            stats["dedupe"] += 1
            continue
        item = dict(src)
        item["cluster_mode"] = "far_geometry_builder"
        item["scale_votes"] = max(1, int(item.get("scale_votes", 1)))
        modes = list(item.get("scale_modes", []))
        if "far_geometry_builder" not in modes:
            modes.append("far_geometry_builder")
        item["scale_modes"] = modes
        item["far_geometry_built"] = True
        item["sparse_rescued"] = False
        item["sparse_discovered"] = False
        out.append(item)
        occupied.append(item)
        stats["built"] += 1
        if max_candidates and len(out) >= max_candidates:
            break

    build_far_geometry_candidates.last_stats = stats
    return out


build_far_geometry_candidates.last_stats = {
    "input": 0, "sparse": 0, "template_pass": 0, "dedupe": 0, "built": 0
}
