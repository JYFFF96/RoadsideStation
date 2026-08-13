from __future__ import print_function

import math


def _to_local(wx, wy, wz, transform):
    """Inverse planar transform using only portable scalar pose values."""
    if not transform:
        return float(wx), float(wy), float(wz)
    dx = float(wx) - float(transform.get("x", 0.0))
    dy = float(wy) - float(transform.get("y", 0.0))
    yaw = float(transform.get("yaw", 0.0))
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy, float(wz) - float(transform.get("z", 0.0))


def _range_xy(x, y):
    return math.hypot(float(x), float(y))


def _near_existing(x, y, clusters, gate):
    g2 = float(gate) * float(gate)
    for c in clusters or []:
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


def track_guided_sparse_rescue(points, previous_tracks, world_transform,
                               existing_clusters, config=None):
    """Recover intermittent sparse LiDAR candidates around stable prior tracks.

    This is deliberately track-guided instead of globally relaxing clustering.
    It never creates a candidate without prior temporal evidence, leaves 0-30m
    untouched, and uses only scalar positions/extents so it can be ported to
    Qt/C++ and real point-cloud inputs later.
    """
    c = config or {}
    if not c.get("sparse_geometry_rescue_enabled", False):
        return []
    if points is None or not previous_tracks:
        return []

    min_range = float(c.get("sparse_geometry_rescue_min_range", 30.0))
    max_range = float(c.get("sparse_geometry_rescue_max_range", 80.0))
    far_range = float(c.get("sparse_geometry_rescue_far_range", 50.0))
    min_hits = max(2, int(c.get("sparse_geometry_rescue_min_track_hits", 3)))
    mid_radius = float(c.get("sparse_geometry_rescue_mid_radius", 2.2))
    far_radius = float(c.get("sparse_geometry_rescue_far_radius", 3.0))
    z_window = float(c.get("sparse_geometry_rescue_z_window", 2.0))
    dedupe = float(c.get("sparse_geometry_rescue_dedupe_distance", 2.0))
    mid_points = max(2, int(c.get("sparse_geometry_rescue_mid_min_points", 3)))
    far_points = max(2, int(c.get("sparse_geometry_rescue_far_min_points", 2)))
    min_length = float(c.get("sparse_geometry_rescue_min_length", 0.18))
    min_width = float(c.get("sparse_geometry_rescue_min_width", 0.08))
    min_height = float(c.get("sparse_geometry_rescue_min_height", 0.05))
    max_length = float(c.get("sparse_geometry_rescue_max_length", 7.5))
    max_width = float(c.get("sparse_geometry_rescue_max_width", 3.5))
    max_height = float(c.get("sparse_geometry_rescue_max_height", 3.0))

    pts = list(points)
    out = []
    occupied = list(existing_clusters or [])
    for track in previous_tracks or []:
        if int(track.get("track_hits", 0)) < min_hits:
            continue
        if str(track.get("track_state", "confirmed")) == "new":
            continue

        lx, ly, lz = _to_local(track.get("x", 0.0), track.get("y", 0.0),
                               track.get("z", 0.0), world_transform)
        rng = _range_xy(lx, ly)
        if rng < min_range or rng > max_range:
            continue
        if _near_existing(lx, ly, occupied, dedupe):
            continue

        radius = far_radius if rng >= far_range else mid_radius
        need = far_points if rng >= far_range else mid_points
        r2 = radius * radius
        support = []
        for p in pts:
            dx = float(p[0]) - lx
            dy = float(p[1]) - ly
            if dx * dx + dy * dy > r2:
                continue
            if abs(float(p[2]) - lz) > z_window:
                continue
            support.append(p)
        if len(support) < need:
            continue

        e = _extent(support)
        hl = max(float(e[0]), float(e[1]))
        hs = min(float(e[0]), float(e[1]))
        h = float(e[2])
        if hl < min_length or hl > max_length:
            continue
        if hs < min_width or hs > max_width:
            continue
        if h < min_height or h > max_height:
            continue

        n = float(len(support))
        item = {
            "x": sum(float(p[0]) for p in support) / n,
            "y": sum(float(p[1]) for p in support) / n,
            "z": sum(float(p[2]) for p in support) / n,
            "point_count": len(support),
            "extent": e,
            "cluster_mode": "sparse_rescue",
            "scale_votes": 1,
            "scale_modes": ["sparse_rescue"],
            "sparse_rescued": True,
            "rescue_track_id": track.get("id"),
            "rescue_track_hits": int(track.get("track_hits", 0)),
            "rescue_range": rng,
        }
        out.append(item)
        occupied.append(item)

    return out
