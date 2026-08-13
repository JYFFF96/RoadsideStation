from __future__ import print_function

import math

from .far_sparse_discovery import discover_far_sparse_candidates


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


def _previous_rescue_streak(track):
    """Recover consecutive rescue count from portable candidate metadata.

    Fusion/tracker already preserve scale_modes on the last associated LiDAR
    candidate, so no tracker-internal or CARLA-specific state is required.
    A normal LiDAR detection has no rescue_streak_* marker and resets to zero.
    """
    for mode in track.get("scale_modes", []) or []:
        text = str(mode)
        if text.startswith("rescue_streak_"):
            try:
                return max(0, int(text.rsplit("_", 1)[-1]))
            except Exception:
                return 0
    return 0


def track_guided_sparse_rescue(points, previous_tracks, world_transform,
                               existing_clusters, config=None):
    """Recover sparse candidates using history plus current-frame discovery.

    V0.6.9 provides track-guided recovery around stable prior tracks.
    V0.6.10 adds track-independent 30-80m current-frame discovery.
    V0.6.11 gates only the track-guided branch by prior track quality and by a
    maximum consecutive rescue streak, preventing weak tracks from surviving
    indefinitely on sparse local point support. NewDiscovery is unchanged.

    Neither path consumes CARLA truth. All candidates still pass normal ROI and
    score gates in fusion before tracking/fusion output.
    """
    c = config or {}
    out = []
    occupied = list(existing_clusters or [])
    diag = {
        "eligible": 0,
        "quality_block": 0,
        "streak_block": 0,
        "support_block": 0,
        "geometry_block": 0,
        "built": 0,
    }

    if c.get("sparse_geometry_rescue_enabled", False) and points is not None and previous_tracks:
        min_range = float(c.get("sparse_geometry_rescue_min_range", 30.0))
        max_range = float(c.get("sparse_geometry_rescue_max_range", 80.0))
        far_range = float(c.get("sparse_geometry_rescue_far_range", 50.0))
        min_hits = max(2, int(c.get("sparse_geometry_rescue_min_track_hits", 3)))
        min_quality = float(c.get("sparse_geometry_rescue_min_quality", 0.55))
        max_streak = max(0, int(c.get("sparse_geometry_rescue_max_streak", 2)))
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

            diag["eligible"] += 1
            quality = float(track.get("track_quality", 0.0))
            if quality < min_quality:
                diag["quality_block"] += 1
                continue
            previous_streak = _previous_rescue_streak(track)
            if previous_streak >= max_streak:
                diag["streak_block"] += 1
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
                diag["support_block"] += 1
                continue

            e = _extent(support)
            hl = max(float(e[0]), float(e[1]))
            hs = min(float(e[0]), float(e[1]))
            h = float(e[2])
            if hl < min_length or hl > max_length or \
                    hs < min_width or hs > max_width or \
                    h < min_height or h > max_height:
                diag["geometry_block"] += 1
                continue

            streak = previous_streak + 1
            n = float(len(support))
            item = {
                "x": sum(float(p[0]) for p in support) / n,
                "y": sum(float(p[1]) for p in support) / n,
                "z": sum(float(p[2]) for p in support) / n,
                "point_count": len(support),
                "extent": e,
                "cluster_mode": "sparse_rescue",
                "scale_votes": 1,
                "scale_modes": ["sparse_rescue", "rescue_streak_%d" % streak],
                "sparse_rescued": True,
                "sparse_discovered": False,
                "rescue_track_id": track.get("id"),
                "rescue_track_hits": int(track.get("track_hits", 0)),
                "rescue_range": rng,
            }
            out.append(item)
            occupied.append(item)
            diag["built"] += 1

    # V0.6.10: independent current-frame discovery remains intentionally
    # unchanged by the V0.6.11 Track Rescue Quality Gate.
    for item in discover_far_sparse_candidates(points, occupied, c):
        x = dict(item)
        x["sparse_rescued"] = True
        x["sparse_discovered"] = True
        x["rescue_track_id"] = None
        x["rescue_track_hits"] = 0
        out.append(x)
        occupied.append(x)

    track_guided_sparse_rescue.last_stats = diag
    return out


track_guided_sparse_rescue.last_stats = {
    "eligible": 0, "quality_block": 0, "streak_block": 0,
    "support_block": 0, "geometry_block": 0, "built": 0,
}
