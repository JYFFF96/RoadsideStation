from __future__ import print_function

import math
import time
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


def _oriented_extent(points):
    """Return PCA-aligned XY length/width plus Z height without numpy."""
    if not points:
        return [0.0, 0.0, 0.0], 0.0
    xs = [float(p[0]) for p in points]
    ys = [float(p[1]) for p in points]
    zs = [float(p[2]) for p in points]
    n = float(len(points))
    mx = sum(xs) / n
    my = sum(ys) / n
    cxx = sum((x - mx) * (x - mx) for x in xs) / n
    cyy = sum((y - my) * (y - my) for y in ys) / n
    cxy = sum((xs[i] - mx) * (ys[i] - my) for i in range(len(xs))) / n

    yaw = 0.5 * math.atan2(2.0 * cxy, cxx - cyy) if len(points) >= 2 else 0.0
    ca = math.cos(yaw)
    sa = math.sin(yaw)
    major = []
    minor = []
    for x, y in zip(xs, ys):
        dx = x - mx
        dy = y - my
        major.append(ca * dx + sa * dy)
        minor.append(-sa * dx + ca * dy)
    length = max(major) - min(major) if major else 0.0
    width = max(minor) - min(minor) if minor else 0.0
    height = max(zs) - min(zs) if zs else 0.0
    if width > length:
        length, width = width, length
        yaw += math.pi * 0.5
    return [length, width, height], yaw


def _point_key(p):
    return (round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 3))


def _unique_points(points):
    out = []
    seen = set()
    for p in points or []:
        key = _point_key(p)
        if key not in seen:
            seen.add(key)
            out.append(p)
    return out


def _center(points):
    n = float(len(points))
    return (sum(float(p[0]) for p in points) / n,
            sum(float(p[1]) for p in points) / n,
            sum(float(p[2]) for p in points) / n)


def _build_history_grid(history_frames, grid_size):
    cells = defaultdict(list)
    inv = 1.0 / max(0.2, float(grid_size))
    for _, pts in history_frames:
        for p in pts:
            x = float(p[0])
            y = float(p[1])
            cells[(int(math.floor(x * inv)), int(math.floor(y * inv)))].append(p)
    return cells, inv


def _temporal_support(current_support, history_cells, inv, gate, z_gate, max_added):
    """Add only history points spatially attached to a current-frame seed.

    History can never create a component by itself because this function is
    called only for components already formed by current-frame cells.
    """
    if not current_support or not history_cells or max_added <= 0:
        return []
    g2 = float(gate) * float(gate)
    zg = float(z_gate)
    best = {}
    for cp in current_support:
        cx = float(cp[0])
        cy = float(cp[1])
        cz = float(cp[2])
        ix = int(math.floor(cx * inv))
        iy = int(math.floor(cy * inv))
        for ox in (-1, 0, 1):
            for oy in (-1, 0, 1):
                for hp in history_cells.get((ix + ox, iy + oy), []):
                    hx = float(hp[0])
                    hy = float(hp[1])
                    hz = float(hp[2])
                    if abs(hz - cz) > zg:
                        continue
                    dx = hx - cx
                    dy = hy - cy
                    d2 = dx * dx + dy * dy
                    if d2 > g2:
                        continue
                    key = _point_key(hp)
                    old = best.get(key)
                    if old is None or d2 < old[0]:
                        best[key] = (d2, hp)
    ranked = sorted(best.values(), key=lambda x: x[0])
    return [p for _, p in ranked[:max_added]]


def _empty_stats():
    return {
        "input_points": 0,
        "components": 0,
        "too_few_points": 0,
        "length_reject": 0,
        "width_reject": 0,
        "height_reject": 0,
        "template_pass": 0,
        "dedupe": 0,
        "built": 0,
        "oriented_components": 0,
        "temporal_history_frames": 0,
        "temporal_current_points": 0,
        "temporal_added_points": 0,
        "temporal_components": 0,
        "recovery_fragments": 0,
        "recovery_attempts": 0,
        "recovery_template_pass": 0,
        "recovery_dedupe": 0,
        "recovery_built": 0,
        "roi_pass": 0,
        "score_pass": 0,
        "dynamic_pass": 0,
    }


def build_far_geometry_candidates(points, existing_geometry, config=None):
    """Supplement 50-80m geometry from sparse LiDAR points.

    V0.6.12.4 adds conservative short-term LiDAR temporal support. Every
    component is still seeded exclusively by current-frame points. History is
    limited by TTL/frame count and can only add nearby support to that seed;
    it can never create a candidate on its own. Candidate center remains based
    on current-frame points to avoid temporal lag. No tracker, camera, CARLA
    actor or ground-truth information is consumed. V0.6.12.6 adds a bounded
    second pass that bridges only nearby, undersized current-frame fragments;
    oversized and tall rejected structures remain ineligible.
    """
    c = config or {}
    stats = _empty_stats()
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
    oriented_enabled = bool(c.get("far_geometry_builder_oriented_extent_enabled", True))

    temporal_enabled = bool(c.get("far_geometry_temporal_enabled", True))
    temporal_ttl = max(0.0, float(c.get("far_geometry_temporal_ttl", 0.25)))
    temporal_frames = max(0, int(c.get("far_geometry_temporal_frames", 2)))
    temporal_gate = max(0.2, float(c.get("far_geometry_temporal_gate", 0.90)))
    temporal_z_gate = max(0.1, float(c.get("far_geometry_temporal_z_gate", 1.20)))
    temporal_max_added = max(0, int(c.get("far_geometry_temporal_max_added", 12)))

    recovery_enabled = bool(c.get("far_geometry_recovery_enabled", True))
    recovery_bridge = max(0.5, float(c.get("far_geometry_recovery_bridge_distance", 3.0)))
    recovery_z_gate = max(0.1, float(c.get("far_geometry_recovery_z_gate", 1.0)))
    recovery_max_fragments = max(2, int(c.get("far_geometry_recovery_max_fragments", 3)))
    recovery_min_current = max(2, int(c.get("far_geometry_recovery_min_current_points", 3)))
    recovery_max_fragment_points = max(1, int(c.get("far_geometry_recovery_max_fragment_points", 12)))
    recovery_fragment_max_height = max(min_height, float(c.get("far_geometry_recovery_fragment_max_height", 2.0)))
    recovery_max_added = max(0, int(c.get("far_geometry_recovery_max_candidates", 8)))

    now = time.time()
    history = getattr(build_far_geometry_candidates, "_history", deque())
    while history and (now - float(history[0][0]) > temporal_ttl):
        history.popleft()
    while temporal_frames >= 0 and len(history) > temporal_frames:
        history.popleft()

    current_far_points = []
    cells = defaultdict(list)
    for p in points:
        x, y = float(p[0]), float(p[1])
        rng = _range_xy(x, y)
        if rng < min_range or rng > max_range:
            continue
        current_far_points.append(p)
        stats["input_points"] += 1
        cells[(int(math.floor(x / cell)), int(math.floor(y / cell)))].append(p)

    stats["temporal_current_points"] = len(current_far_points)
    active_history = list(history) if temporal_enabled and temporal_frames > 0 else []
    stats["temporal_history_frames"] = len(active_history)
    if active_history:
        history_cells, history_inv = _build_history_grid(active_history, temporal_gate)
    else:
        history_cells, history_inv = {}, 1.0

    remaining = set(cells)
    occupied = list(existing_geometry or [])
    out = []
    fragments = []
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

        current_support = []
        for k in keys:
            current_support.extend(cells[k])
        stats["components"] += 1

        temporal_added = _temporal_support(
            current_support, history_cells, history_inv, temporal_gate,
            temporal_z_gate, temporal_max_added) if temporal_enabled else []
        if temporal_added:
            stats["temporal_components"] += 1
            stats["temporal_added_points"] += len(temporal_added)
        support = list(current_support) + list(temporal_added)

        axis_e = _extent(support)
        if oriented_enabled:
            e, yaw = _oriented_extent(support)
            stats["oriented_components"] += 1
        else:
            e = list(axis_e)
            yaw = 0.0

        hl = max(float(e[0]), float(e[1]))
        hs = min(float(e[0]), float(e[1]))
        h = float(e[2])

        rejected = False
        if len(support) < min_points:
            stats["too_few_points"] += 1
            rejected = True
        under = hl < min_length or hs < min_width or h < min_height
        over = hl > max_length or hs > max_width or h > max_height
        if hl < min_length or hl > max_length:
            stats["length_reject"] += 1
            rejected = True
        if hs < min_width or hs > max_width:
            stats["width_reject"] += 1
            rejected = True
        if h < min_height or h > max_height:
            stats["height_reject"] += 1
            rejected = True
        if rejected:
            # Only undersized, bounded fragments are eligible. Oversized/tall
            # structures remain rejected and can never be recovered.
            if (recovery_enabled and under and not over and
                    len(current_support) <= recovery_max_fragment_points and
                    h <= recovery_fragment_max_height):
                cx, cy, cz = _center(current_support)
                fragments.append({"current": list(current_support),
                                  "temporal": list(temporal_added),
                                  "x": cx, "y": cy, "z": cz})
            continue

        stats["template_pass"] += 1
        # Position is intentionally current-frame-only. Historical points are
        # support evidence, never a source of temporal position lag.
        n = float(len(current_support))
        x = sum(float(p[0]) for p in current_support) / n
        y = sum(float(p[1]) for p in current_support) / n
        z = sum(float(p[2]) for p in current_support) / n
        if _near_existing(x, y, occupied, dedupe):
            stats["dedupe"] += 1
            continue

        item = {"x": x, "y": y, "z": z,
                "point_count": len(support),
                "current_point_count": len(current_support),
                "temporal_point_count": len(temporal_added),
                "extent": e,
                "axis_aligned_extent": axis_e,
                "oriented_yaw": yaw,
                "oriented_extent": e,
                "cluster_mode": "far_geometry_builder",
                "scale_votes": 1, "scale_modes": ["far_geometry_builder"],
                "far_geometry_built": True,
                "far_geometry_quality_v2": True,
                "far_geometry_temporal_supported": bool(temporal_added),
                "sparse_rescued": False, "sparse_discovered": False}
        out.append(item)
        occupied.append(item)
        stats["built"] += 1
        if max_candidates and len(out) >= max_candidates:
            break

    # V0.6.12.6: conservatively bridge nearby current-frame fragments. This
    # pass has no tracker, map, camera, CARLA actor or ground-truth input.
    stats["recovery_fragments"] = len(fragments)
    used = set()
    for seed_index, seed in enumerate(fragments):
        if seed_index in used or (recovery_max_added and
                                  stats["recovery_built"] >= recovery_max_added):
            continue
        group = [seed_index]
        candidates = []
        for other_index, other in enumerate(fragments):
            if other_index == seed_index or other_index in used:
                continue
            dz = abs(float(other["z"]) - float(seed["z"]))
            dxy = math.hypot(float(other["x"]) - float(seed["x"]),
                             float(other["y"]) - float(seed["y"]))
            if dz <= recovery_z_gate and dxy <= recovery_bridge:
                candidates.append((dxy, other_index))
        candidates.sort(key=lambda value: value[0])
        built_group = None
        for _, other_index in candidates[:max(0, recovery_max_fragments - 1)]:
            group.append(other_index)
            current = _unique_points([p for index in group
                                      for p in fragments[index]["current"]])
            temporal = _unique_points([p for index in group
                                       for p in fragments[index]["temporal"]])
            support = _unique_points(list(current) + list(temporal))
            stats["recovery_attempts"] += 1
            if len(current) < recovery_min_current or len(support) < min_points:
                continue
            axis_e = _extent(support)
            if oriented_enabled:
                e, yaw = _oriented_extent(support)
            else:
                e, yaw = list(axis_e), 0.0
            hl = max(float(e[0]), float(e[1]))
            hs = min(float(e[0]), float(e[1]))
            h = float(e[2])
            if not (min_length <= hl <= max_length and
                    min_width <= hs <= max_width and
                    min_height <= h <= max_height):
                continue
            built_group = (list(group), current, temporal, axis_e, e, yaw)
            break
        if built_group is None:
            continue
        indices, current, temporal, axis_e, e, yaw = built_group
        stats["recovery_template_pass"] += 1
        x, y, z = _center(current)
        if _near_existing(x, y, occupied, dedupe):
            stats["recovery_dedupe"] += 1
            used.update(indices)
            continue
        item = {"x": x, "y": y, "z": z,
                "point_count": len(current) + len(temporal),
                "current_point_count": len(current),
                "temporal_point_count": len(temporal),
                "extent": e, "axis_aligned_extent": axis_e,
                "oriented_yaw": yaw, "oriented_extent": e,
                "cluster_mode": "far_geometry_builder",
                "scale_votes": len(indices),
                "scale_modes": ["far_geometry_recovery"],
                "far_geometry_built": True,
                "far_geometry_quality_v2": True,
                "far_geometry_temporal_supported": bool(temporal),
                "far_geometry_recovered": True,
                "recovery_fragment_count": len(indices),
                "sparse_rescued": False, "sparse_discovered": False}
        out.append(item)
        occupied.append(item)
        used.update(indices)
        stats["recovery_built"] += 1
        stats["built"] += 1
        if max_candidates and len(out) >= max_candidates:
            break

    if temporal_enabled and temporal_frames > 0 and current_far_points:
        history.append((now, list(current_far_points)))
        while len(history) > temporal_frames:
            history.popleft()
    elif not temporal_enabled:
        history.clear()
    build_far_geometry_candidates._history = history
    build_far_geometry_candidates.last_stats = stats
    return out


build_far_geometry_candidates._history = deque()
build_far_geometry_candidates.last_stats = _empty_stats()
