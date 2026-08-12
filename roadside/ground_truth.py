from __future__ import print_function

import math


def _distance2d(a, b):
    return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))


def collect_vehicle_truth(world, center, radius=80.0):
    """Return CARLA vehicle ground truth inside a 2D radius around the RSU junction.

    This module is simulation-only and must never be used by production fusion.
    It exists only to evaluate perception recall/position error in CARLA.
    """
    result = []
    for actor in world.get_actors().filter("vehicle.*"):
        try:
            loc = actor.get_location()
            if _distance2d(loc, center) > float(radius):
                continue
            vel = actor.get_velocity()
            bbox = actor.bounding_box
            result.append({
                "actor_id": int(actor.id),
                "type_id": str(actor.type_id),
                "x": float(loc.x),
                "y": float(loc.y),
                "z": float(loc.z),
                "vx": float(vel.x),
                "vy": float(vel.y),
                "speed": math.hypot(float(vel.x), float(vel.y)),
                "length": float(bbox.extent.x) * 2.0,
                "width": float(bbox.extent.y) * 2.0,
                "height": float(bbox.extent.z) * 2.0,
            })
        except Exception:
            pass
    return result


def evaluate_tracks(truth, tracks, max_match_distance=4.0):
    """Greedy one-to-one XY association for evaluation only.

    Returns counts and localization errors. This does not feed any association
    result back into the perception stack, so CARLA truth cannot improve fusion.
    """
    candidates = []
    for ti, t in enumerate(truth or []):
        for pi, p in enumerate(tracks or []):
            d = math.hypot(float(t["x"]) - float(p.get("x", 0.0)),
                           float(t["y"]) - float(p.get("y", 0.0)))
            if d <= float(max_match_distance):
                candidates.append((d, ti, pi))
    candidates.sort(key=lambda x: x[0])
    used_truth = set()
    used_tracks = set()
    errors = []
    pairs = []
    for d, ti, pi in candidates:
        if ti in used_truth or pi in used_tracks:
            continue
        used_truth.add(ti)
        used_tracks.add(pi)
        errors.append(float(d))
        pairs.append({"truth_index": ti, "track_index": pi, "distance": float(d)})

    truth_count = len(truth or [])
    track_count = len(tracks or [])
    matched = len(pairs)
    recall = float(matched) / truth_count if truth_count else 1.0
    precision = float(matched) / track_count if track_count else (1.0 if truth_count == 0 else 0.0)
    mean_error = sum(errors) / len(errors) if errors else None
    max_error = max(errors) if errors else None
    return {
        "truth_count": truth_count,
        "track_count": track_count,
        "matched": matched,
        "recall": recall,
        "precision": precision,
        "mean_xy_error": mean_error,
        "max_xy_error": max_error,
        "pairs": pairs,
    }
