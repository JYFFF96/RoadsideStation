from __future__ import division


def _number(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def selected_delayed_risk_gate_passes(item, rule):
    """Apply one sensor-only delayed-reappearance risk rule."""
    item = item or {};rule = rule or {}
    origin = item.get("selected_delayed_reappearance_origin", {}) or {}
    extent = item.get("extent") or []
    fields = {
        "score": item.get("selected_admission_shadow_score",
                          item.get("candidate_score")),
        "origin_score": origin.get("selected_admission_shadow_score",
                                   origin.get("candidate_score")),
        "points": item.get("current_point_count", item.get("point_count")),
        "origin_points": origin.get("current_point_count",
                                    origin.get("point_count")),
        "height": extent[2] if len(extent) > 2 else None,
        "match_distance": item.get(
            "selected_delayed_reappearance_match_distance"),
        "time_gap": item.get("selected_delayed_reappearance_time_gap"),
    }
    gap = _number(fields["time_gap"]);distance = _number(fields["match_distance"])
    fields["apparent_speed"] = (distance / gap if distance is not None and
                                 gap is not None and gap > 0.0 else None)
    for feature in ("score", "origin_score", "points", "origin_points",
                    "height", "match_distance", "time_gap", "apparent_speed"):
        for prefix, comparison in (("min_", lambda value, limit: value >= limit),
                                   ("max_", lambda value, limit: value <= limit)):
            key = prefix + feature
            if key not in rule:continue
            value = _number(fields.get(feature));limit = _number(rule.get(key))
            # Missing sensor evidence fails closed and cannot inflate retention.
            if value is None or limit is None or not comparison(value, limit):
                return False
    return True
