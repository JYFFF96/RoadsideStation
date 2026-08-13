from __future__ import print_function


def _recent_sensor(track, now, memory):
    for key in ("last_camera_time", "last_radar_time"):
        ts = track.get(key)
        if ts is not None and now - float(ts) <= float(memory):
            return True
    return False


def cleanup_stale_tracks(tracker, output_items, now, config=None):
    """Remove weak stale internal tracks after normal tracker update.

    V0.6.12 policy:
      * only tracks that are currently unmatched/coasting are considered;
      * require misses >= configured threshold;
      * require quality below configured threshold;
      * recent camera/radar confirmation protects the track;
      * remove from tracker internal state so it cannot resurrect next frame.

    Uses only generic tracker state. No CARLA actor/ground-truth data.
    """
    c = config or {}
    stats = {"checked": 0, "protected": 0, "quality_keep": 0,
             "miss_keep": 0, "cleaned": 0}
    if not c.get("track_stale_cleanup_enabled", False):
        cleanup_stale_tracks.last_stats = stats
        return list(output_items or [])

    min_misses = max(1, int(c.get("track_stale_cleanup_min_misses", 2)))
    max_quality = float(c.get("track_stale_cleanup_max_quality", 0.55))
    sensor_memory = max(.1, float(c.get("track_stale_cleanup_sensor_memory",
                                       c.get("track_quality_sensor_memory", 1.5))))

    internal = getattr(tracker, "_tracks", {})
    remove_ids = set()
    for tid, track in list(internal.items()):
        misses = int(track.get("misses", 0))
        if misses <= 0:
            continue
        stats["checked"] += 1
        if misses < min_misses:
            stats["miss_keep"] += 1
            continue
        if _recent_sensor(track, now, sensor_memory):
            stats["protected"] += 1
            continue
        try:
            quality = float(tracker._quality_value(track, now))
        except Exception:
            quality = 1.0
        if quality >= max_quality:
            stats["quality_keep"] += 1
            continue
        remove_ids.add(tid)

    for tid in remove_ids:
        if tid in internal:
            del internal[tid]
            stats["cleaned"] += 1

    out = [dict(x) for x in (output_items or []) if x.get("id") not in remove_ids]
    cleanup_stale_tracks.last_stats = stats
    return out


cleanup_stale_tracks.last_stats = {
    "checked": 0, "protected": 0, "quality_keep": 0,
    "miss_keep": 0, "cleaned": 0
}
