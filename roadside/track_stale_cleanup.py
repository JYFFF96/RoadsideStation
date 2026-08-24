from __future__ import print_function

import time


def _recent_sensor(track, now, memory):
    for key in ("last_camera_time", "last_radar_time"):
        ts = track.get(key)
        if ts is not None and now - float(ts) <= float(memory):
            return True
    return False


def cleanup_stale_tracks(tracker, output_items, now, config=None):
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
            remember=getattr(tracker,"_remember_camera_tombstone",None)
            if remember is not None:remember(tid,internal.get(tid),now)
            del internal[tid]
            stats["cleaned"] += 1

    out = [dict(x) for x in (output_items or []) if x.get("id") not in remove_ids]
    cleanup_stale_tracks.last_stats = stats
    return out


def install_stale_cleanup_patch():
    """Install conservative V0.6.12 post-update cleanup on NearestTracker."""
    from .tracking import NearestTracker
    if getattr(NearestTracker, "_v0612_stale_patch", False):
        return
    original_configure = NearestTracker.configure_quality
    original_update = NearestTracker.update

    def configure_quality(self, config):
        original_configure(self, config)
        self._stale_cleanup_config = dict(config or {})
        self._stale_last_print = 0.0

    def update(self, detections, timestamp=None):
        now = time.time() if timestamp is None else float(timestamp)
        out = original_update(self, detections, timestamp)
        out = cleanup_stale_tracks(self, out, now, getattr(self, "_stale_cleanup_config", {}))
        s = dict(getattr(self, "last_stats", {}) or {})
        d = dict(cleanup_stale_tracks.last_stats)
        s["stale_checked"] = int(d.get("checked", 0))
        s["stale_protected"] = int(d.get("protected", 0))
        s["stale_quality_keep"] = int(d.get("quality_keep", 0))
        s["stale_miss_keep"] = int(d.get("miss_keep", 0))
        s["stale_cleaned"] = int(d.get("cleaned", 0))
        self.last_stats = s
        if getattr(self, "_stale_cleanup_config", {}).get("track_stale_cleanup_enabled", False):
            if now - float(getattr(self, "_stale_last_print", 0.0)) >= 1.0:
                print("  [TRACK STALE] Checked:%d MissKeep:%d QualityKeep:%d SensorProtected:%d Cleaned:%d" %
                      (s["stale_checked"], s["stale_miss_keep"], s["stale_quality_keep"],
                       s["stale_protected"], s["stale_cleaned"]))
                self._stale_last_print = now
        return out

    NearestTracker.configure_quality = configure_quality
    NearestTracker.update = update
    NearestTracker._v0612_stale_patch = True


cleanup_stale_tracks.last_stats = {
    "checked": 0, "protected": 0, "quality_keep": 0,
    "miss_keep": 0, "cleaned": 0
}
