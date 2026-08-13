from __future__ import print_function


class DiscoveryDiagnostics(object):
    """Observer-only diagnostics for V0.6.9/V0.6.10 sparse branches.

    Track rescue candidates use cluster_mode='sparse_rescue'. Track-independent
    discovery candidates use cluster_mode='far_discovery_mid/far'. The observer
    never changes candidates or tracker state; it only counts stage flow and
    follows tracks born from discovery long enough to classify confirmation vs
    one-frame disappearance.
    """

    def __init__(self):
        self._discovery_tracks = {}
        self.last_stats = {}

    @staticmethod
    def _origin(item):
        mode = str((item or {}).get("cluster_mode", ""))
        if mode == "sparse_rescue":
            return "track_rescue"
        if mode.startswith("far_discovery_"):
            return "new_discovery"
        return "normal"

    def _count_origin(self, items, origin):
        return sum(1 for x in (items or []) if self._origin(x) == origin)

    def update(self, geometry, roi, scored, dynamic, tracks):
        stats = {
            "track_rescue_built": self._count_origin(geometry, "track_rescue"),
            "track_rescue_roi": self._count_origin(roi, "track_rescue"),
            "track_rescue_score": self._count_origin(scored, "track_rescue"),
            "track_rescue_dynamic": self._count_origin(dynamic, "track_rescue"),
            "new_discovery_built": self._count_origin(geometry, "new_discovery"),
            "new_discovery_roi": self._count_origin(roi, "new_discovery"),
            "new_discovery_score": self._count_origin(scored, "new_discovery"),
            "new_discovery_dynamic": self._count_origin(dynamic, "new_discovery"),
            "discovery_track_new": 0,
            "discovery_track_confirmed": 0,
            "discovery_track_one_frame_drop": 0,
            "discovery_track_active": 0,
        }

        current = {}
        for t in tracks or []:
            tid = t.get("id")
            if tid:
                current[tid] = t
            if tid and self._origin(t) == "new_discovery" and \
                    str(t.get("track_state", "")) == "new" and \
                    tid not in self._discovery_tracks:
                self._discovery_tracks[tid] = {
                    "seen_frames": 1,
                    "confirmed": False,
                }
                stats["discovery_track_new"] += 1

        for tid in list(self._discovery_tracks):
            state = self._discovery_tracks[tid]
            t = current.get(tid)
            if t is None:
                if not state.get("confirmed", False) and int(state.get("seen_frames", 0)) <= 1:
                    stats["discovery_track_one_frame_drop"] += 1
                del self._discovery_tracks[tid]
                continue

            if str(t.get("track_state", "")) == "confirmed":
                if not state.get("confirmed", False):
                    stats["discovery_track_confirmed"] += 1
                state["confirmed"] = True
            state["seen_frames"] = int(state.get("seen_frames", 0)) + 1

        stats["discovery_track_active"] = len(self._discovery_tracks)
        self.last_stats = stats
        return dict(stats)
