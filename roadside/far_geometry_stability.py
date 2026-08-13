from __future__ import print_function

import time

from .far_geometry_builder import build_far_geometry_candidates


def _count_far(items):
    return sum(1 for x in (items or [])
               if str(x.get("cluster_mode", "")) == "far_geometry_builder")


def install_far_geometry_stability_patch():
    """Attach observer-only downstream counters to SimpleFusion.fuse.

    V0.6.12.1 changes diagnostics only. No candidate, ROI, score, dynamic,
    tracker, CARLA truth, or sensor-fusion decision is modified.
    """
    from .fusion import SimpleFusion
    if getattr(SimpleFusion, "_v06121_far_diag_patch", False):
        return

    original_fuse = SimpleFusion.fuse

    def fuse(self, lidar_points, radar_detections, timestamp=None):
        result = original_fuse(self, lidar_points, radar_detections, timestamp)
        stats = dict(getattr(build_far_geometry_candidates, "last_stats", {}) or {})
        stats["roi_pass"] = _count_far(getattr(self, "last_roi_candidates", []))
        stats["score_pass"] = _count_far(getattr(self, "last_scored_candidates", []))
        stats["dynamic_pass"] = _count_far(getattr(self, "last_dynamic_candidates", []))
        build_far_geometry_candidates.last_stats = stats

        now = time.time() if timestamp is None else float(timestamp)
        last_print = float(getattr(self, "_far_geometry_diag_last_print", 0.0))
        if now - last_print >= 1.0:
            print("  [FAR GEOMETRY DETAIL] InputPts:%d Components:%d TooFew:%d "
                  "LengthReject:%d WidthReject:%d HeightReject:%d TemplatePass:%d "
                  "Dedupe:%d Built:%d ROI:%d Score:%d Dynamic:%d" %
                  (stats.get("input_points", 0), stats.get("components", 0),
                   stats.get("too_few_points", 0), stats.get("length_reject", 0),
                   stats.get("width_reject", 0), stats.get("height_reject", 0),
                   stats.get("template_pass", 0), stats.get("dedupe", 0),
                   stats.get("built", 0), stats.get("roi_pass", 0),
                   stats.get("score_pass", 0), stats.get("dynamic_pass", 0)))
            self._far_geometry_diag_last_print = now
        return result

    SimpleFusion.fuse = fuse
    SimpleFusion._v06121_far_diag_patch = True
