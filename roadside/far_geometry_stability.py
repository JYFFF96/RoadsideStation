from __future__ import print_function

import time
from collections import defaultdict

from .far_geometry_builder import build_far_geometry_candidates


def _is_far_builder(item):
    return str((item or {}).get("cluster_mode", "")) == "far_geometry_builder"


def _count_far(items):
    return sum(1 for x in (items or []) if _is_far_builder(x))


def _far_items(items):
    return [x for x in (items or []) if _is_far_builder(x)]


def _recovery_items(items):
    return [x for x in (items or []) if x.get("far_geometry_recovered", False)]


def _reason_counts(items):
    out = defaultdict(int)
    for item in items or []:
        out[str(item.get("reason", "other"))] += 1
    return dict(out)


def _detail_value(details, *names):
    for name in names:
        if name in details and details.get(name) is not None:
            return details.get(name)
    return None


def _fmt(value):
    if value is None:
        return "-"
    try:
        return "%.2f" % float(value)
    except Exception:
        return str(value)


def install_far_geometry_stability_patch():
    """Attach observer-only Far Geometry / ROI / temporal diagnostics."""
    from .fusion import SimpleFusion
    if getattr(SimpleFusion, "_v06122_far_roi_diag_patch", False):
        return

    original_fuse = SimpleFusion.fuse

    def fuse(self, lidar_points, radar_detections, timestamp=None, frame_id=None):
        result = original_fuse(self, lidar_points, radar_detections, timestamp,
                               frame_id=frame_id)

        stats = dict(getattr(build_far_geometry_candidates, "last_stats", {}) or {})
        roi_pass_items = _far_items(getattr(self, "last_roi_candidates", []))
        roi_reject_items = _far_items(getattr(self, "last_roi_rejections", []))
        score_pass_items = _far_items(getattr(self, "last_scored_candidates", []))
        score_reject_items = _far_items(getattr(self, "last_score_rejections", []))
        dynamic_items = _far_items(getattr(self, "last_dynamic_candidates", []))
        recovery_roi = _recovery_items(getattr(self, "last_roi_candidates", []))
        recovery_score = _recovery_items(getattr(self, "last_scored_candidates", []))
        recovery_quality = _recovery_items(
            getattr(self, "last_recovery_quality_candidates", []))
        recovery_quality_reject = _recovery_items(
            getattr(self, "last_recovery_quality_rejections", []))
        recovery_dynamic = _recovery_items(getattr(self, "last_dynamic_candidates", []))
        recovery_track = _recovery_items(getattr(self, "last_tracked_candidates", []))

        roi_reasons = _reason_counts(roi_reject_items)
        score_reasons = _reason_counts(score_reject_items)
        corridor_items = roi_pass_items + roi_reject_items
        adaptive_items = [x for x in corridor_items
                          if ((x.get("roi_details", {}) or x.get("details", {}) or {})
                              .get("adaptive_corridor", {}).get("enabled", False))]
        adaptive_rescued = 0
        for item in roi_pass_items:
            details = item.get("roi_details", {}) or {}
            adaptive = details.get("adaptive_corridor", {}) or {}
            if not adaptive.get("enabled", False):
                continue
            base_allowed = (0.5 * float(details.get("lane_width", 0.0)) +
                            float(adaptive.get("base_margin", 0.0)))
            direct_rescue = float(details.get("lateral", 0.0)) > base_allowed
            if direct_rescue or details.get("geometry_rescued", False):
                adaptive_rescued += 1

        stats["roi_pass"] = len(roi_pass_items)
        stats["roi_reject"] = len(roi_reject_items)
        stats["roi_reject_reasons"] = roi_reasons
        stats["score_pass"] = len(score_pass_items)
        stats["score_reject"] = len(score_reject_items)
        stats["score_reject_reasons"] = score_reasons
        stats["dynamic_pass"] = len(dynamic_items)
        stats["adaptive_corridor_candidates"] = len(adaptive_items)
        stats["adaptive_corridor_rescued"] = adaptive_rescued
        stats["recovery_roi_pass"] = len(recovery_roi)
        stats["recovery_score_pass"] = len(recovery_score)
        stats["recovery_quality_pass"] = len(recovery_quality)
        stats["recovery_quality_reject"] = len(recovery_quality_reject)
        stats["recovery_dynamic_pass"] = len(recovery_dynamic)
        stats["recovery_track_pass"] = len(recovery_track)
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

            print("  [FAR TEMPORAL] HistoryFrames:%d CurrentPts:%d AddedPts:%d Components:%d" %
                  (stats.get("temporal_history_frames", 0),
                   stats.get("temporal_current_points", 0),
                   stats.get("temporal_added_points", 0),
                   stats.get("temporal_components", 0)))

            print("  [FAR GEOMETRY RECOVERY] Fragments:%d Attempts:%d TemplatePass:%d "
                  "Dedupe:%d Built:%d" %
                  (stats.get("recovery_fragments", 0),
                   stats.get("recovery_attempts", 0),
                   stats.get("recovery_template_pass", 0),
                   stats.get("recovery_dedupe", 0),
                   stats.get("recovery_built", 0)))

            print("  [FAR RECOVERY FLOW] Built:%d ROI:%d Score:%d QualityPass:%d "
                  "QualityReject:%d Dynamic:%d Track:%d" %
                  (stats.get("recovery_built", 0),
                   stats.get("recovery_roi_pass", 0),
                   stats.get("recovery_score_pass", 0),
                   stats.get("recovery_quality_pass", 0),
                   stats.get("recovery_quality_reject", 0),
                   stats.get("recovery_dynamic_pass", 0),
                   stats.get("recovery_track_pass", 0)))

            print("  [FAR ROI DIAG] Built:%d ROIPass:%d ROIReject:%d "
                  "ScorePass:%d ScoreReject:%d Dynamic:%d" %
                  (stats.get("built", 0), stats.get("roi_pass", 0),
                   stats.get("roi_reject", 0), stats.get("score_pass", 0),
                   stats.get("score_reject", 0), stats.get("dynamic_pass", 0)))

            print("  [FAR ROI REJECT] lateral:%d above_road:%d other:%d" %
                  (roi_reasons.get("lateral", 0), roi_reasons.get("above_road", 0),
                   sum(v for k, v in roi_reasons.items()
                       if k not in ("lateral", "above_road"))))

            margins=[]
            for item in adaptive_items:
                d=item.get("roi_details", {}) or item.get("details", {}) or {}
                a=d.get("adaptive_corridor", {}) or {}
                if a.get("final_margin") is not None:margins.append(float(a["final_margin"]))
            print("  [FAR ADAPTIVE CORRIDOR] Candidates:%d Rescued:%d Margin(avg/max):%s/%s" %
                  (len(adaptive_items),adaptive_rescued,
                   _fmt(sum(margins)/len(margins) if margins else None),
                   _fmt(max(margins) if margins else None)))

            for item in roi_reject_items[:3]:
                details = item.get("details", {}) or {}
                try:
                    rng = self._sensor_range(item.get("x", 0.0), item.get("y", 0.0))
                except Exception:
                    rng = None
                extent = item.get("extent", [0.0, 0.0, 0.0]) or [0.0, 0.0, 0.0]
                lateral = _detail_value(details, "lateral")
                allowed = _detail_value(details, "allowed_lateral", "limit")
                excess = _detail_value(details, "center_excess")
                overlap = _detail_value(details, "bbox_overlap", "overlap")
                print("    [FAR ROI SAMPLE] range:%sm reason:%s pts:%d "
                      "extent:(%.2f,%.2f,%.2f) lateral:%s allowed:%s "
                      "excess:%s overlap:%s" %
                      (_fmt(rng), item.get("reason", "other"),
                       int(item.get("point_count", 0)), float(extent[0]),
                       float(extent[1]), float(extent[2]), _fmt(lateral),
                       _fmt(allowed), _fmt(excess), _fmt(overlap)))

            if score_reject_items:
                print("  [FAR SCORE REJECT] count:%d reasons:%s" %
                      (len(score_reject_items), score_reasons))

            self._far_geometry_diag_last_print = now
        return result

    SimpleFusion.fuse = fuse
    SimpleFusion._v06122_far_roi_diag_patch = True
