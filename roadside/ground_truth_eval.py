from __future__ import print_function

import math

from .object_taxonomy import carla_actor_class, iter_carla_road_actors, object_group


class GroundTruthEvaluator(object):
    """CARLA-only truth evaluator.

    This module is deliberately isolated from perception/fusion. Real-device
    runtime does not require it; it is only a validation adapter for CARLA.
    """

    def __init__(self, world, center_provider, config=None):
        self.world = world
        self.center_provider = center_provider
        self.config = config or {}
        self.radius = float(self.config.get("radius", 80.0))
        self.match_distance = float(self.config.get("match_distance", 4.0))
        self.range_bins = self._parse_bins(self.config.get("range_bins", [30.0, 50.0, 80.0]))
        self.include_roles = set(self.config.get("include_roles", ["autopilot", "roadside_autopilot", "rsu_local_autopilot"]))
        self._far_admission_last_frame = None
        self._far_admission_totals = self._empty_admission_totals()

    def _parse_bins(self, values):
        bins = []
        for value in values or []:
            try:value = float(value)
            except Exception:continue
            if value > 0.0 and value <= self.radius and value not in bins:bins.append(value)
        bins.sort()
        if not bins or abs(bins[-1] - self.radius) > 1e-6:bins.append(self.radius)
        return bins

    @staticmethod
    def _distance2d(a, b):return math.hypot(float(a.x) - float(b.x), float(a.y) - float(b.y))
    def _center(self):return self.center_provider()

    def truth_objects(self):
        center = self._center();out = []
        if center is None:return out
        obstacle_patterns = self.config.get("obstacle_actor_patterns", [])
        for actor in iter_carla_road_actors(self.world, obstacle_patterns):
            try:
                role = actor.attributes.get("role_name", "")
                object_type = carla_actor_class(actor)
                group = object_group(object_type)
                if group == "vehicle" and self.include_roles and role not in self.include_roles:continue
                if object_type == "person" and not self.config.get("include_walkers", True):continue
                loc = actor.get_location();distance = self._distance2d(loc, center)
                if distance > self.radius:continue
                vel = actor.get_velocity();extent = actor.bounding_box.extent
                out.append({"actor_id":int(actor.id),"type_id":actor.type_id,
                            "object_type":object_type,"object_group":group,"role":role,
                            "x":float(loc.x),"y":float(loc.y),"z":float(loc.z),
                            "vx":float(vel.x),"vy":float(vel.y),
                            "speed":math.hypot(float(vel.x),float(vel.y)),
                            "size":[float(extent.x)*2.0,float(extent.y)*2.0,float(extent.z)*2.0],
                            "range":float(distance)})
            except Exception:continue
        return out

    def truth_vehicles(self):
        """Compatibility view for vehicle-specific legacy diagnostics."""
        return [x for x in self.truth_objects() if x.get("object_group") == "vehicle"]

    def _detected_with_range(self, detected):
        center = self._center();out = []
        if center is None:return out
        for det in detected or []:
            item = dict(det);item["range"] = math.hypot(float(item.get("x",0.0))-float(center.x),float(item.get("y",0.0))-float(center.y))
            if item["range"] <= self.radius:out.append(item)
        return out

    def _match(self, truth, detected):
        candidates=[]
        for ti,gt in enumerate(truth):
            for di,det in enumerate(detected or []):
                d=math.hypot(float(det.get("x",0.0))-gt["x"],float(det.get("y",0.0))-gt["y"])
                if d<=self.match_distance:candidates.append((d,ti,di))
        candidates.sort(key=lambda x:x[0]);used_t=set();used_d=set();pairs=[]
        for d,ti,di in candidates:
            if ti in used_t or di in used_d:continue
            used_t.add(ti);used_d.add(di);pairs.append((ti,di,d))
        return pairs

    @staticmethod
    def _metrics(truth_n, detected_n, pairs):
        matched_n=len(pairs);false_pos=max(0,detected_n-matched_n);missed=max(0,truth_n-matched_n);errors=[p[2] for p in pairs]
        return {"truth":truth_n,"detected":detected_n,"matched":matched_n,"missed":missed,"false_positive":false_pos,"recall":(float(matched_n)/truth_n) if truth_n else None,"precision":(float(matched_n)/detected_n) if detected_n else None,"mean_position_error":(sum(errors)/len(errors)) if errors else None,"max_position_error":max(errors) if errors else None}

    def _range_metrics(self, truth, detected):
        result=[];lower=0.0
        for upper in self.range_bins:
            gt_bin=[x for x in truth if lower<=x.get("range",0.0)<upper or (upper==self.range_bins[-1] and lower<=x.get("range",0.0)<=upper)]
            det_bin=[x for x in detected if lower<=x.get("range",0.0)<upper or (upper==self.range_bins[-1] and lower<=x.get("range",0.0)<=upper)]
            pairs=self._match(gt_bin,det_bin);metrics=self._metrics(len(gt_bin),len(det_bin),pairs);metrics["min_range"]=lower;metrics["max_range"]=upper;result.append(metrics);lower=upper
        return result

    def evaluate_candidates(self, detected_candidates):
        truth=self.truth_objects();detected=self._detected_with_range(detected_candidates);pairs=self._match(truth,detected);metrics=self._metrics(len(truth),len(detected),pairs)
        matched_truth=set(x[0] for x in pairs);classes={}
        for index,item in enumerate(truth):
            name=item.get("object_type","unknown_obstacle")
            bucket=classes.setdefault(name,{"truth":0,"matched":0,"missed":0})
            bucket["truth"]+=1
            if index in matched_truth:bucket["matched"]+=1
            else:bucket["missed"]+=1
        for bucket in classes.values():
            bucket["recall"]=(float(bucket["matched"])/bucket["truth"] if bucket["truth"] else None)
        metrics.update({"truth_objects":truth,"pairs":pairs,"range_bins":self._range_metrics(truth,detected),"class_metrics":classes})
        return metrics

    @staticmethod
    def _empty_admission_totals():
        def feature_bucket():
            return {
                "count": 0,
                "scores": [], "points": [], "ranges": [], "edge_ratios": [],
                "lengths": [], "widths": [], "heights": [],
                "recovery": 0, "sparse": 0, "temporal": 0,
                "far_builder": 0, "score_bypass": 0, "radar": 0,
                "cluster_modes": {},
            }
        def risk_bucket():
            return {
                "total": 0, "kept": 0, "rejected": 0,
                "hard_edge": 0, "soft_risk": 0, "unknown_edge": 0,
            }
        return {
            "frames": 0,
            "would_hold": 0, "would_hold_truth": 0, "would_hold_fp": 0,
            "would_confirm": 0, "would_confirm_truth": 0, "would_confirm_fp": 0,
            "expired": 0, "expired_truth": 0, "expired_fp": 0,
            "match_distances": [], "time_gaps": [], "frame_gaps": [],
            "profiles": dict((decision, {
                "truth": feature_bucket(), "fp": feature_bucket()})
                for decision in ("would_hold", "would_confirm", "expired")),
            "risk_shadow": dict((decision, {
                "truth": risk_bucket(), "fp": risk_bucket()})
                for decision in ("would_hold", "would_confirm", "expired")),
            "risk_classes": dict((decision, {})
                                 for decision in ("would_hold", "would_confirm", "expired")),
        }

    @staticmethod
    def _percentile(values, fraction):
        ordered = sorted(float(x) for x in values)
        if not ordered:return None
        if len(ordered) == 1:return ordered[0]
        position = max(0.0, min(1.0, float(fraction))) * (len(ordered) - 1)
        lower = int(math.floor(position));upper = int(math.ceil(position))
        if lower == upper:return ordered[lower]
        weight = position - lower
        return ordered[lower] * (1.0 - weight) + ordered[upper] * weight

    @classmethod
    def _distribution(cls, values):
        clean = [float(x) for x in values if x is not None]
        return {
            "samples": len(clean),
            "mean": (sum(clean) / len(clean)) if clean else None,
            "p50": cls._percentile(clean, 0.50),
            "p90": cls._percentile(clean, 0.90),
            "max": max(clean) if clean else None,
        }

    def _admission_classification(self, truth, candidates):
        detected = self._detected_with_range(candidates)
        pairs = self._match(truth, detected)
        return detected, pairs

    @staticmethod
    def _append_feature(bucket, name, value):
        if value is None:return
        try:bucket[name].append(float(value))
        except (TypeError, ValueError):pass

    def _record_admission_features(self, bucket, item):
        bucket["count"] += 1
        self._append_feature(bucket, "scores", item.get("candidate_score"))
        self._append_feature(bucket, "points", item.get(
            "current_point_count", item.get("point_count")))
        self._append_feature(bucket, "ranges", item.get("range"))
        extent = list(item.get("extent", [None, None, None]) or [None, None, None])
        while len(extent) < 3:extent.append(None)
        self._append_feature(bucket, "lengths", extent[0])
        self._append_feature(bucket, "widths", extent[1])
        self._append_feature(bucket, "heights", extent[2])
        details = item.get("roi_details", {}) or item.get("details", {}) or {}
        lateral = details.get("lateral");allowed = details.get("allowed_lateral")
        try:
            if lateral is not None and float(allowed) > 0.0:
                self._append_feature(bucket, "edge_ratios",
                                     abs(float(lateral)) / float(allowed))
        except (TypeError, ValueError):pass
        flags = {
            "recovery": bool(item.get("far_geometry_recovered", False)),
            "sparse": bool(item.get("sparse_rescued", False)),
            "temporal": bool(item.get("far_geometry_temporal_supported", False)),
            "far_builder": str(item.get("cluster_mode", "")) == "far_geometry_builder",
            "score_bypass": bool(item.get("candidate_score_bypass", False)),
            "radar": item.get("radar_radial_velocity") is not None,
        }
        for name, enabled in flags.items():
            if enabled:bucket[name] += 1
        mode = str(item.get("cluster_mode", "unknown"))
        bucket["cluster_modes"][mode] = int(bucket["cluster_modes"].get(mode, 0)) + 1

    @staticmethod
    def _admission_edge_ratio(item):
        details = item.get("roi_details", {}) or item.get("details", {}) or {}
        lateral = details.get("lateral");allowed = details.get("allowed_lateral")
        try:
            if lateral is not None and float(allowed) > 0.0:
                return abs(float(lateral)) / float(allowed)
        except (TypeError, ValueError):pass
        return None

    def _admission_shadow_risk_reason(self, item):
        """Return the simulated V0.6.12.7.4 decision reason.

        This method runs only in the CARLA evaluator. Its result is counted for
        diagnostics and is never returned to Fusion, Tracker or ObjectList.
        """
        edge = self._admission_edge_ratio(item)
        if edge is None:return "unknown_edge"
        hard = float(self.config.get("far_admission_edge_hard_ratio", 0.65))
        soft = float(self.config.get("far_admission_edge_soft_ratio", 0.35))
        soft_score = float(self.config.get("far_admission_edge_soft_score", 0.68))
        if edge >= hard:return "hard_edge"
        mode = str(item.get("cluster_mode", ""))
        risk_modes = set(self.config.get("far_admission_edge_risk_modes",
                                         ["far_geometry_builder", "far_discovery_far"]))
        risky_source = (mode in risk_modes or
                        bool(item.get("far_geometry_recovered", False)) or
                        bool(item.get("far_geometry_temporal_supported", False)) or
                        bool(item.get("sparse_rescued", False)))
        score = item.get("candidate_score")
        try:low_score = score is not None and float(score) < soft_score
        except (TypeError, ValueError):low_score = False
        if edge >= soft and (low_score or risky_source):return "soft_risk"
        return "keep"

    def _record_admission_shadow_risk(self, bucket, item):
        reason = self._admission_shadow_risk_reason(item)
        bucket["total"] += 1
        if reason in ("hard_edge", "soft_risk"):
            bucket["rejected"] += 1
        else:
            bucket["kept"] += 1
        if reason in bucket:bucket[reason] += 1

    def _record_admission_risk_class(self, classes, object_type, item):
        bucket = classes.setdefault(str(object_type), {
            "total": 0, "kept": 0, "rejected": 0,
            "hard_edge": 0, "soft_risk": 0, "unknown_edge": 0,
        })
        self._record_admission_shadow_risk(bucket, item)

    def _summarize_feature_bucket(self, bucket):
        result = {"count": int(bucket.get("count", 0))}
        for source in ("recovery", "sparse", "temporal", "far_builder",
                       "score_bypass", "radar"):
            result[source] = int(bucket.get(source, 0))
        result["cluster_modes"] = dict(bucket.get("cluster_modes", {}))
        for name in ("scores", "points", "ranges", "edge_ratios",
                     "lengths", "widths", "heights"):
            result[name] = self._distribution(bucket.get(name, []))
        return result

    def observe_far_admission_decisions(self, held_candidates, admitted_candidates,
                                        expired_candidates, frame_id=None):
        """Accumulate admission labels once per real LiDAR frame for evaluation only.

        CARLA truth is consumed exclusively inside this evaluator. The returned
        labels are never passed to Fusion, Tracker, admission state or ObjectList.
        """
        if frame_id is not None and frame_id == self._far_admission_last_frame:
            return False
        if frame_id is None and not (held_candidates or admitted_candidates or expired_candidates):
            return False
        self._far_admission_last_frame = frame_id
        held = list(held_candidates or [])
        confirmed = [x for x in (admitted_candidates or [])
                     if x.get("far_track_admission_reason") == "repeat"]
        expired = list(expired_candidates or [])
        truth = self.truth_objects()
        totals = self._far_admission_totals
        totals["frames"] += 1
        for name, candidates in (("would_hold", held),
                                 ("would_confirm", confirmed),
                                 ("expired", expired)):
            detected, pairs = self._admission_classification(truth, candidates)
            matched_indices = set(pair[1] for pair in pairs)
            matched_types = dict((pair[1], truth[pair[0]].get(
                "object_type", "unknown_obstacle")) for pair in pairs)
            count = len(detected);matched = len(matched_indices)
            false_positive = max(0, count - matched)
            totals[name] += count
            totals[name + "_truth"] += matched
            totals[name + "_fp"] += false_positive
            profile = totals["profiles"][name]
            risk_profile = totals["risk_shadow"][name]
            for index, item in enumerate(detected):
                label = "truth" if index in matched_indices else "fp"
                self._record_admission_features(profile[label], item)
                if self.config.get("far_admission_edge_risk_shadow", False):
                    self._record_admission_shadow_risk(risk_profile[label], item)
                    if label == "truth":
                        self._record_admission_risk_class(
                            totals["risk_classes"][name], matched_types[index], item)
        for item in held + confirmed:
            value = item.get("far_track_admission_match_distance")
            if value is not None:totals["match_distances"].append(float(value))
            value = item.get("far_track_admission_time_gap")
            if value is not None:totals["time_gaps"].append(float(value))
            value = item.get("far_track_admission_frame_gap")
            if value is not None:totals["frame_gaps"].append(float(value))
        return True

    def report_far_admission_decisions(self, reset=True):
        totals = self._far_admission_totals
        result = dict((k, v) for k, v in totals.items()
                      if k not in ("match_distances", "time_gaps", "frame_gaps",
                                   "profiles", "risk_shadow", "risk_classes"))
        result["candidate_jump"] = self._distribution(totals["match_distances"])
        result["time_gap"] = self._distribution(totals["time_gaps"])
        result["frame_gap"] = self._distribution(totals["frame_gaps"])
        result["feature_profiles"] = dict((decision, dict(
            (label, self._summarize_feature_bucket(bucket))
            for label, bucket in labels.items()))
            for decision, labels in totals["profiles"].items())
        result["edge_risk_shadow"] = dict((decision, dict(
            (label, dict(bucket)) for label, bucket in labels.items()))
            for decision, labels in totals["risk_shadow"].items())
        result["edge_risk_classes"] = dict((decision, dict(
            (name, dict(bucket)) for name, bucket in classes.items()))
            for decision, classes in totals["risk_classes"].items())
        if reset:self._far_admission_totals = self._empty_admission_totals()
        return result

    def _has_match(self, gt, candidates):
        for det in candidates or []:
            d=math.hypot(float(det.get("x",0.0))-float(gt.get("x",0.0)),float(det.get("y",0.0))-float(gt.get("y",0.0)))
            if d<=self.match_distance:return True
        return False

    @staticmethod
    def _empty_drop_counts():
        return {"truth":0,"pass":0,"no_geometry_candidate":0,"roi_reject":0,"roi_lost":0,"score_reject":0,"score_lost":0,"dynamic_drop":0}

    def analyze_detection_drop_reasons(self, geometry_candidates, roi_candidates, scored_candidates,
                                       dynamic_candidates, roi_rejections=None, score_rejections=None):
        """V0.6.8 evaluation-only stage drop diagnosis.

        Ground truth is used only to explain where a truth vehicle disappears from
        the already-produced perception stages. It never feeds back into perception,
        tracking, fusion or FusedObjectList.

        `no_geometry_candidate` deliberately means no candidate survived into the
        Geometry stage. With the current pipeline this cannot yet distinguish raw
        point-support failure from an earlier clustering/geometry rejection.
        """
        truth=self.truth_objects()
        geo=self._detected_with_range(geometry_candidates)
        roi=self._detected_with_range(roi_candidates)
        scored=self._detected_with_range(scored_candidates)
        dyn=self._detected_with_range(dynamic_candidates)
        roi_rej=self._detected_with_range(roi_rejections)
        score_rej=self._detected_with_range(score_rejections)
        total=self._empty_drop_counts();bins=[];lower=0.0
        for upper in self.range_bins:
            c=self._empty_drop_counts();c["min_range"]=lower;c["max_range"]=upper;bins.append(c);lower=upper
        details=[]
        for gt in truth:
            if not self._has_match(gt,geo):reason="no_geometry_candidate"
            elif not self._has_match(gt,roi):reason="roi_reject" if self._has_match(gt,roi_rej) else "roi_lost"
            elif not self._has_match(gt,scored):reason="score_reject" if self._has_match(gt,score_rej) else "score_lost"
            elif not self._has_match(gt,dyn):reason="dynamic_drop"
            else:reason="pass"
            total["truth"]+=1;total[reason]+=1
            rng=float(gt.get("range",0.0));lower=0.0
            for b in bins:
                upper=float(b["max_range"])
                if (lower<=rng<upper) or (upper==self.range_bins[-1] and lower<=rng<=upper):
                    b["truth"]+=1;b[reason]+=1;break
                lower=upper
            details.append({"actor_id":gt.get("actor_id"),"range":rng,"reason":reason})
        total["range_bins"]=bins;total["details"]=details
        return total

    def analyze_roi_false_rejections(self, accepted_candidates, rejected_candidates, min_range=30.0, max_range=50.0):
        """Evaluation-only diagnosis of truth vehicles lost specifically at ROI.

        A truth road object is reported only when it has no accepted ROI candidate
        within the normal evaluator gate but does have a rejected candidate.
        Ground truth never feeds back into perception or fusion.
        """
        lo=float(min_range);hi=float(max_range)
        truth=[x for x in self.truth_objects() if lo<=x.get("range",0.0)<hi]
        accepted=[x for x in self._detected_with_range(accepted_candidates) if lo<=x.get("range",0.0)<hi]
        rejected=[x for x in self._detected_with_range(rejected_candidates) if lo<=x.get("range",0.0)<hi]
        accepted_pairs=self._match(truth,accepted);covered=set(p[0] for p in accepted_pairs)
        remaining=[(idx,gt) for idx,gt in enumerate(truth) if idx not in covered]
        if not remaining or not rejected:return []
        reduced_truth=[gt for _,gt in remaining];pairs=self._match(reduced_truth,rejected);out=[]
        for ti,di,d in pairs:
            original_index,gt=remaining[ti];rej=rejected[di];details=rej.get("details",{}) or {}
            out.append({
                "actor_id":gt.get("actor_id"),"type_id":gt.get("type_id"),"truth_range":gt.get("range"),
                "reason":rej.get("reason","rejected"),"candidate_distance":float(d),
                "lateral":details.get("lateral"),"allowed_lateral":details.get("allowed_lateral"),
                "center_excess":details.get("center_excess"),"bbox_overlap":details.get("bbox_overlap"),
                "roi_margin":details.get("roi_margin"),"candidate_range":rej.get("range")
            })
        out.sort(key=lambda x:(x.get("truth_range",0.0),x.get("candidate_distance",0.0)))
        return out

    def evaluate(self, detected_tracks, camera_objects=None, camera_pairs=None, radar_matched=0):
        metrics=self.evaluate_candidates(detected_tracks)
        metrics.update({"camera_visible":len(camera_objects or []),"camera_lidar_matched":len(camera_pairs or []),"radar_matched":int(radar_matched)})
        return metrics
