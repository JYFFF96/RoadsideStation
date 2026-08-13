from __future__ import print_function

import math


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

    def truth_vehicles(self):
        center = self._center();out = []
        if center is None:return out
        for actor in self.world.get_actors().filter("vehicle.*"):
            try:
                role = actor.attributes.get("role_name", "")
                if self.include_roles and role not in self.include_roles:continue
                loc = actor.get_location();distance = self._distance2d(loc, center)
                if distance > self.radius:continue
                vel = actor.get_velocity();extent = actor.bounding_box.extent
                out.append({"actor_id":int(actor.id),"type_id":actor.type_id,"role":role,"x":float(loc.x),"y":float(loc.y),"z":float(loc.z),"vx":float(vel.x),"vy":float(vel.y),"speed":math.hypot(float(vel.x),float(vel.y)),"size":[float(extent.x)*2.0,float(extent.y)*2.0,float(extent.z)*2.0],"range":float(distance)})
            except Exception:continue
        return out

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
        truth=self.truth_vehicles();detected=self._detected_with_range(detected_candidates);pairs=self._match(truth,detected);metrics=self._metrics(len(truth),len(detected),pairs)
        metrics.update({"truth_objects":truth,"pairs":pairs,"range_bins":self._range_metrics(truth,detected)})
        return metrics

    def analyze_roi_false_rejections(self, accepted_candidates, rejected_candidates, min_range=30.0, max_range=50.0):
        """Evaluation-only diagnosis of truth vehicles lost specifically at ROI.

        A truth vehicle is reported only when it has no accepted ROI candidate
        within the normal evaluator gate but does have a rejected candidate.
        Ground truth never feeds back into perception or fusion.
        """
        lo=float(min_range);hi=float(max_range)
        truth=[x for x in self.truth_vehicles() if lo<=x.get("range",0.0)<hi]
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
