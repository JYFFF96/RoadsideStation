from __future__ import print_function

import math

from .object_taxonomy import carla_actor_class, iter_carla_road_actors, object_group
from .selected_camera_support import selected_camera_rescue_passes
from .selected_delayed_risk import selected_delayed_risk_gate_passes


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
        self.camera_ground_identity_match_gates = sorted(set(
            float(x) for x in self.config.get(
                "camera_ground_identity_match_gates", [1.0,2.0,3.0,4.0])
            if float(x)>0.0))
        self.camera_ground_reassociation_shadow_gates = sorted(set(
            float(x) for x in self.config.get(
                "camera_ground_reassociation_shadow_gates", [4.0,5.0,6.0,8.0])
            if float(x)>0.0))
        self.camera_ground_tombstone_shadow_gates = sorted(set(
            float(x) for x in self.config.get(
                "camera_ground_tombstone_shadow_gates", [2.0,3.5,5.0])
            if float(x)>0.0))
        self.camera_ground_tombstone_feature_mode = str(self.config.get(
            "camera_ground_tombstone_feature_mode","predicted"))
        self.camera_ground_tombstone_feature_gate = float(self.config.get(
            "camera_ground_tombstone_feature_gate",2.0))
        self.camera_ground_tombstone_feature_rules = list(self.config.get(
            "camera_ground_tombstone_feature_rules",[]) or [])
        self.range_bins = self._parse_bins(self.config.get("range_bins", [30.0, 50.0, 80.0]))
        self.include_roles = set(self.config.get("include_roles", ["autopilot", "roadside_autopilot", "rsu_local_autopilot"]))
        self._far_admission_last_frame = None
        self._far_admission_totals = self._empty_admission_totals()
        self._selected_track_admission_last_frame = None
        self._selected_track_admission_totals = self._empty_selected_track_admission_totals()
        self._selected_track_admission_frame = {}
        self._selected_delayed_reappearance_last_frame = None
        self._selected_delayed_reappearance_totals = {}
        self._road_object_samples = {"classes": {}, "false": []}
        self._adaptive_temporal_samples = {"classes": {}, "false": []}
        self._hybrid_selection_samples = {}
        self._hybrid_rescue_samples = {}
        self._road_object_actor_coverage = {}
        self._road_object_stage_coverage = {}
        self._road_object_cap_totals = self._empty_road_object_cap_totals()
        self._selected_enforcement_totals = self._empty_selected_enforcement_totals()
        self._selected_admission_score_totals = self._empty_selected_admission_score_totals()
        self._road_object_benchmark_ids = set()
        self._road_object_benchmark_active = False
        self._road_object_benchmark_generation = 0
        self._truth_lifecycle_prev = {}
        self._truth_lifecycle_totals = {"entered":0,"boundary_exit":0,
                                        "unexpected_exit":0,"teleport":0}
        self._radar_seed_bridge_last_frame = None
        self._radar_seed_bridge_totals = {}
        self._radar_camera_support_last_frame = None
        self._radar_camera_support_totals = {
            "candidates":0,"visible":0,"supported":0,
            "truth":0,"fp":0,"supported_truth":0,"supported_fp":0,
            "truth_classes":{},"camera_classes":{},"sources":{}}
        self._camera_ground_last_frame = None
        self._camera_ground_totals = {
            "candidates":0,"matched":0,"fp":0,"classes":{},"sources":{}}
        self._camera_ground_vru_roi_last_frame = None
        self._camera_ground_vru_roi_totals = {}
        self._camera_ground_temporal_last_frame = None
        self._camera_ground_temporal_totals = {
            "candidates":0,"matched":0,"fp":0,"classes":{},"sources":{}}
        self._camera_ground_counterfactual_last_frame = None
        self._camera_ground_counterfactual_totals = {
            "frames":0,"truth":0,"base_matched":0,"camera_candidates":0,
            "camera_truth":0,"camera_fp":0,"incremental_matched":0,
            "combined_matched":0,"classes":{},"sources":{}}
        self._camera_ground_enforcement_last_frame = None
        self._camera_ground_enforcement_totals = self._empty_camera_enforcement_totals()
        self._camera_ground_enforcement_totals["identity_gates"] = \
            self._empty_camera_identity_gates()
        self._camera_ground_enforcement_totals["reassociation_gates"] = \
            self._empty_camera_reassociation_gates()
        self._camera_ground_enforcement_totals["tombstone_gates"] = \
            self._empty_camera_tombstone_gates()
        self._camera_ground_enforcement_totals["tombstone_feature"] = \
            self._empty_camera_tombstone_feature()
        self._camera_ground_enforcement_totals["tombstone_feature_rules"] = \
            self._empty_camera_tombstone_feature_rules()
        self._camera_ground_enforcement_current = self._empty_camera_enforcement_current()

    @staticmethod
    def _empty_road_object_cap_totals():
        return dict((name,{"candidates":0,"matched":0,"fp":0,"classes":{}})
                    for name in ("baseline","balanced","adaptive","adaptive_ranked",
                                 "adaptive_stratified","adaptive_hybrid",
                                 "adaptive_hybrid_gated","adaptive_hybrid_rescued",
                                 "adaptive_hybrid_geometry_gated","selected"))

    @staticmethod
    def _empty_selected_enforcement_totals():
        return dict((name,{"candidates":0,"matched":0,"fp":0,"classes":{}})
                    for name in ("roi","score","dynamic","track_current","track_ever",
                                 "score_near","score_far","score_strict","score_rescue",
                                 "track_new","track_confirmed","track_coast"))

    @staticmethod
    def _empty_selected_admission_score_totals():
        return {"frames":0,"candidates":0,"matched":0,"fp":0,
                "truth_scores":[],"fp_scores":[],"thresholds":{}}

    @staticmethod
    def _empty_selected_track_admission_totals():
        return {
            "frames": 0,
            "decisions": dict(
                (name, {"candidates": 0, "matched": 0, "fp": 0, "classes": {}})
                for name in ("hold", "confirm", "expired", "existing_track", "sensor")),
            "pending_origins": {},
            "actor_classes": dict((name, {}) for name in ("hold", "confirm", "expired")),
            "outcome_samples": dict((name, {}) for name in ("confirm", "expired")),
            "transitions": dict(
                (name, {"total": 0, "origin_truth": 0, "origin_fp": 0,
                        "origin_unknown": 0, "current_truth": 0,
                        "current_fp": 0, "same_truth_actor": 0,
                        "stable_fp": 0, "changed_label_or_actor": 0})
                for name in ("confirm", "expired")),
        }

    @staticmethod
    def _empty_camera_enforcement_totals():
        return {"frames":0,"track_samples":0,"matched":0,"fp":0,
                "camera_only":0,"lidar_takeover":0,"duplicate_like_fp":0,
                "spatial_fp":0,"states":{},"track_ids":{},"actor_ids":{},
                "actor_tracks":{},"track_actors":{},"per_track_samples":{},
                "classes":{},"association_distances":[],"births":0,
                "birth_reasons":{},"birth_nearest_distances":[],
                "birth_nearest_camera_distances":[],"birth_candidate_counts":[],
                "birth_camera_candidate_counts":[]}

    @staticmethod
    def _empty_camera_enforcement_current():
        return {"tracks":0,"matched":0,"fp":0,"camera_only":0,
                "lidar_takeover":0,"duplicate_like_fp":0,"spatial_fp":0}

    def _empty_camera_identity_gates(self):
        return dict(("%g"%gate,{"track_samples":0,"matched":0,"fp":0,
                                "duplicate_like_fp":0,"spatial_fp":0,
                                "errors":[],"actor_tracks":{},
                                "track_actors":{},"actor_ids":{}})
                    for gate in self.camera_ground_identity_match_gates)

    def _empty_camera_reassociation_gates(self):
        return dict(("%g"%gate,{"eligible":0,"available":0,"claimed":0,
                                "consistent":0,"conflict":0,"ambiguous":0,
                                "unknown":0,"safe_recovery":0,"distances":[]})
                    for gate in self.camera_ground_reassociation_shadow_gates)

    def _empty_camera_tombstone_gates(self):
        return dict(("%s_%g"%(mode,gate),{
            "eligible":0,"consistent":0,"conflict":0,"ambiguous":0,
            "unknown":0,"safe_recovery":0,"distances":[],"gaps":[]})
                    for mode in ("frozen","predicted")
                    for gate in self.camera_ground_tombstone_shadow_gates)

    @staticmethod
    def _empty_camera_tombstone_feature():
        return dict((label,{"samples":0,"same_camera":0,"camera_known":0,
                            "heading_cos":[],"tombstone_speed":[],
                            "temporal_motion":[],"distances":[],"gaps":[]})
                    for label in ("same_actor","conflict","ambiguous","unknown"))

    def _empty_camera_tombstone_feature_rules(self):
        return dict((str(rule.get("name","rule_%d"%index)),{
            "base":0,"passed":0,"feature_missing":0,"rejected":0,
            "consistent":0,"conflict":0,"ambiguous":0,"unknown":0})
                    for index,rule in enumerate(
                        self.camera_ground_tombstone_feature_rules)
                    if isinstance(rule,dict))

    def _reset_road_object_run_metrics(self):
        self._road_object_samples = {"classes": {}, "false": []}
        self._adaptive_temporal_samples = {"classes": {}, "false": []}
        self._hybrid_selection_samples = {}
        self._hybrid_rescue_samples = {}
        self._road_object_actor_coverage = {}
        self._road_object_stage_coverage = {}
        self._road_object_cap_totals = self._empty_road_object_cap_totals()
        self._selected_enforcement_totals = self._empty_selected_enforcement_totals()
        self._selected_admission_score_totals = self._empty_selected_admission_score_totals()
        self._selected_track_admission_last_frame = None
        self._selected_track_admission_totals = self._empty_selected_track_admission_totals()
        self._selected_track_admission_frame = {}
        self._selected_delayed_reappearance_last_frame = None
        self._selected_delayed_reappearance_totals = {}
        self._truth_lifecycle_prev = {}
        self._truth_lifecycle_totals = {"entered":0,"boundary_exit":0,
                                        "unexpected_exit":0,"teleport":0}
        self._radar_seed_bridge_last_frame = None
        self._radar_seed_bridge_totals = {}
        self._radar_camera_support_last_frame = None
        self._radar_camera_support_totals = {
            "candidates":0,"visible":0,"supported":0,
            "truth":0,"fp":0,"supported_truth":0,"supported_fp":0,
            "truth_classes":{},"camera_classes":{},"sources":{}}
        self._camera_ground_last_frame = None
        self._camera_ground_totals = {
            "candidates":0,"matched":0,"fp":0,"classes":{},"sources":{}}
        self._camera_ground_vru_roi_last_frame = None
        self._camera_ground_vru_roi_totals = {}
        self._camera_ground_temporal_last_frame = None
        self._camera_ground_temporal_totals = {
            "candidates":0,"matched":0,"fp":0,"classes":{},"sources":{}}
        self._camera_ground_counterfactual_last_frame = None
        self._camera_ground_counterfactual_totals = {
            "frames":0,"truth":0,"base_matched":0,"camera_candidates":0,
            "camera_truth":0,"camera_fp":0,"incremental_matched":0,
            "combined_matched":0,"classes":{},"sources":{}}
        self._camera_ground_enforcement_last_frame = None
        self._camera_ground_enforcement_totals = self._empty_camera_enforcement_totals()
        self._camera_ground_enforcement_totals["identity_gates"] = \
            self._empty_camera_identity_gates()
        self._camera_ground_enforcement_totals["reassociation_gates"] = \
            self._empty_camera_reassociation_gates()
        self._camera_ground_enforcement_totals["tombstone_gates"] = \
            self._empty_camera_tombstone_gates()
        self._camera_ground_enforcement_totals["tombstone_feature"] = \
            self._empty_camera_tombstone_feature()
        self._camera_ground_enforcement_totals["tombstone_feature_rules"] = \
            self._empty_camera_tombstone_feature_rules()
        self._camera_ground_enforcement_current = self._empty_camera_enforcement_current()

    def _sync_road_object_benchmark_session(self, truth):
        """Start a clean cumulative run when a tagged benchmark batch appears."""
        enabled=bool(self.config.get("road_object_benchmark_session_isolation",True))
        tagged=[item for item in truth or []
                if str(item.get("role","")).startswith("rsu_test_")]
        obstacles=[item for item in tagged if item.get("role")=="rsu_test_obstacle"]
        actor_ids=set(int(item.get("actor_id")) for item in obstacles
                      if item.get("actor_id") is not None)
        reset=False
        if enabled and actor_ids:
            new_batch=(not self._road_object_benchmark_active or
                       (self._road_object_benchmark_ids and
                        actor_ids.isdisjoint(self._road_object_benchmark_ids)))
            if new_batch:
                self._reset_road_object_run_metrics()
                self._road_object_benchmark_ids=set(actor_ids)
                self._road_object_benchmark_generation+=1;reset=True
            self._road_object_benchmark_active=True
            self._road_object_benchmark_ids.update(actor_ids)
        elif enabled and not tagged:
            self._road_object_benchmark_active=False
            self._road_object_benchmark_ids=set()
        return {"enabled":enabled,"active":bool(actor_ids),"reset":reset,
                "generation":self._road_object_benchmark_generation,
                "actors":len(actor_ids),"pending":bool(tagged and not actor_ids)}

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

    def analyze_truth_lifecycle(self, truth=None):
        """Describe tagged CARLA truth churn without feeding perception."""
        if not self.config.get("truth_lifecycle_diagnostics",False):return {"enabled":False}
        tagged=[dict(item) for item in (self.truth_objects() if truth is None else truth)
                if str(item.get("role","" )).startswith("rsu_test_") and
                item.get("actor_id") is not None]
        current=dict((int(item["actor_id"]),item) for item in tagged)
        previous=self._truth_lifecycle_prev;entered=[];boundary=[];unexpected=[];teleports=[]
        margin=max(0.0,float(self.config.get("truth_lifecycle_boundary_margin",10.0)))
        boundary_range=max(0.0,self.radius-margin)
        jump_gate=max(0.0,float(self.config.get("truth_lifecycle_teleport_distance",8.0)))
        for actor_id,item in current.items():
            old=previous.get(actor_id)
            if old is None:entered.append(item);continue
            jump=math.hypot(float(item.get("x",0.0))-float(old.get("x",0.0)),
                            float(item.get("y",0.0))-float(old.get("y",0.0)))
            if jump>=jump_gate:
                event=dict(item);event["jump_distance"]=jump;event["previous_range"]=old.get("range")
                teleports.append(event)
        for actor_id,item in previous.items():
            if actor_id in current:continue
            if float(item.get("range",0.0))>=boundary_range:boundary.append(item)
            else:unexpected.append(item)
        counts={"entered":len(entered),"boundary_exit":len(boundary),
                "unexpected_exit":len(unexpected),"teleport":len(teleports)}
        for name,value in counts.items():self._truth_lifecycle_totals[name]+=value
        self._truth_lifecycle_prev=current
        return {"enabled":True,"active":len(current),"entered":entered,
                "boundary_exit":boundary,"unexpected_exit":unexpected,
                "teleport":teleports,"counts":counts,
                "totals":dict(self._truth_lifecycle_totals),
                "boundary_range":boundary_range,"teleport_distance":jump_gate}

    def truth_vehicles(self):
        """Compatibility view for vehicle-specific legacy diagnostics."""
        return [x for x in self.truth_objects() if x.get("object_group") == "vehicle"]

    def test_targets(self):
        """Return explicitly tagged benchmark actors for a self-contained run log."""
        return [item for item in self.truth_objects()
                if str(item.get("role","")).startswith("rsu_test_")]

    def _detected_with_range(self, detected):
        center = self._center();out = []
        if center is None:return out
        for det in detected or []:
            item = dict(det);item["range"] = math.hypot(float(item.get("x",0.0))-float(center.x),float(item.get("y",0.0))-float(center.y))
            if item["range"] <= self.radius:out.append(item)
        return out

    def _match_gate(self, truth, detected, gate):
        candidates=[]
        for ti,gt in enumerate(truth):
            for di,det in enumerate(detected or []):
                d=math.hypot(float(det.get("x",0.0))-gt["x"],float(det.get("y",0.0))-gt["y"])
                if d<=float(gate):candidates.append((d,ti,di))
        candidates.sort(key=lambda x:x[0]);used_t=set();used_d=set();pairs=[]
        for d,ti,di in candidates:
            if ti in used_t or di in used_d:continue
            used_t.add(ti);used_d.add(di);pairs.append((ti,di,d))
        return pairs

    def _match(self, truth, detected):
        return self._match_gate(truth,detected,self.match_distance)

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

    def observe_radar_seed_bridge(self, rule_candidates, frame_id=None):
        """Attribute Shadow bridge output without feeding truth to fusion."""
        if frame_id is not None and frame_id == self._radar_seed_bridge_last_frame:
            return self.report_radar_seed_bridge()
        self._radar_seed_bridge_last_frame = frame_id
        if not rule_candidates or not any(rule_candidates.values()):
            return self.report_radar_seed_bridge()
        truth = self.truth_objects()
        for rule, candidates in (rule_candidates or {}).items():
            detected = self._detected_with_range(candidates)
            pairs = self._match(truth, detected)
            total = self._radar_seed_bridge_totals.setdefault(
                str(rule), {"candidates": 0, "matched": 0, "fp": 0})
            total["candidates"] += len(detected)
            total["matched"] += len(pairs)
            total["fp"] += max(0, len(detected) - len(pairs))
        return self.report_radar_seed_bridge()

    def report_radar_seed_bridge(self):
        result = {}
        for rule, total in self._radar_seed_bridge_totals.items():
            item = dict(total)
            item["precision"] = (float(item["matched"]) / item["candidates"]
                                 if item["candidates"] else None)
            result[rule] = item
        return result

    def observe_radar_camera_support(self, candidates, frame_id=None):
        """Truth-attribute camera support without exposing truth to runtime."""
        if frame_id is not None and frame_id == self._radar_camera_support_last_frame:
            return self.report_radar_camera_support()
        self._radar_camera_support_last_frame = frame_id
        detected=self._detected_with_range(candidates or [])
        if not detected:
            return self.report_radar_camera_support()
        truth=self.truth_objects();pairs=self._match(truth,detected)
        matched_detected=set(pair[1] for pair in pairs)
        totals=self._radar_camera_support_totals
        totals["candidates"]+=len(detected)
        totals["visible"]+=sum(1 for item in detected
                               if item.get("radar_camera_visible",False))
        totals["supported"]+=sum(1 for item in detected
                                 if item.get("radar_camera_supported",False))
        totals["truth"]+=len(pairs);totals["fp"]+=len(detected)-len(pairs)
        for truth_index,detected_index,unused_distance in pairs:
            name=str(truth[truth_index].get("object_type","unknown_obstacle"))
            totals["truth_classes"][name]=totals["truth_classes"].get(name,0)+1
        for index,item in enumerate(detected):
            if not item.get("radar_camera_supported",False):
                continue
            if index in matched_detected:totals["supported_truth"]+=1
            else:totals["supported_fp"]+=1
            name=str(item.get("radar_camera_class","unknown"))
            totals["camera_classes"][name]=totals["camera_classes"].get(name,0)+1
            source=str(item.get("radar_camera_source","none"))
            totals["sources"][source]=totals["sources"].get(source,0)+1
        return self.report_radar_camera_support()

    def report_radar_camera_support(self):
        item=dict(self._radar_camera_support_totals)
        for key in ("truth_classes","camera_classes","sources"):
            item[key]=dict(self._radar_camera_support_totals[key])
        item["candidate_precision"]=(float(item["truth"])/item["candidates"]
                                     if item["candidates"] else None)
        item["support_rate"]=(float(item["supported"])/item["candidates"]
                              if item["candidates"] else None)
        item["supported_precision"]=(
            float(item["supported_truth"])/item["supported"]
            if item["supported"] else None)
        return item

    def observe_camera_ground_initiation(self, candidates, frame_id=None):
        if frame_id is not None and frame_id==self._camera_ground_last_frame:
            return self.report_camera_ground_initiation()
        self._camera_ground_last_frame=frame_id
        detected=self._detected_with_range(candidates or [])
        if not detected:return self.report_camera_ground_initiation()
        truth=self.truth_objects();pairs=self._match(truth,detected)
        totals=self._camera_ground_totals
        totals["candidates"]+=len(detected);totals["matched"]+=len(pairs)
        totals["fp"]+=len(detected)-len(pairs)
        for truth_index,unused_detected,unused_distance in pairs:
            name=str(truth[truth_index].get("object_type","unknown_obstacle"))
            totals["classes"][name]=totals["classes"].get(name,0)+1
        for item in detected:
            source=str(item.get("camera_source","none"))
            totals["sources"][source]=totals["sources"].get(source,0)+1
        return self.report_camera_ground_initiation()

    def report_camera_ground_initiation(self):
        item=dict(self._camera_ground_totals)
        item["classes"]=dict(self._camera_ground_totals["classes"])
        item["sources"]=dict(self._camera_ground_totals["sources"])
        item["precision"]=(float(item["matched"])/item["candidates"]
                           if item["candidates"] else None)
        return item

    def observe_camera_ground_vru_roi_ablation(self, variants, frame_id=None):
        """Truth-attribute wider VRU road margins without changing runtime."""
        if frame_id is not None and frame_id==self._camera_ground_vru_roi_last_frame:
            return self.report_camera_ground_vru_roi_ablation()
        self._camera_ground_vru_roi_last_frame=frame_id
        truth=self.truth_objects()
        for margin,candidates in (variants or {}).items():
            key="%g"%float(margin);detected=self._detected_with_range(candidates or [])
            totals=self._camera_ground_vru_roi_totals.setdefault(
                key,{"candidates":0,"matched":0,"fp":0,"classes":{}})
            pairs=self._match(truth,detected)
            totals["candidates"]+=len(detected);totals["matched"]+=len(pairs)
            totals["fp"]+=len(detected)-len(pairs)
            for truth_index,unused_detected,unused_distance in pairs:
                name=str(truth[truth_index].get("object_type","unknown_obstacle"))
                totals["classes"][name]=totals["classes"].get(name,0)+1
        return self.report_camera_ground_vru_roi_ablation()

    def report_camera_ground_vru_roi_ablation(self):
        result={}
        for key,value in self._camera_ground_vru_roi_totals.items():
            item=dict(value);item["classes"]=dict(value["classes"])
            item["precision"]=(float(item["matched"])/item["candidates"]
                               if item["candidates"] else None)
            result[key]=item
        return result

    def observe_camera_ground_temporal(self, candidates, frame_id=None):
        if frame_id is not None and frame_id==self._camera_ground_temporal_last_frame:
            return self.report_camera_ground_temporal()
        self._camera_ground_temporal_last_frame=frame_id
        detected=self._detected_with_range(candidates or [])
        truth=self.truth_objects();pairs=self._match(truth,detected)
        totals=self._camera_ground_temporal_totals
        totals["candidates"]+=len(detected);totals["matched"]+=len(pairs)
        totals["fp"]+=len(detected)-len(pairs)
        for truth_index,unused_detected,unused_distance in pairs:
            name=str(truth[truth_index].get("object_type","unknown_obstacle"))
            totals["classes"][name]=totals["classes"].get(name,0)+1
        for item in detected:
            source=str(item.get("camera_source","none"))
            totals["sources"][source]=totals["sources"].get(source,0)+1
        return self.report_camera_ground_temporal()

    def report_camera_ground_temporal(self):
        item=dict(self._camera_ground_temporal_totals)
        item["classes"]=dict(self._camera_ground_temporal_totals["classes"])
        item["sources"]=dict(self._camera_ground_temporal_totals["sources"])
        item["precision"]=(float(item["matched"])/item["candidates"]
                           if item["candidates"] else None)
        return item

    def observe_camera_ground_counterfactual(self, tracks, camera_candidates,
                                             frame_id=None):
        """Measure recall gain if confirmed camera candidates joined Tracker."""
        if frame_id is not None and frame_id==self._camera_ground_counterfactual_last_frame:
            return self.report_camera_ground_counterfactual()
        self._camera_ground_counterfactual_last_frame=frame_id
        truth=self.truth_objects();base=self._detected_with_range(tracks or [])
        camera=self._detected_with_range(camera_candidates or [])
        base_pairs=self._match(truth,base);base_truth=set(x[0] for x in base_pairs)
        remaining=[item for index,item in enumerate(truth) if index not in base_truth]
        incremental_pairs=self._match(remaining,camera)
        camera_pairs=self._match(truth,camera)
        totals=self._camera_ground_counterfactual_totals;totals["frames"]+=1
        totals["truth"]+=len(truth);totals["base_matched"]+=len(base_pairs)
        totals["camera_candidates"]+=len(camera)
        totals["camera_truth"]+=len(camera_pairs)
        totals["camera_fp"]+=len(camera)-len(camera_pairs)
        totals["incremental_matched"]+=len(incremental_pairs)
        totals["combined_matched"]+=len(base_pairs)+len(incremental_pairs)
        for truth_index,unused_detected,unused_distance in incremental_pairs:
            name=str(remaining[truth_index].get("object_type","unknown_obstacle"))
            totals["classes"][name]=totals["classes"].get(name,0)+1
        for item in camera:
            source=str(item.get("camera_source","none"))
            totals["sources"][source]=totals["sources"].get(source,0)+1
        return self.report_camera_ground_counterfactual()

    def report_camera_ground_counterfactual(self):
        item=dict(self._camera_ground_counterfactual_totals)
        item["classes"]=dict(self._camera_ground_counterfactual_totals["classes"])
        item["sources"]=dict(self._camera_ground_counterfactual_totals["sources"])
        truth=item["truth"];candidates=item["camera_candidates"]
        item["base_recall"]=(float(item["base_matched"])/truth if truth else None)
        item["combined_recall"]=(float(item["combined_matched"])/truth if truth else None)
        item["recall_gain"]=(float(item["incremental_matched"])/truth if truth else None)
        item["camera_precision"]=(float(item["camera_truth"])/candidates
                                  if candidates else None)
        return item

    def camera_ground_deployment_verdict(self):
        item=self.report_camera_ground_counterfactual();cfg=self.config
        sources=item.get("sources",{});source=(max(sources,key=sources.get)
                                               if sources else "none")
        checks={
            "detector_source":source=="detector",
            "samples":item["camera_candidates"]>=int(cfg.get(
                "camera_ground_deployment_min_candidates",100)),
            "precision":(item["camera_precision"] is not None and
                         item["camera_precision"]>=float(cfg.get(
                             "camera_ground_deployment_min_precision",.95))),
            "recall_gain":(item["recall_gain"] is not None and
                           item["recall_gain"]>=float(cfg.get(
                               "camera_ground_deployment_min_recall_gain",.05)))}
        if source=="carla_truth":status="BLOCKED_CARLA_TRUTH"
        elif not checks["detector_source"]:status="BLOCKED_NO_DETECTOR_EVIDENCE"
        elif all(checks.values()):status="READY"
        else:status="MORE_EVIDENCE"
        return {"status":status,"source":source,"checks":checks}

    def observe_camera_ground_enforcement(self, tracks, frame_id=None):
        """Truth-attribute actual camera-origin Tracker output after admission."""
        if frame_id is not None and frame_id==self._camera_ground_enforcement_last_frame:
            return self.report_camera_ground_enforcement()
        self._camera_ground_enforcement_last_frame=frame_id
        detected=self._detected_with_range([
            item for item in (tracks or [])
            if item.get("track_camera_ground_origin",False)])
        truth=self.truth_objects();pairs=self._match(truth,detected)
        totals=self._camera_ground_enforcement_totals;totals["frames"]+=1
        totals["track_samples"]+=len(detected);totals["matched"]+=len(pairs)
        totals["fp"]+=len(detected)-len(pairs)
        for item in detected:
            state=str(item.get("track_state","unknown"))
            totals["states"][state]=totals["states"].get(state,0)+1
            track_id=str(item.get("id","unknown"));totals["track_ids"][track_id]=1
            totals["per_track_samples"][track_id]=totals["per_track_samples"].get(track_id,0)+1
            if int(item.get("track_lidar_hits",0))>0:totals["lidar_takeover"]+=1
            else:totals["camera_only"]+=1
            association_distance=item.get("track_association_distance")
            if association_distance is not None:
                totals["association_distances"].append(float(association_distance))
            if state=="new":
                totals["births"]+=1
                reason=str(item.get("track_association_birth_reason","unknown"))
                totals["birth_reasons"][reason]=totals["birth_reasons"].get(reason,0)+1
                nearest=item.get("track_association_nearest_distance")
                if nearest is not None:
                    totals["birth_nearest_distances"].append(float(nearest))
                nearest_camera=item.get(
                    "track_association_nearest_camera_origin_distance")
                if nearest_camera is not None:
                    totals["birth_nearest_camera_distances"].append(
                        float(nearest_camera))
                totals["birth_candidate_counts"].append(int(
                    item.get("track_association_candidate_count",0)))
                totals["birth_camera_candidate_counts"].append(int(item.get(
                    "track_association_camera_origin_candidate_count",0)))
        matched_detected=set()
        for truth_index,detected_index,unused_distance in pairs:
            gt=truth[truth_index];name=str(gt.get("object_type","unknown_obstacle"))
            totals["classes"][name]=totals["classes"].get(name,0)+1
            matched_detected.add(detected_index)
            actor_id=gt.get("actor_id")
            if actor_id is not None:
                actor_key=int(actor_id);totals["actor_ids"][actor_key]=name
                track_key=str(detected[detected_index].get("id","unknown"))
                detection=detected[detected_index]
                nearest_id=detection.get(
                    "track_association_nearest_camera_origin_id")
                nearest_distance=detection.get(
                    "track_association_nearest_camera_origin_distance")
                if (str(detection.get("track_state",""))=="new" and
                        str(detection.get("track_association_birth_reason",""))==
                        "outside_gate" and nearest_id is not None and
                        nearest_distance is not None):
                    previous_actors=totals["track_actors"].get(
                        str(nearest_id),{})
                    claimed=bool(detection.get(
                        "track_association_nearest_camera_origin_claimed",False))
                    for gate in self.camera_ground_reassociation_shadow_gates:
                        if float(nearest_distance)>gate:continue
                        bucket=totals["reassociation_gates"]["%g"%gate]
                        bucket["eligible"]+=1
                        bucket["distances"].append(float(nearest_distance))
                        bucket["claimed" if claimed else "available"]+=1
                        if not previous_actors:
                            bucket["unknown"]+=1
                        elif actor_key in previous_actors:
                            if len(previous_actors)==1:
                                bucket["consistent"]+=1
                                if not claimed:bucket["safe_recovery"]+=1
                            else:bucket["ambiguous"]+=1
                        else:
                            bucket["conflict"]+=1
                if str(detection.get("track_state",""))=="new":
                    for mode in ("frozen","predicted"):
                        tombstone_id=detection.get(
                            "track_camera_tombstone_%s_id"%mode)
                        tombstone_distance=detection.get(
                            "track_camera_tombstone_%s_distance"%mode)
                        tombstone_gap=detection.get(
                            "track_camera_tombstone_%s_gap"%mode)
                        if tombstone_id is None or tombstone_distance is None:
                            continue
                        previous_actors=totals["track_actors"].get(
                            str(tombstone_id),{})
                        for gate in self.camera_ground_tombstone_shadow_gates:
                            if float(tombstone_distance)>gate:continue
                            bucket=totals["tombstone_gates"][
                                "%s_%g"%(mode,gate)]
                            bucket["eligible"]+=1
                            bucket["distances"].append(float(tombstone_distance))
                            if tombstone_gap is not None:
                                bucket["gaps"].append(float(tombstone_gap))
                            if not previous_actors:
                                bucket["unknown"]+=1
                                label="unknown"
                            elif actor_key in previous_actors:
                                if len(previous_actors)==1:
                                    bucket["consistent"]+=1
                                    bucket["safe_recovery"]+=1
                                    label="same_actor"
                                else:
                                    bucket["ambiguous"]+=1;label="ambiguous"
                            else:
                                bucket["conflict"]+=1;label="conflict"
                            if (mode==self.camera_ground_tombstone_feature_mode and
                                    abs(float(gate)-self.camera_ground_tombstone_feature_gate)<1e-6):
                                feature=totals["tombstone_feature"][label]
                                feature["samples"]+=1
                                feature["distances"].append(float(tombstone_distance))
                                if tombstone_gap is not None:
                                    feature["gaps"].append(float(tombstone_gap))
                                current_camera=str(detection.get("camera_id","unknown"))
                                prior_camera=str(detection.get(
                                    "track_camera_tombstone_%s_camera_id"%mode,
                                    "unknown"))
                                if current_camera!="unknown" and prior_camera!="unknown":
                                    feature["camera_known"]+=1
                                    if current_camera==prior_camera:
                                        feature["same_camera"]+=1
                                tvx=detection.get(
                                    "track_camera_tombstone_%s_vx"%mode)
                                tvy=detection.get(
                                    "track_camera_tombstone_%s_vy"%mode)
                                mdx=detection.get("camera_ground_temporal_motion_dx")
                                mdy=detection.get("camera_ground_temporal_motion_dy")
                                if None not in (tvx,tvy,mdx,mdy):
                                    tombstone_speed=math.hypot(float(tvx),float(tvy))
                                    temporal_motion=math.hypot(float(mdx),float(mdy))
                                    feature["tombstone_speed"].append(tombstone_speed)
                                    feature["temporal_motion"].append(temporal_motion)
                                    if tombstone_speed>1e-3 and temporal_motion>1e-3:
                                        feature["heading_cos"].append(
                                            (float(tvx)*float(mdx)+float(tvy)*float(mdy))/
                                            (tombstone_speed*temporal_motion))
                                heading_cos=None
                                if None not in (tvx,tvy,mdx,mdy):
                                    tombstone_speed=math.hypot(float(tvx),float(tvy))
                                    temporal_motion=math.hypot(float(mdx),float(mdy))
                                    if tombstone_speed>1e-3 and temporal_motion>1e-3:
                                        heading_cos=(float(tvx)*float(mdx)+
                                                     float(tvy)*float(mdy))/(
                                            tombstone_speed*temporal_motion)
                                same_camera=(current_camera!="unknown" and
                                             prior_camera!="unknown" and
                                             current_camera==prior_camera)
                                camera_known=(current_camera!="unknown" and
                                              prior_camera!="unknown")
                                for index,rule in enumerate(
                                        self.camera_ground_tombstone_feature_rules):
                                    if not isinstance(rule,dict):continue
                                    rule_name=str(rule.get("name","rule_%d"%index))
                                    rule_bucket=totals["tombstone_feature_rules"].get(
                                        rule_name)
                                    if rule_bucket is None:continue
                                    rule_bucket["base"]+=1
                                    missing=False;passed=True
                                    if bool(rule.get("require_same_camera",False)):
                                        if not camera_known:missing=True
                                        elif not same_camera:passed=False
                                    if rule.get("min_heading_cos") is not None:
                                        if heading_cos is None:missing=True
                                        elif heading_cos<float(rule["min_heading_cos"]):
                                            passed=False
                                    if missing:
                                        rule_bucket["feature_missing"]+=1;continue
                                    if not passed:
                                        rule_bucket["rejected"]+=1;continue
                                    rule_bucket["passed"]+=1
                                    if label=="same_actor":
                                        rule_bucket["consistent"]+=1
                                    else:rule_bucket[label]+=1
                actor_tracks=totals["actor_tracks"].setdefault(actor_key,{})
                actor_tracks[track_key]=1
                track_actors=totals["track_actors"].setdefault(track_key,{})
                track_actors[actor_key]=1
        duplicate_like_fp=0
        for index,item in enumerate(detected):
            if index in matched_detected:continue
            if any(math.hypot(float(item.get("x",0.0))-float(gt["x"]),
                              float(item.get("y",0.0))-float(gt["y"]))<=self.match_distance
                   for gt in truth):duplicate_like_fp+=1
        spatial_fp=(len(detected)-len(pairs))-duplicate_like_fp
        totals["duplicate_like_fp"]+=duplicate_like_fp
        totals["spatial_fp"]+=spatial_fp
        for gate in self.camera_ground_identity_match_gates:
            key="%g"%gate;bucket=totals["identity_gates"][key]
            gate_pairs=self._match_gate(truth,detected,gate)
            gate_matched=set(x[1] for x in gate_pairs)
            gate_duplicate=0
            for index,item in enumerate(detected):
                if index in gate_matched:continue
                if any(math.hypot(float(item.get("x",0.0))-float(gt["x"]),
                                  float(item.get("y",0.0))-float(gt["y"]))<=gate
                       for gt in truth):gate_duplicate+=1
            gate_fp=len(detected)-len(gate_pairs)
            bucket["track_samples"]+=len(detected);bucket["matched"]+=len(gate_pairs)
            bucket["fp"]+=gate_fp;bucket["duplicate_like_fp"]+=gate_duplicate
            bucket["spatial_fp"]+=gate_fp-gate_duplicate
            bucket["errors"].extend(x[2] for x in gate_pairs)
            for truth_index,detected_index,unused_distance in gate_pairs:
                actor_id=truth[truth_index].get("actor_id")
                if actor_id is None:continue
                actor_key=int(actor_id);track_key=str(
                    detected[detected_index].get("id","unknown"))
                bucket["actor_ids"][actor_key]=1
                bucket["actor_tracks"].setdefault(actor_key,{})[track_key]=1
                bucket["track_actors"].setdefault(track_key,{})[actor_key]=1
        self._camera_ground_enforcement_current={
            "tracks":len(detected),"matched":len(pairs),
            "fp":len(detected)-len(pairs),"camera_only":sum(
                1 for x in detected if int(x.get("track_lidar_hits",0))==0),
            "lidar_takeover":sum(
                1 for x in detected if int(x.get("track_lidar_hits",0))>0),
            "duplicate_like_fp":duplicate_like_fp,"spatial_fp":spatial_fp}
        return self.report_camera_ground_enforcement()

    def report_camera_ground_enforcement(self):
        totals=self._camera_ground_enforcement_totals;item=dict(totals)
        item["states"]=dict(totals["states"]);item["classes"]=dict(totals["classes"])
        item["unique_tracks"]=len(totals["track_ids"])
        item["unique_actors"]=len(totals["actor_ids"])
        item["fragmented_actors"]=sum(
            1 for tracks in totals["actor_tracks"].values() if len(tracks)>1)
        item["id_fragments"]=sum(
            max(0,len(tracks)-1) for tracks in totals["actor_tracks"].values())
        item["identity_switch_tracks"]=sum(
            1 for actors in totals["track_actors"].values() if len(actors)>1)
        samples=list(totals["per_track_samples"].values())
        item["avg_track_frames"]=(float(sum(samples))/len(samples) if samples else 0.0)
        item["max_track_frames"]=(max(samples) if samples else 0)
        item["precision"]=(float(totals["matched"])/totals["track_samples"]
                           if totals["track_samples"] else None)
        identity_gates={}
        for key,bucket in totals["identity_gates"].items():
            errors=bucket["errors"]
            identity_gates[key]={
                "track_samples":bucket["track_samples"],"matched":bucket["matched"],
                "fp":bucket["fp"],"precision":(
                    float(bucket["matched"])/bucket["track_samples"]
                    if bucket["track_samples"] else None),
                "duplicate_like_fp":bucket["duplicate_like_fp"],
                "spatial_fp":bucket["spatial_fp"],
                "unique_actors":len(bucket["actor_ids"]),
                "fragmented_actors":sum(
                    1 for tracks in bucket["actor_tracks"].values() if len(tracks)>1),
                "id_fragments":sum(max(0,len(tracks)-1)
                                   for tracks in bucket["actor_tracks"].values()),
                "identity_switch_tracks":sum(
                    1 for actors in bucket["track_actors"].values() if len(actors)>1),
                "error_avg":(sum(errors)/len(errors) if errors else None),
                "error_max":(max(errors) if errors else None)}
        item["identity_gates"]=identity_gates
        reassociation_gates={}
        for key,bucket in totals["reassociation_gates"].items():
            known=(bucket["consistent"]+bucket["conflict"]+
                   bucket["ambiguous"])
            reassociation_gates[key]={
                "eligible":bucket["eligible"],"available":bucket["available"],
                "claimed":bucket["claimed"],"consistent":bucket["consistent"],
                "conflict":bucket["conflict"],"ambiguous":bucket["ambiguous"],
                "unknown":bucket["unknown"],
                "safe_recovery":bucket["safe_recovery"],
                "identity_precision":(
                    float(bucket["consistent"])/known if known else None),
                "distance":self._distribution(bucket["distances"])}
        item["reassociation_gates"]=reassociation_gates
        tombstone_gates={}
        for key,bucket in totals["tombstone_gates"].items():
            known=(bucket["consistent"]+bucket["conflict"]+
                   bucket["ambiguous"])
            tombstone_gates[key]={
                "eligible":bucket["eligible"],
                "consistent":bucket["consistent"],
                "conflict":bucket["conflict"],
                "ambiguous":bucket["ambiguous"],
                "unknown":bucket["unknown"],
                "safe_recovery":bucket["safe_recovery"],
                "identity_precision":(
                    float(bucket["consistent"])/known if known else None),
                "distance":self._distribution(bucket["distances"]),
                "gap":self._distribution(bucket["gaps"])}
        item["tombstone_gates"]=tombstone_gates
        tombstone_feature={}
        for label,bucket in totals["tombstone_feature"].items():
            tombstone_feature[label]={
                "samples":bucket["samples"],
                "camera_known":bucket["camera_known"],
                "same_camera":bucket["same_camera"],
                "same_camera_rate":(
                    float(bucket["same_camera"])/bucket["camera_known"]
                    if bucket["camera_known"] else None),
                "heading_cos":self._distribution(bucket["heading_cos"]),
                "tombstone_speed":self._distribution(bucket["tombstone_speed"]),
                "temporal_motion":self._distribution(bucket["temporal_motion"]),
                "distance":self._distribution(bucket["distances"]),
                "gap":self._distribution(bucket["gaps"])}
        item["tombstone_feature"]={
            "mode":self.camera_ground_tombstone_feature_mode,
            "gate":self.camera_ground_tombstone_feature_gate,
            "labels":tombstone_feature}
        tombstone_feature_rules={}
        for name,bucket in totals["tombstone_feature_rules"].items():
            known=(bucket["consistent"]+bucket["conflict"]+
                   bucket["ambiguous"])
            tombstone_feature_rules[name]={
                "base":bucket["base"],"passed":bucket["passed"],
                "feature_missing":bucket["feature_missing"],
                "rejected":bucket["rejected"],
                "consistent":bucket["consistent"],
                "conflict":bucket["conflict"],
                "ambiguous":bucket["ambiguous"],
                "unknown":bucket["unknown"],
                "safe_recovery":bucket["consistent"],
                "identity_precision":(
                    float(bucket["consistent"])/known if known else None)}
        item["tombstone_feature_rules"]={
            "mode":self.camera_ground_tombstone_feature_mode,
            "gate":self.camera_ground_tombstone_feature_gate,
            "rules":tombstone_feature_rules}
        item["association"]={
            "updates":len(totals["association_distances"]),
            "match_distance":self._distribution(totals["association_distances"]),
            "births":totals["births"],
            "birth_reasons":dict(totals["birth_reasons"]),
            "birth_nearest":self._distribution(totals["birth_nearest_distances"]),
            "birth_nearest_camera":self._distribution(
                totals["birth_nearest_camera_distances"]),
            "birth_candidate_count":self._distribution(
                totals["birth_candidate_counts"]),
            "birth_camera_candidate_count":self._distribution(
                totals["birth_camera_candidate_counts"])}
        item["current"]=dict(self._camera_ground_enforcement_current)
        for key in ("track_ids","actor_ids","actor_tracks","track_actors",
                    "per_track_samples","association_distances","birth_reasons",
                    "birth_nearest_distances","birth_nearest_camera_distances",
                    "birth_candidate_counts","birth_camera_candidate_counts"):
            item.pop(key,None)
        return item

    def analyze_selected_enforcement_attribution(self, roi, scored, dynamic, tracks):
        """Measure selected-output contribution without feeding truth to runtime.

        Candidate stages use the current-frame provenance flag. Track reporting
        separates a current selected measurement from any historical selected
        contribution, including coasting tracks.
        """
        sources={
            "roi":[x for x in (roi or []) if x.get("road_object_selected_enforced",False)],
            "score":[x for x in (scored or []) if x.get("road_object_selected_enforced",False)],
            "dynamic":[x for x in (dynamic or []) if x.get("road_object_selected_enforced",False)],
            "track_current":[x for x in (tracks or []) if x.get("track_selected_enforced_current",False)],
            "track_ever":[x for x in (tracks or []) if x.get("track_selected_enforced_ever",False)],
            "score_near":[x for x in (scored or [])
                          if x.get("road_object_selected_enforced",False) and
                          x.get("adaptive_hybrid_source")=="near_baseline"],
            "score_far":[x for x in (scored or [])
                         if x.get("road_object_selected_enforced",False) and
                         x.get("adaptive_hybrid_source")=="far_ranked"],
            "score_strict":[x for x in (scored or [])
                            if x.get("road_object_selected_enforced",False) and
                            not x.get("adaptive_hybrid_temporal_rescue",False)],
            "score_rescue":[x for x in (scored or [])
                            if x.get("road_object_selected_enforced",False) and
                            x.get("adaptive_hybrid_temporal_rescue",False)],
            "track_new":[x for x in (tracks or [])
                         if x.get("track_selected_enforced_current",False) and
                         x.get("track_state")=="new"],
            "track_confirmed":[x for x in (tracks or [])
                               if x.get("track_selected_enforced_current",False) and
                               x.get("track_state")=="confirmed"],
            "track_coast":[x for x in (tracks or [])
                           if x.get("track_selected_enforced_ever",False) and
                           x.get("track_state")=="coast"],
        }
        truth=self.truth_objects();frame={}
        for name,items in sources.items():
            detected=self._detected_with_range(items);pairs=self._match(truth,detected)
            classes={}
            for ti,unused_di,unused_distance in pairs:
                object_type=str(truth[ti].get("object_type","unknown_obstacle"))
                classes[object_type]=int(classes.get(object_type,0))+1
            matched=len(pairs);count=len(detected);fp=max(0,count-matched)
            result={"candidates":count,"matched":matched,"fp":fp,
                    "precision":(float(matched)/count if count else None),
                    "classes":classes}
            frame[name]=result;total=self._selected_enforcement_totals[name]
            total["candidates"]+=count;total["matched"]+=matched;total["fp"]+=fp
            for object_type,value in classes.items():
                total["classes"][object_type]=int(total["classes"].get(object_type,0))+value
        run={}
        for name,total in self._selected_enforcement_totals.items():
            item={"candidates":total["candidates"],"matched":total["matched"],
                  "fp":total["fp"],"classes":dict(total["classes"])}
            item["precision"]=(float(item["matched"])/item["candidates"]
                               if item["candidates"] else None)
            run[name]=item
        return {"frame":frame,"run":run}

    def _selected_admission_thresholds(self):
        values=self.config.get("selected_admission_score_thresholds",
                               [.20,.25,.30,.35,.40,.45]);out=[]
        for value in values or []:
            try:value=float(value)
            except (TypeError,ValueError):continue
            if 0.0<=value<=1.0 and value not in out:out.append(value)
        return sorted(out) or [.20,.25,.30,.35,.40,.45]

    def _selected_admission_metrics(self, truth, items):
        detected=self._detected_with_range(items);pairs=self._match(truth,detected)
        classes={}
        for ti,unused_di,unused_distance in pairs:
            name=str(truth[ti].get("object_type","unknown_obstacle"))
            classes[name]=int(classes.get(name,0))+1
        count=len(detected);matched=len(pairs)
        return {"candidates":count,"matched":matched,"fp":max(0,count-matched),
                "precision":(float(matched)/count if count else None),"classes":classes},detected,pairs

    def analyze_selected_admission_score_profile(self, scored_candidates):
        """Profile a sensor-only score gate without changing runtime output."""
        if not self.config.get("selected_admission_score_profiling",False):
            return {"enabled":False}
        selected=[x for x in (scored_candidates or [])
                  if x.get("road_object_selected_enforced",False) and
                  x.get("selected_admission_shadow_score") is not None]
        truth=self.truth_objects();base,detected,pairs=self._selected_admission_metrics(truth,selected)
        matched_indices=set(di for unused_ti,di,unused_distance in pairs)
        truth_scores=[float(item["selected_admission_shadow_score"])
                      for index,item in enumerate(detected) if index in matched_indices]
        fp_scores=[float(item["selected_admission_shadow_score"])
                   for index,item in enumerate(detected) if index not in matched_indices]
        thresholds={};totals=self._selected_admission_score_totals;totals["frames"]+=1
        totals["candidates"]+=base["candidates"];totals["matched"]+=base["matched"]
        totals["fp"]+=base["fp"];totals["truth_scores"].extend(truth_scores)
        totals["fp_scores"].extend(fp_scores)
        for value in self._selected_admission_thresholds():
            key="%.2f"%value;kept=[item for item in detected
                                   if float(item.get("selected_admission_shadow_score",0.0))>=value]
            metrics,unused_detected,unused_pairs=self._selected_admission_metrics(truth,kept)
            metrics["truth_retention"]=(float(metrics["matched"])/base["matched"]
                                        if base["matched"] else None)
            thresholds[key]=metrics
            total=totals["thresholds"].setdefault(key,{"candidates":0,"matched":0,"fp":0})
            for name in ("candidates","matched","fp"):total[name]+=metrics[name]
        run={"frames":totals["frames"],"candidates":totals["candidates"],
             "matched":totals["matched"],"fp":totals["fp"],
             "precision":(float(totals["matched"])/totals["candidates"]
                          if totals["candidates"] else None),
             "truth_score":self._distribution(totals["truth_scores"]),
             "fp_score":self._distribution(totals["fp_scores"]),"thresholds":{}}
        for key,total in sorted(totals["thresholds"].items()):
            item=dict(total);item["precision"]=(float(item["matched"])/item["candidates"]
                                                if item["candidates"] else None)
            item["truth_retention"]=(float(item["matched"])/totals["matched"]
                                      if totals["matched"] else None)
            run["thresholds"][key]=item
        frame=dict(base);frame["truth_score"]=self._distribution(truth_scores)
        frame["fp_score"]=self._distribution(fp_scores);frame["thresholds"]=thresholds
        return {"enabled":True,"frame":frame,"run":run}

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

    def observe_selected_track_admission(self, held_candidates, admitted_candidates,
                                          expired_candidates, frame_id=None):
        """Label the Selected new-track Shadow decisions inside CARLA evaluation."""
        if not self.config.get("selected_track_admission_profiling", False):
            return False
        if frame_id is not None and frame_id == self._selected_track_admission_last_frame:
            return False
        if frame_id is None and not (held_candidates or admitted_candidates or expired_candidates):
            return False
        self._selected_track_admission_last_frame = frame_id
        admitted = list(admitted_candidates or [])
        groups = {
            "hold": list(held_candidates or []),
            "confirm": [x for x in admitted
                        if x.get("selected_track_admission_reason") == "repeat"],
            "expired": list(expired_candidates or []),
            "existing_track": [x for x in admitted
                               if x.get("selected_track_admission_reason") == "existing_track"],
            "sensor": [x for x in admitted
                       if x.get("selected_track_admission_reason") == "sensor"],
        }
        truth = self.truth_objects()
        totals = self._selected_track_admission_totals
        totals["frames"] += 1
        frame = {}
        for name, candidates in groups.items():
            detected, pairs = self._admission_classification(truth, candidates)
            truth_by_detected = dict((detected_index, truth_index)
                                     for truth_index, detected_index, unused_distance in pairs)
            classes = {}
            for truth_index, unused_detected, unused_distance in pairs:
                object_type = str(truth[truth_index].get(
                    "object_type", "unknown_obstacle"))
                classes[object_type] = int(classes.get(object_type, 0)) + 1
            matched = len(pairs);count = len(detected);fp = max(0, count - matched)
            frame[name] = {"candidates": count, "matched": matched, "fp": fp,
                           "precision": (float(matched) / count if count else None),
                           "classes": classes}
            bucket = totals["decisions"][name]
            bucket["candidates"] += count;bucket["matched"] += matched;bucket["fp"] += fp
            for object_type, value in classes.items():
                bucket["classes"][object_type] = int(
                    bucket["classes"].get(object_type, 0)) + value
            for detected_index, item in enumerate(detected):
                pending_id = item.get("selected_track_admission_pending_id")
                truth_index = truth_by_detected.get(detected_index)
                actor_id = (truth[truth_index].get("actor_id")
                            if truth_index is not None else None)
                object_type = (str(truth[truth_index].get(
                    "object_type", "unknown_obstacle"))
                    if truth_index is not None else None)
                if truth_index is not None and actor_id is not None and name in totals["actor_classes"]:
                    totals["actor_classes"][name][int(actor_id)] = object_type
                if pending_id is None:
                    continue
                key = str(pending_id)
                if name == "hold":
                    if key not in totals["pending_origins"]:
                        totals["pending_origins"][key] = {
                            "label": "truth" if truth_index is not None else "fp",
                            "actor_id": actor_id, "object_type": object_type,
                            "candidate": dict(item)}
                    continue
                if name not in ("confirm", "expired"):
                    continue
                transition = totals["transitions"][name]
                transition["total"] += 1
                origin = totals["pending_origins"].pop(key, None)
                origin_label = origin.get("label") if origin is not None else "unknown"
                transition["origin_" + origin_label] += 1
                if origin is not None:
                    sample_name = (str(origin.get("object_type", "unknown_obstacle"))
                                   if origin_label == "truth" else "false_positive")
                    sample = dict(origin.get("candidate", {}))
                    if origin.get("actor_id") is not None:
                        sample["_evaluation_actor_id"] = int(origin["actor_id"])
                    totals["outcome_samples"][name].setdefault(
                        sample_name, []).append(sample)
                current_label = "truth" if truth_index is not None else "fp"
                transition["current_" + current_label] += 1
                same_actor = (origin is not None and origin_label == "truth" and
                              truth_index is not None and origin.get("actor_id") is not None and
                              int(origin["actor_id"]) == int(actor_id))
                if same_actor:
                    transition["same_truth_actor"] += 1
                elif origin is not None and origin_label == "fp" and current_label == "fp":
                    transition["stable_fp"] += 1
                elif origin is not None:
                    transition["changed_label_or_actor"] += 1
        self._selected_track_admission_frame = frame
        return True

    def report_selected_track_admission(self):
        if not self.config.get("selected_track_admission_profiling", False):
            return {"enabled": False}
        run = {}
        for name, source in self._selected_track_admission_totals["decisions"].items():
            item = dict(source)
            count = int(item.get("candidates", 0))
            item["classes"] = dict(source.get("classes", {}))
            item["precision"] = (float(item.get("matched", 0)) / count
                                 if count else None)
            run[name] = item
        coverage = {}
        for name, actors in self._selected_track_admission_totals["actor_classes"].items():
            classes = {}
            for object_type in actors.values():
                classes[object_type] = int(classes.get(object_type, 0)) + 1
            coverage[name] = {"actors": len(actors), "classes": classes}
        actor_classes = self._selected_track_admission_totals["actor_classes"]
        held = set(actor_classes["hold"]);confirmed = set(actor_classes["confirm"])
        expired = set(actor_classes["expired"])
        outcome_sets = {
            "confirm_only": (held & confirmed) - expired,
            "expired_only": (held & expired) - confirmed,
            "both": held & confirmed & expired,
            "unresolved": held - confirmed - expired,
        }
        outcomes = {}
        for name, actor_ids in outcome_sets.items():
            classes = {}
            for actor_id in actor_ids:
                object_type = actor_classes["hold"].get(
                    actor_id, actor_classes["confirm"].get(
                        actor_id, actor_classes["expired"].get(
                            actor_id, "unknown_obstacle")))
                classes[object_type] = int(classes.get(object_type, 0)) + 1
            outcomes[name] = {"actors": len(actor_ids), "classes": classes}
        outcomes["held_actors"] = len(held)
        outcomes["ever_confirmed"] = len(held & confirmed)
        outcomes["confirmation_coverage"] = (
            float(len(held & confirmed)) / len(held) if held else None)
        outcome_features = {}
        for decision, buckets in self._selected_track_admission_totals["outcome_samples"].items():
            outcome_features[decision] = dict(
                (name, self._selected_outcome_feature_profile(items))
                for name, items in sorted(buckets.items()))
        camera_rescue = self._selected_camera_rescue_shadow(outcome_sets)
        camera_deployment = self._selected_camera_deployment_verdict(
            camera_rescue)
        delayed_reappearance = self._selected_delayed_reappearance_report(outcome_sets)
        delayed_deployment = self._selected_delayed_deployment_verdict(
            delayed_reappearance)
        return {"enabled": True,
                "frames": int(self._selected_track_admission_totals["frames"]),
                "frame": dict(self._selected_track_admission_frame), "run": run,
                "coverage": coverage,
                "actor_outcomes": outcomes,
                "outcome_features": outcome_features,
                "camera_rescue_shadow": camera_rescue,
                "camera_rescue_deployment_verdict": camera_deployment,
                "delayed_reappearance_shadow": delayed_reappearance,
                "delayed_reappearance_deployment_verdict": delayed_deployment,
                "transitions": dict((name, dict(value)) for name, value in
                                    self._selected_track_admission_totals["transitions"].items()),
                "pending_origins": len(self._selected_track_admission_totals["pending_origins"])}

    def _selected_camera_deployment_verdict(self, camera_report):
        """Require real-detector evidence before camera rescue can be promoted."""
        if not self.config.get(
                "selected_camera_rescue_deployment_verdict_shadow", False):
            return {"enabled": False}
        rule_name = str(self.config.get(
            "selected_camera_rescue_deployment_rule", ""))
        value = (camera_report or {}).get(rule_name, {}) or {}
        kept_person = int(value.get("expired_person_samples_kept", 0))
        kept_fp = int(value.get("expired_fp_samples_kept", 0))
        expired_fp = int(value.get("expired_fp_samples", 0))
        confirm_fp = int(value.get("confirm_fp_samples", 0))
        rescued = int(value.get("expired_only_person_actors_rescued", 0))
        expired_only = int(value.get("expired_only_person_actors", 0))
        sources = dict(value.get("camera_sources", {}) or {})
        required_source = str(self.config.get(
            "selected_camera_rescue_required_source", "detector"))
        values = {
            "expired_person_samples": int(value.get(
                "expired_person_samples", 0)),
            "visible_person_samples": int(value.get(
                "expired_person_visible_samples", 0)),
            "kept_person_samples": kept_person,
            "person_opportunity_rate": (
                float(value.get("expired_person_visible_samples", 0)) /
                int(value.get("expired_person_samples", 0))
                if int(value.get("expired_person_samples", 0)) else None),
            "kept_precision": (float(kept_person) / (kept_person + kept_fp)
                               if kept_person + kept_fp else None),
            "expired_fp_rejection": (1.0 - float(kept_fp) / expired_fp
                                     if expired_fp else None),
            "expired_only_person_actor_coverage": (
                float(rescued) / expired_only if expired_only else None),
            "confirm_fp_rejection": (1.0 - float(value.get(
                "confirm_fp_samples_kept", 0)) / confirm_fp
                if confirm_fp else None),
            "camera_sources": sources}
        criteria = {
            "min_expired_person_samples": int(self.config.get(
                "selected_camera_rescue_min_expired_person_samples", 20)),
            "min_kept_person_samples": int(self.config.get(
                "selected_camera_rescue_min_kept_person_samples", 5)),
            "min_visible_person_samples": int(self.config.get(
                "selected_camera_rescue_min_visible_person_samples", 5)),
            "min_kept_precision": float(self.config.get(
                "selected_camera_rescue_min_kept_precision", .80)),
            "min_expired_fp_rejection": float(self.config.get(
                "selected_camera_rescue_min_expired_fp_rejection", .98)),
            "min_expired_only_person_actor_coverage": float(self.config.get(
                "selected_camera_rescue_min_person_actor_coverage", .50)),
            "min_confirm_fp_rejection": float(self.config.get(
                "selected_camera_rescue_min_confirm_fp_rejection", .99)),
            "required_source": required_source}
        reasons = []
        if values["expired_person_samples"] < criteria[
                "min_expired_person_samples"]:reasons.append("samples")
        if values["kept_person_samples"] < criteria[
                "min_kept_person_samples"]:reasons.append("kept_person_samples")
        if values["visible_person_samples"] < criteria[
                "min_visible_person_samples"]:reasons.append("visible_person_samples")
        for name in ("kept_precision", "expired_fp_rejection",
                     "expired_only_person_actor_coverage",
                     "confirm_fp_rejection"):
            current = values[name]
            if current is None or float(current) < criteria["min_" + name]:
                reasons.append(name)
        if sources.get(required_source, 0) <= 0:reasons.append("camera_source")
        return {"enabled": True, "status": "READY" if not reasons else "BLOCKED",
                "rule": rule_name, "values": values, "criteria": criteria,
                "reasons": reasons, "evaluation_only": True}

    def _selected_delayed_deployment_verdict(self, delayed_report):
        """Gate any future enforcement on cumulative evaluator evidence."""
        if not self.config.get(
                "selected_delayed_reappearance_deployment_verdict_shadow", False):
            return {"enabled": False}
        rule_name = str(self.config.get(
            "selected_delayed_reappearance_deployment_rule", ""))
        gate_name = str(self.config.get(
            "selected_delayed_reappearance_deployment_risk_gate", ""))
        delayed = (delayed_report or {}).get(rule_name, {}) or {}
        incremental = delayed.get("incremental", {}) or {}
        gate = (incremental.get("risk_gate_ablations", {}) or {}).get(
            gate_name, {}) or {}
        criteria = {
            "min_candidates": int(self.config.get(
                "selected_delayed_deployment_min_candidates", 20)),
            "min_precision": float(self.config.get(
                "selected_delayed_deployment_min_precision", .85)),
            "min_truth_retention": float(self.config.get(
                "selected_delayed_deployment_min_truth_retention", .60)),
            "min_expired_only_actors_rescued": int(self.config.get(
                "selected_delayed_deployment_min_expired_only_actors", 1)),
            "min_expired_only_person_actors_rescued": int(self.config.get(
                "selected_delayed_deployment_min_expired_only_person_actors", 1))}
        values = {
            "candidates": int(gate.get("candidates", 0)),
            "precision": gate.get("precision"),
            "truth_retention": gate.get("truth_retention"),
            "expired_only_actors_rescued": int(gate.get(
                "expired_only_actors_rescued", 0)),
            "expired_only_person_actors_rescued": int(gate.get(
                "expired_only_person_actors_rescued", 0))}
        reasons = []
        if values["candidates"] < criteria["min_candidates"]:
            reasons.append("samples")
        for name in ("precision", "truth_retention"):
            value = values[name]
            if value is None or float(value) < criteria["min_" + name]:
                reasons.append(name)
        for name in ("expired_only_actors_rescued",
                     "expired_only_person_actors_rescued"):
            if values[name] < criteria["min_" + name]:reasons.append(name)
        return {"enabled": True, "status": "READY" if not reasons else "BLOCKED",
                "rule": rule_name, "risk_gate": gate_name,
                "values": values, "criteria": criteria, "reasons": reasons,
                "evaluation_only": True}

    def observe_selected_delayed_reappearance(self, rule_candidates,
                                               base_admitted_candidates=None,
                                               frame_id=None):
        """Truth-attribute parallel LiDAR reappearance rules for evaluation only."""
        if not self.config.get("selected_track_admission_profiling", False):
            return False
        if frame_id is not None and frame_id == self._selected_delayed_reappearance_last_frame:
            return False
        self._selected_delayed_reappearance_last_frame = frame_id
        truth = self.truth_objects()
        base_repeats = [item for item in (base_admitted_candidates or [])
                        if item.get("selected_track_admission_reason") == "repeat"]
        duplicate_gate = float(self.config.get(
            "selected_delayed_reappearance_base_duplicate_gate", .05))
        for name, candidates in (rule_candidates or {}).items():
            detected, pairs = self._admission_classification(truth, candidates)
            total = self._selected_delayed_reappearance_totals.setdefault(
                str(name), {"candidates":0, "matched":0, "fp":0,
                            "actors":{}, "time_gaps":[], "match_distances":[],
                            "incremental":{"candidates":0, "matched":0, "fp":0,
                                           "actors":{}, "truth_samples":[],
                                           "fp_samples":[]}})
            total["candidates"] += len(detected);total["matched"] += len(pairs)
            total["fp"] += max(0, len(detected) - len(pairs))
            for truth_index, detected_index, unused_distance in pairs:
                actor_id = truth[truth_index].get("actor_id")
                if actor_id is not None:
                    total["actors"][int(actor_id)] = str(truth[truth_index].get(
                        "object_type", "unknown_obstacle"))
            for item in detected:
                for values, key in ((total["time_gaps"],
                                     "selected_delayed_reappearance_time_gap"),
                                    (total["match_distances"],
                                     "selected_delayed_reappearance_match_distance")):
                    try:values.append(float(item.get(key)))
                    except (TypeError, ValueError):pass
            incremental = []
            for item in detected:
                duplicated = any(math.hypot(
                    float(item.get("x", 0.0)) - float(base.get("x", 0.0)),
                    float(item.get("y", 0.0)) - float(base.get("y", 0.0)))
                    <= duplicate_gate for base in base_repeats)
                if not duplicated:incremental.append(item)
            inc_detected, inc_pairs = self._admission_classification(truth, incremental)
            inc = total["incremental"];inc["candidates"] += len(inc_detected)
            inc["matched"] += len(inc_pairs)
            inc["fp"] += max(0, len(inc_detected) - len(inc_pairs))
            truth_by_detected = dict((detected_index, truth_index)
                                     for truth_index, detected_index, unused in inc_pairs)
            for index, item in enumerate(inc_detected):
                sample = dict(item);truth_index = truth_by_detected.get(index)
                if truth_index is None:
                    inc["fp_samples"].append(sample);continue
                actor_id = truth[truth_index].get("actor_id")
                object_type = str(truth[truth_index].get(
                    "object_type", "unknown_obstacle"))
                sample["_evaluation_object_type"] = object_type
                if actor_id is not None:
                    sample["_evaluation_actor_id"] = int(actor_id)
                inc["truth_samples"].append(sample)
                if actor_id is not None:inc["actors"][int(actor_id)] = object_type
        return True

    def _selected_delayed_risk_rules(self):
        configured = self.config.get(
            "selected_delayed_reappearance_risk_gate_ablations", []) or []
        return [dict(rule) for rule in configured if isinstance(rule, dict)]

    def _selected_delayed_risk_ablations(self, source, expired_only,
                                          expired_only_person):
        truth_samples = list(source.get("truth_samples", []) or [])
        fp_samples = list(source.get("fp_samples", []) or [])
        result = {}
        for index, configured in enumerate(self._selected_delayed_risk_rules()):
            rule = dict(configured);name = str(rule.pop("name", "rule_%d" % index))
            truth_kept = [sample for sample in truth_samples
                          if selected_delayed_risk_gate_passes(sample, rule)]
            fp_kept = [sample for sample in fp_samples
                       if selected_delayed_risk_gate_passes(sample, rule)]
            actor_classes = {}
            for sample in truth_kept:
                actor_id = sample.get("_evaluation_actor_id")
                if actor_id is not None:
                    actor_classes[int(actor_id)] = str(sample.get(
                        "_evaluation_object_type", "unknown_obstacle"))
            actors = set(actor_classes);rescued = actors & expired_only
            candidates = len(truth_kept) + len(fp_kept)
            result[name] = {
                "rule": rule, "candidates": candidates,
                "matched": len(truth_kept), "fp": len(fp_kept),
                "precision": (float(len(truth_kept)) / candidates
                              if candidates else None),
                "truth_retention": (float(len(truth_kept)) / len(truth_samples)
                                    if truth_samples else None),
                "fp_rejection": (1.0 - float(len(fp_kept)) / len(fp_samples)
                                 if fp_samples else None),
                "actors": len(actors),
                "classes": dict((class_name, sum(
                    1 for value in actor_classes.values() if value == class_name))
                    for class_name in set(actor_classes.values())),
                "expired_only_actors_rescued": len(rescued),
                "expired_only_person_actors_rescued": len(
                    rescued & expired_only_person)}
        return result

    def _selected_delayed_feature_profile(self, items):
        items = list(items or [])
        values = dict((name, []) for name in (
            "score", "points", "height", "range", "time_gap",
            "match_distance", "apparent_speed", "origin_score",
            "origin_points"))
        classes = {}
        for item in items:
            origin = item.get("selected_delayed_reappearance_origin", {}) or {}
            fields = (("score", item.get("selected_admission_shadow_score",
                                          item.get("candidate_score"))),
                      ("points", item.get("current_point_count",
                                           item.get("point_count"))),
                      ("range", item.get("range")),
                      ("time_gap", item.get(
                          "selected_delayed_reappearance_time_gap")),
                      ("match_distance", item.get(
                          "selected_delayed_reappearance_match_distance")),
                      ("origin_score", origin.get("selected_admission_shadow_score",
                                                   origin.get("candidate_score"))),
                      ("origin_points", origin.get("current_point_count",
                                                    origin.get("point_count"))))
            for name, value in fields:
                try:values[name].append(float(value))
                except (TypeError, ValueError):pass
            try:values["height"].append(float((item.get("extent") or [])[2]))
            except (IndexError, TypeError, ValueError):pass
            try:
                gap = float(item.get("selected_delayed_reappearance_time_gap"))
                distance = float(item.get(
                    "selected_delayed_reappearance_match_distance"))
                if gap > 0.0:values["apparent_speed"].append(distance / gap)
            except (TypeError, ValueError):pass
            object_type = item.get("_evaluation_object_type")
            if object_type is not None:
                classes[str(object_type)] = int(classes.get(str(object_type), 0)) + 1
        result = dict((name, self._distribution(bucket))
                      for name, bucket in values.items())
        result["samples"] = len(items);result["classes"] = classes
        return result

    def _selected_delayed_reappearance_report(self, outcome_sets):
        expired_only = set(outcome_sets.get("expired_only", set()))
        actor_classes = self._selected_track_admission_totals["actor_classes"]["hold"]
        expired_only_person = set(
            actor_id for actor_id in expired_only
            if actor_classes.get(actor_id) == "person")
        report = {}
        for name, source in sorted(self._selected_delayed_reappearance_totals.items()):
            actors = set(source["actors"])
            rescued = actors & expired_only
            item = {"candidates":source["candidates"], "matched":source["matched"],
                    "fp":source["fp"], "actors":len(actors),
                    "classes":dict((class_name, sum(
                        1 for value in source["actors"].values()
                        if value == class_name))
                        for class_name in set(source["actors"].values())),
                    "expired_only_actors":len(expired_only),
                    "expired_only_actors_rescued":len(rescued),
                    "expired_only_person_actors":len(expired_only_person),
                    "expired_only_person_actors_rescued":len(
                        rescued & expired_only_person),
                    "time_gap":self._distribution(source["time_gaps"]),
                    "match_distance":self._distribution(source["match_distances"])}
            item["precision"] = (float(item["matched"]) / item["candidates"]
                                 if item["candidates"] else None)
            inc_source = source.get("incremental", {}) or {};inc_actors = set(
                (inc_source.get("actors", {}) or {}).keys())
            inc_rescued = inc_actors & expired_only
            incremental = {
                "candidates":int(inc_source.get("candidates", 0)),
                "matched":int(inc_source.get("matched", 0)),
                "fp":int(inc_source.get("fp", 0)),
                "actors":len(inc_actors),
                "classes":dict((class_name, sum(
                    1 for value in (inc_source.get("actors", {}) or {}).values()
                    if value == class_name)) for class_name in set(
                        (inc_source.get("actors", {}) or {}).values())),
                "expired_only_actors_rescued":len(inc_rescued),
                "expired_only_person_actors_rescued":len(
                    inc_rescued & expired_only_person),
                "truth_features":self._selected_delayed_feature_profile(
                    inc_source.get("truth_samples", [])),
                "fp_features":self._selected_delayed_feature_profile(
                    inc_source.get("fp_samples", []))}
            incremental["precision"] = (
                float(incremental["matched"]) / incremental["candidates"]
                if incremental["candidates"] else None)
            incremental["risk_gate_ablations"] = (
                self._selected_delayed_risk_ablations(
                    inc_source, expired_only, expired_only_person))
            item["incremental"] = incremental
            report[name] = item
        return report

    def _selected_camera_rescue_rules(self):
        configured = self.config.get(
            "selected_track_admission_camera_rescue_ablations", []) or []
        rules = []
        for index, item in enumerate(configured):
            if not isinstance(item, dict):continue
            rules.append({
                "name": str(item.get("name", "rule_%d" % index)),
                "min_iou": float(item.get("min_iou", .05)),
                "max_center_distance": float(item.get("max_center_distance", 45.0)),
                "allowed_classes": list(item.get(
                    "allowed_classes", ["person", "pedestrian"]))})
        return rules or [
            {"name":"iou05_or_d30","min_iou":.05,"max_center_distance":30.0,
             "allowed_classes":["person","pedestrian"]},
            {"name":"iou05_or_d45","min_iou":.05,"max_center_distance":45.0,
             "allowed_classes":["person","pedestrian"]},
            {"name":"iou10_or_d45","min_iou":.10,"max_center_distance":45.0,
             "allowed_classes":["person","pedestrian"]}]

    def _selected_camera_rescue_shadow(self, outcome_sets):
        samples = self._selected_track_admission_totals["outcome_samples"]
        expired = samples.get("expired", {}) or {}
        confirmed = samples.get("confirm", {}) or {}
        expired_only = set(outcome_sets.get("expired_only", set()))
        actor_classes = self._selected_track_admission_totals["actor_classes"]["hold"]
        expired_only_person = set(
            actor_id for actor_id in expired_only
            if actor_classes.get(actor_id) == "person")
        result = {}
        for rule in self._selected_camera_rescue_rules():
            def kept(items):
                return [item for item in (items or [])
                        if selected_camera_rescue_passes(
                            item, rule["min_iou"], rule["max_center_distance"],
                            rule["allowed_classes"])]
            expired_person = list(expired.get("person", []))
            expired_fp = list(expired.get("false_positive", []))
            confirm_person = list(confirmed.get("person", []))
            confirm_fp = list(confirmed.get("false_positive", []))
            kept_expired_person = kept(expired_person)
            kept_expired_fp = kept(expired_fp)
            kept_confirm_person = kept(confirm_person)
            kept_confirm_fp = kept(confirm_fp)
            camera_sources = {}
            for item in (kept_expired_person + kept_expired_fp +
                         kept_confirm_person + kept_confirm_fp):
                source = str(item.get(
                    "selected_track_admission_camera_source", "unknown"))
                camera_sources[source] = int(camera_sources.get(source, 0)) + 1
            rescued_actor_ids = set(
                item.get("_evaluation_actor_id") for item in kept_expired_person
                if item.get("_evaluation_actor_id") in expired_only)
            result[rule["name"]] = {
                "min_iou": rule["min_iou"],
                "max_center_distance": rule["max_center_distance"],
                "allowed_classes": list(rule["allowed_classes"]),
                "expired_only_actors": len(expired_only),
                "expired_only_actors_rescued": len(rescued_actor_ids),
                "expired_only_person_actors": len(expired_only_person),
                "expired_only_person_actors_rescued": len(
                    rescued_actor_ids & expired_only_person),
                "expired_person_samples": len(expired_person),
                "expired_person_visible_samples": sum(
                    1 for item in expired_person if item.get(
                        "selected_track_admission_camera_visible", False)),
                "expired_person_samples_kept": len(kept_expired_person),
                "expired_fp_samples": len(expired_fp),
                "expired_fp_samples_kept": len(kept_expired_fp),
                "confirm_person_samples": len(confirm_person),
                "confirm_person_samples_kept": len(kept_confirm_person),
                "confirm_fp_samples": len(confirm_fp),
                "confirm_fp_samples_kept": len(kept_confirm_fp),
                "camera_sources": camera_sources,
            }
        return result

    def _selected_outcome_feature_profile(self, items):
        items = list(items or [])
        profile = self._geometry_profile(items)
        scores = []
        paths = {"near": 0, "far": 0, "strict": 0, "rescue": 0}
        camera = {"visible": 0, "supported": 0, "sources": {}, "classes": {},
                  "projection_rejections": {}}
        camera_ious = [];camera_distances = [];camera_confidences = []
        nearest_distances=[];nearest_ious=[];nearest_confidences=[];nearest_classes={}
        for item in items:
            value = item.get("selected_admission_shadow_score",
                             item.get("candidate_score"))
            try:scores.append(float(value))
            except (TypeError, ValueError):pass
            source = str(item.get("adaptive_hybrid_source", ""))
            if source == "near_baseline":paths["near"] += 1
            elif source == "far_ranked":paths["far"] += 1
            if item.get("adaptive_hybrid_temporal_rescue", False):
                paths["rescue"] += 1
            else:
                paths["strict"] += 1
            source = str(item.get("selected_track_admission_camera_source", "none"))
            camera["sources"][source] = int(camera["sources"].get(source, 0)) + 1
            if item.get("selected_track_admission_camera_visible", False):
                camera["visible"] += 1
            rejection=item.get("selected_track_admission_camera_projection_rejection")
            if rejection is not None:
                rejection=str(rejection)
                camera["projection_rejections"][rejection]=int(
                    camera["projection_rejections"].get(rejection,0))+1
            nearest_class=item.get("selected_track_admission_camera_nearest_class")
            if nearest_class is not None:
                nearest_class=str(nearest_class)
                nearest_classes[nearest_class]=int(nearest_classes.get(nearest_class,0))+1
            for values,key in (
                    (nearest_distances,"selected_track_admission_camera_nearest_distance"),
                    (nearest_ious,"selected_track_admission_camera_nearest_iou"),
                    (nearest_confidences,"selected_track_admission_camera_nearest_confidence")):
                try:values.append(float(item.get(key)))
                except (TypeError,ValueError):pass
            if item.get("selected_track_admission_camera_supported", False):
                camera["supported"] += 1
                camera_class = str(item.get(
                    "selected_track_admission_camera_class", "unknown"))
                camera["classes"][camera_class] = int(
                    camera["classes"].get(camera_class, 0)) + 1
                for values, key in (
                        (camera_ious, "selected_track_admission_camera_iou"),
                        (camera_distances,
                         "selected_track_admission_camera_center_distance"),
                        (camera_confidences,
                         "selected_track_admission_camera_confidence")):
                    try:values.append(float(item.get(key)))
                    except (TypeError, ValueError):pass
        profile["scores"] = {
            "samples": len(scores), "mean": (sum(scores) / len(scores) if scores else None),
            "min": (min(scores) if scores else None),
            "p10": self._percentile(scores, .10), "p50": self._percentile(scores, .50),
            "p90": self._percentile(scores, .90), "max": (max(scores) if scores else None)}
        profile["paths"] = paths
        camera["support_rate"] = (float(camera["supported"]) / camera["visible"]
                                  if camera["visible"] else None)
        camera["visibility_rate"] = (float(camera["visible"]) / len(items)
                                     if items else None)
        camera["iou"] = self._distribution(camera_ious)
        camera["center_distance"] = self._distribution(camera_distances)
        camera["confidence"] = self._distribution(camera_confidences)
        camera["nearest_distance"] = self._distribution(nearest_distances)
        camera["nearest_iou"] = self._distribution(nearest_ious)
        camera["nearest_confidence"] = self._distribution(nearest_confidences)
        camera["nearest_classes"] = nearest_classes
        camera["rescue_ablations"] = dict(
            (rule["name"], sum(1 for item in items
             if selected_camera_rescue_passes(
                 item, rule["min_iou"], rule["max_center_distance"],
                 rule["allowed_classes"])))
            for rule in self._selected_camera_rescue_rules())
        profile["camera"] = camera
        profile["samples"] = len(items)
        return profile

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

    @classmethod
    def _geometry_profile(cls, items):
        points=[];lengths=[];widths=[];heights=[];ranges=[];long_sides=[];short_sides=[];modes={}
        flags={"compact":0,"sparse":0,"recovery":0,"temporal":0,"far_builder":0,
               "road_object":0}
        for item in items or []:
            value=item.get("current_point_count",item.get("point_count"))
            try:points.append(float(value))
            except (TypeError,ValueError):pass
            try:ranges.append(float(item.get("range")))
            except (TypeError,ValueError):pass
            extent=list(item.get("extent",[]) or [])
            for values,index in ((lengths,0),(widths,1),(heights,2)):
                try:values.append(float(extent[index]))
                except (IndexError,TypeError,ValueError):pass
            try:
                side_x=float(extent[0]);side_y=float(extent[1])
                long_sides.append(max(side_x,side_y));short_sides.append(min(side_x,side_y))
            except (IndexError,TypeError,ValueError):pass
            mode=str(item.get("cluster_mode","unknown"))
            modes[mode]=int(modes.get(mode,0))+1
            if item.get("multiclass_compact_geometry",False):flags["compact"]+=1
            if item.get("sparse_rescued",False):flags["sparse"]+=1
            if item.get("far_geometry_recovered",False):flags["recovery"]+=1
            if item.get("far_geometry_temporal_supported",False):flags["temporal"]+=1
            if mode=="far_geometry_builder":flags["far_builder"]+=1
            if item.get("road_object_recovered",False):flags["road_object"]+=1
        def summary(values):
            return {"samples":len(values),"mean":(sum(values)/len(values) if values else None),
                    "min":(min(values) if values else None),
                    "p10":cls._percentile(values,.10),"p50":cls._percentile(values,.50),
                    "p90":cls._percentile(values,.90),"max":(max(values) if values else None)}
        return {"points":summary(points),"length":summary(lengths),"width":summary(widths),
                "long_side":summary(long_sides),"short_side":summary(short_sides),
                "height":summary(heights),"range":summary(ranges),
                "cluster_modes":modes,"sources":flags}

    def _road_object_gate_thresholds(self):
        values=self.config.get("road_object_gate_point_ablations",[8,9,10]);out=[]
        for value in values or []:
            try:value=int(value)
            except (TypeError,ValueError):continue
            if value>0 and value not in out:out.append(value)
        return sorted(out) or [8,9,10]

    def _road_object_gate_failures(self, item, min_points=None):
        """Evaluation-only scalar gate. No truth labels are read here."""
        try:
            points=float(item.get("current_point_count",item.get("point_count",0)))
            height=float((item.get("extent",[]) or [0.0,0.0,0.0])[2])
            distance=float(item.get("range",0.0))
        except (IndexError,TypeError,ValueError):return ["invalid"]
        threshold=float(self.config.get("road_object_gate_min_points",10) if min_points is None else min_points)
        failures=[]
        if points<threshold:failures.append("points")
        if height>float(self.config.get("road_object_gate_max_height",.45)):failures.append("height")
        if distance>float(self.config.get("road_object_gate_max_range",25.0)):failures.append("range")
        return failures

    def _road_object_gate_pass(self, item, min_points=None):
        return not self._road_object_gate_failures(item,min_points)

    def _road_object_rejection_profile(self, items, min_points=None):
        result={"total":len(items or []),"passed":0,"points":0,"height":0,"range":0,"invalid":0}
        for item in items or []:
            failures=self._road_object_gate_failures(item,min_points)
            if not failures:result["passed"]+=1
            for name in failures:result[name]=result.get(name,0)+1
        return result

    def _road_object_gate_profile(self, class_items, false_items, min_points=None):
        classes={};truth_total=0;truth_kept=0
        for name,items in (class_items or {}).items():
            kept=sum(1 for item in items if self._road_object_gate_pass(item,min_points));total=len(items)
            classes[name]={"total":total,"kept":kept,"rejected":total-kept}
            truth_total+=total;truth_kept+=kept
        fp_total=len(false_items or []);fp_kept=sum(1 for item in false_items or [] if self._road_object_gate_pass(item,min_points))
        truth_items=[]
        for items in (class_items or {}).values():truth_items.extend(items)
        return {"enabled":bool(self.config.get("road_object_precision_gate_shadow",False)),
                "min_points":int(self.config.get("road_object_gate_min_points",10) if min_points is None else min_points),
                "candidates":truth_total+fp_total,"kept":truth_kept+fp_kept,
                "rejected":truth_total+fp_total-truth_kept-fp_kept,
                "truth":{"total":truth_total,"kept":truth_kept,"rejected":truth_total-truth_kept,
                         "failures":self._road_object_rejection_profile(truth_items,min_points)},
                "fp":{"total":fp_total,"kept":fp_kept,"rejected":fp_total-fp_kept,
                      "failures":self._road_object_rejection_profile(false_items,min_points)},
                "classes":classes}

    def _road_object_ablations(self, class_items, false_items):
        return dict((str(value),self._road_object_gate_profile(class_items,false_items,value))
                    for value in self._road_object_gate_thresholds())

    def _record_road_object_actor_coverage(self, truth, pairs, detected):
        by_truth=dict((ti,di) for ti,di,_ in pairs);thresholds=self._road_object_gate_thresholds()
        for ti,item in enumerate(truth):
            if item.get("object_type")!="unknown_obstacle" or item.get("role")!="rsu_test_obstacle":continue
            actor_id=int(item.get("actor_id",0));bucket=self._road_object_actor_coverage.setdefault(actor_id,{
                "actor_id":actor_id,"type_id":item.get("type_id","unknown"),"visible_frames":0,
                "matched_frames":0,"gate_kept_frames":0,"range_min":None,"range_max":None,
                "gate_failures":{"points":0,"height":0,"range":0,"invalid":0},
                "ablation_kept":dict((str(value),0) for value in thresholds)})
            distance=float(item.get("range",0.0));bucket["visible_frames"]+=1
            bucket["range_min"]=distance if bucket["range_min"] is None else min(bucket["range_min"],distance)
            bucket["range_max"]=distance if bucket["range_max"] is None else max(bucket["range_max"],distance)
            if ti not in by_truth:continue
            candidate=detected[by_truth[ti]];bucket["matched_frames"]+=1
            failures=self._road_object_gate_failures(candidate)
            if not failures:bucket["gate_kept_frames"]+=1
            for name in failures:bucket["gate_failures"][name]=bucket["gate_failures"].get(name,0)+1
            for value in thresholds:
                if self._road_object_gate_pass(candidate,value):bucket["ablation_kept"][str(value)]+=1

    @staticmethod
    def _candidate_distance(gt, item):
        return math.hypot(float(item.get("x",0.0))-float(gt.get("x",0.0)),
                          float(item.get("y",0.0))-float(gt.get("y",0.0)))

    def analyze_road_object_recovery_stages(self, diagnostics):
        """Attribute low-slice support and every recovery stage to tagged actors."""
        truth=self.truth_objects();raw=(diagnostics or {}).get("input_points",[]) or []
        stages=(diagnostics or {}).get("stages",{}) or {};support_radius=float(self.config.get("road_object_raw_support_radius",1.50))
        stage_gate=float(self.config.get("road_object_stage_match_distance",2.00))
        stage_names=("component","shape","temporal","dedupe_pass","output","balanced_output",
                     "adaptive_component","adaptive_shape","adaptive_temporal",
                     "adaptive_dedupe_pass","adaptive_output","adaptive_ranked_output",
                     "adaptive_stratified_output","adaptive_hybrid_output",
                     "adaptive_hybrid_gated_output","adaptive_hybrid_rescued_output",
                     "adaptive_hybrid_geometry_gated_output","selected_output")
        for gt in truth:
            if gt.get("object_type")!="unknown_obstacle" or gt.get("role")!="rsu_test_obstacle":continue
            actor_id=int(gt.get("actor_id",0));bucket=self._road_object_stage_coverage.setdefault(actor_id,{
                "actor_id":actor_id,"type_id":gt.get("type_id","unknown"),"visible_frames":0,
                "range_min":None,"range_max":None,"raw_frames":0,"raw_points_total":0,
                "raw_points_max":0,"stage_frames":dict((name,0) for name in stage_names)})
            distance=float(gt.get("range",0.0));bucket["visible_frames"]+=1
            bucket["range_min"]=distance if bucket["range_min"] is None else min(bucket["range_min"],distance)
            bucket["range_max"]=distance if bucket["range_max"] is None else max(bucket["range_max"],distance)
            point_count=sum(1 for point in raw if self._candidate_distance(gt,point)<=support_radius)
            bucket["raw_points_total"]+=point_count;bucket["raw_points_max"]=max(bucket["raw_points_max"],point_count)
            if point_count>0:bucket["raw_frames"]+=1
            for name in stage_names:
                if any(self._candidate_distance(gt,item)<=stage_gate for item in stages.get(name,[]) or []):
                    bucket["stage_frames"][name]+=1
        actors=[dict(item) for _,item in sorted(self._road_object_stage_coverage.items())]
        values=self.config.get("road_object_stage_range_bins",[25.0,35.0,45.0]);uppers=[]
        for value in values or []:
            try:value=float(value)
            except (TypeError,ValueError):continue
            if value>0 and value not in uppers:uppers.append(value)
        uppers.sort();lower=float(self.config.get("road_object_recovery_min_range",5.0));bands=[]
        for upper in uppers:
            selected=[]
            for item in actors:
                distance=((item.get("range_min") or 0.0)+(item.get("range_max") or 0.0))*.5
                if lower<=distance<upper or (upper==uppers[-1] and lower<=distance<=upper):selected.append(item)
            visible=sum(item.get("visible_frames",0) for item in selected)
            band={"min_range":lower,"max_range":upper,"actors":len(selected),"visible_frames":visible,
                  "raw_frames":sum(item.get("raw_frames",0) for item in selected)}
            for name in stage_names:band[name]=sum(item.get("stage_frames",{}).get(name,0) for item in selected)
            bands.append(band);lower=upper
        return {"actors":actors,"range_bands":bands,"support_radius":support_radius,"stage_gate":stage_gate}

    def analyze_road_object_cap_comparison(self, baseline_candidates, balanced_candidates,
                                           adaptive_candidates=None,adaptive_ranked_candidates=None,
                                           adaptive_stratified_candidates=None,
                                           adaptive_hybrid_candidates=None,
                                           adaptive_hybrid_gated_candidates=None,
                                           adaptive_hybrid_rescued_candidates=None,
                                           adaptive_hybrid_geometry_gated_candidates=None,
                                           selected_candidates=None):
        """Compare baseline, balanced and adaptive Shadow outputs in evaluation."""
        truth=self.truth_objects();result={}
        variants=(("baseline",baseline_candidates),("balanced",balanced_candidates))
        if adaptive_candidates is not None:variants+=(("adaptive",adaptive_candidates),)
        if adaptive_ranked_candidates is not None:variants+=(("adaptive_ranked",adaptive_ranked_candidates),)
        if adaptive_stratified_candidates is not None:variants+=(("adaptive_stratified",adaptive_stratified_candidates),)
        if adaptive_hybrid_candidates is not None:variants+=(("adaptive_hybrid",adaptive_hybrid_candidates),)
        if adaptive_hybrid_gated_candidates is not None:variants+=(("adaptive_hybrid_gated",adaptive_hybrid_gated_candidates),)
        if adaptive_hybrid_rescued_candidates is not None:variants+=(("adaptive_hybrid_rescued",adaptive_hybrid_rescued_candidates),)
        if adaptive_hybrid_geometry_gated_candidates is not None:variants+=(("adaptive_hybrid_geometry_gated",adaptive_hybrid_geometry_gated_candidates),)
        if selected_candidates is not None:variants+=(("selected",selected_candidates),)
        for name,candidates in variants:
            detected=self._detected_with_range(candidates);pairs=self._match(truth,detected);classes={}
            for ti,_,_ in pairs:
                label=truth[ti].get("object_type","unknown_obstacle");classes[label]=classes.get(label,0)+1
            current={"candidates":len(detected),"matched":len(pairs),"fp":max(0,len(detected)-len(pairs)),"classes":classes}
            total=self._road_object_cap_totals[name]
            for key in ("candidates","matched","fp"):total[key]+=current[key]
            for label,count in classes.items():total["classes"][label]=total["classes"].get(label,0)+count
            result[name]=current;result[name+"_run"]={"candidates":total["candidates"],"matched":total["matched"],
                                                     "fp":total["fp"],"classes":dict(total["classes"])}
        return result

    @classmethod
    def _adaptive_temporal_profile(cls, items):
        values={"points":[],"current_points":[],"history_points":[],"rank_score":[],
                "support_frames":[],"height":[],"long_side":[],
                "short_side":[],"footprint_area":[],"range":[],"sensor_range":[]};bands={}
        for item in items or []:
            for name,key in (("points","point_count"),("current_points","current_point_count"),
                             ("history_points","temporal_point_count"),
                             ("support_frames","support_frames"),("rank_score","adaptive_rank_score"),
                             ("range","range"),("sensor_range","sensor_range")):
                try:values[name].append(float(item.get(key)))
                except (TypeError,ValueError):pass
            extent=list(item.get("extent",[]) or [])
            try:
                x=float(extent[0]);y=float(extent[1]);values["long_side"].append(max(x,y));values["short_side"].append(min(x,y));values["footprint_area"].append(x*y)
                values["height"].append(float(extent[2]))
            except (IndexError,TypeError,ValueError):pass
            band=str(item.get("adaptive_band","unknown"));bands[band]=bands.get(band,0)+1
        result={}
        for name,items in values.items():
            result[name]={"samples":len(items),"mean":(sum(items)/len(items) if items else None),
                          "min":(min(items) if items else None),
                          "p10":cls._percentile(items,.10),"p50":cls._percentile(items,.50),
                          "p90":cls._percentile(items,.90),"max":(max(items) if items else None)}
        result["bands"]=bands
        return result

    @classmethod
    def _adaptive_temporal_summary(cls, class_items, false_items):
        classes={};truth_total=0;band_totals={}
        for name,items in (class_items or {}).items():
            classes[name]={"samples":len(items),"profile":cls._adaptive_temporal_profile(items)}
            truth_total+=len(items)
            for band,count in classes[name]["profile"]["bands"].items():
                bucket=band_totals.setdefault(band,{"truth":0,"fp":0});bucket["truth"]+=count
        false_profile=cls._adaptive_temporal_profile(false_items)
        for band,count in false_profile["bands"].items():
            bucket=band_totals.setdefault(band,{"truth":0,"fp":0});bucket["fp"]+=count
        for bucket in band_totals.values():
            total=bucket["truth"]+bucket["fp"]
            bucket["precision"]=(float(bucket["truth"])/total if total else None)
        total=truth_total+len(false_items or [])
        return {"candidates":total,"matched":truth_total,"fp":len(false_items or []),
                "precision":(float(truth_total)/total if total else None),
                "classes":classes,"false_profile":false_profile,"bands":band_totals}

    def analyze_road_object_adaptive_profile(self, candidates):
        """Profile pre-cap adaptive candidates; evaluation truth never feeds perception."""
        if not self.config.get("road_object_adaptive_feature_profiling",False):return {"enabled":False}
        truth=self.truth_objects();detected=self._detected_with_range(candidates);pairs=self._match(truth,detected)
        used=set(di for _,di,_ in pairs);class_items={}
        for ti,di,_ in pairs:
            name=truth[ti].get("object_type","unknown_obstacle")
            class_items.setdefault(name,[]).append(detected[di])
        false_items=[item for index,item in enumerate(detected) if index not in used]
        for name,items in class_items.items():
            self._adaptive_temporal_samples["classes"].setdefault(name,[]).extend(items)
        self._adaptive_temporal_samples["false"].extend(false_items)
        return {"enabled":True,
                "frame":self._adaptive_temporal_summary(class_items,false_items),
                "run":self._adaptive_temporal_summary(self._adaptive_temporal_samples["classes"],
                                                      self._adaptive_temporal_samples["false"])}

    @classmethod
    def _hybrid_selection_summary(cls, buckets):
        result={}
        for source,bucket in sorted((buckets or {}).items()):
            result[source]=cls._adaptive_temporal_summary(bucket.get("classes",{}),
                                                          bucket.get("false",[]))
        return result

    def analyze_road_object_hybrid_profile(self, candidates):
        """Profile selected Hybrid sources; CARLA truth remains evaluation-only."""
        if not self.config.get("road_object_hybrid_feature_profiling",False):return {"enabled":False}
        truth=self.truth_objects();detected=self._detected_with_range(candidates);pairs=self._match(truth,detected)
        used=set(di for _,di,_ in pairs);frame={}
        for ti,di,_ in pairs:
            item=detected[di];source=str(item.get("adaptive_hybrid_source","unknown"))
            bucket=frame.setdefault(source,{"classes":{},"false":[]})
            name=truth[ti].get("object_type","unknown_obstacle")
            bucket["classes"].setdefault(name,[]).append(item)
        for di,item in enumerate(detected):
            if di in used:continue
            source=str(item.get("adaptive_hybrid_source","unknown"))
            frame.setdefault(source,{"classes":{},"false":[]})["false"].append(item)
        for source,bucket in frame.items():
            total=self._hybrid_selection_samples.setdefault(source,{"classes":{},"false":[]})
            for name,items in bucket["classes"].items():
                total["classes"].setdefault(name,[]).extend(items)
            total["false"].extend(bucket["false"])
        return {"enabled":True,"frame":self._hybrid_selection_summary(frame),
                "run":self._hybrid_selection_summary(self._hybrid_selection_samples)}

    def analyze_road_object_hybrid_rescue_profile(self, candidates):
        """Profile only candidates added back by Hybrid temporal rescue."""
        if not self.config.get("road_object_hybrid_rescue_feature_profiling",False):return {"enabled":False}
        rescued=[item for item in candidates or []
                 if item.get("adaptive_hybrid_temporal_rescue",False)]
        truth=self.truth_objects();detected=self._detected_with_range(rescued)
        pairs=self._match(truth,detected);used=set(di for _,di,_ in pairs);frame={}
        for ti,di,_ in pairs:
            item=detected[di];source=str(item.get("adaptive_hybrid_source","unknown"))
            bucket=frame.setdefault(source,{"classes":{},"false":[]})
            name=truth[ti].get("object_type","unknown_obstacle")
            bucket["classes"].setdefault(name,[]).append(item)
        for di,item in enumerate(detected):
            if di in used:continue
            source=str(item.get("adaptive_hybrid_source","unknown"))
            frame.setdefault(source,{"classes":{},"false":[]})["false"].append(item)
        for source,bucket in frame.items():
            total=self._hybrid_rescue_samples.setdefault(source,{"classes":{},"false":[]})
            for name,items in bucket["classes"].items():
                total["classes"].setdefault(name,[]).extend(items)
            total["false"].extend(bucket["false"])
        return {"enabled":True,"frame":self._hybrid_selection_summary(frame),
                "run":self._hybrid_selection_summary(self._hybrid_rescue_samples),
                "ablations_frame":self._hybrid_rescue_ablations(frame),
                "ablations_run":self._hybrid_rescue_ablations(self._hybrid_rescue_samples)}

    def _hybrid_rescue_ablations(self, buckets):
        """Evaluate portable scalar rescue gates without filtering perception."""
        result={}
        for source,bucket in sorted((buckets or {}).items()):
            truth=[]
            for items in (bucket.get("classes",{}) or {}).values():truth.extend(items)
            false=list(bucket.get("false",[]) or []);tests=[]
            if source=="near_baseline":
                for value in self.config.get("road_object_rescue_near_area_ablations",[]) or []:
                    try:threshold=float(value)
                    except (TypeError,ValueError):continue
                    tests.append(("area>=%.3f"%threshold,
                                  lambda item,t=threshold:self._candidate_footprint_area(item)>=t))
            else:
                for value in self.config.get("road_object_rescue_far_range_ablations",[]) or []:
                    try:threshold=float(value)
                    except (TypeError,ValueError):continue
                    tests.append(("sensor_range<=%.1f"%threshold,
                                  lambda item,t=threshold:float(item.get("sensor_range",1e9))<=t))
            source_result={}
            for label,gate in tests:
                truth_kept=sum(1 for item in truth if gate(item));fp_kept=sum(1 for item in false if gate(item))
                kept=truth_kept+fp_kept
                source_result[label]={"truth":len(truth),"truth_kept":truth_kept,
                                      "fp":len(false),"fp_kept":fp_kept,
                                      "fp_rejected":len(false)-fp_kept,
                                      "precision":(float(truth_kept)/kept if kept else None)}
            result[source]=source_result
        return result

    @staticmethod
    def _candidate_footprint_area(item):
        extent=list(item.get("extent",[]) or [])
        try:return max(0.0,float(extent[0]))*max(0.0,float(extent[1]))
        except (IndexError,TypeError,ValueError):return 0.0

    def analyze_road_object_recovery(self, geometry_candidates):
        """Profile recovery candidates and simulate the precision gate in Shadow."""
        truth=self.truth_objects();session=self._sync_road_object_benchmark_session(truth)
        detected=self._detected_with_range(geometry_candidates)
        pairs=self._match(truth,detected);used=set(di for _,di,_ in pairs);class_items={}
        self._record_road_object_actor_coverage(truth,pairs,detected)
        for ti,di,_ in pairs:
            name=truth[ti].get("object_type","unknown_obstacle")
            class_items.setdefault(name,[]).append(detected[di])
        false_items=[item for index,item in enumerate(detected) if index not in used]
        classes={}
        truth_counts={}
        for item in truth:
            name=item.get("object_type","unknown_obstacle");truth_counts[name]=truth_counts.get(name,0)+1
        for name,total in truth_counts.items():
            items=class_items.get(name,[]);matched=len(items)
            classes[name]={"truth":total,"matched":matched,"no_geometry":total-matched,
                           "recall":(float(matched)/total if total else None),
                           "profile":self._geometry_profile(items)}
        for name,items in class_items.items():self._road_object_samples["classes"].setdefault(name,[]).extend(items)
        self._road_object_samples["false"].extend(false_items)
        cumulative_classes=dict((name,{"matched_samples":len(items),"profile":self._geometry_profile(items)})
                                for name,items in self._road_object_samples["classes"].items())
        cumulative_items=self._road_object_samples["classes"]
        return {"truth":len(truth),"geometry":len(detected),"matched":len(pairs),
                "false_positive":len(false_items),"classes":classes,
                "benchmark_session":session,
                "false_profile":self._geometry_profile(false_items),
                "precision_gate_shadow":self._road_object_gate_profile(class_items,false_items),
                "gate_ablations":self._road_object_ablations(class_items,false_items),
                "cumulative":{"classes":cumulative_classes,
                              "false_profile":self._geometry_profile(self._road_object_samples["false"]),
                              "precision_gate_shadow":self._road_object_gate_profile(cumulative_items,self._road_object_samples["false"]),
                              "gate_ablations":self._road_object_ablations(cumulative_items,self._road_object_samples["false"]),
                              "actor_coverage":[dict(item) for _,item in sorted(self._road_object_actor_coverage.items())]}}

    def analyze_geometry_attribution(self, geometry_candidates):
        """Attribute Geometry-stage candidates to CARLA road-object classes.

        This is evaluation-only. It explains which class obtained geometry and
        what that geometry looked like; labels are never returned to perception.
        """
        truth=self.truth_objects();detected=self._detected_with_range(geometry_candidates)
        pairs=self._match(truth,detected);by_truth=dict((ti,di) for ti,di,_ in pairs)
        used=set(di for _,di,_ in pairs);classes={}
        for ti,gt in enumerate(truth):
            name=gt.get("object_type","unknown_obstacle")
            bucket=classes.setdefault(name,{"truth":0,"matched":0,"no_geometry":0,"items":[]})
            bucket["truth"]+=1
            if ti in by_truth:
                bucket["matched"]+=1;bucket["items"].append(detected[by_truth[ti]])
            else:bucket["no_geometry"]+=1
        for bucket in classes.values():
            bucket["recall"]=(float(bucket["matched"])/bucket["truth"] if bucket["truth"] else None)
            bucket["profile"]=self._geometry_profile(bucket.pop("items"))
        false_items=[item for index,item in enumerate(detected) if index not in used]
        return {"truth":len(truth),"geometry":len(detected),"matched":len(pairs),
                "false_positive":len(false_items),"classes":classes,
                "false_profile":self._geometry_profile(false_items)}

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
        total=self._empty_drop_counts();bins=[];lower=0.0;classes={}
        def matched_truth(candidates):return set(ti for ti,_,_ in self._match(truth,candidates))
        geo_match=matched_truth(geo);roi_match=matched_truth(roi);score_match=matched_truth(scored);dyn_match=matched_truth(dyn)
        roi_reject_match=matched_truth(roi_rej);score_reject_match=matched_truth(score_rej)
        for upper in self.range_bins:
            c=self._empty_drop_counts();c["min_range"]=lower;c["max_range"]=upper;bins.append(c);lower=upper
        details=[]
        for truth_index,gt in enumerate(truth):
            if truth_index not in geo_match:reason="no_geometry_candidate"
            elif truth_index not in roi_match:reason="roi_reject" if truth_index in roi_reject_match else "roi_lost"
            elif truth_index not in score_match:reason="score_reject" if truth_index in score_reject_match else "score_lost"
            elif truth_index not in dyn_match:reason="dynamic_drop"
            else:reason="pass"
            total["truth"]+=1;total[reason]+=1
            name=gt.get("object_type","unknown_obstacle")
            class_counts=classes.setdefault(name,self._empty_drop_counts())
            class_counts["truth"]+=1;class_counts[reason]+=1
            rng=float(gt.get("range",0.0));lower=0.0
            for b in bins:
                upper=float(b["max_range"])
                if (lower<=rng<upper) or (upper==self.range_bins[-1] and lower<=rng<=upper):
                    b["truth"]+=1;b[reason]+=1;break
                lower=upper
            details.append({"actor_id":gt.get("actor_id"),"object_type":name,
                            "range":rng,"reason":reason})
        total["range_bins"]=bins;total["class_counts"]=classes;total["details"]=details
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
