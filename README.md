## V0.6.12.8.2.2.88 — 完整 RSI 数据体

RSI 在大椽 MQTT `type/value/category` 信封内补齐 CSAE
`RoadSideInformation` 的 `moy`、8 字节 `id` 和 `refPos` 字段；`rtes/rtss`
继续使用设备在线协议规定的 MEC 字段名。`eventPos` 为事件绝对经纬度，
`refPos` 为 RSU 实测参考位置。`rsi_id` 必须配置为恰好 8 个 ASCII 字节。
也可在启动时用 `--rsu-rsi-id RSU_0001` 覆盖配置。

## V0.6.12.8.2.2.87 — 单独打印 RSI 数据

每次生成 RSI 时先用 `[RSI DATA]` 单独打印完整 JSON，再打印精简的
`[RSU MQTT TX]` 发送状态和 topic。若未启用 `dachuan_rsu`，启动时会明确
提示必须配置实测经纬度并启用桥接，避免事件触发后没有 RSI 输出却无法定位原因。

## V0.6.12.8.2.2.86 — 合规 RSI 与精简运行日志

道路事件发送前统一编码为大椽 MEC-RSU 在线 MQTT 格式：
`type=RSI`、`value.category=RSI`、`rtes/rtss`。AVW 作为可配置的道路
障碍 RTE 发送，使用检测车辆的实际位置生成经纬度；内部 `event_sort`
不再作为 RSI `eventType`。默认日志只显示重要状态及实际 RSI 发送消息，
`python3.7 main.py --verbose` 可恢复完整诊断；场景脚本同样支持 `--verbose`。

## V0.6.12.8.2.2.85 — 车道安全主车控制

主车改为 CARLA 驾驶航点跟随，禁用场景主车的 Traffic Manager 控制；
对同车道车辆、行人和场景障碍物增加独立安全制动。HLW 自动识别目标
路口的全部入口车道并逐车道布置障碍物，AVW 从目标同车道后方起步。
控制日志以 `[EGO CONTROL]` 输出车速、车道偏差、转向和危险物间距。

## V0.6.12.8.2.2.84 — 场景主车与独立跟随窗口

四个预警场景现在统一创建/复用 `rsu_test_ego`，主程序读取主车实际状态。
运行 `python3.7 tools/scenario_avw.py --ego-view` 可同时打开跟随窗口，
或单独运行 `python3.7 tools/ego_view.py`。按 1/2/3 切换跟随/驾驶/俯视，Q 关闭窗口。
查看 [完整步骤、生命周期和验证限制](docs/SCENARIO_EGO_VIEW.md)。

# RoadsideStation

RoadsideStation is a roadside perception prototype for CARLA 0.9.15.

Current architecture:

CARLA official traffic -> fixed roadside sensors -> perception/fusion -> ObjectList -> RSM-like JSON -> MQTT.

The RoadsideStation process does **not** own normal background traffic. Use CARLA's official `PythonAPI/examples/generate_traffic.py` to create vehicles and walkers, then start RoadsideStation as a separate perception process.

## Target environment

- Ubuntu 22.04
- CARLA 0.9.15
- Python 3.7 for CARLA PythonAPI
- Mosquitto / MQTT (optional)

## Quick start

### 1. Start CARLA

```bash
cd ~/carla
./CarlaUE4.sh
```

If you need a specific map, start/load that map before creating traffic. RoadsideStation defaults to `load_world_on_start: false` so it does not destroy existing traffic.

### 2. Export CARLA PythonAPI

```bash
export PYTHONPATH=$PYTHONPATH:~/carla/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg
```

### 3. Start balanced traffic near the RSU

Open another terminal. This helper uses CARLA map spawn points and Traffic
Manager autopilot only. It balances the initial vehicle supply across the three
evaluation bands and does not recycle actors unless `--recycle` is explicitly
requested:

```bash
cd ~/RoadsideStation
python3.7 tools/spawn_rsu_traffic.py --vehicles 45 --spawn-radius 130
```

Messages such as:

```text
ERROR: Spawn failed because of collision at spawn position
```

can occur when a randomly selected spawn point is already occupied. If the script continues and reports spawned vehicles/walkers, the remaining traffic can still run normally.

### 4. Start RoadsideStation

Open another terminal:

```bash
cd ~/RoadsideStation
python3.7 main.py
```

For the multi-class geometry benchmark, wait until main.py prints
`[BACKGROUND] Status:READY`, then start deterministic crosswalk walkers and
lane obstacles in another terminal. Starting them after background calibration
prevents the intentional static hazards from being learned as fixed map:

```bash
cd ~/RoadsideStation
python3.7 tools/spawn_multiclass_targets.py --walkers 12 --obstacles 6 --seed 42
```

LiDAR-only road occupancy is published as `unknown_obstacle`. Camera-associated
targets use `person`, `bicycle`, `motorcycle`, `car`, `bus` or `truck`.

At startup RoadsideStation reports how many existing CARLA vehicles and walkers it sees, for example:

```text
Traffic mode: external (source=carla_generate_traffic)
Attached to existing CARLA traffic: 30 vehicles, 10 walkers
Traffic near RSU | 00-30m V:4 P:3 | 30-50m V:5 P:4 | 50-80m V:7 P:5 | total:28
```

RoadsideStation then spawns only the fixed roadside sensors and begins perception/evaluation. It does not enable autopilot, set target velocity, or otherwise control the normal traffic vehicles.

## Evaluation

CARLA ground truth is used only for evaluation and does not enter the perception/fusion output.

The console periodically prints metrics such as:

```text
[EVAL 80m] Truth:18 Tracks:14 Matched:13 Missed:5 FP:1 Recall:72.2% Precision:92.9%
```

Range bins are configurable in `config/roadside.yaml` and currently default to 0-30m, 30-50m and 50-80m.

V0.6.12.8.1 also prints per-class Geometry attribution and stage-drop lines.
`NoG` means no Geometry-stage candidate was available for that truth object;
the profiles show point counts, extents, cluster modes and rescue sources for
matched geometry. CARLA truth remains evaluation-only.

V0.6.12.8.2 adds a separate low road-object recovery channel below the normal
0.30m ground cut. Compact candidates must repeat for two consecutive LiDAR
frames before they enter ROI and Tracker. `[ROAD-OBJECT RECOVERY]` reports the
point, shape, temporal and deduplication stages. Stage-drop attribution now uses
one-to-one matching at every stage.

V0.6.12.8.2.1 keeps that recovery channel in Shadow mode: candidates are
profiled against simulation truth but never enter ROI, Tracker or ObjectList.
The target generator defaults to seed 42 and prints every spawned blueprint,
position and RSU range so comparison runs use the same scenario. Shadow logs
separate matched classes from false positives with point, size and range means.

V0.6.12.8.2.2 adds an evaluation-only precision-gate simulation. It reports
candidate point, height, normalized long/short side and range p10/p50/p90
distributions for each matched class and false positives, both per frame and
cumulatively. The provisional gate (points >= 10, height <= 0.45m, range <=
25m) reports truth keep and false-positive rejection without changing the
perception path. Benchmark actors are tagged so main.py records their exact
type, position and range in the same log.

V0.6.12.8.2.2.1 keeps the gate in Shadow and adds actor-level coverage for
every tagged benchmark obstacle. Each actor reports visible frames, recovery
matches, gate keeps and point/height/range rejection counts. Point thresholds
8, 9 and 10 are evaluated in parallel so the next enforcing version can select
a threshold using measured object coverage instead of aggregate samples alone.

V0.6.12.8.2.2.2 attributes every tagged obstacle across the low-slice raw
support, connected-component, shape, temporal, deduplication and output-cap
stages. Per-actor and 5-25m/25-35m/35-45m summaries remain evaluation-only and
show whether a missed object lacks LiDAR support or is rejected later in the
recovery pipeline. No recovery threshold or production output is changed.

V0.6.12.8.2.2.3 adds a parallel range-balanced cap selection in Shadow. The
5-25m, 25-35m and 35-45m bands reserve four of the twelve candidate slots each;
unused slots are refilled by the strongest remaining candidates. Logs compare
the unchanged global Top-12 baseline with the balanced selection per actor and
range band. The balanced list never reaches ROI, Tracker or ObjectList.

V0.6.12.8.2.2.4 adds range-adaptive temporal geometry in a second Shadow path.
The 25-35m band combines up to three low-slice frames and the 35-45m band up to
four, with spatial voxel deduplication, band-specific support-frame requirements
and mandatory current-frame support. Logs compare its component, shape,
temporal, deduplication and capped output against the unchanged Baseline and
Balanced paths. Adaptive candidates never reach ROI, Tracker or ObjectList.

V0.6.12.8.2.2.5 profiles the Adaptive candidates before their output cap.
Evaluation logs split truth classes and false positives by sensor-range band and
report total/current/history point counts, support frames, height and evaluation
range distributions. This identifies a scalar, real-device-portable ranking or
gate without using CARLA labels in perception. The Adaptive path remains Shadow.

V0.6.12.8.2.2.6 adds a parallel scalar ranking for Adaptive pre-cap candidates.
Low road-object height receives the strongest priority, followed by proximity
to the measured five-point support center, current-frame support and temporal
persistence. Range-balanced quotas are retained. The ranked list is evaluated
against the prior Adaptive list but remains isolated from production output.

V0.6.12.8.2.2.7 isolates benchmark sessions in cumulative road-object reports.
When a new batch of tagged `rsu_test_*` actors first appears, evaluation-only
samples, actor/stage coverage and cap-comparison totals are reset before that
frame is recorded. This makes `main.py`-first and target-spawner-first launch
orders comparable without changing perception, recovery or ranking behavior.

V0.6.12.8.2.2.8 adds a label-free height-stratified Adaptive ranking in Shadow.
Each sensor-range band reserves slots for both low geometry and elevated
geometry before refilling from the scalar score. This tests whether the extra
low-obstacle recall from V0.6.12.8.2.2.6 can be retained without displacing as
many pedestrian candidates. Benchmark activation now waits for tagged obstacle
actors, preventing a transient walker-only frame from creating a second reset.

V0.6.12.8.2.2.9 adds a hybrid recovery selection in Shadow. Stable Baseline
candidates supply the 5-25m band while scalar-ranked Adaptive candidates supply
25-45m; range quotas are then applied to the combined, spatially deduplicated
pool. Baseline far candidates can refill unused capacity. The Hybrid list is
evaluation-only and does not alter ROI, Tracker or ObjectList.

V0.6.12.8.2.2.10 profiles the selected Hybrid candidates by source. Frame and
session reports split `near_baseline`, `far_ranked` and fallback candidates into
truth classes and false positives, with score, point support, temporal support,
height, footprint and range distributions. The profiler is evaluation-only and
provides real-device-portable evidence for a later Hybrid admission gate.

V0.6.12.8.2.2.11 adds that source-aware Hybrid admission gate in Shadow. Near
Baseline candidates require sufficient points and either a low profile or a
complete short side. Far Ranked candidates require four-frame stability, or
strong current support inside 32m. Logs compare the gated list with the full
Hybrid list; neither list is connected to production ROI, Tracker or ObjectList.

V0.6.12.8.2.2.12 adds a second Shadow-only temporal rescue after the strict
Hybrid gate. It restores Near Baseline rejects only after three-frame support
with at least five points, and restores Far Ranked rejects inside 32m after
three-frame support with at least four accumulated and one current point. The
rescued list is reported independently and still does not feed production ROI,
Tracker or ObjectList.

V0.6.12.8.2.2.13 profiles only the incremental candidates restored by that
temporal rescue. Frame and session reports split Near Baseline and Far Ranked
rescues into truth classes and false positives, including point, temporal,
shape, score and range distributions. This remains evaluation-only evidence
for tuning the two rescue rules independently.

V0.6.12.8.2.2.13.1 fixes the Rescue profiler activation in the evaluation
configuration and adds benchmark truth-lifecycle diagnostics. Tagged actors are
reported when they enter, cross the evaluation boundary, disappear while still
well inside it, or move by a teleport-sized step. These diagnostics remain
CARLA evaluation-only and never affect perception or tracking.

V0.6.12.8.2.2.14 adds Shadow ablations for a source-aware Rescue geometry
gate. Near Baseline rescues compare minimum footprint areas, while Far Ranked
rescues compare maximum sensor ranges. The profiler now reports sensor range
and footprint area directly, keeping all features portable to a real roadside
sensor coordinate frame. No ablation filters ROI, Tracker or ObjectList.

V0.6.12.8.2.2.15 applies the selected geometry rules to a new Shadow branch.
Strict Hybrid-Gate candidates are always retained. Incremental Near rescues
must have at least 0.04 square metres of footprint, while incremental Far
rescues must be within 28m of the sensor. The resulting complete candidate list
is evaluated beside the ungated Rescue list and remains disconnected from ROI,
Tracker and ObjectList.

V0.6.12.8.2.2.16 introduces a named Selected output policy in Shadow. The
policy currently resolves to the Hybrid Geometry-Gated list and is reported as
a separate stage and cumulative comparison. The recovery function still
returns its legacy baseline and global Shadow remains enabled, so selecting a
policy cannot yet change ROI, Tracker or ObjectList.

V0.6.12.8.2.2.16.1 stabilizes the deterministic pedestrian benchmark after a
run recorded four different test walkers moving 9.68m to 20.83m between truth
samples. Walkers now use balanced approach points, distinct opposite-side
destinations, CARLA AI walker controllers and staggered launches. If an AI
controller is unavailable, the tool falls back to the same staggered crossing
directions with manual WalkerControl. This changes only the test-target
generator; the Selected perception policy remains Shadow-only.

V0.6.12.8.2.2.16.2 makes staggered manual WalkerControl the default after the
Town05 AI pedestrian controller left all 12 walkers stationary for an entire
benchmark run. The balanced approaches, distinct opposite-side directions and
0.8-second launch spacing remain in place to avoid the old centre-point crowd
collision. AI mode is still available explicitly with `--walker-mode ai` for
maps whose pedestrian navigation mesh supports the requested crossing.

V0.6.12.8.2.2.17 retunes the selected Rescue geometry gate for moving
multi-class traffic. The Near Baseline footprint threshold is reduced from
0.04 to 0.02 square metres to preserve compact pedestrian returns, and the Far
Ranked sensor limit is expanded from 28m to 32m. In the supporting run those
rules would retain about 66 of 69 Full-Rescue truth matches while raising the
estimated Selected precision from 23.3% to about 26.0%. The policy remains
Shadow-only and does not yet feed ROI, Tracker or ObjectList.

V0.6.12.8.2.2.18 begins a controlled CARLA enforcing trial for the named
Selected output. The chosen adaptive Hybrid Geometry-Gated candidates now feed
ROI and Tracker when both the selected-enforcing switch is on and global
Road-Object Shadow mode is off. An invalid or disabled selection falls back to
the legacy Baseline. Runtime statistics report the selected policy, enforcing
state and active candidate count so the next benchmark can measure the real
Tracker/ObjectList effect. The selection still uses sensor geometry only and
does not consume CARLA truth.

V0.6.12.8.2.2.19 adds end-to-end attribution for that enforcing trial. Selected
candidate provenance now survives ROI, scoring, dynamic filtering and tracking;
tracks report both a current selected measurement and any selected contribution
in their lifetime, including coast frames. The CARLA-only evaluator prints
frame and cumulative candidate, match, false-positive, precision and class
counts for every stage. This makes it possible to distinguish useful recovery
from temporal FP persistence before changing production thresholds.

V0.6.12.8.2.2.20 profiles a dedicated Selected admission score in Shadow mode.
The normal candidate scorer bypasses objects inside 50 metres, which explains
why the previous run showed identical ROI, Score and Dynamic counts. Selected
candidates now carry the same sensor-only geometry score even in the 5–45m
recovery corridor. The CARLA evaluator compares thresholds from 0.20 to 0.45,
reporting truth retention, false positives and precision without changing the
enforcing output or exposing simulation truth to perception.

V0.6.12.8.2.2.21 enables the conservative Selected admission score gate at
0.20. In the profiling run it retained all 75 truth matches while rejecting
five false-positive samples; thresholds of 0.30 or higher were rejected because
they removed more than 40 percent of truth support. The gate applies only to
the selected recovery channel, remains sensor-only and has an explicit switch
for rollback. Profiling continues across accepted and rejected candidates so
the enforced result can be checked against the same full input population.

V0.6.12.8.2.2.22 adds path-level attribution after the conservative gate. It
separates accepted Selected candidates into Near/Far and Strict/Temporal-Rescue
paths, then separates Selected-influenced tracks into New, Confirmed and Coast
states. The previous run showed that a global score threshold cannot safely
separate truth from false positives and that Near Rescue contains valuable
pedestrians, so this CARLA-only attribution identifies a narrower portable
sensor-policy target before any further enforcement.

V0.6.12.8.2.2.23 profiles a Selected-only new-track temporal admission rule.
The preceding path run found similar precision for Near/Far and Strict/Rescue,
while Selected-created New tracks were weaker than Confirmed and Coast tracks.
In Shadow mode, a Selected candidate without an existing track or radar support
is marked as a first-frame hold and a spatially consistent observation on the
next LiDAR frame is marked as confirmed. Original candidates still reach the
Tracker unchanged. CARLA-only reports compare Hold, Confirm, Expired,
Track-Bypass and Sensor-Bypass truth/false-positive counts and classes; runtime
decisions consume sensor and track geometry only.

V0.6.12.8.2.2.24 adds stable internal IDs to pending Selected admissions and
profiles their terminal transitions. The preceding run showed that two-frame
confirmation improved sample precision only from 19.8 to 24.6 percent, while
expired samples still contained pedestrian evidence; radar bypass was also too
weak to justify enforcement. The CARLA-only evaluator now reports whether each
Hold originated as truth or false positive, whether it later Confirmed or
Expired, whether the same truth actor survived, and unique actor/class coverage.
The identifiers and decisions remain perception-side metadata in Shadow mode;
truth labels never enter Fusion, Tracker or ObjectList.

V0.6.12.8.2.2.25 removes Shadow-created-track contamination from the Selected
admission study. A track can now bypass new-track admission only after at least
one normal non-Selected geometry update; a track created solely because Shadow
allowed a held Selected measurement is not counterfactual evidence. Tracker
provenance records that independent update count through active and coast
states. The evaluator also splits unique held actors into Confirm-Only,
Expired-Only, Both and Unresolved outcomes with per-class counts and overall
confirmation coverage. The gate remains non-enforcing.

V0.6.12.8.2.2.26 profiles the original Hold measurement separately for
Confirmed and Expired outcomes. Each terminal group is split into person,
unknown obstacle and false-positive samples, with score, point count, height,
range, cluster mode and Near/Far plus Strict/Rescue path distributions. The
previous counterfactual run confirmed that a uniform two-frame gate would cover
only about 61.5 percent of held actors and leave four pedestrians in the
Expired-Only group, so this version searches for a portable sensor-only
exception rule before any enforcement. CARLA truth remains evaluator-only.

V0.6.12.8.2.2.27 adds Selected Admission Camera-Support Profiling. Every held
candidate is projected into the configured camera and its visibility, generic
2D association, IoU, center distance, detector class and confidence are copied
only into the evaluator sample. Confirmed and Expired outcome profiles report
camera visible/supported counts and support rates by camera source and detected
class. With `carla_truth` this is a simulation benchmark proxy; with `detector`
the identical path consumes real detector boxes. Neither mode changes Selected
admission, Fusion, Tracker or ObjectList output.

V0.6.12.8.2.2.28 turns those camera features into parallel Shadow rescue
ablations. A rescue requires a configured pedestrian camera class plus either
minimum projected-box IoU or maximum center distance. The evaluator reports
sample retention for Confirmed/Expired pedestrians and false positives, and
also counts how many unique Expired-Only actors and pedestrians would be
rescued. The three candidate rules are compared without changing admission,
Fusion, Tracker or ObjectList. This keeps the rule portable to a real detector
while CARLA camera truth remains only a simulation benchmark proxy.

V0.6.12.8.2.2.29 profiles delayed LiDAR reappearance after the camera study
showed that the remaining Expired-Only pedestrians were outside useful camera
support at admission time. Four parallel sensor-only rules compare 0.75, 1.0
and 1.5 second pending windows plus a wider 3.5 meter spatial gate. Each rule
reports truth/false-positive precision, unique actor coverage, and how many
Expired-Only pedestrians would reappear in time. The parallel pending state is
diagnostic only and cannot confirm or filter a runtime track.

V0.6.12.8.2.2.30 separates each delayed rule's incremental events from
current-frame candidates already confirmed by the original 0.5 second rule.
Only those extra rescues are truth-attributed and profiled by time gap, spatial
displacement, apparent speed, current/origin score and point count, height and
range. This exposes the real cost of the promising 1.5 second window before it
can be considered for enforcement; all logic remains evaluation-only.

V0.6.12.8.2.2.31 enables Town05 startup map selection in `main.py`. CARLA map
names are normalized from either short names or full asset paths. If the
current short name starts with `Town05` (including `Town05_Opt`), the existing
world and traffic are preserved. Otherwise `main.py` loads the configured
`Town05_Opt` world before creating the roadside station. Because a real map
switch removes CARLA actors, traffic and multi-class targets should be started
after `main.py`.

V0.6.12.8.2.2.32 adds sensor-only risk-gate ablations to the incremental
delayed-reappearance profile. Strict, moderate, wide and slow sparse-geometry
rules report precision, truth retention, false-positive rejection, class/actor
coverage and final Expired-Only rescue. All gates remain evaluation-only and
cannot alter Fusion or Tracker input. Town05 startup selection remains enabled.

V0.6.12.8.2.2.33 selects the lowest-risk observed delayed combination as an
operational shadow policy: `ttl075_gate25` plus the `sparse_slow` sensor gate
(score/origin score <= 0.40, current/origin points <= 4, height <= 0.25m and
apparent speed <= 0.50m/s). Each LiDAR frame now reports `WouldKeep` and
`WouldReject` for this exact policy while Tracker, Fusion and ObjectList remain
unchanged. The other delayed windows and risk gates remain parallel evaluator
ablations so subsequent runs can verify that the selection is stable.

V0.6.12.8.2.2.34 adds an evaluator-only deployment verdict for the selected
delayed policy. `ttl075_gate25/sparse_slow` is reported as `READY` only after
meeting all configured evidence requirements: at least 20 gated candidates,
85% precision, 60% truth retention, and at least one rescued Expired-Only actor
and pedestrian. Otherwise the log reports `BLOCKED` with exact reasons. The
verdict cannot enable enforcement or alter Tracker, Fusion, ObjectList or RSM.

V0.6.12.8.2.2.35 adds a separate deployment verdict for the selected camera
rescue rule `iou05_or_d30`. It measures expired-person sample count, kept-sample
precision, expired false-positive rejection, Expired-Only person actor coverage
and confirm false-positive rejection. A sixth mandatory condition requires the
evidence source to be `detector`; CARLA truth results remain `BLOCKED` even when
all numeric thresholds pass. The verdict is evaluator-only and cannot alter the
runtime perception, tracking, fusion, ObjectList or RSM path.

V0.6.12.8.2.2.36 adds an explicit real-detector validation entry. Run
`tools/check_camera_detector.py` to verify the configured ONNX file and OpenCV
DNN loader without starting CARLA. `main.py` now accepts `--camera-source` and
`--camera-model`, allowing a detector benchmark without editing YAML. Detector
startup failure falls back to `none`, never to CARLA truth. All camera-rescue
decisions remain evaluator-only and cannot alter Tracker, Fusion, ObjectList or
RSM.

V0.6.12.8.2.2.37 adds an automatic ONNX Runtime fallback for models that
OpenCV DNN cannot import (including YOLOv5 graphs containing incompatible
`Floor` nodes). OpenCV remains the first choice; the detector switches to
ONNX Runtime only when model loading fails. Python 3.7 uses
`onnxruntime==1.14.1`. The preflight reports the selected runtime explicitly.

V0.6.12.8.2.2.38 adds detector-specific camera-association calibration
shadows at 90px and 100px. The first real-detector run placed true person
support near 87--95px while the first supported false positives were near
107--109px, outside the former 30/45px ablations. These wider rules remain
evaluation-only: the selected deployment rule stays `iou05_or_d30`, and no
Tracker, Fusion, ObjectList or RSM behavior changes.

V0.6.12.8.2.2.39 refines the detector distance calibration with 105px and
110px shadows. The second detector run placed true person support at
104.09--104.18px and the first false-positive support at 108.90px. The 105px
rule tests the narrow separating boundary; 110px is an explicit spillover
control. Both remain evaluation-only and the selected deployment rule remains
`iou05_or_d30`.

V0.6.12.8.2.2.40 promotes `detector_d90` to the evaluator-only candidate
deployment verdict. In the fine-calibration run, d90 and d100 rescued the same
three person samples with zero false positives, while d105 added both an
expired and a confirmed false positive without rescuing another person. The
candidate still cannot affect Tracker, Fusion, ObjectList or RSM; it must meet
all verdict thresholds, including 50% Expired-Only person actor coverage.

V0.6.12.8.2.2.41 adds a deterministic static-walker benchmark mode to
`tools/spawn_multiclass_targets.py`. With `--walker-mode static`, pedestrian
actors are spawned at the same seeded crosswalk locations but receive no AI
controller and no `WalkerControl`, isolating detector projection and
camera/LiDAR association from motion and cross-frame timing. Manual and AI
moving modes remain available and unchanged.

V0.6.12.8.2.2.42 preserves CARLA timestamps in the Camera, LiDAR and Radar
cache entries and reports Camera-minus-LiDAR frame and time deltas once per
second. This is a diagnostic-only shadow for the slow-walker regression: the
cache still exposes the latest frame from each sensor, and no synchronization,
projection, association, Tracker, Fusion, ObjectList or RSM behavior changes.

V0.6.12.8.2.2.43 adds a bounded sensor history and an opt-in timestamp-aligned
benchmark snapshot. Legacy behavior remains the default. Run the same slow
walker case with `python3.7 main.py --camera-source detector --camera-model
models/yolov5n.onnx --sensor-sync aligned`; the runtime selects the newest exact
Camera/LiDAR frame pair and the nearest-timestamp Radar sample. Compare it with
`--sensor-sync latest` before changing any default perception behavior.

V0.6.12.8.2.2.44 keeps the aligned benchmark behavior and adds evaluator-only
nearest-camera-box attribution for every visible Selected HOLD candidate. The
report includes distance, IoU, confidence and detector-class distributions even
when the nearest box lies outside the association gate. This distinguishes a
projection-offset problem from missing or misclassified person detections; it
does not change association, admission, Tracker, Fusion, ObjectList or RSM.

V0.6.12.8.2.2.45 hardens the evaluator-only camera-rescue deployment verdict
with a separate evidence floor for Person samples actually kept by the selected
rule. Total expired Person observations can no longer produce READY when only
two camera-positive samples determine precision. The default floor is five;
camera association and every production output remain unchanged.

V0.6.12.8.2.2.46 reports the actual camera opportunity coverage of expired
Person samples: how many Selected candidates project inside the image and the
corresponding rate. The deployment verdict requires at least five in-frame
Person opportunities in addition to five kept positives, preventing off-camera
samples from being mistaken for detector failures. This remains evaluator-only.

V0.6.12.8.2.2.47 attributes every off-screen Selected camera projection to
behind-camera, left, right, above, below or degenerate geometry. Outcome reports
separate Person and false-positive rejection directions, showing whether low
camera opportunity is expected field-of-view coverage or a projection defect.
The diagnostic path cannot create camera support or alter runtime output.

V0.6.12.8.2.2.48 starts the first deliverable V2X event layer. A standalone
`V2XEventEngine` consumes ObjectList without entering Fusion and emits Dachuan
event2hmi-compatible JSON for abnormal stopped vehicles (`AVW`, event_sort 6)
and speed-limit/overspeed warnings (`SLW`, event_sort 9). AVW uses configurable
speed, dwell and cooldown thresholds. SLW waits for an explicit ego-speed input;
`test_ego_speed_kmh` exists only for bench validation. The MQTT event topic is
configurable and must be confirmed with Dachuan before device-side deployment.

## Current scope

- Two opposite-facing roadside RGB cameras
- Fixed roadside LiDAR
- Fixed roadside radar
- Sensor frame cache
- LiDAR clustering and filtering
- Radar association
- Tracking
- Camera/LiDAR association scaffold
- Canonical FusedObjectList V1.0 output
- CARLA ground-truth evaluation
- RSM-like JSON encoder
- MQTT publisher
- Configurable sensor transforms

## Traffic ownership

Normal traffic should be created by CARLA's official traffic generator:

```text
CARLA generate_traffic.py
        |
        v
vehicles / walkers
        |
        v
RoadsideStation fixed sensors
        |
        v
LiDAR / Camera / Radar
        |
        v
perception -> fusion -> tracking -> ObjectList / RSM
```

Scenario vehicles used later for abnormal-stop, wrong-way, FCW, BSD, VRU or intersection-conflict tests should be managed separately from normal background traffic.

V0.6.12.8.2.2.49 closes the public fused-output boundary. Camera-associated
class labels, normalized object size, radar evidence, track age/state and source
provenance now remain present in the single `FusedObjectList V1.0` consumed by
MQTT, the local-coordinate RSM-like adapter and the V2X event engine. The
previous pre-camera `ObjectList` is retained only as an internal LiDAR/tracker
result. CARLA LiDAR defaults now model the RoboSense Fairy 48TX: 48 channels,
690,000 points/s, 10Hz and -15.84 to +15.84 degree vertical FOV, with an 80m
first-stage acceptance range. Ground Truth remains evaluation-only.

V0.6.12.8.2.2.50 adds the dual-camera coverage baseline required by the Fairy
48TX installation geometry. `CAM_NORTH` and `CAM_SOUTH` have independent
caches, transforms, projection, detector/truth objects and Camera-LiDAR
association before their results enter one canonical `FusedObjectList`.
Multi-camera snapshots align to the LiDAR frame by default. Startup now reports
the level-mount LiDAR blind ranges; at the configured 8.5m sensor height and
-15.84 degree lower FOV these are approximately 30.0m at ground level and
24.0m for a 1.7m target. Fixed-map background learning is enabled for six
seconds, after which newly spawned stationary hazards remain eligible. A
periodic `[FUSED OUTPUT SAMPLE]` exposes the real public JSON fields in logs.
Ground Truth remains evaluation-only and cannot initiate or modify tracks.

V0.6.12.8.2.2.51 adds enforced near-field radar track initiation for the
configured 48TX blind zone. Cartesian radar returns between 2m and 30m are
spatially clustered and must repeat on two distinct radar frames. Only moving
clusters with at least 0.6m/s absolute radial speed, a valid road ROI and no
nearby LiDAR candidate enter the common tracker; static radar clusters remain
diagnostic-only. Radar-only tracks expose source `radar` and may subsequently
receive camera class evidence through the existing dual-camera association.
The implementation uses only scalar radar data and is independent of CARLA
actors so an MR76 adapter can reuse the same fusion boundary. Runtime logs
report the complete decision funnel in `[RADAR INIT]`.

V0.6.12.8.2.2.52 makes the background-calibration transition unmistakable:
the first READY report is wrapped in 72-character separator lines and prints
an explicit instruction to start the benchmark targets. The ordinary compact
background status remains available afterwards. The radar initiator now also
profiles the absolute radial speed of every temporally confirmed cluster and
reports p50, maximum and shadow counts at 0.10/0.20/0.40/0.60m/s. These
threshold comparisons are diagnostic-only; the enforced 0.60m/s gate remains
unchanged until a log demonstrates a safe lower boundary.

V0.6.12.8.2.2.53 adds a dedicated path for sparse moving radar returns. The
previous log showed moving pedestrians with non-zero radial velocity but only
one radar return, while the existing initiator required a two-point spatial
cluster. A single return at or above 0.20m/s may now initiate a candidate only
after three distinct radar frames. It remains subject to the configured road
ROI and LiDAR deduplication, and cannot share temporal confirmation with the
existing multi-point cluster path. `[RADAR INIT]` separately reports component,
single-candidate, single-confirmation and single-emission counts. Static
single-point returns remain rejected before temporal confirmation.

V0.6.12.8.2.2.54 adds full-frame cumulative lifecycle profiling for sparse
radar returns. The one-second status sample in V0.6.12.8.2.2.53 observed only
one moving singleton and could not distinguish a genuinely rare return from a
short-lived event between log samples. The new cumulative line counts every
radar frame, singleton component, speed-threshold crossing, pending start,
temporal match, below-threshold return near an active pending candidate,
expiration grouped by hit count, confirmation and emission. It also counts
moving points buried inside multi-point components, where a static median
could otherwise conceal the return. The profiling remains
sensor-only and diagnostic-only; the enforced three-frame, 0.20m/s single-
return policy and the public fused target list are unchanged.

V0.6.12.8.2.2.55 adds a parallel motion-seed bridge in Shadow mode. The
V0.6.12.8.2.2.54 run observed four moving singleton seeds; none had a second
non-zero-speed match, but two were followed by a nearby below-threshold return.
The new branch therefore requires the first singleton to meet the unchanged
0.20m/s motion threshold, then permits nearby singletons to continue the
candidate when radial velocity becomes zero. Two-frame and three-frame rules
are compared independently and report confirmation, LiDAR deduplication, road
ROI rejection and would-emit totals. CARLA Ground Truth separately attributes
the would-emit events to matched targets or false positives; those labels never
enter the sensor pipeline. Static returns cannot create a seed. The branch is
diagnostic-only and never reaches Tracker or the public ObjectList.

V0.6.12.8.2.2.56 expands the motion-seed bridge into independent 2.5m, 4.0m
and 6.0m match-gate Shadow states. V0.6.12.8.2.2.55 produced three valid
motion seeds but no continuation inside 2.5m, so each wider gate now maintains
its own pending lifecycle and two-frame/three-frame decisions. Every variant
reports confirmation, LiDAR deduplication, road ROI rejection, would-emit and
CARLA truth precision independently. The 2.5m production matching boundary is
unchanged, and all ablation candidates remain outside Tracker and ObjectList.

V0.6.12.8.2.2.57 adds a separate singleton-to-component morphology Shadow
bridge. The V0.6.12.8.2.2.56 run confirmed that every singleton-only bridge
candidate was either already represented by LiDAR or outside the road ROI,
including the wider 4m and 6m gates. This version therefore stops widening the
singleton association and tests whether a moving singleton reappears as a
multi-return radar component within 2.5m or 4.0m. It reports matches, moving
matches, average component size, expiration, LiDAR deduplication, ROI rejection,
would-emit count and CARLA truth precision. The experiment remains sensor-only
and cannot change Tracker or ObjectList output.

V0.6.12.8.2.2.58 pivots from radar-only spatial widening to two-camera support
profiling. The V0.6.12.8.2.2.57 run produced five moving singleton seeds but
only one singleton-to-component transition; that two-point static component
was already represented by LiDAR. Each moving singleton that is not LiDAR-
deduplicated and passes the road ROI is now projected into both cameras and
associated with generic camera objects. Cumulative visibility, support rate,
supported truth/false positives, precision, camera classes and source are
reported. CARLA camera truth remains an evaluation proxy, the identical path
accepts real detector boxes, and no Shadow result enters Tracker or ObjectList.

V0.6.12.8.2.2.59 adds dual-camera ground-plane initiation in Shadow. The
V0.6.12.8.2.2.58 run showed all five moving radar singleton observations were
already within the LiDAR/track deduplication radius, while average 0--30m track
recall was only about 9.7 percent and missed objects were dominated by missing
geometry. For each generic camera detection, the bottom-centre pixel ray is
intersected with the configured road plane to estimate a world position. The
2--30m candidates then pass cross-camera deduplication, LiDAR/track deduplication
and the normal road ROI. Counts and CARLA truth precision are evaluation-only.
This is the same calibration-matrix operation needed by a Qt/C++ detector path;
CARLA actors are never read by the positioning algorithm or runtime policy.

V0.6.12.8.2.2.60 fixes the camera-ground road-ROI adapter. V0.6.12.8.2.2.59
passed the complete candidate dictionary to an API that requires separate
`x`, `y`, `z`, `extent`, and candidate arguments; its fail-closed exception
handling therefore reported every tested candidate as an ROI rejection. The
adapter now supplies the correct arguments, understands the validator's
`(accepted, reason, details)` result, exposes validator errors explicitly, and
reports vehicle versus VRU ROI outcomes plus rejection reasons. The branch
remains Shadow-only and does not change Tracker or ObjectList input.

V0.6.12.8.2.2.61 adds evaluator-only VRU road-ROI margin ablations. The fixed
V0.6.12.8.2.2.60 run produced zero validator errors, 585 accepted camera-ground
samples (581 truth matches and zero false positives), and 156 VRU samples
rejected only for lateral distance. The rejected samples are now compared at
additional 1 m, 2 m, and 3 m margins, with cumulative truth/false-positive
precision reported independently for each margin. No variant enters Tracker or
ObjectList. The background READY banner also uses eight vertical `|` lines above
and below the message so the target-spawn point is easier to spot in the log.

V0.6.12.8.2.2.62 selects the most conservative VRU margin from the Shadow
comparison and adds temporal admission profiling. The V0.6.12.8.2.2.61 run had
12 lateral VRU rejections: extra 1 m recovered 8, extra 2 m recovered 10, and
extra 3 m recovered 11; every recovered CARLA-truth sample matched a person.
Normal-ROI candidates plus only the extra-1-m VRU candidates now require two
distinct camera frames within a 2 m association gate. Confirmed candidates are
truth-attributed and reported, but remain isolated from Tracker and ObjectList.

V0.6.12.8.2.2.63 measures the end-to-end value of that confirmed camera path
without enabling it. The V0.6.12.8.2.2.62 run temporally confirmed 490 of 506
camera inputs; all 490 matched CARLA pedestrians and none was a false positive.
Each frame now compares current Tracker truth coverage with a counterfactual
Tracker-plus-camera result, reporting incremental matches, combined recall,
recall gain, camera precision and class attribution. A deployment verdict is
also explicit: `carla_truth` is always blocked, while a real detector must
provide at least 100 samples, 95 percent precision and 5 percent recall gain.

V0.6.12.8.2.2.64 is the real-detector evidence run. V0.6.12.8.2.2.63 showed
that the confirmed camera path would raise cumulative Tracker recall from about
35.9 percent to 50.5 percent, a 14.6-point gain, with 517/517 CARLA-truth
camera matches. The ONNX preflight now performs an actual blank-frame inference,
and runtime reporting includes processed frames, detections, average/maximum
latency and class counts. No detector evidence reports
`BLOCKED_NO_DETECTOR_EVIDENCE`; CARLA truth remains explicitly blocked.

V0.6.12.8.2.2.65 starts a controlled real-detector camera track-initiation
trial. The V0.6.12.8.2.2.64 run processed 362 detector inferences, temporally
confirmed 121 person candidates, matched all 121 to truth with zero false
positives, and raised counterfactual recall from about 33.3 percent to 38.5
percent. Confirmed candidates are now queued for the next fusion cycle and
enter the common Tracker only when the configured and per-candidate source are
both `detector`, and this first enforcement step is restricted to the evidenced
`person` class. `carla_truth`, a missing detector, other classes, unconfirmed
candidates, and same-position LiDAR observations all fail closed. Camera-
originated tracks keep receiving camera measurements until LiDAR takes
precedence; their class and camera source continue through the public fused
object list. Runtime logs expose queued, consumed, source-rejected, and LiDAR-
deduplicated counts.

V0.6.12.8.2.2.66 adds post-enforcement attribution without widening the
camera policy. The V0.6.12.8.2.2.65 run queued and consumed all 162 confirmed
real-detector person samples with no source, class, or LiDAR-deduplication
failure. Camera candidates that were still incremental over the now-combined
Tracker fell to 39, while cumulative base coverage rose from the preceding
roughly 33 percent to about 42 percent. The old counterfactual deployment
verdict is therefore marked retired after enforcement. Camera-origin provenance
now survives later LiDAR takeover and coasting, allowing evaluator-only reports
of current and cumulative matched tracks, false positives, duplicate tracks,
camera-only samples, LiDAR takeovers, unique tracks, unique truth actors, states,
and classes. CARLA truth remains confined to the evaluator and cannot influence
Tracker admission.

V0.6.12.8.2.2.67 corrects dense-pedestrian identity diagnostics. The
V0.6.12.8.2.2.66 run observed 204 camera-origin track samples, all 204 matched
to person truth with zero false positives, while 19 unique track IDs covered 11
truth actors. Its old proximity-based duplicate count was inflated because
multiple legitimate pedestrians can lie inside the shared 4 m evaluation gate.
An unmatched track is now classified as duplicate-like only when it remains
inside a truth gate; other unmatched tracks are spatial false positives.
Persistent actor-to-track and track-to-actor maps separately report fragmented
actors, extra ID fragments, identity-switching tracks, and average/maximum
track lifetime. Cumulative camera-only and LiDAR-takeover samples are also
printed. These additions remain CARLA evaluator diagnostics and do not alter
the detector-only person enforcement policy or public object output.

V0.6.12.8.2.2.68 adds evaluator-only camera identity-gate ablation. The
V0.6.12.8.2.2.67 run produced 171 actual camera-origin track samples: 164
matched truth, seven were unmatched, six of those remained near truth, and
precision was 95.9 percent. However, the shared 4 m evaluation gate attributed
12 track IDs to 11 actors while simultaneously reporting nine fragmented
actors and six identity-switching tracks, which can be caused by ambiguous
nearest-neighbour assignments among dense pedestrians. This version therefore
runs independent 1 m, 2 m, 3 m, and 4 m truth associations in parallel and
reports precision, duplicate-like and spatial false positives, fragmented
actors, extra ID fragments, switching tracks, and position error for each
gate. The experiment is confined to GroundTruthEvaluator: Tracker matching,
camera admission, detector-only person enforcement, and ObjectList output are
unchanged.

V0.6.12.8.2.2.69 attributes every actual camera-origin track birth to the
production association state. The V0.6.12.8.2.2.68 gate comparison showed
that reducing the truth gate is not a usable fix: 1 m matched only 40.6 percent
of camera-origin samples, 2 m matched 74.3 percent, 3 m matched 95.0 percent,
and 4 m matched 98.5 percent, while identity switches remained at seven or
eight for the 2--4 m variants. The Tracker already predicts position from its
velocity estimate, so this version does not duplicate that experiment.
Instead, it records the actual prediction-to-detection match distance and
classifies a new camera track as no active tracks, outside the 3.5 m production
gate, or assignment conflict when an eligible track was claimed by another
detection. It also reports the nearest active and nearest camera-origin track
distances plus candidate multiplicity. All fields are diagnostic-only; greedy
assignment, gates, track IDs, admission, and ObjectList output are unchanged.

V0.6.12.8.2.2.70 adds source-aware camera reassociation gates in Shadow mode.
The V0.6.12.8.2.2.69 run produced 21 camera-origin track births: 19 had no
active track inside the production 3.5 m gate and only two were assignment
conflicts. Normal updates stayed well inside the gate, with 0.81 m average,
1.51 m P90, and 2.46 m maximum association distance, while a new track's
nearest camera-origin track averaged 7.46 m. A global gate increase would
therefore create dense-pedestrian identity risk. For each outside-gate birth,
this version compares 4 m, 5 m, 6 m, and 8 m camera-origin-only recovery gates.
CARLA truth then labels the nearest prior track as the same actor, a conflicting
actor, ambiguous, or previously unknown; the diagnostic also rejects a simple
recovery when that prior track is already claimed by another detection. Only
an available, single-actor-consistent association is counted as safe recovery.
The experiment does not change production assignment, the 3.5 m gate, Tracker
IDs, camera admission, or ObjectList output.

V0.6.12.8.2.2.71 profiles expired camera-track reappearance in Shadow mode.
The V0.6.12.8.2.2.70 run rejected active-track gate widening: 4 m through 8 m
produced zero safe recoveries and zero-percent identity precision. Nearly every
candidate was already claimed by another detection or belonged to a different
or ambiguous actor. To test whether fragmentation instead follows detection
gaps and track deletion, an expired camera-origin track is now retained for
five seconds as a diagnostic tombstone. New camera tracks compare both its
frozen last position and velocity-predicted position at 2 m, 3.5 m, and 5 m.
Each variant reports truth-consistent, conflicting, ambiguous, and unknown
matches, safe recovery count, identity precision, distance, and last-seen gap.
Both normal max-age deletion and the quality-based stale-cleanup path populate
the tombstone memory. Tombstones never enter matching, never resurrect an ID,
and do not change admission, Tracker output, or ObjectList.

V0.6.12.8.2.2.72 profiles sensor-side features for the best tombstone variant.
The V0.6.12.8.2.2.71 run found two truth-consistent expired-track recoveries,
but the best velocity-predicted 2 m gate also contained one conflicting and one
ambiguous identity, for only 50 percent identity precision. It is therefore not
enabled. Camera temporal admission now carries a smoothed two-frame ground-
motion vector, while tombstones retain their last camera ID and velocity. The
evaluator separates same-actor, conflicting, ambiguous, and unknown candidates
and reports same-camera rate, heading cosine, tombstone speed, new temporal
motion, distance, and gap distributions for the predicted 2 m variant. These
features use detector and tracker metadata available on real equipment; CARLA
actor identity remains evaluation-only. Production association, tombstone
recovery, Tracker IDs, and ObjectList are unchanged.

V0.6.12.8.2.2.73 evaluates sensor-only tombstone recovery rules in Shadow
mode. The uploaded `.72`-named log actually ran V0.6.12.8.2.2.71, so it could
not contain the feature profiling introduced by `.72`; its predicted 2 m rule
nevertheless found three same-actor candidates and no conflicts. This version
uses one run to compare same-camera, non-opposing heading, and two combined
rules on exactly the same predicted 2 m population. Missing camera or motion
features fail closed and are reported separately. CARLA actor identity only
labels each counterfactual result. The rules never alter production matching,
resurrect track IDs, or change Tracker and ObjectList output.

V0.6.12.8.2.2.74 replaces the rejected camera-ID and heading rule comparison
with tight distance, gap, and low-motion Shadow gates. In the `.73` run every
candidate came from the same camera, while heading-based rules retained only a
conflicting actor. The two same-actor samples instead appeared at 0.12--0.17 m
after 2.42--2.50 seconds; five conflicts appeared around 0.99--1.25 m after
4.99--5.91 seconds. This version evaluates 0.20 m and 0.35 m distance limits,
a 3.0-second gap, and a low-speed combination. Missing features fail closed.
All rules remain evaluator-only and leave Tracker IDs and ObjectList unchanged.

V0.6.12.8.2.2.75 pivots the first deliverable from long-lived pedestrian
identity recovery to an event-focused VRUCW warning. The Dachuan event2hmi
manual requires category VRUCW, event_sort 10, participant type, direction,
speed, and time; it does not require a permanent perception track ID. After
two consecutive road-ROI-approved person observations, the event engine now
emits one area-level warning per cooldown window. Multiple fragmented person
IDs are aggregated into one event. Both `ptc_type` from the manual table and
`spc_type` from its JSON example are included pending real-RSU verification.
AVW and SLW remain available, and perception/Tracker output is unchanged.

V0.6.12.8.2.2.76 adds the second first-deliverable scene: an ID-independent
road-hazard warning (`HLW`, event_sort 8). The event engine consumes only the
canonical sensor-derived FusedObjectList. It requires a confirmed, persistent,
stationary, compact unknown obstacle and selected-road, multi-sensor, or high-
quality track evidence. Long thin structures and low-quality clutter are
rejected. Multiple fragmented track IDs still produce one area-level warning
per cooldown window. The message includes the Dachuan HLW fields and event_type
37; CARLA truth remains evaluator-only.

V0.6.12.8.2.2.77 adds the focused abnormal-stopped-vehicle (`AVW`, event_sort
6) validation path. AVW dwell is now area-level, so a fragmented Tracker ID
does not restart the five-second timer. `spawn_multiclass_targets.py` can place
a hand-braked four-wheel vehicle on an approach with `--stopped-vehicles 1`.
`main.py --event-scenario avw` isolates this event during acceptance testing;
the default `all` mode remains unchanged. The `.76` obstacle-only log produced
12 valid HLW events, but also exposed repeated false `person` classifications.
VRUCW now applies a basic fused LiDAR-size sanity gate that rejects the observed
0.11 m-high and 3.05 m-high false-person clusters without using CARLA truth.

V0.6.12.8.2.2.78 closes the missing-classification gap found in the `.77` AVW
run. The stopped vehicle was not visible to the evaluator because its dedicated
role was absent from `include_roles`, and the supplied ONNX model emitted only
`person` labels across 396 frames. The evaluator now recognizes the tagged
vehicle, while production AVW can conservatively classify a confirmed,
persistent, stationary 2.8--7.5 m by 1.2--3.2 m fused LiDAR track as vehicle
geometry. This fallback uses only FusedObjectList sensor fields; CARLA actor
identity remains evaluation-only. A once-per-second `V2X AVW INPUT` line shows
typed, geometry-fallback, stopped, and dwell counts when AVW is isolated.

V0.6.12.8.2.2.79 adds a focused speed-limit-warning (`SLW`, event_sort 9)
acceptance path. `main.py --event-scenario slw --test-ego-speed-kmh 55`
injects an explicit bench-only ego speed and compares it with the configured
40 km/h limit, producing `spd_Flag=2`. Without an OBU speed or this explicit
test input, SLW fails closed and prints `Speed:UNAVAILABLE Event:SUPPRESSED`
instead of fabricating an event. The event schema and comparison logic can be
kept when the bench input is replaced by live OBU vehicle speed.

V0.6.12.8.2.2.80 adds the protocol-correct Dachuan MEC-to-RSU MQTT bridge.
`main.py` now stays running with all four first-release warnings enabled and
publishes the canonical fused participant list as Dachuan RSM at 10 Hz. Four
independent `tools/scenario_*.py` launchers add and remove VRUCW, HLW, AVW, or
SLW test actors without restarting the station. The SLW launcher supplies a
tagged CARLA ego vehicle whose measured velocity replaces the earlier command-
line-only speed injection. RSM requests use a UUID topic, QoS 2, and monitor
`command///res/#`; surveyed WGS84 coordinates remain mandatory. Optional RSI
support stays disabled pending vendor HLW/SLW mapping confirmation. See
`docs/FIRST_RELEASE_SCENARIOS_AND_RSU_MQTT.md` for the complete workflow.

V0.6.12.8.2.2.81 aligns MQTT with the field-proven RSU command rather than the
generic topic shown in the newer protocol note. The default RSM request topic
is now `command/dachuan/DC887-002047/req/{UUID}/rsm`, the broker is the local
Mosquitto instance, and authentication is optional. The device ID remains
configurable for another RSU. The preflight command no longer requires a host
or username when testing the verified localhost/no-auth path.

V0.6.12.8.2.2.82 enforces the protocol boundary between RSM and RSI. Dachuan
RSM now contains only explicitly classified motor vehicles (`ptcType=1`),
non-motor vehicles (`ptcType=2`) and pedestrians (`ptcType=3`); vehicle-sized
unknown obstacles are no longer guessed into the participant list. Filtering
now happens before the 16-participant limit. Supported road events are emitted
through RSI, while unconfirmed `eventType`/`signType` mappings fail closed;
the HMI `event_sort` value is never substituted for an RSI standard type.

V0.6.12.8.2.2.83 separates the two vendor MQTT routes: field-proven RSM keeps
`command/dachuan/{device_id}/req/{uuid}/rsm`, while RSI uses the protocol-manual
channel `command/traffic/event/req/{uuid}/rsi`. Each template is independently
configurable.
