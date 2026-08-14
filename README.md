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

For the V0.6.12.8.1 multi-class geometry benchmark, also start deterministic
crosswalk walkers and lane obstacles in another terminal before main.py:

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

## Current scope

- Fixed roadside RGB camera
- Fixed roadside LiDAR
- Fixed roadside radar
- Sensor frame cache
- LiDAR clustering and filtering
- Radar association
- Tracking
- Camera/LiDAR association scaffold
- Unified ObjectList output
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
