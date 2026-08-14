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
