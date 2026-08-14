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
