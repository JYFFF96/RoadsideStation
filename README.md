# RoadsideStation V0.1

RoadsideStation is a roadside perception prototype for CARLA 0.9.15. V0.1 focuses on a minimal end-to-end pipeline:

CARLA roadside sensors -> sensor adapters -> simple fusion -> ObjectList -> RSM-like JSON -> MQTT -> dashboard.

## Target environment

- Ubuntu 22.04
- CARLA 0.9.15
- Python 3.7 for CARLA PythonAPI
- Mosquitto / MQTT (optional in V0.1)

## Quick start

1. Start CARLA:

```bash
./CarlaUE4.sh
```

2. Export CARLA PythonAPI:

```bash
export PYTHONPATH=$PYTHONPATH:~/carla/PythonAPI/carla/dist/carla-0.9.15-py3.7-linux-x86_64.egg
```

3. Install dependencies:

```bash
python3.7 -m pip install -r requirements.txt
```

4. Run:

```bash
python3.7 main.py
```

## V0.1 scope

- Fixed roadside RGB camera
- Fixed roadside LiDAR
- Fixed roadside radar
- Sensor frame cache
- Basic nearest-neighbor fusion scaffold
- Unified ObjectList output
- RSM-like JSON encoder
- MQTT publisher
- Configurable sensor transforms

## Notes

The fusion in V0.1 is intentionally lightweight. It provides a stable software skeleton first; perception models, calibration, tracking and standards-compliant RSM encoding will be upgraded in later versions.
