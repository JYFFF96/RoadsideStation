# -*- coding: utf-8 -*-
"""Independent ego view; may start before the scene and reconnect to its next ego."""
from __future__ import print_function

import argparse
import math
import os
import signal
import sys
import time

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

from roadside.ego_camera import EgoCamera
from roadside.sim_ego import EGO_ROLE, find_ego_actor


def main():
    parser = argparse.ArgumentParser(description="Independent CARLA ego camera window")
    parser.add_argument("--config", default="config/roadside.yaml")
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--ego-role", default=None)
    parser.add_argument("--width", type=int, default=960)
    parser.add_argument("--height", type=int, default=540)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--view", choices=EgoCamera.PRESETS, default="chase")
    args = parser.parse_args()
    if not (160 <= args.width <= 3840 and 120 <= args.height <= 2160 and 1 <= args.fps <= 60):
        parser.error("width=160..3840, height=120..2160, fps=1..60 required")
    if sys.platform.startswith("linux") and not (os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")):
        parser.error("A desktop display is required; run this viewer in the Ubuntu desktop terminal")
    import carla
    import cv2
    import numpy as np
    import yaml
    with open(args.config, encoding="utf-8") as fp:
        config = yaml.safe_load(fp)
    cc = config.get("carla", {})
    role = args.ego_role or config.get("v2x_events", {}).get("test_ego_role", EGO_ROLE)
    client = carla.Client(args.host or cc.get("host", "127.0.0.1"),
                          args.port if args.port is not None else int(cc.get("port", 2000)))
    client.set_timeout(2.0)
    camera = EgoCamera(args.width, args.height, args.fps, args.view)
    actor = None
    status = "Waiting for ego role=" + role
    next_lookup = 0.0
    window = "RoadsideStation Ego View"

    def stop(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    try:
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        print("[EGO VIEW] V/1/2/3: change view; Q/ESC/close: exit viewer only", flush=True)
        while True:
            now = time.monotonic()
            if now >= next_lookup:
                next_lookup = now + 1.0
                try:
                    world = client.get_world()
                    actor = find_ego_actor(world, role)
                    if actor is None:
                        camera.close()
                        status = "Waiting for ego role=" + role
                    else:
                        if camera.actor_id != actor.id or camera.camera is None or not camera.camera.is_alive:
                            camera.attach(world, actor)
                        velocity = actor.get_velocity()
                        speed = 3.6 * math.sqrt(velocity.x ** 2 + velocity.y ** 2 + velocity.z ** 2)
                        status = "EGO #%d  %.1f km/h  %s" % (actor.id, speed, camera.view.upper())
                except (RuntimeError, ValueError) as exc:
                    actor = None
                    camera.close()
                    status = "Waiting: " + str(exc).splitlines()[0][:100]
                    print("[EGO VIEW] " + status, flush=True)
            latest = camera.latest
            if latest is not None and now - latest[1] <= 2.0:
                image = latest[0]
                view = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                    (image.height, image.width, 4))[:, :, :3].copy()
            else:
                view = np.zeros((args.height, args.width, 3), dtype=np.uint8)
                cv2.putText(view, "Waiting for camera frames / simulation ticks", (16, 90),
                            cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 200, 255), 1)
            cv2.putText(view, status, (16, 28), cv2.FONT_HERSHEY_SIMPLEX, .6, (70, 255, 100), 2)
            cv2.putText(view, "1 Chase | 2 Driver | 3 Top | V Cycle | Q/ESC Exit", (16, args.height - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, .55, (255, 255, 255), 1)
            cv2.imshow(window, view)
            key = cv2.waitKey(max(1, int(1000 / args.fps))) & 0xff
            if key in (27, ord("q")) or cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            if key in (ord("1"), ord("2"), ord("3"), ord("v")):
                index = ((EgoCamera.PRESETS.index(camera.view) + 1) % 3 if key == ord("v")
                         else key - ord("1"))
                camera.view = EgoCamera.PRESETS[index]
                camera.close()
                next_lookup = 0.0
    except KeyboardInterrupt:
        pass
    finally:
        camera.close()
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
