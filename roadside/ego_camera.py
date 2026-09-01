"""An attached display-only camera. Never ticks the world or moves the spectator."""
from __future__ import print_function

import time
import threading
import subprocess


class EgoCamera(object):
    PRESETS = ("chase", "driver", "top")

    def __init__(self, width=960, height=540, fps=20, view="chase"):
        self.width, self.height, self.fps = width, height, fps
        self.view = view
        self.camera = None
        self.actor_id = None
        self.latest = None
        self._generation = 0
        self._lock = threading.Lock()

    def attach(self, world, actor):
        import carla
        self.close()
        if self.view == "driver":
            transform = carla.Transform(carla.Location(x=.5, y=-.35, z=1.3))
        elif self.view == "top":
            transform = carla.Transform(carla.Location(x=-1.0, z=18.0),
                                        carla.Rotation(pitch=-85.0))
        else:
            transform = carla.Transform(carla.Location(x=-7.0, z=3.5),
                                        carla.Rotation(pitch=-15.0))
        bp = world.get_blueprint_library().find("sensor.camera.rgb")
        for key, value in (("image_size_x", self.width), ("image_size_y", self.height),
                           ("fov", 90), ("sensor_tick", 1.0 / self.fps)):
            bp.set_attribute(key, str(value))
        self.camera = world.spawn_actor(bp, transform, attach_to=actor,
                                        attachment_type=carla.AttachmentType.Rigid)
        self.actor_id = actor.id
        generation = self._generation

        def receive(image):
            with self._lock:
                if generation == self._generation:
                    self.latest = (image, time.monotonic())

        try:
            self.camera.listen(receive)
        except Exception:
            self.close()
            raise

    def close(self):
        with self._lock:
            self._generation += 1
            camera, self.camera = self.camera, None
            self.actor_id = None
            self.latest = None
        if camera is not None:
            for operation in (camera.stop, camera.destroy):
                try:
                    operation()
                except RuntimeError:
                    pass


def launch_ego_viewer(config_path, role_name):
    """Use the scenario's Python/CARLA environment for the independent window."""
    import os
    import sys
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return subprocess.Popen([sys.executable, os.path.join(root, "tools", "ego_view.py"),
                             "--config", os.path.abspath(config_path),
                             "--ego-role", role_name])


def stop_ego_viewer(process):
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2)
