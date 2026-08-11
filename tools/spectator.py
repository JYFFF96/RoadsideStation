from __future__ import print_function

import argparse
import math
import sys
import time

import carla

try:
    import pygame
except ImportError:
    pygame = None


def _forward_vector(rotation):
    pitch = math.radians(rotation.pitch)
    yaw = math.radians(rotation.yaw)
    return carla.Vector3D(
        x=math.cos(pitch) * math.cos(yaw),
        y=math.cos(pitch) * math.sin(yaw),
        z=math.sin(pitch))


def _right_vector(rotation):
    yaw = math.radians(rotation.yaw + 90.0)
    return carla.Vector3D(x=math.cos(yaw), y=math.sin(yaw), z=0.0)


def _move(transform, dx, dy, dz):
    forward = _forward_vector(transform.rotation)
    right = _right_vector(transform.rotation)
    transform.location.x += forward.x * dx + right.x * dy
    transform.location.y += forward.y * dx + right.y * dy
    transform.location.z += dz
    return transform


def _find_first_vehicle(world):
    vehicles = world.get_actors().filter("vehicle.*")
    return vehicles[0] if len(vehicles) else None


def _jump_to_overview(world):
    spawn_points = world.get_map().get_spawn_points()
    if not spawn_points:
        return carla.Transform(carla.Location(z=50.0), carla.Rotation(pitch=-70.0))
    cx = sum(p.location.x for p in spawn_points) / float(len(spawn_points))
    cy = sum(p.location.y for p in spawn_points) / float(len(spawn_points))
    return carla.Transform(carla.Location(x=cx, y=cy, z=80.0),
                           carla.Rotation(pitch=-75.0, yaw=0.0, roll=0.0))


def main():
    parser = argparse.ArgumentParser(description="Interactive CARLA spectator controller")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--speed", type=float, default=12.0, help="movement speed in m/s")
    parser.add_argument("--mouse-sensitivity", type=float, default=0.15)
    args = parser.parse_args()

    if pygame is None:
        print("pygame is required for tools/spectator.py")
        print("Install with: python3.7 -m pip install pygame==2.1.3")
        sys.exit(1)

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    spectator = world.get_spectator()

    pygame.init()
    pygame.display.set_caption("RoadsideStation Spectator Controller")
    screen = pygame.display.set_mode((720, 160))
    pygame.event.set_grab(True)
    pygame.mouse.set_visible(False)
    clock = pygame.time.Clock()

    font = pygame.font.SysFont("monospace", 18)
    running = True
    follow_vehicle = None

    print("Spectator controls:")
    print("  W/S/A/D : move")
    print("  Q/E     : down/up")
    print("  Mouse   : look")
    print("  Shift   : faster")
    print("  1       : Town overview")
    print("  2       : follow first vehicle")
    print("  Esc     : exit")

    while running:
        dt = max(0.001, clock.tick(60) / 1000.0)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_ESCAPE:
                    running = False
                elif event.key == pygame.K_1:
                    follow_vehicle = None
                    spectator.set_transform(_jump_to_overview(world))
                elif event.key == pygame.K_2:
                    follow_vehicle = _find_first_vehicle(world)
                    if follow_vehicle is None:
                        print("No vehicle found to follow.")

        if follow_vehicle is not None:
            if not follow_vehicle.is_alive:
                follow_vehicle = None
            else:
                vt = follow_vehicle.get_transform()
                yaw = math.radians(vt.rotation.yaw)
                x = vt.location.x - math.cos(yaw) * 10.0
                y = vt.location.y - math.sin(yaw) * 10.0
                z = vt.location.z + 5.0
                spectator.set_transform(carla.Transform(
                    carla.Location(x=x, y=y, z=z),
                    carla.Rotation(pitch=-15.0, yaw=vt.rotation.yaw, roll=0.0)))
        else:
            transform = spectator.get_transform()
            mx, my = pygame.mouse.get_rel()
            transform.rotation.yaw += float(mx) * args.mouse_sensitivity
            transform.rotation.pitch -= float(my) * args.mouse_sensitivity
            transform.rotation.pitch = max(-89.0, min(89.0, transform.rotation.pitch))

            keys = pygame.key.get_pressed()
            speed = args.speed * (3.0 if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT] else 1.0)
            step = speed * dt
            dx = (step if keys[pygame.K_w] else 0.0) - (step if keys[pygame.K_s] else 0.0)
            dy = (step if keys[pygame.K_d] else 0.0) - (step if keys[pygame.K_a] else 0.0)
            dz = (step if keys[pygame.K_e] else 0.0) - (step if keys[pygame.K_q] else 0.0)
            transform = _move(transform, dx, dy, dz)
            spectator.set_transform(transform)

        screen.fill((25, 25, 25))
        lines = [
            "WASD move | Q/E vertical | Mouse look | Shift boost | Esc quit",
            "1: town overview | 2: follow first vehicle",
            "Click/focus this controller window if keys or mouse do not respond.",
        ]
        for i, text in enumerate(lines):
            screen.blit(font.render(text, True, (230, 230, 230)), (12, 12 + i * 36))
        pygame.display.flip()

    pygame.event.set_grab(False)
    pygame.mouse.set_visible(True)
    pygame.quit()


if __name__ == "__main__":
    main()
