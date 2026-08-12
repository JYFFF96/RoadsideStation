from __future__ import print_function

import math


def _iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    x1 = max(ax1, bx1); y1 = max(ay1, by1)
    x2 = min(ax2, bx2); y2 = min(ay2, by2)
    iw = max(0.0, float(x2 - x1)); ih = max(0.0, float(y2 - y1))
    inter = iw * ih
    aa = max(0.0, float(ax2 - ax1)) * max(0.0, float(ay2 - ay1))
    ba = max(0.0, float(bx2 - bx1)) * max(0.0, float(by2 - by1))
    den = aa + ba - inter
    return 0.0 if den <= 0.0 else inter / den


def _center(box):
    return ((box[0] + box[2]) * 0.5, (box[1] + box[3]) * 0.5)


def associate_camera_to_lidar(camera_objects, lidar_tracks,
                              min_iou=0.05, max_center_distance=120.0):
    """Greedy one-to-one association between camera 2D boxes and projected LiDAR boxes.

    lidar_tracks items must contain: id, bbox and optional object reference.
    camera_objects are CameraObject instances or dict-like entries.
    """
    pairs = []
    used_cam = set(); used_lidar = set(); candidates = []
    for ci, cam in enumerate(camera_objects or []):
        cb = cam.bbox if hasattr(cam, "bbox") else cam["bbox"]
        cc = _center(cb)
        for li, track in enumerate(lidar_tracks or []):
            lb = track["bbox"]
            lc = _center(lb)
            iou = _iou(cb, lb)
            dist = math.hypot(cc[0] - lc[0], cc[1] - lc[1])
            if iou >= float(min_iou) or dist <= float(max_center_distance):
                # Prefer IoU strongly; distance only breaks weak-overlap ties.
                score = iou * 1000.0 - dist
                candidates.append((score, iou, dist, ci, li))
    candidates.sort(reverse=True)
    for score, iou, dist, ci, li in candidates:
        if ci in used_cam or li in used_lidar:
            continue
        used_cam.add(ci); used_lidar.add(li)
        pairs.append({
            "camera_index": ci,
            "lidar_index": li,
            "iou": float(iou),
            "center_distance": float(dist),
        })
    return pairs
