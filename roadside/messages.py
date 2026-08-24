from __future__ import print_function

import json
import math


_RSM_PTC_TYPES = {
    "vehicle": 1, "car": 1, "truck": 1, "bus": 1, "motorcycle": 1,
    "bicycle": 2,
    "person": 3, "pedestrian": 3,
}


def encode_object_list(object_list):
    return json.dumps(object_list.to_dict(), ensure_ascii=False, separators=(",", ":"))


def encode_rsm(object_list):
    """Encode a local-coordinate RSM-like view of the canonical object list.

    This is deliberately not the Dachuan MEC-RSU envelope yet: the vendor
    message requires a surveyed WGS84 reference position and local-to-geodetic
    conversion. Keeping that conversion outside the perception boundary avoids
    publishing plausible-looking but incorrect road coordinates.
    """
    participants = []
    for obj in object_list.objects:
        speed = (obj.vx * obj.vx + obj.vy * obj.vy) ** 0.5
        heading = 0.0
        if speed > 1e-3:
            heading = math.degrees(math.atan2(obj.vy, obj.vx)) % 360.0
        size = getattr(obj, "size", [0.0, 0.0, 0.0])
        type_name = str(obj.object_type).lower()
        participants.append({
            "ptcId": obj.object_id,
            "ptcType": _RSM_PTC_TYPES.get(type_name, 0),
            "typeName": type_name,
            "pos": {"x": obj.x, "y": obj.y, "z": obj.z},
            "speedMps": speed,
            "headingDeg": heading,
            "sizeM": {"length": float(size[0]), "width": float(size[1]),
                      "height": float(size[2])},
            "confidence": obj.confidence,
            "sources": list(getattr(obj, "sources", [])),
        })
    message = {
        "msgType": "RSM",
        "version": "V0.2-local-json",
        "stationId": object_list.station_id,
        "timestamp": object_list.timestamp,
        "frameId": getattr(object_list, "frame_id", None),
        "coordinateFrame": getattr(object_list, "coordinate_frame", "unknown"),
        "participants": participants,
    }
    return json.dumps(message, ensure_ascii=False, separators=(",", ":"))
